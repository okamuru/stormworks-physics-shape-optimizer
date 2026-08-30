"""Cross-platform application service backed by the exact portable model."""

from array import array
from dataclasses import dataclass, replace
from hashlib import sha256
import os
from pathlib import Path
import tempfile
from typing import Callable, List, Optional, Sequence, Tuple

from . import __version__
from .definitions import DefinitionCatalog
from .exact_optimizer import (
    ExactBodyOptimization,
    ExactComponentOrderOptimization,
    ExactVehicleOptimization,
    optimize_staged_component_order,
    optimize_vehicle_portable_exact,
    pinned_non_physics_component_order,
)
from .component_package import install_component_package, plan_component_package
from .model import WorldVoxel
from .optimizer import (
    UnsupportedVehicleError,
    _is_axis_aligned_unit_transform,
    _parse_with_comments,
    validate_mixed_component_groups,
)
from .partition import Box
from .portable_merge import PortableMergeOracle, PortableMergeResult, voxel_clip_plane
from .rotations import GRID_TRANSFORMS
from .source_preserving_xml import write_vehicle_component_order_preserving_source
from .surface_flood_fill import model_surface_physics_flood_fill
from .vehicle import ComponentPlacement, load_vehicle
from .viewer import ShapeMesh, merge_group_mesh


APP_VERSION = __version__


def _sha256_file(path: Path, chunk_size: int = 1024 * 1024) -> str:
    """Hash a file with bounded memory regardless of vehicle XML size."""

    digest = sha256()
    buffer = bytearray(chunk_size)
    view = memoryview(buffer)
    with Path(path).open("rb") as stream:
        while True:
            count = stream.readinto(buffer)
            if not count:
                break
            digest.update(view[:count])
    return digest.hexdigest()


@dataclass(frozen=True)
class SearchModeProfile:
    key: str
    label: str
    stage_evaluations: Tuple[int, ...]
    description: str


SEARCH_MODE_PROFILES = {
    "standard": SearchModeProfile(
        "standard",
        "標準（高速）",
        (64,),
        "64評価・1段階",
    ),
    "deep": SearchModeProfile(
        "deep",
        "深掘り（推奨）",
        (64, 128, 128),
        "最大3段階・64→128評価・改善停止で自動終了",
    ),
    "thorough": SearchModeProfile(
        "thorough",
        "徹底",
        (128, 256, 256, 256, 256, 256),
        "最大6段階・128→256評価・改善停止で自動終了",
    ),
}


def search_mode_profile(key: str) -> SearchModeProfile:
    try:
        return SEARCH_MODE_PROFILES[key]
    except KeyError as error:
        raise ValueError("unknown search mode: {}".format(key)) from error


@dataclass(frozen=True)
class CachedMergeSummary:
    """Scalar merge values retained after the preview meshes are generated.

    ``PortableMergeResult`` owns every merge group and its voxel membership.
    Those details are useful while building the preview, but saving an already
    analyzed vehicle only needs the final shape counts.  Keeping this summary
    avoids pinning all of those voxel tuples in memory for the lifetime of the
    GUI analysis result.
    """

    shape_count: int
    voxel_count: int
    status: str
    stormworks_build_id: str


@dataclass(frozen=True)
class CachedComponentOrderOptimization:
    """Compact, save-ready subset of ``ExactComponentOrderOptimization``.

    The optimized permutation is packed as native unsigned 32-bit integers.
    It is immutable at the Python level and costs four bytes per component,
    instead of retaining two tuples plus complete before/after merge groups.
    Public order properties still return tuples, matching the former result's
    observable interface for callers that inspect an analysis result.
    """

    component_count: int
    packed_optimized_component_order: bytes
    before: CachedMergeSummary
    after: CachedMergeSummary
    evaluated_order_count: int
    search: str
    completed_stage_count: int = 1
    worker_count: int = 1

    @property
    def original_component_order(self) -> Tuple[int, ...]:
        return tuple(range(self.component_count))

    @property
    def optimized_component_order(self) -> Tuple[int, ...]:
        return tuple(self.unpack_optimized_component_order())

    @property
    def changed(self) -> bool:
        return any(
            component_index != original_index
            for original_index, component_index in enumerate(
                self.unpack_optimized_component_order()
            )
        )

    def unpack_optimized_component_order(self) -> array:
        order = array("I")
        order.frombytes(self.packed_optimized_component_order)
        return order


def _cache_component_order_result(
    result: ExactComponentOrderOptimization,
) -> CachedComponentOrderOptimization:
    component_count = len(result.original_component_order)
    if any(
        component_index != original_index
        for original_index, component_index in enumerate(
            result.original_component_order
        )
    ):
        raise RuntimeError("cached analysis requires the original component order")
    order = array("I", result.optimized_component_order)
    if order.itemsize != 4:
        raise RuntimeError("this Python runtime does not provide 32-bit array('I')")
    return CachedComponentOrderOptimization(
        component_count=component_count,
        packed_optimized_component_order=order.tobytes(),
        before=CachedMergeSummary(
            shape_count=result.before.shape_count,
            voxel_count=result.before.voxel_count,
            status=result.before.status,
            stormworks_build_id=result.before.stormworks_build_id,
        ),
        after=CachedMergeSummary(
            shape_count=result.after.shape_count,
            voxel_count=result.after.voxel_count,
            status=result.after.status,
            stormworks_build_id=result.after.stormworks_build_id,
        ),
        evaluated_order_count=result.evaluated_order_count,
        search=result.search,
        completed_stage_count=result.completed_stage_count,
        worker_count=result.worker_count,
    )


@dataclass(frozen=True)
class BodyAnalysis:
    body_index: int
    body_id: str
    component_count: int
    physics_voxel_count: int
    cube_voxel_count: int
    unsupported_voxel_count: int
    overlapping_cube_count: int
    overlap_details: Tuple[str, ...]
    current_shape_count: int
    optimized_shape_count: Optional[int]
    can_optimize: bool
    reason: str
    current_boxes: Tuple[Box, ...]
    optimized_boxes: Optional[Tuple[Box, ...]]
    current_meshes: Tuple[ShapeMesh, ...]
    optimized_meshes: Optional[Tuple[ShapeMesh, ...]]
    extra_collision_shape_count: int
    physics_flooder_component_count: int
    generated_fill_voxel_count: int
    evaluated_order_count: int
    completed_search_stage_count: int = 1
    worker_count: int = 1
    physics_component_count: int = 0
    non_physics_component_count: int = 0
    multi_voxel_component_count: int = 0
    partial_volume_excluded_count: int = 0
    xml_edited_component_count: int = 0
    xml_edited_physics_voxel_count: int = 0
    protected_body: bool = False
    optimization_result: Optional[CachedComponentOrderOptimization] = None


@dataclass(frozen=True)
class VehicleAnalysis:
    vehicle_path: Path
    definitions_path: Path
    bodies: Tuple[BodyAnalysis, ...]
    warnings: Tuple[str, ...]
    search_mode: str = "custom"
    requested_worker_count: int = 1
    source_sha256: str = ""

    @property
    def component_count(self) -> int:
        return sum(body.component_count for body in self.bodies)

    @property
    def current_shape_count(self) -> int:
        return sum(body.current_shape_count for body in self.bodies)

    @property
    def optimized_shape_count(self) -> Optional[int]:
        if not self.can_optimize:
            return None
        return sum(body.optimized_shape_count or 0 for body in self.bodies)

    @property
    def xml_edited_component_count(self) -> int:
        return sum(body.xml_edited_component_count for body in self.bodies)

    @property
    def has_partial_shape_coverage(self) -> bool:
        return self.xml_edited_component_count > 0

    @property
    def protected_body_count(self) -> int:
        return sum(body.protected_body for body in self.bodies)

    @property
    def can_optimize(self) -> bool:
        return bool(self.bodies) and all(body.can_optimize for body in self.bodies)


@dataclass(frozen=True)
class OptimizationOutput:
    report: ExactVehicleOptimization
    verified_analysis: VehicleAnalysis
    sha256: str


def _boxes(result: PortableMergeResult) -> Tuple[Box, ...]:
    return tuple(Box(group.minimum, group.maximum) for group in result.groups)


def _meshes(result: PortableMergeResult) -> Tuple[ShapeMesh, ...]:
    return tuple(merge_group_mesh(group) for group in result.groups)


def _xml_edited_non_cube_component_indices(
    components: Tuple[ComponentPlacement, ...],
    voxels: Sequence[WorldVoxel],
) -> set[int]:
    """Return non-cube placements transformed outside the editor grid.

    Normal editor rotations are signed permutation matrices.  Vehicle XML can
    contain arbitrary integer scale/shear matrices.  Full cubes remain grid
    occupancy cells in the native builder, but a non-cube clip plane under
    such a matrix is outside the portable model's exact contract.
    """

    non_grid_components = {
        component_index
        for component_index, component in enumerate(components)
        if not _is_axis_aligned_unit_transform(component.effective_transform)
    }
    return {
        voxel.component_index
        for voxel in voxels
        if (
            voxel.physics_shape != 0
            and voxel.component_index in non_grid_components
        )
    }


def _unmodelled_component_indices(
    components: Tuple[ComponentPlacement, ...],
    voxels: Sequence[WorldVoxel],
) -> set[int]:
    """Detect component geometry the portable Shape model cannot represent."""

    excluded = _xml_edited_non_cube_component_indices(components, voxels)
    grid_transforms = frozenset(GRID_TRANSFORMS)
    for voxel in voxels:
        if voxel.component_index in excluded:
            continue
        if (
            voxel.physics_shape != 0
            and voxel.physics_rotation not in grid_transforms
        ):
            excluded.add(voxel.component_index)
            continue
        try:
            voxel_clip_plane(voxel)
        except ValueError:
            # One unsupported voxel makes the complete Component an ordering
            # barrier.  Splitting a multi-voxel Component would change the
            # atomic insertion contract used by Stormworks.
            excluded.add(voxel.component_index)
    return excluded


def analyze_vehicle(
    vehicle_path: Path,
    definitions_path: Path,
    beam_width: int = 8,
    max_blocks_per_body: int = 0,
    search_rounds: int = 2,
    max_evaluations: int = 64,
    progress_callback: Optional[Callable[[int, str], None]] = None,
    search_mode: Optional[str] = None,
    worker_count: int = 1,
) -> VehicleAnalysis:
    """Analyze every known shape and search safe whole-component orderings."""

    def report(percent: float, message: str) -> None:
        if progress_callback is not None:
            progress_callback(max(0, min(100, round(percent))), message)

    report(0, "車両XMLを確認中…")
    source = Path(vehicle_path)
    definitions = Path(definitions_path)
    if not source.is_file():
        raise FileNotFoundError("車両XMLが見つかりません: {}".format(source))
    if not (definitions / "01_block.xml").is_file():
        raise FileNotFoundError(
            "Stormworks definitionsフォルダが正しくありません: {}".format(
                definitions
            )
        )
    if max_blocks_per_body < 0:
        raise ValueError("component上限は0以上で指定してください")
    if worker_count < 0:
        raise ValueError("CPUワーカー数は自動（0）または1以上で指定してください")
    source_digest = _sha256_file(source)
    stage_evaluations = (
        search_mode_profile(search_mode).stage_evaluations
        if search_mode is not None
        else (max_evaluations,)
    )

    report(2, "Definitionカタログを準備中…")
    catalog = DefinitionCatalog.for_vehicle(definitions, source)
    vehicle = load_vehicle(source)
    report(5, "車両XMLを読み込みました")
    oracle = PortableMergeOracle(allow_overlaps=True)
    bodies: List[BodyAnalysis] = []
    warnings: List[str] = []
    body_count = len(vehicle.bodies)
    body_weights = tuple(max(1, len(body.components)) for body in vehicle.bodies)
    total_body_weight = max(1, sum(body_weights))
    completed_body_weight = 0
    for body_index, body in enumerate(vehicle.bodies):
        body_start = 5.0 + 93.0 * completed_body_weight / total_body_weight
        body_end = 5.0 + 93.0 * (
            completed_body_weight + body_weights[body_index]
        ) / total_body_weight

        def report_body(fraction: float, message: str) -> None:
            report(
                body_start + (body_end - body_start) * fraction,
                "Body {}/{}: {}".format(body_index + 1, body_count, message),
            )

        report_body(0.0, "Component Definitionを読み込み中…")
        definitions_in_body = tuple(
            catalog.load(component.definition_id) for component in body.components
        )
        flooder_count = sum(
            definition.water_component_type == 19
            for definition in definitions_in_body
        )
        extra_box_count = sum(
            definition.contributes_physics_extra_box
            for definition in definitions_in_body
        )
        static_voxels = vehicle.physics_voxels(catalog, body_index)
        report_body(
            0.15,
            "Physics voxelを展開しました（{} voxel）".format(len(static_voxels)),
        )
        unmodelled_components = _unmodelled_component_indices(
            body.components,
            static_voxels,
        )
        protected_body = bool(unmodelled_components)
        model_excluded_components = (
            set(range(len(body.components)))
            if protected_body
            else set()
        )
        static_voxel_count_by_component = [0] * len(body.components)
        for voxel in static_voxels:
            static_voxel_count_by_component[voxel.component_index] += 1
        unmodelled_physics_components = {
            component_index
            for component_index in unmodelled_components
            if static_voxel_count_by_component[component_index]
        }
        unmodelled_physics_voxel_count = sum(
            static_voxel_count_by_component[component_index]
            for component_index in unmodelled_physics_components
        )
        protected_physics_components = {
            component_index
            for component_index, voxel_count in enumerate(
                static_voxel_count_by_component
            )
            if protected_body and voxel_count
        }
        cube_count = sum(voxel.physics_shape == 0 for voxel in static_voxels)
        non_cube_count = len(static_voxels) - cube_count
        first_voxel_by_position = {}
        overlapping_voxels_by_position = {}
        for voxel in static_voxels:
            first_voxel = first_voxel_by_position.get(voxel.position)
            if first_voxel is None:
                first_voxel_by_position[voxel.position] = voxel
                continue
            group = overlapping_voxels_by_position.get(voxel.position)
            if group is None:
                overlapping_voxels_by_position[voxel.position] = [
                    first_voxel,
                    voxel,
                ]
            else:
                group.append(voxel)
        overlap_groups = tuple(
            (position, tuple(voxels))
            for position, voxels in sorted(
                overlapping_voxels_by_position.items()
            )
        )
        # Do not retain a full-body coordinate table during the expensive
        # ordering search. Only the usually tiny repeated-position groups are
        # needed from this point onward.
        del first_voxel_by_position
        del overlapping_voxels_by_position
        overlap_count = len(overlap_groups)
        overlap_details = tuple(
            "{}: {}".format(
                position,
                " / ".join(
                    "Component {} {} voxel {} shape {}".format(
                        voxel.component_index,
                        voxel.component_definition,
                        voxel.definition_voxel_index,
                        voxel.physics_shape,
                    )
                    for voxel in voxels
                ),
            )
            for position, voxels in overlap_groups
        )

        can_optimize = True
        reason = "全physics shape対応のportable exactモデルで最適化できます"
        # Current preview is independent from optimization eligibility.  In
        # particular, intentional Definition-level overlaps must never erase
        # a whole body from the 3D view.
        current: Optional[PortableMergeResult] = None
        preview_voxels = tuple(
            voxel
            for voxel in static_voxels
            if voxel.component_index not in model_excluded_components
        )
        optimized: Optional[PortableMergeResult] = None
        evaluated = 0
        completed_search_stage_count = 0
        resolved_worker_count = 0
        generated_fill_count = 0
        physics_component_count = 0
        non_physics_component_count = 0
        multi_voxel_component_count = 0
        partial_volume_excluded_count = 0
        order_result: Optional[ExactComponentOrderOptimization] = None
        try:
            if flooder_count and not protected_body:
                flood_fill = model_surface_physics_flood_fill(
                    vehicle,
                    catalog,
                    body_index,
                    progress_callback=lambda fraction, message: report_body(
                        0.16 + 0.12 * fraction, message
                    ),
                    static_voxels=static_voxels,
                    component_definitions=definitions_in_body,
                )
                if not flood_fill.supported:
                    unsupported_status = flood_fill.status
                    del flood_fill
                    raise UnsupportedVehicleError(
                        "Physics Flooderの面モデルが未対応です: {}".format(
                            unsupported_status
                        )
                    )
                grouped_voxels = tuple(
                    voxel
                    for voxel in flood_fill.static_voxels_after_fill
                    if voxel.component_index not in model_excluded_components
                )
                trailing_voxels = flood_fill.new_fill_voxels
                partial_volume_excluded_count = (
                    flood_fill.partial_volume_excluded_count
                )
                # The result also owns the full prepared/static voxel tuples.
                # Only the two selected sequences and scalar count are needed
                # below; release the wrapper now so a large Flooder body is
                # not retained while every later body is analyzed.
                del flood_fill
            else:
                # ``model_surface_physics_flood_fill`` expands the body's
                # physics voxels itself.  Calling it for a body without a
                # Flooder duplicated the largest per-body tuple for no change
                # in output, so retain the expansion already performed above.
                grouped_voxels = tuple(
                    voxel
                    for voxel in static_voxels
                    if voxel.component_index not in model_excluded_components
                )
                trailing_voxels = ()
            preview_voxels = grouped_voxels + trailing_voxels
            generated_fill_count = len(trailing_voxels)
            report_body(
                0.28,
                "Physics Flooderを解析しました（充填{} voxel）".format(
                    generated_fill_count
                ),
            )
            if max_blocks_per_body and len(body.components) > max_blocks_per_body:
                raise UnsupportedVehicleError(
                    "{} componentsあり、設定上限の{}を超えています".format(
                        len(body.components), max_blocks_per_body
                    )
                )
            component_voxels = validate_mixed_component_groups(
                body_index,
                body.components,
                grouped_voxels,
                require_single_voxel=False,
                allow_empty=True,
                allow_non_unit_transform=True,
                allow_overlaps=True,
            )
            physics_component_indices = tuple(
                index
                for index, group in enumerate(component_voxels)
                if group or index in protected_physics_components
            )
            physics_groups = tuple(
                component_voxels[index] for index in physics_component_indices
            )
            physics_component_count = len(physics_component_indices)
            non_physics_component_count = (
                len(body.components) - physics_component_count
            )
            multi_voxel_component_count = sum(
                voxel_count > 1
                for voxel_count in static_voxel_count_by_component
            )
            # The position table above already found every repeated static
            # voxel. Flood fill never creates a duplicate position, so reuse
            # it instead of building the same full-body coordinate map again.
            overlapping_components = {
                voxel.component_index
                for _position, voxels in overlap_groups
                for voxel in voxels
            }
            fixed_physics_indices = tuple(
                local_index
                for local_index, component_index in enumerate(
                    physics_component_indices
                )
                if (
                    component_index in overlapping_components
                    or component_index in protected_physics_components
                )
            )

            def report_search(
                stage_index: int,
                stage_count: int,
                evaluated_count: int,
                expected_count: int,
                best_shape_count: int,
                resolved_workers: int,
            ) -> None:
                completed_budget = sum(stage_evaluations[: stage_index - 1])
                total_budget = max(1, sum(stage_evaluations))
                fraction = min(
                    1.0,
                    (completed_budget + evaluated_count) / total_budget,
                )
                report_body(
                    0.34 + 0.51 * fraction,
                    "探索 {}/{}：配置候補 {}/{}・CPU {}・現在の最良 {} Shapes".format(
                        stage_index,
                        stage_count,
                        evaluated_count,
                        expected_count,
                        resolved_workers,
                        best_shape_count,
                    ),
                )

            if protected_body:
                current = oracle.partition(())
                identity_order = tuple(range(len(body.components)))
                order_result = ExactComponentOrderOptimization(
                    original_component_order=identity_order,
                    optimized_component_order=identity_order,
                    before=current,
                    after=current,
                    evaluated_order_count=0,
                    search="protected_xml_edited_body_identity",
                    completed_stage_count=0,
                    worker_count=0,
                )
                report_body(
                    0.85,
                    "XML編集Shapeを保護するためBody全体の元順序を保持しました",
                )
            else:
                order_result = optimize_staged_component_order(
                    physics_groups,
                    oracle,
                    stage_evaluations=stage_evaluations,
                    beam_width=beam_width,
                    search_rounds=search_rounds,
                    trailing_voxels=trailing_voxels,
                    search_backend="portable_exact",
                    fixed_component_indices=fixed_physics_indices,
                    progress_callback=report_search,
                    worker_count=worker_count,
                )
                full_order = pinned_non_physics_component_order(
                    len(body.components),
                    physics_component_indices,
                    order_result.optimized_component_order,
                )
                order_result = replace(
                    order_result,
                    original_component_order=tuple(range(len(body.components))),
                    optimized_component_order=full_order,
                )
            optimized = order_result.after
            current = order_result.before
            evaluated = order_result.evaluated_order_count
            completed_search_stage_count = order_result.completed_stage_count
            resolved_worker_count = order_result.worker_count
            if overlapping_components:
                reason = (
                    "{}重複座標に関係する{} Componentを元の順序位置へ固定し、"
                    "残りを最適化できます"
                ).format(overlap_count, len(overlapping_components))
            if unmodelled_components:
                reason = (
                    "XML編集または未対応Shapeの{} Componentがあるため、"
                    "相互作用を変えないようBody全体を元の順序に固定しました"
                ).format(len(unmodelled_components))
                warnings.append(
                    "Body {}: XML編集または未対応Shapeの{} Component "
                    "({} physics voxel)を含むためBody全体を順序固定・"
                    "予測対象外にしました".format(
                        body_index,
                        len(unmodelled_components),
                        unmodelled_physics_voxel_count,
                    )
                )
            if current.shape_count != order_result.before.shape_count:
                raise RuntimeError("解析前Shape数が探索入力と一致しません")
        except (UnsupportedVehicleError, ValueError) as error:
            if current is None:
                current = oracle.partition(preview_voxels)
            can_optimize = False
            reason = str(error)
            warnings.append("Body {}: {}".format(body_index, error))

        if current is None:
            raise RuntimeError("現在Shapeの解析結果がありません")
        report_body(0.9, "3Dプレビューを生成中…")
        bodies.append(
            BodyAnalysis(
                body_index=body_index,
                body_id=body.body_id,
                component_count=len(body.components),
                physics_voxel_count=len(static_voxels) + generated_fill_count,
                cube_voxel_count=cube_count,
                unsupported_voxel_count=non_cube_count,
                overlapping_cube_count=overlap_count,
                overlap_details=overlap_details,
                current_shape_count=current.shape_count,
                optimized_shape_count=(
                    optimized.shape_count if optimized is not None else None
                ),
                can_optimize=can_optimize,
                reason=reason,
                current_boxes=_boxes(current),
                optimized_boxes=_boxes(optimized) if optimized is not None else None,
                current_meshes=_meshes(current),
                optimized_meshes=_meshes(optimized) if optimized is not None else None,
                extra_collision_shape_count=extra_box_count,
                physics_flooder_component_count=flooder_count,
                generated_fill_voxel_count=generated_fill_count,
                evaluated_order_count=evaluated,
                completed_search_stage_count=completed_search_stage_count,
                worker_count=resolved_worker_count,
                physics_component_count=physics_component_count,
                non_physics_component_count=non_physics_component_count,
                multi_voxel_component_count=multi_voxel_component_count,
                partial_volume_excluded_count=partial_volume_excluded_count,
                xml_edited_component_count=len(unmodelled_components),
                xml_edited_physics_voxel_count=(
                    unmodelled_physics_voxel_count
                ),
                protected_body=protected_body,
                optimization_result=(
                    _cache_component_order_result(order_result)
                    if can_optimize and order_result is not None
                    else None
                ),
            )
        )
        report_body(1.0, "完了")
        completed_body_weight += body_weights[body_index]
    if not bodies:
        warnings.append("bodyが見つかりません")
    if _sha256_file(source) != source_digest:
        raise RuntimeError("解析中に元の車両XMLが変更されました。もう一度解析してください")
    report(100, "解析完了")
    return VehicleAnalysis(
        source,
        definitions,
        tuple(bodies),
        tuple(warnings),
        search_mode=search_mode or "custom",
        requested_worker_count=worker_count,
        source_sha256=source_digest,
    )


def _placement_signature(component: ComponentPlacement) -> tuple:
    """Compare component data while ignoring its XML-order-derived index."""

    return (
        component.definition_id,
        component.transform_index,
        component.position,
        component.rotation,
        component.microprocessor_width,
        component.microprocessor_length,
    )


def save_analyzed_vehicle_copy(
    analysis: VehicleAnalysis,
    output_path: Path,
    force: bool = False,
    progress_callback: Optional[Callable[[int, str], None]] = None,
) -> OptimizationOutput:
    """Write the already-computed component order without running analysis again."""

    def report(percent: float, message: str) -> None:
        if progress_callback is not None:
            progress_callback(max(0, min(100, round(percent))), message)

    source = Path(analysis.vehicle_path)
    destination = Path(output_path)
    report(0, "解析済み結果と元XMLを確認中…")
    if not analysis.can_optimize:
        raise ValueError("この解析結果は最適化コピーを保存できません")
    if not analysis.source_sha256:
        raise ValueError("保存に必要な解析済みデータがありません。もう一度解析してください")
    if not source.is_file():
        raise FileNotFoundError("元の車両XMLが見つかりません: {}".format(source))
    if source.resolve() == destination.resolve():
        raise ValueError("元の車両XMLとは別の保存先を選んでください")
    if destination.exists() and not force:
        raise FileExistsError("出力先は既に存在します: {}".format(destination))
    if _sha256_file(source) != analysis.source_sha256:
        raise ValueError(
            "解析後に元の車両XMLが変更されました。もう一度解析してください"
        )
    if any(body.optimization_result is None for body in analysis.bodies):
        raise ValueError("Component順序の解析結果が不足しています。もう一度解析してください")

    report(12, "元XMLのComponent構造を読み込み中…")
    vehicle = load_vehicle(source)
    tree = _parse_with_comments(source)
    body_elements = tree.getroot().findall("./bodies/body")
    if len(vehicle.bodies) != len(analysis.bodies):
        raise ValueError("解析時からBody数が変わっています")
    if len(body_elements) != len(vehicle.bodies):
        raise ValueError("XMLのBody構造を正しく読み取れません")

    catalog = DefinitionCatalog.for_vehicle(analysis.definitions_path, source)
    for body in vehicle.bodies:
        for component in body.components:
            catalog.load(component.definition_id)

    body_results: List[ExactBodyOptimization] = []
    component_orders: List[array] = []
    body_count = len(vehicle.bodies)
    for body_index, (vehicle_body, body_element, body_analysis) in enumerate(
        zip(vehicle.bodies, body_elements, analysis.bodies)
    ):
        report(
            20 + 35 * body_index / max(1, body_count),
            "Body {}/{}: 解析済みComponent順序を適用中…".format(
                body_index + 1,
                body_count,
            ),
        )
        if body_analysis.body_index != body_index:
            raise ValueError("解析済みBody順序が元XMLと一致しません")
        if body_analysis.body_id != vehicle_body.body_id:
            raise ValueError("解析時からBody IDが変わっています")
        components_element = body_element.find("components")
        component_elements = (
            [] if components_element is None else list(components_element)
        )
        if any(element.tag != "c" for element in component_elements):
            raise UnsupportedVehicleError(
                "body {} contains non-component children inside <components>".format(
                    body_index
                )
            )
        if len(component_elements) != len(vehicle_body.components):
            raise ValueError("body {} component count mismatch".format(body_index))
        result = body_analysis.optimization_result
        if result is None:
            raise ValueError("body {}の解析済み順序がありません".format(body_index))
        # Decode only this body's packed permutation while it is being
        # written; do not materialize and retain a tuple for every body.
        order = result.unpack_optimized_component_order()
        seen_component_indices = bytearray(len(component_elements))
        valid_order = len(order) == len(component_elements)
        if valid_order:
            for component_index in order:
                if (
                    component_index >= len(component_elements)
                    or seen_component_indices[component_index]
                ):
                    valid_order = False
                    break
                seen_component_indices[component_index] = 1
        if not valid_order:
            raise ValueError("body {}の解析済み順序が不正です".format(body_index))
        component_orders.append(order)
        body_results.append(
            ExactBodyOptimization(
                body_index=body_index,
                body_id=body_analysis.body_id,
                component_count=body_analysis.component_count,
                physics_voxel_count=body_analysis.physics_voxel_count,
                physics_component_count=body_analysis.physics_component_count,
                non_physics_component_count=body_analysis.non_physics_component_count,
                multi_voxel_component_count=body_analysis.multi_voxel_component_count,
                extra_box_count=body_analysis.extra_collision_shape_count,
                physics_flooder_component_count=(
                    body_analysis.physics_flooder_component_count
                ),
                generated_fill_voxel_count=body_analysis.generated_fill_voxel_count,
                partial_volume_excluded_count=(
                    body_analysis.partial_volume_excluded_count
                ),
                result=result,
                xml_edited_component_count=(
                    body_analysis.xml_edited_component_count
                ),
                xml_edited_physics_voxel_count=(
                    body_analysis.xml_edited_physics_voxel_count
                ),
                protected_body=body_analysis.protected_body,
            )
        )

    report(58, "元XMLのコンパクトな書式を保って書き出し中…")
    package_plan = plan_component_package(catalog, destination, force)
    destination.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=destination.name + ".",
        suffix=".tmp",
        dir=str(destination.parent),
    )
    os.close(file_descriptor)
    try:
        write_vehicle_component_order_preserving_source(
            source,
            Path(temporary_name),
            component_orders,
        )
        report(76, "保存前にXML構造を再読込中…")
        verified_vehicle = load_vehicle(Path(temporary_name))
        if len(verified_vehicle.bodies) != len(body_results):
            raise RuntimeError("書き出し後にBody数が変わりました")
        for body_index, (verified_body, source_body, body_analysis) in enumerate(
            zip(verified_vehicle.bodies, vehicle.bodies, analysis.bodies)
        ):
            report(
                78 + 16 * (body_index + 1) / max(1, body_count),
                "Body {}/{}: Component順序と属性を検証中…".format(
                    body_index + 1,
                    body_count,
                ),
            )
            result = body_analysis.optimization_result
            if result is None:
                raise RuntimeError(
                    "body {}の解析済み順序がありません".format(body_index)
                )
            order = result.unpack_optimized_component_order()
            if len(verified_body.components) != len(order) or any(
                _placement_signature(actual)
                != _placement_signature(source_body.components[source_index])
                for actual, source_index in zip(verified_body.components, order)
            ):
                raise RuntimeError(
                    "body {}のComponent順序または属性が保存後に変化しました".format(
                        body_index
                    )
                )
        install_component_package(package_plan)
        os.replace(temporary_name, destination)
    except Exception:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)
        raise

    digest = _sha256_file(destination)
    oracle = PortableMergeOracle(allow_overlaps=True)
    report(100, "解析済み結果から保存完了（再解析なし）")
    return OptimizationOutput(
        report=ExactVehicleOptimization(
            input_path=source,
            output_path=destination,
            backend="portable_exact_cached_analysis",
            binary_path=oracle.binary_path,
            binary_sha256=oracle.binary_sha256,
            output_reload_verified=True,
            bodies=tuple(body_results),
            component_bin_count=package_plan.component_bin_count,
            component_package_path=(
                package_plan.output_root
                if package_plan.component_bin_count
                else None
            ),
        ),
        verified_analysis=analysis,
        sha256=digest,
    )


def optimize_vehicle_copy(
    vehicle_path: Path,
    output_path: Path,
    definitions_path: Path,
    beam_width: int = 8,
    max_blocks_per_body: int = 0,
    search_rounds: int = 2,
    max_evaluations: int = 64,
    force: bool = False,
    search_mode: Optional[str] = None,
    worker_count: int = 1,
    progress_callback: Optional[Callable[[int, str], None]] = None,
) -> OptimizationOutput:
    def emit_progress(percent: float, message: str) -> None:
        if progress_callback is not None:
            progress_callback(max(0, min(100, round(percent))), message)

    stage_evaluations = (
        search_mode_profile(search_mode).stage_evaluations
        if search_mode is not None
        else None
    )
    catalog = DefinitionCatalog.for_vehicle(
        Path(definitions_path), Path(vehicle_path)
    )
    report = optimize_vehicle_portable_exact(
        Path(vehicle_path),
        Path(output_path),
        catalog,
        beam_width=beam_width,
        search_rounds=search_rounds,
        max_evaluations=max_evaluations,
        max_components_per_body=max_blocks_per_body,
        force=force,
        stage_evaluations=stage_evaluations,
        worker_count=worker_count,
        progress_callback=lambda percent, message: emit_progress(
            percent * 0.9,
            message,
        ),
    )
    emit_progress(90, "保存したXMLをアプリモデルで最終確認中…")
    verified = analyze_vehicle(
        Path(output_path),
        Path(definitions_path),
        beam_width=beam_width,
        max_blocks_per_body=max_blocks_per_body,
        search_rounds=search_rounds,
        max_evaluations=1,
        progress_callback=lambda percent, message: emit_progress(
            90 + percent * 0.1,
            message,
        ),
        search_mode=None,
        worker_count=1,
    )
    visible_after = sum(body.result.after.shape_count for body in report.bodies)
    if verified.current_shape_count != visible_after:
        raise AssertionError("書き出し後のShape数が最適化レポートと一致しません")
    digest = _sha256_file(Path(output_path))
    emit_progress(100, "保存・再検証完了")
    return OptimizationOutput(report=report, verified_analysis=verified, sha256=digest)
