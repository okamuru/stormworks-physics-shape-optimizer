from dataclasses import dataclass, replace
import os
from pathlib import Path
import tempfile
from typing import Dict, FrozenSet, List, Mapping, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

from .definitions import DefinitionCatalog
from .component_package import install_component_package, plan_component_package
from .geometry import MixedPartitionResult, partition_mixed_shapes_greedy
from .model import GridPoint, Matrix3, WorldVoxel, unique_points
from .partition import Box, PartitionResult, grow_cube_box, partition_cubes_greedy
from .source_preserving_xml import write_vehicle_component_order_preserving_source
from .vehicle import (
    ComponentPlacement,
    load_vehicle,
    parse_vehicle_tree,
)


class UnsupportedVehicleError(ValueError):
    pass


@dataclass(frozen=True)
class CubeOrderOptimization:
    original_order: Tuple[GridPoint, ...]
    optimized_order: Tuple[GridPoint, ...]
    before: PartitionResult
    after: PartitionResult
    search: str

    @property
    def changed(self) -> bool:
        return self.original_order != self.optimized_order


@dataclass(frozen=True)
class ComponentOrderOptimization:
    original_component_order: Tuple[int, ...]
    optimized_component_order: Tuple[int, ...]
    original_voxel_order: Tuple[GridPoint, ...]
    optimized_voxel_order: Tuple[GridPoint, ...]
    before: PartitionResult
    after: PartitionResult
    search: str

    @property
    def changed(self) -> bool:
        return self.original_component_order != self.optimized_component_order


@dataclass(frozen=True)
class BodyOptimization:
    body_index: int
    body_id: str
    component_count: int
    result: ComponentOrderOptimization


@dataclass(frozen=True)
class VehicleOptimization:
    input_path: Path
    output_path: Path
    bodies: Tuple[BodyOptimization, ...]
    component_bin_count: int = 0
    component_package_path: Optional[Path] = None

    @property
    def before_shape_count(self) -> int:
        return sum(body.result.before.shape_count for body in self.bodies)

    @property
    def after_shape_count(self) -> int:
        return sum(body.result.after.shape_count for body in self.bodies)


@dataclass(frozen=True)
class MixedComponentOrderOptimization:
    original_component_order: Tuple[int, ...]
    optimized_component_order: Tuple[int, ...]
    before: MixedPartitionResult
    after: MixedPartitionResult
    search: str

    @property
    def changed(self) -> bool:
        return self.original_component_order != self.optimized_component_order


@dataclass(frozen=True)
class MixedBodyOptimization:
    body_index: int
    body_id: str
    component_count: int
    result: MixedComponentOrderOptimization


@dataclass(frozen=True)
class MixedVehicleOptimization:
    input_path: Path
    output_path: Path
    bodies: Tuple[MixedBodyOptimization, ...]
    component_bin_count: int = 0
    component_package_path: Optional[Path] = None

    @property
    def before_shape_count(self) -> int:
        return sum(body.result.before.shape_count for body in self.bodies)

    @property
    def after_shape_count(self) -> int:
        return sum(body.result.after.shape_count for body in self.bodies)


@dataclass(frozen=True)
class _SearchState:
    remaining: FrozenSet[GridPoint]
    order: Tuple[GridPoint, ...]
    boxes: Tuple[Box, ...]


@dataclass(frozen=True)
class _ComponentSearchState:
    remaining_components: FrozenSet[int]
    remaining_points: FrozenSet[GridPoint]
    component_order: Tuple[int, ...]
    boxes: Tuple[Box, ...]


def _point_key(point: GridPoint, original_index: Dict[GridPoint, int]) -> Tuple[int, GridPoint]:
    return (original_index[point], point)


def optimize_cube_order(points: Sequence[GridPoint], beam_width: int = 128) -> CubeOrderOptimization:
    """Find a lower-shape XML seed order for unique unit cubes.

    The search chooses a seed, claims the xyz-grown box, and keeps a bounded
    set of promising remaining-voxel states. It never returns an order worse
    than the original order under the validated A/B/C model.
    """

    if beam_width < 1:
        raise ValueError("beam width must be at least 1")
    original = unique_points(points)
    if len(original) != len(points):
        raise ValueError("overlapping cube positions cannot be optimized safely")
    before = partition_cubes_greedy(original, axis_order="xyz", seed_order="xml")
    if not original:
        return CubeOrderOptimization(original, original, before, before, "empty_body")

    original_index = {point: index for index, point in enumerate(original)}
    active = [_SearchState(frozenset(original), (), ())]
    completed: List[_SearchState] = []

    for _depth in range(len(original)):
        next_by_remaining: Dict[FrozenSet[GridPoint], _SearchState] = {}
        completed = []
        for state in active:
            seen_boxes = set()
            for seed in sorted(state.remaining, key=lambda point: _point_key(point, original_index)):
                box = grow_cube_box(state.remaining, seed, axis_order="xyz")
                box_signature = (box.minimum, box.maximum)
                if box_signature in seen_boxes:
                    continue
                seen_boxes.add(box_signature)
                claimed = frozenset(box.points())
                rest_of_box = sorted(
                    claimed - {seed}, key=lambda point: _point_key(point, original_index)
                )
                candidate = _SearchState(
                    remaining=state.remaining - claimed,
                    order=state.order + (seed,) + tuple(rest_of_box),
                    boxes=state.boxes + (box,),
                )
                if not candidate.remaining:
                    completed.append(candidate)
                    continue
                previous = next_by_remaining.get(candidate.remaining)
                candidate_key = tuple(original_index[point] for point in candidate.order)
                previous_key = (
                    tuple(original_index[point] for point in previous.order)
                    if previous is not None
                    else None
                )
                if previous is None or candidate_key < previous_key:
                    next_by_remaining[candidate.remaining] = candidate
        if completed:
            break
        active = sorted(
            next_by_remaining.values(),
            key=lambda state: (
                len(state.remaining),
                tuple(original_index[point] for point in state.order),
            ),
        )[:beam_width]
        if not active:
            raise AssertionError("cube-order search ended without a complete order")

    if not completed:
        raise AssertionError("cube-order search did not produce a complete order")
    best = min(
        completed,
        key=lambda state: (
            len(state.boxes),
            tuple(original_index[point] for point in state.order),
        ),
    )
    candidate_after = partition_cubes_greedy(best.order, axis_order="xyz", seed_order="xml")
    if candidate_after.shape_count < before.shape_count:
        optimized = best.order
        after = candidate_after
    else:
        optimized = original
        after = before
    return CubeOrderOptimization(
        original_order=original,
        optimized_order=optimized,
        before=before,
        after=after,
        search="bounded_seed_box_beam(width={})".format(beam_width),
    )


def _flatten_component_points(
    component_points: Sequence[Sequence[GridPoint]], component_order: Sequence[int]
) -> Tuple[GridPoint, ...]:
    return tuple(
        point
        for component_index in component_order
        for point in component_points[component_index]
    )


def _consume_component(
    remaining_points: FrozenSet[GridPoint], points: Sequence[GridPoint]
) -> Tuple[FrozenSet[GridPoint], Tuple[Box, ...]]:
    remaining = set(remaining_points)
    boxes: List[Box] = []
    for seed in points:
        if seed not in remaining:
            continue
        box = grow_cube_box(remaining, seed, axis_order="xyz")
        remaining.difference_update(box.points())
        boxes.append(box)
    return frozenset(remaining), tuple(boxes)


def optimize_component_cube_order(
    component_points: Sequence[Sequence[GridPoint]], beam_width: int = 128
) -> ComponentOrderOptimization:
    """Optimize XML component order while preserving each component's voxel order.

    Stormworks inserts a component's definition voxels as a group. The search
    may therefore permute component groups, but never splits or reorders the
    physics voxels inside one component.
    """

    if beam_width < 1:
        raise ValueError("beam width must be at least 1")
    groups = tuple(tuple(points) for points in component_points)
    if any(not points for points in groups):
        raise ValueError("every optimized component must contribute at least one physics cube")
    original_components = tuple(range(len(groups)))
    original_voxels = _flatten_component_points(groups, original_components)
    if len(unique_points(original_voxels)) != len(original_voxels):
        raise ValueError("overlapping cube positions cannot be optimized safely")
    before = partition_cubes_greedy(original_voxels, axis_order="xyz", seed_order="xml")
    if len(groups) <= 1:
        return ComponentOrderOptimization(
            original_components,
            original_components,
            original_voxels,
            original_voxels,
            before,
            before,
            "component_group_no_reorder_needed",
        )

    active = [
        _ComponentSearchState(
            remaining_components=frozenset(original_components),
            remaining_points=frozenset(original_voxels),
            component_order=(),
            boxes=(),
        )
    ]
    for _depth in range(len(groups)):
        next_by_state: Dict[
            Tuple[FrozenSet[int], FrozenSet[GridPoint]], _ComponentSearchState
        ] = {}
        for state in active:
            for component_index in sorted(state.remaining_components):
                remaining, added_boxes = _consume_component(
                    state.remaining_points, groups[component_index]
                )
                candidate = _ComponentSearchState(
                    remaining_components=state.remaining_components - {component_index},
                    remaining_points=remaining,
                    component_order=state.component_order + (component_index,),
                    boxes=state.boxes + added_boxes,
                )
                state_key = (candidate.remaining_components, candidate.remaining_points)
                previous = next_by_state.get(state_key)
                candidate_key = (len(candidate.boxes), candidate.component_order)
                previous_key = (
                    (len(previous.boxes), previous.component_order)
                    if previous is not None
                    else None
                )
                if previous is None or candidate_key < previous_key:
                    next_by_state[state_key] = candidate
        active = sorted(
            next_by_state.values(),
            key=lambda state: (
                len(state.boxes),
                len(state.remaining_points),
                state.component_order,
            ),
        )[:beam_width]
        if not active:
            raise AssertionError("component-order search ended without a complete order")

    completed = [
        state
        for state in active
        if not state.remaining_components and not state.remaining_points
    ]
    if not completed:
        raise AssertionError("component-order search did not consume all cube voxels")
    best = min(completed, key=lambda state: (len(state.boxes), state.component_order))
    candidate_voxels = _flatten_component_points(groups, best.component_order)
    candidate_after = partition_cubes_greedy(
        candidate_voxels, axis_order="xyz", seed_order="xml"
    )
    if candidate_after.shape_count != len(best.boxes):
        raise AssertionError("component search state disagrees with final cube partition")
    if candidate_after.shape_count < before.shape_count:
        optimized_components = best.component_order
        optimized_voxels = candidate_voxels
        after = candidate_after
    else:
        optimized_components = original_components
        optimized_voxels = original_voxels
        after = before
    return ComponentOrderOptimization(
        original_component_order=original_components,
        optimized_component_order=optimized_components,
        original_voxel_order=original_voxels,
        optimized_voxel_order=optimized_voxels,
        before=before,
        after=after,
        search="bounded_component_group_beam(width={})".format(beam_width),
    )


def _parse_with_comments(path: Path) -> ET.ElementTree:
    return parse_vehicle_tree(path, insert_comments=True)


def _is_axis_aligned_unit_transform(matrix: Matrix3) -> bool:
    rows = tuple(matrix[offset : offset + 3] for offset in (0, 3, 6))
    return all(sum(abs(value) for value in row) == 1 for row in rows) and all(
        sum(abs(rows[row][column]) for row in range(3)) == 1 for column in range(3)
    )


def validate_cube_component_groups(
    body_index: int,
    placements: Sequence[ComponentPlacement],
    voxels: Sequence[WorldVoxel],
    require_single_voxel: bool = False,
) -> Tuple[Tuple[GridPoint, ...], ...]:
    groups: List[List[GridPoint]] = [[] for _placement in placements]
    for component_index, placement in enumerate(placements):
        if not _is_axis_aligned_unit_transform(placement.effective_transform):
            raise UnsupportedVehicleError(
                "body {} component {} ({}) uses a stretched or non-grid transform".format(
                    body_index, component_index, placement.definition_id
                )
            )
    for voxel in voxels:
        if voxel.component_index < 0 or voxel.component_index >= len(groups):
            raise ValueError("physics voxel component index is out of range")
        if voxel.physics_shape != 0:
            raise UnsupportedVehicleError(
                "body {} component {} ({}) contains non-cube physics_shape {}".format(
                    body_index,
                    voxel.component_index,
                    voxel.component_definition,
                    voxel.physics_shape,
                )
            )
        groups[voxel.component_index].append(voxel.position)
    for component_index, (placement, points) in enumerate(zip(placements, groups)):
        if not points:
            raise UnsupportedVehicleError(
                "body {} component {} ({}) contributes no physics cubes".format(
                    body_index, component_index, placement.definition_id
                )
            )
    flattened = tuple(point for points in groups for point in points)
    if len(unique_points(flattened)) != len(flattened):
        raise UnsupportedVehicleError(
            "body {} contains overlapping cube physics voxels".format(body_index)
        )
    if require_single_voxel:
        for component_index, (placement, points) in enumerate(zip(placements, groups)):
            if len(points) != 1:
                raise UnsupportedVehicleError(
                    "body {} component {} ({}) contributes {} physics voxels; "
                    "Stage 4 showed that multi-voxel components require the exact "
                    "merge_shape model".format(
                        body_index, component_index, placement.definition_id, len(points)
                    )
                )
    return tuple(tuple(points) for points in groups)


def validate_mixed_component_groups(
    body_index: int,
    placements: Sequence[ComponentPlacement],
    voxels: Sequence[WorldVoxel],
    require_single_voxel: bool = False,
    allow_empty: bool = False,
    allow_non_unit_transform: bool = False,
    allow_overlaps: bool = False,
) -> Tuple[Tuple[WorldVoxel, ...], ...]:
    groups: List[List[WorldVoxel]] = [[] for _placement in placements]
    for component_index, placement in enumerate(placements):
        if (
            not allow_non_unit_transform
            and not _is_axis_aligned_unit_transform(placement.effective_transform)
        ):
            raise UnsupportedVehicleError(
                "body {} component {} ({}) uses a stretched or non-grid transform".format(
                    body_index, component_index, placement.definition_id
                )
            )
    for voxel in voxels:
        if voxel.component_index < 0 or voxel.component_index >= len(groups):
            raise ValueError("physics voxel component index is out of range")
        if not 0 <= voxel.physics_shape <= 41:
            raise UnsupportedVehicleError(
                "body {} component {} ({}) has unknown physics_shape {}".format(
                    body_index,
                    voxel.component_index,
                    voxel.component_definition,
                    voxel.physics_shape,
                )
            )
        groups[voxel.component_index].append(voxel)
    for component_index, (placement, group) in enumerate(zip(placements, groups)):
        if not group and not allow_empty:
            raise UnsupportedVehicleError(
                "body {} component {} ({}) contributes no physics voxels".format(
                    body_index, component_index, placement.definition_id
                )
            )
    positions = [voxel.position for group in groups for voxel in group]
    if not allow_overlaps and len(positions) != len(set(positions)):
        raise UnsupportedVehicleError(
            "body {} contains overlapping physics voxel positions".format(body_index)
        )
    if require_single_voxel:
        for component_index, (placement, group) in enumerate(zip(placements, groups)):
            if len(group) != 1:
                raise UnsupportedVehicleError(
                    "body {} component {} ({}) contributes {} physics voxels; "
                    "Stage 4 showed that multi-voxel components require the exact "
                    "merge_shape model".format(
                        body_index, component_index, placement.definition_id, len(group)
                    )
                )
    return tuple(tuple(group) for group in groups)


def _ordered_mixed_voxels(
    groups: Sequence[Sequence[WorldVoxel]], component_order: Sequence[int]
) -> Tuple[WorldVoxel, ...]:
    flattened = [
        voxel
        for component_index in component_order
        for voxel in groups[component_index]
    ]
    return tuple(
        replace(voxel, insertion_index=insertion_index)
        for insertion_index, voxel in enumerate(flattened)
    )


def optimize_mixed_component_order(
    component_voxels: Sequence[Sequence[WorldVoxel]],
    definition_flags: Mapping[str, int],
    beam_width: int = 12,
    search_rounds: int = 3,
) -> MixedComponentOrderOptimization:
    """Experimentally search whole-component moves for cube and non-cube bodies."""

    if beam_width < 1:
        raise ValueError("beam width must be at least 1")
    if search_rounds < 1:
        raise ValueError("search rounds must be at least 1")
    groups = tuple(tuple(group) for group in component_voxels)
    if any(not group for group in groups):
        raise ValueError("every optimized component must contribute physics voxels")
    original_order = tuple(range(len(groups)))
    cache: Dict[Tuple[int, ...], MixedPartitionResult] = {}

    def evaluate(order: Tuple[int, ...]) -> MixedPartitionResult:
        if order not in cache:
            cache[order] = partition_mixed_shapes_greedy(
                _ordered_mixed_voxels(groups, order), definition_flags
            )
        return cache[order]

    before = evaluate(original_order)
    best_order = original_order
    active = (original_order,)
    seen = {original_order}
    for _round in range(search_rounds):
        candidates = set(active)
        for order in active:
            for source_index in range(len(order)):
                for target_index in range(len(order)):
                    if source_index == target_index:
                        continue
                    moved = list(order)
                    component = moved.pop(source_index)
                    moved.insert(target_index, component)
                    candidate = tuple(moved)
                    if candidate not in seen:
                        seen.add(candidate)
                        candidates.add(candidate)
        active = tuple(
            sorted(
                candidates,
                key=lambda order: (evaluate(order).shape_count, order),
            )[:beam_width]
        )
        round_best = active[0]
        if (evaluate(round_best).shape_count, round_best) < (
            evaluate(best_order).shape_count,
            best_order,
        ):
            best_order = round_best

    candidate_after = evaluate(best_order)
    if candidate_after.shape_count < before.shape_count:
        optimized_order = best_order
        after = candidate_after
    else:
        optimized_order = original_order
        after = before
    return MixedComponentOrderOptimization(
        original_component_order=original_order,
        optimized_component_order=optimized_order,
        before=before,
        after=after,
        search="experimental_component_move_beam(width={},rounds={})".format(
            beam_width, search_rounds
        ),
    )


def optimize_vehicle_block_order(
    input_path: Path,
    output_path: Path,
    catalog: DefinitionCatalog,
    beam_width: int = 128,
    max_blocks_per_body: int = 128,
    force: bool = False,
) -> VehicleOptimization:
    source = Path(input_path)
    destination = Path(output_path)
    if source.resolve() == destination.resolve():
        raise ValueError("input and output paths must be different")
    if destination.exists() and not force:
        raise FileExistsError("output already exists; pass --force to replace it: {}".format(destination))
    if max_blocks_per_body < 1:
        raise ValueError("max blocks per body must be at least 1")

    vehicle = load_vehicle(source)
    tree = _parse_with_comments(source)
    body_elements = tree.getroot().findall("./bodies/body")
    if len(body_elements) != len(vehicle.bodies):
        raise ValueError("parsed body count mismatch")

    body_results: List[BodyOptimization] = []
    for body_index, (body, body_element) in enumerate(zip(vehicle.bodies, body_elements)):
        components_element = body_element.find("components")
        component_elements = [] if components_element is None else list(components_element)
        if any(element.tag != "c" for element in component_elements):
            raise UnsupportedVehicleError(
                "body {} contains non-component children inside <components>".format(body_index)
            )
        if len(component_elements) != len(body.components):
            raise ValueError("body {} component count mismatch".format(body_index))
        if len(body.components) > max_blocks_per_body:
            raise UnsupportedVehicleError(
                "body {} has {} blocks; V1 safety limit is {}".format(
                    body_index, len(body.components), max_blocks_per_body
                )
            )
        voxels = vehicle.physics_voxels(catalog, body_index)
        component_points = validate_cube_component_groups(
            body_index, body.components, voxels, require_single_voxel=True
        )
        result = optimize_component_cube_order(component_points, beam_width=beam_width)
        body_results.append(
            BodyOptimization(
                body_index=body_index,
                body_id=body.body_id,
                component_count=len(body.components),
                result=result,
            )
        )

    package_plan = plan_component_package(catalog, destination, force)
    destination.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=destination.name + ".", suffix=".tmp", dir=str(destination.parent)
    )
    os.close(file_descriptor)
    try:
        write_vehicle_component_order_preserving_source(
            source,
            Path(temporary_name),
            tuple(
                body_result.result.optimized_component_order
                for body_result in body_results
            ),
        )
        install_component_package(package_plan)
        os.replace(temporary_name, destination)
    except Exception:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)
        raise

    return VehicleOptimization(
        input_path=source,
        output_path=destination,
        bodies=tuple(body_results),
        component_bin_count=package_plan.component_bin_count,
        component_package_path=(
            package_plan.output_root if package_plan.component_bin_count else None
        ),
    )


def optimize_vehicle_mixed_order_experimental(
    input_path: Path,
    output_path: Path,
    catalog: DefinitionCatalog,
    beam_width: int = 12,
    search_rounds: int = 3,
    max_components_per_body: int = 32,
    force: bool = False,
) -> MixedVehicleOptimization:
    """Write a reordered copy using the unverified mixed-shape grouping model."""

    source = Path(input_path)
    destination = Path(output_path)
    if source.resolve() == destination.resolve():
        raise ValueError("input and output paths must be different")
    if destination.exists() and not force:
        raise FileExistsError(
            "output already exists; pass --force to replace it: {}".format(destination)
        )
    if max_components_per_body < 1:
        raise ValueError("max components per body must be at least 1")

    vehicle = load_vehicle(source)
    tree = _parse_with_comments(source)
    body_elements = tree.getroot().findall("./bodies/body")
    if len(body_elements) != len(vehicle.bodies):
        raise ValueError("parsed body count mismatch")

    body_results: List[MixedBodyOptimization] = []
    for body_index, (body, body_element) in enumerate(zip(vehicle.bodies, body_elements)):
        components_element = body_element.find("components")
        component_elements = [] if components_element is None else list(components_element)
        if any(element.tag != "c" for element in component_elements):
            raise UnsupportedVehicleError(
                "body {} contains non-component children inside <components>".format(body_index)
            )
        if len(component_elements) != len(body.components):
            raise ValueError("body {} component count mismatch".format(body_index))
        if len(body.components) > max_components_per_body:
            raise UnsupportedVehicleError(
                "body {} has {} components; experimental mixed-shape limit is {}".format(
                    body_index, len(body.components), max_components_per_body
                )
            )
        voxels = vehicle.physics_voxels(catalog, body_index)
        component_voxels = validate_mixed_component_groups(
            body_index, body.components, voxels, require_single_voxel=True
        )
        # This byte belongs to the runtime component instance, not the root
        # `flags` attribute in the definition XML. Default, unmirrored vehicle
        # components observed in Stage 4 use zero.
        flags = {voxel.component_definition: 0 for voxel in voxels}
        result = optimize_mixed_component_order(
            component_voxels,
            flags,
            beam_width=beam_width,
            search_rounds=search_rounds,
        )
        body_results.append(
            MixedBodyOptimization(
                body_index=body_index,
                body_id=body.body_id,
                component_count=len(body.components),
                result=result,
            )
        )

    package_plan = plan_component_package(catalog, destination, force)
    destination.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=destination.name + ".", suffix=".tmp", dir=str(destination.parent)
    )
    os.close(file_descriptor)
    try:
        write_vehicle_component_order_preserving_source(
            source,
            Path(temporary_name),
            tuple(
                body_result.result.optimized_component_order
                for body_result in body_results
            ),
        )
        install_component_package(package_plan)
        os.replace(temporary_name, destination)
    except Exception:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)
        raise

    return MixedVehicleOptimization(
        input_path=source,
        output_path=destination,
        bodies=tuple(body_results),
        component_bin_count=package_plan.component_bin_count,
        component_package_path=(
            package_plan.output_root if package_plan.component_bin_count else None
        ),
    )
