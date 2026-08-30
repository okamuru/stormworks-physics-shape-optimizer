"""Whole-component optimization using binary or portable exact grouping."""

from __future__ import annotations

from array import array
from bisect import bisect_right
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
from itertools import permutations, product
import multiprocessing
import os
from pathlib import Path
import tempfile
from typing import Any, Callable, Dict, List, Mapping, Optional, Protocol, Sequence, Tuple

from .component_package import install_component_package, plan_component_package
from .definitions import DefinitionCatalog
from .model import WorldVoxel
from .native_merge import native_backend_available
from .optimizer import UnsupportedVehicleError, _parse_with_comments, validate_mixed_component_groups
from .portable_merge import PortableMergeOracle, PreparedPortableMergeEvaluator
from .source_preserving_xml import write_vehicle_component_order_preserving_source
from .surface_flood_fill import model_surface_physics_flood_fill
from .vehicle import load_vehicle


class PartitionOracle(Protocol):
    def partition(
        self,
        voxels: Sequence[WorldVoxel],
        component_runtime_flags: Optional[Mapping[int, int]] = None,
    ) -> Any: ...


@dataclass(frozen=True)
class ExactComponentOrderOptimization:
    original_component_order: Tuple[int, ...]
    optimized_component_order: Tuple[int, ...]
    before: Any
    after: Any
    evaluated_order_count: int
    search: str
    completed_stage_count: int = 1
    worker_count: int = 1

    @property
    def changed(self) -> bool:
        return self.original_component_order != self.optimized_component_order


@dataclass(frozen=True)
class ExactBodyOptimization:
    body_index: int
    body_id: str
    component_count: int
    physics_voxel_count: int
    physics_component_count: int
    non_physics_component_count: int
    multi_voxel_component_count: int
    extra_box_count: int
    physics_flooder_component_count: int
    generated_fill_voxel_count: int
    partial_volume_excluded_count: int
    result: ExactComponentOrderOptimization
    xml_edited_component_count: int = 0
    xml_edited_physics_voxel_count: int = 0
    protected_body: bool = False

    @property
    def before_shape_count(self) -> int:
        return self.result.before.shape_count + self.extra_box_count

    @property
    def after_shape_count(self) -> int:
        return self.result.after.shape_count + self.extra_box_count


@dataclass(frozen=True)
class ExactVehicleOptimization:
    input_path: Path
    output_path: Path
    backend: str
    binary_path: Optional[Path]
    binary_sha256: str
    output_reload_verified: bool
    bodies: Tuple[ExactBodyOptimization, ...]
    component_bin_count: int = 0
    component_package_path: Optional[Path] = None

    @property
    def before_shape_count(self) -> int:
        return sum(body.before_shape_count for body in self.bodies)

    @property
    def after_shape_count(self) -> int:
        return sum(body.after_shape_count for body in self.bodies)


def ordered_component_voxels(
    groups: Sequence[Sequence[WorldVoxel]],
    component_order: Sequence[int],
    trailing_voxels: Sequence[WorldVoxel] = (),
) -> Tuple[WorldVoxel, ...]:
    flattened = tuple(
        voxel
        for component_index in component_order
        for voxel in groups[component_index]
    ) + tuple(trailing_voxels)
    return tuple(
        replace(voxel, insertion_index=insertion_index)
        for insertion_index, voxel in enumerate(flattened)
    )


def _occupancy_component_lower_bound(
    groups: Sequence[Sequence[WorldVoxel]],
    trailing_voxels: Sequence[WorldVoxel] = (),
) -> int:
    """Count face-connected occupied regions, a hard lower bound on boxes."""

    remaining = {
        voxel.position
        for group in groups
        for voxel in group
    }
    remaining.update(voxel.position for voxel in trailing_voxels)
    connected_components = 0
    while remaining:
        connected_components += 1
        stack = [remaining.pop()]
        while stack:
            x, y, z = stack.pop()
            for neighbor in (
                (x - 1, y, z),
                (x + 1, y, z),
                (x, y - 1, z),
                (x, y + 1, z),
                (x, y, z - 1),
                (x, y, z + 1),
            ):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
    return connected_components


_WORKER_GROUPS: Tuple[Tuple[WorldVoxel, ...], ...] = ()
_WORKER_TRAILING: Tuple[WorldVoxel, ...] = ()
_WORKER_RUNTIME_FLAGS: Optional[Dict[int, int]] = None
_WORKER_PREPARED: Optional[PreparedPortableMergeEvaluator] = None


def _initialize_portable_partition_worker(
    groups: Tuple[Tuple[WorldVoxel, ...], ...],
    trailing_voxels: Tuple[WorldVoxel, ...],
    component_runtime_flags: Optional[Dict[int, int]],
    allow_overlaps: bool,
) -> None:
    """Install immutable body data once in each spawned evaluation worker."""

    global _WORKER_GROUPS
    global _WORKER_TRAILING
    global _WORKER_RUNTIME_FLAGS
    global _WORKER_PREPARED
    _WORKER_GROUPS = groups
    _WORKER_TRAILING = trailing_voxels
    _WORKER_RUNTIME_FLAGS = component_runtime_flags
    _WORKER_PREPARED = PreparedPortableMergeEvaluator(
        groups,
        trailing_voxels,
        component_runtime_flags,
        allow_overlaps=allow_overlaps,
    )


def _partition_order_score_in_worker(order: Tuple[int, ...]) -> int:
    if _WORKER_PREPARED is None:
        raise RuntimeError("portable partition worker was not initialized")
    return _WORKER_PREPARED.shape_count_order(order)


def resolve_worker_count(
    requested: int,
    component_count: int,
    voxel_count: int,
) -> int:
    """Resolve 0=Auto while avoiding process overhead on tiny bodies."""

    if requested < 0:
        raise ValueError("worker count must be 0 (Auto) or greater")
    available = max(1, os.cpu_count() or 1)
    if requested:
        return max(1, min(requested, available))
    if component_count < 64 or voxel_count < 20_000:
        return 1
    automatic = min(4, available)
    if voxel_count >= 150_000:
        automatic = min(automatic, 2)
    return max(1, automatic)


def pinned_non_physics_component_order(
    component_count: int,
    physics_component_indices: Sequence[int],
    optimized_physics_order: Sequence[int],
) -> Tuple[int, ...]:
    """Map a physics-only order back into the original component slots."""

    physics_indices = tuple(physics_component_indices)
    if len(physics_indices) != len(optimized_physics_order):
        raise ValueError("physics component order length mismatch")
    if len(set(physics_indices)) != len(physics_indices):
        raise ValueError("physics component indices must be unique")
    if any(index < 0 or index >= component_count for index in physics_indices):
        raise ValueError("physics component index is out of range")
    if sorted(optimized_physics_order) != list(range(len(physics_indices))):
        raise ValueError("optimized physics order must be a local permutation")
    result = list(range(component_count))
    reordered = tuple(
        physics_indices[local_index] for local_index in optimized_physics_order
    )
    for slot, component_index in zip(physics_indices, reordered):
        result[slot] = component_index
    return tuple(result)


class _MovePairSequence:
    """Compact, deterministic view of the legacy component-move pair order."""

    __slots__ = ("length", "anchors", "cumulative_counts")

    _OFFSET_DISTANCES = frozenset((1, 2, 4, 8, 16))

    def __init__(self, length: int) -> None:
        self.length = length
        self.anchors = (
            (0, length // 4, length // 2, (3 * length) // 4, length - 1)
            if length > 48
            else ()
        )
        cumulative_counts = array("Q", (0,))
        if length > 48:
            total = 0
            for distance in range(1, length):
                if distance in self._OFFSET_DISTANCES:
                    # Every pair at an anchor with this distance is already in
                    # the complete +/- offset row and was de-duplicated by the
                    # former set implementation.
                    total += 2 * (length - distance)
                else:
                    total += sum(
                        int(target >= distance)
                        + int(target + distance < length)
                        for target in self.anchors
                    )
                cumulative_counts.append(total)
        self.cumulative_counts = cumulative_counts

    def __len__(self) -> int:
        if self.length <= 48:
            return self.length * max(0, self.length - 1)
        return self.cumulative_counts[-1]

    def _pair_at_distance(self, distance: int, local_index: int) -> Tuple[int, int]:
        if distance in self._OFFSET_DISTANCES:
            # For length > 48 all supported offsets are below half the body,
            # yielding one/two/one targets across the three source ranges.
            if local_index < distance:
                source = local_index
                return source, source + distance
            local_index -= distance
            middle_pair_count = 2 * (self.length - 2 * distance)
            if local_index < middle_pair_count:
                source = distance + local_index // 2
                if local_index % 2:
                    return source, source + distance
                return source, source - distance
            source = self.length - distance + local_index - middle_pair_count
            return source, source - distance

        pairs = []
        for target in self.anchors:
            source = target - distance
            if source >= 0:
                pairs.append((source, target))
            source = target + distance
            if source < self.length:
                pairs.append((source, target))
        pairs.sort()
        return pairs[local_index]

    def __getitem__(self, index):
        if isinstance(index, slice):
            return tuple(self[item] for item in range(*index.indices(len(self))))
        if index < 0:
            index += len(self)
        if index < 0 or index >= len(self):
            raise IndexError("move pair index out of range")
        if self.length <= 48:
            row_length = self.length - 1
            source, target = divmod(index, row_length)
            if target >= source:
                target += 1
            return source, target
        distance = bisect_right(self.cumulative_counts, index)
        local_index = index - self.cumulative_counts[distance - 1]
        return self._pair_at_distance(distance, local_index)

    def __iter__(self):
        if self.length <= 48:
            for source in range(self.length):
                for target in range(self.length):
                    if source != target:
                        yield source, target
            return
        for distance in range(1, self.length):
            if distance in self._OFFSET_DISTANCES:
                for source in range(self.length):
                    if source >= distance:
                        yield source, source - distance
                    if source + distance < self.length:
                        yield source, source + distance
                continue
            pairs = []
            for target in self.anchors:
                source = target - distance
                if source >= 0:
                    pairs.append((source, target))
                source = target + distance
                if source < self.length:
                    pairs.append((source, target))
            yield from sorted(pairs)


class _OffsetMovePairSequence:
    """Concatenate barrier-delimited move sequences without copying pairs."""

    __slots__ = ("segments", "cumulative_counts")

    def __init__(
        self,
        segments: Sequence[Tuple[int, _MovePairSequence]],
    ) -> None:
        self.segments = tuple(segments)
        cumulative_counts = [0]
        for _offset, pairs in self.segments:
            cumulative_counts.append(cumulative_counts[-1] + len(pairs))
        self.cumulative_counts = tuple(cumulative_counts)

    def __len__(self) -> int:
        return self.cumulative_counts[-1]

    def __getitem__(self, index):
        if isinstance(index, slice):
            return tuple(self[item] for item in range(*index.indices(len(self))))
        if index < 0:
            index += len(self)
        if index < 0 or index >= len(self):
            raise IndexError("move pair index out of range")
        segment_index = bisect_right(self.cumulative_counts, index) - 1
        offset, pairs = self.segments[segment_index]
        source, target = pairs[index - self.cumulative_counts[segment_index]]
        return source + offset, target + offset

    def __iter__(self):
        for offset, pairs in self.segments:
            for source, target in pairs:
                yield source + offset, target + offset


def _candidate_move_pairs(length: int) -> Sequence[Tuple[int, int]]:
    """Return the legacy move order as a compact indexable sequence."""

    return _MovePairSequence(length)


def overlapping_component_indices(
    component_voxels: Sequence[Sequence[WorldVoxel]],
) -> Tuple[int, ...]:
    """Return components that contribute to any repeated world position."""

    by_position: Dict[Tuple[int, int, int], List[int]] = {}
    for component_index, voxels in enumerate(component_voxels):
        for voxel in voxels:
            by_position.setdefault(voxel.position, []).append(component_index)
    return tuple(
        sorted(
            {
                component_index
                for component_indices in by_position.values()
                if len(component_indices) > 1
                for component_index in component_indices
            }
        )
    )


def _candidate_move_pairs_with_barriers(
    length: int, fixed_indices: Sequence[int]
) -> Sequence[Tuple[int, int]]:
    """Return moves that never cross a fixed overlap component."""

    fixed = tuple(sorted(set(fixed_indices)))
    if any(index < 0 or index >= length for index in fixed):
        raise ValueError("fixed component index is out of range")
    if not fixed:
        return _candidate_move_pairs(length)
    segments = []
    start = 0
    for stop in fixed + (length,):
        segment_pairs = _MovePairSequence(stop - start)
        if segment_pairs:
            segments.append((start, segment_pairs))
        start = stop + 1
    return _OffsetMovePairSequence(segments)


_BOUNDED_CANDIDATE_THRESHOLD = 256


def _effective_evaluation_budget(component_count: int, requested: int) -> int:
    """Retain the requested search depth for every body size."""

    return requested


def _evenly_spaced_indices(total: int, limit: int) -> Tuple[int, ...]:
    if total <= 0 or limit <= 0:
        return ()
    if limit >= total:
        return tuple(range(total))
    if limit == 1:
        return (0,)
    return tuple(
        sorted(
            {
                round(index * (total - 1) / (limit - 1))
                for index in range(limit)
            }
        )
    )


def _legacy_identity_candidates(
    original_order: Tuple[int, ...],
    move_pairs: Sequence[Tuple[int, int]],
    seen: set[Tuple[int, ...]],
    limit: int,
) -> Tuple[Tuple[int, ...], ...]:
    """Select the exact legacy first-round sample without full candidate keys.

    For an identity order, a moved permutation's lexicographic rank is fully
    determined by the first affected index and move direction.  The reverse
    form of an adjacent swap is the only duplicate and the legacy loop keeps
    the forward form first.
    """

    ordered_pairs = tuple(
        sorted(
            (
                (source, target)
                for source, target in move_pairs
                if source != target + 1
            ),
            key=lambda pair: (
                abs(pair[0] - pair[1]),
                -min(pair),
                0 if pair[0] < pair[1] else 1,
                pair,
            ),
        )
    )
    selected = []
    for pair_index in _evenly_spaced_indices(len(ordered_pairs), min(limit, len(ordered_pairs))):
        source_index, target_index = ordered_pairs[pair_index]
        moved = list(original_order)
        component = moved.pop(source_index)
        moved.insert(target_index, component)
        candidate = tuple(moved)
        if candidate in seen:
            continue
        seen.add(candidate)
        selected.append(candidate)
    return tuple(selected)


def _native_seed_sweep_candidates(
    component_voxels: Sequence[Sequence[WorldVoxel]],
    fixed_indices: Sequence[int],
    seen: set[Tuple[int, ...]],
    limit: int,
) -> Tuple[Tuple[int, ...], ...]:
    """Build spatial seed orders around Stormworks' six-direction merge walk.

    The native builder consumes the source-order vector only to select each
    next unprocessed seed.  Once seeded, it expands in ``+x,+y,+z,-x,-y,-z``.
    For large bodies, moving one arbitrary component rarely changes a useful
    seed.  These 48 deterministic XYZ axis/sign sweeps instead expose every
    bounding-box corner and axis priority while preserving fixed overlap
    barriers and each component's internal voxel order.
    """

    if limit <= 0:
        return ()
    groups = tuple(tuple(group) for group in component_voxels)
    length = len(groups)
    fixed = tuple(sorted(set(fixed_indices)))
    segments = []
    start = 0
    for stop in fixed + (length,):
        if start < stop:
            segments.append(tuple(range(start, stop)))
        start = stop + 1

    # A sweep key only depends on the component's bounding extrema.  The old
    # implementation recomputed three ``min(...)`` expressions over every
    # voxel for every one of the 48 axis/sign sweeps.  Cache the six possible
    # directional coordinates once per component instead:
    #
    #   min(+position) == minimum
    #   min(-position) == -maximum
    #
    # Fixed components are deliberately excluded.  They are barriers rather
    # than sort members, and the previous implementation never inspected their
    # voxel groups while constructing these candidates either.
    directional_coordinates = [[0] * length for _ in range(6)]
    for segment in segments:
        for component_index in segment:
            group = groups[component_index]
            iterator = iter(group)
            try:
                first = next(iterator)
            except StopIteration:
                # Match the failure produced by the former min(generator)
                # implementation for a sortable empty component group.
                raise ValueError("min() arg is an empty sequence") from None
            min_x, min_y, min_z = first.position
            max_x, max_y, max_z = first.position
            for item in iterator:
                x, y, z = item.position
                if x < min_x:
                    min_x = x
                elif x > max_x:
                    max_x = x
                if y < min_y:
                    min_y = y
                elif y > max_y:
                    max_y = y
                if z < min_z:
                    min_z = z
                elif z > max_z:
                    max_z = z
            directional_coordinates[0][component_index] = min_x
            directional_coordinates[1][component_index] = -max_x
            directional_coordinates[2][component_index] = min_y
            directional_coordinates[3][component_index] = -max_y
            directional_coordinates[4][component_index] = min_z
            directional_coordinates[5][component_index] = -max_z

    component_indices = range(length)

    candidates = []
    candidate_seen = set()
    for axes in permutations(range(3)):
        for signs in product((-1, 1), repeat=3):
            order = list(range(length))
            direction_indices = tuple(
                axis * 2 + (0 if sign == 1 else 1)
                for axis, sign in zip(axes, signs)
            )
            spatial_keys = tuple(
                zip(
                    directional_coordinates[direction_indices[0]],
                    directional_coordinates[direction_indices[1]],
                    directional_coordinates[direction_indices[2]],
                    component_indices,
                )
            )
            for segment in segments:
                reordered = sorted(segment, key=spatial_keys.__getitem__)
                for slot, component_index in zip(segment, reordered):
                    order[slot] = component_index
            candidate = tuple(order)
            if candidate in seen or candidate in candidate_seen:
                continue
            candidate_seen.add(candidate)
            candidates.append(candidate)

    if len(candidates) > limit:
        candidates = [
            candidates[index]
            for index in _evenly_spaced_indices(len(candidates), limit)
        ]
    seen.update(candidates)
    return tuple(candidates)


def _bounded_active_move_candidates(
    active: Sequence[Tuple[int, ...]],
    move_pairs: Sequence[Tuple[int, int]],
    seen: set[Tuple[int, ...]],
    limit: int,
) -> Tuple[Tuple[int, ...], ...]:
    """Materialize only the local candidates that can actually be evaluated."""

    if limit <= 0 or not active or not move_pairs:
        return ()
    selected = []
    for active_index, order in enumerate(active):
        remaining_active = len(active) - active_index
        quota = max(1, (limit - len(selected) + remaining_active - 1) // remaining_active)
        selected_before_order = len(selected)
        probe_count = min(len(move_pairs), max(quota, quota * 3))
        for pair_index in _evenly_spaced_indices(len(move_pairs), probe_count):
            source_index, target_index = move_pairs[pair_index]
            moved = list(order)
            component = moved.pop(source_index)
            moved.insert(target_index, component)
            candidate = tuple(moved)
            if candidate in seen:
                continue
            seen.add(candidate)
            selected.append(candidate)
            if (
                len(selected) >= limit
                or len(selected) - selected_before_order >= quota
            ):
                break
        if len(selected) >= limit:
            break
    if len(selected) < limit:
        for order in active:
            for source_index, target_index in move_pairs:
                moved = list(order)
                component = moved.pop(source_index)
                moved.insert(target_index, component)
                candidate = tuple(moved)
                if candidate in seen:
                    continue
                seen.add(candidate)
                selected.append(candidate)
                if len(selected) >= limit:
                    return tuple(selected)
    return tuple(selected)


def optimize_exact_component_order(
    component_voxels: Sequence[Sequence[WorldVoxel]],
    oracle: PartitionOracle,
    component_runtime_flags: Optional[Mapping[int, int]] = None,
    beam_width: int = 8,
    search_rounds: int = 2,
    max_evaluations: int = 64,
    trailing_voxels: Sequence[WorldVoxel] = (),
    search_backend: str = "binary_exact",
    fixed_component_indices: Sequence[int] = (),
    progress_callback: Optional[Callable[[int, int], None]] = None,
    detailed_progress_callback: Optional[Callable[[int, int, int], None]] = None,
    worker_count: int = 1,
) -> ExactComponentOrderOptimization:
    """Search whole-component relocations using exact grouping counts."""

    if beam_width < 1:
        raise ValueError("beam width must be at least 1")
    if search_rounds < 1:
        raise ValueError("search rounds must be at least 1")
    if max_evaluations < 1:
        raise ValueError("max evaluations must be at least 1")
    groups = tuple(tuple(group) for group in component_voxels)
    trailing = tuple(trailing_voxels)
    if any(not group for group in groups):
        raise ValueError("every optimized component must contribute physics voxels")

    evaluation_budget = _effective_evaluation_budget(len(groups), max_evaluations)
    resolved_worker_count = (
        resolve_worker_count(
            worker_count,
            len(groups),
            sum(len(group) for group in groups) + len(trailing),
        )
        if isinstance(oracle, PortableMergeOracle)
        else 1
    )
    prepared_oracle = (
        PreparedPortableMergeEvaluator(
            groups,
            trailing,
            component_runtime_flags,
            allow_overlaps=oracle.allow_overlaps,
        )
        if isinstance(oracle, PortableMergeOracle)
        else None
    )
    if (
        worker_count == 0
        and prepared_oracle is not None
        and prepared_oracle.native_backend == "rust_cdylib"
    ):
        # One native evaluation already runs outside the GIL and is far
        # shorter than the spawn/pickle cost for a vehicle-sized process pool.
        # Keep explicit 2/4/8 worker requests available for unusually expensive
        # candidates, but make Auto choose the measured low-latency path.
        resolved_worker_count = 1
    original_order = tuple(range(len(groups)))
    score_cache: Dict[Tuple[int, ...], int] = {}
    evaluated_count = 0
    before_result: Any = None
    best_evaluated_key: Optional[Tuple[int, Tuple[int, ...]]] = None
    best_evaluated_result: Any = None
    parallel_executor: Optional[ProcessPoolExecutor] = None
    permutation_cap = 1
    for factor in range(2, len(groups) + 1):
        permutation_cap *= factor
        if permutation_cap >= evaluation_budget:
            permutation_cap = evaluation_budget
            break
    expected_evaluations = max(1, min(evaluation_budget, permutation_cap))

    def record_score(
        order: Tuple[int, ...],
        shape_count: int,
        result: Any = None,
    ) -> int:
        nonlocal evaluated_count
        nonlocal before_result
        nonlocal best_evaluated_key
        nonlocal best_evaluated_result
        score_cache[order] = shape_count
        evaluated_count += 1
        if order == original_order and result is not None:
            before_result = result
        result_key = (shape_count, order)
        if best_evaluated_key is None or result_key < best_evaluated_key:
            best_evaluated_key = result_key
            best_evaluated_result = result
        if progress_callback is not None:
            progress_callback(evaluated_count, expected_evaluations)
        if detailed_progress_callback is not None:
            detailed_progress_callback(
                evaluated_count,
                expected_evaluations,
                best_evaluated_key[0],
            )
        return score_cache[order]

    def evaluate(order: Tuple[int, ...]) -> int:
        if order not in score_cache:
            if prepared_oracle is not None:
                if order == original_order:
                    result = prepared_oracle.partition_order(order)
                    return record_score(order, result.shape_count, result)
                return record_score(
                    order,
                    prepared_oracle.shape_count_order(order),
                )
            result = oracle.partition(
                    ordered_component_voxels(groups, order, trailing),
                    component_runtime_flags,
                )
            return record_score(order, result.shape_count, result)
        return score_cache[order]

    def evaluate_many(orders: Sequence[Tuple[int, ...]]) -> None:
        nonlocal parallel_executor
        missing = tuple(order for order in orders if order not in score_cache)
        if not missing:
            return
        if (
            resolved_worker_count <= 1
            or len(missing) <= 1
            or not isinstance(oracle, PortableMergeOracle)
        ):
            for order in missing:
                evaluate(order)
            return
        runtime_flags = (
            dict(component_runtime_flags)
            if component_runtime_flags is not None
            else None
        )
        if parallel_executor is None:
            context = multiprocessing.get_context("spawn")
            parallel_executor = ProcessPoolExecutor(
                max_workers=resolved_worker_count,
                mp_context=context,
                initializer=_initialize_portable_partition_worker,
                initargs=(groups, trailing, runtime_flags, oracle.allow_overlaps),
            )
        chunksize = max(
            1,
            len(missing) // max(1, resolved_worker_count * 4),
        )
        try:
            for order, shape_count in zip(
                missing,
                parallel_executor.map(
                    _partition_order_score_in_worker,
                    missing,
                    chunksize=chunksize,
                ),
            ):
                record_score(order, shape_count)
        except BaseException:
            parallel_executor.shutdown(wait=True, cancel_futures=True)
            parallel_executor = None
            raise

    evaluate(original_order)
    before = before_result
    reached_lower_bound = before.shape_count <= 1
    if not reached_lower_bound and before.shape_count <= 128:
        reached_lower_bound = before.shape_count == _occupancy_component_lower_bound(
            groups,
            trailing,
        )
    if reached_lower_bound:
        return ExactComponentOrderOptimization(
            original_component_order=original_order,
            optimized_component_order=original_order,
            before=before,
            after=before,
            evaluated_order_count=evaluated_count,
            search="{}_component_move_beam(width={},rounds={},max_evaluations={},requested={})".format(
                search_backend,
                beam_width,
                search_rounds,
                evaluation_budget,
                max_evaluations,
            ),
            completed_stage_count=1,
            worker_count=resolved_worker_count,
        )
    best_order = original_order
    active = (original_order,)
    seen = {original_order}
    fixed = tuple(sorted(set(fixed_component_indices)))
    bounded_candidates = len(original_order) > _BOUNDED_CANDIDATE_THRESHOLD
    move_pairs = _candidate_move_pairs_with_barriers(len(original_order), fixed)
    for _round in range(search_rounds):
        candidates = set(active)
        remaining_budget = evaluation_budget - evaluated_count
        if remaining_budget <= 0:
            break
        if bounded_candidates and active == (original_order,):
            ordered_new = list(
                _native_seed_sweep_candidates(
                    groups,
                    fixed,
                    seen,
                    remaining_budget,
                )
            )
        elif bounded_candidates:
            ordered_new = list(
                _bounded_active_move_candidates(
                    active,
                    move_pairs,
                    seen,
                    remaining_budget,
                )
            )
        else:
            new_candidates = []
            for order in active:
                for source_index, target_index in move_pairs:
                    moved = list(order)
                    component = moved.pop(source_index)
                    moved.insert(target_index, component)
                    candidate = tuple(moved)
                    if candidate not in seen:
                        seen.add(candidate)
                        new_candidates.append(
                            (abs(source_index - target_index), candidate)
                        )
            ordered_new = [
                candidate
                for _distance, candidate in sorted(
                    new_candidates, key=lambda item: (item[0], item[1])
                )
            ]
            if len(ordered_new) > remaining_budget:
                if remaining_budget == 1:
                    ordered_new = [ordered_new[0]]
                else:
                    selected_indices = _evenly_spaced_indices(
                        len(ordered_new), remaining_budget
                    )
                    ordered_new = [ordered_new[index] for index in selected_indices]
        candidates.update(ordered_new)
        if not candidates:
            break
        evaluate_many(tuple(candidates))
        active = tuple(
            sorted(
                candidates,
                key=lambda order: (evaluate(order), order),
            )[:beam_width]
        )
        round_best = active[0]
        if (evaluate(round_best), round_best) < (
            evaluate(best_order),
            best_order,
        ):
            best_order = round_best

    if parallel_executor is not None:
        parallel_executor.shutdown(wait=True)
        parallel_executor = None

    candidate_shape_count = evaluate(best_order)
    if candidate_shape_count < before.shape_count:
        optimized_order = best_order
        if best_evaluated_key != (candidate_shape_count, best_order):
            raise RuntimeError("best evaluated result was not retained")
        if best_evaluated_result is None:
            best_evaluated_result = (
                prepared_oracle.partition_order(best_order)
                if prepared_oracle is not None
                else oracle.partition(
                    ordered_component_voxels(groups, best_order, trailing),
                    component_runtime_flags,
                )
            )
            if best_evaluated_result.shape_count != candidate_shape_count:
                raise RuntimeError("parallel score changed during final evaluation")
        after = best_evaluated_result
    else:
        optimized_order = original_order
        after = before
    return ExactComponentOrderOptimization(
        original_component_order=original_order,
        optimized_component_order=optimized_order,
        before=before,
        after=after,
        evaluated_order_count=evaluated_count,
        search="{}_component_move_beam(width={},rounds={},max_evaluations={},requested={})".format(
            search_backend,
            beam_width,
            search_rounds,
            evaluation_budget,
            max_evaluations,
        ),
        completed_stage_count=1,
        worker_count=resolved_worker_count,
    )


def optimize_staged_component_order(
    component_voxels: Sequence[Sequence[WorldVoxel]],
    oracle: PartitionOracle,
    stage_evaluations: Sequence[int],
    component_runtime_flags: Optional[Mapping[int, int]] = None,
    beam_width: int = 8,
    search_rounds: int = 2,
    trailing_voxels: Sequence[WorldVoxel] = (),
    search_backend: str = "binary_exact",
    fixed_component_indices: Sequence[int] = (),
    progress_callback: Optional[
        Callable[[int, int, int, int, int, int], None]
    ] = None,
    worker_count: int = 1,
) -> ExactComponentOrderOptimization:
    """Repeat the unchanged local search from each strictly better ordering."""

    budgets = tuple(int(value) for value in stage_evaluations)
    if not budgets or any(value < 1 for value in budgets):
        raise ValueError("stage evaluations must contain positive values")
    original_groups = tuple(tuple(group) for group in component_voxels)
    current_groups = original_groups
    composed_order = tuple(range(len(original_groups)))
    baseline_result: Any = None
    accepted_result: Any = None
    total_evaluated = 0
    completed_stages = 0
    resolved_workers = (
        resolve_worker_count(
            worker_count,
            len(original_groups),
            sum(len(group) for group in original_groups)
            + len(tuple(trailing_voxels)),
        )
        if isinstance(oracle, PortableMergeOracle)
        else 1
    )
    if (
        worker_count == 0
        and isinstance(oracle, PortableMergeOracle)
        and native_backend_available()
    ):
        resolved_workers = 1

    for stage_index, evaluation_budget in enumerate(budgets):
        def report_stage(
            current: int,
            expected: int,
            best_shape_count: int,
        ) -> None:
            if progress_callback is not None:
                progress_callback(
                    stage_index + 1,
                    len(budgets),
                    current,
                    expected,
                    best_shape_count,
                    resolved_workers,
                )

        stage_result = optimize_exact_component_order(
            current_groups,
            oracle,
            component_runtime_flags=component_runtime_flags,
            beam_width=beam_width,
            search_rounds=search_rounds,
            max_evaluations=evaluation_budget,
            trailing_voxels=trailing_voxels,
            search_backend=search_backend,
            fixed_component_indices=fixed_component_indices,
            detailed_progress_callback=report_stage,
            worker_count=resolved_workers,
        )
        completed_stages += 1
        total_evaluated += stage_result.evaluated_order_count
        if baseline_result is None:
            baseline_result = stage_result.before
            accepted_result = baseline_result
        if stage_result.after.shape_count >= stage_result.before.shape_count:
            break
        composed_order = tuple(
            composed_order[index]
            for index in stage_result.optimized_component_order
        )
        current_groups = tuple(
            current_groups[index]
            for index in stage_result.optimized_component_order
        )
        accepted_result = stage_result.after

    if baseline_result is None or accepted_result is None:
        raise RuntimeError("staged optimizer did not evaluate the original order")
    return ExactComponentOrderOptimization(
        original_component_order=tuple(range(len(original_groups))),
        optimized_component_order=composed_order,
        before=baseline_result,
        after=accepted_result,
        evaluated_order_count=total_evaluated,
        search="{}_staged_component_move(budgets={},rounds={})".format(
            search_backend,
            ",".join(str(value) for value in budgets),
            search_rounds,
        ),
        completed_stage_count=completed_stages,
        worker_count=resolved_workers,
    )


def _optimize_vehicle_exact(
    input_path: Path,
    output_path: Path,
    catalog: DefinitionCatalog,
    oracle: PartitionOracle,
    backend: str,
    beam_width: int = 8,
    search_rounds: int = 2,
    max_evaluations: int = 64,
    max_components_per_body: int = 0,
    force: bool = False,
    stage_evaluations: Optional[Sequence[int]] = None,
    worker_count: int = 1,
    progress_callback: Optional[Callable[[float, str], None]] = None,
) -> ExactVehicleOptimization:
    """Write a component-reordered copy using the supplied exact backend."""

    def report(percent: float, message: str) -> None:
        if progress_callback is not None:
            progress_callback(max(0.0, min(100.0, percent)), message)

    source = Path(input_path)
    destination = Path(output_path)
    if source.resolve() == destination.resolve():
        raise ValueError("input and output paths must be different")
    if destination.exists() and not force:
        raise FileExistsError(
            "output already exists; pass --force to replace it: {}".format(destination)
        )
    if max_components_per_body < 0:
        raise ValueError("max components per body cannot be negative")

    report(0, "出力用の車両XMLを読み込み中…")
    vehicle = load_vehicle(source)
    tree = _parse_with_comments(source)
    body_elements = tree.getroot().findall("./bodies/body")
    if len(body_elements) != len(vehicle.bodies):
        raise ValueError("parsed body count mismatch")
    body_results: List[ExactBodyOptimization] = []
    body_weights = tuple(max(1, len(body.components)) for body in vehicle.bodies)
    total_body_weight = max(1, sum(body_weights))
    completed_body_weight = 0
    for body_index, (body, body_element) in enumerate(zip(vehicle.bodies, body_elements)):
        body_start = 4.0 + 78.0 * completed_body_weight / total_body_weight
        body_end = 4.0 + 78.0 * (
            completed_body_weight + body_weights[body_index]
        ) / total_body_weight

        def report_body(fraction: float, message: str) -> None:
            report(
                body_start + (body_end - body_start) * fraction,
                "Body {}/{}: {}".format(
                    body_index + 1,
                    len(vehicle.bodies),
                    message,
                ),
            )

        report_body(0.0, "最適化入力を準備中…")
        components_element = body_element.find("components")
        component_elements = [] if components_element is None else list(components_element)
        if any(element.tag != "c" for element in component_elements):
            raise UnsupportedVehicleError(
                "body {} contains non-component children inside <components>".format(
                    body_index
                )
            )
        if len(component_elements) != len(body.components):
            raise ValueError("body {} component count mismatch".format(body_index))
        if max_components_per_body and len(body.components) > max_components_per_body:
            raise UnsupportedVehicleError(
                "body {} has {} components; binary-exact limit is {}".format(
                    body_index, len(body.components), max_components_per_body
                )
            )

        body_definitions = tuple(
            catalog.load(component.definition_id) for component in body.components
        )
        flooder_count = sum(
            definition.water_component_type == 19
            for definition in body_definitions
        )
        flood_fill = model_surface_physics_flood_fill(
            vehicle,
            catalog,
            body_index,
            progress_callback=lambda fraction, message: report_body(
                0.02 + 0.16 * fraction, message
            ),
        )
        if flooder_count and not flood_fill.supported:
            raise UnsupportedVehicleError(
                "body {} contains {} Physics Flooder component(s), but its "
                "Definition buoyancy-surface fill model is unsupported: {}".format(
                    body_index, flooder_count, flood_fill.status
                )
            )
        extra_box_count = sum(
            definition.contributes_physics_extra_box
            for definition in body_definitions
        )

        voxels = (
            flood_fill.static_voxels_after_fill
            if flooder_count
            else vehicle.physics_voxels(catalog, body_index)
        )
        component_voxels = validate_mixed_component_groups(
            body_index,
            body.components,
            voxels,
            require_single_voxel=False,
            allow_empty=True,
            allow_non_unit_transform=True,
            allow_overlaps=True,
        )
        physics_component_indices = tuple(
            component_index
            for component_index, group in enumerate(component_voxels)
            if group
        )
        physics_component_voxels = tuple(
            component_voxels[component_index]
            for component_index in physics_component_indices
        )
        overlapping_components = set(
            overlapping_component_indices(component_voxels)
        )
        fixed_physics_indices = tuple(
            local_index
            for local_index, component_index in enumerate(
                physics_component_indices
            )
            if component_index in overlapping_components
        )
        search_arguments = dict(
            component_voxels=physics_component_voxels,
            oracle=oracle,
            beam_width=beam_width,
            search_rounds=search_rounds,
            trailing_voxels=(
                flood_fill.new_fill_voxels if flooder_count else ()
            ),
            search_backend=backend,
            fixed_component_indices=fixed_physics_indices,
            worker_count=worker_count,
        )
        if stage_evaluations is None:
            def report_single(
                current: int,
                expected: int,
                best_shape_count: int,
            ) -> None:
                report_body(
                    0.2 + 0.72 * current / max(1, expected),
                    "探索 1/1：配置候補 {}/{}・現在の最良 {} Shapes".format(
                        current,
                        expected,
                        best_shape_count,
                    ),
                )

            local_result = optimize_exact_component_order(
                max_evaluations=max_evaluations,
                detailed_progress_callback=report_single,
                **search_arguments,
            )
        else:
            def report_staged(
                stage_index: int,
                stage_count: int,
                current: int,
                expected: int,
                best_shape_count: int,
                resolved_workers: int,
            ) -> None:
                completed_budget = sum(stage_evaluations[: stage_index - 1])
                total_budget = max(1, sum(stage_evaluations))
                fraction = (completed_budget + current) / total_budget
                report_body(
                    0.2 + 0.72 * min(1.0, fraction),
                    "探索 {}/{}：配置候補 {}/{}・CPU {}・現在の最良 {} Shapes".format(
                        stage_index,
                        stage_count,
                        current,
                        expected,
                        resolved_workers,
                        best_shape_count,
                    ),
                )

            local_result = optimize_staged_component_order(
                stage_evaluations=stage_evaluations,
                progress_callback=report_staged,
                **search_arguments,
            )
        full_original_order = tuple(range(len(body.components)))
        full_optimized_order = pinned_non_physics_component_order(
            len(body.components),
            physics_component_indices,
            local_result.optimized_component_order,
        )
        result = replace(
            local_result,
            original_component_order=full_original_order,
            optimized_component_order=full_optimized_order,
        )
        body_results.append(
            ExactBodyOptimization(
                body_index=body_index,
                body_id=body.body_id,
                component_count=len(body.components),
                physics_voxel_count=len(voxels),
                physics_component_count=len(physics_component_indices),
                non_physics_component_count=(
                    len(body.components) - len(physics_component_indices)
                ),
                multi_voxel_component_count=sum(
                    len(group) > 1 for group in component_voxels
                ),
                extra_box_count=extra_box_count,
                physics_flooder_component_count=flooder_count,
                generated_fill_voxel_count=(
                    len(flood_fill.new_fill_voxels) if flooder_count else 0
                ),
                partial_volume_excluded_count=(
                    flood_fill.partial_volume_excluded_count
                    if flooder_count
                    else 0
                ),
                result=result,
            )
        )
        report_body(1.0, "最適化順序を確定しました")
        completed_body_weight += body_weights[body_index]

    report(84, "元XMLのコンパクトな書式を保って書き出し中…")
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

        # Reparse the exact bytes that will be installed and run the same
        # build-pinned oracle again.  This catches an XML reordering/write bug
        # before the temporary file replaces an existing destination.
        verified_vehicle = load_vehicle(Path(temporary_name))
        if len(verified_vehicle.bodies) != len(body_results):
            raise RuntimeError("exact output body count changed after reload")
        for verified_index, body_result in enumerate(body_results):
            report(
                88.0 + 10.0 * verified_index / max(1, len(body_results)),
                "書き出し後のBody {}/{}を再検証中…".format(
                    verified_index + 1,
                    len(body_results),
                ),
            )
            verified_flood_fill = model_surface_physics_flood_fill(
                verified_vehicle,
                catalog,
                body_result.body_index,
            )
            verified_voxels = (
                verified_flood_fill.all_voxels
                if body_result.physics_flooder_component_count
                else verified_vehicle.physics_voxels(
                    catalog, body_result.body_index
                )
            )
            verified_partition = oracle.partition(
                verified_voxels
            )
            if verified_partition.shape_count != body_result.result.after.shape_count:
                raise RuntimeError(
                    "body {} exact output changed from {} to {} shapes after reload".format(
                        body_result.body_index,
                        body_result.result.after.shape_count,
                        verified_partition.shape_count,
                    )
                )
        install_component_package(package_plan)
        os.replace(temporary_name, destination)
    except Exception:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)
        raise

    report(100, "最適化XMLの保存と再検証が完了しました")
    return ExactVehicleOptimization(
        input_path=source,
        output_path=destination,
        backend=backend,
        binary_path=getattr(oracle, "binary_path", None),
        binary_sha256=getattr(oracle, "binary_sha256", ""),
        output_reload_verified=True,
        bodies=tuple(body_results),
        component_bin_count=package_plan.component_bin_count,
        component_package_path=(
            package_plan.output_root if package_plan.component_bin_count else None
        ),
    )


def optimize_vehicle_portable_exact(
    input_path: Path,
    output_path: Path,
    catalog: DefinitionCatalog,
    beam_width: int = 8,
    search_rounds: int = 2,
    max_evaluations: int = 64,
    max_components_per_body: int = 0,
    force: bool = False,
    stage_evaluations: Optional[Sequence[int]] = None,
    worker_count: int = 1,
    progress_callback: Optional[Callable[[float, str], None]] = None,
) -> ExactVehicleOptimization:
    """Write a cross-platform copy using the build-24749959 Python model."""

    return _optimize_vehicle_exact(
        input_path,
        output_path,
        catalog,
        PortableMergeOracle(allow_overlaps=True),
        "portable_exact",
        beam_width,
        search_rounds,
        max_evaluations,
        max_components_per_body,
        force,
        stage_evaluations,
        worker_count,
        progress_callback,
    )
