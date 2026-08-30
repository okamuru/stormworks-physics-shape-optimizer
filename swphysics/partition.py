from dataclasses import dataclass
from itertools import product
from typing import Collection, Dict, Iterable, List, Sequence, Set, Tuple

from .model import GridPoint, unique_points


AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


@dataclass(frozen=True)
class Box:
    minimum: GridPoint
    maximum: GridPoint

    @property
    def size_voxels(self) -> GridPoint:
        return tuple(self.maximum[index] - self.minimum[index] + 1 for index in range(3))  # type: ignore[return-value]

    @property
    def volume_voxels(self) -> int:
        size = self.size_voxels
        return size[0] * size[1] * size[2]

    @property
    def center_meters(self) -> Tuple[float, float, float]:
        return tuple((self.minimum[index] + self.maximum[index]) * 0.125 for index in range(3))  # type: ignore[return-value]

    @property
    def size_meters(self) -> Tuple[float, float, float]:
        return tuple(value * 0.25 for value in self.size_voxels)  # type: ignore[return-value]

    def points(self) -> Tuple[GridPoint, ...]:
        return tuple(
            (x, y, z)
            for x in range(self.minimum[0], self.maximum[0] + 1)
            for y in range(self.minimum[1], self.maximum[1] + 1)
            for z in range(self.minimum[2], self.maximum[2] + 1)
        )


@dataclass(frozen=True)
class PartitionResult:
    algorithm: str
    boxes: Tuple[Box, ...]
    voxel_count: int
    status: str = "hypothesis_unverified_against_game"

    @property
    def shape_count(self) -> int:
        return len(self.boxes)


def _validate_axis_order(axis_order: str) -> Tuple[int, int, int]:
    if len(axis_order) != 3 or set(axis_order) != {"x", "y", "z"}:
        raise ValueError("axis order must be a permutation of xyz")
    return tuple(AXIS_INDEX[axis] for axis in axis_order)  # type: ignore[return-value]


def _ordered_points(points: Sequence[GridPoint], seed_order: str) -> Tuple[GridPoint, ...]:
    unique = unique_points(points)
    if len(unique) != len(points):
        raise ValueError("overlapping cube physics voxels are not supported by the prototype")
    if seed_order == "xml":
        return unique
    if seed_order == "reverse-xml":
        return tuple(reversed(unique))
    if seed_order == "xyz":
        return tuple(sorted(unique))
    if seed_order == "zyx":
        return tuple(sorted(unique, key=lambda point: (point[2], point[1], point[0])))
    raise ValueError("unknown seed order: {}".format(seed_order))


def _face_points(minimum: List[int], maximum: List[int], axis: int, coordinate: int) -> Set[GridPoint]:
    ranges = []
    for index in range(3):
        if index == axis:
            ranges.append((coordinate,))
        else:
            ranges.append(range(minimum[index], maximum[index] + 1))
    return set(product(*ranges))  # type: ignore[arg-type,return-value]


def grow_cube_box(
    remaining: Collection[GridPoint],
    seed: GridPoint,
    axis_order: str = "xyz",
    direction_order: str = "-+",
) -> Box:
    """Grow one box from a seed using the validated cube traversal model."""

    occupied = set(remaining)
    if seed not in occupied:
        raise ValueError("seed is not present in remaining cube points: {}".format(seed))
    axes = _validate_axis_order(axis_order)
    if direction_order not in ("-+", "+-"):
        raise ValueError("direction order must be -+ or +-")
    directions = (-1, 1) if direction_order == "-+" else (1, -1)
    minimum = list(seed)
    maximum = list(seed)
    for axis in axes:
        for direction in directions:
            while True:
                coordinate = minimum[axis] - 1 if direction < 0 else maximum[axis] + 1
                face = _face_points(minimum, maximum, axis, coordinate)
                if not face.issubset(occupied):
                    break
                if direction < 0:
                    minimum[axis] = coordinate
                else:
                    maximum[axis] = coordinate
    return Box(tuple(minimum), tuple(maximum))  # type: ignore[arg-type]


def partition_cubes_greedy(
    points: Sequence[GridPoint],
    axis_order: str = "xyz",
    direction_order: str = "-+",
    seed_order: str = "xml",
) -> PartitionResult:
    """Partition cubes into boxes using an order-sensitive greedy hypothesis.

    This deliberately exposes traversal choices. It is a working model for A/B
    experiments, not yet a claim that Stormworks uses these exact choices.
    """

    _validate_axis_order(axis_order)
    if direction_order not in ("-+", "+-"):
        raise ValueError("direction order must be -+ or +-")
    seeds = _ordered_points(points, seed_order)
    remaining = set(seeds)
    boxes: List[Box] = []
    for seed in seeds:
        if seed not in remaining:
            continue
        box = grow_cube_box(remaining, seed, axis_order, direction_order)
        remaining.difference_update(box.points())
        boxes.append(box)
    if remaining:
        raise AssertionError("greedy partition left voxels unassigned")
    return PartitionResult(
        algorithm="greedy(axis={},direction={},seed={})".format(axis_order, direction_order, seed_order),
        boxes=tuple(boxes),
        voxel_count=len(seeds),
    )


def _all_filled_boxes(points: Tuple[GridPoint, ...]) -> Tuple[Box, ...]:
    occupied = set(points)
    boxes = set()
    for minimum in occupied:
        for maximum in occupied:
            if any(minimum[index] > maximum[index] for index in range(3)):
                continue
            box = Box(minimum, maximum)
            if box.volume_voxels > len(points):
                continue
            if set(box.points()).issubset(occupied):
                boxes.add(box)
    return tuple(sorted(boxes, key=lambda box: (-box.volume_voxels, box.minimum, box.maximum)))


def partition_cubes_exact(points: Sequence[GridPoint], max_voxels: int = 28) -> PartitionResult:
    """Find a minimum axis-aligned box partition for a small cube-only body."""

    unique = unique_points(points)
    if len(unique) != len(points):
        raise ValueError("overlapping cube physics voxels are not supported by the prototype")
    if len(unique) > max_voxels:
        raise ValueError(
            "exact partition is limited to {} voxels; got {}".format(max_voxels, len(unique))
        )
    if not unique:
        return PartitionResult("exact-minimum-box-partition", (), 0)

    index: Dict[GridPoint, int] = {point: offset for offset, point in enumerate(unique)}
    boxes = _all_filled_boxes(unique)
    masks = []
    by_voxel: List[List[int]] = [[] for _ in unique]
    for box_index, box in enumerate(boxes):
        mask = 0
        for point in box.points():
            mask |= 1 << index[point]
        masks.append(mask)
        for voxel_index in range(len(unique)):
            if mask & (1 << voxel_index):
                by_voxel[voxel_index].append(box_index)

    full_mask = (1 << len(unique)) - 1
    best: List[int] = list(range(len(unique)))
    memo: Dict[int, int] = {}

    def search(remaining: int, chosen: List[int]) -> None:
        nonlocal best
        if not remaining:
            if len(chosen) < len(best):
                best = list(chosen)
            return
        if len(chosen) >= len(best):
            return
        previous_depth = memo.get(remaining)
        if previous_depth is not None and previous_depth <= len(chosen):
            return
        memo[remaining] = len(chosen)

        remaining_indices = [offset for offset in range(len(unique)) if remaining & (1 << offset)]
        pivot = min(
            remaining_indices,
            key=lambda voxel_index: sum(
                1 for box_index in by_voxel[voxel_index] if masks[box_index] & remaining == masks[box_index]
            ),
        )
        candidates = [
            box_index
            for box_index in by_voxel[pivot]
            if masks[box_index] & remaining == masks[box_index]
        ]
        candidates.sort(key=lambda box_index: boxes[box_index].volume_voxels, reverse=True)
        for box_index in candidates:
            search(remaining ^ masks[box_index], chosen + [box_index])

    search(full_mask, [])
    selected = tuple(boxes[box_index] for box_index in best)
    return PartitionResult(
        algorithm="exact-minimum-axis-aligned-box-partition",
        boxes=selected,
        voxel_count=len(unique),
        status="mathematically_exact_for_cube_box_model_not_game_validated",
    )


def validate_partition(points: Iterable[GridPoint], boxes: Iterable[Box]) -> None:
    expected = set(points)
    actual: Set[GridPoint] = set()
    total = 0
    for box in boxes:
        box_points = set(box.points())
        total += len(box_points)
        actual.update(box_points)
    if total != len(actual):
        raise ValueError("partition boxes overlap")
    if actual != expected:
        raise ValueError("partition coverage differs from input voxel set")
