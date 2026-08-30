"""Build-pinned mixed physics-shape geometry and greedy grouping model.

The non-cube point sets come directly from Stormworks build 24749959.  The
partition routine is a deliberately labelled portable approximation. Stage 4
showed that merge_shape evaluates the +x,+y,+z,-x,-y,-z switch cases rather
than using them as a fixed priority, and that exact plane collision differs
from the convex-volume acceptance rule used here.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from itertools import product
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .model import GridPoint, WorldVoxel, apply_matrix
from .non_cube_data import NON_CUBE_SAMPLE_POINTS_QUARTERS


ScaledPoint = Tuple[int, int, int]
Triangle = Tuple[ScaledPoint, ScaledPoint, ScaledPoint]
CUBE_POINTS_QUARTERS: Tuple[ScaledPoint, ...] = tuple(product((-2, 2), repeat=3))
DIRECTION_SEQUENCE = ((0, 1), (1, 1), (2, 1), (0, -1), (1, -1), (2, -1))


def _subtract(left: ScaledPoint, right: ScaledPoint) -> ScaledPoint:
    return tuple(left[index] - right[index] for index in range(3))  # type: ignore[return-value]


def _cross(left: ScaledPoint, right: ScaledPoint) -> ScaledPoint:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _dot(left: ScaledPoint, right: ScaledPoint) -> int:
    return sum(left[index] * right[index] for index in range(3))


def _orientation(
    first: ScaledPoint,
    second: ScaledPoint,
    third: ScaledPoint,
    point: ScaledPoint,
) -> int:
    return _dot(_cross(_subtract(second, first), _subtract(third, first)), _subtract(point, first))


def _unique_points(points: Iterable[ScaledPoint]) -> Tuple[ScaledPoint, ...]:
    return tuple(dict.fromkeys(points))


@dataclass(frozen=True)
class ConvexHull:
    points: Tuple[ScaledPoint, ...]
    triangles: Tuple[Triangle, ...]
    volume6_quarter_units: int


def convex_hull(points: Iterable[ScaledPoint]) -> ConvexHull:
    """Return an exact integer incremental 3D hull for quarter-voxel points."""

    unique = _unique_points(points)
    if len(unique) < 4:
        raise ValueError("a 3D convex hull requires at least four unique points")
    first = unique[0]
    second = next((point for point in unique[1:] if point != first), None)
    if second is None:
        raise ValueError("convex hull points are coincident")
    third = next(
        (
            point
            for point in unique
            if _cross(_subtract(second, first), _subtract(point, first)) != (0, 0, 0)
        ),
        None,
    )
    if third is None:
        raise ValueError("convex hull points are collinear")
    fourth = next(
        (point for point in unique if _orientation(first, second, third, point) != 0),
        None,
    )
    if fourth is None:
        raise ValueError("convex hull points are coplanar")

    initial = (first, second, third, fourth)
    # Four times a known interior point.  The initial tetrahedron centroid
    # remains inside every hull produced by adding more points.
    interior_x4 = tuple(sum(point[index] for point in initial) for index in range(3))

    def outward_face(a: ScaledPoint, b: ScaledPoint, c: ScaledPoint) -> Triangle:
        normal = _cross(_subtract(b, a), _subtract(c, a))
        interior_relative_x4 = tuple(interior_x4[index] - 4 * a[index] for index in range(3))
        if _dot(normal, interior_relative_x4) > 0:
            return (a, c, b)
        return (a, b, c)

    faces: List[Triangle] = [
        outward_face(first, second, third),
        outward_face(first, fourth, second),
        outward_face(first, third, fourth),
        outward_face(second, fourth, third),
    ]
    for point in unique:
        visible = [face for face in faces if _orientation(*face, point) > 0]
        if not visible:
            continue
        edge_counts: Dict[Tuple[ScaledPoint, ScaledPoint], int] = {}
        for face in visible:
            for edge in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
                key = tuple(sorted(edge))  # type: ignore[assignment]
                edge_counts[key] = edge_counts.get(key, 0) + 1
        faces = [face for face in faces if face not in visible]
        faces.extend(
            outward_face(edge[0], edge[1], point)
            for edge, count in edge_counts.items()
            if count == 1
        )

    signed_volume6 = sum(_dot(a, _cross(b, c)) for a, b, c in faces)
    return ConvexHull(unique, tuple(faces), abs(signed_volume6))


@dataclass(frozen=True)
class PhysicsVoxelGeometry:
    insertion_index: int
    component_index: int
    component_definition: str
    position: GridPoint
    physics_shape: int
    collision_samples_quarters: Tuple[ScaledPoint, ...]
    hull: ConvexHull


@lru_cache(maxsize=8192)
def _transformed_voxel_geometry(
    physics_shape: int,
    position: GridPoint,
    physics_rotation: Tuple[int, ...],
    definition_flags_low3: int,
) -> Tuple[Tuple[ScaledPoint, ...], ConvexHull]:
    if physics_shape == 0:
        local_points = CUBE_POINTS_QUARTERS
    else:
        try:
            local_points = NON_CUBE_SAMPLE_POINTS_QUARTERS[physics_shape]
        except KeyError as error:
            raise ValueError("unknown physics_shape {}".format(physics_shape)) from error
    signs = tuple(-1 if definition_flags_low3 & (1 << axis) else 1 for axis in range(3))
    offset = tuple(coordinate * 4 for coordinate in position)
    transformed: List[ScaledPoint] = []
    for point in local_points:
        mirrored = tuple(point[axis] * signs[axis] for axis in range(3))
        rotated = apply_matrix(physics_rotation, mirrored)  # type: ignore[arg-type]
        transformed.append(tuple(rotated[axis] + offset[axis] for axis in range(3)))
    samples = tuple(transformed)
    return samples, convex_hull(samples)


def physics_voxel_geometry(
    voxel: WorldVoxel,
    definition_flags: int,
) -> PhysicsVoxelGeometry:
    samples, hull = _transformed_voxel_geometry(
        voxel.physics_shape,
        voxel.position,
        voxel.physics_rotation,
        definition_flags & 7,
    )
    return PhysicsVoxelGeometry(
        insertion_index=voxel.insertion_index,
        component_index=voxel.component_index,
        component_definition=voxel.component_definition,
        position=voxel.position,
        physics_shape=voxel.physics_shape,
        collision_samples_quarters=samples,
        hull=hull,
    )


@dataclass(frozen=True)
class PhysicsShapeGroup:
    seed_insertion_index: int
    voxel_insertion_indices: Tuple[int, ...]
    minimum: GridPoint
    maximum: GridPoint
    hull_points_quarters: Tuple[ScaledPoint, ...]
    volume6_quarter_units: int


@dataclass(frozen=True)
class MixedPartitionResult:
    algorithm: str
    groups: Tuple[PhysicsShapeGroup, ...]
    voxel_count: int
    status: str = "portable_convex_volume_approximation_not_binary_exact"

    @property
    def shape_count(self) -> int:
        return len(self.groups)


def _bounds(geometries: Sequence[PhysicsVoxelGeometry]) -> Tuple[List[int], List[int]]:
    return (
        [min(geometry.position[axis] for geometry in geometries) for axis in range(3)],
        [max(geometry.position[axis] for geometry in geometries) for axis in range(3)],
    )


def _convex_union_hull(geometries: Sequence[PhysicsVoxelGeometry]) -> Optional[ConvexHull]:
    expected_volume = sum(geometry.hull.volume6_quarter_units for geometry in geometries)
    combined = convex_hull(
        point for geometry in geometries for point in geometry.hull.points
    )
    return combined if combined.volume6_quarter_units == expected_volume else None


def partition_mixed_shapes_greedy(
    voxels: Sequence[WorldVoxel],
    definition_flags: Mapping[str, int],
) -> MixedPartitionResult:
    """Partition cube and non-cube voxels with a portable convex approximation.

    The geometry and direction cases are binary-derived. The fixed direction
    priority and convex-volume acceptance rule are known not to reproduce the
    Stage 4 D/E multi-voxel fixtures; use the binary oracle for exact results.
    """

    ordered = tuple(
        physics_voxel_geometry(voxel, definition_flags.get(voxel.component_definition, 0))
        for voxel in voxels
    )
    positions = [geometry.position for geometry in ordered]
    if len(positions) != len(set(positions)):
        raise ValueError("overlapping physics voxel positions are not supported")
    by_position = {geometry.position: index for index, geometry in enumerate(ordered)}
    remaining = set(range(len(ordered)))
    groups: List[PhysicsShapeGroup] = []

    for seed_index, seed in enumerate(ordered):
        if seed_index not in remaining:
            continue
        group_indices = [seed_index]
        remaining.remove(seed_index)
        minimum, maximum = _bounds((seed,))
        group_hull = seed.hull

        for axis, direction in DIRECTION_SEQUENCE:
            while True:
                coordinate = minimum[axis] - 1 if direction < 0 else maximum[axis] + 1
                ranges = [
                    range(minimum[index], maximum[index] + 1) if index != axis else (coordinate,)
                    for index in range(3)
                ]
                candidate_indices = [
                    by_position[position]
                    for position in product(*ranges)
                    if position in by_position and by_position[position] in remaining
                ]
                if not candidate_indices:
                    break
                proposed_indices = group_indices + candidate_indices
                proposed = [ordered[index] for index in proposed_indices]
                proposed_hull = _convex_union_hull(proposed)
                if proposed_hull is None:
                    break
                group_indices = proposed_indices
                group_hull = proposed_hull
                remaining.difference_update(candidate_indices)
                if direction < 0:
                    minimum[axis] = coordinate
                else:
                    maximum[axis] = coordinate

        groups.append(
            PhysicsShapeGroup(
                seed_insertion_index=seed.insertion_index,
                voxel_insertion_indices=tuple(ordered[index].insertion_index for index in group_indices),
                minimum=tuple(minimum),  # type: ignore[arg-type]
                maximum=tuple(maximum),  # type: ignore[arg-type]
                hull_points_quarters=group_hull.points,
                volume6_quarter_units=group_hull.volume6_quarter_units,
            )
        )
    if remaining:
        raise AssertionError("mixed physics partition left voxels unassigned")
    return MixedPartitionResult(
        algorithm="portable-greedy-convex(+x,+y,+z,-x,-y,-z;quarter-integer-hulls)",
        groups=tuple(groups),
        voxel_count=len(ordered),
    )
