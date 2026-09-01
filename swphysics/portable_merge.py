"""Portable build-24749959 physics-voxel merge model.

This module mirrors the game's plane-oriented merge representation without
executing the game binary.  A shape is an axis-aligned voxel bound clipped by
zero or more exact integer planes.  ``merge_shape`` grows that representation
one layer at a time in ``+x,+y,+z,-x,-y,-z`` order.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from itertools import chain, combinations, product
import math
import os
import struct
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .model import GridPoint, WorldVoxel, apply_matrix
from .non_cube_data import (
    NON_CUBE_CLIP_PLANES,
    NON_CUBE_COLLISION_THRESHOLDS,
    NON_CUBE_SAMPLE_POINTS_QUARTERS,
)
from .rotations import GRID_TRANSFORMS


Plane = Tuple[GridPoint, GridPoint]
STORMWORKS_BUILD_ID = "24749959"
SOURCE_BINARY_SHA256 = (
    "15ec31184c9c56a53d154b75edb352da86487774f498de3a2e5145d2a7d5203b"
)
DIRECTIONS: Tuple[GridPoint, ...] = (
    (1, 0, 0),
    (0, 1, 0),
    (0, 0, 1),
    (-1, 0, 0),
    (0, -1, 0),
    (0, 0, -1),
)
# A clipped layer can contain no eligible cells yet become reachable after a
# perpendicular expansion widens the AABB.  Keep that outcome distinct from a
# hard failure so callers can retain the large-vehicle direction cache without
# permanently hiding valid wedge merges.
_RETRY_AFTER_PERPENDICULAR_EXPANSION = object()

# build() shrinks every generated convex hull by this Bullet collision margin.
# The value and the per-shape zero-margin table below are taken from build
# 24749959's c_vehicle_physics_builder::build.  They matter for XML-scaled
# wedges: merge_shape may produce a group whose shrunken hull has fewer than
# four vertices, in which case the game leaves the collision shape null and F2
# does not display/count it.
_FINALIZATION_MARGIN = 0.041999999433755875
_FINALIZATION_PARALLEL_LIMIT = 0.9900000095367432
_FINALIZATION_EPSILON = 9.999999747378752e-06
_ZERO_CUSTOM_PLANE_MARGIN_SHAPES = frozenset(
    (6, 7, 8, 9, 12, 13, 14, 15, 18, 19, 20, 21, 22, 23, 24, 25, 28, 29, 30, 31)
)
_GRID_TRANSFORM_SET = frozenset(GRID_TRANSFORMS)


class UnsupportedPhysicsShapeError(ValueError):
    """A Definition uses a physics_shape unknown to the pinned model."""


def _float32(value: float) -> float:
    """Round one intermediate like the native SSE single-precision path."""

    return struct.unpack("<f", struct.pack("<f", value))[0]


def _finalization_dot(
    left: Sequence[float], right: Sequence[float]
) -> float:
    return sum(left[axis] * right[axis] for axis in range(3))


def _finalization_normal(normal: GridPoint) -> Tuple[float, float, float]:
    values = tuple(_float32(float(value)) for value in normal)
    xy_squared = _float32(
        _float32(values[0] * values[0])
        + _float32(values[1] * values[1])
    )
    length = _float32(
        math.sqrt(
            _float32(xy_squared + _float32(values[2] * values[2]))
        )
    )
    if length <= 0.0:
        # vec3_f32::normalise() leaves the build routine's initialized unit-X
        # fallback in place for a zero XML matrix.
        return (1.0, 0.0, 0.0)
    return tuple(_float32(value / length) for value in values)  # type: ignore[return-value]


def _finalization_determinant(
    first: Sequence[float],
    second: Sequence[float],
    third: Sequence[float],
) -> float:
    return (
        first[0] * (second[1] * third[2] - second[2] * third[1])
        - first[1] * (second[0] * third[2] - second[2] * third[0])
        + first[2] * (second[0] * third[1] - second[1] * third[0])
    )


def _finalization_intersection(
    first: Tuple[Tuple[float, float, float], float],
    second: Tuple[Tuple[float, float, float], float],
    third: Tuple[Tuple[float, float, float], float],
) -> Optional[Tuple[float, float, float]]:
    normals = (first[0], second[0], third[0])
    for left, right in (
        (normals[0], normals[1]),
        (normals[0], normals[2]),
        (normals[1], normals[2]),
    ):
        if abs(_finalization_dot(left, right)) >= _FINALIZATION_PARALLEL_LIMIT:
            return None
    determinant = _finalization_determinant(*normals)
    if determinant == 0.0:
        return None
    distances = (first[1], second[1], third[1])
    coordinates = []
    for column in range(3):
        rows = [list(normal) for normal in normals]
        for row_index in range(3):
            rows[row_index][column] = distances[row_index]
        coordinates.append(_finalization_determinant(*rows) / determinant)
    return tuple(coordinates)  # type: ignore[return-value]


def _finalization_planes(
    minimum: GridPoint,
    maximum: GridPoint,
    planes: Sequence[Plane],
    seed_physics_shape: int,
) -> Tuple[Tuple[Tuple[float, float, float], float], ...]:
    minimum_world = tuple(
        _float32(_float32(float(value)) * 0.25 - 0.125)
        for value in minimum
    )
    maximum_world = tuple(
        _float32(_float32(float(value)) * 0.25 + 0.125)
        for value in maximum
    )
    center = tuple(
        _float32(_float32(minimum_world[axis] + maximum_world[axis]) * 0.5)
        for axis in range(3)
    )
    result: List[Tuple[Tuple[float, float, float], float]] = []
    for axis in range(3):
        negative = tuple(-1.0 if index == axis else 0.0 for index in range(3))
        positive = tuple(1.0 if index == axis else 0.0 for index in range(3))
        negative_distance = _float32(
            _float32(center[axis] - minimum_world[axis]) - _FINALIZATION_MARGIN
        )
        positive_distance = _float32(
            _float32(maximum_world[axis] - center[axis]) - _FINALIZATION_MARGIN
        )
        result.append((negative, float(negative_distance)))
        result.append((positive, float(positive_distance)))

    custom_margin_scale = (
        0.0
        if seed_physics_shape in _ZERO_CUSTOM_PLANE_MARGIN_SHAPES
        else 1.0
    )
    for anchor, normal_int in planes:
        normal = _finalization_normal(normal_int)
        point = []
        for axis in range(3):
            coordinate = _float32(
                _float32(_float32(float(anchor[axis]) - 0.5) * 0.25)
                - center[axis]
            )
            margin = _float32(
                _float32(normal[axis] * _FINALIZATION_MARGIN)
                * custom_margin_scale
            )
            point.append(_float32(coordinate - margin))
        result.append((normal, _finalization_dot(normal, point)))
    return tuple(result)


def finalization_vertex_count(
    minimum: GridPoint,
    maximum: GridPoint,
    planes: Sequence[Plane],
    seed_physics_shape: int,
) -> int:
    """Return build()'s surviving convex vertex count for one merge group.

    Cube-only groups use the native box path and therefore always contribute
    one F2 shape.  Clipped groups enumerate intersections of three sufficiently
    independent planes, discard points outside any half-space, collapse points
    closer than the build's epsilon, and require at least four survivors.
    """

    if not planes:
        return 8
    finalization_planes = _finalization_planes(
        minimum,
        maximum,
        planes,
        seed_physics_shape,
    )
    vertices = []
    for plane_triple in combinations(finalization_planes, 3):
        point = _finalization_intersection(*plane_triple)
        if point is not None:
            vertices.append(point)

    # Mirror mm_vector::erase_reorder: rejected entries are replaced with the
    # current last entry and the same index is tested again.
    index = 0
    while index < len(vertices):
        point = vertices[index]
        if any(
            _finalization_dot(normal, point) > distance + _FINALIZATION_EPSILON
            for normal, distance in finalization_planes
        ):
            vertices[index] = vertices[-1]
            vertices.pop()
        else:
            index += 1

    first_index = 0
    while first_index < len(vertices):
        second_index = first_index + 1
        while second_index < len(vertices):
            distance_squared = sum(
                (vertices[first_index][axis] - vertices[second_index][axis]) ** 2
                for axis in range(3)
            )
            if distance_squared < _FINALIZATION_EPSILON:
                vertices[second_index] = vertices[-1]
                vertices.pop()
            else:
                second_index += 1
        first_index += 1
    return len(vertices)


def _dot(left: Sequence[int], right: Sequence[int]):
    return sum(left[index] * right[index] for index in range(3))


def _transform_grid_anchor(
    anchor: GridPoint, rotation: Tuple[int, ...]
) -> GridPoint:
    # Component matrices act about the centre of the unit voxel.  The native
    # constructor performs this doubled-coordinate calculation with signed
    # integer division, which truncates half-grid results toward zero.  That
    # detail is observable for even XML scale and shear matrices.
    centred = tuple(2 * value - 1 for value in anchor)
    rotated = apply_matrix(rotation, centred)  # type: ignore[arg-type]

    def divide_by_two(value: int) -> int:
        return value // 2 if value >= 0 else -((-value) // 2)

    return tuple(divide_by_two(value + 1) for value in rotated)  # type: ignore[return-value]


def voxel_clip_plane(voxel: WorldVoxel, runtime_flags: int = 0) -> Optional[Plane]:
    """Return the constructor-exact clipping plane for one physics voxel."""

    if voxel.physics_shape == 0:
        return None
    try:
        anchor, normal = NON_CUBE_CLIP_PLANES[voxel.physics_shape]
    except KeyError as error:
        raise UnsupportedPhysicsShapeError(
            "unknown physics_shape {}".format(voxel.physics_shape)
        ) from error
    rotated_anchor = _transform_grid_anchor(
        anchor, voxel.physics_rotation
    )
    rotated_normal = apply_matrix(voxel.physics_rotation, normal)
    anchor_values = list(rotated_anchor)
    normal_values = list(rotated_normal)
    # Constructor runtime mirror bits operate in the already-rotated output
    # axes, not in the primitive's local axes.
    for axis in range(3):
        if runtime_flags & (1 << axis):
            anchor_values[axis] = 1 - anchor_values[axis]
            normal_values[axis] = -normal_values[axis]
    world_anchor = tuple(
        voxel.position[axis] + anchor_values[axis] for axis in range(3)
    )
    return world_anchor, tuple(normal_values)  # type: ignore[return-value]


def planes_coplanar(left: Plane, right: Plane) -> bool:
    if left[1] != right[1]:
        return False
    left_anchor = left[0]
    right_anchor = right[0]
    normal = left[1]
    return (
        (left_anchor[0] - right_anchor[0]) * normal[0]
        + (left_anchor[1] - right_anchor[1]) * normal[1]
        + (left_anchor[2] - right_anchor[2]) * normal[2]
        == 0
    )


def voxel_collide_plane(position: GridPoint, plane: Plane) -> int:
    """Classify a unit grid cell exactly like voxel_collide_plane."""

    anchor, normal = plane
    px, py, pz = position
    ax, ay, az = anchor
    nx, ny, nz = normal
    # Both AABB extrema share the distance from the cell's minimum corner.
    # Computing that dot product once is algebraically identical to applying
    # the sign-selected unit-corner offset independently on every axis.
    corner_distance = (px - ax) * nx + (py - ay) * ny + (pz - az) * nz
    minimum_distance = (
        corner_distance
        + (nx if nx < 0 else 0)
        + (ny if ny < 0 else 0)
        + (nz if nz < 0 else 0)
    )
    maximum_distance = (
        corner_distance
        + (nx if nx >= 0 else 0)
        + (ny if ny >= 0 else 0)
        + (nz if nz >= 0 else 0)
    )
    if minimum_distance >= 0:
        return 1
    if maximum_distance <= 0:
        return -1
    return 0


def voxel_collide_planes(position: GridPoint, planes: Sequence[Plane]) -> int:
    collision = -1
    for plane in planes:
        value = voxel_collide_plane(position, plane)
        if value > 0:
            return 1
        if value == 0:
            collision = 0
    return collision


@lru_cache(maxsize=8192)
def _rotated_sample_offsets(
    shape: int,
    rotation: Tuple[int, ...],
    runtime_flags: int,
) -> Tuple[GridPoint, ...]:
    result = []
    for point in NON_CUBE_SAMPLE_POINTS_QUARTERS[shape]:
        rotated = list(apply_matrix(rotation, point))
        for axis in range(3):
            if runtime_flags & (1 << axis):
                rotated[axis] = -rotated[axis]
        result.append(tuple(rotated))
    return tuple(result)  # type: ignore[return-value]


def physics_voxel_collide_plane(
    voxel: WorldVoxel,
    plane: Plane,
    runtime_flags: int = 0,
    own_plane: Optional[Plane] = None,
) -> int:
    """Mirror physics_voxel_data_collide_plane for build 24749959."""

    anchor, normal = plane
    px, py, pz = voxel.position
    ax, ay, az = anchor
    nx, ny, nz = normal
    corner_distance = (px - ax) * nx + (py - ay) * ny + (pz - az) * nz
    minimum_distance = (
        corner_distance
        + (nx if nx < 0 else 0)
        + (ny if ny < 0 else 0)
        + (nz if nz < 0 else 0)
    )
    maximum_distance = (
        corner_distance
        + (nx if nx >= 0 else 0)
        + (ny if ny >= 0 else 0)
        + (nz if nz >= 0 else 0)
    )
    if minimum_distance >= 0:
        return 1
    if maximum_distance < 0:
        return -1
    if maximum_distance == 0:
        return -1 if voxel.physics_shape else 0
    if voxel.physics_shape == 0:
        return 1

    native_plane = own_plane
    if native_plane is None:
        native_plane = voxel_clip_plane(voxel, runtime_flags)
    if native_plane is None:
        raise AssertionError("non-cube voxel has no clipping plane")
    if native_plane[1] == normal:
        native_anchor = native_plane[0]
        offset = (
            (native_anchor[0] - ax) * nx
            + (native_anchor[1] - ay) * ny
            + (native_anchor[2] - az) * nz
        )
        if offset < 0:
            return 1
        if offset > 0:
            return -1
        return 0

    normal_squared = sum(value * value for value in normal)
    if normal_squared <= 0:
        return -1
    positive = negative = near = 0
    for sample_offset in _rotated_sample_offsets(
        voxel.physics_shape,
        voxel.physics_rotation,
        runtime_flags & 7,
    ):
        # The constructor sample is ``position + 0.5 + quarter / 4``.
        # Multiplying its signed distance numerator by four keeps the complete
        # near-plane test integral.  abs(D / (4 * |normal|)) > 0.01 is exactly
        # ``625 * D^2 > |normal|^2``.  This removes CPU/OS float drift while
        # retaining the native 0.01 boundary for every build-pinned normal.
        distance_numerator = sum(
            (
                4 * (voxel.position[axis] - anchor[axis])
                + 2
                + sample_offset[axis]
            )
            * normal[axis]
            for axis in range(3)
        )
        outside_near_band = (
            625 * distance_numerator * distance_numerator > normal_squared
        )
        if distance_numerator > 0 and outside_near_band:
            positive += 1
        elif distance_numerator < 0 and outside_near_band:
            negative += 1
        else:
            near += 1
    if positive:
        return 1
    threshold = NON_CUBE_COLLISION_THRESHOLDS[voxel.physics_shape - 1]
    count = len(NON_CUBE_SAMPLE_POINTS_QUARTERS[voxel.physics_shape])
    if negative > count - threshold:
        return -1
    return 0 if near == threshold else 1


@dataclass(frozen=True)
class PortableMergeGroup:
    seed_insertion_index: int
    voxel_insertion_indices: Tuple[int, ...]
    component_indices: Tuple[int, ...]
    minimum: GridPoint
    maximum: GridPoint
    planes: Tuple[Plane, ...]
    seed_physics_shape: int = 0

    @property
    def finalization_vertex_count(self) -> int:
        return finalization_vertex_count(
            self.minimum,
            self.maximum,
            self.planes,
            self.seed_physics_shape,
        )

    @property
    def contributes_f2_shape(self) -> bool:
        return not self.planes or self.finalization_vertex_count >= 4


@dataclass(frozen=True)
class PortableMergeResult:
    groups: Tuple[PortableMergeGroup, ...]
    voxel_count: int
    status: str = "portable_build_24749959_plane_merge"
    stormworks_build_id: str = STORMWORKS_BUILD_ID

    @property
    def shape_count(self) -> int:
        return sum(group.contributes_f2_shape for group in self.groups)

    @property
    def merge_group_count(self) -> int:
        return len(self.groups)

    @property
    def rejected_convex_group_count(self) -> int:
        return sum(not group.contributes_f2_shape for group in self.groups)

    @property
    def finalized_groups(self) -> Tuple[PortableMergeGroup, ...]:
        return tuple(group for group in self.groups if group.contributes_f2_shape)


class PortableMergeOracle:
    """Drop-in partition backend used by the optimizer and desktop app."""

    backend = "portable_build_24749959"
    binary_path = None
    binary_sha256 = SOURCE_BINARY_SHA256

    def __init__(self, allow_overlaps: bool = False):
        self.allow_overlaps = allow_overlaps

    def partition(
        self,
        voxels: Sequence[WorldVoxel],
        component_runtime_flags: Optional[Mapping[int, int]] = None,
    ) -> PortableMergeResult:
        return partition_portable_exact(
            voxels,
            component_runtime_flags,
            allow_overlaps=self.allow_overlaps,
        )


class PreparedPortableMergeEvaluator:
    """Reuse immutable native-merge inputs across many component orderings.

    Stormworks constructs each physics voxel and its clip plane once before
    the builder walks its source-order seed vector.  The optimizer used to
    reconstruct all ``WorldVoxel`` objects, clip planes, and spatial lookup
    data for every candidate.  This prepared form keeps the native invariant
    data once and changes only the seed order, insertion ranks, and processed
    flags for each evaluation.
    """

    def __init__(
        self,
        component_voxels: Sequence[Sequence[WorldVoxel]],
        trailing_voxels: Sequence[WorldVoxel] = (),
        component_runtime_flags: Optional[Mapping[int, int]] = None,
        allow_overlaps: bool = False,
    ):
        groups = tuple(tuple(group) for group in component_voxels)
        if any(not group for group in groups):
            raise ValueError("every prepared component must contribute physics voxels")
        trailing = tuple(trailing_voxels)
        voxels: List[WorldVoxel] = []
        group_indices = []
        for group in groups:
            start = len(voxels)
            voxels.extend(group)
            group_indices.append(tuple(range(start, len(voxels))))
        trailing_start = len(voxels)
        voxels.extend(trailing)
        self.voxels = tuple(voxels)
        self.component_voxel_indices = tuple(group_indices)
        self.trailing_voxel_indices = tuple(range(trailing_start, len(voxels)))
        self.runtime_flags = component_runtime_flags or {}
        self.voxel_runtime_flags = tuple(
            self.runtime_flags.get(voxel.component_index, 0) & 7
            for voxel in self.voxels
        )
        self.voxel_planes = tuple(
            voxel_clip_plane(voxel, self.voxel_runtime_flags[index])
            for index, voxel in enumerate(self.voxels)
        )
        positions = tuple(voxel.position for voxel in self.voxels)
        first_by_position: Dict[GridPoint, int] = {}
        duplicate_positions = set()
        for index, position in enumerate(positions):
            if position in first_by_position:
                duplicate_positions.add(position)
            # Keep the original source order's latest winner as the immutable
            # baseline.  Candidate orders which preserve every overlap winner
            # can use this map directly without either copying or mutation.
            first_by_position[position] = index
        self.has_overlaps = bool(duplicate_positions)
        if self.has_overlaps and not allow_overlaps:
            raise ValueError("overlapping physics voxel positions are not supported")
        self.allow_overlaps = allow_overlaps
        self._position_lookup_base = first_by_position
        self._overlap_voxel_indices_by_component = tuple(
            tuple(
                voxel_index
                for voxel_index in indices
                if positions[voxel_index] in duplicate_positions
            )
            for indices in self.component_voxel_indices
        )
        self._overlap_trailing_voxel_indices = tuple(
            voxel_index
            for voxel_index in self.trailing_voxel_indices
            if positions[voxel_index] in duplicate_positions
        )
        self._native_evaluator = None
        self.native_backend = "python"
        self.native_error = None
        # Build 24749959 mirrors a non-grid clip-plane anchor differently from
        # the grid/output-axis rule when runtime flag bits are nonzero.  Keep
        # that unverified combination on the quarantined Python path rather
        # than claiming Rust parity.  Ordinary vehicle analysis supplies zero
        # runtime flags, so XML scale, shear, and singular matrices still use
        # the native scorer.
        requires_python_runtime_mirror = any(
            runtime_flags
            and voxel.physics_shape != 0
            and voxel.physics_rotation not in _GRID_TRANSFORM_SET
            for voxel, runtime_flags in zip(
                self.voxels,
                self.voxel_runtime_flags,
            )
        )
        self._native_verify = os.environ.get(
            "SWPHYSICS_NATIVE_VERIFY", ""
        ).strip().lower() in {"1", "true", "yes", "on"}
        self._native_verified = False
        try:
            from .native_merge import create_native_prepared_evaluator

            if not requires_python_runtime_mirror:
                self._native_evaluator = create_native_prepared_evaluator(
                    self.voxels,
                    self.component_voxel_indices,
                    self.trailing_voxel_indices,
                    self.voxel_planes,
                    self.voxel_runtime_flags,
                    self.allow_overlaps,
                )
            if self._native_evaluator is not None:
                self.native_backend = "rust_cdylib"
            elif requires_python_runtime_mirror:
                self.native_backend = "python_runtime_mirror"
        except Exception as error:
            # The build-pinned Python implementation is the correctness
            # reference and must remain usable on unsupported, quarantined,
            # or partially upgraded installations.
            self.native_error = str(error)

    def partition_order(
        self, component_order: Sequence[int]
    ) -> PortableMergeResult:
        return self._partition_order(
            component_order,
            collect_groups=True,
            validate_order=True,
        )

    def shape_count_order(self, component_order: Sequence[int]) -> int:
        """Evaluate a trusted candidate without constructing preview groups."""

        if self._native_evaluator is not None:
            try:
                native_count = self._native_evaluator.score(component_order)
            except Exception as error:
                self.native_error = str(error)
                self.native_backend = "python_fallback"
                try:
                    self._native_evaluator.close()
                finally:
                    self._native_evaluator = None
            else:
                if self._native_verify and not self._native_verified:
                    python_count = self._partition_order(
                        component_order,
                        collect_groups=False,
                        validate_order=False,
                    )
                    if native_count != python_count:
                        raise RuntimeError(
                            "native/Python shape score mismatch: {} != {}".format(
                                native_count, python_count
                            )
                        )
                    self._native_verified = True
                return native_count

        return self._partition_order(
            component_order,
            collect_groups=False,
            validate_order=False,
        )

    def _partition_order(
        self,
        component_order: Sequence[int],
        collect_groups: bool,
        validate_order: bool,
    ) -> Any:
        order = tuple(component_order)
        component_count = len(self.component_voxel_indices)
        if len(order) != component_count:
            raise ValueError("prepared component order must be a permutation")
        if validate_order:
            seen_components = bytearray(component_count)
            for component_index in order:
                if (
                    component_index < 0
                    or component_index >= component_count
                    or seen_components[component_index]
                ):
                    raise ValueError("prepared component order must be a permutation")
                seen_components[component_index] = 1
        if collect_groups:
            ordered_indices: Iterable[int] = tuple(
                voxel_index
                for component_index in order
                for voxel_index in self.component_voxel_indices[component_index]
            ) + self.trailing_voxel_indices
        else:
            # Score-only search consumes seed indices once and does not need
            # random access or insertion ranks.  Stream the component groups
            # to avoid allocating a vehicle-sized tuple for every candidate.
            ordered_indices = chain(
                chain.from_iterable(
                    map(self.component_voxel_indices.__getitem__, order)
                ),
                self.trailing_voxel_indices,
            )
        insertion_rank = None
        if collect_groups:
            insertion_rank = [0] * len(self.voxels)
            for rank, voxel_index in enumerate(ordered_indices):
                insertion_rank[voxel_index] = rank
        by_position = self._position_lookup_base
        if self.has_overlaps:
            # Unique positions are invariant under component reordering.  Find
            # only the duplicate positions' candidate-specific latest winner;
            # trailing Flooder voxels are deliberately written last, matching
            # the native vector/octree latest-wins split.
            overlap_winners: Dict[GridPoint, int] = {}
            for component_index in order:
                for voxel_index in self._overlap_voxel_indices_by_component[
                    component_index
                ]:
                    overlap_winners[
                        self.voxels[voxel_index].position
                    ] = voxel_index
            for voxel_index in self._overlap_trailing_voxel_indices:
                overlap_winners[self.voxels[voxel_index].position] = voxel_index
            if any(
                by_position[position] != voxel_index
                for position, voxel_index in overlap_winners.items()
            ):
                # Never mutate the shared baseline: changed-winner candidates
                # receive an isolated copy, preserving thread safety.
                by_position = by_position.copy()
                by_position.update(overlap_winners)
        processed = bytearray(len(self.voxels))
        groups: List[PortableMergeGroup] = []
        shape_count = 0
        for seed_index in ordered_indices:
            if processed[seed_index]:
                continue
            seed = self.voxels[seed_index]
            group = [seed_index]
            processed[seed_index] = 1
            minimum = maximum = seed.position
            seed_plane = self.voxel_planes[seed_index]
            planes = tuple(() if seed_plane is None else (seed_plane,))
            # The native builder keeps every source voxel in its seed vector,
            # while its spatial lookup retains only the latest voxel written
            # at a position.  When an older duplicate becomes a seed, the
            # lookup winner inside the initialized seed hull is consumed into
            # that same group immediately.  It does not contribute its own
            # clip plane: the seed still defines the initial geometry.  A
            # singular XML transform can put the seed cell wholly outside its
            # own transformed plane; native merge_shape does not consume the
            # overlap winner in that case.
            if (
                self.has_overlaps
                and voxel_collide_planes(seed.position, planes) <= 0
            ):
                overlap_winner = by_position.get(seed.position)
                if (
                    overlap_winner is not None
                    and overlap_winner != seed_index
                    and not processed[overlap_winner]
                ):
                    group.append(overlap_winner)
                    processed[overlap_winner] = 1
            blocked = [False] * len(DIRECTIONS)
            while True:
                changed = False
                for direction_index, direction in enumerate(DIRECTIONS):
                    if blocked[direction_index]:
                        continue
                    expansion = _try_expand(
                        group,
                        minimum,
                        maximum,
                        planes,
                        direction,
                        self.voxels,
                        by_position,
                        processed,
                        self.voxel_planes,
                        self.voxel_runtime_flags,
                    )
                    if expansion is _RETRY_AFTER_PERPENDICULAR_EXPANSION:
                        continue
                    if expansion is None:
                        blocked[direction_index] = True
                        continue
                    group, minimum, maximum, planes = expansion
                    changed = True
                if not changed:
                    break
            shape_count += int(
                not planes
                or finalization_vertex_count(
                    minimum,
                    maximum,
                    planes,
                    seed.physics_shape,
                ) >= 4
            )
            if collect_groups:
                if insertion_rank is None:
                    raise RuntimeError("prepared insertion ranks were not built")
                groups.append(
                    PortableMergeGroup(
                        seed_insertion_index=insertion_rank[seed_index],
                        voxel_insertion_indices=tuple(
                            sorted(insertion_rank[voxel_index] for voxel_index in group)
                        ),
                        component_indices=tuple(
                            dict.fromkeys(
                                self.voxels[voxel_index].component_index
                                for voxel_index in group
                            )
                        ),
                        minimum=minimum,
                        maximum=maximum,
                        planes=planes,
                        seed_physics_shape=seed.physics_shape,
                    )
                )
        if not collect_groups:
            return shape_count
        return PortableMergeResult(
            tuple(groups),
            len(self.voxels),
            status=(
                "portable_build_24749959_overlap_preview"
                if self.has_overlaps
                else "portable_build_24749959_plane_merge"
            ),
        )


def _bounds(voxels: Sequence[WorldVoxel]) -> Tuple[GridPoint, GridPoint]:
    return (
        tuple(min(voxel.position[axis] for voxel in voxels) for axis in range(3)),
        tuple(max(voxel.position[axis] for voxel in voxels) for axis in range(3)),
    )  # type: ignore[return-value]


def _layer_positions(
    minimum: GridPoint, maximum: GridPoint, direction: GridPoint
) -> Iterable[GridPoint]:
    # This sits in the hottest native-merge loop.  DIRECTIONS contains only
    # the six axis-aligned unit vectors, so avoid rebuilding a list of ranges
    # and scanning the direction on every attempted expansion.  Keep the
    # x/y/z product order identical to the generic implementation.
    dx, dy, dz = direction
    min_x, min_y, min_z = minimum
    max_x, max_y, max_z = maximum
    if dx:
        x = min_x - 1 if dx < 0 else max_x + 1
        return product((x,), range(min_y, max_y + 1), range(min_z, max_z + 1))
    if dy:
        y = min_y - 1 if dy < 0 else max_y + 1
        return product(range(min_x, max_x + 1), (y,), range(min_z, max_z + 1))
    z = min_z - 1 if dz < 0 else max_z + 1
    return product(range(min_x, max_x + 1), range(min_y, max_y + 1), (z,))


def _try_expand(
    group: List[int],
    minimum: GridPoint,
    maximum: GridPoint,
    planes: Sequence[Plane],
    direction: GridPoint,
    voxels: Sequence[WorldVoxel],
    by_position: Mapping[GridPoint, int],
    processed: bytearray,
    voxel_planes: Sequence[Optional[Plane]],
    voxel_runtime_flags: Sequence[int],
) -> Optional[Tuple[List[int], GridPoint, GridPoint, Tuple[Plane, ...]]]:
    additions: List[int] = []
    temporary_planes: List[Plane] = []
    has_existing_planes = bool(planes)
    for position in _layer_positions(minimum, maximum, direction):
        # Cube-only groups have no clipping planes.  This is the overwhelmingly
        # common path on large vehicles, so avoid a Python function call for
        # every candidate layer cell when the native result is unconditionally
        # ``inside`` (-1).
        shape_collision = (
            voxel_collide_planes(position, planes)
            if has_existing_planes
            else -1
        )
        if shape_collision > 0:
            continue
        index = by_position.get(position)
        if index is None or processed[index]:
            return None
        voxel = voxels[index]
        own_plane = voxel_planes[index]
        if own_plane is None:
            # A full cube is valid only when the candidate cell lies wholly
            # inside every existing clipping plane.  An intersecting cube
            # would fill space the current convex boundary excludes.
            if shape_collision >= 0:
                return None
            additions.append(index)
            continue

        if any(planes_coplanar(own_plane, plane) for plane in planes):
            additions.append(index)
            continue

        collided_positive = False
        collided_boundary = False
        for plane in planes:
            if voxel_collide_plane(position, plane) != 0:
                continue
            collision = physics_voxel_collide_plane(
                voxel,
                plane,
                voxel_runtime_flags[index],
                own_plane,
            )
            if collision == 0:
                collided_boundary = True
            elif collision > 0:
                collided_positive = True
        if collided_positive:
            return None

        if not collided_boundary:
            # The candidate contributes a new plane only if that plane does
            # not expose an incompatible voxel anywhere inside the old AABB.
            for old_position in product(
                *(range(minimum[axis], maximum[axis] + 1) for axis in range(3))
            ):
                if voxel_collide_plane(old_position, own_plane) < 0:
                    continue
                old_index = by_position.get(old_position)
                if old_index is None:
                    continue
                old_voxel = voxels[old_index]
                old_plane = voxel_planes[old_index]
                if old_plane is None:
                    return None
                if physics_voxel_collide_plane(
                    old_voxel,
                    own_plane,
                    voxel_runtime_flags[old_index],
                    old_plane,
                ) > 0:
                    return None
            temporary_planes.append(own_plane)
        additions.append(index)
    if not additions:
        return _RETRY_AFTER_PERPENDICULAR_EXPANSION  # type: ignore[return-value]

    # Native second pass: if any old/new plane crosses a candidate unit cell,
    # at least one crossing must also intersect that voxel's real non-cube
    # sample geometry.  This is deliberately not generic convex containment;
    # it preserves the game's shape-specific threshold behaviour.
    candidate_planes = tuple(temporary_planes) + tuple(planes)
    # ``additions`` is exactly the subset of layer cells which the first pass
    # did not reject as lying outside an existing plane.  Reusing it avoids a
    # second spatial lookup and a second full old-plane collision scan for
    # every cell while preserving the native layer order.
    for index in additions:
        voxel = voxels[index]
        position = voxel.position
        own_plane = voxel_planes[index]
        # Native uses only the newly proposed planes as the reject trigger.
        # Existing planes may still provide the shape-intersection evidence,
        # but merely crossing one of them does not by itself invalidate this
        # new layer.  Including old planes in both roles incorrectly split
        # XML-scaled adjacent wedge groups.
        cell_intersects = any(
            voxel_collide_plane(position, plane) == 0
            for plane in temporary_planes
        )
        shape_intersects = False
        for plane in candidate_planes:
            if voxel_collide_plane(position, plane) != 0:
                continue
            if own_plane is not None and physics_voxel_collide_plane(
                voxel,
                plane,
                voxel_runtime_flags[index],
                own_plane,
            ) == 0:
                shape_intersects = True
        if cell_intersects and not shape_intersects:
            return None

    proposed_planes = list(planes)
    for plane in temporary_planes:
        if not any(planes_coplanar(plane, item) for item in proposed_planes):
            proposed_planes.append(plane)
    # Match the axis-specialized layer iterator above and avoid constructing a
    # generator after every successful expansion.
    if direction[0]:
        axis = 0
    elif direction[1]:
        axis = 1
    else:
        axis = 2
    proposed_minimum = list(minimum)
    proposed_maximum = list(maximum)
    if direction[axis] < 0:
        proposed_minimum[axis] -= 1
    else:
        proposed_maximum[axis] += 1
    group.extend(additions)
    for index in additions:
        processed[index] = 1
    return (
        group,
        tuple(proposed_minimum),  # type: ignore[return-value]
        tuple(proposed_maximum),  # type: ignore[return-value]
        tuple(proposed_planes),
    )


def partition_portable_exact(
    voxels: Sequence[WorldVoxel],
    component_runtime_flags: Optional[Mapping[int, int]] = None,
    allow_overlaps: bool = False,
) -> PortableMergeResult:
    """Group voxels without loading the game executable.

    When overlap support is enabled, the spatial lookup retains the latest
    voxel at each coordinate while the source-order seed vector retains every
    voxel.  Older seeds consume the current lookup winner only when that cell
    is inside their initialized hull, matching the native octree/vector split.
    Reordering safety is handled separately by pinning every component
    involved in an overlap.
    """

    ordered = tuple(voxels)
    positions = [voxel.position for voxel in ordered]
    has_overlaps = len(positions) != len(set(positions))
    if has_overlaps and not allow_overlaps:
        raise ValueError("overlapping physics voxel positions are not supported")
    runtime_flags = component_runtime_flags or {}
    voxel_planes = tuple(
        voxel_clip_plane(voxel, runtime_flags.get(voxel.component_index, 0))
        for voxel in ordered
    )
    voxel_runtime_flags = tuple(
        runtime_flags.get(voxel.component_index, 0) & 7 for voxel in ordered
    )
    by_position = {voxel.position: index for index, voxel in enumerate(ordered)}
    processed = bytearray(len(ordered))
    groups: List[PortableMergeGroup] = []
    for seed_index, seed in enumerate(ordered):
        if processed[seed_index]:
            continue
        group = [seed_index]
        processed[seed_index] = 1
        minimum = maximum = seed.position
        planes = tuple(
            ()
            if voxel_planes[seed_index] is None
            else (voxel_planes[seed_index],)
        )
        # Match the game's vector/octree split for repeated positions.  The
        # latest spatial-lookup winner inside the initialized hull joins an
        # older seed's group, but that seed alone supplies the initial plane.
        if has_overlaps and voxel_collide_planes(seed.position, planes) <= 0:
            overlap_winner = by_position.get(seed.position)
            if (
                overlap_winner is not None
                and overlap_winner != seed_index
                and not processed[overlap_winner]
            ):
                group.append(overlap_winner)
                processed[overlap_winner] = 1
        blocked = [False] * len(DIRECTIONS)
        while True:
            changed = False
            for direction_index, direction in enumerate(DIRECTIONS):
                if blocked[direction_index]:
                    continue
                expansion = _try_expand(
                    group,
                    minimum,
                    maximum,
                    planes,
                    direction,
                    ordered,
                    by_position,
                    processed,
                    voxel_planes,
                    voxel_runtime_flags,
                )
                if expansion is _RETRY_AFTER_PERPENDICULAR_EXPANSION:
                    continue
                if expansion is None:
                    blocked[direction_index] = True
                    continue
                group, minimum, maximum, planes = expansion
                changed = True
            if not changed:
                break
        groups.append(
            PortableMergeGroup(
                seed_insertion_index=seed.insertion_index,
                voxel_insertion_indices=tuple(
                    ordered[index].insertion_index
                    for index in sorted(group)
                ),
                component_indices=tuple(
                    dict.fromkeys(ordered[index].component_index for index in group)
                ),
                minimum=minimum,
                maximum=maximum,
                planes=planes,
                seed_physics_shape=seed.physics_shape,
            )
        )
    return PortableMergeResult(
        tuple(groups),
        len(ordered),
        status=(
            "portable_build_24749959_overlap_preview"
            if has_overlaps
            else "portable_build_24749959_plane_merge"
        ),
    )
