"""Classify physics geometry that is outside the exact portable model.

Keeping these diagnostics separate from the GUI and optimizer gives release
work a stable way to measure XML-edit coverage.  A Component remains the
atomic unit: if any of its physics voxels is unsupported, the optimizer pins
that complete Component in its original slot.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Sequence, Tuple

from .model import WorldVoxel
from .portable_merge import (
    UnsupportedPhysicsShapeError,
    voxel_clip_plane,
)
from .rotations import GRID_TRANSFORMS
from .vehicle import ComponentPlacement


NON_GRID_COMPONENT_TRANSFORM = "non_grid_component_transform"
NON_GRID_PHYSICS_ROTATION = "non_grid_physics_rotation"
UNKNOWN_PHYSICS_SHAPE = "unknown_physics_shape"
GRID_TRANSFORM = "grid"
AXIS_SCALE_TRANSFORM = "axis_scale"
GENERAL_NON_AXIS_TRANSFORM = "general_non_axis"
SINGULAR_TRANSFORM = "singular"
_GRID_TRANSFORM_SET = frozenset(GRID_TRANSFORMS)


def _matrix_determinant(matrix: Tuple[int, ...]) -> int:
    # Serialized matrices are column-major; determinant is layout invariant.
    a, d, g, b, e, h, c, f, i = matrix
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def classify_component_transform(matrix: Tuple[int, ...]) -> str:
    """Classify a placement matrix for staged XML-edit implementation."""

    if matrix in _GRID_TRANSFORM_SET:
        return GRID_TRANSFORM
    rows = tuple(
        tuple(matrix[column * 3 + row] for column in range(3))
        for row in range(3)
    )
    columns = tuple(
        tuple(matrix[column * 3 + row] for row in range(3))
        for column in range(3)
    )
    if all(sum(value != 0 for value in row) == 1 for row in rows) and all(
        sum(value != 0 for value in column) == 1 for column in columns
    ):
        return AXIS_SCALE_TRANSFORM
    if _matrix_determinant(matrix) == 0:
        return SINGULAR_TRANSFORM
    return GENERAL_NON_AXIS_TRANSFORM


@dataclass(frozen=True)
class ComponentPhysicsSupport:
    """Coverage result for one Component placement."""

    component_index: int
    definition_id: str
    physics_voxel_count: int
    non_cube_voxel_count: int
    transform_kind: str
    issue_codes: Tuple[str, ...]

    @property
    def supported(self) -> bool:
        return not self.issue_codes


def classify_component_physics_support(
    components: Tuple[ComponentPlacement, ...],
    voxels: Sequence[WorldVoxel],
) -> Tuple[ComponentPhysicsSupport, ...]:
    """Return deterministic, reason-coded coverage for every Component.

    Full cubes and clipped non-cube voxels are supported under the complete
    integer transform matrix stored by the game, including XML scale, shear,
    reflection, and singular transforms.  ``transform_kind`` remains useful
    audit metadata; it is not by itself a reason to exclude a Component.
    """

    voxels_by_component: Dict[int, list[WorldVoxel]] = {
        index: [] for index in range(len(components))
    }
    for voxel in voxels:
        voxels_by_component.setdefault(voxel.component_index, []).append(voxel)

    results = []
    for component_index, component in enumerate(components):
        component_voxels = voxels_by_component.get(component_index, [])
        non_cube_voxels = tuple(
            voxel for voxel in component_voxels if voxel.physics_shape != 0
        )
        transform_kind = classify_component_transform(
            component.effective_transform
        )
        issues = set()

        for voxel in non_cube_voxels:
            try:
                voxel_clip_plane(voxel)
            except UnsupportedPhysicsShapeError:
                issues.add(UNKNOWN_PHYSICS_SHAPE)

        results.append(
            ComponentPhysicsSupport(
                component_index=component_index,
                definition_id=component.definition_id,
                physics_voxel_count=len(component_voxels),
                non_cube_voxel_count=len(non_cube_voxels),
                transform_kind=transform_kind,
                issue_codes=tuple(sorted(issues)),
            )
        )
    return tuple(results)


def unsupported_component_indices(
    components: Tuple[ComponentPlacement, ...],
    voxels: Sequence[WorldVoxel],
) -> set[int]:
    """Return Components that must stay outside the exact scoring model."""

    return {
        result.component_index
        for result in classify_component_physics_support(components, voxels)
        if not result.supported
    }
