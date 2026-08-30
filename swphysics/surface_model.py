"""Build-pinned helpers for Physics Flooder buoyancy-surface probes.

The component Definition ``<buoyancy_surfaces>`` records are a separate grid
from both its ordinary ``<surfaces>`` records and physics voxels. Stormworks
rotates the buoyancy records into the body grid before sealed-volume discovery.
This module keeps the six meaningful roof-facing rotation classes and exposes
a binary-backed signature helper used to select a small, non-duplicated in-game
probe set.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Tuple

from .definitions import ComponentDefinition
from .model import GridPoint, Matrix3, apply_matrix, multiply_matrices

WORLD_DOWN: GridPoint = (0, -1, 0)

# One proper (determinant +1) component rotation for each local direction that
# can face the chamber interior.  Yaw around WORLD_DOWN does not change the
# result of the square test chamber and is deliberately excluded.
ROOF_ROTATIONS: Tuple[Tuple[GridPoint, Matrix3], ...] = (
    ((0, -1, 0), (1, 0, 0, 0, 1, 0, 0, 0, 1)),
    ((0, 1, 0), (1, 0, 0, 0, -1, 0, 0, 0, -1)),
    ((0, 0, 1), (-1, 0, 0, 0, 0, -1, 0, -1, 0)),
    ((0, 0, -1), (-1, 0, 0, 0, 0, 1, 0, 1, 0)),
    ((1, 0, 0), (0, -1, 0, -1, 0, 0, 0, 0, -1)),
    ((-1, 0, 0), (0, 1, 0, -1, 0, 0, 0, 0, 1)),
)

OBSERVED_STAGE7_LOCAL_DIRECTIONS = frozenset(((0, -1, 0), (0, 1, 0)))


@dataclass(frozen=True, order=True)
class SurfaceRecord:
    """One supported Definition surface after component rotation."""

    position: GridPoint
    normal_direction: GridPoint
    primary_type: int
    secondary_type: int
    primary_direction: GridPoint


@dataclass(frozen=True, order=True)
class DefinitionDownSurfaceRecord:
    """A downward Definition record, including native-unsupported shapes."""

    position: GridPoint
    definition_shape: int
    supported: bool
    primary_type: int
    secondary_type: int


def _rotate_float_vector(
    matrix: Matrix3, vector: Tuple[float, float, float]
) -> Tuple[float, float, float]:
    x, y, z = vector
    return (
        matrix[0] * x + matrix[3] * y + matrix[6] * z,
        matrix[1] * x + matrix[4] * y + matrix[7] * z,
        matrix[2] * x + matrix[5] * y + matrix[8] * z,
    )


def rounded_surface_direction(normal: Tuple[float, float, float]) -> GridPoint:
    """Match the axis/diagonal integer normal consumed by get_surface_type."""

    return tuple(
        0 if abs(value) < 0.5 else (1 if value > 0 else -1)
        for value in normal
    )  # type: ignore[return-value]


def binary_surface_signature(
    oracle: Any,
    definition: ComponentDefinition,
    component_rotation: Matrix3,
) -> Tuple[SurfaceRecord, ...]:
    """Return the exact supported build-24749959 surface signature.

    Unsupported Definition surface shapes are skipped exactly as the native
    ``get_surface_type`` call does.  Coordinates remain relative to the
    component anchor so callers can distinguish multi-cell coverage.
    """

    records = []
    for surface in definition.buoyancy_surfaces:
        descriptor = oracle.surface_descriptor(
            surface.orientation,
            surface.rotation,
            surface.shape,
            surface.transmission_type,
        )
        world_normal = _rotate_float_vector(
            component_rotation, descriptor.local_normal
        )
        direction = rounded_surface_direction(world_normal)
        world_rotation = multiply_matrices(
            component_rotation, descriptor.local_rotation
        )
        surface_type = oracle.surface_type(
            world_rotation, surface.shape, direction
        )
        if not surface_type.supported:
            continue
        primary_direction = oracle.surface_type_direction(surface_type.primary)
        records.append(
            SurfaceRecord(
                position=apply_matrix(component_rotation, surface.position),
                normal_direction=direction,
                primary_type=surface_type.primary,
                secondary_type=surface_type.secondary,
                primary_direction=primary_direction,
            )
        )
    return tuple(sorted(records))


def binary_down_surface_signature(
    oracle: Any,
    definition: ComponentDefinition,
    component_rotation: Matrix3,
) -> Tuple[SurfaceRecord, ...]:
    """Return only the supported interface records facing chamber interior."""

    return tuple(
        record
        for record in binary_surface_signature(
            oracle, definition, component_rotation
        )
        if record.normal_direction == WORLD_DOWN
    )


def binary_definition_down_signature(
    oracle: Any,
    definition: ComponentDefinition,
    component_rotation: Matrix3,
) -> Tuple[DefinitionDownSurfaceRecord, ...]:
    """Return all downward Definition records before unsupported shapes vanish."""

    records = []
    for surface in definition.buoyancy_surfaces:
        descriptor = oracle.surface_descriptor(
            surface.orientation,
            surface.rotation,
            surface.shape,
            surface.transmission_type,
        )
        direction = rounded_surface_direction(
            _rotate_float_vector(component_rotation, descriptor.local_normal)
        )
        if direction != WORLD_DOWN:
            continue
        surface_type = oracle.surface_type(
            multiply_matrices(component_rotation, descriptor.local_rotation),
            surface.shape,
            direction,
        )
        records.append(
            DefinitionDownSurfaceRecord(
                position=apply_matrix(component_rotation, surface.position),
                definition_shape=surface.shape,
                supported=surface_type.supported,
                primary_type=surface_type.primary,
                secondary_type=surface_type.secondary,
            )
        )
    return tuple(sorted(records))


def remaining_roof_rotations() -> Tuple[Tuple[GridPoint, Matrix3], ...]:
    return tuple(
        item
        for item in ROOF_ROTATIONS
        if item[0] not in OBSERVED_STAGE7_LOCAL_DIRECTIONS
    )
