"""Conservative Physics Flooder model for fully cubic sealed volumes."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Tuple

from .definitions import DefinitionCatalog
from .model import GridPoint, IDENTITY_MATRIX, WorldVoxel
from .vehicle import Vehicle


_NEIGHBORS: Tuple[GridPoint, ...] = (
    (1, 0, 0),
    (-1, 0, 0),
    (0, 1, 0),
    (0, -1, 0),
    (0, 0, 1),
    (0, 0, -1),
)


@dataclass(frozen=True)
class CubeFloodFillResult:
    supported: bool
    status: str
    static_voxels: Tuple[WorldVoxel, ...]
    fill_voxels: Tuple[WorldVoxel, ...]
    filled_positions: Tuple[GridPoint, ...]
    scan_cell_count: int

    @property
    def all_voxels(self) -> Tuple[WorldVoxel, ...]:
        return self.static_voxels + self.fill_voxels


def model_cube_physics_flood_fill(
    vehicle: Vehicle,
    catalog: DefinitionCatalog,
    body_index: int,
    max_scan_cells: int = 2_000_000,
) -> CubeFloodFillResult:
    """Append cubes for grid cells enclosed by cubic component physics.

    Build 24749959 calls ``fill_physics_voxel`` after sealed-volume discovery.
    Stage 6 I2 established that the two enclosed empty cells of a 3x4x3 cube
    shell are filled and the resulting 36 cubes merge into one F2 shape.  This
    model is deliberately limited to bodies whose static physics voxels are all
    cubes; wedges and other partial-cell sealing still require surface-level
    reconstruction.
    """

    body = vehicle.bodies[body_index]
    static_voxels = vehicle.physics_voxels(catalog, body_index)
    flooder_indices = tuple(
        component.index
        for component in body.components
        if catalog.load(component.definition_id).water_component_type == 19
    )
    if not flooder_indices:
        return CubeFloodFillResult(
            supported=True,
            status="no_physics_flooder",
            static_voxels=static_voxels,
            fill_voxels=(),
            filled_positions=(),
            scan_cell_count=0,
        )
    if any(voxel.physics_shape != 0 for voxel in static_voxels):
        return CubeFloodFillResult(
            supported=False,
            status="unsupported_non_cube_sealing_surface",
            static_voxels=static_voxels,
            fill_voxels=(),
            filled_positions=(),
            scan_cell_count=0,
        )
    if not static_voxels:
        return CubeFloodFillResult(
            supported=False,
            status="unsupported_empty_physics_body",
            static_voxels=static_voxels,
            fill_voxels=(),
            filled_positions=(),
            scan_cell_count=0,
        )

    occupied = {voxel.position for voxel in static_voxels}
    xs = [point[0] for point in occupied]
    ys = [point[1] for point in occupied]
    zs = [point[2] for point in occupied]
    minimum = (min(xs), min(ys), min(zs))
    maximum = (max(xs), max(ys), max(zs))
    outer_minimum = tuple(value - 1 for value in minimum)
    outer_maximum = tuple(value + 1 for value in maximum)
    dimensions = tuple(
        outer_maximum[index] - outer_minimum[index] + 1 for index in range(3)
    )
    scan_cell_count = dimensions[0] * dimensions[1] * dimensions[2]
    if scan_cell_count > max_scan_cells:
        return CubeFloodFillResult(
            supported=False,
            status="unsupported_flood_scan_limit_exceeded",
            static_voxels=static_voxels,
            fill_voxels=(),
            filled_positions=(),
            scan_cell_count=scan_cell_count,
        )

    def inside_outer(point: GridPoint) -> bool:
        return all(
            outer_minimum[index] <= point[index] <= outer_maximum[index]
            for index in range(3)
        )

    start = outer_minimum  # Expanded bounds guarantee this corner is outside.
    outside = {start}
    queue = deque([start])
    while queue:
        point = queue.popleft()
        for delta in _NEIGHBORS:
            neighbor = (
                point[0] + delta[0],
                point[1] + delta[1],
                point[2] + delta[2],
            )
            if (
                inside_outer(neighbor)
                and neighbor not in occupied
                and neighbor not in outside
            ):
                outside.add(neighbor)
                queue.append(neighbor)

    enclosed = tuple(
        (x, y, z)
        for x in range(minimum[0], maximum[0] + 1)
        for y in range(minimum[1], maximum[1] + 1)
        for z in range(minimum[2], maximum[2] + 1)
        if (x, y, z) not in occupied and (x, y, z) not in outside
    )
    first_flooder_index = flooder_indices[0]
    fill_voxels = tuple(
        WorldVoxel(
            body_index=body.index,
            body_id=body.body_id,
            component_index=first_flooder_index,
            component_definition="__physics_flood_fill__",
            definition_voxel_index=-(fill_index + 1),
            insertion_index=len(static_voxels) + fill_index,
            position=position,
            physics_shape=0,
            physics_rotation=IDENTITY_MATRIX,
        )
        for fill_index, position in enumerate(enclosed)
    )
    return CubeFloodFillResult(
        supported=True,
        status="cube_occupancy_enclosure_model_stage6_observed",
        static_voxels=static_voxels,
        fill_voxels=fill_voxels,
        filled_positions=enclosed,
        scan_cell_count=scan_cell_count,
    )
