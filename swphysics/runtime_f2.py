"""F2 debug-overlay voxel input facade.

The build-24749959 static builder now reproduces the observed Pumpjack result
directly.  The result wrapper remains so callers do not need a versioned API
change if a genuinely runtime-only F2 adjustment is discovered later.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from .definitions import DefinitionCatalog
from .model import WorldVoxel
from .portable_merge import STORMWORKS_BUILD_ID
from .vehicle import Vehicle


@dataclass(frozen=True)
class RuntimeF2Adjustment:
    rule_id: str
    component_index: int
    definition_id: str
    definition_physics_signature: str
    removed_voxel_count: int
    evidence: str


@dataclass(frozen=True)
class RuntimeF2VoxelModel:
    voxels: Tuple[WorldVoxel, ...]
    adjustments: Tuple[RuntimeF2Adjustment, ...]
    stormworks_build_id: str = STORMWORKS_BUILD_ID

    @property
    def status(self) -> str:
        return "binary_static_f2_model"


def apply_runtime_f2_voxel_model(
    vehicle: Vehicle,
    catalog: DefinitionCatalog,
    body_index: int,
    voxels: Optional[Tuple[WorldVoxel, ...]] = None,
) -> RuntimeF2VoxelModel:
    """Return the unmodified physics voxels used by the F2 builder."""

    source_voxels = (
        tuple(voxels)
        if voxels is not None
        else vehicle.physics_voxels(catalog, body_index)
    )
    # Keep these parameters in the stable call contract.  They also make it
    # explicit that a caller-provided Flooder-expanded tuple wins over a fresh
    # static expansion.
    _ = catalog, body_index
    return RuntimeF2VoxelModel(source_voxels, ())
