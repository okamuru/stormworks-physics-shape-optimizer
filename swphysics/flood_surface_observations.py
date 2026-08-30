"""Build-pinned qualitative observations for basic non-cube flood surfaces."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from .model import Matrix3


IDENTITY: Matrix3 = (1, 0, 0, 0, 1, 0, 0, 0, 1)
FLIP_X_180: Matrix3 = (1, 0, 0, 0, -1, 0, 0, 0, -1)


@dataclass(frozen=True)
class BasicRoofSurfaceObservation:
    definition_id: str
    identity: str
    flip_x_180: str


_ROWS = (
    ("02_wedge", "open", "open"),
    ("03_pyramid", "open", "open"),
    ("04_invpyramid", "filled_split_visible", "filled_split_visible"),
    ("05_wedge_2", "filled_split_visible", "filled"),
    ("06_pyramid_2", "open", "open"),
    ("07_invpyramid_2", "filled_split_visible", "filled"),
    ("08_wedge_4", "open", "filled"),
    ("09_pyramid_4", "open", "open"),
    ("10_invpyramid_4", "filled_split_visible", "filled"),
    ("11_pyramid_2x2", "open", "open"),
    ("12_pyramid_2x4", "open", "open"),
    ("13_pyramid_4x4", "open", "open"),
    ("14_invpyramid_2x2", "filled_split_visible", "filled"),
    ("15_invpyramid_2x4", "filled_split_visible", "filled"),
    ("16_invpyramid_4x4", "filled_split_visible", "filled"),
)

BASIC_ROOF_SURFACE_OBSERVATIONS: Dict[str, BasicRoofSurfaceObservation] = {
    definition_id: BasicRoofSurfaceObservation(definition_id, identity, flip)
    for definition_id, identity, flip in _ROWS
}


def classify_basic_roof_surface(
    definition_id: str, rotation: Matrix3
) -> Optional[str]:
    """Return the exact Stage 7 roof-probe observation, not a general model."""

    row = BASIC_ROOF_SURFACE_OBSERVATIONS.get(definition_id)
    if row is None:
        return None
    if rotation == IDENTITY:
        return row.identity
    if rotation == FLIP_X_180:
        return row.flip_x_180
    return None


def observed_sealed(value: Optional[str]) -> Optional[bool]:
    if value is None:
        return None
    return value.startswith("filled")
