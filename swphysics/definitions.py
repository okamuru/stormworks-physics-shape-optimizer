from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple
import re
import xml.etree.ElementTree as ET

from .model import (
    DefinitionVoxel,
    IDENTITY_MATRIX,
    Matrix3,
    parse_fixed_point_int,
    point_from_attributes,
)


# The sixteen basic building component IDs are kept as a convenient inventory
# list for validation tools.  They are *not* encoded by vehicle ``c/@t``.
# Build 24749959 parses ``t`` as the Definition transform (mirror) index.
BASIC_DEFINITION_IDS: Tuple[str, ...] = (
    "01_block",
    "02_wedge",
    "03_pyramid",
    "04_invpyramid",
    "05_wedge_2",
    "06_pyramid_2",
    "07_invpyramid_2",
    "08_wedge_4",
    "09_pyramid_4",
    "10_invpyramid_4",
    "11_pyramid_2x2",
    "12_pyramid_2x4",
    "13_pyramid_4x4",
    "14_invpyramid_2x2",
    "15_invpyramid_2x4",
    "16_invpyramid_4x4",
)


def compact_definition_id(component_attributes: dict) -> str:
    """Return the component Definition ID for modern and legacy saves.

    Modern data-version 3 vehicles use ``d`` and default an omitted value to
    ``01_block``.  Older verbose saves use ``definition``.  ``t`` and
    ``trans_index`` select one of eight mirrored Definition variants and must
    therefore never be interpreted as a basic-block type.
    """

    return (
        component_attributes.get("d")
        or component_attributes.get("definition")
        or "01_block"
    )


def _parse_physics_rotation(voxel: ET.Element) -> Matrix3:
    rotation = voxel.find("physics_shape_rotation")
    if rotation is None:
        return IDENTITY_MATRIX
    # matrix33_s32's XML parser stores the attributes consecutively in this
    # exact order.  Keep that native memory layout here: ``apply_matrix`` and
    # the build-pinned constructor both consume the same serialized tuple.
    keys = ("00", "01", "02", "10", "11", "12", "20", "21", "22")
    # Stormworks writes attributes such as 00="1". Names beginning with a
    # digit are not legal XML, so load() prefixes them with m before parsing.
    return tuple(parse_fixed_point_int(rotation.attrib.get("m" + key, "1" if key in ("00", "11", "22") else "0")) for key in keys)  # type: ignore[return-value]


def _parse_stormworks_definition_text(text: str) -> ET.Element:
    sanitized = re.sub(r"(\s)([0-9][0-9])=", r"\1m\2=", text)
    return ET.fromstring(sanitized)


def _parse_stormworks_definition(path: Path) -> ET.Element:
    return _parse_stormworks_definition_text(path.read_text(encoding="utf-8"))


def _read_component_bin(path: Path) -> Tuple[str, ET.Element]:
    """Extract the definition id and embedded XML from a vehicle-package BIN."""

    raw = path.read_bytes()
    if len(raw) < 16:
        raise ValueError("component BIN is too short: {}".format(path))
    xml_start = raw.find(b"<?xml", 8)
    xml_end_marker = b"</definition>"
    xml_end = raw.find(xml_end_marker, xml_start)
    if xml_start < 0:
        # Older component packages place XML directly after the size word.
        xml_start = raw.find(b"<?xml", 4)
    if xml_start < 0 or xml_end < 0:
        xml_end = raw.find(xml_end_marker, xml_start)
    if xml_start < 0 or xml_end < 0:
        raise ValueError("component BIN has no embedded definition XML: {}".format(path))
    identifier_end = raw.find(b"\0", 8, xml_start)
    if xml_start == 4 or identifier_end < 9:
        definition_id = path.stem
    else:
        definition_id = raw[8:identifier_end].decode("utf-8")
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", definition_id):
        raise ValueError(
            "component BIN has an invalid definition id {!r}: {}".format(
                definition_id, path
            )
        )
    xml_end += len(xml_end_marker)
    text = raw[xml_start:xml_end].decode("utf-8")
    return definition_id, _parse_stormworks_definition_text(text)


@dataclass(frozen=True)
class DefinitionSurface:
    position: Tuple[int, int, int]
    orientation: int
    rotation: int
    shape: int
    transmission_type: int
    flags: int


@dataclass(frozen=True)
class ComponentDefinition:
    definition_id: str
    name: str
    flags: int
    component_type: int
    water_component_type: int
    custom_door_type: int
    constraint_type: int
    mesh_data_name: str
    source_path: Path
    voxels: Tuple[DefinitionVoxel, ...]
    surfaces: Tuple[DefinitionSurface, ...]
    buoyancy_surfaces: Tuple[DefinitionSurface, ...]
    compartment_sample_position: Tuple[int, int, int]
    source_format: str = "xml"

    @property
    def contributes_physics_extra_box(self) -> bool:
        # build_physics_begin tests definition flags bit 0x400 and requires
        # the loaded mesh-data pointer before pushing one physics_extra_box.
        return bool(self.flags & 0x400) and bool(self.mesh_data_name)


class DefinitionCatalog:
    def __init__(
        self,
        root: Path,
        component_package_roots: Iterable[Path] = (),
    ):
        self.root = Path(root)
        self.component_package_roots = tuple(
            Path(package_root) for package_root in component_package_roots
        )
        self._cache: Dict[str, ComponentDefinition] = {}
        self._component_bin_index: Optional[Dict[str, Path]] = None

    @property
    def loaded_definitions(self) -> Tuple[ComponentDefinition, ...]:
        """Definitions actually requested from this catalog, without aliases."""

        unique = {definition.source_path: definition for definition in self._cache.values()}
        return tuple(unique[path] for path in sorted(unique))

    @property
    def packaged_definition_ids(self) -> Tuple[str, ...]:
        """Embedded Definition ids available from configured component BINs."""

        identifiers = set()
        for package_root in self.component_package_roots:
            if not package_root.is_dir():
                continue
            for path in sorted(package_root.glob("*.bin")):
                if path.name.startswith("._"):
                    continue
                try:
                    definition_id, _root = _read_component_bin(path)
                except (UnicodeDecodeError, ValueError, ET.ParseError):
                    continue
                identifiers.add(definition_id)
        return tuple(sorted(identifiers))

    @classmethod
    def for_vehicle(
        cls,
        root: Path,
        vehicle_path: Path,
        additional_package_roots: Iterable[Path] = (),
    ) -> "DefinitionCatalog":
        vehicle = Path(vehicle_path)
        sibling_package = vehicle.with_suffix("")
        package_roots = []
        if sibling_package.is_dir():
            package_roots.append(sibling_package)
        package_roots.extend(Path(path) for path in additional_package_roots)
        return cls(root, package_roots)

    def _index_component_bins(self) -> Dict[str, Path]:
        if self._component_bin_index is not None:
            return self._component_bin_index
        index: Dict[str, Path] = {}
        for package_root in self.component_package_roots:
            if not package_root.is_dir():
                continue
            for path in sorted(package_root.glob("*.bin")):
                if path.name.startswith("._"):
                    continue
                try:
                    definition_id, _root = _read_component_bin(path)
                except (UnicodeDecodeError, ValueError, ET.ParseError):
                    continue
                index.setdefault(definition_id, path)
                index.setdefault(path.stem, path)
        self._component_bin_index = index
        return index

    def _definition_source(self, definition_id: str) -> Tuple[Path, ET.Element, str]:
        path = self.root / (definition_id + ".xml")
        if path.is_file():
            return path, _parse_stormworks_definition(path), "xml"
        component_bin = self._index_component_bins().get(definition_id)
        if component_bin is None:
            searched = ", ".join(str(path) for path in self.component_package_roots)
            package_note = " package roots [{}]".format(searched) if searched else ""
            raise FileNotFoundError(
                "component definition not found: {}{}".format(path, package_note)
            )
        _embedded_id, root = _read_component_bin(component_bin)
        return component_bin, root, "vehicle_component_bin"

    def load(self, definition_id: str) -> ComponentDefinition:
        if definition_id in self._cache:
            return self._cache[definition_id]
        path, root, source_format = self._definition_source(definition_id)
        voxels = []
        voxels_element = root.find("voxels")
        if voxels_element is not None:
            for voxel in voxels_element.findall("voxel"):
                position_element = voxel.find("position")
                position = point_from_attributes(position_element.attrib if position_element is not None else {})
                voxels.append(
                    DefinitionVoxel(
                        position=position,
                        flags=int(voxel.attrib.get("flags", "0")),
                        physics_shape=int(voxel.attrib.get("physics_shape", "0")),
                        physics_rotation=_parse_physics_rotation(voxel),
                    )
                )
        def parse_surfaces(tag: str) -> Tuple[DefinitionSurface, ...]:
            parsed = []
            surfaces_element = root.find(tag)
            if surfaces_element is None:
                return ()
            for surface in surfaces_element.findall("surface"):
                position_element = surface.find("position")
                parsed.append(
                    DefinitionSurface(
                        position=point_from_attributes(
                            position_element.attrib
                            if position_element is not None
                            else {}
                        ),
                        orientation=int(surface.attrib.get("orientation", "0")),
                        rotation=int(surface.attrib.get("rotation", "0")),
                        shape=int(surface.attrib.get("shape", "0")),
                        transmission_type=int(surface.attrib.get("trans_type", "0")),
                        flags=int(surface.attrib.get("flags", "0")),
                    )
                )
            return tuple(parsed)

        surfaces = parse_surfaces("surfaces")
        buoyancy_surfaces = parse_surfaces("buoyancy_surfaces")
        compartment_sample_element = root.find("compartment_sample_pos")
        compartment_sample_position = point_from_attributes(
            compartment_sample_element.attrib
            if compartment_sample_element is not None
            else {}
        )
        definition = ComponentDefinition(
            definition_id=definition_id,
            name=root.attrib.get("name", definition_id),
            flags=int(root.attrib.get("flags", "0")),
            component_type=int(root.attrib.get("type", "0")),
            water_component_type=int(root.attrib.get("water_component_type", "0")),
            custom_door_type=int(root.attrib.get("custom_door_type", "0")),
            constraint_type=int(root.attrib.get("constraint_type", "0")),
            mesh_data_name=root.attrib.get("mesh_data_name", ""),
            source_path=path,
            voxels=tuple(voxels),
            surfaces=surfaces,
            buoyancy_surfaces=buoyancy_surfaces,
            compartment_sample_position=compartment_sample_position,
            source_format=source_format,
        )
        self._cache[definition_id] = definition
        return definition
