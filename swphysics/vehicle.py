from dataclasses import dataclass
from pathlib import Path
import re
from typing import Iterable, List, Tuple
import xml.etree.ElementTree as ET

from .definitions import (
    ComponentDefinition,
    DefinitionCatalog,
    DefinitionSurface,
    compact_definition_id,
)
from .model import (
    DEFAULT_COMPONENT_ROTATION,
    DefinitionVoxel,
    GridPoint,
    IDENTITY_MATRIX,
    Matrix3,
    WorldVoxel,
    add_points,
    apply_matrix,
    multiply_matrices,
    parse_fixed_point_int,
    parse_matrix,
    point_from_attributes,
    transform_index_matrix,
)


_NUMERIC_ATTRIBUTE_RE = re.compile(r"(\s)([0-9][0-9])=")
_SANITIZED_NUMERIC_ATTRIBUTE_RE = re.compile(r"m[0-9][0-9]")
MAX_MICROPROCESSOR_WIDTH = 32
MAX_MICROPROCESSOR_LENGTH = 32


def _validated_microprocessor_cell_count(width: int, length: int) -> int:
    """Bound XML-edited dimensions before expanding dynamic Definition data."""

    if width < 1 or length < 1:
        raise ValueError(
            "microprocessor dimensions must be positive; got {}x{}".format(
                width, length
            )
        )
    if (
        width > MAX_MICROPROCESSOR_WIDTH
        or length > MAX_MICROPROCESSOR_LENGTH
    ):
        raise ValueError(
            "microprocessor dimensions {}x{} exceed the native {}x{} limit"
            .format(
                width,
                length,
                MAX_MICROPROCESSOR_WIDTH,
                MAX_MICROPROCESSOR_LENGTH,
            )
        )
    return width * length


def parse_vehicle_tree(path: Path, insert_comments: bool = False) -> ET.ElementTree:
    """Parse Stormworks XML, including its non-standard numeric attributes.

    Stormworks emits matrix attributes such as ``00=\"1\"``.  The game accepts
    them, but XML 1.0 does not permit an attribute name to begin with a digit,
    so ElementTree rejects otherwise valid vehicle saves.  Prefixing those
    names in memory lets us inspect and reorder the file without touching any
    values.
    """

    source = Path(path)
    # Avoid the lazily imported ``utf-8-sig`` codec here.  In a frozen GUI
    # build the first lookup can happen on a QThreadPool thread and fail even
    # when encodings/utf_8_sig.pyc is present in base_library.zip.  Stormworks
    # saves are UTF-8; stripping the optional three-byte BOM explicitly keeps
    # the same parsing contract without a dynamic codec lookup.
    encoded = source.read_bytes()
    if encoded.startswith(b"\xef\xbb\xbf"):
        encoded = encoded[3:]
    text = encoded.decode("utf-8")
    sanitized = _NUMERIC_ATTRIBUTE_RE.sub(r"\1m\2=", text)
    builder = ET.TreeBuilder(insert_comments=insert_comments)
    parser = ET.XMLParser(target=builder)
    root = ET.fromstring(sanitized, parser=parser)
    return ET.ElementTree(root)


def restore_numeric_attribute_names(tree: ET.ElementTree) -> None:
    """Restore the numeric matrix names before serializing a game vehicle."""

    for element in tree.iter():
        replacements = [
            (name, name[1:], value)
            for name, value in element.attrib.items()
            if _SANITIZED_NUMERIC_ATTRIBUTE_RE.fullmatch(name)
        ]
        for old_name, new_name, value in replacements:
            del element.attrib[old_name]
            element.attrib[new_name] = value


@dataclass(frozen=True)
class ComponentPlacement:
    index: int
    definition_id: str
    transform_index: int
    position: GridPoint
    rotation: Matrix3
    microprocessor_width: int = 1
    microprocessor_length: int = 1

    @property
    def effective_transform(self) -> Matrix3:
        """Component rotation composed with the mirrored Definition variant."""

        return multiply_matrices(
            self.rotation, transform_index_matrix(self.transform_index)
        )

    def physics_definition_voxels(
        self, definition: ComponentDefinition
    ) -> Iterable[DefinitionVoxel]:
        """Return the placement-specific physics voxels used by the game.

        ``microprocessor.xml`` contains only a one-voxel placeholder.  The
        native microprocessor component instead rebuilds its Definition data
        from the saved ``width`` and ``length`` attributes, iterating local X
        first and local Z second.  Keeping that order is important because the
        physics merger consumes Definition voxels in insertion order.
        """

        if definition.component_type != 37:
            return definition.voxels
        _validated_microprocessor_cell_count(
            self.microprocessor_width, self.microprocessor_length
        )
        return (
            DefinitionVoxel(
                position=(x, 0, z),
                flags=1,
                physics_shape=0,
                physics_rotation=IDENTITY_MATRIX,
            )
            for x in range(self.microprocessor_width)
            for z in range(self.microprocessor_length)
        )

    def world_physics_voxels(
        self,
        definition: ComponentDefinition,
        body_index: int,
        body_id: str,
        insertion_index_start: int = 0,
    ) -> Tuple[WorldVoxel, ...]:
        """Expand this placement without requiring every Body Definition.

        The per-Component entry point is useful to coverage audits: an unknown
        Component MOD can be reported and skipped without hiding the supported
        placements in the rest of the Body.
        """

        result = []
        effective_transform = self.effective_transform
        insertion_index = insertion_index_start
        for definition_voxel_index, voxel in enumerate(
            self.physics_definition_voxels(definition)
        ):
            if not voxel.contributes_physics:
                continue
            result.append(
                WorldVoxel(
                    body_index=body_index,
                    body_id=body_id,
                    component_index=self.index,
                    component_definition=self.definition_id,
                    definition_voxel_index=definition_voxel_index,
                    insertion_index=insertion_index,
                    position=add_points(
                        self.position,
                        apply_matrix(effective_transform, voxel.position),
                    ),
                    physics_shape=voxel.physics_shape,
                    physics_rotation=multiply_matrices(
                        effective_transform, voxel.physics_rotation
                    ),
                )
            )
            insertion_index += 1
        return tuple(result)

    def buoyancy_definition_surfaces(
        self, definition: ComponentDefinition
    ) -> Iterable[DefinitionSurface]:
        """Return placement-specific surfaces used for sealed-volume tests.

        Native ``c_microprocessor_definition::build_surfaces`` adds one
        buoyancy surface for every footprint cell, but only on the local
        bottom face.  The top and four sides are attachment/render surfaces,
        not watertight barriers.
        """

        if definition.component_type != 37:
            return definition.buoyancy_surfaces
        _validated_microprocessor_cell_count(
            self.microprocessor_width, self.microprocessor_length
        )
        return (
            DefinitionSurface(
                position=(x, 0, z),
                orientation=3,
                rotation=0,
                shape=1,
                transmission_type=0,
                flags=1,
            )
            for x in range(self.microprocessor_width)
            for z in range(self.microprocessor_length)
        )

    def buoyancy_surface_template_key(self) -> Tuple[object, ...]:
        """Identify placement data that changes generated surface geometry."""

        return (
            self.definition_id,
            self.effective_transform,
            self.microprocessor_width,
            self.microprocessor_length,
        )


@dataclass(frozen=True)
class VehicleBody:
    index: int
    body_id: str
    components: Tuple[ComponentPlacement, ...]


@dataclass(frozen=True)
class Vehicle:
    source_path: Path
    data_version: str
    bodies: Tuple[VehicleBody, ...]

    def physics_voxels(self, catalog: DefinitionCatalog, body_index: int) -> Tuple[WorldVoxel, ...]:
        body = self.bodies[body_index]
        result: List[WorldVoxel] = []
        insertion_index = 0
        for component in body.components:
            definition = catalog.load(component.definition_id)
            component_voxels = component.world_physics_voxels(
                definition,
                body.index,
                body.body_id,
                insertion_index,
            )
            result.extend(component_voxels)
            insertion_index += len(component_voxels)
        return tuple(result)


def load_vehicle(path: Path) -> Vehicle:
    source_path = Path(path)
    root = parse_vehicle_tree(source_path).getroot()
    data_version = root.attrib.get("data_version", "")
    # Legacy version 0 takes the typed matrix-parser path, whose omitted value
    # is identity.  Current versions take the compact string-parser path and
    # use the editor's native component orientation below.
    default_component_rotation = (
        IDENTITY_MATRIX
        if parse_fixed_point_int(data_version) == 0
        else DEFAULT_COMPONENT_ROTATION
    )
    bodies = []
    for body_index, body_element in enumerate(root.findall("./bodies/body")):
        placements = []
        components = body_element.find("components")
        if components is not None:
            for component_index, component in enumerate(components.findall("c")):
                object_element = component.find("o")
                object_attributes = object_element.attrib if object_element is not None else {}
                position_element = object_element.find("vp") if object_element is not None else None
                position = point_from_attributes(position_element.attrib if position_element is not None else {})
                rotation = (
                    parse_matrix(object_attributes["r"])
                    if "r" in object_attributes
                    else default_component_rotation
                )
                microprocessor_definition = (
                    object_element.find("microprocessor_definition")
                    if object_element is not None
                    else None
                )
                microprocessor_width = max(
                    1,
                    parse_fixed_point_int(
                        microprocessor_definition.attrib.get("width", "1")
                        if microprocessor_definition is not None
                        else "1"
                    ),
                )
                microprocessor_length = max(
                    1,
                    parse_fixed_point_int(
                        microprocessor_definition.attrib.get("length", "1")
                        if microprocessor_definition is not None
                        else "1"
                    ),
                )
                if microprocessor_definition is not None:
                    _validated_microprocessor_cell_count(
                        microprocessor_width, microprocessor_length
                    )
                placements.append(
                    ComponentPlacement(
                        index=component_index,
                        definition_id=compact_definition_id(component.attrib),
                        transform_index=int(
                            component.attrib.get(
                                "t", component.attrib.get("trans_index", "0")
                            )
                        ),
                        position=position,
                        rotation=rotation,
                        microprocessor_width=microprocessor_width,
                        microprocessor_length=microprocessor_length,
                    )
                )
        bodies.append(
            VehicleBody(
                index=body_index,
                body_id=body_element.attrib.get("unique_id", str(body_index)),
                components=tuple(placements),
            )
        )
    return Vehicle(
        source_path=source_path,
        data_version=data_version,
        bodies=tuple(bodies),
    )
