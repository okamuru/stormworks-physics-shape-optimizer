from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from swphysics.app_service import analyze_vehicle
from swphysics.definitions import DefinitionCatalog
from swphysics.vehicle import load_vehicle


_CUBE_ORIENTATIONS = tuple(range(6))


def _point_attributes(position):
    return {
        axis: str(value)
        for axis, value in zip(("x", "y", "z"), position)
        if value
    }


def _add_position(parent: ET.Element, position) -> None:
    ET.SubElement(parent, "position", _point_attributes(position))


def _add_cube_surfaces(parent: ET.Element, positions) -> None:
    for position in positions:
        for orientation in _CUBE_ORIENTATIONS:
            surface = ET.SubElement(
                parent,
                "surface",
                {
                    "orientation": str(orientation),
                    "rotation": "0",
                    "shape": "1",
                    "trans_type": "0",
                },
            )
            _add_position(surface, position)


def _write_definition_catalog(definitions: Path) -> None:
    block = ET.Element("definition", {"name": "Block"})
    _add_cube_surfaces(ET.SubElement(block, "buoyancy_surfaces"), ((0, 0, 0),))
    block_voxel = ET.SubElement(ET.SubElement(block, "voxels"), "voxel", {"flags": "1"})
    _add_position(block_voxel, (0, 0, 0))
    ET.ElementTree(block).write(
        definitions / "01_block.xml", encoding="utf-8", xml_declaration=True
    )

    flooder = ET.Element(
        "definition", {"name": "Physics Flooder", "water_component_type": "19"}
    )
    _add_cube_surfaces(
        ET.SubElement(flooder, "buoyancy_surfaces"), ((0, 0, 0),)
    )
    flooder_voxels = ET.SubElement(flooder, "voxels")
    flooder_physics = ET.SubElement(flooder_voxels, "voxel", {"flags": "1"})
    _add_position(flooder_physics, (0, 0, 0))
    flooder_sample = ET.SubElement(flooder_voxels, "voxel", {"flags": "0"})
    _add_position(flooder_sample, (0, 1, 0))
    ET.SubElement(
        flooder,
        "compartment_sample_pos",
        _point_attributes((0, 1, 0)),
    )
    ET.ElementTree(flooder).write(
        definitions / "physics_flooder.xml",
        encoding="utf-8",
        xml_declaration=True,
    )

    frame_positions = tuple((x, -2, 0) for x in range(-3, 4))
    panel_positions = tuple(
        (x, y, 0)
        for x in range(-3, 4)
        for y in range(-1, 2)
    )
    door = ET.Element(
        "definition",
        {
            "name": "Large Hinged Door Fixture",
            "type": "50",
            "door_side_dist": "6",
            "door_up_dist": "2",
        },
    )
    _add_cube_surfaces(
        ET.SubElement(door, "buoyancy_surfaces"), frame_positions
    )
    door_voxels = ET.SubElement(door, "voxels")
    for position in frame_positions:
        voxel = ET.SubElement(door_voxels, "voxel", {"flags": "1"})
        _add_position(voxel, position)
    for position in panel_positions:
        voxel = ET.SubElement(door_voxels, "voxel", {"flags": "4"})
        _add_position(voxel, position)
    for name, position in (
        ("door_normal", (0, 0, 1)),
        ("door_side", (1, 0, 0)),
        ("door_up", (0, 1, 0)),
        ("door_base_pos", (-3, -1, 0)),
    ):
        ET.SubElement(door, name, _point_attributes(position))
    ET.ElementTree(door).write(
        definitions / "door_manual_large.xml",
        encoding="utf-8",
        xml_declaration=True,
    )

    custom_door = ET.Element(
        "definition",
        {
            "name": "Custom Door Fixture",
            "type": "30",
            "custom_door_type": "0",
        },
    )
    ET.SubElement(custom_door, "voxels")
    ET.ElementTree(custom_door).write(
        definitions / "custom_door_fixture.xml",
        encoding="utf-8",
        xml_declaration=True,
    )


def _add_component(
    components: ET.Element,
    definition_id: str,
    position,
    rotation: str = "1,0,0,0,1,0,0,0,1",
    scene_id: str = "6",
) -> None:
    component = ET.SubElement(components, "c", {"d": definition_id})
    obj = ET.SubElement(component, "o", {"r": rotation, "sc": scene_id})
    ET.SubElement(
        obj,
        "vp",
        {
            axis: str(value)
            for axis, value in zip(("x", "y", "z"), position)
            if value
        },
    )


def _write_test_door_equivalent(path: Path) -> None:
    """Rebuild the user's observed `test door` geometry reproducibly."""

    vehicle = ET.Element("vehicle", {"data_version": "3", "bodies_id": "3"})
    bodies = ET.SubElement(vehicle, "bodies")
    body = ET.SubElement(bodies, "body", {"unique_id": "3"})
    components = ET.SubElement(body, "components")

    door_panel = {
        (x, y, 0)
        for x in range(-1, 2)
        for y in range(1, 8)
    }
    door_frame = {(2, y, 0) for y in range(1, 8)}
    flooder_position = (0, 0, 1)
    replaced_positions = door_panel | door_frame | {flooder_position}

    for x in range(-4, 5):
        for y in range(9):
            for z in range(8):
                position = (x, y, z)
                is_boundary = (
                    abs(x) == 4 or y in (0, 8) or z in (0, 7)
                )
                if is_boundary and position not in replaced_positions:
                    _add_component(components, "01_block", position)

    _add_component(
        components,
        "door_manual_large",
        (0, 4, 0),
        rotation="0,-1,0,-1,0,0,0,0,-1",
        scene_id="34",
    )
    _add_component(
        components,
        "physics_flooder",
        flooder_position,
        scene_id="5",
    )
    ET.ElementTree(vehicle).write(path, encoding="utf-8", xml_declaration=True)


class StandardDoorFlooderGroundTruthTests(unittest.TestCase):
    def test_large_hinged_door_fills_test_door_as_one_shape(self):
        """Match the game's F2 result observed for the user's `test door`."""

        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            definitions = temporary / "definitions"
            definitions.mkdir()
            _write_definition_catalog(definitions)
            source = temporary / "test door equivalent.xml"
            _write_test_door_equivalent(source)
            vehicle = load_vehicle(source)
            catalog = DefinitionCatalog.for_vehicle(definitions, source)
            static_voxels = vehicle.physics_voxels(catalog, 0)
            analysis = analyze_vehicle(
                source,
                definitions,
                max_evaluations=1,
                worker_count=1,
            )

        body = analysis.bodies[0]
        definition_ids = tuple(
            component.definition_id
            for component in vehicle.bodies[0].components
        )
        self.assertEqual(325, definition_ids.count("01_block"))
        self.assertEqual(1, definition_ids.count("door_manual_large"))
        self.assertEqual(1, definition_ids.count("physics_flooder"))
        self.assertEqual(327, body.component_count)
        self.assertEqual(333, len(static_voxels))
        self.assertEqual(1, body.physics_flooder_component_count)
        self.assertEqual(315, body.generated_fill_voxel_count)
        self.assertEqual(648, body.physics_voxel_count)
        self.assertEqual(1, body.current_shape_count)
        self.assertEqual(1, body.optimized_shape_count)
        self.assertEqual(
            ((-4, 0, 0), (4, 8, 7)),
            (
                body.current_boxes[0].minimum,
                body.current_boxes[0].maximum,
            ),
        )

    def test_custom_door_part_does_not_exclude_the_whole_flooder_body(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            definitions = temporary / "definitions"
            definitions.mkdir()
            _write_definition_catalog(definitions)
            source = temporary / "test door with custom marker.xml"
            _write_test_door_equivalent(source)
            tree = ET.parse(source)
            components = tree.getroot().find("./bodies/body/components")
            self.assertIsNotNone(components)
            _add_component(
                components,
                "custom_door_fixture",
                (20, 0, 0),
            )
            tree.write(source, encoding="utf-8", xml_declaration=True)

            analysis = analyze_vehicle(
                source,
                definitions,
                max_evaluations=1,
                worker_count=1,
            )

        body = analysis.bodies[0]
        self.assertFalse(body.flooder_prediction_excluded)
        self.assertEqual(315, body.generated_fill_voxel_count)
        self.assertEqual(1, body.current_shape_count)


if __name__ == "__main__":
    unittest.main()
