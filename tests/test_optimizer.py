from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET

from swphysics.definitions import DefinitionCatalog
from swphysics.optimizer import (
    UnsupportedVehicleError,
    optimize_component_cube_order,
    optimize_vehicle_block_order,
)
from swphysics.vehicle import load_vehicle


ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests" / "fixtures"


def element_fingerprint(element):
    return (
        element.tag,
        tuple(element.attrib.items()),
        (element.text or "").strip(),
        tuple(element_fingerprint(child) for child in element),
    )


class OptimizerTests(unittest.TestCase):
    def setUp(self):
        self.catalog = DefinitionCatalog(FIXTURES / "definitions")

    def test_order_b_is_reduced_from_four_shapes_to_three(self):
        source = FIXTURES / "vehicles" / "order_b.xml"
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "optimized.xml"
            report = optimize_vehicle_block_order(source, output, self.catalog)
            self.assertEqual(4, report.before_shape_count)
            self.assertEqual(3, report.after_shape_count)
            self.assertTrue(report.bodies[0].result.changed)
            self.assertEqual(source.stat().st_size, output.stat().st_size)
            optimized = load_vehicle(output)
            self.assertEqual(
                sorted(component.position for component in load_vehicle(source).bodies[0].components),
                sorted(component.position for component in optimized.bodies[0].components),
            )

    def test_component_elements_are_preserved_semantically(self):
        source = FIXTURES / "vehicles" / "order_b.xml"
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "optimized.xml"
            optimize_vehicle_block_order(source, output, self.catalog)
            before_components = ET.parse(source).getroot().findall("./bodies/body/components/c")
            after_components = ET.parse(output).getroot().findall("./bodies/body/components/c")
            self.assertEqual(
                sorted(repr(element_fingerprint(element)) for element in before_components),
                sorted(repr(element_fingerprint(element)) for element in after_components),
            )

    def test_component_group_search_never_splits_internal_voxel_order(self):
        groups = (
            ((0, 1, 0), (1, 1, 0)),
            ((0, 0, 0), (1, 0, 0)),
            ((2, 1, 0), (2, 2, 0)),
        )
        result = optimize_component_cube_order(groups)
        self.assertEqual(3, result.before.shape_count)
        self.assertEqual(2, result.after.shape_count)
        self.assertEqual((1, 0, 2), result.optimized_component_order)
        expected = tuple(point for index in result.optimized_component_order for point in groups[index])
        self.assertEqual(expected, result.optimized_voxel_order)

    def test_multi_voxel_cube_vehicle_is_rejected_after_stage4(self):
        source = FIXTURES / "vehicles" / "multi_cube_order_bad.xml"
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "should-not-exist.xml"
            with self.assertRaisesRegex(
                UnsupportedVehicleError, "multi-voxel components require"
            ):
                optimize_vehicle_block_order(source, output, self.catalog)
            self.assertFalse(output.exists())

    def test_mixed_shape_vehicle_is_rejected_without_output(self):
        source = FIXTURES / "vehicles" / "rotation_and_shapes.xml"
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "should-not-exist.xml"
            with self.assertRaises(UnsupportedVehicleError):
                optimize_vehicle_block_order(source, output, self.catalog)
            self.assertFalse(output.exists())

    def test_component_without_physics_is_rejected(self):
        source = FIXTURES / "vehicles" / "no_physics_component.xml"
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "should-not-exist.xml"
            with self.assertRaisesRegex(UnsupportedVehicleError, "contributes no physics cubes"):
                optimize_vehicle_block_order(source, output, self.catalog)
            self.assertFalse(output.exists())

    def test_overlapping_component_voxels_are_rejected(self):
        source = FIXTURES / "vehicles" / "overlapping_cube_components.xml"
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "should-not-exist.xml"
            with self.assertRaisesRegex(UnsupportedVehicleError, "overlapping cube"):
                optimize_vehicle_block_order(source, output, self.catalog)
            self.assertFalse(output.exists())

    def test_stretched_component_transform_is_rejected(self):
        source = FIXTURES / "vehicles" / "stretched_cube_component.xml"
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "should-not-exist.xml"
            with self.assertRaisesRegex(UnsupportedVehicleError, "stretched or non-grid"):
                optimize_vehicle_block_order(source, output, self.catalog)
            self.assertFalse(output.exists())

    def test_input_cannot_be_overwritten(self):
        source = FIXTURES / "vehicles" / "order_b.xml"
        with self.assertRaises(ValueError):
            optimize_vehicle_block_order(source, source, self.catalog, force=True)

    def test_custom_component_bin_is_copied_beside_output_vehicle(self):
        definition_xml = b'''<?xml version="1.0" encoding="UTF-8"?>
<definition name="Packaged Block"><voxels>
<voxel flags="1" physics_shape="0"><position/></voxel>
</voxels></definition>'''
        definition_id = b"packaged_block"
        body = struct.pack("<I", 1) + definition_id + b"\0" + definition_xml
        payload = struct.pack("<I", len(body)) + body
        vehicle_text = '''<?xml version="1.0" encoding="UTF-8"?>
<vehicle data_version="3"><bodies><body unique_id="1"><components>
<c d="packaged_block"><o><vp/></o></c>
</components></body></bodies></vehicle>'''
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "Custom.xml"
            source.write_text(vehicle_text, encoding="utf-8")
            source_package = root / "Custom"
            source_package.mkdir()
            source_bin = source_package / "definition.bin"
            source_bin.write_bytes(payload)
            destination = root / "Optimized.xml"
            catalog = DefinitionCatalog.for_vehicle(
                FIXTURES / "definitions", source
            )
            result = optimize_vehicle_block_order(
                source, destination, catalog
            )
            copied = root / "Optimized" / "definition.bin"
            self.assertEqual(1, result.component_bin_count)
            self.assertEqual(copied, result.component_package_path / copied.name)
            self.assertEqual(payload, copied.read_bytes())
            reloaded = DefinitionCatalog.for_vehicle(
                FIXTURES / "definitions", destination
            ).load("packaged_block")
            self.assertEqual("vehicle_component_bin", reloaded.source_format)

    def test_optimizer_restores_numeric_matrix_attribute_names(self):
        text = '''<?xml version="1.0" encoding="UTF-8"?>
<vehicle data_version="3"><bodies><body unique_id="1">
<initial_local_transform 00="1" 11="1" 22="1" 33="1"/>
<components><c><o><vp/></o></c></components>
</body></bodies></vehicle>'''
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "Numeric.xml"
            destination = root / "Numeric Optimized.xml"
            source.write_text(text, encoding="utf-8")
            optimize_vehicle_block_order(source, destination, self.catalog)
            output = destination.read_text(encoding="utf-8")
            self.assertIn('00="1"', output)
            self.assertNotIn('m00="1"', output)
            self.assertEqual(1, len(load_vehicle(destination).bodies))


if __name__ == "__main__":
    unittest.main()
