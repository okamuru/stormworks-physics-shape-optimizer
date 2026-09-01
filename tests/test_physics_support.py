from pathlib import Path
import tempfile
import unittest

from swphysics.definitions import DefinitionCatalog
from swphysics.physics_support import (
    AXIS_SCALE_TRANSFORM,
    GENERAL_NON_AXIS_TRANSFORM,
    GRID_TRANSFORM,
    SINGULAR_TRANSFORM,
    classify_component_transform,
    classify_component_physics_support,
)
from swphysics.vehicle import load_vehicle
from swphysics.xml_edit_audit import audit_vehicles, workshop_vehicle_paths


FIXTURES = Path(__file__).parent / "fixtures"


class PhysicsSupportTests(unittest.TestCase):
    def test_workshop_inventory_ignores_non_vehicle_xml_payloads(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            item = root / "123456"
            item.mkdir()
            vehicle = item / "vehicle.xml"
            vehicle.write_text("<vehicle/>", encoding="utf-8")
            (item / "microcontroller.xml").write_text(
                "<microprocessor/>", encoding="utf-8"
            )

            self.assertEqual((vehicle.resolve(),), workshop_vehicle_paths((root,)))

    def test_component_transform_classification(self):
        self.assertEqual(
            GRID_TRANSFORM,
            classify_component_transform((0, 1, 0, -1, 0, 0, 0, 0, 1)),
        )
        self.assertEqual(
            AXIS_SCALE_TRANSFORM,
            classify_component_transform((0, 3, 0, -2, 0, 0, 0, 0, 5)),
        )
        self.assertEqual(
            GENERAL_NON_AXIS_TRANSFORM,
            classify_component_transform((1, 1, 0, 0, 1, 0, 0, 0, 1)),
        )
        self.assertEqual(
            SINGULAR_TRANSFORM,
            classify_component_transform((1, 1, 0, 0, 0, 0, 0, 0, 1)),
        )

    def _classify(self, component_xml: str):
        with tempfile.TemporaryDirectory() as temporary_directory:
            vehicle_path = Path(temporary_directory) / "vehicle.xml"
            vehicle_path.write_text(
                '<vehicle data_version="3"><bodies><body><components>{}'
                '</components></body></bodies></vehicle>'.format(component_xml),
                encoding="utf-8",
            )
            vehicle = load_vehicle(vehicle_path)
            catalog = DefinitionCatalog(FIXTURES / "definitions")
            return classify_component_physics_support(
                vehicle.bodies[0].components,
                vehicle.physics_voxels(catalog, 0),
            )

    def test_grid_wedge_is_supported(self):
        result = self._classify('<c d="02_wedge" t="2"><o><vp/></o></c>')

        self.assertEqual(1, len(result))
        self.assertTrue(result[0].supported)
        self.assertEqual((), result[0].issue_codes)

    def test_stretched_cube_remains_supported(self):
        result = self._classify(
            '<c><o r="2,0,0,0,1,0,0,0,1"><vp/></o></c>'
        )

        self.assertTrue(result[0].supported)

    def test_axis_scaled_wedge_is_supported(self):
        result = self._classify(
            '<c d="02_wedge"><o r="2,0,0,0,1,0,0,0,1"><vp/></o></c>'
        )

        self.assertTrue(result[0].supported)
        self.assertEqual(AXIS_SCALE_TRANSFORM, result[0].transform_kind)
        self.assertEqual((), result[0].issue_codes)

    def test_sheared_wedge_is_supported(self):
        result = self._classify(
            '<c d="02_wedge"><o r="1,1,0,0,1,0,0,0,1"><vp/></o></c>'
        )

        self.assertTrue(result[0].supported)
        self.assertEqual(GENERAL_NON_AXIS_TRANSFORM, result[0].transform_kind)
        self.assertEqual((), result[0].issue_codes)

    def test_singular_wedge_is_supported(self):
        result = self._classify(
            '<c d="02_wedge"><o r="0,0,0,0,0,0,-1,0,0"><vp/></o></c>'
        )

        self.assertTrue(result[0].supported)
        self.assertEqual(SINGULAR_TRANSFORM, result[0].transform_kind)
        self.assertEqual((), result[0].issue_codes)

    def test_audit_keeps_supported_components_when_one_definition_is_missing(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            vehicle_path = temporary / "mixed.xml"
            vehicle_path.write_text(
                '''<vehicle data_version="3"><bodies><body><components>
<c><o><vp/></o></c>
<c d="missing_mod"><o><vp x="1"/></o></c>
<c d="02_wedge"><o r="2,0,0,0,1,0,0,0,1"><vp x="2"/></o></c>
</components></body></bodies></vehicle>''',
                encoding="utf-8",
            )

            report = audit_vehicles(
                FIXTURES / "definitions", (vehicle_path,), detail_limit=10
            )

        self.assertEqual(1, report["totals"]["parsed_vehicle_count"])
        self.assertEqual(2, report["totals"]["supported_component_count"])
        self.assertNotIn("unsupported_component_count", report["totals"])
        self.assertEqual(
            1, report["issue_counts"]["definition_or_expansion_error"]
        )


if __name__ == "__main__":
    unittest.main()
