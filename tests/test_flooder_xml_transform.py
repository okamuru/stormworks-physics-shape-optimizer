from pathlib import Path
import shutil
import tempfile
import unittest

from swphysics.app_service import analyze_vehicle
from swphysics.definitions import DefinitionSurface
from swphysics.surface_graph import SurfaceMetadata


ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests" / "fixtures"


class FlooderXmlTransformTests(unittest.TestCase):
    def test_integer_matrix_fallback_matches_every_extracted_grid_result(self):
        metadata = SurfaceMetadata()
        for (rotation, shape, direction), expected in metadata.resolutions.items():
            self.assertEqual(
                expected,
                metadata._fallback_resolution(rotation, shape, direction),
                (rotation, shape, direction),
            )

    def test_non_grid_surface_goldens_match_build_24749959(self):
        metadata = SurfaceMetadata()

        cases = (
            (
                "scaled quarter face",
                (2, 0, 0, 0, 2, 0, 0, 0, 2),
                DefinitionSurface((0, 0, 0), 4, 0, 2, 0, 0),
                (2, 28, 10),
            ),
            (
                "axis reflection",
                (-2, 0, 0, 0, 3, 0, 0, 0, 4),
                DefinitionSurface((0, 0, 0), 0, 0, 1, 0, 0),
                (1, 1, 0),
            ),
            (
                "shear",
                (1, 0, 0, 1, 1, 0, 0, 0, 1),
                DefinitionSurface((0, 0, 0), 2, 0, 1, 0, 0),
                (1, 0, 0),
            ),
            (
                "singular active axis",
                (1, 0, 0, 0, 0, 0, 0, 0, 1),
                DefinitionSurface((0, 0, 0), 4, 0, 1, 0, 0),
                (1, 4, 0),
            ),
            (
                "singular collapsed axis",
                (1, 0, 0, 0, 0, 0, 0, 0, 1),
                DefinitionSurface((0, 0, 0), 2, 0, 1, 0, 0),
                (0, 0, 0),
            ),
            (
                "all zero",
                (0, 0, 0, 0, 0, 0, 0, 0, 0),
                DefinitionSurface((0, 0, 0), 0, 0, 1, 0, 0),
                (0, 0, 0),
            ),
        )
        for label, transform, surface, expected in cases:
            with self.subTest(label):
                resolution = metadata.lookup(transform, surface)
                self.assertIsNotNone(resolution)
                self.assertEqual(
                    expected,
                    (
                        resolution.type_count,
                        resolution.primary,
                        resolution.secondary,
                    ),
                )

    def test_known_non_grid_surfaces_do_not_exclude_flooder_body(self):
        transforms = (
            ("scale", "2,0,0,0,3,0,0,0,4"),
            ("reflection", "-2,0,0,0,3,0,0,0,4"),
            ("shear", "1,0,0,1,1,0,0,0,1"),
            ("singular", "1,0,0,0,0,0,0,0,1"),
            ("all_zero", "0,0,0,0,0,0,0,0,0"),
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            definitions = temporary / "definitions"
            definitions.mkdir()
            shutil.copy(
                FIXTURES / "definitions" / "01_block.xml",
                definitions / "01_block.xml",
            )
            (definitions / "physics_flooder.xml").write_text(
                """<?xml version="1.0" encoding="UTF-8"?>
<definition name="Physics Flooder" water_component_type="19"><voxels/></definition>
""",
                encoding="utf-8",
            )
            (definitions / "known_surface.xml").write_text(
                """<?xml version="1.0" encoding="UTF-8"?>
<definition name="Known Surface">
  <voxels><voxel flags="1" physics_shape="0"><position/></voxel></voxels>
  <buoyancy_surfaces>
    <surface orientation="0" rotation="0" shape="1" flags="1"><position/></surface>
    <surface orientation="1" rotation="0" shape="1" flags="1"><position/></surface>
    <surface orientation="2" rotation="0" shape="1" flags="1"><position/></surface>
    <surface orientation="3" rotation="0" shape="1" flags="1"><position/></surface>
    <surface orientation="4" rotation="0" shape="1" flags="1"><position/></surface>
    <surface orientation="5" rotation="0" shape="1" flags="1"><position/></surface>
  </buoyancy_surfaces>
</definition>
""",
                encoding="utf-8",
            )

            for label, transform in transforms:
                with self.subTest(label):
                    source = temporary / (label + ".xml")
                    source.write_text(
                        '<vehicle data_version="3"><bodies><body><components>'
                        '<c d="physics_flooder"><o><vp/></o></c>'
                        '<c d="known_surface"><o r="{}"><vp x="50"/></o></c>'
                        '</components></body></bodies></vehicle>'.format(transform),
                        encoding="utf-8",
                    )
                    analysis = analyze_vehicle(
                        source,
                        definitions,
                        max_evaluations=1,
                        worker_count=1,
                    )
                    self.assertEqual(
                        0, analysis.flooder_prediction_excluded_body_count
                    )
                    self.assertFalse(
                        analysis.bodies[0].flooder_prediction_excluded
                    )


if __name__ == "__main__":
    unittest.main()
