import unittest

from swphysics.flood_surface_observations import (
    BASIC_ROOF_SURFACE_OBSERVATIONS,
    FLIP_X_180,
    IDENTITY,
    classify_basic_roof_surface,
    observed_sealed,
)


class FloodSurfaceObservationTests(unittest.TestCase):
    def test_all_fifteen_non_cube_basic_definitions_are_recorded(self):
        self.assertEqual(15, len(BASIC_ROOF_SURFACE_OBSERVATIONS))

    def test_observed_orientation_asymmetry_is_preserved(self):
        self.assertEqual(
            "open", classify_basic_roof_surface("08_wedge_4", IDENTITY)
        )
        self.assertEqual(
            "filled", classify_basic_roof_surface("08_wedge_4", FLIP_X_180)
        )
        self.assertFalse(observed_sealed("open"))
        self.assertTrue(observed_sealed("filled"))
        self.assertTrue(observed_sealed("filled_split_visible"))

    def test_unobserved_rotation_is_not_generalized(self):
        self.assertIsNone(
            classify_basic_roof_surface(
                "08_wedge_4", (0, 1, 0, -1, 0, 0, 0, 0, 1)
            )
        )


if __name__ == "__main__":
    unittest.main()
