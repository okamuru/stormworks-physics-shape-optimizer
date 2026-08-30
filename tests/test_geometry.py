from pathlib import Path
import unittest

from swphysics.definitions import DefinitionCatalog
from swphysics.geometry import (
    CUBE_POINTS_QUARTERS,
    convex_hull,
    partition_mixed_shapes_greedy,
    physics_voxel_geometry,
)
from swphysics.model import IDENTITY_MATRIX, WorldVoxel
from swphysics.non_cube_data import NON_CUBE_SAMPLE_POINTS_QUARTERS
from swphysics.vehicle import load_vehicle


ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests" / "fixtures"


def voxel(index, position, shape, definition="test"):
    return WorldVoxel(
        body_index=0,
        body_id="0",
        component_index=index,
        component_definition=definition,
        definition_voxel_index=0,
        insertion_index=index,
        position=position,
        physics_shape=shape,
        physics_rotation=IDENTITY_MATRIX,
    )


class GeometryTests(unittest.TestCase):
    def test_cube_hull_has_exact_quarter_unit_volume(self):
        self.assertEqual(384, convex_hull(CUBE_POINTS_QUARTERS).volume6_quarter_units)

    def test_all_41_binary_shapes_have_positive_3d_hulls(self):
        self.assertEqual(set(range(1, 42)), set(NON_CUBE_SAMPLE_POINTS_QUARTERS))
        for shape_id in range(1, 42):
            geometry = physics_voxel_geometry(voxel(0, (0, 0, 0), shape_id), 0)
            self.assertGreater(geometry.hull.volume6_quarter_units, 0, shape_id)

    def test_constructor_runtime_low_flag_bits_mirror_before_rotation(self):
        original = physics_voxel_geometry(voxel(0, (0, 0, 0), 1), 0)
        mirrored = physics_voxel_geometry(voxel(0, (0, 0, 0), 1), 1)
        self.assertEqual(
            tuple((-point[0], point[1], point[2]) for point in original.collision_samples_quarters),
            mirrored.collision_samples_quarters,
        )

    def test_mixed_model_preserves_the_confirmed_cube_a_b_counts(self):
        catalog = DefinitionCatalog(FIXTURES / "definitions")
        for filename, expected in (("order_a.xml", 3), ("order_b.xml", 4)):
            vehicle = load_vehicle(FIXTURES / "vehicles" / filename)
            voxels = vehicle.physics_voxels(catalog, 0)
            result = partition_mixed_shapes_greedy(voxels, {"01_block": 0})
            self.assertEqual(expected, result.shape_count)

    def test_one_by_two_wedge_segments_form_one_convex_group(self):
        voxels = (
            voxel(0, (0, 0, 0), 4, "05_wedge_2"),
            voxel(1, (0, 0, -1), 5, "05_wedge_2"),
        )
        result = partition_mixed_shapes_greedy(voxels, {"05_wedge_2": 0})
        self.assertEqual(1, result.shape_count)
        self.assertEqual((0, 1), result.groups[0].voxel_insertion_indices)

    def test_portable_approximation_misses_stage4_two_by_two_pyramid(self):
        voxels = (
            voxel(0, (0, 0, 0), 16, "11_pyramid_2x2"),
            voxel(1, (-1, 0, 0), 17, "11_pyramid_2x2"),
            voxel(2, (0, 0, -1), 17, "11_pyramid_2x2"),
        )
        result = partition_mixed_shapes_greedy(voxels, {"11_pyramid_2x2": 0})
        self.assertEqual(3, result.shape_count)


if __name__ == "__main__":
    unittest.main()
