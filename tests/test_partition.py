import unittest

from swphysics.partition import (
    partition_cubes_exact,
    partition_cubes_greedy,
    validate_partition,
)


ORDER_A = (
    (0, 0, 0),
    (0, 1, 0),
    (0, 2, 0),
    (0, 3, 0),
    (1, 0, 0),
    (1, 2, 0),
)

ORDER_B = (
    (0, 0, 0),
    (0, 2, 0),
    (0, 1, 0),
    (0, 3, 0),
    (1, 0, 0),
    (1, 2, 0),
)


class PartitionTests(unittest.TestCase):
    def test_greedy_hypothesis_is_order_sensitive(self):
        result_a = partition_cubes_greedy(ORDER_A)
        result_b = partition_cubes_greedy(ORDER_B)
        self.assertEqual(3, result_a.shape_count)
        self.assertEqual(4, result_b.shape_count)
        validate_partition(ORDER_A, result_a.boxes)
        validate_partition(ORDER_B, result_b.boxes)

    def test_exact_partition_finds_three_boxes(self):
        result = partition_cubes_exact(ORDER_B)
        self.assertEqual(3, result.shape_count)
        validate_partition(ORDER_B, result.boxes)

    def test_solid_cuboid_becomes_one_box(self):
        points = tuple((x, y, z) for x in range(3) for y in range(2) for z in range(4))
        result = partition_cubes_greedy(points, axis_order="zyx")
        self.assertEqual(1, result.shape_count)
        self.assertEqual((3, 2, 4), result.boxes[0].size_voxels)

    def test_overlap_is_rejected(self):
        with self.assertRaises(ValueError):
            partition_cubes_greedy(((0, 0, 0), (0, 0, 0)))


if __name__ == "__main__":
    unittest.main()
