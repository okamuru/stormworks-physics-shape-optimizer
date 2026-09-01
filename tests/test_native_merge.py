import os
import random
import unittest
from unittest import mock

from swphysics.model import WorldVoxel
from swphysics.native_merge import (
    NativeMergeError,
    _reset_native_loader_for_tests,
    native_backend_available,
)
from swphysics.portable_merge import PreparedPortableMergeEvaluator
from swphysics.rotations import PROPER_GRID_ROTATIONS


def voxel(index, position, shape, rotation):
    return WorldVoxel(
        body_index=0,
        body_id="0",
        component_index=index,
        component_definition="native_random",
        definition_voxel_index=0,
        insertion_index=index,
        position=position,
        physics_shape=shape,
        physics_rotation=rotation,
    )


class NativeMergeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not native_backend_available():
            raise unittest.SkipTest("optional native merge library is unavailable")

    def test_random_geometry_and_orders_match_python_reference(self):
        randomizer = random.Random(24749959)
        all_positions = [
            (x, y, z)
            for x in range(-2, 3)
            for y in range(-2, 3)
            for z in range(-1, 2)
        ]
        observed_shapes = set()
        for trial in range(120):
            randomizer.shuffle(all_positions)
            component_count = randomizer.randint(2, 9)
            groups = []
            cursor = 0
            runtime_flags = {}
            for component_index in range(component_count):
                group_size = randomizer.randint(1, 3)
                group = []
                runtime_flags[component_index] = randomizer.randrange(8)
                for _ in range(group_size):
                    shape = (
                        0
                        if randomizer.random() < 0.55
                        else 1 + ((trial + cursor) % 41)
                    )
                    observed_shapes.add(shape)
                    group.append(
                        voxel(
                            component_index,
                            all_positions[cursor],
                            shape,
                            randomizer.choice(PROPER_GRID_ROTATIONS),
                        )
                    )
                    cursor += 1
                groups.append(tuple(group))
            prepared = PreparedPortableMergeEvaluator(
                groups,
                component_runtime_flags=runtime_flags,
            )
            self.assertEqual("rust_cdylib", prepared.native_backend)
            for _ in range(6):
                order = list(range(component_count))
                randomizer.shuffle(order)
                order = tuple(order)
                native_count = prepared.shape_count_order(order)
                python_count = prepared._partition_order(
                    order,
                    collect_groups=False,
                    validate_order=False,
                )
                self.assertEqual(
                    python_count,
                    native_count,
                    "trial={} order={}".format(trial, order),
                )
        self.assertEqual(set(range(42)), observed_shapes)

    def test_overlap_latest_winner_matches_python_for_both_orders(self):
        rotation = PROPER_GRID_ROTATIONS[0]
        groups = (
            (
                voxel(0, (0, 0, 0), 0, rotation),
                voxel(0, (1, 0, 0), 1, rotation),
            ),
            (
                voxel(1, (0, 0, 0), 2, rotation),
                voxel(1, (0, 1, 0), 0, rotation),
            ),
            (voxel(2, (1, 1, 0), 0, rotation),),
        )
        trailing = (voxel(3, (1, 1, 0), 3, rotation),)
        prepared = PreparedPortableMergeEvaluator(
            groups,
            trailing,
            allow_overlaps=True,
        )
        for order in ((0, 1, 2), (2, 1, 0), (1, 0, 2)):
            self.assertEqual(
                prepared._partition_order(
                    order,
                    collect_groups=False,
                    validate_order=False,
                ),
                prepared.shape_count_order(order),
            )

    def test_overlap_seed_plane_and_triple_winner_match_python(self):
        rotation = PROPER_GRID_ROTATIONS[0]
        rejected_then_valid = PreparedPortableMergeEvaluator(
            (
                (voxel(0, (0, 0, 0), 21, rotation),),
                (voxel(1, (0, 0, 0), 10, rotation),),
            ),
            allow_overlaps=True,
        )
        triple_cube = PreparedPortableMergeEvaluator(
            tuple(
                (voxel(index, (0, 0, 0), 0, rotation),)
                for index in range(3)
            ),
            allow_overlaps=True,
        )

        self.assertEqual("rust_cdylib", rejected_then_valid.native_backend)
        self.assertEqual(0, rejected_then_valid.shape_count_order((0, 1)))
        self.assertEqual(1, rejected_then_valid.shape_count_order((1, 0)))
        self.assertEqual("rust_cdylib", triple_cube.native_backend)
        self.assertEqual(2, triple_cube.shape_count_order((0, 1, 2)))
        self.assertEqual(
            triple_cube._partition_order(
                (0, 1, 2),
                collect_groups=False,
                validate_order=False,
            ),
            triple_cube.shape_count_order((0, 1, 2)),
        )

    def test_native_boundary_rejects_invalid_order_without_crashing(self):
        rotation = PROPER_GRID_ROTATIONS[0]
        prepared = PreparedPortableMergeEvaluator(
            (
                (voxel(0, (0, 0, 0), 0, rotation),),
                (voxel(1, (1, 0, 0), 0, rotation),),
            )
        )
        with self.assertRaisesRegex(NativeMergeError, "invalid component order"):
            prepared._native_evaluator.score((0, 0))
        with self.assertRaisesRegex(NativeMergeError, "invalid component order"):
            prepared._native_evaluator.score((0, 2))

    def test_environment_can_force_python_fallback(self):
        rotation = PROPER_GRID_ROTATIONS[0]
        original_environment = dict(os.environ)
        try:
            with mock.patch.dict(
                os.environ,
                {"SWPHYSICS_DISABLE_NATIVE": "1"},
                clear=False,
            ):
                _reset_native_loader_for_tests()
                prepared = PreparedPortableMergeEvaluator(
                    ((voxel(0, (0, 0, 0), 0, rotation),),)
                )
                self.assertIsNone(prepared._native_evaluator)
                self.assertEqual("python", prepared.native_backend)
                self.assertEqual(1, prepared.shape_count_order((0,)))
        finally:
            os.environ.clear()
            os.environ.update(original_environment)
            _reset_native_loader_for_tests()


if __name__ == "__main__":
    unittest.main()
