import unittest

from swphysics.exact_optimizer import (
    _candidate_move_pairs_with_barriers,
    _native_seed_sweep_candidates,
)
from swphysics.model import IDENTITY_MATRIX, WorldVoxel


def voxel(component_index, x):
    return WorldVoxel(
        body_index=0,
        body_id="0",
        component_index=component_index,
        component_definition="01_block",
        definition_voxel_index=0,
        insertion_index=component_index,
        position=(x, 0, 0),
        physics_shape=0,
        physics_rotation=IDENTITY_MATRIX,
    )


class ComponentOrderBarrierTests(unittest.TestCase):
    def test_move_candidates_do_not_cross_omitted_component_barriers(self):
        pairs = tuple(
            _candidate_move_pairs_with_barriers(
                7,
                fixed_indices=(),
                barrier_offsets=(2, 5),
            )
        )

        self.assertTrue(pairs)
        self.assertTrue(
            all(
                (source < 2 and target < 2)
                or (2 <= source < 5 and 2 <= target < 5)
                or (source >= 5 and target >= 5)
                for source, target in pairs
            )
        )

    def test_native_seed_sweeps_keep_omitted_component_segments(self):
        groups = tuple(
            (voxel(index, x),)
            for index, x in enumerate((8, 7, 6, 5, 4, 3, 2))
        )
        identity = tuple(range(len(groups)))
        candidates = _native_seed_sweep_candidates(
            groups,
            fixed_indices=(),
            seen={identity},
            limit=48,
            barrier_offsets=(2, 5),
        )

        self.assertTrue(candidates)
        for candidate in candidates:
            self.assertEqual({0, 1}, set(candidate[:2]))
            self.assertEqual({2, 3, 4}, set(candidate[2:5]))
            self.assertEqual({5, 6}, set(candidate[5:]))


if __name__ == "__main__":
    unittest.main()
