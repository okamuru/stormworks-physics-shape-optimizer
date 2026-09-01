from pathlib import Path
import platform
import random
import unittest

from swphysics.binary_oracle import DEFAULT_BINARY, Uc, BinaryPartitionOracle
from swphysics.definitions import DefinitionCatalog
from swphysics.model import IDENTITY_MATRIX, WorldVoxel
from swphysics.native_merge import native_backend_available
from swphysics.platform_paths import find_definition_directory
from swphysics.portable_merge import (
    DIRECTIONS,
    PortableMergeGroup,
    PortableMergeResult,
    PortableMergeOracle,
    PreparedPortableMergeEvaluator,
    _layer_positions,
    partition_portable_exact,
    voxel_clip_plane,
)
from swphysics.vehicle import load_vehicle


ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests" / "fixtures"
STAGE4 = ROOT / "game_compare" / "stage4"
DEFINITIONS = find_definition_directory() or ROOT / "definitions-not-installed"


def voxel(index, position, shape, rotation=IDENTITY_MATRIX):
    return WorldVoxel(
        body_index=0,
        body_id="0",
        component_index=index,
        component_definition="test",
        definition_voxel_index=0,
        insertion_index=index,
        position=position,
        physics_shape=shape,
        physics_rotation=rotation,
    )


class PortableMergeTests(unittest.TestCase):
    def test_overlap_seed_consumes_latest_lookup_winner(self):
        voxels = (
            voxel(0, (0, 0, 0), 0),
            voxel(1, (0, 0, 0), 0),
        )
        with self.assertRaisesRegex(ValueError, "overlapping physics voxel"):
            partition_portable_exact(voxels)
        result = partition_portable_exact(voxels, allow_overlaps=True)
        self.assertEqual(1, result.shape_count)
        self.assertEqual(
            ((0, 1),),
            tuple(group.voxel_insertion_indices for group in result.groups),
        )

    def test_overlap_uses_seed_plane_and_leaves_middle_duplicates_as_seeds(self):
        cube_then_wedge = partition_portable_exact(
            (
                voxel(0, (0, 0, 0), 0),
                voxel(1, (0, 0, 0), 1),
            ),
            allow_overlaps=True,
        )
        wedge_then_cube = partition_portable_exact(
            (
                voxel(0, (0, 0, 0), 1),
                voxel(1, (0, 0, 0), 0),
            ),
            allow_overlaps=True,
        )
        triple = partition_portable_exact(
            tuple(voxel(index, (0, 0, 0), 0) for index in range(3)),
            allow_overlaps=True,
        )

        self.assertEqual((), cube_then_wedge.groups[0].planes)
        self.assertEqual(
            (voxel_clip_plane(voxel(0, (0, 0, 0), 1)),),
            wedge_then_cube.groups[0].planes,
        )
        self.assertEqual(
            ((0, 2), (1,)),
            tuple(group.voxel_insertion_indices for group in triple.groups),
        )

    def test_overlap_seed_plane_can_reject_the_merged_convex_hull(self):
        result = partition_portable_exact(
            (
                voxel(0, (0, 0, 0), 21),
                voxel(1, (0, 0, 0), 10),
            ),
            allow_overlaps=True,
        )

        self.assertEqual(1, result.merge_group_count)
        self.assertEqual(0, result.shape_count)
        self.assertEqual(1, result.rejected_convex_group_count)

    def test_overlap_winner_outside_seed_plane_remains_a_separate_seed(self):
        zero_matrix = (0,) * 9
        result = partition_portable_exact(
            (
                voxel(0, (0, 0, 0), 1, zero_matrix),
                voxel(1, (0, 0, 0), 0, zero_matrix),
            ),
            allow_overlaps=True,
        )

        self.assertEqual(
            ((0,), (1,)),
            tuple(group.voxel_insertion_indices for group in result.groups),
        )
        self.assertEqual(1, result.shape_count)
        self.assertEqual(1, result.rejected_convex_group_count)

    def test_xml_scale_clip_plane_matches_native_integer_truncation(self):
        scaled = voxel(
            0,
            (-2, 1, 0),
            4,
            (1, 0, 0, 0, 3, 0, 0, 0, 3),
        )

        self.assertEqual(
            ((-2, 0, -4), (0, 6, -3)),
            voxel_clip_plane(scaled),
        )

    def test_even_xml_scale_half_grid_anchor_truncates_toward_zero(self):
        scaled = voxel(
            0,
            (0, 0, 0),
            1,
            (1, 0, 0, 0, -2, 0, 0, 0, 1),
        )

        self.assertEqual(((0, 1, 0), (0, -2, -1)), voxel_clip_plane(scaled))

    def test_shear_and_singular_clip_planes_match_native_goldens(self):
        sheared = voxel(
            0,
            (0, 0, 0),
            1,
            (0, 0, -3, -1, 0, 0, 0, -3, 4),
        )
        singular = voxel(
            1,
            (0, 0, 0),
            1,
            (0, 0, 0, 0, 0, 0, -1, 0, 0),
        )

        self.assertEqual(((1, 2, 0), (-1, 3, -4)), voxel_clip_plane(sheared))
        self.assertEqual(((1, 0, 0), (1, 0, 0)), voxel_clip_plane(singular))
        for item in (sheared, singular):
            result = partition_portable_exact((item,))
            self.assertEqual(1, result.merge_group_count)
            self.assertEqual(1, result.shape_count)

    def test_xml_scaled_wedge_finalization_rejects_degenerate_second_voxel(self):
        rotation = (1, 0, 0, 0, 3, 0, 0, 0, 3)
        voxels = (
            voxel(0, (-2, 1, 0), 4, rotation),
            voxel(1, (-2, 1, -3), 5, rotation),
        )

        result = partition_portable_exact(voxels)

        self.assertEqual(2, result.merge_group_count)
        self.assertEqual(1, result.shape_count)
        self.assertEqual(1, result.rejected_convex_group_count)
        self.assertEqual(
            ((8, True), (0, False)),
            tuple(
                (group.finalization_vertex_count, group.contributes_f2_shape)
                for group in result.groups
            ),
        )

    def test_finalized_groups_omit_convex_hulls_with_fewer_than_four_vertices(self):
        valid = PortableMergeGroup(
            seed_insertion_index=0,
            voxel_insertion_indices=(0,),
            component_indices=(0,),
            minimum=(-2, 1, 0),
            maximum=(-2, 1, 0),
            planes=(((-2, 0, -4), (0, 6, -3)),),
            seed_physics_shape=4,
        )
        rejected = PortableMergeGroup(
            seed_insertion_index=1,
            voxel_insertion_indices=(1,),
            component_indices=(0,),
            minimum=(-2, 1, -3),
            maximum=(-2, 1, -3),
            planes=(((-2, 0, -4), (0, 6, -3)),),
            seed_physics_shape=5,
        )

        result = PortableMergeResult((valid, rejected), 2)

        self.assertEqual((valid,), result.finalized_groups)
        self.assertEqual(1, result.shape_count)

    def test_prepared_xml_scaled_score_uses_finalized_shape_count(self):
        rotation = (1, 0, 0, 0, 3, 0, 0, 0, 3)
        prepared = PreparedPortableMergeEvaluator(
            (
                (
                    voxel(0, (-2, 1, 0), 4, rotation),
                    voxel(0, (-2, 1, -3), 5, rotation),
                ),
            )
        )

        self.assertEqual(
            "rust_cdylib" if native_backend_available() else "python",
            prepared.native_backend,
        )
        self.assertEqual(1, prepared.shape_count_order((0,)))
        self.assertEqual(1, prepared.partition_order((0,)).shape_count)

    def test_existing_plane_crossing_does_not_reject_scaled_wedge_layer(self):
        rotation = (1, 0, 0, 0, 1, 0, 0, 0, -3)
        result = partition_portable_exact(
            (
                voxel(0, (0, 0, 0), 21, rotation),
                voxel(1, (1, 0, 0), 19, rotation),
            )
        )

        self.assertEqual(1, result.merge_group_count)
        self.assertEqual((0, 1), result.groups[0].voxel_insertion_indices)

    @unittest.skipUnless(
        platform.machine() == "x86_64"
        and Uc is not None
        and DEFAULT_BINARY.is_file(),
        "x86 Unicorn and the installed build are required",
    )
    def test_xml_scaled_finalization_matches_installed_binary(self):
        rotation = (1, 0, 0, 0, 3, 0, 0, 0, 3)
        voxels = (
            voxel(0, (-2, 1, 0), 4, rotation),
            voxel(1, (-2, 1, -3), 5, rotation),
        )

        native = BinaryPartitionOracle().finalize(voxels)
        portable = partition_portable_exact(voxels)

        self.assertEqual((2, 1, 1), (
            native.merge_group_count,
            native.shape_count,
            native.rejected_convex_group_count,
        ))
        self.assertEqual(native.merge_group_count, portable.merge_group_count)
        self.assertEqual(native.shape_count, portable.shape_count)
        self.assertEqual(
            native.rejected_convex_group_count,
            portable.rejected_convex_group_count,
        )

    @unittest.skipUnless(
        platform.machine() == "x86_64"
        and Uc is not None
        and DEFAULT_BINARY.is_file(),
        "x86 Unicorn and the installed build are required",
    )
    def test_overlapping_position_goldens_match_installed_binary(self):
        cases = (
            (
                "cube_cube",
                (voxel(0, (0, 0, 0), 0), voxel(1, (0, 0, 0), 0)),
                ((0, 1),),
                1,
                ((0, "box"),),
            ),
            (
                "cube_wedge",
                (voxel(0, (0, 0, 0), 0), voxel(1, (0, 0, 0), 1)),
                ((0, 1),),
                1,
                ((0, "box"),),
            ),
            (
                "wedge_cube",
                (voxel(0, (0, 0, 0), 1), voxel(1, (0, 0, 0), 0)),
                ((0, 1),),
                1,
                ((1, "convex_hull"),),
            ),
            (
                "triple_cube",
                tuple(voxel(index, (0, 0, 0), 0) for index in range(3)),
                ((0, 2), (1,)),
                2,
                ((0, "box"), (0, "box")),
            ),
            (
                "rejected_wedge_pair",
                (voxel(0, (0, 0, 0), 21), voxel(1, (0, 0, 0), 10)),
                ((0, 1),),
                0,
                ((1, "rejected_convex"),),
            ),
            (
                "outside_zero_normal_seed",
                (
                    voxel(0, (0, 0, 0), 1, (0,) * 9),
                    voxel(1, (0, 0, 0), 0, (0,) * 9),
                ),
                ((0,), (1,)),
                1,
                ((1, "rejected_convex"), (0, "box")),
            ),
        )
        oracle = BinaryPartitionOracle()
        for name, voxels, expected_groups, expected_count, expected_final in cases:
            with self.subTest(name=name):
                native = oracle.finalize(voxels)
                portable = partition_portable_exact(
                    voxels,
                    allow_overlaps=True,
                )

                self.assertEqual(
                    expected_groups,
                    tuple(group.voxel_insertion_indices for group in native.groups),
                )
                self.assertEqual(
                    expected_groups,
                    tuple(group.voxel_insertion_indices for group in portable.groups),
                )
                self.assertEqual(expected_count, native.shape_count)
                self.assertEqual(expected_count, portable.shape_count)
                self.assertEqual(
                    expected_final,
                    tuple(
                        (shape.custom_plane_count, shape.representation)
                        for shape in native.finalized_shapes
                    ),
                )
                self.assertEqual(
                    tuple(item[0] for item in expected_final),
                    tuple(len(group.planes) for group in portable.groups),
                )

    def test_cube_order_a_and_b_match_confirmed_counts(self):
        catalog = DefinitionCatalog(FIXTURES / "definitions")
        for filename, expected in (("order_a.xml", 3), ("order_b.xml", 4)):
            vehicle = load_vehicle(FIXTURES / "vehicles" / filename)
            result = partition_portable_exact(vehicle.physics_voxels(catalog, 0))
            self.assertEqual(expected, result.shape_count)

    def test_prepared_evaluator_matches_direct_partition(self):
        catalog = DefinitionCatalog(FIXTURES / "definitions")
        vehicle = load_vehicle(FIXTURES / "vehicles" / "order_b.xml")
        voxels = vehicle.physics_voxels(catalog, 0)
        groups = tuple((item,) for item in voxels)
        prepared = PreparedPortableMergeEvaluator(groups)
        result = prepared.partition_order(tuple(range(len(groups))))
        direct = partition_portable_exact(voxels)
        self.assertEqual(direct.shape_count, result.shape_count)
        self.assertEqual(
            result.shape_count,
            prepared.shape_count_order(tuple(range(len(groups)))),
        )
        self.assertEqual(
            tuple(group.voxel_insertion_indices for group in direct.groups),
            tuple(group.voxel_insertion_indices for group in result.groups),
        )

    def test_prepared_preview_still_validates_component_order(self):
        groups = (
            (voxel(0, (0, 0, 0), 0),),
            (voxel(1, (1, 0, 0), 0),),
        )
        prepared = PreparedPortableMergeEvaluator(groups)
        for invalid_order in ((0,), (0, 0), (0, 2)):
            with self.subTest(order=invalid_order):
                with self.assertRaisesRegex(ValueError, "must be a permutation"):
                    prepared.partition_order(invalid_order)

    def test_axis_specialized_layers_match_generic_xyz_order(self):
        randomizer = random.Random(24749959)
        for _ in range(200):
            minimum = tuple(randomizer.randint(-8, 4) for _ in range(3))
            maximum = tuple(
                minimum[axis] + randomizer.randint(0, 8)
                for axis in range(3)
            )
            for direction in DIRECTIONS:
                axis = next(
                    index for index, value in enumerate(direction) if value
                )
                coordinate = (
                    minimum[axis] - 1
                    if direction[axis] < 0
                    else maximum[axis] + 1
                )
                ranges = [
                    (coordinate,)
                    if index == axis
                    else range(minimum[index], maximum[index] + 1)
                    for index in range(3)
                ]
                expected = tuple(
                    (x, y, z)
                    for x in ranges[0]
                    for y in ranges[1]
                    for z in ranges[2]
                )
                self.assertEqual(
                    expected,
                    tuple(_layer_positions(minimum, maximum, direction)),
                )

    def test_trusted_shape_count_matches_preview_for_random_valid_orders(self):
        groups = (
            (
                voxel(0, (0, 0, 0), 0),
                voxel(0, (1, 0, 0), 0),
            ),
            (voxel(1, (0, 1, 0), 1),),
            (voxel(2, (1, 1, 0), 0),),
            (voxel(3, (2, 0, 0), 0),),
            (voxel(4, (2, 1, 0), 2),),
        )
        prepared = PreparedPortableMergeEvaluator(groups)
        randomizer = random.Random(108)
        for _ in range(100):
            order = list(range(len(groups)))
            randomizer.shuffle(order)
            order = tuple(order)
            self.assertEqual(
                prepared.partition_order(order).shape_count,
                prepared.shape_count_order(order),
            )

    def test_prepared_evaluator_rebuilds_latest_overlap_by_order(self):
        groups = (
            (voxel(0, (0, 0, 0), 0),),
            (voxel(1, (0, 0, 0), 0),),
        )
        prepared = PreparedPortableMergeEvaluator(groups, allow_overlaps=True)
        forward = prepared.partition_order((0, 1))
        reverse = prepared.partition_order((1, 0))
        self.assertEqual(1, forward.shape_count)
        self.assertEqual(1, reverse.shape_count)
        self.assertEqual((0,), tuple(group.seed_insertion_index for group in forward.groups))
        self.assertEqual((0,), tuple(group.seed_insertion_index for group in reverse.groups))
        self.assertEqual((0, 1), forward.groups[0].voxel_insertion_indices)
        self.assertEqual((0, 1), reverse.groups[0].voxel_insertion_indices)

    def test_prepared_overlap_order_changes_the_native_seed_plane(self):
        prepared = PreparedPortableMergeEvaluator(
            (
                (voxel(0, (0, 0, 0), 21),),
                (voxel(1, (0, 0, 0), 10),),
            ),
            allow_overlaps=True,
        )

        self.assertEqual(0, prepared.partition_order((0, 1)).shape_count)
        self.assertEqual(1, prepared.partition_order((1, 0)).shape_count)
        self.assertEqual(0, prepared.shape_count_order((0, 1)))
        self.assertEqual(1, prepared.shape_count_order((1, 0)))

    def test_prepared_overlap_lookup_matches_direct_latest_winner(self):
        groups = (
            (
                voxel(0, (0, 0, 0), 0),
                voxel(0, (1, 0, 0), 0),
            ),
            (
                voxel(1, (0, 0, 0), 1),
                voxel(1, (1, 1, 0), 0),
            ),
            (voxel(2, (0, 1, 0), 0),),
        )
        trailing = (voxel(3, (1, 1, 0), 2),)
        prepared = PreparedPortableMergeEvaluator(
            groups,
            trailing,
            allow_overlaps=True,
        )
        for order in ((0, 1, 2), (2, 1, 0), (1, 0, 2)):
            direct_voxels = tuple(
                item
                for component_index in order
                for item in groups[component_index]
            ) + trailing
            direct = partition_portable_exact(
                direct_voxels,
                allow_overlaps=True,
            )
            preview = prepared.partition_order(order)
            self.assertEqual(direct.shape_count, preview.shape_count)
            self.assertEqual(
                tuple((group.minimum, group.maximum, group.planes) for group in direct.groups),
                tuple((group.minimum, group.maximum, group.planes) for group in preview.groups),
            )
            self.assertEqual(
                direct.shape_count,
                prepared.shape_count_order(order),
            )

    def test_all_non_cube_shape_ids_have_a_constructor_plane(self):
        for shape_id in range(1, 42):
            self.assertIsNotNone(
                voxel_clip_plane(voxel(shape_id, (0, 0, 0), shape_id)),
                shape_id,
            )

    def test_runtime_mirror_moves_plane_in_rotated_output_axis(self):
        source = voxel(0, (2, 3, 4), 1)
        original = voxel_clip_plane(source, 0)
        mirrored = voxel_clip_plane(source, 1)
        self.assertEqual((-original[1][0], original[1][1], original[1][2]), mirrored[1])
        self.assertEqual(5 - original[0][0], mirrored[0][0])

    def test_clipped_direction_retries_after_perpendicular_expansion(self):
        rotation = (0, -1, 0, 0, 0, 1, -1, 0, 0)
        groups = (
            (voxel(0, (-2, 0, 1), 1, rotation),),
            (voxel(1, (-3, 0, 2), 1, rotation),),
            (voxel(2, (-3, 0, 1), 0),),
        )
        direct = partition_portable_exact(
            tuple(item for group in groups for item in group)
        )
        prepared = PreparedPortableMergeEvaluator(groups).partition_order(
            (0, 1, 2)
        )
        for result in (direct, prepared):
            self.assertEqual(1, result.shape_count)
            self.assertEqual(
                ((0, 1, 2),),
                tuple(group.voxel_insertion_indices for group in result.groups),
            )

    def test_stage4_counts_match_all_nine_game_observations(self):
        if not STAGE4.is_dir() or not DEFINITIONS.is_dir():
            self.skipTest("Stage 4 pack and installed definitions are required")
        expected = {
            "Codex Physics Shape D Cube Multi Good.xml": 2,
            "Codex Physics Shape D Cube Multi Bad.xml": 2,
            "Codex Physics Shape G Direction Probe.xml": 3,
            "Codex Physics Shape F Mixed Good.xml": 2,
            "Codex Physics Shape F Mixed Bad.xml": 3,
            "Codex Physics Shape E1 Basic 1x1.xml": 3,
            "Codex Physics Shape E2 Linear Multi.xml": 6,
            "Codex Physics Shape E3 Large Pyramids.xml": 3,
            "Codex Physics Shape E4 Large Inverse.xml": 3,
        }
        oracle = PortableMergeOracle()
        for filename, shape_count in expected.items():
            path = STAGE4 / filename
            vehicle = load_vehicle(path)
            catalog = DefinitionCatalog.for_vehicle(DEFINITIONS, path)
            total = sum(
                oracle.partition(vehicle.physics_voxels(catalog, body.index)).shape_count
                for body in vehicle.bodies
            )
            self.assertEqual(shape_count, total, filename)


if __name__ == "__main__":
    unittest.main()
