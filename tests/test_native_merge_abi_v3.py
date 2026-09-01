import ctypes
from pathlib import Path
import random
import tempfile
import unittest
from unittest import mock

from swphysics.model import WorldVoxel, apply_matrix
from swphysics.native_merge import (
    NATIVE_ABI_VERSION,
    NativePreparedMergeEvaluator,
    _load_native_library,
    _reset_native_loader_for_tests,
    native_backend_available,
    native_backend_status,
)
from swphysics.non_cube_data import NON_CUBE_SAMPLE_POINTS_QUARTERS
from swphysics.portable_merge import (
    PreparedPortableMergeEvaluator,
    partition_portable_exact,
    voxel_clip_plane,
)


SCALED_NON_CUBE_ROTATION = (64, 0, 0, 0, 1, 0, 0, 0, 1)
GRID_REJECTED_WEDGE_ROTATION = (0, 1, 0, -1, 0, 0, 0, 0, 1)
IDENTITY_ROTATION = (1, 0, 0, 0, 1, 0, 0, 0, 1)


def _voxel(component_index, position, physics_shape, rotation):
    return WorldVoxel(
        body_index=0,
        body_id="abi-v3",
        component_index=component_index,
        component_definition="abi_v3_fixture",
        definition_voxel_index=0,
        insertion_index=component_index,
        position=position,
        physics_shape=physics_shape,
        physics_rotation=rotation,
    )


class _FakeFunction:
    def __init__(self, result):
        self.result = result
        self.argtypes = None
        self.restype = None

    def __call__(self):
        return self.result


class _LegacyAbiLibrary:
    def __init__(self):
        self.swp_native_abi_version = _FakeFunction(2)


class _CapturingPreparedLibrary:
    def __init__(self):
        self.library = self
        self.sample_offsets = ()

    def swp_prepared_create(
        self,
        positions,
        plane_values,
        plane_present,
        physics_shapes,
        voxel_sample_patterns,
        sample_counts,
        sample_offsets,
        sample_stride,
        collision_thresholds,
        sample_pattern_count,
        voxel_count,
        component_offsets,
        component_count,
        trailing_start,
        allow_overlaps,
        out_error,
    ):
        length = sample_pattern_count * sample_stride * 3
        self.sample_offsets = tuple(sample_offsets[index] for index in range(length))
        return 1

    def swp_prepared_destroy(self, handle):
        pass


class NativeMergeAbiV3Tests(unittest.TestCase):
    def tearDown(self):
        _reset_native_loader_for_tests()

    def test_abi_three_rejects_an_abi_two_library_and_reports_python_fallback(self):
        self.assertEqual(3, NATIVE_ABI_VERSION)
        with tempfile.TemporaryDirectory() as directory:
            legacy = Path(directory) / "legacy-native-library"
            legacy.touch()
            with mock.patch(
                "swphysics.native_merge._library_candidates",
                return_value=(legacy,),
            ), mock.patch(
                "swphysics.native_merge.ctypes.CDLL",
                return_value=_LegacyAbiLibrary(),
            ):
                _reset_native_loader_for_tests()
                self.assertFalse(native_backend_available())
                status = native_backend_status()
                self.assertIn("python fallback", status)
                self.assertIn("expected 3, got 2", status)
                prepared = PreparedPortableMergeEvaluator(
                    ((_voxel(0, (0, 0, 0), 0, IDENTITY_ROTATION),),)
                )
                self.assertEqual("python", prepared.native_backend)
                self.assertEqual(1, prepared.shape_count_order((0,)))

    def test_loaded_abi_three_declares_the_complete_signature(self):
        library = _load_native_library()
        if library is None:
            self.skipTest("ABI 3 native merge library is unavailable")
        self.assertEqual([], library.library.swp_native_abi_version.argtypes)
        self.assertIs(
            ctypes.c_uint32,
            library.library.swp_native_abi_version.restype,
        )
        self.assertEqual(
            [
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_uint8),
                ctypes.POINTER(ctypes.c_uint8),
                ctypes.POINTER(ctypes.c_uint32),
                ctypes.POINTER(ctypes.c_uint8),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.c_size_t,
                ctypes.POINTER(ctypes.c_uint8),
                ctypes.c_size_t,
                ctypes.c_size_t,
                ctypes.POINTER(ctypes.c_uint32),
                ctypes.c_size_t,
                ctypes.c_size_t,
                ctypes.c_uint8,
                ctypes.POINTER(ctypes.c_int32),
            ],
            library.library.swp_prepared_create.argtypes,
        )
        self.assertIs(
            ctypes.c_void_p,
            library.library.swp_prepared_create.restype,
        )
        self.assertEqual(
            [
                ctypes.c_void_p,
                ctypes.POINTER(ctypes.c_uint32),
                ctypes.c_size_t,
                ctypes.POINTER(ctypes.c_uint32),
            ],
            library.library.swp_prepared_score.argtypes,
        )
        self.assertIs(
            ctypes.c_int32,
            library.library.swp_prepared_score.restype,
        )
        self.assertEqual(
            [ctypes.c_void_p],
            library.library.swp_prepared_destroy.argtypes,
        )
        self.assertIsNone(library.library.swp_prepared_destroy.restype)

    def test_scaled_non_cube_offsets_over_i8_use_i32_buffer(self):
        scaled = _voxel(0, (0, 0, 0), 1, SCALED_NON_CUBE_ROTATION)
        plane = voxel_clip_plane(scaled)
        self.assertIsNotNone(plane)
        transformed_samples = tuple(
            value
            for sample in NON_CUBE_SAMPLE_POINTS_QUARTERS[1]
            for value in apply_matrix(SCALED_NON_CUBE_ROTATION, sample)
        )
        self.assertGreater(
            max(abs(value) for value in transformed_samples),
            127,
        )
        library = _CapturingPreparedLibrary()
        with mock.patch(
            "swphysics.native_merge._load_native_library",
            return_value=library,
        ):
            evaluator = NativePreparedMergeEvaluator(
                (scaled,),
                ((0,),),
                (),
                (plane,),
                (0,),
                False,
            )
        self.addCleanup(evaluator.close)
        self.assertEqual(
            transformed_samples,
            library.sample_offsets[:len(transformed_samples)],
        )

    def test_singular_overlap_winner_rules_match_python_reference(self):
        if not native_backend_available():
            self.skipTest("ABI 3 native merge library is unavailable")
        collapsed_axis = (0, 0, 0, 0, 1, 0, 0, 0, 1)
        zero_matrix = (0,) * 9
        collapsed_cubes = (
            (_voxel(0, (0, 0, 0), 0, collapsed_axis),),
            (_voxel(1, (0, 0, 0), 0, collapsed_axis),),
        )
        outside_seed_plane = (
            (_voxel(0, (0, 0, 0), 1, zero_matrix),),
            (_voxel(1, (0, 0, 0), 0, zero_matrix),),
        )

        merged = PreparedPortableMergeEvaluator(
            collapsed_cubes,
            allow_overlaps=True,
        )
        separate = PreparedPortableMergeEvaluator(
            outside_seed_plane,
            allow_overlaps=True,
        )

        self.assertEqual("rust_cdylib", merged.native_backend)
        self.assertEqual(1, merged.shape_count_order((0, 1)))
        self.assertEqual(
            1,
            merged._partition_order(
                (0, 1),
                collect_groups=False,
                validate_order=False,
            ),
        )
        self.assertEqual(
            ((0, 1),),
            tuple(
                group.voxel_insertion_indices
                for group in merged.partition_order((0, 1)).groups
            ),
        )
        self.assertEqual("rust_cdylib", separate.native_backend)
        self.assertEqual(1, separate.shape_count_order((0, 1)))
        self.assertEqual(
            ((0,), (1,)),
            tuple(
                group.voxel_insertion_indices
                for group in separate.partition_order((0, 1)).groups
            ),
        )

    def test_scale_shear_and_singular_use_native_binary_goldens(self):
        if not native_backend_available():
            self.skipTest("ABI 3 native merge library is unavailable")
        cases = (
            (
                "scale",
                (
                    _voxel(
                        0,
                        (-2, 1, 0),
                        4,
                        (1, 0, 0, 0, 3, 0, 0, 0, 3),
                    ),
                    _voxel(
                        1,
                        (-2, 1, -3),
                        5,
                        (1, 0, 0, 0, 3, 0, 0, 0, 3),
                    ),
                ),
                (2, 1, 1),
            ),
            (
                "shear",
                (
                    _voxel(
                        0,
                        (0, 0, 0),
                        1,
                        (0, 0, -3, -1, 0, 0, 0, -3, 4),
                    ),
                ),
                (1, 1, 0),
            ),
            (
                "singular",
                (
                    _voxel(
                        0,
                        (0, 0, 0),
                        1,
                        (0, 0, 0, 0, 0, 0, -1, 0, 0),
                    ),
                ),
                (1, 1, 0),
            ),
        )
        for name, voxels, expected in cases:
            with self.subTest(name=name):
                prepared = PreparedPortableMergeEvaluator(
                    tuple((voxel,) for voxel in voxels)
                )
                self.assertEqual("rust_cdylib", prepared.native_backend)
                self.assertIsNotNone(prepared._native_evaluator)
                try:
                    order = tuple(range(len(voxels)))
                    preview = prepared.partition_order(order)
                    self.assertEqual(
                        expected,
                        (
                            preview.merge_group_count,
                            preview.shape_count,
                            preview.rejected_convex_group_count,
                        ),
                    )
                    self.assertEqual(
                        preview.shape_count,
                        prepared.shape_count_order(order),
                    )
                finally:
                    prepared._native_evaluator.close()

    def test_non_grid_runtime_mirror_is_quarantined_from_native(self):
        sheared = _voxel(
            0,
            (1, 2, -1),
            3,
            (1, -1, 0, 0, 1, -1, -2, 0, 1),
        )
        # The binary constructor's build-24749959 plane for flags=4 is
        # ((2, 2, -1), (1, 2, 2)); the portable output-axis mirror currently
        # produces the different anchor below.  Do not claim Rust parity until
        # that rare non-grid/runtime-mirror rule is modeled.
        self.assertEqual(
            ((2, 2, 0), (1, 2, 2)),
            voxel_clip_plane(sheared, 4),
        )
        prepared = PreparedPortableMergeEvaluator(
            ((sheared,),),
            component_runtime_flags={0: 4},
        )
        self.assertIsNone(prepared._native_evaluator)
        self.assertEqual("python_runtime_mirror", prepared.native_backend)
        self.assertEqual(1, prepared.shape_count_order((0,)))

    def test_random_integer_transforms_match_python_reference(self):
        if not native_backend_available():
            self.skipTest("ABI 3 native merge library is unavailable")
        randomizer = random.Random(24749959)
        positions = [
            (x, y, z)
            for x in range(-2, 3)
            for y in range(-2, 3)
            for z in range(-1, 2)
        ]

        def matrix(kind):
            if kind == 0:
                return (
                    randomizer.choice((-4, -2, -1, 1, 2, 4)), 0, 0,
                    0, randomizer.choice((-3, -1, 1, 3)), 0,
                    0, 0, randomizer.choice((-4, -1, 1, 4)),
                )
            if kind == 1:
                return (
                    1, randomizer.choice((-3, -1, 1, 3)), 0,
                    0, 1, randomizer.choice((-2, -1, 1, 2)),
                    randomizer.choice((-2, -1, 0, 1, 2)), 0, 1,
                )
            if kind == 2:
                row = tuple(randomizer.randint(-4, 4) for _ in range(3))
                return row + row + (0, 0, 0)
            return tuple(randomizer.randint(-4, 4) for _ in range(9))

        for trial in range(80):
            randomizer.shuffle(positions)
            component_count = randomizer.randint(2, 7)
            groups = tuple(
                (
                    _voxel(
                        component_index,
                        positions[component_index],
                        randomizer.randint(1, 41),
                        matrix((trial + component_index) % 4),
                    ),
                )
                for component_index in range(component_count)
            )
            prepared = PreparedPortableMergeEvaluator(groups)
            self.assertEqual("rust_cdylib", prepared.native_backend)
            try:
                for _ in range(3):
                    order = list(range(component_count))
                    randomizer.shuffle(order)
                    order = tuple(order)
                    self.assertEqual(
                        prepared._partition_order(
                            order,
                            collect_groups=False,
                            validate_order=False,
                        ),
                        prepared.shape_count_order(order),
                        (trial, order),
                    )
            finally:
                prepared._native_evaluator.close()

    def test_native_layer_reject_trigger_uses_only_new_planes(self):
        if not native_backend_available():
            self.skipTest("ABI 3 native merge library is unavailable")
        rotation = (1, 0, 0, 0, 1, 0, 0, 0, -128)
        voxels = (
            _voxel(0, (0, 0, 0), 21, rotation),
            _voxel(1, (1, 0, 0), 19, rotation),
        )
        self.assertGreater(
            max(
                abs(value)
                for voxel in voxels
                for sample in NON_CUBE_SAMPLE_POINTS_QUARTERS[
                    voxel.physics_shape
                ]
                for value in apply_matrix(rotation, sample)
            ),
            127,
        )
        portable = partition_portable_exact(voxels)
        self.assertEqual(1, portable.merge_group_count)
        evaluator = NativePreparedMergeEvaluator(
            voxels,
            ((0,), (1,)),
            (),
            tuple(voxel_clip_plane(voxel) for voxel in voxels),
            (0, 0),
            False,
        )
        self.addCleanup(evaluator.close)
        self.assertEqual(portable.shape_count, evaluator.score((0, 1)))

    def test_grid_wedge_finalization_matches_python_rejected_shape(self):
        if not native_backend_available():
            self.skipTest("ABI 3 native merge library is unavailable")
        wedge = _voxel(
            0,
            (-2, 0, -1),
            11,
            GRID_REJECTED_WEDGE_ROTATION,
        )
        prepared = PreparedPortableMergeEvaluator(
            ((wedge,),),
            component_runtime_flags={0: 6},
        )
        self.assertEqual("rust_cdylib", prepared.native_backend)
        self.assertIsNotNone(prepared._native_evaluator)
        self.addCleanup(prepared._native_evaluator.close)
        self.assertEqual(0, prepared.partition_order((0,)).shape_count)
        self.assertEqual(0, prepared.shape_count_order((0,)))


if __name__ == "__main__":
    unittest.main()
