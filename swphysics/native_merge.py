"""Optional Rust score engine for the prepared portable merger.

The native library is deliberately optional.  Loading, ABI validation, input
marshalling, or an individual score failure all leave the build-pinned Python
implementation available as the correctness reference and fallback.
"""

from __future__ import annotations

from array import array
import ctypes
from functools import lru_cache
import os
from pathlib import Path
import sys
from typing import Optional, Sequence, Tuple

from .model import WorldVoxel, apply_matrix
from .non_cube_data import (
    NON_CUBE_COLLISION_THRESHOLDS,
    NON_CUBE_SAMPLE_POINTS_QUARTERS,
)


NATIVE_ABI_VERSION = 3
MAX_SAMPLE_STRIDE = max(
    len(points) for points in NON_CUBE_SAMPLE_POINTS_QUARTERS.values()
)

_ERROR_NAMES = {
    0: "ok",
    1: "null pointer",
    2: "invalid native input layout",
    3: "overlapping voxels are disabled",
    4: "invalid component order",
    5: "native panic was contained",
    6: "shape count overflow",
}


class NativeMergeError(RuntimeError):
    pass


def _library_filename() -> Optional[str]:
    if sys.platform == "darwin":
        return "libswphysics_native.dylib"
    if sys.platform == "win32":
        return "swphysics_native.dll"
    if sys.platform.startswith("linux"):
        return "libswphysics_native.so"
    return None


def _library_candidates() -> Tuple[Path, ...]:
    filename = _library_filename()
    if filename is None:
        return ()
    candidates = []
    override = os.environ.get("SWPHYSICS_NATIVE_LIBRARY", "").strip()
    if override:
        candidates.append(Path(override).expanduser())
    package_root = Path(__file__).resolve().parent
    candidates.append(package_root / "native" / filename)
    frozen_root = getattr(sys, "_MEIPASS", None)
    if frozen_root:
        frozen = Path(frozen_root)
        candidates.extend(
            (
                frozen / "swphysics" / "native" / filename,
                frozen / "native" / filename,
                frozen / filename,
            )
        )
    # Preserve order while avoiding duplicate diagnostics and load attempts.
    return tuple(dict.fromkeys(candidates))


class _NativeLibrary:
    def __init__(self, path: Path):
        self.path = path
        self.library = ctypes.CDLL(str(path))
        self.library.swp_native_abi_version.argtypes = []
        self.library.swp_native_abi_version.restype = ctypes.c_uint32
        abi = int(self.library.swp_native_abi_version())
        if abi != NATIVE_ABI_VERSION:
            raise NativeMergeError(
                "native ABI mismatch: expected {}, got {}".format(
                    NATIVE_ABI_VERSION, abi
                )
            )

        self.library.swp_prepared_create.argtypes = [
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
        ]
        self.library.swp_prepared_create.restype = ctypes.c_void_p
        self.library.swp_prepared_score.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_uint32),
        ]
        self.library.swp_prepared_score.restype = ctypes.c_int32
        self.library.swp_prepared_destroy.argtypes = [ctypes.c_void_p]
        self.library.swp_prepared_destroy.restype = None


_native_load_error: Optional[str] = None


@lru_cache(maxsize=1)
def _load_native_library() -> Optional[_NativeLibrary]:
    global _native_load_error
    if os.environ.get("SWPHYSICS_DISABLE_NATIVE", "").strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }:
        _native_load_error = "disabled by SWPHYSICS_DISABLE_NATIVE"
        return None
    filename = _library_filename()
    if filename is None:
        _native_load_error = "unsupported platform {}".format(sys.platform)
        return None
    errors = []
    for candidate in _library_candidates():
        if not candidate.is_file():
            continue
        try:
            library = _NativeLibrary(candidate)
        except (AttributeError, OSError, NativeMergeError) as error:
            errors.append("{}: {}".format(candidate, error))
            continue
        _native_load_error = None
        return library
    if errors:
        _native_load_error = "; ".join(errors)
    else:
        _native_load_error = "{} was not found".format(filename)
    return None


def native_backend_status() -> str:
    library = _load_native_library()
    if library is not None:
        return "rust_cdylib_abi{} ({})".format(
            NATIVE_ABI_VERSION, library.path
        )
    return "python fallback ({})".format(
        _native_load_error or "native library unavailable"
    )


def native_backend_available() -> bool:
    return _load_native_library() is not None


def _pointer(values: array, ctype):
    if not values:
        return ctypes.POINTER(ctype)()
    return ctypes.cast(
        (ctype * len(values)).from_buffer(values),
        ctypes.POINTER(ctype),
    )


class NativePreparedMergeEvaluator:
    """Opaque native score handle.

    A handle owns a copied, immutable geometry snapshot plus reusable scratch
    buffers.  It is intentionally not safe for concurrent calls; every worker
    process constructs its own handle, matching the existing prepared Python
    evaluator lifetime.
    """

    def __init__(
        self,
        voxels: Sequence[WorldVoxel],
        component_voxel_indices: Sequence[Sequence[int]],
        trailing_voxel_indices: Sequence[int],
        voxel_planes,
        voxel_runtime_flags: Sequence[int],
        allow_overlaps: bool,
    ):
        library = _load_native_library()
        if library is None:
            raise NativeMergeError(
                _native_load_error or "native library unavailable"
            )
        self._library = library
        self._handle = None
        voxel_count = len(voxels)
        if len(voxel_planes) != voxel_count or len(voxel_runtime_flags) != voxel_count:
            raise NativeMergeError("native voxel metadata length mismatch")

        positions = array("i")
        plane_values = array("i")
        plane_present = array("B")
        physics_shapes = array("B")
        voxel_sample_patterns = array("I")
        pattern_by_key = {}
        patterns = []
        try:
            for index, voxel in enumerate(voxels):
                positions.extend(voxel.position)
                physics_shapes.append(voxel.physics_shape)
                plane = voxel_planes[index]
                if plane is None:
                    plane_present.append(0)
                    plane_values.extend((0, 0, 0, 0, 0, 0))
                    samples = ()
                    collision_threshold = 0
                else:
                    plane_present.append(1)
                    plane_values.extend(plane[0])
                    plane_values.extend(plane[1])
                    samples = NON_CUBE_SAMPLE_POINTS_QUARTERS[
                        voxel.physics_shape
                    ]
                    collision_threshold = NON_CUBE_COLLISION_THRESHOLDS[
                        voxel.physics_shape - 1
                    ]
                transformed_samples = []
                runtime_flags = voxel_runtime_flags[index] & 7
                for sample in samples:
                    rotated = list(apply_matrix(voxel.physics_rotation, sample))
                    for axis in range(3):
                        if runtime_flags & (1 << axis):
                            rotated[axis] = -rotated[axis]
                    transformed_samples.extend(rotated)
                pattern_key = (
                    len(samples),
                    tuple(transformed_samples),
                    collision_threshold,
                )
                pattern_index = pattern_by_key.get(pattern_key)
                if pattern_index is None:
                    pattern_index = len(patterns)
                    pattern_by_key[pattern_key] = pattern_index
                    patterns.append(pattern_key)
                voxel_sample_patterns.append(pattern_index)
        except (KeyError, OverflowError, TypeError, ValueError) as error:
            raise NativeMergeError(
                "could not marshal native voxel data: {}".format(error)
            ) from error

        sample_counts = array("B")
        sample_offsets = array("i")
        collision_thresholds = array("B")
        try:
            for sample_count, transformed_samples, collision_threshold in patterns:
                sample_counts.append(sample_count)
                sample_offsets.extend(transformed_samples)
                sample_offsets.extend(
                    (0,) * (
                        MAX_SAMPLE_STRIDE * 3 - len(transformed_samples)
                    )
                )
                collision_thresholds.append(collision_threshold)
        except (OverflowError, TypeError, ValueError) as error:
            raise NativeMergeError(
                "could not marshal native sample offsets: {}".format(error)
            ) from error

        component_offsets = array("I", (0,))
        expected_start = 0
        for indices in component_voxel_indices:
            indices = tuple(indices)
            if indices != tuple(range(expected_start, expected_start + len(indices))):
                raise NativeMergeError(
                    "native prepared components must be contiguous"
                )
            expected_start += len(indices)
            component_offsets.append(expected_start)
        trailing = tuple(trailing_voxel_indices)
        if trailing != tuple(range(expected_start, voxel_count)):
            raise NativeMergeError("native trailing voxels must be contiguous")

        error_code = ctypes.c_int32(0)
        handle = library.library.swp_prepared_create(
            _pointer(positions, ctypes.c_int32),
            _pointer(plane_values, ctypes.c_int32),
            _pointer(plane_present, ctypes.c_uint8),
            _pointer(physics_shapes, ctypes.c_uint8),
            _pointer(voxel_sample_patterns, ctypes.c_uint32),
            _pointer(sample_counts, ctypes.c_uint8),
            _pointer(sample_offsets, ctypes.c_int32),
            MAX_SAMPLE_STRIDE,
            _pointer(collision_thresholds, ctypes.c_uint8),
            len(patterns),
            voxel_count,
            _pointer(component_offsets, ctypes.c_uint32),
            len(component_voxel_indices),
            expected_start,
            1 if allow_overlaps else 0,
            ctypes.byref(error_code),
        )
        if not handle:
            raise NativeMergeError(
                "native evaluator creation failed: {} ({})".format(
                    _ERROR_NAMES.get(error_code.value, "unknown error"),
                    error_code.value,
                )
            )
        self._handle = ctypes.c_void_p(handle)
        self.component_count = len(component_voxel_indices)

    def score(self, component_order: Sequence[int]) -> int:
        if self._handle is None:
            raise NativeMergeError("native evaluator is closed")
        try:
            order = array("I", component_order)
        except (OverflowError, TypeError, ValueError) as error:
            raise NativeMergeError("invalid native component order") from error
        result = ctypes.c_uint32(0)
        error_code = int(
            self._library.library.swp_prepared_score(
                self._handle,
                _pointer(order, ctypes.c_uint32),
                len(order),
                ctypes.byref(result),
            )
        )
        if error_code:
            raise NativeMergeError(
                "native score failed: {} ({})".format(
                    _ERROR_NAMES.get(error_code, "unknown error"), error_code
                )
            )
        return int(result.value)

    def close(self) -> None:
        handle = self._handle
        if handle is not None:
            self._handle = None
            self._library.library.swp_prepared_destroy(handle)

    def __del__(self):
        try:
            self.close()
        except Exception:
            # Interpreter shutdown can clear ctypes/module globals first.
            pass


def create_native_prepared_evaluator(*args, **kwargs):
    if _load_native_library() is None:
        return None
    return NativePreparedMergeEvaluator(*args, **kwargs)


def _reset_native_loader_for_tests() -> None:
    global _native_load_error
    _load_native_library.cache_clear()
    _native_load_error = None
