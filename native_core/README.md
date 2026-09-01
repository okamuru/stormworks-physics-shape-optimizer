# swphysics native merge evaluator

This crate implements the hot `PreparedPortableMergeEvaluator.shape_count_order`
path as a dependency-free Rust `cdylib`. Python remains responsible for parsing,
Physics Flooder expansion, search policy, result materialization, and source-
preserving XML output.

## ABI

ABI version 3 exports:

- `swp_native_abi_version`
- `swp_prepared_create`
- `swp_prepared_score`
- `swp_prepared_destroy`

One prepared handle owns immutable voxel, plane, coordinate-index, and transformed
sample-pattern data for one Body. Each candidate score crosses FFI once with only
the Component order.

ABI 3 widens transformed quarter-voxel sample offsets from signed 8-bit to signed
32-bit integers and supplies each seed voxel's physics-shape id for the final F2
convex-hull eligibility check. Python checks `swp_native_abi_version` before
configuring or calling the remaining symbols, so an older ABI 2 library is
rejected and the portable Python scorer is used instead.

The Rust scorer applies that final eligibility check for build-pinned grid
transforms, including the rule that a clipped hull with fewer than four surviving
vertices contributes no F2 shape.

ABI 3 also scores integer XML scale, shear, reflection, and singular non-cube
transforms when their runtime mirror flags are zero, which is the ordinary
vehicle-analysis path. A non-grid non-cube combined with nonzero runtime mirror
flags remains on the Python path because the build's clip-plane anchor mirror
rule for that combination is not yet modeled exactly. Values outside the i32
ABI domain likewise fall back safely during marshalling.

## Build

From the project root:

```text
python tools/build_native_core.py --target host
python tools/build_native_core.py --target all
python tools/build_native_core.py --check --target host
```

`--target all` additionally needs the Rust `x86_64-pc-windows-gnu` target and a
MinGW-w64 cross linker when run from macOS. Staged binaries are written under
`swphysics/native/` and are bundled by PyInstaller.

Set `SWPHYSICS_DISABLE_NATIVE=1` to force the Python reference implementation, or
`SWPHYSICS_NATIVE_VERIFY=1` to shadow-compare the first native score of each
prepared evaluator with Python.
