# swphysics native merge evaluator

This crate implements the hot `PreparedPortableMergeEvaluator.shape_count_order`
path as a dependency-free Rust `cdylib`. Python remains responsible for parsing,
Physics Flooder expansion, search policy, result materialization, and source-
preserving XML output.

## ABI

ABI version 2 exports:

- `swp_native_abi_version`
- `swp_prepared_create`
- `swp_prepared_score`
- `swp_prepared_destroy`

One prepared handle owns immutable voxel, plane, coordinate-index, and transformed
sample-pattern data for one Body. Each candidate score crosses FFI once with only
the Component order.

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
