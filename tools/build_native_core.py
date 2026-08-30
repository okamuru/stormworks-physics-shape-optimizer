"""Build and stage the dependency-free Rust score engine."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Optional, Tuple


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "native_core" / "Cargo.toml"
STAGED = ROOT / "swphysics" / "native"


def _cargo() -> str:
    local = ROOT / ".cargo-native" / "bin" / (
        "cargo.exe" if os.name == "nt" else "cargo"
    )
    if local.is_file():
        return str(local)
    resolved = shutil.which("cargo")
    if resolved:
        return resolved
    raise RuntimeError(
        "Cargo was not found. Install Rust or run the project-local rustup setup."
    )


def _host_artifact() -> tuple[Path, str]:
    release = ROOT / "native_core" / "target" / "release"
    if sys.platform == "darwin":
        return release / "libswphysics_native.dylib", "libswphysics_native.dylib"
    if sys.platform == "win32":
        return release / "swphysics_native.dll", "swphysics_native.dll"
    if sys.platform.startswith("linux"):
        return release / "libswphysics_native.so", "libswphysics_native.so"
    raise RuntimeError("unsupported native build platform: {}".format(sys.platform))


def _build_environment() -> dict:
    environment = dict(os.environ)
    local_cargo = ROOT / ".cargo-native"
    local_rustup = ROOT / ".rustup-native"
    if local_cargo.is_dir():
        environment.setdefault("CARGO_HOME", str(local_cargo))
    if local_rustup.is_dir():
        environment.setdefault("RUSTUP_HOME", str(local_rustup))
    return environment


def _run_build(
    target: str, check_only: bool
) -> Optional[Tuple[Path, str]]:
    environment = _build_environment()
    command = [_cargo(), "check" if check_only else "build"]
    if not check_only:
        command.extend(("--release", "--locked"))
    if target == "windows-x64" and sys.platform != "win32":
        compiler = shutil.which("x86_64-w64-mingw32-gcc")
        if compiler is None:
            raise RuntimeError(
                "x86_64-w64-mingw32-gcc was not found; install mingw-w64"
            )
        environment[
            "CARGO_TARGET_X86_64_PC_WINDOWS_GNU_LINKER"
        ] = compiler
        command.extend(("--target", "x86_64-pc-windows-gnu"))
    command.extend(("--manifest-path", str(MANIFEST)))
    subprocess.run(command, cwd=ROOT, check=True, env=environment)
    if check_only:
        return None
    if target == "windows-x64" and sys.platform != "win32":
        return (
            ROOT
            / "native_core"
            / "target"
            / "x86_64-pc-windows-gnu"
            / "release"
            / "swphysics_native.dll",
            "swphysics_native.dll",
        )
    return _host_artifact()


def _stage(artifact: tuple[Path, str]) -> None:
    source, filename = artifact
    if not source.is_file():
        raise RuntimeError("Cargo did not produce {}".format(source))
    STAGED.mkdir(parents=True, exist_ok=True)
    destination = STAGED / filename
    shutil.copy2(source, destination)
    print(destination)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    parser.add_argument(
        "--target",
        choices=("host", "windows-x64", "all"),
        default="host",
    )
    args = parser.parse_args()
    targets = (
        ("host", "windows-x64")
        if args.target == "all" and sys.platform == "darwin"
        else (args.target,)
    )
    for target in targets:
        artifact = _run_build(target, args.check)
        if artifact is not None:
            _stage(artifact)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
