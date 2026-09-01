# Building from Source

The tagged source contains the Python application, QML renderer, Rust native
library, compatibility tables, packaging configuration, and tests used for the
V1.2.0 Alpha release.

## Requirements

- Python 3.9 or newer
- Rust toolchain with Cargo
- macOS 12 or newer for the macOS build
- Windows 10 or newer for the Windows build

Python build dependencies and their exact release versions are listed in
`requirements-build.txt`. The Rust crate has no external crate dependencies.

## Run from source

macOS or Linux:

```sh
python3 -m venv .venv-build
.venv-build/bin/python -m pip install -r requirements-build.txt
.venv-build/bin/python tools/build_native_core.py --target host
.venv-build/bin/python launch_app.py
```

Windows Command Prompt:

```bat
py -3 -m venv .venv-build-windows
.venv-build-windows\Scripts\python.exe -m pip install -r requirements-build.txt
.venv-build-windows\Scripts\python.exe tools\build_native_core.py --target host
.venv-build-windows\Scripts\python.exe launch_app.py
```

## Build a release package

- macOS: run `./build_macos.command`
- Windows: run `build_windows.bat`

Both scripts build the native library, package the application, run the bundled
self-tests, and create an OS-specific ZIP under `release/`.

## Tests

```sh
.venv-build/bin/python -m unittest discover -s tests
```

The public test suite uses the included synthetic fixtures and does not require
game files. Running the application itself requires the user's locally
installed Stormworks component definitions. The application does not modify the
game executable.
