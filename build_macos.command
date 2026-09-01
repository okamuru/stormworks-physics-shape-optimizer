#!/bin/zsh
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
DIST_ONLY=0
if [[ "${1:-}" == "--dist-only" ]]; then
    DIST_ONLY=1
    shift
fi
if (( $# )); then
    print -u2 "Usage: $0 [--dist-only]"
    exit 2
fi
cd "$PROJECT_DIR"
export PYINSTALLER_CONFIG_DIR="$PROJECT_DIR/build/pyinstaller-cache"
export PYTHONPYCACHEPREFIX="$PROJECT_DIR/build/python-cache"
export COPYFILE_DISABLE=1

BUILD_PYTHON="$PROJECT_DIR/.venv-build/bin/python"
if [[ ! -x "$BUILD_PYTHON" ]]; then
  python3 -m venv "$PROJECT_DIR/.venv-build"
fi

if ! "$BUILD_PYTHON" -c "import PIL, PyInstaller; from PySide6 import QtQuick3D, QtQuickWidgets" 2>/dev/null; then
  "$BUILD_PYTHON" -m pip install -r requirements-build.txt
fi
"$BUILD_PYTHON" tools/build_app_icons.py
"$BUILD_PYTHON" tools/build_native_core.py --target host
"$BUILD_PYTHON" -m PyInstaller --noconfirm --clean StormworksPhysicsShapeOptimizer.spec

APP_PATH="$PROJECT_DIR/dist/Stormworks Physics Shape Optimizer.app"
EXECUTABLE="$APP_PATH/Contents/MacOS/StormworksPhysicsShapeOptimizer"
SWPHYSICS_REQUIRE_NATIVE=1 "$EXECUTABLE" --self-test
QSG_RHI_BACKEND=metal "$EXECUTABLE" --gpu-self-test
QT_QPA_PLATFORM=offscreen "$EXECUTABLE" --worker-self-test
QT_QPA_PLATFORM=offscreen "$EXECUTABLE" --parallel-self-test
QT_QPA_PLATFORM=offscreen "$BUILD_PYTHON" tools/qt_ui_smoke.py \
  --output-dir "$PROJECT_DIR/build/ui-smoke"

if (( DIST_ONLY )); then
    print "Built test app only: $APP_PATH"
    exit 0
fi

mkdir -p "$PROJECT_DIR/release"
RELEASE_DIR="$PROJECT_DIR/release/Stormworks Physics Shape Optimizer 1.2.0 Alpha macOS"
mkdir -p "$RELEASE_DIR"
ditto "$APP_PATH" "$RELEASE_DIR/Stormworks Physics Shape Optimizer.app"
cp "$PROJECT_DIR/APP_README.md" "$RELEASE_DIR/APP_README.md"
cp "$PROJECT_DIR/APP_README_EN.md" "$RELEASE_DIR/APP_README_EN.md"
cp "$PROJECT_DIR/LICENSE" "$RELEASE_DIR/LICENSE"
cp "$PROJECT_DIR/THIRD_PARTY_NOTICES.md" "$RELEASE_DIR/THIRD_PARTY_NOTICES.md"
cp "$PROJECT_DIR/SOURCE_CODE.md" "$RELEASE_DIR/SOURCE_CODE.md"
rm -rf "$RELEASE_DIR/LICENSES"
cp -R "$PROJECT_DIR/LICENSES" "$RELEASE_DIR/LICENSES"
find "$RELEASE_DIR" -maxdepth 1 -iname 'RELEASE_NOTES*.txt' -delete
ditto -c -k --norsrc --noextattr --keepParent "$RELEASE_DIR" \
  "$PROJECT_DIR/release/Stormworks Physics Shape Optimizer 1.2.0 Alpha macOS.zip"
shasum -a 256 "$PROJECT_DIR/release/Stormworks Physics Shape Optimizer 1.2.0 Alpha macOS.zip"

print "Built: $APP_PATH"
