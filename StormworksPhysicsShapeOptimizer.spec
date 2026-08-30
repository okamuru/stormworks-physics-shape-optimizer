from pathlib import Path
import sys


project_root = Path(SPECPATH)
is_macos = sys.platform == "darwin"
icon_path = project_root / "assets" / ("app_icon.icns" if is_macos else "app_icon.ico")
native_filename = (
    "libswphysics_native.dylib" if is_macos else "swphysics_native.dll"
)
native_path = project_root / "swphysics" / "native" / native_filename
windows_version_path = project_root / "assets" / "windows_version_info.txt"
if not native_path.is_file():
    raise RuntimeError("required native score engine is missing: {}".format(native_path))

a = Analysis(
    [str(project_root / "launch_app.py")],
    pathex=[str(project_root)],
    binaries=[(str(native_path), "swphysics/native")],
    datas=[
        (str(project_root / "assets" / "app_icon.png"), "assets"),
        (str(project_root / "assets" / "gpu_viewer.qml"), "assets"),
        (str(project_root / "LICENSE"), "."),
        (str(project_root / "THIRD_PARTY_NOTICES.md"), "."),
        (str(project_root / "SOURCE_CODE.md"), "."),
        (str(project_root / "LICENSES"), "LICENSES"),
        (
            str(project_root / "analysis" / "surface_table_build_24749959.json"),
            "analysis",
        ),
        (
            str(project_root / "analysis" / "surface_resolution_build_24749959.json"),
            "analysis",
        ),
    ],
    hiddenimports=[],
    hookspath=[str(project_root / "pyinstaller_hooks")],
    hooksconfig={},
    runtime_hooks=[],
    excludes=["swphysics.binary_oracle", "unicorn"],
    noarchive=False,
)
pyz = PYZ(a.pure)

if is_macos:
    exe = EXE(
        pyz,
        a.scripts,
        [],
        exclude_binaries=True,
        name="StormworksPhysicsShapeOptimizer",
        debug=False,
        bootloader_ignore_signals=False,
        strip=False,
        upx=False,
        console=False,
        disable_windowed_traceback=False,
        argv_emulation=False,
        target_arch=None,
        codesign_identity=None,
        entitlements_file=None,
        icon=str(icon_path),
        version=None,
    )
    collected = COLLECT(
        exe,
        a.binaries,
        a.datas,
        strip=False,
        upx=False,
        name="StormworksPhysicsShapeOptimizer",
    )
    app = BUNDLE(
        collected,
        name="Stormworks Physics Shape Optimizer.app",
        icon=str(icon_path),
        bundle_identifier="com.irisnuiyama164.stormworks-physics-shape-optimizer",
        info_plist={
            "CFBundleDisplayName": "Stormworks Physics Shape Optimizer",
            "CFBundleGetInfoString": "Stormworks Physics Shape Optimizer 1.0.0 Alpha — IrisNuiYaMa_164",
            "CFBundleShortVersionString": "1.0.0",
            "CFBundleVersion": "1000",
            "LSMinimumSystemVersion": "12.0",
            "NSHighResolutionCapable": True,
            "NSHumanReadableCopyright": "Copyright © 2026 IrisNuiYaMa_164",
            "NSRequiresAquaSystemAppearance": False,
        },
    )
else:
    exe = EXE(
        pyz,
        a.scripts,
        a.binaries,
        a.datas,
        [],
        name="StormworksPhysicsShapeOptimizer",
        debug=False,
        bootloader_ignore_signals=False,
        strip=False,
        upx=False,
        console=False,
        disable_windowed_traceback=False,
        icon=str(icon_path),
        version=str(windows_version_path),
    )
