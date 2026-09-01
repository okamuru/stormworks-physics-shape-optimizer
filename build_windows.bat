@echo off
setlocal
cd /d "%~dp0"
set "DIST_ONLY=0"
if /I "%~1"=="--dist-only" set "DIST_ONLY=1"
set "PYINSTALLER_CONFIG_DIR=%CD%\build\pyinstaller-cache-windows"
set "PYTHONPYCACHEPREFIX=%CD%\build\python-cache-windows"

if not exist ".venv-build-windows\Scripts\python.exe" (
  where py >nul 2>&1
  if not errorlevel 1 (
    py -3 -m venv .venv-build-windows
  ) else if exist "C:\Python311\python.exe" (
    "C:\Python311\python.exe" -m venv .venv-build-windows
  ) else (
    python -m venv .venv-build-windows
  )
  if errorlevel 1 exit /b 1
)

set "BUILD_PYTHON=.venv-build-windows\Scripts\python.exe"
"%BUILD_PYTHON%" -c "import PIL, PyInstaller; from PySide6 import QtQuick3D, QtQuickWidgets" >nul 2>&1
if errorlevel 1 (
  "%BUILD_PYTHON%" -m pip install -r requirements-build.txt
  if errorlevel 1 exit /b 1
)
"%BUILD_PYTHON%" tools\build_app_icons.py
if errorlevel 1 exit /b 1
if "%SWPHYSICS_PREBUILT_NATIVE%"=="1" (
  if not exist "swphysics\native\swphysics_native.dll" (
    echo Missing prebuilt swphysics\native\swphysics_native.dll
    exit /b 1
  )
) else (
  "%BUILD_PYTHON%" tools\build_native_core.py --target host
  if errorlevel 1 exit /b 1
)
"%BUILD_PYTHON%" -m PyInstaller --noconfirm --clean StormworksPhysicsShapeOptimizer.spec
if errorlevel 1 exit /b 1

set "SWPHYSICS_REQUIRE_NATIVE=1"
"dist\StormworksPhysicsShapeOptimizer.exe" --self-test
if errorlevel 1 exit /b 1
set "SWPHYSICS_REQUIRE_NATIVE="
if "%SWPHYSICS_WINE_BUILD%"=="1" set "QSG_RHI_BACKEND=vulkan"
"dist\StormworksPhysicsShapeOptimizer.exe" --gpu-self-test
if errorlevel 1 exit /b 1
set "QSG_RHI_BACKEND="
set "QT_QPA_PLATFORM=offscreen"
"dist\StormworksPhysicsShapeOptimizer.exe" --worker-self-test
if errorlevel 1 exit /b 1
"dist\StormworksPhysicsShapeOptimizer.exe" --parallel-self-test
if errorlevel 1 exit /b 1
"%BUILD_PYTHON%" tools\qt_ui_smoke.py --output-dir build\ui-smoke
if errorlevel 1 exit /b 1
set "QT_QPA_PLATFORM="

if "%DIST_ONLY%"=="1" goto build_done

if not exist release mkdir release
set "RELEASE_DIR=release\Stormworks Physics Shape Optimizer 1.2.0 Alpha Windows"
if not exist "%RELEASE_DIR%" mkdir "%RELEASE_DIR%"
if exist "%RELEASE_DIR%\RELEASE_NOTES*.txt" del /Q "%RELEASE_DIR%\RELEASE_NOTES*.txt"
copy /Y "dist\StormworksPhysicsShapeOptimizer.exe" "%RELEASE_DIR%\StormworksPhysicsShapeOptimizer.exe" >nul
copy /Y "APP_README.md" "%RELEASE_DIR%\APP_README.md" >nul
copy /Y "APP_README_EN.md" "%RELEASE_DIR%\APP_README_EN.md" >nul
copy /Y "LICENSE" "%RELEASE_DIR%\LICENSE" >nul
copy /Y "THIRD_PARTY_NOTICES.md" "%RELEASE_DIR%\THIRD_PARTY_NOTICES.md" >nul
copy /Y "SOURCE_CODE.md" "%RELEASE_DIR%\SOURCE_CODE.md" >nul
if exist "%RELEASE_DIR%\LICENSES" rmdir /S /Q "%RELEASE_DIR%\LICENSES"
xcopy /E /I /Y "LICENSES" "%RELEASE_DIR%\LICENSES" >nul
"%BUILD_PYTHON%" -c "import shutil; shutil.make_archive(r'release\Stormworks Physics Shape Optimizer 1.2.0 Alpha Windows', 'zip', root_dir='release', base_dir=r'Stormworks Physics Shape Optimizer 1.2.0 Alpha Windows')"
if errorlevel 1 exit /b 1
"%BUILD_PYTHON%" -c "from pathlib import Path; import hashlib; p=Path(r'release\Stormworks Physics Shape Optimizer 1.2.0 Alpha Windows.zip'); print(hashlib.sha256(p.read_bytes()).hexdigest(), p)"
if errorlevel 1 exit /b 1

:build_done
echo Built: dist\StormworksPhysicsShapeOptimizer.exe
endlocal
