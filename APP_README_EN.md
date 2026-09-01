# Stormworks Physics Shape Optimizer V1.2.1 Alpha

Author: **IrisNuiYaMa_164**

Stormworks Physics Shape Optimizer analyzes a Stormworks vehicle XML file, searches for a Component order that can reduce its Physics Shape count, and saves the result as an optimized copy.

The original vehicle XML is never overwritten. Block settings, logic connections, microcontrollers, Component MOD data, and the vehicle's original Author information are not changed.

## Starting the App

### macOS

1. Open `Stormworks Physics Shape Optimizer.app`.
2. If macOS blocks the app the first time you run it, Control-click the app in Finder and select **Open**.

The macOS release is for Apple Silicon Macs.

### Windows

1. Extract the ZIP archive.
2. Run `StormworksPhysicsShapeOptimizer.exe`.

If Windows displays a security warning, check the download source and filename before running the app.

## Display Language

Use the language menu in the upper-right corner to select `日本語` or `English`. The app remembers your selection for the next launch.

## Basic Use

### 1. Select a Vehicle XML File

Use either of these methods:

- Click `選択…` (Browse…) next to `車両XML` (Vehicle XML).
- Drag and drop a vehicle XML file from Finder or File Explorer onto the app window.

Stormworks normally stores vehicle files in these folders:

- macOS: `~/Library/Application Support/Stormworks/data/vehicles`
- Windows: `%APPDATA%\Stormworks\data\vehicles`

### 2. Check the Definitions Folder

The Definitions path is filled in automatically when Stormworks is installed in its usual location. If the field is empty, click `選択…` (Browse…) next to `Definitions` and select the game's `rom/data/definitions` folder.

### 3. Choose the Analysis Settings

There are three search modes:

- `標準（高速）` (Standard/Fast): Best for getting an initial result quickly.
- `深掘り（推奨）` (Deep/Recommended): Spends more time looking for further reductions.
- `徹底` (Thorough): Intended for long searches on large vehicles.

For most vehicles, leave CPU Workers set to `自動（推奨）` (Auto/Recommended).

- Select `1（省メモリ）` (1/Low Memory) to reduce memory use.
- Select `2`, `4`, or `8` if you want to choose the number of parallel workers manually.

Increasing the worker count does not make every vehicle faster.

### 4. Analyze the Vehicle

Click `解析する` (Analyze). The app shows its current task next to the progress bar.

To cancel an analysis in progress, click `解析を停止` (Stop Analysis). Canceling does not change the original vehicle XML.

When the analysis finishes, the app displays:

- Predicted current F2 Shape count
- Predicted F2 Shape count after optimization
- Component count
- Predicted number of Shapes that can be removed

Known Physics blocks transformed through XML scaling, reflection, shearing, or similar edits are included in the analysis, including Physics Flooder watertight Surface evaluation. If an unsupported Component is present, the app protects its order and optimizes the remaining Components only within safe boundaries.

### 5. Check the 3D Preview

Switch between `現在` (Current) and `最適化後` (Optimized) to compare the Shapes. Use `表示範囲` (Display Range) to show all Bodies or one individual Body.

Preview controls:

- Left-drag: Rotate
- Mouse wheel: Zoom
- Double-click: Reset the view
- `視点リセット` (Reset View): Fit the complete vehicle in the preview

When the pointer is outside the preview, the mouse wheel scrolls the app window up and down.

### 6. Save an Optimized Copy

Click `最適化コピーを保存…` (Save Optimized Copy…) and save the result under a different name from the original XML.

Example:

`MyVehicle Optimized.xml`

Saving uses the completed analysis and does not run the analysis again. If the vehicle uses Component MOD files, the app copies them for the saved vehicle when required.

### 7. Verify the Result in Stormworks

Load the saved copy in Stormworks, enable the F2 Physics display, and check the Shape count and placement.

The numbers shown by the app are predictions. Use the in-game F2 display as the final result.

## Troubleshooting

### The Definitions Folder Is Not Found

Manually select the `rom/data/definitions` folder inside your Stormworks installation. Automatic detection may not find the game when it is installed in another Steam library.

### File Access Is Denied

Copy the vehicle XML to a folder you can write to, such as Documents, and then try analyzing and saving it from there. Save the result under a name different from the original XML.

### Analysis Is Slow or Uses Too Much Memory

Start with `標準（高速）` (Standard/Fast) and CPU Workers set to `1（省メモリ）` (1/Low Memory). For a large vehicle, selecting an individual Body under `表示範囲` (Display Range) may also make the 3D preview lighter.

### The Vehicle or Settings Changed After Analysis

Click `解析する` (Analyze) again. The app will not save an old analysis result if the vehicle contents have changed since that analysis.

### The App and the In-Game Display Do Not Match

Make sure the selected Definitions folder belongs to your current Stormworks installation, then analyze the vehicle again. If the result still differs, report it with the original vehicle and information from the in-game F2 display.

## What Saving Does Not Change

- The original vehicle XML
- Vehicle Author information
- Component properties
- Logic connections
- Microcontroller contents or dimensions
- Component MOD settings
- Positions of Components without Physics

Only the Component order in the optimized copy is changed.

## Version Information

- App: Stormworks Physics Shape Optimizer
- Version: 1.2.1 Alpha
- Author: IrisNuiYaMa_164

## License and Source Code

Copyright (c) 2026 IrisNuiYaMa_164

This software is distributed under the GNU General Public License version 3
(`GPL-3.0-only`) and comes with no warranty. See the included `LICENSE` for the
complete license and `THIRD_PARTY_NOTICES.md` for third-party software notices.

Corresponding source code:
https://github.com/okamuru/stormworks-physics-shape-optimizer
