# Changelog

## V1.2.1 Alpha — 2026-09-01

### 日本語

#### 修正

- 標準ドアで閉じられた空間をPhysics Flooderが開放空間として扱い、ゲーム内より多いPhysics Shape数を予測する問題を修正しました。

### English

#### Fixed

- Fixed Physics Flooder treating spaces sealed by built-in doors as open, which could predict more Physics Shapes than the game generates.

## V1.2.0 Alpha — 2026-09-01

### 日本語

#### 主な追加・改善

- XML編集ブロックのPhysics Shape解析・最適化に全面対応しました。整数行列による拡大・縮小、軸反転、せん断、軸を0にした変形を含む既知のPhysicsブロックを処理できます。
- XML編集されたウェッジなどについて、変形後のclip plane、ボクセル間隔、結合判定をStormworksの挙動に合わせました。
- Physics Flooderの水密Surface判定もXML整数変形に対応しました。
- XML編集ShapeもRustネイティブ評価の対象となり、大型のXML編集車両でも高速な探索経路を利用できます。
- 対応外Componentを順序境界として保持し、対応Componentがその境界を越えない範囲で最適化するよう改善しました。

#### 修正

- XML編集された非立方Shapeで、ゲーム内では生成されない退化したconvex shapeがShape数や3Dプレビューへ含まれる問題を修正しました。
- XML編集Shapeの結合時に、一部の隣接ボクセルが誤って分割される問題を修正しました。
- 3Dプレビューで車両の前後方向が逆に表示される問題を修正しました。

### English

#### Added and improved

- Added comprehensive Physics Shape analysis and optimization support for XML-edited blocks, including integer scaling, axis reflection, shearing, and transforms with a zeroed axis for known Physics blocks.
- Matched transformed clip planes, voxel spacing, and merge behavior for XML-edited non-cube blocks such as wedges to Stormworks.
- Added XML integer-transform support to Physics Flooder watertight Surface evaluation.
- Extended the Rust native evaluator to XML-edited Shapes so large edited vehicles can continue to use the accelerated search path.
- Unsupported Components now remain as ordering barriers, allowing supported Components to be optimized without crossing them.

#### Fixed

- Fixed degenerate convex Shapes from XML-edited non-cube blocks being counted or shown even though the game does not generate them.
- Fixed some adjacent XML-edited Shapes being split into separate groups incorrectly.
- Fixed the 3D preview showing the vehicle's fore-aft direction reversed.
