# Stormworks Physics Shape Optimizer V1.1.0 Alpha

Author: **IrisNuiYaMa_164**

Stormworksの車両XMLを解析し、Physics Shapeを減らせるComponent順序を探して、最適化コピーを保存するアプリです。

元の車両XMLは上書きしません。ブロックの設定、配線、マイコン、Component MOD、車両の作者情報も変更しません。

## 起動方法

### macOS

1. `Stormworks Physics Shape Optimizer.app` を開きます。
2. 初回にmacOSが起動を止めた場合は、FinderでアプリをControlクリックして「開く」を選びます。

配布物はApple Silicon Mac向けです。

### Windows

1. ZIPを展開します。
2. `StormworksPhysicsShapeOptimizer.exe` を起動します。

Windowsの警告が出た場合は、配布元とファイル名を確認してから実行してください。

## 表示言語

画面右上の「言語」から「日本語」または「English」を選べます。選択した言語は次回起動時にも引き継がれます。

## 基本の使い方

### 1. 車両XMLを選ぶ

次のどちらかで選択できます。

- 「車両XML」の「選択…」を押す
- FinderまたはExplorerから車両XMLをウィンドウへドラッグ＆ドロップする

通常の保存場所は次の通りです。

- macOS: `~/Library/Application Support/Stormworks/data/vehicles`
- Windows: `%APPDATA%\Stormworks\data\vehicles`

### 2. Definitionsを確認する

Stormworksが通常の場所にインストールされていれば自動で入力されます。空欄の場合は「Definitions」の「選択…」から、Stormworks内の `rom/data/definitions` フォルダを選んでください。

### 3. 解析設定を選ぶ

探索モードは次の3種類です。

- 「標準（高速）」: まず結果を確認したい時向け
- 「深掘り（推奨）」: 時間をかけてもう少し削減したい時向け
- 「徹底」: 大型車両を長時間かけて調べる時向け

CPUワーカーは通常「自動（推奨）」で構いません。

- メモリ使用量を抑えたい場合は「1（省メモリ）」
- 手動で並列数を試したい場合は「2」「4」「8」

ワーカー数を増やしても、車両によっては速くならない場合があります。

### 4. 解析する

「解析する」を押します。進捗バーの横に現在の作業内容が表示されます。

途中で止めたい場合は「解析を停止」を押してください。停止しても元の車両XMLは変更されません。

解析が終わると、次の情報が表示されます。

- 現在のF2予測Shape数
- 最適化後のF2予測Shape数
- Component数
- 予測できるShape削減数

### 5. 3Dプレビューを確認する

「現在」と「最適化後」を切り替えて、Shapeの変化を確認できます。「表示範囲」から全Bodyまたは個別Bodyを選べます。

操作方法:

- 左ドラッグ: 回転
- マウスホイール: ズーム
- ダブルクリック: 視点リセット
- 「視点リセット」ボタン: 全体が見える位置へ戻す

プレビュー外でホイールを回すと、アプリ画面を上下へスクロールします。

### 6. 最適化コピーを保存する

「最適化コピーを保存…」を押して、元のXMLとは別の名前で保存します。

おすすめの例:

`MyVehicle Optimized.xml`

解析済みの結果を使うため、保存時に解析をやり直すことはありません。使用しているComponent MODのファイルがある場合は、保存先の車両に合わせてコピーされます。

### 7. ゲーム内で確認する

保存したコピーをStormworksで読み込み、F2のPhysics表示でShape数と配置を確認してください。

アプリの数字は予測結果です。最終確認はゲーム内表示を基準にしてください。

## よくある困りごと

### Definitionsが見つからない

Stormworksのインストール先から `rom/data/definitions` を手動で選択してください。ゲームを別のSteamライブラリへ入れている場合は、自動検出されないことがあります。

### ファイルへのアクセスが拒否される

車両XMLをいったん「ドキュメント」など自分が書き込める場所へコピーし、そこから解析と保存を試してください。保存先には元のXMLと違う名前を指定してください。

### 解析が重い、またはメモリ使用量が多い

まず「標準（高速）」とCPUワーカー「1（省メモリ）」を試してください。3Dプレビューは「表示範囲」から個別Bodyを選ぶと軽くなる場合があります。

### 解析後に設定や車両を変更した

もう一度「解析する」を押してください。解析前と内容が変わった車両には、古い結果を保存できません。

### アプリとゲーム内表示が違う

選択したDefinitionsが現在のStormworksと同じものか確認し、再解析してください。それでも違う場合は、元車両とゲーム内F2表示の情報を添えて報告してください。

## 保存時に変更しないもの

- 元の車両XML
- 車両の作者情報
- Componentのプロパティ
- ロジック配線
- マイコンの内容と大きさ
- Component MODの設定
- Physicsを持たないComponentの位置

変更するのは、最適化コピー内のComponent順序だけです。

## バージョン情報

- アプリ: Stormworks Physics Shape Optimizer
- バージョン: 1.1.0 Alpha
- 作者: IrisNuiYaMa_164

## ライセンスとソースコード

Copyright (c) 2026 IrisNuiYaMa_164

本ソフトウェアはGNU General Public License version 3
（`GPL-3.0-only`）で公開されており、無保証です。ライセンス全文は
同梱の `LICENSE`、第三者ソフトウェアについては
`THIRD_PARTY_NOTICES.md` を参照してください。

対応ソースコード:
https://github.com/okamuru/stormworks-physics-shape-optimizer
