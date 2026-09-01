# Stormworks Physics Shape Optimizer

Author: **IrisNuiYaMa_164**

[English instructions](APP_README_EN.md)

[Downloads](https://github.com/okamuru/stormworks-physics-shape-optimizer/releases)

[更新履歴 / Changelog](CHANGELOG.md)

Stormworksの車両XMLを解析し、Physics Shapeを減らせるComponent順序を探して、元ファイルとは別の最適化コピーを保存するアプリです。

## 使い方

1. 車両XMLを選択するか、アプリへドラッグ＆ドロップします。
2. Definitionsが空欄なら、Stormworksの `rom/data/definitions` を選びます。
3. 探索モードとCPUワーカーを選びます。迷った場合は「標準（高速）＋自動（推奨）」を使います。
4. 「解析する」を押します。
5. 「現在」と「最適化後」の3Dプレビューを確認します。
6. 「最適化コピーを保存…」から別名のXMLを保存します。
7. Stormworksでコピーを読み込み、F2のPhysics表示を確認します。

詳しい操作方法とトラブル対処は [APP_README.md](APP_README.md) を参照してください。

## 大切な点

- 元の車両XMLは上書きしません。
- 車両の作者情報は変更しません。
- Componentの設定、配線、マイコン、Component MODの内容は変更しません。
- 最終的なShapeの確認はゲーム内F2表示で行ってください。
- XMLで変形された既知のPhysicsブロックも解析・最適化できます。
- 対応外のComponentは元の位置関係を保ち、それ以外を安全な範囲で最適化します。

## ソースから起動する

必要なPython環境を用意した後、次を実行します。

```text
.venv-build/bin/python launch_app.py
```

配布用ビルド:

```text
./build_macos.command
```

Windowsでは `build_windows.bat` を実行します。

詳しいビルド方法は [BUILDING.md](BUILDING.md) を参照してください。

## ライセンス

Copyright (c) 2026 IrisNuiYaMa_164

本ソフトウェアはGNU General Public License version 3
（`GPL-3.0-only`）で公開しています。無保証です。詳しくは
[LICENSE](LICENSE) と [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) を
参照してください。

## バージョン情報

- バージョン: 1.2.0 Alpha
- 作者: IrisNuiYaMa_164
