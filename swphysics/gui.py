import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import sys
from threading import Event
import traceback
from typing import Callable, Optional, Sequence, Tuple

from PySide6.QtCore import (
    QEvent,
    QEventLoop,
    QItemSelectionModel,
    QObject,
    QRunnable,
    QThreadPool,
    QTimer,
    Qt,
    QUrl,
    Signal,
    Slot,
    qVersion,
)
from PySide6.QtGui import (
    QBrush,
    QColor,
    QDesktopServices,
    QDragEnterEvent,
    QDropEvent,
    QFont,
    QIcon,
    QKeyEvent,
    QKeySequence,
    QPixmap,
    QMouseEvent,
    QShortcut,
)
from PySide6.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLayout,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QSlider,
    QSizePolicy,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from . import __author__
from .app_service import (
    APP_VERSION,
    SEARCH_MODE_PROFILES,
    OptimizationOutput,
    VehicleAnalysis,
    apply_manual_body_exclusions,
    analyze_vehicle,
    save_analyzed_vehicle_copy,
    search_mode_profile,
)
from .platform_paths import find_definition_directory, load_config, save_config, vehicle_directory
from .native_merge import native_backend_status
from .physics_viewer import PhysicsShapeViewer
from .viewer import BodyRenderGroup


APP_NAME = "Stormworks Physics Shape Optimizer"
APP_AUTHOR = __author__

DEFAULT_LANGUAGE = "ja"
SUPPORTED_LANGUAGES = ("ja", "en")

BODY_COLORS = (
    "#22A98D",
    "#E4782F",
    "#8158D0",
    "#368DD2",
    "#D24786",
    "#C19B16",
    "#D14E45",
    "#4D9B58",
    "#506BC7",
    "#A95DA8",
)


@dataclass(frozen=True)
class BodyManagementSnapshot:
    selected_body_ids: Tuple[int, ...]
    hidden_body_ids: Tuple[int, ...]
    manually_excluded_body_ids: Tuple[int, ...]
    selection_anchor_id: Optional[int]


@dataclass(frozen=True)
class BodyHistoryEntry:
    label_key: str
    snapshot: BodyManagementSnapshot


_TEXT = {
    "ja": {
        "vehicle_placeholder": "Stormworks車両XMLを選択、またはここへドロップ",
        "not_analyzed": "未解析",
        "initial_status": "車両XMLを選んで解析してください",
        "subtitle": "全physics shapeとPhysics Flooderを解析し、元XMLを残したままShape数を減らします",
        "language": "言語",
        "author": "Author: {author}",
        "files": "ファイル",
        "vehicle_xml": "車両XML",
        "definitions": "Definitions",
        "select": "選択…",
        "analysis_settings": "解析設定",
        "search_mode": "探索モード",
        "cpu_workers": "CPUワーカー",
        "analyze": "解析する",
        "save_optimized": "最適化コピーを保存…",
        "open_vehicle_folder": "車両フォルダを開く",
        "starting_analysis": "解析を開始しています…",
        "stop_analysis": "解析を停止",
        "analysis_results": "解析結果",
        "current_metric": "現在のF2予測Shape",
        "optimized_metric": "最適化後のF2予測",
        "component_metric": "Component数",
        "display": "表示:",
        "current": "現在",
        "optimized": "最適化後",
        "display_scope": "表示範囲:",
        "manual_body_exclusion": "選択Bodyを最適化しない",
        "manual_excluded_suffix": "（手動除外）",
        "manual_exclusion_note": " / 手動除外 {bodies} Body",
        "manual_exclusion_status_on": "Body {body}を手動除外しました（再解析なし）",
        "manual_exclusion_status_off": "Body {body}を最適化対象へ戻しました（再解析なし）",
        "preview_mode": "モード:",
        "shape_check_mode": "Shape確認",
        "body_management_mode": "Body管理",
        "body_selection_none": "Body未選択・全体表示",
        "body_selection_one": "Body {body}選択　{current} → {optimized} Shapes",
        "body_selection_many": "{count} Body選択　{current} → {optimized} Shapes",
        "body_layers": "Bodyレイヤー",
        "body_list_count": "{listed}/{total}件・{selected}選択",
        "select_filtered": "絞り込み結果を選択",
        "invert_filtered": "絞り込み結果を反転",
        "ghost_unselected": "選択以外をゴースト",
        "undo": "元に戻す",
        "redo": "やり直す",
        "reset_body_state": "表示/除外を初期化",
        "body_filter_all": "すべて",
        "body_filter_selected": "選択中",
        "body_filter_visible": "表示中",
        "body_filter_hidden": "非表示",
        "body_filter_included": "最適化対象",
        "body_filter_excluded": "最適化対象外",
        "body_filter_zero": "削減0",
        "body_filter_warning": "警告あり",
        "body_sort_id": "Body番号",
        "body_sort_current": "現在Shape",
        "body_sort_optimized": "最適化後Shape",
        "body_sort_reduction": "削減数",
        "body_sort_components": "Component数",
        "body_sort_warning": "警告レベル",
        "body_sort_visibility": "表示状態",
        "body_sort_optimization": "最適化状態",
        "sort_ascending": "昇順 ↑",
        "sort_descending": "降順 ↓",
        "ghost_opacity": "ゴースト不透明度",
        "overview_opacity": "未選択時の全体不透明度",
        "bulk_summary": "{count} Body選択・表示 {visible}/{count}・最適化 {optimized}/{count}",
        "show_selected": "表示",
        "hide_selected": "非表示",
        "optimize_selected": "最適化",
        "exclude_selected": "除外",
        "body_column_display": "表示",
        "body_column_body": "Body",
        "body_column_details": "Shape/C",
        "body_column_optimize": "最適化",
        "body_fixed": "固定",
        "body_row_stats": "{current}→{optimized} / C{components}",
        "body_warning_zero": "削減0",
        "body_warning_xml": "XML編集 {count}",
        "body_warning_flooder": "Flooder予測外",
        "body_warning_unsupported": "変更対象外",
        "body_management_scope": "Body管理",
        "no_matching_bodies": "条件に一致するBodyはありません",
        "shortcut_help": "操作ヒント・ショートカット一覧",
        "shortcut_selection_title": "選択",
        "shortcut_view_title": "表示・視点",
        "shortcut_optimization_title": "最適化・履歴",
        "shortcut_click_action": "単独選択／同じBodyで解除",
        "shortcut_enter_action": "一覧のBodyを選択／解除",
        "shortcut_add_action": "選択を追加／解除",
        "shortcut_range_action": "一覧で範囲選択",
        "shortcut_select_all_action": "全Bodyを選択",
        "shortcut_clear_action": "選択を解除",
        "shortcut_cycle_action": "重なったBodyを順番に選択",
        "shortcut_rotate_action": "3D視点を回転",
        "shortcut_zoom_action": "ズーム",
        "shortcut_focus_action": "選択Bodyへフォーカス",
        "shortcut_background_action": "選択を解除",
        "shortcut_hide_action": "選択Bodyを隠す",
        "shortcut_isolate_action": "選択以外を隠す",
        "shortcut_show_all_action": "全Bodyを表示",
        "shortcut_toggle_optimize_action": "選択Bodyの最適化を切替",
        "shortcut_undo_action": "元に戻す",
        "shortcut_redo_action": "やり直す",
        "shortcut_group_action": "複数選択中は全選択Bodyへ適用",
        "shortcut_hover_action": "一覧と3Dの同じBodyを強調",
        "reset_view": "視点リセット",
        "preview_tab": "3Dプレビュー",
        "details_placeholder": "解析結果の詳細がここに表示されます",
        "details_tab": "詳細ログ",
        "interaction_hint": "左ドラッグ: 回転 / ホイール: ズーム / ダブルクリック: リセット",
        "body_interaction_hint": "左ドラッグ: 回転 / ホイール: ズーム / 背景クリック: 選択解除",
        "empty_preview": "解析するとPhysics Shapeがここに表示されます",
        "choose_vehicle": "Stormworks車両XMLを選択",
        "choose_definitions": "Stormworks rom/data/definitionsを選択",
        "vehicle_filter": "Stormworks vehicle XML (*.xml);;All files (*)",
        "save_vehicle_filter": "Stormworks vehicle XML (*.xml)",
        "choose_output": "最適化コピーの保存先",
        "all_bodies": "全Body ({count})",
        "all_bodies_scope": "全Body",
        "auto_worker": "自動（推奨）",
        "one_worker": "1（省メモリ）",
        "worker_hint": "{description}　/　CPUワーカー {workers}",
        "search_standard_label": "標準（高速）",
        "search_standard_description": "64評価・1段階",
        "search_deep_label": "深掘り（推奨）",
        "search_deep_description": "最大3段階・64→128評価・改善停止で自動終了",
        "search_thorough_label": "徹底",
        "search_thorough_description": "最大6段階・128→256評価・改善停止で自動終了",
        "partial_suffix": " + 予測対象外",
        "out_of_scope": "対象外",
        "auto": "自動",
        "warnings": "警告:",
        "vehicle_log": "車両: {path}",
        "settings_log": "探索モード: {mode} / CPUワーカー設定: {workers}",
        "engine_log": "Shape評価エンジン: {engine}",
        "preview_excluded": " / 予測対象外{components} Component・Flooder {flooders} Body（{bodies} Body内の残りを表示）",
        "preview_manual_excluded": " / {bodies} Bodyを手動除外",
        "preview_caption": "{scope} / {state} / {count} Shapes{excluded}　　{hint}",
        "eligibility_full": "最適化可能：予測で{reduction} Shape削減",
        "eligibility_partial": "部分最適化可能：対応範囲で{reduction} Shape削減 / 予測対象外{components} ComponentとFlooder {flooders} Bodyは元位置に保持し、{bodies} Body内の残りを最適化",
        "eligibility_none": "変更対象外：{reasons}",
        "no_supported_body": "対応bodyなし",
        "saved_title": "保存完了",
        "saved_output": "出力: {path}",
        "predicted_shapes": "予測Shape: {before} → {after}",
        "saved_validation": "XMLを再読込し、Component順序と構造を検証済みです。",
        "saved_cached": "解析済み結果を再利用したため、保存時の再解析はありません。",
        "saved_verify_game": "実ゲームでのShape境界はF2表示で確認してください。",
        "saved_partial": "\n予測対象外{components} ComponentとFlooder {flooders} Bodyは元のスロットを保持し、{bodies} Body内の残りだけを最適化しました。最終結果はゲーム内F2で確認してください。",
        "saved_manual": "\n手動除外した{bodies} Bodyは、元のComponent順序を完全に保持しました。",
        "saved_message": "最適化コピーを保存しました。\n\n予測対象: {before} → {after} Shapes\n{path}{partial}",
        "status_percent": "{message}（{percent}%）",
        "config_save_skipped": "設定保存を省略しました: {error}",
    },
    "en": {
        "vehicle_placeholder": "Select a Stormworks vehicle XML or drop it here",
        "not_analyzed": "Not analyzed",
        "initial_status": "Select a vehicle XML to begin analysis",
        "subtitle": "Analyze all physics shapes and Physics Flooders, then reduce Shapes while preserving the original XML",
        "language": "Language",
        "author": "Author: {author}",
        "files": "Files",
        "vehicle_xml": "Vehicle XML",
        "definitions": "Definitions",
        "select": "Browse…",
        "analysis_settings": "Analysis Settings",
        "search_mode": "Search Mode",
        "cpu_workers": "CPU Workers",
        "analyze": "Analyze",
        "save_optimized": "Save Optimized Copy…",
        "open_vehicle_folder": "Open Vehicle Folder",
        "starting_analysis": "Starting analysis…",
        "stop_analysis": "Stop Analysis",
        "analysis_results": "Analysis Results",
        "current_metric": "Current Predicted F2 Shapes",
        "optimized_metric": "Predicted F2 Shapes After Optimization",
        "component_metric": "Components",
        "display": "View:",
        "current": "Current",
        "optimized": "Optimized",
        "display_scope": "Scope:",
        "manual_body_exclusion": "Do not optimize selected Body",
        "manual_excluded_suffix": " (Manually Excluded)",
        "manual_exclusion_note": " / {bodies} manually excluded Bodies",
        "manual_exclusion_status_on": "Body {body} was manually excluded (no re-analysis)",
        "manual_exclusion_status_off": "Body {body} was returned to optimization (no re-analysis)",
        "preview_mode": "Mode:",
        "shape_check_mode": "Shape Check",
        "body_management_mode": "Body Management",
        "body_selection_none": "No Body selected · showing the complete vehicle",
        "body_selection_one": "Body {body} selected · {current} → {optimized} Shapes",
        "body_selection_many": "{count} Bodies selected · {current} → {optimized} Shapes",
        "body_layers": "Body Layers",
        "body_list_count": "{listed}/{total} · {selected} selected",
        "select_filtered": "Select Filtered",
        "invert_filtered": "Invert Filtered",
        "ghost_unselected": "Ghost Unselected",
        "undo": "Undo",
        "redo": "Redo",
        "reset_body_state": "Reset Visibility/Exclusions",
        "body_filter_all": "All",
        "body_filter_selected": "Selected",
        "body_filter_visible": "Visible",
        "body_filter_hidden": "Hidden",
        "body_filter_included": "Optimization Included",
        "body_filter_excluded": "Optimization Excluded",
        "body_filter_zero": "No Reduction",
        "body_filter_warning": "Warnings",
        "body_sort_id": "Body Number",
        "body_sort_current": "Current Shapes",
        "body_sort_optimized": "Optimized Shapes",
        "body_sort_reduction": "Reduction",
        "body_sort_components": "Components",
        "body_sort_warning": "Warning Level",
        "body_sort_visibility": "Visibility",
        "body_sort_optimization": "Optimization State",
        "sort_ascending": "Ascending ↑",
        "sort_descending": "Descending ↓",
        "ghost_opacity": "Ghost Opacity",
        "overview_opacity": "Opacity With No Selection",
        "bulk_summary": "{count} Bodies selected · visible {visible}/{count} · optimized {optimized}/{count}",
        "show_selected": "Show",
        "hide_selected": "Hide",
        "optimize_selected": "Optimize",
        "exclude_selected": "Exclude",
        "body_column_display": "View",
        "body_column_body": "Body",
        "body_column_details": "Shape/C",
        "body_column_optimize": "Optimize",
        "body_fixed": "Locked",
        "body_row_stats": "{current}→{optimized} / C{components}",
        "body_warning_zero": "No reduction",
        "body_warning_xml": "{count} XML-edited",
        "body_warning_flooder": "Flooder unpredicted",
        "body_warning_unsupported": "No changes",
        "body_management_scope": "Body Management",
        "no_matching_bodies": "No Bodies match the current filter",
        "shortcut_help": "Controls and Keyboard Shortcuts",
        "shortcut_selection_title": "Selection",
        "shortcut_view_title": "View and Camera",
        "shortcut_optimization_title": "Optimization and History",
        "shortcut_click_action": "Select one / click it again to clear",
        "shortcut_enter_action": "Select or clear the focused list Body",
        "shortcut_add_action": "Add to or remove from the selection",
        "shortcut_range_action": "Select a range in the list",
        "shortcut_select_all_action": "Select every Body",
        "shortcut_clear_action": "Clear the selection",
        "shortcut_cycle_action": "Cycle through overlapping Bodies",
        "shortcut_rotate_action": "Rotate the 3D camera",
        "shortcut_zoom_action": "Zoom",
        "shortcut_focus_action": "Focus the selected Bodies",
        "shortcut_background_action": "Clear the selection",
        "shortcut_hide_action": "Hide selected Bodies",
        "shortcut_isolate_action": "Hide unselected Bodies",
        "shortcut_show_all_action": "Show every Body",
        "shortcut_toggle_optimize_action": "Toggle optimization for selected Bodies",
        "shortcut_undo_action": "Undo",
        "shortcut_redo_action": "Redo",
        "shortcut_group_action": "Eye/optimization controls apply to the complete selection",
        "shortcut_hover_action": "Highlight the same Body in the list and 3D view",
        "reset_view": "Reset View",
        "preview_tab": "3D Preview",
        "details_placeholder": "Detailed analysis results will appear here",
        "details_tab": "Detailed Log",
        "interaction_hint": "Left drag: Rotate / Wheel: Zoom / Double-click: Reset",
        "body_interaction_hint": "Left drag: Rotate / Wheel: Zoom / Background click: Clear selection",
        "empty_preview": "Physics Shapes will appear here after analysis",
        "choose_vehicle": "Select a Stormworks Vehicle XML",
        "choose_definitions": "Select Stormworks rom/data/definitions",
        "vehicle_filter": "Stormworks vehicle XML (*.xml);;All files (*)",
        "save_vehicle_filter": "Stormworks vehicle XML (*.xml)",
        "choose_output": "Save Optimized Copy",
        "all_bodies": "All Bodies ({count})",
        "all_bodies_scope": "All Bodies",
        "auto_worker": "Auto (Recommended)",
        "one_worker": "1 (Low Memory)",
        "worker_hint": "{description} / CPU workers {workers}",
        "search_standard_label": "Standard (Fast)",
        "search_standard_description": "64 evaluations · 1 stage",
        "search_deep_label": "Deep (Recommended)",
        "search_deep_description": "Up to 3 stages · 64→128 evaluations · stops when no improvement",
        "search_thorough_label": "Thorough",
        "search_thorough_description": "Up to 6 stages · 128→256 evaluations · stops when no improvement",
        "partial_suffix": " + unpredicted",
        "out_of_scope": "Excluded",
        "auto": "Auto",
        "warnings": "Warnings:",
        "vehicle_log": "Vehicle: {path}",
        "settings_log": "Search mode: {mode} / CPU workers: {workers}",
        "engine_log": "Shape evaluation engine: {engine}",
        "preview_excluded": " / {components} unpredicted Components and Flooders in {flooders} Bodies excluded; showing the supported remainder of {bodies} Bodies",
        "preview_manual_excluded": " / {bodies} Bodies manually excluded",
        "preview_caption": "{scope} / {state} / {count} Shapes{excluded}    {hint}",
        "eligibility_full": "Optimization available: predicted reduction of {reduction} Shapes",
        "eligibility_partial": "Partial optimization available: reduction of {reduction} Shapes in the supported scope / {components} unpredicted Components and Flooders in {flooders} Bodies remain in place while the supported remainder of {bodies} Bodies is optimized",
        "eligibility_none": "No changes available: {reasons}",
        "no_supported_body": "no supported Bodies",
        "saved_title": "Save Complete",
        "saved_output": "Output: {path}",
        "predicted_shapes": "Predicted Shapes: {before} → {after}",
        "saved_validation": "The XML was reloaded and its Component order and structure were verified.",
        "saved_cached": "The analyzed result was reused; the vehicle was not analyzed again while saving.",
        "saved_verify_game": "Verify the final Shape boundaries in-game with the F2 overlay.",
        "saved_partial": "\n{components} unpredicted Components and Flooders in {flooders} Bodies kept their original slots; only the supported remainder of {bodies} Bodies was optimized. Verify the final result in-game with F2.",
        "saved_manual": "\n{bodies} manually excluded Bodies kept their complete original Component order.",
        "saved_message": "The optimized copy was saved.\n\nPredicted supported Shapes: {before} → {after}\n{path}{partial}",
        "status_percent": "{message} ({percent}%)",
        "config_save_skipped": "Could not save settings: {error}",
    },
}


_RUNTIME_EXACT_EN = {
    "車両XMLを選んで解析してください": "Select a vehicle XML to begin analysis",
    "解析を開始しています…": "Starting analysis…",
    "処理中は車両XMLを変更できません": "The vehicle XML cannot be changed while a job is running",
    "車両XMLを読み込みました。解析待ちです": "Vehicle XML loaded; ready to analyze",
    "解析設定が変わりました。もう一度解析してください": "Analysis settings changed; analyze the vehicle again",
    "解析を停止しています…": "Stopping analysis…",
    "解析を停止しました": "Analysis stopped",
    "エラー": "Error",
    "完了": "Complete",
    "解析中…": "Analyzing…",
    "解析完了": "Analysis complete",
    "解析済み結果から保存中…": "Saving from analyzed result…",
    "保存完了（再解析なし）": "Save complete (no re-analysis)",
    "車両XMLを確認中…": "Checking vehicle XML…",
    "Definitionカタログを準備中…": "Preparing Definition catalog…",
    "車両XMLを読み込みました": "Vehicle XML loaded",
    "Component Definitionを読み込み中…": "Loading Component Definitions…",
    "3Dプレビューを生成中…": "Building 3D preview…",
    "解析済み結果と元XMLを確認中…": "Checking analyzed result and source XML…",
    "元XMLのComponent構造を読み込み中…": "Loading source Component structure…",
    "元XMLのコンパクトな書式を保って書き出し中…": "Writing while preserving the compact source XML format…",
    "保存前にXML構造を再読込中…": "Reloading XML structure before saving…",
    "解析済み結果から保存完了（再解析なし）": "Saved from analyzed result (no re-analysis)",
    "保存したXMLをアプリモデルで最終確認中…": "Final-checking the saved XML with the application model…",
    "保存・再検証完了": "Save and revalidation complete",
    "Physics Flooder入力を準備中…": "Preparing Physics Flooder input…",
    "Physics Flooderなし": "No Physics Flooder",
    "buoyancy Surfaceを解決中…": "Resolving buoyancy Surfaces…",
    "未知のbuoyancy Surfaceを検出": "Unknown buoyancy Surface detected",
    "密閉Surfaceなし": "No sealing Surface",
    "native Surface境界を圧縮中…": "Compressing native Surface boundaries…",
    "Flooderサンプル区画を探索中…": "Searching Flooder sample compartments…",
    "native edge crawlと体積走査を実行中…": "Running native edge crawl and volume scan…",
    "Physics Flooder解析完了": "Physics Flooder analysis complete",
    "全physics shape対応のportable exactモデルで最適化できます": "Can be optimized with the all-physics-shape portable exact model",
    "XML編集Shapeを保護するためBody全体の元順序を保持しました": "Kept the entire Body in its original order to protect XML-edited Shapes",
    "解析済みComponent順序を適用中…": "Applying analyzed Component order…",
    "Component順序と属性を検証中…": "Verifying Component order and attributes…",
    "component上限は0以上で指定してください": "The Component limit must be zero or greater",
    "CPUワーカー数は自動（0）または1以上で指定してください": "CPU workers must be Auto (0) or one or greater",
    "この解析結果は最適化コピーを保存できません": "This analysis result cannot be saved as an optimized copy",
    "保存に必要な解析済みデータがありません。もう一度解析してください": "The analyzed data required for saving is missing; analyze the vehicle again",
    "元の車両XMLとは別の保存先を選んでください": "Choose a save location different from the original vehicle XML",
    "解析後に元の車両XMLが変更されました。もう一度解析してください": "The original vehicle XML changed after analysis; analyze it again",
    "Component順序の解析結果が不足しています。もう一度解析してください": "The analyzed Component order is incomplete; analyze the vehicle again",
    "解析中に元の車両XMLが変更されました。もう一度解析してください": "The original vehicle XML changed during analysis; analyze it again",
    "bodyが見つかりません": "No Bodies found",
    "完了した解析ジョブが見つかりません": "The completed analysis job could not be found",
}


def normalize_language(value: object) -> str:
    language = str(value or DEFAULT_LANGUAGE).lower()
    return language if language in SUPPORTED_LANGUAGES else DEFAULT_LANGUAGE


def translated_text(language: str, key: str, **values: object) -> str:
    template = _TEXT[normalize_language(language)].get(key, _TEXT[DEFAULT_LANGUAGE][key])
    return template.format(**values)


def translate_runtime_message(message: str, language: str) -> str:
    """Translate progress/reason text emitted by the Japanese service layer."""

    if normalize_language(language) != "en" or not message:
        return message
    exact = _RUNTIME_EXACT_EN.get(message)
    if exact is not None:
        return exact
    error_match = re.fullmatch(r"([A-Za-z_][A-Za-z0-9_.]*Error): (.*)", message)
    if error_match:
        return "{}: {}".format(
            error_match.group(1),
            translate_runtime_message(error_match.group(2), language),
        )
    body_match = re.fullmatch(r"Body (\d+)/(\d+): (.*)", message)
    if body_match:
        return "Body {}/{}: {}".format(
            body_match.group(1),
            body_match.group(2),
            translate_runtime_message(body_match.group(3), language),
        )
    body_warning_match = re.fullmatch(r"Body (\d+): (.*)", message)
    if body_warning_match:
        return "Body {}: {}".format(
            body_warning_match.group(1),
            translate_runtime_message(body_warning_match.group(2), language),
        )
    patterns = (
        (r"Physics voxelを展開しました（(\d+) voxel）", "Expanded physics voxels ({0} voxels)"),
        (r"Physics Flooderを解析しました（充填(\d+) voxel）", "Analyzed Physics Flooder ({0} filled voxels)"),
        (r"Flooderサンプル区画 (\d+)/(\d+)を探索中…", "Searching Flooder sample compartment {0}/{1}…"),
        (r"配置候補を評価中… (\d+)/(\d+)", "Evaluating candidates… {0}/{1}"),
        (r"探索 (\d+)/(\d+)：配置候補 (\d+)/(\d+)・CPU (\d+)・現在の最良 (\d+) Shapes", "Search {0}/{1}: candidates {2}/{3} · CPU {4} · current best {5} Shapes"),
        (r"(\d+)重複座標に関係する(\d+) Componentを元の順序位置へ固定し、残りを最適化できます", "Locked {1} Components involved in {0} overlapping positions and can optimize the remainder"),
        (r"Body (\d+)を手動除外しました（再解析なし）", "Body {0} was manually excluded (no re-analysis)"),
        (r"Body (\d+)を最適化対象へ戻しました（再解析なし）", "Body {0} was returned to optimization (no re-analysis)"),
        (r"XML編集または未対応Shapeの(\d+) Componentがあるため、相互作用を変えないようBody全体を元の順序に固定しました", "Locked the entire Body in its original order because it contains {0} XML-edited or unsupported Components"),
        (r"XML編集または未対応Shapeの(\d+) Component \((\d+) physics voxel\)を含むためBody全体を順序固定・予測対象外にしました", "Locked the complete Body order and excluded it from prediction because it contains {0} XML-edited or unsupported Components ({1} physics voxels)"),
        (r"予測対象外の(\d+) Componentを元スロットへ固定し、残りの対応Componentを最適化できます", "Locked {0} unpredicted Components in their original slots and can optimize the supported remainder"),
        (r"予測対象外の(\d+) Componentを探索から除外して元スロットへ戻します。表示値は対応範囲のみで、最終F2 Shape数には対象外Componentとの相互作用が含まれません", "Excluded {0} unpredicted Components from the search and restored them to their original slots. Displayed values cover only the supported scope and do not include final F2 interactions with excluded Components"),
        (r"Physics Flooderの面モデル(.*)を予測対象外にし、残りのComponentだけを最適化します", "Excluded unsupported Physics Flooder surface model {0} from prediction and will optimize the remaining Components"),
        (r"予測対象外Componentと対応Componentに重複physics座標があるため、該当する対応Componentも元の順序位置へ固定しました", "Unpredicted and supported Components overlap at physics positions, so the affected supported Components were also locked in their original order positions"),
        (r"車両XMLが見つかりません: (.*)", "Vehicle XML not found: {0}"),
        (r"Stormworks definitionsフォルダが正しくありません: (.*)", "Invalid Stormworks definitions folder: {0}"),
        (r"元の車両XMLが見つかりません: (.*)", "Original vehicle XML not found: {0}"),
        (r"出力先は既に存在します: (.*)", "Output already exists: {0}"),
        (r"設定保存を省略しました: (.*)", "Could not save settings: {0}"),
        (r"(\d+) componentsあり、設定上限の(\d+)を超えています", "Contains {0} Components, exceeding the configured limit of {1}"),
        (r"Physics Flooderの面モデルが未対応です: (.*)", "Unsupported Physics Flooder surface model: {0}"),
        (r"(.*)（(\d+)%）", "{0} ({1}%)"),
    )
    for pattern, replacement in patterns:
        match = re.fullmatch(pattern, message)
        if match:
            groups = tuple(
                translate_runtime_message(value, language) if index == 0 and "（" in pattern else value
                for index, value in enumerate(match.groups())
            )
            return replacement.format(*groups)
    return message


class JobCancelled(Exception):
    """Internal cooperative-cancellation signal for a background job."""


def vehicle_xml_from_urls(urls: Sequence[QUrl]) -> Optional[Path]:
    for url in urls:
        if not url.isLocalFile():
            continue
        path = Path(url.toLocalFile())
        if path.is_file() and path.suffix.lower() == ".xml":
            return path
    return None


class WorkerSignals(QObject):
    finished = Signal(object)
    failed = Signal(str)
    progress = Signal(int, str)
    cancelled = Signal()


class Worker(QRunnable):
    def __init__(self, job: Callable[..., object], reports_progress: bool = False):
        super().__init__()
        self.job = job
        self.reports_progress = reports_progress
        self.signals = WorkerSignals()
        self._cancel_requested = Event()

    def cancel(self) -> None:
        self._cancel_requested.set()

    def _report_progress(self, percent: int, message: str) -> None:
        if self._cancel_requested.is_set():
            raise JobCancelled()
        self.signals.progress.emit(percent, message)
        if self._cancel_requested.is_set():
            raise JobCancelled()

    @Slot()
    def run(self) -> None:
        try:
            if self._cancel_requested.is_set():
                raise JobCancelled()
            result = (
                self.job(self._report_progress)
                if self.reports_progress
                else self.job()
            )
            if self._cancel_requested.is_set():
                raise JobCancelled()
        except JobCancelled:
            self.signals.cancelled.emit()
            return
        except Exception:
            self.signals.failed.emit(traceback.format_exc())
            return
        self.signals.finished.emit(result)


class OptimizerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("{} V{}".format(APP_NAME, APP_VERSION))
        self.setMinimumSize(820, 660)
        self.resize(980, 790)
        self.thread_pool = QThreadPool.globalInstance()
        # Keep the QRunnable and its signal object alive until the queued GUI
        # callback has run.  Without this reference a fast job can finish and
        # be auto-deleted before Qt delivers either result signal, leaving the
        # window permanently in its busy state.
        self._active_worker: Optional[Worker] = None
        self._active_done: Optional[Callable[[object], None]] = None
        self._active_cancellable = False
        self.last_analysis: Optional[VehicleAnalysis] = None
        self.last_output: Optional[OptimizationOutput] = None
        self.last_error_trace: Optional[str] = None
        self._fit_next_preview = True
        self._fit_body_ids: Optional[set[int]] = None
        self.busy = False
        self.selected_body_ids: set[int] = set()
        self.hidden_body_ids: set[int] = set()
        self.selection_anchor_id: Optional[int] = None
        self.hovered_body_id: Optional[int] = None
        self.ghost_nonselected = True
        self.ghost_opacity = 0.25
        self.overview_opacity = 1.0
        self.body_filter_key = "all"
        self.body_sort_key = "id"
        self.body_sort_direction = 1
        self._body_history: list[BodyHistoryEntry] = []
        self._body_future: list[BodyHistoryEntry] = []
        self._updating_body_tree = False
        self._updating_body_controls = False
        self._pick_cycle_candidates: Tuple[int, ...] = ()
        self._pick_cycle_index = -1
        self._body_shortcuts: list[QShortcut] = []
        self._shortcut_labels: list[Tuple[QLabel, str]] = []
        self.setAcceptDrops(True)

        config = load_config()
        self.language = normalize_language(config.get("language", DEFAULT_LANGUAGE))
        self._raw_status_message = _TEXT[DEFAULT_LANGUAGE]["initial_status"]
        self._raw_progress_message = _TEXT[DEFAULT_LANGUAGE]["starting_analysis"]
        configured = Path(config["definitions_path"]) if config.get("definitions_path") else None
        detected = find_definition_directory(configured)

        self.vehicle_edit = QLineEdit()
        self.vehicle_edit.setAcceptDrops(False)
        self.definitions_edit = QLineEdit(str(detected or configured or ""))
        self.definitions_edit.setPlaceholderText("Stormworks/rom/data/definitions")
        self.definitions_edit.setCursorPosition(0)
        self.current_shapes = QLabel("—")
        self.optimized_shapes = QLabel("—")
        self.component_count = QLabel("—")
        self.eligibility = QLabel()
        self.eligibility.setWordWrap(True)
        self.language_selector = QComboBox()
        self.language_selector.addItem("日本語", "ja")
        self.language_selector.addItem("English", "en")
        configured_language_index = self.language_selector.findData(self.language)
        self.language_selector.setCurrentIndex(
            configured_language_index if configured_language_index >= 0 else 0
        )
        self.search_mode_selector = QComboBox()
        for profile in SEARCH_MODE_PROFILES.values():
            self.search_mode_selector.addItem(profile.label, profile.key)
        configured_mode = config.get("search_mode", "standard")
        configured_mode_index = self.search_mode_selector.findData(configured_mode)
        self.search_mode_selector.setCurrentIndex(
            configured_mode_index if configured_mode_index >= 0 else 0
        )
        self.worker_selector = QComboBox()
        for label, value in (
            ("自動（推奨）", 0),
            ("1（省メモリ）", 1),
            ("2", 2),
            ("4", 4),
            ("8", 8),
        ):
            self.worker_selector.addItem(label, value)
        try:
            configured_workers = int(config.get("worker_count", "0"))
        except ValueError:
            configured_workers = 0
        configured_worker_index = self.worker_selector.findData(configured_workers)
        self.worker_selector.setCurrentIndex(
            configured_worker_index if configured_worker_index >= 0 else 0
        )
        self.search_hint = QLabel()
        self.search_hint.setObjectName("subtitle")
        self.search_hint.setWordWrap(True)
        self.status = QStatusBar()
        self.setStatusBar(self.status)

        self._build_ui()
        self.vehicle_edit.textEdited.connect(self.analysis_input_edited)
        self.definitions_edit.textEdited.connect(self.analysis_input_edited)
        self.language_selector.currentIndexChanged.connect(self.language_changed)
        self.search_mode_selector.currentIndexChanged.connect(
            self.analysis_settings_changed
        )
        self.worker_selector.currentIndexChanged.connect(
            self.analysis_settings_changed
        )
        self._apply_language()

    def _build_ui(self) -> None:
        self.page_scroll = QScrollArea()
        self.page_scroll.setWidgetResizable(True)
        self.page_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.page_scroll.setFrameShape(QFrame.NoFrame)
        central = QWidget()
        central.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        self.page_scroll.setWidget(central)
        self.setCentralWidget(self.page_scroll)
        layout = QVBoxLayout(central)
        layout.setSizeConstraint(QLayout.SizeConstraint.SetMinimumSize)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(14)

        header = QHBoxLayout()
        self.title_label = QLabel("Physics Shape Optimizer")
        self.title_label.setObjectName("title")
        self.language_label = QLabel()
        self.author_label = QLabel()
        self.author_label.setObjectName("subtitle")
        header.addWidget(self.title_label)
        header.addStretch(1)
        header.addWidget(self.language_label, 0, Qt.AlignBottom)
        header.addWidget(self.language_selector, 0, Qt.AlignBottom)
        header.addSpacing(8)
        header.addWidget(self.author_label, 0, Qt.AlignBottom)
        self.subtitle_label = QLabel()
        self.subtitle_label.setObjectName("subtitle")
        layout.addLayout(header)
        layout.addWidget(self.subtitle_label)

        self.files_group = QGroupBox()
        file_grid = QGridLayout(self.files_group)
        file_grid.setColumnStretch(1, 1)
        self.vehicle_xml_label = QLabel()
        file_grid.addWidget(self.vehicle_xml_label, 0, 0)
        file_grid.addWidget(self.vehicle_edit, 0, 1)
        self.choose_vehicle_button = QPushButton()
        self.choose_vehicle_button.clicked.connect(self.choose_vehicle)
        file_grid.addWidget(self.choose_vehicle_button, 0, 2)
        self.definitions_label = QLabel()
        file_grid.addWidget(self.definitions_label, 1, 0)
        file_grid.addWidget(self.definitions_edit, 1, 1)
        self.choose_definitions_button = QPushButton()
        self.choose_definitions_button.clicked.connect(self.choose_definitions)
        file_grid.addWidget(self.choose_definitions_button, 1, 2)
        layout.addWidget(self.files_group)

        self.search_settings_group = QGroupBox()
        search_layout = QGridLayout(self.search_settings_group)
        self.search_mode_label = QLabel()
        search_layout.addWidget(self.search_mode_label, 0, 0)
        self.search_mode_selector.setMinimumWidth(160)
        search_layout.addWidget(self.search_mode_selector, 0, 1)
        self.cpu_workers_label = QLabel()
        search_layout.addWidget(self.cpu_workers_label, 0, 2)
        self.worker_selector.setMinimumWidth(170)
        search_layout.addWidget(self.worker_selector, 0, 3)
        search_layout.setColumnStretch(4, 1)
        search_layout.addWidget(self.search_hint, 1, 0, 1, 5)
        layout.addWidget(self.search_settings_group)

        actions = QHBoxLayout()
        self.analyze_button = QPushButton()
        self.analyze_button.setObjectName("primaryButton")
        self.analyze_button.clicked.connect(self.start_analysis)
        self.optimize_button = QPushButton()
        self.optimize_button.setObjectName("primaryButton")
        self.optimize_button.setEnabled(False)
        self.optimize_button.clicked.connect(self.choose_output_and_optimize)
        self.open_folder_button = QPushButton()
        self.open_folder_button.clicked.connect(self.open_vehicle_directory)
        actions.addWidget(self.analyze_button)
        actions.addWidget(self.optimize_button)
        actions.addWidget(self.open_folder_button)
        actions.addStretch(1)
        layout.addLayout(actions)

        self.progress_widget = QWidget()
        progress_layout = QHBoxLayout(self.progress_widget)
        progress_layout.setContentsMargins(0, 0, 0, 0)
        self.progress_label = QLabel()
        self.progress_label.setMinimumWidth(260)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.stop_button = QPushButton()
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_current_job)
        progress_layout.addWidget(self.progress_label)
        progress_layout.addWidget(self.progress_bar, 1)
        progress_layout.addWidget(self.stop_button)
        self.progress_widget.setVisible(False)
        layout.addWidget(self.progress_widget)

        self.results_group = QGroupBox()
        result_layout = QVBoxLayout(self.results_group)
        metric_row = QHBoxLayout()
        self.metric_labels = {}
        metric_row.addWidget(self._metric("current_metric", self.current_shapes))
        metric_row.addWidget(self._metric("optimized_metric", self.optimized_shapes))
        metric_row.addWidget(self._metric("component_metric", self.component_count))
        result_layout.addLayout(metric_row)
        result_layout.addWidget(self.eligibility)
        layout.addWidget(self.results_group)

        self.tabs = QTabWidget()
        self.tabs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.tabs.setMinimumHeight(520)
        layout.addWidget(self.tabs, 1)

        self.preview_tab = QWidget()
        preview_layout = QVBoxLayout(self.preview_tab)
        preview_layout.setContentsMargins(8, 8, 8, 8)

        mode_row = QHBoxLayout()
        self.preview_mode_label = QLabel()
        mode_row.addWidget(self.preview_mode_label)
        self.shape_mode_button = QPushButton()
        self.shape_mode_button.setCheckable(True)
        self.body_mode_button = QPushButton()
        self.body_mode_button.setCheckable(True)
        self.preview_mode_group = QButtonGroup(self)
        self.preview_mode_group.setExclusive(True)
        self.preview_mode_group.addButton(self.shape_mode_button, 0)
        self.preview_mode_group.addButton(self.body_mode_button, 1)
        self.shape_mode_button.setChecked(True)
        self.body_mode_button.toggled.connect(self.preview_mode_changed)
        mode_row.addWidget(self.shape_mode_button)
        mode_row.addWidget(self.body_mode_button)
        mode_row.addStretch(1)
        preview_layout.addLayout(mode_row)

        self.preview_toolbar_widget = QWidget()
        toolbar = QHBoxLayout(self.preview_toolbar_widget)
        toolbar.setContentsMargins(0, 0, 0, 0)
        self.display_label = QLabel()
        toolbar.addWidget(self.display_label)
        self.current_radio = QRadioButton()
        self.current_radio.setChecked(True)
        self.current_radio.toggled.connect(self.update_preview)
        self.optimized_radio = QRadioButton()
        self.optimized_radio.setEnabled(False)
        self.optimized_radio.toggled.connect(self.update_preview)
        toolbar.addWidget(self.current_radio)
        toolbar.addWidget(self.optimized_radio)
        toolbar.addSpacing(12)
        self.display_scope_label = QLabel()
        toolbar.addWidget(self.display_scope_label)
        self.body_selector = QComboBox()
        self.body_selector.setMinimumWidth(110)
        self.body_selector.currentIndexChanged.connect(self.body_selection_changed)
        toolbar.addWidget(self.body_selector)
        self.body_selection_label = QLabel()
        self.body_selection_label.setWordWrap(True)
        self.body_selection_label.setVisible(False)
        toolbar.addWidget(self.body_selection_label, 1)
        self.reset_view_button = QPushButton()
        self.reset_view_button.clicked.connect(self.reset_view)
        toolbar.addWidget(self.reset_view_button)
        toolbar.addStretch(1)
        preview_layout.addWidget(self.preview_toolbar_widget)

        self.manual_options_widget = QWidget()
        manual_options = QHBoxLayout(self.manual_options_widget)
        manual_options.setContentsMargins(0, 0, 0, 0)
        self.manual_body_exclusion = QCheckBox()
        self.manual_body_exclusion.setEnabled(False)
        self.manual_body_exclusion.toggled.connect(
            self.manual_body_exclusion_changed
        )
        manual_options.addWidget(self.manual_body_exclusion)
        manual_options.addStretch(1)
        self.manual_options_widget.setVisible(False)
        preview_layout.addWidget(self.manual_options_widget)

        self.viewer = PhysicsShapeViewer()
        self.viewer.setMinimumHeight(420)
        self.viewer.bodyPicked.connect(self.body_picked_from_preview)
        self.viewer.bodyHovered.connect(self.body_hovered_from_preview)
        self.body_management_panel = self._build_body_management_panel()
        self.body_management_panel.setMinimumWidth(285)
        self.body_management_panel.setVisible(False)
        self.preview_splitter = QSplitter(Qt.Horizontal)
        self.preview_splitter.setChildrenCollapsible(False)
        self.preview_splitter.addWidget(self.viewer)
        self.preview_splitter.addWidget(self.body_management_panel)
        self.preview_splitter.setStretchFactor(0, 1)
        self.preview_splitter.setStretchFactor(1, 0)
        preview_layout.addWidget(self.preview_splitter, 1)
        self.preview_caption = QLabel()
        self.preview_caption.setWordWrap(True)
        self.preview_caption.setAlignment(Qt.AlignHCenter)
        preview_layout.addWidget(self.preview_caption)
        self.shortcut_help_container = self._build_shortcut_help()
        self.shortcut_help_container.setVisible(False)
        preview_layout.addWidget(self.shortcut_help_container)
        self._install_body_shortcuts()
        self.tabs.addTab(self.preview_tab, "")

        self.details = QPlainTextEdit()
        self.details.setReadOnly(True)
        self.tabs.addTab(self.details, "")

    def _build_body_management_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("bodyManagementPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(7)

        title_row = QHBoxLayout()
        self.body_layers_label = QLabel()
        self.body_layers_label.setObjectName("bodyLayersTitle")
        self.body_list_count_label = QLabel()
        self.body_list_count_label.setObjectName("subtitle")
        title_row.addWidget(self.body_layers_label)
        title_row.addStretch(1)
        title_row.addWidget(self.body_list_count_label)
        layout.addLayout(title_row)

        selection_tools = QGridLayout()
        selection_tools.setHorizontalSpacing(5)
        selection_tools.setVerticalSpacing(5)
        self.select_filtered_button = QPushButton()
        self.select_filtered_button.clicked.connect(self.select_filtered_bodies)
        self.invert_filtered_button = QPushButton()
        self.invert_filtered_button.clicked.connect(self.invert_filtered_body_selection)
        self.ghost_button = QPushButton()
        self.ghost_button.setCheckable(True)
        self.ghost_button.setChecked(True)
        self.ghost_button.toggled.connect(self.ghost_mode_changed)
        self.undo_body_button = QPushButton()
        self.undo_body_button.setEnabled(False)
        self.undo_body_button.clicked.connect(self.undo_body_change)
        self.redo_body_button = QPushButton()
        self.redo_body_button.setEnabled(False)
        self.redo_body_button.clicked.connect(self.redo_body_change)
        self.reset_body_state_button = QPushButton()
        self.reset_body_state_button.clicked.connect(self.reset_body_management_changes)
        selection_tools.addWidget(self.select_filtered_button, 0, 0)
        selection_tools.addWidget(self.invert_filtered_button, 0, 1)
        selection_tools.addWidget(self.ghost_button, 1, 0, 1, 2)
        selection_tools.addWidget(self.undo_body_button, 2, 0)
        selection_tools.addWidget(self.redo_body_button, 2, 1)
        selection_tools.addWidget(self.reset_body_state_button, 3, 0, 1, 2)
        layout.addLayout(selection_tools)

        list_controls = QHBoxLayout()
        list_controls.setSpacing(5)
        self.body_filter_selector = QComboBox()
        self.body_filter_selector.currentIndexChanged.connect(
            self.body_filter_changed
        )
        self.body_sort_selector = QComboBox()
        self.body_sort_selector.currentIndexChanged.connect(self.body_sort_changed)
        self.body_sort_direction_button = QPushButton()
        self.body_sort_direction_button.setMaximumWidth(92)
        self.body_sort_direction_button.clicked.connect(
            self.toggle_body_sort_direction
        )
        list_controls.addWidget(self.body_filter_selector, 1)
        list_controls.addWidget(self.body_sort_selector, 1)
        list_controls.addWidget(self.body_sort_direction_button)
        layout.addLayout(list_controls)

        opacity_grid = QGridLayout()
        opacity_grid.setHorizontalSpacing(6)
        opacity_grid.setVerticalSpacing(4)
        self.ghost_opacity_label = QLabel()
        self.ghost_opacity_slider = QSlider(Qt.Horizontal)
        self.ghost_opacity_slider.setRange(5, 80)
        self.ghost_opacity_slider.setValue(25)
        self.ghost_opacity_slider.valueChanged.connect(self.body_opacity_changed)
        self.ghost_opacity_value = QLabel("25%")
        self.overview_opacity_label = QLabel()
        self.overview_opacity_slider = QSlider(Qt.Horizontal)
        self.overview_opacity_slider.setRange(5, 100)
        self.overview_opacity_slider.setValue(100)
        self.overview_opacity_slider.valueChanged.connect(self.body_opacity_changed)
        self.overview_opacity_value = QLabel("100%")
        opacity_grid.addWidget(self.ghost_opacity_label, 0, 0)
        opacity_grid.addWidget(self.ghost_opacity_slider, 0, 1)
        opacity_grid.addWidget(self.ghost_opacity_value, 0, 2)
        opacity_grid.addWidget(self.overview_opacity_label, 1, 0)
        opacity_grid.addWidget(self.overview_opacity_slider, 1, 1)
        opacity_grid.addWidget(self.overview_opacity_value, 1, 2)
        layout.addLayout(opacity_grid)

        self.body_tree = QTreeWidget()
        self.body_tree.setColumnCount(4)
        self.body_tree.setRootIsDecorated(False)
        self.body_tree.setAlternatingRowColors(True)
        self.body_tree.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.body_tree.setMouseTracking(True)
        self.body_tree.setUniformRowHeights(True)
        self.body_tree.setMinimumHeight(220)
        self.body_tree.itemSelectionChanged.connect(self.body_tree_selection_changed)
        self.body_tree.itemChanged.connect(self.body_tree_item_changed)
        self.body_tree.itemEntered.connect(self.body_tree_item_hovered)
        self.body_tree.installEventFilter(self)
        self.body_tree.viewport().installEventFilter(self)
        layout.addWidget(self.body_tree, 1)

        self.body_bulk_widget = QFrame()
        self.body_bulk_widget.setObjectName("bodyBulkBar")
        bulk_layout = QVBoxLayout(self.body_bulk_widget)
        bulk_layout.setContentsMargins(7, 6, 7, 6)
        bulk_layout.setSpacing(5)
        self.body_bulk_summary = QLabel()
        self.body_bulk_summary.setWordWrap(True)
        bulk_layout.addWidget(self.body_bulk_summary)
        bulk_buttons = QHBoxLayout()
        bulk_buttons.setSpacing(4)
        self.show_selected_button = QPushButton()
        self.show_selected_button.clicked.connect(
            lambda: self.set_selected_bodies_visible(True)
        )
        self.hide_selected_button = QPushButton()
        self.hide_selected_button.clicked.connect(
            lambda: self.set_selected_bodies_visible(False)
        )
        self.optimize_selected_button = QPushButton()
        self.optimize_selected_button.clicked.connect(
            lambda: self.set_selected_bodies_optimized(True)
        )
        self.exclude_selected_button = QPushButton()
        self.exclude_selected_button.clicked.connect(
            lambda: self.set_selected_bodies_optimized(False)
        )
        for button in (
            self.show_selected_button,
            self.hide_selected_button,
            self.optimize_selected_button,
            self.exclude_selected_button,
        ):
            bulk_buttons.addWidget(button)
        bulk_layout.addLayout(bulk_buttons)
        self.body_bulk_widget.setVisible(False)
        layout.addWidget(self.body_bulk_widget)
        return panel

    def _build_shortcut_help(self) -> QWidget:
        container = QFrame()
        container.setObjectName("shortcutHelp")
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.shortcut_help_button = QPushButton()
        self.shortcut_help_button.setCheckable(True)
        self.shortcut_help_button.toggled.connect(self.shortcut_help_toggled)
        layout.addWidget(self.shortcut_help_button)
        self.shortcut_help_content = QWidget()
        columns = QHBoxLayout(self.shortcut_help_content)
        columns.setContentsMargins(10, 8, 10, 10)
        columns.setSpacing(14)
        columns.addWidget(
            self._shortcut_column(
                "shortcut_selection_title",
                (
                    ("Click", "shortcut_click_action"),
                    ("Enter", "shortcut_enter_action"),
                    ("Cmd/Ctrl + Click", "shortcut_add_action"),
                    ("Shift + Click", "shortcut_range_action"),
                    ("Cmd/Ctrl + A", "shortcut_select_all_action"),
                    ("Esc", "shortcut_clear_action"),
                    ("Alt + 3D Click", "shortcut_cycle_action"),
                ),
            ),
            1,
        )
        columns.addWidget(
            self._shortcut_column(
                "shortcut_view_title",
                (
                    ("Left Drag", "shortcut_rotate_action"),
                    ("Wheel", "shortcut_zoom_action"),
                    ("F", "shortcut_focus_action"),
                    ("Background Click", "shortcut_background_action"),
                    ("H", "shortcut_hide_action"),
                    ("Shift + H", "shortcut_isolate_action"),
                    ("Alt + H", "shortcut_show_all_action"),
                ),
            ),
            1,
        )
        columns.addWidget(
            self._shortcut_column(
                "shortcut_optimization_title",
                (
                    ("Space", "shortcut_toggle_optimize_action"),
                    ("Cmd/Ctrl + Z", "shortcut_undo_action"),
                    ("Cmd/Ctrl + Shift + Z", "shortcut_redo_action"),
                    ("Ctrl + Y", "shortcut_redo_action"),
                    ("Eye / Optimize", "shortcut_group_action"),
                    ("Hover", "shortcut_hover_action"),
                ),
            ),
            1,
        )
        self.shortcut_help_content.setVisible(False)
        layout.addWidget(self.shortcut_help_content)
        return container

    def _shortcut_column(
        self,
        title_key: str,
        rows: Sequence[Tuple[str, str]],
    ) -> QWidget:
        column = QWidget()
        layout = QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        title = QLabel()
        title.setObjectName("shortcutTitle")
        self._shortcut_labels.append((title, title_key))
        layout.addWidget(title)
        for keys, action_key in rows:
            row = QHBoxLayout()
            row.setSpacing(6)
            key_label = QLabel(keys)
            key_label.setObjectName("shortcutKeys")
            key_label.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Preferred)
            action = QLabel()
            action.setWordWrap(True)
            action.setObjectName("shortcutAction")
            self._shortcut_labels.append((action, action_key))
            row.addWidget(key_label)
            row.addWidget(action, 1)
            layout.addLayout(row)
        layout.addStretch(1)
        return column

    def _install_body_shortcuts(self) -> None:
        bindings = (
            (QKeySequence(QKeySequence.StandardKey.SelectAll), self.select_all_bodies),
            (QKeySequence("Escape"), self.clear_body_selection),
            (QKeySequence("F"), self.focus_selected_bodies),
            (QKeySequence("H"), lambda: self.set_selected_bodies_visible(False)),
            (QKeySequence("Shift+H"), self.hide_unselected_bodies),
            (QKeySequence("Alt+H"), self.show_all_bodies),
            (QKeySequence("Space"), self.toggle_selected_body_optimization),
            (QKeySequence(QKeySequence.StandardKey.Undo), self.undo_body_change),
            (QKeySequence(QKeySequence.StandardKey.Redo), self.redo_body_change),
            (QKeySequence("Ctrl+Y"), self.redo_body_change),
            (QKeySequence("Ctrl+Shift+Z"), self.redo_body_change),
        )
        registered_sequences = set()
        for sequence, callback in bindings:
            portable_sequence = sequence.toString(QKeySequence.PortableText)
            if not portable_sequence or portable_sequence in registered_sequences:
                continue
            registered_sequences.add(portable_sequence)
            shortcut = QShortcut(sequence, self.preview_tab)
            shortcut.setContext(Qt.WidgetWithChildrenShortcut)
            shortcut.activated.connect(
                lambda callback=callback: self._run_body_shortcut(callback)
            )
            self._body_shortcuts.append(shortcut)

    def _run_body_shortcut(self, callback: Callable[[], None]) -> None:
        if not self.body_mode_button.isChecked() or self.last_analysis is None:
            return
        focus = QApplication.focusWidget()
        if isinstance(
            focus,
            (
                QLineEdit,
                QPlainTextEdit,
                QComboBox,
                QSlider,
                QCheckBox,
                QRadioButton,
                QPushButton,
            ),
        ):
            return
        callback()

    def eventFilter(self, watched: QObject, event: object) -> bool:
        if (
            hasattr(self, "body_tree")
            and watched is self.body_tree
            and isinstance(event, QKeyEvent)
            and event.type() == QEvent.KeyPress
            and event.key() in (Qt.Key_Return, Qt.Key_Enter)
        ):
            self.toggle_focused_body_selection()
            return True
        if hasattr(self, "body_tree") and watched is self.body_tree.viewport():
            if isinstance(event, QMouseEvent) and event.type() == QEvent.MouseButtonPress:
                item = self.body_tree.itemAt(event.position().toPoint())
                column = self.body_tree.columnAt(int(event.position().x()))
                raw_body_index = (
                    item.data(0, Qt.UserRole) if item is not None else None
                )
                if raw_body_index is not None and event.button() == Qt.LeftButton:
                    body_index = int(raw_body_index)
                    self._set_current_body_tree_item(item, column)
                    if column == 0:
                        self._toggle_body_visibility(body_index)
                        return True
                    if column == 3:
                        body = self._body_by_index(body_index)
                        if (
                            body is not None
                            and body.can_optimize
                            and not body.protected_body
                            and not self.busy
                        ):
                            targets = self._action_body_ids(body_index)
                            optimize = item.checkState(3) != Qt.Checked
                            self._set_body_ids_optimized(targets, optimize)
                        return True
                    if column in (1, 2):
                        self._select_body_from_list_click(
                            body_index,
                            event.modifiers(),
                        )
                        return True
                if event.button() == Qt.LeftButton and item is None:
                    selection_modifiers = (
                        Qt.ShiftModifier
                        | Qt.ControlModifier
                        | Qt.MetaModifier
                        | Qt.AltModifier
                    )
                    if not bool(event.modifiers() & selection_modifiers):
                        self.clear_body_selection()
                    return True
            elif event.type() in (QEvent.MouseMove, QEvent.HoverMove) and hasattr(
                event,
                "position",
            ):
                if (
                    isinstance(event, QMouseEvent)
                    and bool(event.buttons() & Qt.LeftButton)
                ):
                    return True
                if self.body_tree.itemAt(event.position().toPoint()) is None:
                    self._set_hovered_body(None, update_viewer=True)
            elif isinstance(event, QEvent) and event.type() == QEvent.Leave:
                self._set_hovered_body(None, update_viewer=True)
        return super().eventFilter(watched, event)  # type: ignore[arg-type]

    def _body_by_index(self, body_index: int):
        analysis = self.last_analysis
        if analysis is None:
            return None
        return next(
            (
                body
                for body in analysis.bodies
                if body.body_index == body_index
            ),
            None,
        )

    @staticmethod
    def _body_color(body_index: int) -> str:
        base = QColor(BODY_COLORS[body_index % len(BODY_COLORS)])
        cycle = body_index // len(BODY_COLORS)
        if cycle == 0:
            return base.name()
        hue, saturation, value, _alpha = base.getHsv()
        return QColor.fromHsv(
            (hue + cycle * 19) % 360,
            max(130, min(245, saturation + ((cycle % 3) - 1) * 22)),
            max(170, min(245, value + ((cycle % 4) - 1) * 12)),
        ).name()

    @staticmethod
    def _body_optimized_count(body) -> int:
        value = body.effective_optimized_shape_count
        return body.current_shape_count if value is None else value

    def _body_warning_level(self, body) -> int:
        if not body.can_optimize or body.protected_body:
            return 4
        if body.flooder_prediction_excluded:
            return 3
        if body.xml_edited_component_count:
            return 2
        if self._body_optimized_count(body) == body.current_shape_count:
            return 1
        return 0

    def _body_warning_text(self, body) -> str:
        warnings = []
        if not body.can_optimize or body.protected_body:
            warnings.append(self._t("body_warning_unsupported"))
        if body.flooder_prediction_excluded:
            warnings.append(self._t("body_warning_flooder"))
        if body.xml_edited_component_count:
            warnings.append(
                self._t("body_warning_xml", count=body.xml_edited_component_count)
            )
        if (
            self._body_optimized_count(body) == body.current_shape_count
            and not warnings
        ):
            warnings.append(self._t("body_warning_zero"))
        return " / ".join(warnings)

    def _body_matches_filter(self, body) -> bool:
        key = self.body_filter_key
        if key == "selected":
            return body.body_index in self.selected_body_ids
        if key == "visible":
            return body.body_index not in self.hidden_body_ids
        if key == "hidden":
            return body.body_index in self.hidden_body_ids
        if key == "included":
            return not body.manually_excluded
        if key == "excluded":
            return body.manually_excluded
        if key == "zero":
            return self._body_optimized_count(body) == body.current_shape_count
        if key == "warning":
            return self._body_warning_level(body) > 0
        return True

    def _body_sort_value(self, body):
        key = self.body_sort_key
        if key == "current":
            return body.current_shape_count
        if key == "optimized":
            return self._body_optimized_count(body)
        if key == "reduction":
            return body.current_shape_count - self._body_optimized_count(body)
        if key == "components":
            return body.component_count
        if key == "warning":
            return self._body_warning_level(body)
        if key == "visible":
            return int(body.body_index not in self.hidden_body_ids)
        if key == "optimize":
            return int(not body.manually_excluded)
        return body.body_index

    def _listed_bodies(self):
        analysis = self.last_analysis
        if analysis is None:
            return ()
        filtered = [body for body in analysis.bodies if self._body_matches_filter(body)]
        return tuple(
            sorted(
                filtered,
                key=lambda body: (
                    self._body_sort_value(body) * self.body_sort_direction,
                    body.body_index,
                ),
            )
        )

    def _action_body_ids(self, body_index: int) -> set[int]:
        if len(self.selected_body_ids) > 1 and body_index in self.selected_body_ids:
            return set(self.selected_body_ids)
        return {body_index}

    def _manual_excluded_body_ids(self) -> set[int]:
        analysis = self.last_analysis
        if analysis is None:
            return set()
        return {
            body.body_index
            for body in analysis.bodies
            if body.manually_excluded
        }

    def _body_snapshot(self) -> BodyManagementSnapshot:
        return BodyManagementSnapshot(
            selected_body_ids=tuple(sorted(self.selected_body_ids)),
            hidden_body_ids=tuple(sorted(self.hidden_body_ids)),
            manually_excluded_body_ids=tuple(
                sorted(self._manual_excluded_body_ids())
            ),
            selection_anchor_id=self.selection_anchor_id,
        )

    def _set_manual_excluded_body_ids_raw(self, body_ids: set[int]) -> bool:
        analysis = self.last_analysis
        if analysis is None:
            return False
        before = self._manual_excluded_body_ids()
        updated = apply_manual_body_exclusions(analysis, sorted(body_ids))
        after = {
            body.body_index
            for body in updated.bodies
            if body.manually_excluded
        }
        if after == before:
            return False
        self.last_analysis = updated
        self.last_output = None
        self.last_error_trace = None
        return True

    def _refresh_body_management(self, analysis_changed: bool = False) -> None:
        if analysis_changed and self.last_analysis is not None:
            self._render_analysis(self.last_analysis, preserve_selection=True)
            return
        self._render_body_tree()
        self._update_body_management_controls()
        self.update_preview()

    def _commit_body_change(
        self,
        label_key: str,
        mutate: Callable[[], bool],
    ) -> None:
        before = self._body_snapshot()
        analysis_changed = bool(mutate())
        if not self._record_body_history(label_key, before):
            self._update_body_management_controls()
            return
        self._refresh_body_management(analysis_changed=analysis_changed)

    def _record_body_history(
        self,
        label_key: str,
        before: BodyManagementSnapshot,
    ) -> bool:
        if self._body_snapshot() == before:
            return False
        self._body_history.append(BodyHistoryEntry(label_key, before))
        if len(self._body_history) > 50:
            self._body_history.pop(0)
        self._body_future.clear()
        return True

    def _restore_body_snapshot(self, snapshot: BodyManagementSnapshot) -> None:
        analysis = self.last_analysis
        available = (
            {body.body_index for body in analysis.bodies}
            if analysis is not None
            else set()
        )
        self.selected_body_ids = set(snapshot.selected_body_ids) & available
        self.hidden_body_ids = set(snapshot.hidden_body_ids) & available
        self.selection_anchor_id = (
            snapshot.selection_anchor_id
            if snapshot.selection_anchor_id in available
            else None
        )
        changed = self._set_manual_excluded_body_ids_raw(
            set(snapshot.manually_excluded_body_ids) & available
        )
        self._refresh_body_management(analysis_changed=changed)
        if hasattr(self, "body_tree"):
            if self.selection_anchor_id is None:
                self.body_tree.setCurrentItem(None)
            else:
                for row in range(self.body_tree.topLevelItemCount()):
                    item = self.body_tree.topLevelItem(row)
                    if item.data(0, Qt.UserRole) == self.selection_anchor_id:
                        self._set_current_body_tree_item(item, 1)
                        self.body_tree.scrollToItem(
                            item,
                            QAbstractItemView.EnsureVisible,
                        )
                        break

    def _reset_body_management_for_analysis(self) -> None:
        self.selected_body_ids.clear()
        self.hidden_body_ids.clear()
        self.selection_anchor_id = None
        self.hovered_body_id = None
        self._pick_cycle_candidates = ()
        self._pick_cycle_index = -1
        self._fit_body_ids = None
        self._body_history.clear()
        self._body_future.clear()
        if hasattr(self, "viewer"):
            self.viewer.set_hovered_body(None)

    def _rebuild_body_management_selectors(self) -> None:
        filter_key = self.body_filter_key
        sort_key = self.body_sort_key
        self.body_filter_selector.blockSignals(True)
        self.body_filter_selector.clear()
        for key, text_key in (
            ("all", "body_filter_all"),
            ("selected", "body_filter_selected"),
            ("visible", "body_filter_visible"),
            ("hidden", "body_filter_hidden"),
            ("included", "body_filter_included"),
            ("excluded", "body_filter_excluded"),
            ("zero", "body_filter_zero"),
            ("warning", "body_filter_warning"),
        ):
            self.body_filter_selector.addItem(self._t(text_key), key)
        filter_index = self.body_filter_selector.findData(filter_key)
        self.body_filter_selector.setCurrentIndex(max(0, filter_index))
        self.body_filter_selector.blockSignals(False)

        self.body_sort_selector.blockSignals(True)
        self.body_sort_selector.clear()
        for key, text_key in (
            ("id", "body_sort_id"),
            ("current", "body_sort_current"),
            ("optimized", "body_sort_optimized"),
            ("reduction", "body_sort_reduction"),
            ("components", "body_sort_components"),
            ("warning", "body_sort_warning"),
            ("visible", "body_sort_visibility"),
            ("optimize", "body_sort_optimization"),
        ):
            self.body_sort_selector.addItem(self._t(text_key), key)
        sort_index = self.body_sort_selector.findData(sort_key)
        self.body_sort_selector.setCurrentIndex(max(0, sort_index))
        self.body_sort_selector.blockSignals(False)
        self._update_body_sort_direction_text()

    def _render_body_tree(self) -> None:
        if not hasattr(self, "body_tree"):
            return
        analysis = self.last_analysis
        current_item = self.body_tree.currentItem()
        current_body_index = (
            current_item.data(0, Qt.UserRole)
            if current_item is not None
            else self.selection_anchor_id
        )
        scroll_value = self.body_tree.verticalScrollBar().value()
        self._updating_body_tree = True
        self.body_tree.blockSignals(True)
        try:
            self.body_tree.clear()
            listed = self._listed_bodies()
            if analysis is not None and not listed:
                empty = QTreeWidgetItem(("", self._t("no_matching_bodies"), "", ""))
                empty.setFlags(Qt.NoItemFlags)
                self.body_tree.addTopLevelItem(empty)
            for body in listed:
                warning = self._body_warning_text(body)
                suffix = (
                    self._t("manual_excluded_suffix")
                    if body.manually_excluded
                    else ""
                )
                item = QTreeWidgetItem()
                item.setData(0, Qt.UserRole, body.body_index)
                item.setText(1, "Body {}{}".format(body.body_index, suffix))
                swatch = QPixmap(10, 10)
                swatch.fill(QColor(self._body_color(body.body_index)))
                item.setIcon(1, QIcon(swatch))
                item.setText(
                    2,
                    self._t(
                        "body_row_stats",
                        current=body.current_shape_count,
                        optimized=self._body_optimized_count(body),
                        components=body.component_count,
                    ),
                )
                tooltip = self._runtime_text(body.reason)
                if warning:
                    tooltip = "{}\n{}".format(warning, tooltip)
                    item.setText(1, "{}  ⚠".format(item.text(1)))
                for column in range(4):
                    item.setToolTip(column, tooltip)
                flags = item.flags() | Qt.ItemIsSelectable | Qt.ItemIsEnabled
                if body.can_optimize and not body.protected_body:
                    flags |= Qt.ItemIsUserCheckable
                item.setFlags(flags)
                self.body_tree.addTopLevelItem(item)
                if body.body_index in self.selected_body_ids:
                    item.setSelected(True)
            self._update_body_tree_action_states()
            self.body_tree.setColumnWidth(0, 38)
            self.body_tree.setColumnWidth(1, 92)
            self.body_tree.setColumnWidth(2, 92)
            self.body_tree.setColumnWidth(3, 64)
            restored_current = None
            for row in range(self.body_tree.topLevelItemCount()):
                candidate = self.body_tree.topLevelItem(row)
                if candidate.data(0, Qt.UserRole) == current_body_index:
                    restored_current = candidate
                    break
            if restored_current is not None:
                self.body_tree.setCurrentItem(
                    restored_current,
                    1,
                    QItemSelectionModel.NoUpdate,
                )
            self.body_tree.verticalScrollBar().setValue(scroll_value)
        finally:
            self.body_tree.blockSignals(False)
            self._updating_body_tree = False
        self._update_body_management_controls()

    def _update_body_tree_action_states(self) -> None:
        if not hasattr(self, "body_tree"):
            return
        was_blocked = self.body_tree.blockSignals(True)
        try:
            for row in range(self.body_tree.topLevelItemCount()):
                item = self.body_tree.topLevelItem(row)
                raw_body_index = item.data(0, Qt.UserRole)
                if raw_body_index is None:
                    continue
                body_index = int(raw_body_index)
                body = self._body_by_index(body_index)
                if body is None:
                    continue
                targets = self._action_body_ids(body_index)
                visible_count = sum(
                    target not in self.hidden_body_ids for target in targets
                )
                if visible_count == len(targets):
                    item.setText(0, "◉")
                elif visible_count == 0:
                    item.setText(0, "○")
                else:
                    item.setText(0, "◐")
                if body.can_optimize and not body.protected_body:
                    optimized_count = sum(
                        not bool(
                            candidate is not None and candidate.manually_excluded
                        )
                        for candidate in (
                            self._body_by_index(target) for target in targets
                        )
                    )
                    if optimized_count == len(targets):
                        item.setCheckState(3, Qt.Checked)
                    elif optimized_count == 0:
                        item.setCheckState(3, Qt.Unchecked)
                    else:
                        item.setCheckState(3, Qt.PartiallyChecked)
                else:
                    item.setText(3, self._t("body_fixed"))
                hovered = body_index == self.hovered_body_id
                brush = QBrush(QColor("#EDE9FE")) if hovered else QBrush()
                for column in range(4):
                    item.setBackground(column, brush)
        finally:
            self.body_tree.blockSignals(was_blocked)

    def _update_body_management_controls(self) -> None:
        if not hasattr(self, "body_tree"):
            return
        analysis = self.last_analysis
        bodies = () if analysis is None else analysis.bodies
        selected = [
            body for body in bodies if body.body_index in self.selected_body_ids
        ]
        listed_count = len(self._listed_bodies())
        self.body_list_count_label.setText(
            self._t(
                "body_list_count",
                listed=listed_count,
                total=len(bodies),
                selected=len(selected),
            )
        )
        if len(selected) == 1:
            body = selected[0]
            self.body_selection_label.setText(
                self._t(
                    "body_selection_one",
                    body=body.body_index,
                    current=body.current_shape_count,
                    optimized=self._body_optimized_count(body),
                )
            )
        elif selected:
            self.body_selection_label.setText(
                self._t(
                    "body_selection_many",
                    count=len(selected),
                    current=sum(body.current_shape_count for body in selected),
                    optimized=sum(self._body_optimized_count(body) for body in selected),
                )
            )
        else:
            self.body_selection_label.setText(self._t("body_selection_none"))
        visible_selected = sum(
            body.body_index not in self.hidden_body_ids for body in selected
        )
        optimized_selected = sum(not body.manually_excluded for body in selected)
        self.body_bulk_summary.setText(
            self._t(
                "bulk_summary",
                count=len(selected),
                visible=visible_selected,
                optimized=optimized_selected,
            )
            if selected
            else ""
        )
        self.body_bulk_widget.setVisible(
            self.body_mode_button.isChecked() and len(selected) > 1
        )
        has_analysis = analysis is not None and bool(bodies)
        has_selection = bool(selected)
        eligible_selection = any(
            body.can_optimize and not body.protected_body for body in selected
        )
        self.select_filtered_button.setEnabled(has_analysis and listed_count > 0)
        self.invert_filtered_button.setEnabled(has_analysis and listed_count > 0)
        self.show_selected_button.setEnabled(has_selection)
        self.hide_selected_button.setEnabled(has_selection)
        self.optimize_selected_button.setEnabled(
            eligible_selection and not self.busy
        )
        self.exclude_selected_button.setEnabled(
            eligible_selection and not self.busy
        )
        self.reset_body_state_button.setEnabled(has_analysis and not self.busy)
        self.undo_body_button.setEnabled(bool(self._body_history) and not self.busy)
        self.redo_body_button.setEnabled(bool(self._body_future) and not self.busy)
        self._updating_body_controls = True
        try:
            self.ghost_button.setChecked(self.ghost_nonselected)
        finally:
            self._updating_body_controls = False

    @Slot(bool)
    def preview_mode_changed(self, body_mode: bool) -> None:
        self.body_management_panel.setVisible(body_mode)
        self.shortcut_help_container.setVisible(body_mode)
        self.body_selection_label.setVisible(body_mode)
        self.display_scope_label.setVisible(not body_mode)
        self.body_selector.setVisible(not body_mode)
        self.manual_options_widget.setVisible(False)
        if not body_mode:
            self.hovered_body_id = None
            self.viewer.set_hovered_body(None)
        self._render_body_tree()
        self.update_preview()

    @Slot(bool)
    def shortcut_help_toggled(self, expanded: bool) -> None:
        self.shortcut_help_content.setVisible(expanded)
        self._update_shortcut_help_button()

    def _update_shortcut_help_button(self) -> None:
        marker = "▼" if self.shortcut_help_button.isChecked() else "▶"
        self.shortcut_help_button.setText(
            "{} {}".format(marker, self._t("shortcut_help"))
        )

    @Slot(int)
    def body_filter_changed(self, _index: int = -1) -> None:
        value = self.body_filter_selector.currentData()
        if value is None:
            return
        self.body_filter_key = str(value)
        self._render_body_tree()

    @Slot(int)
    def body_sort_changed(self, _index: int = -1) -> None:
        value = self.body_sort_selector.currentData()
        if value is None:
            return
        self.body_sort_key = str(value)
        self._render_body_tree()

    @Slot()
    def toggle_body_sort_direction(self) -> None:
        self.body_sort_direction *= -1
        self._update_body_sort_direction_text()
        self._render_body_tree()

    def _update_body_sort_direction_text(self) -> None:
        if not hasattr(self, "body_sort_direction_button"):
            return
        self.body_sort_direction_button.setText(
            self._t(
                "sort_ascending"
                if self.body_sort_direction == 1
                else "sort_descending"
            )
        )

    @Slot(bool)
    def ghost_mode_changed(self, checked: bool) -> None:
        if self._updating_body_controls:
            return
        self.ghost_nonselected = checked
        self.update_preview()

    @Slot(int)
    def body_opacity_changed(self, _value: int = 0) -> None:
        self.ghost_opacity = self.ghost_opacity_slider.value() / 100.0
        self.overview_opacity = self.overview_opacity_slider.value() / 100.0
        self.ghost_opacity_value.setText(
            "{}%".format(self.ghost_opacity_slider.value())
        )
        self.overview_opacity_value.setText(
            "{}%".format(self.overview_opacity_slider.value())
        )
        self.update_preview()

    @Slot()
    def body_tree_selection_changed(self) -> None:
        if self._updating_body_tree:
            return
        before = self._body_snapshot()
        listed_ids = {
            int(item.data(0, Qt.UserRole))
            for item in (
                self.body_tree.topLevelItem(row)
                for row in range(self.body_tree.topLevelItemCount())
            )
            if item.data(0, Qt.UserRole) is not None
        }
        selected_visible = {
            int(item.data(0, Qt.UserRole))
            for item in self.body_tree.selectedItems()
            if item.data(0, Qt.UserRole) is not None
        }
        self.selected_body_ids = (
            self.selected_body_ids - listed_ids
        ) | selected_visible
        current = self.body_tree.currentItem()
        if current is not None and current.data(0, Qt.UserRole) is not None:
            self.selection_anchor_id = int(current.data(0, Qt.UserRole))
        elif not self.selected_body_ids:
            self.selection_anchor_id = None
        self._record_body_history("shortcut_click_action", before)
        if self.body_filter_key == "selected":
            self._render_body_tree()
        else:
            self._update_body_tree_action_states()
            self._update_body_management_controls()
        self.update_preview()

    def _select_body_from_list_click(
        self,
        body_index: int,
        modifiers: object,
    ) -> None:
        additive = bool(modifiers & (Qt.ControlModifier | Qt.MetaModifier))
        extend_range = bool(modifiers & Qt.ShiftModifier)
        listed_ids = [body.body_index for body in self._listed_bodies()]

        def mutate() -> bool:
            if extend_range:
                anchor = self.selection_anchor_id
                if anchor not in listed_ids:
                    anchor = body_index
                start = listed_ids.index(anchor)
                end = listed_ids.index(body_index)
                range_ids = set(listed_ids[min(start, end) : max(start, end) + 1])
                if additive:
                    self.selected_body_ids.update(range_ids)
                else:
                    self.selected_body_ids = range_ids
                self.selection_anchor_id = anchor
            elif additive:
                if body_index in self.selected_body_ids:
                    self.selected_body_ids.remove(body_index)
                    if self.selection_anchor_id == body_index:
                        self.selection_anchor_id = (
                            min(self.selected_body_ids)
                            if self.selected_body_ids
                            else None
                        )
                else:
                    self.selected_body_ids.add(body_index)
                    self.selection_anchor_id = body_index
            elif self.selected_body_ids == {body_index}:
                self.selected_body_ids.clear()
                self.selection_anchor_id = None
            else:
                self.selected_body_ids = {body_index}
                self.selection_anchor_id = body_index
            return False

        self._commit_body_change("shortcut_click_action", mutate)
        self._focus_body_tree_row(body_index)

    def _focus_body_tree_row(self, body_index: int) -> None:
        self.body_tree.setFocus(Qt.MouseFocusReason)
        for row in range(self.body_tree.topLevelItemCount()):
            item = self.body_tree.topLevelItem(row)
            if item.data(0, Qt.UserRole) == body_index:
                self._set_current_body_tree_item(item, 1)
                self.body_tree.scrollToItem(
                    item,
                    QAbstractItemView.EnsureVisible,
                )
                return

    def _set_current_body_tree_item(
        self,
        item: QTreeWidgetItem,
        column: int,
    ) -> None:
        self.body_tree.setCurrentItem(
            item,
            column,
            QItemSelectionModel.NoUpdate,
        )

    def _toggle_body_visibility(self, body_index: int) -> None:
        targets = self._action_body_ids(body_index)

        def mutate() -> bool:
            make_visible = any(target in self.hidden_body_ids for target in targets)
            if make_visible:
                self.hidden_body_ids.difference_update(targets)
            else:
                self.hidden_body_ids.update(targets)
            return False

        self._commit_body_change(
            "show_selected"
            if any(target in self.hidden_body_ids for target in targets)
            else "hide_selected",
            mutate,
        )

    @Slot(QTreeWidgetItem, int)
    def body_tree_item_changed(self, item: QTreeWidgetItem, column: int) -> None:
        if self._updating_body_tree or column != 3:
            return
        raw_body_index = item.data(0, Qt.UserRole)
        if raw_body_index is None or self.busy:
            self._render_body_tree()
            return
        body_index = int(raw_body_index)
        body = self._body_by_index(body_index)
        if body is None or not body.can_optimize or body.protected_body:
            self._render_body_tree()
            return
        targets = self._action_body_ids(body_index)
        optimize = item.checkState(3) != Qt.Unchecked
        self._set_body_ids_optimized(targets, optimize)

    @Slot(QTreeWidgetItem, int)
    def body_tree_item_hovered(self, item: QTreeWidgetItem, _column: int) -> None:
        raw_body_index = item.data(0, Qt.UserRole)
        self._set_hovered_body(
            int(raw_body_index) if raw_body_index is not None else None,
            update_viewer=True,
        )

    @Slot(object)
    def body_hovered_from_preview(self, body_index: object) -> None:
        normalized = int(body_index) if body_index is not None else None
        if normalized is not None and self._body_by_index(normalized) is None:
            normalized = None
        self._set_hovered_body(normalized, update_viewer=False)

    def _set_hovered_body(
        self,
        body_index: Optional[int],
        *,
        update_viewer: bool,
    ) -> None:
        if body_index == self.hovered_body_id:
            return
        self.hovered_body_id = body_index
        if update_viewer:
            self.viewer.set_hovered_body(body_index)
        self._update_body_tree_action_states()

    @Slot(object, object)
    def body_picked_from_preview(
        self,
        candidates: object,
        modifiers: object,
    ) -> None:
        if not self.body_mode_button.isChecked():
            return
        candidate_ids = tuple(
            int(candidate)
            for candidate in (candidates or ())  # type: ignore[union-attr]
            if self._body_by_index(int(candidate)) is not None
        )
        keyboard_modifiers = modifiers
        additive = bool(
            keyboard_modifiers
            & (Qt.ShiftModifier | Qt.ControlModifier | Qt.MetaModifier)
        )
        alt_cycle = bool(keyboard_modifiers & Qt.AltModifier)
        if not candidate_ids:
            if not additive and not alt_cycle:
                self.clear_body_selection()
            return
        if alt_cycle:
            if candidate_ids != self._pick_cycle_candidates:
                self._pick_cycle_candidates = candidate_ids
                self._pick_cycle_index = 0
            else:
                self._pick_cycle_index = (
                    self._pick_cycle_index + 1
                ) % len(candidate_ids)
            body_index = candidate_ids[self._pick_cycle_index]
        else:
            self._pick_cycle_candidates = ()
            self._pick_cycle_index = -1
            body_index = candidate_ids[0]

        def mutate() -> bool:
            if additive:
                if body_index in self.selected_body_ids:
                    self.selected_body_ids.remove(body_index)
                    if self.selection_anchor_id == body_index:
                        self.selection_anchor_id = (
                            min(self.selected_body_ids)
                            if self.selected_body_ids
                            else None
                        )
                else:
                    self.selected_body_ids.add(body_index)
                    self.selection_anchor_id = body_index
            elif alt_cycle:
                self.selected_body_ids = {body_index}
                self.selection_anchor_id = body_index
            elif self.selected_body_ids == {body_index}:
                self.selected_body_ids.clear()
                self.selection_anchor_id = None
            else:
                self.selected_body_ids = {body_index}
                self.selection_anchor_id = body_index
            return False

        self._commit_body_change("shortcut_click_action", mutate)

    def select_filtered_bodies(self) -> None:
        ids = {body.body_index for body in self._listed_bodies()}

        def mutate() -> bool:
            self.selected_body_ids = set(ids)
            self.selection_anchor_id = min(ids) if ids else None
            return False

        self._commit_body_change("select_filtered", mutate)

    def invert_filtered_body_selection(self) -> None:
        ids = {body.body_index for body in self._listed_bodies()}

        def mutate() -> bool:
            self.selected_body_ids.symmetric_difference_update(ids)
            self.selection_anchor_id = (
                min(self.selected_body_ids) if self.selected_body_ids else None
            )
            return False

        self._commit_body_change("invert_filtered", mutate)

    def select_all_bodies(self) -> None:
        analysis = self.last_analysis
        ids = set() if analysis is None else {body.body_index for body in analysis.bodies}

        def mutate() -> bool:
            self.selected_body_ids = set(ids)
            self.selection_anchor_id = min(ids) if ids else None
            return False

        self._commit_body_change("shortcut_select_all_action", mutate)

    def clear_body_selection(self) -> None:
        def mutate() -> bool:
            self.selected_body_ids.clear()
            self.selection_anchor_id = None
            return False

        self._commit_body_change("shortcut_clear_action", mutate)

    def toggle_focused_body_selection(self) -> None:
        item = self.body_tree.currentItem()
        raw_body_index = (
            item.data(0, Qt.UserRole) if item is not None else None
        )
        if raw_body_index is None:
            return
        body_index = int(raw_body_index)

        def mutate() -> bool:
            if body_index in self.selected_body_ids:
                self.selected_body_ids.remove(body_index)
                if self.selection_anchor_id == body_index:
                    self.selection_anchor_id = (
                        min(self.selected_body_ids)
                        if self.selected_body_ids
                        else None
                    )
            else:
                self.selected_body_ids.add(body_index)
                self.selection_anchor_id = body_index
            return False

        self._commit_body_change("shortcut_enter_action", mutate)

    def focus_selected_bodies(self) -> None:
        visible_selection = self.selected_body_ids - self.hidden_body_ids
        if not visible_selection or self.last_analysis is None:
            return
        self._fit_body_ids = set(visible_selection)
        self._fit_next_preview = True
        self.update_preview()

    def set_selected_bodies_visible(self, visible: bool) -> None:
        targets = set(self.selected_body_ids)
        if not targets:
            return

        def mutate() -> bool:
            if visible:
                self.hidden_body_ids.difference_update(targets)
            else:
                self.hidden_body_ids.update(targets)
            return False

        self._commit_body_change(
            "show_selected" if visible else "hide_selected",
            mutate,
        )

    def hide_unselected_bodies(self) -> None:
        analysis = self.last_analysis
        if analysis is None or not self.selected_body_ids:
            return
        targets = {
            body.body_index
            for body in analysis.bodies
            if body.body_index not in self.selected_body_ids
        }

        def mutate() -> bool:
            self.hidden_body_ids.update(targets)
            return False

        self._commit_body_change("shortcut_isolate_action", mutate)

    def show_all_bodies(self) -> None:
        def mutate() -> bool:
            self.hidden_body_ids.clear()
            return False

        self._commit_body_change("shortcut_show_all_action", mutate)

    def _set_body_ids_optimized(self, body_ids: set[int], optimize: bool) -> None:
        if self.busy:
            return

        def mutate() -> bool:
            excluded = self._manual_excluded_body_ids()
            eligible = {
                body_id
                for body_id in body_ids
                for body in (self._body_by_index(body_id),)
                if body is not None and body.can_optimize and not body.protected_body
            }
            if optimize:
                excluded.difference_update(eligible)
            else:
                excluded.update(eligible)
            return self._set_manual_excluded_body_ids_raw(excluded)

        self._commit_body_change(
            "optimize_selected" if optimize else "exclude_selected",
            mutate,
        )

    def set_selected_bodies_optimized(self, optimize: bool) -> None:
        self._set_body_ids_optimized(set(self.selected_body_ids), optimize)

    def toggle_selected_body_optimization(self) -> None:
        selected = [
            self._body_by_index(body_id) for body_id in self.selected_body_ids
        ]
        eligible = [
            body
            for body in selected
            if body is not None and body.can_optimize and not body.protected_body
        ]
        if not eligible:
            return
        optimize = any(body.manually_excluded for body in eligible)
        self.set_selected_bodies_optimized(optimize)

    def reset_body_management_changes(self) -> None:
        if self.last_analysis is None or self.busy:
            return

        def mutate() -> bool:
            self.hidden_body_ids.clear()
            return self._set_manual_excluded_body_ids_raw(set())

        self._commit_body_change("reset_body_state", mutate)

    def undo_body_change(self) -> None:
        if not self._body_history or self.busy:
            return
        entry = self._body_history.pop()
        self._body_future.append(BodyHistoryEntry(entry.label_key, self._body_snapshot()))
        self._restore_body_snapshot(entry.snapshot)

    def redo_body_change(self) -> None:
        if not self._body_future or self.busy:
            return
        entry = self._body_future.pop()
        self._body_history.append(BodyHistoryEntry(entry.label_key, self._body_snapshot()))
        self._restore_body_snapshot(entry.snapshot)

    def _metric(self, text_key: str, value: QLabel) -> QFrame:
        frame = QFrame()
        frame.setObjectName("metricCard")
        frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        card = QVBoxLayout(frame)
        label = QLabel()
        self.metric_labels[text_key] = label
        label.setObjectName("metricName")
        label.setWordWrap(True)
        value.setObjectName("metricValue")
        value.setAlignment(Qt.AlignHCenter)
        label.setAlignment(Qt.AlignHCenter)
        card.addWidget(label)
        card.addWidget(value)
        return frame

    def _t(self, key: str, **values: object) -> str:
        return translated_text(self.language, key, **values)

    def _runtime_text(self, message: str) -> str:
        return translate_runtime_message(message, self.language)

    def _show_status(self, message: str) -> None:
        self._raw_status_message = message
        self.status.showMessage(self._runtime_text(message))

    def _set_progress_message(self, message: str) -> None:
        self._raw_progress_message = message
        self.progress_label.setText(self._runtime_text(message))

    def _search_profile_label(self, key: str) -> str:
        text_key = "search_{}_label".format(key)
        if text_key in _TEXT[self.language]:
            return self._t(text_key)
        try:
            return search_mode_profile(key).label
        except (KeyError, ValueError):
            return key

    def _search_profile_description(self, key: str) -> str:
        text_key = "search_{}_description".format(key)
        if text_key in _TEXT[self.language]:
            return self._t(text_key)
        try:
            return search_mode_profile(key).description
        except (KeyError, ValueError):
            return key

    def _rebuild_search_selectors(self) -> None:
        search_mode = str(self.search_mode_selector.currentData() or "standard")
        worker_count = self.worker_selector.currentData()

        self.search_mode_selector.blockSignals(True)
        self.search_mode_selector.clear()
        for key in SEARCH_MODE_PROFILES:
            self.search_mode_selector.addItem(self._search_profile_label(key), key)
        search_index = self.search_mode_selector.findData(search_mode)
        self.search_mode_selector.setCurrentIndex(search_index if search_index >= 0 else 0)
        self.search_mode_selector.blockSignals(False)

        self.worker_selector.blockSignals(True)
        self.worker_selector.clear()
        for label, value in (
            (self._t("auto_worker"), 0),
            (self._t("one_worker"), 1),
            ("2", 2),
            ("4", 4),
            ("8", 8),
        ):
            self.worker_selector.addItem(label, value)
        worker_index = self.worker_selector.findData(worker_count)
        self.worker_selector.setCurrentIndex(worker_index if worker_index >= 0 else 0)
        self.worker_selector.blockSignals(False)

    def _apply_language(self) -> None:
        self.vehicle_edit.setPlaceholderText(self._t("vehicle_placeholder"))
        self.language_label.setText(self._t("language"))
        self.author_label.setText(self._t("author", author=APP_AUTHOR))
        self.subtitle_label.setText(self._t("subtitle"))
        self.files_group.setTitle(self._t("files"))
        self.vehicle_xml_label.setText(self._t("vehicle_xml"))
        self.definitions_label.setText(self._t("definitions"))
        self.choose_vehicle_button.setText(self._t("select"))
        self.choose_definitions_button.setText(self._t("select"))
        self.search_settings_group.setTitle(self._t("analysis_settings"))
        self.search_mode_label.setText(self._t("search_mode"))
        self.cpu_workers_label.setText(self._t("cpu_workers"))
        self._rebuild_search_selectors()
        self.update_search_hint()
        self.analyze_button.setText(self._t("analyze"))
        self.optimize_button.setText(self._t("save_optimized"))
        self.open_folder_button.setText(self._t("open_vehicle_folder"))
        self.stop_button.setText(self._t("stop_analysis"))
        self.results_group.setTitle(self._t("analysis_results"))
        for text_key, label in self.metric_labels.items():
            label.setText(self._t(text_key))
        self.preview_mode_label.setText(self._t("preview_mode"))
        self.shape_mode_button.setText(self._t("shape_check_mode"))
        self.body_mode_button.setText(self._t("body_management_mode"))
        self.display_label.setText(self._t("display"))
        self.current_radio.setText(self._t("current"))
        self.optimized_radio.setText(self._t("optimized"))
        self.display_scope_label.setText(self._t("display_scope"))
        self.manual_body_exclusion.setText(self._t("manual_body_exclusion"))
        self.reset_view_button.setText(self._t("reset_view"))
        self.body_layers_label.setText(self._t("body_layers"))
        self.select_filtered_button.setText(self._t("select_filtered"))
        self.invert_filtered_button.setText(self._t("invert_filtered"))
        self.ghost_button.setText(self._t("ghost_unselected"))
        self.undo_body_button.setText(self._t("undo"))
        self.redo_body_button.setText(self._t("redo"))
        self.reset_body_state_button.setText(self._t("reset_body_state"))
        self.ghost_opacity_label.setText(self._t("ghost_opacity"))
        self.overview_opacity_label.setText(self._t("overview_opacity"))
        self.show_selected_button.setText(self._t("show_selected"))
        self.hide_selected_button.setText(self._t("hide_selected"))
        self.optimize_selected_button.setText(self._t("optimize_selected"))
        self.exclude_selected_button.setText(self._t("exclude_selected"))
        self.body_tree.setHeaderLabels(
            (
                self._t("body_column_display"),
                self._t("body_column_body"),
                self._t("body_column_details"),
                self._t("body_column_optimize"),
            )
        )
        self._rebuild_body_management_selectors()
        self._update_shortcut_help_button()
        for label, text_key in self._shortcut_labels:
            label.setText(self._t(text_key))
        self.tabs.setTabText(self.tabs.indexOf(self.preview_tab), self._t("preview_tab"))
        self.tabs.setTabText(self.tabs.indexOf(self.details), self._t("details_tab"))
        self.details.setPlaceholderText(self._t("details_placeholder"))
        self.viewer.set_empty_message(self._t("empty_preview"))
        self._set_progress_message(self._raw_progress_message)
        self.status.showMessage(self._runtime_text(self._raw_status_message))

        if self.last_analysis is not None:
            self._render_analysis(self.last_analysis, preserve_selection=True)
        else:
            self.eligibility.setText(self._t("not_analyzed"))
            self.preview_caption.setText(self._t("interaction_hint"))
            self._render_body_tree()
        if self.last_output is not None:
            self._render_optimization(self.last_output)
        if self.last_error_trace is not None:
            self._render_error_trace(self.last_error_trace)

    @Slot(int)
    def language_changed(self, _index: int = -1) -> None:
        language = normalize_language(self.language_selector.currentData())
        if language == self.language:
            return
        self.language = language
        self._apply_language()
        self.remember_configuration()

    def reset_view(self) -> None:
        if self.body_mode_button.isChecked() and self.last_analysis is not None:
            self._fit_body_ids = None
            self._fit_next_preview = True
            self.update_preview()
            return
        self.viewer.reset_view()

    @Slot(int)
    def body_selection_changed(self, _index: int = -1) -> None:
        self._sync_manual_body_exclusion_control()
        self.update_preview()

    def _sync_manual_body_exclusion_control(self) -> None:
        analysis = self.last_analysis
        selected_body_index = self.body_selector.currentData()
        body = (
            next(
                (
                    candidate
                    for candidate in analysis.bodies
                    if candidate.body_index == selected_body_index
                ),
                None,
            )
            if analysis is not None and selected_body_index != -1
            else None
        )
        self.manual_body_exclusion.blockSignals(True)
        self.manual_body_exclusion.setChecked(
            bool(body is not None and body.manually_excluded)
        )
        self.manual_body_exclusion.setEnabled(
            bool(
                not self.busy
                and body is not None
                and body.can_optimize
                and not body.protected_body
            )
        )
        self.manual_body_exclusion.blockSignals(False)

    @Slot(bool)
    def manual_body_exclusion_changed(self, checked: bool) -> None:
        analysis = self.last_analysis
        selected_body_index = self.body_selector.currentData()
        if analysis is None or selected_body_index == -1 or self.busy:
            self._sync_manual_body_exclusion_control()
            return
        body = next(
            (
                candidate
                for candidate in analysis.bodies
                if candidate.body_index == selected_body_index
            ),
            None,
        )
        if body is None or not body.can_optimize or body.protected_body:
            self._sync_manual_body_exclusion_control()
            return
        excluded = {
            candidate.body_index
            for candidate in analysis.bodies
            if candidate.manually_excluded
        }
        if checked:
            excluded.add(body.body_index)
        else:
            excluded.discard(body.body_index)
        self.last_analysis = apply_manual_body_exclusions(analysis, sorted(excluded))
        self.last_output = None
        self.last_error_trace = None
        self._render_analysis(self.last_analysis, preserve_selection=True)
        self._show_status(
            self._t(
                "manual_exclusion_status_on"
                if checked
                else "manual_exclusion_status_off",
                body=body.body_index,
            )
        )

    def choose_vehicle(self) -> None:
        initial = vehicle_directory()
        if not initial.is_dir():
            initial = Path.home()
        selected, _filter = QFileDialog.getOpenFileName(
            self,
            self._t("choose_vehicle"),
            str(initial),
            self._t("vehicle_filter"),
        )
        if selected:
            self.set_vehicle_path(Path(selected))

    def set_vehicle_path(self, path: Path) -> None:
        if self.busy:
            self._show_status("処理中は車両XMLを変更できません")
            return
        self.vehicle_edit.setText(str(path))
        self.vehicle_edit.setCursorPosition(0)
        self.last_analysis = None
        self.last_output = None
        self.last_error_trace = None
        self._reset_body_management_for_analysis()
        self.optimize_button.setEnabled(False)
        self.optimized_radio.setEnabled(False)
        self.current_radio.setChecked(True)
        self.current_shapes.setText("—")
        self.optimized_shapes.setText("—")
        self.component_count.setText("—")
        self.eligibility.setText(self._t("not_analyzed"))
        self.body_selector.clear()
        self._sync_manual_body_exclusion_control()
        self.viewer.set_shapes((), fit=True)
        self._render_body_tree()
        self.details.clear()
        self.preview_caption.setText(self._t("interaction_hint"))
        self._show_status("車両XMLを読み込みました。解析待ちです")

    def dragEnterEvent(self, event: QDragEnterEvent) -> None:
        if not self.busy and vehicle_xml_from_urls(event.mimeData().urls()) is not None:
            event.acceptProposedAction()
            return
        event.ignore()

    def dropEvent(self, event: QDropEvent) -> None:
        path = vehicle_xml_from_urls(event.mimeData().urls())
        if self.busy or path is None:
            event.ignore()
            return
        self.set_vehicle_path(path)
        event.acceptProposedAction()

    def choose_definitions(self) -> None:
        initial = self.definitions_edit.text().strip() or str(Path.home())
        selected = QFileDialog.getExistingDirectory(
            self, self._t("choose_definitions"), initial
        )
        if selected:
            self.definitions_edit.setText(selected)
            self.remember_definitions(selected)
            self.last_analysis = None
            self.last_output = None
            self.last_error_trace = None
            self._reset_body_management_for_analysis()
            self.optimize_button.setEnabled(False)
            self._sync_manual_body_exclusion_control()
            self._render_body_tree()

    def remember_definitions(self, path: str) -> None:
        self.definitions_edit.setText(path)
        self.remember_configuration()

    def remember_configuration(self) -> None:
        try:
            save_config(
                {
                    "definitions_path": self.definitions_edit.text().strip(),
                    "search_mode": str(self.search_mode_selector.currentData()),
                    "worker_count": str(self.worker_selector.currentData()),
                    "language": self.language,
                }
            )
        except OSError as error:
            self._show_status("設定保存を省略しました: {}".format(error))

    def update_search_hint(self) -> None:
        profile_key = str(self.search_mode_selector.currentData())
        worker_label = self.worker_selector.currentText()
        self.search_hint.setText(
            self._t(
                "worker_hint",
                description=self._search_profile_description(profile_key),
                workers=worker_label,
            )
        )

    @Slot()
    def analysis_settings_changed(self) -> None:
        self.update_search_hint()
        self.remember_configuration()
        self.last_output = None
        self.last_error_trace = None
        if self.last_analysis is not None:
            self.last_analysis = None
            self._reset_body_management_for_analysis()
            self.optimize_button.setEnabled(False)
            self.optimized_radio.setEnabled(False)
            self.current_radio.setChecked(True)
            self._sync_manual_body_exclusion_control()
            self._render_body_tree()
            self._show_status("解析設定が変わりました。もう一度解析してください")

    @Slot(str)
    def analysis_input_edited(self, _text: str = "") -> None:
        self.last_analysis = None
        self.last_output = None
        self.last_error_trace = None
        self._reset_body_management_for_analysis()
        self.optimize_button.setEnabled(False)
        self.optimized_radio.setEnabled(False)
        self.current_radio.setChecked(True)
        self.eligibility.setText(self._t("not_analyzed"))
        self._sync_manual_body_exclusion_control()
        self._render_body_tree()

    def set_busy(self, value: bool, message: str) -> None:
        self.busy = value
        self._show_status(message)
        self.analyze_button.setEnabled(not value)
        self.vehicle_edit.setEnabled(not value)
        self.definitions_edit.setEnabled(not value)
        self.choose_vehicle_button.setEnabled(not value)
        self.choose_definitions_button.setEnabled(not value)
        self.search_mode_selector.setEnabled(not value)
        self.worker_selector.setEnabled(not value)
        self.optimize_button.setEnabled(
            not value and self.last_analysis is not None and self.last_analysis.can_optimize
        )
        self.stop_button.setEnabled(value and self._active_cancellable)
        self._sync_manual_body_exclusion_control()
        if hasattr(self, "body_management_panel"):
            self.body_management_panel.setEnabled(not value)
            self._update_body_management_controls()
        if not value:
            self.progress_widget.setVisible(False)

    def run_background(
        self,
        job: Callable[..., object],
        done: Callable[[object], None],
        message: str,
        reports_progress: bool = False,
        cancellable: bool = False,
    ) -> None:
        if self.busy:
            return
        self.last_error_trace = None
        self._active_cancellable = cancellable
        self.set_busy(True, message)
        self.progress_widget.setVisible(reports_progress)
        if reports_progress:
            self.progress_bar.setValue(0)
            self._set_progress_message(message)
        worker = Worker(job, reports_progress=reports_progress)
        self._active_worker = worker
        self._active_done = done
        worker.signals.finished.connect(self.job_finished)
        worker.signals.failed.connect(self.job_failed)
        worker.signals.progress.connect(self.job_progress)
        worker.signals.cancelled.connect(self.job_cancelled)
        self.thread_pool.start(worker)

    @Slot()
    def stop_current_job(self) -> None:
        worker = self._active_worker
        if not self.busy or not self._active_cancellable or worker is None:
            return
        worker.cancel()
        self.stop_button.setEnabled(False)
        self._set_progress_message("解析を停止しています…")
        self._show_status("解析を停止しています…")

    @Slot()
    def job_cancelled(self) -> None:
        self._active_worker = None
        self._active_done = None
        self._active_cancellable = False
        self.set_busy(False, "解析を停止しました")

    @Slot(int, str)
    def job_progress(self, percent: int, message: str) -> None:
        if not self.busy:
            return
        clamped = max(0, min(100, percent))
        self.progress_bar.setValue(clamped)
        self._set_progress_message(message)
        self._raw_status_message = "{}（{}%）".format(message, clamped)
        self.status.showMessage(
            self._t(
                "status_percent",
                message=self._runtime_text(message),
                percent=clamped,
            )
        )

    @Slot(str)
    def job_failed(self, trace: str) -> None:
        self._active_worker = None
        self._active_done = None
        self._active_cancellable = False
        self.set_busy(False, "エラー")
        self.last_error_trace = trace
        self._render_error_trace(trace)
        final_line = trace.strip().splitlines()[-1] if trace.strip() else "Unknown error"
        QMessageBox.critical(self, APP_NAME, self._runtime_text(final_line))

    def _render_error_trace(self, trace: str) -> None:
        lines = trace.splitlines()
        if lines:
            lines[-1] = self._runtime_text(lines[-1])
        self.details.setPlainText("\n".join(lines))

    @Slot(object)
    def job_finished(self, result: object) -> None:
        done = self._active_done
        self._active_worker = None
        self._active_done = None
        self._active_cancellable = False
        self.set_busy(False, "完了")
        if done is None:
            self.job_failed("RuntimeError: 完了した解析ジョブが見つかりません")
            return
        done(result)

    def start_analysis(self) -> None:
        vehicle_path = Path(self.vehicle_edit.text().strip())
        definitions_path = Path(self.definitions_edit.text().strip())
        search_mode = str(self.search_mode_selector.currentData())
        worker_count = int(self.worker_selector.currentData())
        self.last_analysis = None
        self.last_output = None
        self._reset_body_management_for_analysis()
        self._render_body_tree()
        self.optimize_button.setEnabled(False)
        self.optimized_radio.setEnabled(False)
        self.current_radio.setChecked(True)
        self.remember_configuration()
        self.run_background(
            lambda report: analyze_vehicle(
                vehicle_path,
                definitions_path,
                progress_callback=report,
                search_mode=search_mode,
                worker_count=worker_count,
            ),
            self.show_analysis,
            "解析中…",
            reports_progress=True,
            cancellable=True,
        )

    def show_analysis(self, raw_result: object) -> None:
        if not isinstance(raw_result, VehicleAnalysis):
            raise TypeError("unexpected analysis result")
        analysis = raw_result
        self.last_analysis = analysis
        self.last_output = None
        self.last_error_trace = None
        self._reset_body_management_for_analysis()
        self.vehicle_edit.setCursorPosition(0)
        self.definitions_edit.setCursorPosition(0)
        self._render_analysis(analysis, preserve_selection=False)
        self._show_status("解析完了")
        if self.definitions_edit.text().strip():
            self.remember_definitions(self.definitions_edit.text().strip())

    def _render_analysis(
        self,
        analysis: VehicleAnalysis,
        *,
        preserve_selection: bool,
    ) -> None:
        partial_suffix = self._t("partial_suffix") if analysis.has_partial_shape_coverage else ""
        self.current_shapes.setText(
            "{}{}".format(analysis.current_shape_count, partial_suffix)
        )
        self.optimized_shapes.setText(
            "{}{}".format(analysis.optimized_shape_count, partial_suffix)
            if analysis.optimized_shape_count is not None
            else self._t("out_of_scope")
        )
        self.component_count.setText(str(analysis.component_count))
        manual_note = (
            self._t(
                "manual_exclusion_note",
                bodies=analysis.manually_excluded_body_count,
            )
            if analysis.manually_excluded_body_count
            else ""
        )
        if analysis.can_optimize:
            reduction = analysis.current_shape_count - (analysis.optimized_shape_count or 0)
            if analysis.has_partial_shape_coverage:
                eligibility = (
                    self._t(
                        "eligibility_partial",
                        reduction=reduction,
                        components=analysis.xml_edited_component_count,
                        bodies=analysis.partially_optimized_body_count,
                        flooders=analysis.flooder_prediction_excluded_body_count,
                    )
                )
            else:
                eligibility = self._t("eligibility_full", reduction=reduction)
            self.eligibility.setText("{}{}".format(eligibility, manual_note))
            self.optimize_button.setEnabled(not self.busy)
            self.optimized_radio.setEnabled(not self.busy)
        else:
            reasons = [
                self._runtime_text(body.reason)
                for body in analysis.bodies
                if not body.can_optimize
            ]
            self.eligibility.setText(
                self._t(
                    "eligibility_none",
                    reasons=" / ".join(reasons) or self._t("no_supported_body"),
                )
            )
            self.optimize_button.setEnabled(False)
            self.optimized_radio.setEnabled(False)
            self.current_radio.setChecked(True)

        selected_body = self.body_selector.currentData() if preserve_selection else -1
        self.body_selector.blockSignals(True)
        self.body_selector.clear()
        if analysis.bodies:
            self.body_selector.addItem(
                self._t("all_bodies", count=len(analysis.bodies)), -1
            )
        for body in analysis.bodies:
            suffix = self._t("manual_excluded_suffix") if body.manually_excluded else ""
            self.body_selector.addItem(
                "Body {}{}".format(body.body_index, suffix),
                body.body_index,
            )
        selected_index = self.body_selector.findData(selected_body)
        if selected_index < 0 and analysis.bodies:
            selected_index = 0
        if selected_index >= 0:
            self.body_selector.setCurrentIndex(selected_index)
        self.body_selector.blockSignals(False)
        self._sync_manual_body_exclusion_control()
        if not preserve_selection:
            self._fit_next_preview = True
        self._render_body_tree()
        self.update_preview()

        lines = [
            self._t("vehicle_log", path=analysis.vehicle_path),
            "Definitions: {}".format(analysis.definitions_path),
            self._t(
                "settings_log",
                mode=self._search_profile_label(analysis.search_mode),
                workers=(
                    self._t("auto")
                    if analysis.requested_worker_count == 0
                    else analysis.requested_worker_count
                ),
            ),
            self._t("engine_log", engine=native_backend_status()),
            "",
        ]
        for body in analysis.bodies:
            lines.append(
                "Body {0} (ID {1}): components={2}, physics voxels={3}, "
                "F2 Shapes={4}, optimized={5}, non-cube={6}, flood-fill={7}, "
                "overlapping positions={8}, extra collision shapes={9}, "
                "evaluated orders={10}, stages={11}, workers={12}, "
                "xml-edited excluded components={13}, excluded voxels={14}, "
                "protected body={15}, manually excluded={16}, "
                "Flooder prediction excluded={17}".format(
                    body.body_index,
                    body.body_id,
                    body.component_count,
                    body.physics_voxel_count,
                    body.current_shape_count,
                    body.effective_optimized_shape_count if body.effective_optimized_shape_count is not None else self._t("out_of_scope"),
                    body.unsupported_voxel_count,
                    body.generated_fill_voxel_count,
                    body.overlapping_cube_count,
                    body.extra_collision_shape_count,
                    body.evaluated_order_count,
                    body.completed_search_stage_count,
                    body.worker_count,
                    body.xml_edited_component_count,
                    body.xml_edited_physics_voxel_count,
                    body.protected_body,
                    body.manually_excluded,
                    body.flooder_prediction_excluded,
                )
            )
            lines.append("  {}".format(self._runtime_text(body.reason)))
            for overlap in body.overlap_details:
                lines.append("  overlap {}".format(overlap))
        if analysis.warnings:
            lines.extend(("", self._t("warnings")))
            lines.extend(
                "- {}".format(self._runtime_text(warning))
                for warning in analysis.warnings
            )
        self.details.setPlainText("\n".join(lines))

    def update_preview(self) -> None:
        if (
            hasattr(self, "body_mode_button")
            and self.body_mode_button.isChecked()
        ):
            self._update_body_management_preview()
            return
        analysis = self.last_analysis
        if analysis is None or not analysis.bodies:
            self.viewer.set_shapes((), fit=self._fit_next_preview)
            self._fit_next_preview = False
            self.preview_caption.setText(self._t("interaction_hint"))
            return
        fit_preview = self._fit_next_preview
        reference_meshes = (
            (
                mesh
                for body in analysis.bodies
                for mesh_group in (
                    body.current_meshes,
                    body.effective_optimized_meshes or (),
                )
                for mesh in mesh_group
            )
            if fit_preview
            else None
        )
        optimized = self.optimized_radio.isChecked()
        selected_body_index = self.body_selector.currentData()
        if selected_body_index == -1:
            showing_optimized = optimized and all(
                body.effective_optimized_meshes is not None
                for body in analysis.bodies
            )
            meshes = tuple(
                mesh
                for body in analysis.bodies
                for mesh in (
                    body.effective_optimized_meshes
                    if showing_optimized and body.effective_optimized_meshes is not None
                    else body.current_meshes
                )
            )
            scope_label = self._t("all_bodies_scope")
            excluded_component_count = analysis.xml_edited_component_count
            partially_optimized_body_count = analysis.partially_optimized_body_count
            flooder_excluded_body_count = (
                analysis.flooder_prediction_excluded_body_count
            )
            manually_excluded_body_count = analysis.manually_excluded_body_count
        else:
            body = next(
                (
                    candidate
                    for candidate in analysis.bodies
                    if candidate.body_index == selected_body_index
                ),
                analysis.bodies[0],
            )
            meshes = (
                body.effective_optimized_meshes
                if optimized and body.effective_optimized_meshes is not None
                else body.current_meshes
            )
            showing_optimized = optimized and body.effective_optimized_meshes is not None
            scope_label = "Body {}".format(body.body_index)
            excluded_component_count = body.xml_edited_component_count
            partially_optimized_body_count = int(
                bool(
                    body.xml_edited_component_count
                    or body.flooder_prediction_excluded
                )
                and not body.protected_body
            )
            flooder_excluded_body_count = int(body.flooder_prediction_excluded)
            manually_excluded_body_count = int(body.manually_excluded)
        self.viewer.set_shapes(
            meshes,
            fit=fit_preview,
            reference_meshes=reference_meshes,
        )
        self._fit_next_preview = False
        label = self._t("optimized") if showing_optimized else self._t("current")
        excluded_label = (
            self._t(
                "preview_excluded",
                components=excluded_component_count,
                bodies=partially_optimized_body_count,
                flooders=flooder_excluded_body_count,
            )
            if excluded_component_count or flooder_excluded_body_count
            else ""
        )
        if manually_excluded_body_count:
            excluded_label += self._t(
                "preview_manual_excluded",
                bodies=manually_excluded_body_count,
            )
        self.preview_caption.setText(
            self._t(
                "preview_caption",
                scope=scope_label,
                state=label,
                count=len(meshes),
                excluded=excluded_label,
                hint=self._t("interaction_hint"),
            )
        )

    def _update_body_management_preview(self) -> None:
        analysis = self.last_analysis
        if analysis is None or not analysis.bodies:
            self.viewer.set_body_groups((), fit=self._fit_next_preview)
            self.viewer.set_hovered_body(None)
            self._fit_next_preview = False
            self.preview_caption.setText(self._t("body_interaction_hint"))
            return

        fit_preview = self._fit_next_preview
        focus_body_ids = self._fit_body_ids
        reference_meshes = (
            tuple(
                mesh
                for body in analysis.bodies
                if focus_body_ids is None or body.body_index in focus_body_ids
                for mesh_group in (
                    body.current_meshes,
                    body.effective_optimized_meshes or (),
                )
                for mesh in mesh_group
            )
            if fit_preview
            else None
        )
        optimized = self.optimized_radio.isChecked()
        visible_selected_body_ids = (
            self.selected_body_ids - self.hidden_body_ids
        )
        has_selection = bool(visible_selected_body_ids)
        groups = []
        showing_optimized = False
        for body in analysis.bodies:
            if body.body_index in self.hidden_body_ids:
                continue
            use_optimized = (
                optimized and body.effective_optimized_meshes is not None
            )
            meshes = (
                body.effective_optimized_meshes
                if use_optimized
                else body.current_meshes
            )
            if meshes is None:
                meshes = body.current_meshes
            showing_optimized = showing_optimized or use_optimized
            selected = body.body_index in self.selected_body_ids
            if not has_selection:
                opacity = self.overview_opacity
            elif selected:
                opacity = 1.0
            elif self.ghost_nonselected:
                opacity = self.ghost_opacity
            else:
                opacity = 1.0
            groups.append(
                BodyRenderGroup(
                    body_index=body.body_index,
                    meshes=tuple(meshes),
                    color=self._body_color(body.body_index),
                    opacity=opacity,
                    selected=selected,
                )
            )

        self.viewer.set_body_groups(
            groups,
            fit=fit_preview,
            reference_meshes=reference_meshes,
            preserve_view_angles=(
                fit_preview and focus_body_ids is not None
            ),
        )
        self.viewer.set_hovered_body(self.hovered_body_id)
        self._fit_next_preview = False
        self._fit_body_ids = None
        excluded_component_count = analysis.xml_edited_component_count
        partially_optimized_body_count = analysis.partially_optimized_body_count
        flooder_excluded_body_count = (
            analysis.flooder_prediction_excluded_body_count
        )
        manually_excluded_body_count = analysis.manually_excluded_body_count
        excluded_label = (
            self._t(
                "preview_excluded",
                components=excluded_component_count,
                bodies=partially_optimized_body_count,
                flooders=flooder_excluded_body_count,
            )
            if excluded_component_count or flooder_excluded_body_count
            else ""
        )
        if manually_excluded_body_count:
            excluded_label += self._t(
                "preview_manual_excluded",
                bodies=manually_excluded_body_count,
            )
        self.preview_caption.setText(
            self._t(
                "preview_caption",
                scope=self._t("body_management_scope"),
                state=(
                    self._t("optimized")
                    if showing_optimized
                    else self._t("current")
                ),
                count=sum(len(group.meshes) for group in groups),
                excluded=excluded_label,
                hint=self._t("body_interaction_hint"),
            )
        )

    def choose_output_and_optimize(self) -> None:
        analysis = self.last_analysis
        if analysis is None or not analysis.can_optimize:
            return
        source = analysis.vehicle_path
        initial_directory = vehicle_directory() if vehicle_directory().is_dir() else source.parent
        suggested = initial_directory / (source.stem + " Optimized V1.xml")
        selected, _filter = QFileDialog.getSaveFileName(
            self,
            self._t("choose_output"),
            str(suggested),
            self._t("save_vehicle_filter"),
        )
        if not selected:
            return
        output = Path(selected)
        self.run_background(
            lambda report: save_analyzed_vehicle_copy(
                analysis,
                output,
                force=output.exists(),
                progress_callback=report,
            ),
            self.show_optimization,
            "解析済み結果から保存中…",
            reports_progress=True,
        )

    def show_optimization(self, raw_result: object) -> None:
        if not isinstance(raw_result, OptimizationOutput):
            raise TypeError("unexpected optimization result")
        result = raw_result
        self.last_output = result
        self.last_error_trace = None
        visible_before, visible_after, partial_note = self._render_optimization(result)
        self._show_status("保存完了（再解析なし）")
        QMessageBox.information(
            self,
            APP_NAME,
            self._t(
                "saved_message",
                before=visible_before,
                after=visible_after,
                path=result.report.output_path,
                partial=partial_note,
            ),
        )

    def _render_optimization(
        self,
        result: OptimizationOutput,
    ) -> tuple[int, int, str]:
        report = result.report
        visible_before = sum(body.result.before.shape_count for body in report.bodies)
        visible_after = sum(body.result.after.shape_count for body in report.bodies)
        excluded_components = sum(
            body.xml_edited_component_count for body in report.bodies
        )
        partially_optimized_bodies = (
            result.verified_analysis.partially_optimized_body_count
        )
        flooder_excluded_bodies = (
            result.verified_analysis.flooder_prediction_excluded_body_count
        )
        partial_note = (
            self._t(
                "saved_partial",
                components=excluded_components,
                bodies=partially_optimized_bodies,
                flooders=flooder_excluded_bodies,
            )
            if excluded_components or flooder_excluded_bodies
            else ""
        )
        manually_excluded_bodies = (
            result.verified_analysis.manually_excluded_body_count
        )
        if manually_excluded_bodies:
            partial_note += self._t(
                "saved_manual",
                bodies=manually_excluded_bodies,
            )
        self.details.setPlainText(
            "\n".join(
                (
                    self._t("saved_title"),
                    self._t("saved_output", path=report.output_path),
                    self._t(
                        "predicted_shapes",
                        before=visible_before,
                        after=visible_after,
                    ),
                    "SHA-256: {}".format(result.sha256),
                    "",
                    self._t("saved_validation"),
                    self._t("saved_cached"),
                    "{}{}".format(self._t("saved_verify_game"), partial_note),
                )
            )
        )
        return visible_before, visible_after, partial_note

    def open_vehicle_directory(self) -> None:
        directory = vehicle_directory()
        directory.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(directory)))


def resource_path(relative: str) -> Path:
    base = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parents[1]))
    return base / relative


def configure_application_metadata(application: QApplication) -> None:
    application.setApplicationName(APP_NAME)
    application.setApplicationVersion(APP_VERSION)
    application.setOrganizationName(APP_AUTHOR)
    application.setOrganizationDomain("irisnuiyama164")


def apply_style(application: QApplication) -> None:
    application.setStyle("Fusion")
    application.setStyleSheet(
        """
        QMainWindow, QWidget { background: #F8FAFC; color: #101828; }
        QGroupBox { border: 1px solid #D0D5DD; border-radius: 8px; margin-top: 10px; padding-top: 10px; font-weight: 600; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
        QLabel#title { font-size: 26px; font-weight: 700; color: #101828; }
        QLabel#subtitle, QLabel#metricName { color: #667085; }
        QLabel#bodyLayersTitle, QLabel#shortcutTitle { font-weight: 700; color: #344054; }
        QLabel#shortcutKeys { background: #F2F4F7; border: 1px solid #D0D5DD; border-radius: 4px; padding: 2px 5px; color: #344054; font-family: monospace; }
        QLabel#shortcutAction { color: #475467; }
        QLabel#metricValue { font-size: 27px; font-weight: 700; color: #6941C6; }
        QFrame#metricCard { background: #FFFFFF; border: 1px solid #EAECF0; border-radius: 8px; padding: 7px; }
        QFrame#bodyManagementPanel { background: #FFFFFF; border: 1px solid #D0D5DD; border-radius: 7px; }
        QFrame#bodyBulkBar { background: #F4F3FF; border: 1px solid #D9D6FE; border-radius: 6px; }
        QFrame#shortcutHelp { background: #FFFFFF; border: 1px solid #EAECF0; border-radius: 7px; }
        QPushButton { background: #FFFFFF; border: 1px solid #D0D5DD; border-radius: 6px; padding: 7px 12px; }
        QPushButton:hover { background: #F2F4F7; }
        QPushButton:checked { background: #EDE9FE; color: #53389E; border-color: #7F56D9; font-weight: 600; }
        QPushButton#primaryButton { background: #6941C6; color: white; border: 1px solid #6941C6; font-weight: 600; }
        QPushButton#primaryButton:hover { background: #7F56D9; }
        QPushButton:disabled { background: #EAECF0; color: #98A2B3; border-color: #D0D5DD; }
        QLineEdit, QComboBox, QPlainTextEdit, QTreeWidget { background: #FFFFFF; border: 1px solid #D0D5DD; border-radius: 5px; padding: 6px; selection-background-color: #7F56D9; }
        QProgressBar { background: #EAECF0; border: 1px solid #D0D5DD; border-radius: 6px; text-align: center; min-height: 18px; }
        QProgressBar::chunk { background: #6941C6; border-radius: 5px; }
        QTabWidget::pane { border: 1px solid #D0D5DD; background: #FFFFFF; }
        QTabBar::tab { background: #EAECF0; padding: 8px 16px; margin-right: 2px; }
        QTabBar::tab:selected { background: #FFFFFF; color: #6941C6; font-weight: 600; }
        QFrame#bodyManagementPanel { background: #FFFFFF; border: 1px solid #D0D5DD; border-radius: 7px; }
        QFrame#bodyBulkBar { background: #F4F3FF; border: 1px solid #D6BBFB; border-radius: 6px; }
        QFrame#shortcutHelp { background: #FFFFFF; border: 1px solid #D0D5DD; border-radius: 6px; }
        QTreeWidget { background: #FFFFFF; border: 1px solid #D0D5DD; alternate-background-color: #F8FAFC; }
        QTreeWidget::item:selected { background: #EDE9FE; color: #101828; }
        QLabel#shortcutKeys { color: #53389E; font-weight: 600; }
        QStatusBar { background: #F2F4F7; color: #475467; }
        """
    )


def self_test() -> int:
    from .definitions import ComponentDefinition
    from .model import IDENTITY_MATRIX
    from .native_merge import native_backend_available, native_backend_status
    from .non_cube_data import NON_CUBE_CLIP_PLANES
    from .portable_merge import SOURCE_BINARY_SHA256, STORMWORKS_BUILD_ID
    from .surface_graph import SurfaceMetadata
    from .vehicle import ComponentPlacement

    detected = find_definition_directory()
    native_available = native_backend_available()
    if (
        os.environ.get("SWPHYSICS_REQUIRE_NATIVE", "").strip().lower()
        in {"1", "true", "yes", "on"}
        and not native_available
    ):
        raise RuntimeError(
            "required native score engine is unavailable: {}".format(
                native_backend_status()
            )
        )
    metadata = SurfaceMetadata()
    if metadata.stormworks_build_id != STORMWORKS_BUILD_ID:
        raise RuntimeError("bundled surface metadata build mismatch")
    if metadata.binary_sha256 != SOURCE_BINARY_SHA256:
        raise RuntimeError("bundled surface metadata binary hash mismatch")
    if set(NON_CUBE_CLIP_PLANES) != set(range(1, 42)):
        raise RuntimeError("portable non-cube plane table is incomplete")
    microprocessor_definition = ComponentDefinition(
        definition_id="microprocessor",
        name="Microprocessor",
        flags=0,
        component_type=37,
        water_component_type=0,
        custom_door_type=0,
        constraint_type=0,
        mesh_data_name="",
        source_path=Path("microprocessor.xml"),
        voxels=(),
        surfaces=(),
        buoyancy_surfaces=(),
        compartment_sample_position=(0, 0, 0),
    )
    microprocessor = ComponentPlacement(
        index=0,
        definition_id="microprocessor",
        transform_index=0,
        position=(0, 0, 0),
        rotation=IDENTITY_MATRIX,
        microprocessor_width=3,
        microprocessor_length=2,
    )
    dynamic_voxels = tuple(
        microprocessor.physics_definition_voxels(microprocessor_definition)
    )
    dynamic_surfaces = tuple(
        microprocessor.buoyancy_definition_surfaces(
            microprocessor_definition
        )
    )
    expected_footprint = tuple(
        (x, 0, z) for x in range(3) for z in range(2)
    )
    if tuple(voxel.position for voxel in dynamic_voxels) != expected_footprint:
        raise RuntimeError("variable-size microprocessor voxel expansion failed")
    if (
        tuple(surface.position for surface in dynamic_surfaces)
        != expected_footprint
        or any(surface.orientation != 3 for surface in dynamic_surfaces)
    ):
        raise RuntimeError("microprocessor bottom buoyancy surfaces failed")
    print(
        json.dumps(
            {
                "status": "ok",
                "app": APP_NAME,
                "version": APP_VERSION,
                "author": APP_AUTHOR,
                "python": sys.version.split()[0],
                "platform": sys.platform,
                "qt": qVersion(),
                "physics_backend": "portable_build_24749959",
                "score_engine": native_backend_status(),
                "native_score_engine": native_available,
                "physics_shape_ids": "0..41",
                "surface_type_count": len(metadata.types),
                "surface_transform_count": 48,
                "microprocessor_physics_voxel_count": len(dynamic_voxels),
                "microprocessor_bottom_surface_count": len(dynamic_surfaces),
                "definitions_detected": str(detected) if detected else None,
            },
            ensure_ascii=False,
        )
    )
    return 0


def worker_self_test() -> int:
    """Exercise BOM vehicle parsing and Worker delivery in the packaged GUI."""

    from tempfile import TemporaryDirectory

    from .vehicle import parse_vehicle_tree

    application = QApplication.instance() or QApplication([])
    configure_application_metadata(application)
    window = OptimizerWindow()
    results = []
    event_loop = QEventLoop()

    def done(value: object) -> None:
        results.append(value)
        event_loop.quit()

    with TemporaryDirectory(prefix="swphysics-worker-") as temp_directory:
        vehicle_path = Path(temp_directory) / "bom_vehicle.xml"
        vehicle_path.write_bytes(
            b"\xef\xbb\xbf<vehicle data_version=\"3\"><bodies/></vehicle>"
        )
        window.run_background(
            lambda: parse_vehicle_tree(vehicle_path).getroot().tag,
            done,
            "worker self-test",
        )
        QTimer.singleShot(2000, event_loop.quit)
        event_loop.exec()
    if results != ["vehicle"] or window.busy:
        raise RuntimeError("BOM vehicle Worker completion was not delivered")
    window.close()
    print(
        json.dumps(
            {
                "status": "ok",
                "app": APP_NAME,
                "version": APP_VERSION,
                "bom_vehicle_worker_result": results[0],
                "busy": window.busy,
            },
            ensure_ascii=False,
        )
    )
    return 0


def parallel_self_test() -> int:
    """Exercise the frozen-safe spawned candidate evaluator."""

    from .exact_optimizer import optimize_exact_component_order
    from .model import IDENTITY_MATRIX, WorldVoxel
    from .portable_merge import PortableMergeOracle

    # This connected six-voxel fixture deliberately starts at four Shapes and
    # has a one-Shape occupancy lower bound.  The former diagonal fixture was
    # already at its disconnected occupancy lower bound, so both calls stopped
    # after the original order and the "parallel" self-test never spawned a
    # worker process at all.
    positions = (
        (0, 0, 0),
        (0, 2, 0),
        (0, 1, 0),
        (0, 3, 0),
        (1, 0, 0),
        (1, 2, 0),
    )
    groups = tuple(
        (
            WorldVoxel(
                body_index=0,
                body_id="1",
                component_index=index,
                component_definition="parallel_test_{}".format(index),
                definition_voxel_index=0,
                insertion_index=index,
                position=position,
                physics_shape=0,
                physics_rotation=IDENTITY_MATRIX,
            ),
        )
        for index, position in enumerate(positions)
    )
    serial = optimize_exact_component_order(
        groups,
        PortableMergeOracle(allow_overlaps=True),
        search_rounds=1,
        max_evaluations=8,
        worker_count=1,
    )
    parallel = optimize_exact_component_order(
        groups,
        PortableMergeOracle(allow_overlaps=True),
        search_rounds=1,
        max_evaluations=8,
        worker_count=2,
    )
    if (
        parallel.evaluated_order_count <= 1
        or parallel.worker_count <= 1
        or serial.before.shape_count != parallel.before.shape_count
        or serial.after.shape_count != parallel.after.shape_count
        or serial.optimized_component_order != parallel.optimized_component_order
    ):
        raise RuntimeError("serial and parallel search results differ")
    print(
        json.dumps(
            {
                "status": "ok",
                "app": APP_NAME,
                "version": APP_VERSION,
                "evaluations": parallel.evaluated_order_count,
                "workers": parallel.worker_count,
                "shape_count": parallel.after.shape_count,
            },
            ensure_ascii=False,
        )
    )
    return 0


def gpu_self_test() -> int:
    """Prove that the packaged Qt Quick 3D scene renders through a GPU API."""

    from .partition import Box
    from .physics_viewer import PhysicsShapeViewer
    from .viewer import BodyRenderGroup, box_mesh

    application = QApplication([])
    configure_application_metadata(application)
    viewer = PhysicsShapeViewer()
    if not viewer.using_gpu:
        raise RuntimeError("Qt Quick 3D GPU viewer fell back to software")
    viewer.resize(640, 420)
    viewer.set_boxes((Box((3, 0, 5), (3, 0, 5)),))
    preview_center_x = sum(
        vertex[0] for vertex in viewer.meshes[0].vertices
    ) / len(viewer.meshes[0].vertices)
    preview_center_z = sum(
        vertex[2] for vertex in viewer.meshes[0].vertices
    ) / len(viewer.meshes[0].vertices)
    if abs(preview_center_x + 3.0) > 1e-7 or abs(preview_center_z + 5.0) > 1e-7:
        raise RuntimeError("Stormworks preview Y-axis rotation was not applied")
    viewer.show()
    loop = QEventLoop()
    QTimer.singleShot(750, loop.quit)
    loop.exec()
    application.processEvents()
    image = viewer.capture_image()
    if image.isNull() or image.width() < 1 or image.height() < 1:
        raise RuntimeError("Qt Quick 3D framebuffer capture failed")
    center = image.pixelColor(image.width() // 2, image.height() // 2)
    background = "#101828"
    if center.alpha() != 255 or center.name().lower() == background:
        raise RuntimeError(
            "Qt Quick 3D did not render the test shape at the viewport centre"
        )
    left_body_mesh = box_mesh(Box((-1, 0, 0), (-1, 0, 0)))
    right_body_mesh = box_mesh(Box((1, 0, 0), (1, 0, 0)))
    body_groups = (
        BodyRenderGroup(
            body_index=0,
            meshes=(left_body_mesh,),
            color="#E4782F",
            selected=True,
        ),
        BodyRenderGroup(
            body_index=1,
            meshes=(right_body_mesh,),
            color="#22A98D",
            opacity=0.25,
        ),
    )
    viewer.set_body_groups(
        body_groups,
        fit=True,
    )
    application.processEvents()
    backend = viewer.viewer
    if (
        not backend.body_interaction_enabled
        or backend.geometry.data.triangle_count != 12
        or backend.ghost_geometry.data.triangle_count != 12
        or backend.selection_outlines.data.segment_count != 12
        or abs(float(backend.root.property("ghostOpacity")) - 0.25) > 1e-7
    ):
        raise RuntimeError("Qt Quick 3D Body management layers were not rendered")
    body_layer_triangles = (
        backend.geometry.data.triangle_count,
        backend.ghost_geometry.data.triangle_count,
    )
    full_body_scene = viewer.scene_state()
    viewer.set_view_angles(21.0, -8.0)
    focus_angles = viewer.camera_state()[:2]
    viewer.set_body_groups(
        body_groups,
        fit=True,
        reference_meshes=(left_body_mesh,),
        preserve_view_angles=True,
    )
    if (
        viewer.camera_state()[:2] != focus_angles
        or viewer.scene_state() == full_body_scene
    ):
        raise RuntimeError("Body focus reset the camera orientation")
    viewer.set_boxes((Box((0, 0, 0), (0, 0, 0)),), fit=False)
    if (
        backend.body_interaction_enabled
        or backend.ghost_geometry.data.triangle_count != 0
        or backend.selection_outlines.data.segment_count != 0
    ):
        raise RuntimeError("Shape mode did not clear the Body management layers")
    print(
        json.dumps(
            {
                "status": "ok",
                "app": APP_NAME,
                "version": APP_VERSION,
                "renderer": viewer.backend_name(),
                "framebuffer": [image.width(), image.height()],
                "center": center.name(),
                "preview_center_x": preview_center_x,
                "preview_center_z": preview_center_z,
                "triangles": viewer.viewer.geometry.data.triangle_count,
                "line_segments": viewer.viewer.outlines.data.segment_count,
                "body_layer_triangles": body_layer_triangles,
                "body_focus_preserved_angles": True,
                "body_mode_cleared": not backend.body_interaction_enabled,
            },
            ensure_ascii=False,
        )
    )
    viewer.close()
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog=APP_NAME)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--gpu-self-test", action="store_true")
    parser.add_argument("--worker-self-test", action="store_true")
    parser.add_argument("--parallel-self-test", action="store_true")
    parser.add_argument("--version", action="store_true")
    args = parser.parse_args(argv)
    if args.version:
        print(APP_VERSION)
        return 0
    if args.self_test:
        return self_test()
    if args.gpu_self_test:
        return gpu_self_test()
    if args.worker_self_test:
        return worker_self_test()
    if args.parallel_self_test:
        return parallel_self_test()
    application = QApplication(list(argv) if argv is not None else sys.argv)
    configure_application_metadata(application)
    icon = resource_path("assets/app_icon.png")
    if icon.is_file():
        application.setWindowIcon(QIcon(str(icon)))
    apply_style(application)
    window = OptimizerWindow()
    window.show()
    return application.exec()
