import argparse
import json
import os
from pathlib import Path
import re
import sys
from threading import Event
import traceback
from typing import Callable, Optional, Sequence

from PySide6.QtCore import (
    QEventLoop,
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
from PySide6.QtGui import QDesktopServices, QDragEnterEvent, QDropEvent, QFont, QIcon
from PySide6.QtWidgets import (
    QApplication,
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
    QSizePolicy,
    QStatusBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from . import __author__
from .app_service import (
    APP_VERSION,
    SEARCH_MODE_PROFILES,
    OptimizationOutput,
    VehicleAnalysis,
    analyze_vehicle,
    save_analyzed_vehicle_copy,
    search_mode_profile,
)
from .platform_paths import find_definition_directory, load_config, save_config, vehicle_directory
from .native_merge import native_backend_status
from .physics_viewer import PhysicsShapeViewer


APP_NAME = "Stormworks Physics Shape Optimizer"
APP_AUTHOR = __author__

DEFAULT_LANGUAGE = "ja"
SUPPORTED_LANGUAGES = ("ja", "en")


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
        "reset_view": "視点リセット",
        "preview_tab": "3Dプレビュー",
        "details_placeholder": "解析結果の詳細がここに表示されます",
        "details_tab": "詳細ログ",
        "interaction_hint": "左ドラッグ: 回転 / ホイール: ズーム / ダブルクリック: リセット",
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
        "partial_suffix": " + 対象外",
        "out_of_scope": "対象外",
        "auto": "自動",
        "warnings": "警告:",
        "vehicle_log": "車両: {path}",
        "settings_log": "探索モード: {mode} / CPUワーカー設定: {workers}",
        "engine_log": "Shape評価エンジン: {engine}",
        "preview_excluded": " / XML編集等{components} Componentを含む{bodies} Bodyは表示対象外",
        "preview_caption": "{scope} / {state} / {count} Shapes{excluded}　　{hint}",
        "eligibility_full": "最適化可能：予測で{reduction} Shape削減",
        "eligibility_partial": "部分最適化可能：予測対象内で{reduction} Shape削減 / XML編集等{components} Componentを含む{bodies} Bodyは全体固定・対象外",
        "eligibility_none": "変更対象外：{reasons}",
        "no_supported_body": "対応bodyなし",
        "saved_title": "保存完了",
        "saved_output": "出力: {path}",
        "predicted_shapes": "予測Shape: {before} → {after}",
        "saved_validation": "XMLを再読込し、Component順序と構造を検証済みです。",
        "saved_cached": "解析済み結果を再利用したため、保存時の再解析はありません。",
        "saved_verify_game": "実ゲームでのShape境界はF2表示で確認してください。",
        "saved_partial": "\nXML編集等{components} Componentを含む{bodies} Bodyは元のComponent順序を全て保持し、Shape予測・3D表示の対象外です。",
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
        "reset_view": "Reset View",
        "preview_tab": "3D Preview",
        "details_placeholder": "Detailed analysis results will appear here",
        "details_tab": "Detailed Log",
        "interaction_hint": "Left drag: Rotate / Wheel: Zoom / Double-click: Reset",
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
        "partial_suffix": " + excluded",
        "out_of_scope": "Excluded",
        "auto": "Auto",
        "warnings": "Warnings:",
        "vehicle_log": "Vehicle: {path}",
        "settings_log": "Search mode: {mode} / CPU workers: {workers}",
        "engine_log": "Shape evaluation engine: {engine}",
        "preview_excluded": " / {components} XML-edited or unsupported Components in {bodies} Bodies are excluded from preview",
        "preview_caption": "{scope} / {state} / {count} Shapes{excluded}    {hint}",
        "eligibility_full": "Optimization available: predicted reduction of {reduction} Shapes",
        "eligibility_partial": "Partial optimization available: predicted reduction of {reduction} Shapes in supported Bodies / {components} XML-edited or unsupported Components in {bodies} Bodies are locked in their original order",
        "eligibility_none": "No changes available: {reasons}",
        "no_supported_body": "no supported Bodies",
        "saved_title": "Save Complete",
        "saved_output": "Output: {path}",
        "predicted_shapes": "Predicted Shapes: {before} → {after}",
        "saved_validation": "The XML was reloaded and its Component order and structure were verified.",
        "saved_cached": "The analyzed result was reused; the vehicle was not analyzed again while saving.",
        "saved_verify_game": "Verify the final Shape boundaries in-game with the F2 overlay.",
        "saved_partial": "\n{components} XML-edited or unsupported Components in {bodies} Bodies kept their complete original Component order and are excluded from Shape prediction and 3D preview.",
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
        (r"XML編集または未対応Shapeの(\d+) Componentがあるため、相互作用を変えないようBody全体を元の順序に固定しました", "Locked the entire Body in its original order because it contains {0} XML-edited or unsupported Components"),
        (r"XML編集または未対応Shapeの(\d+) Component \((\d+) physics voxel\)を含むためBody全体を順序固定・予測対象外にしました", "Locked the complete Body order and excluded it from prediction because it contains {0} XML-edited or unsupported Components ({1} physics voxels)"),
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
        self.busy = False
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
        toolbar = QHBoxLayout()
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
        self.body_selector.currentIndexChanged.connect(self.update_preview)
        toolbar.addWidget(self.body_selector)
        self.reset_view_button = QPushButton()
        self.reset_view_button.clicked.connect(self.reset_view)
        toolbar.addWidget(self.reset_view_button)
        toolbar.addStretch(1)
        preview_layout.addLayout(toolbar)
        self.viewer = PhysicsShapeViewer()
        self.viewer.setMinimumHeight(420)
        preview_layout.addWidget(self.viewer, 1)
        self.preview_caption = QLabel()
        self.preview_caption.setAlignment(Qt.AlignHCenter)
        preview_layout.addWidget(self.preview_caption)
        self.tabs.addTab(self.preview_tab, "")

        self.details = QPlainTextEdit()
        self.details.setReadOnly(True)
        self.tabs.addTab(self.details, "")

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
        self.display_label.setText(self._t("display"))
        self.current_radio.setText(self._t("current"))
        self.optimized_radio.setText(self._t("optimized"))
        self.display_scope_label.setText(self._t("display_scope"))
        self.reset_view_button.setText(self._t("reset_view"))
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
        self.viewer.reset_view()

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
        self.optimize_button.setEnabled(False)
        self.optimized_radio.setEnabled(False)
        self.current_radio.setChecked(True)
        self.current_shapes.setText("—")
        self.optimized_shapes.setText("—")
        self.component_count.setText("—")
        self.eligibility.setText(self._t("not_analyzed"))
        self.body_selector.clear()
        self.viewer.set_shapes((), fit=True)
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
            self.optimize_button.setEnabled(False)

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
            self.optimize_button.setEnabled(False)
            self.optimized_radio.setEnabled(False)
            self.current_radio.setChecked(True)
            self._show_status("解析設定が変わりました。もう一度解析してください")

    @Slot(str)
    def analysis_input_edited(self, _text: str = "") -> None:
        self.last_analysis = None
        self.last_output = None
        self.last_error_trace = None
        self.optimize_button.setEnabled(False)
        self.optimized_radio.setEnabled(False)
        self.current_radio.setChecked(True)
        self.eligibility.setText(self._t("not_analyzed"))

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
        if analysis.can_optimize:
            reduction = analysis.current_shape_count - (analysis.optimized_shape_count or 0)
            if analysis.has_partial_shape_coverage:
                self.eligibility.setText(
                    self._t(
                        "eligibility_partial",
                        reduction=reduction,
                        components=analysis.xml_edited_component_count,
                        bodies=analysis.protected_body_count,
                    )
                )
            else:
                self.eligibility.setText(
                    self._t("eligibility_full", reduction=reduction)
                )
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
            self.body_selector.addItem("Body {}".format(body.body_index), body.body_index)
        selected_index = self.body_selector.findData(selected_body)
        if selected_index < 0 and analysis.bodies:
            selected_index = 0
        if selected_index >= 0:
            self.body_selector.setCurrentIndex(selected_index)
        self.body_selector.blockSignals(False)
        if not preserve_selection:
            self._fit_next_preview = True
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
                "protected body={15}".format(
                    body.body_index,
                    body.body_id,
                    body.component_count,
                    body.physics_voxel_count,
                    body.current_shape_count,
                    body.optimized_shape_count if body.optimized_shape_count is not None else self._t("out_of_scope"),
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
        analysis = self.last_analysis
        if analysis is None or not analysis.bodies:
            self.viewer.set_shapes((), fit=self._fit_next_preview)
            self._fit_next_preview = False
            self.preview_caption.setText(self._t("interaction_hint"))
            return
        optimized = self.optimized_radio.isChecked()
        selected_body_index = self.body_selector.currentData()
        if selected_body_index == -1:
            showing_optimized = optimized and all(
                body.optimized_meshes is not None for body in analysis.bodies
            )
            meshes = tuple(
                mesh
                for body in analysis.bodies
                for mesh in (
                    body.optimized_meshes
                    if showing_optimized and body.optimized_meshes is not None
                    else body.current_meshes
                )
            )
            scope_label = self._t("all_bodies_scope")
            excluded_component_count = analysis.xml_edited_component_count
            protected_body_count = analysis.protected_body_count
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
                body.optimized_meshes
                if optimized and body.optimized_meshes is not None
                else body.current_meshes
            )
            showing_optimized = optimized and body.optimized_meshes is not None
            scope_label = "Body {}".format(body.body_index)
            excluded_component_count = body.xml_edited_component_count
            protected_body_count = int(body.protected_body)
        self.viewer.set_shapes(meshes, fit=self._fit_next_preview)
        self._fit_next_preview = False
        label = self._t("optimized") if showing_optimized else self._t("current")
        excluded_label = (
            self._t(
                "preview_excluded",
                components=excluded_component_count,
                bodies=protected_body_count,
            )
            if excluded_component_count
            else ""
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
        protected_bodies = sum(body.protected_body for body in report.bodies)
        partial_note = (
            self._t(
                "saved_partial",
                components=excluded_components,
                bodies=protected_bodies,
            )
            if excluded_components
            else ""
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
        QLabel#metricValue { font-size: 27px; font-weight: 700; color: #6941C6; }
        QFrame#metricCard { background: #FFFFFF; border: 1px solid #EAECF0; border-radius: 8px; padding: 7px; }
        QPushButton { background: #FFFFFF; border: 1px solid #D0D5DD; border-radius: 6px; padding: 7px 12px; }
        QPushButton:hover { background: #F2F4F7; }
        QPushButton#primaryButton { background: #6941C6; color: white; border: 1px solid #6941C6; font-weight: 600; }
        QPushButton#primaryButton:hover { background: #7F56D9; }
        QPushButton:disabled { background: #EAECF0; color: #98A2B3; border-color: #D0D5DD; }
        QLineEdit, QComboBox, QPlainTextEdit { background: #FFFFFF; border: 1px solid #D0D5DD; border-radius: 5px; padding: 6px; selection-background-color: #7F56D9; }
        QProgressBar { background: #EAECF0; border: 1px solid #D0D5DD; border-radius: 6px; text-align: center; min-height: 18px; }
        QProgressBar::chunk { background: #6941C6; border-radius: 5px; }
        QTabWidget::pane { border: 1px solid #D0D5DD; background: #FFFFFF; }
        QTabBar::tab { background: #EAECF0; padding: 8px 16px; margin-right: 2px; }
        QTabBar::tab:selected { background: #FFFFFF; color: #6941C6; font-weight: 600; }
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

    application = QApplication([])
    configure_application_metadata(application)
    viewer = PhysicsShapeViewer()
    if not viewer.using_gpu:
        raise RuntimeError("Qt Quick 3D GPU viewer fell back to software")
    viewer.resize(640, 420)
    viewer.set_boxes((Box((3, 0, 0), (3, 0, 0)),))
    preview_center_x = sum(
        vertex[0] for vertex in viewer.meshes[0].vertices
    ) / len(viewer.meshes[0].vertices)
    if abs(preview_center_x + 3.0) > 1e-7:
        raise RuntimeError("Stormworks preview X conversion was not applied")
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
                "triangles": viewer.viewer.geometry.data.triangle_count,
                "line_segments": viewer.viewer.outlines.data.segment_count,
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
