import json
import os
from pathlib import Path
from string import Formatter
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QMessageBox

from swphysics.app_service import analyze_vehicle, save_analyzed_vehicle_copy
from swphysics.gui import (
    _TEXT,
    OptimizerWindow,
    configure_application_metadata,
    translate_runtime_message,
)


ROOT = Path(__file__).resolve().parents[1]
VEHICLE = ROOT / "tests/fixtures/vehicles/order_b.xml"
DEFINITIONS = ROOT / "tests/fixtures/definitions"


class GuiI18nTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])
        configure_application_metadata(cls.application)

    def setUp(self):
        self.temporary_directory = TemporaryDirectory(prefix="swphysics-i18n-")
        self.addCleanup(self.temporary_directory.cleanup)
        self.config_path = Path(self.temporary_directory.name) / "config.json"
        self.environment = patch.dict(
            os.environ,
            {"SWPHYSICS_CONFIG_FILE": str(self.config_path)},
        )
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def make_window(self, config=None):
        if config is not None:
            self.config_path.write_text(json.dumps(config), encoding="utf-8")
        window = OptimizerWindow()
        self.addCleanup(window.close)
        return window

    def test_japanese_remains_the_default(self):
        window = self.make_window()

        self.assertEqual("ja", window.language)
        self.assertEqual("ja", window.language_selector.currentData())
        self.assertEqual("解析する", window.analyze_button.text())
        self.assertEqual(
            "車両XMLを選んで解析してください",
            window.status.currentMessage(),
        )
        self.assertIn("最大3段階", self._search_hint(window, "deep"))

    def test_saved_english_language_is_applied_at_startup(self):
        window = self.make_window(
            {
                "definitions_path": str(DEFINITIONS),
                "search_mode": "deep",
                "worker_count": "4",
                "language": "en",
            }
        )

        self.assertEqual("en", window.language)
        self.assertEqual("Analyze", window.analyze_button.text())
        self.assertEqual("Save Optimized Copy…", window.optimize_button.text())
        self.assertIn("Up to 3 stages", window.search_hint.text())
        self.assertIn("CPU workers 4", window.search_hint.text())
        self.assertEqual(
            "Select a vehicle XML to begin analysis",
            window.status.currentMessage(),
        )
        self.assertEqual(
            "Physics Shapes will appear here after analysis",
            window.viewer.viewer.empty_message,
        )

    def test_unknown_or_non_string_saved_language_falls_back_to_japanese(self):
        for language in ("unknown", 42, ["en"]):
            with self.subTest(language=language):
                window = self.make_window({"language": language})
                self.assertEqual("ja", window.language)
                self.assertEqual("解析する", window.analyze_button.text())
                window.close()

    def test_translation_catalog_keys_and_placeholders_match(self):
        self.assertEqual(set(_TEXT["ja"]), set(_TEXT["en"]))
        formatter = Formatter()
        for key in _TEXT["ja"]:
            placeholders = []
            for language in ("ja", "en"):
                placeholders.append(
                    {
                        field_name
                        for _literal, field_name, _format_spec, _conversion in formatter.parse(
                            _TEXT[language][key]
                        )
                        if field_name is not None
                    }
                )
            self.assertEqual(placeholders[0], placeholders[1], key)

    def test_live_switch_preserves_result_view_and_busy_progress(self):
        window = self.make_window()
        analysis = analyze_vehicle(VEHICLE, DEFINITIONS, search_mode="standard")
        window.vehicle_edit.setText(str(VEHICLE))
        window.definitions_edit.setText(str(DEFINITIONS))
        window.show_analysis(analysis)
        window.optimized_radio.setChecked(True)
        window.viewer.set_view_angles(13.0, -7.0)
        camera_before = window.viewer.camera_state()
        selected_body_before = window.body_selector.currentData()
        search_mode_before = window.search_mode_selector.currentData()
        worker_count_before = window.worker_selector.currentData()

        window._active_cancellable = True
        window.set_busy(True, "解析中…")
        window.progress_widget.setVisible(True)
        window.job_progress(47, "Body 1/2: 配置候補を評価中… 15/32")
        window.language_selector.setCurrentIndex(
            window.language_selector.findData("en")
        )
        self.application.processEvents()

        self.assertIs(analysis, window.last_analysis)
        self.assertTrue(window.busy)
        self.assertTrue(window.optimized_radio.isChecked())
        self.assertEqual(selected_body_before, window.body_selector.currentData())
        self.assertEqual(search_mode_before, window.search_mode_selector.currentData())
        self.assertEqual(worker_count_before, window.worker_selector.currentData())
        self.assertEqual(camera_before, window.viewer.camera_state())
        self.assertEqual("Analyze", window.analyze_button.text())
        self.assertEqual("Stop Analysis", window.stop_button.text())
        self.assertEqual(
            "Body 1/2: Evaluating candidates… 15/32",
            window.progress_label.text(),
        )
        self.assertEqual(
            "Body 1/2: Evaluating candidates… 15/32 (47%)",
            window.status.currentMessage(),
        )
        self.assertIn("Vehicle:", window.details.toPlainText())
        self.assertIn("Search mode: Standard (Fast)", window.details.toPlainText())
        self.assertIn("All Bodies", window.body_selector.itemText(0))

        persisted = json.loads(self.config_path.read_text(encoding="utf-8"))
        self.assertEqual("en", persisted["language"])
        self.assertEqual("standard", persisted["search_mode"])
        self.assertEqual("0", persisted["worker_count"])

        window.language_selector.setCurrentIndex(
            window.language_selector.findData("ja")
        )
        self.application.processEvents()
        self.assertIn("配置候補", window.progress_label.text())
        self.assertIn("車両:", window.details.toPlainText())
        self.assertIs(analysis, window.last_analysis)
        self.assertEqual(camera_before, window.viewer.camera_state())
        window._active_cancellable = False
        window.set_busy(False, "完了")

    def test_saved_result_log_and_dialog_switch_without_reopening_dialog(self):
        window = self.make_window({"language": "en"})
        analysis = analyze_vehicle(VEHICLE, DEFINITIONS, search_mode="standard")
        output_path = Path(self.temporary_directory.name) / "optimized.xml"
        result = save_analyzed_vehicle_copy(analysis, output_path)
        window.show_analysis(analysis)

        with patch.object(QMessageBox, "information") as information:
            window.show_optimization(result)
            self.assertIn("The optimized copy was saved", information.call_args.args[2])
            self.assertIn("Save Complete", window.details.toPlainText())

            window.language_selector.setCurrentIndex(
                window.language_selector.findData("ja")
            )
            self.application.processEvents()

        self.assertEqual(1, information.call_count)
        self.assertIn("保存完了", window.details.toPlainText())
        self.assertIs(result, window.last_output)

    def test_custom_search_mode_and_unsupported_warning_render_in_english(self):
        window = self.make_window({"language": "en"})
        custom = analyze_vehicle(VEHICLE, DEFINITIONS, max_evaluations=16)
        window.show_analysis(custom)
        self.assertIn("Search mode: custom", window.details.toPlainText())

        unsupported = analyze_vehicle(
            VEHICLE,
            DEFINITIONS,
            max_blocks_per_body=1,
            max_evaluations=16,
        )
        window.show_analysis(unsupported)
        details = window.details.toPlainText()
        self.assertIn("exceeding the configured limit", details)
        self.assertNotRegex(details, r"[぀-ヿ㐀-鿿]")

    def test_error_dialog_uses_the_selected_language(self):
        window = self.make_window({"language": "en"})
        trace = (
            "Traceback (most recent call last):\n"
            "FileNotFoundError: 車両XMLが見つかりません: /tmp/missing.xml"
        )
        with patch.object(QMessageBox, "critical") as critical:
            window.job_failed(trace)

        self.assertEqual(
            "FileNotFoundError: Vehicle XML not found: /tmp/missing.xml",
            critical.call_args.args[2],
        )
        self.assertIn("Vehicle XML not found", window.details.toPlainText())

        window.language_selector.setCurrentIndex(
            window.language_selector.findData("ja")
        )
        self.application.processEvents()
        self.assertIn("車両XMLが見つかりません", window.details.toPlainText())

    def test_nested_body_progress_and_protection_warning_are_translated(self):
        messages = (
            "Body 1/2: 解析済みComponent順序を適用中…",
            "Body 1/2: Component順序と属性を検証中…",
            "Body 0: XML編集または未対応Shapeの1 Component "
            "(3 physics voxel)を含むためBody全体を順序固定・"
            "予測対象外にしました",
        )

        translated = tuple(translate_runtime_message(message, "en") for message in messages)

        self.assertIn("Applying analyzed Component order", translated[0])
        self.assertIn("Verifying Component order and attributes", translated[1])
        self.assertIn("Locked the complete Body order", translated[2])
        for message in translated:
            self.assertNotRegex(message, r"[぀-ヿ㐀-鿿]")

    def test_manual_input_edit_invalidates_a_cached_optimization(self):
        window = self.make_window()
        analysis = analyze_vehicle(VEHICLE, DEFINITIONS, search_mode="standard")
        window.show_analysis(analysis)
        self.assertTrue(window.optimize_button.isEnabled())

        window.analysis_input_edited("different.xml")

        self.assertIsNone(window.last_analysis)
        self.assertIsNone(window.last_output)
        self.assertFalse(window.optimize_button.isEnabled())
        self.assertEqual("未解析", window.eligibility.text())

    @staticmethod
    def _search_hint(window, mode):
        window.search_mode_selector.setCurrentIndex(
            window.search_mode_selector.findData(mode)
        )
        return window.search_hint.text()


if __name__ == "__main__":
    unittest.main()
