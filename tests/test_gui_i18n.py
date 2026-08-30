import json
import os
from pathlib import Path
from string import Formatter
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, QPointF, Qt
from PySide6.QtGui import QKeySequence, QMouseEvent
from PySide6.QtTest import QTest
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
MULTI_BODY_VEHICLE = ROOT / "tests/fixtures/vehicles/multi_body_non_physics.xml"
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

    def test_body_palette_stays_distinct_for_a_sixty_body_vehicle(self):
        colors = tuple(OptimizerWindow._body_color(index) for index in range(60))
        self.assertEqual(60, len(set(colors)))

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
            "Body 1: 予測対象外の2 Componentを探索から除外して"
            "元スロットへ戻します。表示値は対応範囲のみで、最終F2 "
            "Shape数には対象外Componentとの相互作用が含まれません",
            "Body 1: Physics Flooderの面モデルunsupported_surface_metadata_missing"
            "を予測対象外にし、残りのComponentだけを最適化します",
        )

        translated = tuple(translate_runtime_message(message, "en") for message in messages)

        self.assertIn("Applying analyzed Component order", translated[0])
        self.assertIn("Verifying Component order and attributes", translated[1])
        self.assertIn("Locked the complete Body order", translated[2])
        self.assertIn("original slots", translated[3])
        self.assertIn("Excluded unsupported Physics Flooder", translated[4])
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

    def test_selected_body_can_be_excluded_and_restored_without_reanalysis(self):
        window = self.make_window()
        analysis = analyze_vehicle(VEHICLE, DEFINITIONS, search_mode="standard")
        window.show_analysis(analysis)
        window.body_selector.setCurrentIndex(window.body_selector.findData(0))
        window.optimized_radio.setChecked(True)
        window.viewer.set_view_angles(19.0, -11.0)
        camera_before = window.viewer.camera_state()

        self.assertTrue(window.manual_body_exclusion.isEnabled())
        self.assertEqual("3", window.optimized_shapes.text())
        window.manual_body_exclusion.setChecked(True)
        self.application.processEvents()

        self.assertEqual(camera_before, window.viewer.camera_state())
        self.assertTrue(window.last_analysis.bodies[0].manually_excluded)
        self.assertEqual(1, window.last_analysis.manually_excluded_body_count)
        self.assertEqual("4", window.optimized_shapes.text())
        self.assertIn("手動除外", window.body_selector.currentText())
        self.assertIn("手動除外", window.preview_caption.text())
        self.assertIs(
            analysis.bodies[0].optimization_result,
            window.last_analysis.bodies[0].optimization_result,
        )

        window.language_selector.setCurrentIndex(
            window.language_selector.findData("en")
        )
        self.application.processEvents()
        self.assertTrue(window.manual_body_exclusion.isChecked())
        self.assertIn("Manually Excluded", window.body_selector.currentText())
        self.assertEqual(camera_before, window.viewer.camera_state())

        window.manual_body_exclusion.setChecked(False)
        self.application.processEvents()
        self.assertFalse(window.last_analysis.bodies[0].manually_excluded)
        self.assertEqual("3", window.optimized_shapes.text())
        self.assertEqual(camera_before, window.viewer.camera_state())

        window.body_selector.setCurrentIndex(window.body_selector.findData(-1))
        self.assertFalse(window.manual_body_exclusion.isEnabled())

    def test_body_scope_switch_keeps_the_all_body_scene_frame(self):
        window = self.make_window()
        analysis = analyze_vehicle(
            MULTI_BODY_VEHICLE,
            DEFINITIONS,
            search_mode="standard",
        )
        window.show_analysis(analysis)
        window.viewer.set_view_angles(17.0, -9.0)
        scene_before = window.viewer.scene_state()
        camera_before = window.viewer.camera_state()

        for body_index in (0, 1):
            window.body_selector.setCurrentIndex(
                window.body_selector.findData(body_index)
            )
            self.application.processEvents()
            self.assertEqual(scene_before, window.viewer.scene_state())
            self.assertEqual(camera_before, window.viewer.camera_state())

        window.optimized_radio.setChecked(True)
        self.application.processEvents()
        self.assertEqual(scene_before, window.viewer.scene_state())
        self.assertEqual(camera_before, window.viewer.camera_state())

        window.body_selector.setCurrentIndex(window.body_selector.findData(-1))
        self.application.processEvents()
        self.assertEqual(scene_before, window.viewer.scene_state())
        self.assertEqual(camera_before, window.viewer.camera_state())

    def test_body_management_preview_starts_with_ghosting_and_reuses_viewer(self):
        window = self.make_window()
        analysis = analyze_vehicle(
            MULTI_BODY_VEHICLE,
            DEFINITIONS,
            search_mode="standard",
        )
        window.show_analysis(analysis)
        viewer_identity = id(window.viewer)

        self.assertTrue(window.shape_mode_button.isChecked())
        self.assertTrue(window.body_management_panel.isHidden())
        self.assertTrue(window.ghost_nonselected)
        self.assertTrue(window.ghost_button.isChecked())

        window.body_mode_button.setChecked(True)
        self.application.processEvents()

        self.assertEqual(viewer_identity, id(window.viewer))
        self.assertFalse(window.body_management_panel.isHidden())
        self.assertTrue(window.body_selector.isHidden())
        self.assertTrue(window.manual_options_widget.isHidden())
        self.assertIn("背景クリック", window.preview_caption.text())
        self.assertNotIn("ダブルクリック", window.preview_caption.text())
        self.assertEqual(len(analysis.bodies), window.body_tree.topLevelItemCount())
        groups = window.viewer.viewer.body_groups
        self.assertEqual(
            [body.body_index for body in analysis.bodies],
            [group.body_index for group in groups],
        )
        self.assertTrue(all(group.opacity == 1.0 for group in groups))

        window.body_picked_from_preview((0,), Qt.NoModifier)
        self.application.processEvents()
        groups = {group.body_index: group for group in window.viewer.viewer.body_groups}
        self.assertEqual({0}, window.selected_body_ids)
        self.assertTrue(groups[0].selected)
        self.assertEqual(1.0, groups[0].opacity)
        self.assertFalse(groups[1].selected)
        self.assertEqual(0.25, groups[1].opacity)

        window.ghost_button.setChecked(False)
        self.application.processEvents()
        groups = {group.body_index: group for group in window.viewer.viewer.body_groups}
        self.assertEqual(1.0, groups[1].opacity)

        window.overview_opacity_slider.setValue(60)
        window.clear_body_selection()
        self.application.processEvents()
        self.assertTrue(
            all(group.opacity == 0.6 for group in window.viewer.viewer.body_groups)
        )

        window.shape_mode_button.setChecked(True)
        self.application.processEvents()
        self.assertFalse(window.viewer.viewer.body_interaction_enabled)

    def test_body_group_visibility_has_mixed_state_and_undo_redo(self):
        window = self.make_window()
        analysis = analyze_vehicle(
            MULTI_BODY_VEHICLE,
            DEFINITIONS,
            search_mode="standard",
        )
        window.show_analysis(analysis)
        window.body_mode_button.setChecked(True)
        window.show()
        self.application.processEvents()

        window.body_picked_from_preview((0,), Qt.NoModifier)
        window.set_selected_bodies_visible(False)
        window.body_picked_from_preview((1,), Qt.ControlModifier)
        self.application.processEvents()

        self.assertEqual({0, 1}, window.selected_body_ids)
        self.assertEqual({0}, window.hidden_body_ids)
        self.assertEqual(
            {"◐"},
            {
                window.body_tree.topLevelItem(row).text(0)
                for row in range(window.body_tree.topLevelItemCount())
            },
        )
        visible_groups = {
            group.body_index for group in window.viewer.viewer.body_groups
        }
        self.assertEqual({1}, visible_groups)

        first_item = window.body_tree.topLevelItem(0)
        item_rect = window.body_tree.visualItemRect(first_item)
        click_position = item_rect.center()
        click_position.setX(12)
        QTest.mouseClick(
            window.body_tree.viewport(),
            Qt.LeftButton,
            Qt.NoModifier,
            click_position,
        )
        self.application.processEvents()
        self.assertEqual({0, 1}, window.selected_body_ids)
        self.assertEqual(set(), window.hidden_body_ids)
        self.assertEqual(
            {"◉"},
            {
                window.body_tree.topLevelItem(row).text(0)
                for row in range(window.body_tree.topLevelItemCount())
            },
        )

        window.undo_body_change()
        self.assertEqual({0}, window.hidden_body_ids)
        window.redo_body_change()
        self.assertEqual(set(), window.hidden_body_ids)

    def test_body_group_optimization_and_history_reuse_cached_analysis(self):
        window = self.make_window()
        analysis = analyze_vehicle(
            MULTI_BODY_VEHICLE,
            DEFINITIONS,
            search_mode="standard",
        )
        window.show_analysis(analysis)
        window.body_mode_button.setChecked(True)
        window.select_all_bodies()
        window.show()
        self.application.processEvents()
        original_results = {
            body.body_index: body.optimization_result for body in analysis.bodies
        }

        first_item = window.body_tree.topLevelItem(0)
        item_rect = window.body_tree.visualItemRect(first_item)
        check_position = item_rect.center()
        check_position.setX(
            window.body_tree.header().sectionPosition(3) + 10
        )
        QTest.mouseClick(
            window.body_tree.viewport(),
            Qt.LeftButton,
            Qt.NoModifier,
            check_position,
        )
        self.application.processEvents()
        self.assertEqual({0, 1}, window.selected_body_ids)
        self.assertEqual(
            {body.body_index for body in analysis.bodies},
            {
                body.body_index
                for body in window.last_analysis.bodies
                if body.manually_excluded
            },
        )
        self.assertTrue(
            all(
                window.body_tree.topLevelItem(row).checkState(3) == Qt.Unchecked
                for row in range(window.body_tree.topLevelItemCount())
            )
        )
        self.assertEqual(
            original_results,
            {
                body.body_index: body.optimization_result
                for body in window.last_analysis.bodies
            },
        )

        window.undo_body_change()
        self.assertFalse(
            any(body.manually_excluded for body in window.last_analysis.bodies)
        )
        window.redo_body_change()
        self.assertTrue(
            all(body.manually_excluded for body in window.last_analysis.bodies)
        )

    def test_body_filter_pick_hover_and_language_switch_preserve_state(self):
        window = self.make_window()
        analysis = analyze_vehicle(
            MULTI_BODY_VEHICLE,
            DEFINITIONS,
            search_mode="standard",
        )
        window.show_analysis(analysis)
        window.body_mode_button.setChecked(True)
        window.viewer.set_view_angles(21.0, -8.0)
        camera_before = window.viewer.camera_state()

        window.body_picked_from_preview((0,), Qt.NoModifier)
        window.body_picked_from_preview((1,), Qt.ControlModifier)
        self.assertEqual({0, 1}, window.selected_body_ids)
        window.body_filter_selector.setCurrentIndex(
            window.body_filter_selector.findData("selected")
        )
        self.assertEqual(2, window.body_tree.topLevelItemCount())

        window.body_filter_selector.setCurrentIndex(
            window.body_filter_selector.findData("all")
        )
        window.body_sort_selector.setCurrentIndex(
            window.body_sort_selector.findData("current")
        )
        window.toggle_body_sort_direction()
        self.assertEqual(1, window.body_tree.topLevelItem(0).data(0, Qt.UserRole))

        window.body_hovered_from_preview(1)
        self.assertEqual(1, window.hovered_body_id)
        hovered_item = next(
            window.body_tree.topLevelItem(row)
            for row in range(window.body_tree.topLevelItemCount())
            if window.body_tree.topLevelItem(row).data(0, Qt.UserRole) == 1
        )
        self.assertTrue(hovered_item.background(0).color().isValid())
        window._set_hovered_body(0, update_viewer=True)
        self.assertEqual(0, window.viewer.viewer.hovered_body_index)

        window.shortcut_help_button.setChecked(True)
        window.language_selector.setCurrentIndex(
            window.language_selector.findData("en")
        )
        self.application.processEvents()
        self.assertTrue(window.body_mode_button.isChecked())
        self.assertEqual({0, 1}, window.selected_body_ids)
        self.assertEqual(camera_before, window.viewer.camera_state())
        self.assertIn("Controls and Keyboard Shortcuts", window.shortcut_help_button.text())
        self.assertFalse(window.shortcut_help_content.isHidden())

        window.body_picked_from_preview((), Qt.NoModifier)
        self.assertEqual(set(), window.selected_body_ids)

    def test_plain_body_click_replaces_selection_on_list_and_preview(self):
        window = self.make_window()
        analysis = analyze_vehicle(
            MULTI_BODY_VEHICLE,
            DEFINITIONS,
            search_mode="standard",
        )
        window.show_analysis(analysis)
        window.body_mode_button.setChecked(True)
        window.show()
        self.application.processEvents()

        def click_list_body(body_index, modifiers=Qt.NoModifier):
            item = next(
                window.body_tree.topLevelItem(row)
                for row in range(window.body_tree.topLevelItemCount())
                if window.body_tree.topLevelItem(row).data(0, Qt.UserRole)
                == body_index
            )
            position = window.body_tree.visualItemRect(item).center()
            position.setX(window.body_tree.header().sectionPosition(1) + 20)
            QTest.mouseClick(
                window.body_tree.viewport(),
                Qt.LeftButton,
                modifiers,
                position,
            )
            self.application.processEvents()

        click_list_body(0)
        self.assertEqual({0}, window.selected_body_ids)
        click_list_body(1)
        self.assertEqual({1}, window.selected_body_ids)
        click_list_body(0, Qt.ControlModifier)
        self.assertEqual({0, 1}, window.selected_body_ids)
        click_list_body(1)
        self.assertEqual({1}, window.selected_body_ids)
        click_list_body(1)
        self.assertEqual(set(), window.selected_body_ids)
        click_list_body(0)
        click_list_body(1, Qt.ShiftModifier)
        self.assertEqual({0, 1}, window.selected_body_ids)

        window.clear_body_selection()
        window.body_picked_from_preview((0,), Qt.NoModifier)
        window.body_picked_from_preview((1,), Qt.NoModifier)
        self.assertEqual({1}, window.selected_body_ids)
        window.body_picked_from_preview((0,), Qt.ControlModifier)
        self.assertEqual({0, 1}, window.selected_body_ids)
        window.body_picked_from_preview((1,), Qt.NoModifier)
        self.assertEqual({1}, window.selected_body_ids)

        window.body_picked_from_preview((0, 1), Qt.AltModifier)
        self.assertEqual({0}, window.selected_body_ids)
        window.body_picked_from_preview((0, 1), Qt.AltModifier)
        self.assertEqual({1}, window.selected_body_ids)

    def test_dragging_across_body_list_rows_does_not_extend_selection(self):
        window = self.make_window()
        analysis = analyze_vehicle(
            MULTI_BODY_VEHICLE,
            DEFINITIONS,
            search_mode="standard",
        )
        window.show_analysis(analysis)
        window.body_mode_button.setChecked(True)
        window.show()
        self.application.processEvents()

        first = window.body_tree.topLevelItem(0)
        second = window.body_tree.topLevelItem(1)
        start = window.body_tree.visualItemRect(first).center()
        start.setX(window.body_tree.header().sectionPosition(1) + 20)
        end = window.body_tree.visualItemRect(second).center()
        end.setX(start.x())

        QTest.mousePress(
            window.body_tree.viewport(),
            Qt.LeftButton,
            Qt.NoModifier,
            start,
        )
        QTest.mouseMove(window.body_tree.viewport(), end, delay=20)
        QTest.mouseRelease(
            window.body_tree.viewport(),
            Qt.LeftButton,
            Qt.NoModifier,
            end,
        )
        self.application.processEvents()

        self.assertEqual({0}, window.selected_body_ids)
        self.assertEqual(
            [0],
            [
                int(item.data(0, Qt.UserRole))
                for item in window.body_tree.selectedItems()
            ],
        )

    def test_body_blank_hover_hidden_ghost_and_shortcuts_are_consistent(self):
        window = self.make_window()
        analysis = analyze_vehicle(
            MULTI_BODY_VEHICLE,
            DEFINITIONS,
            search_mode="standard",
        )
        window.show_analysis(analysis)
        window.body_mode_button.setChecked(True)
        window.show()
        self.application.processEvents()

        shortcut_sequences = [
            shortcut.key().toString(QKeySequence.PortableText)
            for shortcut in window._body_shortcuts
        ]
        self.assertEqual(len(shortcut_sequences), len(set(shortcut_sequences)))
        self.assertIn("Ctrl+Y", shortcut_sequences)
        self.assertIn("Ctrl+Shift+Z", shortcut_sequences)

        window.body_picked_from_preview((0,), Qt.NoModifier)
        window.set_selected_bodies_visible(False)
        self.application.processEvents()
        visible_groups = window.viewer.viewer.body_groups
        self.assertEqual([1], [group.body_index for group in visible_groups])
        self.assertEqual(1.0, visible_groups[0].opacity)

        window.body_picked_from_preview((1,), Qt.ControlModifier)
        window.body_filter_selector.setCurrentIndex(
            window.body_filter_selector.findData("visible")
        )
        blank = window.body_tree.viewport().rect().center()
        blank.setY(window.body_tree.viewport().height() - 4)
        visible_item = window.body_tree.topLevelItem(0)
        QTest.mouseMove(
            window.body_tree.viewport(),
            window.body_tree.visualItemRect(visible_item).center(),
        )
        self.application.processEvents()
        window._set_hovered_body(1, update_viewer=True)
        self.assertIsNone(window.body_tree.itemAt(blank))
        QApplication.sendEvent(
            window.body_tree.viewport(),
            QMouseEvent(
                QEvent.MouseMove,
                QPointF(blank),
                QPointF(blank),
                QPointF(window.body_tree.viewport().mapToGlobal(blank)),
                Qt.NoButton,
                Qt.NoButton,
                Qt.NoModifier,
            ),
        )
        self.application.processEvents()
        self.assertIsNone(window.hovered_body_id)
        QTest.mouseClick(
            window.body_tree.viewport(),
            Qt.LeftButton,
            Qt.NoModifier,
            blank,
        )
        self.application.processEvents()
        self.assertEqual(set(), window.selected_body_ids)

    def test_body_undo_restores_current_row_for_enter_toggle(self):
        window = self.make_window()
        analysis = analyze_vehicle(
            MULTI_BODY_VEHICLE,
            DEFINITIONS,
            search_mode="standard",
        )
        window.show_analysis(analysis)
        window.body_mode_button.setChecked(True)
        window._select_body_from_list_click(0, Qt.NoModifier)
        window._select_body_from_list_click(1, Qt.NoModifier)

        window.undo_body_change()

        self.assertEqual({0}, window.selected_body_ids)
        self.assertEqual(
            0,
            window.body_tree.currentItem().data(0, Qt.UserRole),
        )
        window.toggle_focused_body_selection()
        self.assertEqual(set(), window.selected_body_ids)

    def test_body_list_selection_enter_focus_and_history_shortcuts(self):
        window = self.make_window()
        analysis = analyze_vehicle(
            MULTI_BODY_VEHICLE,
            DEFINITIONS,
            search_mode="standard",
        )
        window.show_analysis(analysis)
        window.body_mode_button.setChecked(True)
        window.show()
        self.application.processEvents()
        full_scene = window.viewer.scene_state()

        first_item = window.body_tree.topLevelItem(0)
        item_rect = window.body_tree.visualItemRect(first_item)
        body_position = item_rect.center()
        body_position.setX(
            window.body_tree.header().sectionPosition(1) + 20
        )
        QTest.mouseClick(
            window.body_tree.viewport(),
            Qt.LeftButton,
            Qt.NoModifier,
            body_position,
        )
        self.application.processEvents()
        self.assertEqual({0}, window.selected_body_ids)

        window.undo_body_change()
        self.assertEqual(set(), window.selected_body_ids)
        window.redo_body_change()
        self.assertEqual({0}, window.selected_body_ids)

        first_item = window.body_tree.topLevelItem(0)
        item_rect = window.body_tree.visualItemRect(first_item)
        body_position = item_rect.center()
        body_position.setX(
            window.body_tree.header().sectionPosition(1) + 20
        )
        QTest.mouseClick(
            window.body_tree.viewport(),
            Qt.LeftButton,
            Qt.NoModifier,
            body_position,
        )
        self.application.processEvents()
        self.assertEqual(set(), window.selected_body_ids)

        first_item = window.body_tree.topLevelItem(0)
        window.body_tree.setCurrentItem(first_item)
        window.body_tree.clearSelection()
        window.body_tree.setFocus()
        QTest.keyClick(window.body_tree, Qt.Key_Return)
        self.application.processEvents()
        self.assertEqual({0}, window.selected_body_ids)

        window.viewer.set_view_angles(21.0, -8.0)
        angles_before_focus = window.viewer.camera_state()[:2]
        QTest.keyClick(window.body_tree, Qt.Key_F)
        self.application.processEvents()
        self.assertEqual(
            angles_before_focus,
            window.viewer.camera_state()[:2],
        )
        self.assertNotEqual(full_scene, window.viewer.scene_state())
        window.reset_view()
        self.assertEqual(full_scene, window.viewer.scene_state())

    @staticmethod
    def _search_hint(window, mode):
        window.search_mode_selector.setCurrentIndex(
            window.search_mode_selector.findData(mode)
        )
        return window.search_hint.text()


if __name__ == "__main__":
    unittest.main()
