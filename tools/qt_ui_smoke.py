import argparse
from hashlib import sha256
import json
import os
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
if os.environ.get("SWPHYSICS_GPU_SMOKE") != "1":
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["SWPHYSICS_CONFIG_FILE"] = str(
    ROOT / "build/ui-smoke-home/config.json"
)
sys.path.insert(0, str(ROOT))

from PySide6.QtCore import (  # noqa: E402
    QEventLoop,
    QMimeData,
    QPoint,
    QPointF,
    QTimer,
    Qt,
    QUrl,
)
from PySide6.QtGui import QDragEnterEvent, QDropEvent  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from swphysics.app_service import analyze_vehicle  # noqa: E402
from swphysics.gui import OptimizerWindow, apply_style, vehicle_xml_from_urls  # noqa: E402


def digest(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(ROOT / "build/ui-smoke"))
    args = parser.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)

    application = QApplication([])
    apply_style(application)
    window = OptimizerWindow()
    # Keep the legacy Japanese smoke assertions deterministic even when the
    # reused smoke config was last written by an English-language run.
    window.language_selector.setCurrentIndex(window.language_selector.findData("ja"))
    window.show()

    dropped_vehicle = ROOT / "tests/fixtures/vehicles/order_b.xml"
    if vehicle_xml_from_urls([QUrl.fromLocalFile(str(dropped_vehicle))]) != dropped_vehicle:
        raise AssertionError("a dropped local vehicle XML must be recognized")
    if vehicle_xml_from_urls([QUrl("https://example.com/vehicle.xml")]) is not None:
        raise AssertionError("remote URLs must not be accepted as vehicle files")
    drop_data = QMimeData()
    drop_data.setUrls([QUrl.fromLocalFile(str(dropped_vehicle))])
    drag_event = QDragEnterEvent(
        QPoint(20, 20),
        Qt.CopyAction,
        drop_data,
        Qt.LeftButton,
        Qt.NoModifier,
    )
    application.sendEvent(window, drag_event)
    if not drag_event.isAccepted():
        raise AssertionError("window must accept a local vehicle XML drag")
    drop_event = QDropEvent(
        QPointF(20, 20),
        Qt.CopyAction,
        drop_data,
        Qt.LeftButton,
        Qt.NoModifier,
    )
    application.sendEvent(window, drop_event)
    if Path(window.vehicle_edit.text()) != dropped_vehicle:
        raise AssertionError("dropped vehicle XML must populate the vehicle path")

    if window.search_mode_selector.count() != 3:
        raise AssertionError("standard/deep/thorough search modes must be selectable")
    deep_index = window.search_mode_selector.findData("deep")
    window.search_mode_selector.setCurrentIndex(deep_index)
    if "最大3段階" not in window.search_hint.text():
        raise AssertionError("deep search mode details must be visible")
    window.search_mode_selector.setCurrentIndex(
        window.search_mode_selector.findData("standard")
    )
    if window.worker_selector.findData(0) < 0 or window.worker_selector.findData(8) < 0:
        raise AssertionError("Auto through 8 CPU workers must be selectable")

    # Regression check: a fast background job used to lose its queued result
    # signal when its QRunnable was auto-deleted.  The visible symptom was an
    # app that stayed on "解析中…" forever with zero CPU usage.
    background_result = []
    background_loop = QEventLoop()

    def background_done(value: object) -> None:
        background_result.append(value)
        background_loop.quit()

    def progress_worker(report):
        report(37, "Body 1/2: 配置候補を評価中… 12/32")
        return "worker-ok"

    window.run_background(
        progress_worker,
        background_done,
        "worker smoke",
        reports_progress=True,
    )
    QTimer.singleShot(2000, background_loop.quit)
    background_loop.exec()
    if background_result != ["worker-ok"] or window.busy:
        raise AssertionError("background worker completion was not delivered")
    if window.progress_bar.value() != 37:
        raise AssertionError("background worker progress was not delivered")

    cancellation_done = []
    cancellation_loop = QEventLoop()

    def cancellable_worker(report):
        for step in range(200):
            time.sleep(0.005)
            report(min(99, step), "cancellable worker")
        return "must-not-complete"

    window.run_background(
        cancellable_worker,
        lambda value: cancellation_done.append(value),
        "cancellation smoke",
        reports_progress=True,
        cancellable=True,
    )
    if not window.stop_button.isEnabled():
        raise AssertionError("stop button must be enabled during analysis")
    active_worker = window._active_worker
    if active_worker is None:
        raise AssertionError("cancellable worker was not retained")
    active_worker.signals.cancelled.connect(cancellation_loop.quit)
    QTimer.singleShot(30, window.stop_current_job)
    QTimer.singleShot(2000, cancellation_loop.quit)
    cancellation_loop.exec()
    application.processEvents()
    if cancellation_done or window.busy:
        raise AssertionError("cancelled analysis must not deliver a completed result")
    if "停止しました" not in window.status.currentMessage():
        raise AssertionError("cancelled analysis must show a stopped status")

    progress_path = output / "analysis-progress.png"
    window._active_cancellable = True
    window.set_busy(True, "解析中…")
    window.progress_widget.setVisible(True)
    window.job_progress(47, "Body 1/2: 配置候補を評価中… 15/32")
    application.processEvents()
    if not window.progress_widget.isVisible():
        raise AssertionError("analysis progress controls must be visible while busy")
    if not window.stop_button.isEnabled():
        raise AssertionError("stop button must be visible and enabled during analysis")
    if window.progress_bar.value() != 47 or "配置候補" not in window.progress_label.text():
        raise AssertionError("analysis progress must show percentage and current task")
    if not window.grab().save(str(progress_path)):
        raise RuntimeError("could not save analysis progress UI screenshot")
    window._active_cancellable = False
    window.set_busy(False, "進捗表示テスト完了")

    analysis = analyze_vehicle(
        ROOT / "tests/fixtures/vehicles/order_b.xml",
        ROOT / "tests/fixtures/definitions",
    )
    window.vehicle_edit.setText(str(analysis.vehicle_path))
    window.definitions_edit.setText(str(analysis.definitions_path))
    window.show_analysis(analysis)
    application.processEvents()
    vertical_scroll = window.page_scroll.verticalScrollBar()
    if vertical_scroll.maximum() <= 0:
        raise AssertionError("main content must scroll when the preview exceeds the window")
    if window.viewer.minimumHeight() < 400:
        raise AssertionError("3D preview must retain a useful minimum height")
    vertical_scroll.setValue(vertical_scroll.maximum())
    application.processEvents()

    current_path = output / "current-4-shapes.png"
    interaction_path = output / "current-4-shapes-interaction-opaque.png"
    optimized_path = output / "optimized-3-shapes.png"
    rotated_path = output / "optimized-3-shapes-rotated.png"
    non_cube_path = output / "non-cube-physics-shapes.png"
    all_bodies_path = output / "all-bodies-6-shapes.png"
    overlap_path = output / "overlap-2-shapes-pinned.png"
    english_path = output / "english-ui-820x660.png"
    if not window.grab().save(str(current_path)):
        raise RuntimeError("could not save current UI screenshot")

    if window.viewer.using_gpu:
        quick = window.viewer.viewer.quick
        center = QPoint(quick.width() // 2, quick.height() // 2)
        orbit_before = window.viewer.camera_state()
        QTest.mousePress(quick, Qt.LeftButton, pos=center)
        QTest.mouseMove(quick, center + QPoint(0, 40), delay=20)
        QTest.mouseRelease(quick, Qt.LeftButton, pos=center + QPoint(0, 40))
        orbit_after = window.viewer.camera_state()
        if abs(orbit_before[0] - orbit_after[0]) > 1e-6:
            raise AssertionError("vertical legacy drag must not change yaw")
        if abs(orbit_after[1] - (orbit_before[1] + 40.0 * 0.012)) > 1e-6:
            raise AssertionError("vertical legacy drag must use the old pitch sensitivity")

        window.viewer.reset_view()
        orbit_before = window.viewer.camera_state()
        QTest.mousePress(quick, Qt.LeftButton, pos=center)
        QTest.mouseMove(quick, center + QPoint(40, 0), delay=20)
        QTest.mouseRelease(quick, Qt.LeftButton, pos=center + QPoint(40, 0))
        orbit_after = window.viewer.camera_state()
        if abs(orbit_after[0] - (orbit_before[0] + 40.0 * 0.012)) > 1e-6:
            raise AssertionError("horizontal legacy drag must use the old yaw sensitivity")
        if abs(orbit_before[1] - orbit_after[1]) > 1e-6:
            raise AssertionError("horizontal legacy drag must not change pitch")
        window.viewer.reset_view()

    window.viewer.set_view_angles(-20.0, 10.0)
    application.processEvents()
    interaction_image = window.viewer.capture_image()
    if not window.grab().save(str(interaction_path)):
        raise RuntimeError("could not save rotated renderer screenshot")
    interaction_center = interaction_image.pixelColor(
        interaction_image.width() // 2,
        interaction_image.height() // 2,
    )
    if interaction_center.alpha() != 255:
        raise AssertionError("3D preview must stay fully opaque")

    camera_before_toggle = window.viewer.camera_state()
    window.optimized_radio.setChecked(True)
    application.processEvents()
    camera_after_toggle = window.viewer.camera_state()
    if len(camera_before_toggle) != len(camera_after_toggle) or any(
        abs(before - after) > 1e-6
        for before, after in zip(camera_before_toggle, camera_after_toggle)
    ):
        raise AssertionError("current/optimized toggle must preserve the camera")
    if not window.grab().save(str(optimized_path)):
        raise RuntimeError("could not save optimized UI screenshot")
    window.viewer.set_view_angles(5.0, 6.0)
    application.processEvents()
    if not window.grab().save(str(rotated_path)):
        raise RuntimeError("could not save rotated UI screenshot")

    non_cube = analyze_vehicle(
        ROOT / "tests/fixtures/vehicles/rotation_and_shapes.xml",
        ROOT / "tests/fixtures/definitions",
    )
    window.vehicle_edit.setText(str(non_cube.vehicle_path))
    window.show_analysis(non_cube)
    window.current_radio.setChecked(True)
    application.processEvents()
    if not window.grab().save(str(non_cube_path)):
        raise RuntimeError("could not save non-cube UI screenshot")
    if not any(
        len(mesh.vertices) != 8 or len(mesh.faces) != 6
        for mesh in non_cube.bodies[0].current_meshes
    ):
        raise AssertionError("non-cube viewer fixture must include a clipped mesh")

    multi_body = analyze_vehicle(
        ROOT / "tests/fixtures/vehicles/multi_body_non_physics.xml",
        ROOT / "tests/fixtures/definitions",
    )
    window.vehicle_edit.setText(str(multi_body.vehicle_path))
    window.show_analysis(multi_body)
    application.processEvents()
    if window.body_selector.currentData() != -1:
        raise AssertionError("all-body preview must be selected by default")
    if window.body_selector.count() != len(multi_body.bodies) + 1:
        raise AssertionError("body selector must include all-body plus each body")
    if len(window.viewer.meshes) != multi_body.current_shape_count:
        raise AssertionError("all-body preview must contain every current shape")
    if not window.grab().save(str(all_bodies_path)):
        raise RuntimeError("could not save all-body UI screenshot")
    window.body_selector.setCurrentIndex(2)
    application.processEvents()
    if len(window.viewer.meshes) != len(multi_body.bodies[1].current_meshes):
        raise AssertionError("individual body preview must remain selectable")

    overlap = analyze_vehicle(
        ROOT / "tests/fixtures/vehicles/overlapping_cube_components.xml",
        ROOT / "tests/fixtures/definitions",
    )
    window.vehicle_edit.setText(str(overlap.vehicle_path))
    window.show_analysis(overlap)
    application.processEvents()
    if not overlap.can_optimize:
        raise AssertionError("overlap components must be pinned instead of rejecting the vehicle")
    if len(window.viewer.meshes) != 2:
        raise AssertionError("overlap preview must retain both source physics voxels")
    if not window.grab().save(str(overlap_path)):
        raise RuntimeError("could not save overlap UI screenshot")

    window.language_selector.setCurrentIndex(
        window.language_selector.findData("en")
    )
    window.resize(820, 660)
    window.page_scroll.verticalScrollBar().setValue(0)
    application.processEvents()
    if window.analyze_button.text() != "Analyze":
        raise AssertionError("English UI selection must update the main actions")
    if not window.grab().save(str(english_path)):
        raise RuntimeError("could not save English UI screenshot")

    hashes = {
        path.name: digest(path)
        for path in (
            progress_path,
            current_path,
            interaction_path,
            optimized_path,
            rotated_path,
            non_cube_path,
            all_bodies_path,
            overlap_path,
            english_path,
        )
    }
    if len(set(hashes.values())) != 9:
        raise AssertionError("all UI renders must differ")
    if len(analysis.bodies[0].current_boxes) != 4:
        raise AssertionError("current viewer fixture should contain 4 shapes")
    if len(analysis.bodies[0].optimized_boxes or ()) != 3:
        raise AssertionError("optimized viewer fixture should contain 3 shapes")
    print(
        json.dumps(
            {
                "status": "ok",
                "window": [window.width(), window.height()],
                "current_shapes": 4,
                "optimized_shapes": 3,
                "non_cube_shapes": non_cube.current_shape_count,
                "all_body_shapes": multi_body.current_shape_count,
                "overlap_shapes": overlap.current_shape_count,
                "renderer": window.viewer.backend_name(),
                "screenshots": hashes,
            },
            indent=2,
        )
    )
    window.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
