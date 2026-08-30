"""GPU-first Physics Shape viewer with a recoverable software fallback."""

import math
import os
from typing import Iterable, Optional, Tuple

from PySide6.QtCore import Signal
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QVBoxLayout, QWidget

from .partition import Box
from .qt_viewer import PhysicsShapeViewer as SoftwarePhysicsShapeViewer
from .viewer import (
    BodyRenderGroup,
    ShapeMesh,
    box_mesh,
    preview_frame,
    stormworks_preview_mesh,
)


class PhysicsShapeViewer(QWidget):
    """Stable GUI boundary shared by the GPU and software implementations."""

    bodyPicked = Signal(object, object)
    bodyHovered = Signal(object)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.backend_error = ""
        self.using_gpu = False
        self._preview_mesh_cache: dict[int, Tuple[ShapeMesh, ShapeMesh]] = {}
        platform_name = os.environ.get("QT_QPA_PLATFORM", "").lower()
        force_software = (
            os.environ.get("SWPHYSICS_FORCE_SOFTWARE_VIEWER") == "1"
            or platform_name in ("offscreen", "minimal")
        )
        viewer: QWidget
        if not force_software:
            try:
                from .gpu_viewer import GpuPhysicsShapeViewer

                viewer = GpuPhysicsShapeViewer()
                self.using_gpu = True
            except Exception as error:
                self.backend_error = "{}: {}".format(type(error).__name__, error)
                viewer = SoftwarePhysicsShapeViewer()
        else:
            viewer = SoftwarePhysicsShapeViewer()
            self.backend_error = "software viewer selected for {} platform".format(
                platform_name or "configured"
            )
        self.viewer = viewer
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(viewer)
        viewer.bodyPicked.connect(self.bodyPicked.emit)  # type: ignore[attr-defined]
        viewer.bodyHovered.connect(self.bodyHovered.emit)  # type: ignore[attr-defined]

    @property
    def meshes(self) -> Tuple[ShapeMesh, ...]:
        return self.viewer.meshes  # type: ignore[attr-defined]

    def set_boxes(self, boxes: Iterable[Box], fit: bool = True) -> None:
        self.set_shapes((box_mesh(box) for box in boxes), fit=fit)

    def set_shapes(
        self,
        meshes: Iterable[ShapeMesh],
        fit: bool = True,
        reference_meshes: Optional[Iterable[ShapeMesh]] = None,
    ) -> None:
        if fit:
            self._preview_mesh_cache.clear()
        preview_meshes = tuple(self._preview_mesh(mesh) for mesh in meshes)
        frame = (
            preview_frame(
                self._preview_mesh(mesh) for mesh in reference_meshes
            )
            if reference_meshes is not None
            else None
        )
        self.viewer.set_shapes(preview_meshes, fit=fit, frame=frame)  # type: ignore[attr-defined]

    def set_body_groups(
        self,
        groups: Iterable[BodyRenderGroup],
        fit: bool = True,
        reference_meshes: Optional[Iterable[ShapeMesh]] = None,
        preserve_view_angles: bool = False,
    ) -> None:
        if fit:
            self._preview_mesh_cache.clear()
        preview_groups = tuple(
            BodyRenderGroup(
                body_index=group.body_index,
                meshes=tuple(self._preview_mesh(mesh) for mesh in group.meshes),
                color=group.color,
                opacity=group.opacity,
                selected=group.selected,
            )
            for group in groups
        )
        frame = (
            preview_frame(
                self._preview_mesh(mesh) for mesh in reference_meshes
            )
            if reference_meshes is not None
            else None
        )
        self.viewer.set_body_groups(  # type: ignore[attr-defined]
            preview_groups,
            fit=fit,
            frame=frame,
            preserve_view_angles=preserve_view_angles,
        )

    def _preview_mesh(self, mesh: ShapeMesh) -> ShapeMesh:
        """Reuse the immutable axis-converted mesh throughout one analysis."""

        key = id(mesh)
        cached = self._preview_mesh_cache.get(key)
        if cached is not None and cached[0] is mesh:
            return cached[1]
        converted = stormworks_preview_mesh(mesh)
        self._preview_mesh_cache[key] = (mesh, converted)
        return converted

    def set_hovered_body(self, body_index: Optional[int]) -> None:
        self.viewer.set_hovered_body(body_index)  # type: ignore[attr-defined]

    def set_empty_message(self, message: str) -> None:
        setter = getattr(self.viewer, "set_empty_message", None)
        if setter is not None:
            setter(message)

    def reset_view(self) -> None:
        self.viewer.reset_view()  # type: ignore[attr-defined]

    def set_view_angles(self, yaw_degrees: float, pitch_degrees: float) -> None:
        if self.using_gpu:
            self.viewer.set_view_angles(yaw_degrees, pitch_degrees)  # type: ignore[attr-defined]
        else:
            self.viewer.yaw = math.radians(yaw_degrees)  # type: ignore[attr-defined]
            self.viewer.pitch = math.radians(pitch_degrees)  # type: ignore[attr-defined]
            self.viewer._invalidate_frame()  # type: ignore[attr-defined]
            self.viewer.update()

    def capture_image(self) -> QImage:
        if self.using_gpu:
            return self.viewer.capture_image()  # type: ignore[attr-defined]
        return self.viewer.grab().toImage()

    def camera_state(self) -> Tuple[float, ...]:
        if self.using_gpu:
            return self.viewer.camera_state()  # type: ignore[attr-defined]
        return (
            float(self.viewer.yaw),  # type: ignore[attr-defined]
            float(self.viewer.pitch),  # type: ignore[attr-defined]
            float(self.viewer.zoom),  # type: ignore[attr-defined]
        )

    def scene_state(self) -> Tuple[float, ...]:
        return self.viewer.scene_state()  # type: ignore[attr-defined]

    def interaction_hint(self) -> str:
        return "左ドラッグ: 回転 / ホイール: ズーム / ダブルクリック: リセット"

    def backend_name(self) -> str:
        if not self.using_gpu:
            return "Software"
        return "GPU/{}".format(self.viewer.graphics_api_name())  # type: ignore[attr-defined]
