import math
from typing import Iterable, Optional, Sequence, Tuple

from PySide6.QtCore import QPointF, QTimer, Qt, Signal
from PySide6.QtGui import (
    QColor,
    QGuiApplication,
    QImage,
    QMouseEvent,
    QPainter,
    QPen,
    QPolygonF,
    QWheelEvent,
)
from PySide6.QtWidgets import QSizePolicy, QWidget

from .partition import Box
from .viewer import (
    BodyRenderGroup,
    ProjectedBodyMeshBounds,
    PreviewFrame,
    SHAPE_COLORS,
    ShapeMesh,
    box_mesh,
    face_is_visible,
    outward_face_normal,
    pick_body_candidates_from_bounds,
    preview_frame,
    project_point,
    project_body_mesh_bounds,
    rasterize_meshes_rgba,
    rotate_point,
    shade_color,
)


FAST_PREVIEW_MAX_DIMENSION = 360
FAST_PREVIEW_ZBUFFER_SHAPE_LIMIT = 96
FAST_PREVIEW_SOLID_COLOR = "#6941C6"


def preview_face_normals(
    meshes: Tuple[ShapeMesh, ...],
) -> Tuple[Tuple[Tuple[float, float, float], ...], ...]:
    """Precompute camera-independent face normals for interaction frames."""

    return tuple(
        tuple(outward_face_normal(mesh, face) for face in mesh.faces)
        for mesh in meshes
    )


def _convex_hull_2d(
    points: Tuple[Tuple[float, float], ...],
) -> Tuple[Tuple[float, float], ...]:
    """Return the counter-clockwise convex hull of projected points."""

    ordered = sorted(set(points))
    if len(ordered) <= 2:
        return tuple(ordered)

    def cross(
        origin: Tuple[float, float],
        first: Tuple[float, float],
        second: Tuple[float, float],
    ) -> float:
        return (
            (first[0] - origin[0]) * (second[1] - origin[1])
            - (first[1] - origin[1]) * (second[0] - origin[0])
        )

    lower = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return tuple(lower[:-1] + upper[:-1])


def fast_preview_silhouettes(
    meshes: Tuple[ShapeMesh, ...],
    center: Tuple[float, float, float],
    yaw: float,
    pitch: float,
    scale: float,
    viewport: Tuple[float, float],
) -> Tuple[Tuple[Tuple[float, float], ...], ...]:
    """Project each convex mesh to one opaque 2D silhouette.

    The union of these silhouettes preserves openings and separated meshes, but
    needs only one polygon per shape and cannot reveal a rear shape through a
    foreground shape when every polygon uses the same solid colour.
    """

    silhouettes = []
    center_x, center_y, center_z = center
    viewport_x, viewport_y = viewport
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    for mesh in meshes:
        points_list = []
        for vertex_x, vertex_y, vertex_z in mesh.vertices:
            relative_x = vertex_x - center_x
            relative_y = vertex_y - center_y
            relative_z = vertex_z - center_z
            rotated_x = cos_yaw * relative_x + sin_yaw * relative_z
            yaw_depth = -sin_yaw * relative_x + cos_yaw * relative_z
            rotated_y = cos_pitch * relative_y - sin_pitch * yaw_depth
            points_list.append(
                (
                    viewport_x + rotated_x * scale,
                    viewport_y - rotated_y * scale,
                )
            )
        points = tuple(points_list)
        hull = _convex_hull_2d(points)
        if len(hull) >= 3:
            silhouettes.append(hull)
    return tuple(silhouettes)


def fast_preview_faces(
    meshes: Tuple[ShapeMesh, ...],
    center: Tuple[float, float, float],
    yaw: float,
    pitch: float,
    scale: float,
    viewport: Tuple[float, float],
    face_normals: Optional[
        Tuple[Tuple[Tuple[float, float, float], ...], ...]
    ] = None,
    solid_color: Optional[str] = None,
) -> Tuple[Tuple[float, int, Tuple[Tuple[float, float], ...], str], ...]:
    """Build far-to-near vector faces for transient interaction frames."""

    layers = []
    shade_factors = (0.72, 0.92, 0.62, 1.08, 0.82, 0.98, 0.76, 1.02)
    normals_by_mesh = face_normals or preview_face_normals(meshes)
    for shape_index, (mesh, normals) in enumerate(zip(meshes, normals_by_mesh)):
        projected = tuple(
            project_point(vertex, center, yaw, pitch, scale, viewport)
            for vertex in mesh.vertices
        )
        for face_index, (indices, normal) in enumerate(zip(mesh.faces, normals)):
            if rotate_point(normal, yaw, pitch)[2] <= 1e-9:
                continue
            points = tuple(projected[index] for index in indices)
            depth = sum(point[2] for point in points) / len(points)
            priority = shape_index * 256 + face_index
            color = (
                solid_color
                or shade_color(
                    SHAPE_COLORS[shape_index % len(SHAPE_COLORS)],
                    shade_factors[face_index % len(shade_factors)],
                )
            )
            layers.append(
                (depth, priority, tuple((point[0], point[1]) for point in points), color)
            )
    return tuple(sorted(layers, key=lambda layer: (layer[0], layer[1])))


def rasterize_fast_preview(
    meshes: Tuple[ShapeMesh, ...],
    center: Tuple[float, float, float],
    yaw: float,
    pitch: float,
    scale: float,
    viewport: Tuple[float, float],
    width: int,
    height: int,
    max_dimension: int = FAST_PREVIEW_MAX_DIMENSION,
    shape_colors: Sequence[str] = SHAPE_COLORS,
    shape_opacities: Optional[Sequence[float]] = None,
) -> Tuple[bytearray, int, int]:
    """Render a reduced-resolution preview with exact per-pixel occlusion.

    Sorting whole faces by their average depth is cheap, but it cannot order
    faces whose projected depth ranges overlap.  On large vehicles an
    actually-hidden face could then be painted last and look translucent while
    the camera was moving.  A small Z-buffer keeps interaction responsive while
    making the visible surface unambiguous at every preview pixel.
    """

    if width <= 0 or height <= 0:
        return bytearray(), 0, 0
    render_ratio = min(1.0, max_dimension / max(width, height))
    render_width = max(1, round(width * render_ratio))
    render_height = max(1, round(height * render_ratio))
    pixels = rasterize_meshes_rgba(
        meshes,
        center,
        yaw,
        pitch,
        scale * render_ratio,
        (viewport[0] * render_ratio, viewport[1] * render_ratio),
        render_width,
        render_height,
        shape_colors=shape_colors,
        shape_opacities=shape_opacities,
        draw_outlines=False,
    )
    return pixels, render_width, render_height


class PhysicsShapeViewer(QWidget):
    """Cross-platform software preview with adaptive interaction quality."""

    bodyPicked = Signal(object, object)
    bodyHovered = Signal(object)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.meshes: Tuple[ShapeMesh, ...] = ()
        self.body_groups: Tuple[BodyRenderGroup, ...] = ()
        self.body_interaction_enabled = False
        self.shape_colors: Tuple[str, ...] = SHAPE_COLORS
        self.shape_opacities: Tuple[float, ...] = ()
        self.hovered_body_index: Optional[int] = None
        self._projected_body_bounds: Optional[
            Tuple[ProjectedBodyMeshBounds, ...]
        ] = None
        self.yaw = math.radians(-42)
        self.pitch = math.radians(24)
        self.zoom = 1.0
        self.center = (0.0, 0.0, 0.0)
        self.base_scale = 40.0
        self.preview_frame = preview_frame(())
        self.drag_origin: Optional[QPointF] = None
        self.press_origin: Optional[QPointF] = None
        self.drag_moved = False
        self._preview_mode = False
        self._settle_timer = QTimer(self)
        self._settle_timer.setSingleShot(True)
        self._settle_timer.setInterval(140)
        self._settle_timer.timeout.connect(self._finish_preview)
        self._frame_buffer: Optional[bytearray] = None
        self._frame_image: Optional[QImage] = None
        self.empty_message = "解析するとPhysics Shapeがここに表示されます"
        self.setMinimumHeight(320)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.setCursor(Qt.OpenHandCursor)

    def set_empty_message(self, message: str) -> None:
        self.empty_message = message
        if not self.meshes:
            self.update()

    def set_boxes(self, boxes: Iterable[Box], fit: bool = True) -> None:
        self.set_shapes((box_mesh(box) for box in boxes), fit=fit)

    def set_shapes(
        self,
        meshes: Iterable[ShapeMesh],
        fit: bool = True,
        frame: Optional[PreviewFrame] = None,
    ) -> None:
        self.body_groups = ()
        self.body_interaction_enabled = False
        self.hovered_body_index = None
        self._invalidate_body_pick_cache()
        self.meshes = tuple(meshes)
        self.shape_colors = SHAPE_COLORS
        self.shape_opacities = (1.0,) * len(self.meshes)
        if frame is not None:
            self.preview_frame = frame
        elif fit:
            self.preview_frame = preview_frame(self.meshes)
        if fit or frame is not None:
            self.fit_geometry()
        self._invalidate_frame()
        self._begin_preview()
        self.update()

    def set_body_groups(
        self,
        groups: Iterable[BodyRenderGroup],
        fit: bool = True,
        frame: Optional[PreviewFrame] = None,
        preserve_view_angles: bool = False,
    ) -> None:
        self.body_groups = tuple(group for group in groups if group.opacity > 0.005)
        self.body_interaction_enabled = True
        self._invalidate_body_pick_cache()
        meshes = []
        colors = []
        opacities = []
        for group in self.body_groups:
            meshes.extend(group.meshes)
            colors.extend((group.color,) * len(group.meshes))
            opacities.extend((group.opacity,) * len(group.meshes))
        self.meshes = tuple(meshes)
        self.shape_colors = tuple(colors) or SHAPE_COLORS
        self.shape_opacities = tuple(opacities)
        if frame is not None:
            self.preview_frame = frame
        elif fit:
            self.preview_frame = preview_frame(self.meshes)
        if fit and not preserve_view_angles:
            self.yaw = math.radians(-42)
            self.pitch = math.radians(24)
            self.zoom = 1.0
        elif fit:
            self.zoom = 1.0
        if fit or frame is not None:
            self.fit_geometry()
        self._invalidate_frame()
        self._begin_preview()
        self.update()

    def set_hovered_body(self, body_index: Optional[int]) -> None:
        if body_index == self.hovered_body_index:
            return
        self.hovered_body_index = body_index
        if self.body_interaction_enabled:
            self.update()

    def body_candidates_at(self, position: QPointF) -> Tuple[int, ...]:
        if self._projected_body_bounds is None:
            self._projected_body_bounds = project_body_mesh_bounds(
                self.body_groups,
                self.center,
                self.yaw,
                self.pitch,
                self.base_scale * self.zoom,
                (self.width() / 2.0, self.height() / 2.0),
            )
        return pick_body_candidates_from_bounds(
            self._projected_body_bounds,
            self.center,
            self.yaw,
            self.pitch,
            self.base_scale * self.zoom,
            (self.width() / 2.0, self.height() / 2.0),
            (position.x(), position.y()),
        )

    def _invalidate_body_pick_cache(self) -> None:
        self._projected_body_bounds = None

    def hover_body_at(self, position: QPointF) -> None:
        candidates = self.body_candidates_at(position)
        hovered = candidates[0] if candidates else None
        self.set_hovered_body(hovered)
        self.bodyHovered.emit(hovered)

    def _invalidate_frame(self) -> None:
        self._frame_image = None
        self._frame_buffer = None

    def _begin_preview(self) -> None:
        self._preview_mode = True
        self._settle_timer.start()

    def _finish_preview(self) -> None:
        if self.drag_origin is not None:
            return
        self._preview_mode = False
        self._invalidate_frame()
        self.update()

    def reset_view(self) -> None:
        self.yaw = math.radians(-42)
        self.pitch = math.radians(24)
        self.zoom = 1.0
        self.fit_geometry()
        self._invalidate_body_pick_cache()
        self._invalidate_frame()
        self._begin_preview()
        self.update()

    def fit_geometry(self) -> None:
        self.center = self.preview_frame.center
        available = max(240.0, min(float(self.width()), float(self.height())))
        self.base_scale = max(8.0, available * 0.62 / self.preview_frame.span)

    def scene_state(self) -> Tuple[float, ...]:
        return (
            *self.preview_frame.center,
            self.preview_frame.span,
            self.preview_frame.fit_diameter,
        )

    def resizeEvent(self, event: object) -> None:
        self.fit_geometry()
        self._invalidate_body_pick_cache()
        self._invalidate_frame()
        self._begin_preview()
        super().resizeEvent(event)  # type: ignore[arg-type]

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton:
            self.drag_origin = event.position()
            self.press_origin = event.position()
            self.drag_moved = False
            self.setCursor(Qt.ClosedHandCursor)
            self._begin_preview()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton:
            was_click = not self.drag_moved
            self.drag_origin = None
            self.press_origin = None
            self.setCursor(Qt.OpenHandCursor)
            self._preview_mode = False
            self._settle_timer.stop()
            self._invalidate_frame()
            self.update()
            if was_click and self.body_interaction_enabled:
                self.bodyPicked.emit(
                    self.body_candidates_at(event.position()),
                    QGuiApplication.queryKeyboardModifiers(),
                )
            elif self.body_interaction_enabled:
                self.hover_body_at(event.position())
        super().mouseReleaseEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self.drag_origin is not None and event.buttons() & Qt.LeftButton:
            current = event.position()
            if not self.drag_moved and self.press_origin is not None:
                total = current - self.press_origin
                if abs(total.x()) + abs(total.y()) <= 4.0:
                    super().mouseMoveEvent(event)
                    return
                self.drag_moved = True
                delta = total
            else:
                delta = current - self.drag_origin
            self.drag_origin = current
            self.yaw += delta.x() * 0.012
            self.pitch = max(-1.45, min(1.45, self.pitch + delta.y() * 0.012))
            self._invalidate_body_pick_cache()
            self._begin_preview()
            self._invalidate_frame()
            self.update()
        elif self.body_interaction_enabled:
            self.hover_body_at(event.position())
        super().mouseMoveEvent(event)

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton and not self.body_interaction_enabled:
            self.reset_view()
        super().mouseDoubleClickEvent(event)

    def leaveEvent(self, event: object) -> None:
        if self.drag_origin is None and self.body_interaction_enabled:
            self.set_hovered_body(None)
            self.bodyHovered.emit(None)
        super().leaveEvent(event)  # type: ignore[arg-type]

    def wheelEvent(self, event: QWheelEvent) -> None:
        factor = 1.12 if event.angleDelta().y() > 0 else 1 / 1.12
        self.zoom = max(0.2, min(8.0, self.zoom * factor))
        self._invalidate_body_pick_cache()
        self._begin_preview()
        self._invalidate_frame()
        self.update()
        event.accept()

    def _draw_axes(self, painter: QPainter, viewport: Tuple[float, float], scale: float) -> None:
        origin = project_point(self.center, self.center, self.yaw, self.pitch, scale, viewport)
        axes = (
            ((1.2, 0.0, 0.0), "#F97066", "X"),
            ((0.0, 1.2, 0.0), "#32D583", "Y"),
            ((0.0, 0.0, 1.2), "#53B1FD", "Z"),
        )
        for vector, color, label in axes:
            endpoint_world = tuple(self.center[index] + vector[index] for index in range(3))
            endpoint = project_point(
                endpoint_world, self.center, self.yaw, self.pitch, scale, viewport  # type: ignore[arg-type]
            )
            painter.setPen(QPen(QColor(color), 2.0))
            painter.drawLine(QPointF(origin[0], origin[1]), QPointF(endpoint[0], endpoint[1]))
            painter.drawText(QPointF(endpoint[0] + 3, endpoint[1] - 3), label)

    def _draw_fast_preview(
        self, painter: QPainter, viewport: Tuple[float, float], scale: float
    ) -> None:
        if (
            not self.body_interaction_enabled
            and len(self.meshes) > FAST_PREVIEW_ZBUFFER_SHAPE_LIMIT
        ):
            # A reduced Z-buffer still spends time visiting every triangle.
            # Above this limit, draw an opaque single-colour silhouette instead:
            # whole-face ordering can no longer expose a differently-coloured
            # rear shape, and detail returns as soon as interaction settles.
            painter.fillRect(self.rect(), QColor("#101828"))
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(FAST_PREVIEW_SOLID_COLOR))
            for points in fast_preview_silhouettes(
                self.meshes,
                self.center,
                self.yaw,
                self.pitch,
                scale,
                viewport,
            ):
                painter.drawPolygon(
                    QPolygonF([QPointF(x, y) for x, y in points])
                )
            return

        frame_buffer, render_width, render_height = rasterize_fast_preview(
            self.meshes,
            self.center,
            self.yaw,
            self.pitch,
            scale,
            viewport,
            max(1, self.width()),
            max(1, self.height()),
            shape_colors=self.shape_colors,
            shape_opacities=self.shape_opacities,
        )
        if not frame_buffer:
            painter.fillRect(self.rect(), QColor("#101828"))
            return
        frame_image = QImage(
            frame_buffer,
            render_width,
            render_height,
            render_width * 4,
            QImage.Format_RGBA8888,
        )
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        painter.drawImage(self.rect(), frame_image)

    def _draw_body_highlights(
        self,
        painter: QPainter,
        viewport: Tuple[float, float],
        scale: float,
    ) -> None:
        if not self.body_interaction_enabled:
            return

        def draw_groups(groups: Tuple[BodyRenderGroup, ...], color: str, width: float) -> None:
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(QColor(color), width))
            for group in groups:
                for mesh in group.meshes:
                    projected = tuple(
                        project_point(
                            vertex,
                            self.center,
                            self.yaw,
                            self.pitch,
                            scale,
                            viewport,
                        )
                        for vertex in mesh.vertices
                    )
                    for face in mesh.faces:
                        if not face_is_visible(mesh, face, self.yaw, self.pitch):
                            continue
                        points = [QPointF(projected[index][0], projected[index][1]) for index in face]
                        if points:
                            points.append(points[0])
                            painter.drawPolyline(QPolygonF(points))

        draw_groups(
            tuple(group for group in self.body_groups if group.selected),
            "#FDE68A",
            2.0,
        )
        draw_groups(
            tuple(
                group
                for group in self.body_groups
                if group.body_index == self.hovered_body_index
            ),
            "#FFFFFF",
            3.0,
        )

    def paintEvent(self, event: object) -> None:
        painter = QPainter(self)
        interactive = self._preview_mode or self.drag_origin is not None
        painter.setRenderHint(QPainter.Antialiasing, not interactive)
        if not self.meshes:
            painter.fillRect(self.rect(), QColor("#101828"))
            painter.setPen(QColor("#98A2B3"))
            painter.drawText(self.rect(), Qt.AlignCenter, self.empty_message)
            painter.end()
            return

        scale = self.base_scale * self.zoom
        viewport = (self.width() / 2.0, self.height() / 2.0)
        if interactive:
            self._draw_fast_preview(painter, viewport, scale)
        elif self._frame_image is None:
            render_width = max(1, self.width())
            render_height = max(1, self.height())
            self._frame_buffer = rasterize_meshes_rgba(
                self.meshes,
                self.center,
                self.yaw,
                self.pitch,
                scale,
                viewport,
                render_width,
                render_height,
                shape_colors=self.shape_colors,
                shape_opacities=self.shape_opacities,
                draw_outlines=not self.body_interaction_enabled,
            )
            self._frame_image = QImage(
                self._frame_buffer,
                render_width,
                render_height,
                render_width * 4,
                QImage.Format_RGBA8888,
            )
        if not interactive and self._frame_image is not None:
            painter.drawImage(self.rect(), self._frame_image)
        self._draw_body_highlights(painter, viewport, scale)
        self._draw_axes(painter, (52.0, self.height() - 52.0), min(scale, 36.0))
        painter.setPen(QColor("#F2F4F7"))
        font = painter.font()
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(14, 25, "{} Shapes".format(len(self.meshes)))
        painter.end()
