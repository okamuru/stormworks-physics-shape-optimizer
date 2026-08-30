"""Qt Quick 3D proof-of-concept viewer backed by the platform GPU."""

from dataclasses import dataclass
from functools import lru_cache
import math
from pathlib import Path
import struct
import sys
import time
from typing import Iterable, Optional, Sequence, Tuple

from PySide6.QtCore import QByteArray, QObject, QPointF, Qt, QUrl, Signal
from PySide6.QtGui import (
    QColor,
    QGuiApplication,
    QImage,
    QMouseEvent,
    QQuaternion,
    QVector3D,
    QWheelEvent,
)
from PySide6.QtQml import qmlRegisterType
from PySide6.QtQuick3D import QQuick3DGeometry
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtWidgets import QSizePolicy, QVBoxLayout, QWidget

from .partition import Box
from .viewer import (
    BodyRenderGroup,
    PreviewFrame,
    ProjectedBodyMeshBounds,
    SHAPE_COLORS,
    ShapeMesh,
    box_mesh,
    outward_face_normal,
    pick_body_candidates_from_bounds,
    preview_frame,
    project_body_mesh_bounds,
)


GPU_VERTEX_STRIDE = 7 * 4
GPU_LINE_VERTEX_STRIDE = 3 * 4
_SHADE_FACTORS = (0.72, 0.92, 0.62, 1.08, 0.82, 0.98, 0.76, 1.02)
_VERTEX_STRUCT = struct.Struct("<7f")
_QML_TYPE_REGISTERED = False


@dataclass(frozen=True)
class GpuGeometryData:
    vertex_data: bytes
    vertex_count: int
    triangle_count: int
    center: Tuple[float, float, float]
    bounds_min: Tuple[float, float, float]
    bounds_max: Tuple[float, float, float]
    span: float
    fit_diameter: float


@dataclass(frozen=True)
class GpuOutlineData:
    vertex_data: bytes
    vertex_count: int
    segment_count: int
    bounds_min: Tuple[float, float, float]
    bounds_max: Tuple[float, float, float]


def _srgb_to_linear(value: float) -> float:
    return value / 12.92 if value <= 0.04045 else ((value + 0.055) / 1.055) ** 2.4


@lru_cache(maxsize=128)
def _rgb_float(hex_color: str, factor: float) -> Tuple[float, float, float, float]:
    color = QColor(hex_color)
    red = min(1.0, color.redF() * factor)
    green = min(1.0, color.greenF() * factor)
    blue = min(1.0, color.blueF() * factor)
    return (
        _srgb_to_linear(red),
        _srgb_to_linear(green),
        _srgb_to_linear(blue),
        1.0,
    )


def _geometric_normal(
    vertices: Tuple[Tuple[float, float, float], ...],
    face: Tuple[int, ...],
) -> Tuple[float, float, float]:
    first = vertices[face[0]]
    for offset in range(1, len(face) - 1):
        second = vertices[face[offset]]
        third = vertices[face[offset + 1]]
        left = tuple(second[axis] - first[axis] for axis in range(3))
        right = tuple(third[axis] - first[axis] for axis in range(3))
        normal = (
            left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0],
        )
        if sum(value * value for value in normal) > 1e-18:
            return normal
    return (0.0, 0.0, 0.0)


def build_gpu_geometry(
    meshes: Iterable[ShapeMesh],
    center: Optional[Tuple[float, float, float]] = None,
    shape_colors: Optional[Sequence[str]] = None,
) -> GpuGeometryData:
    """Combine convex meshes into one opaque, vertex-coloured triangle stream."""

    mesh_tuple = tuple(meshes)
    if not mesh_tuple:
        return GpuGeometryData(
            b"",
            0,
            0,
            center or (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            1.0,
            1.0,
        )
    all_points = tuple(vertex for mesh in mesh_tuple for vertex in mesh.vertices)
    world_min = tuple(min(point[axis] for point in all_points) for axis in range(3))
    world_max = tuple(max(point[axis] for point in all_points) for axis in range(3))
    if center is None:
        center = tuple(
            (world_min[axis] + world_max[axis]) / 2.0 for axis in range(3)
        )
    centered_min = tuple(world_min[axis] - center[axis] for axis in range(3))
    centered_max = tuple(world_max[axis] - center[axis] for axis in range(3))
    span = max(world_max[axis] - world_min[axis] for axis in range(3))
    fit_diameter = math.sqrt(
        sum((world_max[axis] - world_min[axis]) ** 2 for axis in range(3))
    )

    triangle_count = sum(
        max(0, len(face) - 2)
        for mesh in mesh_tuple
        for face in mesh.faces
    )
    vertex_count = triangle_count * 3
    data = bytearray(vertex_count * GPU_VERTEX_STRIDE)
    write_offset = 0
    for shape_index, mesh in enumerate(mesh_tuple):
        colors = shape_colors or SHAPE_COLORS
        base_color = colors[shape_index % len(colors)]
        for face_index, face in enumerate(mesh.faces):
            if len(face) < 3:
                continue
            outward = outward_face_normal(mesh, face)
            geometric = _geometric_normal(mesh.vertices, face)
            ordered = (
                face
                if sum(geometric[axis] * outward[axis] for axis in range(3)) >= 0.0
                else tuple(reversed(face))
            )
            red, green, blue, alpha = _rgb_float(
                base_color,
                _SHADE_FACTORS[face_index % len(_SHADE_FACTORS)],
            )
            for offset in range(1, len(ordered) - 1):
                for vertex_index in (ordered[0], ordered[offset], ordered[offset + 1]):
                    vertex = mesh.vertices[vertex_index]
                    _VERTEX_STRUCT.pack_into(
                        data,
                        write_offset,
                        vertex[0] - center[0],
                        vertex[1] - center[1],
                        vertex[2] - center[2],
                        red,
                        green,
                        blue,
                        alpha,
                    )
                    write_offset += GPU_VERTEX_STRIDE
    return GpuGeometryData(
        bytes(data),
        vertex_count,
        triangle_count,
        center,  # type: ignore[arg-type]
        centered_min,  # type: ignore[arg-type]
        centered_max,  # type: ignore[arg-type]
        max(span, 1.0),
        max(fit_diameter, 1.0),
    )


def build_gpu_outlines(
    meshes: Iterable[ShapeMesh],
    geometry: Optional[GpuGeometryData] = None,
    *,
    center: Optional[Tuple[float, float, float]] = None,
) -> GpuOutlineData:
    """Build one deduplicated line stream for all convex-mesh edges."""

    mesh_tuple = tuple(meshes)
    if geometry is not None:
        center = geometry.center
        centered_min = geometry.bounds_min
        centered_max = geometry.bounds_max
    elif mesh_tuple:
        first_point = next(
            (vertex for mesh in mesh_tuple for vertex in mesh.vertices),
            (0.0, 0.0, 0.0),
        )
        mutable_min = list(first_point)
        mutable_max = list(first_point)
        for mesh in mesh_tuple:
            for point in mesh.vertices:
                for axis in range(3):
                    mutable_min[axis] = min(mutable_min[axis], point[axis])
                    mutable_max[axis] = max(mutable_max[axis], point[axis])
        world_min = tuple(mutable_min)
        world_max = tuple(mutable_max)
        if center is None:
            center = tuple(
                (world_min[axis] + world_max[axis]) / 2.0 for axis in range(3)
            )
        centered_min = tuple(
            world_min[axis] - center[axis] for axis in range(3)
        )
        centered_max = tuple(
            world_max[axis] - center[axis] for axis in range(3)
        )
    else:
        center = center or (0.0, 0.0, 0.0)
        centered_min = (0.0, 0.0, 0.0)
        centered_max = (0.0, 0.0, 0.0)

    vertices = []
    for mesh in mesh_tuple:
        edges = set()
        for face in mesh.faces:
            for index in range(len(face)):
                first = face[index]
                second = face[(index + 1) % len(face)]
                edge = (min(first, second), max(first, second))
                if edge in edges:
                    continue
                edges.add(edge)
                for vertex_index in edge:
                    vertex = mesh.vertices[vertex_index]
                    vertices.extend(
                        (
                            vertex[0] - center[0],
                            vertex[1] - center[1],
                            vertex[2] - center[2],
                        )
                    )
    data = struct.pack("<{}f".format(len(vertices)), *vertices) if vertices else b""
    return GpuOutlineData(
        data,
        len(vertices) // 3,
        len(vertices) // 6,
        centered_min,  # type: ignore[arg-type]
        centered_max,  # type: ignore[arg-type]
    )


def combine_gpu_outlines(
    outlines: Iterable[GpuOutlineData],
) -> GpuOutlineData:
    """Join compatible Body outline streams without rebuilding their edges."""

    outline_tuple = tuple(outline for outline in outlines if outline.vertex_count)
    if not outline_tuple:
        return GpuOutlineData(
            b"",
            0,
            0,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )
    return GpuOutlineData(
        b"".join(outline.vertex_data for outline in outline_tuple),
        sum(outline.vertex_count for outline in outline_tuple),
        sum(outline.segment_count for outline in outline_tuple),
        tuple(
            min(outline.bounds_min[axis] for outline in outline_tuple)
            for axis in range(3)
        ),  # type: ignore[arg-type]
        tuple(
            max(outline.bounds_max[axis] for outline in outline_tuple)
            for axis in range(3)
        ),  # type: ignore[arg-type]
    )


class PhysicsShapeGeometry(QQuick3DGeometry):
    """QML-facing custom geometry containing every Physics Shape."""

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)
        self.data = build_gpu_geometry(())
        self.set_geometry(self.data)

    def set_geometry(self, geometry: GpuGeometryData) -> None:
        self.clear()
        self.data = geometry
        self.setStride(GPU_VERTEX_STRIDE)
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.PositionSemantic,
            0,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.ColorSemantic,
            12,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        self.setVertexData(QByteArray(geometry.vertex_data))
        self.setBounds(QVector3D(*geometry.bounds_min), QVector3D(*geometry.bounds_max))
        self.update()


class PhysicsShapeOutlineGeometry(QQuick3DGeometry):
    """QML-facing line geometry for Physics Shape boundaries."""

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)
        self.data = build_gpu_outlines((), build_gpu_geometry(()))
        self.set_geometry(self.data)

    def set_geometry(self, geometry: GpuOutlineData) -> None:
        self.clear()
        self.data = geometry
        self.setStride(GPU_LINE_VERTEX_STRIDE)
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Lines)
        self.addAttribute(
            QQuick3DGeometry.Attribute.Semantic.PositionSemantic,
            0,
            QQuick3DGeometry.Attribute.ComponentType.F32Type,
        )
        self.setVertexData(QByteArray(geometry.vertex_data))
        self.setBounds(QVector3D(*geometry.bounds_min), QVector3D(*geometry.bounds_max))
        self.update()


def register_gpu_qml_type() -> None:
    global _QML_TYPE_REGISTERED
    if _QML_TYPE_REGISTERED:
        return
    qmlRegisterType(
        PhysicsShapeGeometry,
        "StormworksPhysicsGpu",
        1,
        0,
        "PhysicsShapeGeometry",
    )
    qmlRegisterType(
        PhysicsShapeOutlineGeometry,
        "StormworksPhysicsGpu",
        1,
        0,
        "PhysicsShapeOutlineGeometry",
    )
    _QML_TYPE_REGISTERED = True


def _resource_path(relative: str) -> Path:
    base = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parents[1]))
    return base / relative


def legacy_orbit_angles(
    yaw: float,
    pitch: float,
    delta_x: float,
    delta_y: float,
) -> Tuple[float, float]:
    """Apply the exact orbit sensitivity and pitch clamp of the old viewer."""

    return (
        yaw + delta_x * 0.012,
        max(-1.45, min(1.45, pitch + delta_y * 0.012)),
    )


def legacy_view_orientation(yaw: float, pitch: float) -> QQuaternion:
    """Compose the GPU view in the same order as the software renderer.

    ``viewer.rotate_point`` applies yaw first and pitch second.  Qt's
    ``fromEulerAngles(pitch, yaw, 0)`` composes those rotations in the reverse
    order, which shears an oblique view sideways and can make an asymmetric
    vehicle look mirrored.  Keep this conversion at the renderer boundary so
    vehicle, merger, and saved XML coordinates remain untouched.
    """

    pitch_rotation = QQuaternion.fromAxisAndAngle(
        QVector3D(1.0, 0.0, 0.0), math.degrees(pitch)
    )
    yaw_rotation = QQuaternion.fromAxisAndAngle(
        QVector3D(0.0, 1.0, 0.0), math.degrees(yaw)
    )
    return pitch_rotation * yaw_rotation


class GpuQuickWidget(QQuickWidget):
    """QQuickWidget using the legacy software viewer's orbit controls."""

    def __init__(self, viewer: "GpuPhysicsShapeViewer"):
        super().__init__(viewer)
        self.viewer = viewer
        self.dragging = False
        self.drag_moved = False
        self.press_position = QPointF()
        self.previous_position = QPointF()
        self.setMouseTracking(True)
        self.setCursor(Qt.OpenHandCursor)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.drag_moved = False
            self.press_position = event.position()
            self.previous_position = event.position()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self.dragging and event.buttons() & Qt.LeftButton:
            if not self.drag_moved:
                total = event.position() - self.press_position
                if abs(total.x()) + abs(total.y()) <= 4.0:
                    event.accept()
                    return
                self.drag_moved = True
                delta = total
            else:
                delta = event.position() - self.previous_position
            self.viewer.orbit_view(delta.x(), delta.y())
            self.previous_position = event.position()
            event.accept()
            return
        if self.viewer.body_interaction_enabled:
            self.viewer.hover_body_at(event.position())
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton and self.dragging:
            was_click = not self.drag_moved
            self.dragging = False
            self.setCursor(Qt.OpenHandCursor)
            if was_click and self.viewer.body_interaction_enabled:
                self.viewer.pick_body_at(
                    event.position(),
                    QGuiApplication.queryKeyboardModifiers(),
                )
            elif self.viewer.body_interaction_enabled:
                self.viewer.hover_body_at(event.position())
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton and not self.viewer.body_interaction_enabled:
            self.viewer.reset_view()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def leaveEvent(self, event: object) -> None:
        if not self.dragging and self.viewer.body_interaction_enabled:
            self.viewer.set_hovered_body(None)
            self.viewer.bodyHovered.emit(None)
        super().leaveEvent(event)  # type: ignore[arg-type]

    def wheelEvent(self, event: QWheelEvent) -> None:
        delta = event.angleDelta().y()
        if delta == 0:
            delta = event.pixelDelta().y()
        if delta:
            self.viewer.zoom_view(1.12 if delta > 0 else 1.0 / 1.12)
        event.accept()


class GpuPhysicsShapeViewer(QWidget):
    """Widget-compatible Qt Quick 3D viewer with a software-viewer-like API."""

    bodyPicked = Signal(object, object)
    bodyHovered = Signal(object)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        register_gpu_qml_type()
        self.meshes: Tuple[ShapeMesh, ...] = ()
        self.body_groups: Tuple[BodyRenderGroup, ...] = ()
        self.body_interaction_enabled = False
        self.hovered_body_index: Optional[int] = None
        self._projected_body_bounds: Optional[
            Tuple[ProjectedBodyMeshBounds, ...]
        ] = None
        self._body_outline_cache: dict[
            int,
            Tuple[
                Tuple[ShapeMesh, ...],
                Tuple[float, float, float],
                GpuOutlineData,
            ],
        ] = {}
        self.preview_frame = preview_frame(())
        self.geometry_build_seconds = 0.0
        self.yaw = math.radians(-42.0)
        self.pitch = math.radians(24.0)
        self.quick = GpuQuickWidget(self)
        self.quick.setResizeMode(QQuickWidget.ResizeMode.SizeRootObjectToView)
        self.quick.setClearColor(QColor("#101828"))
        self.quick.setSource(QUrl.fromLocalFile(str(_resource_path("assets/gpu_viewer.qml"))))
        if self.quick.status() == QQuickWidget.Status.Error:
            raise RuntimeError(
                "Qt Quick 3D QML load failed: {}".format(
                    "; ".join(error.toString() for error in self.quick.errors())
                )
            )
        root = self.quick.rootObject()
        if root is None:
            raise RuntimeError("Qt Quick 3D root item was not created")
        geometry = root.findChild(PhysicsShapeGeometry, "physicsGeometry")
        if geometry is None:
            raise RuntimeError("Qt Quick 3D geometry object was not created")
        self.root = root
        self.geometry = geometry
        outlines = root.findChild(PhysicsShapeOutlineGeometry, "physicsOutlines")
        if outlines is None:
            raise RuntimeError("Qt Quick 3D outline object was not created")
        self.outlines = outlines
        self.ghost_geometry = self._required_geometry("ghostGeometry")
        self.ghost_outlines = self._required_outline_geometry("ghostOutlines")
        self.selection_outlines = self._required_outline_geometry(
            "selectionOutlines"
        )
        self.hover_outlines = self._required_outline_geometry("hoverOutlines")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.quick)
        self.quick.setMinimumHeight(0)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def _required_geometry(self, object_name: str) -> PhysicsShapeGeometry:
        geometry = self.root.findChild(PhysicsShapeGeometry, object_name)
        if geometry is None:
            raise RuntimeError("Qt Quick 3D geometry {} was not created".format(object_name))
        return geometry

    def _required_outline_geometry(
        self, object_name: str
    ) -> PhysicsShapeOutlineGeometry:
        geometry = self.root.findChild(PhysicsShapeOutlineGeometry, object_name)
        if geometry is None:
            raise RuntimeError("Qt Quick 3D outline {} was not created".format(object_name))
        return geometry

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
        self._body_outline_cache.clear()
        self.meshes = tuple(meshes)
        if frame is not None:
            self.preview_frame = frame
        elif fit:
            self.preview_frame = preview_frame(self.meshes)
        started = time.perf_counter()
        geometry = build_gpu_geometry(self.meshes, center=self.preview_frame.center)
        outlines = build_gpu_outlines(self.meshes, geometry)
        self.geometry.set_geometry(geometry)
        self.outlines.set_geometry(outlines)
        empty_geometry = build_gpu_geometry((), center=self.preview_frame.center)
        empty_outlines = build_gpu_outlines((), empty_geometry)
        self.ghost_geometry.set_geometry(empty_geometry)
        self.ghost_outlines.set_geometry(empty_outlines)
        self.selection_outlines.set_geometry(empty_outlines)
        self.hover_outlines.set_geometry(empty_outlines)
        self.root.setProperty("ghostOpacity", 0.0)
        self.geometry_build_seconds = time.perf_counter() - started
        self.root.setProperty("shapeCount", len(self.meshes))
        self.root.setProperty("sceneSpan", self.preview_frame.span)
        self.root.setProperty("fitDiameter", self.preview_frame.fit_diameter)
        if fit:
            self.reset_view()

    @staticmethod
    def _flatten_groups(
        groups: Iterable[BodyRenderGroup],
    ) -> Tuple[Tuple[ShapeMesh, ...], Tuple[str, ...]]:
        meshes = []
        colors = []
        for group in groups:
            meshes.extend(group.meshes)
            colors.extend((group.color,) * len(group.meshes))
        return tuple(meshes), tuple(colors)

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
        self.meshes = tuple(
            mesh for group in self.body_groups for mesh in group.meshes
        )
        if frame is not None:
            self.preview_frame = frame
        elif fit:
            self.preview_frame = preview_frame(self.meshes)
        primary_groups = tuple(
            group for group in self.body_groups if group.opacity >= 0.995
        )
        ghost_groups = tuple(
            group for group in self.body_groups if group.opacity < 0.995
        )
        primary_meshes, primary_colors = self._flatten_groups(primary_groups)
        ghost_meshes, ghost_colors = self._flatten_groups(ghost_groups)
        self._refresh_body_outline_cache()
        started = time.perf_counter()
        primary_geometry = build_gpu_geometry(
            primary_meshes,
            center=self.preview_frame.center,
            shape_colors=primary_colors,
        )
        ghost_geometry = build_gpu_geometry(
            ghost_meshes,
            center=self.preview_frame.center,
            shape_colors=ghost_colors,
        )
        self.geometry.set_geometry(primary_geometry)
        self.outlines.set_geometry(
            self._outlines_for_groups(primary_groups)
        )
        self.ghost_geometry.set_geometry(ghost_geometry)
        self.ghost_outlines.set_geometry(
            self._outlines_for_groups(ghost_groups)
        )
        self.root.setProperty(
            "ghostOpacity",
            max((group.opacity for group in ghost_groups), default=0.0),
        )
        self._update_selection_outlines()
        self._update_hover_outline()
        self.geometry_build_seconds = time.perf_counter() - started
        self.root.setProperty("shapeCount", len(self.meshes))
        self.root.setProperty("sceneSpan", self.preview_frame.span)
        self.root.setProperty("fitDiameter", self.preview_frame.fit_diameter)
        if fit:
            if preserve_view_angles:
                self.root.setProperty("zoom", 1.0)
                self.root.setProperty("panX", 0.0)
                self.root.setProperty("panY", 0.0)
                self._invalidate_body_pick_cache()
            else:
                self.reset_view()

    def _refresh_body_outline_cache(self) -> None:
        center = self.preview_frame.center
        retained = {}
        for group in self.body_groups:
            cached = self._body_outline_cache.get(group.body_index)
            if (
                cached is not None
                and cached[0] == group.meshes
                and cached[1] == center
            ):
                retained[group.body_index] = cached
                continue
            retained[group.body_index] = (
                group.meshes,
                center,
                build_gpu_outlines(group.meshes, center=center),
            )
        self._body_outline_cache = retained

    def _outlines_for_groups(
        self,
        groups: Iterable[BodyRenderGroup],
    ) -> GpuOutlineData:
        return combine_gpu_outlines(
            self._body_outline_cache[group.body_index][2]
            for group in groups
        )

    def _update_selection_outlines(self) -> None:
        self.selection_outlines.set_geometry(
            self._outlines_for_groups(
                group for group in self.body_groups if group.selected
            )
        )

    def _update_hover_outline(self) -> None:
        self.hover_outlines.set_geometry(
            self._outlines_for_groups(
                group
                for group in self.body_groups
                if group.body_index == self.hovered_body_index
            )
        )

    def set_hovered_body(self, body_index: Optional[int]) -> None:
        if body_index == self.hovered_body_index:
            return
        self.hovered_body_index = body_index
        if self.body_interaction_enabled:
            self._update_hover_outline()

    def _projection_scale(self) -> float:
        return (
            max(1.0, min(float(self.quick.width()), float(self.quick.height())))
            * 0.86
            / self.preview_frame.fit_diameter
            * float(self.root.property("zoom"))
        )

    def _invalidate_body_pick_cache(self) -> None:
        self._projected_body_bounds = None

    def _body_pick_viewport(self) -> Tuple[float, float]:
        return (self.quick.width() / 2.0, self.quick.height() / 2.0)

    def _ensure_body_pick_cache(self) -> Tuple[ProjectedBodyMeshBounds, ...]:
        if self._projected_body_bounds is None:
            self._projected_body_bounds = project_body_mesh_bounds(
                self.body_groups,
                self.preview_frame.center,
                self.yaw,
                self.pitch,
                self._projection_scale(),
                self._body_pick_viewport(),
            )
        return self._projected_body_bounds

    def body_candidates_at(self, position: QPointF) -> Tuple[int, ...]:
        return pick_body_candidates_from_bounds(
            self._ensure_body_pick_cache(),
            self.preview_frame.center,
            self.yaw,
            self.pitch,
            self._projection_scale(),
            self._body_pick_viewport(),
            (position.x(), position.y()),
        )

    def hover_body_at(self, position: QPointF) -> None:
        candidates = self.body_candidates_at(position)
        hovered = candidates[0] if candidates else None
        self.set_hovered_body(hovered)
        self.bodyHovered.emit(hovered)

    def pick_body_at(self, position: QPointF, modifiers: object) -> None:
        self.bodyPicked.emit(self.body_candidates_at(position), modifiers)

    def reset_view(self) -> None:
        self.yaw = math.radians(-42.0)
        self.pitch = math.radians(24.0)
        self._apply_view_angles()
        self.root.setProperty("zoom", 1.0)
        self.root.setProperty("panX", 0.0)
        self.root.setProperty("panY", 0.0)
        self._invalidate_body_pick_cache()

    def set_view_angles(self, yaw_degrees: float, pitch_degrees: float) -> None:
        self.yaw = math.radians(yaw_degrees)
        self.pitch = max(-1.45, min(1.45, math.radians(pitch_degrees)))
        self._apply_view_angles()
        self._invalidate_body_pick_cache()

    def _apply_view_angles(self) -> None:
        self.root.setProperty(
            "orientation",
            legacy_view_orientation(self.yaw, self.pitch),
        )

    def orbit_view(self, delta_x: float, delta_y: float) -> None:
        self.yaw, self.pitch = legacy_orbit_angles(
            self.yaw, self.pitch, delta_x, delta_y
        )
        self._apply_view_angles()
        self._invalidate_body_pick_cache()

    def zoom_view(self, factor: float) -> None:
        zoom = float(self.root.property("zoom")) * factor
        self.root.setProperty("zoom", max(0.2, min(8.0, zoom)))
        self._invalidate_body_pick_cache()

    def resizeEvent(self, event: object) -> None:
        self._invalidate_body_pick_cache()
        super().resizeEvent(event)  # type: ignore[arg-type]

    def camera_state(self) -> Tuple[float, ...]:
        return (
            self.yaw,
            self.pitch,
            float(self.root.property("zoom")),
        )

    def scene_state(self) -> Tuple[float, ...]:
        return (
            *self.preview_frame.center,
            self.preview_frame.span,
            self.preview_frame.fit_diameter,
        )

    def capture_image(self) -> QImage:
        return self.quick.grabFramebuffer()

    def graphics_api_name(self) -> str:
        api = self.quick.quickWindow().rendererInterface().graphicsApi()
        return api.name
