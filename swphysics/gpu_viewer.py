"""Qt Quick 3D proof-of-concept viewer backed by the platform GPU."""

from dataclasses import dataclass
from functools import lru_cache
import math
from pathlib import Path
import struct
import sys
import time
from typing import Iterable, Optional, Tuple

from PySide6.QtCore import QByteArray, QObject, QPointF, Qt, QUrl
from PySide6.QtGui import (
    QColor,
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
from .viewer import SHAPE_COLORS, ShapeMesh, box_mesh, outward_face_normal


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


def build_gpu_geometry(meshes: Iterable[ShapeMesh]) -> GpuGeometryData:
    """Combine convex meshes into one opaque, vertex-coloured triangle stream."""

    mesh_tuple = tuple(meshes)
    if not mesh_tuple:
        return GpuGeometryData(
            b"",
            0,
            0,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            1.0,
            1.0,
        )
    all_points = tuple(vertex for mesh in mesh_tuple for vertex in mesh.vertices)
    world_min = tuple(min(point[axis] for point in all_points) for axis in range(3))
    world_max = tuple(max(point[axis] for point in all_points) for axis in range(3))
    center = tuple((world_min[axis] + world_max[axis]) / 2.0 for axis in range(3))
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
        base_color = SHAPE_COLORS[shape_index % len(SHAPE_COLORS)]
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
    meshes: Iterable[ShapeMesh], geometry: GpuGeometryData
) -> GpuOutlineData:
    """Build one deduplicated line stream for all convex-mesh edges."""

    vertices = []
    center = geometry.center
    for mesh in meshes:
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
        geometry.bounds_min,
        geometry.bounds_max,
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
        self.previous_position = QPointF()
        self.setCursor(Qt.OpenHandCursor)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.previous_position = event.position()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self.dragging and event.buttons() & Qt.LeftButton:
            delta = event.position() - self.previous_position
            self.viewer.orbit_view(delta.x(), delta.y())
            self.previous_position = event.position()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton and self.dragging:
            self.dragging = False
            self.setCursor(Qt.OpenHandCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton:
            self.viewer.reset_view()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def wheelEvent(self, event: QWheelEvent) -> None:
        delta = event.angleDelta().y()
        if delta == 0:
            delta = event.pixelDelta().y()
        if delta:
            self.viewer.zoom_view(1.12 if delta > 0 else 1.0 / 1.12)
        event.accept()


class GpuPhysicsShapeViewer(QWidget):
    """Widget-compatible Qt Quick 3D viewer with a software-viewer-like API."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        register_gpu_qml_type()
        self.meshes: Tuple[ShapeMesh, ...] = ()
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
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.quick)
        self.quick.setMinimumHeight(0)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_boxes(self, boxes: Iterable[Box], fit: bool = True) -> None:
        self.set_shapes((box_mesh(box) for box in boxes), fit=fit)

    def set_shapes(self, meshes: Iterable[ShapeMesh], fit: bool = True) -> None:
        self.meshes = tuple(meshes)
        started = time.perf_counter()
        geometry = build_gpu_geometry(self.meshes)
        outlines = build_gpu_outlines(self.meshes, geometry)
        self.geometry.set_geometry(geometry)
        self.outlines.set_geometry(outlines)
        self.geometry_build_seconds = time.perf_counter() - started
        self.root.setProperty("shapeCount", len(self.meshes))
        self.root.setProperty("sceneSpan", geometry.span)
        self.root.setProperty("fitDiameter", geometry.fit_diameter)
        if fit:
            self.reset_view()

    def reset_view(self) -> None:
        self.yaw = math.radians(-42.0)
        self.pitch = math.radians(24.0)
        self._apply_view_angles()
        self.root.setProperty("zoom", 1.0)
        self.root.setProperty("panX", 0.0)
        self.root.setProperty("panY", 0.0)

    def set_view_angles(self, yaw_degrees: float, pitch_degrees: float) -> None:
        self.yaw = math.radians(yaw_degrees)
        self.pitch = max(-1.45, min(1.45, math.radians(pitch_degrees)))
        self._apply_view_angles()

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

    def zoom_view(self, factor: float) -> None:
        zoom = float(self.root.property("zoom")) * factor
        self.root.setProperty("zoom", max(0.2, min(8.0, zoom)))

    def camera_state(self) -> Tuple[float, ...]:
        return (
            self.yaw,
            self.pitch,
            float(self.root.property("zoom")),
        )

    def capture_image(self) -> QImage:
        return self.quick.grabFramebuffer()

    def graphics_api_name(self) -> str:
        api = self.quick.quickWindow().rendererInterface().graphicsApi()
        return api.name
