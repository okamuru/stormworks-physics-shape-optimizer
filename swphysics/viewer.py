import math
from dataclasses import dataclass
from itertools import combinations
from typing import List, Optional, Sequence, Tuple

from .partition import Box
from .portable_merge import Plane, PortableMergeGroup


Point3 = Tuple[float, float, float]
Point2 = Tuple[float, float]
RGBA = Tuple[int, int, int, int]


@dataclass(frozen=True)
class ShapeMesh:
    vertices: Tuple[Point3, ...]
    faces: Tuple[Tuple[int, ...], ...]


def stormworks_preview_mesh(mesh: ShapeMesh) -> ShapeMesh:
    """Convert Stormworks vehicle X into the game's F2 screen convention.

    A same-frame top-down comparison shows that the game and this viewer agree
    on vertical placement but put every asymmetric landmark on the opposite
    horizontal side.  Keep that handedness conversion at the display boundary
    so merge groups, optimization order, and source-preserving XML remain in
    the native Stormworks coordinate system.
    """

    return ShapeMesh(
        tuple((-x, y, z) for x, y, z in mesh.vertices),
        mesh.faces,
    )


def box_mesh(box: Box) -> ShapeMesh:
    return ShapeMesh(box_vertices(box), box_face_indices())


def _solve_planes(
    first: Tuple[Tuple[float, float, float], float],
    second: Tuple[Tuple[float, float, float], float],
    third: Tuple[Tuple[float, float, float], float],
) -> Optional[Point3]:
    rows = (first[0], second[0], third[0])
    values = (first[1], second[1], third[1])
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if abs(determinant) < 1e-9:
        return None

    def replaced(column: int) -> float:
        matrix = [list(row) for row in rows]
        for row_index in range(3):
            matrix[row_index][column] = values[row_index]
        return (
            matrix[0][0]
            * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
            - matrix[0][1]
            * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2]
            * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
        )

    return tuple(replaced(axis) / determinant for axis in range(3))  # type: ignore[return-value]


def _ordered_face(
    indices: Sequence[int],
    vertices: Sequence[Point3],
    normal: Tuple[float, float, float],
) -> Tuple[int, ...]:
    center = tuple(
        sum(vertices[index][axis] for index in indices) / len(indices)
        for axis in range(3)
    )
    normal_length = math.sqrt(sum(value * value for value in normal))
    unit_normal = tuple(value / normal_length for value in normal)
    reference = (1.0, 0.0, 0.0) if abs(unit_normal[0]) < 0.8 else (0.0, 1.0, 0.0)
    axis_u = (
        unit_normal[1] * reference[2] - unit_normal[2] * reference[1],
        unit_normal[2] * reference[0] - unit_normal[0] * reference[2],
        unit_normal[0] * reference[1] - unit_normal[1] * reference[0],
    )
    length_u = math.sqrt(sum(value * value for value in axis_u))
    axis_u = tuple(value / length_u for value in axis_u)
    axis_v = (
        unit_normal[1] * axis_u[2] - unit_normal[2] * axis_u[1],
        unit_normal[2] * axis_u[0] - unit_normal[0] * axis_u[2],
        unit_normal[0] * axis_u[1] - unit_normal[1] * axis_u[0],
    )

    def angle(index: int) -> float:
        relative = tuple(vertices[index][axis] - center[axis] for axis in range(3))
        return math.atan2(
            sum(relative[axis] * axis_v[axis] for axis in range(3)),
            sum(relative[axis] * axis_u[axis] for axis in range(3)),
        )

    return tuple(sorted(indices, key=angle))


def merge_group_mesh(group: PortableMergeGroup) -> ShapeMesh:
    """Build a display convex hull from one portable merge group."""

    minimum = tuple(value - 0.5 for value in group.minimum)
    maximum = tuple(value + 0.5 for value in group.maximum)
    halfspaces: List[Tuple[Tuple[float, float, float], float]] = []
    for axis in range(3):
        negative = tuple(-1.0 if index == axis else 0.0 for index in range(3))
        positive = tuple(1.0 if index == axis else 0.0 for index in range(3))
        halfspaces.append((negative, -minimum[axis]))
        halfspaces.append((positive, maximum[axis]))
    for anchor, normal_int in group.planes:
        normal = tuple(float(value) for value in normal_int)
        display_anchor = tuple(float(value) - 0.5 for value in anchor)
        distance = sum(normal[axis] * display_anchor[axis] for axis in range(3))
        candidate = (normal, distance)
        if candidate not in halfspaces:
            halfspaces.append(candidate)

    vertices: List[Point3] = []
    for equations in combinations(halfspaces, 3):
        point = _solve_planes(*equations)
        if point is None:
            continue
        if all(
            sum(normal[axis] * point[axis] for axis in range(3)) <= distance + 1e-7
            for normal, distance in halfspaces
        ) and not any(
            all(abs(point[axis] - existing[axis]) < 1e-7 for axis in range(3))
            for existing in vertices
        ):
            vertices.append(point)

    faces = []
    for normal, distance in halfspaces:
        indices = tuple(
            index
            for index, point in enumerate(vertices)
            if abs(
                sum(normal[axis] * point[axis] for axis in range(3)) - distance
            )
            < 1e-7
        )
        if len(indices) >= 3:
            ordered = _ordered_face(indices, vertices, normal)
            if ordered not in faces:
                faces.append(ordered)
    return ShapeMesh(tuple(vertices), tuple(faces))


SHAPE_COLORS = (
    "#7F56D9",
    "#12B76A",
    "#2E90FA",
    "#F79009",
    "#EE46BC",
    "#06AED4",
    "#F04438",
    "#84ADFF",
    "#A6EF67",
    "#FDB022",
)


def box_vertices(box: Box) -> Tuple[Point3, ...]:
    x0, y0, z0 = (value - 0.5 for value in box.minimum)
    x1, y1, z1 = (value + 0.5 for value in box.maximum)
    return (
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    )


def box_face_indices() -> Tuple[Tuple[int, int, int, int], ...]:
    return (
        (0, 1, 2, 3),
        (4, 7, 6, 5),
        (0, 4, 5, 1),
        (3, 2, 6, 7),
        (0, 3, 7, 4),
        (1, 5, 6, 2),
    )


def rotate_point(point: Point3, yaw: float, pitch: float) -> Point3:
    x, y, z = point
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x_yaw = cos_yaw * x + sin_yaw * z
    z_yaw = -sin_yaw * x + cos_yaw * z
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    y_pitch = cos_pitch * y - sin_pitch * z_yaw
    z_pitch = sin_pitch * y + cos_pitch * z_yaw
    return (x_yaw, y_pitch, z_pitch)


def outward_face_normal(mesh: ShapeMesh, face: Sequence[int]) -> Point3:
    """Return an outward normal without relying on the mesh winding order.

    The box fixture and the portable clipped hull historically used opposite
    winding conventions.  Orienting the geometric normal away from the convex
    mesh centre gives the viewer one consistent culling rule for both.
    """

    if len(face) < 3:
        return (0.0, 0.0, 0.0)
    first = mesh.vertices[face[0]]
    normal: Point3 = (0.0, 0.0, 0.0)
    for offset in range(1, len(face) - 1):
        second = mesh.vertices[face[offset]]
        third = mesh.vertices[face[offset + 1]]
        left = tuple(second[axis] - first[axis] for axis in range(3))
        right = tuple(third[axis] - first[axis] for axis in range(3))
        candidate = (
            left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0],
        )
        if sum(value * value for value in candidate) > 1e-18:
            normal = candidate
            break
    if normal == (0.0, 0.0, 0.0):
        return normal

    mesh_center = tuple(
        sum(vertex[axis] for vertex in mesh.vertices) / len(mesh.vertices)
        for axis in range(3)
    )
    face_center = tuple(
        sum(mesh.vertices[index][axis] for index in face) / len(face)
        for axis in range(3)
    )
    outward = tuple(face_center[axis] - mesh_center[axis] for axis in range(3))
    if sum(normal[axis] * outward[axis] for axis in range(3)) < 0.0:
        normal = tuple(-value for value in normal)  # type: ignore[assignment]
    return normal


def face_is_visible(
    mesh: ShapeMesh,
    face: Sequence[int],
    yaw: float,
    pitch: float,
    epsilon: float = 1e-9,
) -> bool:
    """Return whether a convex-mesh face points towards the camera."""

    normal = outward_face_normal(mesh, face)
    if normal == (0.0, 0.0, 0.0):
        return False
    return rotate_point(normal, yaw, pitch)[2] > epsilon


def project_point(
    point: Point3,
    center: Point3,
    yaw: float,
    pitch: float,
    scale: float,
    viewport: Point2,
) -> Tuple[float, float, float]:
    relative = tuple(point[index] - center[index] for index in range(3))
    x, y, depth = rotate_point(relative, yaw, pitch)  # type: ignore[arg-type]
    return (viewport[0] + x * scale, viewport[1] - y * scale, depth)


def shade_color(hex_color: str, factor: float) -> str:
    raw = hex_color.lstrip("#")
    channels = [int(raw[index : index + 2], 16) for index in (0, 2, 4)]
    shaded = [max(0, min(255, round(channel * factor))) for channel in channels]
    return "#{:02x}{:02x}{:02x}".format(*shaded)


def _rgba(hex_color: str) -> RGBA:
    raw = hex_color.lstrip("#")
    return (
        int(raw[0:2], 16),
        int(raw[2:4], 16),
        int(raw[4:6], 16),
        255,
    )


def _write_pixel(pixels: bytearray, index: int, color: RGBA) -> None:
    offset = index * 4
    pixels[offset] = color[0]
    pixels[offset + 1] = color[1]
    pixels[offset + 2] = color[2]
    pixels[offset + 3] = color[3]


def _rasterize_triangle(
    points: Sequence[Tuple[float, float, float]],
    width: int,
    height: int,
    depths: List[float],
    priorities: List[int],
    pixels: bytearray,
    color: RGBA,
    priority: int,
) -> None:
    """Fill one projected triangle with an orthographic per-pixel depth test."""

    minimum_y = max(0, math.ceil(min(point[1] for point in points) - 0.5))
    maximum_y = min(
        height - 1, math.floor(max(point[1] for point in points) - 0.5)
    )
    if minimum_y > maximum_y:
        return
    edges = ((points[0], points[1]), (points[1], points[2]), (points[2], points[0]))
    for y in range(minimum_y, maximum_y + 1):
        sample_y = y + 0.5
        intersections = []
        for start, end in edges:
            low = min(start[1], end[1])
            high = max(start[1], end[1])
            if not (low <= sample_y < high) or abs(end[1] - start[1]) < 1e-12:
                continue
            fraction = (sample_y - start[1]) / (end[1] - start[1])
            intersections.append(
                (
                    start[0] + (end[0] - start[0]) * fraction,
                    start[2] + (end[2] - start[2]) * fraction,
                )
            )
        if len(intersections) < 2:
            continue
        intersections.sort(key=lambda item: item[0])
        left_x, left_depth = intersections[0]
        right_x, right_depth = intersections[-1]
        if right_x - left_x < 1e-12:
            continue
        minimum_x = max(0, math.ceil(left_x - 0.5))
        maximum_x = min(width - 1, math.floor(right_x - 0.5))
        if minimum_x > maximum_x:
            continue
        depth_step = (right_depth - left_depth) / (right_x - left_x)
        depth = left_depth + ((minimum_x + 0.5) - left_x) * depth_step
        pixel_index = y * width + minimum_x
        for _x in range(minimum_x, maximum_x + 1):
            previous_depth = depths[pixel_index]
            if depth > previous_depth + 1e-7 or (
                abs(depth - previous_depth) <= 1e-7
                and priority >= priorities[pixel_index]
            ):
                depths[pixel_index] = depth
                priorities[pixel_index] = priority
                _write_pixel(pixels, pixel_index, color)
            depth += depth_step
            pixel_index += 1


def _rasterize_depth_line(
    start: Tuple[float, float, float],
    end: Tuple[float, float, float],
    width: int,
    height: int,
    depths: Sequence[float],
    priorities: Sequence[int],
    pixels: bytearray,
    color: RGBA,
    priority: int,
) -> None:
    """Draw a mesh edge only where it is not hidden by a nearer surface."""

    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    steps = max(1, math.ceil(max(abs(delta_x), abs(delta_y))))
    for step in range(steps + 1):
        fraction = step / steps
        x = round(start[0] + delta_x * fraction)
        y = round(start[1] + delta_y * fraction)
        if not (0 <= x < width and 0 <= y < height):
            continue
        depth = start[2] + (end[2] - start[2]) * fraction
        pixel_index = y * width + x
        previous_depth = depths[pixel_index]
        if depth > previous_depth + 1e-7 or (
            abs(depth - previous_depth) <= 1e-7
            and priority >= priorities[pixel_index]
        ):
            _write_pixel(pixels, pixel_index, color)


def rasterize_meshes_rgba(
    meshes: Sequence[ShapeMesh],
    center: Point3,
    yaw: float,
    pitch: float,
    scale: float,
    viewport: Point2,
    width: int,
    height: int,
    shape_colors: Sequence[str] = SHAPE_COLORS,
    background: str = "#101828",
    outline: str = "#D0D5DD",
    draw_outlines: bool = True,
) -> bytearray:
    """Render convex meshes with a true per-pixel depth buffer.

    Painter-style face-centre sorting cannot represent intersecting projected
    depth ranges, which made otherwise-correct front faces overwrite each
    other at a few camera angles.  This software rasterizer keeps the same
    cross-platform Qt surface while comparing depth at every pixel.
    """

    if width <= 0 or height <= 0:
        return bytearray()
    background_pixel = bytes(_rgba(background))
    pixels = bytearray(background_pixel * (width * height))
    depths = [float("-inf")] * (width * height)
    priorities = [-1] * (width * height)
    shade_factors = (0.72, 0.92, 0.62, 1.08, 0.82, 0.98, 0.76, 1.02)
    visible_edges = []

    for shape_index, mesh in enumerate(meshes):
        projected = tuple(
            project_point(vertex, center, yaw, pitch, scale, viewport)
            for vertex in mesh.vertices
        )
        for face_index, indices in enumerate(mesh.faces):
            if not face_is_visible(mesh, indices, yaw, pitch):
                continue
            face_points = tuple(projected[index] for index in indices)
            color = _rgba(
                shade_color(
                    shape_colors[shape_index % len(shape_colors)],
                    shade_factors[face_index % len(shade_factors)],
                )
            )
            priority = shape_index * 256 + face_index
            for offset in range(1, len(face_points) - 1):
                _rasterize_triangle(
                    (face_points[0], face_points[offset], face_points[offset + 1]),
                    width,
                    height,
                    depths,
                    priorities,
                    pixels,
                    color,
                    priority,
                )
            if draw_outlines:
                visible_edges.extend(
                    (
                        face_points[index],
                        face_points[(index + 1) % len(face_points)],
                        priority,
                    )
                    for index in range(len(face_points))
                )

    if draw_outlines:
        outline_color = _rgba(outline)
        for start, end, priority in visible_edges:
            _rasterize_depth_line(
                start,
                end,
                width,
                height,
                depths,
                priorities,
                pixels,
                outline_color,
                priority,
            )
    return pixels
