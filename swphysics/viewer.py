import math
from dataclasses import dataclass
from itertools import combinations
from typing import Iterable, List, Optional, Sequence, Tuple

from .partition import Box
from .portable_merge import Plane, PortableMergeGroup


Point3 = Tuple[float, float, float]
Point2 = Tuple[float, float]
RGBA = Tuple[int, int, int, int]


@dataclass(frozen=True)
class ShapeMesh:
    vertices: Tuple[Point3, ...]
    faces: Tuple[Tuple[int, ...], ...]


@dataclass(frozen=True)
class BodyRenderGroup:
    """One Body's preview meshes and presentation-only render state."""

    body_index: int
    meshes: Tuple[ShapeMesh, ...]
    color: str
    opacity: float = 1.0
    selected: bool = False


@dataclass(frozen=True)
class ProjectedBodyMeshBounds:
    """Cheap screen-space rejection data for one Body mesh."""

    body_index: int
    mesh: ShapeMesh
    min_x: float
    min_y: float
    max_x: float
    max_y: float


@dataclass(frozen=True)
class PreviewFrame:
    """Stable world-space framing shared by every preview scope."""

    center: Point3
    span: float
    fit_diameter: float


def preview_frame(meshes: Iterable[ShapeMesh]) -> PreviewFrame:
    """Measure a scene without retaining another copy of its mesh collection."""

    minimum: Optional[List[float]] = None
    maximum: Optional[List[float]] = None
    for mesh in meshes:
        for point in mesh.vertices:
            if minimum is None:
                minimum = list(point)
                maximum = list(point)
                continue
            assert maximum is not None
            for axis in range(3):
                minimum[axis] = min(minimum[axis], point[axis])
                maximum[axis] = max(maximum[axis], point[axis])
    if minimum is None or maximum is None:
        return PreviewFrame((0.0, 0.0, 0.0), 1.0, 1.0)
    spans = tuple(maximum[axis] - minimum[axis] for axis in range(3))
    return PreviewFrame(
        (
            (minimum[0] + maximum[0]) / 2.0,
            (minimum[1] + maximum[1]) / 2.0,
            (minimum[2] + maximum[2]) / 2.0,
        ),
        max(max(spans), 1.0),
        max(math.sqrt(sum(value * value for value in spans)), 1.0),
    )


def stormworks_preview_mesh(mesh: ShapeMesh) -> ShapeMesh:
    """Convert native vehicle axes into the game's F2 screen convention.

    Mirror the lateral X axis while preserving Stormworks' native fore-aft Z
    axis.  Reversing one axis changes handedness, so reverse every face as well
    to preserve its winding for renderers that use back-face culling.  Keep
    this display-only conversion at the renderer boundary so merge groups,
    optimization order, and source-preserving XML remain in the native
    Stormworks coordinate system.
    """

    return ShapeMesh(
        tuple((-x, y, z) for x, y, z in mesh.vertices),
        tuple(tuple(reversed(face)) for face in mesh.faces),
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
        if normal == (0.0, 0.0, 0.0):
            # A singular XML transform can collapse a non-cube clip normal to
            # zero.  Stormworks' convex-hull finalizer keeps its initialized
            # +X normal when normalization cannot replace it; use the same
            # fallback for display geometry instead of passing a zero vector
            # to the face-ordering basis calculation.
            normal = (1.0, 0.0, 0.0)
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


def _triangle_depth_at_point(
    point: Point2,
    first: Tuple[float, float, float],
    second: Tuple[float, float, float],
    third: Tuple[float, float, float],
) -> Optional[float]:
    """Return interpolated depth when a screen point is inside a triangle."""

    denominator = (
        (second[1] - third[1]) * (first[0] - third[0])
        + (third[0] - second[0]) * (first[1] - third[1])
    )
    if abs(denominator) < 1e-12:
        return None
    first_weight = (
        (second[1] - third[1]) * (point[0] - third[0])
        + (third[0] - second[0]) * (point[1] - third[1])
    ) / denominator
    second_weight = (
        (third[1] - first[1]) * (point[0] - third[0])
        + (first[0] - third[0]) * (point[1] - third[1])
    ) / denominator
    third_weight = 1.0 - first_weight - second_weight
    if min(first_weight, second_weight, third_weight) < -1e-7:
        return None
    return (
        first_weight * first[2]
        + second_weight * second[2]
        + third_weight * third[2]
    )


def pick_body_candidates(
    groups: Sequence[BodyRenderGroup],
    center: Point3,
    yaw: float,
    pitch: float,
    scale: float,
    viewport: Point2,
    point: Point2,
) -> Tuple[int, ...]:
    """Return every Body below a screen point, nearest first."""

    body_depths = {}
    for group in groups:
        if group.opacity <= 0.005:
            continue
        for mesh in group.meshes:
            projected = tuple(
                project_point(vertex, center, yaw, pitch, scale, viewport)
                for vertex in mesh.vertices
            )
            for face in mesh.faces:
                if len(face) < 3 or not face_is_visible(mesh, face, yaw, pitch):
                    continue
                face_points = tuple(projected[index] for index in face)
                for offset in range(1, len(face_points) - 1):
                    depth = _triangle_depth_at_point(
                        point,
                        face_points[0],
                        face_points[offset],
                        face_points[offset + 1],
                    )
                    if depth is not None:
                        body_depths[group.body_index] = max(
                            depth,
                            body_depths.get(group.body_index, float("-inf")),
                        )
    return tuple(
        body_index
        for body_index, _depth in sorted(
            body_depths.items(),
            key=lambda item: (-item[1], item[0]),
        )
    )


def project_body_mesh_bounds(
    groups: Sequence[BodyRenderGroup],
    center: Point3,
    yaw: float,
    pitch: float,
    scale: float,
    viewport: Point2,
) -> Tuple[ProjectedBodyMeshBounds, ...]:
    """Project each mesh once so repeated pointer hits avoid a full face scan."""

    projected_bounds = []
    for group in groups:
        if group.opacity <= 0.005:
            continue
        for mesh in group.meshes:
            projected = tuple(
                project_point(vertex, center, yaw, pitch, scale, viewport)
                for vertex in mesh.vertices
            )
            if not projected:
                continue
            projected_bounds.append(
                ProjectedBodyMeshBounds(
                    body_index=group.body_index,
                    mesh=mesh,
                    min_x=min(point[0] for point in projected),
                    min_y=min(point[1] for point in projected),
                    max_x=max(point[0] for point in projected),
                    max_y=max(point[1] for point in projected),
                )
            )
    return tuple(projected_bounds)


def pick_body_candidates_from_bounds(
    bounds: Sequence[ProjectedBodyMeshBounds],
    center: Point3,
    yaw: float,
    pitch: float,
    scale: float,
    viewport: Point2,
    point: Point2,
    padding: float = 1.5,
) -> Tuple[int, ...]:
    """Run the exact hit test only for meshes whose projected bounds match."""

    meshes_by_body = {}
    for bound in bounds:
        if not (
            bound.min_x - padding <= point[0] <= bound.max_x + padding
            and bound.min_y - padding <= point[1] <= bound.max_y + padding
        ):
            continue
        meshes_by_body.setdefault(bound.body_index, []).append(bound.mesh)
    if not meshes_by_body:
        return ()
    candidates = tuple(
        BodyRenderGroup(
            body_index=body_index,
            meshes=tuple(meshes),
            color="#FFFFFF",
        )
        for body_index, meshes in meshes_by_body.items()
    )
    return pick_body_candidates(
        candidates,
        center,
        yaw,
        pitch,
        scale,
        viewport,
        point,
    )


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


def _blend_pixel(pixels: bytearray, index: int, color: RGBA) -> None:
    """Composite one straight-alpha colour over an opaque target pixel."""

    alpha = color[3] / 255.0
    if alpha <= 0.0:
        return
    if alpha >= 1.0:
        _write_pixel(pixels, index, color)
        return
    offset = index * 4
    inverse = 1.0 - alpha
    pixels[offset] = round(color[0] * alpha + pixels[offset] * inverse)
    pixels[offset + 1] = round(color[1] * alpha + pixels[offset + 1] * inverse)
    pixels[offset + 2] = round(color[2] * alpha + pixels[offset + 2] * inverse)
    pixels[offset + 3] = 255


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


def _rasterize_transparent_triangle(
    points: Sequence[Tuple[float, float, float]],
    width: int,
    height: int,
    opaque_depths: Sequence[float],
    layer_depths: List[float],
    layer_priorities: List[int],
    layer_pixels: bytearray,
    color: RGBA,
    priority: int,
) -> None:
    """Keep the nearest transparent fragment that is in front of opaque data."""

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
            if depth > opaque_depths[pixel_index] + 1e-7:
                previous_depth = layer_depths[pixel_index]
                if depth > previous_depth + 1e-7 or (
                    abs(depth - previous_depth) <= 1e-7
                    and priority >= layer_priorities[pixel_index]
                ):
                    layer_depths[pixel_index] = depth
                    layer_priorities[pixel_index] = priority
                    _write_pixel(layer_pixels, pixel_index, color)
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
    shape_opacities: Optional[Sequence[float]] = None,
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

    opacities = shape_opacities or (1.0,) * len(meshes)
    for shape_index, mesh in enumerate(meshes):
        opacity = opacities[shape_index % len(opacities)]
        if opacity < 0.995:
            continue
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

    transparent_indices = tuple(
        index
        for index in range(len(meshes))
        if 0.005 < opacities[index % len(opacities)] < 0.995
    )
    if transparent_indices:
        layer_depths = [float("-inf")] * (width * height)
        layer_priorities = [-1] * (width * height)
        layer_pixels = bytearray(width * height * 4)
        for shape_index in transparent_indices:
            mesh = meshes[shape_index]
            opacity = opacities[shape_index % len(opacities)]
            projected = tuple(
                project_point(vertex, center, yaw, pitch, scale, viewport)
                for vertex in mesh.vertices
            )
            for face_index, indices in enumerate(mesh.faces):
                if not face_is_visible(mesh, indices, yaw, pitch):
                    continue
                face_points = tuple(projected[index] for index in indices)
                shaded = _rgba(
                    shade_color(
                        shape_colors[shape_index % len(shape_colors)],
                        shade_factors[face_index % len(shade_factors)],
                    )
                )
                color = (shaded[0], shaded[1], shaded[2], round(opacity * 255))
                priority = shape_index * 256 + face_index
                for offset in range(1, len(face_points) - 1):
                    _rasterize_transparent_triangle(
                        (
                            face_points[0],
                            face_points[offset],
                            face_points[offset + 1],
                        ),
                        width,
                        height,
                        depths,
                        layer_depths,
                        layer_priorities,
                        layer_pixels,
                        color,
                        priority,
                    )
        for pixel_index in range(width * height):
            offset = pixel_index * 4
            if layer_pixels[offset + 3]:
                _blend_pixel(
                    pixels,
                    pixel_index,
                    tuple(layer_pixels[offset : offset + 4]),  # type: ignore[arg-type]
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
