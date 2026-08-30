from dataclasses import dataclass
import re
from typing import Iterable, Tuple


GridPoint = Tuple[int, int, int]
Matrix3 = Tuple[int, int, int, int, int, int, int, int, int]

IDENTITY_MATRIX: Matrix3 = (1, 0, 0, 0, 1, 0, 0, 0, 1)
# ``c_scene_vehicle_component_base::parse_data`` in build 24749959 passes this
# serialized matrix as the default value for an omitted ``o/@r`` attribute.
# It is the editor's native component orientation, not the mathematical
# identity matrix.
DEFAULT_COMPONENT_ROTATION: Matrix3 = (0, 0, 1, -1, 0, 0, 0, -1, 0)
_ATOLL_PREFIX_RE = re.compile(r"^[\t\n\v\f\r ]*([+-]?[0-9]+)")


def parse_fixed_point_int(raw: str) -> int:
    """Match the game's build-24749959 fixed-point integer XML parser.

    The installed binary's ``parse_value_fixed_point<int>`` calls ``atoll``.
    That matters for edited saves containing values such as ``-0.218``: the
    game consumes the integer prefix (zero here) rather than rejecting XML.
    """

    match = _ATOLL_PREFIX_RE.match(raw)
    return int(match.group(1)) if match is not None else 0


def add_points(left: GridPoint, right: GridPoint) -> GridPoint:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def apply_matrix(matrix: Matrix3, point: GridPoint) -> GridPoint:
    """Apply Stormworks' column-major serialized matrix to a column vector."""

    x, y, z = point
    return (
        matrix[0] * x + matrix[3] * y + matrix[6] * z,
        matrix[1] * x + matrix[4] * y + matrix[7] * z,
        matrix[2] * x + matrix[5] * y + matrix[8] * z,
    )


def multiply_matrices(left: Matrix3, right: Matrix3) -> Matrix3:
    # Serialized tuples are column-major.  Compose them so that applying the
    # result is equivalent to apply(left, apply(right, point)).
    values = []
    for column in range(3):
        basis = tuple(1 if index == column else 0 for index in range(3))
        transformed = apply_matrix(left, apply_matrix(right, basis))
        values.extend(transformed)
    return tuple(values)  # type: ignore[return-value]


def transform_index_matrix(transform_index: int) -> Matrix3:
    """Return the local-axis reflection selected by vehicle ``c/@t``.

    The transformed Definition variants use bits 0, 1 and 2 for local X, Y
    and Z respectively.  The native Definition ``flip`` routine multiplies
    voxel coordinates by these signs before the component rotation is applied.
    """

    if not 0 <= transform_index <= 7:
        raise ValueError(
            "component transform index must be between 0 and 7: {}".format(
                transform_index
            )
        )
    return (
        -1 if transform_index & 1 else 1,
        0,
        0,
        0,
        -1 if transform_index & 2 else 1,
        0,
        0,
        0,
        -1 if transform_index & 4 else 1,
    )


def parse_matrix(raw: str) -> Matrix3:
    values = tuple(parse_fixed_point_int(value) for value in raw.split(","))
    if len(values) != 9:
        raise ValueError("rotation matrix must contain exactly 9 integers: {!r}".format(raw))
    return values  # type: ignore[return-value]


def point_from_attributes(attributes: dict) -> GridPoint:
    return (
        parse_fixed_point_int(attributes.get("x", "0")),
        parse_fixed_point_int(attributes.get("y", "0")),
        parse_fixed_point_int(attributes.get("z", "0")),
    )


@dataclass(frozen=True)
class DefinitionVoxel:
    position: GridPoint
    flags: int
    physics_shape: int
    physics_rotation: Matrix3

    @property
    def contributes_physics(self) -> bool:
        return bool(self.flags & 1)


@dataclass(frozen=True)
class WorldVoxel:
    body_index: int
    body_id: str
    component_index: int
    component_definition: str
    definition_voxel_index: int
    insertion_index: int
    position: GridPoint
    physics_shape: int
    physics_rotation: Matrix3


def unique_points(points: Iterable[GridPoint]) -> Tuple[GridPoint, ...]:
    seen = set()
    result = []
    for point in points:
        if point not in seen:
            seen.add(point)
            result.append(point)
    return tuple(result)
