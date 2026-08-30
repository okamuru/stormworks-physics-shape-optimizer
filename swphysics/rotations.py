"""Exact quarter-turn component rotations used by Stormworks vehicle XML."""

from __future__ import annotations

from itertools import permutations, product
from typing import Tuple

from .model import Matrix3


def _determinant(matrix: Matrix3) -> int:
    # Serialized matrices are column-major; determinant is layout invariant.
    a, d, g, b, e, h, c, f, i = matrix
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def proper_grid_rotations() -> Tuple[Matrix3, ...]:
    """Return the 24 determinant-positive signed permutation matrices."""

    result = []
    for axes in permutations(range(3)):
        for signs in product((-1, 1), repeat=3):
            columns = []
            for axis, sign in zip(axes, signs):
                columns.extend(sign if row == axis else 0 for row in range(3))
            matrix: Matrix3 = tuple(columns)  # type: ignore[assignment]
            if _determinant(matrix) == 1:
                result.append(matrix)
    return tuple(sorted(result))


PROPER_GRID_ROTATIONS = proper_grid_rotations()


def grid_transforms() -> Tuple[Matrix3, ...]:
    """Return all 48 signed permutation transforms, including reflections."""

    result = []
    for axes in permutations(range(3)):
        for signs in product((-1, 1), repeat=3):
            columns = []
            for axis, sign in zip(axes, signs):
                columns.extend(sign if row == axis else 0 for row in range(3))
            result.append(tuple(columns))
    return tuple(sorted(result))  # type: ignore[return-value]


GRID_TRANSFORMS = grid_transforms()
