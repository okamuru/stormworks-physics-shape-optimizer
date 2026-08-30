import math
import struct
import unittest

from PySide6.QtGui import QVector3D

from swphysics.gpu_viewer import (
    GPU_LINE_VERTEX_STRIDE,
    GPU_VERTEX_STRIDE,
    build_gpu_geometry,
    build_gpu_outlines,
    legacy_orbit_angles,
    legacy_view_orientation,
)
from swphysics.partition import Box
from swphysics.viewer import box_mesh, rotate_point


class GpuGeometryTests(unittest.TestCase):
    def test_cube_is_one_gpu_triangle_stream(self):
        geometry = build_gpu_geometry((box_mesh(Box((0, 0, 0), (0, 0, 0))),))

        self.assertEqual(12, geometry.triangle_count)
        self.assertEqual(36, geometry.vertex_count)
        self.assertEqual(36 * GPU_VERTEX_STRIDE, len(geometry.vertex_data))
        self.assertEqual((0.0, 0.0, 0.0), geometry.center)
        self.assertEqual(1.0, geometry.span)
        self.assertAlmostEqual(math.sqrt(3.0), geometry.fit_diameter)

    def test_gpu_triangles_are_outward_and_fully_opaque(self):
        geometry = build_gpu_geometry((box_mesh(Box((0, 0, 0), (0, 0, 0))),))
        vertices = tuple(
            struct.unpack_from("<7f", geometry.vertex_data, offset)
            for offset in range(0, len(geometry.vertex_data), GPU_VERTEX_STRIDE)
        )

        for offset in range(0, len(vertices), 3):
            first, second, third = vertices[offset : offset + 3]
            left = tuple(second[axis] - first[axis] for axis in range(3))
            right = tuple(third[axis] - first[axis] for axis in range(3))
            normal = (
                left[1] * right[2] - left[2] * right[1],
                left[2] * right[0] - left[0] * right[2],
                left[0] * right[1] - left[1] * right[0],
            )
            triangle_center = tuple(
                (first[axis] + second[axis] + third[axis]) / 3.0
                for axis in range(3)
            )
            self.assertGreater(
                sum(normal[axis] * triangle_center[axis] for axis in range(3)),
                0.0,
            )
            self.assertEqual((1.0, 1.0, 1.0), (first[6], second[6], third[6]))

    def test_cube_outline_deduplicates_its_twelve_edges(self):
        meshes = (box_mesh(Box((0, 0, 0), (0, 0, 0))),)
        geometry = build_gpu_geometry(meshes)
        outlines = build_gpu_outlines(meshes, geometry)

        self.assertEqual(12, outlines.segment_count)
        self.assertEqual(24, outlines.vertex_count)
        self.assertEqual(24 * GPU_LINE_VERTEX_STRIDE, len(outlines.vertex_data))

    def test_legacy_orbit_matches_software_viewer_sensitivity_and_clamp(self):
        yaw, pitch = legacy_orbit_angles(0.2, -0.3, 10.0, 20.0)
        self.assertAlmostEqual(0.32, yaw)
        self.assertAlmostEqual(-0.06, pitch)
        self.assertEqual((0.0, 1.45), legacy_orbit_angles(0.0, 1.4, 0.0, 20.0))
        self.assertEqual((0.0, -1.45), legacy_orbit_angles(0.0, -1.4, 0.0, -20.0))

    def test_gpu_orientation_matches_software_for_asymmetric_landmarks(self):
        yaw = math.radians(-42.0)
        pitch = math.radians(24.0)
        orientation = legacy_view_orientation(yaw, pitch)

        for point in (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (2.0, -3.0, 5.0),
        ):
            expected = rotate_point(point, yaw, pitch)
            actual = orientation.rotatedVector(QVector3D(*point))
            for axis, value in enumerate((actual.x(), actual.y(), actual.z())):
                self.assertAlmostEqual(expected[axis], value, delta=2e-6)


if __name__ == "__main__":
    unittest.main()
