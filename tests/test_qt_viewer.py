import math
import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from swphysics.partition import Box
from swphysics.physics_viewer import PhysicsShapeViewer as UnifiedPhysicsShapeViewer
from swphysics.qt_viewer import (
    FAST_PREVIEW_SOLID_COLOR,
    FAST_PREVIEW_ZBUFFER_SHAPE_LIMIT,
    PhysicsShapeViewer,
    fast_preview_faces,
    fast_preview_silhouettes,
    rasterize_fast_preview,
)
from swphysics.viewer import SHAPE_COLORS, box_mesh, shade_color


class UnifiedPreviewCoordinateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def test_shared_viewer_mirrors_vehicle_x_for_software_and_gpu_boundary(self):
        viewer = UnifiedPhysicsShapeViewer()
        viewer.set_shapes((box_mesh(Box((3, 0, 0), (3, 0, 0))),))

        preview_x = tuple(vertex[0] for vertex in viewer.meshes[0].vertices)
        self.assertEqual((-3.5, -2.5), (min(preview_x), max(preview_x)))
        viewer.close()

    def test_set_boxes_uses_the_same_preview_conversion(self):
        viewer = UnifiedPhysicsShapeViewer()
        viewer.set_boxes((Box((-4, 0, 0), (-4, 0, 0)),))

        preview_x = tuple(vertex[0] for vertex in viewer.meshes[0].vertices)
        self.assertEqual((3.5, 4.5), (min(preview_x), max(preview_x)))
        viewer.close()


class FastPreviewFaceTests(unittest.TestCase):
    def test_cube_culls_back_faces_during_interaction(self):
        faces = fast_preview_faces(
            (box_mesh(Box((0, 0, 0), (0, 0, 0))),),
            center=(0.0, 0.0, 0.0),
            yaw=0.0,
            pitch=0.0,
            scale=32.0,
            viewport=(32.0, 32.0),
        )

        self.assertEqual(1, len(faces))

    def test_faces_are_sorted_far_to_near(self):
        faces = fast_preview_faces(
            (
                box_mesh(Box((0, 0, 2), (0, 0, 2))),
                box_mesh(Box((0, 0, 0), (0, 0, 0))),
            ),
            center=(0.0, 0.0, 1.0),
            yaw=0.0,
            pitch=0.0,
            scale=32.0,
            viewport=(32.0, 32.0),
        )

        self.assertEqual(sorted(face[0] for face in faces), [face[0] for face in faces])


class FastPreviewRasterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def test_preview_uses_a_bounded_render_target(self):
        pixels, width, height = rasterize_fast_preview(
            (box_mesh(Box((0, 0, 0), (0, 0, 0))),),
            center=(0.0, 0.0, 0.0),
            yaw=0.0,
            pitch=0.0,
            scale=100.0,
            viewport=(600.0, 400.0),
            width=1200,
            height=800,
        )

        self.assertEqual((360, 240), (width, height))
        self.assertEqual(width * height * 4, len(pixels))

    def test_silhouette_preview_preserves_a_gap_between_shapes(self):
        silhouettes = fast_preview_silhouettes(
            (
                box_mesh(Box((-3, 0, 0), (-3, 0, 0))),
                box_mesh(Box((3, 0, 0), (3, 0, 0))),
            ),
            center=(0.0, 0.0, 0.0),
            yaw=0.0,
            pitch=0.0,
            scale=20.0,
            viewport=(100.0, 100.0),
        )

        self.assertEqual(2, len(silhouettes))
        self.assertLess(max(point[0] for point in silhouettes[0]), 100.0)
        self.assertGreater(min(point[0] for point in silhouettes[1]), 100.0)

    def test_interaction_preview_is_opaque_from_opposite_views(self):
        far = box_mesh(Box((0, 0, 0), (0, 0, 0)))
        near = box_mesh(Box((0, 0, 2), (0, 0, 2)))
        viewer = PhysicsShapeViewer()
        viewer.resize(320, 320)
        viewer.set_shapes((far, near))
        viewer.pitch = 0.0
        viewer._preview_mode = True

        viewer.yaw = 0.0
        front_image = viewer.grab().toImage()
        front = front_image.pixelColor(160, 160)
        self.assertEqual(
            shade_color(SHAPE_COLORS[1], 0.92), front.name()
        )
        self.assertEqual(255, front.alpha())

        viewer.yaw = math.pi
        back_image = viewer.grab().toImage()
        back = back_image.pixelColor(160, 160)
        self.assertEqual(
            shade_color(SHAPE_COLORS[0], 0.72), back.name()
        )
        self.assertEqual(255, back.alpha())
        viewer.close()

    def test_large_interaction_preview_is_an_opaque_silhouette(self):
        cube = box_mesh(Box((0, 0, 0), (0, 0, 0)))
        viewer = PhysicsShapeViewer()
        viewer.resize(320, 320)
        viewer.set_shapes(
            (cube,) * (FAST_PREVIEW_ZBUFFER_SHAPE_LIMIT + 1)
        )
        viewer.yaw = 0.0
        viewer.pitch = 0.0
        viewer._preview_mode = True

        image = viewer.grab().toImage()
        center = image.pixelColor(160, 160)
        self.assertEqual(FAST_PREVIEW_SOLID_COLOR.lower(), center.name())
        self.assertEqual(255, center.alpha())
        viewer.close()


if __name__ == "__main__":
    unittest.main()
