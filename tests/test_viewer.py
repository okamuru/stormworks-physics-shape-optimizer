import math
import unittest

from swphysics.partition import Box
from swphysics.portable_merge import PortableMergeGroup
from swphysics.viewer import (
    ShapeMesh,
    box_mesh,
    box_vertices,
    face_is_visible,
    merge_group_mesh,
    outward_face_normal,
    project_point,
    rasterize_meshes_rgba,
    rotate_point,
    shade_color,
    stormworks_preview_mesh,
)


class ViewerMathTests(unittest.TestCase):
    def test_box_vertices_include_half_voxel_extents(self):
        vertices = box_vertices(Box((0, 0, 0), (1, 2, 3)))
        self.assertIn((-0.5, -0.5, -0.5), vertices)
        self.assertIn((1.5, 2.5, 3.5), vertices)

    def test_stormworks_preview_mirrors_only_x_without_changing_faces(self):
        source = ShapeMesh(
            ((2.0, -3.0, 5.0), (-7.0, 11.0, 13.0), (17.0, 19.0, -23.0)),
            ((0, 1, 2),),
        )

        preview = stormworks_preview_mesh(source)

        self.assertEqual(
            ((-2.0, -3.0, 5.0), (7.0, 11.0, 13.0), (-17.0, 19.0, -23.0)),
            preview.vertices,
        )
        self.assertEqual(source.faces, preview.faces)

    def test_zero_rotation_preserves_point(self):
        self.assertEqual((1.0, 2.0, 3.0), rotate_point((1.0, 2.0, 3.0), 0.0, 0.0))

    def test_projection_places_center_at_viewport_center(self):
        projected = project_point(
            (4.0, 5.0, 6.0),
            (4.0, 5.0, 6.0),
            math.radians(30),
            math.radians(20),
            50.0,
            (320.0, 180.0),
        )
        self.assertAlmostEqual(320.0, projected[0])
        self.assertAlmostEqual(180.0, projected[1])

    def test_color_shading_is_clamped(self):
        self.assertEqual("#ffffff", shade_color("#ffffff", 2.0))
        self.assertEqual("#000000", shade_color("#123456", 0.0))

    def test_merge_group_mesh_clips_a_wedge_plane(self):
        group = PortableMergeGroup(
            seed_insertion_index=0,
            voxel_insertion_indices=(0,),
            component_indices=(0,),
            minimum=(0, 0, 0),
            maximum=(0, 0, 0),
            planes=(((0, 1, 0), (1, 1, 0)),),
        )
        mesh = merge_group_mesh(group)
        self.assertEqual(6, len(mesh.vertices))
        self.assertEqual(5, len(mesh.faces))
        self.assertTrue(
            all(point[0] + point[1] <= 0.0 + 1e-7 for point in mesh.vertices)
        )

    def test_box_face_normals_are_oriented_outward_despite_inward_winding(self):
        mesh = box_mesh(Box((0, 0, 0), (0, 0, 0)))
        normals = {outward_face_normal(mesh, face) for face in mesh.faces}
        self.assertEqual(
            {
                (-1.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, -1.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, -1.0),
                (0.0, 0.0, 1.0),
            },
            normals,
        )

    def test_backface_culling_tracks_opposite_camera_angles(self):
        mesh = box_mesh(Box((0, 0, 0), (0, 0, 0)))
        front_at_zero = {
            index
            for index, face in enumerate(mesh.faces)
            if face_is_visible(mesh, face, 0.0, 0.0)
        }
        front_after_half_turn = {
            index
            for index, face in enumerate(mesh.faces)
            if face_is_visible(mesh, face, math.pi, 0.0)
        }
        self.assertEqual({1}, front_at_zero)
        self.assertEqual({0}, front_after_half_turn)

    def test_generic_camera_angle_exposes_three_cube_faces(self):
        mesh = box_mesh(Box((0, 0, 0), (0, 0, 0)))
        visible = [
            face
            for face in mesh.faces
            if face_is_visible(mesh, face, math.radians(-42), math.radians(24))
        ]
        self.assertEqual(3, len(visible))

    def test_clipped_mesh_faces_swap_visibility_from_opposite_view(self):
        mesh = merge_group_mesh(
            PortableMergeGroup(
                seed_insertion_index=0,
                voxel_insertion_indices=(0,),
                component_indices=(0,),
                minimum=(0, 0, 0),
                maximum=(0, 0, 0),
                planes=(((0, 1, 0), (1, 1, 0)),),
            )
        )
        yaw = math.radians(17)
        pitch = math.radians(31)
        for face in mesh.faces:
            self.assertNotEqual(
                face_is_visible(mesh, face, yaw, pitch),
                face_is_visible(mesh, face, yaw + math.pi, -pitch),
            )

    def test_z_buffer_selects_nearer_shape_independent_of_draw_order(self):
        far = box_mesh(Box((0, 0, 0), (0, 0, 0)))
        near = box_mesh(Box((0, 0, 2), (0, 0, 2)))

        def center_pixel(meshes, colors):
            pixels = rasterize_meshes_rgba(
                meshes,
                center=(0.0, 0.0, 1.0),
                yaw=0.0,
                pitch=0.0,
                scale=20.0,
                viewport=(32.0, 32.0),
                width=64,
                height=64,
                shape_colors=colors,
            )
            offset = (32 * 64 + 32) * 4
            return tuple(pixels[offset : offset + 4])

        expected_near_green = (0, 235, 0, 255)
        self.assertEqual(
            expected_near_green,
            center_pixel((far, near), ("#ff0000", "#00ff00")),
        )
        self.assertEqual(
            expected_near_green,
            center_pixel((near, far), ("#00ff00", "#ff0000")),
        )

    def test_z_buffer_uses_one_stable_winner_for_coincident_surfaces(self):
        cube = box_mesh(Box((0, 0, 0), (0, 0, 0)))
        pixels = rasterize_meshes_rgba(
            (cube, cube),
            center=(0.0, 0.0, 0.0),
            yaw=0.0,
            pitch=0.0,
            scale=32.0,
            viewport=(32.0, 32.0),
            width=64,
            height=64,
            shape_colors=("#ff0000", "#00ff00"),
        )

        interior = {
            tuple(pixels[(y * 64 + x) * 4 : (y * 64 + x) * 4 + 4])
            for y in range(18, 46)
            for x in range(18, 46)
        }
        self.assertEqual({(0, 235, 0, 255)}, interior)

    def test_fast_preview_can_skip_outlines_without_losing_surfaces(self):
        cube = box_mesh(Box((0, 0, 0), (0, 0, 0)))
        pixels = rasterize_meshes_rgba(
            (cube,),
            center=(0.0, 0.0, 0.0),
            yaw=0.0,
            pitch=0.0,
            scale=32.0,
            viewport=(32.0, 32.0),
            width=64,
            height=64,
            shape_colors=("#00ff00",),
            draw_outlines=False,
        )

        offset = (32 * 64 + 32) * 4
        self.assertEqual((0, 235, 0, 255), tuple(pixels[offset : offset + 4]))


if __name__ == "__main__":
    unittest.main()
