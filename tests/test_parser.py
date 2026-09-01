from pathlib import Path
import struct
import tempfile
import unittest

from swphysics.definitions import DefinitionCatalog
from swphysics.model import (
    DEFAULT_COMPONENT_ROTATION,
    apply_matrix,
    multiply_matrices,
    parse_matrix,
)
from swphysics.vehicle import load_vehicle


FIXTURES = Path(__file__).parent / "fixtures"


class ParserTests(unittest.TestCase):
    def test_vehicle_parser_accepts_utf8_bom_without_utf8_sig_codec(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bom_vehicle.xml"
            path.write_bytes(
                b"\xef\xbb\xbf<vehicle data_version=\"3\"><bodies/></vehicle>"
            )
            vehicle = load_vehicle(path)
            self.assertEqual("3", vehicle.data_version)
            self.assertEqual((), vehicle.bodies)

    def test_stormworks_matrices_use_column_major_serialization(self):
        x_to_y = (0, 1, 0, -1, 0, 0, 0, 0, 1)
        y_to_z = (1, 0, 0, 0, 0, 1, 0, -1, 0)
        self.assertEqual((0, 1, 0), apply_matrix(x_to_y, (1, 0, 0)))
        combined = multiply_matrices(y_to_z, x_to_y)
        self.assertEqual((0, 0, 1), apply_matrix(combined, (1, 0, 0)))

    def test_matrix_parser_matches_game_atoll_integer_prefix_behavior(self):
        self.assertEqual(
            (0, 0, 1, -1, 2, -2, 0, 0, 1),
            parse_matrix(
                "-0.218,0.75,1.064,-1.7,2.9,-2.9,garbage,,1"
            ),
        )

    def test_definition_physics_rotation_keeps_native_xml_storage_order(self):
        text = '''<?xml version="1.0" encoding="UTF-8"?>
<definition name="Asymmetric Rotation"><voxels>
<voxel flags="1" physics_shape="1"><position/>
<physics_shape_rotation 00="0" 01="-1" 02="0" 10="0" 11="0" 12="1" 20="-1" 21="0" 22="0"/>
</voxel></voxels></definition>'''
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "asymmetric_rotation.xml"
            path.write_text(text, encoding="utf-8")
            rotation = DefinitionCatalog(Path(temporary_directory)).load(
                "asymmetric_rotation"
            ).voxels[0].physics_rotation
        self.assertEqual(
            (0, -1, 0, 0, 0, 1, -1, 0, 0),
            rotation,
        )
        self.assertEqual((0, -1, 0), apply_matrix(rotation, (1, 0, 0)))

    def test_component_order_and_default_definition(self):
        vehicle = load_vehicle(FIXTURES / "vehicles" / "order_b.xml")
        self.assertEqual(1, len(vehicle.bodies))
        self.assertEqual("01_block", vehicle.bodies[0].components[0].definition_id)
        self.assertEqual((0, 2, 0), vehicle.bodies[0].components[1].position)

    def test_omitted_component_rotation_uses_native_parser_default(self):
        vehicle = load_vehicle(FIXTURES / "vehicles" / "order_b.xml")

        self.assertEqual(
            DEFAULT_COMPONENT_ROTATION,
            vehicle.bodies[0].components[0].rotation,
        )

    def test_t_is_transform_index_not_basic_definition_type(self):
        text = '''<vehicle data_version="3"><bodies><body><components>
<c t="1"><o><vp x="4"/></o></c>
<c d="02_wedge" t="2"><o><vp/></o></c>
</components></body></bodies></vehicle>'''
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "transforms.xml"
            path.write_text(text, encoding="utf-8")
            components = load_vehicle(path).bodies[0].components
            self.assertEqual("01_block", components[0].definition_id)
            self.assertEqual(1, components[0].transform_index)
            self.assertEqual(
                (0, 0, -1, -1, 0, 0, 0, -1, 0),
                components[0].effective_transform,
            )
            self.assertEqual("02_wedge", components[1].definition_id)
            self.assertEqual(2, components[1].transform_index)

    def test_legacy_definition_and_trans_index_are_supported(self):
        text = '''<vehicle><bodies><body><components>
<c definition="02_wedge" trans_index="4"><o><vp/></o></c>
</components></body></bodies></vehicle>'''
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "legacy.xml"
            path.write_text(text, encoding="utf-8")
            component = load_vehicle(path).bodies[0].components[0]
            self.assertEqual("02_wedge", component.definition_id)
            self.assertEqual(4, component.transform_index)
            self.assertEqual(
                (1, 0, 0, 0, 1, 0, 0, 0, 1),
                component.rotation,
            )

    def test_definition_voxels_rotation_and_flags(self):
        catalog = DefinitionCatalog(FIXTURES / "definitions")
        self.assertEqual(0, catalog.load("02_wedge").flags)
        vehicle = load_vehicle(FIXTURES / "vehicles" / "rotation_and_shapes.xml")
        voxels = vehicle.physics_voxels(catalog, 0)
        self.assertEqual(3, len(voxels))
        self.assertEqual((5, 6, 7), voxels[0].position)
        self.assertEqual((5, 5, 7), voxels[1].position)
        self.assertEqual((9, 8, 7), voxels[2].position)
        self.assertEqual(1, voxels[2].physics_shape)
        self.assertEqual("02_wedge", voxels[2].component_definition)

    def test_xml_scale_expands_spacing_between_definition_voxels(self):
        text = '''<vehicle data_version="3"><bodies><body><components>
<c d="custom_two"><o r="3,0,0,0,1,0,0,0,1"><vp x="10"/></o></c>
</components></body></bodies></vehicle>'''
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "scaled_spacing.xml"
            path.write_text(text, encoding="utf-8")
            vehicle = load_vehicle(path)
            voxels = vehicle.physics_voxels(
                DefinitionCatalog(FIXTURES / "definitions"), 0
            )

        self.assertEqual(((10, 0, 0), (13, 0, 0)), tuple(
            voxel.position for voxel in voxels
        ))
        self.assertEqual(3, voxels[1].position[0] - voxels[0].position[0])

    def test_vehicle_component_bin_definition_is_loaded_from_sibling_package(self):
        definition_xml = b'''<?xml version="1.0" encoding="UTF-8"?>
<definition name="Custom Wedge" flags="57"><voxels>
<voxel flags="1" physics_shape="10"><position z="-1"/>
<physics_shape_rotation 00="1" 11="1" 22="1"/></voxel>
</voxels></definition>'''
        definition_id = b"custom_wedge"
        body = struct.pack("<I", 1) + definition_id + b"\0" + definition_xml
        payload = struct.pack("<I", len(body)) + body
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            vehicle = temporary / "Custom Vehicle.xml"
            vehicle.write_text("<vehicle/>", encoding="utf-8")
            package = temporary / "Custom Vehicle"
            package.mkdir()
            (package / "component.bin").write_bytes(payload)
            catalog = DefinitionCatalog.for_vehicle(
                FIXTURES / "definitions", vehicle
            )
            definition = catalog.load("custom_wedge")
            self.assertEqual("vehicle_component_bin", definition.source_format)
            self.assertEqual(1, len(definition.voxels))
            self.assertEqual(10, definition.voxels[0].physics_shape)
            self.assertEqual((0, 0, -1), definition.voxels[0].position)

    def test_legacy_component_bin_uses_filename_hash_as_definition_id(self):
        definition_xml = b'''<?xml version="1.0" encoding="UTF-8"?>
<definition name="Legacy Custom"><voxels>
<voxel flags="1"><position/></voxel>
</voxels></definition>'''
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            vehicle = temporary / "Legacy.xml"
            vehicle.write_text("<vehicle/>", encoding="utf-8")
            package = temporary / "Legacy"
            package.mkdir()
            body = definition_xml
            (package / "abc123.bin").write_bytes(
                struct.pack("<I", len(body)) + body
            )
            definition = DefinitionCatalog.for_vehicle(
                FIXTURES / "definitions", vehicle
            ).load("abc123")
            self.assertEqual("vehicle_component_bin", definition.source_format)
            self.assertEqual(1, len(definition.voxels))

    def test_vehicle_parser_accepts_stormworks_numeric_matrix_attributes(self):
        text = '''<?xml version="1.0" encoding="UTF-8"?>
<vehicle data_version="3"><bodies><body unique_id="7">
<initial_local_transform 00="1" 11="1" 22="1" 33="1"/>
<components><c><o><vp x="2"/></o></c></components>
</body></bodies></vehicle>'''
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "numeric-attributes.xml"
            path.write_text(text, encoding="utf-8")
            vehicle = load_vehicle(path)
            self.assertEqual(1, len(vehicle.bodies))
            self.assertEqual((2, 0, 0), vehicle.bodies[0].components[0].position)

    def test_definition_surface_collections_are_loaded_separately(self):
        text = '''<?xml version="1.0" encoding="UTF-8"?>
<definition name="Surface Fixture"><voxels>
<voxel flags="1"><position/></voxel>
</voxels><surfaces>
<surface orientation="5" rotation="3" shape="2" trans_type="7" flags="9">
<position x="-1" y="2" z="3"/>
</surface>
</surfaces><buoyancy_surfaces>
<surface orientation="4" rotation="2" shape="1" trans_type="6" flags="8">
<position x="3" y="2" z="-1"/>
</surface>
</buoyancy_surfaces><compartment_sample_pos x="4" y="-2" z="7"/></definition>'''
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "surface_fixture.xml"
            path.write_text(text, encoding="utf-8")
            definition = DefinitionCatalog(Path(temporary_directory)).load(
                "surface_fixture"
            )
            self.assertEqual(1, len(definition.surfaces))
            self.assertEqual(1, len(definition.buoyancy_surfaces))
            self.assertEqual((4, -2, 7), definition.compartment_sample_position)
            surface = definition.surfaces[0]
            self.assertEqual((-1, 2, 3), surface.position)
            self.assertEqual(
                (5, 3, 2, 7, 9),
                (
                    surface.orientation,
                    surface.rotation,
                    surface.shape,
                    surface.transmission_type,
                    surface.flags,
                ),
            )
            buoyancy_surface = definition.buoyancy_surfaces[0]
            self.assertEqual((3, 2, -1), buoyancy_surface.position)
            self.assertEqual(
                (4, 2, 1, 6, 8),
                (
                    buoyancy_surface.orientation,
                    buoyancy_surface.rotation,
                    buoyancy_surface.shape,
                    buoyancy_surface.transmission_type,
                    buoyancy_surface.flags,
                ),
            )

    def test_standard_door_seal_loads_without_static_flag4_panel_physics(self):
        definition_text = '''<?xml version="1.0" encoding="UTF-8"?>
<definition name="Door Fixture" type="13" door_side_dist="0" door_up_dist="1"
 door_lower_limit="-2" door_upper_limit="1" door_flipped="false"><voxels>
<voxel flags="1"><position x="-1"/></voxel>
<voxel flags="4"><position y="-1" z="2"/></voxel>
<voxel flags="4"><position y="0" z="2"/></voxel>
</voxels>
<door_size x="0.5" y="2" z="1"/>
<door_normal x="-1" y="0" z="0"/>
<door_side x="0" y="0" z="1"/>
<door_up x="0" y="1" z="0"/>
<door_base_pos x="0" y="-1" z="2"/>
</definition>'''
        vehicle_text = '''<vehicle data_version="3"><bodies><body unique_id="7">
<components><c d="door_fixture"><o r="1,0,0,0,1,0,0,0,1">
<vp x="10"/></o></c></components></body></bodies></vehicle>'''
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            (temporary / "door_fixture.xml").write_text(
                definition_text, encoding="utf-8"
            )
            vehicle_path = temporary / "door_vehicle.xml"
            vehicle_path.write_text(vehicle_text, encoding="utf-8")
            catalog = DefinitionCatalog(temporary)
            definition = catalog.load("door_fixture")
            vehicle = load_vehicle(vehicle_path)
            component = vehicle.bodies[0].components[0]
            static_voxels = vehicle.physics_voxels(catalog, 0)
            door_surfaces = component.closed_standard_door_surfaces(definition)

        self.assertTrue(definition.is_standard_door_component)
        self.assertTrue(definition.has_standard_door_seal)
        self.assertIsNotNone(definition.door_seal)
        self.assertEqual((-1, 0, 0), definition.door_seal.normal)
        self.assertEqual((0, 0, 1), definition.door_seal.side)
        self.assertEqual((0, 1, 0), definition.door_seal.up)
        self.assertEqual((0, -1, 2), definition.door_seal.base_position)
        self.assertEqual(0, definition.door_seal.side_distance)
        self.assertEqual(1, definition.door_seal.up_distance)
        self.assertEqual(((9, 0, 0),), tuple(v.position for v in static_voxels))
        self.assertEqual(
            (1, 4, 4),
            tuple(voxel.flags for voxel in definition.voxels),
        )
        self.assertEqual(((0, -1, 2), (0, 0, 2)), tuple(
            surface.position for surface in door_surfaces
        ))
        self.assertEqual((1, 1), tuple(
            surface.orientation for surface in door_surfaces
        ))


if __name__ == "__main__":
    unittest.main()
