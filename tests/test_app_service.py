from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

from swphysics import app_service
from swphysics.app_service import (
    apply_manual_body_exclusions,
    analyze_vehicle,
    optimize_vehicle_copy,
    save_analyzed_vehicle_copy,
)
from swphysics.vehicle import load_vehicle


ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests" / "fixtures"


def _deep_size(value):
    """Return retained Python size for an acyclic analysis result graph."""

    seen = set()

    def visit(item):
        identity = id(item)
        if identity in seen:
            return 0
        seen.add(identity)
        size = sys.getsizeof(item)
        if isinstance(item, dict):
            return size + sum(visit(key) + visit(child) for key, child in item.items())
        if isinstance(item, (tuple, list, set, frozenset)):
            return size + sum(visit(child) for child in item)
        if hasattr(item, "__dict__"):
            return size + visit(vars(item))
        return size

    return visit(value)


class AppServiceTests(unittest.TestCase):
    def test_omitted_rotation_wedge_row_is_one_preview_shape(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            source = Path(temporary_directory) / "logic.xml"
            components = "".join(
                '<c d="02_wedge" t="2"><o><vp x="10" y="12" z="{}"/></o></c>'.format(
                    z
                )
                for z in range(-16, 0)
            )
            source.write_text(
                '<vehicle data_version="3"><bodies><body unique_id="logic">'
                '<components>{}</components></body></bodies></vehicle>'.format(
                    components
                ),
                encoding="utf-8",
            )

            analysis = analyze_vehicle(source, FIXTURES / "definitions")

            self.assertEqual(1, analysis.current_shape_count)
            self.assertEqual(1, len(analysis.bodies[0].current_meshes))
            mesh = analysis.bodies[0].current_meshes[0]
            self.assertEqual(6, len(mesh.vertices))
            self.assertEqual(5, len(mesh.faces))
            self.assertEqual(
                (-16.5, -0.5),
                (
                    min(vertex[2] for vertex in mesh.vertices),
                    max(vertex[2] for vertex in mesh.vertices),
                ),
            )

    def test_manual_body_exclusion_is_reversible_without_reanalysis(self):
        analysis = analyze_vehicle(
            FIXTURES / "vehicles" / "order_b.xml",
            FIXTURES / "definitions",
        )
        self.assertEqual(3, analysis.optimized_shape_count)

        excluded = apply_manual_body_exclusions(analysis, (0,))

        self.assertEqual(1, excluded.manually_excluded_body_count)
        self.assertTrue(excluded.bodies[0].manually_excluded)
        self.assertEqual(4, excluded.optimized_shape_count)
        self.assertEqual(
            excluded.bodies[0].current_meshes,
            excluded.bodies[0].effective_optimized_meshes,
        )
        self.assertIs(
            analysis.bodies[0].optimization_result,
            excluded.bodies[0].optimization_result,
        )

        restored = apply_manual_body_exclusions(excluded, ())

        self.assertEqual(0, restored.manually_excluded_body_count)
        self.assertEqual(3, restored.optimized_shape_count)
        self.assertEqual(
            analysis.bodies[0].optimized_meshes,
            restored.bodies[0].effective_optimized_meshes,
        )

    def test_manual_body_exclusion_saves_that_body_in_its_exact_original_order(self):
        source = FIXTURES / "vehicles" / "order_b.xml"
        analysis = analyze_vehicle(source, FIXTURES / "definitions")
        excluded = apply_manual_body_exclusions(analysis, (0,))

        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "excluded.xml"
            result = save_analyzed_vehicle_copy(excluded, output)

            self.assertEqual(source.read_bytes(), output.read_bytes())
            self.assertEqual(4, result.report.before_shape_count)
            self.assertEqual(4, result.report.after_shape_count)
            self.assertEqual(
                "manual_body_exclusion_identity",
                result.report.bodies[0].result.search,
            )

    def test_manual_body_exclusion_only_locks_the_selected_body(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            source = temporary / "two_bodies.xml"
            output = temporary / "optimized.xml"
            component_xml = """\
  <c><o><vp/></o></c>
  <c><o><vp y=\"2\"/></o></c>
  <c><o><vp y=\"1\"/></o></c>
  <c><o><vp y=\"3\"/></o></c>
  <c><o><vp x=\"1\"/></o></c>
  <c><o><vp x=\"1\" y=\"2\"/></o></c>
"""
            source.write_text(
                "<vehicle data_version=\"3\"><bodies>"
                "<body unique_id=\"first\"><components>{}</components></body>"
                "<body unique_id=\"second\"><components>{}</components></body>"
                "</bodies></vehicle>".format(component_xml, component_xml),
                encoding="utf-8",
            )
            analysis = analyze_vehicle(source, FIXTURES / "definitions")
            excluded = apply_manual_body_exclusions(analysis, (1,))

            self.assertEqual(8, excluded.current_shape_count)
            self.assertEqual(7, excluded.optimized_shape_count)
            self.assertFalse(excluded.bodies[0].manually_excluded)
            self.assertTrue(excluded.bodies[1].manually_excluded)

            save_analyzed_vehicle_copy(excluded, output)
            before = load_vehicle(source)
            after = load_vehicle(output)
            self.assertNotEqual(before.bodies[0].components, after.bodies[0].components)
            self.assertEqual(before.bodies[1].components, after.bodies[1].components)

    def test_manual_body_exclusion_rejects_an_unknown_body_index(self):
        analysis = analyze_vehicle(
            FIXTURES / "vehicles" / "order_b.xml",
            FIXTURES / "definitions",
        )

        with self.assertRaisesRegex(ValueError, "unknown Body index: 99"):
            apply_manual_body_exclusions(analysis, (99,))

    def test_vehicle_hashing_streams_without_path_read_bytes(self):
        source = FIXTURES / "vehicles" / "order_b.xml"
        expected = app_service.sha256(source.read_bytes()).hexdigest()
        with patch.object(
            Path,
            "read_bytes",
            side_effect=AssertionError("vehicle hashing must be streamed"),
        ):
            actual = app_service._sha256_file(source)

        self.assertEqual(expected, actual)

    def test_analysis_caches_only_a_compact_save_ready_order(self):
        raw_results = []
        optimize = app_service.optimize_staged_component_order

        def capture_result(*args, **kwargs):
            result = optimize(*args, **kwargs)
            raw_results.append(result)
            return result

        with patch(
            "swphysics.app_service.optimize_staged_component_order",
            side_effect=capture_result,
        ):
            analysis = analyze_vehicle(
                FIXTURES / "vehicles" / "order_b.xml",
                FIXTURES / "definitions",
            )

        cached = analysis.bodies[0].optimization_result
        self.assertIsNotNone(cached)
        assert cached is not None
        self.assertEqual(
            raw_results[0].optimized_component_order,
            cached.optimized_component_order,
        )
        self.assertEqual(raw_results[0].before.shape_count, cached.before.shape_count)
        self.assertEqual(raw_results[0].after.shape_count, cached.after.shape_count)
        self.assertEqual(
            cached.component_count * 4,
            len(cached.packed_optimized_component_order),
        )
        self.assertEqual(raw_results[0].changed, cached.changed)
        self.assertLess(_deep_size(cached), _deep_size(raw_results[0]) // 2)

    def test_body_without_flooder_skips_duplicate_flood_fill_expansion(self):
        progress = []
        with patch(
            "swphysics.app_service.model_surface_physics_flood_fill",
            side_effect=AssertionError("Flooder-free body must skip flood fill"),
        ):
            analysis = analyze_vehicle(
                FIXTURES / "vehicles" / "order_b.xml",
                FIXTURES / "definitions",
                progress_callback=lambda percent, message: progress.append(
                    (percent, message)
                ),
            )

        self.assertTrue(analysis.can_optimize)
        self.assertEqual(4, analysis.current_shape_count)
        self.assertEqual(3, analysis.optimized_shape_count)
        self.assertTrue(any("Physics Flooder" in message for _value, message in progress))

    def test_deep_mode_runs_stages_until_the_first_plateau(self):
        analysis = analyze_vehicle(
            FIXTURES / "vehicles" / "order_b.xml",
            FIXTURES / "definitions",
            search_mode="deep",
            worker_count=0,
        )

        self.assertEqual("deep", analysis.search_mode)
        self.assertEqual(0, analysis.requested_worker_count)
        self.assertGreaterEqual(
            analysis.bodies[0].completed_search_stage_count,
            1,
        )
        self.assertLessEqual(
            analysis.bodies[0].completed_search_stage_count,
            3,
        )

    def test_analysis_reports_monotonic_real_work_progress(self):
        progress = []
        analysis = analyze_vehicle(
            FIXTURES / "vehicles" / "order_b.xml",
            FIXTURES / "definitions",
            progress_callback=lambda percent, message: progress.append(
                (percent, message)
            ),
        )

        self.assertTrue(analysis.can_optimize)
        self.assertEqual(0, progress[0][0])
        self.assertEqual(100, progress[-1][0])
        self.assertEqual(
            sorted(percent for percent, _message in progress),
            [percent for percent, _message in progress],
        )
        self.assertTrue(any("配置候補" in message for _percent, message in progress))
        self.assertTrue(any("3Dプレビュー" in message for _percent, message in progress))

    def test_analysis_exposes_current_and_optimized_boxes(self):
        analysis = analyze_vehicle(
            FIXTURES / "vehicles" / "order_b.xml", FIXTURES / "definitions"
        )
        self.assertTrue(analysis.can_optimize)
        self.assertEqual(4, analysis.current_shape_count)
        self.assertEqual(3, analysis.optimized_shape_count)
        self.assertEqual(4, len(analysis.bodies[0].current_boxes))
        self.assertEqual(3, len(analysis.bodies[0].optimized_boxes or ()))

    def test_mixed_vehicle_is_previewed_and_optimized_by_portable_exact_model(self):
        analysis = analyze_vehicle(
            FIXTURES / "vehicles" / "rotation_and_shapes.xml", FIXTURES / "definitions"
        )
        self.assertTrue(analysis.can_optimize)
        self.assertIsNotNone(analysis.optimized_shape_count)
        self.assertGreaterEqual(len(analysis.bodies[0].current_meshes), 1)

    def test_multi_voxel_cube_components_are_optimized_as_whole_groups(self):
        analysis = analyze_vehicle(
            FIXTURES / "vehicles" / "multi_cube_order_bad.xml", FIXTURES / "definitions"
        )
        self.assertTrue(analysis.can_optimize)
        self.assertEqual(2, analysis.current_shape_count)
        self.assertIsNotNone(analysis.optimized_shape_count)
        self.assertEqual(6, analysis.bodies[0].cube_voxel_count)
        self.assertGreater(analysis.bodies[0].evaluated_order_count, 0)

    def test_non_physics_and_stretched_cube_components_remain_supported(self):
        for name in ("no_physics_component.xml", "stretched_cube_component.xml"):
            with self.subTest(name=name):
                analysis = analyze_vehicle(
                    FIXTURES / "vehicles" / name,
                    FIXTURES / "definitions",
                )
                self.assertTrue(analysis.can_optimize)
                self.assertFalse(analysis.has_partial_shape_coverage)

    def test_unpredicted_non_cube_is_fixed_while_supported_components_optimize(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            source = temporary / "mixed_xml_edited.xml"
            output = temporary / "optimized.xml"
            source.write_text(
                '''<?xml version="1.0" encoding="UTF-8"?>
<vehicle data_version="3"><authors/><bodies><body unique_id="25"><components>
  <c><o><vp/></o></c>
  <c><o><vp y="2"/></o></c>
  <c><o><vp y="1"/></o></c>
  <c><o><vp y="3"/></o></c>
  <c><o><vp x="1"/></o></c>
  <c><o><vp x="1" y="2"/></o></c>
</components></body><body unique_id="26"><components>
  <c d="02_wedge"><o r="-3,0,0,1,3,0,0,0,-2"><vp x="99"/></o></c>
  <c><o><vp x="100"/></o></c>
</components></body></bodies></vehicle>
''',
                encoding="utf-8",
            )

            analysis = analyze_vehicle(source, FIXTURES / "definitions")
            self.assertTrue(analysis.can_optimize)
            self.assertEqual(1, analysis.xml_edited_component_count)
            self.assertEqual(0, analysis.protected_body_count)
            self.assertEqual(1, analysis.partially_optimized_body_count)
            self.assertEqual(5, analysis.current_shape_count)
            self.assertEqual(4, analysis.optimized_shape_count)
            self.assertFalse(analysis.bodies[0].protected_body)
            self.assertFalse(analysis.bodies[1].protected_body)
            self.assertEqual(1, analysis.bodies[1].current_shape_count)
            self.assertEqual(1, analysis.bodies[1].optimized_shape_count)
            self.assertIn("元スロット", analysis.bodies[1].reason)

            save_analyzed_vehicle_copy(analysis, output)
            before = load_vehicle(source)
            after = load_vehicle(output)
            self.assertNotEqual(
                tuple(component.position for component in before.bodies[0].components),
                tuple(component.position for component in after.bodies[0].components),
            )
            self.assertEqual(
                before.bodies[1].components,
                after.bodies[1].components,
            )
            source_bytes = source.read_bytes()
            output_bytes = output.read_bytes()
            body_marker = b'<body unique_id="26">'
            source_start = source_bytes.index(body_marker)
            output_start = output_bytes.index(body_marker)
            source_end = source_bytes.index(b"</body>", source_start) + len(b"</body>")
            output_end = output_bytes.index(b"</body>", output_start) + len(b"</body>")
            self.assertEqual(
                source_bytes[source_start:source_end],
                output_bytes[output_start:output_end],
            )

    def test_unpredicted_component_keeps_its_slot_while_same_body_is_reordered(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            source = temporary / "partial_body.xml"
            output = temporary / "optimized.xml"
            source.write_text(
                '''<?xml version="1.0" encoding="UTF-8"?>
<vehicle data_version="3"><authors/><bodies><body unique_id="partial"><components>
  <c><o><vp/></o></c>
  <c><o><vp y="2"/></o></c>
  <c d="02_wedge" custom_marker="keep-me"><o r="-3,0,0,1,3,0,0,0,-2"><vp x="99"/></o></c>
  <c><o><vp y="1"/></o></c>
  <c><o><vp y="3"/></o></c>
  <c><o><vp x="1"/></o></c>
  <c><o><vp x="1" y="2"/></o></c>
</components></body></bodies></vehicle>
''',
                encoding="utf-8",
            )

            analysis = analyze_vehicle(source, FIXTURES / "definitions")
            body = analysis.bodies[0]
            order = body.optimization_result.optimized_component_order

            self.assertTrue(analysis.can_optimize)
            self.assertEqual(4, analysis.current_shape_count)
            self.assertEqual(3, analysis.optimized_shape_count)
            self.assertEqual(2, order[2])
            self.assertNotEqual(tuple(range(7)), order)

            save_analyzed_vehicle_copy(analysis, output)
            before = load_vehicle(source).bodies[0]
            after = load_vehicle(output).bodies[0]
            self.assertEqual(before.components[2], after.components[2])
            self.assertNotEqual(before.components, after.components)
            excluded_bytes = (
                b'<c d="02_wedge" custom_marker="keep-me"><o '
                b'r="-3,0,0,1,3,0,0,0,-2"><vp x="99"/></o></c>'
            )
            self.assertIn(excluded_bytes, output.read_bytes())

    def test_unknown_flooder_surface_is_excluded_while_other_components_optimize(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            definitions = temporary / "definitions"
            shutil.copytree(FIXTURES / "definitions", definitions)
            (definitions / "physics_flooder.xml").write_text(
                '''<?xml version="1.0" encoding="UTF-8"?>
<definition name="Physics Flooder" water_component_type="19">
  <voxels/>
</definition>
''',
                encoding="utf-8",
            )
            (definitions / "xml_edited_surface.xml").write_text(
                '''<?xml version="1.0" encoding="UTF-8"?>
<definition name="XML Edited Surface">
  <voxels><voxel flags="1" physics_shape="0"><position/></voxel></voxels>
  <buoyancy_surfaces>
    <surface orientation="0" rotation="0" shape="1" flags="1"><position/></surface>
    <surface orientation="1" rotation="0" shape="1" flags="1"><position/></surface>
    <surface orientation="2" rotation="0" shape="1" flags="1"><position/></surface>
    <surface orientation="3" rotation="0" shape="1" flags="1"><position/></surface>
    <surface orientation="4" rotation="0" shape="1" flags="1"><position/></surface>
    <surface orientation="5" rotation="0" shape="1" flags="1"><position/></surface>
  </buoyancy_surfaces>
</definition>
''',
                encoding="utf-8",
            )
            source = temporary / "mixed_unknown_flooder_surface.xml"
            output = temporary / "optimized.xml"
            source.write_text(
                '''<?xml version="1.0" encoding="UTF-8"?>
<vehicle data_version="3"><authors/><bodies>
<body unique_id="unsupported"><components>
  <c d="physics_flooder"><o><vp/></o></c>
  <c d="xml_edited_surface"><o r="2,0,0,0,1,0,0,0,1"><vp x="50"/></o></c>
</components></body>
<body unique_id="supported"><components>
  <c><o><vp/></o></c>
  <c><o><vp y="2"/></o></c>
  <c><o><vp y="1"/></o></c>
  <c><o><vp y="3"/></o></c>
  <c><o><vp x="1"/></o></c>
  <c><o><vp x="1" y="2"/></o></c>
</components></body></bodies></vehicle>
''',
                encoding="utf-8",
            )

            analysis = analyze_vehicle(source, definitions)

            self.assertTrue(analysis.can_optimize)
            self.assertEqual(0, analysis.protected_body_count)
            self.assertEqual(1, analysis.partially_optimized_body_count)
            self.assertEqual(1, analysis.flooder_prediction_excluded_body_count)
            self.assertEqual(1, analysis.xml_edited_component_count)
            self.assertFalse(analysis.bodies[0].protected_body)
            self.assertTrue(analysis.bodies[0].flooder_prediction_excluded)
            self.assertEqual(0, analysis.bodies[0].current_shape_count)
            self.assertEqual(0, analysis.bodies[0].optimized_shape_count)
            self.assertEqual(4, analysis.current_shape_count)
            self.assertEqual(3, analysis.optimized_shape_count)

            save_analyzed_vehicle_copy(analysis, output)
            before = load_vehicle(source)
            after = load_vehicle(output)
            self.assertEqual(before.bodies[0].components, after.bodies[0].components)
            source_bytes = source.read_bytes()
            output_bytes = output.read_bytes()
            body_marker = b'<body unique_id="unsupported">'
            source_start = source_bytes.index(body_marker)
            output_start = output_bytes.index(body_marker)
            source_end = source_bytes.index(b"</body>", source_start) + len(b"</body>")
            output_end = output_bytes.index(b"</body>", output_start) + len(b"</body>")
            self.assertEqual(
                source_bytes[source_start:source_end],
                output_bytes[output_start:output_end],
            )
            self.assertNotEqual(
                tuple(component.position for component in before.bodies[1].components),
                tuple(component.position for component in after.bodies[1].components),
            )

    def test_xml_edited_microprocessor_supports_native_thirty_two_by_thirty_two(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            source = temporary / "microprocessor_32.xml"
            source.write_text(
                '''<vehicle data_version="3"><bodies><body><components>
<c d="microprocessor"><o>
  <microprocessor_definition width="32" length="32"/>
  <vp/>
</o></c>
</components></body></bodies></vehicle>''',
                encoding="utf-8",
            )

            analysis = analyze_vehicle(source, FIXTURES / "definitions")

            self.assertTrue(analysis.can_optimize)
            self.assertEqual(1_024, analysis.bodies[0].physics_voxel_count)
            self.assertEqual(1, analysis.current_shape_count)
            self.assertEqual(1, analysis.optimized_shape_count)

            source.write_text(
                source.read_text(encoding="utf-8").replace(
                    'width="32"', 'width="33"'
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "native 32x32 limit"):
                load_vehicle(source)

    def test_non_grid_non_cube_is_excluded_even_when_clip_anchor_is_integral(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            source = Path(temporary_directory) / "integral_anchor_edit.xml"
            source.write_text(
                '''<?xml version="1.0" encoding="UTF-8"?>
<vehicle data_version="3"><authors/><bodies><body unique_id="27"><components>
  <c d="02_wedge"><o r="-3,0,0,0,1,0,-2,0,-9"><vp/></o></c>
</components></body></bodies></vehicle>
''',
                encoding="utf-8",
            )

            analysis = analyze_vehicle(source, FIXTURES / "definitions")

            self.assertTrue(analysis.can_optimize)
            self.assertEqual(0, analysis.protected_body_count)
            self.assertEqual(1, analysis.partially_optimized_body_count)
            self.assertEqual(1, analysis.xml_edited_component_count)
            self.assertFalse(analysis.bodies[0].protected_body)
            self.assertGreater(analysis.bodies[0].evaluated_order_count, 0)
            self.assertEqual((0,), analysis.bodies[0].optimization_result.optimized_component_order)

    def test_non_grid_non_cube_rotation_from_definition_excludes_component(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            definitions = temporary / "definitions"
            shutil.copytree(FIXTURES / "definitions", definitions)
            (definitions / "custom_non_grid.xml").write_text(
                '''<?xml version="1.0" encoding="UTF-8"?>
<definition name="Custom Non Grid"><voxels>
  <voxel flags="1" physics_shape="1"><position/>
    <physics_shape_rotation 00="-3" 01="0" 02="0" 10="0" 11="1" 12="0" 20="-2" 21="0" 22="-9"/>
  </voxel>
</voxels></definition>
''',
                encoding="utf-8",
            )
            source = temporary / "custom_definition_rotation.xml"
            source.write_text(
                '''<?xml version="1.0" encoding="UTF-8"?>
<vehicle data_version="3"><authors/><bodies><body unique_id="28"><components>
  <c d="custom_non_grid"><o><vp/></o></c>
</components></body></bodies></vehicle>
''',
                encoding="utf-8",
            )

            analysis = analyze_vehicle(source, definitions)

            self.assertEqual(0, analysis.protected_body_count)
            self.assertEqual(1, analysis.partially_optimized_body_count)
            self.assertEqual(1, analysis.xml_edited_component_count)
            self.assertFalse(analysis.bodies[0].protected_body)

    def test_overlapping_physics_positions_are_previewed_and_pinned(self):
        analysis = analyze_vehicle(
            FIXTURES / "vehicles" / "overlapping_cube_components.xml",
            FIXTURES / "definitions",
        )
        self.assertTrue(analysis.can_optimize)
        self.assertEqual(2, analysis.current_shape_count)
        self.assertEqual(2, len(analysis.bodies[0].current_meshes))
        self.assertEqual(1, analysis.bodies[0].overlapping_cube_count)
        self.assertEqual(1, len(analysis.bodies[0].overlap_details))
        self.assertIn("Component 0", analysis.bodies[0].overlap_details[0])
        self.assertIn("Componentを元の順序位置へ固定", analysis.bodies[0].reason)

    def test_unsupported_body_keeps_preview_without_unassigned_order_result(self):
        analysis = analyze_vehicle(
            FIXTURES / "vehicles" / "order_b.xml",
            FIXTURES / "definitions",
            max_blocks_per_body=1,
        )

        self.assertFalse(analysis.can_optimize)
        self.assertEqual(4, analysis.current_shape_count)
        self.assertGreater(len(analysis.bodies[0].current_meshes), 0)
        self.assertEqual(0, analysis.bodies[0].evaluated_order_count)
        self.assertEqual(0, analysis.bodies[0].completed_search_stage_count)
        self.assertEqual(0, analysis.bodies[0].worker_count)
        self.assertIn("設定上限", analysis.bodies[0].reason)
        self.assertTrue(any("Body 0" in warning for warning in analysis.warnings))

    def test_optimization_output_is_reloaded_and_hashed(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "optimized.xml"
            progress = []
            result = optimize_vehicle_copy(
                FIXTURES / "vehicles" / "order_b.xml",
                output,
                FIXTURES / "definitions",
                search_mode="deep",
                worker_count=1,
                progress_callback=lambda percent, message: progress.append(
                    (percent, message)
                ),
            )
            self.assertEqual(3, result.verified_analysis.current_shape_count)
            self.assertEqual(64, len(result.sha256))
            self.assertEqual(
                (FIXTURES / "vehicles" / "order_b.xml").stat().st_size,
                output.stat().st_size,
            )
            self.assertEqual(0, progress[0][0])
            self.assertEqual(100, progress[-1][0])
            self.assertEqual(
                sorted(percent for percent, _message in progress),
                [percent for percent, _message in progress],
            )

    def test_cached_analysis_is_saved_without_running_optimizer_or_analysis_again(self):
        analysis = analyze_vehicle(
            FIXTURES / "vehicles" / "order_b.xml",
            FIXTURES / "definitions",
            search_mode="deep",
            worker_count=1,
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "optimized.xml"
            progress = []
            with patch(
                "swphysics.app_service.optimize_vehicle_portable_exact",
                side_effect=AssertionError("optimizer must not run while saving"),
            ), patch(
                "swphysics.app_service.analyze_vehicle",
                side_effect=AssertionError("analysis must not run while saving"),
            ):
                result = save_analyzed_vehicle_copy(
                    analysis,
                    output,
                    progress_callback=lambda percent, message: progress.append(
                        (percent, message)
                    ),
                )

            self.assertTrue(output.is_file())
            self.assertIs(analysis, result.verified_analysis)
            self.assertEqual("portable_exact_cached_analysis", result.report.backend)
            self.assertEqual(3, result.report.after_shape_count)
            self.assertEqual(64, len(result.sha256))
            self.assertEqual(0, progress[0][0])
            self.assertEqual(100, progress[-1][0])
            self.assertTrue(any("再解析なし" in message for _value, message in progress))
            self.assertFalse(any("探索" in message for _value, message in progress))
            source_order = tuple(
                (component.definition_id, component.position)
                for component in load_vehicle(analysis.vehicle_path).bodies[0].components
            )
            output_order = tuple(
                (component.definition_id, component.position)
                for component in load_vehicle(output).bodies[0].components
            )
            self.assertNotEqual(source_order, output_order)

    def test_cached_save_preserves_compact_source_size_and_formatting(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            source = temporary / "source.xml"
            output = temporary / "optimized.xml"
            source_bytes = (FIXTURES / "vehicles" / "order_b.xml").read_bytes()
            source.write_bytes(source_bytes)

            analysis = analyze_vehicle(
                source,
                FIXTURES / "definitions",
                worker_count=1,
            )
            save_analyzed_vehicle_copy(analysis, output)
            output_bytes = output.read_bytes()

            self.assertEqual(len(source_bytes), len(output_bytes))
            self.assertEqual(source_bytes.count(b"\n"), output_bytes.count(b"\n"))
            self.assertEqual(source_bytes.count(b"\r"), output_bytes.count(b"\r"))
            self.assertEqual(source_bytes.startswith(b"\xef\xbb\xbf"), output_bytes.startswith(b"\xef\xbb\xbf"))
            self.assertNotEqual(source_bytes, output_bytes)
            self.assertEqual(
                sorted(
                    (component.definition_id, component.position)
                    for component in load_vehicle(source).bodies[0].components
                ),
                sorted(
                    (component.definition_id, component.position)
                    for component in load_vehicle(output).bodies[0].components
                ),
            )

    def test_cached_save_rejects_source_changed_after_analysis(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            source = Path(temporary_directory) / "source.xml"
            output = Path(temporary_directory) / "optimized.xml"
            shutil.copyfile(FIXTURES / "vehicles" / "order_b.xml", source)
            analysis = analyze_vehicle(source, FIXTURES / "definitions")
            source.write_bytes(source.read_bytes() + b"\n")

            with self.assertRaisesRegex(ValueError, "変更されました"):
                save_analyzed_vehicle_copy(analysis, output)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
