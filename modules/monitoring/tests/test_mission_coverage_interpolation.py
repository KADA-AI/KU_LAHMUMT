# -*- coding: utf-8 -*-
from __future__ import annotations

import unittest

from shapely.affinity import rotate, translate
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

from modules.monitoring.logic.area_search_interpolation import (
    build_frame_interpolated_footprint_geometry,
    build_path_frame_interpolated_footprint_geometry,
    build_sweep_endpoint_fill_geometry,
    resolve_frame_sample_fractions,
)
from modules.monitoring.logic.coverage_settings import (
    DEFAULT_COVERAGE_SETTINGS,
    resolve_footprint_interpolation_steps,
)
from modules.monitoring.logic.mission_coverage import (
    MissionCoverageDefinition,
    build_interpolated_footprint_geometry,
    build_path_interpolated_footprint_geometry,
    build_projected_sweep_path,
    project_coordinate_to_sweep_path,
)
from modules.monitoring.logic.mission_progress import (
    MissionCoverageState,
    MissionMeta,
    MissionProgressTracker,
)


class _IdentityTransformer:
    @staticmethod
    def transform(longitude: float, latitude: float) -> tuple[float, float]:
        return longitude, latitude


class _TenXTransformer:
    @staticmethod
    def transform(longitude: float, latitude: float) -> tuple[float, float]:
        return longitude * 10.0, latitude * 10.0


def _footprint(left: float, bottom: float, right: float, top: float) -> list[dict[str, float]]:
    return [
        {"longitude": left, "latitude": top},
        {"longitude": right, "latitude": top},
        {"longitude": right, "latitude": bottom},
        {"longitude": left, "latitude": bottom},
    ]


class FootprintInterpolationTests(unittest.TestCase):
    def test_imported_frame_grid_keeps_exact_thirty_hz_timestamps(self) -> None:
        fractions = resolve_frame_sample_fractions(
            10_000,
            10_208,
            dict(DEFAULT_COVERAGE_SETTINGS),
        )

        self.assertIsNotNone(fractions)
        assert fractions is not None
        self.assertEqual(len(fractions), 8)
        sample_offsets_ms = [round(fraction * 208.0, 3) for fraction in fractions]
        self.assertEqual(
            sample_offsets_ms,
            [0.0, 33.333, 66.667, 100.0, 133.333, 166.667, 200.0, 208.0],
        )

    def test_imported_frame_union_does_not_continuously_sweep_between_frames(self) -> None:
        previous = box(-2.0, -0.5, 2.0, 0.5)
        current = rotate(previous, 60.0, origin=(0.0, 0.0))
        fractions = resolve_frame_sample_fractions(
            10_000,
            10_200,
            dict(DEFAULT_COVERAGE_SETTINGS),
        )

        sampled = build_frame_interpolated_footprint_geometry(
            previous,
            current,
            fractions,
        )
        continuous_edge_sweep = build_interpolated_footprint_geometry(
            previous,
            current,
            6,
        )

        self.assertLess(sampled.area, continuous_edge_sweep.area)
        self.assertGreater(sampled.area, previous.area)

    def test_imported_path_interpolation_adds_only_frame_and_turn_samples(self) -> None:
        previous = box(-5.0, -2.0, 5.0, 2.0)
        current = translate(previous, yoff=80.0)
        sweep_path = LineString(
            [(0.0, 0.0), (100.0, 0.0), (100.0, 80.0), (0.0, 80.0)]
        )
        fractions = resolve_frame_sample_fractions(
            10_000,
            10_200,
            dict(DEFAULT_COVERAGE_SETTINGS),
        )

        sampled = build_path_frame_interpolated_footprint_geometry(
            previous,
            current,
            sweep_path,
            0.0,
            sweep_path.length,
            fractions,
        )

        self.assertTrue(sampled.covers(Point(100.0, 0.0)))
        self.assertTrue(sampled.covers(Point(100.0, 80.0)))
        self.assertFalse(sampled.covers(Point(50.0, 40.0)))

    def test_imported_sweep_endpoint_fill_reaches_only_after_completion(self) -> None:
        source = box(-2.0, -1.0, 2.0, 1.0)
        assignment = box(-3.0, -2.0, 23.0, 2.0)
        sweep_path = LineString([(0.0, 0.0), (20.0, 0.0)])

        endpoint_fill = build_sweep_endpoint_fill_geometry(
            source,
            sweep_path,
            10.0,
            spacing_fraction=0.5,
            minimum_spacing_m=1.0,
            max_samples=32,
            assignment_geometry=assignment,
        )

        self.assertTrue(endpoint_fill.covers(Point(20.0, 0.0)))
        self.assertFalse(endpoint_fill.covers(Point(0.0, 0.0)))
        self.assertTrue(assignment.covers(endpoint_fill))

    def test_five_hz_telemetry_uses_six_intervals_at_thirty_hz(self) -> None:
        steps = resolve_footprint_interpolation_steps(
            10_000,
            10_200,
            dict(DEFAULT_COVERAGE_SETTINGS),
        )
        self.assertEqual(steps, 6)

    def test_telemetry_gap_breaks_interpolation_continuity(self) -> None:
        steps = resolve_footprint_interpolation_steps(
            10_000,
            11_001,
            dict(DEFAULT_COVERAGE_SETTINGS),
        )
        self.assertEqual(steps, 0)

    def test_interpolation_sweeps_corner_edges_without_convex_hull_overfill(self) -> None:
        previous = box(-2.0, -0.5, 2.0, 0.5)
        current = rotate(previous, 60.0, origin=(0.0, 0.0))
        sampled = build_interpolated_footprint_geometry(previous, current, 6)
        previous_vertices = list(previous.exterior.coords)[:-1]
        current_vertices = list(current.exterior.coords)[:-1]
        polygon_samples = []
        for step in range(7):
            fraction = step / 6
            polygon_samples.append(
                Polygon(
                    [
                        (
                            prev_x + ((curr_x - prev_x) * fraction),
                            prev_y + ((curr_y - prev_y) * fraction),
                        )
                        for (prev_x, prev_y), (curr_x, curr_y) in zip(
                            previous_vertices,
                            current_vertices,
                        )
                    ]
                )
            )
        polygon_samples_only = unary_union(polygon_samples)
        convex_bridge = unary_union([previous, current]).convex_hull

        self.assertGreater(sampled.area, polygon_samples_only.area + 0.1)
        self.assertLess(sampled.area, convex_bridge.area)

    def test_assignment_boundary_clips_every_interpolated_footprint(self) -> None:
        assignment = box(0.0, 0.0, 10.0, 10.0)
        previous = box(-2.0, 4.0, 2.0, 6.0)
        current = box(1.0, 4.0, 5.0, 6.0)
        sampled = build_interpolated_footprint_geometry(previous, current, 6)
        covered = assignment.intersection(sampled)

        self.assertAlmostEqual(covered.area, 10.0, places=6)
        self.assertTrue(assignment.covers(covered))

    def test_interpolation_clips_corner_and_end_sweeps_to_assignment(self) -> None:
        assignment = box(0.0, 0.0, 10.0, 10.0)
        previous = box(-4.0, 3.0, 2.0, 5.0)
        current = translate(
            rotate(box(-3.0, -1.0, 3.0, 1.0), 55.0, origin=(0.0, 0.0)),
            xoff=7.0,
            yoff=6.0,
        )

        swept = build_interpolated_footprint_geometry(
            previous,
            current,
            6,
            assignment_geometry=assignment,
        )

        self.assertFalse(swept.is_empty)
        self.assertTrue(assignment.covers(swept))
        self.assertAlmostEqual(swept.difference(assignment).area, 0.0, places=6)

    def test_large_telemetry_jump_does_not_bridge_unobserved_ground(self) -> None:
        previous = box(-2.0, -1.0, 2.0, 1.0)
        current = translate(previous, xoff=100.0)

        swept = build_interpolated_footprint_geometry(previous, current, 30)

        self.assertTrue(swept.equals(current))
        self.assertFalse(swept.intersects(previous))

    def test_hairpin_sweep_fills_local_corners_without_filling_u_interior(self) -> None:
        base = box(-3.0, -1.0, 3.0, 1.0)
        footprints = [
            translate(base, xoff=0.0, yoff=0.0),
            translate(base, xoff=6.0, yoff=0.0),
            translate(rotate(base, 45.0, origin=(0.0, 0.0)), xoff=8.0, yoff=2.0),
            translate(rotate(base, 90.0, origin=(0.0, 0.0)), xoff=8.0, yoff=5.0),
            translate(rotate(base, 135.0, origin=(0.0, 0.0)), xoff=6.0, yoff=8.0),
            translate(rotate(base, 180.0, origin=(0.0, 0.0)), xoff=0.0, yoff=8.0),
        ]
        segments = [
            build_interpolated_footprint_geometry(previous, current, 3)
            for previous, current in zip(footprints, footprints[1:])
        ]
        swept = unary_union(segments)

        # Every row-end segment joins the next one, including its leading and
        # trailing corners, so the accumulated coverage remains one component.
        self.assertEqual(swept.geom_type, "Polygon")
        self.assertEqual(len(swept.interiors), 0)
        # The local edge sweeps must not shortcut across the unobserved middle
        # of the U-shaped flight path.
        self.assertFalse(swept.covers(Point(4.0, 4.0)))
        self.assertLess(swept.area, swept.convex_hull.area * 0.75)

    def test_implausible_footprint_size_jump_breaks_interpolation(self) -> None:
        previous = box(-1.0, -1.0, 1.0, 1.0)
        current = box(-5.0, -5.0, 5.0, 5.0)

        swept = build_interpolated_footprint_geometry(previous, current, 6)

        self.assertTrue(swept.equals(current))

    def test_planned_path_bridges_high_speed_gap_rejected_by_direct_interpolation(self) -> None:
        previous = box(-5.0, -2.0, 5.0, 2.0)
        current = translate(previous, xoff=400.0)
        sweep_path = LineString([(0.0, 0.0), (200.0, 0.0), (400.0, 0.0)])

        direct = build_interpolated_footprint_geometry(previous, current, 6)
        routed = build_path_interpolated_footprint_geometry(
            previous,
            current,
            sweep_path,
            0.0,
            400.0,
            6,
        )

        self.assertTrue(direct.equals(current))
        self.assertTrue(routed.covers(Point(200.0, 0.0)))
        self.assertGreater(routed.area, direct.area * 20.0)

    def test_planned_hairpin_follows_connector_without_diagonal_shortcut(self) -> None:
        previous = box(-5.0, -2.0, 5.0, 2.0)
        current = translate(previous, yoff=100.0)
        sweep_path = LineString(
            [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]
        )

        routed = build_path_interpolated_footprint_geometry(
            previous,
            current,
            sweep_path,
            0.0,
            sweep_path.length,
            6,
        )

        self.assertTrue(routed.covers(Point(50.0, 0.0)))
        self.assertTrue(routed.covers(Point(100.0, 50.0)))
        self.assertTrue(routed.covers(Point(50.0, 100.0)))
        self.assertFalse(routed.covers(Point(50.0, 50.0)))
        self.assertLess(routed.area, routed.convex_hull.area * 0.35)

    def test_local_chainage_projection_does_not_jump_to_older_parallel_row(self) -> None:
        transformer = _IdentityTransformer()
        sweep_path = build_projected_sweep_path(
            [
                {"longitude": 0.0, "latitude": 0.0},
                {"longitude": 100.0, "latitude": 0.0},
                {"longitude": 100.0, "latitude": 4.0},
                {"longitude": 0.0, "latitude": 4.0},
            ],
            transformer,
        )
        coordinate = {"longitude": 80.0, "latitude": 2.0}

        global_chainage, _global_offset = project_coordinate_to_sweep_path(
            sweep_path,
            coordinate,
            transformer,
        )
        local_chainage, local_offset = project_coordinate_to_sweep_path(
            sweep_path,
            coordinate,
            transformer,
            previous_chainage_m=110.0,
            search_window_m=100.0,
        )

        self.assertAlmostEqual(global_chainage or 0.0, 80.0, places=6)
        self.assertAlmostEqual(local_chainage or 0.0, 124.0, places=6)
        self.assertAlmostEqual(local_offset or 0.0, 2.0, places=6)

    def test_tracker_uses_planned_connector_for_sparse_high_speed_frames(self) -> None:
        tracker = MissionProgressTracker()
        assignment = box(-10.0, -10.0, 110.0, 110.0)
        tracker._mission_coverage_defs = {
            101: MissionCoverageDefinition(
                assignment.area,
                assignment,
                _IdentityTransformer(),
            )
        }
        tracker._mission_meta = {
            101: MissionMeta(
                mission_id=101,
                aircraft_id=4,
                input_id=9,
                package_id=8,
                path_id=7,
                planned_seconds=10.0,
                waypoint_ids=[1],
                waypoint_eta_cumulative={1: 10.0},
                waypoint_index={1: 0},
            )
        }
        tracker._mission_sweep_paths = {
            (101, 1): LineString(
                [(0.0, 0.0), (100.0, 0.0), (100.0, 80.0), (0.0, 80.0)]
            )
        }
        state = {
            "current_waypoint_id": 1,
            "filming": 1,
            "flying": 1,
            "sensor_operation_mode": 2,
            "sensor_center_coordinate": {"longitude": 0.0, "latitude": 0.0},
            "footprint_corners": _footprint(-5.0, -2.0, 5.0, 2.0),
        }
        tracker._update_mission_coverage(101, state, timestamp_ms=10_000)
        state["sensor_center_coordinate"] = {"longitude": 0.0, "latitude": 80.0}
        state["footprint_corners"] = _footprint(-5.0, 78.0, 5.0, 82.0)
        tracker._update_mission_coverage(101, state, timestamp_ms=10_200)

        covered = tracker._mission_coverage_state[101].covered_geometry
        self.assertIsNotNone(covered)
        self.assertTrue(covered.covers(Point(100.0, 40.0)))
        self.assertFalse(covered.covers(Point(50.0, 40.0)))

    def test_tracker_keeps_fast_area_scan_locked_to_forward_chainage(self) -> None:
        tracker = MissionProgressTracker()
        assignment = box(-10.0, -10.0, 560.0, 10.0)
        tracker._mission_coverage_defs = {
            101: MissionCoverageDefinition(
                assignment.area,
                assignment,
                _TenXTransformer(),
            )
        }
        tracker._mission_meta = {
            101: MissionMeta(
                mission_id=101,
                aircraft_id=5,
                input_id=9,
                package_id=8,
                path_id=7,
                planned_seconds=10.0,
                waypoint_ids=[1],
                waypoint_eta_cumulative={1: 10.0},
                waypoint_index={1: 0},
            )
        }
        tracker._mission_sweep_paths = {
            (101, 1): LineString([(0.0, 0.0), (550.0, 0.0)])
        }
        state = {
            "current_waypoint_id": 1,
            "filming": 1,
            "flying": 1,
            "sensor_operation_mode": 2,
            "sensor_center_coordinate": {"longitude": 0.0, "latitude": 0.0},
            "footprint_corners": _footprint(-0.5, -0.2, 0.5, 0.2),
        }
        tracker._update_mission_coverage(101, state, timestamp_ms=10_000)
        state["sensor_center_coordinate"] = {"longitude": 55.0, "latitude": 0.0}
        state["footprint_corners"] = _footprint(54.5, -0.2, 55.5, 0.2)
        tracker._update_mission_coverage(101, state, timestamp_ms=10_200)

        covered = tracker._mission_coverage_state[101].covered_geometry
        self.assertIsNotNone(covered)
        assert covered is not None
        self.assertTrue(covered.covers(Point(275.0, 0.0)))

    def test_tracker_fills_remaining_sweep_endpoint_only_on_filming_done(self) -> None:
        tracker = MissionProgressTracker()
        assignment = box(-3.0, -2.0, 23.0, 2.0)
        tracker._mission_coverage_defs = {
            101: MissionCoverageDefinition(
                assignment.area,
                assignment,
                _IdentityTransformer(),
            )
        }
        tracker._mission_meta = {
            101: MissionMeta(
                mission_id=101,
                aircraft_id=4,
                input_id=9,
                package_id=8,
                path_id=7,
                planned_seconds=10.0,
                waypoint_ids=[1],
                waypoint_eta_cumulative={1: 10.0},
                waypoint_index={1: 0},
            )
        }
        tracker._mission_sweep_paths = {
            (101, 1): LineString([(0.0, 0.0), (20.0, 0.0)])
        }
        state = {
            "current_waypoint_id": 1,
            "filming": 1,
            "flying": 1,
            "sensor_operation_mode": 2,
            "sensor_center_coordinate": {"longitude": 10.0, "latitude": 0.0},
            "footprint_corners": _footprint(8.0, -1.0, 12.0, 1.0),
        }
        tracker._update_mission_coverage(101, state, timestamp_ms=10_000)

        before_completion = tracker._mission_coverage_state[101].covered_geometry
        self.assertIsNotNone(before_completion)
        assert before_completion is not None
        self.assertFalse(before_completion.covers(Point(20.0, 0.0)))

        state["filming"] = 2
        tracker._update_mission_coverage(101, state, timestamp_ms=10_200)

        after_completion = tracker._mission_coverage_state[101].covered_geometry
        self.assertIsNotNone(after_completion)
        assert after_completion is not None
        self.assertTrue(after_completion.covers(Point(20.0, 0.0)))

    def test_tracker_accumulates_only_the_selected_individual_mission(self) -> None:
        tracker = MissionProgressTracker()
        definition = MissionCoverageDefinition(
            planned_area_m2=100.0,
            assignment_geometry=box(0.0, 0.0, 10.0, 10.0),
            transformer=_IdentityTransformer(),
        )
        tracker._mission_coverage_defs = {101: definition, 202: definition}
        capture_state = {
            "filming": 1,
            "flying": 1,
            "sensor_operation_mode": 2,
            "footprint_corners": _footprint(-2.0, 4.0, 2.0, 6.0),
        }
        tracker._update_mission_coverage(101, capture_state, timestamp_ms=10_000)
        capture_state["footprint_corners"] = _footprint(1.0, 4.0, 5.0, 6.0)
        tracker._update_mission_coverage(101, capture_state, timestamp_ms=10_200)

        self.assertAlmostEqual(
            tracker._mission_coverage_state[101].covered_area_m2,
            10.0,
            places=6,
        )
        self.assertNotIn(202, tracker._mission_coverage_state)

    def test_input_coverage_unions_overlapping_aircraft_assignments(self) -> None:
        tracker = MissionProgressTracker()
        tracker._mission_coverage_defs = {
            101: MissionCoverageDefinition(100.0, box(0.0, 0.0, 10.0, 10.0), _IdentityTransformer()),
            202: MissionCoverageDefinition(100.0, box(5.0, 0.0, 15.0, 10.0), _IdentityTransformer()),
        }
        shared_capture = box(5.0, 0.0, 10.0, 10.0)
        tracker._mission_coverage_state = {
            101: MissionCoverageState(covered_geometry=shared_capture, covered_area_m2=50.0),
            202: MissionCoverageState(covered_geometry=shared_capture, covered_area_m2=50.0),
        }

        result = tracker._aggregate_input_footprint_geometry([101, 202])

        self.assertAlmostEqual(result["planned_area_m2"], 150.0, places=6)
        self.assertAlmostEqual(result["covered_area_m2"], 50.0, places=6)
        self.assertEqual(result["coverage_percent"], 33)

    def test_input_coverage_reaches_full_only_after_union_is_covered(self) -> None:
        tracker = MissionProgressTracker()
        first = box(0.0, 0.0, 10.0, 10.0)
        second = box(5.0, 0.0, 15.0, 10.0)
        tracker._mission_coverage_defs = {
            101: MissionCoverageDefinition(100.0, first, _IdentityTransformer()),
            202: MissionCoverageDefinition(100.0, second, _IdentityTransformer()),
        }
        tracker._mission_coverage_state = {
            101: MissionCoverageState(covered_geometry=first, covered_area_m2=100.0),
            202: MissionCoverageState(covered_geometry=second, covered_area_m2=100.0),
        }

        result = tracker._aggregate_input_footprint_geometry([101, 202])

        self.assertAlmostEqual(result["covered_area_m2"], result["planned_area_m2"], places=6)
        self.assertEqual(result["coverage_percent"], 100)


if __name__ == "__main__":
    unittest.main()
