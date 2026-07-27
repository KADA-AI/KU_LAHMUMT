from __future__ import annotations

from types import SimpleNamespace

import pytest
from shapely.geometry import GeometryCollection, LineString, box

from modules.monitoring.logic.mission_area_progress_monitor import (
    _mark_line_state_done_from_progress,
    _trim_current_line_from_progress,
)


def test_line_trim_keeps_input_direction_when_aircraft_moves_reverse() -> None:
    state = SimpleNamespace(
        is_current=True,
        last_center_xy=(75.0, 0.0),
        cut_half_width_m=10.0,
        mission_type="line",
        progress_boundary_line_index=None,
        centerline_points=[(80.0, 0.0), (75.0, 0.0)],
        preferred_track_vector=(-1.0, 0.0),
    )
    source = LineString([(0.0, 0.0), (100.0, 0.0)])

    trimmed = _trim_current_line_from_progress(
        state,
        source,
        width_m=20.0,
        min_length_m=3.0,
    )

    assert list(trimmed.coords)[0][0] == pytest.approx(73.0)
    assert list(trimmed.coords)[-1][0] == pytest.approx(100.0)


def test_completed_line_latches_empty_remaining_geometry() -> None:
    assignment = box(0.0, 0.0, 100.0, 20.0)
    planned_lines = [
        LineString([(0.0, 5.0), (100.0, 5.0)]),
        LineString([(0.0, 15.0), (100.0, 15.0)]),
    ]
    state = SimpleNamespace(
        mission_type="line",
        planned_cut_lines=planned_lines,
        done=False,
        assignment_geometry=assignment,
        covered_geometry=GeometryCollection(),
        completed_cut_line_indexes=set(),
        cut_lines=[],
        last_cut_line_index=-1,
        progress_origin_line_index=None,
        progress_boundary_line_index=None,
        progress_direction_sign=None,
    )

    _mark_line_state_done_from_progress(state)

    assert state.done is True
    assert state.covered_geometry.equals(assignment)
    assert state.completed_cut_line_indexes == {0, 1}
    assert state.progress_boundary_line_index == 1
