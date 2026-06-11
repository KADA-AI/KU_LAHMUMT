from __future__ import annotations

import os
import sys
from pathlib import Path

from shapely.geometry import LineString, Point, Polygon


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def configure_import_paths(project_root: Path = PROJECT_ROOT) -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("KU_ROLE", "monitoring")
    desired = (
        project_root,
        project_root / "modules",
    )
    for path in reversed(desired):
        path_str = str(path)
        if not path.exists():
            continue
        while path_str in sys.path:
            sys.path.remove(path_str)
        sys.path.insert(0, path_str)


def main() -> int:
    configure_import_paths()
    from modules.monitoring.logic.mission_area_progress_monitor import (
        _MissionAreaState,
        _commit_observed_planned_line,
        _rebuild_completed_sweep_coverage,
        _remaining_geometry_for_state,
    )

    assignment = Polygon([(0, 0), (100, 0), (100, 40), (0, 40)])
    planned_lines = [
        LineString([(x, 0), (x, 40)])
        for x in (0, 25, 50, 75, 100)
    ]
    state = _MissionAreaState(
        mission_id=1,
        aircraft_id=4,
        input_id=7,
        mission_type="area",
        source_plan_id=700001,
        path_id=400001,
        coverage_def=None,
        width_hint_m=20.0,
        assignment_geometry=assignment,
        planned_area_m2=float(assignment.area),
        planned_cut_lines=planned_lines,
        cut_half_width_m=3.0,
    )

    if not _commit_observed_planned_line(
        state,
        tracked_index=4,
        center_point=Point(100, 20),
        distance_limit_m=20.0,
    ):
        raise RuntimeError("initial reverse-side area row commit failed")
    _rebuild_completed_sweep_coverage(state)
    first_remaining = _remaining_geometry_for_state(state)
    if state.progress_direction_sign is not None:
        raise RuntimeError("area direction was fixed from a single row sample")
    if first_remaining.is_empty or float(first_remaining.area) < float(assignment.area) * 0.65:
        raise RuntimeError("single area row sample clipped a whole side before direction was known")

    if not _commit_observed_planned_line(
        state,
        tracked_index=3,
        center_point=Point(75, 20),
        distance_limit_m=20.0,
    ):
        raise RuntimeError("second reverse-side area row commit failed")
    _rebuild_completed_sweep_coverage(state)
    remaining = _remaining_geometry_for_state(state)
    if int(state.progress_direction_sign or 0) != -1:
        raise RuntimeError(f"reverse area row direction not learned: {state.progress_direction_sign}")
    if remaining.is_empty:
        raise RuntimeError("reverse area row progress removed all remaining geometry")
    if float(remaining.bounds[2]) > 78.5:
        raise RuntimeError(f"reverse area row progress left the wrong side: bounds={remaining.bounds}")

    print("area reverse row progress smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
