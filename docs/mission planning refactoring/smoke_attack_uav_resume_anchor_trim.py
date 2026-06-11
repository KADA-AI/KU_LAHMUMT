from __future__ import annotations

import os
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def configure_import_paths(project_root: Path = PROJECT_ROOT) -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("KU_ROLE", "mission")
    desired = (
        project_root,
        project_root / "modules",
        project_root / "modules" / "mission_planning",
        project_root / "modules" / "mission_planning" / "MissionPlanner",
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
    from modules.mission_planning.replanning.triggers.attack import pipeline as attack

    waypoints = [
        {
            "waypointID": 101,
            "coordinate": {"latitude": 38.0, "longitude": 127.0, "altitude": 1000},
            "isDone": True,
        },
        {
            "waypointID": 102,
            "coordinate": {"latitude": 38.001, "longitude": 127.0, "altitude": 1000},
            "isDone": False,
        },
        {
            "waypointID": 103,
            "coordinate": {"latitude": 38.002, "longitude": 127.0, "altitude": 1000},
            "isDone": False,
        },
    ]
    artifacts = SimpleNamespace(
        current_waypoint_id=102,
        previous_waypoint_id=101,
        path_id=400000001,
    )
    next_id = 900001

    def next_waypoint_id() -> int:
        nonlocal next_id
        next_id += 1
        return next_id

    logs: list[str] = []
    timing: dict[str, Any] = {}
    done_waypoints, resume_waypoints, removed_wp_id = attack._split_done_resume_path(
        {"aircraftID": 4, "pathID": 400000001, "waypointList": waypoints},
        artifacts=artifacts,
        sweep_progress={},
        emit=logs.append,
        force_nonempty_resume=True,
        append_replan_anchor=False,
        replan_coordinate=None,
        resume_trim_anchor_coord={"latitude": 38.002, "longitude": 127.0, "altitude": 1000},
        waypoint_id_provider=next_waypoint_id,
        timing=timing,
    )
    if not done_waypoints or not resume_waypoints:
        raise RuntimeError("attack UAV resume split produced an empty done/resume branch")
    if removed_wp_id is None:
        raise RuntimeError("attack UAV resume split did not report a removed waypoint")
    if "trim_waypoints_before_replan_anchor" not in timing:
        raise RuntimeError("attack UAV resume split did not record anchor trim timing")
    if any(bool(wp.get("isDone")) for wp in resume_waypoints if isinstance(wp, dict)):
        raise RuntimeError("attack UAV resume split left a resume waypoint marked done")

    print("attack UAV resume anchor trim smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
