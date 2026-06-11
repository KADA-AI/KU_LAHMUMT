from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path
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


def area_detail(size: float) -> dict[str, Any]:
    return {
        "coordinateList": [],
        "lineList": [],
        "areaList": [
            {
                "isHole": False,
                "coordinateList": [
                    {"latitude": 37.0, "longitude": 127.0, "altitude": 1000},
                    {"latitude": 37.0, "longitude": 127.0 + size, "altitude": 1000},
                    {"latitude": 37.0 + size, "longitude": 127.0 + size, "altitude": 1000},
                    {"latitude": 37.0 + size, "longitude": 127.0, "altitude": 1000},
                ],
            }
        ],
    }


def area_entry(plan_id: int, input_id: int, remaining_area_m2: float) -> dict[str, Any]:
    detail = area_detail(max(0.0001, remaining_area_m2 / 1_000_000_000.0))
    return {
        "missionPlanID": int(plan_id),
        "inputMissionID": int(input_id),
        "missionType": "area",
        "aircraftIDs": [4],
        "individualMissionIDs": [900001],
        "plannedAreaM2": 2000.0,
        "remainingAreaM2": float(remaining_area_m2),
        "coveragePercent": 0,
        "isDone": False,
        "remainingDetail": detail,
        "areaOwnershipDetails": [
            {
                "aircraftID": 4,
                "individualMissionID": 900001,
                "inputMissionID": int(input_id),
                "sourceMissionPlanID": int(plan_id),
                "pathID": 400001,
                "takeoverPolicy": "piece_only",
                "remainingAreaM2": float(remaining_area_m2),
                "remainingDetail": detail,
            }
        ],
        "geometryDiagnostics": {"replanInputGeometry": "single_area_polygon"},
    }


def snapshot(plan_id: int, input_id: int, remaining_area_m2: float) -> dict[str, Any]:
    return {
        "missionPlanID": int(plan_id),
        "missionCount": 1,
        "missions": [area_entry(plan_id, input_id, remaining_area_m2)],
    }


def first_remaining_area(snapshot_payload: dict[str, Any]) -> float:
    missions = snapshot_payload.get("missions")
    if not isinstance(missions, list) or not missions:
        raise RuntimeError("snapshot has no missions")
    return float((missions[0] or {}).get("remainingAreaM2") or 0.0)


def main() -> int:
    configure_import_paths()
    from modules.common import mission_area_replan_store as store

    original_detail_dir = store._detail_dir
    with tempfile.TemporaryDirectory() as tmp:
        temp_root = Path(tmp)
        store._detail_dir = lambda: temp_root
        try:
            store.save_snapshot(700001, snapshot(700001, 7, 1000.0))
            store.save_snapshot(700002, snapshot(700002, 7, 1600.0))
            grown = store.load_snapshot(700002)
            if first_remaining_area(grown) != 1000.0:
                raise RuntimeError("central ledger allowed remaining area growth")
            latest = store.load_snapshot_entry(None, 7, allow_latest=True)
            latest_area = float(((latest or {}).get("entry") or {}).get("remainingAreaM2") or 0.0)
            if latest_area != 1000.0:
                raise RuntimeError("central ledger did not override a stale latest snapshot fallback")
            ready_latest = store.load_replan_ready_snapshot_entry(None, 7, allow_latest=True)
            ready_area = float(((ready_latest or {}).get("entry") or {}).get("remainingAreaM2") or 0.0)
            if ready_area != 1000.0:
                raise RuntimeError("central ledger did not override a stale replan-ready fallback")

            store.save_snapshot(700003, snapshot(700003, 7, 700.0))
            old_exact = store.load_snapshot_entry(700002, 7, allow_latest=False)
            old_area = float(((old_exact or {}).get("entry") or {}).get("remainingAreaM2") or 0.0)
            if old_area != 700.0:
                raise RuntimeError("central ledger did not override an old exact snapshot entry")

            carried_path = store.carry_forward_snapshot(700002, 700004, reason="central_ledger_smoke")
            if carried_path is None:
                raise RuntimeError("central ledger carry-forward did not write a snapshot")
            carried = store.load_snapshot(700004)
            if first_remaining_area(carried) != 700.0:
                raise RuntimeError("central ledger carry-forward resurrected a larger area")

            reject_reason = store.snapshot_entry_replan_reject_reason(
                area_entry(700005, 7, 700.0),
                exact=True,
            )
            if reject_reason:
                raise RuntimeError(f"geometry-backed area entry was rejected: {reject_reason}")
        finally:
            store._detail_dir = original_detail_dir

    print("area central ledger smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
