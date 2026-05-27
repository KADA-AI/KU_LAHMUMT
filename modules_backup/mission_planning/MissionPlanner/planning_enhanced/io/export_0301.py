from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


_COUNTER_FILE = Path(__file__).resolve().parents[2] / "temp" / "id_0301_counter.json"
_MISSION_PLAN_START = 700_000_001


def _now_ms_since_2000() -> int:
    epoch_2000 = 946684800.0
    return int((time.time() - epoch_2000) * 1000.0)


def _sw_code(default: str = "MMR") -> str:
    role = (os.environ.get("KU_ROLE") or "").lower()
    return {"mission": "MMR", "monitoring": "MSM", "decision": "MOB"}.get(role, default)


def _to_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _to_uint32(value: Any, default: int = 0) -> int:
    try:
        return int(value) & 0xFFFFFFFF
    except Exception:
        return int(default) & 0xFFFFFFFF


def _next_mission_plan_id() -> int:
    _COUNTER_FILE.parent.mkdir(parents=True, exist_ok=True)
    last = _MISSION_PLAN_START - 1
    if _COUNTER_FILE.exists():
        try:
            obj = json.loads(_COUNTER_FILE.read_text(encoding="utf-8"))
            if isinstance(obj, dict):
                last = _to_int(obj.get("missionPlanID"), last)
        except Exception:
            pass
    nxt = int(last) + 1
    _COUNTER_FILE.write_text(json.dumps({"missionPlanID": int(nxt)}, ensure_ascii=False), encoding="utf-8")
    return int(nxt)


def _validate_0301(plan: Dict[str, Any]) -> None:
    for k in (
        "timestamp",
        "Source",
        "missionPlanID",
        "missionPlanTimestamp",
        "planningTime",
        "plannerID",
        "inputMissionPackageID",
        "missionReferencePackageID",
        "aircraftList",
    ):
        if k not in plan:
            raise ValueError(f"[0301] missing '{k}'")

    al = plan.get("aircraftList")
    if not isinstance(al, list) or not al:
        raise ValueError("[0301] aircraftList must be non-empty")

    seen_aid: set[int] = set()
    for row in al:
        if not isinstance(row, dict):
            raise ValueError("[0301] aircraftList entry must be object")
        aid = _to_int(row.get("aircraftID"), 0)
        imp = _to_int(row.get("individualMissionPackageID"), 0)
        if aid < 1 or aid > 6:
            raise ValueError(f"[0301] invalid aircraftID: {aid}")
        if aid in seen_aid:
            raise ValueError(f"[0301] duplicate aircraftID: {aid}")
        seen_aid.add(aid)
        if imp <= 0:
            raise ValueError(f"[0301] invalid individualMissionPackageID: {imp}")


def build_0301_from_0302_packages(
    packages: List[Dict[str, Any]],
    *,
    cmpk: Dict[str, Any],
    mrpk: Dict[str, Any],
    mission_plan_id: Optional[int] = None,
    planner_id: int = 1,
    planning_time_ms: float = 0.0,
    source: Optional[str] = None,
) -> Dict[str, Any]:
    if not isinstance(packages, list) or not packages:
        raise ValueError("[0301] 0302 package list is empty.")

    rows: List[Dict[str, int]] = []
    seen: set[int] = set()
    for pkg in sorted(packages, key=lambda p: _to_int(p.get("aircraftID"), 0)):
        if not isinstance(pkg, dict):
            continue
        aid = _to_int(pkg.get("aircraftID"), 0)
        imp = _to_int(pkg.get("individualMissionPackageID"), 0)
        if aid <= 0 or imp <= 0:
            continue
        if aid in seen:
            continue
        seen.add(aid)
        rows.append({"aircraftID": int(aid), "individualMissionPackageID": int(imp)})
    if not rows:
        raise ValueError("[0301] no valid aircraft rows from 0302 packages.")

    ts = _now_ms_since_2000()
    plan_id = int(mission_plan_id) if mission_plan_id is not None else _next_mission_plan_id()
    in_pkg_id = _to_uint32(cmpk.get("inputMissionPackageID"), 0)
    ref_pkg_id = _to_uint32(
        mrpk.get("missionReferencePackageID", mrpk.get("inputMissionPackageID", 0)),
        0,
    )
    mp = {
        "timestamp": int(ts),
        "Source": source or _sw_code(),
        "missionPlanID": int(plan_id),
        "missionPlanTimestamp": int(ts),
        "planningTime": float(planning_time_ms),
        "plannerID": int(planner_id),
        "inputMissionPackageID": int(in_pkg_id),
        "missionReferencePackageID": int(ref_pkg_id),
        "aircraftList": rows,
    }
    _validate_0301(mp)
    return mp


def save_0301_plan(plan: Dict[str, Any], out_dir: str | Path) -> Path:
    root = Path(out_dir)
    root.mkdir(parents=True, exist_ok=True)
    mid = _to_int(plan.get("missionPlanID"), 0)
    if mid <= 0:
        raise ValueError("[0301] missionPlanID is invalid")
    path = root / f"MissionPlan_{mid}.json"
    path.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")
    return path

