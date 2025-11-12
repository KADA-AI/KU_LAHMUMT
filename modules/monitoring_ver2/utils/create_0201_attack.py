from __future__ import annotations

import argparse
import json
import math
import time
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.append(str(PROJECT_ROOT))

from modules.common import db_paths


CURRENT_SCENARIO_PATH = PROJECT_ROOT / "current_scenario.json"


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def _load_current_scenario_fallback() -> Dict[str, Any]:
    try:
        return _load_json(CURRENT_SCENARIO_PATH)
    except FileNotFoundError:
        return {}


def load_active_agency_dir(explicit_dir: Optional[str], explicit_agency: Optional[str]) -> Path:
    info = db_paths.get_info()
    scenario_dir_str = explicit_dir or info.get("scenario_dir")
    agency = explicit_agency or info.get("agency")
    db_root = info.get("db_root")
    if not scenario_dir_str and not db_root:
        fallback = _load_current_scenario_fallback()
        scenario_dir_str = fallback.get("scenario_dir")
        agency = agency or fallback.get("agency")
        db_root = db_root or fallback.get("db_root")

    agency = agency or "SBC3"
    if scenario_dir_str:
        scenario_dir = Path(scenario_dir_str).resolve()
        return scenario_dir / agency
    if db_root:
        return Path(db_root).resolve()
    return (PROJECT_ROOT / "Logs" / agency).resolve()


def select_base_input_plan(agency_dir: Path, package_id: Optional[int]) -> Tuple[Dict[str, Any], Path]:
    base_dir = agency_dir / "InputMissionPlan"
    candidates: List[Path] = []
    if package_id is not None:
        candidates.append(base_dir / f"{package_id}.json")
    else:
        candidates.extend(sorted(base_dir.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True))
    for path in candidates:
        if path.exists():
            return _load_json(path), path
    raise FileNotFoundError(f"No InputMissionPlan JSON available under {base_dir}")


def select_target_entry(agency_dir: Path, target_key: Optional[str], target_id: Optional[int], watcher_id: Optional[int]) -> Tuple[str, Dict[str, Any]]:
    target_path = agency_dir / "DSS_Internal" / "targetInfo.json"
    target_data = _load_json(target_path)
    target_list = target_data.get("targetList") or {}
    if not target_list:
        raise RuntimeError(f"No targets recorded inside {target_path}")
    if target_key and target_key in target_list:
        return target_key, target_list[target_key]
    for key, entry in target_list.items():
        tid = entry.get("targetID")
        wid = entry.get("watcherID")
        if target_id is not None and tid != target_id:
            continue
        if watcher_id is not None and wid != watcher_id:
            continue
        return key, entry
    key, entry = next(iter(target_list.items()))
    return key, entry


def _ensure_aircraft_entries(raw_list: Iterable[Any]) -> List[Dict[str, int]]:
    result: List[Dict[str, int]] = []
    for entry in raw_list:
        if not isinstance(entry, dict):
            continue
        aircraft_id = entry.get("aircraftID")
        try:
            aircraft_id = int(aircraft_id)
        except (TypeError, ValueError):
            continue
        result.append({"aircraftID": aircraft_id})
    if not result:
        result = [{"aircraftID": idx} for idx in range(1, 7)]
    return result


def _is_trueish(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "t", "y", "yes", "on"}
    return False


def _coord_value(coord: Dict[str, Any], base_key: str, default: Optional[float] = None) -> float:
    for key in (base_key, base_key.capitalize(), base_key.upper()):
        if key in coord and coord[key] is not None:
            return float(coord[key])
    if default is not None:
        return float(default)
    raise RuntimeError(f"{base_key} missing in target coordinate block")


def _square_area(lat: float, lon: float, altitude: float, size_km: float) -> List[Dict[str, float]]:
    half = max(size_km, 0.1) / 2.0
    deg_lat = half / 111.0
    cos_lat = math.cos(math.radians(lat)) or 1e-6
    deg_lon = half / (111.0 * cos_lat)
    north = lat + deg_lat
    south = lat - deg_lat
    east = lon + deg_lon
    west = lon - deg_lon
    return [
        {"latitude": north, "longitude": west, "altitude": altitude},
        {"latitude": north, "longitude": east, "altitude": altitude},
        {"latitude": south, "longitude": east, "altitude": altitude},
        {"latitude": south, "longitude": west, "altitude": altitude},
    ]


def _deep_copy(obj: Any) -> Any:
    return json.loads(json.dumps(obj))


def _normalize_coordinate(source: Any) -> Dict[str, Any]:
    if isinstance(source, dict):
        return source
    if source is None:
        return {}
    coord: Dict[str, Any] = {}
    for key in ("latitude", "Latitude"):
        if hasattr(source, key):
            coord["latitude"] = getattr(source, key)
            break
    for key in ("longitude", "Longitude", "lon", "Lon"):
        if hasattr(source, key):
            coord["longitude"] = getattr(source, key)
            break
    for key in ("altitude", "Altitude"):
        if hasattr(source, key):
            coord["altitude"] = getattr(source, key)
            break
    return coord


def _coerce_int(value: Any) -> Optional[int]:
    try:
        if value is None:
            return None
        return int(value)
    except (TypeError, ValueError):
        try:
            return int(float(str(value).strip()))
        except Exception:
            return None


def _extract_watcher_ids(entry: Any) -> Set[int]:
    if entry is None:
        return set()
    watcher_ids: Set[int] = set()

    def _add(value: Any) -> None:
        candidate = _coerce_int(value)
        if candidate is not None:
            watcher_ids.add(candidate)

    if isinstance(entry, dict):
        for key in ("watcherID", "watcherId", "watcher_id"):
            if key in entry:
                _add(entry[key])
        watcher_obj = entry.get("watcher")
        if isinstance(watcher_obj, dict):
            _add(watcher_obj.get("aircraftID") or watcher_obj.get("AircraftID"))
    else:
        for attr in ("watcherID", "WatcherID", "watcherId"):
            if hasattr(entry, attr):
                _add(getattr(entry, attr))
        watcher_obj = getattr(entry, "watcher", None) or getattr(entry, "Watcher", None)
        if watcher_obj is not None:
            for attr in ("aircraftID", "AircraftID"):
                if hasattr(watcher_obj, attr):
                    _add(getattr(watcher_obj, attr))
                    break

    return watcher_ids


def _inject_area_mission(missions: List[Dict[str, Any]], area_coords: List[Dict[str, float]]) -> int:
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        detail = mission.get("missionDetail")
        if not isinstance(detail, dict):
            continue
        if not _is_trueish(mission.get("isDone")):
            detail["areaList"] = [{"isHole": False, "coordinateList": area_coords}]
            detail["lineList"] = None
            detail["coordinateList"] = None
            mission["missionDetail"] = detail
            mission["isDone"] = False
            mission["inputMissionType"] = 2
            try:
                return int(mission.get("inputMissionID") or 0)
            except Exception:
                return 0
    next_id = max((int(m.get("inputMissionID") or 0) for m in missions if isinstance(m, dict)), default=0) + 1
    missions.append(
        {
            "inputMissionID": next_id,
            "inputMissionType": 2,
            "isDone": False,
            "missionDetail": {
                "coordinateList": None,
                "lineList": None,
                "areaList": [{"isHole": False, "coordinateList": area_coords}],
            },
        }
    )
    return next_id


def build_payload(base_data: Dict[str, Any], area_coords: List[Dict[str, float]], timestamp: Optional[int]) -> Tuple[Dict[str, Any], int]:
    payload = _deep_copy(base_data)
    missions = payload.get("inputMissionList") or []
    if not isinstance(missions, list):
        raise RuntimeError("inputMissionList is missing or malformed in base 0201 data")
    mission_id = _inject_area_mission(missions, area_coords)
    payload["inputMissionList"] = missions
    payload["timestamp"] = int(timestamp or time.time_ns() // 1_000_000)
    payload.setdefault("source", "MSM")
    payload["availableAircraftList"] = _ensure_aircraft_entries(payload.get("availableAircraftList") or [])
    return payload, mission_id


def _allocate_attack_path(target_dir: Path, preferred_name: Optional[str]) -> Path:
    target_dir.mkdir(parents=True, exist_ok=True)
    if preferred_name:
        candidate = Path(preferred_name)
        if not candidate.suffix:
            candidate = candidate.with_suffix(".json")
        final_path = target_dir / candidate.name
        return final_path
    idx = 1
    while True:
        candidate = target_dir / f"0201_{idx:02d}.json"
        if not candidate.exists():
            return candidate
        idx += 1


def write_payload(payload: Dict[str, Any], agency_dir: Path, output_name: Optional[str]) -> Path:
    target_dir = agency_dir / "DSS_Internal"
    output_path = _allocate_attack_path(target_dir, output_name)
    with output_path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
    return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Create a temporary 0201 payload focused on a newly detected target.")
    parser.add_argument("--scenario-dir", type=str, default=None, help="Override scenario directory (defaults to current_scenario.json).")
    parser.add_argument("--agency", type=str, default=None, help="Override agency folder (defaults to value in current_scenario.json).")
    parser.add_argument("--package-id", type=int, default=None, help="Specific InputMissionPackageID JSON to clone.")
    parser.add_argument("--target-key", type=str, default=None, help="Key inside targetInfo.json (e.g., '8-4').")
    parser.add_argument("--target-id", type=int, default=None, help="TargetID to match when selecting the target entry.")
    parser.add_argument("--watcher-id", type=int, default=None, help="WatcherID to match when selecting the target entry.")
    parser.add_argument("--size-km", type=float, default=2.0, help="Width/height of the square AOI in kilometers.")
    parser.add_argument("--output-name", type=str, default=None, help="Explicit filename for the attack 0201 (defaults to auto-incremented 0201_##.json).")
    return parser.parse_args()


def create_attack_plan_from_target(
    *,
    target_entry: Optional[Dict[str, Any]] = None,
    target_key: Optional[str] = None,
    target_id: Optional[int] = None,
    watcher_id: Optional[int] = None,
    scenario_dir: Optional[str] = None,
    agency: Optional[str] = None,
    package_id: Optional[int] = None,
    size_km: float = 2.0,
    output_name: Optional[str] = None,
) -> Tuple[Path, Dict[str, Any]]:
    agency_dir = load_active_agency_dir(scenario_dir, agency)
    base_data, base_path = select_base_input_plan(agency_dir, package_id)
    resolved_key = target_key
    resolved_entry = target_entry
    if resolved_entry is None:
        resolved_key, resolved_entry = select_target_entry(agency_dir, target_key, target_id, watcher_id)

    if isinstance(resolved_entry, dict):
        coord_obj = resolved_entry.get("coordinate") or resolved_entry.get("Coordinate")
    else:
        coord_obj = getattr(resolved_entry, "coordinate", None) or getattr(resolved_entry, "Coordinate", None)
    coord = _normalize_coordinate(coord_obj)
    latitude = _coord_value(coord, "latitude")
    longitude = _coord_value(coord, "longitude")
    altitude = _coord_value(coord, "altitude", default=0.0)
    area_coords = _square_area(latitude, longitude, altitude, size_km)

    if isinstance(resolved_entry, dict):
        timestamp = resolved_entry.get("firstDetected") or resolved_entry.get("lastUpdated")
    else:
        timestamp = getattr(resolved_entry, "firstDetected", None) or getattr(resolved_entry, "lastUpdated", None)

    payload, mission_id = build_payload(base_data, area_coords, timestamp)
    watcher_ids = _extract_watcher_ids(resolved_entry)
    if watcher_ids:
        original_list = payload.get("availableAircraftList") or []
        filtered = []
        for item in original_list:
            if not isinstance(item, dict):
                continue
            candidate_id = _coerce_int(item.get("aircraftID"))
            if candidate_id is None or candidate_id not in watcher_ids:
                filtered.append(item)
        if filtered:
            payload["availableAircraftList"] = filtered
    target_id_value = None
    if isinstance(resolved_entry, dict):
        target_id_value = _coerce_int(resolved_entry.get("targetID"))
    else:
        target_id_value = _coerce_int(getattr(resolved_entry, "targetID", None))
    attack_context = {
        "target": {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
        },
        "targetID": target_id_value,
        "detectedTimestamp": (
            resolved_entry.get("firstDetected") if isinstance(resolved_entry, dict) else getattr(resolved_entry, "firstDetected", None)
        )
        or (
            resolved_entry.get("lastUpdated") if isinstance(resolved_entry, dict) else getattr(resolved_entry, "lastUpdated", None)
        ),
        "watcherAircraftIDs": sorted(watcher_ids),
        "targetCount": 1,
    }
    payload["_attackContext"] = attack_context
    output_path = write_payload(payload, agency_dir, output_name)
    meta = {
        "mission_id": mission_id,
        "target_key": resolved_key,
        "base_plan": base_path.name,
        "output_path": str(output_path),
        "attack_context": attack_context,
    }
    if watcher_ids:
        meta["excluded_aircraft"] = sorted(watcher_ids)
    return output_path, meta


def main() -> None:
    args = parse_args()
    output_path, meta = create_attack_plan_from_target(
        scenario_dir=args.scenario_dir,
        agency=args.agency,
        package_id=args.package_id,
        target_key=args.target_key,
        target_id=args.target_id,
        watcher_id=args.watcher_id,
        size_km=args.size_km,
        output_name=args.output_name,
    )
    print(
        f"Created {output_path} using base plan {meta.get('base_plan')}; "
        f"updated missionID={meta.get('mission_id')}, targetKey={meta.get('target_key')}, AOI size={args.size_km}km"
    )


if __name__ == "__main__":
    main()
