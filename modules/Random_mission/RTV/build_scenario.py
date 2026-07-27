from __future__ import annotations

import argparse
import json
import re
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TEMPLATE = ROOT / "RTV" / "newScenario_251030.json"
DEFAULT_DB = ROOT / "database"
DEFAULT_OUT_DIR = DEFAULT_DB / "Scenario"


def scenario_name_now(now: datetime | None = None, seq: int | None = None) -> str:
    now = now or datetime.now()
    if seq is None:
        return f"randomScenario_{now:%y%m%d_%H%M}"
    return f"randomScenario_{now:%y%m%d}_{int(seq):04d}"


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)


def _latest_by_seq(dir_path: Path) -> Optional[Path]:
    if not dir_path.exists():
        return None
    best = None
    best_seq = None
    for path in dir_path.glob("*.json"):
        seq = _extract_seq(path.stem)
        if seq is None:
            continue
        if best_seq is None or seq > best_seq:
            best_seq = seq
            best = path
    return best


def _extract_seq(stem: str) -> int | None:
    if not stem:
        return None
    if stem.isdigit():
        return int(stem)
    match = re.search(r"(\d+)$", stem)
    if not match:
        return None
    try:
        return int(match.group(1))
    except Exception:
        return None


def _int_value(value: Any, default: float = 0.0) -> int:
    try:
        return int(round(float(value)))
    except Exception:
        return int(round(float(default)))


def _float_override(value: Any) -> Optional[float]:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return float(text)
    except Exception:
        return None


def _coord_llh(coord: Dict[str, Any], *, alt_default: float = 0.0) -> Dict[str, float | int]:
    return {
        "Latitude": float(coord.get("latitude", 0.0)),
        "Longitude": float(coord.get("longitude", 0.0)),
        "Altitude": _int_value(coord.get("altitude", alt_default), alt_default),
    }


def _coord_ll(coord: Dict[str, Any]) -> Dict[str, float]:
    return {
        "Latitude": float(coord.get("latitude", 0.0)),
        "Longitude": float(coord.get("longitude", 0.0)),
    }


def _coord_list(coords: Iterable[Dict[str, Any]], *, alt_default: float = 0.0) -> List[Dict[str, float]]:
    return [_coord_llh(c, alt_default=alt_default) for c in coords]


def _area_list(area: Dict[str, Any]) -> Dict[str, Any]:
    coords = area.get("coordinateList") or []
    return {
        "IsHole": bool(area.get("isHole", False)),
        "CoordinateListN": len(coords),
        "CoordinateList": _coord_list(coords),
    }


def _polyline(line: Dict[str, Any]) -> Dict[str, Any]:
    coords = line.get("coordinateList") or []
    return {
        "Width": _int_value(line.get("width", 0)),
        "CoordinateListN": len(coords),
        "CoordinateList": _coord_list(coords),
    }


def _set_polylines(
    out: Dict[str, Any],
    lines: Iterable[Dict[str, Any]],
    *,
    modern_schema: bool,
) -> None:
    items = [_polyline(line) for line in lines if isinstance(line, dict)]
    if modern_schema:
        # 260709 RTV ICD: PolyLines 안에 LineListN/LineList를 둔다.
        out["PolyLines"] = {"LineListN": len(items), "LineList": items}
        out.pop("PolylinesN", None)
        out.pop("Polylines", None)
    else:
        # 기존 Random_mission 출력 계약을 유지한다.
        out["PolylinesN"] = len(items)
        out["Polylines"] = items
        out.pop("PolyLines", None)
    out.pop("PolyLine", None)


def _build_input_missions(
    imp_payload: Dict[str, Any],
    *,
    coordinate_template: Dict[str, Any],
    display_default: bool,
    include_display: bool,
    modern_line_schema: bool,
    sequence_start: int,
) -> List[Dict[str, Any]]:
    missions_out: List[Dict[str, Any]] = []
    for offset, mission in enumerate(imp_payload.get("inputMissionList") or []):
        seq = int(sequence_start) + offset
        mtype = int(mission.get("inputMissionType", 0) or 0)
        input_id_raw = mission.get("inputMissionID", seq)
        try:
            input_id = int(input_id_raw)
        except Exception:
            input_id = seq

        out: Dict[str, Any] = {
            "InputMissionID": 70_000_000 + input_id,
            "SequenceNumber": seq,
            "InputMissionType": mtype,
            "RegionType": int(mission.get("regionType", 0) or 0),
            "IsDone": False,
            "ShapeType": 2 if mtype == 1 else 3,
            "Coordinate": deepcopy(coordinate_template),
            "Polygons": {"AreaListN": 0, "AreaList": []},
        }
        if include_display:
            out["IsDisplay"] = bool(display_default)
        _set_polylines(out, [], modern_schema=modern_line_schema)

        detail = mission.get("missionDetail") or {}
        lines = detail.get("lineList") or []
        areas = detail.get("areaList") or []
        coords = detail.get("coordinateList") or []

        if lines:
            out["ShapeType"] = 2
            _set_polylines(out, lines, modern_schema=modern_line_schema)
            out["Polygons"] = {"AreaListN": 0, "AreaList": []}
        elif areas:
            out["ShapeType"] = 3
            _set_polylines(out, [], modern_schema=modern_line_schema)
            out["Polygons"] = {
                "AreaListN": len(areas),
                "AreaList": [_area_list(a) for a in areas],
            }
        elif coords:
            out["ShapeType"] = 1
            out["Coordinate"] = _coord_llh(coords[0])
            _set_polylines(out, [], modern_schema=modern_line_schema)
            out["Polygons"] = {"AreaListN": 0, "AreaList": []}

        missions_out.append(out)

    return missions_out


def _build_mission_reference(mr_payload: Dict[str, Any], template: Dict[str, Any]) -> Dict[str, Any]:
    out = {
        "MissionReferencePackageID": _int_value(
            mr_payload.get(
                "missionReferencePackageID",
                template.get("MissionReferencePackageID", 0),
            )
        ),
        "PresenceVector": template.get("PresenceVector", 0),
        "Timestamp": template.get("Timestamp", "AAAAAAA="),
    }

    take_over = mr_payload.get("takeOverInfoList") or []
    out["TakeOverInfoListN"] = len(take_over)
    out["TakeOverInfoList"] = [
        {
            "AircraftID": int(entry.get("aircraftID", 0)),
            "CoordinateList": _coord_llh(entry.get("coordinate") or {}),
        }
        for entry in take_over
    ]

    hand_over = mr_payload.get("handOverInfoList") or []
    out["HandOverInfoListN"] = len(hand_over)
    out["HandOverInfoList"] = [
        {
            "AircraftID": int(entry.get("aircraftID", 0)),
            "CoordinateList": _coord_llh(entry.get("coordinate") or {}),
        }
        for entry in hand_over
    ]

    rtb = mr_payload.get("rtbCoordinateList") or []
    out["RTBCoordinateListN"] = len(rtb)
    out["RTBCoordinateList"] = [_coord_llh(entry) for entry in rtb]

    flight_areas = mr_payload.get("flightAreaList") or []
    out["FlightAreaListN"] = len(flight_areas)
    out["FlightAreaList"] = []
    for area in flight_areas:
        coords = area.get("areaLatLonList") or []
        limits = area.get("altitudeLimits") or {}
        out["FlightAreaList"].append(
            {
                "AreaLatLonListN": len(coords),
                "AreaLatLonList": [_coord_ll(c) for c in coords],
                "AltitudeLimits": {
                    "LowerLimit": _int_value(limits.get("lowerLimit", 0)),
                    "UpperLimit": _int_value(limits.get("upperLimit", 0)),
                },
            }
        )

    prohib_areas = mr_payload.get("prohibitedAreaList") or []
    out["ProhibitedAreaListN"] = len(prohib_areas)
    out["ProhibitedAreaList"] = []
    for area in prohib_areas:
        coords = area.get("areaLatLonList") or []
        limits = area.get("altitudeLimits") or {}
        out["ProhibitedAreaList"].append(
            {
                "AreaLatLonListN": len(coords),
                "AreaLatLonList": [_coord_ll(c) for c in coords],
                "AltitudeLimits": {
                    "LowerLimit": _int_value(limits.get("lowerLimit", 0)),
                    "UpperLimit": _int_value(limits.get("upperLimit", 0)),
                },
            }
        )

    for key in ("RegionInfosN", "RegionInfo"):
        if key in template:
            out[key] = deepcopy(template[key])

    return out


def _find_unit(units: Iterable[Dict[str, Any]], predicate) -> Optional[Dict[str, Any]]:
    for unit in units:
        if predicate(unit):
            return unit
    return None


def _build_unit_objects(
    template_units: List[Dict[str, Any]],
    imp_payload: Dict[str, Any],
    mr_payload: Dict[str, Any],
    tgt_payload: Dict[str, Any],
    *,
    detect_pixel: Any = None,
    recog_pixel: Any = None,
) -> List[Dict[str, Any]]:
    units_by_id = {int(u.get("ID", -1)): u for u in template_units if "ID" in u}
    detect_pixel_value = _float_override(detect_pixel)
    recog_pixel_value = _float_override(recog_pixel)

    lah_default = _find_unit(template_units, lambda u: u.get("Type") == 3 and u.get("Identification") == 1)
    uav_default = _find_unit(template_units, lambda u: u.get("Type") == 1 and u.get("Identification") == 1)
    tgt_default = _find_unit(template_units, lambda u: u.get("Type") == 5 and u.get("Identification") == 2)
    tgt_strong = _find_unit(
        template_units,
        lambda u: u.get("Type") == 5 and u.get("Identification") == 2 and float(u.get("AttackDamage", 0)) >= 100.0,
    )

    if lah_default is None and template_units:
        lah_default = template_units[0]
    if uav_default is None and template_units:
        uav_default = template_units[0]
    if tgt_default is None and template_units:
        tgt_default = template_units[0]
    if tgt_strong is None:
        tgt_strong = tgt_default

    take_over = {int(e.get("aircraftID", -1)): e.get("coordinate") for e in mr_payload.get("takeOverInfoList") or []}

    aircraft_ids = [int(e.get("aircraftID", 0)) for e in imp_payload.get("availableAircraftList") or []]
    aircraft_ids = [aid for aid in aircraft_ids if aid > 0]

    units_out: List[Dict[str, Any]] = []
    for aid in sorted(aircraft_ids):
        base = units_by_id.get(aid)
        if base is None:
            base = lah_default if aid <= 3 else uav_default
        unit = deepcopy(base)
        unit["ID"] = aid
        if aid <= 3:
            unit["Type"] = 3
            unit["Identification"] = 1
        else:
            unit["Type"] = 1
            unit["Identification"] = 1

        if aid in take_over:
            coord = take_over[aid] or {}
            loc = unit.get("LOC") or {}
            loc["Latitude"] = float(coord.get("latitude", loc.get("Latitude", 0.0)))
            loc["Longitude"] = float(coord.get("longitude", loc.get("Longitude", 0.0)))
            loc["Altitude"] = _int_value(coord.get("altitude", loc.get("Altitude", 0)))
            unit["LOC"] = loc
        if detect_pixel_value is not None:
            unit["DetectPixel"] = detect_pixel_value
        if recog_pixel_value is not None:
            unit["RecogPixel"] = recog_pixel_value
        units_out.append(unit)

    max_aircraft_id = max(aircraft_ids, default=0)
    targets = tgt_payload.get("targetList") or []
    for target in targets:
        base = tgt_strong if int(target.get("targetType", 0)) == 2 else tgt_default
        unit = deepcopy(base)
        target_id = int(target.get("targetID", 0))
        unit["ID"] = max_aircraft_id + target_id
        loc = unit.get("LOC") or {}
        location = target.get("location") or {}
        loc["Latitude"] = float(location.get("latitude", loc.get("Latitude", 0.0)))
        loc["Longitude"] = float(location.get("longitude", loc.get("Longitude", 0.0)))
        if "altitude" in location:
            loc["Altitude"] = _int_value(location.get("altitude", loc.get("Altitude", 0)))
        unit["LOC"] = loc
        units_out.append(unit)

    return units_out


def build_scenario(
    *,
    template_path: Path,
    imp_path: Path,
    mr_path: Path,
    tgt_path: Path,
    scenario_name: str | None = None,
    detect_pixel: Any = None,
    recog_pixel: Any = None,
) -> Dict[str, Any]:
    template = _load_json(template_path)
    imp_payload = _load_json(imp_path)
    mr_payload = _load_json(mr_path)
    tgt_payload = _load_json(tgt_path)

    scenario = deepcopy(template)
    scenario["ScenarioName"] = scenario_name_now() if not scenario_name else scenario_name

    init = scenario.get("InitScenario") or {}
    scenario["InitScenario"] = init

    pkg = init.get("InputMissionPackage") or {}
    init["InputMissionPackage"] = pkg
    pkg["InputMissionPackageID"] = _int_value(
        imp_payload.get("inputMissionPackageID", pkg.get("InputMissionPackageID", 0))
    )
    pkg["MissionType"] = _int_value(
        imp_payload.get("inputMissionPackageType", pkg.get("MissionType", 0))
    )

    template_missions = pkg.get("InputMissionList") or []
    coordinate_template = {}
    display_default = True
    if template_missions:
        coordinate_template = deepcopy(template_missions[0].get("Coordinate") or {})
        display_default = bool(template_missions[0].get("IsDisplay", True))
    include_display = bool(template_missions and "IsDisplay" in template_missions[0])
    if not coordinate_template:
        coordinate_template = {"Latitude": 0.0, "Longitude": 0.0, "Altitude": 700}
    if "Altitude" in coordinate_template:
        coordinate_template["Altitude"] = _int_value(coordinate_template.get("Altitude", 0))

    modern_line_schema = any("PolyLines" in mission for mission in template_missions)
    sequence_start = 0 if any(
        _int_value(mission.get("SequenceNumber", 1), 1) == 0
        for mission in template_missions
    ) else 1
    missions_out = _build_input_missions(
        imp_payload,
        coordinate_template=coordinate_template,
        display_default=display_default,
        include_display=include_display,
        modern_line_schema=modern_line_schema,
        sequence_start=sequence_start,
    )
    pkg["InputMissionList"] = missions_out
    pkg["InputMissionListN"] = len(missions_out)

    aircraft_ids = [int(e.get("aircraftID", 0)) for e in imp_payload.get("availableAircraftList") or []]
    aircraft_ids = [aid for aid in aircraft_ids if aid > 0]
    pkg["AircraftIDs"] = aircraft_ids
    pkg["AircraftIDsN"] = len(aircraft_ids)

    mr_template = init.get("MissionReferencePackage") or {}
    init["MissionReferencePackage"] = _build_mission_reference(mr_payload, mr_template)

    scenario["UnitObjectList"] = _build_unit_objects(
        template.get("UnitObjectList") or [],
        imp_payload,
        mr_payload,
        tgt_payload,
        detect_pixel=detect_pixel,
        recog_pixel=recog_pixel,
    )

    return scenario


def main() -> None:
    parser = argparse.ArgumentParser(description="0201/0203/타깃 JSON으로 시나리오를 생성합니다.")
    parser.add_argument("--template", type=str, default=str(DEFAULT_TEMPLATE))
    parser.add_argument("--imp", type=str, default="")
    parser.add_argument("--scn", type=str, default="")
    parser.add_argument("--tgt", type=str, default="")
    parser.add_argument("--out-dir", type=str, default=str(DEFAULT_OUT_DIR))
    args = parser.parse_args()

    template_path = Path(args.template).resolve()
    db_root = DEFAULT_DB

    imp_path = Path(args.imp) if args.imp else _latest_by_seq(db_root / "InputMissionPlan")
    mr_path = Path(args.scn) if args.scn else _latest_by_seq(db_root / "MissionReferenceInfo")
    tgt_path = Path(args.tgt) if args.tgt else _latest_by_seq(db_root / "TargetInfo")

    if imp_path is None or mr_path is None or tgt_path is None:
        raise SystemExit("IMP/SCN/TGT JSON을 찾지 못했습니다. --imp/--scn/--tgt를 지정해 주세요.")

    seq = _extract_seq(Path(imp_path).stem) or _extract_seq(Path(mr_path).stem) or _extract_seq(Path(tgt_path).stem)
    scenario_name = scenario_name_now(seq=seq)
    out_dir = Path(args.out_dir)
    out_path = out_dir / f"{scenario_name}.json"

    scenario = build_scenario(
        template_path=template_path,
        imp_path=Path(imp_path).resolve(),
        mr_path=Path(mr_path).resolve(),
        tgt_path=Path(tgt_path).resolve(),
        scenario_name=scenario_name,
    )
    _write_json(out_path, scenario)
    print(out_path)


if __name__ == "__main__":
    main()
