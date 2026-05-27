from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
except Exception:  # pragma: no cover - optional runtime dependency
    LineString = None
    Polygon = None
    unary_union = None


PROJECT_ROOT = Path(__file__).resolve().parents[2]
EARTH_RADIUS_M = 6_371_008.8


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _coerce_int(value: Any) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _coerce_datetime_ms(value: Any) -> int | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        if text.endswith("Z"):
            text = text[:-1] + "+00:00"
        return int(datetime.fromisoformat(text).timestamp() * 1000)
    except Exception:
        return None


def _load_json(path: Path) -> dict[str, Any] | list[Any] | None:
    """Load a JSON file.  Returns *None* on any failure."""
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _load_json_dict(path: Path) -> dict[str, Any]:
    """Load a JSON file, always returning a *dict* (empty on failure)."""
    data = _load_json(path)
    if isinstance(data, dict):
        return data
    return {}


def _load_first_json_dict(folder: Path, preferred_stems: tuple[str, ...] = ()) -> dict[str, Any]:
    """Load a preferred JSON file from *folder*, then the first numeric JSON."""
    if not folder.is_dir():
        return {}
    for stem in preferred_stems:
        data = _load_json_dict(folder / f"{stem}.json")
        if data:
            return data

    def _sort_key(path: Path) -> tuple[int, str]:
        try:
            return (int(path.stem), path.stem)
        except Exception:
            return (1_000_000_000, path.stem)

    for fp in sorted(folder.glob("*.json"), key=_sort_key):
        data = _load_json_dict(fp)
        if data:
            return data
    return {}


def _read_text(path: Path) -> str | None:
    for encoding in ("utf-8-sig", "utf-8", "cp949"):
        try:
            return path.read_text(encoding=encoding)
        except Exception:
            continue
    return None


def _extract_waypoints(data: dict[str, Any]) -> list[dict[str, Any]]:
    for key in ("lahWaypointList", "uavWaypointList", "waypointList"):
        lst = data.get(key)
        if isinstance(lst, list):
            return lst
    return []


def _extract_waypoint_details(data: dict[str, Any]) -> list[dict[str, Any]]:
    """Extract waypoint details (id, coordinate, speed, eta, etc.) for frontend."""
    wps = _extract_waypoints(data)
    result = []
    for wp in wps:
        if not isinstance(wp, dict):
            continue
        coord = wp.get("coordinate") or {}
        lat = coord.get("latitude") or coord.get("Latitude")
        lon = coord.get("longitude") or coord.get("Longitude")
        alt = coord.get("altitude") or coord.get("Altitude")
        if lat is None or lon is None:
            continue
        # Determine pass type: UAV uses waypointPassType, LAH uses attack/hovering/loiter
        pass_type = wp.get("waypointPassType")
        lah_type = None
        attack_info = None
        hovering_info = None
        loiter_info = None

        atk = wp.get("attack") or wp.get("Attack")
        if isinstance(atk, dict) and (atk.get("targetID") or atk.get("weaponType")):
            lah_type = "attack"
            attack_info = {
                "targetID": atk.get("targetID"),
                "weaponType": atk.get("weaponType"),
            }

        hvr = wp.get("hovering") or wp.get("Hovering")
        if isinstance(hvr, dict) and hvr.get("time"):
            lah_type = "hovering"
            hovering_info = {"time": hvr.get("time")}

        ltr = wp.get("loiter") or wp.get("Loiter")
        if isinstance(ltr, dict) and (ltr.get("radius") or ltr.get("time")):
            lah_type = "loiter"
            loiter_info = {
                "radius": ltr.get("radius"),
                "direction": ltr.get("direction"),
                "time": ltr.get("time"),
                "speed": ltr.get("speed"),
            }

        result.append({
            "waypointID": wp.get("waypointID") or wp.get("WaypointID"),
            "latitude": lat,
            "longitude": lon,
            "altitude": alt,
            "speed": wp.get("speed"),
            "eta": wp.get("eta"),
            "waypointPassType": pass_type,
            "lahType": lah_type,
            "attack": attack_info,
            "hovering": hovering_info,
            "loiter": loiter_info,
            "nextWaypointID": wp.get("nextWaypointID"),
        })
    return result


def _extract_coord(item: dict[str, Any]) -> tuple[float, float, float | None] | None:
    coord = item.get("coordinate") or item.get("Coordinate")
    if not isinstance(coord, dict):
        return None
    lat = coord.get("latitude") if "latitude" in coord else coord.get("Latitude")
    lon = coord.get("longitude") if "longitude" in coord else coord.get("Longitude")
    alt = coord.get("altitude") if "altitude" in coord else coord.get("Altitude")
    if lat is None or lon is None:
        return None
    try:
        return float(lat), float(lon), float(alt) if alt is not None else None
    except Exception:
        return None


def _order_waypoints(raw: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not raw:
        return []
    by_id: dict[int, dict[str, Any]] = {}
    next_ids: set[int] = set()
    for wp in raw:
        if not isinstance(wp, dict):
            continue
        wid = wp.get("waypointID") or wp.get("WaypointID")
        if wid is None:
            continue
        try:
            wid_i = int(wid)
        except Exception:
            continue
        by_id[wid_i] = wp
        nxt = wp.get("nextWaypointID") or wp.get("NextWaypointID")
        if nxt is None:
            continue
        try:
            nxt_i = int(nxt)
        except Exception:
            continue
        if nxt_i > 0:
            next_ids.add(nxt_i)

    if not by_id:
        return list(raw)

    start_id = None
    for wid in by_id:
        if wid not in next_ids:
            start_id = wid
            break

    ordered: list[dict[str, Any]] = []
    visited: set[int] = set()
    if start_id is not None:
        curr = start_id
        while curr and curr in by_id and curr not in visited:
            wp = by_id[curr]
            ordered.append(wp)
            visited.add(curr)
            nxt = wp.get("nextWaypointID") or wp.get("NextWaypointID")
            try:
                curr = int(nxt)
            except Exception:
                break
            if curr == 0:
                break

    for wp in raw:
        if not isinstance(wp, dict):
            continue
        wid = wp.get("waypointID") or wp.get("WaypointID")
        if wid is None:
            ordered.append(wp)
            continue
        try:
            wid_i = int(wid)
        except Exception:
            ordered.append(wp)
            continue
        if wid_i not in visited:
            ordered.append(wp)
    return ordered


def _agent_label(aircraft_id: int) -> str:
    if 1 <= aircraft_id <= 3:
        return f"LAH{aircraft_id}"
    if 4 <= aircraft_id <= 6:
        return f"UAV{aircraft_id - 3}"
    return f"AC{aircraft_id}"


def _waypoint_pass_mode(item: dict[str, Any]) -> str:
    loiter = (
        item.get("loiter")
        or item.get("Loiter")
        or item.get("loiterProperty")
        or item.get("LoiterProperty")
        or item.get("loiter_prop")
    )
    pass_type = _coerce_int(item.get("waypointPassType") or item.get("WaypointPassType"))
    if loiter is not None or pass_type == 2:
        return "loiter"
    if pass_type == 1:
        return "fly-by"
    return "fly-over"


def _coerce_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return bool(int(value))
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ("1", "true", "yes", "y", "on"):
            return True
        if lowered in ("0", "false", "no", "n", "off"):
            return False
    return default


# ---------------------------------------------------------------------------
# GeoJSON builders
# ---------------------------------------------------------------------------

def _build_flight_path_geojson(data: dict[str, Any]) -> dict[str, Any] | None:
    """Build a GeoJSON LineString feature from a FlightPath JSON dict."""
    waypoints = _extract_waypoints(data)
    if not waypoints:
        return None
    waypoints = _order_waypoints(waypoints)

    coords: list[list[float]] = []
    for wp in waypoints:
        if not isinstance(wp, dict):
            continue
        c = _extract_coord(wp)
        if c is None:
            continue
        lat, lon, alt = c
        coords.append([lon, lat] if alt is None else [lon, lat, alt])

    if len(coords) < 1:
        return None

    return {
        "type": "Feature",
        "geometry": {
            "type": "LineString",
            "coordinates": coords,
        },
        "properties": {
            "pathID": _coerce_int(data.get("pathID") or data.get("PathID")),
            "aircraftID": _coerce_int(data.get("aircraftID") or data.get("AircraftID")),
        },
    }


def _coord_list_to_geojson_coords(coord_list: list[dict]) -> list[list[float]]:
    """Convert a coordinateList-style array to [[lon, lat], ...]."""
    result: list[list[float]] = []
    for item in coord_list:
        if not isinstance(item, dict):
            continue
        c = _extract_coord(item) if "coordinate" in item or "Coordinate" in item else None
        if c is not None:
            lat, lon, alt = c
            result.append([lon, lat])
            continue
        lat = item.get("latitude") or item.get("Latitude")
        lon = item.get("longitude") or item.get("Longitude")
        if lat is not None and lon is not None:
            try:
                result.append([float(lon), float(lat)])
            except Exception:
                continue
    return result


def _build_area_features(areas: list[dict], area_type: str) -> list[dict[str, Any]]:
    """Build GeoJSON Polygon features from area definitions."""
    features: list[dict[str, Any]] = []
    for area in areas:
        if not isinstance(area, dict):
            continue
        coord_list = (
            area.get("coordinateList")
            or area.get("CoordinateList")
            or area.get("areaLatLonList")
            or area.get("AreaLatLonList")
            or []
        )
        coords = _coord_list_to_geojson_coords(coord_list)
        if len(coords) < 3:
            continue
        # Close the ring
        if coords[0] != coords[-1]:
            coords.append(coords[0])
        features.append({
            "type": "Feature",
            "geometry": {"type": "Polygon", "coordinates": [coords]},
            "properties": {
                "areaType": area_type,
                "areaID": (
                    area.get("areaID")
                    or area.get("AreaID")
                    or area.get("flightAreaID")
                    or area.get("FlightAreaID")
                    or area.get("prohibitedAreaID")
                    or area.get("ProhibitedAreaID")
                ),
            },
        })
    return features


def _build_point_features(points: list[dict], point_type: str) -> list[dict[str, Any]]:
    """Build GeoJSON Point features from takeOver/handOver style lists."""
    features: list[dict[str, Any]] = []
    for pt in points:
        if not isinstance(pt, dict):
            continue
        coord = pt.get("coordinate") or pt.get("Coordinate")
        if not isinstance(coord, dict):
            continue
        lat = coord.get("latitude") if "latitude" in coord else coord.get("Latitude")
        lon = coord.get("longitude") if "longitude" in coord else coord.get("Longitude")
        if lat is None or lon is None:
            continue
        try:
            lat_v, lon_v = float(lat), float(lon)
        except Exception:
            continue
        features.append({
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [lon_v, lat_v]},
            "properties": {"pointType": point_type, **{k: v for k, v in pt.items() if k not in ("coordinate", "Coordinate")}},
        })
    return features


def _build_line_features(areas: list[dict], line_type: str) -> list[dict[str, Any]]:
    """Build GeoJSON LineString features from coordinateList areas."""
    features: list[dict[str, Any]] = []
    for area in areas:
        if not isinstance(area, dict):
            continue
        coord_list = area.get("coordinateList") or area.get("CoordinateList") or []
        coords = _coord_list_to_geojson_coords(coord_list)
        if len(coords) < 2:
            continue
        features.append({
            "type": "Feature",
            "geometry": {"type": "LineString", "coordinates": coords},
            "properties": {
                "lineType": line_type,
                "areaID": area.get("areaID") or area.get("AreaID"),
            },
        })
    return features


def _parse_tracks(base: Path) -> dict[str, Any]:
    """Parse log_0401_agent_status_sim.jsonl into per-aircraft track data.

    Returns ``{aircraftID: {timestamps: [ms,...], coordinates: [[lon,lat],...]}}``.
    Downsamples to ~1 point/second to keep payload small.
    """
    jsonl = base / "DSS_Internal" / "log_0401_agent_status_sim.jsonl"
    if not jsonl.exists():
        return {}

    tracks: dict[int, dict[str, list]] = {}
    last_ts: dict[int, int] = {}
    last_effective_ts: int | None = None
    last_raw_ts: int | None = None
    min_interval_ms = 800  # ≈1Hz downsample

    try:
        with jsonl.open("r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    entry = json.loads(line)
                except Exception:
                    continue
                raw_ts = _coerce_int(entry.get("timestamp") or entry.get("Timestamp"))
                logged_at_ms = _coerce_datetime_ms(entry.get("logged_at") or entry.get("LoggedAt"))

                if logged_at_ms is not None:
                    ts = logged_at_ms
                elif raw_ts is not None:
                    if (
                        last_effective_ts is not None
                        and last_raw_ts is not None
                        and raw_ts > last_raw_ts
                        and raw_ts < 10_000_000_000
                        and (raw_ts - last_raw_ts) < 10
                    ):
                        ts = last_effective_ts + 1000
                    elif last_effective_ts is not None and raw_ts <= last_effective_ts:
                        ts = last_effective_ts + 1000
                    else:
                        ts = raw_ts
                elif last_effective_ts is not None:
                    ts = last_effective_ts + 1000
                else:
                    ts = 0

                last_effective_ts = ts
                if raw_ts is not None:
                    last_raw_ts = raw_ts

                for agent in (entry.get("agentStateList") or entry.get("AgentStateList") or []):
                    if not isinstance(agent, dict):
                        continue
                    aid = _coerce_int(agent.get("aircraftID") or agent.get("AircraftID"))
                    if aid is None:
                        continue
                    coord = agent.get("coordinate") or agent.get("Coordinate") or {}
                    lat = coord.get("latitude") if "latitude" in coord else coord.get("Latitude")
                    lon = coord.get("longitude") if "longitude" in coord else coord.get("Longitude")
                    if lat is None or lon is None:
                        continue
                    prev = last_ts.get(aid)
                    if prev is not None and ts - prev < min_interval_ms:
                        continue
                    last_ts[aid] = ts
                    if aid not in tracks:
                        tracks[aid] = {"timestamps": [], "coordinates": []}
                    tracks[aid]["timestamps"].append(ts)
                    tracks[aid]["coordinates"].append([float(lon), float(lat)])
    except Exception:
        pass

    return {str(k): v for k, v in sorted(tracks.items())}


def _0401_sort_key(path: Path) -> tuple[int, str]:
    stem = path.stem
    if stem == "0401":
        return (0, stem)
    if stem.startswith("0401_"):
        suffix = stem.replace("0401_", "", 1)
        try:
            return (int(suffix) + 1, stem)
        except Exception:
            pass
    return (1_000_000, stem)


def _0401_json_files(folder: Path) -> list[Path]:
    if not folder.is_dir():
        return []
    return sorted(
        [fp for fp in folder.glob("0401*.json") if fp.is_file()],
        key=_0401_sort_key,
    )


def _resolve_0401_source(base: Path) -> dict[str, Any]:
    simlog_dir = base / "simlog_0401"
    simlog_files = _0401_json_files(simlog_dir)
    if simlog_files:
        return {"kind": "simlog_0401", "path": simlog_dir, "files": simlog_files}

    local_dir = base / "0401"
    local_files = _0401_json_files(local_dir)
    if local_files:
        return {"kind": "0401", "path": local_dir, "files": local_files}

    fallback_dir = PROJECT_ROOT / "Logs" / "Scenario_2026-05-21T105639" / "SBC3" / "0401"
    fallback_files = _0401_json_files(fallback_dir)
    if fallback_files:
        return {"kind": "fallback_0401", "path": fallback_dir, "files": fallback_files}

    jsonl = base / "DSS_Internal" / "log_0401_agent_status_sim.jsonl"
    if jsonl.exists():
        return {"kind": "dss_jsonl", "path": jsonl, "files": [jsonl]}

    return {"kind": "missing", "path": None, "files": []}


def _json_messages(path: Path) -> list[dict[str, Any]]:
    raw = _read_text(path)
    if not raw:
        return []
    text = raw.strip()
    if not text:
        return []
    try:
        data = json.loads(text)
        if isinstance(data, list):
            return [item for item in data if isinstance(item, dict)]
        if isinstance(data, dict):
            return [data]
    except Exception:
        pass

    messages: list[dict[str, Any]] = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            item = json.loads(line)
        except Exception:
            continue
        if isinstance(item, dict):
            messages.append(item)
    return messages


def _unwrap_0401_message(entry: dict[str, Any]) -> dict[str, Any] | None:
    if entry.get("agentStateList") or entry.get("AgentStateList"):
        return entry
    for key in ("raw", "message", "payload", "data", "body"):
        nested = entry.get(key)
        if isinstance(nested, dict) and (nested.get("agentStateList") or nested.get("AgentStateList")):
            if "logged_at" in entry and "logged_at" not in nested:
                nested = {**nested, "logged_at": entry.get("logged_at")}
            return nested
    return None


def _extract_agent_coord(agent: dict[str, Any]) -> list[float] | None:
    coord = agent.get("coordinate") or agent.get("Coordinate") or {}
    if not isinstance(coord, dict):
        return None
    lat = coord.get("latitude") if "latitude" in coord else coord.get("Latitude")
    lon = coord.get("longitude") if "longitude" in coord else coord.get("Longitude")
    alt = coord.get("altitude") if "altitude" in coord else coord.get("Altitude")
    if lat is None or lon is None:
        return None
    try:
        result = [float(lon), float(lat)]
        if alt is not None:
            result.append(float(alt))
        return result
    except Exception:
        return None


def _extract_agent_footprint(agent: dict[str, Any]) -> list[list[float]] | None:
    unmanned = agent.get("unmannedInfo") or agent.get("UnmannedInfo") or {}
    if not isinstance(unmanned, dict):
        return None
    sensor = unmanned.get("sensorInfo") or unmanned.get("SensorInfo") or {}
    if not isinstance(sensor, dict):
        return None
    raw = sensor.get("footprintCornerList") or sensor.get("FootprintCornerList") or []
    if not isinstance(raw, list):
        return None

    coords = _coord_list_to_geojson_coords(raw)
    if len(coords) < 3:
        return None
    if coords[0] != coords[-1]:
        coords.append(coords[0])
    return coords


def _effective_0401_timestamp(
    message: dict[str, Any],
    last_effective_ts: int | None,
    last_raw_ts: int | None,
) -> tuple[int, int | None]:
    raw_ts = _coerce_int(message.get("timestamp") or message.get("Timestamp"))
    logged_at_ms = _coerce_datetime_ms(message.get("logged_at") or message.get("LoggedAt"))
    if logged_at_ms is not None:
        return logged_at_ms, raw_ts
    if raw_ts is not None:
        if (
            last_effective_ts is not None
            and last_raw_ts is not None
            and raw_ts > last_raw_ts
            and raw_ts < 10_000_000_000
            and (raw_ts - last_raw_ts) < 10
        ):
            return last_effective_ts + 1000, raw_ts
        if last_effective_ts is not None and raw_ts <= last_effective_ts:
            return last_effective_ts + 1000, raw_ts
        return raw_ts, raw_ts
    if last_effective_ts is not None:
        return last_effective_ts + 1000, last_raw_ts
    return 0, last_raw_ts


def _parse_0401_playback(base: Path) -> dict[str, Any]:
    """Parse 0401 JSON folders into tracks and sensor footprint playback data."""
    source = _resolve_0401_source(base)
    tracks: dict[int, dict[str, list]] = {}
    footprints: dict[int, dict[str, list]] = {}
    last_ts: dict[int, int] = {}
    last_effective_ts: int | None = None
    last_raw_ts: int | None = None
    min_ts: int | None = None
    max_ts: int | None = None
    message_count = 0
    footprint_count = 0
    min_interval_ms = 800

    for fp in source.get("files") or []:
        for raw_entry in _json_messages(fp):
            message = _unwrap_0401_message(raw_entry)
            if message is None:
                continue
            message_count += 1
            ts, next_raw = _effective_0401_timestamp(message, last_effective_ts, last_raw_ts)
            last_effective_ts = ts
            if next_raw is not None:
                last_raw_ts = next_raw
            min_ts = ts if min_ts is None else min(min_ts, ts)
            max_ts = ts if max_ts is None else max(max_ts, ts)

            for agent in (message.get("agentStateList") or message.get("AgentStateList") or []):
                if not isinstance(agent, dict):
                    continue
                aid = _coerce_int(agent.get("aircraftID") or agent.get("AircraftID"))
                if aid is None:
                    continue
                prev = last_ts.get(aid)
                if prev is not None and ts - prev < min_interval_ms:
                    continue
                coord = _extract_agent_coord(agent)
                footprint = _extract_agent_footprint(agent)
                if coord is None and footprint is None:
                    continue
                last_ts[aid] = ts
                if coord is not None:
                    tracks.setdefault(aid, {"timestamps": [], "coordinates": []})
                    tracks[aid]["timestamps"].append(ts)
                    tracks[aid]["coordinates"].append(coord[:2])
                if footprint is not None:
                    center = coord[:2] if coord is not None else _polygon_center(footprint)
                    footprints.setdefault(aid, {"timestamps": [], "polygons": [], "centers": []})
                    footprints[aid]["timestamps"].append(ts)
                    footprints[aid]["polygons"].append(footprint)
                    footprints[aid]["centers"].append(center)
                    footprint_count += 1

    sample_count = sum(len(v.get("timestamps") or []) for v in tracks.values())
    return {
        "ok": bool(message_count),
        "sourceKind": source.get("kind"),
        "sourcePath": str(source.get("path")) if source.get("path") else None,
        "fileCount": len(source.get("files") or []),
        "messageCount": message_count,
        "sampleCount": sample_count,
        "footprintCount": footprint_count,
        "minTimestamp": min_ts,
        "maxTimestamp": max_ts,
        "tracks": {str(k): v for k, v in sorted(tracks.items())},
        "footprints": {str(k): v for k, v in sorted(footprints.items())},
    }


def _polygon_center(coords: list[list[float]]) -> list[float]:
    usable = coords[:-1] if len(coords) > 1 and coords[0] == coords[-1] else coords
    if not usable:
        return [0.0, 0.0]
    lon = sum(c[0] for c in usable) / len(usable)
    lat = sum(c[1] for c in usable) / len(usable)
    return [lon, lat]


def _extract_input_mission_sections(input_plan: dict[str, Any]) -> list[dict[str, Any]]:
    if not input_plan:
        return []
    missions = input_plan.get("inputMissionList") or input_plan.get("InputMissionList") or input_plan.get("missionList") or []
    sections: list[dict[str, Any]] = []
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        mission_id = _coerce_int(mission.get("inputMissionID") or mission.get("InputMissionID") or mission.get("missionID"))
        mission_type = _coerce_int(mission.get("inputMissionType") or mission.get("InputMissionType") or mission.get("missionType"))
        detail = mission.get("missionDetail") or mission.get("MissionDetail") or mission
        if not isinstance(detail, dict):
            detail = {}

        line_list = detail.get("lineList") or detail.get("LineList") or []
        if isinstance(line_list, list):
            for idx, line in enumerate(line_list, start=1):
                if not isinstance(line, dict):
                    continue
                coords = _coord_list_to_geojson_coords(line.get("coordinateList") or line.get("CoordinateList") or [])
                if len(coords) < 2:
                    continue
                width = _coerce_float(line.get("width") or line.get("Width"), 0.0)
                sections.append({
                    "sectionId": f"M{mission_id or '?'}-L{idx}",
                    "inputMissionID": mission_id,
                    "inputMissionType": mission_type,
                    "sectionType": "corridor",
                    "widthM": width,
                    "coordinates": coords,
                })

        area_list = detail.get("areaList") or detail.get("AreaList") or []
        if isinstance(area_list, list):
            for idx, area in enumerate(area_list, start=1):
                if not isinstance(area, dict) or _coerce_bool(area.get("isHole") or area.get("IsHole"), False):
                    continue
                coords = _coord_list_to_geojson_coords(area.get("coordinateList") or area.get("CoordinateList") or [])
                if len(coords) < 3:
                    continue
                if coords[0] != coords[-1]:
                    coords.append(coords[0])
                sections.append({
                    "sectionId": f"M{mission_id or '?'}-A{idx}",
                    "inputMissionID": mission_id,
                    "inputMissionType": mission_type,
                    "sectionType": "area",
                    "widthM": None,
                    "coordinates": coords,
                })

        if not sections or all(s.get("inputMissionID") != mission_id for s in sections):
            coords = _coord_list_to_geojson_coords(detail.get("coordinateList") or detail.get("CoordinateList") or [])
            if len(coords) >= 3 and mission_type in (2, 3):
                if coords[0] != coords[-1]:
                    coords.append(coords[0])
                sections.append({
                    "sectionId": f"M{mission_id or '?'}-A1",
                    "inputMissionID": mission_id,
                    "inputMissionType": mission_type,
                    "sectionType": "area",
                    "widthM": None,
                    "coordinates": coords,
                })
            elif len(coords) >= 2:
                sections.append({
                    "sectionId": f"M{mission_id or '?'}-L1",
                    "inputMissionID": mission_id,
                    "inputMissionType": mission_type,
                    "sectionType": "corridor",
                    "widthM": 0.0,
                    "coordinates": coords,
                })
    return sections


def _coerce_float(value: Any, default: float | None = None) -> float | None:
    try:
        return float(value)
    except Exception:
        return default


def _projection_origin(sections: list[dict[str, Any]], playback: dict[str, Any]) -> tuple[float, float]:
    coords: list[list[float]] = []
    for section in sections:
        coords.extend(section.get("coordinates") or [])
    for fp_data in (playback.get("footprints") or {}).values():
        for polygon in fp_data.get("polygons") or []:
            coords.extend(polygon)
    usable = [c for c in coords if isinstance(c, list) and len(c) >= 2]
    if not usable:
        return 0.0, 0.0
    lon = sum(float(c[0]) for c in usable) / len(usable)
    lat = sum(float(c[1]) for c in usable) / len(usable)
    return lon, lat


def _project_lonlat(coord: list[float], lon0: float, lat0: float) -> tuple[float, float]:
    lon, lat = float(coord[0]), float(coord[1])
    x = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * EARTH_RADIUS_M
    return x, y


def _unproject_xy(coord: tuple[float, float], lon0: float, lat0: float) -> list[float]:
    x, y = coord
    lon = lon0 + math.degrees(x / (EARTH_RADIUS_M * max(0.000001, math.cos(math.radians(lat0)))))
    lat = lat0 + math.degrees(y / EARTH_RADIUS_M)
    return [lon, lat]


def _polygon_geom(coords: list[list[float]], lon0: float, lat0: float) -> Any | None:
    if Polygon is None or len(coords) < 3:
        return None
    pts = [_project_lonlat(c, lon0, lat0) for c in coords]
    if pts[0] != pts[-1]:
        pts.append(pts[0])
    try:
        geom = Polygon(pts)
        if not geom.is_valid:
            geom = geom.buffer(0)
        if geom.is_empty or geom.area <= 0:
            return None
        return geom
    except Exception:
        return None


def _section_geom(section: dict[str, Any], lon0: float, lat0: float) -> Any | None:
    coords = section.get("coordinates") or []
    if section.get("sectionType") == "area":
        return _polygon_geom(coords, lon0, lat0)
    if LineString is None or len(coords) < 2:
        return None
    try:
        pts = [_project_lonlat(c, lon0, lat0) for c in coords]
        line = LineString(pts)
        width = float(section.get("widthM") or 0.0)
        buffer_m = max(1.0, width / 2.0)
        geom = line.buffer(buffer_m, cap_style=2, join_style=2)
        if not geom.is_valid:
            geom = geom.buffer(0)
        if geom.is_empty or geom.area <= 0:
            return None
        return geom
    except Exception:
        return None


def _geometry_to_geojson(geom: Any, lon0: float, lat0: float) -> dict[str, Any] | None:
    if geom is None or getattr(geom, "is_empty", True):
        return None

    def _ring_to_lonlat(ring: Any) -> list[list[float]]:
        return [_unproject_xy((float(x), float(y)), lon0, lat0) for x, y in ring.coords]

    if geom.geom_type == "Polygon":
        rings = [_ring_to_lonlat(geom.exterior)]
        rings.extend(_ring_to_lonlat(interior) for interior in geom.interiors)
        return {"type": "Polygon", "coordinates": rings}
    if geom.geom_type == "MultiPolygon":
        polygons = []
        for poly in geom.geoms:
            rings = [_ring_to_lonlat(poly.exterior)]
            rings.extend(_ring_to_lonlat(interior) for interior in poly.interiors)
            polygons.append(rings)
        return {"type": "MultiPolygon", "coordinates": polygons}
    return None


def _fallback_section_geometry(section: dict[str, Any]) -> dict[str, Any] | None:
    coords = section.get("coordinates") or []
    if section.get("sectionType") == "area" and len(coords) >= 3:
        ring = [list(c[:2]) for c in coords]
        if ring[0] != ring[-1]:
            ring.append(ring[0])
        return {"type": "Polygon", "coordinates": [ring]}
    if len(coords) >= 2:
        return {"type": "LineString", "coordinates": [list(c[:2]) for c in coords]}
    return None


def _compute_footprint_coverage(
    sections: list[dict[str, Any]],
    playback: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    if not sections:
        return {
            "available": False,
            "reason": "No input mission sections",
            "sections": [],
            "summary": {"sectionCount": 0, "coveredPercent": 0.0},
        }, None

    lon0, lat0 = _projection_origin(sections, playback)
    coverage_rows: list[dict[str, Any]] = []
    features: list[dict[str, Any]] = []

    if Polygon is None or unary_union is None:
        for section in sections:
            geom_json = _fallback_section_geometry(section)
            if geom_json is None:
                continue
            props = _section_properties(section, 0.0, None, None, {})
            features.append({"type": "Feature", "geometry": geom_json, "properties": props})
            coverage_rows.append(props)
        return {
            "available": False,
            "reason": "Shapely is not available",
            "sections": coverage_rows,
            "summary": {"sectionCount": len(coverage_rows), "coveredPercent": 0.0},
        }, {"type": "FeatureCollection", "features": features}

    footprint_by_aircraft: dict[str, list[Any]] = {}
    all_footprints: list[Any] = []
    for aid, fp_data in (playback.get("footprints") or {}).items():
        for polygon in fp_data.get("polygons") or []:
            geom = _polygon_geom(polygon, lon0, lat0)
            if geom is None:
                continue
            footprint_by_aircraft.setdefault(str(aid), []).append(geom)
            all_footprints.append(geom)

    all_union = unary_union(all_footprints) if all_footprints else None
    by_aircraft_union = {
        aid: unary_union(geoms)
        for aid, geoms in footprint_by_aircraft.items()
        if geoms
    }

    total_area = 0.0
    total_covered = 0.0
    for section in sections:
        geom = _section_geom(section, lon0, lat0)
        if geom is None:
            geom_json = _fallback_section_geometry(section)
            if geom_json is None:
                continue
            props = _section_properties(section, 0.0, None, None, {})
            features.append({"type": "Feature", "geometry": geom_json, "properties": props})
            coverage_rows.append(props)
            continue

        area_m2 = float(geom.area)
        covered_m2 = float(geom.intersection(all_union).area) if all_union is not None else 0.0
        coverage_percent = (covered_m2 / area_m2 * 100.0) if area_m2 > 0 else 0.0
        by_aircraft: dict[str, float] = {}
        for aid, union_geom in by_aircraft_union.items():
            aircraft_covered = float(geom.intersection(union_geom).area) if union_geom is not None else 0.0
            by_aircraft[aid] = round((aircraft_covered / area_m2 * 100.0) if area_m2 > 0 else 0.0, 2)

        total_area += area_m2
        total_covered += min(covered_m2, area_m2)
        props = _section_properties(section, coverage_percent, area_m2, covered_m2, by_aircraft)
        geom_json = _geometry_to_geojson(geom, lon0, lat0)
        if geom_json:
            features.append({"type": "Feature", "geometry": geom_json, "properties": props})
        coverage_rows.append(props)

    weighted = (total_covered / total_area * 100.0) if total_area > 0 else 0.0
    values = [r.get("coveragePercent", 0.0) for r in coverage_rows]
    return {
        "available": bool(all_footprints),
        "reason": None if all_footprints else "No footprint samples",
        "sections": coverage_rows,
        "summary": {
            "sectionCount": len(coverage_rows),
            "coveredPercent": round(weighted, 2),
            "minPercent": round(min(values), 2) if values else 0.0,
            "maxPercent": round(max(values), 2) if values else 0.0,
            "footprintSamples": sum(len(v.get("polygons") or []) for v in (playback.get("footprints") or {}).values()),
            "areaM2": round(total_area, 2),
            "coveredM2": round(total_covered, 2),
        },
    }, {"type": "FeatureCollection", "features": features} if features else None


def _section_properties(
    section: dict[str, Any],
    coverage_percent: float,
    area_m2: float | None,
    covered_m2: float | None,
    by_aircraft: dict[str, float],
) -> dict[str, Any]:
    return {
        "sectionId": section.get("sectionId"),
        "inputMissionID": section.get("inputMissionID"),
        "missionType": section.get("inputMissionType"),
        "sectionType": section.get("sectionType"),
        "widthM": section.get("widthM"),
        "coveragePercent": round(max(0.0, min(100.0, coverage_percent)), 2),
        "areaM2": round(area_m2, 2) if area_m2 is not None else None,
        "coveredM2": round(covered_m2, 2) if covered_m2 is not None else None,
        "byAircraft": by_aircraft,
    }


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def _looks_like_scenario_dir(path: Path) -> bool:
    if not path.is_dir():
        return False
    sbc3 = path / "SBC3"
    if not sbc3.is_dir():
        return False
    return any(
        (sbc3 / name).is_dir()
        for name in ("MissionPlan", "IndividualMissionPlan", "FlightPath", "DSS_Internal")
    )


def _scenario_timestamp_text(path: Path) -> str:
    if path.name.startswith("Scenario_"):
        ts_str = path.name.replace("Scenario_", "")
        try:
            return datetime.strptime(ts_str, "%Y-%m-%dT%H%M%S").isoformat()
        except Exception:
            return ts_str
    try:
        return datetime.fromtimestamp(path.stat().st_mtime).isoformat()
    except Exception:
        return path.name


def list_scenarios(logs_dir: Path) -> list[dict[str, Any]]:
    """Scan *logs_dir* for scenario-like folders that contain ``SBC3/``.

    Returns a list sorted newest-first with keys: name, path, timestamp.
    """
    results: list[dict[str, Any]] = []
    if not logs_dir.exists():
        return results

    for entry in logs_dir.iterdir():
        if not _looks_like_scenario_dir(entry):
            continue
        results.append({
            "name": entry.name,
            "path": str(entry),
            "timestamp": _scenario_timestamp_text(entry),
        })

    results.sort(key=lambda r: r["timestamp"] or "", reverse=True)
    return results


def parse_scenario(scenario_path: Path) -> dict[str, Any]:
    """Parse all data from a single scenario folder and return a dict
    suitable for the frontend.

    Handles missing files gracefully — returns empty data, never crashes.
    """
    base = scenario_path / "SBC3"
    if not base.is_dir():
        return {"ok": False, "error": "SBC3 subfolder not found", "scenario": scenario_path.name}

    # ------------------------------------------------------------------
    # 1. MissionPlan/*.json
    # ------------------------------------------------------------------
    mission_plans: list[dict[str, Any]] = []
    mission_plan_dir = base / "MissionPlan"
    if mission_plan_dir.is_dir():
        for fp in sorted(mission_plan_dir.glob("*.json")):
            data = _load_json_dict(fp)
            if data:
                mission_plans.append(data)
    mission_plans.sort(key=lambda p: _coerce_int(p.get("missionPlanID") or p.get("MissionPlanID")) or 0)

    # ------------------------------------------------------------------
    # 2. IndividualMissionPlan/*.json — index by IMP ID
    # ------------------------------------------------------------------
    individual_plans: dict[int, dict[str, Any]] = {}
    imp_dir = base / "IndividualMissionPlan"
    if imp_dir.is_dir():
        for fp in sorted(imp_dir.glob("*.json")):
            data = _load_json_dict(fp)
            if not data:
                continue
            imp_id = _coerce_int(
                data.get("individualMissionPackageID")
                or data.get("IndividualMissionPackageID")
                or data.get("individualMissionPackageId")
            )
            if imp_id is None:
                imp_id = _coerce_int(fp.stem)
            if imp_id is not None:
                individual_plans[imp_id] = data

    # ------------------------------------------------------------------
    # 3. FlightPath/*.json — index by pathID, build GeoJSON
    # ------------------------------------------------------------------
    flight_paths: dict[int, dict[str, Any]] = {}
    flight_path_features: list[dict[str, Any]] = []
    fp_dir = base / "FlightPath"
    if fp_dir.is_dir():
        for fp in sorted(fp_dir.glob("*.json")):
            data = _load_json_dict(fp)
            if not data:
                continue
            path_id = _coerce_int(data.get("pathID") or data.get("PathID"))
            if path_id is None:
                path_id = _coerce_int(fp.stem)
            if path_id is not None:
                waypoints = _extract_waypoints(data)
                ordered = _order_waypoints(waypoints)
                coords: list[list[float]] = []
                for wp in ordered:
                    if not isinstance(wp, dict):
                        continue
                    c = _extract_coord(wp)
                    if c is None:
                        continue
                    lat, lon, alt = c
                    coords.append([lon, lat] if alt is None else [lon, lat, alt])
                flight_paths[path_id] = {
                    "raw": data,
                    "coordinates": coords,
                }
                geojson = _build_flight_path_geojson(data)
                if geojson is not None:
                    flight_path_features.append(geojson)

    # ------------------------------------------------------------------
    # 4. InputMissionPlan/*.json
    # ------------------------------------------------------------------
    input_mission_plan = _load_first_json_dict(base / "InputMissionPlan", ("100", "1"))
    if not input_mission_plan:
        # Fallback: try root Logs directory level
        input_mission_plan = _load_first_json_dict(scenario_path / "InputMissionPlan", ("100", "1"))

    # ------------------------------------------------------------------
    # 5. MissionReferenceInfo/*.json
    # ------------------------------------------------------------------
    mission_ref = _load_first_json_dict(base / "MissionReferenceInfo", ("0", "1"))
    if not mission_ref:
        mission_ref = _load_first_json_dict(scenario_path / "MissionReferenceInfo", ("0", "1"))

    # ------------------------------------------------------------------
    # 6. DSS_Internal data
    # ------------------------------------------------------------------
    dss = base / "DSS_Internal"

    # 6a. replan_request_transport/*.json — each file is a JSON array
    replan_entries: list[dict[str, Any]] = []
    replan_dir = dss / "replan_request_transport"
    if replan_dir.is_dir():
        for fp in sorted(replan_dir.glob("*.json")):
            data = _load_json(fp)
            if isinstance(data, list):
                for item in data:
                    if isinstance(item, dict):
                        replan_entries.append(item)
            elif isinstance(data, dict):
                replan_entries.append(data)
    replan_entries.sort(key=lambda r: r.get("timestamp") or r.get("Timestamp") or 0)

    # 6b. DSS_Internal/missionPlan_*.json — metadata snapshots
    dss_mission_plans: dict[int, dict[str, Any]] = {}
    if dss.is_dir():
        for fp in sorted(dss.glob("missionPlan_*.json")):
            data = _load_json_dict(fp)
            if not data:
                continue
            mp_id = _coerce_int(data.get("missionPlanID") or data.get("MissionPlanID"))
            if mp_id is None:
                # Try parsing from filename: missionPlan_700000001.json
                stem = fp.stem
                if stem.startswith("missionPlan_"):
                    mp_id = _coerce_int(stem.replace("missionPlan_", ""))
            if mp_id is not None:
                dss_mission_plans[mp_id] = data

    # 6c. Single-file DSS_Internal data
    coverage_progress = _load_json_dict(dss / "coverage_progress.json")
    id_tracker = _load_json_dict(dss / "id_tracker.json")
    sweep_progress = _load_json_dict(dss / "sweep_progress.json")
    target_info = _load_json_dict(dss / "targetInfo.json")

    # ------------------------------------------------------------------
    # 7. VehicleStatus/status.json
    # ------------------------------------------------------------------
    vehicle_status = _load_json_dict(base / "VehicleStatus" / "status.json")

    # ------------------------------------------------------------------
    # 8. Build GeoJSON features from input mission & reference
    # ------------------------------------------------------------------
    reference_features: list[dict[str, Any]] = []

    # Mission areas from input plan
    if input_mission_plan:
        mission_area_list = input_mission_plan.get("missionAreaList") or input_mission_plan.get("MissionAreaList") or []
        reference_features.extend(_build_area_features(mission_area_list, "missionArea"))

    # Flight/prohibited areas from reference
    if mission_ref:
        flight_area_list = mission_ref.get("flightAreaList") or mission_ref.get("FlightAreaList") or []
        reference_features.extend(_build_area_features(flight_area_list, "flightArea"))

        prohibited_area_list = mission_ref.get("prohibitedAreaList") or mission_ref.get("ProhibitedAreaList") or []
        reference_features.extend(_build_area_features(prohibited_area_list, "prohibitedArea"))

        # TakeOver / HandOver points
        take_over_list = mission_ref.get("takeOverInfoList") or mission_ref.get("TakeOverInfoList") or []
        reference_features.extend(_build_point_features(take_over_list, "takeOver"))

        hand_over_list = mission_ref.get("handOverInfoList") or mission_ref.get("HandOverInfoList") or []
        reference_features.extend(_build_point_features(hand_over_list, "handOver"))

    # ------------------------------------------------------------------
    # 9. 0401 playback tracks, footprints, and mission-section coverage
    # ------------------------------------------------------------------
    playback_0401 = _parse_0401_playback(base)
    mission_sections = _extract_input_mission_sections(input_mission_plan)
    footprint_coverage, mission_section_features = _compute_footprint_coverage(mission_sections, playback_0401)

    # ------------------------------------------------------------------
    # 10. Resolve each MissionPlan's full tree
    # ------------------------------------------------------------------
    resolved_plans: list[dict[str, Any]] = []
    for plan in mission_plans:
        mp_id = _coerce_int(plan.get("missionPlanID") or plan.get("MissionPlanID"))
        aircraft_list = plan.get("aircraftList") or []
        aircraft_map: dict[int, dict[str, Any]] = {}

        for ac_entry in aircraft_list:
            if not isinstance(ac_entry, dict):
                continue
            ac_id = _coerce_int(ac_entry.get("aircraftID") or ac_entry.get("AircraftID"))
            if ac_id is None:
                continue
            imp_id = _coerce_int(
                ac_entry.get("individualMissionPackageID")
                or ac_entry.get("individualMissionPackageId")
                or ac_entry.get("IndividualMissionPackageID")
            )

            missions: list[dict[str, Any]] = []
            paths: list[dict[str, Any]] = []

            imp_data = individual_plans.get(imp_id) if imp_id is not None else None
            if imp_data:
                for mission in imp_data.get("individualMissionList") or []:
                    if not isinstance(mission, dict):
                        continue
                    m_id = _coerce_int(mission.get("individualMissionID") or mission.get("IndividualMissionID"))
                    m_info = mission.get("individualMissionInfo") or mission.get("IndividualMissionInfo") or {}
                    m_type = _coerce_int(
                        m_info.get("individualMissionType")
                        or m_info.get("IndividualMissionType")
                        or mission.get("missionType")
                        or mission.get("MissionType")
                    )
                    m_pattern = _coerce_int(m_info.get("patternType") or m_info.get("PatternType"))
                    m_target_id = _coerce_int(m_info.get("targetID") or m_info.get("TargetID"))
                    pid = _coerce_int(mission.get("pathID") or mission.get("PathID"))
                    is_done = _coerce_bool(mission.get("isDone"), False)
                    related = mission.get("relatedMission") or mission.get("RelatedMission") or {}
                    input_mission_id = _coerce_int(related.get("inputMissionID") or related.get("InputMissionID"))
                    prior_mission_id = _coerce_int(related.get("priorMissionID") or related.get("PriorMissionID"))
                    coord_list_raw = m_info.get("coordinateList") or m_info.get("CoordinateList") or []
                    coord_list = _coord_list_to_geojson_coords(coord_list_raw) if coord_list_raw else []
                    missions.append({
                        "id": m_id,
                        "type": m_type,
                        "patternType": m_pattern,
                        "targetID": m_target_id,
                        "pathID": pid,
                        "isDone": is_done,
                        "inputMissionID": input_mission_id,
                        "priorMissionID": prior_mission_id,
                        "coordinateList": coord_list,
                    })
                    if pid is not None and pid in flight_paths:
                        paths.append({
                            "pathID": pid,
                            "coordinates": flight_paths[pid]["coordinates"],
                        })

            aircraft_map[ac_id] = {
                "impID": imp_id,
                "missions": missions,
                "paths": paths,
            }

        resolved_plans.append({
            "missionPlanID": mp_id,
            "plan": plan,
            "resolved": {"aircraft": aircraft_map},
        })

    # ------------------------------------------------------------------
    # 10. Build timeline by interleaving plans and replans
    # ------------------------------------------------------------------
    timeline: list[dict[str, Any]] = []

    for rp in resolved_plans:
        plan = rp.get("plan") or {}
        ts = plan.get("timestamp") or plan.get("Timestamp") or 0
        timeline.append({
            "type": "missionPlan",
            "timestamp": ts,
            "missionPlanID": rp["missionPlanID"],
            "data": rp,
        })

    for replan in replan_entries:
        ts = replan.get("timestamp") or replan.get("Timestamp") or 0
        # Try to link to resulting missionPlanID from optionList
        linked_mp_id = None
        option_list = replan.get("optionList") or replan.get("OptionList") or []
        for opt in option_list:
            if isinstance(opt, dict):
                candidate = _coerce_int(opt.get("missionPlanID") or opt.get("MissionPlanID"))
                if candidate is not None:
                    linked_mp_id = candidate
                    break
        timeline.append({
            "type": "replan",
            "timestamp": ts,
            "linkedMissionPlanID": linked_mp_id,
            "data": replan,
        })

    timeline.sort(key=lambda t: t.get("timestamp") or 0)

    # ------------------------------------------------------------------
    # 11. Mark selected/applied plans
    # ------------------------------------------------------------------
    # The plan referenced as sourceMissionPlanID by the NEXT replan is
    # the one that was actually selected/applied from an option set.
    selected_plan_ids: set[int] = set()
    for entry in timeline:
        if entry["type"] != "replan":
            continue
        src_id = _coerce_int(
            (entry.get("data") or {}).get("replanDetail", {}).get("sourceMissionPlanID")
            or (entry.get("data") or {}).get("replanDetail", {}).get("currentMissionPlanID")
        )
        if src_id is not None:
            selected_plan_ids.add(src_id)

    # Also mark the very last mission plan (it's the currently active one)
    last_plan_id = None
    for entry in reversed(timeline):
        if entry["type"] == "missionPlan":
            last_plan_id = entry.get("missionPlanID")
            break
    if last_plan_id is not None:
        selected_plan_ids.add(last_plan_id)

    for entry in timeline:
        if entry["type"] == "missionPlan":
            entry["isSelected"] = entry.get("missionPlanID") in selected_plan_ids

    # ------------------------------------------------------------------
    # Assemble result
    # ------------------------------------------------------------------
    return {
        "ok": True,
        "scenario": scenario_path.name,
        "missionPlans": resolved_plans,
        "missionPlanCount": len(resolved_plans),
        "individualPlans": {str(k): v for k, v in individual_plans.items()},
        "flightPaths": {
            str(k): {
                "coordinates": v["coordinates"],
                "waypoints": _extract_waypoint_details(v.get("raw") or {}),
            }
            for k, v in flight_paths.items()
        },
        "flightPathFeatures": flight_path_features,
        "flightPathCount": len(flight_paths),
        "inputMissionPlan": input_mission_plan,
        "missionReferenceInfo": mission_ref,
        "referenceFeatures": {"type": "FeatureCollection", "features": reference_features} if reference_features else None,
        "missionSectionFeatures": mission_section_features,
        "footprintCoverage": footprint_coverage,
        "playback0401": playback_0401,
        "replanRequests": replan_entries,
        "dssMissionPlans": {str(k): v for k, v in dss_mission_plans.items()},
        "coverageProgress": coverage_progress,
        "idTracker": id_tracker,
        "sweepProgress": sweep_progress,
        "targetInfo": target_info,
        "vehicleStatus": vehicle_status,
        "timeline": timeline,
        "tracks": playback_0401.get("tracks") or {},
    }
