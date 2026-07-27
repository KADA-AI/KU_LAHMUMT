from __future__ import annotations

import json
import math
import re
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

try:
    from modules.common.turn_dynamics import (
        bank_angle_for_turn_radius_deg,
        interpolate_reference_turn_radius,
        turn_radius_from_rate_m,
    )
except ModuleNotFoundError:  # pragma: no cover - support direct script execution.
    _PROJECT_ROOT = Path(__file__).resolve().parents[2]
    if str(_PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(_PROJECT_ROOT))
    from modules.common.turn_dynamics import (
        bank_angle_for_turn_radius_deg,
        interpolate_reference_turn_radius,
        turn_radius_from_rate_m,
    )


REPORT_SCHEMA_VERSION = "logAnalyzer.dynamics.v1"
REPORT_DIR_NAME = "LogAnalyzer_Reports"
PLANNER_TURN_RADIUS_SPEEDS_MPS = (30.0, 40.0, 50.0)
PLANNER_TURN_RADIUS_WINDOW_MPS = 5.0
PLANNER_TURN_RADIUS_MIN_OBSERVED_SAMPLES = 50
ROLL_FIELD_RE = re.compile(r"\b(roll|bank|bankAngle|attitudeRoll|phi)\b", re.IGNORECASE)
SIGNAL_PATTERNS: dict[str, re.Pattern[str]] = {
    "roll_or_bank": re.compile(r"\b(roll|bank|bankAngle|attitudeRoll|phi)\b", re.IGNORECASE),
    "pitch": re.compile(r"\bpitch\b", re.IGNORECASE),
    "yaw": re.compile(r"\byaw\b", re.IGNORECASE),
    "heading": re.compile(r"\bheading\b", re.IGNORECASE),
    "turn_radius": re.compile(r"\b(turnRadius|radius|loiterProperty|minTurnRadius)\b", re.IGNORECASE),
    "flight_mode": re.compile(r"\b(flightMode|flightModeCommand)\b", re.IGNORECASE),
    "waypoint": re.compile(r"\b(currentWaypointID|waypointID|startWaypointID|pathID)\b", re.IGNORECASE),
    "target": re.compile(r"\b(targetFollowing|targetTracking|targetID)\b", re.IGNORECASE),
    "sensor": re.compile(r"\b(sensorInfo|filmingModeCommand|footprintCornerList|fov|fieldOfView)\b", re.IGNORECASE),
    "replan": re.compile(r"\b(replan|Replan|0902|pathDeviation|turn_radius)\b", re.IGNORECASE),
}


@dataclass
class FlightSample:
    t_ms: int
    aircraft_id: int
    is_uav: bool
    lat: float
    lon: float
    alt: float
    speed: float
    heading: float
    flight_mode: int | None = None
    flying: int | None = None
    waypoint_id: int | None = None
    target_id: int | None = None
    sensor_mode: int | None = None
    filming: int | None = None
    fov: float | None = None
    roll: float | None = None
    pitch: float | None = None
    yaw: float | None = None


def _base_dir(scenario_path: Path) -> Path:
    path = Path(scenario_path)
    return path / "SBC3" if (path / "SBC3").is_dir() else path


def _num(value: Any, default: float | None = None) -> float | None:
    try:
        if value is None:
            return default
        value = float(value)
        return value if math.isfinite(value) else default
    except Exception:
        return default


def _int(value: Any, default: int | None = None) -> int | None:
    try:
        if value is None:
            return default
        return int(float(value))
    except Exception:
        return default


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def _wrap_delta_deg(value: float) -> float:
    return ((float(value) + 180.0) % 360.0) - 180.0


def _percentile(values: Iterable[float], pct: float, default: float = 0.0) -> float:
    seq = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not seq:
        return float(default)
    if len(seq) == 1:
        return seq[0]
    pos = _clamp(float(pct), 0.0, 100.0) / 100.0 * (len(seq) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return seq[lo]
    alpha = pos - lo
    return seq[lo] * (1.0 - alpha) + seq[hi] * alpha


def _stats(values: Iterable[float], *, digits: int = 3) -> dict[str, float | int]:
    seq = [float(v) for v in values if math.isfinite(float(v))]
    if not seq:
        return {"count": 0}
    return {
        "count": len(seq),
        "min": round(min(seq), digits),
        "p25": round(_percentile(seq, 25.0), digits),
        "p50": round(_percentile(seq, 50.0), digits),
        "p75": round(_percentile(seq, 75.0), digits),
        "p90": round(_percentile(seq, 90.0), digits),
        "p95": round(_percentile(seq, 95.0), digits),
        "max": round(max(seq), digits),
    }


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8-sig")
    except UnicodeDecodeError:
        return path.read_text(encoding="cp949", errors="ignore")


def _messages_from_text(raw: str) -> list[dict[str, Any]]:
    raw = raw.strip()
    if not raw:
        return []
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        messages: list[dict[str, Any]] = []
        for line in raw.splitlines():
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
    if isinstance(data, list):
        return [item for item in data if isinstance(item, dict)]
    if isinstance(data, dict):
        for key in ("messages", "items", "data", "records"):
            nested = data.get(key)
            if isinstance(nested, list):
                return [item for item in nested if isinstance(item, dict)]
        return [data]
    return []


def _json_messages(path: Path) -> list[dict[str, Any]]:
    try:
        return _messages_from_text(_read_text(path))
    except Exception:
        return []


def _message_files(base: Path, message_id: str) -> list[Path]:
    folder = base / message_id
    if not folder.is_dir():
        return []
    return sorted(p for p in folder.glob(f"{message_id}*.json") if p.is_file())


def _find_numeric_deep(value: Any, wanted: set[str], depth: int = 0) -> float | None:
    if depth > 5:
        return None
    if isinstance(value, dict):
        for key, item in value.items():
            if str(key).lower() in wanted:
                number = _num(item)
                if number is not None:
                    return number
        for item in value.values():
            found = _find_numeric_deep(item, wanted, depth + 1)
            if found is not None:
                return found
    elif isinstance(value, list):
        for item in value[:8]:
            found = _find_numeric_deep(item, wanted, depth + 1)
            if found is not None:
                return found
    return None


def _coord(agent: dict[str, Any]) -> tuple[float, float, float] | None:
    coord = agent.get("coordinate") or agent.get("Coordinate")
    if not isinstance(coord, dict):
        return None
    lat = _num(coord.get("latitude") if "latitude" in coord else coord.get("Latitude"))
    lon = _num(coord.get("longitude") if "longitude" in coord else coord.get("Longitude"))
    alt = _num(coord.get("altitude") if "altitude" in coord else coord.get("Altitude"), 0.0)
    if lat is None or lon is None:
        return None
    return float(lat), float(lon), float(alt or 0.0)


def _agent_sample(message: dict[str, Any], agent: dict[str, Any]) -> FlightSample | None:
    t_ms = _int(message.get("timestamp") or message.get("Timestamp"))
    aircraft_id = _int(agent.get("aircraftID") or agent.get("AircraftID"))
    coord = _coord(agent)
    if t_ms is None or aircraft_id is None or coord is None:
        return None

    velocity = agent.get("velocity") or agent.get("Velocity") or {}
    if not isinstance(velocity, dict):
        velocity = {}
    speed = _num(velocity.get("speed") if "speed" in velocity else velocity.get("Speed"), 0.0) or 0.0
    heading = _num(velocity.get("heading") if "heading" in velocity else velocity.get("Heading"), 0.0) or 0.0
    unmanned = agent.get("unmannedInfo") or agent.get("UnmannedInfo") or {}
    if not isinstance(unmanned, dict):
        unmanned = {}
    sensor = unmanned.get("sensorInfo") or unmanned.get("SensorInfo") or {}
    if not isinstance(sensor, dict):
        sensor = {}
    waypoint = unmanned.get("currentWaypointID") or unmanned.get("CurrentWaypointID")
    waypoint_id = None
    if isinstance(waypoint, dict):
        waypoint_id = _int(waypoint.get("waypointID") or waypoint.get("WaypointID"))
    else:
        waypoint_id = _int(waypoint)
    target = unmanned.get("targetFollowing") or unmanned.get("TargetFollowing")
    target_id = _int(target.get("targetID") or target.get("TargetID")) if isinstance(target, dict) else None
    is_unmanned = agent.get("isUnmanned")
    is_uav = (4 <= aircraft_id <= 6) or bool(is_unmanned is True)
    lat, lon, alt = coord
    return FlightSample(
        t_ms=int(t_ms),
        aircraft_id=int(aircraft_id),
        is_uav=is_uav,
        lat=lat,
        lon=lon,
        alt=alt,
        speed=float(speed),
        heading=float(heading) % 360.0,
        flight_mode=_int(unmanned.get("flightMode") or unmanned.get("FlightMode")),
        flying=_int(unmanned.get("flying") or unmanned.get("Flying")),
        waypoint_id=waypoint_id,
        target_id=target_id,
        sensor_mode=_int(sensor.get("operationalMode") or sensor.get("OperationalMode")),
        filming=_int(sensor.get("filming") or sensor.get("Filming")),
        fov=_num(sensor.get("fov") if "fov" in sensor else sensor.get("Fov")),
        roll=_find_numeric_deep(agent, {"roll", "bank", "bankangle", "attituderoll", "phi"}),
        pitch=_find_numeric_deep(agent, {"pitch", "attitudepitch", "theta"}),
        yaw=_find_numeric_deep(agent, {"yaw", "attitudeyaw", "psi"}),
    )


def _load_samples(base: Path) -> tuple[dict[int, list[FlightSample]], list[str]]:
    files = _message_files(base, "0401")
    by_agent: dict[int, list[FlightSample]] = {}
    for file_path in files:
        for message in _json_messages(file_path):
            agents = message.get("agentStateList") or message.get("AgentStateList") or []
            if not isinstance(agents, list):
                continue
            for agent in agents:
                if not isinstance(agent, dict):
                    continue
                sample = _agent_sample(message, agent)
                if sample is None:
                    continue
                by_agent.setdefault(sample.aircraft_id, []).append(sample)

    for aircraft_id, samples in list(by_agent.items()):
        samples.sort(key=lambda item: item.t_ms)
        deduped: list[FlightSample] = []
        seen: set[int] = set()
        for sample in samples:
            if sample.t_ms in seen:
                continue
            deduped.append(sample)
            seen.add(sample.t_ms)
        by_agent[aircraft_id] = deduped
    return by_agent, [str(path) for path in files]


def _project(samples: list[FlightSample]) -> list[tuple[FlightSample, float, float]]:
    if not samples:
        return []
    lat0 = _percentile([s.lat for s in samples], 50.0, samples[0].lat)
    lon0 = _percentile([s.lon for s in samples], 50.0, samples[0].lon)
    lat_scale = 111_320.0
    lon_scale = 111_320.0 * max(0.05, math.cos(math.radians(lat0)))
    return [
        (sample, (sample.lon - lon0) * lon_scale, (sample.lat - lat0) * lat_scale)
        for sample in samples
    ]


def _phase_key(sample: FlightSample) -> str:
    mode = sample.flight_mode
    if sample.target_id is not None or mode == 9:
        base = "target-track"
    elif sample.sensor_mode in (1, 2, 3) or sample.filming:
        base = "imaging"
    elif sample.waypoint_id is not None or mode == 7:
        base = "path-follow"
    elif mode is None:
        base = "unknown"
    else:
        base = f"mode-{mode}"
    if sample.flying == 2:
        return f"{base}:hold"
    return base


def _speed_bucket(speed: float) -> str:
    if speed < 20.0:
        return "<20"
    if speed < 35.0:
        return "20-35"
    if speed < 50.0:
        return "35-50"
    if speed < 70.0:
        return "50-70"
    return ">=70"


def _circumradius(p0: tuple[float, float], p1: tuple[float, float], p2: tuple[float, float]) -> float | None:
    a = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    b = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    c = math.hypot(p2[0] - p0[0], p2[1] - p0[1])
    if min(a, b, c) < 2.0:
        return None
    area2 = abs((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]))
    if area2 < 1e-3:
        return None
    radius = (a * b * c) / (2.0 * area2)
    return radius if 20.0 <= radius <= 20_000.0 else None


def _agent_label(aircraft_id: int) -> str:
    if 1 <= aircraft_id <= 3:
        return f"LAH{aircraft_id}"
    if 4 <= aircraft_id <= 6:
        return f"UAV{aircraft_id - 3}"
    return f"AC{aircraft_id}"


def _finalize_bucket(rows: dict[str, list[float]]) -> list[dict[str, Any]]:
    result = []
    for key in ("<20", "20-35", "35-50", "50-70", ">=70"):
        values = rows.get(key, [])
        if not values:
            continue
        result.append({"bucket": key, "turnRadiusM": _stats(values, digits=1)})
    return result


def _planner_turn_radius_config_key(speed_mps: float) -> str:
    return f"turn_radius_{int(round(float(speed_mps)))}_m"


def _finalize_planner_speed_targets(rows: dict[float, list[float]]) -> list[dict[str, Any]]:
    result = []
    for speed in PLANNER_TURN_RADIUS_SPEEDS_MPS:
        values = rows.get(speed, [])
        result.append(
            {
                "speedMps": speed,
                "configKey": _planner_turn_radius_config_key(speed),
                "windowMps": PLANNER_TURN_RADIUS_WINDOW_MPS,
                "turnRadiusM": _stats(values, digits=1),
            }
        )
    return result


def _recommended_planner_speed_table(uavs: list[dict[str, Any]], radius_scale: float) -> list[dict[str, Any]]:
    observed_by_speed: dict[float, list[float]] = {speed: [] for speed in PLANNER_TURN_RADIUS_SPEEDS_MPS}
    sample_count_by_speed: dict[float, int] = {speed: 0 for speed in PLANNER_TURN_RADIUS_SPEEDS_MPS}
    for row in uavs:
        for target in row.get("plannerSpeedTurnRadiusTable") or []:
            try:
                speed = float(target.get("speedMps"))
            except Exception:
                continue
            if speed not in observed_by_speed:
                continue
            stats = target.get("turnRadiusM") or {}
            if stats.get("count") and stats.get("p50") is not None:
                observed_by_speed[speed].append(float(stats.get("p50")))
                sample_count_by_speed[speed] += int(stats.get("count") or 0)

    table = []
    for speed in PLANNER_TURN_RADIUS_SPEEDS_MPS:
        reference = float(interpolate_reference_turn_radius(speed))
        scaled_reference = reference * float(radius_scale)
        observed_stats = _stats(observed_by_speed.get(speed, []), digits=1)
        observed_count = int(sample_count_by_speed.get(speed, 0))
        observed_agent_count = int(observed_stats.get("count") or 0)
        if (
            observed_count >= PLANNER_TURN_RADIUS_MIN_OBSERVED_SAMPLES
            and observed_agent_count >= 2
            and observed_stats.get("p50") is not None
        ):
            recommended = float(observed_stats["p50"])
            source = "observed_window"
        else:
            recommended = scaled_reference
            source = "scaled_reference"
            if observed_count:
                source = "scaled_reference_sparse_observed"
        table.append(
            {
                "speedMps": speed,
                "configKey": _planner_turn_radius_config_key(speed),
                "windowMps": PLANNER_TURN_RADIUS_WINDOW_MPS,
                "currentReferenceRadiusM": round(reference, 1),
                "scaledReferenceRadiusM": round(scaled_reference, 1),
                "observedSampleCount": observed_count,
                "observedAgentCount": observed_agent_count,
                "observedAgentP50RadiusM": observed_stats,
                "recommendedTurnRadiusM": round(recommended, 1),
                "source": source,
            }
        )
    return table


def _analyze_agent(samples: list[FlightSample]) -> dict[str, Any]:
    projected = _project(samples)
    if len(projected) < 3:
        return {"sampleCount": len(samples), "usable": False}

    dt_values: list[float] = []
    speed_values: list[float] = []
    ground_speeds: list[float] = []
    accel_values: list[float] = []
    vertical_rates: list[float] = []
    yaw_rates: list[float] = []
    yaw_accels: list[float] = []
    radii: list[float] = []
    geometry_radii: list[float] = []
    bank_proxy: list[float] = []
    direct_roll: list[float] = [float(s.roll) for s in samples if s.roll is not None and math.isfinite(float(s.roll))]
    direct_pitch: list[float] = [float(s.pitch) for s in samples if s.pitch is not None and math.isfinite(float(s.pitch))]
    phase_metrics: dict[str, dict[str, list[float]]] = {}
    bucket_radii: dict[str, list[float]] = {}
    planner_target_radii: dict[float, list[float]] = {speed: [] for speed in PLANNER_TURN_RADIUS_SPEEDS_MPS}
    aggressive_events: list[dict[str, Any]] = []
    reversals: list[float] = []
    last_yaw_rate: float | None = None
    last_turn_sign = 0
    last_turn_time_s: float | None = None
    active_event: dict[str, Any] | None = None
    distance_m = 0.0
    start_t = projected[0][0].t_ms
    end_t = projected[-1][0].t_ms

    for idx in range(1, len(projected)):
        prev, x0, y0 = projected[idx - 1]
        curr, x1, y1 = projected[idx]
        dt_s = (curr.t_ms - prev.t_ms) / 1000.0
        if dt_s <= 0.02 or dt_s > 10.0:
            continue
        dist = math.hypot(x1 - x0, y1 - y0)
        ground_speed = dist / dt_s
        speed = (max(0.0, prev.speed) + max(0.0, curr.speed)) * 0.5
        if speed <= 1.0 and ground_speed > 1.0:
            speed = ground_speed
        heading_delta = _wrap_delta_deg(curr.heading - prev.heading)
        yaw_rate = heading_delta / dt_s
        if abs(yaw_rate) > 160.0:
            continue
        distance_m += dist
        dt_values.append(dt_s)
        speed_values.append(speed)
        ground_speeds.append(ground_speed)
        accel_values.append((curr.speed - prev.speed) / dt_s)
        vertical_rates.append((curr.alt - prev.alt) / dt_s)
        yaw_rates.append(yaw_rate)
        if last_yaw_rate is not None:
            yaw_accels.append((yaw_rate - last_yaw_rate) / dt_s)
        last_yaw_rate = yaw_rate

        phase = _phase_key(curr)
        phase_row = phase_metrics.setdefault(
            phase,
            {"speed": [], "yaw": [], "radius": [], "bank": [], "accel": []},
        )
        phase_row["speed"].append(speed)
        phase_row["yaw"].append(abs(yaw_rate))
        phase_row["accel"].append(abs(accel_values[-1]))

        radius = None
        if speed > 5.0 and abs(yaw_rate) >= 0.2:
            candidate = turn_radius_from_rate_m(speed, yaw_rate)
            if candidate is None:
                continue
            if 25.0 <= candidate <= 20_000.0:
                radius = candidate
                radii.append(candidate)
                phase_row["radius"].append(candidate)
                bucket_radii.setdefault(_speed_bucket(speed), []).append(candidate)
                for target_speed in PLANNER_TURN_RADIUS_SPEEDS_MPS:
                    if abs(speed - target_speed) <= PLANNER_TURN_RADIUS_WINDOW_MPS:
                        planner_target_radii[target_speed].append(candidate)
                bank = bank_angle_for_turn_radius_deg(speed, candidate)
                if bank is None:
                    continue
                if 0.0 <= bank <= 85.0:
                    bank_proxy.append(bank)
                    phase_row["bank"].append(bank)

        if idx + 1 < len(projected):
            _next, x2, y2 = projected[idx + 1]
            geo = _circumradius((x0, y0), (x1, y1), (x2, y2))
            if geo is not None:
                geometry_radii.append(geo)

        turn_sign = 1 if yaw_rate > 1.2 else -1 if yaw_rate < -1.2 else 0
        now_s = (curr.t_ms - start_t) / 1000.0
        if turn_sign:
            if last_turn_sign and turn_sign != last_turn_sign and last_turn_time_s is not None:
                reversals.append(max(0.0, now_s - last_turn_time_s))
            last_turn_sign = turn_sign
            last_turn_time_s = now_s

        hard_turn = abs(yaw_rate) >= 5.0 or (radius is not None and radius <= 450.0)
        if hard_turn:
            if active_event is None:
                active_event = {
                    "aircraftId": curr.aircraft_id,
                    "label": _agent_label(curr.aircraft_id),
                    "startMs": curr.t_ms,
                    "endMs": curr.t_ms,
                    "durationS": 0.0,
                    "phase": phase,
                    "maxYawRateDps": abs(yaw_rate),
                    "minRadiusM": radius,
                    "maxBankProxyDeg": bank_proxy[-1] if bank_proxy else None,
                    "meanSpeedMps": [],
                    "turnAngleDeg": 0.0,
                }
            active_event["endMs"] = curr.t_ms
            active_event["durationS"] = round((curr.t_ms - int(active_event["startMs"])) / 1000.0, 3)
            active_event["maxYawRateDps"] = max(float(active_event["maxYawRateDps"]), abs(yaw_rate))
            active_event["turnAngleDeg"] = float(active_event["turnAngleDeg"]) + abs(heading_delta)
            active_event["meanSpeedMps"].append(speed)
            if radius is not None:
                active_event["minRadiusM"] = radius if active_event["minRadiusM"] is None else min(float(active_event["minRadiusM"]), radius)
            if bank_proxy:
                current_bank = bank_proxy[-1]
                active_event["maxBankProxyDeg"] = current_bank if active_event["maxBankProxyDeg"] is None else max(float(active_event["maxBankProxyDeg"]), current_bank)
        elif active_event is not None:
            speeds = active_event.pop("meanSpeedMps", [])
            active_event["meanSpeedMps"] = round(_percentile(speeds, 50.0), 2) if speeds else 0.0
            if float(active_event["durationS"]) >= 0.4:
                aggressive_events.append(active_event)
            active_event = None

    if active_event is not None:
        speeds = active_event.pop("meanSpeedMps", [])
        active_event["meanSpeedMps"] = round(_percentile(speeds, 50.0), 2) if speeds else 0.0
        if float(active_event["durationS"]) >= 0.4:
            aggressive_events.append(active_event)

    phase_rows = []
    for phase, row in sorted(phase_metrics.items()):
        phase_rows.append(
            {
                "phase": phase,
                "sampleCount": len(row["speed"]),
                "speedMps": _stats(row["speed"], digits=2),
                "absYawRateDps": _stats(row["yaw"], digits=3),
                "turnRadiusM": _stats(row["radius"], digits=1),
                "bankProxyDeg": _stats(row["bank"], digits=2),
                "absAccelMps2": _stats(row["accel"], digits=3),
            }
        )

    aggressive_events.sort(
        key=lambda item: (
            float(item.get("maxYawRateDps") or 0.0),
            -(float(item.get("minRadiusM") or 999999.0)),
        ),
        reverse=True,
    )

    abs_yaw = [abs(v) for v in yaw_rates]
    usable = len(radii) >= 20 and _percentile(speed_values, 50.0) > 5.0
    return {
        "sampleCount": len(samples),
        "durationS": round(max(0.0, (end_t - start_t) / 1000.0), 3),
        "distanceM": round(distance_m, 1),
        "usable": usable,
        "timeStepS": _stats(dt_values, digits=3),
        "speedMps": _stats(speed_values, digits=3),
        "groundSpeedMps": _stats(ground_speeds, digits=3),
        "accelMps2": _stats(accel_values, digits=3),
        "verticalRateMps": _stats(vertical_rates, digits=3),
        "yawRateDps": _stats(yaw_rates, digits=3),
        "absYawRateDps": _stats(abs_yaw, digits=3),
        "yawAccelDps2": _stats(yaw_accels, digits=3),
        "turnRadiusM": _stats(radii, digits=2),
        "geometryRadiusM": _stats(geometry_radii, digits=2),
        "bankProxyDeg": _stats(bank_proxy, digits=3),
        "directRollDeg": _stats(direct_roll, digits=3),
        "directPitchDeg": _stats(direct_pitch, digits=3),
        "oppositeTurnGapS": _stats(reversals, digits=3),
        "speedBuckets": _finalize_bucket(bucket_radii),
        "plannerSpeedTurnRadiusTable": _finalize_planner_speed_targets(planner_target_radii),
        "phaseMetrics": phase_rows,
        "aggressiveEvents": aggressive_events[:18],
        "rollObserved": bool(direct_roll),
    }


def _load_0602_commands(base: Path) -> list[dict[str, Any]]:
    commands: list[dict[str, Any]] = []
    for file_path in _message_files(base, "0602"):
        for msg in _json_messages(file_path):
            aircraft_id = _int(msg.get("aircraftID") or msg.get("AircraftID"))
            timestamp = _int(msg.get("timestamp") or msg.get("Timestamp"))
            if aircraft_id is None or timestamp is None:
                continue
            flight = msg.get("flightModeCommand") or msg.get("FlightModeCommand")
            filming = msg.get("filmingModeCommand") or msg.get("FilmingModeCommand")
            if isinstance(flight, dict):
                path_following = flight.get("pathFollowing") or flight.get("PathFollowing") or {}
                target_tracking = flight.get("targetTracking") or flight.get("TargetTracking") or {}
                commands.append(
                    {
                        "timestamp": timestamp,
                        "aircraftId": aircraft_id,
                        "type": "flightMode",
                        "flightMode": _int(flight.get("flightMode") or flight.get("FlightMode")),
                        "startWaypointId": _int(path_following.get("startWaypointID") or path_following.get("StartWaypointID"))
                        if isinstance(path_following, dict)
                        else None,
                        "targetId": _int(target_tracking.get("targetID") or target_tracking.get("TargetID"))
                        if isinstance(target_tracking, dict)
                        else None,
                    }
                )
            if isinstance(filming, dict):
                commands.append(
                    {
                        "timestamp": timestamp,
                        "aircraftId": aircraft_id,
                        "type": "filming",
                        "sensorMode": _int(filming.get("operationMode") or filming.get("OperationMode")),
                        "fov": _num(filming.get("fieldOfView") or filming.get("FieldOfView")),
                    }
                )
    commands.sort(key=lambda item: item["timestamp"])
    return commands


def _command_responses(commands: list[dict[str, Any]], samples_by_agent: dict[int, list[FlightSample]]) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    latencies: list[float] = []
    for command in commands:
        samples = samples_by_agent.get(int(command["aircraftId"]), [])
        hit: FlightSample | None = None
        for sample in samples:
            if sample.t_ms < int(command["timestamp"]):
                continue
            if sample.t_ms - int(command["timestamp"]) > 60_000:
                break
            if command["type"] == "flightMode":
                mode_match = command.get("flightMode") is not None and sample.flight_mode == command.get("flightMode")
                wp_match = command.get("startWaypointId") is not None and sample.waypoint_id == command.get("startWaypointId")
                target_match = command.get("targetId") is not None and sample.target_id == command.get("targetId")
                if mode_match or wp_match or target_match:
                    hit = sample
                    break
            elif command["type"] == "filming":
                sensor_match = command.get("sensorMode") is not None and sample.sensor_mode == command.get("sensorMode")
                filming_match = sample.filming is not None and sample.filming > 0
                fov_match = command.get("fov") is not None and sample.fov is not None and abs(float(sample.fov) - float(command["fov"])) <= 1.0
                if sensor_match or (filming_match and fov_match):
                    hit = sample
                    break
        latency = None
        if hit is not None:
            latency = (hit.t_ms - int(command["timestamp"])) / 1000.0
            latencies.append(latency)
        rows.append(
            {
                "aircraftId": command["aircraftId"],
                "label": _agent_label(int(command["aircraftId"])),
                "type": command["type"],
                "timestamp": command["timestamp"],
                "expected": {
                    key: value
                    for key, value in command.items()
                    if key not in {"timestamp", "aircraftId", "type"} and value is not None
                },
                "latencyS": round(latency, 3) if latency is not None else None,
                "matched": hit is not None,
            }
        )
    return {
        "commandCount": len(commands),
        "matchedCount": sum(1 for row in rows if row["matched"]),
        "latencyS": _stats(latencies, digits=3),
        "rows": rows[:80],
    }


def _scan_log_signals(base: Path) -> dict[str, Any]:
    counts = {key: 0 for key in SIGNAL_PATTERNS}
    files_by_folder: dict[str, int] = {}
    matched_files: dict[str, list[str]] = {key: [] for key in SIGNAL_PATTERNS}
    total_files = 0
    total_bytes = 0
    for path in sorted(base.rglob("*")):
        if not path.is_file() or path.suffix.lower() not in {".json", ".jsonl", ".log", ".txt"}:
            continue
        total_files += 1
        rel = path.relative_to(base).as_posix()
        folder = rel.split("/", 1)[0]
        files_by_folder[folder] = files_by_folder.get(folder, 0) + 1
        try:
            raw = path.read_text(encoding="utf-8", errors="ignore")[:2_000_000]
        except Exception:
            continue
        total_bytes += min(path.stat().st_size, 2_000_000)
        for key, pattern in SIGNAL_PATTERNS.items():
            matches = len(pattern.findall(raw))
            if matches:
                counts[key] += matches
                if len(matched_files[key]) < 12:
                    matched_files[key].append(rel)
    return {
        "totalFilesScanned": total_files,
        "sampledBytes": total_bytes,
        "filesByFolder": dict(sorted(files_by_folder.items())),
        "signalCounts": counts,
        "matchedFiles": matched_files,
        "rollSignalsAvailable": counts.get("roll_or_bank", 0) > 0,
    }


def _recommendations(agent_results: dict[str, dict[str, Any]]) -> dict[str, Any]:
    uavs = [row for key, row in agent_results.items() if key in {"4", "5", "6"} and row.get("usable")]
    if not uavs:
        uavs = [row for row in agent_results.values() if row.get("usable")]
    speeds = [float(row.get("speedMps", {}).get("p50", 0.0)) for row in uavs if row.get("speedMps", {}).get("count")]
    radii = [float(row.get("turnRadiusM", {}).get("p50", 0.0)) for row in uavs if row.get("turnRadiusM", {}).get("count")]
    yaw95 = [float(row.get("absYawRateDps", {}).get("p95", 0.0)) for row in uavs if row.get("absYawRateDps", {}).get("count")]
    bank95 = [float(row.get("bankProxyDeg", {}).get("p95", 0.0)) for row in uavs if row.get("bankProxyDeg", {}).get("count")]
    bank50 = [float(row.get("bankProxyDeg", {}).get("p50", 0.0)) for row in uavs if row.get("bankProxyDeg", {}).get("count")]
    reversal = [
        float(row.get("oppositeTurnGapS", {}).get("p50", 0.0))
        for row in uavs
        if row.get("oppositeTurnGapS", {}).get("count")
    ]
    median_speed = _percentile(speeds, 50.0, 40.0)
    median_radius = _percentile(radii, 50.0, 450.0)
    current_ref = float(interpolate_reference_turn_radius(median_speed))
    radius_scale = _clamp(median_radius / max(1.0, current_ref), 0.55, 1.85)
    yaw_rate_p95 = _percentile(yaw95, 75.0, 4.5)
    bank_p95 = _percentile(bank95, 75.0, 25.0)
    bank_p50 = _percentile(bank50, 50.0, 18.0)
    reversal_gap = _percentile([v for v in reversal if v > 0.0], 50.0, 0.0)
    if reversal_gap > 0.15 and bank_p50 > 1.0:
        roll_rate = _clamp((2.0 * bank_p50 / reversal_gap) * 1.15, 8.0, 55.0)
    else:
        roll_rate = _clamp(bank_p95 * 0.95, 12.0, 45.0)
    turn_bank_limit = _clamp(bank_p95 * 1.12, 18.0, 60.0)
    roll_limit = _clamp(max(turn_bank_limit + 5.0, 35.0), 35.0, 68.0)
    lookahead = _clamp(max(70.0, median_radius * 0.45), 70.0, 460.0)
    freeze_yaw = _clamp(max(35.0, median_radius * 0.14), 35.0, 180.0)
    return {
        "quality": "ok" if len(uavs) >= 2 else "warn",
        "basis": {
            "usableAircraft": len(uavs),
            "medianSpeedMps": round(median_speed, 3),
            "medianTurnRadiusM": round(median_radius, 2),
            "currentReferenceRadiusM": round(current_ref, 2),
            "absYawRateP95Dps": round(yaw_rate_p95, 3),
            "bankProxyP95Deg": round(bank_p95, 3),
            "oppositeTurnGapP50S": round(reversal_gap, 3),
        },
        "simDynamicsProfile": {
            "banked_turn_enabled": True,
            "bank_yaw_rate_blend": 0.85,
            "reference_turn_radius_scale": round(radius_scale, 4),
            "max_yaw_rate_dps": round(_clamp(yaw_rate_p95 * 1.18, 2.5, 35.0), 3),
            "max_roll_rate_dps": round(roll_rate, 3),
            "turn_bank_limit_deg": round(turn_bank_limit, 3),
            "roll_limit_deg": round(roll_limit, 3),
            "turn_roll_gain": 2.2,
            "use_reference_turn_radius": True,
        },
        "missionPlanningHints": {
            "plannerTurnRadiusScale": round(radius_scale, 4),
            "nominalTurnRadiusM": round(median_radius, 1),
            "conservativeTurnRadiusM": round(_percentile(radii, 75.0, median_radius), 1) if radii else None,
            "aggressiveTurnRadiusM": round(_percentile(radii, 25.0, median_radius), 1) if radii else None,
            "speedTurnRadiusTable": _recommended_planner_speed_table(uavs, radius_scale),
            "lookaheadM": round(lookahead, 1),
            "freezeYawDistanceM": round(freeze_yaw, 1),
            "notes": [
                "Use conservative radius for replan routes near hand-over, target-track, and post-attack reconnect.",
                "Use speedTurnRadiusTable for mission-planning turn_radius_30_m/40_m/50_m style settings.",
                "Direct roll is used when logs expose it; otherwise bankProxyDeg is derived from speed and turn radius.",
            ],
        },
    }


def _cohort(agent_results: dict[str, dict[str, Any]]) -> dict[str, Any]:
    all_rows = list(agent_results.values())
    uav_rows = [row for key, row in agent_results.items() if key in {"4", "5", "6"}]
    def collect(rows: list[dict[str, Any]], metric: str, field: str = "p50") -> list[float]:
        return [float(row.get(metric, {}).get(field, 0.0)) for row in rows if row.get(metric, {}).get("count")]

    return {
        "aircraftCount": len(all_rows),
        "uavCount": len(uav_rows),
        "usableUavCount": sum(1 for row in uav_rows if row.get("usable")),
        "durationS": _stats([float(row.get("durationS", 0.0)) for row in all_rows], digits=2),
        "speedMps": _stats(collect(uav_rows, "speedMps"), digits=2),
        "turnRadiusM": _stats(collect(uav_rows, "turnRadiusM"), digits=1),
        "absYawRateDps": _stats(collect(uav_rows, "absYawRateDps", "p95"), digits=3),
        "bankProxyDeg": _stats(collect(uav_rows, "bankProxyDeg", "p95"), digits=2),
        "rollObservedAircraft": [
            _agent_label(int(key))
            for key, row in agent_results.items()
            if row.get("rollObserved")
        ],
    }


def analyze_scenario_dynamics(scenario_path: Path) -> dict[str, Any]:
    scenario_path = Path(scenario_path)
    base = _base_dir(scenario_path)
    if not base.is_dir():
        return {"ok": False, "error": f"SBC3 folder not found: {scenario_path}", "scenario": scenario_path.name}
    samples_by_agent, source_files = _load_samples(base)
    agent_results = {
        str(aircraft_id): {
            "aircraftId": aircraft_id,
            "label": _agent_label(aircraft_id),
            "isUav": (4 <= aircraft_id <= 6) or bool(samples and samples[0].is_uav),
            **_analyze_agent(samples),
        }
        for aircraft_id, samples in sorted(samples_by_agent.items())
    }
    commands = _load_0602_commands(base)
    command_analysis = _command_responses(commands, samples_by_agent)
    all_events = []
    for row in agent_results.values():
        all_events.extend(row.get("aggressiveEvents") or [])
    all_events.sort(
        key=lambda item: (
            float(item.get("maxYawRateDps") or 0.0),
            -(float(item.get("minRadiusM") or 999999.0)),
        ),
        reverse=True,
    )
    signal_scan = _scan_log_signals(base)
    return {
        "ok": True,
        "scenario": scenario_path.name,
        "source": {
            "scenarioPath": str(scenario_path),
            "basePath": str(base),
            "files0401": source_files,
            "fileCount0401": len(source_files),
            "sampleCount0401": sum(len(samples) for samples in samples_by_agent.values()),
        },
        "cohort": _cohort(agent_results),
        "agents": agent_results,
        "commands": command_analysis,
        "events": all_events[:30],
        "logSignals": signal_scan,
        "recommendations": _recommendations(agent_results),
    }


def _fmt_report(value: Any, digits: int = 1, suffix: str = "") -> str:
    if value is None:
        return "-"
    try:
        number = float(value)
    except Exception:
        return str(value)
    if not math.isfinite(number):
        return "-"
    return f"{number:.{digits}f}{suffix}"


def _stat_report(obj: dict[str, Any] | None, key: str = "p50", digits: int = 1, suffix: str = "") -> str:
    if not obj or not obj.get("count"):
        return "-"
    return _fmt_report(obj.get(key), digits, suffix)


def _markdown_table(headers: list[str], rows: list[list[Any]]) -> str:
    if not rows:
        return "_No rows._\n"
    out = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        out.append("| " + " | ".join(str(cell).replace("\n", " ") for cell in row) + " |")
    return "\n".join(out) + "\n"


def _render_report_markdown(analysis: dict[str, Any], saved_at: str) -> str:
    source = analysis.get("source") or {}
    cohort = analysis.get("cohort") or {}
    rec = analysis.get("recommendations") or {}
    basis = rec.get("basis") or {}
    hints = rec.get("missionPlanningHints") or {}
    speed_turn_table = hints.get("speedTurnRadiusTable") or []
    profile = rec.get("simDynamicsProfile") or {}
    commands = analysis.get("commands") or {}
    agents = analysis.get("agents") or {}
    events = analysis.get("events") or []
    signals = analysis.get("logSignals") or {}

    lines = [
        f"# Dynamics Analysis - {analysis.get('scenario', '-')}",
        "",
        f"- Saved at: `{saved_at}`",
        f"- Scenario path: `{source.get('scenarioPath', '-')}`",
        f"- 0401 files: `{source.get('fileCount0401', 0)}`",
        f"- 0401 samples: `{source.get('sampleCount0401', 0)}`",
        f"- Quality: `{rec.get('quality', '-')}`",
        "",
        "## Cohort Summary",
        "",
        _markdown_table(
            ["Metric", "Value"],
            [
                ["Usable UAV", f"{cohort.get('usableUavCount', 0)} / {cohort.get('uavCount', 0)}"],
                ["Speed p50", _stat_report(cohort.get("speedMps"), "p50", 2, " m/s")],
                ["Turn radius p50", _stat_report(cohort.get("turnRadiusM"), "p50", 1, " m")],
                ["Yaw rate p95", _stat_report(cohort.get("absYawRateDps"), "p50", 3, " dps")],
                ["Bank proxy p95", _stat_report(cohort.get("bankProxyDeg"), "p50", 2, " deg")],
                ["Command latency p50", _stat_report(commands.get("latencyS"), "p50", 3, " s")],
            ],
        ),
        "## Planning / Runtime Fit",
        "",
        _markdown_table(
            ["Metric", "Value"],
            [
                ["Median speed", _fmt_report(basis.get("medianSpeedMps"), 2, " m/s")],
                ["Median turn radius", _fmt_report(basis.get("medianTurnRadiusM"), 1, " m")],
                ["Current reference radius", _fmt_report(basis.get("currentReferenceRadiusM"), 1, " m")],
                ["Planner turn radius scale", _fmt_report(hints.get("plannerTurnRadiusScale"), 4, "x")],
                ["Nominal turn radius", _fmt_report(hints.get("nominalTurnRadiusM"), 1, " m")],
                ["Conservative turn radius", _fmt_report(hints.get("conservativeTurnRadiusM"), 1, " m")],
                ["Aggressive turn radius", _fmt_report(hints.get("aggressiveTurnRadiusM"), 1, " m")],
                ["Lookahead", _fmt_report(hints.get("lookaheadM"), 1, " m")],
                ["Freeze yaw distance", _fmt_report(hints.get("freezeYawDistanceM"), 1, " m")],
            ],
        ),
        "## Recommended Speed Turn Radius Table",
        "",
        _markdown_table(
            ["Config", "Speed", "Current ref", "Observed samples", "Observed agents", "Observed R50", "Scaled ref", "Recommended", "Source"],
            [
                [
                    row.get("configKey", "-"),
                    _fmt_report(row.get("speedMps"), 0, " m/s"),
                    _fmt_report(row.get("currentReferenceRadiusM"), 1, " m"),
                    row.get("observedSampleCount", 0),
                    row.get("observedAgentCount", 0),
                    _stat_report(row.get("observedAgentP50RadiusM"), "p50", 1, " m"),
                    _fmt_report(row.get("scaledReferenceRadiusM"), 1, " m"),
                    _fmt_report(row.get("recommendedTurnRadiusM"), 1, " m"),
                    row.get("source", "-"),
                ]
                for row in speed_turn_table
            ],
        ),
        "## SIM-Compatible Dynamics Profile",
        "",
        "```json",
        json.dumps(profile, ensure_ascii=False, indent=2),
        "```",
        "",
    ]

    agent_rows: list[list[Any]] = []
    for agent in sorted(agents.values(), key=lambda item: int(item.get("aircraftId") or 0)):
        agent_rows.append([
            agent.get("label", "-"),
            "yes" if agent.get("isUav") else "no",
            agent.get("sampleCount", 0),
            _fmt_report(agent.get("durationS"), 1, " s"),
            _stat_report(agent.get("speedMps"), "p50", 1, " m/s"),
            _stat_report(agent.get("turnRadiusM"), "p50", 0, " m"),
            _stat_report(agent.get("turnRadiusM"), "p75", 0, " m"),
            _stat_report(agent.get("absYawRateDps"), "p95", 2, " dps"),
            _stat_report(agent.get("bankProxyDeg"), "p95", 1, " deg"),
            "direct" if agent.get("rollObserved") else "proxy",
        ])
    lines.extend([
        "## Aircraft Breakdown",
        "",
        _markdown_table(
            ["AC", "UAV", "Samples", "Duration", "V50", "R50", "R75", "Yaw95", "Bank95", "Roll"],
            agent_rows,
        ),
    ])

    phase_rows: list[list[Any]] = []
    for agent in agents.values():
        for phase in agent.get("phaseMetrics") or []:
            if not phase.get("sampleCount"):
                continue
            phase_rows.append([
                agent.get("label", "-"),
                phase.get("phase", "-"),
                phase.get("sampleCount", 0),
                _stat_report(phase.get("speedMps"), "p50", 1, " m/s"),
                _stat_report(phase.get("turnRadiusM"), "p50", 0, " m"),
                _stat_report(phase.get("absYawRateDps"), "p95", 2, " dps"),
                _stat_report(phase.get("bankProxyDeg"), "p95", 1, " deg"),
            ])
    phase_rows.sort(key=lambda row: int(row[2] or 0), reverse=True)
    lines.extend([
        "## Mission Phase Sensitivity",
        "",
        _markdown_table(["AC", "Phase", "Samples", "V50", "R50", "Yaw95", "Bank95"], phase_rows[:40]),
    ])

    command_rows = []
    for row in (commands.get("rows") or [])[:40]:
        command_rows.append([
            row.get("label", "-"),
            row.get("type", "-"),
            "matched" if row.get("matched") else "miss",
            _fmt_report(row.get("latencyS"), 3, " s") if row.get("latencyS") is not None else "-",
            json.dumps(row.get("expected") or {}, ensure_ascii=False),
        ])
    lines.extend([
        "## 0602 Command Response",
        "",
        f"- Matched: `{commands.get('matchedCount', 0)} / {commands.get('commandCount', 0)}`",
        f"- Latency p50: `{_stat_report(commands.get('latencyS'), 'p50', 3, ' s')}`",
        f"- Latency p95: `{_stat_report(commands.get('latencyS'), 'p95', 3, ' s')}`",
        "",
        _markdown_table(["AC", "Command", "State", "Latency", "Expected"], command_rows),
    ])

    event_rows = []
    for event in events[:30]:
        event_rows.append([
            event.get("label", "-"),
            event.get("phase", "-"),
            _fmt_report(event.get("durationS"), 2, " s"),
            _fmt_report(event.get("maxYawRateDps"), 2, " dps"),
            _fmt_report(event.get("minRadiusM"), 0, " m"),
            _fmt_report(event.get("maxBankProxyDeg"), 1, " deg"),
            _fmt_report(event.get("turnAngleDeg"), 0, " deg"),
        ])
    lines.extend([
        "## Aggressive Turn Events",
        "",
        _markdown_table(["AC", "Phase", "Duration", "Yaw Max", "Radius Min", "Bank Max", "Turn Angle"], event_rows),
        "## All-Log Signal Scan",
        "",
        f"- Files scanned: `{signals.get('totalFilesScanned', 0)}`",
        f"- Sampled bytes: `{signals.get('sampledBytes', 0)}`",
        f"- Roll fields: `{'present' if signals.get('rollSignalsAvailable') else 'absent; bank proxy used'}`",
        "",
        _markdown_table(
            ["Signal", "Count"],
            [[key, value] for key, value in sorted((signals.get("signalCounts") or {}).items())],
        ),
    ])
    return "\n".join(lines).rstrip() + "\n"


def save_scenario_dynamics_report(scenario_path: Path, result: dict[str, Any] | None = None) -> dict[str, Any]:
    """Persist a read-only dynamics report under the scenario folder."""
    scenario_path = Path(scenario_path)
    if not scenario_path.is_dir():
        raise FileNotFoundError(f"Scenario not found: {scenario_path}")
    analysis = result if result is not None else analyze_scenario_dynamics(scenario_path)
    if not analysis.get("ok"):
        raise ValueError(str(analysis.get("error") or "Dynamics analysis failed"))

    saved_at = datetime.now().astimezone().isoformat(timespec="seconds")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_dir = scenario_path / REPORT_DIR_NAME
    report_dir.mkdir(parents=True, exist_ok=True)

    json_path = report_dir / f"dynamics_analysis_{stamp}.json"
    md_path = report_dir / f"dynamics_analysis_{stamp}.md"
    latest_json_path = report_dir / "dynamics_analysis_latest.json"
    latest_md_path = report_dir / "dynamics_analysis_latest.md"

    payload = {
        "kind": "logAnalyzerDynamicsReport",
        "schemaVersion": REPORT_SCHEMA_VERSION,
        "savedAt": saved_at,
        "scenario": analysis.get("scenario"),
        "analysis": analysis,
        "intendedUse": {
            "readOnly": True,
            "notes": [
                "Use this report as mission planning and replan tuning evidence.",
                "This file does not apply or mutate SIM runtime PID/profile settings.",
                "Read analysis.recommendations.simDynamicsProfile for runtime-compatible candidate values.",
                "Read analysis.recommendations.missionPlanningHints for planner/replan radius guidance.",
            ],
        },
    }
    json_text = json.dumps(payload, ensure_ascii=False, indent=2)
    markdown_text = _render_report_markdown(analysis, saved_at)
    json_path.write_text(json_text, encoding="utf-8")
    latest_json_path.write_text(json_text, encoding="utf-8")
    md_path.write_text(markdown_text, encoding="utf-8")
    latest_md_path.write_text(markdown_text, encoding="utf-8")

    return {
        "ok": True,
        "savedAt": saved_at,
        "scenario": analysis.get("scenario"),
        "reportDir": str(report_dir),
        "files": {
            "json": str(json_path),
            "markdown": str(md_path),
            "latestJson": str(latest_json_path),
            "latestMarkdown": str(latest_md_path),
        },
        "fileNames": {
            "json": json_path.name,
            "markdown": md_path.name,
            "latestJson": latest_json_path.name,
            "latestMarkdown": latest_md_path.name,
        },
        "summary": {
            "uavCount": analysis.get("cohort", {}).get("uavCount", 0),
            "usableUavCount": analysis.get("cohort", {}).get("usableUavCount", 0),
            "sampleCount0401": analysis.get("source", {}).get("sampleCount0401", 0),
            "medianTurnRadiusM": analysis.get("recommendations", {}).get("basis", {}).get("medianTurnRadiusM"),
            "quality": analysis.get("recommendations", {}).get("quality"),
        },
    }
