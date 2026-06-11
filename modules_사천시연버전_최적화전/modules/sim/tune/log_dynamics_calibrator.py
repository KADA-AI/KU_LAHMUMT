from __future__ import annotations

import json
import math
import os
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from modules.common.turn_radius import interpolate_reference_turn_radius
from modules.sim.runtime.controllers.waypoint_pid import DEFAULT_TUNED_GAINS, PIDGains


ROOT_DIR = Path(__file__).resolve().parents[3]
SIM_DIR = Path(__file__).resolve().parents[1]
RUNTIME_CONTROLLER_DIR = SIM_DIR / "runtime" / "controllers"
UAV_PID_DB_PATH = RUNTIME_CONTROLLER_DIR / "uav_pid_db.json"
UAV_DYNAMICS_PROFILE_PATH = RUNTIME_CONTROLLER_DIR / "uav_dynamics_profile.json"


@dataclass
class AgentSample:
    t_ms: int
    aircraft_id: int
    lat: float
    lon: float
    alt: float
    speed: float
    heading: float


def _num(value: Any, default: float | None = None) -> float | None:
    try:
        if value is None:
            return default
        return float(value)
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


def _median(values: Iterable[float], default: float = 0.0) -> float:
    return _percentile(values, 50.0, default)


def _resolve_log_path(raw_path: str | os.PathLike[str] | None) -> Path:
    if raw_path is None:
        raise ValueError("log path is required")
    text = str(raw_path).strip().strip("\"'")
    if not text:
        raise ValueError("log path is required")
    path = Path(text).expanduser()
    if not path.is_absolute():
        path = (ROOT_DIR / path).resolve()
    else:
        path = path.resolve()
    if not path.exists():
        raise FileNotFoundError(str(path))
    return path


def _find_0401_files(path: Path) -> list[Path]:
    if path.is_file():
        return [path] if path.suffix.lower() == ".json" else []

    candidates = [
        path / "0401",
        path / "SBC3" / "0401",
    ]
    if path.name.lower() == "sbc3":
        candidates.insert(0, path / "0401")
    if path.name == "0401":
        candidates.insert(0, path)

    found: list[Path] = []
    for candidate in candidates:
        if candidate.exists() and candidate.is_dir():
            found.extend(sorted(candidate.glob("0401*.json")))
            if found:
                break
    if not found:
        found.extend(sorted(path.glob("**/0401/0401*.json")))
    return [p for p in found if p.is_file()]


def _json_messages_from_text(raw: str) -> list[dict[str, Any]]:
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
        raw = path.read_text(encoding="utf-8-sig")
    except UnicodeDecodeError:
        raw = path.read_text(encoding="cp949", errors="ignore")
    return _json_messages_from_text(raw)


def _extract_coord(agent: dict[str, Any]) -> tuple[float, float, float] | None:
    coord = agent.get("coordinate") or agent.get("Coordinate")
    if not isinstance(coord, dict):
        return None
    lat = _num(coord.get("latitude") if "latitude" in coord else coord.get("Latitude"))
    lon = _num(coord.get("longitude") if "longitude" in coord else coord.get("Longitude"))
    alt = _num(coord.get("altitude") if "altitude" in coord else coord.get("Altitude"), 0.0)
    if lat is None or lon is None:
        return None
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return None
    return float(lat), float(lon), float(alt or 0.0)


def _extract_velocity(agent: dict[str, Any]) -> tuple[float, float]:
    velocity = agent.get("velocity") or agent.get("Velocity")
    if not isinstance(velocity, dict):
        velocity = {}
    speed = _num(velocity.get("speed") if "speed" in velocity else velocity.get("Speed"), 0.0)
    heading = _num(velocity.get("heading") if "heading" in velocity else velocity.get("Heading"), 0.0)
    return float(speed or 0.0), float(heading or 0.0) % 360.0


def _is_uav_agent(agent: dict[str, Any], aircraft_id: int) -> bool:
    if 4 <= aircraft_id <= 6:
        return True
    is_unmanned = agent.get("isUnmanned")
    if isinstance(is_unmanned, bool):
        return is_unmanned
    if is_unmanned is not None:
        try:
            return bool(int(is_unmanned))
        except Exception:
            return False
    return False


def _load_agent_samples_from_message_batches(message_batches: Iterable[list[dict[str, Any]]]) -> dict[int, list[AgentSample]]:
    by_agent: dict[int, list[AgentSample]] = {}
    for messages in message_batches:
        for message in messages:
            t_ms = _int(message.get("timestamp") or message.get("Timestamp"))
            if t_ms is None:
                continue
            agents = message.get("agentStateList") or message.get("AgentStateList") or []
            if not isinstance(agents, list):
                continue
            for agent in agents:
                if not isinstance(agent, dict):
                    continue
                aircraft_id = _int(agent.get("aircraftID") or agent.get("AircraftID"))
                if aircraft_id is None or not _is_uav_agent(agent, aircraft_id):
                    continue
                coord = _extract_coord(agent)
                if coord is None:
                    continue
                speed, heading = _extract_velocity(agent)
                lat, lon, alt = coord
                by_agent.setdefault(aircraft_id, []).append(
                    AgentSample(
                        t_ms=int(t_ms),
                        aircraft_id=int(aircraft_id),
                        lat=float(lat),
                        lon=float(lon),
                        alt=float(alt),
                        speed=float(speed),
                        heading=float(heading),
                    )
                )
    for aircraft_id, samples in list(by_agent.items()):
        samples.sort(key=lambda item: item.t_ms)
        deduped: list[AgentSample] = []
        last_t: int | None = None
        for sample in samples:
            if sample.t_ms == last_t:
                continue
            deduped.append(sample)
            last_t = sample.t_ms
        by_agent[aircraft_id] = deduped
    return by_agent


def _load_agent_samples(files: list[Path]) -> dict[int, list[AgentSample]]:
    return _load_agent_samples_from_message_batches(_json_messages(file_path) for file_path in files)


def _project_samples(samples: list[AgentSample]) -> list[tuple[AgentSample, float, float]]:
    if not samples:
        return []
    lat0 = _median([s.lat for s in samples], samples[0].lat)
    lon0 = _median([s.lon for s in samples], samples[0].lon)
    lat_scale = 111_320.0
    lon_scale = 111_320.0 * max(0.05, math.cos(math.radians(lat0)))
    return [
        (
            sample,
            (sample.lon - lon0) * lon_scale,
            (sample.lat - lat0) * lat_scale,
        )
        for sample in samples
    ]


def _analyze_agent(samples: list[AgentSample]) -> dict[str, Any]:
    projected = _project_samples(samples)
    if len(projected) < 3:
        return {
            "sample_count": len(samples),
            "duration_s": 0.0,
            "turn_sample_count": 0,
            "usable": False,
        }

    speeds: list[float] = []
    yaw_rates: list[float] = []
    radii: list[float] = []
    banks: list[float] = []
    reversal_gaps: list[float] = []
    dist_total = 0.0
    last_sig_sign = 0
    last_sig_time_s: float | None = None
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
        speed = max(0.0, (float(prev.speed) + float(curr.speed)) * 0.5)
        if speed <= 1.0 and ground_speed > 1.0:
            speed = ground_speed
        if speed <= 5.0:
            continue
        dist_total += dist
        speeds.append(speed)
        heading_delta = _wrap_delta_deg(curr.heading - prev.heading)
        yaw_rate = heading_delta / dt_s
        if abs(yaw_rate) > 120.0:
            continue
        yaw_rates.append(yaw_rate)
        if abs(yaw_rate) >= 0.25:
            radius = speed / max(1e-6, math.radians(abs(yaw_rate)))
            if 40.0 <= radius <= 8000.0:
                radii.append(radius)
                bank = math.degrees(math.atan((speed * speed) / max(1e-6, 9.80665 * radius)))
                if 0.0 <= bank <= 80.0:
                    banks.append(bank)
        if abs(yaw_rate) >= 0.6:
            sign = 1 if yaw_rate > 0 else -1
            now_s = (curr.t_ms - start_t) / 1000.0
            if last_sig_sign and sign != last_sig_sign and last_sig_time_s is not None:
                reversal_gaps.append(max(0.0, now_s - last_sig_time_s))
            last_sig_sign = sign
            last_sig_time_s = now_s

    abs_yaw = [abs(v) for v in yaw_rates]
    duration_s = max(0.0, (end_t - start_t) / 1000.0)
    median_speed = _median(speeds, 0.0)
    median_radius = _median(radii, 0.0)
    current_radius = interpolate_reference_turn_radius(median_speed) if median_speed > 0.0 else 0.0
    return {
        "sample_count": len(samples),
        "duration_s": duration_s,
        "distance_m": dist_total,
        "median_speed_mps": median_speed,
        "speed_p95_mps": _percentile(speeds, 95.0, median_speed),
        "turn_sample_count": len(radii),
        "median_turn_radius_m": median_radius,
        "turn_radius_p25_m": _percentile(radii, 25.0, median_radius),
        "turn_radius_p75_m": _percentile(radii, 75.0, median_radius),
        "current_reference_radius_m": current_radius,
        "abs_yaw_rate_p50_dps": _percentile(abs_yaw, 50.0, 0.0),
        "abs_yaw_rate_p95_dps": _percentile(abs_yaw, 95.0, 0.0),
        "bank_proxy_p50_deg": _percentile(banks, 50.0, 0.0),
        "bank_proxy_p95_deg": _percentile(banks, 95.0, 0.0),
        "opposite_turn_gap_p50_s": _percentile(reversal_gaps, 50.0, 0.0),
        "opposite_turn_count": len(reversal_gaps),
        "usable": len(radii) >= 20 and median_speed > 5.0,
    }


def _load_current_uav_gains() -> dict[str, float]:
    data = asdict(DEFAULT_TUNED_GAINS)
    try:
        raw = json.loads(UAV_PID_DB_PATH.read_text(encoding="utf-8"))
        records = raw.get("records") if isinstance(raw, dict) else None
        if isinstance(records, list) and records:
            gains = records[0].get("gains")
            if isinstance(gains, dict):
                for key in data:
                    if key in gains:
                        data[key] = float(gains[key])
    except Exception:
        pass
    return data


def _build_proposal(summary: dict[str, Any], source_path: Path) -> dict[str, Any]:
    agents = [item for item in (summary.get("agents") or {}).values() if isinstance(item, dict) and item.get("usable")]
    current_gains = _load_current_uav_gains()
    if agents:
        median_speed = _median([float(a.get("median_speed_mps", 0.0)) for a in agents], 40.0)
        median_radius = _median([float(a.get("median_turn_radius_m", 0.0)) for a in agents], 450.0)
        yaw_p95 = _percentile([float(a.get("abs_yaw_rate_p95_dps", 0.0)) for a in agents], 70.0, 4.5)
        bank_p95 = _percentile([float(a.get("bank_proxy_p95_deg", 0.0)) for a in agents], 70.0, 25.0)
        bank_p50 = _median([float(a.get("bank_proxy_p50_deg", 0.0)) for a in agents], 18.0)
        reversal_gap = _median([float(a.get("opposite_turn_gap_p50_s", 0.0)) for a in agents if float(a.get("opposite_turn_gap_p50_s", 0.0)) > 0.0], 0.0)
    else:
        median_speed = 40.0
        median_radius = 450.0
        yaw_p95 = 4.5
        bank_p95 = 25.0
        bank_p50 = 18.0
        reversal_gap = 0.0

    current_ref_radius = float(interpolate_reference_turn_radius(median_speed))
    radius_scale = 1.0
    if median_radius > 1.0 and current_ref_radius > 1.0:
        radius_scale = _clamp(median_radius / current_ref_radius, 0.65, 1.55)

    turn_bank_limit = _clamp(bank_p95 * 1.12, 18.0, 55.0)
    roll_limit = _clamp(max(turn_bank_limit + 5.0, 35.0), 35.0, 60.0)
    max_yaw_rate = _clamp(yaw_p95 * 1.18, 2.5, 30.0)
    if reversal_gap > 0.2 and bank_p50 > 2.0:
        max_roll_rate = _clamp((2.0 * bank_p50 / reversal_gap) * 1.15, 8.0, 45.0)
    else:
        max_roll_rate = _clamp(turn_bank_limit * 0.95, 12.0, 40.0)

    lookahead = _clamp(max(70.0, median_radius * 0.45), 70.0, 420.0)
    freeze_yaw = _clamp(max(35.0, median_radius * 0.14), 35.0, 140.0)
    yaw_scale = _clamp(current_ref_radius / max(1.0, median_radius), 0.68, 1.22)
    gains = dict(current_gains)
    gains["yaw"] = _clamp(float(current_gains.get("yaw", DEFAULT_TUNED_GAINS.yaw)) * yaw_scale, 0.35, 1.05)
    gains["yaw_i"] = _clamp(float(current_gains.get("yaw_i", DEFAULT_TUNED_GAINS.yaw_i)) * yaw_scale, 0.0, 0.08)
    gains["lookahead_m"] = lookahead
    gains["freeze_yaw_dist"] = freeze_yaw

    quality = "ok" if agents else "warn"
    notes = []
    if agents:
        notes.append(f"{len(agents)} UAV track(s) used")
    else:
        notes.append("usable UAV turn samples were insufficient; conservative defaults suggested")
    if reversal_gap > 0.0:
        notes.append(f"opposite-turn response gap median {reversal_gap:.2f}s")

    now = datetime.now(timezone.utc).isoformat()
    return {
        "quality": quality,
        "created_at": now,
        "source_log": str(source_path),
        "pid": {
            "gains": {key: float(gains[key]) for key in PIDGains.__dataclass_fields__ if key in gains},
            "changed_fields": ["yaw", "yaw_i", "lookahead_m", "freeze_yaw_dist"],
        },
        "dynamics": {
            "enabled": True,
            "params": {
                "banked_turn_enabled": True,
                "bank_yaw_rate_blend": 0.85,
                "reference_turn_radius_scale": radius_scale,
                "max_yaw_rate_dps": max_yaw_rate,
                "max_roll_rate_dps": max_roll_rate,
                "turn_bank_limit_deg": turn_bank_limit,
                "roll_limit_deg": roll_limit,
                "turn_roll_gain": 2.2,
                "use_reference_turn_radius": True,
            },
        },
        "notes": notes,
        "basis": {
            "median_speed_mps": median_speed,
            "median_turn_radius_m": median_radius,
            "current_reference_radius_m": current_ref_radius,
            "reference_turn_radius_scale": radius_scale,
            "abs_yaw_rate_p95_dps": yaw_p95,
            "bank_proxy_p95_deg": bank_p95,
            "opposite_turn_gap_p50_s": reversal_gap,
        },
    }


def analyze_log_path(raw_path: str | os.PathLike[str]) -> dict[str, Any]:
    source_path = _resolve_log_path(raw_path)
    files = _find_0401_files(source_path)
    if not files:
        raise FileNotFoundError(f"0401 log files were not found under {source_path}")
    samples_by_agent = _load_agent_samples(files)
    agents = {
        str(aircraft_id): _analyze_agent(samples)
        for aircraft_id, samples in sorted(samples_by_agent.items())
    }
    usable_agents = sum(1 for item in agents.values() if item.get("usable"))
    total_samples = sum(int(item.get("sample_count", 0)) for item in agents.values())
    summary = {
        "source_path": str(source_path),
        "files": [str(p) for p in files],
        "file_count": len(files),
        "agent_count": len(agents),
        "usable_agent_count": usable_agents,
        "total_sample_count": total_samples,
        "agents": agents,
    }
    return {
        "ok": True,
        "summary": summary,
        "proposal": _build_proposal(summary, source_path),
    }


def analyze_uploaded_0401_files(files: list[dict[str, Any]]) -> dict[str, Any]:
    if not isinstance(files, list) or not files:
        raise ValueError("uploaded 0401 files are required")

    accepted: list[dict[str, Any]] = []
    batches: list[list[dict[str, Any]]] = []
    total_bytes = 0
    for item in files:
        if not isinstance(item, dict):
            continue
        name = str(item.get("name") or item.get("relativePath") or "").replace("\\", "/")
        base_name = name.rsplit("/", 1)[-1].lower()
        if not (base_name.startswith("0401") and base_name.endswith(".json")):
            continue
        content = item.get("content")
        if not isinstance(content, str) or not content.strip():
            continue
        total_bytes += len(content.encode("utf-8", errors="ignore"))
        batches.append(_json_messages_from_text(content))
        accepted.append(
            {
                "name": name or base_name,
                "bytes": len(content.encode("utf-8", errors="ignore")),
            }
        )

    if not accepted:
        raise FileNotFoundError("uploaded folder did not contain 0401*.json files")
    samples_by_agent = _load_agent_samples_from_message_batches(batches)
    agents = {
        str(aircraft_id): _analyze_agent(samples)
        for aircraft_id, samples in sorted(samples_by_agent.items())
    }
    usable_agents = sum(1 for item in agents.values() if item.get("usable"))
    total_samples = sum(int(item.get("sample_count", 0)) for item in agents.values())
    summary = {
        "source_path": "uploaded-log-folder",
        "files": [item["name"] for item in accepted],
        "file_count": len(accepted),
        "agent_count": len(agents),
        "usable_agent_count": usable_agents,
        "total_sample_count": total_samples,
        "uploaded_bytes": total_bytes,
        "agents": agents,
    }
    return {
        "ok": True,
        "summary": summary,
        "proposal": _build_proposal(summary, Path("uploaded-log-folder")),
    }


def _save_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(tmp, path)


def _backup_file(path: Path, label: str) -> str | None:
    if not path.exists():
        return None
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = path.with_name(f"{path.name}.bak_{label}_{stamp}")
    shutil.copy2(path, backup)
    return str(backup)


def _sanitize_gains(raw: dict[str, Any]) -> dict[str, float]:
    fallback = asdict(DEFAULT_TUNED_GAINS)
    result: dict[str, float] = {}
    for key in PIDGains.__dataclass_fields__:
        value = _num(raw.get(key), fallback.get(key, 0.0))
        if value is not None and math.isfinite(value):
            result[key] = float(value)
    result["yaw"] = _clamp(result.get("yaw", DEFAULT_TUNED_GAINS.yaw), 0.05, 2.0)
    result["yaw_i"] = _clamp(result.get("yaw_i", DEFAULT_TUNED_GAINS.yaw_i), 0.0, 0.2)
    result["lookahead_m"] = _clamp(result.get("lookahead_m", DEFAULT_TUNED_GAINS.lookahead_m), 1.0, 1000.0)
    result["freeze_yaw_dist"] = _clamp(result.get("freeze_yaw_dist", DEFAULT_TUNED_GAINS.freeze_yaw_dist), 0.0, 500.0)
    return result


def _sanitize_dynamics(raw: dict[str, Any]) -> dict[str, Any]:
    return {
        "banked_turn_enabled": bool(raw.get("banked_turn_enabled", True)),
        "bank_yaw_rate_blend": _clamp(_num(raw.get("bank_yaw_rate_blend"), 0.85) or 0.85, 0.0, 1.0),
        "reference_turn_radius_scale": _clamp(_num(raw.get("reference_turn_radius_scale"), 1.0) or 1.0, 0.25, 3.0),
        "max_yaw_rate_dps": _clamp(_num(raw.get("max_yaw_rate_dps"), 30.0) or 30.0, 0.5, 90.0),
        "max_roll_rate_dps": _clamp(_num(raw.get("max_roll_rate_dps"), 40.0) or 40.0, 1.0, 120.0),
        "turn_bank_limit_deg": _clamp(_num(raw.get("turn_bank_limit_deg"), 55.0) or 55.0, 1.0, 80.0),
        "roll_limit_deg": _clamp(_num(raw.get("roll_limit_deg"), 60.0) or 60.0, 1.0, 89.0),
        "turn_roll_gain": _clamp(_num(raw.get("turn_roll_gain"), 2.2) or 2.2, 0.1, 10.0),
        "use_reference_turn_radius": bool(raw.get("use_reference_turn_radius", True)),
    }


def apply_proposal(proposal: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(proposal, dict):
        raise ValueError("proposal must be an object")
    pid = proposal.get("pid") if isinstance(proposal.get("pid"), dict) else {}
    dynamics = proposal.get("dynamics") if isinstance(proposal.get("dynamics"), dict) else {}
    gains = _sanitize_gains(pid.get("gains") if isinstance(pid.get("gains"), dict) else {})
    params = _sanitize_dynamics(dynamics.get("params") if isinstance(dynamics.get("params"), dict) else {})

    backups: dict[str, str | None] = {
        "uav_pid_db": _backup_file(UAV_PID_DB_PATH, "dynfit"),
        "uav_dynamics_profile": _backup_file(UAV_DYNAMICS_PROFILE_PATH, "dynfit"),
    }

    db_payload: dict[str, Any]
    if UAV_PID_DB_PATH.exists():
        try:
            db_payload = json.loads(UAV_PID_DB_PATH.read_text(encoding="utf-8"))
        except Exception:
            db_payload = {}
    else:
        db_payload = {}
    records = db_payload.get("records")
    if not isinstance(records, list) or not records:
        records = [
            {
                "time_scale": 1.0,
                "dt_coarse": 0.02,
                "dt_fine": 0.01,
                "best_score": 0.0,
                "best_stage": "log-dynamics-fit",
                "gains": dict(gains),
            }
        ]
        db_payload["records"] = records
    for record in records:
        if not isinstance(record, dict):
            continue
        current = record.get("gains")
        if not isinstance(current, dict):
            current = {}
            record["gains"] = current
        current.update(gains)
        record["best_stage"] = "log-dynamics-fit"
        record["updated_by"] = "sim-dynamics-calibration"

    profile_payload = {
        "enabled": bool(dynamics.get("enabled", True)),
        "updated_at": datetime.now(timezone.utc).isoformat(),
        "source_log": proposal.get("source_log"),
        "params": params,
        "basis": proposal.get("basis") if isinstance(proposal.get("basis"), dict) else {},
        "notes": proposal.get("notes") if isinstance(proposal.get("notes"), list) else [],
    }
    _save_json_atomic(UAV_PID_DB_PATH, db_payload)
    _save_json_atomic(UAV_DYNAMICS_PROFILE_PATH, profile_payload)
    return {
        "ok": True,
        "applied": {
            "uav_pid_db": str(UAV_PID_DB_PATH),
            "uav_dynamics_profile": str(UAV_DYNAMICS_PROFILE_PATH),
        },
        "backups": backups,
        "profile": profile_payload,
    }


def disable_profile() -> dict[str, Any]:
    backup = _backup_file(UAV_DYNAMICS_PROFILE_PATH, "dynfit_disable")
    profile: dict[str, Any] = {}
    if UAV_DYNAMICS_PROFILE_PATH.exists():
        try:
            raw = json.loads(UAV_DYNAMICS_PROFILE_PATH.read_text(encoding="utf-8"))
            if isinstance(raw, dict):
                profile = raw
        except Exception:
            profile = {}
    profile["enabled"] = False
    profile["updated_at"] = datetime.now(timezone.utc).isoformat()
    profile.setdefault("params", {})
    profile.setdefault("notes", [])
    if isinstance(profile["notes"], list):
        profile["notes"].append("disabled from sim dynamics calibration panel")
    _save_json_atomic(UAV_DYNAMICS_PROFILE_PATH, profile)
    return {
        "ok": True,
        "profile_path": str(UAV_DYNAMICS_PROFILE_PATH),
        "backup": backup,
        "profile": profile,
    }


def current_status() -> dict[str, Any]:
    profile: dict[str, Any] | None = None
    if UAV_DYNAMICS_PROFILE_PATH.exists():
        try:
            raw = json.loads(UAV_DYNAMICS_PROFILE_PATH.read_text(encoding="utf-8"))
            if isinstance(raw, dict):
                profile = raw
        except Exception:
            profile = None
    return {
        "ok": True,
        "profile_path": str(UAV_DYNAMICS_PROFILE_PATH),
        "pid_db_path": str(UAV_PID_DB_PATH),
        "profile": profile,
        "has_profile": profile is not None,
    }
