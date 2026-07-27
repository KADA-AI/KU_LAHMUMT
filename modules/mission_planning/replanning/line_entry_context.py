from __future__ import annotations

import math
from copy import deepcopy
from typing import Any, Dict, Mapping

from modules.common.turn_dynamics import turn_radius_from_rate_m


def _to_int(value: Any) -> int | None:
    try:
        if value is None or value == "":
            return None
        return int(value)
    except Exception:
        return None


def _to_float(value: Any) -> float | None:
    try:
        if value is None or value == "":
            return None
        parsed = float(value)
    except Exception:
        return None
    return parsed if math.isfinite(parsed) else None


def _first_float(*values: Any) -> float | None:
    for value in values:
        parsed = _to_float(value)
        if parsed is not None:
            return float(parsed)
    return None


def _speed_to_mps(value: Any) -> float | None:
    speed = _to_float(value)
    if speed is None or speed <= 0.0:
        return None
    return (float(speed) * 1000.0 / 3600.0) if float(speed) > 70.0 else float(speed)


def normalize_coordinate(value: Any) -> Dict[str, float] | None:
    if not isinstance(value, Mapping):
        return None
    lat = _first_float(value.get("latitude"), value.get("Latitude"), value.get("lat"))
    lon = _first_float(value.get("longitude"), value.get("Longitude"), value.get("lon"))
    if lat is None or lon is None:
        return None
    out: Dict[str, float] = {"latitude": float(lat), "longitude": float(lon)}
    alt = _first_float(value.get("altitude"), value.get("Altitude"), value.get("alt"))
    if alt is not None:
        out["altitude"] = float(alt)
    return out


def _nested_coordinate(row: Mapping[str, Any], *keys: str) -> Dict[str, float] | None:
    for key in keys:
        coord = normalize_coordinate(row.get(key))
        if coord is not None:
            return coord
    unmanned = row.get("unmannedInfo") if isinstance(row.get("unmannedInfo"), Mapping) else {}
    manned = row.get("mannedInfo") if isinstance(row.get("mannedInfo"), Mapping) else {}
    for container in (unmanned, manned):
        coord = normalize_coordinate(container.get("coordinate"))
        if coord is not None:
            return coord
    return None


def _state_heading_deg(row: Mapping[str, Any]) -> float | None:
    velocity = row.get("velocity") if isinstance(row.get("velocity"), Mapping) else {}
    return _first_float(
        row.get("headingDeg"),
        row.get("heading"),
        row.get("Heading"),
        velocity.get("heading"),
        velocity.get("Heading"),
    )


def _state_speed_mps(row: Mapping[str, Any]) -> float | None:
    velocity = row.get("velocity") if isinstance(row.get("velocity"), Mapping) else {}
    explicit = _first_float(row.get("speedMps"), row.get("speed_mps"))
    if explicit is not None and explicit > 0.0:
        return float(explicit)
    return _speed_to_mps(_first_float(row.get("speed"), row.get("Speed"), velocity.get("speed"), velocity.get("Speed")))


def _state_turn_rate_dps(row: Mapping[str, Any]) -> float | None:
    velocity = row.get("velocity") if isinstance(row.get("velocity"), Mapping) else {}
    return _first_float(
        row.get("turnRateDps"),
        row.get("turn_rate_dps"),
        row.get("yawRateDps"),
        row.get("yaw_rate_dps"),
        velocity.get("turnRateDps"),
        velocity.get("yawRateDps"),
    )


def _state_turn_sign(row: Mapping[str, Any], turn_rate_dps: float | None) -> int:
    sign = _to_int(row.get("turnSign"))
    if sign is not None:
        return 1 if sign > 0 else -1 if sign < 0 else 0
    if turn_rate_dps is None or abs(float(turn_rate_dps)) < 0.2:
        return 0
    return 1 if float(turn_rate_dps) > 0.0 else -1


def _state_turn_radius_m(row: Mapping[str, Any], speed_mps: float | None, turn_rate_dps: float | None) -> float | None:
    radius = _first_float(
        row.get("turnRadiusM"),
        row.get("turnCircleRadiusM"),
        row.get("actualTurnRadiusM"),
        row.get("idealTurnRadiusM"),
        row.get("turn_radius_m"),
    )
    if radius is not None and radius > 0.0:
        return float(radius)
    if speed_mps is None or speed_mps <= 0.0 or turn_rate_dps is None or abs(float(turn_rate_dps)) < 0.2:
        return None
    return turn_radius_from_rate_m(speed_mps, turn_rate_dps)


def build_line_entry_context_row(
    *,
    aircraft_id: int,
    state: Mapping[str, Any] | None = None,
    entry_coordinate: Mapping[str, Any] | None = None,
    heading_deg: float | None = None,
    current_coordinate: Mapping[str, Any] | None = None,
) -> Dict[str, Any] | None:
    aid = _to_int(aircraft_id)
    if aid is None or aid <= 0:
        return None
    state_row = state if isinstance(state, Mapping) else {}

    entry_coord = normalize_coordinate(entry_coordinate) or normalize_coordinate(state_row.get("coordinate"))
    current_coord = (
        normalize_coordinate(current_coordinate)
        or normalize_coordinate(state_row.get("currentCoordinate"))
        or normalize_coordinate(state_row.get("current_coordinate"))
        or _nested_coordinate(state_row, "originalCoordinate", "currentPosition", "positionCoordinate")
        or normalize_coordinate(state_row.get("coordinate"))
    )
    if entry_coord is None:
        entry_coord = current_coord
    if entry_coord is None:
        return None

    resolved_heading = _first_float(heading_deg, _state_heading_deg(state_row))
    speed_mps = _state_speed_mps(state_row)
    turn_rate_dps = _state_turn_rate_dps(state_row)
    turn_radius_m = _state_turn_radius_m(state_row, speed_mps, turn_rate_dps)
    turn_sign = _state_turn_sign(state_row, turn_rate_dps)

    row: Dict[str, Any] = deepcopy(dict(state_row))
    row.pop("predictedEntryCoordinate", None)
    row.pop("predictedEntryEtaS", None)
    row.pop("predictionLeadS", None)
    row["aircraftID"] = int(aid)
    row["coordinate"] = dict(entry_coord)
    if current_coord is not None:
        row["currentCoordinate"] = dict(current_coord)
    if resolved_heading is not None:
        row["headingDeg"] = float(resolved_heading) % 360.0
    if speed_mps is not None:
        row["speedMps"] = float(speed_mps)
    if turn_rate_dps is not None:
        row["turnRateDps"] = float(turn_rate_dps)
    row["turnSign"] = int(turn_sign)
    if turn_radius_m is not None and turn_radius_m > 0.0:
        row["turnRadiusM"] = float(turn_radius_m)
    for key in ("idealTurnRadiusM", "actualTurnRadiusM", "turnCircleRadiusM"):
        value = _first_float(state_row.get(key))
        if value is not None and value > 0.0:
            row[key] = float(value)
    eta_s = _first_float(state_row.get("etaS"))
    if eta_s is not None:
        row["etaS"] = float(eta_s)

    # The generic prediction fields above belong to older Area/alternate-WP
    # flows.  Preserve only the confidence-bearing recent-turn projection that
    # is explicitly produced for next-collab Line planning.
    line_predicted_coord = normalize_coordinate(state_row.get("linePredictedEntryCoordinate"))
    if line_predicted_coord is not None:
        row["linePredictedEntryCoordinate"] = dict(line_predicted_coord)
    else:
        row.pop("linePredictedEntryCoordinate", None)
    for key in (
        "lineTrendHeadingDeg",
        "lineTrendTurnRateDps",
        "lineTurnDirectionConfidence",
        "linePredictedHeadingDeg",
        "linePredictionLeadS",
        "lineTurnDataAgeS",
    ):
        value = _to_float(state_row.get(key))
        if value is None:
            row.pop(key, None)
            continue
        if key in {"lineTrendHeadingDeg", "linePredictedHeadingDeg"}:
            value = float(value) % 360.0
        elif key == "lineTurnDirectionConfidence":
            value = max(0.0, min(1.0, float(value)))
        elif key in {"linePredictionLeadS", "lineTurnDataAgeS"}:
            value = max(0.0, float(value))
        row[key] = float(value)
    trend_sign = _to_int(state_row.get("lineTrendTurnSign"))
    row["lineTrendTurnSign"] = (
        1 if trend_sign is not None and trend_sign > 0 else -1 if trend_sign is not None and trend_sign < 0 else 0
    )
    trend_samples = _to_int(state_row.get("lineTrendSampleCount"))
    if trend_samples is not None:
        row["lineTrendSampleCount"] = max(0, int(trend_samples))
    return row


def build_line_entry_context_map(
    *,
    state_map: Mapping[int, Mapping[str, Any]] | None = None,
    entry_coord_map: Mapping[int, Mapping[str, Any]] | None = None,
    heading_map: Mapping[int, float] | None = None,
    base_context_map: Mapping[int, Mapping[str, Any]] | None = None,
) -> Dict[int, Dict[str, Any]]:
    def _lookup(source: Mapping[int, Any] | None, aid: int) -> Any:
        if not isinstance(source, Mapping):
            return None
        if aid in source:
            return source.get(aid)
        return source.get(str(aid))

    ids: set[int] = set()
    for source in (state_map, entry_coord_map, heading_map, base_context_map):
        if isinstance(source, Mapping):
            for raw_id in source.keys():
                aid = _to_int(raw_id)
                if aid is not None and aid > 0:
                    ids.add(int(aid))
    out: Dict[int, Dict[str, Any]] = {}
    for aid in sorted(ids):
        base = dict(_lookup(base_context_map, aid) or {}) if isinstance(base_context_map, Mapping) else {}
        state = dict(_lookup(state_map, aid) or {}) if isinstance(state_map, Mapping) else {}
        if base:
            merged_state = dict(base)
            merged_state.update(state)
        else:
            merged_state = state
        row = build_line_entry_context_row(
            aircraft_id=int(aid),
            state=merged_state,
            entry_coordinate=_lookup(entry_coord_map, aid),
            heading_deg=_lookup(heading_map, aid),
        )
        if row is not None:
            out[int(aid)] = row
    return out


def build_line_entry_context_map_from_entry_rows(rows: Any) -> Dict[int, Dict[str, Any]]:
    out: Dict[int, Dict[str, Any]] = {}
    for item in rows or []:
        if not isinstance(item, Mapping):
            continue
        aid = _to_int(item.get("aircraftID"))
        if aid is None or aid <= 0:
            continue
        row = build_line_entry_context_row(
            aircraft_id=int(aid),
            state=item,
            entry_coordinate=item.get("coordinate"),
            heading_deg=_state_heading_deg(item),
            current_coordinate=item.get("currentCoordinate"),
        )
        if row is not None:
            out[int(aid)] = row
    return out
