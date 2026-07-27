from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Optional


EARTH_RADIUS_M = 6_371_008.8
GRAVITY_MPS2 = 9.80665
MAX_PROJECTION_S = 120.0
REFERENCE_TURN_RADIUS_TABLE_MPS: tuple[tuple[float, float], ...] = (
    (30.0, 340.0),
    (40.0, 450.0),
    (50.0, 560.0),
)


@dataclass(frozen=True)
class TurnPerformance:
    """Aircraft-specific constraints used by the shared turn model."""

    reference_radius_table_mps: tuple[tuple[float, float], ...] = REFERENCE_TURN_RADIUS_TABLE_MPS
    reference_radius_scale: float = 1.0
    use_reference_radius: bool = False
    min_turn_radius_m: float | None = None
    max_turn_rate_dps: float | None = None
    bank_limit_deg: float = 85.0
    roll_rate_limit_dps: float = 40.0
    roll_time_constant_s: float = 0.45
    turn_rate_scale: float = 1.0


@dataclass(frozen=True)
class TurnPrediction:
    coordinate: dict[str, float]
    heading_deg: float
    roll_deg: float
    average_roll_deg: float
    turn_rate_nav_dps: float
    effective_radius_m: float | None
    horizontal_distance_m: float
    lead_s: float
    model: str


def _finite_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _read_ci(value: Any, *names: str) -> Any:
    if isinstance(value, Mapping):
        lower = {str(key).casefold(): item for key, item in value.items()}
        for name in names:
            if name in value:
                return value[name]
            key = name.casefold()
            if key in lower:
                return lower[key]
        return None
    for name in names:
        if hasattr(value, name):
            return getattr(value, name)
        title = name[:1].upper() + name[1:]
        if hasattr(value, title):
            return getattr(value, title)
    return None


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_attitude(attitude: Any) -> Optional[dict[str, float]]:
    if attitude is None:
        return None
    roll = _finite_float(_read_ci(attitude, "roll", "rollDeg"))
    pitch = _finite_float(_read_ci(attitude, "pitch", "pitchDeg"))
    yaw = _finite_float(_read_ci(attitude, "yaw", "yawDeg"))
    if roll is None or pitch is None or yaw is None:
        return None
    return {
        "roll": _clamp(roll, -180.0, 180.0),
        "pitch": _clamp(pitch, -90.0, 90.0),
        "yaw": yaw % 360.0,
    }


def shortest_heading_error_deg(target_deg: Any, actual_deg: Any) -> Optional[float]:
    target = _finite_float(target_deg)
    actual = _finite_float(actual_deg)
    if target is None or actual is None:
        return None
    return (target - actual + 180.0) % 360.0 - 180.0


def normalize_radius_table(
    table: Iterable[Sequence[float]] | None,
) -> tuple[tuple[float, float], ...]:
    rows: dict[float, float] = {}
    for row in table or REFERENCE_TURN_RADIUS_TABLE_MPS:
        if not isinstance(row, Sequence) or len(row) < 2:
            continue
        speed = _finite_float(row[0])
        radius = _finite_float(row[1])
        if speed is None or radius is None or speed < 0.0 or radius <= 0.0:
            continue
        rows[float(speed)] = float(radius)
    if not rows:
        return REFERENCE_TURN_RADIUS_TABLE_MPS
    return tuple(sorted(rows.items()))


def interpolate_turn_radius(
    speed_mps: Any,
    table: Iterable[Sequence[float]] | None = None,
    *,
    scale: Any = 1.0,
) -> float:
    rows = normalize_radius_table(table)
    speed = _finite_float(speed_mps)
    radius_scale = _finite_float(scale)
    if speed is None:
        speed = rows[0][0]
    if radius_scale is None or radius_scale <= 0.0:
        radius_scale = 1.0
    if speed <= rows[0][0]:
        return float(rows[0][1] * radius_scale)
    if speed >= rows[-1][0]:
        return float(rows[-1][1] * radius_scale)
    for (speed0, radius0), (speed1, radius1) in zip(rows[:-1], rows[1:]):
        if speed0 <= speed <= speed1:
            alpha = (speed - speed0) / max(1.0e-9, speed1 - speed0)
            return float((radius0 + ((radius1 - radius0) * alpha)) * radius_scale)
    return float(rows[-1][1] * radius_scale)


def interpolate_reference_turn_radius(speed_mps: Any) -> float:
    return interpolate_turn_radius(speed_mps, REFERENCE_TURN_RADIUS_TABLE_MPS)


def horizontal_speed_mps(speed_mps: Any, pitch_deg: Any = 0.0) -> Optional[float]:
    speed = _finite_float(speed_mps)
    pitch = _finite_float(pitch_deg)
    if speed is None or pitch is None or speed < 0.0 or abs(pitch) > 89.9:
        return None
    return max(0.0, speed * math.cos(math.radians(pitch)))


def coordinated_turn_rate_nav_dps(
    roll_deg: Any,
    speed_mps: Any,
    pitch_deg: Any = 0.0,
    *,
    turn_rate_scale: Any = 1.0,
) -> Optional[float]:
    roll = _finite_float(roll_deg)
    speed_xy = horizontal_speed_mps(speed_mps, pitch_deg)
    scale = _finite_float(turn_rate_scale)
    if (
        roll is None
        or speed_xy is None
        or speed_xy <= 0.1
        or abs(roll) > 85.0
        or scale is None
        or scale <= 0.0
    ):
        return None
    return math.degrees(GRAVITY_MPS2 * math.tan(math.radians(roll)) / speed_xy) * scale


def turn_radius_from_bank_m(
    speed_mps: Any,
    bank_deg: Any,
    pitch_deg: Any = 0.0,
) -> Optional[float]:
    bank = _finite_float(bank_deg)
    speed_xy = horizontal_speed_mps(speed_mps, pitch_deg)
    if bank is None or speed_xy is None or speed_xy <= 0.1 or abs(bank) > 85.0:
        return None
    tangent = abs(math.tan(math.radians(bank)))
    if tangent <= 1.0e-9:
        return None
    return float((speed_xy * speed_xy) / (GRAVITY_MPS2 * tangent))


def bank_angle_for_turn_radius_deg(
    speed_mps: Any,
    radius_m: Any,
    pitch_deg: Any = 0.0,
    *,
    turn_sign: Any = 1.0,
) -> Optional[float]:
    speed_xy = horizontal_speed_mps(speed_mps, pitch_deg)
    radius = _finite_float(radius_m)
    sign = _finite_float(turn_sign)
    if speed_xy is None or radius is None or radius <= 0.0 or sign is None:
        return None
    bank = math.degrees(math.atan((speed_xy * speed_xy) / (GRAVITY_MPS2 * radius)))
    return math.copysign(bank, sign) if sign != 0.0 else 0.0


def bank_angle_for_turn_rate_deg(
    speed_mps: Any,
    turn_rate_nav_dps: Any,
    pitch_deg: Any = 0.0,
) -> Optional[float]:
    speed_xy = horizontal_speed_mps(speed_mps, pitch_deg)
    turn_rate = _finite_float(turn_rate_nav_dps)
    if speed_xy is None or turn_rate is None:
        return None
    return math.degrees(
        math.atan(speed_xy * math.radians(turn_rate) / GRAVITY_MPS2)
    )


def turn_rate_for_radius_nav_dps(
    speed_mps: Any,
    radius_m: Any,
    pitch_deg: Any = 0.0,
    *,
    turn_sign: Any = 1.0,
) -> Optional[float]:
    speed_xy = horizontal_speed_mps(speed_mps, pitch_deg)
    radius = _finite_float(radius_m)
    sign = _finite_float(turn_sign)
    if speed_xy is None or radius is None or radius <= 0.0 or sign is None:
        return None
    rate = math.degrees(speed_xy / radius)
    return math.copysign(rate, sign) if sign != 0.0 else 0.0


def turn_rate_for_radius_rad_s(
    speed_mps: Any,
    radius_m: Any,
    pitch_deg: Any = 0.0,
    *,
    turn_sign: Any = 1.0,
) -> Optional[float]:
    rate_dps = turn_rate_for_radius_nav_dps(
        speed_mps,
        radius_m,
        pitch_deg,
        turn_sign=turn_sign,
    )
    return math.radians(rate_dps) if rate_dps is not None else None


def turn_radius_from_rate_m(
    speed_mps: Any,
    turn_rate_dps: Any,
    pitch_deg: Any = 0.0,
) -> Optional[float]:
    speed_xy = horizontal_speed_mps(speed_mps, pitch_deg)
    turn_rate = _finite_float(turn_rate_dps)
    if speed_xy is None or speed_xy <= 0.1 or turn_rate is None:
        return None
    angular_rate = abs(math.radians(turn_rate))
    if angular_rate <= 1.0e-9:
        return None
    return float(speed_xy / angular_rate)


def turn_arc_time_s(
    angle_deg: Any,
    radius_m: Any,
    speed_mps: Any,
    pitch_deg: Any = 0.0,
) -> Optional[float]:
    angle = _finite_float(angle_deg)
    radius = _finite_float(radius_m)
    speed_xy = horizontal_speed_mps(speed_mps, pitch_deg)
    if angle is None or radius is None or radius <= 0.0 or speed_xy is None or speed_xy <= 0.0:
        return None
    return abs(math.radians(angle)) * radius / speed_xy


def turn_period_s(radius_m: Any, speed_mps: Any, pitch_deg: Any = 0.0) -> Optional[float]:
    return turn_arc_time_s(360.0, radius_m, speed_mps, pitch_deg)


def max_achievable_turn_rate_dps(
    speed_mps: Any,
    performance: TurnPerformance,
    pitch_deg: Any = 0.0,
) -> float:
    speed_xy = horizontal_speed_mps(speed_mps, pitch_deg)
    if speed_xy is None or speed_xy <= 0.1:
        return 0.0
    limits: list[float] = []
    direct_limit = _finite_float(performance.max_turn_rate_dps)
    if direct_limit is not None and direct_limit > 0.0:
        limits.append(abs(direct_limit))
    if performance.use_reference_radius:
        reference_radius = interpolate_turn_radius(
            speed_xy,
            performance.reference_radius_table_mps,
            scale=performance.reference_radius_scale,
        )
        rate = turn_rate_for_radius_nav_dps(speed_xy, reference_radius)
        if rate is not None:
            limits.append(abs(rate))
    bank_limit = _finite_float(performance.bank_limit_deg)
    if bank_limit is not None and 0.1 < abs(bank_limit) <= 85.0:
        rate = coordinated_turn_rate_nav_dps(
            abs(bank_limit),
            speed_xy,
            turn_rate_scale=performance.turn_rate_scale,
        )
        if rate is not None:
            limits.append(abs(rate))
    min_radius = _finite_float(performance.min_turn_radius_m)
    if min_radius is not None and min_radius > 0.0:
        rate = turn_rate_for_radius_nav_dps(speed_xy, min_radius)
        if rate is not None:
            limits.append(abs(rate))
    return max(0.0, min(limits)) if limits else math.inf


def clamp_turn_rate_dps(
    turn_rate_dps: Any,
    speed_mps: Any,
    performance: TurnPerformance,
    pitch_deg: Any = 0.0,
) -> Optional[float]:
    rate = _finite_float(turn_rate_dps)
    if rate is None:
        return None
    limit = max_achievable_turn_rate_dps(speed_mps, performance, pitch_deg)
    if math.isfinite(limit):
        return _clamp(rate, -limit, limit)
    return rate


def select_effective_turn_radius(
    ideal_radius_m: Any,
    actual_radius_m: Any,
    *,
    actual_min_factor: Any = 0.55,
    actual_max_factor: Any = 2.25,
) -> Optional[float]:
    ideal = _finite_float(ideal_radius_m)
    actual = _finite_float(actual_radius_m)
    if actual is None or actual <= 1.0:
        return ideal if ideal is not None and ideal > 1.0 else None
    if ideal is None or ideal <= 1.0:
        return actual
    min_factor = _finite_float(actual_min_factor) or 0.55
    max_factor = _finite_float(actual_max_factor) or 2.25
    min_factor = max(0.05, min_factor)
    max_factor = max(min_factor, max_factor)
    if ideal * min_factor <= actual <= ideal * max_factor:
        return actual
    return ideal


def fuse_turn_rate_nav_dps(
    observed_rate_dps: Any,
    attitude_rate_dps: Any,
    *,
    attitude_weight: Any = 0.35,
    direction_deadband_dps: Any = 0.2,
) -> tuple[Optional[float], str]:
    observed = _finite_float(observed_rate_dps)
    attitude = _finite_float(attitude_rate_dps)
    parsed_weight = _finite_float(attitude_weight)
    parsed_deadband = _finite_float(direction_deadband_dps)
    weight = _clamp(0.35 if parsed_weight is None else parsed_weight, 0.0, 1.0)
    deadband = max(0.0, 0.2 if parsed_deadband is None else parsed_deadband)
    if observed is None:
        return (attitude, "attitude") if attitude is not None else (None, "none")
    if attitude is None:
        return observed, "observed"
    if abs(observed) > deadband and abs(attitude) > deadband and observed * attitude < 0.0:
        return observed, "observed-direction-conflict"
    return (observed * (1.0 - weight)) + (attitude * weight), "observed+attitude"


def estimate_linear_rate(samples: Iterable[tuple[Any, Any]]) -> Optional[float]:
    points: list[tuple[float, float]] = []
    for timestamp, value in samples:
        t = _finite_float(timestamp)
        y = _finite_float(value)
        if t is not None and y is not None:
            points.append((t, y))
    if len(points) < 2:
        return None
    points.sort(key=lambda item: item[0])
    base_time = points[0][0]
    xs = [item[0] - base_time for item in points]
    ys = [item[1] for item in points]
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    numerator = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))
    denominator = sum((x - x_mean) ** 2 for x in xs)
    if denominator <= 1.0e-9:
        return None
    return numerator / denominator


def estimate_angular_rate_dps(samples: Iterable[tuple[Any, Any]]) -> Optional[float]:
    points: list[tuple[float, float]] = []
    for timestamp, angle_deg in samples:
        t = _finite_float(timestamp)
        angle = _finite_float(angle_deg)
        if t is not None and angle is not None:
            points.append((t, angle))
    if len(points) < 2:
        return None
    points.sort(key=lambda item: item[0])
    unwrapped: list[tuple[float, float]] = [points[0]]
    previous_raw = points[0][1]
    previous_unwrapped = points[0][1]
    for timestamp, raw in points[1:]:
        delta = (raw - previous_raw + 180.0) % 360.0 - 180.0
        previous_unwrapped += delta
        unwrapped.append((timestamp, previous_unwrapped))
        previous_raw = raw
    return estimate_linear_rate(unwrapped)


def advance_roll_toward_target(
    roll_deg: Any,
    target_roll_deg: Any,
    dt_s: Any,
    *,
    bank_limit_deg: Any = 85.0,
    roll_rate_limit_dps: Any = 40.0,
    roll_time_constant_s: Any = 0.45,
    measured_roll_rate_dps: Any = None,
    measured_rate_weight: Any = 0.0,
) -> Optional[float]:
    roll = _finite_float(roll_deg)
    target = _finite_float(target_roll_deg)
    dt = _finite_float(dt_s)
    bank_limit = abs(_finite_float(bank_limit_deg) or 85.0)
    rate_limit = abs(_finite_float(roll_rate_limit_dps) or 40.0)
    time_constant = max(1.0e-3, _finite_float(roll_time_constant_s) or 0.45)
    if roll is None or target is None or dt is None or dt < 0.0:
        return None
    bank_limit = _clamp(bank_limit, 0.1, 85.0)
    target = _clamp(target, -bank_limit, bank_limit)
    feedback_rate = _clamp((target - roll) / time_constant, -rate_limit, rate_limit)
    measured_rate = _finite_float(measured_roll_rate_dps)
    weight = _clamp(_finite_float(measured_rate_weight) or 0.0, 0.0, 1.0)
    if measured_rate is not None:
        measured_rate = _clamp(measured_rate, -rate_limit, rate_limit)
        roll_rate = (feedback_rate * (1.0 - weight)) + (measured_rate * weight)
    else:
        roll_rate = feedback_rate
    next_roll = _clamp(roll + (roll_rate * dt), -bank_limit, bank_limit)
    if (target - roll) * (target - next_roll) < 0.0:
        next_roll = target
    return next_roll


def _normalize_coordinate(coordinate: Any) -> Optional[dict[str, float]]:
    if coordinate is None:
        return None
    latitude = _finite_float(_read_ci(coordinate, "latitude", "lat"))
    longitude = _finite_float(_read_ci(coordinate, "longitude", "lon", "lng"))
    altitude = _finite_float(_read_ci(coordinate, "altitude", "alt"))
    if latitude is None or longitude is None or not -90.0 <= latitude <= 90.0:
        return None
    result = {
        "latitude": latitude,
        "longitude": ((longitude + 180.0) % 360.0) - 180.0,
    }
    if altitude is not None:
        result["altitude"] = altitude
    return result


def _coordinate_from_offset(
    coordinate: Mapping[str, float],
    east_m: float,
    north_m: float,
    altitude_m: float,
    *,
    include_altitude: bool = True,
) -> Optional[dict[str, float]]:
    latitude = float(coordinate["latitude"])
    longitude = float(coordinate["longitude"])
    latitude_out = latitude + math.degrees(north_m / EARTH_RADIUS_M)
    mean_lat_rad = math.radians((latitude + latitude_out) * 0.5)
    lon_scale = EARTH_RADIUS_M * math.cos(mean_lat_rad)
    if abs(lon_scale) < 1.0:
        return None
    longitude_out = longitude + math.degrees(east_m / lon_scale)
    result = {
        "latitude": latitude_out,
        "longitude": ((longitude_out + 180.0) % 360.0) - 180.0,
    }
    if include_altitude:
        result["altitude"] = altitude_m
    return result


def predict_turn_motion(
    coordinate: Any,
    speed_mps: Any,
    heading_deg: Any,
    roll_deg: Any,
    pitch_deg: Any,
    lead_s: Any,
    *,
    roll_rate_dps: Any = None,
    target_roll_deg: Any = None,
    roll_target_horizon_s: Any = 1.5,
    performance: TurnPerformance | None = None,
    integration_step_s: Any = 0.1,
) -> Optional[TurnPrediction]:
    source = _normalize_coordinate(coordinate)
    speed = _finite_float(speed_mps)
    heading = _finite_float(heading_deg)
    roll = _finite_float(roll_deg)
    pitch = _finite_float(pitch_deg)
    lead = _finite_float(lead_s)
    roll_rate = _finite_float(roll_rate_dps)
    target_roll = _finite_float(target_roll_deg)
    if (
        source is None
        or speed is None
        or heading is None
        or roll is None
        or pitch is None
        or lead is None
        or speed < 0.0
        or lead < 0.0
        or abs(roll) > 85.0
        or abs(pitch) > 89.0
    ):
        return None

    lead = min(lead, MAX_PROJECTION_S)
    perf = performance or TurnPerformance()
    bank_limit = _clamp(abs(_finite_float(perf.bank_limit_deg) or 85.0), 0.1, 85.0)
    roll_time_constant_s = max(
        1.0e-3,
        _finite_float(perf.roll_time_constant_s) or 0.45,
    )
    current_roll = _clamp(roll, -bank_limit, bank_limit)
    parsed_target_horizon = _finite_float(roll_target_horizon_s)
    target_horizon = max(0.0, 1.5 if parsed_target_horizon is None else parsed_target_horizon)
    if target_roll is None:
        if roll_rate is not None and abs(roll_rate) > 0.05:
            target_roll = current_roll + (roll_rate * target_horizon)
        else:
            target_roll = current_roll
    target_roll = _clamp(target_roll, -bank_limit, bank_limit)

    speed_xy = horizontal_speed_mps(speed, pitch)
    if speed_xy is None:
        return None
    vertical_speed = speed * math.sin(math.radians(pitch))
    source_has_altitude = "altitude" in source
    source_altitude = float(source.get("altitude", 0.0))
    if lead <= 0.0 or speed_xy <= 0.01:
        return TurnPrediction(
            coordinate=dict(source),
            heading_deg=heading % 360.0,
            roll_deg=current_roll,
            average_roll_deg=current_roll,
            turn_rate_nav_dps=0.0,
            effective_radius_m=None,
            horizontal_distance_m=0.0,
            lead_s=lead,
            model="stationary",
        )

    step = _clamp(_finite_float(integration_step_s) or 0.1, 0.01, 1.0)
    dynamic_roll = (
        roll_rate is not None and abs(roll_rate) > 0.05
    ) or abs(target_roll - current_roll) > 0.05
    elapsed = 0.0
    east_m = 0.0
    north_m = 0.0
    heading_rad = math.radians(heading % 360.0)
    total_heading_delta_rad = 0.0
    roll_integral = 0.0
    final_turn_rate_dps = 0.0

    while elapsed < lead - 1.0e-9:
        dt = min(step, lead - elapsed)
        if dynamic_roll:
            measured_weight = math.exp(-elapsed / max(0.1, roll_time_constant_s * 2.0))
            next_roll = advance_roll_toward_target(
                current_roll,
                target_roll,
                dt,
                bank_limit_deg=bank_limit,
                roll_rate_limit_dps=perf.roll_rate_limit_dps,
                roll_time_constant_s=roll_time_constant_s,
                measured_roll_rate_dps=roll_rate,
                measured_rate_weight=measured_weight,
            )
            if next_roll is None:
                return None
        else:
            next_roll = current_roll
        midpoint_roll = (current_roll + next_roll) * 0.5
        turn_rate = coordinated_turn_rate_nav_dps(
            midpoint_roll,
            speed,
            pitch,
            turn_rate_scale=perf.turn_rate_scale,
        )
        if turn_rate is None:
            turn_rate = 0.0
        constrained_rate = clamp_turn_rate_dps(turn_rate, speed, perf, pitch)
        if constrained_rate is None:
            return None
        delta_heading = math.radians(constrained_rate) * dt
        midpoint_heading = heading_rad + (delta_heading * 0.5)
        east_m += speed_xy * math.sin(midpoint_heading) * dt
        north_m += speed_xy * math.cos(midpoint_heading) * dt
        heading_rad += delta_heading
        total_heading_delta_rad += delta_heading
        roll_integral += midpoint_roll * dt
        final_turn_rate_dps = constrained_rate
        current_roll = next_roll
        elapsed += dt

    projected = _coordinate_from_offset(
        source,
        east_m,
        north_m,
        source_altitude + (vertical_speed * lead),
        include_altitude=source_has_altitude,
    )
    if projected is None:
        return None
    horizontal_distance = speed_xy * lead
    effective_radius = None
    if abs(total_heading_delta_rad) > 1.0e-7:
        effective_radius = horizontal_distance / abs(total_heading_delta_rad)
    average_turn_rate = math.degrees(total_heading_delta_rad / lead)
    return TurnPrediction(
        coordinate=projected,
        heading_deg=math.degrees(heading_rad) % 360.0,
        roll_deg=current_roll,
        average_roll_deg=roll_integral / lead,
        turn_rate_nav_dps=average_turn_rate if math.isfinite(average_turn_rate) else final_turn_rate_dps,
        effective_radius_m=effective_radius,
        horizontal_distance_m=horizontal_distance,
        lead_s=lead,
        model="roll-transition-coordinated-turn" if dynamic_roll else "constant-bank-coordinated-turn",
    )


def project_attitude_motion(
    coordinate: Any,
    speed_mps: Any,
    heading_deg: Any,
    roll_deg: Any,
    pitch_deg: Any,
    lead_s: Any,
) -> Optional[dict[str, float]]:
    prediction = predict_turn_motion(
        coordinate,
        speed_mps,
        heading_deg,
        roll_deg,
        pitch_deg,
        lead_s,
    )
    return dict(prediction.coordinate) if prediction is not None else None


__all__ = [
    "EARTH_RADIUS_M",
    "GRAVITY_MPS2",
    "MAX_PROJECTION_S",
    "REFERENCE_TURN_RADIUS_TABLE_MPS",
    "TurnPerformance",
    "TurnPrediction",
    "advance_roll_toward_target",
    "bank_angle_for_turn_radius_deg",
    "bank_angle_for_turn_rate_deg",
    "clamp_turn_rate_dps",
    "coordinated_turn_rate_nav_dps",
    "estimate_angular_rate_dps",
    "estimate_linear_rate",
    "fuse_turn_rate_nav_dps",
    "horizontal_speed_mps",
    "interpolate_reference_turn_radius",
    "interpolate_turn_radius",
    "max_achievable_turn_rate_dps",
    "normalize_attitude",
    "normalize_radius_table",
    "predict_turn_motion",
    "project_attitude_motion",
    "select_effective_turn_radius",
    "shortest_heading_error_deg",
    "turn_arc_time_s",
    "turn_period_s",
    "turn_radius_from_bank_m",
    "turn_radius_from_rate_m",
    "turn_rate_for_radius_nav_dps",
    "turn_rate_for_radius_rad_s",
]
