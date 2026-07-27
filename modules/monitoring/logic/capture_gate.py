# -*- coding: utf-8 -*-
"""촬영 커버리지 적립 공통 게이트.

Line/Area 촬영 진행(커버리지) 적립은 "기체가 실제로 계획 스윕을 촬영 중"일 때만
허용해야 한다. 내부 sim은 필드가 정형적이라 문제가 드러나지 않지만, 실제 운용
시뮬레이션 피드에서는 아래와 같은 오적립이 발생한다:

- 다음협업기저임무 등에서 좌표지향(고정)모드로 원거리에서 임무영역을 미리
  주시하면 대형 풋프린트가 영역에 겹쳐 스윕한 것으로 적립됨
- 선회/대기 중 센서가 우발적으로 영역을 지나가며 일부가 적립됨

품질(요구 공간해상도) 모니터가 센서 운용모드·비행/촬영 상태로 측정 샘플을
게이팅하는 것과 동일한 원칙을 적립 경로 전체에 적용한다.

0401 SensorInfo.operationalMode:
  0=None, 1=좌표지향모드, 2=구간탐색/구역감시모드, 3=자동추적모드,
  4=기체고정모드, 5=자동주사모드
→ 스윕 적립은 2(구간탐색/구역감시)에서만 유효. 0/None은 필드를 채우지 않는
  레거시 피드와의 호환을 위해 통과시킨다(차단은 명시적 비스윕 모드에 한정).
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any

SENSOR_OPERATION_MODE_SWEEP = 2
_BLOCKED_SENSOR_OPERATION_MODES = frozenset({1, 3, 4, 5})

_MAX_SENSOR_OFFSET_ENV = "KU_CAPTURE_GATE_MAX_SENSOR_OFFSET_M"
_MAX_SENSOR_OFFSET_DEFAULT_M = 1500.0
_FOOTPRINT_WIDTH_RATIO_ENV = "KU_CAPTURE_GATE_FOOTPRINT_WIDTH_RATIO"
_FOOTPRINT_WIDTH_RATIO_DEFAULT = 3.0
_FOOTPRINT_WIDTH_MARGIN_M = 60.0


@dataclass(frozen=True)
class CaptureGateDecision:
    allowed: bool
    reason: str

    def __bool__(self) -> bool:
        return bool(self.allowed)


_ALLOWED = CaptureGateDecision(True, "")


def _as_int(value: object | None) -> int | None:
    try:
        return None if value is None else int(value)
    except Exception:
        return None


def _as_float(value: object | None) -> float | None:
    try:
        return None if value is None else float(value)
    except Exception:
        return None


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None or not str(raw).strip():
        return float(default)
    try:
        return float(str(raw).strip())
    except Exception:
        return float(default)


def max_sensor_offset_m() -> float:
    return max(0.0, _env_float(_MAX_SENSOR_OFFSET_ENV, _MAX_SENSOR_OFFSET_DEFAULT_M))


def footprint_width_ratio_limit() -> float:
    return max(1.0, _env_float(_FOOTPRINT_WIDTH_RATIO_ENV, _FOOTPRINT_WIDTH_RATIO_DEFAULT))


def _coord_lat_lon(value: object | None) -> tuple[float, float] | None:
    if not isinstance(value, dict):
        return None
    lat = _as_float(value.get("latitude") or value.get("Latitude"))
    lon = _as_float(value.get("longitude") or value.get("Longitude"))
    if lat is None or lon is None:
        return None
    if not (-90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0):
        return None
    return float(lat), float(lon)


def _ground_distance_m(
    left: tuple[float, float] | None,
    right: tuple[float, float] | None,
) -> float | None:
    if left is None or right is None:
        return None
    lat1 = math.radians(left[0])
    lon1 = math.radians(left[1])
    lat2 = math.radians(right[0])
    lon2 = math.radians(right[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 6_371_000.0 * (2.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a))))


def footprint_width_m(footprint_corners: list[Any] | None) -> float | None:
    """모서리 4점(UL, UR, LR, LL)의 상/하변 평균 폭[m]."""
    if not isinstance(footprint_corners, list) or len(footprint_corners) < 4:
        return None
    pts = [_coord_lat_lon(item) for item in footprint_corners[:4]]
    if any(pt is None for pt in pts):
        return None
    top = _ground_distance_m(pts[0], pts[1])
    bottom = _ground_distance_m(pts[3], pts[2])
    if top is not None and bottom is not None:
        return (float(top) + float(bottom)) * 0.5
    return top if top is not None else bottom


def sensor_mode_allows_capture(sensor_operation_mode: int | None) -> bool:
    mode = _as_int(sensor_operation_mode)
    if mode is None:
        return True
    return int(mode) not in _BLOCKED_SENSOR_OPERATION_MODES


def evaluate_capture_gate(
    agent_state: dict[str, Any] | None,
    *,
    width_hint_m: float | None = None,
    allow_sensor_offset_bypass: bool = False,
) -> CaptureGateDecision:
    """0401 단일 기체 상태로 커버리지 적립 가능 여부를 판정한다.

    필드가 없는(None) 항목의 검사는 통과 처리해 레거시/부분 피드와의 호환을
    유지한다 — 차단은 명시적으로 관측된 비스윕 상태에 한해서만 일어난다.
    """
    state = agent_state if isinstance(agent_state, dict) else {}

    # ``filming`` is a three-state runtime value (0=off, 1=capturing,
    # 2=finished).  Older feeds omit it, so absence keeps the legacy fallback;
    # once the feed explicitly supplies a value, only the active state may
    # accrue coverage.  In particular, state 2 is common during a turn/exit
    # and must not keep carving the assignment with the last footprint.
    raw_filming = state.get("filming")
    if raw_filming is not None:
        filming = _as_int(raw_filming)
        if filming != 1:
            reason = "filming_off" if filming == 0 else "filming_not_active"
            return CaptureGateDecision(False, reason)

    sensor_mode = _as_int(state.get("sensor_operation_mode"))
    if not sensor_mode_allows_capture(sensor_mode):
        return CaptureGateDecision(False, f"sensor_mode_{int(sensor_mode)}")

    flying = _as_int(state.get("flying"))
    if flying == 0:
        return CaptureGateDecision(False, "not_flying")

    offset_limit_m = max_sensor_offset_m()
    if offset_limit_m > 0.0:
        offset_m = _ground_distance_m(
            _coord_lat_lon(state.get("coordinate")),
            _coord_lat_lon(state.get("sensor_center_coordinate")),
        )
        if (
            offset_m is not None
            and float(offset_m) > float(offset_limit_m)
            and not bool(allow_sensor_offset_bypass)
        ):
            return CaptureGateDecision(False, "sensor_far_stare")

    hint = _as_float(width_hint_m)
    if hint is not None and float(hint) > 0.0:
        observed_width_m = footprint_width_m(state.get("footprint_corners"))
        if observed_width_m is not None:
            limit_m = max(
                float(hint) * footprint_width_ratio_limit(),
                float(hint) + _FOOTPRINT_WIDTH_MARGIN_M,
            )
            if float(observed_width_m) > limit_m:
                return CaptureGateDecision(False, "footprint_oversize")

    return _ALLOWED
