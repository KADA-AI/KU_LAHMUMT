# -*- coding: utf-8 -*-
"""FOV DB 조회를 대체하는 실시간 촬영 기하 선택기 (물리 기반, DB-free).

기존 구조는 "요구폭 → FOV DB 표 조회 → {fov, sep}"였다.  이 모듈은 같은 자리에
꽂히는 계산기다: 진입 WP에서 측정한 횡 오프셋(이격)과 요구폭으로 촬영 슬랜트를
구하고, 요구공간해상도(JANT 2024 면적 GSD — 모니터링 품질탭과 동일 식·기본값)를
만족하는 최대 FOV에서 30% 작게(불확실성 여유) 선택한다.

    슬랜트 최대   R_max(fov)  :  프레임 면적 4R²·tan(h/2)·tan(v/2) = 허용면적
    선택         fov = min(GSD 최대 FOV, 카메라 최대 FOV) × margin(0.7)
    이격(sep)    선택의 역함수 — 이 FOV가 나오는 표준 이격 (왕복 일관)
    속도/간격    기존 capture_geometry 법칙이 FOV에서 자동 유도 (여기서 안 건드림)

폭 검증은 fps 15 · 좌우 중첩 50% 기준으로 "한 행에서 덮을 수 있는 폭"을
계산해 보고한다.  요구폭보다 좁아도 실패가 아니다 — 계획기는 지금처럼 스윕
줄 수를 늘려 흡수한다(기존 DB의 '폭 부족 시 최대 width 행 강등'과 같은 밸브).

모든 공개 함수는 실패 시 None(또는 0.0)을 반환하고 예외를 삼킨다.  호출부는
None이면 기존 FOV DB 경로로 그대로 진행한다(fail-open).  런타임 킬스위치
``physics_fov_selection_enabled``(기본 켜짐)로 전체를 끌 수 있다.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

from .capture_geometry import (
    capture_altitude_m,
    capture_params,
    capture_speed_plan,
    diagonal_to_horizontal_vertical_fov,
)
from .runtime_settings import (
    get_runtime_float,
    get_runtime_value,
    load_runtime_settings_readonly,
)

# ── 기본값 (전부 런타임 키로 교체 가능) ──────────────────────────────────────
PHYSICS_ENABLED_DEFAULT = True            # physics_fov_selection_enabled
PHYSICS_FPS = 15.0                        # physics_capture_fps  (폭 검증용)
PHYSICS_LATERAL_OVERLAP = 0.5             # physics_lateral_overlap_ratio (좌우 중첩)
PHYSICS_FOV_MARGIN = 0.7                  # physics_fov_margin_ratio ("큰거에서 30% 작게")
PHYSICS_FOV_MIN_DEG = 1.2                 # physics_fov_min_deg  (카메라 줌 하한)
PHYSICS_FOV_MAX_DEG = 8.2                 # physics_fov_max_deg  (카메라 줌 상한)
# AREA 스윕 전용 마진 — 일반 margin(0.7) 대신 적용한다.  평지 GSD 모델로는
# 나디어 근방에서 카메라 상한×0.7 이 항상 "만족"으로 나오지만, 실비행은 짧은
# 행 사이 급선회·뱅킹·지형 스트레치로 프레임이 모델보다 훨씬 커져 실측
# 요구공간해상도가 깨진다.  0.6 이면 고도 1000 m 기준 실슬랜트 ~2.6 km 까지
# 요구 풋프린트가 유지된다(프레임 면적 ∝ tan²(FOV/2)).
PHYSICS_AREA_FOV_MARGIN = 0.6             # physics_area_fov_margin_ratio
# 진입 WP가 아직 없어 이격을 측정 못 했을 때의 기본 보어사이트(연직 기준).
# 이격 = 고도 × tan(기본 틸트).  45°는 이격 = 고도(1 km)라 초기계획 첫 WP가
# 회랑에서 1 km 밖에 찍혔고, 그 이격이 뒤 스윕 앵커에도 그대로 적용돼 계속
# 멀리서 촬영했다.  14°면 이격 ≈ 249 m 로 재계획이 실측 진입 기하에서 만드는
# 117~250 m 대역에 들어간다.  이 값이 FOV 선택의 입력이므로(같은 함수에서
# 슬랜트를 만든다) FOV·커버폭·스윕 줄수가 같은 거리로 함께 결정된다 —
# 실측: 45°/14° 모두 FOV 6.56°, 커버폭 749~838 m, 요구폭 1 km 에 2줄.
PHYSICS_DEFAULT_TILT_DEG = 14.0           # physics_default_boresight_tilt_deg

# 요구공간해상도 스펙 — 모니터링 quality_monitor_tab의 _SR_DEFAULT_* 와 동일.
#   γ = (표적 가로×세로) / (최소 요구 픽셀 가로×세로) [m²/px]
#   허용 풋프린트 면적 = γ × (이미지 가로×세로 px)
PHYSICS_OBJ_W_M = 6.0                     # physics_obj_w_m
PHYSICS_OBJ_H_M = 3.6                     # physics_obj_h_m
PHYSICS_OBJ_MIN_PX_X = 38                 # physics_obj_min_px_x
PHYSICS_OBJ_MIN_PX_Y = 22                 # physics_obj_min_px_y
PHYSICS_IMG_W_PX = 1920.0                 # physics_img_w_px
PHYSICS_IMG_H_PX = 1080.0                 # physics_img_h_px


def _runtime_payload(runtime_cfg: Dict[str, Any] | None) -> Dict[str, Any]:
    """Settings document for a physics lookup.

    Read-only: nothing in this module mutates the payload, and these lookups
    happen per waypoint, so copying the whole document each time was pure
    overhead.
    """
    if isinstance(runtime_cfg, dict):
        return runtime_cfg
    try:
        payload = load_runtime_settings_readonly()
    except Exception:
        payload = {}
    return payload if isinstance(payload, dict) else {}


def physics_selection_enabled(runtime_cfg: Dict[str, Any] | None = None) -> bool:
    payload = _runtime_payload(runtime_cfg)
    try:
        raw = get_runtime_value(
            "physics_fov_selection_enabled", PHYSICS_ENABLED_DEFAULT, payload
        )
    except Exception:
        return bool(PHYSICS_ENABLED_DEFAULT)
    if isinstance(raw, str):
        return raw.strip().lower() not in {"0", "false", "off", "no"}
    return bool(raw)


def physics_params(runtime_cfg: Dict[str, Any] | None = None) -> Dict[str, float]:
    """물리 선택에 쓰는 손잡이 일습 (기본값 + 런타임 오버라이드)."""

    payload = _runtime_payload(runtime_cfg)

    def _f(key: str, default: float, low: float, high: float) -> float:
        value = get_runtime_float(key, default, payload)
        if not math.isfinite(value):
            return float(default)
        return float(min(max(value, low), high))

    fov_min = _f("physics_fov_min_deg", PHYSICS_FOV_MIN_DEG, 0.1, 60.0)
    fov_max = _f("physics_fov_max_deg", PHYSICS_FOV_MAX_DEG, fov_min, 60.0)
    return {
        "fps": _f("physics_capture_fps", PHYSICS_FPS, 1.0, 120.0),
        "lateral_overlap": _f("physics_lateral_overlap_ratio", PHYSICS_LATERAL_OVERLAP, 0.0, 0.9),
        "fov_margin": _f("physics_fov_margin_ratio", PHYSICS_FOV_MARGIN, 0.3, 1.0),
        "area_fov_margin": _f(
            "physics_area_fov_margin_ratio", PHYSICS_AREA_FOV_MARGIN, 0.2, 1.0
        ),
        "fov_min_deg": fov_min,
        "fov_max_deg": fov_max,
        "default_tilt_deg": _f(
            "physics_default_boresight_tilt_deg", PHYSICS_DEFAULT_TILT_DEG, 0.0, 80.0
        ),
        "obj_w_m": _f("physics_obj_w_m", PHYSICS_OBJ_W_M, 0.1, 1000.0),
        "obj_h_m": _f("physics_obj_h_m", PHYSICS_OBJ_H_M, 0.1, 1000.0),
        "obj_min_px_x": _f("physics_obj_min_px_x", float(PHYSICS_OBJ_MIN_PX_X), 1.0, 10000.0),
        "obj_min_px_y": _f("physics_obj_min_px_y", float(PHYSICS_OBJ_MIN_PX_Y), 1.0, 10000.0),
        "img_w_px": _f("physics_img_w_px", PHYSICS_IMG_W_PX, 16.0, 100000.0),
        "img_h_px": _f("physics_img_h_px", PHYSICS_IMG_H_PX, 16.0, 100000.0),
    }


# 한 스윕 행이 fps 15 · 좌우 중첩 50% 로 덮을 수 있는 좌우 폭의 보수적 하한.
# (프레임 폭 ≈100 m × 15장 × (1−0.5) ≈ 750 m 에서 여유를 둔 값.)
# 할당된 AREA 의 행 폭이 이 값을 넘으면 그 소유 기체의 영역을 두 갈래로 쪼개
# 순차 수행한다.  0 이면 비활성.
AREA_SEQUENTIAL_SPLIT_WIDTH_M = 700.0     # area_sequential_split_width_m
AREA_SEQUENTIAL_SPLIT_MAX_WIDTH_M = 900.0  # area_sequential_split_max_width_m


def area_sequential_split_width_m(runtime_cfg: Dict[str, Any] | None = None) -> float:
    """AREA 순차 2분할 임계 폭 (m).  0 이하면 비활성."""

    try:
        value = get_runtime_float(
            "area_sequential_split_width_m",
            AREA_SEQUENTIAL_SPLIT_WIDTH_M,
            _runtime_payload(runtime_cfg),
        )
    except Exception:
        return float(AREA_SEQUENTIAL_SPLIT_WIDTH_M)
    if not math.isfinite(value) or value < 0.0:
        return float(AREA_SEQUENTIAL_SPLIT_WIDTH_M)
    return float(value)


def area_sequential_split_max_width_m(
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    """Allowed AREA stage width while retaining 700 m as the target."""

    target_m = area_sequential_split_width_m(runtime_cfg)
    if target_m <= 0.0:
        return 0.0
    try:
        value = get_runtime_float(
            "area_sequential_split_max_width_m",
            AREA_SEQUENTIAL_SPLIT_MAX_WIDTH_M,
            _runtime_payload(runtime_cfg),
        )
    except Exception:
        value = float(AREA_SEQUENTIAL_SPLIT_MAX_WIDTH_M)
    if not math.isfinite(value) or value <= 0.0:
        value = float(AREA_SEQUENTIAL_SPLIT_MAX_WIDTH_M)
    return float(max(target_m, value))


def projected_span_m_xy(points_xy: Any, bearing_deg: Any) -> float:
    """XY 점들을 d0303 폭 기준축(nx=cosθ, ny=−sinθ)에 투영한 스팬 (m).

    d0303 `_projected_polygon_span_m` 과 같은 규약 — FOV 요구폭(widthRefM)을
    만드는 그 축이며, AREA 순차 2분할의 폭 판정도 같은 축을 쓴다.
    """

    try:
        theta = math.radians(float(bearing_deg or 0.0))
    except Exception:
        return 0.0
    nx, ny = math.cos(theta), -math.sin(theta)
    values: list[float] = []
    for point in points_xy or []:
        if isinstance(point, dict):
            x = point.get("x", point.get(0))
            y = point.get("y", point.get(1))
        elif isinstance(point, (tuple, list)) and len(point) >= 2:
            x, y = point[0], point[1]
        else:
            continue
        try:
            values.append(nx * float(x) + ny * float(y))
        except Exception:
            continue
    if len(values) < 2:
        return 0.0
    return float(max(values) - min(values))


def projected_span_m_llh(coords: Any, bearing_deg: Any) -> float:
    """위경도 좌표 목록의 같은 축 투영 스팬 (m, 등장방형 근사)."""

    rows = [c for c in (coords or []) if isinstance(c, dict)]
    if len(rows) < 3:
        return 0.0
    try:
        lat0 = float(rows[0].get("latitude"))
        lon0 = float(rows[0].get("longitude"))
    except Exception:
        return 0.0
    cos_lat = math.cos(math.radians(lat0))
    points_xy = []
    for c in rows:
        try:
            points_xy.append(
                (
                    (float(c["longitude"]) - lon0) * 111_320.0 * cos_lat,
                    (float(c["latitude"]) - lat0) * 111_132.9,
                )
            )
        except Exception:
            continue
    return projected_span_m_xy(points_xy, bearing_deg)


def max_sweep_row_chord_m_xy(
    points_xy: Any,
    bearing_deg: Any,
    *,
    lane_samples: int = 24,
) -> float:
    """스윕 행 축을 따라 폴리곤을 가로지르는 최대 현(chord) 길이 (m).

    바운딩 투영(projected_span_m_xy)은 비스듬한 도형에서 긴 변이 섞여 행
    길이를 과대평가한다(500 m 폭이 6.7° 회전만으로 731 m 로 보임).  실제
    스윕 행은 레인 위치별 폴리곤 절단선이므로, 레인을 샘플링해 가장 긴
    연속 현을 잰다.  shapely 가 없으면 투영 스팬으로 보수적 폴백.
    """

    try:
        theta = math.radians(float(bearing_deg or 0.0))
    except Exception:
        return 0.0
    # 행 축 u = (cosθ, −sinθ) — projected_span_m_xy 와 동일한 폭 기준축.
    ux, uy = math.cos(theta), -math.sin(theta)
    vx, vy = -uy, ux  # 레인 진행 축 (행 축과 직교)
    uv_points: list[tuple[float, float]] = []
    for point in points_xy or []:
        if isinstance(point, (tuple, list)) and len(point) >= 2:
            x, y = point[0], point[1]
        elif isinstance(point, dict):
            x, y = point.get("x"), point.get("y")
        else:
            continue
        try:
            fx, fy = float(x), float(y)
        except Exception:
            continue
        uv_points.append((ux * fx + uy * fy, vx * fx + vy * fy))
    if len(uv_points) < 3:
        return 0.0
    try:
        from shapely.geometry import LineString as _LineString, Polygon as _Polygon

        polygon = _Polygon(uv_points)
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
        if polygon.is_empty:
            return 0.0
        min_u, min_v, max_u, max_v = polygon.bounds
        pad = max(1.0, (max_u - min_u) * 0.05)
        samples = max(3, int(lane_samples))
        longest = 0.0
        for index in range(samples):
            v = min_v + (max_v - min_v) * (index + 0.5) / samples
            cut = polygon.intersection(
                _LineString([(min_u - pad, v), (max_u + pad, v)])
            )
            pieces = getattr(cut, "geoms", [cut])
            for piece in pieces:
                length = float(getattr(piece, "length", 0.0) or 0.0)
                if length > longest:
                    longest = length
        return float(longest)
    except Exception:
        return projected_span_m_xy(points_xy, bearing_deg)


def max_sweep_row_chord_m_llh(coords: Any, bearing_deg: Any) -> float:
    """위경도 좌표 목록의 최대 스윕 행 현 길이 (m, 등장방형 근사)."""

    rows = [c for c in (coords or []) if isinstance(c, dict)]
    if len(rows) < 3:
        return 0.0
    try:
        lat0 = float(rows[0].get("latitude"))
        lon0 = float(rows[0].get("longitude"))
    except Exception:
        return 0.0
    cos_lat = math.cos(math.radians(lat0))
    points_xy = []
    for c in rows:
        try:
            points_xy.append(
                (
                    (float(c["longitude"]) - lon0) * 111_320.0 * cos_lat,
                    (float(c["latitude"]) - lat0) * 111_132.9,
                )
            )
        except Exception:
            continue
    return max_sweep_row_chord_m_xy(points_xy, bearing_deg)


def route_offset_scale(runtime_cfg: Dict[str, Any] | None = None) -> float:
    """SEP 중 실제로 경로 이격이 되는 비율 (line_route_offset_scale).

    비행 경로는 SEP 전체가 아니라 SEP × 이 비율만큼 회랑에서 떨어진다
    (d0303 `_line_wp_xy_from_sweep`, line_runner `active_route_offset_m`).
    FOV 는 '실제로 그 거리에서 찍는지'로 정해져야 하므로 이 비율을 반영한다.
    """

    try:
        value = get_runtime_float(
            "line_route_offset_scale", 1.0, _runtime_payload(runtime_cfg)
        )
    except Exception:
        return 1.0
    if not math.isfinite(value) or value <= 0.0:
        return 1.0
    return float(min(value, 1.0))


def required_footprint_area_m2(runtime_cfg: Dict[str, Any] | None = None) -> float:
    """JANT 면적 GSD의 허용 풋프린트 면적 s_req = γ × 전체 픽셀."""

    params = physics_params(runtime_cfg)
    gamma = (params["obj_w_m"] * params["obj_h_m"]) / (
        params["obj_min_px_x"] * params["obj_min_px_y"]
    )
    return float(gamma * params["img_w_px"] * params["img_h_px"])


def _aspect_ratio(runtime_cfg: Dict[str, Any] | None) -> float:
    try:
        aspect = float(capture_params(_runtime_payload(runtime_cfg))["aspect_ratio"])
    except Exception:
        aspect = 16.0 / 9.0
    return aspect if aspect > 0.0 else 16.0 / 9.0


def frame_area_m2(
    fov_deg: float,
    slant_m: float,
    runtime_cfg: Dict[str, Any] | None = None,
    *,
    altitude_m: Optional[float] = None,
) -> Optional[float]:
    """슬랜트 R·고도 h에서 대각 FOV 프레임의 실제 지상 면적.

    경사로 찍으면 세로 footprint 가 1/cosθ 로 늘어난다.  지상 촬영에서는
    cosθ = h/R 이므로:

        가로 w = 2R·tan(h/2)
        세로 l = 2R·tan(v/2) / cosθ = 2R·tan(v/2) · R/h
        면적   = 4R³·tan(h/2)·tan(v/2) / h

    나디어(R = h)에서는 기존 4R²·tan·tan 과 일치한다.  altitude_m 을 주지
    않으면 운용 고도(capture_altitude_m)를 쓴다.
    """

    try:
        fov = float(fov_deg)
        slant = float(slant_m)
    except Exception:
        return None
    if fov <= 0.0 or slant <= 0.0:
        return None
    altitude = (
        float(altitude_m)
        if altitude_m is not None and float(altitude_m) > 0.0
        else capture_altitude_m(_runtime_payload(runtime_cfg))
    )
    # 슬랜트가 고도보다 짧을 수는 없다(지상 표적).  수치 방어만 해 둔다.
    altitude = min(max(altitude, 1.0), slant)
    horizontal_deg, vertical_deg = diagonal_to_horizontal_vertical_fov(
        fov, aspect_ratio=_aspect_ratio(runtime_cfg)
    )
    return float(
        4.0
        * slant
        * slant
        * (slant / altitude)
        * math.tan(math.radians(horizontal_deg) * 0.5)
        * math.tan(math.radians(vertical_deg) * 0.5)
    )


def max_slant_for_fov(
    fov_deg: float,
    runtime_cfg: Dict[str, Any] | None = None,
    *,
    altitude_m: Optional[float] = None,
) -> Optional[float]:
    """이 FOV가 요구해상도를 만족하는 최대 슬랜트 (면적식의 역산, 닫힌형).

    면적 = 4R³·tanΓh·tanΓv / h  →  R_max = ∛(s_req·h / (4·tanΓh·tanΓv)).
    """

    try:
        fov = float(fov_deg)
    except Exception:
        return None
    if fov <= 0.0:
        return None
    altitude = (
        float(altitude_m)
        if altitude_m is not None and float(altitude_m) > 0.0
        else capture_altitude_m(_runtime_payload(runtime_cfg))
    )
    horizontal_deg, vertical_deg = diagonal_to_horizontal_vertical_fov(
        fov, aspect_ratio=_aspect_ratio(runtime_cfg)
    )
    tan_product = math.tan(math.radians(horizontal_deg) * 0.5) * math.tan(
        math.radians(vertical_deg) * 0.5
    )
    if tan_product <= 0.0:
        return None
    s_req = required_footprint_area_m2(runtime_cfg)
    slant = (s_req * max(altitude, 1.0) / (4.0 * tan_product)) ** (1.0 / 3.0)
    # 지상 표적이므로 최소 슬랜트는 고도(나디어)다.
    return float(max(slant, altitude))


def max_fov_for_slant(
    slant_m: float,
    runtime_cfg: Dict[str, Any] | None = None,
    *,
    altitude_m: Optional[float] = None,
) -> Optional[float]:
    """슬랜트 R·고도 h에서 요구해상도를 정확히 만족하는 최대 대각 FOV (닫힌형).

    tan(h/2)·tan(v/2) = a·t²/(1+a²)  (t = tan(diag/2), a = 종횡비) 이므로
    t = √(s_req·h·(1+a²) / (4·a·R³)).
    """

    try:
        slant = float(slant_m)
    except Exception:
        return None
    if slant <= 0.0:
        return None
    altitude = (
        float(altitude_m)
        if altitude_m is not None and float(altitude_m) > 0.0
        else capture_altitude_m(_runtime_payload(runtime_cfg))
    )
    altitude = min(max(altitude, 1.0), slant)
    aspect = _aspect_ratio(runtime_cfg)
    s_req = required_footprint_area_m2(runtime_cfg)
    t_squared = s_req * altitude * (1.0 + aspect * aspect) / (4.0 * aspect * slant**3)
    if t_squared <= 0.0:
        return None
    return float(math.degrees(2.0 * math.atan(math.sqrt(t_squared))))


def select_fov_deg(
    slant_m: float,
    runtime_cfg: Dict[str, Any] | None = None,
    *,
    altitude_m: Optional[float] = None,
) -> Optional[float]:
    """실현가능 최대 FOV(min(GSD 한계, 카메라 상한))에서 margin(30%)만큼 작게."""

    if not physics_selection_enabled(runtime_cfg):
        return None
    gsd_max = max_fov_for_slant(slant_m, runtime_cfg, altitude_m=altitude_m)
    if gsd_max is None or gsd_max <= 0.0:
        return None
    params = physics_params(runtime_cfg)
    feasible_max = min(float(gsd_max), params["fov_max_deg"])
    selected = feasible_max * params["fov_margin"]
    return float(max(selected, params["fov_min_deg"]))


def physics_standoff_for_fov(
    fov_deg: float,
    width_m: float = 0.0,
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    """이 FOV가 우리 선택 규칙에서 나오게 되는 표준 횡 이격 (선택의 역함수).

    선택은 "먼 가장자리 슬랜트의 실현가능 최대 FOV × margin"이므로, 역으로
    margin을 걷어낸 FOV의 GSD 최대 슬랜트에서 고도·반폭을 빼면 이격이 나온다.
    카메라 상한에 눌려 있던 FOV(= margin 적용 후에도 상한×margin)는 GSD 여유가
    남아돌므로 역산 슬랜트가 훨씬 커진다 — GSD 최대 슬랜트를 그대로 쓰되
    호출부의 기존 이격 힌트와 min을 취하는 것은 호출부 몫이다.
    _fov_db_min_sep_for_fov(DB의 그 FOV 최소 sep) 대체용.
    """

    try:
        if not physics_selection_enabled(runtime_cfg):
            return 0.0
        params = physics_params(runtime_cfg)
        margin = params["fov_margin"]
        if margin <= 0.0:
            return 0.0
        unmargined_fov = float(fov_deg) / margin
        altitude = capture_altitude_m(_runtime_payload(runtime_cfg))
        slant = max_slant_for_fov(unmargined_fov, runtime_cfg, altitude_m=altitude)
        if slant is None or slant <= 0.0:
            return 0.0
        if slant <= altitude:
            return 0.0
        ground_reach = math.sqrt(slant * slant - altitude * altitude)
        half_width = max(float(width_m), 0.0) * 0.5
        return float(max(ground_reach - half_width, 0.0))
    except Exception:
        return 0.0


def _coverable_width_m(
    fov_deg: float,
    lateral_offset_m: float,
    width_m: float,
    altitude_m: float,
    runtime_cfg: Dict[str, Any] | None,
) -> Dict[str, float]:
    """fps·좌우중첩 제약에서 한 행이 덮을 수 있는 폭 (보수적 추정).

    행 시간은 기존 촬영 간격 법칙(scanIntervalSec)에서 가져오고, 프레임 폭은
    회랑 구간에서 가장 좁은(가까운 쪽) 프레임을 쓴다.  속도·간격 자체는
    capture_geometry가 결정하며 여기서는 읽기만 한다.
    """

    params = physics_params(runtime_cfg)
    plan = capture_speed_plan(
        fov_deg, runtime_cfg=_runtime_payload(runtime_cfg)
    )
    if not isinstance(plan, dict):
        return {"coverWidthM": 0.0, "framesPerRow": 0.0}
    scan_interval_s = float(plan.get("scanIntervalSec", 0.0) or 0.0)
    frames = params["fps"] * scan_interval_s
    near_ground_m = max(0.0, abs(float(lateral_offset_m)) - max(float(width_m), 0.0) * 0.5)
    near_slant_m = math.hypot(float(altitude_m), near_ground_m)
    horizontal_deg, _vertical_deg = diagonal_to_horizontal_vertical_fov(
        float(fov_deg), aspect_ratio=_aspect_ratio(runtime_cfg)
    )
    frame_width = 2.0 * near_slant_m * math.tan(math.radians(horizontal_deg) * 0.5)
    cover = frames * frame_width * (1.0 - params["lateral_overlap"])
    return {
        "coverWidthM": float(max(cover, 0.0)),
        "framesPerRow": float(max(frames, 0.0)),
        "frameWidthM": float(frame_width),
        "scanIntervalSec": scan_interval_s,
    }


def physics_line_row(
    width_m: float,
    sep_hint_m: float = 0.0,
    *,
    line_length_m: Optional[float] = None,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Optional[Dict[str, Any]]:
    """LINE 요구폭 + 진입 WP에서 측정한 이격 → FOV DB 행과 같은 모양의 결과.

    반환 {"width","sep","fov","vel"}는 기존 DB 행 계약 그대로다:
      fov   선택 FOV (실현가능 최대 × margin)
      sep   이격 — 측정 힌트가 있으면 그 값(진입 WP 우선), 없으면 표준 이격
      vel   capture 법칙의 기체 속도 (km/h — DB vel과 동일 단위)
      width fps·중첩 제약으로 이 행이 덮을 수 있는 폭 (요구폭보다 작을 수 있음
            — 그 경우 계획기가 지금처럼 스윕 줄 수로 흡수)
    "physics" 키에 검증용 메타(슬랜트, 보어사이트 틸트, GSD 면적)를 싣는다.
    """

    try:
        if not physics_selection_enabled(runtime_cfg):
            return None
        width = max(float(width_m), 0.0)
        measured = max(float(sep_hint_m or 0.0), 0.0)
        payload = _runtime_payload(runtime_cfg)
        altitude = capture_altitude_m(payload)
        params = physics_params(runtime_cfg)
        # 진입 WP에서 측정한 이격이 최우선.  없으면 기본 보어사이트(45°)로
        # 이격 = 고도×tanθ.  GSD 한계 이격을 그대로 쓰면 경로가 회랑에서
        # 수 km 밖에 깔릴 수 있으므로 기본값으로 삼지 않는다.
        offset = (
            measured
            if measured > 0.0
            else altitude * math.tan(math.radians(params["default_tilt_deg"]))
        )
        # FOV·SEP·GSD 상한은 모두 SEP 공간에서 계산한다.  line_route_offset_scale
        # 은 경로를 회랑에 붙이는 노브일 뿐이며, 여기에 곱해 넣으면 FOV 선택과
        # 스윕 간격이 함께 흔들려 첫 LINE 임무가 좁은 FOV 로 떨어지고 영역을
        # 채우지 못한다.  SEP 원값 기준이 보수적(GSD 여유가 남는 쪽)이다.
        far_ground_m = offset + width * 0.5
        slant_far = math.hypot(altitude, far_ground_m)
        fov = select_fov_deg(slant_far, runtime_cfg, altitude_m=altitude)
        if fov is None or fov <= 0.0:
            return None
        # 선택된 FOV의 GSD 한계를 넘는 이격은 허용하지 않는다(상한).
        gsd_cap = physics_standoff_for_fov(fov, width, runtime_cfg)
        sep_out = min(offset, gsd_cap) if gsd_cap > 0.0 else offset
        plan = capture_speed_plan(fov, line_length_m=line_length_m, runtime_cfg=payload)
        vel_kmh = float(plan.get("aircraftSpeedKmh", 0.0) or 0.0) if isinstance(plan, dict) else 0.0
        coverage = _coverable_width_m(fov, sep_out, width, altitude, runtime_cfg)
        area_far = frame_area_m2(fov, slant_far, runtime_cfg, altitude_m=altitude)
        return {
            "width": float(coverage.get("coverWidthM", 0.0)),
            "sep": float(sep_out),
            "fov": float(fov),
            "vel": float(vel_kmh),
            "physics": {
                "source": "capture_physics",
                "aglM": float(altitude),
                "slantFarM": float(slant_far),
                "boresightTiltDeg": float(math.degrees(math.atan2(offset, altitude))),
                "requiredAreaM2": float(required_footprint_area_m2(runtime_cfg)),
                "frameAreaFarM2": float(area_far if area_far is not None else 0.0),
                "framesPerRow": float(coverage.get("framesPerRow", 0.0)),
                "coverWidthM": float(coverage.get("coverWidthM", 0.0)),
                "requestedWidthM": float(width),
                "widthSatisfied": bool(
                    float(coverage.get("coverWidthM", 0.0)) + 1e-6 >= width
                ),
                "sepHintM": float(measured),
                "gsdStandoffCapM": float(gsd_cap),
            },
        }
    except Exception:
        return None


def physics_route_offset_cap_m(
    fov_deg: float,
    default_sep_m: float,
    width_m: float = 0.0,
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    """경로 이격의 물리 상한 적용 — DB의 'FOV별 최소 sep' 조회 대체.

    측정/기존 이격(default)이 최우선이고, 물리는 그 값이 선택 FOV의 GSD 한계
    이격을 넘지 않도록 자르기만 한다.  default가 없으면 0을 돌려주어 호출부의
    기존 기본값 체계가 그대로 흐르게 한다.
    """

    try:
        if not physics_selection_enabled(runtime_cfg):
            return 0.0
        default = max(float(default_sep_m or 0.0), 0.0)
        if default <= 0.0:
            return 0.0
        cap = physics_standoff_for_fov(fov_deg, width_m, runtime_cfg)
        if cap <= 0.0:
            return default
        # 둘 다 SEP 공간이다: route_offset_scale 로 환산하면 상한이 1/scale 배로
        # 부풀어 GSD 보증이 무력화된다.
        return float(min(default, cap))
    except Exception:
        return 0.0


def physics_sweep_row_fov_deg(
    *,
    row_length_m: float,
    lateral_offset_m: float = 0.0,
    altitude_m: Optional[float] = None,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Optional[float]:
    """정지 WP가 한 행 전체를 팬할 때 행 길이까지 고려한 FOV 선택.

    카메라는 한 waypoint 에서 스윕 행을 따라 팬하며 찍으므로, 그 행의 먼 끝단이
    실제 최악 슬랜트다.  진입 이격만 보면 행 길이를 통째로 놓치게 되고, 실측에서
    횡거리가 이격의 3배 이상까지 벌어졌다.  여기서는 (이격, 행 절반 길이)를
    직각 성분으로 합쳐 최악 지상거리를 잡는다.

    주의: corridor LINE처럼 기체가 회랑 길이축을 따라 이동하면서 횡 스윕을
    촬영하는 경로에는 쓰면 안 된다. 그 경우 행의 길이축은 항공기 이동으로
    상쇄되며 ``physics_line_row``의 할당 폭+SEP가 동시 촬영 기하다.
    """

    try:
        if not physics_selection_enabled(runtime_cfg):
            return None
        altitude = (
            float(altitude_m)
            if altitude_m is not None and float(altitude_m) > 0.0
            else capture_altitude_m(_runtime_payload(runtime_cfg))
        )
        offset = max(float(lateral_offset_m or 0.0), 0.0)
        half_row = max(float(row_length_m or 0.0), 0.0) * 0.5
        far_ground_m = math.hypot(offset, half_row) if half_row > 0.0 else offset
        slant = math.hypot(altitude, far_ground_m)
        return select_fov_deg(slant, runtime_cfg, altitude_m=altitude)
    except Exception:
        return None


def physics_area_fov_deg(
    runtime_cfg: Dict[str, Any] | None = None,
    *,
    row_length_m: float = 0.0,
    lateral_offset_m: float = 0.0,
) -> Optional[float]:
    """AREA(직하방 스윕) FOV — 실현가능 최대 × area 전용 마진.

    슬랜트는 나디어(고도)에 행 기하(WP 횡 오프셋, 행 끝단 팬)를 직각 성분으로
    더한 최악 지상거리에서 잡는다.  마진은 일반 margin(0.7)이 아니라
    physics_area_fov_margin_ratio(기본 0.6) — 좁은 조각의 짧은 행에서는 평지
    GSD 모델이 항상 카메라 상한을 허용하지만, 급선회·뱅킹이 잦은 실비행에서는
    그 FOV로 요구공간해상도가 깨지므로 area 는 더 보수적으로 내려 잡는다.
    """

    try:
        if not physics_selection_enabled(runtime_cfg):
            return None
        altitude = capture_altitude_m(_runtime_payload(runtime_cfg))
        offset = max(float(lateral_offset_m or 0.0), 0.0)
        half_row = max(float(row_length_m or 0.0), 0.0) * 0.5
        ground = math.hypot(offset, half_row)
        slant = math.hypot(altitude, ground)
        gsd_max = max_fov_for_slant(slant, runtime_cfg, altitude_m=altitude)
        if gsd_max is None or gsd_max <= 0.0:
            return None
        params = physics_params(runtime_cfg)
        feasible_max = min(float(gsd_max), params["fov_max_deg"])
        selected = feasible_max * params["area_fov_margin"]
        return float(max(selected, params["fov_min_deg"]))
    except Exception:
        return None


def _point_to_flight_leg_geometry_m(
    target: Dict[str, Any],
    leg_start: Dict[str, Any],
    leg_end: Dict[str, Any],
) -> tuple[float, float, float] | None:
    """촬영점에서 이동 중인 항공기 leg까지의 횡거리·보간고도·진행률.

    LINE의 lineSearch 좌표열은 한 WP에서 정지한 채 전부 팬하는 점 목록이
    아니라, 이전 WP→현재 WP를 비행하면서 순서대로 바라보는 스윕 열이다.
    따라서 촬영점과 현재 WP의 직선거리가 아니라 해당 비행 leg까지의 수직
    거리가 동시 촬영 기하를 나타낸다.
    """

    try:
        end_lat = float(leg_end.get("latitude"))
        end_lon = float(leg_end.get("longitude"))
        end_alt = float(leg_end.get("altitude"))
        start_lat = float(leg_start.get("latitude"))
        start_lon = float(leg_start.get("longitude"))
        start_alt = float(leg_start.get("altitude"))
        target_lat = float(target.get("latitude"))
        target_lon = float(target.get("longitude"))
    except Exception:
        return None

    cos_lat = math.cos(math.radians((start_lat + end_lat + target_lat) / 3.0))

    def _xy(lat: float, lon: float) -> tuple[float, float]:
        return (
            (lon - end_lon) * 111_320.0 * cos_lat,
            (lat - end_lat) * 111_132.9,
        )

    start_x, start_y = _xy(start_lat, start_lon)
    target_x, target_y = _xy(target_lat, target_lon)
    leg_x = -start_x
    leg_y = -start_y
    leg_len_sq = leg_x * leg_x + leg_y * leg_y
    if leg_len_sq <= 1.0:
        return None
    progress = (
        (target_x - start_x) * leg_x + (target_y - start_y) * leg_y
    ) / leg_len_sq
    progress = max(0.0, min(1.0, progress))
    closest_x = start_x + progress * leg_x
    closest_y = start_y + progress * leg_y
    ground_m = math.hypot(target_x - closest_x, target_y - closest_y)
    aircraft_alt_m = start_alt + progress * (end_alt - start_alt)
    return float(ground_m), float(aircraft_alt_m), float(progress)


def certify_waypoint_gsd_inplace(
    waypoints: Any,
    mission_info: Dict[str, Any] | None = None,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    """경로 확정 후, 실제 WP↔촬영점 기하로 요구공간해상도를 전수 검증·보정한다.

    각 waypoint의 3D 위치와 그 waypoint가 실제로 찍는 lineSearch 촬영점들
    (DEM 정규화된 지형고 포함) 사이의 슬랜트·고도차로 프레임 면적을 계산한다.
    최악 지점이 허용 풋프린트를 넘으면, 그 기하에서 요구해상도를 만족하는
    최대 FOV × margin 으로 경로 FOV를 내린다. 최종 FOV는 하나의 임무/
    pathID 계약이므로 lineSearch WP만 내리지 않고 진입 방향설정·영역
    전환 WP를 포함한 해당 경로의 모든 filmingProperty에 동일하게 반영한다.
    이미 더 좁은 WP FOV가 있으면 그 값을 경로 공통값으로 사용해 어떤
    WP의 FOV도 올리지 않는다.

    반환 요약: checked/worstSlantM/worstAglM/worstAreaM2/requiredAreaM2/
    fovBeforeDeg/fovAfterDeg/clamped/unreachable.
    """

    summary: Dict[str, Any] = {
        "checked": 0,
        "worstSlantM": 0.0,
        "worstGroundM": 0.0,
        "worstAglM": 0.0,
        "worstAreaM2": 0.0,
        "requiredAreaM2": 0.0,
        "fovBeforeDeg": 0.0,
        "fovAfterDeg": 0.0,
        "clamped": False,
        "normalized": False,
        "normalizedWaypointCount": 0,
        "unreachable": False,
    }
    try:
        if not physics_selection_enabled(runtime_cfg):
            summary["skipped"] = "physics_disabled"
            return summary
        rows = [wp for wp in (waypoints or []) if isinstance(wp, dict)]
        corridor_moving_geometry = bool(
            isinstance(mission_info, dict)
            and mission_info.get("lineList")
            and not mission_info.get("areaList")
        )
        summary["geometryMode"] = (
            "corridor_moving_leg" if corridor_moving_geometry else "waypoint_static"
        )
        params = physics_params(runtime_cfg)
        s_req = required_footprint_area_m2(runtime_cfg)
        summary["requiredAreaM2"] = float(s_req)

        worst_slant = 0.0
        worst_ground = 0.0
        worst_agl = 0.0
        worst_score = 0.0  # 면적 ∝ slant³/agl
        current_fov = 0.0
        if isinstance(mission_info, dict):
            try:
                current_fov = float(mission_info.get("FOV", 0.0) or 0.0)
            except Exception:
                current_fov = 0.0

        filming_waypoints: list[Dict[str, Any]] = []
        path_fovs: list[float] = []
        for wp_index, wp in enumerate(rows):
            filming = wp.get("filmingProperty")
            if not isinstance(filming, dict):
                continue
            filming_waypoints.append(wp)
            try:
                path_fov = float(filming.get("fieldOfView", 0.0) or 0.0)
            except Exception:
                path_fov = 0.0
            if path_fov > 0.0:
                path_fovs.append(path_fov)
            line_search = filming.get("lineSearch")
            if not isinstance(line_search, dict):
                continue
            targets = line_search.get("coordinateList")
            if not isinstance(targets, list) or not targets:
                continue
            coord = wp.get("coordinate")
            if not isinstance(coord, dict):
                continue
            try:
                wp_lat = float(coord.get("latitude"))
                wp_lon = float(coord.get("longitude"))
                wp_alt = float(coord.get("altitude"))
            except Exception:
                continue
            leg_start = None
            leg_end = coord
            if corridor_moving_geometry:
                for previous_index in range(wp_index - 1, -1, -1):
                    previous_coord = rows[previous_index].get("coordinate")
                    if isinstance(previous_coord, dict):
                        leg_start = previous_coord
                        break
                # 비정상적으로 첫 WP부터 lineSearch인 레거시 입력만 출발 leg를
                # 폴백으로 쓴다. 일반 0303은 진입 orient WP가 앞에 있다.
                if leg_start is None:
                    for next_index in range(wp_index + 1, len(rows)):
                        next_coord = rows[next_index].get("coordinate")
                        if isinstance(next_coord, dict):
                            leg_start = coord
                            leg_end = next_coord
                            break
            # 임무 FOV가 없으면 스윕 waypoint 중 가장 큰 값을 기준으로 본다
            # (가장 큰 FOV가 가장 큰 프레임을 만들므로 최악을 대표한다).
            try:
                wp_fov = float(filming.get("fieldOfView", 0.0) or 0.0)
            except Exception:
                wp_fov = 0.0
            if wp_fov > current_fov:
                current_fov = wp_fov
            cos_lat = math.cos(math.radians(wp_lat))
            for target in targets:
                if not isinstance(target, dict):
                    continue
                try:
                    t_lat = float(target.get("latitude"))
                    t_lon = float(target.get("longitude"))
                    t_alt = float(target.get("altitude", 0.0) or 0.0)
                except Exception:
                    continue
                moving_geometry = None
                if isinstance(leg_start, dict) and isinstance(leg_end, dict):
                    moving_geometry = _point_to_flight_leg_geometry_m(
                        target, leg_start, leg_end
                    )
                if moving_geometry is not None:
                    ground_m, aircraft_alt_m, _progress = moving_geometry
                    agl = max(aircraft_alt_m - t_alt, 1.0)
                else:
                    ground_m = math.hypot(
                        (t_lat - wp_lat) * 111_132.9,
                        (t_lon - wp_lon) * 111_320.0 * cos_lat,
                    )
                    agl = max(wp_alt - t_alt, 1.0)
                slant = math.hypot(ground_m, agl)
                summary["checked"] += 1
                score = slant**3 / agl
                if score > worst_score:
                    worst_score = score
                    worst_slant = slant
                    worst_ground = ground_m
                    worst_agl = agl

        if summary["checked"] == 0 or worst_slant <= 0.0:
            return summary
        summary["worstSlantM"] = float(worst_slant)
        summary["worstGroundM"] = float(worst_ground)
        summary["worstAglM"] = float(worst_agl)
        summary["fovBeforeDeg"] = float(current_fov)
        if current_fov <= 0.0:
            return summary

        worst_area = frame_area_m2(
            current_fov, worst_slant, runtime_cfg, altitude_m=worst_agl
        )
        summary["worstAreaM2"] = float(worst_area or 0.0)
        summary["fovAfterDeg"] = float(current_fov)
        allowed: float | None = None
        if worst_area is not None and worst_area > s_req:
            gsd_max = max_fov_for_slant(
                worst_slant, runtime_cfg, altitude_m=worst_agl
            )
            if gsd_max is not None and gsd_max > 0.0:
                allowed = (
                    min(float(gsd_max), params["fov_max_deg"])
                    * params["fov_margin"]
                )
                if allowed < params["fov_min_deg"]:
                    # 카메라 최소 FOV로도 이 기하에서는 요구해상도를 못 맞춘다.
                    allowed = params["fov_min_deg"]
                    summary["unreachable"] = True
                if allowed < current_fov:
                    summary["clamped"] = True

        # 경로 공통 FOV는 검증 상한과 이미 방출된 값 중 가장 좁은 값이다.
        # 이렇게 해야 같은 pathID 안에서 줌이 바뀌지 않고, 기존의
        # 더 보수적인 FOV를 올리지도 않는다.
        uniform_candidates = [value for value in path_fovs if value > 0.0]
        if isinstance(mission_info, dict):
            try:
                mission_fov = float(mission_info.get("FOV", 0.0) or 0.0)
            except Exception:
                mission_fov = 0.0
            if mission_fov > 0.0:
                uniform_candidates.append(mission_fov)
        if allowed is not None and allowed > 0.0:
            uniform_candidates.append(float(allowed))
        if not uniform_candidates:
            return summary
        uniform_fov = float(min(uniform_candidates))

        normalized_count = 0
        for wp in filming_waypoints:
            filming = wp.get("filmingProperty")
            if not isinstance(filming, dict):
                continue
            try:
                existing = float(filming.get("fieldOfView", 0.0) or 0.0)
            except Exception:
                existing = 0.0
            if existing <= 0.0:
                continue
            if not math.isclose(existing, uniform_fov, rel_tol=0.0, abs_tol=1e-12):
                filming["fieldOfView"] = uniform_fov
                normalized_count += 1
        if isinstance(mission_info, dict):
            try:
                existing_info_fov = float(mission_info.get("FOV", 0.0) or 0.0)
            except Exception:
                existing_info_fov = 0.0
            normalized_info_fov = round(uniform_fov, 3)
            if not math.isclose(
                existing_info_fov,
                normalized_info_fov,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                mission_info["FOV"] = normalized_info_fov
                summary["normalized"] = True
        summary["fovAfterDeg"] = uniform_fov
        summary["normalizedWaypointCount"] = normalized_count
        summary["normalized"] = bool(summary["normalized"] or normalized_count)
        return summary
    except Exception as exc:  # pragma: no cover - 검증 실패가 계획을 막으면 안 된다
        summary["error"] = str(exc)
        return summary


def sync_mission_fov_from_flight_plans(
    missions: Any,
    flight_plans: Any,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    """0303에서 확정한 path 공통 FOV를 부모 임무(0302 원본)에 동기화.

    초기계획 0303은 기체별 워커 프로세스에서 만들어지므로, 경로 후처리가
    ``individualMissionInfo.FOV``를 바꿔도 부모 프로세스의 0302 입력에는
    반영되지 않는다. pathID(+aircraftID)로 다시 매칭해 디스크에 쓰기 전
    0302 FOV와 0303 waypoint FOV가 같은 임무 계약을 가리키게 한다.
    """

    result: Dict[str, Any] = {
        "matchedPaths": 0,
        "updatedMissions": 0,
        "normalizedPaths": 0,
        "normalizedWaypoints": 0,
    }
    try:
        if not physics_selection_enabled(runtime_cfg):
            result["skipped"] = "physics_disabled"
            return result

        mission_by_path: Dict[int, Dict[str, Any]] = {}
        mission_by_aircraft_path: Dict[tuple[int, int], Dict[str, Any]] = {}
        for mission in missions or []:
            if not isinstance(mission, dict):
                continue
            try:
                path_id = int(mission.get("pathID") or 0)
                aircraft_id = int(mission.get("aircraftID") or 0)
            except Exception:
                continue
            if path_id <= 0:
                continue
            mission_by_path.setdefault(path_id, mission)
            if aircraft_id > 0:
                mission_by_aircraft_path.setdefault((aircraft_id, path_id), mission)

        for flight_plan in flight_plans or []:
            if not isinstance(flight_plan, dict):
                continue
            try:
                path_id = int(flight_plan.get("pathID") or 0)
                aircraft_id = int(flight_plan.get("aircraftID") or 0)
            except Exception:
                continue
            if path_id <= 0:
                continue
            waypoints = flight_plan.get("waypointList")
            if not isinstance(waypoints, list):
                continue

            filming_rows: list[Dict[str, Any]] = []
            fovs: list[float] = []
            has_sweep = False
            for waypoint in waypoints:
                if not isinstance(waypoint, dict):
                    continue
                filming = waypoint.get("filmingProperty")
                if not isinstance(filming, dict):
                    continue
                filming_rows.append(filming)
                line_search = filming.get("lineSearch")
                if isinstance(line_search, dict) and line_search.get("coordinateList"):
                    has_sweep = True
                try:
                    fov = float(filming.get("fieldOfView", 0.0) or 0.0)
                except Exception:
                    fov = 0.0
                if fov > 0.0:
                    fovs.append(fov)
            # 점찰·인수인계 등 스윗이 없는 특수 경로의 FOV 계약은 건드리지 않는다.
            if not has_sweep or not fovs:
                continue

            uniform_fov = float(min(fovs))
            normalized_this_path = 0
            for filming in filming_rows:
                try:
                    existing = float(filming.get("fieldOfView", 0.0) or 0.0)
                except Exception:
                    existing = 0.0
                if existing <= 0.0 or math.isclose(
                    existing, uniform_fov, rel_tol=0.0, abs_tol=1e-12
                ):
                    continue
                filming["fieldOfView"] = uniform_fov
                normalized_this_path += 1
            if normalized_this_path:
                result["normalizedPaths"] += 1
                result["normalizedWaypoints"] += normalized_this_path

            mission = mission_by_aircraft_path.get((aircraft_id, path_id))
            if mission is None:
                mission = mission_by_path.get(path_id)
            if not isinstance(mission, dict):
                continue
            result["matchedPaths"] += 1
            mission_info = mission.get("individualMissionInfo")
            if not isinstance(mission_info, dict):
                continue
            mission_fov = round(uniform_fov, 3)
            try:
                existing_mission_fov = float(mission_info.get("FOV", 0.0) or 0.0)
            except Exception:
                existing_mission_fov = 0.0
            if math.isclose(
                existing_mission_fov,
                mission_fov,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                continue
            mission_info["FOV"] = mission_fov
            result["updatedMissions"] += 1
        return result
    except Exception as exc:  # pragma: no cover - 동기화 실패가 계획을 막으면 안 된다
        result["error"] = str(exc)
        return result


def physics_signature(runtime_cfg: Dict[str, Any] | None = None) -> Dict[str, Any]:
    """캐시 키에 넣을 물리 손잡이 서명 — 값이 바뀌면 계획 캐시가 무효화돼야 한다."""

    try:
        signature: Dict[str, Any] = {
            "enabled": physics_selection_enabled(runtime_cfg),
        }
        signature.update(
            {key: round(float(value), 9) for key, value in physics_params(runtime_cfg).items()}
        )
        signature["altitudeM"] = round(
            float(capture_altitude_m(_runtime_payload(runtime_cfg))), 6
        )
        return signature
    except Exception:
        return {"enabled": False, "error": True}


__all__ = [
    "certify_waypoint_gsd_inplace",
    "PHYSICS_AREA_FOV_MARGIN",
    "PHYSICS_DEFAULT_TILT_DEG",
    "PHYSICS_FOV_MARGIN",
    "PHYSICS_FPS",
    "PHYSICS_LATERAL_OVERLAP",
    "frame_area_m2",
    "max_fov_for_slant",
    "max_slant_for_fov",
    "area_sequential_split_width_m",
    "area_sequential_split_max_width_m",
    "physics_area_fov_deg",
    "physics_line_row",
    "physics_params",
    "physics_route_offset_cap_m",
    "physics_selection_enabled",
    "physics_sweep_row_fov_deg",
    "physics_signature",
    "max_sweep_row_chord_m_llh",
    "max_sweep_row_chord_m_xy",
    "physics_standoff_for_fov",
    "projected_span_m_llh",
    "projected_span_m_xy",
    "required_footprint_area_m2",
    "route_offset_scale",
    "select_fov_deg",
    "sync_mission_fov_from_flight_plans",
]
