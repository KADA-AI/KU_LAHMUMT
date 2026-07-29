"""Shared capture-geometry law for line/area mission planning.

The FOV DB is used only to SELECT a camera FOV mode. Footprint, sweep
spacing, scan interval, camera sweep speed and aircraft speed are derived
geometrically from the selected FOV, following the algorithm prototypes in
test/line_mission_algorithm and test/area_mission_algorithm:

- nadir footprint: diagonal FOV -> horizontal/vertical FOV split by the
  image aspect ratio; 가로 = 2*alt*tan(hfov/2), 세로 = 2*alt*tan(vfov/2)
- sweep spacing (세로 간격) = 세로 footprint * (1 - vertical overlap),
  an UPPER bound — tighter spacing is always allowed
- scan interval / camera sweep speed / aircraft speed from the same
  overlap-driven timing plan (port of _scan_interval_speed_plan).

Production deviation from the prototype: when a sweep line is too long for
the camera to finish within the vertical-overlap window (camera-limited
branch), the prototype lets line spacing grow beyond the overlap bound.
Here the spacing bound is hard — spacing stays at the law, the aircraft
speed pins to the scan minimum, and the plan is flagged cameraLimited.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

from .runtime_settings import get_runtime_float, load_runtime_settings_readonly


def _resolve_runtime_payload(runtime_cfg: Dict[str, Any] | None) -> Dict[str, Any]:
    """Resolve the settings payload once per public call.

    Read-only: nothing here mutates the payload, so the cached document is used
    directly rather than a per-call deep copy of the whole settings file.
    """
    if isinstance(runtime_cfg, dict):
        return runtime_cfg
    try:
        payload = load_runtime_settings_readonly()
    except Exception:
        payload = {}
    return payload if isinstance(payload, dict) else {}

CAPTURE_HORIZONTAL_OVERLAP_RATIO = 0.20
CAPTURE_VERTICAL_OVERLAP_RATIO = 0.20
CAPTURE_FPS = 30.0
CAPTURE_SCAN_MIN_AIRCRAFT_SPEED_MPS = 40.0
CAPTURE_SCAN_MAX_AIRCRAFT_SPEED_MPS = 55.0
CAPTURE_SCAN_PREFERRED_INTERVAL_S = 1.0
CAPTURE_SPACING_SCALE = 1.0
# Base overlap leaves 0.8 × footprint.  Multiplying by 5/6 makes the AREA
# strip spacing exactly 2/3 × footprint, i.e. 1.5 × search-line density.
CAPTURE_AREA_SPACING_SCALE = 5.0 / 6.0
CAPTURE_IMAGE_WIDTH_PX = 1920.0
CAPTURE_IMAGE_HEIGHT_PX = 1080.0
CAPTURE_MIN_SWEEP_SPACING_M = 1.0


def capture_params(runtime_cfg: Dict[str, Any] | None = None) -> Dict[str, float]:
    """Resolve the capture-law runtime knobs with prototype defaults."""
    runtime_cfg = _resolve_runtime_payload(runtime_cfg)
    horizontal_overlap = get_runtime_float(
        "capture_horizontal_overlap_ratio", CAPTURE_HORIZONTAL_OVERLAP_RATIO, runtime_cfg
    )
    vertical_overlap = get_runtime_float(
        "capture_vertical_overlap_ratio", CAPTURE_VERTICAL_OVERLAP_RATIO, runtime_cfg
    )
    fps = get_runtime_float("capture_fps", CAPTURE_FPS, runtime_cfg)
    min_speed = get_runtime_float(
        "capture_scan_min_aircraft_speed_mps", CAPTURE_SCAN_MIN_AIRCRAFT_SPEED_MPS, runtime_cfg
    )
    max_speed = get_runtime_float(
        "capture_scan_max_aircraft_speed_mps", CAPTURE_SCAN_MAX_AIRCRAFT_SPEED_MPS, runtime_cfg
    )
    preferred_interval = get_runtime_float(
        "capture_scan_preferred_interval_s", CAPTURE_SCAN_PREFERRED_INTERVAL_S, runtime_cfg
    )
    spacing_scale = get_runtime_float("capture_spacing_scale", CAPTURE_SPACING_SCALE, runtime_cfg)
    image_width_px = get_runtime_float("capture_image_width_px", CAPTURE_IMAGE_WIDTH_PX, runtime_cfg)
    image_height_px = get_runtime_float("capture_image_height_px", CAPTURE_IMAGE_HEIGHT_PX, runtime_cfg)
    return {
        "horizontal_overlap_ratio": min(max(float(horizontal_overlap), 0.0), 0.95),
        "vertical_overlap_ratio": min(max(float(vertical_overlap), 0.0), 0.95),
        "fps": max(float(fps), 1.0),
        "min_aircraft_speed_mps": max(float(min_speed), 0.1),
        "max_aircraft_speed_mps": max(float(max_speed), max(float(min_speed), 0.1)),
        "preferred_interval_s": max(float(preferred_interval), 0.1),
        "spacing_scale": float(spacing_scale) if spacing_scale > 0.0 else 1.0,
        "aspect_ratio": max(float(image_width_px), 1.0) / max(float(image_height_px), 1.0),
    }


def capture_altitude_m(
    runtime_cfg: Dict[str, Any] | None = None,
    *,
    altitude_m: Optional[float] = None,
) -> float:
    if altitude_m is not None:
        try:
            value = float(altitude_m)
            if math.isfinite(value) and value > 0.0:
                return value
        except Exception:
            pass
    value = get_runtime_float("altitude_m", 1000.0, _resolve_runtime_payload(runtime_cfg))
    if not math.isfinite(value) or value <= 0.0:
        return 1000.0
    return float(value)


def diagonal_to_horizontal_vertical_fov(
    diagonal_fov_deg: float,
    aspect_ratio: float = 16.0 / 9.0,
) -> tuple[float, float]:
    diag_radian = math.radians(float(diagonal_fov_deg))
    tan_diag_half = math.tan(diag_radian * 0.5)
    tan_vertical_half = tan_diag_half / math.sqrt(1.0 + aspect_ratio**2)
    tan_horizontal_half = aspect_ratio * tan_vertical_half
    return (
        math.degrees(2.0 * math.atan(tan_horizontal_half)),
        math.degrees(2.0 * math.atan(tan_vertical_half)),
    )


def nadir_footprint_m(
    fov_deg: Optional[float],
    *,
    altitude_m: Optional[float] = None,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Optional[Dict[str, float]]:
    """Nadir footprint (가로/세로) for a diagonal FOV at the mission altitude."""
    if fov_deg is None:
        return None
    try:
        fov_value = float(fov_deg)
    except Exception:
        return None
    if not math.isfinite(fov_value) or fov_value <= 0.0:
        return None
    runtime_cfg = _resolve_runtime_payload(runtime_cfg)
    params = capture_params(runtime_cfg)
    horizontal_fov_deg, vertical_fov_deg = diagonal_to_horizontal_vertical_fov(
        fov_value,
        aspect_ratio=params["aspect_ratio"],
    )
    alt = capture_altitude_m(runtime_cfg, altitude_m=altitude_m)
    horizontal_m = 2.0 * alt * math.tan(math.radians(horizontal_fov_deg) * 0.5)
    vertical_m = 2.0 * alt * math.tan(math.radians(vertical_fov_deg) * 0.5)
    if horizontal_m <= 1e-6 or vertical_m <= 1e-6:
        return None
    return {
        "fovDeg": float(fov_value),
        "horizontalFovDeg": float(horizontal_fov_deg),
        "verticalFovDeg": float(vertical_fov_deg),
        "altitudeM": float(alt),
        "horizontalM": float(horizontal_m),
        "verticalM": float(vertical_m),
        "areaM2": float(horizontal_m * vertical_m),
    }


def capture_area_spacing_scale(runtime_cfg: Dict[str, Any] | None = None) -> float:
    """AREA-only spacing multiplier; default gives 1.5× footprint density."""
    scale = get_runtime_float(
        "capture_area_spacing_scale",
        CAPTURE_AREA_SPACING_SCALE,
        _resolve_runtime_payload(runtime_cfg),
    )
    return float(scale) if scale > 0.0 else 1.0


def vertical_sweep_spacing_m(
    fov_deg: Optional[float],
    *,
    altitude_m: Optional[float] = None,
    runtime_cfg: Dict[str, Any] | None = None,
    extra_scale: float = 1.0,
) -> Optional[float]:
    """세로 간격 = 세로 footprint * (1 - vertical overlap) * spacing scale."""
    runtime_cfg = _resolve_runtime_payload(runtime_cfg)
    footprint = nadir_footprint_m(fov_deg, altitude_m=altitude_m, runtime_cfg=runtime_cfg)
    if footprint is None:
        return None
    params = capture_params(runtime_cfg)
    try:
        extra = float(extra_scale)
    except Exception:
        extra = 1.0
    if extra <= 0.0:
        extra = 1.0
    spacing = (
        footprint["verticalM"]
        * (1.0 - params["vertical_overlap_ratio"])
        * params["spacing_scale"]
        * extra
    )
    return max(float(spacing), CAPTURE_MIN_SWEEP_SPACING_M)


def area_vertical_sweep_spacing_m(
    fov_deg: Optional[float],
    *,
    altitude_m: Optional[float] = None,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Optional[float]:
    """AREA 세로 간격 = 통일 법칙 * 전용 배수(기본 footprint / 1.5)."""
    runtime_cfg = _resolve_runtime_payload(runtime_cfg)
    return vertical_sweep_spacing_m(
        fov_deg,
        altitude_m=altitude_m,
        runtime_cfg=runtime_cfg,
        extra_scale=capture_area_spacing_scale(runtime_cfg),
    )


def scan_interval_speed_plan(
    *,
    line_length_m: float,
    horizontal_footprint_m: float,
    vertical_footprint_m: float,
    horizontal_overlap_ratio: float,
    vertical_overlap_ratio: float,
    fps: float,
    min_aircraft_speed_mps: float,
    max_aircraft_speed_mps: float,
    preferred_interval_s: float,
) -> Dict[str, float | bool]:
    """Port of the prototype _scan_interval_speed_plan (test/line_mission_algorithm)."""
    horizontal_overlap_target = min(max(float(horizontal_overlap_ratio), 0.0), 0.95)
    vertical_overlap_target = min(max(float(vertical_overlap_ratio), 0.0), 0.95)
    horizontal_footprint = max(float(horizontal_footprint_m), 1e-6)
    vertical_footprint = max(float(vertical_footprint_m), 1e-6)
    line_length = max(float(line_length_m), 1e-6)
    frames_per_sec = max(float(fps), 1.0)
    vertical_spacing_max = max(vertical_footprint * (1.0 - vertical_overlap_target), 1e-6)
    horizontal_frame_step_max = max(horizontal_footprint * (1.0 - horizontal_overlap_target), 1e-6)
    camera_speed_max = horizontal_frame_step_max * frames_per_sec
    horizontal_interval_min = line_length / max(camera_speed_max, 1e-6)
    min_speed_mps = max(float(min_aircraft_speed_mps), 0.1)
    max_speed_mps = max(float(max_aircraft_speed_mps), min_speed_mps)
    desired_interval = max(float(preferred_interval_s), horizontal_interval_min)
    vertical_interval_min = vertical_spacing_max / max_speed_mps
    vertical_interval_max = vertical_spacing_max / min_speed_mps

    if horizontal_interval_min <= vertical_interval_max + 1e-9:
        scan_interval = min(max(desired_interval, vertical_interval_min), vertical_interval_max)
        aircraft_speed = min(max_speed_mps, max(min_speed_mps, vertical_spacing_max / max(scan_interval, 1e-6)))
        line_spacing = vertical_spacing_max
        camera_limited = False
    else:
        scan_interval = max(horizontal_interval_min, 1e-6)
        aircraft_speed = min_speed_mps
        line_spacing = aircraft_speed * scan_interval
        camera_limited = True

    camera_speed = line_length / max(scan_interval, 1e-6)
    horizontal_frame_step = camera_speed / frames_per_sec
    horizontal_overlap = 1.0 - (horizontal_frame_step / horizontal_footprint)
    vertical_overlap = 1.0 - (line_spacing / vertical_footprint)
    return {
        "aircraftSpeedMps": aircraft_speed,
        "scanIntervalSec": scan_interval,
        "lineSpacingM": line_spacing,
        "cameraSearchSpeedMps": camera_speed,
        "horizontalFrameStepM": horizontal_frame_step,
        "horizontalMaxFrameStepM": horizontal_frame_step_max,
        "verticalMaxLineSpacingM": vertical_spacing_max,
        "horizontalOverlapRatio": max(0.0, min(1.0, horizontal_overlap)),
        "verticalOverlapRatio": max(0.0, min(1.0, vertical_overlap)),
        "horizontalMinOverlapRatio": horizontal_overlap_target,
        "verticalMinOverlapRatio": vertical_overlap_target,
        "horizontalMinOverlapSatisfied": horizontal_overlap + 1e-9 >= horizontal_overlap_target,
        "verticalMinOverlapSatisfied": vertical_overlap + 1e-9 >= vertical_overlap_target,
        "cameraLimited": camera_limited,
    }


def capture_speed_plan(
    fov_deg: Optional[float],
    *,
    line_length_m: Optional[float] = None,
    altitude_m: Optional[float] = None,
    runtime_cfg: Dict[str, Any] | None = None,
    extra_spacing_scale: float = 1.0,
) -> Optional[Dict[str, Any]]:
    """FOV -> footprint -> timing plan; spacing bound enforced as hard.

    line_length_m is the representative sweep-line length. When unknown it
    falls back to the horizontal footprint (short line, always feasible),
    which leaves the aircraft-speed result unaffected in the vertical-limited
    branch. extra_spacing_scale folds a caller-side densifier (e.g. the area
    scale) into the plan so its spacing/interval match the produced sweeps.
    """
    runtime_cfg = _resolve_runtime_payload(runtime_cfg)
    footprint = nadir_footprint_m(fov_deg, altitude_m=altitude_m, runtime_cfg=runtime_cfg)
    if footprint is None:
        return None
    params = capture_params(runtime_cfg)
    try:
        extra = float(extra_spacing_scale)
    except Exception:
        extra = 1.0
    if extra <= 0.0:
        extra = 1.0
    scale = params["spacing_scale"] * extra
    length = 0.0
    if line_length_m is not None:
        try:
            length = float(line_length_m)
        except Exception:
            length = 0.0
    if not math.isfinite(length) or length <= 0.0:
        length = footprint["horizontalM"]
    plan = scan_interval_speed_plan(
        line_length_m=length,
        horizontal_footprint_m=footprint["horizontalM"] * scale,
        vertical_footprint_m=footprint["verticalM"] * scale,
        horizontal_overlap_ratio=params["horizontal_overlap_ratio"],
        vertical_overlap_ratio=params["vertical_overlap_ratio"],
        fps=params["fps"],
        min_aircraft_speed_mps=params["min_aircraft_speed_mps"],
        max_aircraft_speed_mps=params["max_aircraft_speed_mps"],
        preferred_interval_s=params["preferred_interval_s"],
    )
    spacing_bound = max(float(plan["verticalMaxLineSpacingM"]), CAPTURE_MIN_SWEEP_SPACING_M)
    if float(plan["lineSpacingM"]) > spacing_bound:
        # Hard spacing bound: keep the overlap law, report the effective
        # interval the production spacing/speed pair implies.
        plan["lineSpacingM"] = spacing_bound
        aircraft_speed = max(float(plan["aircraftSpeedMps"]), 0.1)
        plan["scanIntervalSec"] = spacing_bound / aircraft_speed
        plan["cameraSearchSpeedMps"] = length / max(float(plan["scanIntervalSec"]), 1e-6)
        plan["verticalOverlapRatio"] = max(
            0.0, min(1.0, 1.0 - (spacing_bound / (footprint["verticalM"] * scale)))
        )
        plan["verticalMinOverlapSatisfied"] = True
        plan["horizontalFrameStepM"] = float(plan["cameraSearchSpeedMps"]) / params["fps"]
        horizontal_overlap = 1.0 - (
            float(plan["horizontalFrameStepM"]) / (footprint["horizontalM"] * scale)
        )
        plan["horizontalOverlapRatio"] = max(0.0, min(1.0, horizontal_overlap))
        plan["horizontalMinOverlapSatisfied"] = (
            horizontal_overlap + 1e-9 >= params["horizontal_overlap_ratio"]
        )
    plan["sweepSpacingM"] = max(float(plan["lineSpacingM"]), CAPTURE_MIN_SWEEP_SPACING_M)
    plan["aircraftSpeedKmh"] = float(plan["aircraftSpeedMps"]) * 3.6
    plan["footprint"] = footprint
    plan["lineLengthM"] = float(length)
    return plan


def capture_aircraft_speed_kmh(
    fov_deg: Optional[float],
    *,
    line_length_m: Optional[float] = None,
    altitude_m: Optional[float] = None,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Optional[float]:
    plan = capture_speed_plan(
        fov_deg,
        line_length_m=line_length_m,
        altitude_m=altitude_m,
        runtime_cfg=runtime_cfg,
    )
    if plan is None:
        return None
    return float(plan["aircraftSpeedKmh"])


def capture_cruise_speed_mps(
    fov_deg: Optional[float],
    *,
    line_length_m: Optional[float] = None,
    altitude_m: Optional[float] = None,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Optional[float]:
    plan = capture_speed_plan(
        fov_deg,
        line_length_m=line_length_m,
        altitude_m=altitude_m,
        runtime_cfg=runtime_cfg,
    )
    if plan is None:
        return None
    return float(plan["aircraftSpeedMps"])
