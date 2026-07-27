from __future__ import annotations

import json
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Mapping

from ..uav import UAVParams


DEFAULT_PROFILE_PATH = Path(__file__).with_name("shinil_mission_profile.json")


@dataclass(frozen=True)
class ShinilMotionOverlay:
    """SIM-only UAV parameter overlay for one observed mission phase."""

    reference_turn_radius_scale: float = 1.0
    accel: float = 1.2
    max_roll_rate_dps: float = 10.0
    turn_roll_gain: float = 0.9


@dataclass(frozen=True)
class ShinilCameraOverlay:
    """Observed payload-command and line-of-sight response for one phase."""

    command_delay_s: float = 0.2
    line_search_delay_s: float = 1.4
    fixed_to_line_search_delay_s: float = 1.8
    coordinate_slew_rate_dps: float = 30.0
    line_search_slew_rate_dps: float = 150.0
    settle_tolerance_deg: float = 0.5


@dataclass(frozen=True)
class ShinilFootprintModel:
    """Legacy footprint calibration metadata retained for profile compatibility.

    Runtime footprint geometry no longer branches on these values: detection,
    0401, and display all use the regional DEM ray-intersection truth model.
    """

    flat_center_altitude_plane: bool = False
    center_altitude_quantum_m: float = 1.0
    couple_image_axis_to_body_attitude: bool = True
    effective_fov_scale: float = 1.0
    sample_short_period_s: float = 0.2
    sample_long_period_s: float = 0.4


@dataclass(frozen=True)
class ShinilMissionProfile:
    source_log: str
    phases: Mapping[str, ShinilMotionOverlay]
    camera_phases: Mapping[str, ShinilCameraOverlay]
    footprint_model: ShinilFootprintModel = ShinilFootprintModel()
    notes: tuple[str, ...] = ()
    analysis: Mapping[str, Any] | None = None

    def overlay_for(self, phase: str) -> ShinilMotionOverlay:
        return self.phases.get(str(phase), self.phases["default"])

    def apply(self, baseline: UAVParams, phase: str) -> UAVParams:
        overlay = self.overlay_for(phase)
        return replace(
            baseline,
            reference_turn_radius_scale=overlay.reference_turn_radius_scale,
            accel=overlay.accel,
            max_roll_rate_dps=overlay.max_roll_rate_dps,
            turn_roll_gain=overlay.turn_roll_gain,
        )

    def camera_overlay_for(self, phase: str) -> ShinilCameraOverlay:
        return self.camera_phases.get(str(phase), self.camera_phases["default"])


_FALLBACK_PHASES: dict[str, ShinilMotionOverlay] = {
    "default": ShinilMotionOverlay(),
    "type1_region3": ShinilMotionOverlay(max_roll_rate_dps=6.0, turn_roll_gain=0.6),
    "type1_region4": ShinilMotionOverlay(max_roll_rate_dps=12.0, turn_roll_gain=1.0),
    "area_lead": ShinilMotionOverlay(max_roll_rate_dps=12.0, turn_roll_gain=1.1),
    "area_scan": ShinilMotionOverlay(max_roll_rate_dps=6.0, turn_roll_gain=0.7),
    "type1_region5": ShinilMotionOverlay(max_roll_rate_dps=12.0, turn_roll_gain=1.0),
    "individual_search": ShinilMotionOverlay(max_roll_rate_dps=8.0, turn_roll_gain=0.8),
    "individual_line": ShinilMotionOverlay(max_roll_rate_dps=10.0, turn_roll_gain=0.9),
}


_FALLBACK_CAMERA_PHASES: dict[str, ShinilCameraOverlay] = {
    "default": ShinilCameraOverlay(),
    "type1_region3": ShinilCameraOverlay(
        coordinate_slew_rate_dps=30.0,
        line_search_slew_rate_dps=70.0,
    ),
    "type1_region4": ShinilCameraOverlay(
        coordinate_slew_rate_dps=30.0,
        line_search_slew_rate_dps=55.0,
    ),
    "area_lead": ShinilCameraOverlay(coordinate_slew_rate_dps=30.0),
    "area_scan": ShinilCameraOverlay(
        coordinate_slew_rate_dps=30.0,
        line_search_slew_rate_dps=150.0,
    ),
    "type1_region5": ShinilCameraOverlay(
        coordinate_slew_rate_dps=30.0,
        line_search_slew_rate_dps=70.0,
    ),
    "individual_search": ShinilCameraOverlay(
        coordinate_slew_rate_dps=30.0,
        line_search_slew_rate_dps=100.0,
    ),
    "individual_line": ShinilCameraOverlay(
        coordinate_slew_rate_dps=30.0,
        line_search_slew_rate_dps=100.0,
    ),
}


def _finite_clamped(value: object, fallback: float, lo: float, hi: float) -> float:
    try:
        parsed = float(value)
    except Exception:
        return float(fallback)
    if not math.isfinite(parsed):
        return float(fallback)
    return max(float(lo), min(float(hi), parsed))


def _parse_bool(value: object, fallback: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
    return bool(fallback)


def _parse_overlay(raw: object, fallback: ShinilMotionOverlay) -> ShinilMotionOverlay:
    data = raw if isinstance(raw, dict) else {}
    return ShinilMotionOverlay(
        reference_turn_radius_scale=_finite_clamped(
            data.get("reference_turn_radius_scale"),
            fallback.reference_turn_radius_scale,
            0.65,
            1.75,
        ),
        accel=_finite_clamped(data.get("accel"), fallback.accel, 0.5, 15.0),
        max_roll_rate_dps=_finite_clamped(
            data.get("max_roll_rate_dps"),
            fallback.max_roll_rate_dps,
            4.0,
            40.0,
        ),
        turn_roll_gain=_finite_clamped(
            data.get("turn_roll_gain"),
            fallback.turn_roll_gain,
            0.3,
            4.0,
        ),
    )


def _parse_camera_overlay(raw: object, fallback: ShinilCameraOverlay) -> ShinilCameraOverlay:
    data = raw if isinstance(raw, dict) else {}
    return ShinilCameraOverlay(
        command_delay_s=_finite_clamped(
            data.get("command_delay_s"), fallback.command_delay_s, 0.0, 5.0
        ),
        line_search_delay_s=_finite_clamped(
            data.get("line_search_delay_s"), fallback.line_search_delay_s, 0.0, 10.0
        ),
        fixed_to_line_search_delay_s=_finite_clamped(
            data.get("fixed_to_line_search_delay_s"),
            fallback.fixed_to_line_search_delay_s,
            0.0,
            10.0,
        ),
        coordinate_slew_rate_dps=_finite_clamped(
            data.get("coordinate_slew_rate_dps"),
            fallback.coordinate_slew_rate_dps,
            1.0,
            360.0,
        ),
        line_search_slew_rate_dps=_finite_clamped(
            data.get("line_search_slew_rate_dps"),
            fallback.line_search_slew_rate_dps,
            1.0,
            720.0,
        ),
        settle_tolerance_deg=_finite_clamped(
            data.get("settle_tolerance_deg"),
            fallback.settle_tolerance_deg,
            0.05,
            10.0,
        ),
    )


def _parse_footprint_model(raw: object) -> ShinilFootprintModel:
    fallback = ShinilFootprintModel()
    data = raw if isinstance(raw, dict) else {}
    return ShinilFootprintModel(
        flat_center_altitude_plane=_parse_bool(
            data.get("flat_center_altitude_plane"),
            fallback.flat_center_altitude_plane,
        ),
        center_altitude_quantum_m=_finite_clamped(
            data.get("center_altitude_quantum_m"),
            fallback.center_altitude_quantum_m,
            0.0,
            100.0,
        ),
        couple_image_axis_to_body_attitude=_parse_bool(
            data.get("couple_image_axis_to_body_attitude"),
            fallback.couple_image_axis_to_body_attitude,
        ),
        effective_fov_scale=_finite_clamped(
            data.get("effective_fov_scale"),
            fallback.effective_fov_scale,
            0.5,
            1.5,
        ),
        sample_short_period_s=_finite_clamped(
            data.get("sample_short_period_s"),
            fallback.sample_short_period_s,
            0.02,
            2.0,
        ),
        sample_long_period_s=_finite_clamped(
            data.get("sample_long_period_s"),
            fallback.sample_long_period_s,
            0.02,
            2.0,
        ),
    )


def load_shinil_mission_profile(path: str | Path | None = None) -> ShinilMissionProfile:
    """Load the isolated 0401-derived profile without touching planner turn settings."""

    profile_path = Path(path) if path is not None else DEFAULT_PROFILE_PATH
    raw: dict[str, Any] = {}
    try:
        loaded = json.loads(profile_path.read_text(encoding="utf-8"))
        if isinstance(loaded, dict):
            raw = loaded
    except Exception:
        raw = {}

    configured = raw.get("phases") if isinstance(raw.get("phases"), dict) else {}
    phases: dict[str, ShinilMotionOverlay] = {}
    for name, fallback in _FALLBACK_PHASES.items():
        phases[name] = _parse_overlay(configured.get(name), fallback)

    configured_camera = (
        raw.get("camera_phases") if isinstance(raw.get("camera_phases"), dict) else {}
    )
    camera_phases: dict[str, ShinilCameraOverlay] = {}
    for name, fallback in _FALLBACK_CAMERA_PHASES.items():
        camera_phases[name] = _parse_camera_overlay(configured_camera.get(name), fallback)

    notes_raw = raw.get("notes")
    notes = tuple(str(item) for item in notes_raw if item is not None) if isinstance(notes_raw, list) else ()
    analysis = raw.get("analysis") if isinstance(raw.get("analysis"), dict) else None
    return ShinilMissionProfile(
        source_log=str(raw.get("source_log") or "Logs/Scenario_2026-07-15T112330/SBC3/0401"),
        phases=phases,
        camera_phases=camera_phases,
        footprint_model=_parse_footprint_model(raw.get("footprint_model")),
        notes=notes,
        analysis=analysis,
    )


def select_shinil_phase(
    *,
    input_mission_type: int | None,
    region_type: int | None,
    individual_mission_type: int | None,
    pattern_type: int | None,
    filming_operation_mode: int | None,
) -> str:
    """Resolve a stable mission phase from metadata retained on the target WP."""

    if input_mission_type == 2 and region_type == 4:
        return "area_scan" if filming_operation_mode == 2 else "area_lead"
    if input_mission_type == 1 and region_type == 3:
        return "type1_region3"
    if input_mission_type == 1 and region_type == 4:
        return "type1_region4"
    if input_mission_type == 1 and region_type == 5:
        return "type1_region5"
    if individual_mission_type == 3 or pattern_type == 6:
        return "individual_search"
    if individual_mission_type == 6 or pattern_type == 8:
        return "individual_line"
    return "default"
