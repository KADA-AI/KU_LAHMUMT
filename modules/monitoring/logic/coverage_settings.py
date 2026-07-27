# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import math
import threading
from pathlib import Path
from typing import Any

from modules.common import db_paths


DEFAULT_COVERAGE_SETTINGS: dict[str, float | int | bool] = {
    "footprint_interpolation_hz": 30.0,
    "max_interpolation_gap_ms": 1000,
    "max_interpolation_steps": 120,
    "include_sweep_endpoint_coverage": True,
    "sweep_turn_spacing_fraction": 0.5,
    "sweep_turn_min_spacing_m": 5.0,
    "max_sweep_turn_fill_samples": 32,
}

COVERAGE_SETTINGS_PATH = (
    db_paths.PROJECT_ROOT / "modules" / "monitoring" / "coverage_monitor_settings.json"
)

_LOCK = threading.RLock()
_CACHE: dict[str, float | int | bool] | None = None
_CACHE_MTIME_NS: int | None = None


def _as_float(value: object, fallback: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(fallback)


def _as_int(value: object, fallback: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(fallback)


def _as_bool(value: object, fallback: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value or "").strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    return bool(fallback)


def normalize_coverage_settings(
    value: dict[str, Any] | None,
) -> dict[str, float | int | bool]:
    raw = value if isinstance(value, dict) else {}
    return {
        "footprint_interpolation_hz": max(
            1.0,
            min(
                120.0,
                _as_float(
                    raw.get("footprint_interpolation_hz"),
                    float(DEFAULT_COVERAGE_SETTINGS["footprint_interpolation_hz"]),
                ),
            ),
        ),
        "max_interpolation_gap_ms": max(
            100,
            min(
                5000,
                _as_int(
                    raw.get("max_interpolation_gap_ms"),
                    int(DEFAULT_COVERAGE_SETTINGS["max_interpolation_gap_ms"]),
                ),
            ),
        ),
        "max_interpolation_steps": max(
            1,
            min(
                300,
                _as_int(
                    raw.get("max_interpolation_steps"),
                    int(DEFAULT_COVERAGE_SETTINGS["max_interpolation_steps"]),
                ),
            ),
        ),
        "include_sweep_endpoint_coverage": _as_bool(
            raw.get("include_sweep_endpoint_coverage"),
            bool(DEFAULT_COVERAGE_SETTINGS["include_sweep_endpoint_coverage"]),
        ),
        "sweep_turn_spacing_fraction": max(
            0.05,
            min(
                1.0,
                _as_float(
                    raw.get("sweep_turn_spacing_fraction"),
                    float(DEFAULT_COVERAGE_SETTINGS["sweep_turn_spacing_fraction"]),
                ),
            ),
        ),
        "sweep_turn_min_spacing_m": max(
            1.0,
            min(
                1000.0,
                _as_float(
                    raw.get("sweep_turn_min_spacing_m"),
                    float(DEFAULT_COVERAGE_SETTINGS["sweep_turn_min_spacing_m"]),
                ),
            ),
        ),
        "max_sweep_turn_fill_samples": max(
            1,
            min(
                1024,
                _as_int(
                    raw.get("max_sweep_turn_fill_samples"),
                    int(DEFAULT_COVERAGE_SETTINGS["max_sweep_turn_fill_samples"]),
                ),
            ),
        ),
    }


def load_coverage_settings(*, force: bool = False) -> dict[str, float | int | bool]:
    global _CACHE, _CACHE_MTIME_NS
    with _LOCK:
        try:
            mtime_ns = COVERAGE_SETTINGS_PATH.stat().st_mtime_ns
        except OSError:
            mtime_ns = None
        if not force and _CACHE is not None and mtime_ns == _CACHE_MTIME_NS:
            return dict(_CACHE)
        raw: dict[str, Any] = {}
        try:
            parsed = json.loads(COVERAGE_SETTINGS_PATH.read_text(encoding="utf-8"))
            if isinstance(parsed, dict):
                raw = parsed
        except (OSError, json.JSONDecodeError):
            pass
        _CACHE = normalize_coverage_settings(raw)
        _CACHE_MTIME_NS = mtime_ns
        return dict(_CACHE)


def save_coverage_settings(
    value: dict[str, Any],
) -> dict[str, float | int | bool]:
    global _CACHE, _CACHE_MTIME_NS
    with _LOCK:
        current = load_coverage_settings(force=True)
        current.update(value if isinstance(value, dict) else {})
        normalized = normalize_coverage_settings(current)
        COVERAGE_SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(f"{COVERAGE_SETTINGS_PATH}.tmp")
        temporary.write_text(
            json.dumps(normalized, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(COVERAGE_SETTINGS_PATH)
        _CACHE = normalized
        _CACHE_MTIME_NS = COVERAGE_SETTINGS_PATH.stat().st_mtime_ns
        return dict(normalized)


def resolve_footprint_interpolation_steps(
    previous_timestamp_ms: int,
    current_timestamp_ms: int,
    settings: dict[str, float | int | bool] | None = None,
) -> int:
    """Return interpolation intervals, or zero when continuity must be broken."""
    resolved = normalize_coverage_settings(settings) if settings is not None else load_coverage_settings()
    delta_ms = int(current_timestamp_ms) - int(previous_timestamp_ms)
    if delta_ms <= 0 or delta_ms > int(resolved["max_interpolation_gap_ms"]):
        return 0
    return max(
        1,
        min(
            int(resolved["max_interpolation_steps"]),
            int(math.ceil((delta_ms / 1000.0) * float(resolved["footprint_interpolation_hz"]))),
        ),
    )
