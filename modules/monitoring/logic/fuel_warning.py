# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import time
from typing import Any, Callable


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _coerce_float(value: object) -> float | None:
    try:
        return float(value)
    except Exception:
        return None


def resolve_fuel_capacity_liters() -> float:
    """Resolve the fuel capacity from environment with a safe default."""
    raw = os.getenv("KU_MON_FUEL_CAPACITY_L", "15")
    try:
        value = float(raw) if raw is not None else 15.0
        if value > 0:
            return float(value)
    except (TypeError, ValueError):
        pass
    return 15.0


_EPOCH2000_MS = 946684800000


def _default_now_ms_since_2000() -> int:
    return int(time.time() * 1000) - _EPOCH2000_MS


def fuel_state_from_liters(
    fuel_liters: float | None,
    *,
    capacity_liters: float,
) -> tuple[str, int]:
    """Return (state_text, fuelLevel) using the legacy ver2 thresholds."""
    if fuel_liters is None:
        return "unknown", 0
    liters = max(0.0, float(fuel_liters))
    capacity = float(capacity_liters) if capacity_liters > 0 else 15.0
    red_threshold = capacity * 0.1
    yellow_threshold = capacity * 0.2
    if liters <= red_threshold:
        return "red", 2
    if liters <= yellow_threshold:
        return "yellow", 1
    return "green", 0


def fuel_state_from_warning_code(fuel_warning: int | None) -> tuple[str, int]:
    """Map 0401 fuelWarning code to (state_text, 0504 fuelLevel)."""
    code = _coerce_int(fuel_warning)
    if code == 3:
        return "red", 2
    if code == 2:
        return "yellow", 1
    if code == 1:
        return "green", 0
    if code == 0:
        return "unknown", 0
    return "unknown", 0


def resolve_fuel_state(
    *,
    fuel_liters: float | None,
    fuel_warning: int | None,
    capacity_liters: float,
    use_threshold_logic: bool = False,
) -> tuple[str, int]:
    """Resolve fuel state from 0401 or optional liters-threshold logic."""
    if not use_threshold_logic:
        _ = fuel_liters
        _ = capacity_liters
        return fuel_state_from_warning_code(fuel_warning)
    liters_state, liters_level = fuel_state_from_liters(
        fuel_liters,
        capacity_liters=capacity_liters,
    )
    warning_state, warning_level = fuel_state_from_warning_code(fuel_warning)
    level = max(int(liters_level), int(warning_level))
    if level >= 2:
        return "red", 2
    if level == 1:
        return "yellow", 1
    if liters_state == "green" or warning_state == "green":
        return "green", 0
    return "unknown", 0


class FuelWarningCoordinator:
    """Deduping coordinator for 0504 fuel warnings and UI status."""

    def __init__(
        self,
        *,
        capacity_liters: float | None = None,
        now_fn: Callable[[], int] | None = None,
        use_threshold_logic: bool = False,
    ) -> None:
        self.capacity_liters = (
            float(capacity_liters) if capacity_liters is not None else resolve_fuel_capacity_liters()
        )
        self._now_fn = now_fn or _default_now_ms_since_2000
        self._use_threshold_logic = bool(use_threshold_logic)
        self._prev_state: dict[int, str] = {}

    def set_threshold_logic_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled == self._use_threshold_logic:
            return
        self._use_threshold_logic = enabled
        self._prev_state.clear()

    def update(
        self,
        *,
        agent_states: list[dict[str, Any]],
        timestamp_ms: int | None,
        source: str = "MSM",
    ) -> tuple[list[dict[str, int | str]], dict[int, str]]:
        """Compute 0504 payloads to send and the current fuel state map."""
        warnings: list[dict[str, int | str]] = []
        state_map: dict[int, str] = {}
        ts_value = int(timestamp_ms) if timestamp_ms is not None else int(self._now_fn())

        for state in agent_states:
            if not isinstance(state, dict):
                continue
            is_unmanned_val = state.get("is_unmanned")
            try:
                is_unmanned = int(is_unmanned_val) if is_unmanned_val is not None else 0
            except Exception:
                is_unmanned = 1 if bool(is_unmanned_val) else 0
            if is_unmanned != 1:
                continue
            aircraft_id = _coerce_int(state.get("aircraft_id"))
            if aircraft_id is None:
                continue
            fuel_raw = state.get("fuel_liters")
            if fuel_raw is None:
                fuel_raw = state.get("fuel")
            fuel_liters = _coerce_float(fuel_raw)
            fuel_warning_raw = state.get("fuel_warning")
            if fuel_warning_raw is None:
                fuel_warning_raw = state.get("fuelWarning")
            fuel_warning = _coerce_int(fuel_warning_raw)
            state_text, fuel_level = resolve_fuel_state(
                fuel_liters=fuel_liters,
                fuel_warning=fuel_warning,
                capacity_liters=self.capacity_liters,
                use_threshold_logic=self._use_threshold_logic,
            )
            state_map[int(aircraft_id)] = state_text
            prev_text = self._prev_state.get(int(aircraft_id))
            if fuel_level in (1, 2) and prev_text != state_text:
                warnings.append(
                    {
                        "timestamp": ts_value,
                        "source": str(source),
                        "aircraftID": int(aircraft_id),
                        "fuelLevel": int(fuel_level),
                    }
                )
            self._prev_state[int(aircraft_id)] = state_text

        return warnings, state_map
