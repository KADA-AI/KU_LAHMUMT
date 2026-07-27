"""ECF (Estimated Consumed Fuel) for emitted flight-plan waypoints.

ICD 0303/0304 define ``ecf`` as::

    float ecf: unit = Liter, min 0, max 1000
    해당 구간(이전 WP -> 이 WP)을 비행하는 동안 소모할 것으로 예상되는 연료량

That is a **per-leg** quantity in litres, not a progress fraction.  The value
is estimated from the leg's duration against a nominal endurance, because leg
time is the one thing every emitter already computes.

The first waypoint of a plan has no preceding leg, so its ECF is 0 - matching
the ETA contract, where the first waypoint is fixed at 0 and later waypoints
accumulate.
"""

from __future__ import annotations

import math
from typing import Any, Iterable

# Airframe endurance the estimate is normalised against.  ~1,080 L of JP-8
# (LAHOperationalEnvelope.fuel_capacity_kg = 920 kg) burned over a two-hour
# mission, i.e. 540 L/h.
NOMINAL_USABLE_FUEL_L = 1080.0
NOMINAL_ENDURANCE_S = 2.0 * 3600.0
NOMINAL_BURN_L_PER_S = NOMINAL_USABLE_FUEL_L / NOMINAL_ENDURANCE_S

# ICD range for the field.
ECF_MIN_L = 0.0
ECF_MAX_L = 1000.0


def leg_fuel_litres(leg_seconds: float, *, burn_l_per_s: float | None = None) -> float:
    """Estimated fuel for one leg, clamped into the ICD range."""

    try:
        seconds = float(leg_seconds)
    except (TypeError, ValueError):
        return ECF_MIN_L
    if not math.isfinite(seconds) or seconds <= 0.0:
        return ECF_MIN_L
    rate = burn_l_per_s if burn_l_per_s is not None else NOMINAL_BURN_L_PER_S
    try:
        rate = float(rate)
    except (TypeError, ValueError):
        rate = NOMINAL_BURN_L_PER_S
    if not math.isfinite(rate) or rate <= 0.0:
        rate = NOMINAL_BURN_L_PER_S
    return round(min(ECF_MAX_L, max(ECF_MIN_L, seconds * rate)), 3)


def _eta_seconds(waypoint: Any) -> float:
    if not isinstance(waypoint, dict):
        return 0.0
    for key in ("eta", "ETA", "Eta"):
        if key not in waypoint:
            continue
        try:
            value = float(waypoint.get(key) or 0.0)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value) and value >= 0.0:
            return value
    return 0.0


def apply_leg_fuel_inplace(
    waypoints: Iterable[Any],
    *,
    hover_key: str = "hovering",
    burn_l_per_s: float | None = None,
) -> None:
    """Write per-leg ECF onto an ordered waypoint list.

    Leg duration comes from the cumulative ETA difference.  A commanded hover
    burns fuel without advancing ETA, so its dwell is added on top.
    """

    ordered = [wp for wp in (waypoints or []) if isinstance(wp, dict)]
    if not ordered:
        return
    previous_eta_s = _eta_seconds(ordered[0])
    ordered[0]["ecf"] = ECF_MIN_L
    for waypoint in ordered[1:]:
        current_eta_s = _eta_seconds(waypoint)
        leg_seconds = max(0.0, current_eta_s - previous_eta_s)
        hover = waypoint.get(hover_key)
        if isinstance(hover, dict):
            try:
                leg_seconds += max(0.0, float(hover.get("time") or 0.0))
            except (TypeError, ValueError):
                pass
        waypoint["ecf"] = leg_fuel_litres(leg_seconds, burn_l_per_s=burn_l_per_s)
        previous_eta_s = max(previous_eta_s, current_eta_s)


__all__ = [
    "ECF_MAX_L",
    "ECF_MIN_L",
    "NOMINAL_BURN_L_PER_S",
    "NOMINAL_ENDURANCE_S",
    "NOMINAL_USABLE_FUEL_L",
    "apply_leg_fuel_inplace",
    "leg_fuel_litres",
]
