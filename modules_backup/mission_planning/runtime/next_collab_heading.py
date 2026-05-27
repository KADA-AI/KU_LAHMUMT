from __future__ import annotations

from typing import Any


def monitor_heading_to_planner_bearing_deg(raw_heading_deg: Any) -> float:
    """Monitoring and next-collab planner both use 0=north, 90=east."""
    try:
        return float(raw_heading_deg) % 360.0
    except Exception:
        return 0.0
