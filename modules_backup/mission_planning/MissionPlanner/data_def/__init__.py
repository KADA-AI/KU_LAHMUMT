from __future__ import annotations

from typing import Any

__all__ = ["build_lah_flight_plans_fixed"]


def build_lah_flight_plans_fixed(*args: Any, **kwargs: Any):
    from .d0304 import build_lah_flight_plans_fixed as _impl

    return _impl(*args, **kwargs)
