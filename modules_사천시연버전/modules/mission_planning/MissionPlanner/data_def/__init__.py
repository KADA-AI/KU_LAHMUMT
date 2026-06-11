from __future__ import annotations

import sys
from typing import Any

_CANONICAL_NAME = "modules.mission_planning.MissionPlanner.data_def"
if __name__ == "data_def":
    sys.modules.setdefault(_CANONICAL_NAME, sys.modules[__name__])
elif __name__ == _CANONICAL_NAME:
    sys.modules.setdefault("data_def", sys.modules[__name__])

__all__ = ["build_lah_flight_plans_fixed"]


def build_lah_flight_plans_fixed(*args: Any, **kwargs: Any):
    from .d0304 import build_lah_flight_plans_fixed as _impl

    return _impl(*args, **kwargs)
