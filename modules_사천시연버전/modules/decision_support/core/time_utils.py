"""Time-related helpers shared across decision support logic."""

from __future__ import annotations

import time

_EPOCH2000_MS = 946_684_800_000


def now_ms_since_2000() -> int:
    """Return current time in milliseconds relative to 2000-01-01 UTC."""
    return int(time.time() * 1000) - _EPOCH2000_MS


__all__ = ["now_ms_since_2000"]
