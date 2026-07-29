"""Compatibility wrapper for the active mission-planning next-collab store."""

from modules.mission_planning.runtime.next_collab_replan_store import (
    load_detail,
    load_latest_detail_at_or_before,
    save_detail,
    save_event,
)

__all__ = [
    "load_detail",
    "load_latest_detail_at_or_before",
    "save_detail",
    "save_event",
]
