from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .base import OperationContext, OperationMode, OperationResult


@dataclass
class ModeAutoTracking(OperationMode):
    """Mode 3: auto-tracking placeholder."""

    mode_id: int = 3

    def apply(
        self,
        *,
        uav: Any,
        filming_prop: dict,
        ctx: OperationContext,
        dt: float,
        current_wp_id: int | None,
        prev_state: Any,
    ) -> OperationResult:
        return OperationResult(target=ctx.default_target_fn(uav), state=None)
