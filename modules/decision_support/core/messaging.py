"""Message sender helpers for the decision support module."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Optional

from .time_utils import now_ms_since_2000

try:  # Default push dispatcher
    from push_center import push_message as _default_push_message  # type: ignore
except Exception:  # pragma: no cover - runtime guard
    _default_push_message = None  # type: ignore


PushFunc = Callable[[str, Any], Any]


@dataclass
class SelfCheckMessenger:
    """Handles crafting and pushing 0102 self-check payloads."""

    node_cls: Any
    source: str = "MOB"
    push_func: Optional[Callable[..., Any]] = None
    last_error: Optional[Exception] = field(default=None, init=False)

    def __post_init__(self) -> None:
        if self.push_func is None:
            self.push_func = _default_push_message

    def build_payload(self, status: int) -> dict[str, Any]:
        return {
            "Timestamp": now_ms_since_2000(),
            "Status": int(status),
            "Source": self.source,
        }

    def send(self, status: int) -> bool:
        self.last_error = None
        if not callable(self.push_func):
            self.last_error = RuntimeError("push_message unavailable")
            return False
        body = self.build_payload(status)
        try:
            return bool(self.push_func("0102", self.node_cls, body_dict=body))
        except Exception as exc:  # pragma: no cover - runtime logging only
            self.last_error = exc
            return False


@dataclass
class OptionInfoMessenger:
    """Handles pushing 0701 option info payloads."""

    node_cls: Any
    push_func: Optional[Callable[..., Any]] = None
    last_error: Optional[Exception] = field(default=None, init=False)

    def __post_init__(self) -> None:
        if self.push_func is None:
            self.push_func = _default_push_message

    def send(self, body: dict[str, Any]) -> bool:
        self.last_error = None
        if not callable(self.push_func):
            self.last_error = RuntimeError("push_message unavailable")
            return False
        try:
            return bool(self.push_func("0701", self.node_cls, body_dict=body))
        except Exception as exc:  # pragma: no cover - runtime logging only
            self.last_error = exc
            return False


__all__ = ["SelfCheckMessenger", "OptionInfoMessenger"]
