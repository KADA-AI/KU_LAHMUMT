"""Utilities for broadcasting monitoring updates."""

from __future__ import annotations

import json
import socket
from typing import Any


class MonitorBroadcaster:
    """UDP broadcaster to mirror MOB activity to local monitor."""

    def __init__(self, port: int):
        self._port = int(port)

    def send(self, kind: str, **payload: Any) -> None:
        data: dict[str, Any] = {"kind": str(kind), **payload}
        try:
            buf = json.dumps(data, ensure_ascii=False).encode("utf-8", "ignore")
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(buf, ("127.0.0.1", self._port))
            sock.close()
        except Exception:
            pass


__all__ = ["MonitorBroadcaster"]
