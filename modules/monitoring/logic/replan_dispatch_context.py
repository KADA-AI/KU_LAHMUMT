# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class Replan0401DispatchContext:
    raw_payload: object | None
    parsed_body: dict[str, Any] | None
    canonical_signature: bytes | None
    timestamp_ms: int | None
    agent_states: list[dict[str, Any]]
    settings_snapshot: dict[str, Any]
    db_json_cache: dict[tuple[str, int], dict[str, Any]] = field(default_factory=dict)

    def cached_db_json(self, folder: str, file_id: int | None, loader) -> dict[str, Any]:
        try:
            key = (str(folder), int(file_id))  # type: ignore[arg-type]
        except Exception:
            return loader(folder, file_id)
        cached = self.db_json_cache.get(key)
        if cached is not None:
            return cached
        payload = loader(folder, file_id)
        if isinstance(payload, dict):
            self.db_json_cache[key] = payload
            return payload
        return {}
