# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from typing import Any

try:
    from modules.common import replan_perf
except Exception:
    _COMMON_DIR = next(
        (
            parent / "common"
            for parent in Path(__file__).resolve().parents
            if (parent / "common" / "replan_perf.py").exists()
        ),
        None,
    )
    if _COMMON_DIR is not None and str(_COMMON_DIR) not in sys.path:
        sys.path.insert(0, str(_COMMON_DIR))
    import replan_perf  # type: ignore


def payload_signature_context(payload: object | None) -> tuple[bytes | None, dict[str, Any] | None]:
    perf_start = replan_perf.start_timer()
    if payload is None:
        replan_perf.add_elapsed("monitoring.payload_signature", perf_start, payload_none=1)
        return None, None
    parsed_body: dict[str, Any] | None = None
    if isinstance(payload, bytes):
        raw = payload
    elif isinstance(payload, bytearray):
        raw = bytes(payload)
    elif isinstance(payload, str):
        raw = payload.encode("utf-8", "ignore")
    elif isinstance(payload, dict):
        parsed_body = dict(payload)
        try:
            raw = json.dumps(
                payload,
                ensure_ascii=False,
                sort_keys=True,
                default=str,
            ).encode("utf-8", "ignore")
        except Exception:
            raw = repr(payload).encode("utf-8", "ignore")
    elif isinstance(payload, list):
        if payload:
            last = payload[-1]
            if isinstance(last, dict):
                parsed_body = dict(last)
        try:
            raw = json.dumps(
                payload,
                ensure_ascii=False,
                sort_keys=True,
                default=str,
            ).encode("utf-8", "ignore")
        except Exception:
            raw = repr(payload).encode("utf-8", "ignore")
    else:
        try:
            raw = json.dumps(
                payload,
                ensure_ascii=False,
                sort_keys=True,
                default=str,
            ).encode("utf-8", "ignore")
        except Exception:
            raw = repr(payload).encode("utf-8", "ignore")
    try:
        text = raw.decode("utf-8", "ignore")
        match = re.search(r"\{.*\}", text, flags=re.S)
        if match:
            obj = json.loads(match.group(0))
            if parsed_body is None and isinstance(obj, dict):
                parsed_body = dict(obj)
            signature = json.dumps(
                obj,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                default=str,
            ).encode("utf-8", "ignore")
            replan_perf.add_elapsed(
                "monitoring.payload_signature",
                perf_start,
                raw_bytes=len(raw),
                canonicalized=1,
                signature_bytes=len(signature),
            )
            return signature, parsed_body
    except Exception:
        pass
    replan_perf.add_elapsed(
        "monitoring.payload_signature",
        perf_start,
        raw_bytes=len(raw),
        fallback_raw=1,
    )
    return raw, parsed_body
