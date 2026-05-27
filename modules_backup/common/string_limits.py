# -*- coding: utf-8 -*-
from __future__ import annotations


DEFAULT_STRING_BYTE_LIMIT = 128
NOTICE_CONTENTS_BYTE_LIMIT = 128
REPLAN_REASON_BYTE_LIMIT = 128


def limit_utf8_bytes(value: object, max_bytes: int = DEFAULT_STRING_BYTE_LIMIT) -> str:
    """Return text that fits the configured UTF-8 byte budget."""
    text = str(value or "").strip()
    limit = max(0, int(max_bytes or 0))
    if limit <= 0:
        return ""
    data = text.encode("utf-8", "ignore")
    if len(data) <= limit:
        return text
    return data[:limit].decode("utf-8", "ignore").rstrip()
