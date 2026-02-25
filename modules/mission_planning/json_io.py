# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

try:
    import orjson as _orjson
except Exception:  # pragma: no cover - optional dependency
    _orjson = None


def dumps_json(
    data: Any,
    *,
    pretty: bool = True,
    ensure_ascii: bool = False,
    sort_keys: bool = False,
) -> bytes:
    if _orjson is not None:
        option = 0
        if pretty:
            option |= _orjson.OPT_INDENT_2
        if sort_keys:
            option |= _orjson.OPT_SORT_KEYS
        if ensure_ascii:
            option |= _orjson.OPT_ESCAPE_UNICODE
        return _orjson.dumps(data, option=option)

    if pretty:
        text = json.dumps(
            data,
            ensure_ascii=ensure_ascii,
            indent=2,
            sort_keys=sort_keys,
        )
    else:
        text = json.dumps(
            data,
            ensure_ascii=ensure_ascii,
            separators=(",", ":"),
            sort_keys=sort_keys,
        )
    return text.encode("utf-8")


def write_json(
    path: Path,
    data: Any,
    *,
    pretty: bool = True,
    ensure_ascii: bool = False,
    sort_keys: bool = False,
    skip_if_unchanged: bool = True,
) -> bool:
    payload = dumps_json(
        data,
        pretty=pretty,
        ensure_ascii=ensure_ascii,
        sort_keys=sort_keys,
    )

    if skip_if_unchanged and path.exists():
        try:
            if path.stat().st_size == len(payload):
                if path.read_bytes() == payload:
                    return False
        except Exception:
            pass

    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_bytes(payload)
    tmp_path.replace(path)
    return True
