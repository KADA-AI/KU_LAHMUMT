"""System-mode and self-check message helpers."""

from __future__ import annotations

import json
import re
from typing import Any, Callable, Mapping


MODE_LABELS = ("초기화 모드", "대기모드", "초기 임무 계획", "임무 수행")
DEFAULT_0102_SOURCE = "MMR"
DEFAULT_0102_STATUS = 1

_JSON_OBJECT_RE = re.compile(r"\{.*\}", flags=re.S)
_SYSTEM_MODE_RE = re.compile(r'"systemMode"\s*:\s*([0-9]+)')


def decode_raw_payload(raw: bytes | bytearray | None) -> str:
    if raw is None:
        return ""
    if isinstance(raw, bytearray):
        raw = bytes(raw)
    if isinstance(raw, bytes):
        return raw.decode("utf-8", "ignore")
    return str(raw)


def parse_payload_body(raw: bytes | bytearray | None) -> dict[str, Any]:
    text = decode_raw_payload(raw)
    match = _JSON_OBJECT_RE.search(text)
    json_text = match.group(0) if match else text.strip()
    try:
        payload = json.loads(json_text) if json_text.startswith("{") else {}
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def extract_mode_code(body: Mapping[str, Any] | None) -> int | None:
    if not isinstance(body, Mapping):
        return None
    lowered = {str(key).lower(): body[key] for key in body.keys() if key is not None}
    for key in ("systemmode", "mode", "modecode", "state"):
        if key not in lowered:
            continue
        value = lowered[key]
        if isinstance(value, bool):
            return 1 if value else 0
        try:
            return int(value)
        except Exception:
            try:
                return int(float(str(value).strip()))
            except Exception:
                return None
    return None


def extract_mode_code_from_raw(raw: bytes | bytearray | None) -> int | None:
    text = decode_raw_payload(raw)
    match = _SYSTEM_MODE_RE.search(text)
    if not match:
        return None
    try:
        return int(match.group(1))
    except Exception:
        return None


def extract_system_mode_code(
    raw: bytes | bytearray | None,
    body: Mapping[str, Any] | None = None,
) -> int | None:
    code = extract_mode_code(body)
    if code is not None:
        return code
    return extract_mode_code_from_raw(raw)


def normalize_0102_body_template(
    body: Mapping[str, Any] | None,
    *,
    status: int | None = None,
) -> dict[str, Any]:
    template = dict(body or {})
    template.pop("Timestamp", None)
    template.pop("timestamp", None)

    source = template.get("source") or template.get("Source") or DEFAULT_0102_SOURCE
    template["source"] = str(source)
    template.pop("Source", None)

    if status is None:
        try:
            status = int(template.get("status", template.get("Status", DEFAULT_0102_STATUS)))
        except Exception:
            status = DEFAULT_0102_STATUS
    template["status"] = int(status)
    template.pop("Status", None)
    return template


def build_0102_body(
    template: Mapping[str, Any] | None,
    *,
    now_ms: Callable[[], int],
    status: int | None = None,
) -> dict[str, Any]:
    body = normalize_0102_body_template(template, status=status)
    body["timestamp"] = int(now_ms())
    return body


def resolve_mode_code_from_text(text: str) -> int:
    normalized = re.sub(r"\s+", "", str(text)).lower()
    mapping = {
        "전원off": 0,
        "off": 0,
        "poweroff": 0,
        "전원on": 0,
        "on": 0,
        "poweron": 0,
        "0": 0,
        "초기화": 0,
        "초기화모드": 0,
        "초기화mode": 0,
        "1": 1,
        "대기모드": 1,
        "대기": 1,
        "standby": 1,
        "2": 2,
        "초기임무계획": 2,
        "초기임무계획모드": 2,
        "initplan": 2,
        "initial": 2,
        "3": 3,
        "임무수행": 3,
        "execution": 3,
    }
    return int(mapping.get(normalized, 1))
