from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def fail(message: str) -> None:
    raise AssertionError(message)


def expect_equal(label: str, actual: object, expected: object) -> None:
    if actual != expected:
        fail(f"{label} changed: expected {expected!r}, got {actual!r}")


def main() -> int:
    from modules.mission_planning.app.message_handlers import system_mode

    expect_equal("decode None", system_mode.decode_raw_payload(None), "")
    expect_equal("decode bytearray", system_mode.decode_raw_payload(bytearray(b"abc")), "abc")

    wrapped_raw = b'prefix {"systemMode": 2, "ignored": true} suffix'
    expect_equal("parse wrapped JSON object", system_mode.parse_payload_body(wrapped_raw), {"systemMode": 2, "ignored": True})
    expect_equal("parse non-object body", system_mode.parse_payload_body(b'["not", "dict"]'), {})
    expect_equal("parse invalid body", system_mode.parse_payload_body(b"{invalid"), {})

    alias_cases = (
        ("systemMode", {"systemMode": 0}, 0),
        ("mode", {"mode": "1"}, 1),
        ("modeCode", {"modeCode": "2.0"}, 2),
        ("state", {"state": 3}, 3),
        ("case-insensitive SystemMode", {"SystemMode": 2}, 2),
        ("case-insensitive MODECODE", {"MODECODE": "3"}, 3),
    )
    for label, body, expected in alias_cases:
        expect_equal(f"0101 top-level alias {label}", system_mode.extract_mode_code(body), expected)

    expect_equal(
        "0101 alias precedence",
        system_mode.extract_mode_code({"state": 3, "modeCode": 2, "mode": 1, "systemMode": 0}),
        0,
    )
    expect_equal("0101 bool true mapping", system_mode.extract_mode_code({"systemMode": True}), 1)
    expect_equal("0101 bool false mapping", system_mode.extract_mode_code({"systemMode": False}), 0)
    expect_equal("0101 invalid alias value", system_mode.extract_mode_code({"systemMode": "invalid"}), None)
    expect_equal("0101 nested body ignored by body parser", system_mode.extract_mode_code({"outer": {"systemMode": 2}}), None)
    expect_equal("0101 unsupported body", system_mode.extract_mode_code(["systemMode", 2]), None)

    expect_equal(
        "0101 system extract uses body first",
        system_mode.extract_system_mode_code(b'{"systemMode": 3}', {"mode": 1}),
        1,
    )
    expect_equal(
        "0101 raw fallback numeric systemMode",
        system_mode.extract_system_mode_code(b'noise before "systemMode": 3 noise after', {}),
        3,
    )
    expect_equal(
        "0101 raw fallback also accepts nested lexical marker",
        system_mode.extract_system_mode_code(b'{"outer":{"systemMode":2}}', {}),
        2,
    )
    expect_equal("0101 raw fallback rejects uppercase key", system_mode.extract_mode_code_from_raw(b'"SystemMode": 2'), None)
    expect_equal("0101 raw fallback rejects quoted number", system_mode.extract_mode_code_from_raw(b'"systemMode": "2"'), None)
    expect_equal("0101 raw fallback rejects mode alias", system_mode.extract_mode_code_from_raw(b'"mode": 2'), None)

    expect_equal("0101 text on maps to power-on mode", system_mode.resolve_mode_code_from_text("on"), 0)
    expect_equal("0101 text poweron maps to power-on mode", system_mode.resolve_mode_code_from_text("poweron"), 0)
    expect_equal("0101 text off maps to power-on/off mode", system_mode.resolve_mode_code_from_text("off"), 0)
    expect_equal("0101 text standby maps to standby mode", system_mode.resolve_mode_code_from_text("standby"), 1)
    expect_equal("0101 text initplan maps to init planning mode", system_mode.resolve_mode_code_from_text("init plan"), 2)
    expect_equal("0101 text execution maps to execution mode", system_mode.resolve_mode_code_from_text("execution"), 3)
    expect_equal("0101 unknown text falls back to standby", system_mode.resolve_mode_code_from_text("unknown"), 1)

    print("0101 parsing allow-list smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
