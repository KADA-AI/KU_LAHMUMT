# 0101 Parsing Allow-List Progress

## Scope

This checkpoint freezes the current 0101 system-mode parsing contract before further message-handler refactoring.

## Added

- `smoke_0101_parsing_allow_list.py`

## Contract Captured

- `decode_raw_payload()`:
  - `None` decodes to an empty string.
  - `bytearray` is accepted and decoded as UTF-8.
- `parse_payload_body()`:
  - extracts the first JSON object from wrapped raw text.
  - returns `{}` for non-dict JSON and invalid JSON.
- `extract_mode_code()`:
  - only reads top-level keys from mapping bodies.
  - allowed aliases are `systemMode`, `mode`, `modeCode`, and `state`.
  - key matching is case-insensitive for mapping bodies.
  - alias precedence is `systemMode` -> `mode` -> `modeCode` -> `state`.
  - `True` maps to `1`; `False` maps to `0`.
  - invalid values return `None`.
  - nested body values are ignored by the body parser.
- `extract_system_mode_code()`:
  - body extraction wins over raw fallback.
  - raw fallback accepts the lexical marker `"systemMode": <number>`.
  - raw fallback is intentionally narrow: uppercase `"SystemMode"`, quoted numbers, and `"mode"` aliases are not accepted.
  - current raw fallback is lexical, so a nested raw `"systemMode": <number>` marker is still matched when body extraction fails.
- `resolve_mode_code_from_text()`:
  - `"on"` and `"poweron"` map to mode `0`.
  - `"off"` also maps to mode `0`.
  - `"standby"` maps to mode `1`.
  - `"init plan"` maps to mode `2`.
  - `"execution"` maps to mode `3`.
  - unknown text falls back to mode `1`.

## Boundary

This smoke validates helper-level parsing only. It does not exercise GUI side effects after a mode change, 0102 heartbeat send behavior, nFusion receive loops, or dashboard controls.

## Why This Is Safe

No runtime code changed. The smoke imports only `modules.mission_planning.app.message_handlers.system_mode` and calls pure helper functions.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_0101_parsing_allow_list.py"
python "docs\mission planning refactoring\smoke_0101_parsing_allow_list.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
0101 parsing allow-list smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `0201/0203 latest input fixture`.
