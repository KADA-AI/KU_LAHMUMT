# 0902 Replay And Store-Backed Detail Progress

## Scope

This checkpoint freezes the current 0902 replay sidecar and store-backed replan detail behavior.

## Added

- `smoke_0902_replay_store_detail.py`

## Contract Captured

- `replan_request_transport_store.payload_path_for_payload(...)` resolves paths from positive `timestamp` or `replanRequestTime.replanRequestTimestamp`.
- `save_payload(...)` writes sidecar entries under `DSS_Internal/replan_request_transport/replan_request_<timestamp>.json`.
- Compact sidecar mode writes a JSON list.
- Duplicate payload identity is deduped.
- Different identity entries with the same timestamp append to the same sidecar file.
- `load_payload(...)` can filter by reason and replan level.
- If filters miss, `load_payload(...)` returns the last entry.
- `load_latest_payload()` returns the newest timestamp entry.
- Sidecar off mode disables `save_payload(...)`.
- GUI `_capture_replan_payload_for_replay(...)` writes the sidecar capture path into `ctx["_0902_capture_path"]` and records `0902_archived`.
- Generic `runtime.replan_store` saves/loads detail files and sanitized event files under `DSS_Internal/<store_name>`.
- `runtime.next_collab_replan_store` preserves the generic store path contract.
- Common prior/imaging-schedule/path-deviation detail stores save/load their trigger-specific detail files.

## Why This Is Safe

No runtime code changed. The smoke monkeypatches `modules.common.db_paths.get_db_subpath` inside temporary directories and restores it afterward. It also closes process-console file sinks opened by `mission_planning_gui.py` import so temporary log files do not leak handles.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_0902_replay_store_detail.py"
python "docs\mission planning refactoring\smoke_0902_replay_store_detail.py"
```

Result:

```text
0902 replay/store-backed detail smoke ok
```

## Next

Next incomplete TODO: `replan dispatcher priority/handled semantics fixture 작성`.
