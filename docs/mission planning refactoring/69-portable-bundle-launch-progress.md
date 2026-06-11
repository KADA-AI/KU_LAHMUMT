# Portable Bundle Launch Progress

## Scope

This checkpoint verifies the portable mission bundle launch contract for both direct Python execution and the Windows batch launcher.

## Added

- `smoke_portable_bundle_launch.py`

## Contract Captured

- `portable_mission_bundle/app.py`:
  - inserts the bundle root into `sys.path`.
  - creates the Flask app with `create_app(ROOT)`.
  - uses `MISSION_APP_HOST`, defaulting to `127.0.0.1`.
  - uses `MISSION_APP_PORT`, defaulting to `8877`.
  - runs with `debug=False`.
- `portable_mission_bundle/run_portable.bat`:
  - changes directory to the bundle root with `cd /d "%~dp0"`.
  - runs `python app.py`.
- Flask endpoints:
  - `/api/health` responds.
  - `/api/model` responds.
  - model file remains `latest_model.zip`.
  - config file remains `model_config.json`.

## Boundary

This smoke does not upload DEMs, create sessions, or run RL inference. It starts the local Flask server on a temporary localhost port, probes health/model metadata, and terminates the process tree.

## Why This Is Safe

No runtime code changed. The smoke uses temporary ports via `MISSION_APP_PORT`, reads only existing bundle files, and stops spawned processes with a process-tree kill on Windows.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_portable_bundle_launch.py"
python "docs\mission planning refactoring\smoke_portable_bundle_launch.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
portable bundle launch smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `next-collab/next-area manual planner flow-mode smoke`.
