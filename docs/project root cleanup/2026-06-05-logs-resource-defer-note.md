# 2026-06-05 Logs And Resource Defer Note

`Logs/` and `resource/` are still visible at the project root, but they are not
safe deletion targets.

`Logs/` evidence:

- `settings/current_scenario.json` points `base_root` to
  `C:\Users\LAHMUMT_2\Desktop\DSS_KU\Logs`.
- `modules.common.db_paths.DEFAULT_SCENARIO_BASE` resolves to root `Logs`.
- Runtime helpers use the active DB root for message payloads, mission plans,
  vehicle status, monitoring state, and simulation payload observations.
- `Logs/` has thousands of tracked files in git history plus current untracked
  scenario folders.

Decision:

- Keep `Logs/` in place for this checkpoint.
- Keep `/Logs/` ignored so new runtime scenario data is not staged
  accidentally.
- Moving `Logs/` must be a separate migration with explicit user approval.

`resource/` evidence:

- Mission planning and simulation code directly reads `resource/db`,
  `resource/korea.mbtiles`, and `resource/*.tif`.
- nFusion schema and generated message resources are tracked under `resource/`.
- Existing mission-planning smoke tests assert the current `resource/` contract.

Decision:

- Keep `resource/` in place for this checkpoint.
- Do not move it until resource path resolution is centralized and runtime
  smokes are updated.
