# 2026-06-05 Settings Migration Note

Scope:

- Moved project-level operator/runtime settings from the project root to
  `settings/`.
- Added `modules.common.settings_paths` as the shared path contract.
- Updated runtime users to read/write canonical `settings/` paths while keeping
  root-level legacy fallbacks for migration safety where needed.
- Added `smoke_project_settings_surface.py` to fail if migrated settings or a
  root `__pycache__` reappear in the project root.

Canonical files:

- `settings/current_scenario.json`
- `settings/nFusionSettings.json`
- `settings/nFusionLicense.lic`
- `settings/replan_settings.json`
- `settings/replan_settings_defaults.json`
- `settings/uav_params.json`

Cleanup performed:

- Removed a duplicate untracked root `nFusionSettings.json` after confirming the
  canonical `settings/nFusionSettings.json` existed. The duplicate differed from
  the canonical file and did not include the full canonical middleware shape, so
  it was not merged.
- Removed root `__pycache__` generated during smoke execution.

Verification:

- `python -m py_compile modules\common\settings_paths.py "docs\project root cleanup\smoke_project_settings_surface.py"`
- `python "docs\project root cleanup\smoke_project_settings_surface.py"`
- `python "docs\mission planning refactoring\smoke_runtime_artifact_paths.py"`
- `python "docs\mission planning refactoring\smoke_nfusion_contract.py"`

Deferred:

- Root folders such as `modules copy`, `Logs`, `resource`, `ref`, and `temp`
  were not removed. Those need a separate owner/risk check because they may
  contain backups, runtime logs, or active resources.
