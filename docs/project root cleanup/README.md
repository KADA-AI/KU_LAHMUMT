# Project Root Cleanup

The project root should stay limited to launchers, top-level folders, and a
small number of repository metadata files. Operator-editable runtime settings
belong in `settings/`.

Canonical settings files:

- `settings/current_scenario.json`
- `settings/nFusionSettings.json`
- `settings/nFusionLicense.lic`
- `settings/replan_settings.json`
- `settings/replan_settings_defaults.json`
- `settings/uav_params.json`

Migration guard:

- `smoke_project_settings_surface.py` fails if any migrated setting file exists
  in the project root again.
- `smoke_project_root_inventory.py` classifies every top-level project item and
  fails if a new loose root item appears without an explicit owner.
- `modules.common.settings_paths` is the shared path contract for these files.
- Legacy root fallbacks may exist in code while migration settles, but runtime
  writes should target `settings/`.
- `modules_bkup/` is a user-managed backup folder. Do not edit, move, or delete
  it during this cleanup work.
- `modules copy/` is treated as a local backup/quarantine name if it appears,
  not a project source root.
- Legacy root `ref/` documents were moved to `docs/reference/`.
- Legacy root `temp/` is absent and should stay absent; scratch/runtime files
  should not return to the project surface.
- `Logs/` and `resource/` remain at root for now because current runtime paths
  and active resource contracts still depend on them.
