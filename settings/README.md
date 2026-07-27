# Project Settings (machine-local only)

This folder now contains only machine-local runtime files — values that are
specific to this PC (active scenario pointer, nFusion runtime config/license)
and must not be shared through version control:

- `current_scenario.json`
- `nFusionSettings.json`
- `nFusionLicense.lic`

Algorithm-tuning settings moved to `modules/resource/` (version-controlled,
shipped with the code):

- `modules/resource/uav_params.json`
- `modules/resource/replan_settings.json`
- `modules/resource/replan_settings_defaults.json`

Path resolution lives in `modules/common/settings_paths.py`
(`algo_settings_file()`). If an algorithm file is missing there but a legacy
copy still exists in this folder, it is copied over automatically on first
access; the legacy copy is kept for rollback.
