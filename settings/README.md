# Project Settings

This folder contains project-level runtime settings and operator-editable JSON
files that used to live in the repository root.

Do not keep duplicate copies of these files in the project root. Runtime code
should read and write the canonical files here, with root-level paths used only
as temporary legacy fallbacks during migration.

Canonical files:

- `current_scenario.json`
- `nFusionSettings.json`
- `nFusionLicense.lic`
- `replan_settings.json`
- `replan_settings_defaults.json`
- `uav_params.json`
