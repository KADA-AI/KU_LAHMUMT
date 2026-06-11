# Runtime Artifact/Resource Paths Progress

## Scope

This checkpoint freezes the active runtime artifact/resource path manifest before more folder moves or cleanup decisions.

## Added

- `smoke_runtime_artifact_paths.py`

## Contract Captured

- Active mission DB roots remain:
  - `temp/database` as the legacy fallback DB root.
  - `Logs/Scenario_<iso>/<agency>` as the scenario DB shape.
  - `settings/current_scenario.json` as the scenario pointer.
  - `KU_MISSION_DB_ROOT`, `KU_SCENARIO_ROOT`, and `KU_SCENARIO_BASE_ROOT` as the env override names.
- Active DB scaffold directories still include `DSS_Internal`, mission payload folders, `MissionReferenceInfo`, and `mission_output`.
- 0401 runtime artifacts remain:
  - `DSS_Internal/latest_0401_agent_status.json`
  - `DSS_Internal/log_0401_agent_status_sim.jsonl`
  - `simlog_0401/0401.json`, `simlog_0401/0401_<n>.json`
  - `KU_SIM_0401_LOG_DIR` as the JSON-array log override.
- 0902/replan sidecar stores remain under `DSS_Internal`:
  - `replan_request_transport/replan_request_<timestamp>.json`
  - `replan_inputs/0201_override_source<missionPlanID>_<suffix>.json`
  - `NextCollab_<inputMissionID>_<timestamp>.json`
  - `next_collab_replan/next_collab_detail_<missionPlanID>.json`
  - `prior_replan/prior_detail_<missionPlanID>.json`
  - `imaging_schedule_replan/imaging_schedule_detail_<missionPlanID>.json`
  - `path_deviation_replan/path_deviation_detail_<missionPlanID>.json`
  - `mission_area_replan/mission_area_snapshot_<missionPlanID>.json`
  - `mission_area_replan/mission_area_snapshot_audit.jsonl`
  - `prior_target_rediscovery/state.json`
- Runtime/debug artifact mode env names remain `REPLAN_RUNTIME_ARTIFACT_MODE` and `REPLAN_DEBUG_ARTIFACT_MODE`; 0902 transport mode env names remain `REPLAN_0902_SIDECAR_MODE` and `REPLAN_SIDECAR_MODE`.
- Resource paths remain:
  - default FOV DB path: `resource/db/fov_db.csv`
  - active `settings/uav_params.json` FOV DB override: `resource/db/fov_db_탱크_72_38_6_3.2.csv`
  - `resource/korea.mbtiles` through SIM map config
  - `resource/*.tif` DEM candidates for mission planner terrain use
  - attack assistance fallback scan order: `resource/`, then `resources/`
  - `modules/mission_planning/MissionPlanner/AnS/DEM.jpg`
  - `modules/mission_planning/MissionPlanner/AnS/_id_counters.json`
  - `modules/mission_planning/MissionPlanner/portable_mission_bundle/models/latest_model.zip`
  - `modules/mission_planning/MissionPlanner/portable_mission_bundle/models/model_config.json`
  - portable bundle `data/inputs` and `data/work`
  - nFusion config/license candidates and `modules/common/msg_files/MessageLibrary(.dll)`
- Log/output artifact markers remain:
  - `DSS_Internal/module_logs/<module>.log`
  - `DSS_Internal/missionPlan_<planID>.json`
  - `DSS_Internal/missionPlan_<planID>_<token>.json`
  - `DSS_Internal/missionPlan_pending_<token>.json`
  - `DSS_Internal/mission_planning_gui_<token>.log`
  - active DB `mission_output/`

## Audit Drift Noted

The second-review audit named `DSS_Internal/agent_status_0401.jsonl`, but the current implementation uses `DSS_Internal/log_0401_agent_status_sim.jsonl`. The smoke locks the implementation name, not the older audit label.

The default FOV DB setting remains `resource/db/fov_db.csv`, but the current operator config in `settings/uav_params.json` points to `resource/db/fov_db_탱크_72_38_6_3.2.csv`. The smoke checks the active override exists so a refactor does not silently fall back to the absent default CSV.

## Boundary

This is a path manifest smoke, not a helper behavior smoke. It intentionally avoids validating `json_io`, `latest_input_cache`, `mission_plan_file_logger`, and `mission_planning_pipeline_logging` behavior because that is the next roadmap item.

This checkpoint also does not cover:

- ID/state JSON artifact semantics.
- Runtime DB state JSON semantics such as `targetInfo.json`, `VehicleStatus/status.json`, progress/state JSON.
- HTML/PNG output semantics.
- Portable bundle launch behavior.

Those are separate roadmap items.

## Why This Is Safe

No runtime code changed. The smoke uses a temporary DB root and monkeypatches `db_paths` path helpers while checking runtime paths. Source-only checks are used for heavyweight GUI/AnS/portable modules so the smoke does not import the full GUI, stable-baselines/PPO path, GDAL attack helper, or nFusion DLL loader.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_runtime_artifact_paths.py"
python "docs\mission planning refactoring\smoke_runtime_artifact_paths.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
runtime artifact/resource path manifest smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `runtime I/O/cache/log helper smoke`.
