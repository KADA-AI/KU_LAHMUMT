# Line/Area Resume Stability TODO

Date: 2026-06-07

## Scope

This TODO covers line/area resume and rejoin stability across current remaining, next-collab, attack, prior, post-attack, and reexecute flows.

The issue is not only `0401` or `currentWaypointID`. The larger risk is that callers interpret current input, entry position, remaining geometry, and snapshot provenance differently before calling the shared replanning helpers.

The short-term attack guard now preserves area assignments when a UAV temporarily leaves an area input for target tracking. This document defines the broader follow-up work so line and area behavior stays stable across all triggers.

## Observed Failure

Scenario observed on 2026-06-07:

- UAV 4, UAV 5, and UAV 6 were assigned to the same area input.
- UAV 6 detected/tracked a target while inside that area input.
- Existing attack logic treated UAV 6 as unavailable for the current input and rebuilt the remaining area for UAV 4 and UAV 5.
- UAV 4 and UAV 5 therefore abandoned their own current area work and took over split pieces of UAV 6's area.

The immediate attack-side guard now skips collaborative takeover for area inputs and preserves non-tracking UAV assignments. The latest log shows the intended result:

- `Logs/Scenario_2026-06-07T173852/SBC3/DSS_Internal/log_attack_algorithm_20260607T084422_758380.json`
- messages include `area input keeps existing UAV assignments`, `preserved_area_assignment`, and `collabRuns=[]`.

## Current Risk Map

- Entry coordinates are resolved through different paths in normal current-remaining, reexecute-first, attack, and prior flows.
- `currentWaypointID` mismatch can fall back to a first-mission guess, then still feed line/area resume logic.
- Snapshot lookup uses exact plan snapshots in some paths, but `allow_latest=True` fallback in others.
- Snapshot provenance is not consistently propagated to downstream policy decisions or logs.
- Area remaining geometry can preserve multiple polygons/holes, but `_prepare_area_replacements` currently sends only the first polygon to the division planner.
- Line remaining geometry can be lossy-compressed from multiple segments into one representative centerline and expanded width.
- `prepare_next_collab_input_replacements` callers decide preserve/redivide behavior separately, so the same line/area input can behave differently by trigger.
- Missing entry coordinates can silently shrink the target UAV set and change the split result.
- Planner failure fallback and intentional policy preserve/skip are not always distinguishable in logs.

## Target Contract

Add a shared resume/rejoin policy decision before any line/area redivision builder runs.

Minimal P0 decision fields:

- `caller`
- `trigger`
- `missionType`
- `decision`: `preserve`, `redivide`, `lost_uav_takeover_only`, or `skip_fail_closed`
- `reason`
- `sourcePlanID`
- `inputMissionID`
- `snapshotMode`: `exact`, `carried_forward`, `latest_fallback`, or `missing`
- `expectedAircraftIDs`
- `unavailableAircraftIDs`
- `activeAircraftIDs`
- `entryResolvedAircraftIDs`
- `geometryRisk`

Default policy:

- Area plus temporary attack/tracking unavailable: preserve existing non-tracking UAV assignments.
- Area plus permanent unavailable aircraft: only consider piece-only takeover, not full area redivision.
- Area with multi-polygon/hole geometry and no planner support: preserve or fail closed.
- Line with a single or safely collinear residual segment: redivide may be allowed.
- Line with disjoint or multi-block residual geometry: preserve or fail closed until segment-wise planning exists.
- Any execution replan with stale current input, first-mission fallback, missing exact/carry-forward snapshot, or incomplete target aircraft set: do not silently redivide.

## P0 TODO

- Add a shared resume policy gate used before line/area redivision.
- Thread the policy decision into logs, plan metadata, and debug artifacts.
- Add current input confidence checks. If `currentWaypointID` cannot be matched to the active plan and falls back to the first mission, line/area redivision must be blocked.
- Add target aircraft set checks. Record expected/template UAVs, unavailable UAVs, active UAVs, and entry-resolved UAVs separately.
- Add snapshot provenance propagation. Execution replans should use exact or carry-forward-proven snapshots; latest fallback should fail closed unless explicitly allowed for a non-execution/manual path.
- Gate `allow_latest=True` use in execution paths.
- Add an area first-polygon guard. If remaining area has multiple outer polygons or holes, do not feed only the first polygon into the division planner without an explicit policy decision.
- Add a line lossy-compression guard. If line remaining has disjoint or multi-block segments, do not silently collapse it into one representative centerline.
- Centralize the temporary attack/tracking area behavior: preserve existing area assignment, including non-tracking UAVs, outside only the current attack patch.
- Separate planner failure fallback from policy preserve/skip in logs and metadata.
- Add a replay fixture for the 2026-06-07 area failure: three UAVs on one area input, one UAV tracking, non-tracking UAV assignments preserved.

## P1 TODO

- Introduce a `RemainingGeometry` model that carries mission type, geometry list, source plan, snapshot provenance, and lossy/simplification flags.
- Introduce a fuller `ResumePolicyDecision` model and move trigger-specific policy branches into one module.
- Implement area lost-UAV piece-only takeover for permanent aircraft loss cases.
- Preserve per-UAV area assignment provenance so a temporary tracking event does not turn into whole-area redistribution.
- Implement line segment-wise remaining planning for multi-segment residual geometry.
- Add current input confidence scoring from current waypoint, latest applied MissionPlan, mission progress, and snapshot timestamp.
- Build log-based replay tests for current remaining, attack, prior, post-attack, and reexecute-first line/area flows.

## P2 TODO

- Extend the area planner to support multi-polygon and hole-aware remaining geometry.
- Reduce representative centerline fallback to single-segment or explicitly verified collinear cases.
- Clean up duplicate/legacy remaining hybrid paths after the policy gate is stable.
- Document the boundary between `sweep_progress.json`, `mission_area_replan` snapshots, and live `0401` state.
- Expose `preserve`, `redivide`, `lost_uav_takeover_only`, and `skip_fail_closed` decisions in operator-facing diagnostics.

## Validation Scenarios

- Area, three UAVs, one target-tracking UAV: non-tracking UAVs keep existing area assignments.
- Area, permanent UAV loss: only the lost UAV's existing piece can be considered for takeover.
- Area, multi-polygon or hole remaining geometry: no first-polygon-only redivision without an explicit allow decision.
- Line, single residual segment: redivision remains available.
- Line, disjoint residual segments: no silent representative centerline compression.
- Stale or unmatched `currentWaypointID`: current input confidence fails and line/area redivision is blocked.
- Snapshot missing exact/carry-forward provenance: execution replan fails closed instead of using latest fallback.
- Entry coordinate missing for one expected UAV: target aircraft set mismatch is logged and policy decides preserve/skip instead of silently shrinking the split.

## Files To Review

- `modules/mission_planning/replanning/triggers/next_collab/pipeline.py`
  - `prepare_next_collab_input_replacements`
  - `_prepare_line_replacements`
  - `_prepare_area_replacements`
  - `_resolve_next_collab_target_aircraft_ids`
- `modules/mission_planning/replanning/triggers/remaining_hybrid/current.py`
  - `build_current_remaining_hybrid`
  - `merge_current_remaining_hybrid`
  - `filter_generic_flightpath_missions_for_hybrid`
- `modules/mission_planning/replanning/triggers/remaining_hybrid/current_replan.py`
  - `prepare_current_remaining_hybrid_replacements`
- `modules/mission_planning/replanning/triggers/remaining_hybrid/reexecute_first.py`
  - `prepare_reexecute_first_mission_replacements`
- `modules/mission_planning/replanning/triggers/prior/pipeline.py`
  - `_build_remaining_input_mission_for_collaborative_replan`
  - `_merge_line_remaining_detail`
  - `_merge_area_remaining_detail`
  - `_prepare_uav_collaborative_resume_replan`
- `modules/mission_planning/replanning/triggers/attack/pipeline.py`
  - `_attack_input_is_area`
  - `_prepare_uav_collaborative_resume_replan` call site
  - `preserved_area_assignment` branch
- `modules/mission_planning/replanning/triggers/post_attack/pipeline.py`
  - `prepare_next_collab_input_replacements` call sites
  - phased line rejoin path
- `modules/mission_planning/mission_planning_gui.py`
  - `_override_input_missions_with_remaining_snapshot`
  - `_snapshot_apply_whitelist_for_current_remaining_hybrid`
  - `_build_current_remaining_hybrid_request`
- `modules/common/mission_area_replan_store.py`
  - `load_snapshot_entry`
  - `_merge_with_existing_snapshot`
  - `carry_forward_snapshot`

## Non-Goals For P0

- Do not build a new whole-area redistribution algorithm.
- Do not add multi-polygon/hole planner support in the first stabilization pass.
- Do not finish line multi-segment planning in P0.
- Do not replace the whole `0401/currentWaypointID` inference system first.
- Do not make latest snapshot fallback smarter for execution replans. Fail closed first, then add proven exceptions later.

## Review Notes

This TODO was reviewed in three sub-agent discussion rounds on 2026-06-07:

- Round 1 mapped the current line/area resume flow and identified scattered current input, entry coordinate, snapshot, and sweep progress usage.
- Round 2 compared preserve vs redivide policy and recommended area preserve by default for temporary attack/tracking unavailable cases.
- Round 3 reviewed this TODO structure and added P0 requirements for current input confidence, target aircraft set contracts, snapshot provenance propagation, line lossy-compression guards, and non-goals.
