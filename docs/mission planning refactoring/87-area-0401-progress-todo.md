# Area 0401 Progress Stabilization TODO

Date: 2026-06-08

## Purpose

Apply the line-progress stabilization pattern to area missions without changing unrelated replanning behavior.

The line fix worked because it stopped treating unstable footprint geometry as the primary source of truth. Instead, it uses `0401` `currentWaypointID` plus sensor center position to infer the exact `lineSearch` point progress, then applies that progress monotonically to the remaining snapshot.

Area missions need the same principle:

- Use planned sweep order and live `0401` position as the primary progress signal.
- Treat footprint polygons as supporting evidence, not the authority that can resurrect completed work.
- Once an area sweep row/frontier is considered passed, do not let later noisy footprint samples reopen it.
- Keep the first implementation narrow enough to preserve current attack, prior, post-attack, and current-remaining behavior.

## Current Risk

- Area remaining currently depends heavily on accumulated footprint/covered geometry.
- Footprint shape can be wrong or inconsistent because of UAV motion, sensor behavior, heading, altitude, or simulation timing.
- A noisy footprint can leave holes, disconnected islands, or re-open already passed regions.
- Area redivision is more dangerous than line redivision because a bad remaining polygon can send UAVs back into old work or redistribute ownership unexpectedly.
- Existing area planner paths may only consume a simplified polygon, so feeding fragmented remaining geometry can be worse than preserving the old assignment.

## Target Contract

For area missions, progress must be driven by a monotonic sweep frontier.

Minimum contract:

- Resolve the active individual mission from the current plan and `0401` state.
- Match `currentWaypointID` to a waypoint in the active flight path.
- If that waypoint has `filmingProperty.lineSearch.coordinateList`, locate the nearest coordinate to the `0401` sensor center.
- Convert the matched point to a cumulative sweep point count, the same way line missions now do.
- Convert cumulative sweep progress into an area sweep row/frontier index.
- Mark all sweep rows before that frontier as completed.
- Never decrease that completed frontier during the same mission/plan.
- Use footprint geometry only for the current row or for visual/diagnostic refinement.

## P0 TODO

- Add an area-specific progress application helper beside the current line helper in `mission_progress_area_management_tab.py`.
- Reuse `sweep_progress_points` and `sweep_point_count` from `MissionProgressTracker` instead of adding a separate progress source first.
- Map `sweep_progress_points / sweep_point_count` to `planned_cut_lines` for area missions.
- Apply the mapped boundary monotonically to `_MissionAreaState.progress_boundary_line_index`.
- Mark rows up to the boundary as completed and rebuild covered geometry from planned sweep strips.
- Keep current-row partial coverage conservative. Do not mark the current row done only because the footprint overlaps it once.
- If `currentWaypointID` does not match the active plan/path, do not advance the area frontier.
- If `sweep_point_count` is missing or zero, leave existing footprint-based behavior unchanged.
- Preserve existing non-tracking UAV area assignments during temporary attack/tracking events.
- Add logs or debug fields showing `areaProgressSource`, `sweepProgressPoints`, `sweepPointCount`, `mappedBoundaryLineIndex`, and `currentWaypointID`.

## P0 Implementation Status

Completed on 2026-06-08:

- Added `_apply_sweep_point_progress_to_area_state` in `mission_progress_area_management_tab.py`.
- Reused `MissionProgressTracker` `sweep_progress_points` and `sweep_point_count`.
- Mapped area sweep point progress to `planned_cut_lines` boundary indexes.
- Applied the mapped boundary monotonically through `_MissionAreaState.progress_boundary_line_index`.
- Kept `area_progress_*` provenance/debug fields monotonic as well; lower 0401 samples no longer overwrite the last accepted mapped boundary.
- Rebuilt completed area coverage from planned sweep strips when the boundary advances.
- Required a valid `currentWaypointID` before advancing the area frontier.
- Added a helper-side sweep waypoint membership guard so an area frontier cannot advance when `0401 currentWaypointID` is not one of the active mission's line-search waypoints.
- Left existing footprint behavior unchanged when sweep point progress is unavailable.
- Added area progress debug fields in `mission_area_replan` snapshots:
  - `areaProgressSource`
  - `currentWaypointID`
  - `sweepProgressPoints`
  - `sweepPointCount`
  - `mappedBoundaryLineIndex`
  - `areaProgressDetails`
- Added a next-collab area guard that initially failed closed instead of sending only the first polygon when remaining area had multiple outer polygons or holes. This was later replaced by P2 component decomposition support.
- Added `smoke_area_0401_progress.py` for area progress mapping, component decomposition checks, and legacy unsafe-partial-plan fail-closed checks.

## P0 Fail-Closed Rules

- Do not feed a fragmented multi-polygon remaining area into the division planner by taking only the first polygon.
- Do not redivide area on latest-snapshot fallback unless the caller explicitly marks it as non-execution/manual.
- Do not shrink the target UAV set silently when one active UAV has no entry coordinate.
- Do not let footprint-only coverage move the area frontier backward.
- Do not treat a single noisy footprint sample as proof that a skipped future row is complete.

## P1 TODO

- Store area progress provenance in `mission_area_replan` snapshots:
  - `progressSource`
  - `sourceMissionPlanID`
  - `pathID`
  - `currentWaypointID`
  - `sweepProgressPoints`
  - `sweepPointCount`
  - `mappedBoundaryLineIndex`
  - `confidence`
- Add replay coverage for area current-remaining, attack, prior, post-attack, and reexecute-first flows.
- Add an area ownership/provenance model so temporary tracking does not become full-area redistribution.
- Add a permanent-lost-UAV policy that only offers takeover of the lost UAV's remaining piece.
- Separate display coverage, snapshot remaining geometry, and replan input geometry in diagnostics.

## P1 Implementation Status

Partially completed on 2026-06-08:

- Added area progress provenance fields to `mission_area_replan` snapshots.
- Added per-aircraft `areaProgressDetails` and `areaOwnershipDetails`.
- Attached each owner state's own `remainingDetail` to `areaOwnershipDetails`.
- Added `geometryDiagnostics` that separates:
  - display coverage source
  - snapshot remaining geometry
  - replan input geometry
- Added `piece_only_takeover` metadata for area ownership snapshots.
- Updated collaborative remaining input generation so area replanning with unavailable UAVs uses only the unavailable UAVs' owner `remainingDetail`.
- Fail-closed area collaborative replanning when unavailable UAV owner geometry is missing instead of falling back to the full remaining area.
- Connected the piece-only policy through prior, attack collaborative resume, and post-attack collaborative update paths that share `_prepare_uav_collaborative_resume_replan`.
- Extended `smoke_area_0401_progress.py` with synthetic owner-piece checks for:
  - selecting only the unavailable UAV's remaining area
  - preserving the next input mission
  - skipping when the unavailable UAV has no owner geometry
- Updated next-collab area replacement insertion so `piece_only_takeover` preserves each remaining UAV's existing current area mission and inserts takeover-piece missions after it instead of replacing the original assignment.
- Added operator-facing decision diagnostics for:
  - `preserved_assignment`
  - `monotonic_progress_trim`
  - `planner_redivision`
  - `fail_closed_skip`
- Added source-contract smoke checks that keep the area piece-only policy connected through current-remaining, attack, prior, post-attack, and reexecute-first entry points.

Still pending:

- Add replay coverage for real captured area current-remaining, attack, prior, post-attack, and reexecute-first flows. Synthetic/source-contract coverage exists, but it is not a replacement for captured replay.
- Add full production replay for multi-polygon/hole-aware area planning.

## P2 TODO

- Add multi-polygon and hole-aware area planning support.
- Add segment/row-aware area remaining payloads instead of reducing everything to a single polygon.
- Add operator-facing diagnostics that distinguish:
  - preserved assignment
  - monotonic progress trim
  - planner redivision
  - fail-closed skip
- Consider replacing area footprint accumulation with a sweep-row model as the default once replay tests are stable.

## P2 Implementation Status

Partially completed on 2026-06-09:

- Added next-collab area component decomposition for remaining `areaList`.
- Multi-polygon remaining areas are split into independent planner components instead of taking only the first polygon.
- Hole-bearing remaining areas are decomposed into hole-free polygon components before planner execution.
- Added hole-cut decomposition before triangulation so many-hole area geometry can often be opened into a small number of hole-free planner components instead of immediately exceeding the triangulation component cap.
- `_prepare_area_replacements` now runs the existing area division planner per component and aggregates:
  - replacement missions
  - generated flight paths
  - planner workflow text
  - review diagnostics
- Added `areaPlannerComponentCount` and `areaPlannerComponents` to `areaReview`.
- Added `componentDecomposition` to `areaReview.areaPlannerComponents` so review/audit can distinguish direct, hole-cut, and triangulated-hole components.
- Updated snapshot diagnostics so multi-polygon/hole remaining geometry reports `area_component_decomposition_multi_polygon_or_hole` instead of `area_fail_closed_multi_polygon_or_hole`.
- Added `areaSegmentList` / `areaSegmentPolicy` to area remaining snapshots, based on uncompleted planned sweep rows clipped to the current remaining geometry.
- Updated next-collab area component generation so `areaSegmentList` is preferred over polygon decomposition when present.
- Preserved `areaSegmentList` through piece-only takeover owner remaining-detail merging.
- Added direct operator-facing UI surfacing in the monitoring area tab:
  - selected area mission diagnostics table
  - grouped `inputMissionID` snapshot lookup
  - `replanInputGeometry`
  - row segment count
  - outer/hole counts
  - 0401 progress detail count
  - ownership count
  - operator decision categories/reasons
- Extended `smoke_area_0401_progress.py` to verify:
  - mission-area snapshot store field summary for `snapshot_saved` audit rows
  - mission-area audit persistence for `snapshot_saved`, `snapshot_carried_forward`, `snapshot_entry_preserved`, `snapshot_entry_exact`, and `snapshot_entry_latest_fallback` using a temporary DB root
  - multi-polygon component extraction
  - hole decomposition
  - planned sweep-row segment extraction and planner component conversion
  - lower progress samples preserving both the committed frontier and the exported progress provenance
  - wrong/stale `currentWaypointID` and missing sweep count fail-closed cases for area progress mapping
  - `_prepare_area_replacements` planner-call aggregation via monkeypatched planner
  - captured multi-polygon area snapshots from `Logs` replay through component decomposition
  - captured new-field audit for `areaProgressDetails`, `areaOwnershipDetails`, `geometryDiagnostics`, and `areaSegmentList`
  - complex hole geometry that used to exceed triangulation cap now decomposes through hole-cut components below cap
  - excessive row segment component count still fails closed
  - legacy `remaining_hybrid/general.py` still fails closed instead of doing unsafe partial area planning
  - monitoring tab source-contract exposure for the operator diagnostics table
  - replay fixture collector availability and strict mode
- Added `AREA_PLANNER_COMPONENT_MAX_COUNT` to prevent complex hole triangulation from creating excessive planner components.
- Added explicit next-collab area component failure summaries so missing geometry, segment component cap, and complex multi/hole extraction failures are visible in logs.
- Added `snapshot_saved` audit summaries to `mission_area_snapshot_audit.jsonl` so future live logs show counts for:
  - area missions
  - `areaProgressDetails`
  - `areaOwnershipDetails`
  - `areaSegmentList`
  - `geometryDiagnostics`
  - `replanInputGeometry`
  - `areaSegmentPolicy`
- Added strict area readiness summaries to `snapshot_carried_forward` and `snapshot_entry_preserved` audit rows so carry-forward and anti-resurrection preservation events are visible in replay reports.
- Tightened mission-area snapshot audit readiness so `areaEntryNewFieldReady` / `areaSnapshotNewFieldReady` require:
  - all area progress provenance keys (`progressSource`, `sourceMissionPlanID`, `pathID`, `currentWaypointID`, `sweepProgressPoints`, `sweepPointCount`, `mappedBoundaryLineIndex`, `confidence`)
  - valid area ownership/provenance keys (`aircraftID`, `individualMissionID`, `inputMissionID`, `sourceMissionPlanID`, `pathID`, `takeoverPolicy`, `remainingDetail`)
  - owner `takeoverPolicy` to be `piece_only` and owner `remainingDetail` to contain takeover geometry
  - valid planned-sweep row segment identifiers (`source`, `lineIndex`, `aircraftID`, `individualMissionID`, `inputMissionID`, `areaM2`, `coordinateList`)
  - every area mission in the saved snapshot to be ready, not only one representative area mission
- Kept DONE area readiness strict for progress/ownership/diagnostics, but allowed completed area entries to have no `areaSegmentList` and no owner takeover geometry when `isDone=true`, `remainingAreaM2` is effectively zero, and no remaining geometry exists.
- Added `areaReadinessSchemaVersion=2` to area snapshot audit summaries so the replay collector does not treat older lenient ready rows as strict replay evidence.
- Added `snapshot_entry_exact` and `snapshot_entry_latest_fallback` audit summaries for area snapshot reads so attack/prior/post-attack/current-remaining flows show whether the loaded entry had:
  - `areaEntryNewFieldReady`
  - `areaEntryMissingNewFieldCategories`
  - `replanInputGeometry`
  - `areaSegmentPolicy`
- Added `auditContext` to area snapshot read audit rows for current GUI application, prior collaborative resume, and post-attack snapshot reads.
- Added an execution guard so area snapshot geometry is used for replanning only when the entry passes the strict schema v2 readiness check and comes from the exact requested mission plan; unready or latest-fallback area snapshots now emit `snapshot_entry_rejected_unready` and fail closed instead of feeding old footprint-derived geometry back into current-remaining, prior/collaborative resume, or post-attack area checks.
- Preserved `areaSegmentList` / `areaSegmentPolicy` when mission-planning GUI applies a ready area remaining snapshot to an input mission, and taught post-attack area row extraction to consume segment-only remaining details.
- Tightened snapshot merge anti-resurrection: a strict-ready DONE area snapshot is preserved even if a later sample reports remaining area again; legacy/unverified DONE snapshots can be reopened only by a strict-ready incoming area snapshot with real remaining geometry.
- Split mission-planning GUI snapshot apply `auditContext` values so current-remaining and reexecute-first snapshot application can be distinguished when those paths are captured:
  - `mission_planning_gui_current_remaining_snapshot_apply`
  - `mission_planning_gui_reexecute_first_snapshot_apply`
- Split shared collaborative-resume `auditContext` values so future live logs can distinguish:
  - prior collaborative resume
  - attack collaborative resume
  - post-attack collaborative resume
  - post-attack active-only remaining update
- Added direct exact-map access auditing for the mission-planning GUI path that applies already-loaded snapshot entries without going through latest fallback.
- Added `collect_area_0401_replay_fixture.py` to scan `Logs/**/mission_area_snapshot_*.json` plus `mission_area_snapshot_audit.jsonl` and produce ready/partial replay candidate reports once live logs contain the new area snapshot fields.
- Extended the replay fixture collector with flow-group coverage so captured logs report which area paths have ready audit rows:
  - current remaining snapshot application
  - reexecute-first snapshot application
  - prior collaborative resume
  - attack collaborative resume
  - post-attack collaborative resume
  - post-attack snapshot reads

Still pending:

- Real captured replay fixtures with the new 0401-derived area progress/ownership fields for area current-remaining, attack, prior, post-attack, and reexecute-first.
- Captured replay fixtures that include the new `areaSegmentList` payload from a live monitoring run. The smoke now audits for these fields and can be made strict with `AREA_0401_REQUIRE_CAPTURED_NEW_FIELDS=1`; the fixture collector can emit a candidate JSON report with `--write` and can enforce flow coverage with `--strict --require-flow-contexts`.
- Adversarial complex-hole geometries that cannot be opened by the hole-cut decomposition without exceeding the component cap still fail closed until a real captured example justifies a more specific planner policy.

## Validation Scenarios

- Area, normal sweep: `0401` moves through rows, `sweep_progress_points` increases, completed frontier only increases.
- Area, noisy footprint: footprint jumps or shrinks, completed rows do not resurrect.
- Area, current row partial progress: previous rows are removed, current row remains conservative.
- Area, attack temporary tracking: tracking UAV leaves, non-tracking UAVs keep existing assignments unless policy allows takeover.
- Area, permanent UAV loss: only the lost UAV's own remaining piece is considered for takeover.
- Area, stale `currentWaypointID`: area frontier does not advance and redivision fails closed.
- Area, multi-polygon remaining: no first-polygon-only planner input without an explicit policy decision.
- Area, post-attack return: already completed rows stay completed after carry-forward snapshot.

## Files To Review

- `modules/monitoring/logic/mission_progress.py`
  - `_update_sweep_point_progress`
  - `sweep_progress_points`
  - `sweep_point_count`
- `modules/monitoring/gui/tabs/mission_progress_area_management_tab.py`
  - `_apply_sweep_point_progress_to_line_state`
  - `_MissionAreaState`
  - `_build_planned_sweep_lines`
  - `_rebuild_completed_sweep_coverage`
  - `_build_state_remaining_detail`
  - `_build_group_remaining_detail`
- `modules/monitoring/gui/tabs/monitoring_visualization_tab.py`
  - `_log_sweep_progress`
  - `sweep_progress.json` persistence
- `modules/monitoring/logic/mission_update.py`
  - `sweep_line_coordinate_lists`
  - `sweep_point_count`
  - waypoint `line_search_point_count`
- `modules/common/mission_area_replan_store.py`
  - snapshot save, merge, and carry-forward behavior
- `modules/mission_planning/replanning/triggers/next_collab/pipeline.py`
  - `_prepare_area_replacements`
  - current remaining replacement generation
- `modules/mission_planning/replanning/triggers/prior/pipeline.py`
  - `_build_remaining_input_mission_for_collaborative_replan`
  - `_merge_area_remaining_detail`
  - `_prepare_uav_collaborative_resume_replan`
- `modules/mission_planning/replanning/triggers/attack/pipeline.py`
  - area preserve branch
  - collaborative resume call sites
- `modules/mission_planning/replanning/triggers/post_attack/pipeline.py`
  - area/current remaining and rejoin call sites

## Non-Goals For P0

- Do not build a new area division planner.
- Do not add full multi-polygon or hole-aware planning in the first pass.
- Do not replace all footprint visualization logic.
- Do not change line behavior that was just stabilized.
- Do not make whole-area redistribution more aggressive.
- Do not use time-only progress as the area authority when `0401` point progress is available.

## Implementation Order

1. Confirm area missions expose usable `sweep_line_coordinate_lists` and `sweep_point_count` in `MissionProgressTracker`.
2. Add an area helper that maps `sweep_progress_points` to planned sweep rows.
3. Apply the helper before footprint-based area updates in `update_agent_status`.
4. Rebuild area covered geometry from completed planned rows.
5. Persist remaining snapshot and verify it does not resurrect completed rows.
6. Add replay or smoke coverage using a captured area scenario before broadening replan policy.
