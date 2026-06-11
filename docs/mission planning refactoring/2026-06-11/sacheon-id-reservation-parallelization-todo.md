# Sacheon ID Reservation / Parallel Build TODO

Date: 2026-06-11
Scope: `modules_사천시연버전`

## Goal

Reduce attack and post-attack replanning latency without changing mission behavior, mission parameters, artifact schema, or ID semantics.

The main pattern is:

1. Reserve artifact IDs before build when they are part of filenames or cross references.
2. Build geometry-heavy FlightPath payloads without consuming waypoint IDs.
3. Reassign waypoint IDs after build in a deterministic serial order.
4. Validate that IDs, references, and `nextWaypointID` links are identical in structure and collision-free.

## Current ID Model

Central allocator:

- `modules_사천시연버전/modules/mission_planning/engine/mission_generation/id_allocation/allocator.py`
- Public APIs:
  - `next_mission_plan_id()`
  - `next_imp_id()`
  - `next_individual_mission_id()`
  - `next_path_id(aircraft_id)`
  - `next_waypoint_id()`
  - `reserve_mission_plan_ids(count)`
  - `reserve_imp_ids(count)`
  - `reserve_individual_mission_ids(count)`
  - `reserve_path_ids(aircraft_id, count)`
  - `reserve_path_id_blocks(count_by_aircraft)`
  - `reserve_replan_id_bundle(...)`
  - `reserve_waypoint_block(count)`
  - `reserve_waypoint_blocks(counts)`

Replan wrapper:

- `modules_사천시연버전/modules/mission_planning/runtime/ids/replan_reservation.py`
- `ReplanIdReservation.reserve(...)` wraps bundle reservation and exposes:
  - `next_plan()`
  - `next_imp()`
  - `next_individual()`
  - `next_path(aircraft_id)`
  - `next_waypoint()`

## ID Classification

Must be reserved before artifact build:

- `missionPlanID`: used as MissionPlan file name and result identity.
- `individualMissionPackageID`: used as IMP file name and `MissionPlan.aircraftList[*].individualMissionPackageID`.
- `individualMissionID`: referenced by FlightPath `individualMissionID` and IMP mission entries.
- `pathID`: used as FlightPath file name and IMP mission `pathID`.

Can usually be assigned after geometry build:

- `waypointID`: internal to waypoint list and `nextWaypointID` relink.
- `nextWaypointID`: derived from waypoint order after final IDs are known.

Exceptions:

- Any helper that embeds a waypoint ID into another artifact before final list relink must keep provider-driven assignment.
- LAH attack/hold single waypoint helpers in `attack/pipeline.py` should be audited before deferring IDs.

## Already Applied In Current Working Copy

These are code changes already made in this thread and should be tested by the user:

- `modules_사천시연버전/modules/mission_planning/pipelines/next_collab_path_builder.py`
  - Added `assign_waypoint_ids` option to `build_flight_path_from_planned_row`.
  - Added `assign_waypoint_ids` option to `build_formation_flight_path_from_template`.
  - Added scan-line precompute reuse for area/line sweep collection.

- `modules_사천시연버전/modules/mission_planning/replanning/triggers/next_collab/pipeline.py`
  - Added `_assign_replacement_waypoint_ids_in_order`.
  - LINE and AREA replacement FlightPath builds use `assign_waypoint_ids=False`.
  - LINE and AREA replacement path builds use at least 2 workers.
  - Waypoint IDs are assigned afterward in build item/path order.

- `modules_사천시연버전/modules/mission_planning/replanning/triggers/attack/pipeline.py`
  - Warm path loads attack assist and caches raster discovery.
  - Descriptor worker context avoids full `deepcopy(ctx)`.

- `modules_사천시연버전/modules/mission_planning/replanning/triggers/post_attack/pipeline.py`
  - Added run-cache entries for snapshot detail and line remaining detail.
  - Added read-only `copy_result=False` paths for cached IMP/FlightPath reads.
  - Avoids re-reading generated IMP when path IDs are already known and no follow-up scan is needed.

## Minimal Follow-Up TODO

### P0: Validate The Current NextCollab Deferred Waypoint-ID Patch

Files:

- `modules_사천시연버전/modules/mission_planning/replanning/triggers/next_collab/pipeline.py`
- `modules_사천시연버전/modules/mission_planning/pipelines/next_collab_path_builder.py`

Checks:

- Confirm generated FlightPath `waypointList[*].waypointID` is non-zero and unique.
- Confirm `nextWaypointID` links match the next waypoint's `waypointID`.
- Confirm `lahWaypointList`, if present, matches `waypointList` after ID assignment.
- Confirm logs show:
  - `replacement FlightPath build items=N workers=2`
  - `waypoint IDs assigned after parallel build`

Risk:

- Medium. Geometry output should remain the same, but numeric waypoint IDs can shift if previous parallel provider timing had already been non-deterministic. Treat path order as the intended deterministic order.

### P1: Move Post-Attack Single-ID Reservations To ReplanIdReservation

File:

- `modules_사천시연버전/modules/mission_planning/replanning/triggers/post_attack/pipeline.py`

Current hot spots:

- LAH resume update around `_reserve_individual_mission_ids(1)`, `_reserve_path_ids(...)`, `_reserve_imp_ids(1)`.
- Active phased line rejoin around single individual/path reservations.
- Active done follow-up around done marker/path/IMP reservations.
- Tracking return-only package around single individual/path/IMP reservations.

Plan:

- Import and use `ReplanIdReservation`.
- For each builder, compute required counts before mutation.
- Reserve once per update:
  - `imp_count=1`
  - `individual_count=<needed missions>`
  - `path_count_by_aircraft={aircraft_id: <needed paths>}`
  - `waypoint_count=<known fixed marker count only>`
- Pass `reservation.next_waypoint` into helpers that already accept waypoint providers.
- Preserve current artifact order when consuming IDs.

Risk:

- Medium. ID values can change compared with old interleaved single reservations, but references must remain valid. Do not change mission content.

### P1: Pass Waypoint Providers Through Prior Clone Calls Used By Attack/Post-Attack

File:

- `modules_사천시연버전/modules/mission_planning/replanning/triggers/prior/pipeline.py`

Current pattern:

- `_clone_follow_up_replan_artifacts(...)` can accept providers.
- Some call sites still omit provider and fall back to `reserve_waypoint_block(...)` inside lower-level helpers.

Plan:

- Audit call sites used by attack/post-attack.
- Ensure `individual_id_provider`, `path_id_provider`, and `waypoint_id_provider` are passed when the caller already has a reservation object.
- Keep call sites outside attack/post-attack unchanged unless directly reused by those flows.

Risk:

- Low to medium. This mainly removes global allocator calls during clone loops.

### P2: Add A Small Common Helper For Deferred Waypoint Assignment

Candidate file:

- `modules_사천시연버전/modules/mission_planning/pipelines/mission_path_trim.py`

Possible helper:

```python
def reassign_flight_path_waypoints_in_order(payloads, waypoint_id_provider):
    ...
```

Plan:

- Reuse existing `reassign_unique_waypoint_ids_inplace`.
- Ensure `waypointList` and optional `lahWaypointList` stay in sync.
- Use only from replacement/replan build paths after local validation.

Risk:

- Low if helper is additive and current call sites remain unchanged.

### P2: Keep Initial Planning As Reference Pattern, Not Immediate Target

Files:

- `modules_사천시연버전/modules/mission_planning/runtime/aircraft_parallel_0303.py`
- `modules_사천시연버전/modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py`
- `modules_사천시연버전/modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0304.py`

Observation:

- Initial planning already has examples of parallel geometry build followed by waypoint ID reassignment.
- Do not refactor this path for the current attack/post-attack optimization unless validation shows ID collisions or allocator waits.

Risk:

- High blast radius if changed now. Use as reference only.

### P3: Avoid Old AnS ID Refactor For Now

File:

- `modules_사천시연버전/modules/mission_planning/MissionPlanner/AnS/mission_pipeline.py`

Observation:

- Contains older loop-local `reserve_*` calls.
- It is not the current attack/post-attack hot path.

Risk:

- High. Skip for this optimization round.

## Validation Plan

Static validation:

```powershell
python -m py_compile `
  "modules_사천시연버전\modules\mission_planning\pipelines\next_collab_path_builder.py" `
  "modules_사천시연버전\modules\mission_planning\replanning\triggers\next_collab\pipeline.py" `
  "modules_사천시연버전\modules\mission_planning\replanning\triggers\attack\pipeline.py" `
  "modules_사천시연버전\modules\mission_planning\replanning\triggers\post_attack\pipeline.py" `
  "modules_사천시연버전\modules\mission_planning\replanning\triggers\prior\pipeline.py"
```

Runtime validation:

- Run one attack plan.
- Run one post-attack rejoin.
- Compare logs before/after:
  - attack total
  - post-attack total
  - `replacement FlightPath build`
  - `waypoint IDs assigned after parallel build`
  - `areaCollectRowsMs`
  - `areaBuildMax`
  - `group_evaluation`

Artifact integrity checks:

- For every generated FlightPath:
  - file name stem equals payload `pathID`.
  - all waypoint IDs are positive and unique within generated artifacts.
  - `nextWaypointID` points to the next waypoint ID, last is `0`.
  - `lahWaypointList` equals `waypointList` when both are present.
- For every generated IMP:
  - file name stem equals `individualMissionPackageID`.
  - every mission `pathID` exists in generated or existing FlightPath DB.
  - every mission `individualMissionID` is positive and unique within generated artifacts.
- For generated MissionPlan:
  - `aircraftList[*].individualMissionPackageID` points to generated or existing IMP.

Performance acceptance:

- If replacement path count is at least 2, expect `workers=2` in logs.
- If path count is 1, no material parallel speedup is expected.
- A useful win is:
  - latest fast attack/post-attack: 200-700 ms reduction.
  - area-heavy cases: 500 ms to 1.2 s reduction.

Rollback criteria:

- Any missing FlightPath/IMP cross-reference.
- Duplicate waypoint IDs across generated current artifacts.
- Broken `nextWaypointID` chain.
- Runtime exception in `prepare_next_collab_input_replacements`.
- Any mission content difference beyond ID values and timestamp/log fields.

## Notes From Sub-Agent Review

- Central allocator and `ReplanIdReservation` are already suitable for local block reservation.
- `pathID`/IMP/individual IDs must stay pre-reserved.
- `waypointID` is the main candidate for deferred deterministic assignment.
- Post-attack is the next highest-value scope after NextCollab replacement.
- Old `AnS/mission_pipeline.py` should not be included in the minimal change set.
