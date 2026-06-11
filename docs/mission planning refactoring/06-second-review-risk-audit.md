# Second Review Risk Audit

이 문서는 `modules/mission_planning` 리팩토링 계획을 한 번 더 검토하면서 발견한 누락 계약과 기능 변화 위험을 정리한다.

현재 코드 자체는 변경하지 않았다. 따라서 지금 이 문서 추가만으로 현재 기능 변화는 없다. 다만 아래 항목을 문서와 테스트에 반영하지 않은 채 실제 move/rename/delete를 시작하면 기능 변화 가능성이 높다.

## 결론

- 기존 리팩토링 방향은 맞다.
- 그러나 "기능 변화 없음"을 보장하려면 단순 import smoke와 함수 signature snapshot만으로 부족하다.
- 특히 hot reload, 0902 normalization, delivery state machine, ID allocator reset, resource path, manual entrypoint를 별도 계약으로 고정해야 한다.
- 실제 코드 이동 전 Phase 0 범위를 확장해야 한다.

## 추가로 고정해야 하는 계약

### 1. Launcher/control contract

보존해야 할 항목:

- `modules/mission_planning/mission_planning_gui.py` basename
- `run.py`의 mission alias, role mapping, process cleanup key
- mission role control port 기본값 `45981`
- `KU_CTRL_PORT`, `KU_START_HIDDEN`, `KU_HIDE_ON_CLOSE` 동작
- hidden relaunch/fallback script discovery
- `app/ui/main_window.py`의 mission planning launch candidate list

기능 변화 위험:

- public launcher 파일명을 바꾸면 dashboard/app launcher가 mission planning을 못 띄울 수 있다.
- role alias 또는 cleanup key가 바뀌면 기존 프로세스 종료/재시작 동작이 달라질 수 있다.

보강 TODO:

- [ ] launcher alias snapshot 작성
- [ ] `run.py` mission role launch smoke 작성
- [ ] `app/ui/main_window.py` launch candidate smoke 작성

### 2. Planner hot-reload contract

현재 `mission_planning_gui.py`는 `_PLANNER_RUNTIME_WATCH_RELATIVE_PATHS`로 특정 파일의 mtime/size signature를 추적한다. 이후 정해진 순서로 module reload를 수행하고, reload된 함수들을 global binding에 다시 주입한다.

보존해야 할 항목:

- watched path 목록
- reload order
- reload 후 갱신되는 global bindings
- `runtime_force_reload` 판단 기준
- warmed GUI에서 stale helper가 남지 않는 동작

기능 변화 위험:

- 파일을 이동했는데 watch list가 바뀌지 않으면 GUI가 예전 helper를 계속 쓸 수 있다.
- pipeline module을 나눴는데 global rebinding 목록이 빠지면 runtime reload가 조용히 실패할 수 있다.

보강 TODO:

- [ ] `_PLANNER_RUNTIME_WATCH_RELATIVE_PATHS` inventory 작성
- [ ] reload order와 global binding 목록 문서화
- [ ] watched file touch 후 `runtime_force_reload=True` 확인 smoke 작성
- [ ] reload 전후 function identity가 바뀌는지 확인

### 3. Engine relocation invariants

`MissionPlanner/AnS`, `MissionPlanner/data_def`, `MissionPlanner/planning_enhanced` 이동은 가장 위험하다.

보존해야 할 항목:

- `sys.path` injection order
- bare import compatibility: `AnS`, `data_def`, `config`, `UAV_missionPlanning`
- `__file__` 기준 resource lookup
- `MissionPlanner/AnS/DEM.jpg`
- `portable_mission_bundle/models/latest_model.zip`
- `portable_mission_bundle/models/model_config.json`
- `portable_mission_bundle/portable_mission/**`
- root DEM `.tif` 탐색
- `MissionPlanner/config.py`
- ID tracker files under `MissionPlanner/data_def`

기능 변화 위험:

- resource path가 바뀌면 DEM, RL model, FOV/runtime config를 못 찾을 수 있다.
- bare import가 깨지면 GUI 실행 중에만 실패하는 문제가 생긴다.

보강 TODO:

- [ ] engine resource manifest 작성
- [ ] `__file__`-relative path snapshot 작성
- [ ] bare import smoke: `import AnS`, `from data_def import d0302, d0303, d0304`
- [ ] relocated path에서도 동일 resolver 결과가 나오는지 확인

### 4. 0902 normalization contract

현재 0902 수신은 매우 관대하게 처리된다.

보존해야 할 입력 alias:

- raw bytes/string/dict payload
- `optionList`
- `pendingOptionList`
- `missionPlanIDList`
- `missionPlanIDs`
- `replanDetail.missionPlanID`
- `replanRequest`
- `replanReason`
- `replanLevel`
- `triggerType`
- staged context fallback
- latest 0201/0203 fallback

추가로 보존해야 할 동작:

- 0902 replay capture
- running pipeline 중 deferred 0902 queue
- delay scheduling
- power-off guard
- quality-speed request는 option names를 비우고 force-direct로 전환

기능 변화 위험:

- dispatcher를 깨끗하게 만들면서 alias normalization을 줄이면 monitoring에서 보내는 기존 payload가 일부 무시될 수 있다.
- deferred queue semantics가 바뀌면 빠른 연속 0902 처리 순서가 달라진다.

보강 TODO:

- [ ] normalized 0902 context schema 작성
- [ ] alias별 fixture 작성
- [ ] deferred queue ordering test 작성
- [ ] power-off guard test 작성

### 5. Replan dispatcher priority

dispatcher를 추출할 때 현재 우선순위와 "handled but skipped" 의미를 유지해야 한다.

보존해야 할 우선순위:

1. attack option preprocessing/splitting
2. post-attack rejoin
3. prior post-rejoin
4. next collaborative mission
5. imaging schedule / quality speed
6. path deviation
7. prior mission
8. general/current remaining fallback

특수 trigger:

- `nextCollaborativeMission`
- `pathDeviation`
- `imagingScheduleDeviation`
- `qualityMonitorSep`
- `attackClosedDestroyed`
- `priorClosedResume`
- `inputRefresh`
- `collabReexecuteInputRefresh`
- `forcedCommand`
- `unexpectedRTB`
- RTB/fuel/health/payload fault derived reasons

기능 변화 위험:

- handler가 "handled=True, skipped"를 반환하는 흐름에서 fallback을 계속 태우면 현재와 다른 plan이 생성될 수 있다.
- quality speed는 0901 option을 만들면 안 된다.

보강 TODO:

- [ ] dispatcher priority table 작성
- [ ] handled/skipped/fallback semantics 문서화
- [ ] trigger별 0902 fixture 작성

### 6. Delivery state machine

단순히 `0301 -> 0305 -> 0901/0903`이 아니다.

보존해야 할 항목:

- queued 0301 plan IDs
- 0305 status=1 pipeline start notification
- 0301 송신 성공 후 0305 status=2 completion
- post-0301 readiness
- optional 0101 mode readiness
- grace/fallback timers
- option flow: 0901
- force-direct flow: 0903
- optional 0702 fallback
- `suppress_0702_fallback`
- attack delivery suppress flag
- quality-speed: 0901 blocked, 0702 suppressed
- next-collab 0305 reason underscore prefix

기능 변화 위험:

- 타이머와 readiness 조건이 조금만 달라져도 0901/0903 전송 시점이나 0702 fallback 여부가 달라진다.

보강 TODO:

- [ ] delivery matrix 작성
- [ ] fake `push_message` 기반 message order test 작성
- [ ] quality-speed no-0901 test 작성
- [ ] force-direct 0903/0702 suppression test 작성

### 7. Pipeline result shape contract

함수 signature뿐 아니라 반환 객체의 field shape도 계약이다.

보존해야 할 대표 field:

- `plan_ids`
- `option_names`
- `plan_meta_map`
- generated ID sets
- preserved ID sets
- `new_input_package_id`
- `status`
- `summary`
- `log_path`
- validation summary
- direct delivery flags

기능 변화 위험:

- pipeline을 이동하며 dataclass field 이름을 바꾸면 `MainWindow`가 결과를 잘못 해석한다.

보강 TODO:

- [ ] pipeline result dataclass inventory 작성
- [ ] required fields snapshot 작성
- [ ] result shape smoke 작성

### 8. ID allocator and launch reset contract

ID allocator 계약은 public reserve API만이 아니다.

보존해야 할 항목:

- `run.py` cold-start reset behavior
- `MissionPlanner/data_def/id_tracker.json`
- `MissionPlanner/data_def/id_tracker_0202.json`
- `MissionPlanner/data_def/_id_counters.json`
- `id_allocator._state`
- `id_allocator._volatile_counters`
- active DB scoped `DSS_Internal/id_tracker.json`
- `DSS_Internal/path_usage.json`
- `DSS_Internal/waypoint_usage.json`
- artifact directory scan behavior
- file lock/retry behavior

기능 변화 위험:

- ID reset이나 active DB scoping이 바뀌면 MissionPlanID, IndividualMissionPlanID, FlightPath pathID, waypointID band가 달라질 수 있다.

보강 TODO:

- [ ] cold reset parity test 작성
- [ ] concurrent reserve test 작성
- [ ] active DB switch test 작성
- [ ] path/waypoint usage high-water test 작성

### 9. Runtime artifact and resource paths

보존해야 할 path:

- `DSS_Internal/latest_0401_agent_status.json`
- `DSS_Internal/agent_status_0401.jsonl`
- `DSS_Internal/next_collab_replan/*`
- `DSS_Internal/NextCollab_*.json`
- `DSS_Internal/replan_inputs/*`
- `DSS_Internal/replan_request_transport/*`
- `DSS_Internal/mission_area_replan/*`
- `resource/korea.mbtiles`
- `resource/db/fov_db.csv`
- `nFusionSettings.json`
- `modules/common/msg_files/MessageLibrary`
- nFusion DLL/message library paths

기능 변화 위험:

- artifact filename/prefix가 바뀌면 monitoring/common/sim이 최신 상태나 detail payload를 못 찾는다.

보강 TODO:

- [ ] runtime artifact manifest 작성
- [ ] resource resolver 추가 전까지 resource move 금지
- [ ] artifact path existence smoke 작성

### 10. Manual/operator entrypoints

삭제/이동 전 owner 판단이 필요한 entrypoint:

- `modules/mission_planning/next_area_mode/main.py`
- `modules/mission_planning/MissionVisualizer/main_visualizer.py`
- `modules/mission_planning/MissionPlanner/main_MP.py`
- `modules/mission_planning/MissionPlanner/tools/test_div_area.py`
- `modules/mission_planning/MissionPlanner/tools/*`
- `modules/mission_planning/MissionPlanner/portable_mission_bundle/run_portable.bat`
- `modules/data/replan_rules_notes.txt`에 적힌 `MissionPlanner/data_def/id_allocator.py` 경로

기능 변화 위험:

- 운영자가 직접 실행하는 도구는 자동 import 검색만으로 잡히지 않는다.

보강 TODO:

- [ ] manual entrypoint inventory 작성
- [ ] 유지/폐기 owner decision 기록
- [ ] 유지 대상은 wrapper 제공

## 수정된 판단

삭제 후보 중 `duplicate Dubins_Path.py under UAV pattern tools` 표현은 너무 넓다. `MissionPlanner/tools/UAV_pattern/Nadir_BF/Dubins_Path.py`는 active chain에서 쓰일 수 있으므로 삭제 후보에서 제외한다.

삭제 금지 또는 보류 목록에 추가해야 할 항목:

- `modules/mission_planning/id_relationship_tab.py`
- `modules/mission_planning/MissionVisualizer/main_visualizer.py`
- `modules/mission_planning/planners/next_collab_division/**`
- `modules/mission_planning/MissionPlanner/config.py`
- `modules/mission_planning/MissionPlanner/tools/UAV_pattern/Nadir_BF/**`
- `modules/mission_planning/MissionPlanner/portable_mission_bundle/portable_mission/**`
- `modules/mission_planning/MissionPlanner/portable_mission_bundle/models/model_config.json`
- `modules/mission_planning/MissionPlanner/portable_mission_bundle/models/latest_model.zip`

## 다음 계획 보강

Phase 0은 다음 산출물을 먼저 만들도록 확장해야 한다.

- contract inventory
- fixture set
- import smoke
- launcher smoke
- hot reload smoke
- delivery order smoke
- pipeline result shape smoke
- ID allocator parity smoke
- resource/artifact path manifest
