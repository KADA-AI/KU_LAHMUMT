# Current Architecture

## 현재 파일 규모

`modules/mission_planning`에는 Python 파일 약 251개가 있다. 주요 분포는 다음과 같다.

| 영역 | Python 파일 수 | 판단 |
| --- | ---: | --- |
| `MissionPlanner/` | 106 | 기존 핵심 계획 엔진, data_def, planning_enhanced, tools, portable bundle이 혼재 |
| `legacy/` | 60 | wrappers, archived apps/tests/docs/static leftovers |
| `runtime/` | 19 | 상태, JSON I/O, validation, cache, logging, next-collab runner |
| `pipelines/` | 16 | attack/prior/next-collab/path-deviation/imaging/post-attack 등 재계획 실행 |
| `planners/` | 10 | next-collab division planner GUI/headless 의존 대상 |
| `ui/` | 7 | active PyQt tab/widget |
| `logic_test/`, `next_area_mode`, `MissionVisualizer` | 기타 | manual tools 또는 active helper가 섞여 있음 |

대형 파일:

- `mission_planning_gui.py`: 약 14,515 lines
- `MissionPlanner/data_def/d0303.py`: 약 8,346 lines
- `planners/next_collab_division/_planner_window.py`: 약 7,440 lines
- `pipelines/attack_plan_pipeline.py`: 약 7,255 lines
- `pipelines/post_attack_rejoin_pipeline.py`: 약 4,734 lines
- `pipelines/prior_mission_pipeline_impl.py`: 약 4,675 lines
- `pipelines/next_collab_replan_pipeline_impl.py`: 약 3,086 lines

## 기능 경계

### 1. 전체 임무 계획 관리

현재 중심은 `mission_planning_gui.py`의 `MainWindow`다.

주요 책임:

- GUI bootstrapping 및 nFusion 환경 구성
- 0101/0102/0201/0203/0301/0305/0702/0803/0901/0902/0903 메시지 흐름
- 초기 임무 계획 실행
- 재계획 요청 staging 및 dispatch
- plan delivery scheduling
- planner warmup, runtime reload, terrain cache warmup
- mission visualization tab
- id relationship tab update
- GUI log/file log 관리

문제:

- `MainWindow`가 application service, protocol adapter, GUI widget, runtime cache manager를 모두 직접 소유한다.
- 리팩토링 1순위는 `MainWindow`를 얇게 만들고, 기능별 controller/service로 빼는 것이다.

### 2. 상황 발생 후 재계획

재계획 흐름은 대략 다음 구조다.

1. `modules/monitoring/logic/*`가 상황을 감지하고 0902-like payload를 구성한다.
2. `mission_planning_gui.py`가 0902 payload를 수신하고 reason/level/trigger를 판별한다.
3. `pipelines/`의 trigger-specific pipeline을 실행한다.
4. `runtime/`이 state, ID reservation, JSON write, validation, debug artifact, logging을 보조한다.
5. GUI가 0301/0305/0901/0903 delivery를 수행한다.

주요 pipeline:

- `pipelines/attack_plan_pipeline.py`
- `pipelines/prior_mission_pipeline_impl.py`
- `pipelines/next_collab_replan_pipeline_impl.py`
- `pipelines/path_deviation_replan_pipeline_impl.py`
- `pipelines/imaging_schedule_replan_pipeline_impl.py`
- `pipelines/post_attack_rejoin_pipeline.py`
- `pipelines/current_remaining_hybrid.py`
- `pipelines/current_remaining_hybrid_replan.py`
- `pipelines/general_remaining_hybrid_replan.py`
- `pipelines/reexecute_first_mission_hybrid.py`
- `pipelines/recon_specialized_pipeline.py`

### 3. 핵심 계획 엔진

현재 핵심 산출물 생성은 다음 경로가 중요하다.

- `MissionPlanner/AnS/mission_pipeline.py::run_divide_and_pattern`
- `MissionPlanner/data_def/d0301.py::build_mission_plan`
- `MissionPlanner/data_def/d0302.py::build_mission_packages`
- `MissionPlanner/data_def/d0303.py::build_flight_plans`
- `MissionPlanner/data_def/d0304.py::build_lah_flight_plans_from_mrpk`
- `MissionPlanner/planning_enhanced/pipeline.py::run_enhanced_divide_and_pattern`
- `MissionPlanner/planning_enhanced/algo`, `pathing`, `scheduling`, `type_decider`, `io`

이 영역은 active runtime path이므로 이름 변경과 이동을 매우 천천히 해야 한다.

### 4. Runtime support

`runtime/`은 비교적 좋은 리팩토링 출발점이다.

- `json_io.py`: JSON serialization/write normalization
- `replan_validation.py`: generated payload validation
- `replan_id_reservation.py`: replan ID reservation wrapper
- `source_artifact_cache.py`: source artifact read/cache
- `mission_plan_file_logger.py`: plan run file log
- `mission_planning_pipeline_logging.py`: pipeline event/session log
- `latest_input_cache.py`: latest input package cache
- `attack_assignment_state.py`, `attack_tracking_state.py`, `prior_tracking_state.py`: reactive state
- `next_collab_*`: next collaborative mission state/runtime/headless runner
- `aircraft_parallel_0303.py`: aircraft-parallel 0303 generation helper

## 외부 호출자

다음 파일들은 mission planning 경로를 직접 참조한다.

- `run.py`: `mission_planning_gui.py` 실행, role alias, ID counter reset
- `app/ui/main_window.py`: mission planning launch 및 `MissionPlanner.runtime_settings` import
- `modules/common/agent_status_snapshot.py`: attack/prior tracking state import
- `modules/common/next_collab_replan_store.py`: compatibility store import
- `modules/monitoring/logic/init_replan.py`: mission plan ID allocation
- `modules/monitoring/logic/next_collab_replan.py`: next-collab runtime/store/settings
- `modules/monitoring/logic/prior_mission_replan.py`: prior/attack tracking state
- `modules/monitoring/logic/target_detection_replan.py`: attack assignment/tracking state
- `modules/monitoring/logic/turn_radius_monitor.py`: runtime settings
- `modules/monitoring/logic/mission_update.py`: FOV DB rows
- `modules/monitoring/monitoring_gui.py`: runtime settings and attack assignment state
- `modules/sim/runtime/sim_service.py`: `MissionPlanner.data_def.mission_helpers`

## 유지해야 하는 계약

- `modules/mission_planning/mission_planning_gui.py` 파일명과 실행 가능성
- `MainWindow` class
- `run_*_pipeline`, `warm_*_pipeline` public API
- `run_divide_and_pattern` signature
- `d0301/d0302/d0303/d0304` builder signature와 payload shape
- `MissionPlanner`, `AnS`, `data_def` import compatibility
- `runtime_settings.py` public functions
- `id_allocator.py` ID band, seed, file lock, reserve APIs
- 0902 payload keys, replan level semantics, trigger strings
- delivery ordering: 0301 -> 0305 status=2 -> 0901/0903
- DB artifact folders and `DSS_Internal` state/detail JSON locations
