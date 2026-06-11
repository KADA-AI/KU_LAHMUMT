# Target Architecture

목표는 기능 이름만 봐도 코드 위치를 추론할 수 있는 구조다. 단, 실제 적용은 compatibility wrapper를 유지한 채 단계적으로 한다.

## 제안 구조

```text
modules/mission_planning/
  mission_planning_gui.py          # public launcher shim, keep during migration
  __init__.py
  _paths.py

  app/
    bootstrap.py                   # sys.path, Qt/nFusion bootstrap
    main_window.py                 # thin MainWindow shell
    message_handlers/
      system_mode.py               # 0101/0102
      input_packages.py            # 0201/0203
      replan_requests.py           # 0902
      execution_commands.py        # 0803/0702/0903
    delivery/
      mission_plan_delivery.py     # 0301/0305/0901 sequencing
      option_delivery.py
    visualization/
      mission_visualization_tab.py

  mission_control/
    initial_planning_service.py
    planner_runtime.py             # warmup/reload/runtime source signature
    mission_session.py
    plan_metrics.py
    remaining_snapshot.py

  replanning/
    dispatcher.py                  # reason/level/trigger routing
    common/
      path_trim.py
      payload_validation.py
      source_artifacts.py
      id_reservation.py
      generated_artifacts.py
    triggers/
      attack/
      prior/
      next_collab/
      path_deviation/
      imaging_schedule/
      post_attack/
      remaining_hybrid/
      recon_specialized/
      reexecute_first_mission/

  engine/
    mission_generation/
      ans/
      artifacts_0301_0302_0303_0304/
      uav_pathing/
      lah_pathing/
      id_allocation/
    optimization/
      split/
      scheduling/
      pathing/
      type_decider/
      map/

  runtime/
    cache/
    state/
    logging/
    persistence/
    validation/
    ids/

  interfaces/
    nfusion/
    db/
    messages/
    json/

  ui/
    tabs/
    widgets/
    planners/

  config/
    runtime_settings.py
    schemas.py

  compat/
    root_wrappers/
    legacy_imports/

  legacy/
```

## 현재 파일 이동 방향

| 현재 | 목표 | 우선순위 | 비고 |
| --- | --- | ---: | --- |
| `mission_planning_gui.py` | `app/main_window.py`, `app/message_handlers/*`, `app/delivery/*`, `mission_control/*` | 1 | root file은 launcher shim으로 유지 |
| `pipelines/*_pipeline*.py` | `replanning/triggers/<trigger>/pipeline.py` | 2 | public wrapper 유지 |
| `pipelines/mission_path_trim.py` | `replanning/common/path_trim.py` | 2 | monitoring visualization import 주의 |
| `runtime/replan_validation.py` | `runtime/validation/replan_payloads.py` 또는 `replanning/common/payload_validation.py` | 2 | generated artifact contract 보존 |
| `runtime/replan_id_reservation.py` | `runtime/ids/replan_reservation.py` | 2 | ID allocator와 분리 유지 |
| `runtime/source_artifact_cache.py` | `runtime/cache/source_artifacts.py` | 2 | pipeline 중복 loader 통합 후보 |
| `runtime/attack_*_state.py`, `prior_tracking_state.py` | `runtime/state/<domain>.py` | 2 | monitoring import compatibility 필수 |
| `runtime/mission_*logging*.py` | `runtime/logging/*` | 2 | GUI log tab 연동 확인 |
| `MissionPlanner/runtime_settings.py` | `config/runtime_settings.py` | 3 | 외부 import 많으므로 wrapper 필수 |
| `MissionPlanner/data_def/id_allocator.py` | `engine/mission_generation/id_allocation/allocator.py` | 3 | ID band/file lock 고위험 |
| `MissionPlanner/data_def/d0301-d0304.py` | `engine/mission_generation/artifacts_0301_0302_0303_0304/` | 4 | signature 고정 후 이동 |
| `MissionPlanner/AnS` | `engine/mission_generation/ans` | 4 | bare import compatibility 확보 후 이동 |
| `MissionPlanner/planning_enhanced/*` | `engine/optimization/*` | 4 | 비교적 clean하지만 active path |
| `ui/*` | `ui/tabs/*`, `ui/widgets/*` | 2 | import path wrapper 유지 |
| `planners/next_collab_division/*` | `ui/planners/next_collab_division/*` | 3 | runtime headless runner가 내부 import |
| `MissionVisualizer/*` | `app/visualization/*` 또는 `ui/tools/visualizer/*` | 3 | duplicate visualizer 정리와 함께 |
| root wrapper files | `compat/root_wrappers/*` 또는 root 유지 | 5 | deprecation 후 삭제 |

## 새 구조에서 책임 기준

### `app`

PyQt/nFusion application shell만 둔다. GUI widget, message listener registration, button action binding은 여기까지 가능하지만, 실제 계획/재계획 계산은 service로 넘긴다.

### `mission_control`

전체 임무 계획 관리 영역이다.

- 초기 계획 실행
- planner runtime warmup/reload
- 현재 mission/session context
- post-delivery state
- plan metrics
- remaining snapshot apply/merge

### `replanning`

상황 발생 후 재계획 전용 영역이다.

- trigger 판단
- replan context normalization
- attack/prior/next-collab/path-deviation/imaging/post-attack/hybrid pipeline
- generated artifact validation
- source plan artifact loading

### `engine`

임무 계획 산출물 생성 알고리즘 영역이다. message protocol이나 GUI를 알면 안 된다.

### `runtime`

상태, cache, log, persistence, validation, ID reservation 같은 cross-cutting support다. GUI에 의존하면 안 된다.

### `interfaces`

nFusion, DB path, message payload conversion 같은 외부 세계 adapter를 둔다.

### `compat`

기존 import 경로를 살리는 wrapper만 둔다. 실제 로직은 새 구조에 있어야 한다.
