# Refactoring Roadmap and TODO

## Phase 0. 안전망 구축

목표: 코드 이동 전 깨짐을 빠르게 감지한다.

TODO:

- [x] `modules/mission_planning` active import smoke script 작성
- [x] `mission_planning_gui.py` import/launch smoke 정의
- [x] wrapper/import-order contract smoke 작성: `smoke_import_contract.py`
- [x] moved trigger old/new import object identity smoke 작성
- [x] PR 직전 `smoke_import_contract.py --require-git-tracked` 실행
- [x] `KU_ROLE=mission`과 0301/0302/0303/0304 SW code baseline smoke 작성
- [x] `run.py` 경유 launch와 `app/ui/main_window.py` 경유 launch env parity smoke 작성
- [x] `run.py` cold start 전체 GUI launch/control-port smoke 작성
- [x] 주요 pipeline import smoke 정의
- [x] `run_attack_plan_pipeline`, `run_prior_mission_pipeline`, `run_next_collab_replan_pipeline`, `run_path_deviation_replan_pipeline`, `run_imaging_schedule_replan_pipeline`, `run_post_attack_rejoin_pipeline` signature snapshot 작성
- [x] `d0301/d0302/d0303/d0304` public builder signature snapshot 작성
- [x] ID allocator counter 파일과 reserve API baseline 기록
- [x] sample 0201/0203/0902 payload fixture 확보
- [x] generated 0301/0302/0303/0304 artifact link validation script 작성
- [x] launcher/control contract inventory 작성
- [x] cwd/sys.path/bare import matrix 작성: repo root, `modules/mission_planning`, dashboard child cwd
- [x] nFusion config 있음/없음 실패 정책과 `MMR_ReceiveNode` channel smoke 작성
- [x] planner hot-reload watch list와 reload order snapshot 작성
- [x] planner hot-reload `globals()` rebinding fixture 작성
- [x] `recon_specialized_pipeline.py` watch-only/reload 정책 결정 및 smoke 작성
- [x] bootstrap import-order contract 작성: `KU_ROLE=mission`, console/file logging 설치 전 side-effect 금지
- [x] 0902 normalization fixture 작성
- [x] 0902 ID extraction priority fixture 작성: `optionList/pendingOptionList` -> `missionPlanIDList` -> `replanDetail.missionPlanID`
- [x] 0902 malformed `optionList`가 valid `pendingOptionList`로 fallback되지 않는 현행 동작 fixture 작성
- [x] 0902 `inputMissionIDList` dict-only 추출 fixture 작성
- [x] 0902 trigger/delay exact-match fixture 작성
- [x] trigger별 0902 delay/deferred queue fixture 작성
- [x] init planning 중 0902 terrain warmup/timing marker가 deferred queue보다 먼저 예약되는 순서 fixture 작성
- [x] captured 0902 replay와 store-backed detail fixture 작성
- [x] replan dispatcher priority/handled semantics fixture 작성
- [x] monitoring queue priority/source-plan rebound/suppress semantics fixture 작성
- [x] delivery order matrix와 fake `push_message` smoke 작성
- [x] quality-speed/direct delivery matrix와 0901/0702 suppression smoke 작성
- [x] attack delivery suppress flag smoke 작성
- [x] post-delivery waypoint mark/snapshot carry-forward smoke 작성
- [x] pipeline result shape snapshot 작성
- [x] ID allocator cold-reset/concurrent-reserve parity test 작성
- [x] runtime artifact/resource path manifest 작성
- [x] runtime I/O/cache/log helper smoke 작성: `json_io`, `latest_input_cache`, `mission_plan_file_logger`, `mission_planning_pipeline_logging`
- [x] 0101 parsing allow-list fixture 작성: top-level alias, raw fallback, bool mapping, text mode `"on" -> 0`
- [x] 0201/0203 latest input fixture 작성: 0202 제외, non-empty core list 저장, `MissionReferenceInfo` 디렉터리, `Source/source` 로그 원문 보존
- [x] ID/state JSON artifact manifest 작성
- [x] runtime DB state artifact manifest 작성: `DSS_Internal/targetInfo.json`, `VehicleStatus/status.json`, progress/state JSON
- [x] HTML/PNG 산출물 manifest 작성: mission map HTML, attack visualization PNG
- [x] manual/operator entrypoint inventory 작성
- [x] monitoring/common/app 외부 import contract inventory 작성
- [x] `logic_test`, tool GUI, portable bundle manual workflow owner decision 작성
- [x] `lah_attack_assistance.py` subprocess smoke 작성
- [x] portable bundle `python app.py`/`run_portable.bat` smoke 작성
- [x] next-collab/next-area manual planner flow-mode smoke 작성
- [x] current-remaining hybrid failure fallback/pathID mapping fixture 작성

완료 조건:

- import smoke가 현재 main branch 상태에서 통과한다.
- 최소 하나의 representative mission/replan scenario 결과를 baseline으로 기록한다.
- hot reload, delivery order, ID allocator, 0902 normalization의 현재 동작이 baseline으로 기록된다.

## Phase 1. Compatibility scaffold 공식화

목표: 대규모 rename/move가 가능한 wrapper 정책을 먼저 만든다.

TODO:

- [x] root wrapper 파일 목록 작성
- [x] 외부 import가 있는 wrapper와 없는 wrapper 분리
- [x] wrapper 지원 경로 matrix 작성
- [x] wrapper template 통일
- [x] `compat/` 또는 root 유지 전략 결정
- [x] deprecated import 경로 로그/문서화 여부 결정
- [x] `mission_planning_gui.py`를 public launcher로 고정하고 내부 import target을 새 파일로 넘기는 구조 준비

완료 조건:

- 기존 import 경로와 새 import 경로가 동시에 동작한다.
- 외부 호출자인 `run.py`, `app`, `monitoring`, `common`, `sim`을 수정하지 않아도 smoke가 통과한다.

## Phase 2. `mission_planning_gui.py` 분해

목표: 14k line application shell을 기능별 service/controller로 나눈다.

우선 추출 순서:

1. bootstrap/path setup
2. planner warmup/reload/runtime source signature
3. 0101/0102 system mode/heartbeat
4. 0201/0203 latest input handling
5. 0902 replan parsing/staging/dispatch
6. 0301/0305/0901/0903 delivery scheduling
7. plan metrics and lifecycle logging
8. mission visualization tab

TODO:

- [x] `app/bootstrap.py` 추출
- [x] `mission_control/planner_runtime.py` 추출
- [x] `app/message_handlers/system_mode.py` 추출
- [x] `app/message_handlers/input_packages.py` 추출
- [x] `app/message_handlers/replan_requests.py` 추출
- [x] `app/delivery/mission_plan_delivery.py` 추출
- [x] `mission_control/plan_metrics.py` 추출
- [x] `app/visualization/mission_visualization_tab.py` 추출

완료 조건:

- root `mission_planning_gui.py`는 `MainWindow` export와 executable launcher를 유지한다.
- 각 추출 후 import smoke와 GUI startup smoke를 통과한다.

## Phase 3. 재계획 pipeline 재구성

목표: 상황별 재계획 코드를 trigger 기준으로 묶는다.

TODO:

- [x] `replanning/dispatcher.py` 생성
- [x] attack pipeline을 `replanning/triggers/attack/`로 이동
- [x] prior pipeline을 `replanning/triggers/prior/`로 이동
- [x] next-collab pipeline을 `replanning/triggers/next_collab/`로 이동
- [x] path-deviation pipeline을 `replanning/triggers/path_deviation/`로 이동
- [x] imaging schedule pipeline을 `replanning/triggers/imaging_schedule/`로 이동
- [x] post-attack rejoin pipeline을 `replanning/triggers/post_attack/`로 이동
- [x] current/general remaining hybrid를 `replanning/triggers/remaining_hybrid/`로 묶기
- [x] recon/reexecute helper를 명확한 trigger 하위로 이동
- [x] 이동 완료된 기존 `pipelines/*.py`는 wrapper로 유지
- [x] 이동 완료 trigger의 stale old-impl import guard 작성

완료 조건:

- `mission_planning_gui.py`는 dispatcher만 호출한다.
- 기존 `modules.mission_planning.pipelines.*` import가 유지된다.

## Phase 4. Runtime support 재배치

목표: state/cache/logging/validation/ID/persistence를 기능별로 묶는다.

TODO:

- [x] `runtime/state/attack_assignment.py` 추출
- [x] `runtime/state/attack_tracking.py` 추출
- [x] `runtime/state/prior_tracking.py` 추출
- [x] `runtime/cache/source_artifacts.py` 추출
- [x] `runtime/cache/latest_input.py` 추출
- [x] `runtime/logging/pipeline_events.py` 추출
- [x] `runtime/logging/plan_file_logger.py` 추출
- [x] `runtime/validation/replan_payloads.py` 추출
- [x] `runtime/ids/replan_reservation.py` 추출
- [x] 기존 `runtime/*.py`는 wrapper로 유지

완료 조건:

- monitoring/common 외부 import가 기존 경로로 계속 동작한다.
- state file path와 DSS_Internal artifact 위치가 바뀌지 않는다.

## Phase 5. Engine 이름 정리

목표: `MissionPlanner`의 핵심 엔진을 의미 있는 이름으로 옮긴다.

주의:

이 단계는 가장 위험하다. `AnS`, `data_def`, bare import, dynamic sys.path, file watcher, model/DEM artifact가 얽혀 있다.

TODO:

- [x] `MissionPlanner/data_def` public API inventory 작성
- [x] bare import `AnS`, `data_def`, `config` 사용처 제거 또는 shim화
- [x] `id_allocator.py`를 먼저 wrapper 뒤로 숨김
- [x] `planning_enhanced`를 `engine/optimization`으로 이동할 수 있는지 import graph 확인
- [x] `d0301-d0304`를 artifact builder package로 이동
- [x] `AnS/mission_pipeline.py` 이동 전 path bootstrap과 dynamic reload 목록 수정

완료 조건:

- representative mission scenario에서 0301/0302/0303/0304 산출물 ID/link가 이전과 동일한 규칙을 따른다.

## Phase 6. 삭제/아카이브

목표: 검증된 불필요 코드만 작게 삭제한다.

TODO:

- [x] 삭제 후보별 reachability 검증
- [x] 삭제 후보별 owner/manual workflow 확인
- [x] generated output을 fixture로 유지할지 삭제할지 결정
- [x] root wrapper deprecation 기간 결정
- [x] `legacy` bucket 삭제 또는 archive strategy 결정
- [x] backup-style 파일 삭제 여부 결정

완료 조건:

- 삭제 후 import smoke, scenario smoke, generated artifact validation 통과
- 삭제 이유와 검증 로그가 PR에 남아 있음

## 추천 PR 순서

1. safety smoke scripts and docs
2. `mission_planning_gui.py` bootstrap/runtime extraction
3. message handler extraction
4. delivery scheduler extraction
5. replanning dispatcher introduction
6. trigger-specific pipeline moves with wrappers
7. runtime support moves with wrappers
8. conservative deletion batch 1
9. engine migration prep
10. engine migration small batches
