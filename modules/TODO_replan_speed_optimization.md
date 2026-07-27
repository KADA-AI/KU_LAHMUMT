# 재계획 속도 개선 TODO

작성일: 2026-06-16

## 범위와 원칙

- 대상 범위는 이 폴더(`modules_사천시연_최적화_ver2`) 내부만이다.
- 목적은 계획 및 재계획의 wall-clock 시간, 큐 대기 시간, 고빈도 상태 처리 비용을 줄이는 것이다.
- 기능, 판단 조건, 파라미터, 토글 기본값, 임계값, 확률, coalesce 시간, delivery grace/timeout은 변경하지 않는다.
- 변경 전후의 0902/0301/0901/0903/0702 의미가 동일해야 한다.
- 모든 TODO는 먼저 계측으로 병목 여부를 확인한 뒤 적용한다.

## 1. 확인된 재계획 요소

### Monitoring에서 생성되는 0902 재계획 요청

1. `0201` 입력임무 갱신 재계획
   - 경로: `monitoring/monitoring_gui.py::_on_rx_0201`
   - coordinator: `monitoring/logic/input_refresh_replan.py::InputRefreshReplanCoordinator`
   - detail: `trigger=0201`, `triggerType=inputRefresh`
   - 조건: 모드 3/4, 중복 window 회피, 토글 ON, 재수행 처리와 충돌하지 않을 때

2. `0803 execute=2` 이후 `0201` 재도착에 따른 협업임무 재수행
   - 경로: `monitoring/monitoring_gui.py::_on_rx_0803`, `_on_rx_0201`
   - coordinator: `monitoring/logic/collab_reexecute.py::CollabReexecuteCoordinator`
   - detail: `triggerType=collabReexecuteInputRefresh`

3. `0202` 선행임무 정보 기반 재계획
   - 경로: `monitoring/monitoring_gui.py::_on_rx_0202`
   - coordinator: `monitoring/logic/prior_mission_replan.py::PriorMissionReplanCoordinator`
   - mission planning pipeline: `mission_planning/replanning/triggers/prior/pipeline.py`

4. `0401` 선행임무 종료 후 복귀
   - 경로: `monitoring/monitoring_gui.py::_on_rx_0401`의 `prior_close` phase
   - coordinator: `PriorMissionReplanCoordinator.on_agent_states`
   - detail: `triggerType=priorClosedResume`
   - mission planning direct pipeline: `_try_run_prior_post_rejoin_pipeline`

5. `0401` DL 위험 기반 선행임무 재계획
   - 경로: `monitoring/monitoring_gui.py::_update_dl_inference`, `_maybe_trigger_dl_replan`
   - coordinator: `PriorMissionReplanCoordinator.on_risk_update`
   - 조건: 위험 지수, cooldown, 모드, 토글 조건

6. `0401` RTB/비정상/비가용 상태
   - 경로: `monitoring/monitoring_gui.py::_on_rx_0401`
   - coordinator: `monitoring/logic/rtb_replan.py::RtbReplanCoordinator`
   - detail 후보: `unexpectedRTB`, `abnormalHealthUnavailable`, `payloadHealthUnavailable`, `communicationLossUnavailable`
   - `fuel_threshold`는 단독 0902 생성보다 fuel warning 및 RTB detail context에 영향을 준다.

7. `0802` 강제명령 재계획
   - 경로: `monitoring/monitoring_gui.py::_on_rx_0802`
   - coordinator: `monitoring/logic/forced_command_replan.py::ForcedCommandReplanCoordinator`
   - detail: `triggerType=forcedCommand`
   - `mandatoryType=2` 즉시, `mandatoryType=1` hold 만료 후, `mandatoryType=3` 복귀 조건

8. `0401` 경로 이탈/선회 기반 재계획
   - 경로: `monitoring/monitoring_gui.py::_on_rx_0401`
   - coordinator: `monitoring/logic/path_deviation_replan.py::PathDeviationReplanCoordinator`
   - detail: `triggerType=pathDeviation`
   - mission planning direct pipeline: `_try_run_path_deviation_replan_pipeline_impl`

9. `0401` 촬영품질 속도 재계획
   - 경로: `monitoring/monitoring_gui.py::_on_rx_0401`
   - coordinator: `monitoring/logic/quality_speed_replan.py::QualitySpeedReplanCoordinator`
   - detail: `triggerType=qualityMonitorSep`
   - mission planning direct pipeline: `_try_run_imaging_schedule_replan_pipeline`

10. `0401` 촬영 일정 이탈 재계획
    - 경로: `monitoring/monitoring_gui.py::_on_rx_0401`
    - coordinator: `monitoring/logic/imaging_schedule_replan.py::ImagingScheduleReplanCoordinator`
    - detail: `triggerType=imagingScheduleDeviation`
    - mission planning direct pipeline: `_try_run_imaging_schedule_replan_pipeline`

11. `0803 execute=1` 다음 협업기저임무
    - 경로: `monitoring/monitoring_gui.py::_on_rx_0803`
    - coordinator: `monitoring/logic/next_collab_replan.py::NextCollabMissionReplanCoordinator`
    - detail: `triggerType=nextCollaborativeMission`
    - mission planning direct pipeline: `_try_run_next_collab_replan_pipeline_impl`

12. `0402` 표적 탐지/공격 재계획
    - 경로: `monitoring/monitoring_gui.py::_on_rx_0402`
    - coordinator: `monitoring/logic/target_detection_replan.py::TargetDetectionCoordinator`
    - mission planning route: `mission_planning/replanning/dispatcher.py::should_use_attack_pipeline`
    - mission planning direct pipeline: `_try_run_attack_plan_pipeline`

13. `0402` 공격 후 복귀
    - 경로: `TargetDetectionCoordinator._build_attack_close_payloads`, `monitoring_gui.py::_queue_0402_replan_payloads`
    - detail: `triggerType=attackClosedDestroyed`
    - mission planning direct pipeline: `_try_run_post_attack_rejoin_pipeline`
    - 큐에서 일반 target attack보다 우선될 수 있다.

14. `0801` 운용자 임무재계획 명령
    - `common/receive/message0801_receiver.py`에서 수신/notify는 확인된다.
    - 현재 확인한 `monitoring`/`mission_planning` 경로에서는 0801 수신이 자동 0902 생성으로 직접 연결되는 핸들러가 보이지 않는다.
    - TODO: 실제 UI 버튼/수동 명령 흐름이 별도 연결되어 있는지 추가 확인한다.

### Mission Planning 전용 분기

- `mission_planning/replanning/dispatcher.py`의 특수 파이프라인 우선순위:
  1. `post_attack_rejoin`
  2. `next_collab`
  3. `imaging_schedule`
  4. `path_deviation`
  5. `prior`
- 위 특수 파이프라인이 처리하지 못하면 일반 재계획으로 내려간다.
- 일반 재계획 핵심 흐름:
  - `mission_planning/mission_planning_gui.py::_schedule_replan_pipeline`
  - `_run_replan_pipeline_async`
  - `_run_replan_pipeline_do`
  - `_run_general_variant_core`
  - `run_divide_and_pattern`
  - `build_mission_plan_0301`
  - `0303/0304/0302` 생성 및 저장/검증

## 2. 속도 개선 가설

### H1. 0401 처리의 중복 parse/save/fanout 비용

근거:
- `monitoring/monitoring_gui.py::_payload_signature`가 JSON 정규화를 위해 payload를 파싱/직렬화한다.
- 이후 `_on_rx_0401`에서 `parse_payload`와 `extract_0401_agent_states`가 다시 상태를 해석한다.
- `_on_rx_0401` 한 번에 fuel, prior, visualization, snapshot save, DL, turn tab, schedule tab, quality tab, RTB, path deviation, quality speed, imaging schedule, forced availability가 순차 실행된다.
- `monitoring/logic/agent_status_snapshot.py` 저장 경로는 기존 snapshot을 읽고 새 JSON을 다시 쓰므로 고빈도 0401에서 별도 I/O 병목이 될 수 있다.

가설:
- 같은 0401 payload에 대해 parsed dict, canonical signature, extracted states를 한 번 만든 뒤 coordinator에 공유하면 JSON 파싱과 상태 추출 비용을 줄일 수 있다.
- 0401 저장/trace/log는 기존 의미를 유지하되, 변경 감지/배치/지연 쓰기 가능성을 계측한 뒤 검토한다.

주의:
- `_0401_coalesce_ms`, cooldown, 확률, quality threshold 등 동작 파라미터는 변경하지 않는다.

### H2. DB JSON 로딩과 fallback glob/read 반복

근거:
- `monitoring/logic/mission_update.py::_resolve_db_json_path`는 직접 파일 경로가 없으면 folder 내 `*.json`을 돌며 파일을 열어 ID를 비교한다.
- `collect_available_aircraft_ids`도 fallback에서 `InputMissionPlan/*.json`을 스캔할 수 있다.
- path deviation, quality speed, imaging schedule coordinator가 각자 `MissionPlan`, `IndividualMissionPlan`, `FlightPath`를 반복 로딩한다.

가설:
- 0401 처리 또는 단일 0902 생성 단위의 `JsonLoadCache`를 두고 `(folder, id, mtime_ns, size)`로 캐시하면 같은 payload 내 반복 I/O를 제거할 수 있다.
- fallback scan은 folder별 `id -> path` index를 유지하고, mtime/size 변화 시 무효화한다.

주의:
- 직접 `ID.json` 경로가 항상 존재하는 정상 DB에서는 fallback glob/read가 hot path가 아닐 수 있다. 먼저 fallback 발생 횟수와 시간을 측정한다.

### H3. source artifact resolver 중복

근거:
- `monitoring/logic/path_deviation_replan.py`, `quality_speed_replan.py`, `imaging_schedule_replan.py`의 `_resolve_source_artifacts`가 유사하게 plan/IMP/path를 읽고 mission/waypoint를 선형 탐색한다.
- 현재 waypoint 확인은 list membership 후 `index()` 재탐색 패턴이 여러 pipeline에도 있다.

가설:
- 공통 `SourceArtifactResolver` 또는 `CurrentPlanIndex`를 만들어 aircraft, mission, path, waypoint id set/index를 한 번만 구성한다.
- 각 coordinator는 동일 index를 읽기 전용으로 사용한다.

### H4. ReplanQueueManager의 deepcopy/signature 비용

근거:
- `monitoring/logic/replan_queue_manager.py`는 enqueue, result, snapshot에서 payload deepcopy와 signature 생성을 수행한다.
- 내부에 `_copy_metrics`가 있어 이미 copy 비용을 관찰할 수 있는 구조가 있다.
- `monitoring/gui/tabs/replan_queue_tab.py::set_snapshot`도 snapshot을 deepcopy하고, `_rebuild_list`는 매번 위젯을 전체 재생성한다.

가설:
- 먼저 `_copy_metrics`와 queue refresh 시간을 계측한다.
- 큐 내부는 불변 DTO 또는 필요한 필드만 복사하고, UI는 item id/version 기반 diff 갱신으로 바꾼다.

주의:
- queue 중복 제거, 병합, target detection suppression, attack-close preemption, option suppression은 재계획 기능이다. 속도 개선 명목으로 판정 조건이나 순서를 바꾸지 않는다.

### H5. replan settings deepcopy 비용

근거:
- `monitoring/logic/replan_runtime_settings.py::load_replan_settings`는 캐시된 설정을 반환할 때도 deepcopy한다.
- 고빈도 coordinator가 settings getter를 반복 호출한다.

가설:
- 0401 dispatch 시작 시 settings snapshot을 한 번 읽고 각 coordinator에 전달한다.
- 또는 읽기 전용 immutable snapshot을 캐시해 반복 deepcopy를 줄인다.

주의:
- 설정 값, 기본값, 토글 의미는 바꾸지 않는다.

### H6. 0902 payload sidecar/JSON 왕복 비용

근거:
- `common/push/message0902_push.py`는 rich payload를 sidecar에 저장하고, C# 객체 변환 중 `replanDetail`을 JSON 문자열화한다.
- `common/receive/message0902_receiver.py`는 C# 객체를 dict로 바꾼 뒤 sidecar를 다시 읽어 병합한다.

가설:
- 같은 프로세스/같은 host 경로에서는 full dict를 직접 전달하거나 canonical JSON을 재사용하는 fast path를 둔다.
- sidecar는 호환성 fallback 또는 대용량 payload 보존용으로 유지한다.

주의:
- sidecar 제거 또는 비활성화는 금지한다. CLR 메시지 whitelist 이후 보존되지 않는 rich field를 복원하는 안전장치로 보인다.
- 메시지 wire format과 외부 연동 호환성을 깨지 않는 범위에서 canonical JSON 재사용, 중복 write 억제, compact 저장만 검토한다.

### H7. Mission planning 입력 JSON 재로딩과 SourceArtifactCache deepcopy

근거:
- GUI가 `SourceArtifactCache`로 0201/0203을 읽은 뒤에도 `run_divide_and_pattern`/`build_mission_plan_0301`에서 같은 파일을 다시 `json.load`할 수 있다.
- `mission_planning/runtime/cache/source_artifacts.py::SourceArtifactCache.read_json`은 기본적으로 cache hit/miss 모두 deepcopy를 반환한다.

가설:
- 내부 함수에 이미 파싱된 payload 또는 cache handle을 전달한다.
- 읽기 전용 호출 경로는 `copy_result=False`를 확대하고, 수정이 필요한 branch만 copy-on-write로 복사한다.

주의:
- `copy_result=False` 전역 전환은 금지한다. 여러 파이프라인이 반환값을 caller-owned copy로 보고 mutate할 가능성이 있으므로 읽기 전용으로 증명된 지점만 국소 적용한다.

### H8. 특수 파이프라인의 JSON write batch와 read-back 비교 비용

근거:
- GUI에는 병렬 serialize/write helper가 있으나 `mission_planning/runtime/json_io.py::write_json_batch`는 순차 루프다.
- `write_json_bytes`는 기존 파일 size가 같으면 `read_bytes()`로 전체 payload를 비교한다.
- next-collab/path-deviation/imaging/prior가 공용 batch write 경로를 쓴다.

가설:
- 공용 `write_json_batch`를 병렬 serialize/write 가능 구조로 맞춘다.
- 동일 run에서 새로 생성한 경로는 기존 파일 read-back 비교를 생략할 수 있는지 계측한다.
- 파일별 hash/mtime cache로 read-back을 줄일 수 있는지 확인한다.

### H9. 0303/0304와 lineSearch/terrain 중복 pass

근거:
- `d0303.py`는 DEM/ground cache가 있지만 packet prewarm, area, non-area, formation follower, final prepass에서 waypoint 계열을 여러 번 스캔한다.
- `aircraft_parallel_0303.py`는 metric용으로 lineSearch를 별도 JSON 직렬화한다.

가설:
- run-local ground context와 dirty flag를 명시적으로 전달해 중복 scan과 lock 진입을 줄인다.
- metric byte 계산은 최종 직렬화 단계와 합치거나 lazy로 전환한다.

### H10. 선형 탐색, O(N^2), 전수탐색 제거

근거:
- waypoint `in list` 후 `list.index()` 재탐색이 prior/attack/path-deviation pipeline에 있다.
- attack pipeline target dedupe가 누적 리스트 `any(...)` 패턴이다.
- remaining hybrid에서 `target_aircraft_ids.index(...)` 반복이 있다.
- ID allocator가 FlightPath/IMP 디렉터리 전체 glob/read를 여러 reservation 흐름에서 반복한다.
- next-collab/area planner 쪽에 `permutations`/`combinations` 전수 탐색이 있다.
- MILP scheduler는 변수/제약 생성이 후보 수 제곱으로 커질 수 있다.

가설:
- waypoint id -> index dict, aircraft id -> entry dict, normalized target key set, directory snapshot, sparse transition table을 추가한다.
- 전수탐색 대체는 결과 동일성 증명이 필요하므로, 우선 candidate pruning이 기존 불가능 전이 제거에만 해당하는지 검증한다.

주의:
- target dedupe O(N^2), waypoint `list.index`, latest-file glob/stat는 실제 입력 크기가 작거나 fallback 조건일 수 있으므로 P0 계측 전에는 고우선 구현으로 보지 않는다.
- Hungarian/min-cost matching 등 알고리즘 대체는 결과 tie-break와 최적성 기준을 바꿀 수 있다. 기능 변경 위험이 있으므로 별도 검증 없이는 P1 구현 대상이 아니다.

### H11. 병렬 executor 중첩과 worker budget

근거:
- 일반 variant 병렬, store/json 병렬, 0303/0304 병렬, 0303 내부 aircraft 병렬이 중첩될 수 있다.
- 0303 쪽은 이미 내부 lineSearch 병렬 억제 컨텍스트가 있으므로 단순 worker 증가가 답이 아닐 수 있다.

가설:
- 기존 상한 파라미터를 바꾸지 않고, outer 작업 수에 따라 inner worker를 조정하는 중앙 budget을 둔다.
- oversubscription 여부는 CPU 사용률, context switch, stage timing으로 확인한다.

### H12. UI/로그/폴링 비용

근거:
- `monitoring_gui.py`는 여러 RX message poller에서 table row를 반복 scan한다.
- listener 기반 처리와 poller 기반 처리가 함께 있어, 동일 메시지에 대한 중복 감시 비용이 생길 수 있다.
- worker thread에서 `log_sig.emit`과 timing event emit이 많이 발생한다.
- `ReplanQueueTab`은 snapshot마다 카드 전체 rebuild를 한다.

가설:
- RX table row index cache를 만들고 table rebuild 시 무효화한다.
- 로그는 stage 단위 버퍼링/스로틀링을 검토한다.
- 큐 UI는 diff update로 바꾼다.

주의:
- UI 표현 지연이나 로그 누락이 생기면 안 된다. 계산 hot path보다 후순위로 둔다.

## 3. TODO

### P0. 기준 계측과 재현 시나리오 고정

- [x] 대표 재계획 시나리오를 최소 5개로 고정한다.
  - [x] 0401 고빈도 + path/quality/imaging 후보가 많은 상황
  - [x] 0402 target detection + attack pipeline
  - [x] 0402 attackClosedDestroyed + post-attack rejoin
  - [x] 0803 execute=1 + next collaborative mission
  - [x] 0202/prior 또는 0401 priorClosedResume
- [x] 기존 timing을 수집한다.
  - [x] `mission_planning_gui.py`의 `[REPLAN][TIME]`
  - [x] 특수 pipeline의 `timingMs`
  - [x] `ReplanQueueManager._copy_metrics`
  - [x] `0401TRACE`
- [x] 추가 계측 후보를 기능 변경 없이 넣는다.
  - [x] `load_db_json`: folder/id, hit/miss, fallback scan count, bytes, ms
  - [x] `_resolve_source_artifacts`: 호출 횟수, 읽은 파일 수, waypoint scan 수, ms
  - [x] `write_json_batch`: 파일 수, serialize ms, write ms, unchanged read-back ms
- [x] 0902 sidecar: save/read/merge ms, payload bytes
- [x] queue UI refresh: snapshot deepcopy ms, rebuild ms, widget count
- [x] 0401 snapshot save: previous snapshot read ms, new snapshot write ms, bytes
- [x] RX listener/poller: listener 수신 횟수, poller scan 횟수, poller enqueue 중복 회피 횟수
- [x] 0303: `generateLineSearchMs`, `groundPrepassMs`, `jsonReadyMs`, `innerParallelWorkers`
- [x] 변경 전 산출물 hash/equality baseline을 저장한다.
  - [x] timestamp, 생성 ID, 로그처럼 실행마다 달라지는 필드는 비교 제외 규칙을 문서화한다.
  - [x] missionPlan/individualMissionPlan/flightPath의 구조적 동일성을 비교한다.

### P1. Monitoring 0401 fast path 정리

- [x] `_payload_signature`, `parse_payload`, `extract_0401_agent_states`의 중복 JSON parse 여부를 계측한다.
- [x] 0401 dispatch context 객체를 만든다.
  - [x] raw payload
  - [x] parsed dict
  - [x] canonical signature
  - [x] extracted agent states
  - [x] settings snapshot
  - [x] per-dispatch DB JSON cache
- [x] 각 coordinator가 context를 선택적으로 받도록 확장한다.
- [x] 기존 API는 유지해 다른 호출 경로가 깨지지 않게 한다.
- [x] 변경 전후 trigger 발생 여부와 trigger order가 동일한지 replay로 확인한다.
- [x] 고빈도 0401 stress에서 누락/중복 replan, queue 순서 역전, snapshot 누락이 없는지 확인한다.

### P1. DB JSON cache와 folder id index

- [x] `monitoring/logic/mission_update.py`에 run-local JSON cache 후보를 설계한다.
- [x] fallback `_resolve_db_json_path`에 folder별 `id -> path` index를 추가할 수 있는지 확인한다.
- [x] cache key는 최소 `(absolute_path, mtime_ns, size)`를 포함한다.
- [x] `save_db_json` 이후 cache/index 무효화 규칙을 정한다.
- [x] path deviation, quality speed, imaging schedule에서 반복 load가 줄어드는지 측정한다.

### P1. 공통 source artifact resolver/index

- [x] path deviation, quality speed, imaging schedule의 `_resolve_source_artifacts` 공통 필드를 비교한다.
- [x] current mission plan 기준 index를 만든다.
  - [x] `aircraft_id -> aircraft_entry`
  - [x] `individualMissionPlanID -> payload`
  - [x] `mission_id -> mission entry`
  - [x] `flightPathID -> payload`
  - [x] `waypoint_id -> index`
  - [x] `waypoint_id set`
- [x] 각 coordinator가 기존 payload 구조를 그대로 생성하는지 golden replay로 비교한다.
- [x] resolver 내부에서 불필요한 full deepcopy가 생기지 않게 읽기 전용 payload를 유지한다.

### P1. Mission planning JSON/cache hot path

- [x] `SourceArtifactCache.read_json(copy_result=True)` 호출 지점을 분류한다.
  - [x] 실제 mutate하는 호출
  - [x] 읽기 전용 호출
  - [x] 일부 branch만 mutate하는 호출
- [x] 읽기 전용 호출은 `copy_result=False` 전환 가능 여부를 확인한다.
- [x] `run_divide_and_pattern`/`build_mission_plan_0301`에 이미 파싱된 payload 또는 cache handle을 전달하는 경로를 설계한다.
- [x] 특수 파이프라인의 full-plan/waypoint deepcopy 위치를 계측하고 copy-on-write 후보를 표시한다.

### P1. JSON write path 통합 최적화

- [x] `mission_planning/runtime/json_io.py::write_json_batch` 사용처를 목록화한다.
- [x] GUI의 병렬 serialize/write helper와 공용 batch write의 동작 차이를 비교한다.
- [x] 공용 batch write가 pre-serialized bytes를 받을 수 있게 API 확장 가능성을 검토한다.
- [x] `write_json_bytes`의 unchanged read-back 시간이 큰지 측정한다.
- [x] 새 run에서 생성되는 파일은 read-back 비교 생략이 가능한지 파일명/경로 규칙을 확인한다.

### P2. ReplanQueueManager와 queue UI 비용 축소

- [x] `_copy_metrics`를 활성 수집해 deepcopy label별 비용을 확인한다.
- [x] enqueue/result/snapshot에서 전체 payload deepcopy가 필요한지 분류한다.
- [x] signature 생성 대상 필드를 줄여도 중복 판단 의미가 동일한지 확인한다.
- [x] `ReplanQueueTab.set_snapshot`의 deepcopy와 `_rebuild_list` 전체 widget rebuild 시간을 측정한다.
- [x] item id/version 기반 diff update 설계를 만든다.

### P2. 0902 transport fast path 검토

- [x] `message0902_push.py`의 sidecar 저장, detail JSON stringify, C# 객체 변환 시간을 계측한다.
- [x] `message0902_receiver.py`의 sidecar read/merge 시간을 계측한다.
- [x] canonical JSON을 한 번만 만들고 push/receiver가 재사용할 수 있는지 확인한다.
- [x] sidecar field 보존 테스트를 정의한다.
- [x] 중복 write 억제 또는 compact 저장을 적용해도 receiver merge 결과가 동일한지 확인한다.
- [x] sidecar fallback 유지 조건과 외부 호환성 테스트를 정의한다.

### P2. 0303/terrain/lineSearch 중복 pass 줄이기

- [x] 추정으로 먼저 고치지 말고 기존 metric 기준으로 stage를 좁힌다.
- [x] `d0303.py` stage별 waypoint scan 횟수와 DEM/ground cache hit ratio를 측정한다.
- [x] `generateLineSearchMs`, `groundPrepassMs`, `jsonReadyMs`, `innerParallelWorkers`를 기준 metric으로 수집한다.
- [x] run-local ground context를 명시적으로 전달할 수 있는지 설계한다.
- [x] `aircraft_parallel_0303.py` lineSearch metric용 JSON 직렬화가 실제 병목인지 측정한다.
- [x] metric은 최종 serialize 결과에서 수집할 수 있는지 확인한다.

### P3. 선형 탐색과 O(N^2) 제거

- [x] waypoint list membership + index 재탐색 지점에 `waypoint_id -> index` dict를 만든다.
- [x] attack target dedupe에 normalized target key set을 병행한다.
- [x] remaining hybrid의 `target_aircraft_ids.index(...)` 반복을 `aircraft_id -> entry` map으로 바꿀 수 있는지 확인한다.
- [x] ID allocator의 directory snapshot 공유 범위를 reservation 단위로 제한한다.
- [x] MILP scheduler의 불가능 전이 사전 제거 조건이 기존 제약과 완전히 동일한지 증명한다.
- [x] 각 항목은 입력 크기와 phase time이 충분히 큰 경우에만 구현 대상으로 승격한다.

### P3. 폴링, 로그, UI 부하 정리

- [x] RX table message id -> row index cache를 만들 수 있는지 확인한다.
- [x] table rebuild 또는 row 삽입/삭제 시 index 무효화 위치를 찾는다.
- [x] listener 기반 처리와 poller 기반 처리의 역할을 구분하고, 중복 enqueue 방지가 유지되는지 확인한다.
- [x] `log_sig.emit` 빈도와 UI event queue 지연을 측정한다.
- [x] stage 단위 로그 버퍼링이 로그 순서와 누락 없이 가능한지 확인한다.
- [x] simulation 0401 JSONL/JSON 배열 로그와 monitoring snapshot save의 중복 I/O가 재계획 지연에 영향을 주는지 측정한다.

### P4. 전수탐색/스케줄러 구조 개선 검토

- [x] next-collab/area planner의 `permutations`/`combinations` 전수 탐색 입력 크기와 시간 분포를 계측한다.
  - metric:
    - `mission_planning.next_collab.prediction_assignment_permutation`
    - `mission_planning.next_collab.assignment_path1_comb_perm`
    - `mission_planning.next_collab.assignment_path2_comb_perm`
    - `mission_planning.next_area.prediction_assignment_permutation`
    - `mission_planning.split_runner.takeover_permutation`
- [x] 결과 동일성을 보장하는 pruning만 먼저 적용한다.
  - next-collab Path1/Path2에서 기존 dense permutation 중 `(aircraftID, targetKey)` 후보가 없는 조합만 생성 전 제거한다.
  - valid permutation의 상대 순서는 기존 `itertools.permutations` dense-filter 결과와 같게 유지한다.
  - metric `pruned_candidates`로 제거된 invalid dense 후보 수를 기록한다.
- [x] Hungarian/min-cost matching 대체는 tie-break와 objective가 완전히 동일함을 증명하기 전까지 실험 브랜치로만 둔다.
  - 결론: production 대체는 미적용한다.
  - 이유: 현재 Path1/Path2는 tuple objective, sparse candidate map, 기존 permutation 순서 기반 tie-break가 결합되어 있어 외부 matching 알고리즘으로 바꾸면 동률 선택이 바뀔 수 있다.
- [x] MILP sparse transition table은 제약 생성 수, solve time, objective equality를 함께 비교한다.
  - metric:
    - `mission_planning.milp.legacy.model_build`
    - `mission_planning.milp.legacy.solve`
    - `mission_planning.milp.barrier.model_build`
    - `mission_planning.milp.barrier.solve`
    - `mission_planning.milp.run_result`
  - 결론: 기존 feasible 전이를 안전하게 제거할 판정식이 아직 없으므로 sparse table은 미적용한다. 현재는 dense transition 수, 변수/제약 수, solve time, objective를 기록해 실험 브랜치와 비교할 수 있게 했다.

## 4. 개발 세션 분할 권고안

권고: 기본 개발은 총 15회 세션으로 나누는 것이 적절하다. 여기서 1회 세션은 "범위 확정 -> 구현/계측 -> replay/동일성 확인 -> 결과 기록"까지 끝나는 작업 단위다. 일정이 빡빡하면 10회로 압축할 수 있지만, 이 코드는 재계획 trigger, queue 상태 전이, sidecar transport, mission output이 강하게 얽혀 있으므로 15회가 더 안전하다.

추가로 P4의 전수탐색/MILP 알고리즘 대체까지 실제 구현 대상으로 삼으면 선택 연구 3회를 별도로 잡는다. 이 3회는 기본 속도 개선 범위에 넣지 않는 것을 권장한다.

총 18회를 배정하면 기본 15회와 선택 연구 3회를 모두 포함하므로, 현재 TODO의 실행/검증/기록 범위를 끝까지 완료하는 계획으로 본다. 단, 선택 연구에서 결과 동일성을 증명하지 못한 알고리즘 대체는 "미적용 결론"도 완료로 인정한다.

### 기본 15회 세션

1. 세션 1: 범위 잠금과 replay corpus 확보
   - 확인: `0201`, `0202`, `0401`, `0402`, `0802`, `0803 execute=1`, `0803 execute=2` 대표 입력을 고정한다.
   - 산출: replay 입력, 비교 제외 필드 규칙, baseline 결과 저장 위치.
   - 완료 기준: 동일 입력을 반복 실행해 baseline이 안정적으로 재현된다.

2. 세션 2: P0 공통 계측 추가
   - 확인: `[REPLAN][TIME]`, `timingMs`, `_copy_metrics`, `0401TRACE`로 부족한 phase를 찾는다.
   - 해볼 것: JSON load/write, deepcopy, 0902 sidecar, 0401 snapshot, queue enqueue/dispatch 계측을 추가한다.
   - 완료 기준: 코드 의미 변화 없이 병목 phase별 ms/count/bytes가 나온다.

3. 세션 3: 0401 중복 parse/signature/save baseline 분석
   - 확인: `_payload_signature`, `parse_payload`, `extract_0401_agent_states`, snapshot save의 중복 비용.
   - 해볼 것: 실제 고빈도 0401에서 parse, canonical dump, snapshot read/write 시간을 분리 측정한다.
   - 완료 기준: 0401 fast path 구현 대상과 제외 대상을 수치로 확정한다.

4. 세션 4: 0401 dispatch context 도입
   - 확인: 기존 coordinator API와 fallback 호출 경로.
   - 해볼 것: raw payload, parsed dict, canonical signature, extracted states, settings snapshot을 묶은 context를 선택적으로 전달한다.
   - 완료 기준: trigger 발생 여부, reason, queue 순서가 baseline과 동일하다.

5. 세션 5: 0401 snapshot/RX listener-poller 비용 정리
   - 확인: listener 기반 처리와 poller 기반 처리의 중복 감시, snapshot 이전 파일 read/write.
   - 해볼 것: row index cache, snapshot write 최적화, 중복 enqueue 방지 유지 여부를 검토한다.
   - 완료 기준: 고빈도 0401 stress에서 누락/중복 replan 없이 handler 시간이 줄어든다.

6. 세션 6: DB JSON cache와 folder id index
   - 확인: `load_db_json`, `_resolve_db_json_path`, fallback glob/read 발생 횟수.
   - 해볼 것: per-dispatch JSON cache와 mtime/size 기반 folder id index를 도입한다.
   - 완료 기준: path/quality/imaging 후보가 많은 0401에서 JSON read 횟수와 ms가 감소한다.

7. 세션 7: 공통 source artifact resolver/index
   - 확인: path deviation, quality speed, imaging schedule의 `_resolve_source_artifacts` 중복 로딩.
   - 해볼 것: `aircraft_id`, `mission_id`, `flightPathID`, `waypoint_id` index를 한 번 만든다.
   - 완료 기준: 세 coordinator의 0902 payload가 baseline과 동일하고 source artifact load 시간이 줄어든다.

8. 세션 8: replan settings snapshot과 queue deepcopy 비용 축소
   - 확인: `load_replan_settings()` cache hit deepcopy, `ReplanQueueManager` copy metrics.
   - 해볼 것: 0401/queue 처리 단위 settings snapshot, 필요한 필드 중심 copy 후보를 적용한다.
   - 완료 기준: 설정 값과 queue merge/suppression 결과가 동일하고 deepcopy ms가 감소한다.

9. 세션 9: Mission planning 입력 JSON/cache 경로 정리
   - 확인: `SourceArtifactCache.read_json(copy_result=True)` 호출 중 읽기 전용 지점.
   - 해볼 것: 전역 변경 없이 읽기 전용으로 증명된 호출만 `copy_result=False` 또는 cache handle 전달로 바꾼다.
   - 완료 기준: 일반 재계획과 특수 재계획 출력이 동일하고 입력 JSON load/deepcopy 시간이 줄어든다.

10. 세션 10: JSON write batch와 unchanged read-back 최적화
    - 확인: `write_json_batch`, `write_json_bytes`, 기존 파일 read-back 비용.
    - 해볼 것: 공용 batch write에 pre-serialized bytes/병렬 write 가능성을 추가한다.
    - 완료 기준: 파일 내용 동일성 유지, write ms 감소, read-back 생략 조건 검증.

11. 세션 11: 0902 transport fast path
    - 확인: sidecar save/read/merge, `replanDetail` JSON stringify, CLR 변환 비용.
    - 해볼 것: sidecar 제거 없이 canonical JSON 재사용, compact 저장, 중복 write 억제를 검토한다.
    - 완료 기준: receiver merge 후 rich field 손실이 없고 0902 enqueue-to-planning start 시간이 줄어든다.

12. 세션 12: 0303/terrain/lineSearch targeted 개선
    - 확인: `generateLineSearchMs`, `groundPrepassMs`, `jsonReadyMs`, `innerParallelWorkers`.
    - 해볼 것: 계측상 큰 phase에 한해 run-local ground context, metric 직렬화 중복 제거를 적용한다.
    - 완료 기준: 0303 출력 동일성 유지, targeted phase median 개선.

13. 세션 13: queue UI, 로그, 폴링 부하 정리
    - 확인: `ReplanQueueTab` full rebuild, `log_sig.emit`, RX table scan이 실제 지연에 미치는 영향.
    - 해볼 것: item id/version diff update, row index cache, stage log buffering을 적용 가능한 범위에서 정리한다.
    - 완료 기준: UI 표시 의미와 로그 순서 유지, 0401/0305/0701 고빈도 처리 지연 감소.

14. 세션 14: 낮은 위험의 선형 탐색/index 개선
    - 확인: 입력 크기가 큰 waypoint/aircraft/ID allocator 경로만 추린다.
    - 해볼 것: `waypoint_id -> index`, `aircraft_id -> entry`, reservation 단위 directory snapshot을 적용한다.
    - 완료 기준: 작은 입력에서 악화 없음, 큰 입력에서 phase ms 감소, output 동일.

15. 세션 15: 통합 회귀, 성능 리포트, TODO 갱신
    - 확인: 모든 대표 시나리오의 trigger/order/payload/output 동일성.
    - 해볼 것: 전후 성능표, 악화 항목, 미적용 항목, 다음 후보를 문서화한다.
    - 완료 기준: targeted phase median 20% 이상 또는 end-to-end median 10% 이상 개선, 비대상 p95 5% 초과 악화 없음.

### 선택 연구 3회

1. 선택 세션 A: next-collab/area planner 전수탐색 입력 규모 분석
   - 목적: `permutations`/`combinations`가 실제 병목인지 확인한다.
   - 권고: 결과 동일성 보장 pruning만 실험한다.

2. 선택 세션 B: MILP sparse transition table 검증
   - 목적: 기존 제약과 완전히 동일한 불가능 전이 제거만 가능한지 증명한다.
   - 권고: objective, assignment, solve status가 모두 동일할 때만 채택한다.

3. 선택 세션 C: 알고리즘 대체 실험
   - 목적: Hungarian/min-cost matching 등 대체 알고리즘의 tie-break와 objective 동일성을 검증한다.
   - 권고: 동일성 증명 전에는 본선 개발에 병합하지 않는다.

### 운영 권고

- 세션 1-3은 반드시 먼저 진행한다. 계측 없이 최적화부터 들어가면 기능 변경 위험이 커진다.
- 세션 4-8은 monitoring/queue hot path라 재계획 시작 지연에 직접 영향을 준다. 가장 먼저 실제 효과를 볼 가능성이 높다.
- 세션 9-12는 mission planning wall-clock을 줄이는 구간이다. output 동일성 검증 비용이 커서 한 세션에 여러 항목을 섞지 않는다.
- 세션 13-14는 보조 병목이다. P0/P1 효과를 확인한 뒤 진행한다.
- 총 18회 기준은 기본 개선 15회와 선택 연구 3회까지 포함한 넉넉한 완료 계획이다. 18회 안에서는 구현뿐 아니라 py_compile, probe/replay, TODO 갱신까지 매 세션 완료 항목으로 포함한다.
- P4 알고리즘 대체는 성능 욕심으로 본선에 섞지 않는다. 결과가 조금이라도 달라지면 기능 변경이다.

### 진행 기록

- [x] 세션 1: 범위 잠금
  - 대상 폴더 `modules_사천시연_최적화_ver2` 내부만 수정하는 것으로 고정했다.
  - 새 라이브러리는 추가하지 않았다.
  - 재계획 trigger, queue 판정, 파라미터, output 생성 로직은 변경하지 않았다.

- [x] 세션 2: opt-in 공통 계측 기반 추가
  - 추가: `common/replan_perf.py`
  - 환경변수 `DSS_REPLAN_PERF=1`일 때만 metric을 집계한다.
  - 환경변수 `DSS_REPLAN_PERF_LOG=<path>`를 주면 프로세스 종료 시 aggregate snapshot을 JSON으로 저장한다.
  - 계측 연결 위치:
    - `monitoring/logic/mission_update.py`: DB JSON resolve/load/save, fallback scan count
    - `mission_planning/runtime/json_io.py`: JSON serialize/write/read-back/batch write
    - `mission_planning/runtime/cache/source_artifacts.py`: cache hit/miss, load ms, deepcopy ms
    - `monitoring/logic/replan_runtime_settings.py`: settings cache hit/miss, deepcopy ms
    - `common/replan_request_transport_store.py`: 0902 sidecar save/load/load-latest
    - `common/agent_status_snapshot.py`: 0401 snapshot previous read, tracking update, write
    - `monitoring/monitoring_gui.py`: payload signature canonicalization
  - 검증:
    - 수정 파일 `py_compile` 통과
    - target-folder fallback import smoke test 통과

- [x] 세션 3: 계측 probe 추가 및 첫 snapshot 확보
  - 추가: `tools/replan_perf_probe.py`
  - probe는 GUI를 띄우지 않고 DB JSON, source artifact cache, JSON write, 0902 sidecar, 0401 snapshot 저장 경로를 실행한다.
  - probe의 모든 write는 `DSS_Internal/replan_perf_probe` 내부로 제한한다.
  - 생성 snapshot: `DSS_Internal/replan_perf_probe/replan_perf_probe_snapshot.json`
  - 첫 probe 결과:
    - metric 16종 수집
    - `monitoring.db.load_json`, `monitoring.db.resolve`
    - `mission_planning.source_artifact.read_json`
    - `mission_planning.json.write_*`
    - `common.replan_sidecar.*`
    - `common.agent_status_snapshot.*`
  - 검증:
    - `tools/replan_perf_probe.py` `py_compile` 통과
    - probe 실행 성공

- [x] 세션 4: 0401 parsed payload 재사용 1차 구현
  - 변경: `monitoring/monitoring_gui.py`
  - 추가: `monitoring/logic/payload_signature.py`
  - `_payload_signature_context()`를 추가해 기존 signature 계산 중 얻은 parsed dict를 함께 반환한다.
  - signature/parsed-body 계산은 pure helper `payload_signature_context()`로 분리했다.
  - `_enqueue_0401_payload()`가 pending payload와 함께 parsed body를 저장한다.
  - `_drain_0401_payload()`가 `_on_rx_0401(..., raw_body=...)`로 전달한다.
  - 직접 `_on_rx_0401(payload)` 호출 경로는 기존처럼 parse한다.
  - `parse` trace phase는 reused 여부만 추가하고 항상 유지한다.
  - 기능 불변성 의도:
    - dedupe signature 계산 방식은 유지
    - 0401 coalesce, pending overwrite, last signature 비교, drain scheduling은 유지
    - trigger/queue/replan payload 생성 로직은 변경하지 않음
  - 검증:
    - `monitoring/monitoring_gui.py` `py_compile` 통과

- [x] 세션 5: 0401 parsed payload 재사용 동등성 probe 보강
  - 변경: `tools/replan_perf_probe.py`
  - 검증 항목:
    - bytes 0401 payload에서 `payload_signature_context()`가 보관한 parsed body가 `parse_payload()` 결과와 동일
    - dict payload의 canonical signature가 bytes payload signature와 동일
    - list payload는 기존 동작처럼 latest dict body를 재사용하되, signature fallback은 허용
  - probe snapshot에서 `monitoring.payload_signature` metric 수집을 확인했다.
  - 검증:
    - `monitoring/logic/payload_signature.py` `py_compile` 통과
    - `monitoring/monitoring_gui.py` `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공

- [x] 세션 6: DB JSON cache와 folder id index 1차 구현
  - 변경: `monitoring/logic/mission_update.py`
  - `load_db_json()`에 path/mtime/size 기반 JSON payload cache를 추가했다.
  - cache hit 반환도 기존 caller-owned 동작을 유지하기 위해 deepcopy를 반환한다.
  - direct `ID.json`이 없을 때 folder id index를 만들어 fallback glob/read 반복을 줄인다.
  - folder id index도 file name/mtime/size signature로 무효화한다.
  - `save_db_json()` 이후 해당 path cache와 folder index를 무효화한다.
  - 계측 추가:
    - `monitoring.db.folder_id_index`
    - `monitoring.db.load_json.deepcopy`
    - `monitoring.db.load_json`의 cache hit/miss counter
  - probe 보강:
    - direct file이 없는 `mission_plan_alias.json`의 `missionPlanID=999` fallback resolve 확인
    - cache hit 이후 반환 객체 mutation이 다음 load에 전파되지 않는지 확인
  - 검증:
    - `monitoring/logic/mission_update.py` `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공
    - probe snapshot에서 `monitoring.db.folder_id_index`, `cache_hit`, `fallback_hit` 확인

- [x] 세션 7: path/quality/imaging source artifact index 1차 구현
  - 추가: `monitoring/logic/source_artifact_index.py`
  - 변경:
    - `monitoring/logic/path_deviation_replan.py`
    - `monitoring/logic/quality_speed_replan.py`
    - `monitoring/logic/imaging_schedule_replan.py`
  - 공통화 범위:
    - source MissionPlan load
    - aircraftID -> aircraft entry
    - aircraftID -> individualMissionPackageID
    - individualMissionPackageID -> individualMissionList
    - pathID -> waypoint id set
    - pathID/currentWaypointID -> waypoint dict
  - 기능 불변성 의도:
    - 각 coordinator의 반환 dataclass는 유지
    - inputMissionID fallback 규칙은 유지
    - trigger 조건, random roll, suppression, replan payload 구조는 변경하지 않음
  - probe 보강:
    - `SourceArtifactIndex.from_source_plan(100)` 생성 확인
    - inputMissionPackageID, aircraft->IMP, waypoint id set, waypoint dict lookup 확인
  - 검증:
    - 세 coordinator와 `source_artifact_index.py` `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공

- [x] 세션 8: replan settings snapshot과 queue deepcopy 계측/축소 1차
  - 변경:
    - `monitoring/logic/replan_runtime_settings.py`
    - `monitoring/logic/replan_queue_manager.py`
    - `tools/replan_perf_probe.py`
  - settings 개선:
    - `load_replan_settings()`의 기존 full mutable copy 반환 계약은 유지했다.
    - `get_*_settings()`/`get_replan_group()`/`get_replan_toggles()`는 cached root를 재사용하고 필요한 group만 deepcopy하도록 변경했다.
    - group 반환 객체 mutation이 cache에 전파되지 않는지 probe로 확인했다.
  - queue deepcopy:
    - 기존 `ReplanQueueManager._copy_metrics`는 유지했다.
    - 같은 deepcopy 비용을 `replan_perf` aggregate metric에도 기록하도록 추가했다.
    - probe에서 `monitoring.replan_queue.deepcopy`, `monitoring.replan_queue.deepcopy.enqueue_payload`, `monitoring.replan_queue.deepcopy.result_dispatch_payload` 수집 확인.
  - 검증:
    - `monitoring/logic/replan_runtime_settings.py` `py_compile` 통과
    - `monitoring/logic/replan_queue_manager.py` `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공

- [x] 세션 9: Mission planning 입력 JSON/cache 경로 정리 1차
  - 변경:
    - `mission_planning/runtime/cache/source_artifacts.py`
    - `tools/replan_perf_probe.py`
  - active `SourceArtifactCache`가 없는 경로에서도 `copy_result=True` 호출은 mtime/size 기반 fallback cache를 재사용하도록 추가했다.
  - caller-owned 동작 보존을 위해 fallback cache hit도 deepcopy를 반환한다.
  - `copy_result=False`의 no-active 경로는 기존처럼 fresh parse를 유지해 전역 공유 객체 위험을 피했다.
  - 계측 추가:
    - `mission_planning.source_artifact.read_json.no_active_cache`
  - probe 보강:
    - no-active cached read 결과를 mutation해도 다음 read에 전파되지 않는지 확인
  - 검증:
    - `mission_planning/runtime/cache/source_artifacts.py` `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공
    - probe snapshot에서 `mission_planning.source_artifact.read_json.no_active_cache` 확인

- [x] 세션 10: JSON write batch opt-in 병렬 경로 1차
  - 변경:
    - `mission_planning/runtime/json_io.py`
    - `tools/replan_perf_probe.py`
  - 기본 동작은 기존과 같이 sequential write로 유지했다.
  - `DSS_REPLAN_JSON_WRITE_WORKERS>=2`일 때만 `write_json_batch()`가 unique path batch를 병렬 write로 처리할 수 있게 했다.
  - 동일 path가 batch에 중복되면 기존 순차 경로로 fallback해 overwrite 순서 의미를 보존한다.
  - 병렬 경로에서도 반환 row 형태는 기존 `path`, `name`, `written`, `skipped`만 유지한다.
  - 직렬화 bytes/시간 정보는 반환값이 아니라 opt-in `replan_perf` metric으로만 기록한다.
  - 계측 추가:
    - `mission_planning.json.write_json_batch.serialize`
    - `mission_planning.json.write_json_batch` metadata: `workers`, `parallel`
  - probe 보강:
    - worker 2개 설정에서 unique path 2개 write 성공 확인
    - snapshot에서 `workers=2`, `parallel=1` 확인
  - 검증:
    - `mission_planning/runtime/json_io.py` `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공
    - 반환 row에 `payloadBytes`, `serializeMs`가 노출되지 않는지 확인

- [x] 세션 11: 0902 transport fast path 1차
  - 변경:
    - `common/replan_request_transport_store.py`
    - `common/push/message0902_push.py`
    - `common/receive/message0902_receiver.py`
    - `tools/replan_perf_probe.py`
  - sidecar 저장 개선:
    - timestamp path의 entries를 mtime/size 기반으로 cache한다.
    - 동일 identity payload가 이미 있고 현재 mode의 직렬화 결과가 파일 내용과 같으면 atomic replace를 생략한다.
    - write 성공 후 cache를 최신 entries/text로 갱신한다.
  - sidecar load 개선:
    - `load_payload()`와 `load_latest_payload()`가 같은 파일을 반복 JSON parse하지 않도록 cache hit을 사용한다.
    - 반환 payload는 deepcopy해서 caller mutation이 cache에 남지 않게 했다.
  - 0902 계측 보강:
    - `common.replan_sidecar.save.read`
    - `common.replan_sidecar.load_latest.read`
    - `common.replan_0902_push.sidecar_save_call`
    - `common.replan_0902_push.detail_json`
    - `common.replan_0902_push.dict_to_obj`
    - `common.replan_sidecar.receiver_merge`
  - 기능 불변성 의도:
    - sidecar off/compact/pretty mode는 유지
    - sidecar 제거 없이 rich field 복원 안전장치를 유지
    - receiver merge 규칙은 transport payload 우선, base payload fallback으로 유지
  - probe 보강:
    - 동일 payload 2회 저장 시 파일 mtime/size가 변하지 않는지 확인
    - duplicate save metric `skipped_duplicate` 확인
    - `load_payload()` 반환 객체의 nested `replanDetail` mutation이 다음 load에 전파되지 않는지 확인
    - `load_latest_payload()`가 같은 rich payload를 반환하는지 확인
  - 검증:
    - `common/replan_request_transport_store.py` `py_compile` 통과
    - `common/push/message0902_push.py` `py_compile` 통과
    - `common/receive/message0902_receiver.py` `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공
    - probe snapshot에서 `skipped_duplicate`, sidecar read `cache_hit` 확인

- [x] 세션 12: 0303/terrain/lineSearch targeted 개선 후보 1차
  - 변경:
    - `mission_planning/runtime/aircraft_parallel_0303.py`
    - `tools/replan_perf_probe.py`
  - dense metric merge 개선:
    - `_merge_dense_linesearch_metrics()` 내부에서 반복 생성하던 metric key set을 module-level `frozenset` 상수로 이동했다.
    - max/reason/float metric 병합 규칙은 기존과 동일하게 유지했다.
  - lineSearch metric 계측:
    - `_summarize_line_search_counts()`의 `lineSearchJsonBytes` 계산 값은 유지했다.
    - JSON bytes 계산 시간이 실제 병목인지 확인할 수 있도록 opt-in metric을 추가했다.
    - 추가 metric:
      - `mission_planning.0303.line_search_summary`
      - `mission_planning.0303.line_search_summary.json_serialize`
  - 기능 불변성 의도:
    - FlightPath/waypoint/lineSearch 생성 로직은 변경하지 않음
    - `lineSearchJsonBytes` 계산 방식과 로그 값은 유지
    - dense metric merge 결과의 max/sum/reason 결합 규칙 유지
  - probe 보강:
    - lineSearch count/coord/json bytes 요약 확인
    - dense metric max/sum/reason merge 결과 확인
  - 검증:
    - `mission_planning/runtime/aircraft_parallel_0303.py` `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공
    - probe snapshot에서 `mission_planning.0303.line_search_summary*` 확인

- [x] 세션 13: queue UI, 로그, RX table polling 부하 정리 1차
  - 변경:
    - `monitoring/monitoring_gui.py`
    - `monitoring/gui/tabs/replan_queue_tab.py`
  - RX/TX table row lookup 개선:
    - `MainWindow._find_message_row()` 공통 helper를 추가했다.
    - `tbl_rx`/`tbl_tx` message id -> row index를 cache한다.
    - cache hit도 해당 row의 message id 셀을 다시 검증하므로 row reorder, rebuild, 교체 시 자동으로 scan fallback한다.
    - 0101/0305/0701/0001/0903/0702/0201/0202/0401/0402/0802/0803 poller의 반복 full scan을 `_find_rx_row()`로 치환했다.
  - queue UI 계측:
    - `ReplanQueueTab.set_snapshot()`의 copy 비용을 기존 내부 perf 외에 `replan_perf` metric으로도 기록한다.
    - `_refresh_view()`와 `_rebuild_list()`의 widget rebuild 비용, removed/added widget count를 opt-in metric으로 기록한다.
  - queue snapshot 계측:
    - `monitoring.replan_queue.snapshot_build`
    - `monitoring.replan_queue.tab_set_snapshot`
  - log emit 계측:
    - `_append_log_line()`의 UI thread/queued-to-UI 호출 빈도와 append 시간을 `monitoring.log.append_request`, `monitoring.log.append_ui`로 기록한다.
  - 기능 불변성 의도:
    - listener/poller trigger 조건, last raw 비교, timestamp 비교, coalesce 흐름은 변경하지 않음
    - 로그 버퍼링은 아직 적용하지 않고 계측만 추가해 순서/누락 위험을 피함
    - queue UI 표시 내용과 refresh throttle 값은 유지
  - 검증:
    - `monitoring/monitoring_gui.py` `py_compile` 통과
    - `monitoring/gui/tabs/replan_queue_tab.py` `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공

- [x] 세션 14: 낮은 위험의 선형 탐색/index 개선 1차
  - 변경:
    - `mission_planning/replanning/triggers/next_collab/pipeline.py`
    - `monitoring/gui/tabs/monitoring_visualization_tab.py`
  - next-collab ID dedupe 개선:
    - `_reserve_unique_individual_ids()`에서 `reserved` list membership 반복을 `reserved_seen` set으로 대체했다.
    - 반환 리스트 순서와 catch-up reservation 흐름은 유지했다.
  - visualization target aircraft dedupe 개선:
    - execute-next context 생성 중 `target_aircraft_ids`와 `current_input_aircraft_ids` 중복 검사를 set으로 병행했다.
    - append 순서는 유지해 downstream payload/log의 aircraft 순서를 바꾸지 않았다.
  - 확인:
    - `remaining_hybrid/current.py`에는 TODO에서 지적한 `target_aircraft_ids.index(...)` 직접 반복이 현재 없음을 확인했다.
    - ID allocator는 이미 `reserve_replan_id_bundle()`, `reserve_path_id_blocks()`, waypoint usage fast path를 갖고 있어 이번 세션에서는 ID 배분 구조를 건드리지 않았다.
  - 기능 불변성 의도:
    - ID reservation count, catch-up 조건, returned id order 유지
    - target aircraft output order 유지
    - mission/flightPath/waypoint 생성 로직 불변
  - 검증:
    - `mission_planning/replanning/triggers/next_collab/pipeline.py` `py_compile` 통과
    - `monitoring/gui/tabs/monitoring_visualization_tab.py` `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공

- [x] 세션 15: 통합 smoke 회귀와 성능 snapshot 리포트 1차
  - 변경:
    - `TODO_replan_speed_optimization.md`
  - 통합 검증:
    - 변경 파일 전체 `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공
    - snapshot: `DSS_Internal/replan_perf_probe/replan_perf_probe_snapshot.json`
  - 컴파일 대상:
    - `common/replan_perf.py`
    - `common/agent_status_snapshot.py`
    - `common/replan_request_transport_store.py`
    - `common/push/message0902_push.py`
    - `common/receive/message0902_receiver.py`
    - `mission_planning/runtime/json_io.py`
    - `mission_planning/runtime/cache/source_artifacts.py`
    - `mission_planning/runtime/aircraft_parallel_0303.py`
    - `mission_planning/replanning/triggers/next_collab/pipeline.py`
    - `monitoring/logic/mission_update.py`
    - `monitoring/logic/payload_signature.py`
    - `monitoring/logic/source_artifact_index.py`
    - `monitoring/logic/path_deviation_replan.py`
    - `monitoring/logic/quality_speed_replan.py`
    - `monitoring/logic/imaging_schedule_replan.py`
    - `monitoring/logic/replan_runtime_settings.py`
    - `monitoring/logic/replan_queue_manager.py`
    - `monitoring/monitoring_gui.py`
    - `monitoring/gui/tabs/replan_queue_tab.py`
    - `monitoring/gui/tabs/monitoring_visualization_tab.py`
    - `tools/replan_perf_probe.py`
  - probe 기준 성능 snapshot 요약:
    - `monitoring.payload_signature`: count=3, totalMs=0.115
    - `monitoring.db.load_json`: count=25, totalMs=7.341, cache_hit=21, cache_miss=4
    - `monitoring.db.resolve`: count=26, totalMs=2.583, fallback_hit=6
    - `monitoring.db.folder_id_index`: count=6, totalMs=1.826, cache_hit=5, cache_miss=1
    - `monitoring.replan_settings.load_cached`: count=2, totalMs=1.404, cache_hit=1, cache_miss=1
    - `monitoring.replan_queue.deepcopy`: count=2, totalMs=0.009
    - `mission_planning.source_artifact.read_json.no_active_cache`: count=2, totalMs=0.296
    - `mission_planning.json.write_json_batch`: count=1, totalMs=2.061, workers=2, parallel=1
    - `common.replan_sidecar.save`: count=2, totalMs=1.529, skipped_duplicate=1
    - `common.replan_sidecar.load.read`: count=2, totalMs=0.211, cache_hit=2
    - `common.agent_status_snapshot.save`: count=1, totalMs=1.197
    - `mission_planning.0303.line_search_summary.json_serialize`: count=1, totalMs=0.013
  - 아직 완료로 보지 않는 항목:
    - 5개 대표 재계획 replay corpus 고정
    - 변경 전 산출물 hash/equality baseline
    - 고빈도 0401 stress에서 queue/order/snapshot 동일성 검증
    - receiver merge 결과의 rich field 완전 동일성 replay
    - `d0303.py` 내부 waypoint scan/DEM/ground cache hit ratio 정밀 계측
    - 전수탐색/MILP/알고리즘 대체의 동일성 증명
  - 판단:
    - 현재까지는 기능 변경 없이 hot path 계측과 저위험 cache/index 개선을 적용한 상태다.
    - 실제 end-to-end median 10% 또는 targeted phase 20% 개선은 probe만으로 증명할 수 없으므로, 대표 replay 확보 전까지 완료 기준 충족으로 보지 않는다.

- [x] 세션 16: 선택 세션 A - next-collab/area planner 전수탐색 입력 규모 계측
  - 변경:
    - `mission_planning/MissionPlanner/planning_enhanced/algo/split_runner.py`
    - `mission_planning/next_area_mode/planner_window.py`
    - `mission_planning/planners/next_collab_division/_planner_window.py`
    - `tools/replan_perf_probe.py`
    - `TODO_replan_speed_optimization.md`
  - 확인된 전수탐색 hot spot:
    - next-collab headless area replanning: `_assign_split_result_by_prediction_distance()`
      - 입력 규모: `P(usable_uavs, target_items)`
      - metric: `mission_planning.next_collab.prediction_assignment_permutation`
    - next-collab assignment path 1: `_assignment_path_1()`
      - 입력 규모: `C(active_uavs, assign_count) * P(target_keys, assign_count)`
      - metric: `mission_planning.next_collab.assignment_path1_comb_perm`
    - next-collab assignment path 2: `_assignment_next_mission()`
      - 입력 규모: `C(active_uavs, assign_count) * P(target_keys, assign_count)`
      - metric: `mission_planning.next_collab.assignment_path2_comb_perm`
    - next-area/line planner assignment: `_assign_split_result_by_prediction_distance()`
      - 입력 규모: `P(usable_uavs, target_items)`
      - metric: `mission_planning.next_area.prediction_assignment_permutation`
    - shared enhanced split assignment: `_assign_group_by_takeover_distance()`
      - 입력 규모: `P(usable_uavs, pieces)`, 기존 guard는 `pieces <= usable_uavs <= 8`
      - metric: `mission_planning.split_runner.takeover_permutation`
  - sub agent 재검토:
    - Ptolemy가 production `permutations`/`combinations` hot spot 5개를 확인했다.
    - Averroes가 active headless next-collab area path는 shared takeover-distance helper가 아니라 prediction-distance assignment를 사용한다는 점을 확인했다.
    - 따라서 shared takeover helper의 최적화만으로는 active area replanning 개선 효과가 작을 가능성이 높고, 실제 후보는 next-collab prediction assignment와 path1/path2 조합 loop다.
  - 기능 불변성 의도:
    - loop 내부 score 계산, best 갱신 조건, strict `<` tie-break, fallback greedy branch를 변경하지 않았다.
    - `DSS_REPLAN_PERF`가 켜진 경우에만 elapsed/candidate counter를 누적한다.
    - per-candidate 파일 I/O나 로그 출력은 넣지 않았다.
  - 검증:
    - 변경 파일 `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공
    - probe snapshot에서 `mission_planning.split_runner.takeover_permutation` 확인:
      - count=1
      - estimated_candidates=6
      - checked_candidates=6
  - 판단:
    - P4 첫 항목인 전수탐색 입력 크기/시간 분포 계측은 완료했다.
    - pruning 또는 Hungarian/min-cost 대체는 아직 output tie-break 동일성 증명이 없으므로 미적용이다.
    - headless area replanning에서 UAV/target 수가 8 이상으로 커지면 `C(A,K) * P(G,K)`가 급증할 수 있으므로, 선택 세션 C에서 동일성 보장 대체 실험을 별도로 진행한다.

- [x] 세션 17: 선택 세션 B - MILP sparse transition table 검증
  - 변경:
    - `mission_planning/MissionPlanner/planning_enhanced/scheduling/milp_scheduler.py`
    - `tools/replan_perf_probe.py`
    - `TODO_replan_speed_optimization.md`
  - 추가 계측:
    - legacy solver:
      - `mission_planning.milp.legacy.model_build`
      - `mission_planning.milp.legacy.solve`
    - barrier solver:
      - `mission_planning.milp.barrier.model_build`
      - `mission_planning.milp.barrier.solve`
    - 공통 run result:
      - `mission_planning.milp.run_result`
  - 기록되는 값:
    - slot 수, UAV 수, candidate 수, max candidates per slot
    - dense transition 수
    - `x`, `w`, `q`, `y0`, `r` 변수 수
    - 전체 variable/constraint 수
    - solve time, optimal/fallback 여부, objective value
  - probe 기준 snapshot:
    - `mission_planning.milp.barrier.model_build`
      - slots=2, uavs=2, candidates=4, dense_transitions=8
      - variables=134, constraints=138
      - q_variables=8, y0_variables=28, r_variables=56
    - `mission_planning.milp.barrier.solve`
      - totalMs=211.314, optimal=1, fallback=0
    - `mission_planning.milp.run_result`
      - objective_value=185.5802568743929, used_greedy=0
  - 판단:
    - 현재 MILP는 모든 candidate transition을 objective와 arrival constraint에 사용한다.
    - 기존 코드 안에는 "불가능 전이"를 별도로 판정하는 predicate가 없으므로 sparse transition table을 바로 적용하면 objective/tie-break가 바뀔 수 있다.
    - 따라서 이번 세션에서는 기능 변경 없이 dense baseline 계측만 적용하고, sparse transition 제거는 미적용 결론으로 둔다.

- [x] 세션 18: 선택 세션 C - 전수탐색 pruning/Hungarian-min-cost 대체 실험
  - 변경:
    - `mission_planning/runtime/assignment_search.py`
    - `mission_planning/planners/next_collab_division/_planner_window.py`
    - `tools/replan_perf_probe.py`
    - `TODO_replan_speed_optimization.md`
  - 적용한 pruning:
    - Path1/Path2의 `itertools.permutations(target_keys, assign_count)` 전체 생성 대신, `(aircraftID, targetKey)` 후보가 존재하는 valid permutation만 생성한다.
    - 기존 loop에서 `candidate is None`으로 버리던 invalid 조합만 제거한다.
    - valid permutation의 상대 순서는 기존 dense permutation 후 filter한 결과와 동일하다.
  - 유지한 동작:
    - `itertools.combinations(active_uavs, assign_count)` 순서 유지
    - candidate row lookup, weighted/horizon/segment sum 계산 유지
    - `score < best_score` strict tie-break 유지
    - no-record fallback/경고 흐름 유지
  - Hungarian/min-cost matching 판단:
    - production 대체는 미적용한다.
    - Path1/Path2 objective는 tuple score와 기존 permutation order tie-break가 결합되어 있어 일반 matching solver로 바꾸면 동률 선택이 달라질 수 있다.
  - 검증:
    - `mission_planning/runtime/assignment_search.py` `py_compile` 통과
    - `mission_planning/planners/next_collab_division/_planner_window.py` `py_compile` 통과
    - `mission_planning/MissionPlanner/planning_enhanced/scheduling/milp_scheduler.py` `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공
    - probe에서 dense-filter permutation 결과와 pruning generator 결과가 같은 순서임을 확인했다.
  - 판단:
    - P4의 전수탐색 관련 실제 적용은 "불가능 조합 생성 제거"까지로 제한한다.
    - 알고리즘 대체는 동일성 증명 전까지 적용하지 않는 결론으로 완료한다.

다음 작업:
- 대표 replay corpus와 변경 전 산출물 hash/equality baseline을 확보한다.
- 고빈도 0401 stress와 0902 receiver merge golden replay를 수행한다.
- 실제 scenario snapshot으로 P0/P1/P4 계측값을 수집해 다음 구현 승격 대상을 정한다.

18회 범위 최종 감사:
- 통과:
  - 변경 Python 파일 전체 `py_compile` 통과
  - `tools/replan_perf_probe.py` 실행 성공
  - probe snapshot: `DSS_Internal/replan_perf_probe/replan_perf_probe_snapshot.json`
- probe snapshot 주요 추가 metric:
  - `mission_planning.split_runner.takeover_permutation`
  - `mission_planning.milp.barrier.model_build`
  - `mission_planning.milp.barrier.solve`
  - `mission_planning.milp.run_result`
- 18회 완료로 인정하는 범위:
  - 공통 opt-in 계측 기반
  - monitoring DB/cache/settings/queue/source artifact 계측 및 저위험 cache/index 개선
  - mission planning JSON/source artifact/sidecar/0303 targeted 계측 및 저위험 개선
  - queue UI/RX row scan/선형 탐색 일부 개선
  - P4 전수탐색 계측, valid permutation pruning, MILP dense baseline 계측, 알고리즘 대체 미적용 결론
- 18회 후에도 남는 미완료 범위:
  - 대표 재계획 replay corpus 5개 고정
  - 변경 전 산출물 hash/equality baseline
  - 고빈도 0401 stress에서 queue/order/snapshot 동일성 검증
  - 0902 receiver merge rich field golden replay
  - `d0303.py` 내부 waypoint scan/DEM/ground cache hit ratio 정밀 계측
  - `copy_result=False`, full-plan deepcopy, pre-serialized bytes API 등 결과 동일성 검증이 필요한 구조 변경

- [x] 세션 19: P0 replay manifest와 hash/equality baseline 고정
  - 변경:
    - `tools/replan_replay_baseline.py`
    - `DSS_Internal/replay_baseline/replay_manifest.json`
    - `DSS_Internal/replay_baseline/comparison_rules.json`
    - `DSS_Internal/replay_baseline/structural_baseline.json`
    - `TODO_replan_speed_optimization.md`
  - 실행:
    - `python tools/replan_replay_baseline.py`
  - 결과:
    - Random_mission database에서 replay 후보 47개를 스캔했다.
    - 대표 replay manifest 5개를 고정했다.
      - `0401_high_frequency_path_quality_imaging`
      - `0402_target_detection_attack`
      - `0402_attack_closed_destroyed_rejoin`
      - `0803_execute_next_collab`
      - `0202_prior_or_0401_prior_resume`
    - 각 replay 항목은 generator 파일, Random_mission scenario/input/target source, event plan, source hash를 포함한다.
    - structural baseline은 21개 JSON 파일을 기록한다.
    - baseline에는 probe DB의 `MissionPlan`, `IndividualMissionPlan`, `FlightPath` 샘플도 포함한다.
  - 비교 규칙:
    - `canonicalJsonSha256`: 전체 stable JSON hash
    - `volatileExcludedSha256`: timestamp, trace/log, timing field 제외 hash
    - `idRelaxedSha256`: volatile 제외 후 generated mission/path/waypoint ID 완화 hash
  - 검증:
    - `tools/replan_replay_baseline.py` `py_compile` 통과
    - baseline 생성 성공
    - `structural_baseline.json`에서 `MissionPlan`, `IndividualMissionPlan`, `FlightPath` 샘플 포함 확인
  - 판단:
    - P0의 "대표 시나리오 고정"과 "hash/equality baseline 저장"은 완료했다.
    - 실제 trigger/order/golden replay 실행 검증은 아직 남아 있으며, 다음 세션에서 이 manifest를 입력으로 사용한다.

- [x] 세션 20: P0 timing evidence report 생성
  - 변경:
    - `tools/replan_timing_report.py`
    - `DSS_Internal/timing_report/timing_report.json`
    - `TODO_replan_speed_optimization.md`
  - 실행:
    - `python tools/replan_timing_report.py`
  - 결과:
    - `DSS_Internal/replan_perf_probe/replan_perf_probe_snapshot.json`에서 34개 metric을 확인했다.
    - source timing marker:
      - `[REPLAN][TIME]`: 4 occurrences
      - `timingMs`: 26 occurrences
      - `_copy_metrics`/`copy_metrics`: 12 occurrences
      - `[0401TRACE]`: 3 occurrences
      - `generateLineSearchMs`/`groundPrepassMs`/`jsonReadyMs`/`innerParallelWorkers`: 21 occurrences
    - available log timing:
      - 현재 대상 폴더 안에서 운영 로그 timing match는 0건이다.
      - 따라서 실제 replay/stress timing 수집은 다음 replay 실행 항목에서 별도 수행해야 한다.
  - 검증:
    - `tools/replan_timing_report.py` `py_compile` 통과
    - timing report 생성 성공
    - 생성 위치가 target 폴더 내부 `DSS_Internal/timing_report/timing_report.json`임을 확인했다.

- [x] 세션 21: `_resolve_source_artifacts` 세부 계측 보강
  - 변경:
    - `monitoring/logic/source_artifact_index.py`
    - `tools/replan_perf_probe.py`
    - `TODO_replan_speed_optimization.md`
  - 추가 metric:
    - `monitoring.source_artifact_index.from_source_plan`
    - `monitoring.source_artifact_index.individual_missions`
    - `monitoring.source_artifact_index.flight_path`
    - `monitoring.source_artifact_index.waypoint_scan`
  - probe snapshot 확인:
    - `monitoring.source_artifact_index.from_source_plan`: read_files=1, aircraft_entries=2
    - `monitoring.source_artifact_index.flight_path`: read_files=1, waypoint_entries=2
    - `monitoring.source_artifact_index.waypoint_scan`: waypoint_scan=2, cache_hit=1, cache_miss=1
  - 기능 불변성 의도:
    - source artifact lookup 결과, cache key, payload copy 정책은 변경하지 않았다.
    - `DSS_REPLAN_PERF`가 켜진 경우에만 metric을 누적한다.
  - 검증:
    - `monitoring/logic/source_artifact_index.py` `py_compile` 통과
    - `tools/replan_perf_probe.py` 실행 성공
    - probe assertion에 waypoint scan metric 검증 추가

- [x] 세션 22: RX listener/poller/enqueue 계측 보강
  - 변경:
    - `monitoring/monitoring_gui.py`
    - `TODO_replan_speed_optimization.md`
  - 추가 metric:
    - `monitoring.rx.listener.<msg_id>`
    - `monitoring.rx.poller.scan.<msg_id>`
    - `monitoring.rx.enqueue.0401`
    - `monitoring.rx.enqueue.0402`
  - 확인:
    - 0101, 0305, 0701, 0001, 0903, 0702, 0201, 0202, 0401, 0402, 0802, 0803 listener 수신을 계측한다.
    - poller row lookup은 found/missing counter로 구분한다.
    - 0401/0402 enqueue는 duplicate signature, coalesced pending, scheduled, accepted, skipped counter를 남긴다.
  - 기능 불변성 의도:
    - 수신/중복 판단/타이머 coalesce 조건은 변경하지 않고 opt-in `DSS_REPLAN_PERF` metric만 추가했다.
    - listener는 primary event path, poller는 RX table/legacy fallback path로 유지한다.
  - 검증:
    - `monitoring/monitoring_gui.py` `py_compile` 통과
    - metric call site 존재를 `rg`로 확인

- [x] 세션 23: static hot-path report와 P1/P2/P3 설계 TODO 정리
  - 변경:
    - `tools/replan_static_hotpath_report.py`
    - `DSS_Internal/static_hotpath_report/static_hotpath_report.json`
    - `TODO_replan_speed_optimization.md`
  - 실행:
    - `python tools/replan_static_hotpath_report.py`
  - 결과:
    - Python 파일 605개를 스캔했다.
    - write 호출 59개, source/cache read 호출 55개, deepcopy 호출 582개를 분류했다.
    - `write_json_batch` 운영 사용처 7개를 목록화했다.
    - GUI 병렬 serialize/write helper는 `monitoring_gui.py`가 아니라 `mission_planning/mission_planning_gui.py` 내부 helper임을 정정했다.
  - 정리한 항목:
    - `SourceArtifactCache.copy_result` 호출 분류와 `copy_result=False` 전환 금지/허용 조건
    - `run_divide_and_pattern`/`build_mission_plan_0301` optional parsed payload/cache handle 설계
    - 특수 파이프라인 full-plan/waypoint deepcopy 위치와 copy-on-write 후보
    - 공용 batch write와 GUI pre-serialized bytes helper 차이
    - 신규 run read-back 생략 조건
    - queue payload deepcopy 필요성, signature 축소 위험, item id/version diff update 설계
    - 0902 canonical JSON reuse 가능 범위와 sidecar fallback/외부 호환 조건
    - d0303 run-local ground context 설계와 final serialize metric 수집 불가/sidecar 권고
    - 선형 탐색/O(N^2) 후보의 구현 승격 gate
  - 미적용 판단:
    - signature field 축소, 0902 compact/중복 write, `copy_result=False` 확대, read-back 생략은 replay/golden 검증 전에는 production 동작으로 적용하지 않는다.
    - d0303 stage별 실제 hit ratio는 정적 리포트가 아니라 runtime scenario metric으로 별도 수집해야 한다.
  - 검증:
    - `tools/replan_static_hotpath_report.py` `py_compile` 통과
    - static report 생성 성공
    - `DSS_Internal/static_hotpath_report/static_hotpath_report.json`에 주요 섹션 포함 확인

- [x] 세션 24: 0401 dispatch context 구조 추가
  - 변경:
    - `monitoring/logic/replan_dispatch_context.py`
    - `monitoring/monitoring_gui.py`
    - `monitoring/logic/prior_mission_replan.py`
    - `monitoring/logic/rtb_replan.py`
    - `monitoring/logic/path_deviation_replan.py`
    - `monitoring/logic/quality_speed_replan.py`
    - `monitoring/logic/imaging_schedule_replan.py`
    - `TODO_replan_speed_optimization.md`
  - 내용:
    - `Replan0401DispatchContext`를 추가해 raw payload, parsed body, canonical signature, timestamp, extracted agent states, settings snapshot, per-dispatch DB JSON cache holder를 묶었다.
    - 0401 pending queue가 canonical signature를 보존해 handler context에 전달한다.
    - 0401 handler가 parse/extract 직후 context를 만들고 prior, RTB, path deviation, quality speed, imaging schedule coordinator에 선택 인자로 전달한다.
  - 기능 불변성 의도:
    - coordinator 내부 로직은 context를 아직 읽지 않는다.
    - 기존 parse, trigger 조건, queue 순서, random roll, payload 생성 구조를 변경하지 않았다.
    - per-dispatch DB JSON cache는 holder만 만들었고 fast path 적용은 replay 이후로 둔다.
  - 검증:
    - `replan_dispatch_context.py`, `monitoring_gui.py`, 5개 coordinator 파일 `py_compile` 통과
    - context call site를 `rg`로 확인

- [x] 세션 25: 0902 sidecar compact/duplicate/merge 계약 검증
  - 변경:
    - `tools/replan_0902_transport_check.py`
    - `DSS_Internal/transport_0902_check/transport_0902_check.json`
    - `TODO_replan_speed_optimization.md`
  - 실행:
    - `python tools/replan_0902_transport_check.py`
  - 결과:
    - pretty mode: 1 entry, 745 bytes
    - compact mode: 1 entry, 468 bytes
    - exact duplicate save는 두 mode 모두 entry 1개를 유지했다.
    - timestamp 차이를 제외한 semantic payload hash와 receiver merge hash가 동일했다.
  - 주의:
    - reason/level/detail identity만 같고 다른 rich field가 있는 payload는 중복 억제 대상으로 승격하지 않는다.
    - sidecar off fallback, pretty default, compact opt-in, C# `replanDetail` string fallback은 유지해야 한다.
  - 검증:
    - `tools/replan_0902_transport_check.py` `py_compile` 통과
    - sidecar 검증 산출물이 target 내부 `DSS_Internal/transport_0902_check`에만 생성됨을 확인
    - 초기 실행 중 target 밖에 생긴 테스트 sidecar 파일 2개는 정확한 파일명으로 삭제했다.

- [x] Session 26: P3 linear-search hot path equivalence
  - Changed:
    - `mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py`
    - `mission_planning/replanning/triggers/attack/pipeline.py`
    - `mission_planning/replanning/triggers/path_deviation/pipeline.py`
    - `mission_planning/replanning/triggers/prior/pipeline.py`
    - `mission_planning/replanning/triggers/remaining_hybrid/general.py`
    - `mission_planning/engine/mission_generation/id_allocation/allocator.py`
    - `tools/replan_hotpath_equivalence_check.py`
    - `DSS_Internal/hotpath_equivalence/hotpath_equivalence_check.json`
  - Result:
    - Replaced first-index waypoint lookup with `waypoint_id -> first index` maps.
    - Replaced attack target O(N^2) dedupe with id/key sets that preserve legacy `_same_target_identity` edge cases.
    - Replaced remaining-hybrid `target_aircraft_ids.index(...)` with `aircraftID -> entry`.
    - Replaced d0303 formation follower `followers.index(...)` with `aircraftID -> follower index`.
    - Added reservation-scoped thread-local directory snapshot cache in ID allocator reservation helpers.
  - Verification:
    - `python -m py_compile` passed for edited modules and the check script.
    - `python tools/replan_hotpath_equivalence_check.py` passed.

- [x] Session 27: d0303 runtime metric measurement
  - Changed:
    - `tools/replan_d0303_metric_report.py`
    - `DSS_Internal/d0303_metric_report/d0303_metric_report.json`
  - Result:
    - Runs a small type-5 d0303 runtime probe with local `_WPAllocator`.
    - Records stage ms for filming candidate scan, ground prepass/scan, DEM lookup, line-search/JSON-ready phases.
    - Calculates DEM cache hit ratio and ground cache ratios when counters are present.
  - Verification:
    - `python -m py_compile tools/replan_d0303_metric_report.py` passed.
    - `python tools/replan_d0303_metric_report.py` produced `measurementStatus=runtime_dense_metrics_found`.

- [x] Session 28: 0401 replay/stress/golden proxy
  - Changed:
    - `tools/replan_0401_replay_stress_check.py`
    - `DSS_Internal/replay_0401_stress/replay_0401_stress_check.json`
  - Result:
    - Verifies canonical 0401 signatures are equal for raw/dict/list payload forms.
    - Verifies duplicate suppression, active item stability, queued trigger order, and snapshot save/load under 20 high-frequency 0401 samples.
    - Confirms 0401 dispatch context is passed as optional signature-only coordinator argument and does not alter payload builder logic.
  - Verification:
    - `python -m py_compile tools/replan_0401_replay_stress_check.py` passed.
    - `python tools/replan_0401_replay_stress_check.py` passed.

- [x] Session 29: logging/stage buffering and 0401 I/O measurement
  - Changed:
    - `tools/replan_logging_io_report.py`
    - `DSS_Internal/logging_io_report/logging_io_report.json`
  - Result:
    - Measures 30 samples for snapshot save, JSONL append, and JSON-array append under isolated target DB root.
    - Verifies consecutive stage-buffer flush preserves order and no entries are dropped.
    - Records that interleaved stage buffering requires monotonic sequence sorting before flush.
  - Verification:
    - `python -m py_compile tools/replan_logging_io_report.py` passed.
    - `python tools/replan_logging_io_report.py` passed.

- [x] Session 30: MILP impossible-transition proof
  - Changed:
    - `tools/replan_milp_transition_proof.py`
    - `DSS_Internal/milp_transition_proof/milp_transition_proof.json`
  - Result:
    - Confirms no source-level impossible-transition predicate exists that is demonstrably identical to current MILP constraints.
    - Keeps dense transition variables unchanged and records the sparse-precheck conclusion as not applied.
    - Uses existing MILP model-build/solve metrics as the safe baseline.
  - Verification:
    - `python -m py_compile tools/replan_milp_transition_proof.py` passed.
    - `python tools/replan_milp_transition_proof.py` passed.

- [x] Session 31: cross-replan low-risk hotpath reuse
  - Changed:
    - `mission_planning/pipelines/mission_path_trim.py`
    - `mission_planning/replanning/triggers/prior/pipeline.py`
    - `mission_planning/replanning/triggers/path_deviation/pipeline.py`
    - `mission_planning/replanning/triggers/post_attack/pipeline.py`
    - `mission_planning/replanning/triggers/remaining_hybrid/general.py`
    - `monitoring/logic/prior_mission_replan.py`
    - `mission_planning/MissionPlanner/planning_enhanced/io/export_0303_0304.py`
    - `tools/replan_0401_replay_stress_check.py`
  - Result:
    - Added per-call route-offset lookup context for shared line-search trim/reanchor utilities.
    - Reused FOV rows, FOV DB signature, camera-adjust scale, and route-offset scale inside trim/realign calls.
    - Moved prior other-UAV resume package writes to `write_json_batch`.
    - Moved path-deviation fallback/current helper reads to `read_json_cached` and a small two-file write to `write_json_batch`.
    - Moved post-attack source artifact reads to `read_json_cached` and multi-file branch writes to `write_json_batch`.
    - Moved remaining-hybrid common `_load_json()` through `read_json_cached`.
    - Used existing 0401 `dispatch_context.settings_snapshot` and `cached_db_json` in prior-close monitoring.
    - Removed duplicate `load_runtime_settings()` in enhanced 0303/0304 export by passing the already-loaded runtime snapshot.
  - Verification:
    - `python -m py_compile` passed for edited modules and the 0401 stress script.
    - `python modules/tools/replan_hotpath_equivalence_check.py` passed.
    - `python modules/tools/replan_0401_replay_stress_check.py` passed.
    - `python modules/tools/replan_attack_allocator_equivalence_check.py` passed.
    - `python modules/tools/replan_timing_report.py` completed.
  - Remaining recommended sessions:
    - Current reexecute hybrid: precompute variant-invariant carry-forward mission/path indexes before variant build.
    - Prior resume: evaluate `SourceArtifactCache.copy_result=False` only for read-only scans, then replay before wider use.
    - Enhanced 0302: pass a runtime snapshot through per-piece FOV/area helpers or wrap the build in one runtime override.
    - Enhanced 0303/0304: keep DEM/groundRequired timing first, then test optional aircraft-parallel 0303 in enhanced export with canonical output comparison.

- [x] Session 32: attack collaborative IMP update batching
  - Changed:
    - `mission_planning/replanning/triggers/prior/pipeline.py`
    - `DSS_Internal/attack_pipeline_replay/attack_pipeline_replay_report.json`
  - Finding:
    - Latest attack log `Scenario_2026-06-17T101848` had `apply_attack_plan_overrides=3080ms`.
    - Main bottleneck was collaborative remaining replan: `collab elapsed=2395.942ms`.
    - Inside that, `imp_update_write=1061.218ms` was sequential per-aircraft validate/write work.
  - Result:
    - Split collaborative IMP update into payload build, aggregate validation, and batch write.
    - Kept existing single-aircraft `_write_collaborative_remaining_imp_update()` compatibility path.
    - Replay of the 3080ms source attack log improved to `current elapsedMs=1163.312 total=1157 override=1125`.
    - Replay of the multi-target source attack log improved to `current elapsedMs=1098.57 total=1094 override=1051`.
    - `REPLAN_NEXT_COLLAB_DEM_PREWARM=0` and `REPLAN_ATTACK_DESCRIPTOR_WORKERS=6` can be tested as runtime variants, but default behavior was not changed.
  - Verification:
    - `python -m py_compile modules/mission_planning/replanning/triggers/prior/pipeline.py modules/mission_planning/replanning/triggers/attack/pipeline.py modules/mission_planning/replanning/triggers/post_attack/pipeline.py modules/tools/replan_attack_pipeline_replay.py` passed.
    - `python modules/tools/replan_attack_pipeline_replay.py --attack-log Logs/Scenario_2026-06-17T101848/SBC3/DSS_Internal/log_attack_algorithm_20260617T011917_211947.json` passed.
    - `python modules/tools/replan_attack_pipeline_replay.py --attack-log Logs/Scenario_2026-06-17T101848/SBC3/DSS_Internal/log_attack_algorithm_20260617T011920_702303.json --variants current no_dem_prewarm descriptor_workers_6` passed.
    - `python modules/tools/replan_hotpath_equivalence_check.py` passed.
    - `python modules/tools/replan_0401_replay_stress_check.py` passed.
    - `python modules/tools/replan_attack_allocator_equivalence_check.py` passed.

## 5. 5회 재검토 기록

1. 1차: 로컬 코드 흐름 검토
   - `monitoring`, `mission_planning`, `common`, `decision_support`, `sim`의 재계획 흐름을 확인했다.
   - 0401 fanout, 0902 queue/transport, mission planning direct pipeline, 일반 pipeline을 큰 흐름으로 묶었다.

2. 2차: 재계획 trigger/state 검토
   - sub agent Faraday가 0201/0202/0401/0402/0802/0803 기반 trigger와 queue 상태 전이를 정리했다.
   - fuel threshold와 0801은 단독 자동 0902 여부를 구분해야 한다는 점을 반영했다.

3. 3차: mission planning pipeline 검토
   - sub agent Sagan이 일반/특수 pipeline의 반복 JSON load, deepcopy, write batch, 0303/lineSearch/terrain, executor 중첩을 정리했다.
   - 계획 계산 내부보다 입력 재로딩과 write path가 먼저 확인할 대상이라는 결론을 반영했다.

4. 4차: dataflow/static smell 검토
   - sub agent Jason/Nash가 0902 sidecar, queue UI rebuild, DB JSON fallback scan, waypoint/index 선형탐색, O(N^2) dedupe, allocator scan, 전수탐색 후보를 정리했다.
   - 기능 변경 위험이 큰 알고리즘 대체는 후순위 검토로 낮췄다.

5. 5차: 안전성/누락 항목 검토
   - sub agent Epicurus가 trigger 의미, queue 병합/억제, sidecar field 보존, `copy_result=False` 전역 적용 위험을 재검토했다.
   - 0401 중복 parse와 snapshot 저장은 상향하고, 전수탐색/알고리즘 대체와 소규모 선형탐색은 계측 전 후순위로 낮췄다.
   - 수용 기준에 trigger/order/payload 동일성, 0401 stress, sidecar field 손실 방지, 성능 목표를 추가했다.

## 6. 변경 금지 또는 주의 항목

- 재계획 토글 기본값 변경 금지
- trigger 조건, triggerType, replan level 변경 금지
- `_0401_coalesce_ms`, cooldown, probability, threshold 변경 금지
- delivery grace/timeout, 0305/0901/0903/0702 상태 전이 변경 금지
- queue priority/order 변경 금지
- random roll 순서 변경 금지
- 알고리즘 대체로 output tie-break가 바뀌는 변경 금지
- 로그/trace 누락 금지
- 외부 C# 메시지 wire format 호환성 파괴 금지
- `SourceArtifactCache.copy_result=False` 전역 적용 금지
- 0902 sidecar 제거/비활성화 금지
- ReplanQueueManager의 필요 판정, 병합, 억제 조건 변경 금지

## 7. 완료 기준

- 동일 입력으로 `0201`, `0202`, `0401`, `0402`, `0802`, `0803 execute=1`, `0803 execute=2` 재계획 시나리오를 각각 재현한다.
- 기준 시나리오에서 재계획 trigger 발생 여부, reason, queue 순서, suppression/merge 결과가 변경 전후 동일하다.
- 0902 payload의 핵심 detail, 0301/0901/0903/0702 의미가 동일하다.
- sidecar 병합 후 보존되어야 하는 rich field가 손실되지 않는다.
- missionPlan, individualMissionPlan, flightPath 출력이 구조적으로 동일하거나, 차이가 timestamp/생성 ID/진단 metric 등 허용 필드에 한정된다.
- waypoint 수/order/좌표 허용오차, speed/altitude, aircraft assignment, mission-plan 관계가 유지된다.
- P0 계측에서 최소 다음 지표가 개선된다.
  - 0401 handler 총 시간
  - 0902 enqueue-to-dispatch 시간
  - mission planning pipeline wall-clock 시간
  - JSON load/write 횟수 및 ms
  - deepcopy 총 ms
- 후보별 targeted phase median 20% 이상 개선 또는 end-to-end median 10% 이상 개선을 목표로 한다.
- 비대상 시나리오 p95 wall-clock이 5%를 초과해 악화되지 않아야 한다.
- 고빈도 0401 stress에서 누락/중복 replan, sidecar 필드 손실, queue 순서 역전이 없어야 한다.
- 개선이 확인되지 않은 리팩터링은 적용하지 않는다.
