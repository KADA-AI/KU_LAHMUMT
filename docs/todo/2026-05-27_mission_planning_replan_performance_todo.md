# 2026-05-27 Mission Planning Replan Performance TODO

작성 목적: Mission Planning 재계획 파이프라인을 3초 목표에 가깝게 줄이되, ID 중복, ICD 불일치, 기존 재계획 동작 파괴를 막기 위한 작업 TODO를 코드 기준으로 정리한다.

기준 workspace: `C:\Users\LAHMUMT_2\Desktop\DSS_KU`

검토 기준:

- 실제 로그 집계: `Logs/**/DSS_Internal/module_logs/mission_planning.log`
- 실제 요청 저장소: `Logs/**/DSS_Internal/replan_request_transport/replan_request_*.json`
- 주요 코드:
  - `modules/mission_planning/mission_planning_gui.py`
  - `modules/mission_planning/pipelines/*.py`
  - `modules/mission_planning/runtime/*.py`
  - `modules/mission_planning/MissionPlanner/data_def/id_allocator.py`
  - `modules/mission_planning/MissionPlanner/planning_enhanced/pipeline.py`
  - `modules/common/option_codes.py`
  - `modules/common/process_console.py`
  - `modules/mission_planning/runtime/mission_planning_pipeline_logging.py`

sub-agent 검토 반영:

- ID/ICD sub-agent: `id_allocator.py`, `option_codes.py`, GUI validation helper, `id_relationship_tab.py` 기준으로 ID 발급/검증 위험을 별도 확인했다.
- 공격 계열 sub-agent: `attack_plan_pipeline.py`, `post_attack_rejoin_pipeline.py`, GUI 공격 라우팅/전달부 기준으로 공격 특화/배제/종료 후 복귀 위험을 별도 확인했다.
- 일반/hybrid 계열은 로컬 코드/로그 검토 기준으로 문서화했다.

코드 커버리지 매핑:

- GUI routing: `mission_planning_gui.py`
  - `_should_use_attack_pipeline` -> 1/2
  - `_should_use_post_attack_rejoin_pipeline`, `_try_run_post_attack_rejoin_pipeline` -> 3
  - `_try_run_next_collab_replan_pipeline` -> 4
  - `_try_run_imaging_schedule_replan_pipeline` -> 5/6
  - `_try_run_path_deviation_replan_pipeline` -> 7
  - `_try_run_prior_mission_pipeline`, `_try_run_prior_post_rejoin_pipeline` -> 8/9
  - generic fallback option build/apply branch -> 10/11/12/13/14/15
- Pipeline/runtime entry points:
  - `run_attack_plan_pipeline` -> 1
  - `run_attack_exclusion_pipeline` -> 2
  - `run_post_attack_rejoin_pipeline` -> 3
  - `run_next_collab_replan_pipeline`, `prepare_next_collab_input_replacements`, `run_next_collab_line_plan`, `run_next_collab_division_plan` -> 4
  - `run_imaging_schedule_replan_pipeline` with `imagingScheduleDeviation` -> 5
  - `run_imaging_schedule_replan_pipeline` with `qualityMonitorSep` -> 6
  - `run_path_deviation_replan_pipeline` -> 7
  - `run_prior_mission_pipeline` -> 8
  - `run_prior_post_rejoin_pipeline` -> 9
  - `run_divide_and_pattern`, `planning_enhanced/pipeline.py`, `build_flight_plans`, `build_0303_flight_plans_aircraft_parallel` -> 10/15
  - `build_current_remaining_hybrid`, `prepare_current_remaining_hybrid_replacements`, `merge_current_remaining_hybrid` -> 11
  - `apply_remaining_hybrid_replan` -> 12
  - `prepare_reexecute_first_mission_replacements` -> 13
  - `is_recon_specialized_option`, `build_recon_specialized_runtime_payload`, `recon_area_review` -> 14
- 별도 재계획 종류로 세지 않는 보조 로직:
  - `warm_*` 함수는 warmup/preload.
  - `_deliver_*_direct_now`, `_push_0301`, `_push_0305`, `_push_0903`, `_push_0702_auto_apply`는 delivery/ICD 전송 경로.
  - `*_store.py`, runtime state module은 state persistence/lookup.
  - path builder/line runner/division runner는 next-collab 및 hybrid 내부 알고리즘 구성요소.

주의:

- 이 문서는 코드 수정 결과가 아니라 앞으로의 작업 계획이다.
- 현재 worktree에는 unrelated 변경/삭제가 매우 많다. 구현 시 unrelated 파일을 되돌리지 않는다.
- 사용자가 만들어 둔 `app`/`modules` 백업본은 작업 대상에서 제외한다. 구현, 포맷, 검색 기반 일괄 변경, 정리 작업 모두 백업 디렉터리를 건드리지 않는다.
- ID는 재사용하지 않는 forward-only 정책을 유지한다. 성능 개선을 위해 이미 발급된 ID를 회수하거나 되감지 않는다.
- ICD 필드 타입을 바꾸지 않는다. 특히 `missionPlanID`, `individualMissionPackageID`, `individualMissionID`, `pathID`, `waypointID`, `aircraftID`, `inputMissionID`, `timestamp` 계열은 int 유지가 원칙이다.
- `0301/0302/0303/0304/0305/0702/0901/0903/0001` 전달 정책을 임의로 합치지 않는다. 성능 최적화와 운용 메시지 정책 변경은 분리한다.
- `optionName`은 0901/옵션 전달에서는 숫자 코드가 canonical이다. 내부 label 최적화가 option code `2/3/4/5/6` 매핑을 깨면 안 된다.
- 로깅은 장애 추적용이어야지 새 병목이 되면 안 된다. hot path에서 동기 파일 쓰기, 큰 payload pretty print, waypoint별 과다 로그는 금지한다.

## 절대 지켜야 할 ID/ICD 불변식

- `MissionPlan/<missionPlanID>.json`의 `aircraftList[].individualMissionPackageID`는 실제 `IndividualMissionPlan/<id>.json` 파일과 일치해야 한다.
- `IndividualMissionPlan`의 각 `individualMissionList[].pathID`는 실제 `FlightPath/<pathID>.json` 파일과 일치해야 한다.
- `FlightPath.pathID`, 파일명, `aircraftID`, `individualMissionID`는 해당 0302 mission row와 일치해야 한다.
- `pathID`는 항공기 prefix 규칙을 유지한다.
  - LAH 1/2/3: `100000xxx`, `200000xxx`, `300000xxx`
  - UAV 4/5/6: `400000xxx`, `500000xxx`, `600000xxx`
- `waypointID`는 전역 중복이 없어야 하고 `nextWaypointID` 체인이 같은 payload 안에서 유효해야 한다.
- 병렬 worker 안에서는 전역 `_next()` 호출을 최소화한다. 병렬 진입 전 reserve한 local allocator 또는 명시 ID block만 사용한다.
- `id_allocator.py`의 process-local `threading.Lock`만으로는 멀티프로세스 재계획 중복을 막을 수 없다. file lock 또는 atomic lockfile 기반 read-modify-write가 필요하다.
- `modules/mission_planning/MissionPlanner/planning_enhanced/io/export_0301.py`, `modules/mission_planning/MissionPlanner/planning_enhanced/io/export_0302.py`의 독립 counter/fixed ID는 active DB 저장 전에 공용 allocator 기반 ID로 remap되어야 한다.
- reserve 후 실패로 인한 ID 누수는 허용 가능하지만 반드시 로그에 남긴다. 누수보다 중복이 훨씬 위험하다.
- 기존 source MissionPlan/IMP/FlightPath를 직접 수정하지 않는다. 새 plan/imp/path 파일을 만들고 lineage/meta를 보존한다.
- `isDone`, `relatedMission`, `priorMissionID`, `inputMissionID`, `targetID`, `formationInfo`, filming/loiter/search 속성은 복사/trim/merge 단계에서 누락되면 안 된다.
- `option_codes.py`의 Korean label은 UTF-8 원문을 유지하고, 알 수 없는 label은 silent default로 바꾸지 않는다.
- `id_relationship_tab.py`는 진단/시각화 도구로만 본다. 저장 전 차단 validator를 대체하면 안 된다.

## 공통 선행 TODO

### C0. 계측과 기준 고정

- [ ] SLA 기준을 문서화한다.
  - 1차: Mission Planning 내부 `0902_received_ms -> pipeline_done_ms`.
  - 2차: 사용자 체감 `0902_received_ms -> 0901_sent/0903_sent`.
  - 보조: Monitoring `0305 status=1 -> status=2`.
- [ ] 모든 전용 pipeline에 `PipelinePhaseTimer` 또는 동등한 phase timer를 넣는다.
  - 이미 있음: next-collab, prior 일부, attack detail.
  - 부족함: post-attack, prior-post-rejoin, path-deviation, imaging-schedule, quality-speed.
- [ ] ID allocator 내부 계측을 추가한다.
  - lock wait ms
  - `_read_store_state` ms
  - `_save` ms
  - directory scan ms
  - waypoint/path usage update ms
- [ ] 0902 archive 기반 replay benchmark를 만든다.
  - 원본 DB를 직접 쓰지 말고 임시 DB copy 또는 dry-run output root 사용.
  - benchmark가 ID tracker를 오염시키지 않도록 별도 DB root를 사용.
  - trigger별 p50/p95/max와 phase별 평균을 출력.

### C1. 공통 validation 모듈화

- [ ] `mission_planning_gui.py` 안에 흩어진 검증 로직을 재사용 가능한 validator로 분리한다.
  - `_validate_unique_flightpath_ids`
  - `_validate_mission_flightpath_links`
  - `_sync_flight_plan_individual_mission_ids`
  - `_repair_missing_flight_path_files`
- [ ] 모든 pipeline 저장 직전에 공통 validator를 호출한다.
- [ ] validator는 실패 시 파일 쓰기 전에 중단해야 한다.
- [ ] validator 실패 메시지에 plan/imp/path/aircraft/inputMissionID를 포함한다.
- [ ] validator 필수 검증 항목을 golden invariant로 고정한다.
  - 0301 `aircraftList` -> 0302 IMP 매핑.
  - 0302 package 존재 및 aircraft 일치.
  - `individualMissionID` 전역 유일성.
  - `pathID` 전역 유일성 및 aircraft band 일치.
  - 0302 pathID와 0303/0304 FlightPath 1:1 매칭.
  - orphan FlightPath 금지.
  - FlightPath의 `aircraftID`/`individualMissionID` 역참조 일치.
  - `waypointID` 유일성 및 `nextWaypointID` chain 유효성.
- [ ] validator 적용 경로를 parallel, sequential, attack, prior, path deviation, next collab, post-attack rejoin까지 모두 동일하게 만든다.
- [ ] 공통 ICD type helper를 만든다.
  - `aircraftID` 1..6
  - path band
  - uint32 범위
  - timestamp
  - bool/float 변환
- [ ] GUI option label normalization을 한 함수로 통합한다.
  - `공격 특화`, `공격 배제`, `정찰 특화`, `최소 시간`, `정찰/시간 균형`을 정확한 int code로 매핑.
  - unknown string은 조용히 default option으로 바꾸지 않고 warning/failure 정책 적용.

### C2. ID reserve 인프라

- [ ] `id_allocator.py`의 기존 `reserve_*` API를 감싸는 재계획 전용 reservation context를 만든다.
  - 예: `ReplanIdReservation`
  - plan id block
  - imp id block
  - individual mission id block
  - aircraft별 path id block
  - waypoint id block
- [ ] reservation context는 local allocator를 제공해야 한다.
  - `next_imp()`
  - `next_individual()`
  - `next_path(aircraft_id)`
  - `next_waypoint()`
- [ ] local allocator 범위 초과 시 즉시 실패하고, 전역 allocator로 조용히 fallback하지 않는다.
- [ ] `id_allocator.py`의 store read/modify/write를 file lock으로 감싼다.
  - Windows 기준 `msvcrt` 또는 검증된 lockfile helper 사용.
  - lock wait time과 lock hold time을 별도 로그로 남김.
- [ ] `reserve_*` API를 유일한 active DB ID 발급 경로로 고정한다.
  - `planning_enhanced` 독립 counter는 임시 산출물 전용으로 제한하거나 active DB 저장 전 remap.
  - fixed ID가 active DB에 들어가는 경로는 validator에서 실패 처리.
- [ ] 각 pipeline 결과 log에 reserved/used/unused ID 범위를 남긴다.
- [ ] 10개 프로세스 동시 reserve 테스트를 추가한다.
  - `reserve_path_ids`
  - `reserve_waypoint_block`
  - `reserve_mission_plan_ids`
  - duplicate 0건을 assert.
- [ ] 구현 순서:
  - 1차: 계측만 추가.
  - 2차: 단일-thread pipeline에서 local allocator 사용.
  - 3차: general/attack 병렬 worker로 확대.

### C3. source artifact cache

- [ ] 재계획 1회 단위 source artifact cache를 명시 컨텍스트로 전달한다.
  - MissionPlan
  - IndividualMissionPlan
  - FlightPath
  - pathID -> waypoint/currentWP index
  - inputMissionID -> mission row
- [ ] cache는 source plan 기준 read-only로 유지한다.
- [ ] pipeline 도중 source 파일이 바뀔 가능성을 가정하지 말고, 변경 가능성이 있는 곳은 새 파일로만 쓴다.

### C4. JSON write와 debug artifact 정책

- [ ] batch write helper를 전용 pipeline에도 적용하되, 파일별 성공/실패를 log에 남긴다.
- [ ] debug artifact write는 기본 동작을 유지하고, 성능 모드 flag는 별도 PR로 분리한다.
- [ ] `skip_if_unchanged=True` 최적화가 timestamp 갱신/ICD 송신 의도를 깨지 않는지 pipeline별로 확인한다.

### C5. 저부하 module log / observability

목표: 모든 프로세스와 Mission Planning 알고리즘 중간 단계가 꺼짐/멈춤/분기 실패를 추적할 수 있을 만큼 module log에 남되, 재계획 성능과 기능 동작에는 체감 영향이 없게 한다.

- [ ] 공통 log event contract를 정의한다.
  - `event`
  - `module`
  - `processId`/`threadName`
  - `replanRequestId` 또는 `replanTransactionId`
  - `trigger`/`triggerType`
  - `pipeline`
  - `phase`
  - `missionPlanID`
  - `aircraftID`
  - `elapsedMs`
  - `outcome`
  - `reason`
- [ ] 기존 `modules/common/process_console.py`의 async process file logging을 우선 사용한다.
  - hot path에서 직접 `open/write/flush`하지 않는다.
  - bounded queue, drop count, flush failure count를 남긴다.
  - 반복 warning은 rate-limit한다.
- [ ] 모든 주요 프로세스 lifecycle 로그를 남긴다.
  - process start
  - DB root bind/rebind
  - listener start/stop/fail
  - heartbeat worker start/stop/fail
  - power on/off
  - graceful shutdown
  - uncaught exception
  - worker thread exception
  - future timeout/cancel
  - queue backlog 증가
- [ ] Mission Planning 재계획 공통 checkpoint 로그를 표준화한다.
  - 0902 received
  - context parsed
  - pipeline selected
  - source artifacts loaded
  - branch decision
  - ID reserve start/end
  - algorithm start/end
  - build start/end
  - validation start/end
  - write start/end
  - delivery queued/sent/suppressed
  - fallback/no-op/failure reason
- [ ] 알고리즘 내부 반복 로그는 개별 waypoint/segment마다 남기지 않고 집계값으로 남긴다.
  - input count
  - generated mission/path/waypoint count
  - cache hit/miss
  - skipped branch count
  - retry/fallback count
- [ ] `PipelinePhaseTimer` 결과와 module log event를 연결한다.
  - 같은 `replanTransactionId`로 timing event, phase detail, delivery event를 추적 가능해야 함.
  - replay benchmark가 module log를 읽어 phase별 p50/p95를 만들 수 있어야 함.
- [ ] 로깅 overhead budget을 정한다.
  - replay 기준 p95 증가 100ms 이하를 1차 목표로 둔다.
  - 단일 log enqueue는 p95 1ms 이하를 목표로 둔다.
  - queue full/drop이 발생하면 event 개수와 drop count만 저주기로 남긴다.
- [ ] 적용 순서는 Mission Planning부터 시작하고, 이후 다른 프로세스가 opt-in할 수 있는 common helper로 확장한다.
  - 1차: Mission Planning lifecycle + 재계획 checkpoint.
  - 2차: Monitoring/Simulation/GUI 연계 프로세스 lifecycle.
  - 3차: 전체 프로세스 공통 health summary.

## 구현 분할 권장 순서

1. PR-A: observability contract와 logging budget
   - 코드 동작 변경 없이 event schema, rate-limit/drop 정책, overhead 측정 기준 확정.
2. PR-B: 공통 저부하 module log helper
   - `process_console.py` 기반 async logging, lifecycle event, queue/drop metric 추가.
3. PR-C: 계측 전용
   - phase timer, ID allocator timing, benchmark CLI, module log checkpoint 골격 추가.
4. PR-D: validator 모듈화
   - 저장 전 링크/중복 검증을 공통화하고 fail-closed 적용 범위를 확정.
5. PR-E: ID allocator file lock
   - 멀티프로세스 reserve 테스트와 lock timing log를 먼저 완료.
6. PR-F: ID reservation context 도입
   - 병렬화 없이 단일 pipeline local allocator부터 적용.
7. PR-G: option code/ICD helper 정리
   - UTF-8 label, unknown string 정책, ICD type helper를 먼저 안정화.
8. PR-H: 공격 특화/공격 배제 ID bulk reserve 적용
   - 가장 큰 ID 병목 후보를 먼저 줄인다.
9. PR-I: post-attack 복귀/재합류 안전성 개선
   - tracking assignment clear 순서, return-only 성공 기준, direct delivery transaction log.
10. PR-J: general 3-option variant ID/waypoint block 선예약
   - 병렬 worker 안전성 확보.
11. PR-K: general/current hybrid 공통 계산 공유
   - split/parse/cache 공유, option별 의미가 다른 계산은 공유 금지.
12. PR-L: 전용 pipeline checkpoint logging과 소규모 최적화
   - next/prior/path/imaging/quality/post-rejoin에 lifecycle/checkpoint log와 phase timer 적용.
13. PR-M: attack descriptor 병렬화
   - local ID allocator와 validator가 안정화된 뒤 진행.
14. PR-N: 0303 FlightPath build 최적화
   - waypoint block, worker 수, reassign 최적화.
15. PR-O: replay benchmark와 실제 로그 회귀 기준 갱신
   - 변경 전/후 p50/p95/max 비교.

## 1. 공격 특화 재계획

현재 코드:

- `mission_planning_gui.py`: attack option 감지, attack exclusion parallel 시작, attack 결과 delivery 병합.
- `pipelines/attack_plan_pipeline.py`
  - `run_attack_plan_pipeline`
  - attack override detail
  - descriptor loop
  - `_build_lah_attack_sequence_package`
  - `_build_lah_hold_resume_package`
  - `_build_uav_attack_tracking_package`
  - `_build_uav_attack_resume_package`

실측 병목:

- 공격 특화 override 평균 약 6.93초.
- `descriptor_loop` 평균 약 6.86초.
- descriptor 내부 `allocate_ids`, `split_lah_resume`, `clone_followups`, `payload_build` 비중이 큼.

TODO:

- [ ] `descriptor_loop` 안의 ID 할당 시간을 더 쪼갠다.
  - imp id
  - path id
  - individual mission id
  - waypoint id
  - id tracker read/write/scan
- [ ] `_allocate_fresh_plan_id()`의 directory scan 기반 planID 할당을 공용 allocator/reservation으로 통합한다.
  - 공격 특화와 공격 배제가 병렬로 돌 때 planID가 disjoint인지 보장.
  - 실패로 인한 ID 누수는 허용하되 중복은 fail.
- [ ] `_compute_attack_point()` subprocess에 timeout을 추가한다.
  - timeout 시 adaptive standoff fallback 또는 명시 실패 notice로 3초 예산 내 복귀.
  - fallback 사용 여부를 `timingMs.overrideDetail`에 남김.
- [ ] multi-target 공격에서 attack point cache를 둔다.
  - key: target/aircraft/heading/standoff config.
  - cache hit/miss와 saved ms를 phase timing에 기록.
- [ ] attack 전용 `AttackIdReservation`을 만든다.
  - 선택된 LAH attack 수
  - LAH hold/resume 수
  - UAV tracking 수
  - UAV resume 수
  - collab replacement 수
  - follow-up clone 최대치를 conservative하게 산정
- [ ] builder 함수들이 `_next_*`를 직접 부르지 않고 reservation context에서 ID를 받도록 바꾼다.
- [ ] UAV_RESUME이 `handled_by_collab`로 끝날 것이 확실한 경우 artifact load/deepcopy 전에 early skip한다.
- [ ] follow-up clone은 전체 IMP deepcopy 대신 target index 이후 suffix만 clone한다.
- [ ] descriptor별 JSON write를 모아 batch write한다.
- [ ] 공격 실패 notice 문자열 mojibake를 상수화하고 UTF-8 회귀 테스트를 둔다.
- [ ] descriptor 병렬화는 마지막에 한다.
  - LAH_ATTACK/LAH_HOLD_RESUME/UAV_TRACK builder는 local ID block 사용.
  - `new_plan_data.aircraftList` 업데이트는 deterministic merge 단계에서만 수행.
  - attack tracking assignment state write는 race가 없도록 마지막 직렬 단계에서 수행.

구현 분할:

1. 계측만 추가.
2. ID reservation context 주입, 병렬화 없음.
3. early skip + suffix clone 최적화.
4. batch write.
5. descriptor 병렬화.

검증:

- [ ] 공격 1/2/3 target 케이스에서 generated MissionPlan/IMP/FlightPath 링크 검증.
- [ ] `attack_tracking_state.json`이 targetID/aircraftID/sourcePlanID를 잃지 않는지 확인.
- [ ] 공격 실패 fallback에서 공격 배제 옵션이 잘못 생략/중복되지 않는지 확인.
- [ ] option code `2`가 0901/0903 delivery에서 유지되는지 확인.
- [ ] attack point subprocess 지연 주입 시 timeout/fallback이 3초 예산 내 동작하는지 확인.
- [ ] `python -m py_compile` 대상으로 `attack_plan_pipeline.py`, `post_attack_rejoin_pipeline.py`, `mission_planning_gui.py`를 포함.

## 2. 공격 배제 재계획

현재 코드:

- `mission_planning_gui.py`: `run_attack_exclusion_pipeline`을 attack specialized와 병렬로 시작 가능.
- `pipelines/attack_plan_pipeline.py`: `run_attack_exclusion_pipeline`.

실측 병목:

- 공격 배제 평균 약 4.40초.
- unavailable UAV 1/2 조건에서는 평균 3초 이상.

TODO:

- [ ] 공격 특화와 source artifact cache를 공유한다.
- [ ] 공격 특화와 ID reservation을 공유하지 말고, 같은 상위 request 안에서 disjoint block을 선예약한다.
- [ ] currentWaypointID 누락 복구용 plan/path/waypoint index를 run 단위로 만든다.
  - 최근 MissionPlan/waypoint 반복 scan을 한 번의 source cache lookup으로 대체.
  - 복구 실패 시 기존 fallback/no-op 정책을 유지.
- [ ] 공격 배제용 resume/return branch 생성 범위를 최소화한다.
- [ ] 공격 특화 실패 시 fallback 규칙을 문서화하고 코드에 guard log를 추가한다.
  - `0402` attack override 실패 시 exclusion fallback 생략 조건.
  - attack option은 생성됐지만 exclusion만 실패한 경우 delivery 정책.
- [ ] 공격 배제 결과를 일반 fallback variant와 섞지 않도록 option filtering을 유지한다.

구현 분할:

1. attack exclusion phase timer 보강.
2. source cache 공유.
3. ID disjoint reservation.
4. resume branch 최소화.
5. attack/exclusion 병렬 join 정책 점검.

검증:

- [ ] option code `3`이 공격 배제로 유지되는지 확인.
- [ ] 공격 특화 planID와 공격 배제 planID가 충돌하지 않는지 확인.
- [ ] 공격 배제 FlightPath가 원본 경로를 직접 덮어쓰지 않는지 확인.
- [ ] currentWaypointID 누락 케이스에서 완료된 area/line이 되살아나지 않는지 확인.

## 3. 공격 종료 후 복귀/재합류 재계획

현재 코드:

- `mission_planning_gui.py`: `_try_run_post_attack_rejoin_pipeline`.
- `pipelines/post_attack_rejoin_pipeline.py`: `run_post_attack_rejoin_pipeline`.
- runtime state:
  - `modules/mission_planning/runtime/attack_tracking_state.py`
  - `modules/mission_planning/runtime/attack_assignment_state.py`
  - `modules/common/mission_area_replan_store.py`

실측:

- pipeline 평균 약 2.85초.
- 0903 송신까지 평균 약 3.37초.
- step timer가 부족해서 내부 병목은 아직 불명확.

TODO:

- [ ] phase timer 추가.
  - detail validation
  - tracking state load
  - current MissionPlan load
  - group evaluation
  - LAH resume update
  - active-only/collab update
  - tracking return-only update
  - MissionPlan write
  - state release
  - delivery queue
- [ ] 협업 재계획 실패 시 tracking assignment를 즉시 clear하지 않는다.
  - return-only IMP/FlightPath 생성 성공 후 clear.
  - return-only도 실패하면 기존 assignment 유지 또는 명시 실패 상태로 남김.
  - `collab is None`/skipped 경로에서 clear가 먼저 일어나지 않도록 audit.
- [ ] prior-post-rejoin과 공통인 return/rejoin helper를 식별하되, 먼저 동작 차이를 문서화한다.
- [ ] attack slot release는 plan write 성공 이후에만 수행하는지 확인한다.
- [ ] plan/IMP/FlightPath/sweep/coverage payload를 per-run cache로 공유한다.
  - ETA 산정
  - lookahead 예측
  - return-only 생성
- [ ] post-attack 전용 산출물 validator를 공통 validator에 연결한다.
  - tracking branch 제거 여부.
  - return-only terminal hold.
  - `postAttackBoundaryHold`.
  - mission area snapshot carry-forward.
- [ ] no-op 케이스에서 `0305 status=2` + `0001` 정책이 기존 Monitoring queue와 충돌하지 않는지 확인한다.
- [ ] active progress skip, remaining work threshold를 timing log와 함께 남긴다.
- [ ] 0301/0305/0903/0702 전달에 replan transaction ID를 남긴다.
  - post-attack direct 0903.
  - suppress 0702.
  - 정확히 한 번 flush되는지 로그 검증.

구현 분할:

1. 계측만 추가.
2. state write 순서 audit.
3. source artifact cache 적용.
4. ID reservation 적용.
5. prior-post-rejoin과 공통 helper 추출.

검증:

- [ ] target destroyed 후 tracking UAV가 남아 있는 케이스.
- [ ] remaining work too small no-op 케이스.
- [ ] attack assignment release가 너무 이르거나 늦지 않은지 확인.
- [ ] force direct 0903 delivery가 유지되는지 확인.
- [ ] return-only path, terminal loiter, tracking assignment clear가 plan write 성공 이후에만 일어나는지 확인.
- [ ] delivery 로그에서 0301/0305/0903/0702가 transaction 단위로 한 번씩만 기록되는지 확인.

## 4. 다음 협업기저임무 재계획

현재 코드:

- `mission_planning_gui.py`: `_try_run_next_collab_replan_pipeline_impl`.
- `pipelines/next_collab_replan_pipeline_impl.py`.
- `runtime/next_collab_line_runner.py`.
- `runtime/next_collab_division_runner.py`.
- `pipelines/next_collab_path_builder.py`.

실측:

- pipeline 평균 약 2.74초.
- `prepare_replacements` 평균 약 2.52초로 대부분을 차지.

TODO:

- [ ] `prepare_replacements` 내부를 세분 계측한다.
  - source load
  - target input resolve
  - entry coordinate resolve
  - line/area planner run
  - replacement mission build
  - FlightPath build
  - write
- [ ] replacement 생성에서 UAV별 독립 작업을 분리한다.
- [ ] path builder가 source artifact를 반복 로드하는지 확인하고 cache를 주입한다.
- [ ] next-collab 전용 ID reservation을 도입한다.
- [ ] short-line skip fallback은 유지하되, skip 판단 시간을 계측한다.
- [ ] FOV/SEP/turn radius/runtime 설정이 source/template에서 유지되는지 검증한다.

구현 분할:

1. phase timer 보강.
2. source cache 적용.
3. ID reservation 적용.
4. replacement worker 병렬화.
5. short-line/no-op 회귀 테스트.

검증:

- [ ] `force_direct_update=True`, `suppress_0702_fallback=True` 유지.
- [ ] generated input package/IMP/path IDs가 result summary와 session scope에 모두 반영되는지 확인.
- [ ] line mission과 area mission을 분리해 benchmark.

## 5. 촬영 스케줄 이탈 재계획

현재 코드:

- `mission_planning_gui.py`: `_try_run_imaging_schedule_replan_pipeline`.
- `pipelines/imaging_schedule_replan_pipeline_impl.py`.
- `modules/common/imaging_schedule_replan_store.py`.

실측:

- 현재 2026-05-21 로그 세트에는 trigger 샘플 없음.
- 코드상 write 구간만 ms로 찍힘.

TODO:

- [ ] phase timer 추가.
  - detail/store load
  - source artifact resolve/load
  - waypoint trim/replacement
  - ID allocation
  - plan/imp/fp build
  - write artifacts
  - log artifact
- [ ] plan_ids는 정확히 1개만 허용하는 현재 정책을 유지한다.
- [ ] replacementWaypointID가 없거나 currentWaypointID가 invalid인 경우 no-op/failure 정책을 명확히 한다.
- [ ] original FlightPath의 filming/loiter/search 속성이 replacement 후 보존되는지 검사한다.
- [ ] write 전 공통 validator 적용.

구현 분할:

1. 계측.
2. validator 적용.
3. ID reservation 적용.
4. trim/replacement 최적화.

검증:

- [ ] currentWP trim 케이스.
- [ ] replacement waypoint 케이스.
- [ ] `imagingScheduleDeviation` trigger가 quality trigger와 섞이지 않는지 확인.

## 6. 품질 기반 속도 보정 재계획

현재 코드:

- `pipelines/imaging_schedule_replan_pipeline_impl.py`.
- `QUALITY_TRIGGER_TYPE = "qualityMonitorSep"`.
- 0901 옵션 생성은 quality trigger에서 차단됨.

실측:

- 현재 로그 세트에는 trigger 샘플 없음.

TODO:

- [ ] imaging schedule과 같은 phase timer를 공유하되, quality 전용 step을 별도 기록한다.
  - current aircraft coordinate resolve
  - resume trim
  - searchSpeed scale
  - sweep trim count
- [ ] `searchSpeedScale` 타입과 범위를 검증한다.
  - 0 이하 금지.
  - 누락 시 1.0.
  - direction 값은 기록만 하고 계산 정책과 분리.
- [ ] quality trigger에서는 0901 option request가 나가지 않는 정책을 유지한다.
- [ ] `force_direct_update`/`0702 fallback` 정책을 명확히 문서화한다.

구현 분할:

1. 계측.
2. input validation 강화.
3. speed scaling 적용 범위 audit.
4. replay sample 확보 후 최적화.

검증:

- [ ] searchSpeed 변경 전/후가 FlightPath에 정확히 반영되는지 확인.
- [ ] option list가 생성되지 않는지 확인.
- [ ] trimmed sweep points가 음수/중복이 되지 않는지 확인.

## 7. 경로 이탈 재계획

현재 코드:

- `mission_planning_gui.py`: `_try_run_path_deviation_replan_pipeline`.
- `pipelines/path_deviation_replan_pipeline_impl.py`.
- `modules/common/path_deviation_replan_store.py`.

실측:

- 현재 2026-05-21 로그 세트에는 trigger 샘플 없음.
- 코드상 write 구간만 ms로 찍힘.

TODO:

- [ ] phase timer 추가.
  - detail/store load
  - source plan/imp/fp resolve
  - currentWP trim
  - alternate waypoint insert
  - other UAV resume update
  - preserved LAH artifact handling
  - ID allocation
  - write artifacts
  - log artifact
- [ ] preserved LAH IMP/path가 generated set에서 누락되어 UI/전달에서 사라지는지 확인한다.
- [ ] synthetic alternate waypoint ID가 기존 waypointID와 충돌하지 않도록 local waypoint allocator를 사용한다.
- [ ] source MissionPlan deepcopy 후 이탈 UAV와 필요한 UAV만 교체하는 정책을 유지한다.
- [ ] path deviation detail이 0902에 없을 때 store lookup fallback을 유지한다.

구현 분할:

1. 계측.
2. generated/preserved metadata 검증.
3. ID reservation 적용.
4. other UAV update 최적화.

검증:

- [ ] 이탈 UAV만 교체되는 케이스.
- [ ] LAH 경로 보존 케이스.
- [ ] currentWP=0 또는 terminal currentWP 예외 케이스.
- [ ] pathID prefix가 aircraftID와 맞는지 확인.

## 8. 선행임무 재계획

현재 코드:

- `mission_planning_gui.py`: `_try_run_prior_mission_pipeline`.
- `pipelines/prior_mission_pipeline_impl.py`.
- `modules/common/prior_replan_store.py`.
- `runtime/prior_tracking_state.py`.

실측:

- 현재 로그 세트에는 trigger 샘플 없음.
- 코드상 phase timer는 있음: `resolve_artifacts`, `allocate_ids`, `build_artifacts`, `write_artifacts`, `total`.

TODO:

- [ ] 실제 prior trigger replay sample을 확보한다.
- [ ] existing phase timer를 유지하면서 세부 단계 추가를 검토한다.
  - agent snapshot load
  - target/prior record resolve
  - nearest UAV select
  - collaborative resume build
  - other UAV resume package
- [ ] ID reservation 적용.
  - selected UAV done/prior/resume paths
  - prior/resume individual mission IDs
  - other UAV updates
  - collab replacement artifacts
- [ ] target coordinate altitude DEM fallback이 filming altitude guard와 충돌하지 않는지 확인한다.
- [ ] `priorMissionID`, `relatedMissionType`, `targetID`, `autoTracking` 보존 규칙을 문서화한다.

구현 분할:

1. replay sample/계측 확인.
2. ID reservation.
3. collab resume cache 적용.
4. other UAV update 최적화.

검증:

- [ ] missionType=2 target tracking.
- [ ] missionType!=2 coordinate prior.
- [ ] unavailable/RTB UAV 제외.
- [ ] priorMissionID가 0302 relatedMission에 유지되는지 확인.

## 9. 선행임무 종료 후 재합류 재계획

현재 코드:

- `mission_planning_gui.py`: `_try_run_prior_post_rejoin_pipeline`.
- `pipelines/prior_mission_pipeline_impl.py`: `run_prior_post_rejoin_pipeline`.

실측:

- 현재 로그 세트에는 trigger 샘플 없음.

TODO:

- [ ] post-attack rejoin과 phase timer 구조를 맞춘다.
- [ ] prior tracking state와 attack tracking state가 섞이지 않도록 state key를 검증한다.
- [ ] no-op 시 `0305 status=2` + `0001` 정책을 post-attack과 동일하게 검토한다.
- [ ] ID reservation은 post-attack과 별도 block을 사용한다.
- [ ] source MissionPlan lineage와 priorMissionID lineage를 모두 log에 남긴다.

구현 분할:

1. 계측.
2. state/load guard.
3. ID reservation.
4. post-attack 공통 helper 추출 여부 결정.

검증:

- [ ] `triggerType=priorClosedResume`.
- [ ] remaining work too small.
- [ ] prior tracking aircraft가 unavailable인 경우.

## 10. 일반 재계획 fallback

현재 코드:

- `mission_planning_gui.py`: 일반 variant 생성 loop.
- `MissionPlanner/AnS/mission_pipeline.py`: `run_divide_and_pattern`.
- `MissionPlanner/planning_enhanced/pipeline.py`: enhanced split/type/expected path pipeline.
- `runtime/aircraft_parallel_0303.py`.

실측:

- 단일 일반 fallback 평균 약 2.81초.
- 3옵션 일반 계열 pipeline 평균 약 14.89초.
- `collabReexecuteInputRefresh` option 4/5/6 variant 평균 약 8.38/5.04/4.57초.

TODO:

- [ ] 일반 variant 별 phase를 강제 기록한다.
  - divide_and_pattern
  - build_0301
  - collect_missions
  - hybrid build
  - 0303 build
  - 0304 build
  - pathID mapping
  - build_0302
  - write_0302
  - write_FlightPath
  - write_0301
- [ ] option-independent parsing/cache만 공유한다.
  - source 0201/0203 load
  - vehicle status filter
  - mission whitelist
  - current snapshot input
- [ ] option-dependent 결과는 검증 전 공유하지 않는다.
  - split result
  - type-decider result
  - expected paths
  - recon area review
- [ ] 3옵션 병렬화를 확대하기 전에 variant별 ID/waypoint block을 먼저 분리한다.
- [ ] `_evaluate_general_parallel_safety` 조건을 runtime 설정으로 노출하되, 기본값은 현재 안전 동작 유지.

구현 분할:

1. 계측/benchmark.
2. ID/waypoint block 선예약.
3. option-independent cache 공유.
4. current hybrid 위치 조정.
5. worker 수 확대.

검증:

- [ ] option code `6/4/5` 순서가 유지되는지 확인.
- [ ] 각 option의 MissionPlanID가 요청 pending plan ID와 일치하는지 확인.
- [ ] 0303/0304 FlightPath 누락/중복 pathID 검증.
- [ ] single option fallback도 동일 validator를 통과해야 한다.

## 11. 현재 잔여임무 hybrid 재계획

현재 코드:

- `mission_planning_gui.py`: `_build_current_remaining_hybrid_request`, `_apply_current_remaining_hybrid_to_variant`.
- `pipelines/current_remaining_hybrid.py`.
- `pipelines/current_remaining_hybrid_replan.py`.

실측:

- 일반 3옵션 계열 안에 포함되어 측정됨.
- current remaining hybrid가 켜지면 generic 0303 skip/merge/pathID mapping이 추가됨.

TODO:

- [ ] hybrid build 시간을 별도 계측한다.
- [ ] `_CURRENT_REMAINING_HYBRID_BUILD_LOCK`이 실제로 variant 병렬성을 얼마나 막는지 계측한다.
- [ ] option-independent hybrid 결과를 공유할 수 있는지 판단한다.
  - entryAircraftList, currentInputMissionID, sourcePlanID가 동일한 경우만 후보.
  - option 4 recon runtime override가 hybrid 결과에 영향을 주면 공유 금지.
- [ ] generic 0303 skip 이후 missing FlightPath가 생기지 않도록 validator를 강제한다.
- [ ] merge 후 pathID mapping은 local reserved block으로 처리한다.

구현 분할:

1. hybrid timer 추가.
2. lock wait 계측.
3. 공유 가능성 판정 로그.
4. safe cache 적용.
5. lock 범위 축소.

검증:

- [ ] currentInputMissionID가 trigger 시점 active mission과 일치하는지 확인.
- [ ] entryAircraftList aircraftID 4/5/6만 사용.
- [ ] hybrid path와 generic path가 중복되지 않는지 확인.

## 12. 일반 잔여임무 hybrid 재계획

현재 코드:

- `mission_planning_gui.py`: `_apply_remaining_hybrid_customization`.
- `pipelines/general_remaining_hybrid_replan.py`.
- `modules/common/mission_area_replan_store.py`.

실측:

- 독립 pipeline timer 없음. 일반 fallback variant total에 포함됨.

TODO:

- [ ] 적용 여부와 elapsed를 명시 로그로 남긴다.
  - applied
  - skipped reason
  - inputMissionID
  - aircraftIDs
  - generated path count
- [ ] snapshot mutated일 때만 수행되는 현재 조건을 유지한다.
- [ ] mission area snapshot carry-forward가 plan lineage를 깨지 않는지 확인한다.
- [ ] current/remaining geometry가 source 0201의 inputMissionID와 일치하는지 validator를 추가한다.

구현 분할:

1. 계측.
2. validator.
3. source cache 적용.
4. ID reservation 적용.

검증:

- [ ] snapshot applied/marked_done 케이스.
- [ ] no snapshot mutation skip 케이스.
- [ ] mission area snapshot이 새 planID로 carry-forward 되는지 확인.

## 13. 현재임무 재수행/첫 임무 hybrid 재계획

현재 코드:

- `mission_planning_gui.py`: `_build_current_remaining_hybrid_request`.
- `planner_mode = "reexecute_first_mission"` when `trigger=0201` and `triggerType=collabReexecuteInputRefresh`.
- `pipelines/reexecute_first_mission_hybrid.py`.

실측:

- `collabReexecuteInputRefresh` 7건에서 option 4/5/6 평균 8.38/5.04/4.57초.

TODO:

- [ ] reexecute-first mode와 current-remaining mode의 입력 차이를 문서화한다.
  - `reexecuteSourceInputMissionID`
  - `currentInputMissionID`
  - `entryAircraftList`
  - `representativeEntryCoordinate`
- [ ] variant별 hybrid request isolation은 유지한다.
- [ ] active current resolver가 inputMissionIDList[0]로 회귀하지 않도록 회귀 테스트를 둔다.
- [ ] reexecute source input과 current input이 다를 때 generic skip 범위를 잘못 잡지 않는지 확인한다.
- [ ] option 4 recon runtime override가 reexecute hybrid 결과에 미치는지 측정한다.

구현 분할:

1. 계측/로그 보강.
2. input validator.
3. ID reservation.
4. hybrid 공유/lock 최적화.

검증:

- [ ] currentInputMissionID 회귀 방지.
- [ ] 3 UAV entry coordinate 변주.
- [ ] option 4/5/6 각각 FlightPath 링크 검증.

## 14. 정찰 특화 옵션 재계획

현재 코드:

- `modules/common/option_codes.py`: option code `4`.
- `pipelines/recon_specialized_pipeline.py`.
- `MissionPlanner/planning_enhanced/pipeline.py`: `recon_area_review`.
- `MissionPlanner/planning_enhanced/algo/area_review.py`.

실측:

- current reexecute option 4 평균 약 8.38초.
- `recon_area_review`는 평균 0.46초 수준이지만, area piece 증가 후 0303 build/variant total이 커진다.
- abnormal health 케이스에서는 0303 build가 약 4.46초로 큼.

TODO:

- [ ] recon area review 전/후 piece/path/waypoint 증가량을 항상 기록한다.
- [ ] split cap 정책을 명확히 한다.
  - `recon_area_review_max_split_count`
  - `recon_area_review_min_segment_m`
  - `maxSegmentM`
- [ ] option 4 path 폭증을 막는 post-review merge 또는 cap을 검토한다.
- [ ] 정확도 저하 없이 줄일 수 있는 조건만 적용한다.
  - 아주 짧은 segment merge
  - 같은 aircraft/inputMissionID 연속 segment merge
  - min segment threshold
- [ ] option 4 전용 worker cap이 병렬성을 막는지 계측한다.

구현 분할:

1. 계측/로그 보강.
2. split cap 설정 노출.
3. path explosion guard.
4. 정확도 회귀 테스트.
5. worker cap 조정.

검증:

- [ ] piece count 증가가 설정 cap을 넘지 않는지 확인.
- [ ] 정찰 coverage/line search 품질 지표가 기준 이하로 떨어지지 않는지 확인.
- [ ] option 4와 option 6 결과가 잘못 동일해지지 않는지 확인.

## 15. 0303 FlightPath build 공통 병목

현재 코드:

- `runtime/aircraft_parallel_0303.py`: aircraft 단위 ThreadPoolExecutor.
- `MissionPlanner/data_def/d0303.py`: `_WPAllocator`, `build_flight_plans`.

실측:

- abnormalHealthUnavailable 계열 0303 build 평균 약 4.31~4.47초.
- current reexecute 계열은 케이스에 따라 0303 build가 크게 튐.

TODO:

- [ ] `build_0303_flight_plans_aircraft_parallel` 내부를 더 쪼개 계측한다.
  - group missions
  - per-aircraft build
  - sort by input order
  - normalize timestamps
  - reassign waypoint IDs
- [ ] 기본 worker 수 2가 UAV 3대 케이스에서 병목인지 확인한다.
- [ ] worker 수를 3으로 올리기 전 waypoint ID block 분리가 반드시 선행되어야 한다.
- [ ] per-aircraft worker 안에서 전역 `_next_waypoint_id()`가 호출되지 않는지 테스트한다.
- [ ] `reassign_waypoint_ids_inplace`가 큰 병목이면 local block allocator와 정확한 waypoint count 추정 전략을 검토한다.

구현 분할:

1. 세부 계측.
2. waypoint block 분리.
3. worker=3 실험.
4. reassign 최적화.

검증:

- [ ] 모든 waypointID unique.
- [ ] nextWaypointID 체인 valid.
- [ ] 0303/0304 동시 생성 시 waypointID 충돌 없음.

## 완료 조건

- [ ] 각 재계획 종류별 최소 1개 replay sample 확보.
- [ ] trigger별 p50/p95/max를 변경 전/후 비교.
- [ ] ID allocator lock/scan/write 시간이 로그로 보임.
- [ ] module log에 process lifecycle, pipeline checkpoint, branch decision, fallback/no-op/failure reason이 남음.
- [ ] logging overhead가 replay p95 기준 100ms 이하이고, 단일 log enqueue p95가 1ms 이하.
- [ ] module log queue full/drop 발생 시 drop count가 저주기로 남고 재계획은 중단되지 않음.
- [ ] 멀티프로세스 10개가 동시에 ID reserve를 호출해도 중복 0건.
- [ ] 깨진 `id_tracker.json`, 누락된 `path_usage.json`, 대량 FlightPath 존재 상태에서도 next ID가 후퇴하지 않음.
- [ ] 병렬 worker 안의 전역 `_next()` 호출이 0건인 pipeline부터 병렬화 적용.
- [ ] 모든 생성 산출물이 공통 validator 통과.
- [ ] 의도적으로 만든 duplicate pathID, orphan FlightPath, aircraft mismatch, missing FlightPath가 저장 전에 실패.
- [ ] 한국어 option label이 정확한 int code로 매핑되고 mojibake 회귀 없음.
- [ ] 0901/0903/0702/0305/0001 전달 정책 회귀 없음.
- [ ] 공격/post-attack direct 경로에서 delivery transaction이 정확히 한 번씩만 기록됨.
- [ ] 기존 no-op/failure 문구와 queue 완료 정책 회귀 없음.
- [ ] 새 TODO를 구현 PR 단위로 쪼갤 때, 각 PR은 단독 실행/롤백 가능해야 함.
