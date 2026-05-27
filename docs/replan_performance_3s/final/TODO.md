# 재계획 3초 성능 최종 TODO

이 TODO는 `README.md`/`TODO.md`와 `claude/*`를 통합한 실행 순서다. 기존 P0/P1/P2를 그대로 따르지 않고, 병렬화 선행조건을 P0로 승격했다.

## 진행 메모

- 2026-04-28 1차 적용: 기능 동작 변경 없이 관측/안전성 패치만 반영했다.
  - `id_allocator._load(path=None)`의 `target` 미정의 문제 보정 및 실패 warning/timing 로그 추가
  - mission planning 재계획 수명주기 로그 추가: `0902_received`, `pipeline_scheduled`, `scheduled_start`, `pipeline_done`, `0301_sent`, `0305_status_1/2`, `0901_sent`, `0903_sent`
  - monitoring queue 완료 로그에 `active_ms` 표시
  - next-collab/prior dedicated pipeline에 append-only `timingMs` 기록 추가
  - worker 수, ID 선예약 구조, FlightPath 생성 방식, delivery grace는 아직 변경하지 않음
- 2026-04-28 sim smoke 확인: 내부 sim에서는 `0902` archive가 남지 않지만, 현재 적용분의 기능 흐름은 정상 동작하는 것으로 확인했다.
  - replay benchmark는 실제 `0902` payload 확보 방식 또는 별도 capture hook/synthetic replay 입력이 필요하다.
- 2026-04-28 2차 적용: 재계획 variant 시작/종료 계측과 attack pipeline phase timer를 추가했다.
  - 일반/병렬/공격배제 variant에 `variant_started`, `variant_finished` 로그 추가
  - 병렬 core 완료는 `variant_core_finished`로 별도 기록하고, 저장 완료 시 `variant_finished` 기록
  - attack pipeline `timingMs`에 `read_source`, `descriptor_build`, `write_artifacts`, `delivery_wait` phase 기록 추가
- 2026-04-28 3차 적용: 내부 sim처럼 송신 sidecar가 남지 않는 경로를 위해 GUI 수신 직후 `0902` capture hook을 추가했다.
  - 기본 저장 위치는 기존 `DSS_Internal/replan_request_transport`이며, timestamp가 없으면 `DSS_Internal/replan_request_archive`에 fallback 저장
  - `REPLAN_CAPTURE_0902=0`으로 capture를 끌 수 있다.
- 2026-04-28 4차 적용: 일반 3옵션 병렬 생성 앞단에 safety gate와 worker 수 설정을 추가했다.
  - `REPLAN_VARIANT_PARALLEL=0`이면 즉시 sequential fallback
  - `REPLAN_VARIANT_WORKERS`로 worker 수를 제한 가능
  - current-remaining hybrid, remaining snapshot/collapse mutation, source artifact 누락, ID allocator reserve 불가 시 sequential fallback
- 2026-04-28 5차 적용: P0 병렬 안전성과 replay benchmark 기반을 마감했다.
  - parallel variant core의 GUI/Qt 직접 접근과 직접 로그 emit을 제거하고, worker 로그/타이밍은 buffer 후 main/publish 단계에서 flush
  - worker 내부 최종 publish ID 예약을 제거하고, pathID 매핑/검증은 `_store_general_variant()` parent 단계로 이동
  - 병렬 variant별 `0303`/`0304` waypointID block을 parent에서 분리 선예약해 worker에 전달
  - `REPLAN_VARIANT_WAYPOINT_BLOCK_SIZE`로 builder별 waypoint block 크기를 조정 가능
  - option `4` 포함 시 hard-coded worker 2 제한을 제거하고, 필요 시 `REPLAN_RECON_WORKER_CAP=2`로만 제한
  - `tools/replan_replay_benchmark.py` 추가: captured `0902` payload inventory, synthetic fallback, `--command` replay, `[REPLAN][TIME]` log SLA 집계 지원
- 2026-04-28 6차 적용(P1-1): 실측 기준 확보용 metric 로그와 benchmark 집계를 추가했다.
  - option `4`의 `review_assigned_areas_local()` 전후 `pieces/areaPieces/maxSegmentM/FOV/sweepSeparation` metric 기록
  - review 결과의 `targets/localized/changed_details/split_count_sum/max_split_count/max_projected_span_m` 기록
  - variant별 `FlightPath` path 수, waypoint 수, max waypoint/path, empty/formation path 수 기록
  - `tools/replan_replay_benchmark.py --log`가 `[REPLAN][METRIC]` 행을 name별 numeric p50/p95/max로 집계
- 2026-04-28 7차 적용(P1-2): 재계획 1회 단위 source artifact cache를 추가했다.
  - GUI 일반 경로와 dedicated next-collab/prior/attack 경로의 source MissionPlan/InputMissionPlan/IMP/FlightPath 반복 JSON 로드를 cache context로 공유
  - cache는 재계획 1회 scope에서만 유지하고, 기본 반환값은 deep copy로 유지해 기존 mutation 흐름과 격리
  - `pathID -> waypointIDs` 조회는 source FlightPath cache를 사용하도록 연결
  - `[REPLAN][METRIC] source_artifact_cache entries/hits/misses`와 plan summary의 `sourceArtifactCache`로 cache 효과를 관측
- 2026-04-28 8차 적용(P1-3): current-remaining hybrid 선적용을 안전 조건부로 반영했다.
  - `current_remaining_hybrid.py`, `current_remaining_hybrid_replan.py` 입력 계약 확인 스크립트 추가
  - hybrid 생성이 먼저 성공한 경우에만 해당 UAV/current input의 generic `0303` FlightPath 입력을 제외
  - hybrid 생성 실패 시 기존 generic `0303/0304` 경로를 그대로 유지해 기능 fallback 보존
  - hybrid merge 위치를 제거된 current 임무 자리로 바꿔 완료된 prefix와 후속 임무 순서 보존
- 2026-04-28 9차 적용(P1-4): option `4` path 폭증 원인 분석 리포트를 benchmark에 추가했다.
  - `tools/replan_replay_benchmark.py --log` 출력에 `logs.reconOptionAnalysis` 추가
  - `recon_area_review` 파라미터 세트와 이후 `flightpath_counts`/`flightpath_write`를 로그 라인 순서로 묶어 path/waypoint 통계 출력
  - `maxSegmentM`, FOV, sweep separation 변경 실험을 parameter set별로 비교할 수 있게 함
  - 정찰 coverage/촬영 품질 회귀 checklist를 리포트에 포함
- 2026-04-28 10차 적용(P1-5): option `4` path 수 제어를 runtime gate로 추가했다.
  - 기본값은 `recon_area_review_max_split_count=0`, `recon_area_review_min_segment_m=0.0`으로 기존 분할 동작 유지
  - 설정을 켠 경우에만 `review_assigned_areas_local()`의 local area split 수를 상한/최소 구간 길이 기준으로 제한
  - 제한 적용 여부를 `rawSplitCount`, `splitCapped`, `splitCapReason`과 `[REPLAN][METRIC] recon_area_review`로 기록
  - benchmark `reconOptionAnalysis`가 split cap 파라미터와 raw/capped split 통계를 parameter set별로 비교
- 2026-04-28 11차 적용(P1-6): `0303` 내부 aircraft 단위 병렬화를 안전 gate 조건부로 추가했다.
  - formation follower/leader 복제 경로가 감지되면 기존 sequential 생성으로 fallback
  - 일반 UAV mission은 aircraft별 worker에서 `0303` FlightPath를 생성하고, 최종 waypointID/nextWaypointID는 parent allocator로 재배정
  - `REPLAN_0303_AIRCRAFT_PARALLEL=0`으로 비활성화 가능, `REPLAN_0303_AIRCRAFT_WORKERS`로 worker 수 조정 가능
  - `[REPLAN][METRIC] flightpath_build_0303`로 build mode, worker 수, aircraft 수, fallback 사유, 재배정 waypoint 수를 기록
- 2026-04-28 12차 적용(P2-5/P2-6): 남은 보조 최적화와 회귀 체크리스트를 정리했다.
  - DEM terrain 조회는 기존 `lru_cache(maxsize=65536)`가 이미 적용되어 있어 새 memoize는 추가하지 않고 `terrain_cache_info()`/`terrain_precision_probe()` 계측 API만 추가
  - `Logs/Scenario_2026-04-28T164544/SBC3/FlightPath` 122개 waypoint 좌표 기준 1회차 hit/miss=12/110, 2회차 누적 hit/miss=134/110 확인
  - 같은 좌표 샘플에서 소수 4/5/6자리 rounding unique 수가 모두 110으로 동일해 precision key 조정은 현재 보류
  - ProcessPoolExecutor는 현재 병목 근거가 IO/serialization/grace/UI 쪽에 있고 Windows spawn/ID allocator/Qt 경계 위험이 커서 보류
  - 같은 로그 산출물 기준 MissionPlanID/IMP package ID/pathID/waypointID uniqueness와 `nextWaypointID` chain 정상 확인
- 2026-04-28 P2 재점검 보강:
  - 내부 재계획 0201 snapshot(`DSS_Internal/replan_inputs`)도 runtime debug artifact mode를 따르도록 `write_debug_json()` 적용
  - 기본 pretty 모드는 기존 저장 유지, `REPLAN_RUNTIME_ARTIFACT_MODE=compact|off`에서만 축소/생략

## 현재 상태 요약

| 항목 | 상태 | 비고 |
| --- | --- | --- |
| SLA 기준 확정 | 완료 | 계획 산출 SLA와 운용/direct SLA를 분리 고정 |
| trigger 분류표 | 완료 | 3옵션/2옵션/single dedicated-direct 분류 기준 유지 |
| end-to-end timing | 완료 | 주요 수명주기와 variant별 started/finished 로그 추가 완료 |
| dedicated phase timer | 완료 | next-collab/prior/attack timingMs 추가 완료 |
| archived 0902 replay benchmark | 완료 | capture hook + CLI/harness + synthetic fallback 완료 |
| id_allocator correctness | 완료 | `_load(path=None)` 보정, warning/timing 로그 추가, 시작 ID 50 규칙 유지 |
| 병렬화 safety gate | 완료 | worker core self/GUI 접근 제거, publish ID parent 예약, 실패 시 미송신 |
| ReplanReservationPlan | 완료(P0 경량) | 최종 publish ID parent 예약 + variant/builder waypoint block 선예약. 별도 class 추출은 필요 시 P1 |
| 0303/0304 waypoint block 분리 | 완료(P0 병렬 경로) | 병렬 variant에서 0303/0304 builder별 waypoint block 분리 |
| current-remaining hybrid 선적용 | 완료(P1-3) | hybrid 성공 시에만 generic 0303 skip, 실패 시 기존 generic fallback 유지 |
| variant worker 3개 동시 실행 | 완료 | 기본 `REPLAN_VARIANT_WORKERS=3`, option4 hard cap은 `REPLAN_RECON_WORKER_CAP`로 설정화 |
| source artifact cache | 완료(P1-2) | 재계획 1회 scope cache, 기본 deep copy 반환, dedicated helper와 context 공유 |
| option 4 path 폭증 분석 | 완료(P1-4) | benchmark `reconOptionAnalysis`로 parameter set별 path/waypoint 비교 |
| option 4 path 수 제어 | 완료(P1-5) | 기본값 꺼짐, runtime 설정 시에만 area review split cap 적용 및 로그/benchmark 비교 |
| 0303 내부 aircraft 단위 병렬화 | 완료(P1-6) | formation 경로는 sequential fallback, 일반 UAV는 aircraft별 생성 후 waypointID parent 재배정 |
| DEM/terrain cache 효과 측정 | 완료(P2-5) | 기존 `lru_cache` hit/miss/precision probe 추가, 현재 좌표 precision 조정 보류 |
| ProcessPoolExecutor 검토 | 완료(P2-6, 보류) | CPU/GIL 병목 확정 전까지 미적용. Windows spawn/ID allocator/Qt 경계 위험이 큼 |
| 회귀 검증 | 부분 완료 | ID/chain/순서/option4 분석 테스트 완료. p95 최종 확정은 현재 코드 적용 후 실 sim/replay 필요 |

## P0 - 기준, 계측, 안전성

- [x] SLA 기준 확정 - 완료
  - 계획 산출 SLA: `0305_status_1_ms -> 0305_status_2_ms`, trigger별 p95 <= 3000 ms
  - 운용/direct SLA: `0902_received_ms -> 0901_sent_ms/0903_sent_ms`, direct apply trigger p95 <= 3000 ms
  - option-selection flow는 `0301/0901` 전달 grace가 포함되므로 운용/direct SLA와 분리 보고
  - benchmark는 두 기준을 모두 출력하되 서로 대체 기준으로 비교하지 않음

- [x] trigger 분류표 고정 - 완료
  - 3옵션 일반: 입력갱신, 현재임무 재수행, RTB, 강제명령
  - 2옵션 공격: 신규 표적 `0402`의 공격 특화/공격 배제
  - single dedicated/direct: next-collab, prior Level-4, priorClosedResume, imaging schedule, path deviation, quality-speed
  - DL risk Level-5는 prior Level-4 dedicated와 분리 계측
  - post-attack rejoin/attackClosedDestroyed는 신규 표적 2옵션과 분리

- [x] end-to-end timing 로그 추가 - 완료
  - [x] `0902_received_ms`
  - [x] `pipeline_scheduled_ms`
  - [x] `scheduled_start_ms`
  - [x] `0305_status_1_ms`
  - [x] `variant_started_ms`, `variant_finished_ms`
  - [x] `pipeline_done_ms`
  - [x] `0301_sent_ms`
  - [x] `0305_status_2_ms`
  - [x] `0901_or_0903_sent_ms`
  - [x] monitoring queue `active_ms`
  - [x] 기존 `0305 status=2 elapsed=` 로그는 호환성을 위해 유지

- [x] dedicated pipeline phase timer 추가 - 완료
  - [x] next-collab: append-only `timingMs` 기록 추가
  - [x] prior: append-only `timingMs` 기록 추가
  - [x] attack: 기존 timing 유지 상태에서 `read_source`/`descriptor_build`/`write_artifacts`/`delivery_wait` 추가

- [x] archived `0902` replay benchmark 추가 - 완료
  - trigger별 p50/p95/max 출력
  - `0305` 기준과 `0902 -> 0903` 기준 모두 출력
  - benchmark는 시간 측정 용도임을 명시
  - 기본 모드는 payload/log 읽기만 수행해 DB를 변경하지 않음
  - [x] 내부 sim에서 `0902` archive가 남지 않는 경로를 위한 GUI capture hook 추가
  - [x] capture payload를 재실행하는 replay CLI/harness 작성: `tools/replan_replay_benchmark.py --command "... {payload} ..."`
  - [x] synthetic replay 입력 fallback 작성: `--synthetic` / `--synthetic-only`

- [x] `id_allocator` correctness 보정 - 완료
  - [x] `_load(path=None)`의 미정의 `target` 사용 수정
  - [x] load 실패를 조용히 `{}`로 삼키지 않도록 warning/timing 로그 추가
  - [x] waypoint 시작 ID 50 규칙 유지 확인
  - [x] 기존 tracker/usage 파일과 충돌 없음 확인

- [x] 병렬화 safety gate 추가 - 완료
  - [x] worker core에 Qt 객체, GUI widget, NodeMessenger 객체 전달 금지
    - `_run_general_variant_core()` 내부 `self.*` 직접 접근 없음 확인
    - worker log/timing은 `log_messages`/`timing_events`로 buffer 후 parent에서 flush
  - [x] worker output은 variant temp root에만 기록
  - [x] 검증 성공 후 main thread store/publish
  - [x] 실패/취소 시 `0305 status=2`, `0901`, `0903` 미송신
  - [x] current-remaining hybrid, snapshot/collapse mutation 시 sequential fallback
  - [x] `REPLAN_VARIANT_PARALLEL=0` 강제 sequential fallback
  - [x] `REPLAN_VARIANT_WORKERS` worker 수 제한
  - [x] `REPLAN_VARIANT_WAYPOINT_BLOCK_SIZE` builder별 waypoint block 설정
  - [x] option `4` worker hard cap을 `REPLAN_RECON_WORKER_CAP` 설정으로 전환
  - [x] in-process worker에서 module reload/global constant mutation 금지

- [x] `ReplanReservationPlan` 도입 - 완료(P0 경량)
  - [x] parent publish 단계에서 MissionPlanID 예약
  - [x] parent publish 단계에서 variant별 IMP ID 예약
  - [x] parent publish 단계에서 aircraft별 pathID 예약/매핑/검증
  - [x] 병렬 variant별 `0303`/`0304` waypointID block 선예약
  - [x] worker 산출물은 temp root에만 쓰고 최종 ID는 publish 단계에서 확정
  - [x] 실패한 variant의 예약 ID는 재사용하지 않음
  - P1 note: 별도 `ReplanReservationPlan` class 추출은 리팩터링 후보

- [x] `0303/0304` waypoint block 분리 - 완료(P0 병렬 경로)
  - [x] 일반 재계획 hot path의 `0303 -> 0304` 순차 구조 유지
  - [x] 병렬 variant에서 `d0303`용 waypoint range와 `d0304`용 waypoint range 분리
  - [x] `_WPAllocator(start, end)` block 상한 검증 추가
  - [x] `nextWaypointID` relink는 기존 builder 후처리 보존
  - [x] `d0304.apply_uav_eta_follow_speed_plan()` 후처리 보존
  - P1 note: 별도 `export_0303_0304.py` helper의 동시 submit 경로 정리

- [x] variant worker 3개 동시 실행 설정화 - 완료
  - [x] `REPLAN_VARIANT_WORKERS` 환경변수로 worker 수 제한 가능
  - [x] option `4` 포함 시 worker 2 제한을 runtime 설정으로 전환: `REPLAN_RECON_WORKER_CAP`
  - [x] P0 safety gate와 ID block 분리 완료 후 기본 worker 3 허용
  - [x] 병렬 완료 순서와 무관하게 option 순서 `(6, 4, 5)`와 missionPlanID 매핑 고정
  - [x] 3개 variant `variant_started` timing으로 시작 간격 검증 가능

## P1 - 주요 성능 개선

- [x] P1-1 실측 기준 확보 - 완료
  - [x] 실제 sim 로그의 `[REPLAN][TIME]` SLA 집계 경로 유지
  - [x] option `4` `review_assigned_areas_local()` 전후 segment/piece 수 기록
  - [x] option `4` `max_segment_m`, FOV, sweep separation 기록
  - [x] variant별 FlightPath 파일 수와 waypoint 수 기록
  - [x] benchmark `--log`가 metric 로그를 집계하는지 샘플 검증
  - 남은 확인: 실제 sim 1회 수행 후 `tools\replan_replay_benchmark.py --log <로그파일>` 결과 저장

- [x] current-remaining hybrid 선적용 - 완료(P1-3)
  - [x] `current_remaining_hybrid.py`, `current_remaining_hybrid_replan.py` 입력 계약 테스트 작성
  - [x] hybrid 대상 UAV/current input에서 generic `0303` 생성을 skip
  - [x] source/current inputMissionID mapping과 completed prefix 보존 검증
  - [x] hybrid 생성 실패 시 기존 generic `0303/0304` fallback 유지
  - [x] P0에서는 기능 의미 변경을 피하기 위해 current-remaining/snapshot/collapse 경로를 sequential fallback으로 보호
  - 남은 확인: 실제 sim current-remaining trigger 1회에서 `current remaining generic 0303 skipped` 로그와 최종 FlightPath 누락 없음 확인

- [x] option `4` path 폭증 원인 분석 - 완료(P1-4)
  - [x] `review_assigned_areas_local()` 전후 segment/piece 수 기록
  - [x] FlightPath 파일 수와 waypoint 수 기록
  - [x] `max_segment_m`, FOV, sweep separation 관측값 기록
  - [x] `max_segment_m`, FOV, sweep separation 변경 영향을 replay log parameter set별로 비교
  - [x] 정찰 coverage/촬영 품질 회귀 checklist 정의
  - 남은 확인: 서로 다른 runtime parameter로 option `4` sim/replay 2회 이상 수행 후 `logs.reconOptionAnalysis.parameterSets` 비교

- [x] option `4` path 수 제어 - 완료(P1-5)
  - [x] split이 정책성 과분할이면 threshold/runtime 설정화: `recon_area_review_max_split_count`, `recon_area_review_min_segment_m`
  - [x] 기본값은 0으로 두어 기존 기능/품질 의미를 유지하고, 실험 시에만 제한 적용
  - [x] 제한 전 raw split 수와 제한 후 split 수를 모두 기록해 품질 회귀 판단 근거 보존
  - [x] benchmark `reconOptionAnalysis`에서 split cap 파라미터와 `split_capped_count`/`raw_split_count_sum` 비교 가능
  - 남은 확인: option `4` sim/replay에서 제한값을 켠 parameter set과 기본 parameter set의 path/waypoint 수 및 coverage/촬영 품질 비교

- [x] `0303` 내부 aircraft 단위 병렬화 - 완료(P1-6)
  - [x] aircraft 간 독립성 재확인: `d0303`의 상태는 aircraft별 tail/area 상태로 분리되어 있어 일반 UAV mission은 분리 가능
  - [x] formation follower가 leader waypoint list를 복제하는 경로는 예외 처리: formation-like mission 감지 시 sequential fallback
  - [x] aircraft worker별 local waypoint range 분리: worker 내부는 local dummy allocator 사용
  - [x] worker별 packet 생성 후 최종 relink: parent allocator로 waypointID/nextWaypointID 재배정
  - [x] `REPLAN_0303_AIRCRAFT_PARALLEL`, `REPLAN_0303_AIRCRAFT_WORKERS` 설정 및 `flightpath_build_0303` metric 추가
  - 남은 확인: 실제 3 UAV sim/replay에서 `buildMode=aircraft_parallel` 여부, waypointID uniqueness, `nextWaypointID` chain validity 확인

- [x] 재계획 1회 단위 source artifact cache 추가 - 완료(P1-2)
  - [x] source `MissionPlan`
  - [x] source `InputMissionPlan`
  - [x] source IMP payload
  - [x] source FlightPath payload
  - [x] `pathID -> waypointIDs`
  - [x] next-collab/prior/attack helper가 같은 cache context 공유
  - [x] cache hit/miss/entry metric 기록

- [x] dedicated/direct 흐름 최적화
  - next-collab/prior/attack phase timer 유지, direct delivery 병목은 post-`0301` grace로 분리
  - prior Level-4와 DL risk Level-5 trigger/SLA 분리
  - attack 신규 표적 2옵션과 post-attack direct 흐름 분리
  - path-deviation/quality-speed direct flow는 `0902 -> 0903` 기준으로 측정

- [x] force-direct post-`0301` grace 축소
  - `0901/0701` 옵션 선택 흐름은 기존 grace 유지
  - direct `0903`만 기본 250ms grace 후보로 축소
  - `0301` 송신 성공, `0305 status=2` 송신/mark 확인 후 flush
  - 수신측 race window는 `completion_ready` gate와 `post_0301_*` timing event로 검증

## P2 - 보조 최적화

- [x] `push_center` hot message module resolution cache
  - 대상: `0301`, `0305`, `0902`, `0903`
  - warm-up 후 importlib 반복 호출 0회 목표

- [x] `0902` sidecar disk IO 토글 - 완료(P2-2)
  - 기본값은 현재 동작 유지
  - `REPLAN_0902_SIDECAR_MODE=compact`에서 compact JSON 저장
  - `REPLAN_0902_SIDECAR_MODE=off` 계열 값에서 송신/GUI capture sidecar 생략
  - audit/debug 모드에서는 기존 pretty JSON 유지

- [x] runtime 성능 모드 debug artifact 축소 - 완료(P2-3)
  - 기본/debug mode: full pretty artifact 유지
  - `REPLAN_RUNTIME_ARTIFACT_MODE=performance|compact`에서 debug artifact compact write
  - `REPLAN_RUNTIME_ARTIFACT_MODE=off` 계열 값에서 debug artifact 생략
  - 내부 재계획 0201 snapshot(`DSS_Internal/replan_inputs`)도 동일 mode 적용
  - 공식 산출물 `MissionPlan`/`IndividualMissionPlan`/`FlightPath` pretty write는 변경하지 않음

- [x] monitoring queue/UI 보조 최적화 - 완료(P2-4)
  - payload deep-copy 비용을 `replan_queue.stats.copy_metrics`로 계측
  - queue 내부 dispatch result 중 실제 송신에 쓰지 않는 payload deep-copy 생략
  - `ReplanQueueTab` snapshot copy/refresh 비용 계측 및 refresh throttle 적용
  - 8~12초 병목의 주 해결책으로 간주하지 않음

- [x] DEM/terrain cache 효과 측정 - 완료(P2-5)
  - 새 memoize 추가 없이 기존 `_terrain_elev_cached`/`_load_dem_data`/`_available_dem_tiles` `lru_cache` 상태를 조회하는 `terrain_cache_info()` 추가
  - 테스트/분석용 `clear_terrain_cache()`와 `terrain_precision_probe()` 추가
  - `Logs/Scenario_2026-04-28T164544/SBC3/FlightPath` 122개 waypoint 좌표 기준: 1회차 hit/miss=12/110, 2회차 누적 hit/miss=134/110
  - 같은 좌표 샘플에서 소수 4/5/6자리 rounding unique 수가 모두 110이라 현재 precision key 조정 이득 없음

- [x] ProcessPoolExecutor 검토 - 완료(P2-6, 보류)
  - phase timer/가용 로그 기준 현재 직접적인 CPU/GIL 병목보다 IO/serialization/delivery grace/UI 비용이 우선순위
  - Windows spawn 비용, child process import 비용, JSON 직렬화 비용을 감수할 만큼 순수 CPU phase가 아직 확인되지 않음
  - child process에서 global ID allocator, Qt/GUI 객체, runtime reload/global mutation이 섞이면 기능 회귀 위험이 큼
  - 실제 captured replay에서 p95가 계속 3초 초과하고 특정 순수 CPU phase가 지배적일 때만 별도 P3로 재검토

## 회귀 검증

- 2026-04-28 sub-agent 포함 재검증:
  - SLA/benchmark 검증 agent와 산출물/계약 검증 agent가 read-only로 독립 확인
  - 로컬 재실행: `tools/replan_replay_benchmark.py`, `tools/test_replan_option4_analysis.py`, `tools/test_recon_area_review_controls.py`, `tools/test_0303_aircraft_parallel_contract.py`, `tools/test_current_remaining_hybrid_contract.py`, `tools/test_source_artifact_cache.py`, `tools/test_monitoring_queue_perf.py`, `tools/test_runtime_debug_artifacts.py`, `python -m compileall -q ...`
  - `164544` 로그 집계 명령: `tools/replan_replay_benchmark.py --input Logs/Scenario_2026-04-28T164544/SBC3/DSS_Internal/replan_request_transport --log Logs/Scenario_2026-04-28T164544/SBC3/DSS_Internal/module_logs/mission_planning.log`
  - `164544` 계획 SLA `0305_status_1 -> 0305_status_2`: count=2, p95=1032.315ms, max=1053.5ms
  - `164544` 운용 `0902 -> 0903`: count=2, p95=4159.27ms. 단, trigger가 `unknown`으로만 집계되고 현재 force-direct 검증용 로그가 아님
  - `194908` 로그 집계 명령: `tools/replan_replay_benchmark.py --log Logs/Scenario_2026-04-28T194908/SBC3/DSS_Internal/module_logs/mission_planning.log --synthetic-only --repeat 3 --limit 3`
  - `194908` 계획 SLA: count=1, p95=1098.5ms, max=1098.5ms
  - `194908` 운용 `0902 -> 0903`: count=1, p95=4233.9ms. 이 로그는 `force_direct=0`, `mode=option_or_0901`, `grace_ms=1800`, `timeout_ms=3000`의 option/timeout flush 경로라 direct grace 축소 검증값으로 사용하지 않음
  - `164544+194908` 결합 계획 SLA: count=3, p95=1094.0ms, max=1098.5ms
  - `164544+194908` 결합 운용 `0902 -> 0903`: count=3, p95=4229.18ms. option/timeout delivery wait가 섞여 있어 direct SLA 통과/실패 판정에서 제외
  - 현재 검색 가능한 로그에는 `forceDirect=1` 또는 `mode=direct_0903` 기록이 없어 direct flow p95는 새 sim/replay가 필요
  - `194908` 산출물 기준 MissionPlanID 1개, IMP package ID 6개, FlightPath 24개, waypoint 113개, empty FlightPath 0개, ID uniqueness 및 `nextWaypointID` chain 정상
  - `164544` 산출물 기준 MissionPlanID/IMP package ID/FlightPath payload pathID/waypointID uniqueness 및 chain 정상. 다만 IMP mission entry 기준 pathID 재참조가 있어 `pathID uniqueness`는 FlightPath payload/file scope로 한정
  - 최신 두 로그에는 option 4 recon metric count가 0이므로 option 4 coverage/촬영 품질 최종 비교는 실제 option 4 sim/replay가 필요
  - 최신 두 로그에는 `0901` payload가 없어 option 순서와 공격 2옵션 label은 테스트/기존 송부 로그 기준 확인이며, 현재 코드 실행 `0901` payload 재확인은 별도 필요

- [ ] 입력갱신 3옵션 replay p95 3초 이내 - trigger가 분리된 현재 코드 captured replay 또는 실 sim 로그 필요
- [ ] 현재임무 재수행 3옵션 replay p95 3초 이내 - trigger가 분리된 현재 코드 captured replay 또는 실 sim 로그 필요
- [ ] RTB/강제명령 3옵션 replay p95 3초 이내 - trigger가 분리된 현재 코드 captured replay 또는 실 sim 로그 필요
- [ ] 신규 표적 공격 2옵션 replay p95 3초 이내 - 현재 코드 captured replay 또는 실 sim 로그 필요
- [ ] next-collab dedicated pipeline p95 3초 이내 - 현재 코드 captured replay 또는 실 sim 로그 필요
- [ ] prior Level-4 dedicated pipeline p95 3초 이내 - 현재 코드 captured replay 또는 실 sim 로그 필요
- [ ] direct flow `0902 -> 0903` p95 3초 이내 - `forceDirect=1` 또는 `mode=direct_0903` 새 sim/replay 로그 필요
- [x] MissionPlanID uniqueness - `164544`/`194908` 산출물 기준 정상
- [x] IndividualMissionPlanPackageID uniqueness - `164544`/`194908` 산출물 기준 정상
- [x] FlightPath payload/file `pathID` uniqueness 및 IMP pathID resolution - `164544`/`194908` 산출물 기준 정상. cross-plan IMP mission pathID 재참조는 scope에서 제외
- [x] waypointID uniqueness - `164544`/`194908` 산출물 기준 정상
- [x] `nextWaypointID` chain validity - `164544`/`194908` 산출물 기준 정상
- [x] option `(6, 4, 5)` 순서 유지 - `tools/test_replan_option4_analysis.py` 및 기존 송부 로그 `0901` payload 기준 확인
- [ ] 현재 코드 실행 `0901` payload의 option `(6, 4, 5)` 순서 재확인 - 최신 두 로그에는 `0901` payload 없음
- [x] 공격 2옵션 label 유지 - attack/post-attack 흐름 분리 테스트 및 기존 송부 로그 공격 `[2,3]` payload 기준 확인
- [ ] 현재 코드 실행 `0901` payload의 공격 2옵션 label 재확인 - 최신 두 로그에는 공격 `0901` payload 없음
- [x] `0301 -> 0305 status=2 -> 0901/0903` 순서 보존 - 최신 로그 3개 delivery group 및 기존 송부 로그 11개 delivery group 기준 순서 위반 0건
- [x] partial artifact 미송신 gate - 실패 시 `_schedule_plan_delivery()` 전 중단 구조와 parent publish 구조 기준 송신 gate 확인
- [ ] store-stage 실패 주입 시 official IMP/FlightPath partial artifact 정리 검증 - 별도 실패 주입 테스트 필요
- [x] option `4` coverage/촬영 품질 검증 경로 - 기본값은 기존 split 유지, metric/checklist/benchmark 경로와 `test_recon_area_review_controls.py` 통과
- [ ] option `4` coverage/촬영 품질 실측 비교 - 실제 option 4 sim/replay에서 기본값 vs 제한값 coverage/촬영 품질 비교 필요
