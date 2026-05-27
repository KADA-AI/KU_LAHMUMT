# 재계획 3초 성능 최종 공통 수정안

## 목적

이 문서는 `README.md`/`TODO.md`와 `claude/*` 검토를 하나로 합친 최종안이다. 두 산출물의 공통 결론은 같다.

- 기능은 줄이지 않는다.
- 일반 재계획의 기본 3옵션 `(6, 4, 5)`은 유지한다.
- 공격 신규 표적은 2옵션, dedicated/direct 계열은 single flow로 따로 본다.
- 3초 문제는 단일 스위치로 해결되지 않는다.
- 계측, ID 안전성, runtime 상태 격리, FlightPath 생성 구조를 순서대로 고쳐야 한다.

## 2026-04-28 P0 적용 상태

- SLA는 두 기준으로 고정한다.
  - 계획 산출: `0305_status_1_ms -> 0305_status_2_ms`, trigger별 p95 <= 3000 ms
  - 운용/direct: `0902_received_ms -> 0901_sent_ms/0903_sent_ms`, direct apply trigger p95 <= 3000 ms
- replay benchmark CLI는 `tools/replan_replay_benchmark.py`로 추가했다.
  - captured `0902` payload inventory
  - `--synthetic` / `--synthetic-only`
  - `--command "... {payload} ..."` replay command 측정
  - `[REPLAN][TIME]` 로그의 p50/p95/max 및 SLA delta 집계
- 일반 3옵션 병렬 경로는 safety gate 통과 시 `REPLAN_VARIANT_WORKERS=3` 기본값으로 동작한다.
- option `4` 포함 시 hard-coded worker 2 제한은 제거했고, 필요할 때만 `REPLAN_RECON_WORKER_CAP=2`로 제한한다.
- 병렬 variant worker core는 GUI/Qt 직접 접근 없이 산출하고, 로그/타이밍은 parent 단계에서 flush한다.
- 최종 publish ID는 parent 단계에서 확정한다. 병렬 variant의 `0303`/`0304` waypointID block은 parent에서 분리 선예약한다.
- current-remaining hybrid 선적용은 기능 의미 변경 위험이 있어 P1로 두고, P0에서는 해당 경로를 sequential fallback으로 보호한다.

## 2026-04-28 P1-1 적용 상태

- 실측 기준 확보용 `[REPLAN][METRIC]` 로그를 추가했다.
  - `recon_area_review_before`: option `4` area-review 전 pieces/areaPieces/maxSegmentM/FOV/sweepSeparation
  - `recon_area_review`: option `4` area-review 후 pieces, targets, localized, split count, max projected span
  - `flightpath_counts`: variant별 `0303/0304` path 수, waypoint 수, max waypoint/path, empty/formation path 수
  - `flightpath_write`: variant별 FlightPath write 파일 수와 write 시간
- replay benchmark는 `--log` 입력에서 `[REPLAN][TIME]`과 `[REPLAN][METRIC]`을 함께 집계한다.
- P1-1 완료 기준은 코드 계측/benchmark 파싱 검증까지이며, 실제 성능 판정은 sim 1회 후 `tools\replan_replay_benchmark.py --log <로그파일>` 결과로 한다.

## 2026-04-28 P1-2 적용 상태

- 재계획 1회 scope의 source artifact cache를 추가했다.
  - GUI 일반 경로와 next-collab/prior/attack dedicated helper가 같은 cache context를 공유한다.
  - source `MissionPlan`, `InputMissionPlan`, IMP payload, `FlightPath`, `pathID -> waypointIDs` 조회를 cache 경유로 연결했다.
  - 기본 반환값은 deep copy라 기존 코드가 payload를 수정해도 cache 원본과 다른 흐름에 영향을 주지 않는다.
- 재계획 종료 시 `[REPLAN][METRIC] source_artifact_cache entries/hits/misses`를 남기고, plan summary에는 `sourceArtifactCache`를 기록한다.
- P1-2 완료 기준은 문법 컴파일, cache 단위 검증, synthetic benchmark 통과까지다. 실제 효과는 sim 1회 후 source cache hit/miss metric과 기존 SLA 로그를 함께 비교한다.

## 2026-04-28 P1-3 적용 상태

- current-remaining hybrid 선적용을 안전 조건부로 반영했다.
  - hybrid 생성이 먼저 성공한 경우에만 해당 UAV/current input의 generic `0303` FlightPath 입력을 제외한다.
  - hybrid 생성 실패 시에는 generic `0303/0304` 생성 경로를 그대로 유지한다.
  - hybrid merge는 제거된 current 임무 위치에 삽입해 완료된 prefix와 후속 임무 순서를 보존한다.
- `tools\test_current_remaining_hybrid_contract.py`로 `current_remaining_hybrid.py`와 `current_remaining_hybrid_replan.py`의 입력 계약, source/current inputMissionID mapping, completed prefix 보존을 확인한다.
- P1-3 완료 기준은 계약 테스트, 문법 컴파일, synthetic benchmark 통과까지다. 실제 효과는 current-remaining trigger sim 1회에서 `current remaining generic 0303 skipped` 로그와 최종 FlightPath 누락 없음으로 확인한다.

## 2026-04-28 P1-4 적용 상태

- option `4` path 폭증 원인 분석 리포트를 benchmark에 추가했다.
  - `tools\replan_replay_benchmark.py --log <로그파일>` 결과의 `logs.reconOptionAnalysis`에서 확인한다.
  - `recon_area_review` 파라미터 세트와 이후 `flightpath_counts`/`flightpath_write`를 로그 라인 순서로 묶어 path/waypoint 통계를 낸다.
  - `maxSegmentM`, FOV, sweep separation을 바꿔 여러 번 실행하면 `parameterSets` 단위로 비교할 수 있다.
- 리포트에는 정찰 coverage/촬영 품질 회귀 checklist도 포함한다.
- P1-4 완료 기준은 synthetic option `4` metric log 테스트, 문법 컴파일, 기존 로그 파싱 통과까지다. 실제 원인 판정은 서로 다른 runtime parameter로 option `4` sim/replay 2회 이상 수행한 뒤 `parameterSets`를 비교한다.

## 2026-04-28 P1-5 적용 상태

- option `4` path 수 제어를 runtime gate로 추가했다.
  - 기본값은 `recon_area_review_max_split_count=0`, `recon_area_review_min_segment_m=0.0`이라 기존 area review 분할 동작을 그대로 유지한다.
  - 값을 켠 경우에만 `review_assigned_areas_local()`의 local area split 수를 최대 분할 수 또는 최소 구간 길이 기준으로 제한한다.
  - 제한 전 raw split 수와 제한 후 split 수를 `reviewArea` 및 `[REPLAN][METRIC] recon_area_review`에 같이 기록한다.
  - benchmark `logs.reconOptionAnalysis.parameterSets`가 split cap 파라미터와 `split_capped_count`/`raw_split_count_sum`을 함께 비교한다.
- P1-5 완료 기준은 split resolver 단위 테스트, option `4` metric log 테스트, 문법 컴파일, synthetic benchmark 통과까지다. 실제 적용값은 sim/replay에서 coverage/촬영 품질 회귀가 없는 범위로 정해야 한다.

## 2026-04-28 P1-6 적용 상태

- `0303` 내부 aircraft 단위 병렬화를 추가했다.
  - 일반 UAV mission은 aircraft별 worker에서 `d0303.build_flight_plans()`를 수행한다.
  - worker 내부는 local dummy waypoint allocator를 사용하고, 최종 output은 parent allocator로 waypointID/nextWaypointID를 다시 배정한다.
  - formation follower가 leader waypoint list를 복제하는 mission은 기존 sequential 생성으로 fallback한다.
  - `REPLAN_0303_AIRCRAFT_PARALLEL=0`으로 끌 수 있고, `REPLAN_0303_AIRCRAFT_WORKERS`로 worker 수를 조정한다. 기본 worker cap은 2다.
  - `[REPLAN][METRIC] flightpath_build_0303`에서 build mode, worker 수, aircraft 수, fallback 사유, 재배정 waypoint 수를 확인한다.
- P1-6 완료 기준은 helper 계약 테스트, 문법 컴파일, 기존 P1 테스트, synthetic benchmark 통과까지다. 실제 효과는 3 UAV sim/replay에서 `buildMode=aircraft_parallel`과 waypoint chain 검증으로 확인한다.

## 최종 판단

8~12초 초과의 1차 원인은 모니터링 GUI가 아니라 임무계획 산출 구조다. 특히 일반 3옵션 흐름에서 option `4`가 포함되면 variant worker가 2개로 제한되고, variant 내부 FlightPath 생성과 저장/ID 매핑이 충분히 병렬화되어 있지 않다.

다만 `max_workers=2`를 `3`으로 바꾸는 단독 수정은 금지한다. ID allocator, waypoint block, runtime 전역 상태, partial artifact publish 문제가 먼저 정리되지 않으면 병렬처럼 보이지만 lock 경합과 ID 충돌 위험만 커진다.

## 비판적 통합 결과

| 쟁점 | 기존 README/TODO | Claude 검토 | 최종 결론 |
| --- | --- | --- | --- |
| SLA 기준 | `0305`와 `0902 -> 0903`가 다르다고 지적 | SLA 확정을 최우선으로 올리자고 지적 | 맞다. 최우선은 SLA/계측 정의다. `0305 status=1 -> 2`와 `0902 -> 0903`를 둘 다 기록하되, 어떤 값을 3초 합격 기준으로 볼지 고정한다. |
| variant 병렬화 | option `4` 포함 시 worker 2 제한을 병목으로 봄 | ID 선예약 전 worker 확대는 위험하다고 지적 | worker 제한 해제는 맞는 방향이지만 P0 gate 통과 후 적용한다. |
| `0303/0304` 병렬화 | 일반 hot path가 순차라고 봄 | `export_0303_0304.py`에는 동시 submit이 있다고 지적 | 둘 다 맞다. 일반 재계획 core는 `mission_planning_gui.py`에서 순차 실행한다. 별도 helper 경로는 동시 submit이 있으나 allocator/runtime 상태 때문에 그대로 확장하면 위험하다. |
| ID block 선예약 | P1 최적화로 둠 | 병렬화 선행조건이라고 지적 | P0 gate로 승격한다. parent에서 ID range를 예약하고 worker는 local pool만 소비한다. |
| current-remaining hybrid | generic FlightPath 생성 후 덮어쓰는 낭비 지적 | 입력 가정 검증 후 선적용해야 한다고 지적 | 살린다. 단, `current_remaining_hybrid*` 입력 계약 테스트를 먼저 만든 뒤 generic 생성을 skip한다. |
| option `4` path 폭증 | 27 -> 80 수준의 관측을 병목으로 봄 | 원인이 split 정책인지 정찰 정확도 요구인지 미확정이라고 지적 | 맞다. 바로 병합하지 말고 `review_assigned_areas_local()` 출력 segment 수, coverage 품질, waypoint 수를 같이 계측한다. |
| DEM memoize | 새 최적화 후보로 둠 | 이미 cache가 있다고 지적 | TODO에서 "추가"가 아니라 "효과 측정/키 효율 점검"으로 낮춘다. 현재 `MissionPlanner/data_def/mission_helpers.py`에 `lru_cache`가 있다. |
| delivery grace | `0902 -> 0903` 병목으로 지적 | 이것만 줄이면 8~12초는 해결 안 된다고 지적 | direct 흐름에서만 후반 중위험 작업으로 둔다. 일반 8~12초 병목의 주 해결책은 아니다. |
| ProcessPool | CPU-bound 후보로 둠 | Windows spawn/pickle/allocator/process-safe 위험 지적 | 마지막 수단이다. phase timer로 GIL/CPU 병목이 확인될 때만 적용한다. |

## 공통 수정 방향

### 1. 계측과 기준을 먼저 고정

재계획 한 건마다 최소 다음 시점을 남긴다.

- `0902_received_ms`
- `scheduled_start_ms`
- `0305_status_1_ms`
- `variant_started_ms` / `variant_finished_ms`
- `pipeline_done_ms`
- `0301_sent_ms`
- `0305_status_2_ms`
- `0901_or_0903_sent_ms`

이후 replay benchmark가 trigger별 p50/p95/max를 같은 기준으로 출력해야 한다.

### 2. 병렬화 safety gate

병렬 worker 진입 전에 다음 조건을 만족해야 한다.

- worker에 Qt 객체, GUI widget, NodeMessenger 객체를 넘기지 않는다.
- worker는 global ID allocator를 직접 호출하지 않는다.
- parent가 MissionPlanID, IMP ID, pathID, waypointID range를 예약한다.
- worker는 할당받은 local pool만 소비한다.
- worker output은 variant temp root에 쓰고 검증 후 publish한다.
- partial artifact 상태에서는 `0305 status=2`, `0901`, `0903`을 보내지 않는다.
- in-process thread worker에서는 module reload/global constant mutation 경로를 금지한다.

### 3. ID 예약 구조

`ReplanReservationPlan`을 만든다.

- variant별 MissionPlanID
- variant/aircraft별 IMP ID
- aircraft별 pathID range
- builder별 waypointID range
- 실패 variant의 ID range는 재사용하지 않고 skip log만 남김

`id_allocator._load(path=None)`의 `target` 미정의 문제는 병렬화 전 correctness 수정으로 먼저 처리한다.

### 4. FlightPath 생성 구조

일반 재계획 hot path는 현재 `0303 -> 0304` 순차 호출이다. 이를 바로 worker만 늘리는 식으로 바꾸지 않고 다음 순서로 진행한다.

1. `d0303`/`d0304`가 외부 waypoint range 또는 local allocator를 받을 수 있게 한다.
2. `0303`과 `0304`에 서로 다른 waypoint block을 준다.
3. `nextWaypointID` relink를 최종 단계에서 검증한다.
4. `d0304.apply_uav_eta_follow_speed_plan()` 후처리 순서를 보존한다.
5. 이후 `0303` 내부 aircraft 단위 병렬화를 검토한다.

P1-6에서는 위 5번을 일반 UAV mission에 한해 적용했다. Formation flight처럼 leader/follower 간 waypoint 복제가 필요한 경우는 병렬 대상에서 제외하고 sequential fallback한다.

### 5. 불필요한 생성 제거

current-remaining hybrid는 generic `0303/0304` 생성 뒤에 덮어쓰는 구조라 낭비가 있다. 입력 계약을 확인한 뒤 hybrid 대상 variant는 generic FlightPath 생성을 skip하도록 바꾼다.

### 6. option `4`는 정확도 gate와 함께 줄인다

option `4` path 수를 줄이는 변경은 정찰 정확도를 건드릴 수 있다. 따라서 기본 동작은 유지하고, runtime gate를 켠 실험에서만 split 수를 제한한다. 다음 데이터를 함께 확인한다.

- `review_assigned_areas_local()` 전후 segment 수
- raw split 수와 제한 후 split 수
- FlightPath 파일 수
- waypoint 수
- coverage/촬영 품질 지표
- path 수 감소 전후의 option label/의미 보존 여부

### 7. dedicated/direct는 별도 트랙

next-collab, prior, attack, path-deviation, quality-speed는 일반 3옵션 병렬화와 분리한다. 이쪽은 phase timer, source artifact cache, direct delivery wait 축소가 핵심이다.

공격 재계획도 두 흐름으로 나눈다.

- 신규 표적 `0402`: 공격 특화/공격 배제 2옵션
- `attackClosedDestroyed`, post-attack rejoin: direct 성격

DL risk Level-5는 현재 prior Level-4 dedicated 조건과 같다고 가정하지 않는다. routing/timer를 따로 계측한다.

## 적용 순서

1. SLA 기준과 trigger 분류를 고정한다.
2. end-to-end timing, dedicated phase timer를 추가한다.
3. archived `0902` replay benchmark를 만든다.
4. `id_allocator` correctness를 보정한다.
5. 병렬화 safety gate와 `ReplanReservationPlan`을 만든다.
6. current-remaining hybrid 선적용/skip 구조를 검증 후 반영한다.
7. `0303/0304` waypoint block 분리를 반영한다.
8. option `4` worker 제한을 설정화하고 3 worker 동시 실행을 켠다.
9. option `4` path 폭증을 정확도 gate와 함께 제어한다.
10. dedicated/direct 흐름 최적화를 진행한다.
11. P2 보조 최적화와 ProcessPool 검토를 마지막에 한다.

## 금지할 결론

- 옵션 수를 줄여서 3초를 맞추는 방식
- `max_workers=2`를 바로 `3`으로 바꾸는 방식
- global allocator를 child process/worker에서 직접 호출하는 방식
- delivery grace만 줄이면 전체 문제가 해결된다는 가정
- DEM memoize를 새로 추가해야 한다는 가정
- ProcessPool을 계측 없이 먼저 도입하는 방식

## 성공 기준

- 입력갱신/현재임무 재수행/RTB/강제명령 3옵션 replay에서 p95 3초 이내
- option `(6, 4, 5)` 순서와 label 유지
- 공격 신규 표적 2옵션 흐름 보존
- next/prior/direct single flow의 `0902 -> 0903` 시간 별도 측정
- MissionPlanID, IMP ID, pathID, waypointID uniqueness 유지
- `0301 -> 0305 status=2 -> 0901/0903` 순서 보존
- 실패 시 partial artifact 미송신
