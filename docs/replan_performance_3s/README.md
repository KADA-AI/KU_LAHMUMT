# 재계획 3초 성능 분석

## 범위

검토 대상 조건은 다음과 같다.

- 기본 재계획, 현재임무 재수행, 다음 재계획, 공격 재계획, 선행 임무 재계획이 모두 3초를 넘으면 안 됨
- 기능 정확도보다는 HW 사용 구조, 동기 처리, 파일 IO, 경로 생성 비용 관점에서 확인

이번 작업에서는 실행 로직을 바꾸지 않고, 보관 로그 반복 분석과 코드 경로 리뷰, 서브 에이전트 분석 결과를 합쳐 병목과 개선 TODO를 정리했다.

2026-04-28 추가 리뷰에서는 단순 로그 해석이 아니라 실제 코드 전체 흐름을 다시 따라가며, 아래 개선 방향이 현재 구조에서 실제로 3초 문제를 해결할 수 있는지와 선행 조건을 문서에 반영했다.

## 측정 기준

분석한 로그:

- `Logs/**/mission_planning.log`
- 파일 수: 30개
- `0305 status=2 ... elapsed=`: 204건
- `[TIME] variant_total`: 60건
- `[TIME] divide_and_pattern`: 60건
- `FlightPath build time (sequential)`: 60건
- `NEXTCOLLAB ... stored`: 33건
- `[ATTACK][TIME] override_total`: 28건

주의할 점:

- 현재 화면/로그의 계획 시간은 임무계획이 `0305 status=1`을 보낸 뒤부터 `0305 status=2`를 보낼 때까지로 보인다.
- 이 값에는 보통 100 ms인 `0902` 수신 후 파이프라인 시작 지연은 빠져 있다.
- 외부 요구가 `0902 수신 -> 0903 적용 요청` 기준이라면, 계획 완료 후 `0301` 뒤 delivery grace가 별도로 1.8~3.0초를 추가할 수 있다.

## 요약 결과

| 항목 | 건수 | 3초 초과 | 최대 | p50 | p95 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `0305 status=2 elapsed` | 204 | 18 | 12650 ms | 180 ms | 7020 ms |
| `variant_total` | 60 | 22 | 9022.9 ms | 2395.8 ms | 8373.8 ms |
| `divide_and_pattern` | 60 | 0 | 2269.5 ms | 561.3 ms | 2139.7 ms |
| `FlightPath build sequential` | 60 | 18 | 6521.7 ms | 1189.5 ms | 5670.1 ms |
| `NEXTCOLLAB stored` | 33 | 0 | 85.4 ms | 19.1 ms | 64.6 ms |
| `ATTACK override_total` | 28 | 0 | 2399.0 ms | 1775.0 ms | 2004.0 ms |

3초 초과가 몰린 사유:

- `협업기저임무 재입력에 대한 재계획`: 7/7건 초과, 최대 12.65초
- `협업기저임무 재수행 요청`: 3/3건 초과, 최대 9.91초
- `_협업 기저 전환으로 인한 재계획`: 2/3건 초과, 최대 8.96초
- `협업 기저 전환으로 인한 재계획`: 5/30건 초과, 최대 4.16초
- 단일 초기임무계획 계열은 분석 로그 기준 최대 2.82초로 3초 이내였다.

## 핵심 판단

현재 초과 원인은 모니터링 모듈 과부하가 1차 원인으로 보이지 않는다. 더 큰 원인은 임무계획 쪽의 작업 구조다.

1. 일부 runtime 재계획이 옵션 3개를 동기적으로 모두 생성한다.
2. 각 옵션마다 divide/pattern과 FlightPath 생성을 다시 수행한다.
3. `0303`과 `0304` FlightPath 생성은 waypoint ID 안전성 때문에 순차 실행된다.
4. 현재임무 remaining/hybrid 치환이 generic FlightPath 생성 뒤에 적용되는 경로가 있어, 나중에 대체될 경로도 먼저 만든다.

옵션 3개가 모두 만들어져야 완료라는 조건이 고정이면, 해결 방향은 옵션 개수를 줄이는 것이 아니다. 또한 동일 산출물을 복사해서 다른 옵션으로 쓰는 방식은 옵션 품질 요구와 맞지 않는다. 목표는 `3개 옵션을 각각 실제 계산`하되, ID 충돌 없이 완전 병렬화하고 option 4 path 폭증을 줄여 3초 안에 끝내는 것이다.

## 2026-04-28 전체 코드 재검토 결과

문서의 큰 방향은 코드 기준으로도 유효하다. 다만 `max_workers`만 늘리거나 post-`0301` 대기만 줄이는 식의 단일 변경으로는 3초 조건을 안정적으로 만족하기 어렵다. 실제 해결은 다음 전제가 함께 맞아야 한다.

| 개선 방향 | 코드상 확인 사항 | 판단 |
| --- | --- | --- |
| 3개 variant 완전 병렬화 | `mission_planning_gui.py`는 일반 재계획에서 `PlanVariant` `ThreadPoolExecutor`를 이미 사용하지만, option `4`가 포함되면 worker를 2로 낮춘다. | 방향은 맞다. 단, worker 제한 제거는 ID 선예약과 CPU/IO 경쟁 측정 후 적용해야 한다. |
| ID block 선예약 | `MissionPlanID`/IMP/pathID는 store 단계에서 일부 batch 예약하고, waypointID는 `d0303`/`d0304` 생성 말미에 block 예약한다. `reserve_waypoint_block()`은 호출 때마다 FlightPath scan/usage write를 수행한다. | 병렬화의 선택사항이 아니라 선행조건이다. variant/aircraft/0303/0304별 local allocator를 먼저 설계해야 한다. |
| `0303`/`0304` 병렬화 | 일반 재계획에서는 같은 `wp_alloc`을 공유하므로 `0303 -> 0304` 순차로 실행한다. `d0303._WPAllocator(start=...)`, `d0304._WPAllocator(start=...)`는 local start를 받을 수 있다. | 가능하다. 다만 생성 전 waypoint 수를 예측하거나, 임시 waypoint 후 재할당하는 2-phase 구조가 필요하다. |
| current-remaining/hybrid 선적용 | current-remaining hybrid는 generic FlightPath 생성 뒤 `merge` 방식으로 대체한다. | 효과가 있다. 대체될 path를 먼저 만들지 않도록 0302/FlightPath 입력을 선별하는 쪽이 맞다. |
| option `4` path 폭증 제어 | `planning_enhanced/pipeline.py`에서 option `4`는 `review_assigned_areas_local()`을 거치고, split count는 `max_segment_m`에 따라 증가한다. | 성능 병목의 실제 원인 중 하나다. 정확도 유지 조건에서 segment 병합/상한/후처리 기준을 별도로 검증해야 한다. |
| dedicated pipeline 최적화 | next-collab은 로그상 store가 빠르지만 전체 phase timer가 부족하다. prior/attack은 source plan/IMP/FlightPath 로드와 clone/write가 여러 helper에 분산되어 있다. | 일반 3옵션 병렬화와 별개로 phase timer와 source cache가 필요하다. |
| force-direct delivery 축소 | `_queue_post_0301_delivery()`는 direct 여부와 무관하게 기본 grace를 둔다. | `0902 -> 0903` SLA에는 중요하다. 단, `0901/0701` 옵션 선택 흐름과 direct `0903` 흐름을 분리해서 줄여야 한다. |

현재 병렬화는 `run_divide_and_pattern`, `0301` 임시 생성, `0303/0304` FlightPath build까지의 variant core에 한정된다. 모든 future가 끝난 뒤 `_store_general_variant()`가 idx 순서대로 `0302`, FlightPath, `0301` 저장과 최종 planID/IMP/pathID 매핑을 직렬 처리한다. 따라서 "이미 3개 옵션이 완전 병렬"이라고 보면 안 되고, "core 일부 병렬 + 저장/ID 매핑 직렬 + variant 내부 FlightPath 순차"로 보는 것이 정확하다.

주의할 점도 있다. runtime override 자체는 thread-local이라 variant별 runtime payload를 동시에 쓰는 방향은 가능하다. 하지만 `MissionPlanner/planning_enhanced/io/export_0303_0304.py`의 `_import_runtime_modules()`는 `d0303`/`d0304`를 reload하고, `_apply_runtime_params()`는 모듈 전역 상수를 수정한다. 이 경로를 in-process thread 병렬화에 섞으면 option별 runtime 값이 서로 침범할 수 있으므로, pure config 객체로 넘기거나 `ProcessPoolExecutor`로 격리하는 방식이 필요하다.

ID allocator도 병렬화 전에 보정해야 한다. `MissionPlanner/data_def/id_allocator.py`의 `_load(path=None)`는 `target` 미정의 예외를 삼키고 `{}`를 반환할 수 있어 tracker direct load가 사실상 무력화될 수 있다. 또한 `reserve_waypoint_block()`은 lock 안에서 usage 파일과 `FlightPath/*.json` scan을 수행하므로, worker마다 전역 allocator를 호출하면 병렬화 효과를 상쇄한다. `ProcessPoolExecutor`를 쓰는 경우 현재 `_LOCK`은 process-safe가 아니므로 ID는 parent process에서 한 번에 예약하고 worker에는 local pool만 넘겨야 한다.

따라서 문서의 해결 방향은 다음처럼 보정한다.

1. 계측과 replay benchmark로 어떤 SLA를 실패하는지 먼저 고정한다.
2. worker 제한 제거는 단독 작업이 아니라 ID 선예약, runtime 전역 상태 격리, output atomic publish와 묶어서 적용한다.
3. `0303/0304` 병렬화는 waypointID block을 분리한 뒤 진행한다.
4. option `4`는 제거하지 않고, local area review 이후 path 수와 waypoint 수를 제어한다.
5. dedicated/direct 흐름은 일반 3옵션 병렬화와 별도 트랙으로 phase timer, source cache, delivery wait 축소를 적용한다.

Delivery 흐름도 SLA 해석에 중요하다. 일반적으로 `0305 status=1`은 재계획 파이프라인 시작 직후 나가고, 산출물 저장 완료 후 `_schedule_plan_delivery()`가 `0301` 전송을 큐잉한다. `0301` 전송 성공 후 `0305 status=2`가 나가고, 이후 실행 모드/force-direct 여부에 따라 `0901` 또는 `0903`이 나간다. 따라서 `0305 status=1 -> 2`가 3초 이내여도, `0902 -> 0903` 기준에서는 post-`0301` grace가 별도 병목이 될 수 있다.

## 재계획 트리거 분류

코드 기준으로 3초 초과 대응은 트리거별로 나눠야 한다.

| 트리거 | 옵션 구조 | 주요 대응 |
| --- | --- | --- |
| 입력갱신 `input_refresh_replan.py` | 기본 `(6, 4, 5)` 3옵션 | variant 완전 병렬화, option 4 path 제어, `0303/0304` 병렬화 |
| 현재임무 재수행 `collab_reexecute.py` | 기본 `(6, 4, 5)` 3옵션 | 입력갱신과 동일 |
| 강제명령/RTB `forced_command_replan.py`, `rtb_replan.py` | 기본 `(6, 4, 5)` 3옵션 | 일반 재계획 병렬화 경로와 동일 |
| 표적탐지 `target_detection_replan.py` | 기본 공격 특화/공격 배제 2옵션 | attack customization과 direct delivery wait 분리 측정 |
| 다음 협업기저 `next_collab_replan.py` | single dedicated/direct | phase timer, source cache, artifact write 최적화 |
| 선행임무 `prior_mission_replan.py` | Level-4 single dedicated/direct | source FlightPath lookup cache, DEM memoization, post-rejoin 비용 계측 |
| DL 위험도 `prior_mission_replan.py` | Level-5 single option, 현재 prior dedicated 분기와 별도 확인 필요 | 실제 routing과 phase timer를 먼저 분리 계측 |
| 선행임무 종료/복귀 | `missionPlanIDList` direct | post-rejoin delivery wait와 source artifact 재사용 확인 |
| 촬영품질/경로이탈/품질속도 | single dedicated/direct 또는 `missionPlanIDList` | 일반 3옵션보다 delivery wait와 write path가 중요 |

## 기본 / 현재임무 재수행 / 입력갱신 재계획

관련 경로:

- `modules/monitoring/logic/input_refresh_replan.py`
- `modules/monitoring/logic/collab_reexecute.py`
- `modules/common/option_codes.py`
- `modules/mission_planning/mission_planning_gui.py`

현재 구조:

- `DEFAULT_OPTION_CODE_SEQUENCE = (6, 4, 5)`
- 입력갱신과 현재임무 재수행이 옵션 3개를 모두 `pendingOptionList`에 넣는다.
- 임무계획은 수신한 옵션 수만큼 variant를 실제 파일 산출물까지 모두 생성한다.
- 각 variant는 `run_divide_and_pattern`, `0301` 생성, IMP 로드/수집, `0303/0304` FlightPath 생성, `0302` 생성/쓰기, FlightPath 쓰기, `0301` 쓰기를 반복한다.

관측 예:

- 입력갱신 `0305 elapsed`: 9.30~12.65초
- 현재임무 재수행 `0305 elapsed`: 8.47~9.91초
- 최대 `variant_total`: 9.02초
- 최대 순차 FlightPath 생성: `0303=4028.4 ms`, `0304=2493.3 ms`, 합계 6.52초

가장 큰 비용은 옵션 3개 전체 생성이고, 그 안에서 `0303/0304` 순차 FlightPath 생성이 p95 기준 5.67초까지 올라간다.

중요한 추가 관찰:

- 일반 3옵션 재계획은 이미 variant 단위 `ThreadPoolExecutor`를 사용한다.
- 즉 완전히 `1번 완료 -> 2번 시작 -> 3번 시작` 구조는 아니다.
- 다만 option 4, 즉 정찰특화 옵션이 포함되면 worker가 2개로 제한되어 3개 variant가 동시에 돌지 못한다.
- 각 variant 내부에서는 `0303` 생성 후 `0304` 생성이 순차로 실행된다.
- option 4는 `review_assigned_areas_local()` 이후 `0303` FlightPath 수가 크게 늘어난다.
- 2026-04-24T101957 최악 구간에서 option 4는 `0303=80`, `0304=27`, 총 FlightPath 107개를 만들었다.
- 같은 구간에서 option 6/5는 각각 FlightPath 54개 수준이다.

따라서 3옵션 필수 조건에서의 우선순위는 다음과 같다.

1. 3개 variant worker를 모두 동시에 돌린다. option 4가 있어도 worker를 2로 제한하지 않는다.
2. variant별 MissionPlanID, IMP ID, pathID, waypointID block을 실행 전에 선예약해서 병렬 생성 중 ID 충돌을 원천 차단한다.
3. `0303`과 `0304`를 같은 waypoint allocator에 묶어 순차 실행하지 말고, 미리 waypoint ID block을 나눠 병렬 생성한다.
4. `0303` 내부도 aircraft 단위로 나눠 병렬 생성한다. path continuity는 aircraft 내부에서만 필요하므로 aircraft 간 병렬화 여지가 있다.
5. option 4의 local area review 이후 path 수 폭증을 줄이거나 후처리에서 병합한다.
6. `ThreadPoolExecutor`로 충분하지 않으면 CPU-bound geometry/path 생성부를 `ProcessPoolExecutor`로 옮긴다. 단, Qt 객체와 GUI signal은 worker process로 넘기지 않는다.

## 다음 협업기저 재계획

관련 경로:

- `modules/monitoring/logic/next_collab_replan.py`
- `modules/mission_planning/pipelines/next_collab_replan_pipeline_impl.py`
- `modules/mission_planning/runtime/next_collab_line_runner.py`
- `modules/mission_planning/pipelines/next_collab_path_builder.py`

로그상 `NEXTCOLLAB stored`는 최대 85.4 ms로 매우 작다. 다만 이 값만으로 전체 파이프라인이 항상 3초 이내라고 확정하기는 어렵다. source load, geometry planner, path builder, artifact write가 단계별로 충분히 분리 측정되지 않는다.

커질 수 있는 후보는 source `MissionPlan/InputMissionPlan/MissionReferenceInfo/IMP` 순차 로드, full plan/IMP deep-copy, 전체 aircraft IMP 재작성, DEM/terrain sampling 반복이다.

## 선행 임무 재계획

관련 경로:

- `modules/monitoring/logic/prior_mission_replan.py`
- `modules/mission_planning/pipelines/prior_mission_pipeline_impl.py`

리스크:

- dedicated phase timer가 부족해 3초 SLA를 증명하기 어렵다.
- 선행 임무 후 협업 resume/rejoin이 필요하면 next-collab 비용을 그대로 물려받을 수 있다.
- source `MissionPlan/IMP/FlightPath`를 helper 경로에서 반복 로드하는 부분이 있다.
- mission별 waypoint ID를 매번 읽는 대신 plan 단위 `pathID -> waypointIDs` 캐시가 필요하다.

## 공격 재계획

관련 경로:

- `modules/monitoring/logic/target_detection_replan.py`
- `modules/mission_planning/pipelines/attack_plan_pipeline.py`
- `modules/mission_planning/mission_planning_gui.py`

공격 override 자체는 분석 로그에서 3초 이내였다.

- 최대 `override_total`: 2399 ms
- 중앙값 `override_total`: 1775 ms

다만 공격 재계획도 한 가지 흐름으로 묶으면 안 된다. 신규 표적 `0402` 기반 재계획은 기본적으로 공격 특화/공격 배제 2옵션을 생성하는 option delivery 흐름이고, `attackClosedDestroyed`나 post-attack rejoin 계열은 direct 적용 성격이 강하다. 따라서 공격 성능 개선은 `attack_plan_pipeline` override 시간, 2옵션 산출 시간, direct delivery wait를 나눠서 봐야 한다.

다만 SLA가 `0902 -> 0903` 기준이면 별도 병목이 있다. `0301` 뒤 `_queue_post_0301_delivery()`가 `grace_ms = 1800 + 250 ms * 추가 plan 수`, fallback `grace_ms + 1200`을 둔다. direct apply 흐름에서는 이 grace만으로도 3초 예산을 대부분 사용할 수 있다.

## 모니터링 / 큐 오버헤드

관련 경로:

- `modules/monitoring/monitoring_gui.py`
- `modules/monitoring/logic/replan_queue_manager.py`
- `modules/common/push/message0902_push.py`
- `modules/common/replan_request_transport_store.py`

판단:

- 8~12초 초과의 주범은 모니터링 큐로 보이지 않는다.
- 다만 burst 상황에서는 payload deep-copy, snapshot 재생성, UI refresh, `0902` sidecar disk IO가 누적될 수 있다.

## 결론

3초 조건을 맞추려면 먼저 구조를 바꿔야 한다.

- 옵션 3개 산출은 유지한다.
- 세 옵션은 각각 실제 계산한다.
- variant 1/2/3을 ID block 선예약 후 동시에 실행한다.
- option 4는 path 수 폭증을 제어하고, aircraft 단위 병렬 FlightPath 생성을 적용한다.
- `0303/0304`는 waypoint ID block 예약 후 병렬 생성한다.
- 그 다음 source artifact 캐시, DEM sampling cache, direct delivery grace 축소를 진행한다.
