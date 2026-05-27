# 재계획 3초 성능 TODO

## P0 - 3초 조건 증명/충족에 필요한 작업

- [ ] 트리거별 재계획 경로를 성능 관점에서 분류
  - 3옵션 일반 재계획: 입력갱신, 현재임무 재수행, 강제명령, RTB
  - 2옵션 공격 재계획: 신규 표적 `0402`의 공격 특화/공격 배제
  - single dedicated/direct: next-collab, prior Level-4, priorClosedResume, imaging schedule, path deviation, quality-speed
  - DL risk Level-5: 현재 mission planning의 prior dedicated 조건과 별도이므로 실제 routing/타이머를 먼저 확인
  - post-attack rejoin/attackClosedDestroyed: 신규 표적 2옵션 흐름과 direct delivery 흐름을 분리
  - 각 트리거가 `0305` 기준 SLA인지 `0902 -> 0903` 기준 SLA인지 표시

- [ ] 모든 재계획 요청에 end-to-end timing 로그 추가
  - `0902_received_ms`
  - `scheduled_start_ms`
  - `0305_status_1_ms`
  - `pipeline_done_ms`
  - `0301_sent_ms`
  - `0305_status_2_ms`
  - `0901_or_0903_sent_ms`
  - `total_0902_to_0903_ms`

- [ ] dedicated pipeline 단계별 타이머 추가
  - next-collab: `load_source`, `planner`, `build_paths`, `write_artifacts`, `total`
  - prior: `resolve_artifacts`, `prior_insert`, `collab_resume`, `other_uav_resume`, `write_artifacts`, `total`
  - attack: 기존 타이머 유지 + `read_source`, `descriptor_build`, `write_artifacts`, `delivery_wait`

- [ ] archived `0902` payload를 재실행하는 benchmark 명령 추가
  - 각 trigger별 p50/p95/max 출력
  - 3초 초과 시 실패 코드 반환
  - `0305` 기준과 `0902 -> 0903` 기준을 모두 측정

- [ ] 3옵션 전체를 각각 실제 계산하되 variant 3개를 완전 병렬 실행
  - option `4` 포함 시 worker를 2개로 제한하는 로직 제거 또는 runtime 설정화
  - 단순 제한 제거 전 CPU 사용률, IO 대기, nested parallel 경쟁을 replay benchmark로 확인
  - variant 1/2/3이 동시에 시작되는지 로그로 검증
  - variant별 실패/취소 처리와 GUI signal emit은 main thread 안전성을 유지
  - 병렬 완료 순서와 무관하게 option 순서 `(6, 4, 5)`와 missionPlanID 매핑을 고정

- [ ] SLA 기준을 명확히 확정
  - `0305 status=1 -> 0305 status=2`
  - `0902 received -> 0903 sent`
  - `0803/0201 trigger -> active plan applied`

- [ ] 병렬화 안전성 gate 추가
  - `runtime_override`는 thread-local이지만, `export_0303_0304._apply_runtime_params()`처럼 모듈 전역 상수를 수정하는 경로는 병렬 worker에서 금지
  - worker에는 Qt 객체, GUI widget, NodeMessenger 객체를 넘기지 않음
  - worker output은 variant temp root에만 쓰고, 검증 후 publish

- [ ] `id_allocator` 기본 correctness 보정
  - `_load(path=None)`에서 미정의 `target` 대신 실제 대상 path를 사용하도록 수정
  - tracker load 실패가 조용히 `{}`로 떨어지지 않도록 최소 warning/timing 로그 추가
  - 수정 후 기존 ID tracker/usage 파일과 충돌 없이 waypoint 시작 ID 50 규칙이 유지되는지 확인

## P1 - 효과가 큰 최적화

- [ ] 병렬 실행 전 ID block 선예약 구조 추가
  - MissionPlanID 3개 선예약
  - variant별 IMP ID block 선예약
  - aircraft별 pathID block 선예약
  - variant별 waypointID block 선예약
  - worker 내부에서는 전역 allocator를 호출하지 않고 할당받은 local allocator만 사용
  - `ReplanReservationPlan` 형태로 variant별 ID pool을 만든 뒤 worker spec에 포함
  - 실패한 variant의 예약 ID는 재사용하지 않고 skip/commit log만 남김
  - process 병렬화를 쓰는 경우 parent process에서만 전역 allocator를 호출하고 child worker는 local pool만 소비

- [ ] option `4` path 폭증 원인 제어
  - `review_assigned_areas_local()` 이후 `0303` path 수가 27개에서 80개 수준으로 증가하는 구간 확인
  - 같은 aircraft/inputMissionID의 연속 segment를 병합할 수 있는지 검증
  - `enhanced_area_review_max_segment_m`, FOV, sweep separation 변경이 path 수/촬영품질에 주는 영향을 replay로 비교
  - FlightPath 파일 수와 waypoint 수를 함께 SLA counter로 기록

- [ ] current-remaining hybrid 치환을 generic `0303/0304` 생성 전으로 이동
  - 나중에 대체될 current path를 먼저 만들지 않도록 한다.
  - 선적용 후에도 source/current inputMissionID mapping과 completed prefix가 유지되는지 검증

- [ ] 재계획 1회 단위 source artifact 캐시 추가
  - source `MissionPlan`
  - source `InputMissionPlan`
  - source IMP payload
  - source FlightPath payload
  - `pathID -> waypointIDs`

- [ ] waypoint ID block을 builder별로 예약한 뒤 `0303/0304` 병렬 생성 지원
  - 현재 순차 실행 이유가 waypoint ID 충돌 회피이므로 ID block 예약이 선행되어야 한다.
  - `d0303`용 block과 `d0304`용 block을 분리해서 동시에 생성한다.
  - 방법 A: FlightPath를 임시 waypointID `0`으로 만든 뒤 count 계산 후 재할당
  - 방법 B: 생성 전 mission별 waypoint count estimator를 만들고 block을 선예약
  - 두 방법 모두 `nextWaypointID` relink를 최종 단계에서 다시 수행

- [ ] `0303` 내부를 aircraft 단위로 병렬화
  - path continuity는 aircraft 내부에서만 유지하면 된다.
  - UAV 4/5/6 각각을 독립 worker에서 packet 생성 후 최종 waypoint ID/nextWaypointID를 조립한다.
  - formation follower가 leader waypoint list를 복제하는 경로는 aircraft 병렬화 예외로 분리

- [ ] CPU-bound 구간은 process 병렬화 검토
  - pure Python geometry/path 생성이 GIL에 막히면 `ThreadPoolExecutor`만으로는 부족할 수 있다.
  - `ProcessPoolExecutor` 후보: divide/pattern, `0303` aircraft별 path build, option 4 area review
  - worker process에는 JSON-serializable payload만 넘기고 Qt/GUI 객체는 넘기지 않는다.
  - 현재 allocator lock/temp file은 process-safe가 아니므로 child process에서 global ID reservation을 호출하지 않는다.

- [ ] DEM/terrain elevation 조회를 요청 단위로 memoize
  - lat/lon을 일정 precision으로 round한 key 사용

- [ ] next-collab/prior에서 영향받은 aircraft IMP만 쓰는 구조 검토
  - downstream이 unchanged IMP reference를 허용하는지 검증 필요

- [ ] force-direct 흐름의 post-`0301` grace 축소
  - `0901/0701` 옵션 흐름은 기존 grace 유지
  - direct `0903`은 `0301` write/send 확인 직후 flush
  - quality-speed, next-collab, prior, post-attack rejoin, path-deviation direct flow를 각각 확인

## P2 - 보조 개선

- [ ] `push_center` hot message module resolution 캐시
  - 대상: `0301`, `0305`, `0902`, `0903`

- [ ] 같은 process 내 `0902` sidecar disk round-trip 제거 또는 compact JSON 쓰기
  - 디버깅/복구 목적으로 필요한 경우에만 pretty JSON 유지

- [ ] runtime 성능 모드에서 hot-path debug artifact 생성을 줄이기
  - debug mode: full pretty JSON 유지
  - performance mode: compact write / optional artifact
  - `MissionPlan`, `IndividualMissionPlan`, `FlightPath` 공식 산출물의 pretty 여부는 호환성 확인 후 변경

- [ ] 성능 dashboard counter 추가
  - trigger별 p50/p95/max
  - 동기 생성 option 수
  - `0303`, `0304` FlightPath 생성 시간
  - post-`0301` delivery wait 시간

- [ ] `export_0303_0304` runtime module reload/전역 상수 수정 경로 정리
  - in-process thread 병렬화에서는 reload/global mutation을 피한다.
  - 필요한 경우 ProcessPool worker 내부에서만 runtime module reload를 허용한다.

## 회귀 검증 항목

- [ ] 입력갱신: UAV 3대 + LAH 3대 조건에서 첫 유효 plan 3초 이내
- [ ] 현재임무 재수행: 첫 유효 plan 3초 이내
- [ ] 다음 협업기저 재계획: dedicated pipeline 전체 3초 이내
- [ ] 선행 임무 삽입: dedicated pipeline 전체 3초 이내
- [ ] 공격 재계획: direct delivery 포함 `0902 -> 0903` 3초 이내
- [ ] ID 불변성 유지
  - MissionPlanID uniqueness
  - IndividualMissionPlanPackageID uniqueness
  - pathID uniqueness
  - waypointID uniqueness
  - source/current inputMissionID mapping preserved
- [ ] 병렬 실행 후 옵션 의미 유지
  - option 6: 정찰/시간 균형
  - option 4: 정찰 특화
  - option 5: 최소 시간
  - 병렬 완료 순서와 관계없이 0901/0903 payload 순서와 labels 유지
- [ ] 중간 실패 시 partial artifact 미송신
  - variant temp root 산출물만 남고 `0305 status=2` 또는 `0903`이 나가지 않아야 함
