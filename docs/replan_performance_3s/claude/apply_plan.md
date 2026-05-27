# 적용 순서와 PR 분할안 (Claude)

[`REVIEW.md`](REVIEW.md)에서 정리한 우선순위를 PR 단위로 잘게 쪼갠 안. 모든 PR은 단독으로 회귀 위험이 작아야 하고, 다음 PR을 차단하지 않아야 한다는 기준으로 분리했다.

## 의존 그래프

```
SLA 정의(P0-5)
   |
   +--> timing 로그(P0-1) ---+--> phase timer(P0-2)
   |                          |
   |                          +--> 0902 benchmark CLI(P0-3)
   |
hybrid 위치 이동(P1-3) [단독, 안전]
   |
ID block 선예약(P1-1) ---+--> waypoint block 분리(P1-4)
                         |        |
                         |        +--> 0303 aircraft 병렬(P1-5)
                         |
                         +--> variant 3 worker 동시(P0-4)
                                  |
                                  +--> option 4 worker 제한 해제
                                  |
                                  +--> recon path 폭증 제어(P1-2)

source artifact 캐시(P1-7)         [후반]
push_center 캐시(P2-1)              [후반, 저위험]
0902 sidecar 토글(P2-2)             [후반, 저위험]
grace 축소(P2-3)                    [후반, 중위험]
queue deep-copy 제거(P2-4)          [후반, 중위험]
ProcessPoolExecutor(P1-6)           [측정 후, 마지막]
```

## PR 단위

### PR #1 — SLA 기준 합의 (문서/설정만)
- 변경: `docs/replan_performance_3s/`, `replan_runtime_settings.py`에 SLA 키 1개
- 합의: `0305 status=1 → 0305 status=2`를 1차 기준으로 삼고, `0902→0903`은 grace 포함 보조 기준
- 회귀 위험: 0
- 성공 기준: 후속 PR이 어느 기준에 맞춰 측정할지 모호하지 않음

### PR #2 — variant timing 로그
- 변경: [`replan_queue_manager.py`](../../../modules/monitoring/logic/replan_queue_manager.py) `_QueueItem` timing 필드, [`mission_planning_gui.py`](../../../modules/mission_planning/mission_planning_gui.py) variant loop에 started_at/finished_at, [`replan_request_transport_store.py`](../../../modules/common/replan_request_transport_store.py)에 receive ms 추가
- 회귀 위험: 0 (로그만 추가)
- 성공 기준: 1회 재계획에서 0902_received_ms / scheduled_start_ms / variant_*_started / variant_*_finished / 0301_sent_ms / 0305_status_2_ms / 0903_sent_ms 가 한 줄에 모두 잡힘

### PR #3 — pipeline phase timer
- 변경: [`runtime/mission_planning_pipeline_logging.py`](../../../modules/mission_planning/runtime/mission_planning_pipeline_logging.py)에 phase context manager, next-collab/prior/attack 파이프라인에서 phase 라벨 부여
- 회귀 위험: 0 (로그만 추가)
- 성공 기준: next-collab 1회에서 load_source/planner/build_paths/write_artifacts 합이 total과 5% 이내로 일치

### PR #4 — 0902 archived benchmark 명령
- 변경: 신규 `tools/replan_benchmark.py`. [`replan_request_transport_store.py`](../../../modules/common/replan_request_transport_store.py) 기존 load 함수 활용
- 회귀 위험: 0 (별도 진입점)
- 성공 기준: 보관된 0902 1개를 입력으로, p50/p95/max를 trigger별로 출력

### PR #5 — current-remaining hybrid 호출 위치 이동 (P1-3)
- 변경: [`mission_planning_gui.py`](../../../modules/mission_planning/mission_planning_gui.py) divide → hybrid → (필요 시) generic 0303/0304 순서
- 회귀 위험: 중간 (hybrid 분기 누락 시 산출물 누락 가능)
- 사전 점검: `current_remaining_hybrid_replan.py`, `current_remaining_hybrid.py` 입력 가정
- 성공 기준: hybrid가 필요한 variant에서 generic 0303/0304 산출물이 더 이상 만들어지지 않음. variant_total ≤ 기존의 70%를 목표

### PR #6 — ID block 선예약 인프라 (P1-1)
- 변경: [`id_allocator.py`](../../../modules/mission_planning/MissionPlanner/data_def/id_allocator.py)의 `_reserve_range`를 호출 측에서 사용하는 helper 추가, variant executor 진입 전 reserve, builder 시그니처에 `(start, end)` 또는 로컬 allocator 주입
- 회귀 위험: 큼 (ID 충돌/누수)
- 성공 기준: 한 번의 재계획에서 worker 안의 전역 `_next()` 호출 0건. ID 누수 측정 로그 추가
- 단독 적용 가능: ID는 reserve 후 사용하고 결과는 동일. 병렬 worker 수를 늘리지 않은 상태에서도 정상 동작해야 함

### PR #7 — waypoint block 분리 (P1-4)
- 변경: [`export_0303_0304.py`](../../../modules/mission_planning/MissionPlanner/planning_enhanced/io/export_0303_0304.py) 진입 전 d0303용/d0304용 waypoint range 분리 reserve, builder 내부 `_next_waypoint_id()` 경로 차단
- 회귀 위험: 큼 (waypoint 충돌)
- 의존: PR #6
- 성공 기준: d0303 worker와 d0304 worker가 각자 다른 ID 범위에서 진행. ThreadPoolExecutor max_workers=2의 효과가 처음으로 측정에 나타남 (FlightPath build sequential 시간이 절반 가까이 줄어야 함)

### PR #8 — variant 3 worker 동시 실행 + option 4 worker 제한 해제 (P0-4)
- 변경: [`mission_planning_gui.py`](../../../modules/mission_planning/mission_planning_gui.py) `_run_general_planning`의 max_workers 결정 블록을 [`replan_runtime_settings.py`](../../../modules/monitoring/logic/replan_runtime_settings.py) 키로 빼냄
- 회귀 위험: PR #6/#7가 깐 위에서는 중간
- 의존: PR #6, PR #7
- 성공 기준: option 4 포함 케이스에서 variant 3개의 started_at 간격이 100ms 이내. variant_total p95가 기존 8373.8ms에서 대폭 감소 (목표 < 3000ms)

### PR #9 — option 4 path 폭증 제어 (P1-2)
- 변경: [`area_review.py`](../../../modules/mission_planning/MissionPlanner/planning_enhanced/algo/area_review.py) `review_assigned_areas_local()` 출력에 후처리 병합 단계, 또는 split 정책 파라미터 노출
- 회귀 위험: 큼 (정찰 정확도 영향)
- 사전 점검: split 정책의 본질이 정확도 요구인지 단순 정책인지 확인
- 성공 기준: option 4의 0303 path 수가 기존 80개에서 의미 있게 감소. recon mission 정확도 회귀 테스트 통과

### PR #10 — 0303 aircraft 단위 병렬 (P1-5)
- 변경: d0303 `build_flight_plans()`를 aircraft 단위 ThreadPoolExecutor로 분할
- 회귀 위험: 중간 (aircraft 간 의존 가정)
- 의존: PR #7
- 성공 기준: aircraft 4/5/6 worker가 동시 실행됨이 로그로 확인. variant 1개 안의 0303 시간 단축

### PR #11 — source artifact 캐시 (P1-7)
- 변경: 재계획 1회 컨텍스트 dataclass, helper들이 이를 공유
- 회귀 위험: 중간 (stale 데이터)
- 성공 기준: 같은 재계획 안에서 source 파일 IO 횟수가 1회로 수렴

### PR #12 — push_center module resolution 캐시 (P2-1)
- 변경: 0301/0305/0902/0903 push 모듈에 resolved type dict
- 회귀 위험: 0
- 성공 기준: 매 호출당 importlib 횟수 0회 (워밍업 후)

### PR #13 — 0902 sidecar 토글 (P2-2)
- 변경: [`replan_request_transport_store.py`](../../../modules/common/replan_request_transport_store.py)에 debug flag, 기본값은 현재 동작 유지
- 회귀 위험: 0 (default 유지)
- 성공 기준: 성능 모드에서 sidecar 쓰기 IO 0건

### PR #14 — force-direct grace 축소 (P2-3)
- 변경: [`mission_planning_gui.py:3919`](../../../modules/mission_planning/mission_planning_gui.py)의 `_queue_post_0301_delivery()` force_direct 분기
- 회귀 위험: 중간 (수신측 race)
- 사전: 0305 status=2 ack 확인 로직과 함께 묶어서
- 성공 기준: direct 흐름 0902→0903 시간이 기존 대비 대략 1초 단축

### PR #15 — monitoring queue deep-copy 제거 + UI throttle (P2-4)
- 변경: [`replan_queue_manager.py`](../../../modules/monitoring/logic/replan_queue_manager.py), monitoring_gui UI refresh
- 회귀 위험: 중간 (UI 동시성)
- 성공 기준: burst 5회 재계획에서 UI 갱신 누락 없음, deep-copy 호출 0건

### PR #16 — ProcessPoolExecutor (P1-6)
- 변경: divide_and_pattern, d0303 aircraft worker, area_review를 process pool로
- 회귀 위험: 큼 (직렬화/Qt 경계)
- 사전 조건: PR #2/#3 결과로 GIL 병목이 측정으로 확인된 경우에만
- 성공 기준: variant_total p95가 thread pool 대비 추가 단축

## PR 묶음 진행 시 합의해야 할 것

1. SLA 기준 (PR #1) — 의사결정 1회로 끝내기
2. ID 누수 허용량 — reserve 후 미사용 ID를 어디까지 허용할지
3. recon 옵션 정확도 회귀 테스트 — PR #9 진입 조건
4. force-direct grace의 race window 정책 — PR #14 진입 조건
5. 성능 모드 토글의 기본값 — PR #13 default 결정

이 5가지를 PR #1 단계에서 함께 고정해두면, 후속 PR이 매번 정책 의사결정을 새로 만들지 않아도 된다.
