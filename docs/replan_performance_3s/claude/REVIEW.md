# 재계획 3초 SLA 검토 의견 (Claude)

대상: `docs/replan_performance_3s/README.md`, `TODO.md`
작성: 2026-04-28
방법: Claude + Explore sub-agent 3개로 P0 / P1 / P2-보조 항목을 분담해 코드 경로 검증

이 문서는 사용자가 작성한 README/TODO에서 잡아낸 방향이 코드 기준으로 정합한지, 적용 우선순위와 의존관계를 어떻게 잡아야 안전한지를 정리한 의견서다. 상세 근거는 [`findings.md`](findings.md), 적용 순서는 [`apply_plan.md`](apply_plan.md)에 따로 정리했다.

## 한 줄 요약

방향은 맞다. 다만 P1의 **ID 선예약**과 **current-remaining hybrid 위치 이동**이 P0의 "3 variant 완전 병렬"보다 먼저 들어가지 않으면, P0를 먼저 적용해도 lock 경쟁으로 인해 사실상 직렬화되거나 중복 산출물이 그대로 남는다. 적용 순서를 README가 시사하는 P0→P1 순으로 가지 말고 **선행 P1 → P0 → 잔여 P1 → P2** 순으로 가는 것이 안전하다.

## 사용자 가설에 대한 코드 기반 검증

| README/TODO의 가설 | 코드 검증 결과 | 비고 |
| --- | --- | --- |
| 일반 3옵션은 이미 variant ThreadPoolExecutor 사용 | **일치** | [`mission_planning_gui.py:7950-7989`](../../../modules/mission_planning/mission_planning_gui.py)의 `_run_general_planning()`에 variant executor 존재 |
| option 4 포함 시 worker가 2로 제한됨 | **일치** | 같은 함수에서 `is_recon_specialized_option()`로 감지 후 `max_workers = 2`를 하드코딩으로 강제 |
| `0303 -> 0304`가 순차 실행 | **부분 일치** | [`export_0303_0304.py:294`](../../../modules/mission_planning/MissionPlanner/planning_enhanced/io/export_0303_0304.py)는 ThreadPoolExecutor(max_workers=2)로 같이 띄우지만, 두 builder가 모두 전역 waypoint allocator의 lock을 잡으므로 실제 효과는 직렬화에 가까움 |
| current-remaining hybrid 치환이 generic 생성 후에 들어감 | **일치** | divide(line 7569) → 0303/0304 → hybrid 치환(line 7683) 순서. 나중에 대체될 generic path를 먼저 만든다 |
| ID allocator는 전역 lock | **일치** | [`id_allocator.py:11, 409`](../../../modules/mission_planning/MissionPlanner/data_def/id_allocator.py)의 threading.Lock + `_next()` 구조. 다만 `_reserve_range()` (line 456-510)이 이미 있어서 P1의 선예약 구조는 신규 인터페이스 추가가 아니라 **호출자 전환** 작업이다 |
| DEM/terrain memoize | **이미 적용됨** | [`mission_helpers.py:131-168`](../../../modules/mission_planning/MissionPlanner/planning_enhanced/algo/mission_helpers.py)에 `_terrain_elev_cached(...)` lru_cache(65536) 존재. README가 적은 "memoize 추가"는 이미 들어가 있고, 효과 측정만 남았다 |
| post-0301 grace 1.8~3.0초 | **일치** | [`mission_planning_gui.py:3919`](../../../modules/mission_planning/mission_planning_gui.py)의 `_queue_post_0301_delivery()`에서 `1800 + 250 * (plan_count - 1)`, fallback `+1200` |
| `_cs()` 매 호출마다 importlib | **일치** | [`message0902_push.py:32-37`](../../../modules/common/push/message0902_push.py) 외 0301/0305 push에서도 동일 패턴. resolved 타입 캐시가 없다 |

## 사용자 가설을 보강해야 할 지점

1. **0303/0304 ThreadPoolExecutor가 실제로는 효과를 못 내고 있음.**
   README는 "각 variant 내부에서는 0303 후 0304 순차 실행"이라고 적었는데, 코드상으로는 max_workers=2로 *동시 submit*은 한다. 다만 두 builder가 같은 전역 waypoint counter의 lock을 잡으므로 결과적으로 직렬에 가깝다. 즉 P1의 "waypoint ID block을 builder별 분리"가 들어가야 이 ThreadPoolExecutor가 비로소 의미를 갖는다. 단순히 max_workers를 늘리는 것만으로는 여기서는 효과가 없다.

2. **option 4 path 폭증 27→80은 `area_review.py:598`의 `review_assigned_areas_local()`이 1차 후보.**
   sub-agent가 이 함수의 정의 위치는 잡았지만 split/merge 내부 로직까지는 들어가지 않았다. 적용 전에 이 함수의 출력 segment 수가 무엇으로 결정되는지 따로 한 번 더 들여다볼 필요가 있다 (recon 옵션의 본질적 요구 vs 단순 split 정책). 본질이면 후처리 병합으로, 정책이면 옵션 파라미터로 풀 수 있다.

3. **end-to-end timing 로그의 scope가 양면적이다.**
   현재 `0305 elapsed`는 `replan_queue_manager._QueueItem`에서 status=1/2 전후로 잡히지만, 0902 수신과 0903 전송 시점은 기록되지 않는다. SLA 기준이 `0305` 인지 `0902→0903` 인지를 먼저 확정하지 않으면, 같은 변경을 두 번 건드릴 수 있다. P0 1번(timing 로그) 작업 전에 P0 5번(SLA 기준 확정)이 먼저 끝나야 한다.

4. **option 4 worker=2 제한 해제는 단독으로 적용하면 회귀 위험.**
   3 variant가 동시에 전역 ID allocator의 lock을 두드리면, 단순히 "병렬"인 척하면서 직렬보다 lock contention이 더 클 수 있다. 반드시 P1 ID 선예약과 짝으로 묶어야 한다.

## 권장 적용 우선순위 (README의 P0→P1→P2 순서를 다시 짠 안)

1. **P0-5 SLA 기준 확정** (문서/설정만 변경)
2. **P0-1 end-to-end timing 로그** + **P0-2 dedicated phase timer**
3. **P1-3 current-remaining hybrid 치환을 generic 생성 전으로 이동** (단독 안전, 즉시 효과)
4. **P1-1 ID block 선예약** + **P1-4 waypoint block 분리** (병렬화 진입 전 필수)
5. **P0-4 variant 3 worker 동시 실행** + option 4 worker=2 제한 해제 (3, 4를 깐 위에서)
6. **P0-3 archived 0902 benchmark 명령** (1~5번의 검증용 회귀 도구)
7. **P1-5 0303 aircraft 단위 병렬화** (5번이 안정화된 이후)
8. **P2 캐시/grace/push** (마무리 단계, 회귀 위험이 가장 작음)
9. **P1-6 ProcessPoolExecutor** (마지막 수단, GIL 병목이 측정으로 확인되면)

상세는 [`apply_plan.md`](apply_plan.md)에 PR 단위로 분리해 적었다.

## 추가로 확인이 필요한 것 (이번 검토에서 결론을 내지 못한 항목)

- `area_review.review_assigned_areas_local()` 내부의 split 정책: option 4 path 폭증의 본질
- `next_collab_replan_pipeline_impl`의 단계별 비용 분포: phase timer가 들어가야 알 수 있음
- prior 재계획의 실제 worst-case 측정값: 현재 dedicated phase timer가 없어 README도 "리스크" 수준에서만 언급됨
- direct apply 흐름에서 grace 축소 시 수신측의 0305 race window 폭

이 4개는 P0-1, P0-2 timing 로그가 깔리고 나면 자연스럽게 데이터가 나오므로, 그 시점에 다시 점검하면 된다.
