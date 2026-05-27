# 항목별 코드 검증 결과 (Claude)

[`REVIEW.md`](REVIEW.md)의 근거 모음. 사용자가 작성한 [`../TODO.md`](../TODO.md) 항목 순서를 따라가되, 코드 위치와 변경 시 영향 범위를 함께 적었다.

---

## P0-1. end-to-end timing 로그 추가

**현재 상태**

- `0305 elapsed`는 [`replan_queue_manager.py`](../../../modules/monitoring/logic/replan_queue_manager.py) 내부 `_QueueItem`에서 잡히고 있다 (status=1 시점부터 status=2 수신까지). 그러나 다음은 빠져 있다.
  - `0902_received_ms`: [`message0902_push.py:32-37`](../../../modules/common/push/message0902_push.py)에서 sidecar로 `save_payload`만 부르고 timestamp는 외부에 emit 하지 않는다.
  - `scheduled_start_ms`, `pipeline_done_ms`: variant executor loop에서 worker 수만 찍고 (`workers=N`), 개별 시작/종료 ms는 미기록.
  - `0903_sent_ms`: 송신 후 별도 마킹 없음.
- variant 단위 시간은 `[TIME] variant_total`로만 집계됨 ([`mission_planning_gui.py`](../../../modules/mission_planning/mission_planning_gui.py) 내).

**변경 위치**

- [`modules/monitoring/logic/replan_queue_manager.py`](../../../modules/monitoring/logic/replan_queue_manager.py) `_QueueItem` (line 288-315)에 timing 필드 추가
- [`modules/common/replan_request_transport_store.py`](../../../modules/common/replan_request_transport_store.py) `save_payload` (line 57-80) — 수신 ms를 payload에 함께 기록 (이미 timestamp는 있으므로 receive_perf_counter_ms 추가 정도)
- [`modules/mission_planning/mission_planning_gui.py`](../../../modules/mission_planning/mission_planning_gui.py) variant loop — variant id, started_at, finished_at, options 라벨 묶어 한 줄 로그

**위험**

- worker thread의 `time.time()` vs main thread `time.perf_counter()` 단위 혼재 주의. perf_counter는 단일 process 내 monotonic이므로 OK, 다만 시작점이 임의이므로 절대 시각이 필요한 항목 (예: 0902 수신 시각)은 `time.time()`을 함께 기록해야 0902 sidecar 시각과 매칭된다.
- 로그 포맷이 변하면 기존 분석 스크립트가 깨질 수 있음 — 새 키를 추가하는 방향으로 가고, 기존 `0305 status=2 elapsed=` 라인은 그대로 둘 것.

**선행 의존**: 없음 (P1 작업 전에 단독 적용 가능)

---

## P0-2. dedicated pipeline 단계별 타이머

**현재 상태**

- next-collab: [`next_collab_replan_pipeline_impl.py`](../../../modules/mission_planning/pipelines/next_collab_replan_pipeline_impl.py)에 `NEXTCOLLAB stored` 한 라인만 존재 (p95 64.6ms). 이 값은 마지막 store 시점만 잡으므로 source load / planner / build_paths / write_artifacts가 합쳐진 시간을 분해할 수 없다.
- prior: [`prior_mission_pipeline_impl.py`](../../../modules/mission_planning/pipelines/prior_mission_pipeline_impl.py)는 dedicated phase timer가 없다.
- attack: [`attack_plan_pipeline.py`](../../../modules/mission_planning/pipelines/attack_plan_pipeline.py)는 `[ATTACK][TIME] override_total`은 있으나 read_source / descriptor_build / delivery_wait 분리 없음.

**변경 위치**

세 파이프라인 함수에 컨텍스트 매니저 한 개를 만들어 일관된 키 (`phase=...`, `elapsed_ms=...`)로 출력하는 helper를 [`runtime/mission_planning_pipeline_logging.py`](../../../modules/mission_planning/runtime/mission_planning_pipeline_logging.py)에 추가하는 게 깔끔하다 (이미 `mission_planning_pipeline_logging.py`가 존재).

**위험**

- 단계 경계가 모호하면 phase 시간 합이 total보다 작아 보일 수 있음 → "측정 안 한 구간"이 어디인지 명시.
- DEM IO처럼 lazy 호출되는 부분은 phase 어디에 들어가는지 일관 기준이 필요.

**선행 의존**: 없음. 다만 P1 캐시 적용 후에 baseline을 다시 잡는 게 좋음.

---

## P0-3. archived 0902 payload benchmark 명령

**현재 상태**

- payload 저장은 이미 됨: [`replan_request_transport_store.py:57-80`](../../../modules/common/replan_request_transport_store.py)의 `save_payload`, [`load_payload`/`load_latest_payload`](../../../modules/common/replan_request_transport_store.py) (line 124-150).
- 보관 위치: `db_paths.get_db_subpath("DSS_Internal", "replan_request_transport")` 아래 `replan_request_<timestamp_ms>.json`.
- benchmark CLI/명령은 없음.

**변경 위치**

- 신규 진입점 (예: `tools/replan_benchmark.py`)을 만들고, 기존 load 함수를 호출 → mission_planning 파이프라인을 직접 invoke. 단 GUI 주입이 필요한 호출은 mock receiver로 대체.

**위험**

- 0902 단독 재실행은 isolated. 후속 0801/MissionAssignment 흐름이 빠지므로, 산출물의 정합성이 아니라 *시간 측정* 용도로만 써야 한다.
- 빈번한 재실행이 같은 ID space를 소모하면 allocator 로그가 부풀음 → benchmark 모드에서는 별도 ID prefix 또는 dry-run 모드.

**선행 의존**: P0-1 timing 로그가 먼저 들어가야 결과가 의미 있음.

---

## P0-4. variant 3개 완전 병렬 + option 4 worker 제한 해제

**현재 상태**

- [`mission_planning_gui.py:7950-7989`](../../../modules/mission_planning/mission_planning_gui.py) `_run_general_planning`:
  ```
  if max_workers > 2:
      has_recon_specialized_variant = any(is_recon_specialized_option(...))
      if has_recon_specialized_variant:
          max_workers = 2
  ```
- option 4 (정찰특화)가 하나라도 있으면 worker가 2로 강제. 결과적으로 variant 3개가 동시에 시작되지 못한다.
- [`replan_runtime_settings.py`](../../../modules/monitoring/logic/replan_runtime_settings.py)에 worker 제어 설정 키 없음.

**변경 위치**

- 위 if 블록을 runtime 설정으로 빼냄 (`replan_variant_workers`, `allow_recon_worker_limit`).
- [`recon_specialized_pipeline.py`](../../../modules/mission_planning/pipelines/recon_specialized_pipeline.py) 자체는 worker 결정에 관여하지 않으므로 호출 측에서만 손대면 됨.

**위험 — 단독 적용 금지**

- 3 variant가 동시에 [`id_allocator.py:11, 409`](../../../modules/mission_planning/MissionPlanner/data_def/id_allocator.py)의 전역 lock을 두드리면 lock contention이 worker 2일 때보다 더 커질 수 있다.
- 반드시 P1-1 ID block 선예약과 짝으로 적용.

**선행 의존**: P1-1, P1-4 (waypoint block 분리)

---

## P0-5. SLA 기준 확정

**현재 상태**

- README가 측정한 3가지 후보 (`0305 status=2 elapsed`, `variant_total`, `0902→0903`)가 코드상에서도 다른 시점에 측정된다.
- 정책 결정이 빠져 있어, 후속 PR이 어느 기준을 기준선으로 삼을지 매번 다시 정해야 한다.

**변경 위치**: 문서/설정만. 코드는 P0-1 timing 로그 정의에서 자연스럽게 반영.

**선행 의존**: 없음. 가장 먼저 끝내는 것이 좋다.

---

## P1-1. ID block 선예약

**현재 상태**

- [`id_allocator.py`](../../../modules/mission_planning/MissionPlanner/data_def/id_allocator.py)는 전역 `threading.Lock`(line 11) + `_next()`(line 409) 구조. waypoint는 volatile counter(line 27)로 디스크 sync를 피함.
- `_reserve_range(count)` (line 456-510)는 **이미 존재**한다. 즉 인터페이스는 있고, 호출자가 worker 안에서 `_next()`를 부르는 패턴을 worker 진입 전 reserve로 바꾸면 된다.
- d0303 내부에는 [`d0303.py:5333`](../../../modules/mission_planning/MissionPlanner/planning_enhanced/io/d0303.py)의 `_WPAllocator` 같은 로컬 카운터 클래스가 이미 사용된다 — reserve 결과를 주입할 자리가 마련돼 있다.

**변경 위치**

- variant executor 진입 전: MissionPlanID 3개 reserve, IMP block reserve, aircraft별 pathID block reserve, waypoint block reserve.
- builder들이 받는 인자에 `(start, end)` 또는 로컬 allocator를 추가.
- worker 내부에서는 전역 `_next()`를 호출하지 않도록 가드.

**위험**

- reserve 했으나 실패한 worker의 ID 범위가 누수됨 → forward-only 정책상 ID 낭비. 큰 문제는 아니지만, 누수 분량을 로그로 집계.
- waypoint allocator의 volatile 특성상, reserve 후 다른 흐름이 같은 키를 사용하면 충돌 가능. reserve 시 bump한 amount를 명시적으로 점유하도록.

**선행 의존**: 없음. P0-4 전에 들어가야 한다.

---

## P1-2. option 4 path 폭증 제어

**현재 상태**

- [`area_review.py:598`](../../../modules/mission_planning/MissionPlanner/planning_enhanced/algo/area_review.py)의 `review_assigned_areas_local()`이 split을 결정한다고 추정. sub-agent가 함수 위치까지만 도달.
- 관측: 0303 path 27 → 80 (3배), 같은 aircraft / inputMissionID 범위.

**변경 위치**

- 후처리 병합: 같은 (aircraft, inputMissionID) 범위에서 연속 segment를 합치는 단계를 area_review 출력 직후 추가.
- 또는 area_review 자체의 split 파라미터(예: 최소 segment 길이)를 옵션화.

**위험**

- recon 옵션의 정확도 요구 (정찰밀도) 때문에 split이 의도된 동작일 수 있음. 정확도와 path 수의 trade-off를 옵션 파라미터로 노출하는 방향이 안전.
- pathID forward-only 정책 — 일단 발급한 ID를 회수하지 말고, "발급 안 함" 형태의 사전 제어가 깔끔.

**선행 의존**: 없음. 단 P0-2의 phase timer가 깔린 후에 효과 측정 가능.

---

## P1-3. current-remaining hybrid 치환을 generic 생성 전으로 이동

**현재 상태**

- 현재 호출 순서 ([`mission_planning_gui.py`](../../../modules/mission_planning/mission_planning_gui.py)):
  1. divide_and_pattern (line 7569)
  2. 0303/0304 generic 생성 (export_0303_0304)
  3. `_apply_current_remaining_hybrid_to_variant` (line 7683)
- 즉 generic 0303을 만든 다음 hybrid로 덮어쓰는 구조 → generic 생성 비용이 낭비된다.

**변경 위치**

- divide 직후에 hybrid 적용 여부를 분기 → hybrid가 필요한 variant는 generic 0303/0304 생성을 skip하고 곧장 hybrid path build로 전환.

**위험**

- hybrid path build가 generic 산출물을 입력으로 가정하고 있다면 (분기가 단순 덮어쓰기가 아닌 변환), 입력 자료의 사전 준비가 추가로 필요할 수 있음 — `current_remaining_hybrid.py`의 입력 가정 점검 필요.

**선행 의존**: 없음. 단독으로 효과가 큰 안전한 변경.

---

## P1-4. waypoint ID block을 builder별로 예약 → 0303/0304 병렬

**현재 상태**

- [`export_0303_0304.py:294`](../../../modules/mission_planning/MissionPlanner/planning_enhanced/io/export_0303_0304.py)는 ThreadPoolExecutor(max_workers=2)로 0303/0304를 동시 submit하고 있다. 그러나 두 builder가 동일 전역 waypoint counter의 lock을 잡으므로 실 효과는 직렬에 가깝다.
- **README가 적은 "0303/0304 순차 실행"은 외부적으로 그렇게 보이는 것이지, 실제 코드는 동시 submit + lock 직렬화 구조다.** 이 사실을 사용자에게 명확히 전달할 필요가 있다.

**변경 위치**

- export_0303_0304 진입 전 두 개의 waypoint range를 reserve → d0303 builder에 range A, d0304 builder에 range B 주입.
- d0303/d0304 내부의 `_next_waypoint_id()` 호출 경로를 로컬 allocator로 교체.

**위험**

- waypoint 정확 개수를 사전에 알 수 없으므로 conservative하게 over-reserve해야 함 → ID 낭비.
- 두 builder 사이의 의존성 (있으면) 점검 필요. 아주 단순 가정상 둘은 입력 분리되어 있어야 한다.

**선행 의존**: P1-1.

---

## P1-5. 0303 내부 aircraft 단위 병렬화

**현재 상태**

- d0303의 `build_flight_plans()`는 mission list 순회 sequential. 같은 aircraft 내 path continuity 때문에 aircraft 단위로 split 후 ThreadPoolExecutor에 던지면 자연스럽게 병렬화 가능.

**변경 위치**: d0303 내부.

**위험**

- aircraft 간 의존이 없다는 가정이 본질적으로 맞는지 재확인 필요. 협업 옵션에서 aircraft 간 시간 동기가 있다면 단순 병렬은 깨진다.

**선행 의존**: P1-4 (각 worker가 자기 waypoint range를 받아야 함).

---

## P1-6. ProcessPoolExecutor 검토

**현재 상태**

- ThreadPool은 GIL에 막혀 CPU-bound geometry/path 생성에서 한계.
- 대신 worker process로 넘기려면:
  - JSON-serializable 입력만 허용 (mission dict, area polygon, DEM tile cache).
  - Qt 객체, GUI signal callback, lazy import 모듈, runtime_settings reload 결과는 직렬화 불가.

**변경 위치**

- divide_and_pattern, d0303 aircraft worker, area_review 정도가 후보.

**위험**

- worker 부팅 비용 (Windows spawn) 100~300ms. 단발성이면 손해. process pool을 process 기간 동안 유지해야 의미.
- pickling 실패가 런타임에서 터지므로 entry payload를 typed dict로 좁혀 사전 검증 필요.

**선행 의존**: P0-2 phase timer로 GIL 병목이 실제로 측정된 후. 그 전에는 추측.

---

## P1-7. 재계획 1회 단위 source artifact 캐시

**현재 상태**

- next-collab/prior helper 경로에서 source MissionPlan / IMP / FlightPath / `pathID→waypointIDs`를 반복 로드한다.
- [`prior_mission_pipeline_impl.py:113-127`](../../../modules/mission_planning/pipelines/prior_mission_pipeline_impl.py)의 `_load_mission_helpers_module()`은 모듈 단위 캐시는 있지만, 그 안의 데이터 캐시는 별도.

**변경 위치**: 재계획 1회 컨텍스트(예: dataclass)에 1차 로드 결과를 보관하고, 같은 재계획 안에서 helper들이 그것을 공유.

**위험**: 재계획 도중 source가 갱신될 일은 거의 없지만, 외부 쓰기 흐름이 끼어들 가능성을 한 번 점검.

---

## P2-1. push_center hot module resolution 캐시

**현재 상태**

- [`message0902_push.py:32-37`](../../../modules/common/push/message0902_push.py), `message0301_push.py:26-37`, `message0305_push.py:32-40`의 `_cs(name)`이 매 호출마다 `importlib.import_module()`을 3회 시도한다.

**변경 위치**: 모듈 전역 dict에 resolved 결과 캐시.

**위험**: 모듈 reload 시나리오를 쓰지 않으면 거의 zero risk.

---

## P2-2. 0902 sidecar disk IO

**현재 상태**: [`replan_request_transport_store.py:57-80`](../../../modules/common/replan_request_transport_store.py)의 `save_payload`가 매 0902 수신마다 호출 (json + temp + rename, 50~200KB).

**옵션**:
- 화이트리스트 필드만 저장
- 디버그/벤치마크 모드에서만 저장
- compact JSON

**위험**: replan audit trail이 줄어듦. 본 기능에 사용되지 않으므로 디버그 토글로 가는 게 안전.

---

## P2-3. force-direct 흐름 grace 축소

**현재 상태**: [`mission_planning_gui.py:3919`](../../../modules/mission_planning/mission_planning_gui.py)의 `_queue_post_0301_delivery()`에서 `grace_ms = 1800 + 250 * (plan_count - 1)`, fallback `grace_ms + 1200`.

**변경 위치**: force_direct=True 경로만 분기해서 grace 축소.

**위험**: 수신측이 0305 status=2를 기다리지 않은 채 0903을 받으면 race. delivery 직전에 0305 ack 확인 로직을 두는 편이 안전.

---

## P2-4. 모니터링 큐 오버헤드

**현재 상태**: [`replan_queue_manager.py:200`](../../../modules/monitoring/logic/replan_queue_manager.py) 부근에서 payload deep-copy. monitoring_gui의 UI refresh가 매 큐 변동마다 발생.

**변경 위치**: deep-copy를 reference + 화이트리스트 필드 추출로 교체. UI refresh throttle.

**위험**: UI 동시성 — Qt event loop 위에서만 갱신되도록 유지.

---

## 이미 적용되어 있는 항목 (TODO에서 제외 후보)

- [`mission_helpers.py:131-168`](../../../modules/mission_planning/MissionPlanner/planning_enhanced/algo/mission_helpers.py)의 `_terrain_elev_cached(...)` lru_cache(65536) — README의 "DEM/terrain memoize"는 코드상 이미 들어가 있다. 다만 round precision은 함수 안의 `math.isclose()` 비교 방식이고, 명시적 `round(lat,5)` 키가 아니므로 효율 측정은 별도로 해야 한다.
