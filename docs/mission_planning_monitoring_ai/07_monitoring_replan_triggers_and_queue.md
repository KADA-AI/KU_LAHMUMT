# Monitoring 재계획 트리거와 큐

마지막 재정리: 2026-05-04

Monitoring은 여러 이벤트에서 재계획 요청을 만들지만, `ReplanQueueManager`가 한 번에 하나의 active 재계획만 dispatch하도록 직렬화한다. 따라서 트리거 조건과 큐 상태를 함께 봐야 실제 0902 송신 순서를 이해할 수 있다.

## 현재 runtime 토글

실제 기준 파일은 프로젝트 루트의 `replan_settings.json`이다.

| 토글 | 현재값 |
| --- | --- |
| `input_refresh` | true |
| `prior_mission` | true |
| `dl_risk` | false |
| `target_detection` | true |
| `post_attack_rejoin` | true |
| `forced_command` | true |
| `rtb` | true |
| `path_deviation` | true |
| `quality_monitor` | true |
| `quality_speed` | false |
| `imaging_schedule` | false |
| `next_collab` | true |
| `fuel_threshold` | false |

모듈 내부의 `modules/monitoring/replan_settings.json`은 값이 다를 수 있다. 문서나 로그 분석에서는 루트 설정을 우선한다.

## 트리거 표

| source tag | 입력 | level | trigger/type | 주요 조건 |
| --- | --- | --- | --- | --- |
| `input_refresh` | `0201` | 3 | `0201` / `inputRefresh` | system mode 3/4, toggle on, reexecute active block 없음 |
| `reexecute_0201` | `0201` | 3 | reexecute detail | 협업 재수행/좌표 재실행 흐름 |
| `prior_mission` | `0202`, `0401` | 4 | prior / `priorClosedResume` | 선행 임무 또는 선행 종료 후 원 임무 복귀 |
| `dl_risk` | `0401` | 5 | DL risk | 현재 토글 off |
| `target_detection` | `0402` | 2 | `0402` / target detection | target actionable, cooldown, attack slot 상태 |
| `post_attack_rejoin` | `0402` | 2 또는 상세 기준 | `0402` / `attackClosedDestroyed` | target destroyed/closed, attack tracking assignment |
| `path_deviation` | `0401` | 3 | `0401` / `pathDeviation` | turn-radius warning, alternate waypoint, active plan |
| `quality_speed` | `0401` | 3 | `0401` / `qualityMonitorSep` | quality monitor + quality speed toggle on |
| `imaging_schedule` | `0401` | 3 | `0401` / `imagingScheduleDeviation` | imaging schedule toggle on |
| `next_collab` | `0803` | 3 | `0803` / `nextCollaborativeMission` | execute=1, 다음 협업 임무 가능 |
| `rtb` | `0401` | 1 | RTB/fault detail | unexpected RTB, abnormal health, signal loss, payload unavailable |
| `forced_command` | `0802` | 1 | forced command | mandatoryType 1/2/3 |
| `manual` | UI | 다양 | manual detail | 수동 재계획 |

`current_remaining`은 독립 source tag라기보다 RTB/forced/path/일반 재계획 payload에 붙는 context다. `currentRemainingCollaborativeReplan` 상세에는 entry coordinate, entry aircraft list, 현재 input mission 정보가 들어간다.

## ReplanQueueManager

대표 파일: `modules/monitoring/logic/replan_queue_manager.py`

큐 매니저의 역할은 다음과 같다.

- 중복 signature 생성 및 dedup
- active/queued/history 관리
- 한 번에 하나의 0902만 dispatch
- target detection delay/merge/priority 처리
- post-attack rejoin이 active target detection을 선점하는 처리
- option suppression flag 기록
- timeout 및 완료 신호 처리
- 큐 탭 표시용 snapshot 제공

대표 stage label은 다음과 같다.

| stage | 의미 |
| --- | --- |
| `dispatching` | 0902 송신 중 |
| `waiting_output` | Mission Planning 산출 대기 |
| `planning_started` | 계획 시작 신호 |
| `planning_finished` | 계획 완료 신호 |
| `options_requested` | 옵션 요청/안내 단계 |
| `options_sent` | 옵션 정보 송신됨 |
| `plan_update_sent` | 0903/계획 갱신 송신됨 |
| `decision_received` | 0702 결정 수신 |
| `dispatch_failed` | 송신 실패 |
| `option_suppressed` | 옵션 흐름 억제 |
| `no_replan_needed` | 0001 no-op |
| `timed_out` | active timeout |

## 완료 신호

큐는 다음 신호를 보고 active 재계획을 완료하거나 상태를 갱신한다.

| 신호 | 처리 |
| --- | --- |
| `0305` | 계획 상태/옵션 정보 수신. 실제 MMR 송신 기준 `missionPlanningStatus`는 1=수행 중, 2=완료 흐름을 사용한다. |
| `0701` | 옵션 안내/요청. 현재 `release_on_option_info=false`라 이것만으로 active를 해제하지 않는다. |
| `0702` | 사용자 결정 또는 자동 선택. `ignore=2`이면 적용 신호다. |
| `0903` | 직접 계획 갱신. active 완료 신호로 취급된다. |
| `0001` | 실패/no-op/suppression 안내. active 완료 또는 suppressed 처리. |
| timeout | `active_timeout_ms` 초과 시 timed out 처리 |

현재 `replan_queue` 설정:

| 항목 | 값 |
| --- | --- |
| `active_timeout_ms` | 45000 |
| `history_limit` | 30 |
| `target_dispatch_delay_ms` | 800 |
| `release_on_option_info` | false |

## 중복 signature

큐는 payload 전체를 그대로 비교하지 않고 source/level/trigger/대상 ID를 조합해 signature를 만든다.

`0402` target detection은 특별 취급된다. signature에 다음 값이 들어간다.

- source tag
- replan level
- trigger
- input IDs
- source/current plan
- attack target IDs
- target bundle count
- follow-up 여부

그 외 payload는 reason, trigger, aircraft, target, watcher, prior, mandatory, input IDs, 정리된 detail 등을 사용한다.

이 방식 때문에 같은 물리 이벤트가 약간 다른 JSON 필드 순서로 들어와도 중복으로 보지 않는다. 반대로 target bundle이 달라지면 같은 0402라도 별도 재계획이 될 수 있다.

## Target detection 특수 처리

target detection은 일반 큐와 다르게 동작한다.

- dispatch 전 `target_dispatch_delay_ms=800` 동안 묶어서 볼 수 있음
- target type priority로 정렬
- 같은 target/유사 bundle은 merge 가능
- post-attack rejoin이 들어오면 active target detection을 option-suppressed로 선점 가능
- post-attack 완료 뒤 target detection을 resume할 수 있음

`target_info.json`의 `isUsed`, `isIgnored`, `isDestroyed` 상태와 attack slot 상태가 큐 진입 여부에 영향을 준다.

## Path deviation 설정

현재 루트 `replan_settings.json` 기준 주요 값은 다음과 같다.

| 항목 | 값 |
| --- | --- |
| `turn_rate_threshold_dps` | 2.0 |
| `turn_window_s` | 8.0 |
| `stale_timeout_s` | 5.0 |
| `spiral_window_s` | 75.0 |
| `spiral_min_points` | 5 |
| `center_ignore_radius_m` | 40.0 |
| `watch_angle_deg` | 60.0 |
| `warning_angle_deg` | 90.0 |
| `hold_s` | 2.0 |
| `release_min_distance_m` | 240.0 |
| `release_factor` | 2.3 |
| `alt_waypoint_trigger_s` | 2.0 |
| `alt_waypoint_lead_time_s` | 10.0 |
| `next_mission_entry_lead_time_s` | 10.0 |
| `turn_radius_30_m` | 340.0 |
| `turn_radius_40_m` | 450.0 |
| `turn_radius_50_m` | 560.0 |

`TurnRadiusMonitorTab`의 synthetic alternate waypoint와 predicted entry coordinate가 path deviation, next collab, current remaining replan의 공통 입력으로 쓰인다.

## RTB/forced command 주의점

RTB와 forced command는 availability override를 공유한다. forced return은 RTB suppression 상태를 만들 수 있고, forced wait은 특정 aircraft를 일시적으로 사용 불가 처리한다.

RTB 주요 설정:

| 항목 | 값 |
| --- | --- |
| `unexpected_rtb_flight_mode` | 5 |
| `abnormal_health_value` | 2 |
| `fuel_warning_replan_level` | 2 |
| `signal_loss_grace_ms` | 10000 |
| `replan_hold_ms` | 5000 |
| `fault_unavailable_hold_ms` | 55000 |
| `command_aircraft_id` | 1 |

forced command 주요 설정:

| 항목 | 값 |
| --- | --- |
| `hold_delay_seconds` | 30.0 |
| `signature_dedup_seconds` | 0.6 |

## 0902 dispatch 전 보강

`_prepare_0902_payload_for_dispatch`는 dispatch 직전에 payload를 보강한다.

- `0402`: current/source plan ID, attack target bundle, target fields 보강
- `0401`/path deviation: current/source plan ID 보강
- current remaining: entry aircraft/coordinate 정보 부착
- sidecar 저장소: nFusion 스키마에 없는 확장 `replanDetail` 보존

이 보강 때문에 큐에 들어간 원본 payload와 Mission Planning이 받는 최종 payload가 다를 수 있다. 정확한 분석은 `replan_request_transport` 저장본을 확인한다.
