# Third Review Risk Audit

요청: 리팩토링 착수 전 빠진 것이 없는지, 현재 기능 변화가 생기지 않을지 한 번 더 조심스럽게 검토한다.

범위: 코드 수정 없음. `modules/mission_planning`의 active entrypoint, 0902/replan, delivery, 삭제/rename 리스크를 실제 검색 결과와 대조했다.

## 결론

기존 문서의 큰 방향은 맞다. 다만 실제 기능 변화 위험을 막으려면 Phase 0에 아래 계약을 추가해야 한다.

1. `mission_planning_gui.py` 공개 실행 경로와 mission role/control port 계약
2. power-off/deferred/replay-capture가 포함된 0902 수신 계약
3. quality-speed 직접전송과 0901/0702 suppression 계약
4. attack delivery suppress flag 계약
5. post-delivery waypoint mark와 mission-area snapshot carry-forward 계약
6. monitoring/common/app에서 mission_planning 내부를 import하는 외부 계약
7. `logic_test`, tool GUI, portable bundle의 manual workflow owner decision

따라서 바로 대규모 move/delete로 들어가면 안 된다. 첫 코드 리팩토링은 wrapper/compat를 유지한 작은 extraction만 가능하고, 삭제는 아직 보류해야 한다.

## 1. Launcher/Entrypoint Contract

확인 근거:

- `run.py:465` mission role script map이 `"mission": "mission_planning_gui.py"`를 사용한다.
- `run.py:1276`, `run.py:1462`에도 `mission_planning_gui.py` basename 기반 role 판단이 있다.
- `app/ui/main_window.py:1306-1307`는 `modules/mission_planning/mission_planning_gui.py`와 `app/modules/mission_planning/mission_planning_gui.py` 후보 경로를 직접 찾는다.
- `mission_planning_gui.py:15459-15460`은 직접 실행 가능한 PyQt entrypoint다.

보존 계약:

- root `modules/mission_planning/mission_planning_gui.py` 파일명은 유지한다.
- 내부 구현을 옮겨도 `MainWindow` export와 direct executable behavior를 유지한다.
- mission role control port `45981`, `KU_ROLE=mission`, `KU_CONSOLE_TITLE` 흐름은 launcher contract inventory에 포함한다.

추가 TODO:

- `run.py` mission role launch smoke
- `app/ui/main_window.py` script candidate smoke
- direct `python modules/mission_planning/mission_planning_gui.py` launch smoke
- `KU_ROLE=mission`이 산출물 SW code를 `MMR`로 유지하는지 확인하는 smoke
- `run.py`와 app launcher의 env/cwd/control-port parity smoke
- nFusion config 있음/없음 실패 정책과 `MMR_ReceiveNode` channel smoke

추가 sub-agent 근거:

- `run.py:8`은 기본 `KU_ROLE=decision`을 설정하고, `mission_planning_gui.py:8`이 mission process에서 `mission`으로 덮어쓴다.
- `d0301.py`, `d0302.py`, `d0304.py`는 `KU_ROLE`로 module/SW code를 결정한다.
- `run.py`와 `app/ui/main_window.py`는 mission process env/cwd/control 설정을 서로 다른 코드 경로에서 구성한다.
- `run.py` cold start는 mission 단독 실행이 아니라 mission/monitor/decision/info 전체 GUI launch를 포함한다.
- mission GUI는 `MMR_ReceiveNode`로 nFusion receiver를 초기화한다.

## 2. 0902 Intake/Deferred Contract

확인 근거:

- `mission_planning_gui.py:2191-2206` power state, deferred replan list, pending push, delay timer state가 초기화된다.
- `mission_planning_gui.py:2251-2253` replan callback과 power gate hook이 설치된다.
- `mission_planning_gui.py:2364` deferred replan resume signal이 연결된다.
- `mission_planning_gui.py:6439-6751` 0902 parsing, replay capture, context normalization, power guard, delayed scheduling이 같은 흐름에 있다.
- `mission_planning_gui.py:6736-6738` 지연이 있는 0902는 deferred queue로 들어가고 replay capture가 별도로 수행된다.
- `mission_planning_gui.py:6743-6748` 새 0902 수신 시 pending delivery와 attack delivery buffer가 초기화된다.

보존 계약:

- power OFF 상태에서는 0902를 무시한다.
- delay가 있는 0902는 `_deferred_replan_requests` 순서와 resume signal semantics를 유지한다.
- replay capture path와 `REPLAN_CAPTURE_0902` 동작은 추출 중 바뀌면 안 된다.
- 새 0902 수신 시 이전 `_pending_plan_push`와 attack delivery buffer reset 동작을 유지한다.

추가 TODO:

- power-off 0902 ignored fixture
- delayed 0902 queue ordering fixture
- replay capture sidecar/archive smoke
- new-0902 pending-delivery reset smoke
- 0902 ID extraction priority fixture: `optionList/pendingOptionList`, `missionPlanIDList`, `replanDetail.missionPlanID`
- trigger별 delay fixture: collab reexecute short delay, 0402/attack close immediate, 일부 0401/RTB delayed
- store-backed detail fixture: next-collab/path-deviation/imaging detail JSON reload

추가 sub-agent 근거:

- 0902 normalizer는 `optionList/pendingOptionList`를 먼저 보고, 이후 `missionPlanIDList`, `replanDetail.missionPlanID`를 fallback으로 사용한다.
- `qualityMonitorSep`/quality-speed는 option names를 비우고 direct delivery로 전환한다.
- `collabReexecuteInputRefresh`, `0402`, `attackClosedDestroyed`, RTB류 0401은 서로 다른 delay/deferred semantics를 가진다.
- captured 0902 replay와 `DSS_Internal/*_replan/*_detail_<plan>.json` 기반 재로드를 payload fixture와 함께 검증해야 한다.

## 2.1 Dispatcher/Monitoring Contract

확인 근거:

- GUI dispatcher는 attack, post-attack, next-collab, imaging/quality, path-deviation, prior, general 순서로 전용 pipeline을 시도한다.
- 전용 pipeline이 실패해도 `handled=True`로 일반 fallback을 막는 경로가 있다.
- monitoring queue manager는 target-detection, non-target, close/rejoin payload 우선순위와 suppress flag를 관리한다.
- monitoring GUI/logic은 source/current plan을 현재 적용 plan으로 rebound하는 경로를 갖는다.

보존 계약:

- dispatcher의 `handled`는 성공 여부가 아니라 "일반 경로로 내려가지 말라"는 의미를 포함한다.
- 전용 trigger 실패 뒤 일반 3-option replan이 생성되지 않는 현재 동작을 유지한다.
- monitoring queue priority, source/current plan rebound, suppress flag는 mission dispatcher와 함께 regression scope에 넣는다.
- current-remaining/hybrid는 별도 trigger가 아니라 general replan 내부 보강 단계다. hybrid 실패 시 generic path를 유지해야 한다.

추가 TODO:

- dispatcher priority + handled/skipped/fallback fixture
- monitoring queue priority/source-plan rebound fixture
- current-remaining hybrid failure fallback fixture
- hybrid merge 이후 pathID mapping fixture

## 3. Delivery State Machine Contract

확인 근거:

- `mission_planning_gui.py:1805-1817` plan count와 force-direct 여부로 post-0301 delivery delay/mode가 결정된다.
- `mission_planning_gui.py:4839-5012`는 0301 이후 0305 completion, 0901/0903/0702 후속 전송을 결정한다.
- `mission_planning_gui.py:4951-4953` quality-speed는 force-direct와 0702 suppression으로 강제된다.
- `mission_planning_gui.py:4956`, `mission_planning_gui.py:5803`, `mission_planning_gui.py:6014-6035`는 attack suppress flag가 post-0301, 0305, 0901 단계에 개입한다.
- `mission_planning_gui.py:4960-4966` execution option mode에서는 0901을 보낸다.
- `mission_planning_gui.py:4975-4993` direct mode에서는 0903을 보내고 조건에 따라 0702 fallback을 보내거나 억제한다.
- `mission_planning_gui.py:5245-5300` pending 0301 push flush에서 force-direct, quality-speed, post-delivery 후처리를 다시 적용한다.
- `mission_planning_gui.py:6039-6046` quality-speed context는 0901 option 생성을 차단한다.

보존 계약:

- 단순 순서만이 아니라 delivery mode matrix를 유지해야 한다.
- quality-speed replan은 0901 option 생성 없이 direct delivery로 유지한다.
- force-direct이면서 `suppress_0702_fallback=False`인 경우 0903 뒤 0702 fallback을 유지한다.
- attack suppress flag는 stale context 검증 후 post-0301/0305/0901 delivery를 막는다.
- 0305 status=2 실패/억제 시 post-0301 delivery drop 동작을 유지한다.

추가 TODO:

- fake `push_message` 기반 delivery matrix smoke
- quality-speed no-0901/no-0702 smoke
- force-direct 0903+0702 fallback smoke
- attack suppress flag context-match/stale-ignore smoke
- 0305 suppression/drop smoke
- 0301 failure blocks 0305/0901/0903 smoke
- mode-ready/completion-ready/grace-timeout flush smoke
- flow별 0702 fallback matrix: next-collab/post-attack/prior-post-rejoin suppress, path-deviation/prior direct fallback 가능, imaging quality만 suppress

## 4. Post-Delivery Runtime Side Effects

확인 근거:

- `mission_planning_gui.py:5028-5103` post-delivery waypoint mark payload를 normalize/merge/schedule한다.
- `mission_planning_gui.py:5117-5229` mission-area snapshot carry-forward payload를 normalize/merge/schedule한다.
- `mission_planning_gui.py:5299-5300` 0301이 성공한 뒤 post-delivery waypoint mark와 snapshot carry-forward가 예약된다.
- `mission_planning_gui.py:15389-15422` pending delivery merge 때 post-delivery payload도 병합된다.

보존 계약:

- delivery extraction 시 0301 성공 이후에만 후처리를 예약하는 조건을 유지한다.
- 여러 pending delivery가 병합될 때 waypoint mark/snapshot carry-forward payload merge rules를 유지한다.
- 후처리 실패는 log/metric으로 남기되 delivery 자체를 되돌리지 않는 현재 흐름을 유지한다.

추가 TODO:

- post-delivery waypoint mark fake smoke
- snapshot carry-forward fake smoke
- merged pending delivery 후처리 smoke

## 5. External Import Contract

확인 근거:

- `run.py:298-299`는 `MissionPlanner.data_def.id_allocator`와 `id_allocator_0202`를 직접 import해 reset한다.
- `app/ui/main_window.py:28-31`은 `MissionPlanner.runtime_settings`의 FOV DB API를 직접 import한다.
- `modules/common/agent_status_snapshot.py:14-21`은 mission_planning runtime tracking state를 import한다.
- `modules/common/next_collab_replan_store.py:3`은 mission_planning runtime store를 re-export한다.
- `modules/monitoring/logic/*`는 `MissionPlanner.runtime_settings`, `runtime.attack_tracking_state`, `runtime.prior_tracking_state`, `runtime.attack_assignment_state` 등을 직접 import한다.

보존 계약:

- `MissionPlanner.runtime_settings`, `MissionPlanner.data_def.id_allocator`, `runtime.attack_tracking_state`, `runtime.prior_tracking_state`, `runtime.attack_assignment_state`, next-collab stores는 이동 전 wrapper를 제공한다.
- external callers가 사용하는 public function name은 signature snapshot에 포함한다.
- monitoring/common/app imports는 `modules/mission_planning` 내부 refactor 검증에 포함한다.

추가 TODO:

- external import graph snapshot
- moved-module wrapper import smoke
- ID reset parity smoke through `run.py`
- monitoring replan logic import smoke
- untracked active runtime module inventory: `replan_id_reservation.py`, `replan_validation.py`
- runtime state/resource artifact manifest

## 6. Delete/Rename Hold Additions

확인 근거:

- `modules/mission_planning/MissionVisualizer/main_visualizer.py:52`는 root `id_relationship_tab.RelationshipCache`를 import한다.
- `modules/mission_planning/MissionPlanner/tools/main_visualizer.py:5`, `:1312-1319`는 별도 public/manual visualizer entrypoint처럼 동작한다.
- `modules/mission_planning/manual/lah_rl_planner_gui.py`는 `MissionPlanner/portable_mission_bundle/models/latest_model.zip`을 직접 참조한다. Root `lah_rl_planner_gui.py`는 compatibility wrapper다.
- `MissionPlanner/portable_mission_bundle/portable_mission/service.py:31-32`는 `latest_model.zip`, `model_config.json`을 직접 로드한다.
- `runtime/next_collab_line_runner.py:32-39`는 `next_area_mode.config`, `next_area_mode.planner_window`를 active import한다.
- `runtime/next_collab_division_runner.py:17-27`는 `planners/next_collab_division` 내부 위젯/geo/runtime logic을 active import한다.
- `logic_test/division_test/division_planner_gui.py:37-65`도 active `planning_enhanced`와 runtime settings를 직접 import한다. active runtime은 아니지만 golden/manual fixture일 수 있다.

보존 계약:

- `next_area_mode`, `planners/next_collab_division`, `MissionVisualizer/main_visualizer.py`, root `id_relationship_tab.py`, `manual/lah_rl_planner_gui.py`, root `lah_rl_planner_gui.py` wrapper, portable model/config artifacts는 삭제 금지 또는 보류다.
- `logic_test/division_test/**`는 active runtime 삭제 후보가 아니라 manual/golden fixture 여부를 먼저 결정한다.
- duplicate visualizer는 삭제가 아니라 한쪽을 wrapper로 고정한 뒤 owner decision을 받는다.
- `runtime/replan_id_reservation.py`, `runtime/replan_validation.py`, `runtime/source_artifact_cache.py`, `runtime/debug_artifacts.py`는 active imports가 있으므로 미추적/작아 보인다는 이유로 삭제하면 안 된다.
- current-remaining hybrid 계열, next-collab support 계열, `recon_specialized_pipeline.py`, `MissionPlanner/UAV_missionPlanning.py`는 active flow 또는 bare import 호환성 때문에 보류한다.
- ID/state JSON, `DSS_Internal` runtime state JSON, FOV/resource DB, DEM/GeoTIFF, portable bundle root files는 code cleanup 대상이 아니라 runtime artifact manifest 대상으로 분리한다.

추가 TODO:

- manual/tool entrypoint inventory
- `python -m` advertised launch string inventory
- owner decision: keep, wrapper, archive, delete
- duplicate visualizer public entrypoint decision
- attack assistance subprocess smoke: `lah_attack_assistance.py --friendly-lat/lon --enemy-lat/lon --output-json`
- portable bundle `python app.py`/`run_portable.bat` smoke with env port override
- next-collab/next-area manual planner default flow-mode smoke

## Refactoring Gate

다음 코드 변경 전 필수 조건:

- Phase 0 smoke/checklist 중 launcher, 0902, delivery, external import, deletion-hold 항목을 먼저 작성한다.
- 첫 코드 변경은 삭제나 폴더 대이동이 아니라 behavior-preserving extraction으로 제한한다.
- 추출 대상은 wrapper로 기존 function/class 이름을 유지하고, 기존 import path가 살아 있어야 한다.
- 실제 삭제는 별도 batch로 진행하고, 삭제 후보별 reachability와 manual owner decision을 PR에 남긴다.
