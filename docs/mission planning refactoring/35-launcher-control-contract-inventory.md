# Launcher and Control Contract Inventory

## 목적

`modules/mission_planning` 리팩토링 중 public launcher와 dashboard control-plane 동작이 바뀌지 않도록 현재 계약을 고정한다. 이 문서는 코드 이동 전 inventory이며, 런타임 코드는 변경하지 않았다.

## Public Launch Surface

- `modules/mission_planning/mission_planning_gui.py`는 mission planning public executable launcher다.
- `modules.mission_planning.mission_planning_gui.MainWindow`는 계속 import 가능해야 한다.
- `MISSION_PLANNING_GUI_SMOKE_LAUNCH=1`은 headless launch smoke 경로로 유지한다.
- 직접 실행 시 `QApplication`, shared stylesheet, `MainWindow`, `apply_initial_visibility()` 순서로 GUI를 시작한다.
- import 직후 `configure_mission_role()`이 `KU_ROLE=mission`을 강제한다. 부모 `run.py`가 `KU_ROLE=decision`이어도 mission child는 0301/0302/0303/0304 SW code 기준에서 mission role을 유지해야 한다.

## Dashboard Launch Routes

### `run.py`

- `main()`은 dashboard `MainWindow`를 표시하고 `DashboardOrchestrator`를 만든 뒤 800 ms 후 `_launch_all_guis()`를 예약한다.
- `_launch_all_guis()`는 `mission_planning_gui.py`, `monitoring_gui.py`, `decision_support_gui.py`, `info_manage.py` 네 GUI를 hidden으로 launch한다.
- post-launch timers는 1000 ms mode text, 1000 ms log, 1200 ms service-status refresh 순서를 유지한다.
- `run.py`의 role map은 cold start 기준이다.

| Role | Script | Ctrl port | Offset | Console title |
| --- | --- | ---: | --- | --- |
| mission | `modules/mission_planning/mission_planning_gui.py` | 45981 | `40,40` | `KU Mission Planning Console` |
| monitor | `modules/monitoring/monitoring_gui.py` 또는 `test_monitoring.py --gui` | 45982 | `130,90` | `KU Monitoring Console` |
| decision | `modules/decision_support/decision_support_gui.py` | 45983 | `220,140` | `KU Decision Support Console` |
| info | `modules/info_manage/info_manage.py` | 45984 | `310,190` | `KU Info Manage Console` |

`run.py` child process contract:

- `subprocess.Popen([...])` list args를 사용한다. `shell=True`로 바꾸지 않는다.
- child `cwd`는 repo root다.
- `start_new_session=True`다.
- `creationflags_for_subprocess(show_console=should_show_module_consoles(), new_process_group=True)`를 사용한다.
- stale role process cleanup 후 launch한다.
- live child process는 `_role_processes[role]`에 등록한다.
- child env에는 `KU_LAUNCHED_BY_DASHBOARD=1`, non-empty `KU_MISSION_DB_ROOT`, `PYTHONUNBUFFERED=1`이 들어간다.

### `app/ui/main_window.py`

- `MainWindow._launch_role("mission", start_hidden=True)`는 dashboard 내부 launch route다.
- mission candidate 우선순위는 `modules/mission_planning/mission_planning_gui.py`가 먼저이고, legacy assignment GUI는 fallback이다.
- child `cwd`는 repo root다.
- `shell=False`를 명시한다.
- `creationflags_for_subprocess(show_console=should_show_module_consoles())`를 사용한다. 현재 이 경로는 `new_process_group=True`를 쓰지 않는다.
- stale process cleanup 후 launch하고 `_role_processes[role]`에 등록한다.

허용된 차이:

- `run.py` route는 `KU_LAUNCHED_BY_DASHBOARD=1`과 `KU_MISSION_DB_ROOT`를 직접 설정한다.
- `app/ui/main_window.py` route는 현재 두 env를 직접 설정하지 않는다. smoke는 `KU_LAUNCHED_BY_DASHBOARD`가 없거나 `1`인 경우를 허용하고, `KU_MISSION_DB_ROOT`는 있으면 비어 있지 않아야 한다.

## Mission Launch Env Contract

두 mission launch route는 아래 env 값에 대해 parity를 유지해야 한다.

| Env | Value |
| --- | --- |
| `KU_CTRL_PORT` | `45981` |
| `KU_START_HIDDEN` | `1` |
| `KU_HIDE_ON_CLOSE` | `1` |
| `KU_VIEWER_ONLY` | `0` |
| `KU_WINDOW_OFFSET` | `40,40` |
| `KU_CONSOLE_TITLE` | `KU Mission Planning Console` |
| `PYTHONUNBUFFERED` | `1` |

Launcher 관련 console env:

- `KU_SHOW_RUN_CONSOLE`: root dashboard console 표시 여부.
- `KU_SHOW_MODULE_CONSOLES`: child module console 표시 여부.
- `KU_RUNPY_GUI_RELAUNCHED`: Windows hidden launcher 재실행 guard.

## Control Plane Contract

- Control payload는 UDP JSON이다.
- 송신 host는 `127.0.0.1`이다.
- 수신 listener는 `modules.common.ctrl_listener.start_ctrl_listener()`가 만들고, `SO_REUSEADDR`를 켠 UDP socket을 `127.0.0.1:<port>`에 bind한다.
- listener thread name은 `CTRL@{port}`이고 daemon thread다.
- `env_ctrl_port(default)`는 `KU_CTRL_PORT`가 양수 int이면 그 값을 쓰고, 아니면 default port를 쓴다.
- mission GUI는 `env_ctrl_port(45981)`로 listener를 시작한다.
- `run.py`의 `_send_role_ctrl(role, payload, port=None)`은 payload에 `role`이 없으면 삽입하고 role별 port로 전송한다.
- `app/ui/main_window.py`의 `_broadcast_ctrl()`와 `_send_ctrl_single()`도 `127.0.0.1` UDP JSON control을 사용한다.

Window-control command contract:

- show commands: `show_window`, `show_gui`, `open_gui`, `raise_window`
- hide commands: `hide_window`, `hide_gui`
- optional delay: `delay_ms`
- target keys: `role` 또는 `target`
- mission accepted targets: `mission`, `assignment`, `mission_planning`, `mmr`, `all`, `*`
- `KU_START_HIDDEN=1`이면 child GUI는 시작 시 숨김 상태다.
- `KU_HIDE_ON_CLOSE=1`이면 close event는 process 종료 대신 hide로 처리된다.

Mission-specific control handling order:

1. `MainWindow._handle_ctrl_payload()`가 먼저 `handle_window_control(..., role="mission")`을 호출한다.
2. window-control이 처리되지 않은 payload만 mission-specific `self_check`, `mode`, `db_root`, `init_plan_context` 처리로 넘어간다.
3. power off 상태에서는 `mode`, `db_root`, `debug_db_root`, `log_db_root` 외 control payload를 차단한다.

## Path and cwd Contract

- `run.py` `_bootstrap_paths()`는 project root, `modules`, `modules/common`, `modules/decision_support` 경로를 sys.path에 넣고 repo root로 `chdir`한다.
- `mission_planning_gui.py`는 package import 전에 project root를 sys.path에 넣는다.
- mission child subprocess `cwd`는 모든 launcher smoke에서 repo root로 검증한다.
- 더 자세한 import/cwd matrix는 별도 TODO인 `cwd/sys.path/bare import matrix 작성`에서 다룬다.

## Verification

현재 이 inventory를 증명하는 smoke:

```powershell
python "docs\mission planning refactoring\smoke_mission_planning_gui.py" --timeout-s 30
python "docs\mission planning refactoring\smoke_launch_env_parity.py" --timeout-s 30
python "docs\mission planning refactoring\smoke_run_py_cold_start.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
```

Expected success lines:

- `mission_planning_gui all smoke ok`
- `mission launch env parity smoke ok`
- `run.py cold-start GUI/control-port smoke ok`
- `mission planning refactor import-contract smoke ok`

## Refactor Guardrails

- `mission_planning_gui.py`를 rename하거나 public executable 역할에서 제거하지 않는다. 이동이 필요하면 compatibility launcher를 먼저 둔다.
- `MainWindow` export를 유지한다.
- mission launch env parity table의 값은 기능 변경 의도가 없는 한 바꾸지 않는다.
- role port 45981/45982/45983/45984를 바꾸지 않는다.
- control sender/listener는 localhost UDP JSON 계약을 유지한다.
- `handle_window_control(..., role="mission")`을 mission-specific control 처리보다 먼저 실행한다.
- `run.py` route와 `app/ui/main_window.py` route의 현재 process-group 차이는 의도된 차이로 유지하거나, 바꾸려면 smoke와 문서를 함께 갱신한다.

## 다음 지점

다음 미완료 TODO는 `cwd/sys.path/bare import matrix 작성: repo root, modules/mission_planning, dashboard child cwd`다.
