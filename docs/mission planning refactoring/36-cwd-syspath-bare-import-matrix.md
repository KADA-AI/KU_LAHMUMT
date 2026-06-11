# cwd, sys.path, and Bare Import Matrix

## 목적

`modules/mission_planning` 리팩토링 중 cwd와 `sys.path` 차이 때문에 `AnS`, `data_def`, `config`, `mission_planning_gui.py` import 경로가 갈라지지 않도록 현재 지원 matrix를 고정한다.

## Matrix

| Context | cwd | sys.path 전제 | 지원 계약 | 검증 |
| --- | --- | --- | --- | --- |
| Repo root shell with explicit bootstrap | repo root | caller가 `planner_runtime.ensure_mission_planner_import_paths(project_root)` 호출 | project-root `AnS/`, `data_def/`, `config.py` shim은 없어야 한다. Bare `AnS`, `AnS.mission_pipeline`, `data_def`, `data_def.d0302/d0303/d0304`, `data_def.id_allocator`, `config`는 내부 `MissionPlanner` path bootstrap으로 import 가능하고 canonical module/function identity를 유지한다. | `smoke_cwd_import_matrix.py` repo-root bootstrap case, `smoke_import_contract.py` |
| Mission dir with explicit bootstrap | `modules/mission_planning` | caller가 repo root를 넣은 뒤 `planner_runtime.ensure_mission_planner_import_paths(project_root)` 호출 | `sys.path[:4]`는 repo root, `modules`, `modules/mission_planning`, `modules/mission_planning/MissionPlanner`. 이후 bare `AnS`, `data_def`, `config` import 가능. | `smoke_cwd_import_matrix.py` mission-dir bootstrap case |
| Mission dir GUI import | `modules/mission_planning` | `import mission_planning_gui` | module top-level bootstrap이 repo root를 넣고 `_bootstrap_paths(Path(__file__))`가 repo root로 `chdir`한다. `PROJECT_ROOT`와 `KU_ROLE=mission` 유지. | `smoke_cwd_import_matrix.py` mission-dir GUI import case |
| Mission dir direct script | `modules/mission_planning` | `python mission_planning_gui.py` 직접 실행 | script top-level bootstrap이 repo root를 넣고 `_bootstrap_paths(Path(__file__))`가 repo root로 `chdir`한다. `MISSION_PLANNING_GUI_SMOKE_LAUNCH=1` headless launch 유지. | `smoke_cwd_import_matrix.py` direct script case |
| MissionPlanner legacy cwd | `modules/mission_planning/MissionPlanner` | bare `AnS`, `data_def`, `config`, `planning_enhanced` | legacy MissionPlanner cwd behavior is supported. Bare imports resolve to canonical modules or existing MissionPlanner packages without splitting identity. | `smoke_cwd_import_matrix.py` MissionPlanner legacy cwd case, `smoke_import_contract.py` |
| Dashboard child process | repo root | launcher가 child `cwd=repo root`로 실행 | `run.py` route와 `app/ui/main_window.py` route 모두 mission child cwd가 repo root여야 한다. mission launch env parity 유지. | `smoke_cwd_import_matrix.py` dashboard child case, `smoke_launch_env_parity.py`, `smoke_run_py_cold_start.py` |
| Raw mission dir without bootstrap | `modules/mission_planning` | repo root/PYTHONPATH/explicit bootstrap 없음 | bare `AnS` import는 현재 지원 계약이 아니다. 이 상태를 지원으로 간주하지 않는다. | 관찰 결과: sanitized env에서 `ModuleNotFoundError: No module named 'AnS'` |

## Bootstrap Contracts

### Bare import bootstrap without project-root shims

- Project-root `AnS/`, `data_def/`, `config.py` shim은 없어야 한다.
- `planner_runtime.ensure_mission_planner_import_paths(project_root)`는 repo root, `modules`, `modules/mission_planning`, `modules/mission_planning/MissionPlanner`를 이 순서로 `sys.path` 앞에 둔다.
- 이후 bare `AnS`, `AnS.mission_pipeline`, `data_def`, `data_def.d0301/d0302/d0303/d0304`, `data_def.id_allocator`, `data_def.mission_helpers`, `data_def.search_speed`, `config`는 내부 `MissionPlanner` tree에서 resolve된다.
- `AnS.mission_pipeline`은 canonical `modules.mission_planning.MissionPlanner.AnS.mission_pipeline`과 같은 module object여야 한다.
- `config` assignment forwarding은 project-root proxy file이 아니라 canonical `modules.mission_planning.MissionPlanner.config`를 통해 유지한다.
- `planning_enhanced`는 `modules/mission_planning/MissionPlanner` cwd에서 bare import 가능한 legacy package로 유지한다.

### Mission GUI bootstrap

- `mission_planning_gui.py`는 package import 전에 script 기준 repo root를 `sys.path`에 넣는다.
- `modules/mission_planning/ui/mission_planning_gui_env.py::_bootstrap_paths(Path(__file__))`는 `modules/mission_planning`, `modules/common`, repo root를 path에 넣고 repo root로 `chdir`한다.
- `PROJECT_ROOT`는 repo root여야 하고 `COMMON_DIR`은 `modules/common`이어야 한다.
- `configure_mission_role()`은 import 시점에 `KU_ROLE=mission`을 설정한다.

### Planner runtime bootstrap

- `planner_runtime.MISSION_PLANNER_IMPORT_RELATIVE_PATHS`는 다음 네 경로를 포함한다.
  - `.`
  - `modules`
  - `modules/mission_planning`
  - `modules/mission_planning/MissionPlanner`
- `ensure_mission_planner_import_paths(project_root)` 호출 후 `sys.path[:4]`는 아래 순서여야 한다.
  - `project_root`
  - `project_root/modules`
  - `project_root/modules/mission_planning`
  - `project_root/modules/mission_planning/MissionPlanner`

## Import-order Risks

- `modules/mission_planning` cwd에서 raw `import AnS`가 우연히 된다고 가정하면 안 된다. direct GUI import/script나 explicit bootstrap 경로만 지원한다.
- root shim과 canonical package를 둘 다 import할 때 module identity가 갈라지면 hot reload와 global state가 깨진다.
- `config`는 proxy라서 읽기뿐 아니라 assignment forwarding도 계약이다.
- `data_def.id_allocator`는 counter/global state를 공유해야 하므로 wrapper assignment와 canonical state identity가 중요하다.
- stdlib `logging` 같은 이름이 local package로 shadowing되면 안 된다. 기존 `smoke_import_contract.py`가 runtime bare import group에서 shadow repair를 확인한다.

## Verification

추가한 smoke:

```powershell
python "docs\mission planning refactoring\smoke_cwd_import_matrix.py"
```

결과:

```text
cwd/sys.path/bare import matrix smoke ok
```

관련 기존 smoke:

```powershell
python "docs\mission planning refactoring\smoke_mission_planning_gui.py" --timeout-s 30
python "docs\mission planning refactoring\smoke_launch_env_parity.py" --timeout-s 30
python "docs\mission planning refactoring\smoke_run_py_cold_start.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
```

## Refactor Guardrails

- project-root `AnS`, `data_def`, `config.py` shim은 만들지 않는다. Bare imports는 explicit `MissionPlanner` path bootstrap으로만 지원한다.
- `mission_planning_gui.py` direct script launch를 깨지 않도록 top-level repo-root insertion을 유지한다.
- `modules/mission_planning` cwd의 `import mission_planning_gui`는 cwd를 repo root로 바꾸는 현재 bootstrap 부작용까지 포함한 계약으로 취급한다.
- `modules/mission_planning/MissionPlanner` legacy cwd에서 `AnS`, `data_def`, `config`, `planning_enhanced` bare imports를 유지한다.
- `ensure_mission_planner_import_paths()`의 네 경로와 순서를 바꾸지 않는다.
- `modules/mission_planning` raw cwd에서 bare imports가 실패하는 것은 현재 비지원 상태다. 이를 지원하려면 별도 TODO와 smoke 갱신이 필요하다.
- dashboard launch child cwd는 repo root로 유지한다.

## 다음 지점

다음 미완료 TODO는 `nFusion config 있음/없음 실패 정책과 MMR_ReceiveNode channel smoke 작성`이다.
