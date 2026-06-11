# Bare Import Bootstrap Without Project-Root Shims

Date: 2026-06-05, Asia/Seoul

## Decision

Do not keep project-root `AnS/`, `data_def/`, or `config.py` compatibility shims.

Bare imports are preserved by bootstrapping `modules/mission_planning/MissionPlanner` onto `sys.path` before mission-planning code imports:

- `import AnS`
- `import AnS.mission_pipeline`
- `import data_def`
- `from data_def import d0302, d0303, d0304`
- `from data_def.id_allocator import ...`
- `import config`
- `from config import DEFAULT_SWEEP_SEPARATION_M`

## Rationale

- The user-facing project root should not gain mission-planning folders or files just to support legacy imports.
- The real implementation already lives under `modules/mission_planning/MissionPlanner`.
- `ensure_mission_planner_import_paths()` provides the required path bootstrap for GUI/runtime flows.
- Canonical packages still register bare aliases when imported from the `MissionPlanner` path:
  - `MissionPlanner/AnS/__init__.py`
  - `MissionPlanner/AnS/mission_pipeline.py`
  - `MissionPlanner/data_def/__init__.py`
  - `MissionPlanner/data_def/d0303.py`
  - `MissionPlanner/data_def/mission_helpers.py`
  - `MissionPlanner/config.py`

## Guardrails

- `smoke_import_contract.py` now fails if project-root `AnS/`, `data_def/`, or `config.py` exists.
- Bare import smoke calls `ensure_mission_planner_import_paths()` before importing bare planner modules.
- `smoke_compat_root_strategy_contract.py` verifies internal `MissionPlanner` bare-import targets remain present while project-root shims stay absent.
- Root surface inventory keeps `modules/mission_planning` focused on mission-planning package contents rather than adding top-level project shims.

## Verification

```powershell
python "docs\mission planning refactoring\smoke_import_contract.py" --require-git-tracked
python "docs\mission planning refactoring\smoke_compat_root_strategy_contract.py"
python "docs\mission planning refactoring\smoke_wrapper_template_contract.py"
python "docs\mission planning refactoring\smoke_planner_hot_reload_snapshot.py"
```
