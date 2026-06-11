# 24. Active Import Smoke Progress

## Scope

이번 수정은 Phase 0의 `modules/mission_planning` active import smoke script 작성 항목이다. GUI launch나 control-port 확인은 별도 TODO로 남겨 두고, 현재 활성 canonical module들이 빠르게 import 가능한지 확인하는 smoke를 추가했다.

## Added Script

- `docs/mission planning refactoring/smoke_active_imports.py`

스크립트는 다음 그룹의 활성 모듈을 import하고 주요 public attribute 존재를 확인한다.

- `app`: bootstrap, message handlers, delivery, visualization tab
- `mission_control`: planner runtime, plan metrics
- `replanning`: dispatcher와 trigger별 canonical pipeline
- `runtime`: state/cache/logging/validation/ID, next-collab helpers, JSON/debug helpers
- `engine`: ID allocator와 0301/0302/0303/0304 artifact builders
- `planner`: AnS, data_def, planning_enhanced public packages

## Contract

- repo root, `modules`, `modules/mission_planning`, `MissionPlanner` import path를 명시적으로 세팅한다.
- `QT_QPA_PLATFORM=offscreen`을 기본값으로 둬서 PyQt import가 headless 환경에서 깨지지 않게 한다.
- import 후 stdlib `logging`이 shadow되지 않았는지 확인한다.
- `--group`, `--module`, `--list` 옵션으로 범위를 좁혀 실행할 수 있다.

## Verification

이번 항목을 완료 처리하기 전에 아래 명령을 실행했다.

- `python "docs\mission planning refactoring\smoke_active_imports.py"`
- `python -m py_compile "docs\mission planning refactoring\smoke_active_imports.py" "docs\mission planning refactoring\smoke_import_contract.py"`
- `python "docs\mission planning refactoring\smoke_import_contract.py"`
- `git diff --check -- "docs/mission planning refactoring"`
