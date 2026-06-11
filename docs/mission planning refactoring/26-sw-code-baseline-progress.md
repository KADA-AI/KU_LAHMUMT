# 26. SW Code Baseline Progress

## Scope

이번 수정은 Phase 0의 `KU_ROLE=mission`과 0301/0302/0303/0304 SW code baseline smoke 작성 항목이다.

## Added Script

- `docs/mission planning refactoring/smoke_sw_code_baseline.py`

## Contract

스크립트는 다음 계약을 확인한다.

- `KU_ROLE=mission`에서 canonical 0301/0302/0303/0304 artifact builder의 `_sw_code()`가 모두 `MMR`을 반환한다.
- old absolute import와 bare `data_def.d030N` compatibility path가 canonical module object와 동일하며 같은 SW code를 반환한다.
- 0301 최소 output의 `Source`가 `MMR`이다.
- 0302 최소 output의 `Source`가 `MMR`이다.
- 0303/0304는 full route generation을 돌리지 않고 final payload assembly source에서 `("Source", _sw_code())` 계약을 고정한다.

## Verification

이번 항목을 완료 처리하기 전에 아래 명령을 실행했다.

- `python "docs\mission planning refactoring\smoke_sw_code_baseline.py"`
- `python -m py_compile "docs\mission planning refactoring\smoke_sw_code_baseline.py" "docs\mission planning refactoring\smoke_import_contract.py"`
- `python "docs\mission planning refactoring\smoke_import_contract.py"`
- `git diff --check -- "docs/mission planning refactoring"`
