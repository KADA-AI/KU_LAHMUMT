# Wrapper Template Consolidation Progress

## Scope

This checkpoint freezes the compatibility wrapper templates without changing import/export behavior.

## Added

- `smoke_wrapper_template_contract.py`

## Template Categories

- broad project-bootstrap wrapper: root and legacy wrappers that add the wrapper-side path, import a canonical module, copy non-dunder names, and mirror canonical `__all__`.
- broad package wrapper: moved `pipelines/*.py` files that re-export canonical trigger modules without path bootstrap.
- runtime-safe broad wrapper: old `runtime/*.py` wrappers that call `import_runtime_compat_module(..., __file__)` to survive bare imports from the runtime directory.
- explicit public wrapper: public entrypoint wrappers whose `__all__` is intentionally smaller than the canonical module.
- `sys.modules` alias wrapper: artifact and internal bare `data_def.*` module wrappers that should resolve as the canonical module object.
- special proxy wrapper: mutable-state wrappers such as `MissionPlanner/config.py` and `MissionPlanner/data_def/id_allocator.py`.
- package shim wrapper: internal `MissionPlanner` package bootstrap wrappers for `data_def` and `AnS`.

## Boundary

No runtime implementation changed. The smoke is source/AST-only and preserves the existing broad-vs-explicit export split documented in `10-wrapper-support-matrix.md`.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_wrapper_template_contract.py"
python "docs\mission planning refactoring\smoke_wrapper_template_contract.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
wrapper template contract smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: decide whether compatibility paths stay at root or move under `compat/`.
