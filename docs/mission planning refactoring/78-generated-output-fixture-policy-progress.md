# Generated Output Fixture Policy Progress

## Scope

This checkpoint decides whether checked-in generated output JSON should be kept as fixtures or deleted.

## Decision

fixture-hold.

Do not delete generated output JSON in this refactor phase.

## Rationale

- `manual/logic_test/division_test` and archived `legacy/tests/division_test` write `output/auto_0302`, `output/auto_0303`, and `output/auto_0304` as manual planner outputs.
- The checked-in JSON payloads currently act as manual/golden fixture candidate data for division planner behavior.
- Deleting them before a fixture owner and representative scenario are defined would remove regression evidence without improving runtime behavior.
- The earlier reachability and owner/manual workflow checkpoints explicitly avoid deletion approval.

## Fixture Buckets

| Bucket | Policy | Current count |
| --- | --- | --- |
| `manual/logic_test/division_test/output/auto_0302` | keep as fixture candidate | 3 JSON |
| `manual/logic_test/division_test/output/auto_0303` | keep as fixture candidate | 16 JSON |
| `legacy/tests/division_test/output/auto_0302` | keep as archive fixture candidate | 3 JSON |
| `legacy/tests/division_test/output/auto_0303` | keep as archive fixture candidate | 13 JSON |

## Boundary

No deletion is approved. No output files were rewritten. Future cleanup may move these into a dedicated fixture folder only after owner signoff and smoke coverage for the manual planner scenario.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_generated_output_fixture_policy.py"
python "docs\mission planning refactoring\smoke_generated_output_fixture_policy.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
generated output fixture policy smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: decide root wrapper deprecation period.
