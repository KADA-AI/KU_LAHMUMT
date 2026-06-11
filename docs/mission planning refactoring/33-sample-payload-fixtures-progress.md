# 33. Sample Payload Fixtures Progress

## Scope

이번 수정은 Phase 0의 sample 0201/0203/0902 payload fixture 확보 항목이다.

## Added Fixtures

- `docs/mission planning refactoring/fixtures/payloads/sample_0201.json`
- `docs/mission planning refactoring/fixtures/payloads/sample_0203.json`
- `docs/mission planning refactoring/fixtures/payloads/sample_0902.json`

## Added Script

- `docs/mission planning refactoring/smoke_sample_payload_fixtures.py`

## Fixture Boundary

fixture는 실제 로그 후보에서 필요한 구조만 축소한 순수 payload JSON이다. payload 안에 fixture metadata를 넣지 않았다.

0201 fixture는 다음 계약을 제공한다.

- `inputMissionPackageID=1`
- non-empty `availableAircraftList`
- non-empty `inputMissionList`
- `Source` 원문 값
- cache target directory: `InputMissionPlan`

0203 fixture는 다음 계약을 제공한다.

- `missionReferencePackageID=1`
- non-empty `takeOverInfoList`
- non-empty `handOverInfoList`
- non-empty `flightAreaList`
- lowercase `source` 원문 값
- cache target directory: `MissionReferenceInfo`

0902 fixture는 다음 계약을 제공한다.

- `optionList` 기반 selected `missionPlanID=700000001`
- `missionPlanIDList`와 `replanDetail.missionPlanID` fallback 후보를 같이 포함
- dict-only `inputMissionIDList`
- `replanDetail.trigger=0401`, `triggerType=communicationLossRTB`
- current delay policy baseline: default `55000` ms

## Verification

이번 항목을 완료 처리하기 전에 아래 명령을 실행했다.

- `python -m py_compile "docs\mission planning refactoring\smoke_sample_payload_fixtures.py"`
- `python "docs\mission planning refactoring\smoke_sample_payload_fixtures.py"`
- `python -m json.tool "docs\mission planning refactoring\fixtures\payloads\sample_0201.json"`
- `python -m json.tool "docs\mission planning refactoring\fixtures\payloads\sample_0203.json"`
- `python -m json.tool "docs\mission planning refactoring\fixtures\payloads\sample_0902.json"`

## Next TODO

다음 미완료 TODO는 generated 0301/0302/0303/0304 artifact link validation script 작성이다.
