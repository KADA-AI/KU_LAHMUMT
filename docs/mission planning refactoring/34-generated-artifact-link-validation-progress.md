# Generated 0301/0302/0303/0304 Artifact Link Validation

## 목적

0301 MissionPlan, 0302 IndividualMissionPlan, 0303/0304 FlightPath 생성 산출물을 새로 만들지 않고, 이미 DB에 생성된 파일들의 링크 계약이 유지되는지 확인하는 smoke를 추가했다.

## 추가 파일

- `smoke_generated_artifact_links.py`

## 검증 범위

- `settings/current_scenario.json`의 `db_root`를 읽기 전용으로 해석한다.
- MissionPlan 파일 단위 closure만 검사한다. 전체 DB 폴더를 전역 검사하지 않아 과거 plan artifact 때문에 생기는 duplicate false-fail을 피한다.
- `MissionPlan/{missionPlanID}.json` 파일명과 본문 `missionPlanID`를 비교한다.
- `aircraftList[*].aircraftID`와 `individualMissionPackageID` 계열 링크를 검사한다.
- IMP 파일 존재, 본문 package ID, IMP `aircraftID`, `individualMissionList`를 검사한다.
- 각 individual mission의 `individualMissionID`, `pathID`, aircraft별 pathID band를 검사한다.
- FlightPath 파일 존재, 본문 `pathID`, `aircraftID`, 선택적 `individualMissionID` 링크를 검사한다.
- waypoint list는 `waypointList`, `uavWaypointList`, `lahWaypointList`를 모두 인정한다.
- waypoint의 `waypointID` 중복과 nonzero `nextWaypointID`의 로컬 참조를 검사한다.

## 호환 키

- MissionPlan -> IMP 링크: `individualMissionPackageID`, `individualMissionPlanPackageID`, `individualMissionPackageId`
- IMP package ID: `individualMissionPackageID`, `individualMissionPlanPackageID`
- FlightPath waypoint list: `waypointList`, `uavWaypointList`, `lahWaypointList`

## 검증 결과

명령:

```powershell
python "docs\mission planning refactoring\smoke_generated_artifact_links.py" --require-waypoints
```

결과:

```text
generated artifact link smoke ok: db_root=C:\Users\LAHMUMT_2\Desktop\DSS_KU\Logs\Scenario_2026-06-04T200605\SBC3 missionPlans=6 aircraft=36 individualMissionPackages=36 individualMissions=181 flightPaths=181 waypoints=539
```

## 중단 지점

이번 수정은 generated artifact link validation smoke 추가까지 완료했다. 다음 미완료 TODO는 `launcher/control contract inventory 작성`이다.
