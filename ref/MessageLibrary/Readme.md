# 메시지 ID

0000 ~ 0904 : 단위과제#2 내부 메시지

17000 : ACK

~~60000 ~ 69999 : ACS 외부1 인터페이스~~

51200 ~ 53200 : ACS 외부2 인터페이스

---
### 수정 내역

### V1.07

중복되는 nftype은 CommonType 폴더로 이동 (ex. Coordinate, Area)

---
### 과거 수정 내역

<details>
<summary>5월 9일</summary>

### 5월 9일 수정내역
0201 AircraftID 오타 수정, LowerLimit 오타 수정

0203 AltitudeLimits 수정

0301 AircraftID 오타 수정

0303 RightLimit 오타 수정

0305 MissionPlanningStatus 오타 수정

0401 FootprintCornerList 오타 수정
</details>

<details>
<summary>V1.0</summary>

### 5월 21일 수정내역 (V1.0)

공통: ulong timestamp 추가

0102 필드 추가: SourceModuleName

0201

Float -> Uint : InputMissionPackageID, InputMissionID

Float -> Ulong : InputTimestamp

0203 필드명 변경: Timestamp(Float) -> InputTimestamp(Ulong)

0301 필드명 변경 : AircraftIDList -> AircraftList, Timestamp(Float) -> MissionPlanTimestamp(Ulong)

0302

String -> Uint : IndividualMissionPackageID, IndividualMissionID, InputMissionID, PriorMissionID, TargetID, IndividualMissionID

Float -> int : Altitude

0303 필드명 변경 : FormationList -> FormationDistanceList

Float -> Uint : ETA, Time, TargetID

Float -> int : Radius

0304

Float -> Uint : ETA, Time, TargetID

Float -> int : Radius

0402

String -> Uint : TargetID

0501

Float -> Uint : IndividualMissionID, MissionPlanID, InputMissionID, InputMissionPackageID, PriorMissionID, CurrentMissionID

0502

Float -> Ulong : Timestamp

0503 필드명 변경 : Timestamp(Float) -> EndTime(Ulong)

0602

String -> Uint : TargetID

0701

Timestamp(Float) -> MissionPlanStartTime (Ulong)

String -> Uint : MissionPlanID

Float -> Uint : Time

0702

String -> Uint : MissionPlanID

0801

Float -> Uint : OperatorReplanRequestTime

String -> Uint : InputMissionPackageID, MissionReferencePackageID

0802

String -> Uint : IndividualMissionID, InputMissionID, PriorMissionID

0804

String -> Uint : InputMissionID

0901

Float -> Uint : RequestTime

String -> Uint : OptionID

0902 필드명 변경 : Timestamp(Float) -> ReplanRequestTimestamp(Ulong)

String -> Uint
OptionID, MissionPlanID, PriorMissionID, IndividualMissionID, InputMissionID

0903

String -> Uint : MissionPlanID
</details>

<details>
<summary>V1.01</summary>

### 5월 22일 수정내역 (V1.01)
메시지 소문자로 수정 : 0203, 0501, 0502

0303 FormationList -> FormationDistanceList

0701 : 오타수정

Timestamp 추가 : 0202, 0402

0202 : 필드 타입 변경 TargetID (String -> Uint)

</details>

<details>
<summary>V1.02</summary>

### 5월 23일 수정내역 (V1.02)
내부 ICD 변경에 따른 nFusion 메시지 수정
메시지 수정 : 0201, 0401, 0701, 0802


---
해당 브랜치는 06/19에 Main으로 Merge예정입니다.
Merge할 때, sln파일 제외, Readme 수정

</details>

<details>
<summary>V1.03 ~ 1.06</summary>

### (V1.03 ~ 1.06)

ID가 String -> Uint로 변경
일부 필드 삭제

V1.05

내용이 많은 메시지는 DB로 관리 (0201,0203, 0301 ~ 0304)

일부 메시지 외부 메시지에 맞취서 구조 변경

V1.06

모든 메시지에 Source 추가

</details>

---
V1.07
</details>

<details>
<summary>V1.03 ~ 1.06</summary>

내용 작성
1. 필드 설명 추가
2. 0504 메시지 추가
3. 0804 메시지 추가
4. 0904 필드 변경
5. 0304 HoveringTime -> Ulong으로 변경

</details>