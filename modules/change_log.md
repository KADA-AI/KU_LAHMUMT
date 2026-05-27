2026-05-21 v1.3.31 - 당일 통합 수정. 메인 대시보드에서 모듈 서비스 상태 문구 표시, 개발관리 버튼, Change Log 전용 표 창, Quick Actions 배치를 정리해 변경 이력 확인성을 보강함. 현재임무 재수행 재계획은 옵션 후보 병렬 실행을 적용하되 후보별 입력/FlightPath/결과 병합 경계를 분리해 정찰특화와 일반 옵션 결과가 꼬이지 않도록 보강하고 처리 시간을 줄임. 촬영 계획 고도 산정에서는 lineSearch 지형 최대고도 계산의 중복 DEM 샘플링을 제거하고 `d0303.py` 내부 DEM/groundRequired 캐시를 추가해 촘촘한 sweep에서도 coordinateList/FOV/SEP 데이터 개수는 유지한 채 고도 보정 계산량을 줄임. 모니터링 장시간 운용 중 `Qt5Core.dll` 종료 로그는 이벤트 로그와 모듈 로그 기준으로 원인을 추적하고 PyQt UI thread 접근/고주기 갱신 위험 지점을 점검함.
2026-05-14 v1.3.30 - 촬영 계획의 target/lineSearch/areaSearch 고도를 비행체 고도와 분리해 DEM 기준으로 재산정하고, 촬영점 고도가 WP 비행고도보다 높아지는 경우 WP 고도를 최소 안전 여유만큼 보정하도록 공통 guard를 추가함. 인제 지역은 `resource/Inje_10m.tif`를 EPSG:32652 지역 DEM으로 우선 사용하도록 DEM 선택 로직을 확장했으며, 선택된 DEM 파일명/좌표/고도 샘플이 `DSS_Internal/dem_usage.jsonl`에 남도록 진단 로그를 추가함. area 촬영계획의 DB 최대 SEP 대신 운용 파라미터로 촬영 segment 상한을 조정할 수 있도록 GUI/runtime 연결을 보강하고, 메인 화면 최근 업데이트 날짜를 26-05-14로 갱신함.
2026-05-11 v1.3.29 - 당일 통합 수정. 0501 currentInputMissionID 롤백 방지 및 heartbeat/queued 0501 stale payload 폐기 보강; 0803 현재임무 재수행 시 0201/InputMissionPlan 복제와 현재 협업기저임무 복제/삽입 처리 추가; 다음 협업기저임무 line 재계획의 촬영 준비점/진입 방향/현재 위치 기반 분할을 보정하고 편대비행 재계획 경로를 추가; 공격 후 복귀 재계획에서 복귀 UAV와 임무 완료 UAV의 마지막 hold/잔여 WP를 보존하고 후속 input mission 시각화가 사라지지 않도록 보강; 경로 미추종 재계획이 현재 input mission 밖의 다음 임무 WP를 잡지 않도록 currentInputMissionID 경계 검사를 추가; sim input 경계 waypoint에서 0803 없이 다음 협업기저임무로 자동 진행되지 않도록 boundary loiter 처리를 수정; post-attack 단일 잔여 WP의 searchSpeed를 현재 UAV-WP 거리 기반으로 재산정하고 배수 적용을 조정; 정찰특화/area 선회 연결이 임무계획 알고리즘 설정의 공통 Dubins 절대 선회반경을 따르도록 정리; line 진행영역 관리에서 순간적인 FOV/line 추적 끊김으로 남은 line이 뒤로 풀리지 않도록 provisional frontier sticky 보정을 추가; 0902 sidecar 저장을 고유 tmp/retry 방식으로 보강해 배속 시 다음협업기저임무 요청이 수신 후 처리 예약 전 누락되는 상황을 방지; 60 byte 표시 제한에 맞춰 주요 GUI/notice/재계획 사유 문구를 축약하고 다음 협업기저임무 전환 사유에 `_` prefix를 적용; 메인 화면 최근 업데이트 날짜를 26-05-11로 갱신함.
2026-04-29 v1.3.28 - 0401 기반 비가용 재계획을 보강하여 통신두절/무인기 고장/임무장비 고장이 발생 후 55초간 지속될 때 health 값과 별개로 비가용 처리 후 재계획하도록 정리하고, 동시 발생 시 재계획 사유 우선순위를 통신두절 > 무인기 고장 > 임무장비 고장 순으로 고정함. 기존 RTB 비가용 판단 및 재계획 경로는 유지하면서, 선행임무 인가/협업 잔여 재계획/타 UAV 복귀 갱신 후보 선정에서도 VehicleStatus 비가용 및 RTB UAV를 제외하도록 보강하고, 선행임무 요청/후보 선정이 스킵되는 경우 원인 로그를 남기도록 진단성을 보강함. 또한 current-remaining 재계획의 기존 잔여 임무 순서/ID 정합성을 맞추고, 0301 이후 0305/0901/0903 송신 전 대기와 ID 관계 탭 갱신 블로킹을 줄여 재계획 결과 송신 지연을 완화함.
2026-04-24 v1.3.27 - 공격 후 복귀 재계획에서 활성 UAV 평균 진행률이 70% 이상이라 협업 재합류를 스킵하는 경우에도 그냥 종료하지 않고, 현재 남은 임무만 기준으로 active UAV들의 IMP/FlightPath를 다시 생성해 지나온 prefix mission/path를 버린 채 남은 2대가 분담 수행하도록 보강함. 동시에 표적 자동추적 UAV는 tracking branch를 제거하고 마지막 복귀점으로 향하는 return-only path(58m/s)와 후속 mission만 남긴 새 IMP로 교체되며, closed tracking assignment도 함께 정상 종료되도록 정리함.
2026-04-06 v1.3.26 - 일반 0201 재계획(0401/0802 한정)의 current remaining hybrid 경로를 정리하여 monitoring 진행영역 snapshot으로 합쳐진 현재 남은 line/area 임무를 next-collab line/area planner로 재생성하고, 나머지 후속 임무는 기존 초기 임무 재계획 흐름(run_divide_and_pattern + 0303/0304) 그대로 유지하도록 보강함. 병렬/순차 일반 재계획 경로 모두 동일한 current-hybrid 적용 지점을 사용하도록 맞췄고, 0802 hold 만료 지연 재계획에도 currentRemainingCollaborativeReplan / entryAircraftList / turnRadiusScale 문맥이 붙도록 monitoring payload augmentation을 확장했으며, fresh 0201 입력 갱신형 재계획은 기존처럼 제외함.
2026-04-06 v1.3.25 - 다음 협업기저 임무 line 재계획을 area와 유사한 구조로 확장하여 선회 반영 entry/heading 기반 진입점 계산, line piece 재분할/재할당, 전용 line path/sweep 생성을 지원하도록 추가 보강함. corridor sweep 전개 방향과 route anchor를 기존 line scan mode에 맞게 다듬고, raw sweep spacing과 grouped lineSearch waypoint 병합 로직을 정리하여 여러 sweep이 하나의 WP에 묶이도록 수정했으며, GUI/runtime의 line density 및 route offset 배수가 해당 재계획 경로에도 직접 반영되도록 연결함.
2026-04-03 v1.3.24 - 임무계획 line 밀도 배수가 corridor sweep 간격에 직접 반영되도록 수정하고 route WP 간격과 분리함. line/area 밀도 배수 상한을 10.0으로 확대했으며, hold->entry 구간에서 search entry point 고도가 다음 sweep 고도로 급강하하던 문제에 상승/하강률 제한을 추가해 비현실적인 고도 점프를 막음. run.py / run_offline.bat 실행 차이로 섞이던 UI import 경로도 정리해 동일한 화면과 동작을 사용하도록 고정함.
2026-04-03 v1.3.23 - 임무 진행영역 관리의 소거 로직을 전면 개선함. 기존 개별 cut-line strip + bridge 방식에서 boundary cut line 기준 반평면 차감 방식으로 교체하여, 곡선 경로 구간에서 strip 간 빈틈(초승달 모양 미소거 영역)이 반복 발생하던 문제를 해소함. 이제 진행 방향 기준으로 boundary 뒤쪽 영역은 전부 소거 완료로 처리됨.
2026-04-02 v1.3.22 - 다음 협업기저임무의 FOV 해석을 수정해 PATH0 / assignment-path 행이 최종 `resolvedFovDeg`로 entry-T' SEP 휴리스틱 행을 재사용하지 않도록 함. 이제 파이프라인이 sweep builder와 동일한 width/SEP 호환 DB 기준으로 FOV/속도를 해석하므로, DB 기반 sweep planning이 활성화된 동안 1.7° 같은 작은 값으로 잘못 축소되지 않고 `fov_db.csv`의 큰 가용 FOV 값을 복원함.
2026-04-02 v1.3.21 - Done/resume sweep 분할이 이제 sweep trimming에만 추가 ~3초 lookahead를 적용함. `sweep_progress.json`에 명시적 `buffer_seconds`가 없을 때 임무계획 재계획 사용처(attack/prior/imaging)는 `elapsed_seconds`, `planned_seconds`, `sweep_point_count`, `seconds_per_point`로 버퍼된 sweep point를 추정하므로, mission ID, path ID, non-sweep branching logic은 바꾸지 않은 채 resume path가 몇 개의 sweep point 앞에서 잘리도록 보정함.
2026-04-02 v1.3.20 - Attack UAV resume trimming을 비공격 UAV resume split과 정렬함. 공격 재계획이 탐지 UAV에서 과도하게 크게 잘린 sweep을 보면, 이제 전체 미절단 branch를 복원하는 대신 `progress_points` 기반 trimming으로 fallback하여 추적 UAV가 attack-exclude/prior 재계획과 동일한 부분 trim sweep waypoint 구조로 재개하도록 함.
2026-04-02 v1.3.18 - 공격 임무 생성 전 과정의 실패 사유를 `0001` NoticeInfo로 통보하도록 보강함. 공격 특화 파이프라인과 일반 공격 옵션 적용 경로 모두에서 `가용 유인기 없음`, `잔여 탄약(type1~3) 부족`, `표적/공격기 좌표 누락`, `기준 MissionPlan 로드 실패`, `공격 산출물 생성 실패` 같은 케이스를 사람 읽을 수 있는 문구로 정리해 발송하며, 특히 잔여 탄약이 모두 부족한 경우 공격 임무를 생성하지 않고 바로 안내 메시지를 내보내도록 수정함.
2026-04-02 v1.3.17 - 공격 임무의 무기 선택에 실시간 0401 탄약 재고(`type1/type2/type3`)를 반영함. 표적 type으로 먼저 선호 `weaponType`을 정한 뒤, 선택된 공격기의 남은 탄약을 확인하여 동일한 우선순위(예: 유도탄 우선 시 `2→1→3`) 안에서 실제 발사 가능한 무기로 자동 보정하고, 적용된 재고/선택 결과를 공격 로그와 메타데이터에 함께 남기도록 보강함.
2026-04-02 v1.3.16 - 공격 임무가 `0402 targetType`과 GUI 설정을 기준으로 자동 무기를 선택하도록 확장함. 공격 표적 후보는 `표적 우선순위(target_type_priority)` 순서대로 선택하고, 선택된 표적의 `targetType`별 `weaponType` 매핑(`weapon_for_target_type_1~6`)을 적용하며, GUI에서 fallback 무기와 표적별 기본 무기를 함께 조정할 수 있도록 연결함. 기존처럼 명시적 0402 표적이 들어오면 그 표적을 우선 사용하고, targetType 정보가 없을 때만 fallback 무기를 사용함.
2026-04-02 v1.3.15 - 공격 임무의 `weaponType` 해석을 ICD 기준으로 고정해 `1=기관포, 2=유도탄, 3=로켓` 의미를 코드와 GUI에 반영하고, GUI 입력 범위와 runtime 정규화를 `0~3`으로 제한함. 현재 공격 임무는 여전히 전역 `weapon_type` 설정값을 사용하지만, ICD 범위를 벗어난 값은 저장/적용 단계에서 자동 보정되도록 정리함.
2026-04-02 v1.3.14 - 임무계획 알고리즘 파라미터에 `Line SEP 배수(line_route_offset_scale)`를 추가해 LINE/corridor 경로 waypoint의 sweep 대비 lateral offset을 GUI와 runtime 설정에서 직접 조정할 수 있도록 연결함. 기존 `Area SEP 배수(area_route_offset_scale)`는 그대로 유지하며, 두 값 모두 기본 1.0이라 값을 바꾸지 않으면 기존 동작을 유지함.
2026-04-02 v1.3.13 - 공간해상도(GSD) 만족 판정 기준을 기존 축별 GSD 비교(gsd_x ≤ req_gsd_x AND gsd_y ≤ req_gsd_y)에서 논문(JANT 2024) 기반 면적 GSD로 교체함. γ = (객체 가로 × 객체 세로) / (최소 픽셀 가로 × 최소 픽셀 세로) [m²/px], 허용 최대 풋프린트 면적 s_req = γ × (이미지 총 픽셀)로 계산하며, fp_w × fp_h ≤ s_req 일 때 만족으로 판정함. 차트 기준선은 √γ (등가 선형 GSD)로 표시하고, 폼에 실제 풋프린트 면적과 요구 면적을 함께 노출함.
2026-04-02 v1.3.12 - UAV 탐색 WP 고도 계산을 전면 개선함. ① 고도 기준을 기존 경로 샘플링 (min+max)/2에서 해당 WP의 coordinateList 3점(p1·중심·p2) DEM 정확 평균값으로 교체함. ② 이전 WP 고도에서 climb_rate×(거리/속도)를 초과하는 상승은 물리적으로 도달 불가 고도로 보고 캡을 적용하는 상승 제약을 추가함. ③ UAV 상승률(m/s) 파라미터를 algo_config_tab "WP/선회" 그룹에 GUI 항목으로 노출하고 uav_params.json에 저장되도록 연결함(기본값 5.0). ④ Area 임무에서 WP 고도를 pack 이전에 개별 WP별로 먼저 산정한 뒤, 그룹 내 최고 고도를 packed WP에 보존하도록 순서를 재조정해 실제 지형 기복에 따라 packed WP 간 고도가 오르내리도록 개선함.
2026-04-02 v1.3.11 - 촬영품질 모니터 탭에 공간해상도(GSD) 모니터링 섹션을 추가함. 0401 footprintCornerList 4점 좌표로 실제 촬영 폭·높이를 계산하고, 이미지 해상도(기본 1920×1080)로 나눠 GSD(m/px)를 산출한 뒤, 탐지 요구조건(객체 크기÷최소 픽셀)과 비교하여 만족/미달 여부 및 최근 300회 누적 만족률을 무인기별 카드와 시계열 차트로 표시함. 이미지 크기·객체 크기·최소 픽셀값은 UI SpinBox로 실시간 변경 가능함.
2026-04-02 v1.3.10 - 시뮬레이션이 협업기저임무 경계에서 0803 execute=1 수신 시 자동으로 다음 임무로 진행하도록 수정하고, 웹 UI의 직접 `/api/sim/next_mission` 호출을 제거해 UAV 전진과 모니터링 상태 전환이 동일한 0803 수신 시점에 동기화되도록 레이스 컨디션을 해소함.
2026-04-02 v1.3.09 - 공격 임무 재계획에서 source MissionPlan/IMP/FlightPath 중복 로딩과 done-input 스캔을 1회 캐시로 정리하고, 단계별 timing 로그를 추가해 로직/ID 체계 변경 없이 병목을 추적하고 처리 시간을 줄이도록 보강함.
2026-04-02 v1.3.09 - 임무계획 GUI의 `협업기저 재계획 테스트` 탭을 제거하고, 다음 협업기저임무 재계획의 T' 진입 기준을 최대 SEP의 단순 0.8 배수가 아니라 해당 비율 이하의 실제 FOV DB 행을 선택해 그 행의 SEP/FOV/속도를 함께 적용하도록 조정함.
2026-03-31 v1.3.08 - 다음 협업기저임무 인가 시 자동 재계획(WithoutPilotDecision)을 수행하도록 적용하고, 협업기저임무 진입 시 leg를 무인기의 실시간 위치 및 속도(벡터)를 고려하여 유동적으로 생성하도록 개선함.
2026-03-31 v1.3.07 - 임무 계획의 FOV 기준을 사람이 아닌 탱크 기준으로 변경하고, 탱크 발견 후 줌인 도중 사람을 발견한 경우에는 사람을 타게팅하도록 로직을 적용함.
2026-03-31 v1.3.06 - 유인기가 비가용(health==2) 상태일 때 경고 메시지를 남기는 로직을 추가하고, 해당 기체를 제외한 재계획이 자동 발동되도록 적용함.
2026-03-31 v1.3.05 - 선회모니터에서 raw heading 대신 좌표 기반 fallback이 우선 적용되어 헤딩이 순간적으로 90도 꺾이며 팅기던 문제를 수정함.
2026-03-31 v1.3.04 - 선행임무에 대한 FOV를 사람 기준에서 탱크 기준으로 변경함.
2026-03-31 v1.3.03 - 촬영품질 모니터 용어를 `촬영품질`로 통일하고, 품질 기준값을 `sep` 단독이 아닌 `sqrt(sep² + (width/2)²)` 대각선 maximum으로 재정의함.
2026-03-31 v1.3.02 - 협업기저임무 완료 처리에서 `0803 execute=1` 중복 반영을 방지하여, 완료 신호가 두 번 날아가며 다음 임무가 두 번 넘어간 것처럼 보이던 문제를 수정함.
2026-03-31 v1.3.01 - Area 임무에서 Sweep 중간점이 정상 반영되지 않던 문제를 수정하여, sweep 정렬·구간 병합·segment 방향 처리가 초기임무계획과 동일하게 동작하도록 보정함.
2026-03-26 v1.2.18 - SIM 적 표적 시각화를 전면 고급화해 전차/장갑차/방사포/곡사포/고정고사포/군인별 3D 형상을 서로 다르게 렌더링하고, 이동 속도 기반 trail·탐지 halo·사격 ring·피격 후 침묵한 잔존 표현까지 함께 표시하도록 개선함.
2026-03-26 v1.2.17 - SIM 적 표적 상태 직렬화에 heading·headingRate·실속도·속도범위·roam 반경·탐지 여부·노출 시간·탄약·무기 종류/사거리/재장전·마지막 발사 경과시간을 추가하고, 정지형 대공/포대는 실제 교전 대상 방향을 보도록 heading을 갱신해 시각 모델과 교전 방향이 어긋나지 않게 보강함.
2026-03-26 v1.2.16 - SIM 내부 nFusion 연결을 다시 `nFusionSettings.json`·`current_scenario.json` 기반으로 정리하고, `sim_main`은 `0.0.0.0`에 바인드한 뒤 충돌 없는 빈 포트를 자동 선택해 다른 PC에서도 안정적으로 접속되도록 복구함.
2026-03-26 v1.2.15 - SIM 모니터링 모드를 `RX 0401 @ 5Hz` 최신 프레임 기준 시각화로 정리하고, 실시간 임무 변경에도 재시각화가 이어지도록 하며 latest-only polling과 mission payload 최소화로 steady-state 부하를 크게 줄임.
2026-03-26 v1.2.14 - 비행 경로 시각화에 waypoint `Fly-by/Fly-over/Loiter` 유형 표시와 현재 추종 WP 강조를 추가하고, 0401/라이브 상태와 패널 표시를 맞춰 실제 추종 중인 임무 진행 상태를 더 직접적으로 읽을 수 있게 개선함.
2026-03-26 v1.2.13 - SIM 고속 재생 시 끊김을 줄이도록 history 처리·footprint/trail 갱신 주기를 최적화하고, DEM blank tile fallback·terrain/hillshade source 분리·geometry 경고 완화까지 묶어 지도 렌더링 안정성을 보강함.
2026-03-26 v1.2.12 - SIM 웹 UI를 dark 톤 기준으로 다시 정리해 좌상단 Visual Mode 카드, 시나리오 입력부, 상단 toolbar, flight-path legend, 우하단 맵 컨트롤의 색감과 계층을 통일하고 monitoring/agent panel 겹침까지 정리함.
2026-03-26 v1.2.11 - SIM 조작 디테일을 다듬어 좌우 패널 화살표 hover 튐을 제거하고, 적 배치 radial menu 간격을 재조정했으며, 3D building 농도도 소폭 올려 배경 지형 위에서 형태가 더 또렷하게 보이도록 보정함.
2026-03-26 v1.2.10 - 모니터링 시각화 구성을 재정비해 `실시간 위험도 예측`을 단독 탭으로 분리하고, 기존 시각화 탭은 임무 상태 카드·표 정렬을 다시 맞춰 더 넓고 균일하게 보이도록 재배치함.
2026-03-26 v1.2.9 - 스케줄 모니터 탭의 재계획 버튼을 모두 제거해 표시 전용으로 단순화하고, 관련 제어는 `임무 재계획 관리` 탭에서만 통합 관리하도록 정리함.
2026-03-26 v1.2.8 - `임무 재계획 관리` 탭을 신설해 경로추종·촬영품질·강제대기·입력갱신·선행임무·DL 위험·촬영계획·다음협업·RTB·표적탐지·연료 등의 ON/OFF와 세부 파라미터를 한곳에서 조정하도록 연결하고, 탭 순서도 `모니터링 CSC` 바로 뒤로 재배치함.
2026-03-26 v1.2.7 - 재계획 설정의 현재값과 권장값을 JSON으로 분리 저장하고, 재시작 후에도 마지막 설정을 복원하며 권장값 복원/저장까지 GUI에서 직접 수행할 수 있도록 구성함.
2026-03-26 v1.2.6 - 재계획 관리 탭 UI를 전체 스크롤 카드형 레이아웃으로 재구성하고, 마우스 휠 오입력 방지·스핀 화살표 아이콘·정렬·색감·용어(`선행임무`)를 다듬어 다른 모니터링 화면과 통일감을 맞춤.
2026-03-26 v1.2.5 - 경로 미추종 재계획 기준을 runtime 설정으로 외부화하고, 선회율·경고각·누적시간·대체 WP 생성 지연 등 숨은 파라미터까지 GUI에 노출해 더 타이트하게 조정할 수 있도록 보강함.
2026-03-26 v1.2.4 - `0401 선회 모니터`를 `경로추종 모니터링`으로 개편하고, 밝은 톤 UI와 HUD 정렬을 정비했으며 현재 WP의 `Fly By/Fly Over/Loiter` 타입을 카드와 지도에서 직접 확인할 수 있도록 개선함.
2026-03-26 v1.2.3 - `0401` heading 판정에서 raw heading을 우선 사용하고 좌표 기반 fallback을 보수화했으며, 중복 `0401` 처리도 정리해 순간적인 90도 꺾임 표시와 선회 재계획 누락 가능성을 줄임.
2026-03-26 v1.2.2 - 촬영품질 모니터 용어를 `촬영품질`로 통일하고, 품질 기준값을 `sep` 단독이 아닌 `sqrt(sep^2 + (width/2)^2)`로 재정의했으며 이때 width는 `0401 sensorInfo.fov`에 해당하는 DB 구간의 최대 width를 사용하도록 보정함.
2026-03-26 v1.2.1 - 협업기저임무 완료 처리에서 `0803 execute=1` 중복 반영을 막고 transient `100% -> 하락 -> 100%` 구간이 있더라도 다음 임무가 두 번 넘어간 것처럼 보이지 않도록 mission progress 연계 처리를 보강함.
2026-03-21 v1.1.33 - 협업기저·선행임무 재계획의 LINE/corridor 생성에서 sweep 정렬·구간 병합·segment 방향 처리를 보정해, 초기임무계획처럼 여러 sweep가 한 waypoint에 정상 묶이도록 수정함.
2026-03-21 v1.1.33 - non-area 재계획 경로의 lineSearch 순서 뒤집힘과 잘못된 POINT 변환을 정리해, 끝까지 갔다가 다시 돌아오는 경로와 마지막 sweep 유실 문제를 보정함.
2026-03-21 v1.1.33 - 모니터링의 `다음 협업기저임무 수행` 명령이 즉시 다음 input mission을 active 대상으로 전환하도록 수정하고, 다음 임무가 없을 때는 `0001`로 `모니터링 모듈: 다음 협업기저임무가 없습니다.`를 통지하도록 보완함.
2026-03-21 v1.1.33 - 다음 협업기저임무 재계획 payload가 없을 때도 `execute=1` 일반 처리로 정상 fallback 되도록 보정하고, 해당 기능의 기본값은 다시 OFF로 정리함.
2026-03-21 v1.1.33 - `정찰/시간 균형` 재계획이 현재 초기임무계획 세팅을 그대로 따르도록 정리하고, 초기임무계획 알고리즘 설정 GUI는 실제 연결된 항목만 남긴 구조로 재정비함.
2026-03-21 v1.1.33 - Line/Area FOV Auto·Custom, Area FOV 배수, Area SEP 배수, Area 나누는 폭, UAV/LAH WP 간격, Dubins 선회반경을 GUI에서 조정하고 runtime에 직접 반영하도록 연결함.
2026-03-21 v1.1.33 - 재계획·이어붙이기 경로의 waypointID 재사용을 제거해 유인기·무인기를 포함한 전체 waypointID가 전역 유일하게 증가하도록 보정함.
2026-03-21 v1.1.33 - 유인헬기 공격 재계획에서 비선정 LAH의 hold/resume 진입점 처리와 현재 waypoint 제거를 정리하고, 공격 분석 반경은 2km로 축소했으며 `onMission=2` UAV는 경로 미추종 재계획에서 제외함.
2026-03-21 v1.1.33 - `modules/monitoring`의 PyInstaller build 산출물, `__pycache__`, 백업 파일 등 기능과 무관한 생성물을 정리해 용량을 축소함.
2026-03-20 v1.1.32 - SIM 웹에 `filmingProperty.lineSearch.coordinateList` 기반 실제 sweep 선/점 토글을 추가하고 얇은 점선 스타일로 조정했으며, 파괴된 표적은 `targetInfo.isDestroyed` 상태를 기준으로 자동추적/재추적하지 않도록 보정함.
2026-03-20 v1.1.32 - 좌표 임무 기본 FOV를 31.2도로 조정하고, AREA 임무도 실제 폭과 `enhanced_area_review_max_segment_m=500m` 기준으로 DB FOV/SEP를 선택하도록 확장했으며, AREA 수색 `searchSpeed` 가중치는 1.2로 상향함.
2026-03-20 v1.1.32 - AREA 경로계획에서 첫 chunk waypoint를 첫 실제 sweep line 기준 SEP 이격으로 재배치하고, 시작 anchor의 진행방향 lead-in 및 고도를 첫 chunk와 맞추며, 연속 AREA 임무는 offset 방향이 번갈아 적용되도록 정리함.
2026-03-20 v1.1.32 - AREA 간 Dubins flyover link는 옵션으로 켜고 끌 수 있게 정비하고, 가까운 transition link 및 중복 entry anchor는 자동으로 생략하며, 다음 AREA 시작 side 판단은 이전 임무의 마지막 waypoint 좌표 기준으로 보정함.
2026-03-20 v1.1.32 - 공격 재계획 시 유인헬기(LAH)는 완료된 done 개별임무/path를 별도로 보존하지 않고 `attack 또는 hold + resume + follow-up`만 남기도록 재구성했으며, Dubins turn link GUI의 import 경로도 직접 실행 가능하게 수정함.
2026-03-19 v1.1.31 - 다음 협업기저임무 재계획이 `WaypointID pool exhausted`로 실패하던 문제를 수정하고, 0303/0304 local waypoint allocator가 현재 906xxx대 waypoint ID 체계에서도 정상 동작하도록 65,535 상한 체크를 제거함.
2026-03-19 v1.1.30 - 모니터링 mission progress tracker가 같은 UAV의 mission sequence에서 더 뒤 individual mission으로 점프할 때 inputMission 경계를 넘더라도 중간에 건너뛴 individual mission과 input mission을 완료 처리하도록 보정함.
2026-03-19 v1.1.24 - 모니터링 mission tracker snapshot에 aircraft별 실제 current individual mission과 active input mission을 노출하고, 시각화 탭·스케줄 탭·0501 payload가 DB의 next pending mission 대신 0401/currentWP 기반 현재 수행 임무를 따라가도록 보정함.
2026-03-19 v1.1.23 - 경로 미추종 재계획 0902 사유 문구의 고정 placeholder `000 무인기`를 제거하고, aircraftID 4/5/6을 실제 `무인기 1/2/3번`으로 매핑해 replanRequest/replanReason/replanDetail.selectedReplanReason에 반영하도록 수정함.
2026-03-19 v1.1.22 - UAV 0303 search waypoint 간격 기준을 800m로 조정하고, 각 search waypoint 고도를 해당 구간 지형의 최고/최저 고도 중간값에 UAV별 고도층 610/620/630m를 더한 값으로 재산정하도록 보정함.
2026-03-19 v1.1.21 - LAH-UAV ETA 기반 속도조정 후 최종 speed profile은 LAH 1번을 기준으로 다른 LAH에 동기화하고, LAH 1번이 없으면 LAH 2번을 기준으로 따르도록 보정해 편대가 같은 속도 프로파일로 움직이게 수정함.
2026-03-19 v1.1.20 - SIM spawn 좌표를 takeOverInfo보다 FlightPath 첫 waypoint 기준으로 우선하도록 바꿔, 유인기를 포함한 기체가 시작 시 첫 waypoint 위치와 고도로 바로 배치되게 수정함.
2026-03-19 v1.1.19 - LAH-UAV ETA follow speed에서 UAV `eta`는 초, LAH `eta`는 밀리초로 저장되던 단위 차이와 aircraft 단위 단일 path 참조 문제를 수정해, 같은 짝의 UAV path 중 시작/끝 좌표가 가장 가까운 경로를 기준으로 LAH waypoint speed가 실제로 재계산되도록 보정함.
2026-03-19 v1.1.18 - mission_planning_gui의 실제 FlightPath 생성 경로 두 곳에도 LAH-UAV ETA follow speed 후처리를 연결해, 실행 계획에서 LAH waypoint speed가 기본값 15.0으로만 남지 않고 paired UAV ETA 기준으로 다시 계산되도록 수정함.
2026-03-19 v1.1.17 - LAH 0304 생성 후 paired UAV(1↔4, 2↔5, 3↔6)의 cumulative ETA를 기준으로 UAV 예상 위치를 보간해, LAH 경로 좌표는 유지한 채 waypoint speed/eta만 후처리하여 UAV보다 먼저 가지 않으면서도 과도하게 뒤처지지 않도록 추종 속도 계획을 추가함.
2026-03-19 v1.1.16 - LAH 경로에서 fallback 직선 분할뿐 아니라 정상 route planner 결과도 500m 간격으로 재샘플링해 긴 leg가 남지 않도록 보정함.
2026-03-19 v1.1.15 - LINE/corridor 수색에서 500m 간격 bundle waypoint를 만들 때 SEP offset anchor(`offset_wps`/`orange_pts`) 좌표를 실제 waypoint로 사용하도록 수정해 첫 점뿐 아니라 이후 bundle waypoint도 lateral offset을 유지하게 정리함.
2026-03-19 v1.1.14 - 촬영품질 확인 탭 기본값을 모니터링 ON, 재계획 판단 OFF로 분리해 그래프와 상태 확인은 유지하면서 자동 품질 0902는 기본 비활성으로 변경함.
2026-03-19 v1.1.13 - LINE/AREA 수색 경로 waypoint를 약 500m 간격으로 재구성해 한 waypoint가 과도한 sweep 묶음을 들고 가지 않도록 조정함.
2026-03-19 v1.1.12 - 촬영품질 모니터 자동 재계획 기본값을 OFF로 변경하고 기본 분기를 보수적으로 조정해 품질 0902가 기본 발동하지 않도록 수정함.
2026-03-19 v1.1.11 - 품질 재계획 적용 후 FlightPath 앞부분의 `isDone=true` waypoint를 SIM controller가 다시 따라가지 않도록 첫 미완료 waypoint부터 재개하게 수정함.
2026-03-19 v1.1.10 - 촬영품질 기반 자동 재계획에 MissionPlan 적용 직후 10초 grace를 추가하고 해당 구간은 샘플을 누적하지 않도록 조정함.
2026-03-19 v1.1.9 - 0902 CLR 전송에서 `missionPlanIDList`와 `replanDetail` 같은 custom field 누락을 sidecar transport store로 보완해 품질개선 direct 파이프라인이 plan ID/detail 없이 깨지지 않도록 복구함.
2026-03-19 v1.1.8 - 품질개선 재계획을 선행임무/다음임무와 같은 direct 처리로 고정하고 0901/0701 옵션 UI가 생성되지 않도록 수신·전달 단계의 차단 조건을 보강함.
2026-03-19 v1.1.7 - 품질개선 0902 수신 시 `missionPlanIDList`가 비어 있어도 `replanDetail.missionPlanID`로 plan ID를 복구하고, `force_direct_update`와 `suppress_0702_fallback`을 강제해 옵션 경로로 빠지지 않도록 보완함.
2026-03-19 v1.1.6 - 0401 기반 촬영품질 평균으로 UAV별 `searchSpeed`를 ±10% 보정하는 품질개선 재계획을 추가하고, `flightMode=8`에서는 비활성화하며 옵션 없는 `0902 -> direct 0301/0305/0903` 경로로 처리하도록 구성함.
2026-03-19 v1.1.6 - 품질개선 재계획에 현재 UAV 위치 anchor, 기존 sweep 진행분 보존, 남은 sweep trim과 speed scale 반영을 추가하고 사유를 `UAV N 촬영 품질 개선`으로 기록하도록 정리함.
2026-03-19 v1.1.5 - 모니터링에 촬영품질 확인 탭을 추가하고, 0401 기반 UAV SEP 추이·기준 SEP·유인기-무인기 거리 그래프를 실시간 시각화하도록 구성함.
2026-03-19 v1.1.4 - 0401의 payload 고장과 LAH-UAV datalink 두절은 0902를 보내지 않고 0001 고장/통신두절 통지만 보내도록 정리했으며, 연료 경고는 0504만 유지하도록 정비함.
2026-03-19 v1.1.4 - UAV `health==2`는 즉시 비가용으로 판단해 해당 기체를 제외한 0902를 발동하고, 초기 임무계획은 `InputMissionPlan/100`의 `availableAircraftList`를 기준으로 가용 기체를 구성하도록 수정함.
2026-03-19 v1.1.4 - 0001 통지 문구를 유인기/무인기 번호 기반으로 정리하고, 통신두절은 LAH-UAV pair별로 `유인기 N - 무인기 N 통신두절` 형태로 출력하도록 변경함.
2026-03-19 v1.1.4 - 모니터링 GUI의 기체 상태/0401 신호 상태 갱신 충돌을 분리하고, 신호 판정은 GUI 수신 시각 기준으로 바꿔 상태 깜빡임을 줄이도록 보정함.
2026-03-19 v1.1.4 - 새 0201에서 `isDone=false`인 임무의 `inputMissionType=0`이 line/area 도형이면 자동 보정하고, 0001 임무 type 이상 경고만 띄운 채 임무계획은 계속 진행하도록 처리함.
2026-03-19 v1.1.4 - 공격배제로 처리된 same `targetID`도 선행임무(표적추적/좌표지향)로 다시 발견하면 rediscovery 상태를 기록해 `isUsed/isIgnored`를 다시 0으로 내리고, 실제 0902가 생성되는 시점에만 다시 consume하도록 수정함.
2026-03-17 v1.1.3 - Dubins Mode preset/UAV plan mode를 추가하고 division test 및 LINE/AREA 경로 생성, 시작점/진입점 처리, sweep 표시를 정비함.
2026-03-17 v1.1.3 - UAV별 take-over 기준 분할/스케줄링, RTB 재계획 지연·우선순위·팝업, LAH datalink 기반 통신두절 판정과 resume sweep/search speed 보정을 반영함.
2026-03-17 v1.1.3 - SIM에 SEP 표시와 초과 경고를 추가하고 공격배제 시 추적을 종료한 뒤 새 임무 수행으로 복귀하도록 수정했으며 공격 임무 계산 성능을 개선함.
2026-03-16 v1.1.2 - 0201 InputMission에 regionType 필드를 반영하고 common generator/push/receive 직렬화를 갱신함.
2026-03-16 v1.1.2 - 0104 공격통제모듈 상태정보의 common generator/push/receive를 추가하고 모든 모듈 수신 및 정보관리 송수신을 허용함.
2026-03-16 v1.1.1 - change_log.md 파일을 생성하고 변경 이력을 한 줄 요약 형식으로 누적하기 시작함.
2026-03-16 v1.1.1 - SIM 모듈 기본 서버 호스트를 192.168.100.251에서 127.0.0.1로 변경함.
2026-03-19 v1.1.25 - AREA 임무 entry waypoint가 측면 reference 선택이 아니라, 최종 area 경로의 첫 번째 leg 방향을 기준으로 뒤로 entry_offset만큼 직선 offset 되도록 보정함.
2026-03-19 v1.1.26 - SIM mission preserve-state 재로딩 시 path가 바뀌지 않은 비재계획 기체의 controller curr_idx/block/loiter 및 input mission index를 유지해, 경로 미추종 재계획 때 다른 UAV가 새 plan 첫 상태로 다시 해석되며 다음 임무로 넘어간 것처럼 보이던 문제를 보정함.
2026-03-19 v1.1.28 - AREA 임무의 기본 0303 경로와 다음 협업기저임무 전용 재계획 경로에서 보조 entry waypoint를 생성하지 않도록 수정해, AREA 경로가 첫 search waypoint부터 바로 시작되게 보정함.
2026-03-19 v1.1.29 - 다음 협업기저임무 전용 재계획에서 AREA local area-review가 `missionReferencePackageID=0`일 때 `0.json`을 읽지 못해 `missing_takeover`로 최대 폭 분할이 스킵되던 문제를 수정하고, takeover 정보가 비어 있으면 `entryAircraftList` 좌표를 `takeOverInfoList` fallback으로 주입해 AREA 세분화가 계속 적용되게 보정함.
2026-04-02 v1.3.19 - 표적 탐지 재계획 후 sweep-progress trim이 남은 waypoint를 전부 제거하게 되는 경우에도 Attack UAV tracking/resume split이 resume branch를 보존하도록 수정해, 탐지한 UAV가 다음 임무로 바로 건너뛰는 문제를 방지함.
