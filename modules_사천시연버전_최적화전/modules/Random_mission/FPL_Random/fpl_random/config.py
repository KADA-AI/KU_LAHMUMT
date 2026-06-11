# -*- coding: utf-8 -*-
from __future__ import annotations

"""fpl_random 공통 설정."""

from pathlib import Path

# 00 자동 임무 생성 구역 -------------------------
AUTO_MISSION_AREA_SW = (38.0370740, 127.2060580)  # 남서(SW) 좌표
AUTO_MISSION_AREA_NE = (38.2204690, 127.4299630)  # 북동(NE) 좌표


# 02 시작 참조점(지포리 내 TakeOver 기준) -------------------------
START_REFERENCE_POINTS_RAW = (
    (38.0450000, 127.2120000),
    (38.0900000, 127.2120000),
    (38.1400000, 127.2120000),
    (38.1900000, 127.2120000),
    (38.0450000, 127.4240000),
    (38.0900000, 127.4240000),
    (38.1400000, 127.4240000),
    (38.1900000, 127.4240000),
    (38.0420000, 127.2300000),
    (38.0420000, 127.3200000),
    (38.0420000, 127.4100000),
    (38.1920000, 127.2300000),
    (38.1920000, 127.3200000),
    (38.1920000, 127.4100000),
)

# 03 FlightReferenceInfo 생성 파라미터 -------------------------
UAV_IDS = (4, 5, 6)  # TakeOver/HandOver/RTB에 사용하는 UAV ID
SIDE_M = 150.0  # TakeOver 정삼각형 변 길이 (m)
HANDOVER_OFFSET_M = 300.0  # HandOver 오프셋 (m)
RTB_OFFSET_M = 300.0  # RTB 오프셋 (m)
PROHIBITED_OFFSET_MIN = 100.0  # 금지구역 중심 최소 오프셋 (m)
PROHIBITED_OFFSET_MAX = 200.0  # 금지구역 중심 최대 오프셋 (m)
PROHIBITED_RADIUS_M = 500.0  # 금지구역 반경 (m)
PROHIBITED_AREA_COUNT = 2  # 금지구역 개수
TAKEOVER_ALT_AGL_M = 600.0  # TakeOver/HandOver/RTB AGL 고도 (m)
TARGET_ALT_AGL_M = 20.0  # Target AGL 고도 (m)

# 04 InputMissionPlan 개수/타입 -------------------------
INPUT_MISSION_COUNT_RANGE = (3, 10)  # 전체 미션 개수 범위
LINE_MISSION_COUNT_RANGE = (1, 4)  # 라인 미션 개수 범위
LINE_MISSION_TYPE_WEIGHTS = {1: 0.7, 4: 0.15, 5: 0.15}  # 라인 미션 타입 가중치
POINT_MISSION_TYPE_WEIGHTS = {2: 0.4, 3: 0.2, 6: 0.2, 4: 0.1, 5: 0.1}  # 포인트/면 미션 타입 가중치

# 05 라인 미션 파라미터 -------------------------
LINE_SEGMENT_MIN_M = 4000.0  # 라인 미션 길이 최소 (m)
LINE_SEGMENT_MAX_M = 9000.0  # 라인 미션 길이 최대 (m)
LINE_POINT_COUNT_RANGE = (1, 10)  # 라인 좌표 개수 범위
LINE_ALT_MIN_M = 600.0  # 라인 고도 최소 (m)
LINE_ALT_MAX_M = 1500.0  # 라인 고도 최대 (m)

PER_AIRCRAFT_WIDTH_MIN_M = 200.0  # 기체별 라인 폭 최소 (m)
PER_AIRCRAFT_WIDTH_MAX_M = 500.0  # 기체별 라인 폭 최대 (m)
WIDTH_STEP_M = 50.0  # 라인 폭 증가 단위 (m)

# 06 면 미션 파라미터 -------------------------
AREA_SIDE_MIN_M = 4000.0  # 면 미션 변 길이 최소 (m)
AREA_SIDE_MAX_M = 8000.0  # 면 미션 변 길이 최대 (m)
AREA_SIDE_STEP_M = 500.0  # 면 미션 변 길이 증가 단위 (m)
AREA_PENTAGON_RATIO = 0.3  # 면 미션 오각형 비율 (0~1)

# 07 간격/경계/시도 횟수 -------------------------
EDGE_GAP_MIN_M = 2000.0  # 미션 간 최소 간격 (m)
EDGE_GAP_MAX_M = 10000.0  # 미션 간 최대 간격 (m)

BORDER_MARGIN_M = 400.0  # 생성 구역 경계 여유 (m)
MAX_GEN_ATTEMPTS = 100  # 전체 생성 재시도 횟수
GENERATION_RETRY_MAX = 3  # 생성 실패 시 자동 재시도 횟수
SEGMENT_ATTEMPTS = 100  # 라인 생성 재시도 횟수
RECT_ATTEMPTS = 120  # 면 생성 재시도 횟수

# 08 헤딩/정렬 제한 -------------------------
HEADING_DELTA_MAX_DEG = 60.0  # 라인 세그먼트 헤딩 변화 최대 (deg)
CONTINUE_HEADING_MAX_DEG = 60.0  # 연속 헤딩 허용 최대 (deg)
FORWARD_ALIGN_DEG = 30.0  # 면 배치 전방 정렬 허용 (deg)

# 09 시작점 오프셋 -------------------------
START_OFFSET_MIN_M = 500.0  # TakeOver 기준 시작점 최소 오프셋 (m)
START_OFFSET_MAX_M = 2000.0  # TakeOver 기준 시작점 최대 오프셋 (m)

# 09B 생성 UI 기본값/재시도 -------------------------
GENERATION_DEFAULT_COUNT = 1  # GUI 기본 생성 개수
GENERATION_DEFAULT_MAX_OFFSET_M = START_OFFSET_MAX_M  # GUI 기본 시작 오프셋 상한
GENERATION_DEFAULT_TARGET_TYPES = (1, 2)  # GUI 기본 타겟 타입 풀
GENERATION_BATCH_ATTEMPTS_PER_MISSION = 3  # 배치 생성 시 미션당 추가 시도 횟수

# 10 센서/기체 목록 -------------------------
MAIN_SENSOR_WEIGHTS = {1: 0.8, 2: 0.2}  # MainSensor 가중치 (1=EO, 2=IR)
AIRCRAFT_IDS = (1, 2, 3, 4, 5, 6)  # AvailableAircraftList ID 목록
SCENARIO_DETECT_PIXEL = 10000.0  # Scenario UnitObjectList DetectPixel 기본값
SCENARIO_RECOG_PIXEL = 10.0  # Scenario UnitObjectList RecogPixel 기본값

# 11 Target 생성 파라미터 -------------------------
TARGET_COUNT_RATIO_RANGE = (0.2, 0.333333)  # 전체 대비 타겟 비율 범위
TARGET_MIN_SEP_M = 300.0  # 타겟 최소 간격 (m)
LINE_LATERAL_OFFSET_M = 120.0  # 라인 측면 오프셋 (m)
TAKEOVER_CLEARANCE_M = 2500.0  # TakeOver로부터 타겟 최소 거리 (m)
TARGET_PATH_RADIUS_M = 250.0  # 타겟 경로 반경 (m)
TARGET_PATH_POINTS_RANGE = (3, 5)  # 타겟 경로 점 개수 범위
MOVING_TARGET_TYPES = (1, 2, 6)  # 이동 경로를 생성하는 타겟 타입
MANEUVER_TANK_RANGE = (0, 3)  # 기동(라인) 탱크 개수 범위
TARGET_PLACEMENT_ATTEMPTS = 120  # 타겟 배치 재시도 횟수

# 11B Target Random(지형 분석) 생성 -------------------------
TARGET_RANDOM_COUNT_RANGE = (4, 8)  # Target Random 타겟 개수 범위
TARGET_RANDOM_TYPE_POOL = (5,)  # 지형 분석 기반 Target Random 타입 풀
TARGET_RANDOM_DEM_FILE = None  # None이면 DEM_CORRIDOR_FILE 사용
TARGET_RANDOM_DEM_MARGIN_M = 3000.0  # 임무영역 주변 DEM 분석 여유 폭 (m)
TARGET_RANDOM_REFERENCE_POINTS = 3  # 지형 분석 기준점 개수
TARGET_RANDOM_REFERENCE_ATTEMPTS = 12  # 지형 분석 기준점 샘플링 시도 횟수
TARGET_RANDOM_SAM_CANDIDATE_LIMIT = 50  # 기준점당 사용할 SAM 후보 최대 개수
TARGET_RANDOM_RADAR_CANDIDATE_LIMIT = 20  # 기준점당 사용할 RADAR 후보 최대 개수
TARGET_RANDOM_INCLUDE_RADAR_CANDIDATES = False  # RADAR 후보를 타겟 풀에 포함할지 여부

# 12 DEM 통로 기반 라인 -------------------------
DEM_CORRIDOR_ENABLE = True  # DEM 통로 기반 라인 미션 사용 여부
DEM_CORRIDOR_USE_NETWORK = True  # Whitebox 통로 네트워크 사용 여부(무거움)
DEM_CORRIDOR_CACHE_ENABLE = True  # Whitebox 통로 디스크 캐시 사용
DEM_CORRIDOR_CACHE_DIR = "database/corridor_cache"  # 통로 캐시 저장 경로
DEM_CORRIDOR_LOG_ENABLE = True  # 통로 캐시/생성 로그 출력
DEM_CORRIDOR_LOG_FILE = "database/corridor_cache/corridor.log"  # 통로 로그 파일
_PROJECT_ROOT = Path(__file__).resolve().parents[4]
_RESOURCE_DIR = _PROJECT_ROOT / "resource"
DEM_CORRIDOR_FILE = str(_RESOURCE_DIR / "Jipo_48km.tif")  # DEM 경로
DEM_CORRIDOR_FLOW_ACC_THRESHOLD = 1000  # Flow Accumulation 임계값
DEM_CORRIDOR_MIN_LENGTH_M = 6000.0  # 통로 최소 길이 (m)
DEM_CORRIDOR_ALLOW_LINE_FALLBACK = False  # 통로 라인 재생성(whitebox) 폴백 허용 여부


DEM_PATH_GOAL_ATTEMPTS = 60  # DEM 경로 목표점 시도 횟수
DEM_PATH_MAX_ROUTE_RATIO = 0.0  # DEM 경로 길이/직선거리 제한(0=제한 없음)
DEM_PATH_TURN_DEG = 20.0  # 경로 꺾임 각도 기준 (deg)
DEM_PATH_MIN_SEG_M = 1000.0  # 경로 좌표 최소 구간 길이 (m)
DEM_PATH_GRID_SIZE = 120  # DEM 경로 탐색 그리드 해상도
DEM_PATH_BUFFER_M = 5000.0  # DEM 경로 탐색 버퍼 (m)
DEM_PATH_ELEV_WEIGHT = 6.0  # 저고도 선호 가중치
DEM_PATH_ELEV_SAMPLES = 60  # 경로 고도 샘플 수
DEM_PATH_MAX_ELEV_M = 0.0  # 경로 최대 고도 제한(0=제한 없음)


DEM_3D_BUFFER_M = 4000.0  # 3D 프리뷰 영역 버퍼 (m)
DEM_3D_GRID_SIZE = 180  # 3D 프리뷰 그리드 해상도
DEM_3D_VIEW_ELEV = 38.0  # 3D 프리뷰 기본 고도 각도
DEM_3D_VIEW_AZIM = 130.0  # 3D 프리뷰 기본 방위 각도
DEM_3D_Z_MAX = 1500.0  # 3D 프리뷰 Z축 최대값
DEM_3D_LINE_OFFSET_M = 350.0  # 3D 프리뷰 라인 Z 오프셋 (m)
DEM_3D_SURFACE_ALPHA = 0.9  # 3D 프리뷰 지형 투명도


# TYPE1 대기갑 항공 타격 작전 -------------------------
ANTI_ARMOR_ROUTE_TANK_RANGE = (1, 2)  # 대전차 경로 탱크 개수 범위
ANTI_ARMOR_AREA_TANK_RANGE = (2, 3)  # 대전차 지역 탱크 개수 범위
ANTI_ARMOR_AREA_MLRS_RANGE = (1, 2)  # 대전차 지역 MLRS 개수 범위
ANTI_ARMOR_AREA_AAA_RANGE = (1, 2)  # 대전차 지역 AAA 개수 범위

# TYPE5 도시지역 작전 -------------------------
CITY_MISSION_AREA_NW = (38.148888, 127.300531)
CITY_MISSION_AREA_SE = (38.142553, 127.316269)
