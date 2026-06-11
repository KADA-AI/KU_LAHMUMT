from __future__ import annotations

import io
import json
import math
import os
import re
import shutil
import socket
import sys
import threading
import time
import uuid
import webbrowser
from copy import deepcopy
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, unquote, urlparse

ROOT = Path(__file__).resolve().parent
DB_ROOT = ROOT / "database"
RTV_DIR = ROOT / "RTV"
UI_DIR = ROOT / "ui"
TEMPLATE_PATH = RTV_DIR / "newScenario_251030.json"
SCENARIO_DIR = DB_ROOT / "Scenario"
RTV_SCENARIO_DIR = DB_ROOT / "RTV_scenario"

FPL_DIR = ROOT / "FPL_Random"
sys.path.insert(0, str(FPL_DIR))
sys.path.insert(0, str(RTV_DIR))

from FPL_Random.fpl_random import areas, config, flight_ref, mission_plan, pipeline, paths, dem_preview  # noqa: E402
import RTV.build_scenario  # noqa: E402


CONFIG_SCHEMA: List[Dict[str, Any]] = [
    {
        "group": "시나리오 생성 구역",
        "items": [
            {
                "key": "AUTO_MISSION_AREA",
                "label": "자동 임무 생성 구역",
                "desc": "기본값: 지포리 영역 5배 크기의 정사각형. 입력 형식: [[위도, 경도], [위도, 경도]]",
            },
            {
                "key": "START_REFERENCE_POINTS_RAW",
                "label": "구역 내 시작 참조점",
                "desc": "자동 임무 생성 구역 내 시작 참조점 N개 사전 설정. TakeOverInfoList 선정에 사용(무분별한 시작 위치 방지).",
            },
        ],
    },
    {
        "group": "비행참조정보 (FlightReferenceInfoData)",
        "items": [
            {
                "key": "SCN_FILENAME",
                "label": "파일명",
                "desc": "0001.json 형식으로 오름차순 생성.",
                "value": "0001.json",
                "editable": False,
            },
            {
                "key": "SCN_TIMESTAMP",
                "label": "Timestamp",
                "desc": "2000-01-01부터 ms. 생성 시점 자동 입력.",
                "value": "자동 생성",
                "editable": False,
            },
            {
                "key": "SCN_PACKAGE_ID",
                "label": "MissionReferencePackageID",
                "desc": "비행참조정보 ID. 1부터 오름차순 생성.",
                "value": "자동 증가",
                "editable": False,
            },
            {
                "key": "SCN_INPUT_TIMESTAMP",
                "label": "InputTimestamp",
                "desc": "입력된 시점. 생성 시점 자동 입력.",
                "value": "자동 생성",
                "editable": False,
            },
            {
                "key": "UAV_IDS",
                "label": "TakeOverInfoList - AircraftID",
                "desc": "무인기 3대 ID 목록(기본 4,5,6). TakeOver/HandOver/RTB에 공통 적용.",
            },
            {
                "key": "SIDE_M",
                "label": "TakeOverInfoList - 삼각형 변 길이 (m)",
                "desc": "구역 내 시작 참조점을 기준으로 150m 간격 정삼각형 배치.",
            },
            {
                "key": "TAKEOVER_COORD_RULE",
                "label": "TakeOverInfoList - Coordinate",
                "desc": "구역 내 시작 참조점 중 랜덤 선택. 유인기 시작점은 삼각형 아래 변에서 남쪽 300m 규칙.",
                "value": "자동 생성",
                "editable": False,
            },
            {
                "key": "TAKEOVER_ALT_AGL_M",
                "label": "TakeOver/HandOver/RTB 고도 (m)",
                "desc": "무인기 통제권 획득/인계/귀환 위치 고도.",
            },
            {
                "key": "HANDOVER_OFFSET_M",
                "label": "HandOverInfoList 오프셋 (m)",
                "desc": "TakeOver 기준 좌/우 300m 랜덤 이동.",
            },
            {
                "key": "RTB_OFFSET_M",
                "label": "RTBCoordinateList 오프셋 (m)",
                "desc": "HandOver 기준 반대 방향 300m 이동.",
            },
            {
                "key": "FLIGHT_AREA_RULE",
                "label": "FlightAreaList 생성 규칙",
                "desc": "자동 임무 생성 구역을 비행가능구역으로 사용. ProhibitedArea 차집합 기준. 고도 하한 0, 상한 5000 고정.",
                "value": "자동 생성",
                "editable": False,
            },
            {
                "key": "PROHIBITED_OFFSET_MIN",
                "label": "ProhibitedAreaList 오프셋 최소 (m)",
                "desc": "비행금지구역 중심을 생성할 때 사용하는 최소 오프셋.",
            },
            {
                "key": "PROHIBITED_OFFSET_MAX",
                "label": "ProhibitedAreaList 오프셋 최대 (m)",
                "desc": "비행금지구역 중심을 생성할 때 사용하는 최대 오프셋.",
            },
            {
                "key": "PROHIBITED_RADIUS_M",
                "label": "ProhibitedAreaList 반경 (m)",
                "desc": "비행금지구역 크기(5각형 반경).",
            },


            {
                "key": "PROHIBITED_SHAPE_RULE",
                "label": "ProhibitedAreaList 생성 규칙",
                "desc": "Radar/SAM 예상 위치 2개 지역을 5각형으로 생성. 임무지역을 방해하지 않도록 마지막 단계에서 배치.",
                "value": "5각형 고정",
                "editable": False,
            },
        ],
    },
    {
        "group": "협업기저임무계획 (InputMissionPlanData)",
        "items": [
            {
                "key": "IMP_FILENAME",
                "label": "파일명",
                "desc": "0001.json 형식으로 오름차순 생성.",
                "value": "0001.json",
                "editable": False,
            },
            {
                "key": "IMP_TIMESTAMP",
                "label": "Timestamp",
                "desc": "2000-01-01부터 ms. 입력 시점 자동 입력.",
                "value": "자동 생성",
                "editable": False,
            },
            {
                "key": "IMP_PACKAGE_ID",
                "label": "InputMissionPackageID",
                "desc": "협업기저임무 패키지 ID. 1부터 오름차순 생성.",
                "value": "자동 증가",
                "editable": False,
            },
            {
                "key": "IMP_PACKAGE_TYPE",
                "label": "InputMissionPackageType",
                "desc": "1~5 임무/작전명 구분. 랜덤 생성.",
                "value": "자동 생성",
                "editable": False,
            },
            {
                "key": "MAIN_SENSOR_WEIGHTS",
                "label": "MainSensor 확률 (EO/IR)",
                "desc": "센서 선택 확률. 기본: EO 80%, IR 20%. JSON 딕셔너리 입력 예: {\"1\": 0.8, \"2\": 0.2}",
            },
            {
                "key": "AIRCRAFT_IDS",
                "label": "AvailableAircraftList - AircraftID",
                "desc": "가용 유무인기 ID 목록. 무인기 3대(4~6)는 항상 포함.",
            },
            {
                "key": "INPUT_MISSION_COUNT_RANGE",
                "label": "InputMissionList 개수 범위",
                "desc": "협업기저임무 ID는 3~10개 범위에서 자동 생성.",
            },
            {
                "key": "INPUT_MISSION_TYPE_RULE",
                "label": "InputMissionType 규칙",
                "desc": "선=이동, 점/면=작전 구조로 배치. 협업기동(1)을 중심으로 수색/경계/엄호 임무를 확률에 따라 혼합.",
                "value": "자동 생성",
                "editable": False,
            },
            {
                "key": "LINE_MISSION_COUNT_RANGE",
                "label": "직선 임무 개수 범위",
                "desc": "직선 구간(라인) 임무 개수 범위.",
            },
            {
                "key": "LINE_MISSION_TYPE_WEIGHTS",
                "label": "라인 임무 타입 가중치",
                "desc": "라인 임무 타입 가중치. 예: {\"1\": 0.7, \"4\": 0.15, \"5\": 0.15}",
            },
            {
                "key": "POINT_MISSION_TYPE_WEIGHTS",
                "label": "점/면 임무 타입 가중치",
                "desc": "점/면 임무 타입 가중치. 예: {\"2\": 0.4, \"3\": 0.2, \"6\": 0.2, \"4\": 0.1, \"5\": 0.1}",
            },
            {
                "key": "IS_DONE_RULE",
                "label": "IsDone",
                "desc": "수행완료 여부는 항상 False.",
                "value": "False",
                "editable": False,
            },
            {
                "key": "LINE_SEGMENT_MIN_M",
                "label": "직선 구간 최소 길이 (m)",
                "desc": "시작점에서 0.5~2km 사이 랜덤 이동 규칙의 최소값.",
            },
            {
                "key": "LINE_SEGMENT_MAX_M",
                "label": "직선 구간 최대 길이 (m)",
                "desc": "시작점에서 0.5~2km 사이 랜덤 이동 규칙의 최대값.",
            },
            {
                "key": "LINE_POINT_COUNT_RANGE",
                "label": "직선 구간 좌표 개수 범위",
                "desc": "직선 이동을 반복해 생성하는 점 개수 범위.",
            },
            {
                "key": "PER_AIRCRAFT_WIDTH_MIN_M",
                "label": "구간 폭 최소 (기체당, m)",
                "desc": "기체 1대 기준 200~500m 범위.",
            },
            {
                "key": "PER_AIRCRAFT_WIDTH_MAX_M",
                "label": "구간 폭 최대 (기체당, m)",
                "desc": "기체 1대 기준 200~500m 범위.",
            },
            {
                "key": "WIDTH_STEP_M",
                "label": "구간 폭 증감 단위 (m)",
                "desc": "폭은 50m 단위로 증감.",
            },
            {
                "key": "LINE_ALT_MIN_M",
                "label": "라인 구간 고도 최소 (m)",
                "desc": "라인 좌표 고도는 AGL 600~1500m 범위에서 랜덤.",
            },
            {
                "key": "LINE_ALT_MAX_M",
                "label": "라인 구간 고도 최대 (m)",
                "desc": "라인 좌표 고도는 AGL 600~1500m 범위에서 랜덤.",
            },
            {
                "key": "AREA_SIDE_MIN_M",
                "label": "면 임무 한 변 최소 (m)",
                "desc": "면 임무 크기 최소값(직경 2~7km 범위, 500m 단위).",
            },
            {
                "key": "AREA_SIDE_MAX_M",
                "label": "면 임무 한 변 최대 (m)",
                "desc": "면 임무 크기 최대값(직경 2~7km 범위, 500m 단위).",
            },
            {
                "key": "AREA_SIDE_STEP_M",
                "label": "면 임무 한 변 증감 단위 (m)",
                "desc": "면 임무 크기는 500m 단위로 증감.",
            },
            {
                "key": "EDGE_GAP_MIN_M",
                "label": "선/면 간격 최소 (m)",
                "desc": "직선 임무와 면 임무 간 최소 이격.",
            },
            {
                "key": "EDGE_GAP_MAX_M",
                "label": "선/면 간격 최대 (m)",
                "desc": "직선 임무와 면 임무 간 최대 이격.",
            },
            {
                "key": "BORDER_MARGIN_M",
                "label": "임무 생성 경계 여유 (m)",
                "desc": "자동 임무 생성 구역 경계로부터 여유 거리.",
            },
            {
                "key": "START_OFFSET_MIN_M",
                "label": "시작점 오프셋 최소 (m)",
                "desc": "TakeOver 기준 시작점 최소 거리.",
            },
            {
                "key": "START_OFFSET_MAX_M",
                "label": "시작점 오프셋 최대 (m)",
                "desc": "TakeOver 기준 시작점 최대 거리.",
            },
            {
                "key": "HEADING_DELTA_MAX_DEG",
                "label": "구간 헤딩 변화 최대 (deg)",
                "desc": "직선 구간 간 최대 회전각.",
            },
            {
                "key": "CONTINUE_HEADING_MAX_DEG",
                "label": "연속 헤딩 변화 허용 (deg)",
                "desc": "전 구간 대비 진행 방향 일관성 허용치.",
            },
            {
                "key": "FORWARD_ALIGN_DEG",
                "label": "전방 정렬 허용 (deg)",
                "desc": "면 임무 배치 시 진행 방향 정렬 허용치.",
            },
            {
                "key": "MAX_GEN_ATTEMPTS",
                "label": "전체 생성 재시도 횟수",
                "desc": "생성 실패 시 전체 재시도 횟수.",
            },
            {
                "key": "SEGMENT_ATTEMPTS",
                "label": "라인 생성 재시도 횟수",
                "desc": "직선 구간 생성 재시도 횟수.",
            },
            {
                "key": "RECT_ATTEMPTS",
                "label": "면/출입점 생성 재시도 횟수",
                "desc": "면 임무 및 출입점 생성 재시도 횟수.",
            },
        ],
    },
    {
        "group": "표적 배치 (TargetInfo)",
        "items": [
            {
                "key": "TGT_FILENAME",
                "label": "파일명",
                "desc": "0001.json 형식으로 오름차순 생성.",
                "value": "0001.json",
                "editable": False,
            },
            {
                "key": "TGT_TARGET_ID_RULE",
                "label": "TargetID",
                "desc": "1부터 자동으로 생성.",
                "value": "자동 생성",
                "editable": False,
            },
            {
                "key": "TARGET_COUNT_RULE",
                "label": "표적 개수 규칙",
                "desc": "전체 타깃 수 = 임무 개수 × (1/5~1/3) 범위에서 생성.",
                "value": "임무수×(1/5~1/3)",
                "editable": False,
            },
            {
                "key": "TARGET_TYPE_RULE",
                "label": "TargetType",
                "desc": "0: None, 1: 전차, 2: 장갑차, 3: 방사포, 4: 곡사포, 5: 고정고사포, 6: 군인.",
                "value": "랜덤/시나리오별 세트",
                "editable": False,
            },
            {
                "key": "TARGET_COORD_RULE",
                "label": "Coordinate",
                "desc": "표적 위치는 생성된 임무 영역(라인/면) 내부에 배정.",
                "value": "임무 영역 기준",
                "editable": False,
            },
            {
                "key": "TARGET_PATH_RULE",
                "label": "Path",
                "desc": "이동 가능한 객체는 생성 위치 기준 직경 500m 내에서 랜덤 이동 경로 생성.",
                "value": "이동 객체만 경로 생성",
                "editable": False,
            },
            {
                "key": "TARGET_COUNT_RATIO_RANGE",
                "label": "전체 타깃 수 비율 범위",
                "desc": "전체 타깃 수 = 임무 개수 × (1/5~1/3) 비율 범위.",
            },
            {
                "key": "MANEUVER_TANK_RANGE",
                "label": "기동(라인) 탱크 개수 범위",
                "desc": "협업기동 임무에는 0~3개 탱크만 배치.",
            },
            {
                "key": "ANTI_ARMOR_ROUTE_TANK_RANGE",
                "label": "대기갑 길목 탱크 개수 범위",
                "desc": "대기갑 항공타격: 길목에 탱크 2대 기준, 개수 변주.",
            },
            {
                "key": "ANTI_ARMOR_AREA_TANK_RANGE",
                "label": "대기갑 목표지역 탱크 개수 범위",
                "desc": "대기갑 항공타격: 목표지역 내 탱크 3대 기준 변주.",
            },
            {
                "key": "ANTI_ARMOR_AREA_MLRS_RANGE",
                "label": "대기갑 목표지역 방사포 개수 범위",
                "desc": "대기갑 항공타격: 목표지역 내 방사포 2개 기준 변주.",
            },
            {
                "key": "ANTI_ARMOR_AREA_AAA_RANGE",
                "label": "대기갑 목표지역 고정고사포 개수 범위",
                "desc": "대기갑 항공타격: 목표지역 내 고정고사포 포함.",
            },
            {
                "key": "TARGET_PATH_RADIUS_M",
                "label": "표적 경로 반경 (m)",
                "desc": "이동 가능한 표적의 이동 반경. 직경 500m 기준이면 250m.",
            },
            {
                "key": "TARGET_PATH_POINTS_RANGE",
                "label": "표적 경로 좌표 개수 범위",
                "desc": "이동 경로 좌표 수(시작점 포함).",
            },
            {
                "key": "MOVING_TARGET_TYPES",
                "label": "이동 가능한 표적 타입",
                "desc": "경로를 생성할 타입 목록.",
            },
            {
                "key": "TARGET_MIN_SEP_M",
                "label": "타깃 최소 간격 (m)",
                "desc": "타깃 간 최소 이격 거리.",
            },
            {
                "key": "LINE_LATERAL_OFFSET_M",
                "label": "라인 측면 오프셋 (m)",
                "desc": "라인 임무에서 타깃을 측면으로 살짝 이동.",
            },
            {
                "key": "TAKEOVER_CLEARANCE_M",
                "label": "TakeOver 최소 이격 (m)",
                "desc": "TakeOver 위치와 타깃 간 최소 거리.",
            },
        ],
    },
    {
        "group": "Scenario Sensor Override",
        "items": [
            {
                "key": "SCENARIO_DETECT_PIXEL",
                "label": "DetectPixel",
                "desc": "Scenario UnitObjectList 우군 기체 DetectPixel 값. 빈 값이면 템플릿 값을 그대로 사용.",
            },
            {
                "key": "SCENARIO_RECOG_PIXEL",
                "label": "RecogPixel",
                "desc": "Scenario UnitObjectList 우군 기체 RecogPixel 값. 빈 값이면 템플릿 값을 그대로 사용.",
            },
        ],
    },
]


def _default_config_values() -> Dict[str, Any]:
    values: Dict[str, Any] = {}
    for group in CONFIG_SCHEMA:
        for item in group["items"]:
            key = item["key"]
            if "value" in item:
                values[key] = item["value"]
            elif key == "AUTO_MISSION_AREA":
                values[key] = [
                    list(config.AUTO_MISSION_AREA_SW),
                    list(config.AUTO_MISSION_AREA_NE),
                ]
            elif hasattr(config, key):
                values[key] = deepcopy(getattr(config, key))
            else:
                values[key] = ""
    return values


def _default_generation_values() -> Dict[str, Any]:
    target_types = getattr(config, "GENERATION_DEFAULT_TARGET_TYPES", (1, 2))
    return {
        "count": str(int(getattr(config, "GENERATION_DEFAULT_COUNT", 1))),
        "max_offset": str(float(getattr(config, "GENERATION_DEFAULT_MAX_OFFSET_M", 2500.0))),
        "target_types": json.dumps(list(target_types), ensure_ascii=True),
        "composite_route": True,
        "terrain_targeting": False,
    }


def _to_json_text(value: Any) -> str:
    if isinstance(value, (tuple, list, dict)):
        return json.dumps(value, ensure_ascii=True)
    return str(value)


def _normalize_value(raw: Any, template: Any) -> Any:
    if isinstance(template, tuple):
        if template and isinstance(template[0], tuple):
            return tuple(tuple(item) for item in raw)
        return tuple(raw)
    if isinstance(template, list):
        return list(raw)
    if isinstance(template, dict):
        if template and all(isinstance(k, int) for k in template.keys()):
            return {int(k): v for k, v in raw.items()}
        return dict(raw)
    return raw


def _parse_value(text: str, template: Any) -> Any:
    text = text.strip()
    if isinstance(template, bool):
        if text.lower() in ("true", "1", "yes", "y", "on"):
            return True
        if text.lower() in ("false", "0", "no", "n", "off"):
            return False
        raise ValueError("불리언 값이 필요합니다.")
    if isinstance(template, int) and not isinstance(template, bool):
        return int(text)
    if isinstance(template, float):
        return float(text)
    if isinstance(template, (tuple, list, dict)):
        data = json.loads(text)
        return _normalize_value(data, template)
    return text


def _refresh_config_dependent_modules() -> None:
    areas.AUTO_MISSION_AREA = areas.ScenarioArea(
        southwest=areas.LatLon(*config.AUTO_MISSION_AREA_SW),
        northeast=areas.LatLon(*config.AUTO_MISSION_AREA_NE),
    )
    areas.START_REFERENCE_POINTS = tuple(
        areas.LatLon(lat, lon) for lat, lon in config.START_REFERENCE_POINTS_RAW
    )

    flight_ref.AUTO_MISSION_AREA = areas.AUTO_MISSION_AREA
    flight_ref.START_REFERENCE_POINTS = areas.START_REFERENCE_POINTS

    mission_plan.AUTO_MISSION_AREA = areas.AUTO_MISSION_AREA
    mission_plan.REF_LAT = (
        areas.AUTO_MISSION_AREA.southwest.latitude + areas.AUTO_MISSION_AREA.northeast.latitude
    ) / 2.0
    mission_plan.REF_LON = (
        areas.AUTO_MISSION_AREA.southwest.longitude + areas.AUTO_MISSION_AREA.northeast.longitude
    ) / 2.0


def _parse_int_list(text: str) -> List[int]:
    text = text.strip()
    if not text:
        return []
    if text.startswith("["):
        data = json.loads(text)
        return [int(v) for v in data]
    return [int(v.strip()) for v in text.split(",") if v.strip()]


def _validate_generation_settings(max_offset: float) -> None:
    sw_lat, sw_lon = map(float, config.AUTO_MISSION_AREA_SW)
    ne_lat, ne_lon = map(float, config.AUTO_MISSION_AREA_NE)
    if ne_lat <= sw_lat or ne_lon <= sw_lon:
        raise ValueError("AUTO_MISSION_AREA must be ordered as southwest -> northeast.")

    start_min = float(config.START_OFFSET_MIN_M)
    start_max = float(config.START_OFFSET_MAX_M)
    if start_min < 0.0 or start_max < 0.0:
        raise ValueError("START_OFFSET_MIN_M and START_OFFSET_MAX_M must be non-negative.")
    if start_max < start_min:
        raise ValueError("START_OFFSET_MAX_M must be greater than or equal to START_OFFSET_MIN_M.")
    if max_offset <= 0.0:
        raise ValueError("generation max_offset must be greater than 0.")
    if max_offset < start_min:
        raise ValueError("generation max_offset must be greater than or equal to START_OFFSET_MIN_M.")

    refs = getattr(config, "START_REFERENCE_POINTS_RAW", ()) or ()
    for idx, item in enumerate(refs, start=1):
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            raise ValueError(f"START_REFERENCE_POINTS_RAW[{idx}] must be [lat, lon].")
        lat, lon = float(item[0]), float(item[1])
        if not (sw_lat <= lat <= ne_lat and sw_lon <= lon <= ne_lon):
            raise ValueError(
                f"START_REFERENCE_POINTS_RAW[{idx}] must stay inside AUTO_MISSION_AREA."
            )

def _read_json_file(path: Path) -> Any:
    try:
        payload = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        payload = path.read_text(encoding="utf-8", errors="replace")
    return json.loads(payload)


def _coords_from_list(items: Any) -> List[List[float]]:
    coords: List[List[float]] = []
    if not isinstance(items, list):
        return coords
    for item in items:
        if not isinstance(item, dict):
            continue
        lat = item.get("latitude")
        lon = item.get("longitude")
        if lat is None or lon is None:
            continue
        coords.append([float(lat), float(lon)])
    return coords


def _coord_from_obj(item: Any) -> Optional[List[float]]:
    if not isinstance(item, dict):
        return None
    lat = item.get("latitude")
    lon = item.get("longitude")
    if lat is None or lon is None:
        return None
    return [float(lat), float(lon)]


def _distance_m(a: List[float], b: List[float]) -> float:
    lat_mean = math.radians((a[0] + b[0]) / 2.0)
    dlat = (b[0] - a[0]) * 111_320.0
    dlon = (b[1] - a[1]) * 111_320.0 * math.cos(lat_mean)
    return math.hypot(dlat, dlon)


def _line_length_m(coords: List[List[float]]) -> float:
    total = 0.0
    for i in range(1, len(coords)):
        total += _distance_m(coords[i - 1], coords[i])
    return total


def _extract_preview(imp_path: Path, scn_path: Path, tgt_path: Path) -> Dict[str, Any]:
    preview: Dict[str, Any] = {
        "flight_areas": [],
        "prohibited_areas": [],
        "takeover": [],
        "handover": [],
        "rtb": [],
        "missions": {"lines": [], "areas": [], "points": []},
        "targets": [],
        "target_paths": [],
        "bounds": None,
        "errors": [],
    }

    all_points: List[List[float]] = []

    def add_points(points: List[List[float]]) -> None:
        if points:
            all_points.extend(points)

    try:
        scn = _read_json_file(scn_path)
        for area in scn.get("flightAreaList", []) or []:
            poly = _coords_from_list(area.get("areaLatLonList"))
            if poly:
                preview["flight_areas"].append(poly)
                add_points(poly)
        for area in scn.get("prohibitedAreaList", []) or []:
            poly = _coords_from_list(area.get("areaLatLonList"))
            if poly:
                preview["prohibited_areas"].append(poly)
                add_points(poly)
        for item in scn.get("takeOverInfoList", []) or []:
            coord = _coord_from_obj(item.get("coordinate"))
            if coord:
                preview["takeover"].append(coord)
                add_points([coord])
        for item in scn.get("handOverInfoList", []) or []:
            coord = _coord_from_obj(item.get("coordinate"))
            if coord:
                preview["handover"].append(coord)
                add_points([coord])
        for item in scn.get("rtbCoordinateList", []) or []:
            coord = _coord_from_obj(item)
            if coord:
                preview["rtb"].append(coord)
                add_points([coord])
    except Exception as exc:
        preview["errors"].append(f"scn: {exc}")

    try:
        imp = _read_json_file(imp_path)
        for mission in imp.get("inputMissionList", []) or []:
            mission_id = mission.get("inputMissionID")
            detail = mission.get("missionDetail") or {}
            point_list = _coords_from_list(detail.get("coordinateList"))
            if point_list:
                for coord in point_list:
                    preview["missions"]["points"].append({"id": mission_id, "coord": coord})
                    add_points([coord])
            for line in detail.get("lineList") or []:
                coords = _coords_from_list(line.get("coordinateList"))
                if coords:
                    preview["missions"]["lines"].append(
                        {"id": mission_id, "coords": coords, "length_m": _line_length_m(coords)}
                    )
                    add_points(coords)
            for area in detail.get("areaList") or []:
                coords = _coords_from_list(area.get("coordinateList"))
                if coords:
                    preview["missions"]["areas"].append({"id": mission_id, "coords": coords})
                    add_points(coords)
    except Exception as exc:
        preview["errors"].append(f"imp: {exc}")

    try:
        tgt = _read_json_file(tgt_path)
        for target in tgt.get("targetList", []) or []:
            coord = _coord_from_obj(target.get("location"))
            if coord:
                preview["targets"].append(
                    {
                        "coord": coord,
                        "type": target.get("targetType"),
                        "mission": target.get("inputMissionID"),
                    }
                )
                add_points([coord])
            path = _coords_from_list(target.get("path"))
            if path:
                preview["target_paths"].append(path)
                add_points(path)
    except Exception as exc:
        preview["errors"].append(f"tgt: {exc}")

    if all_points:
        lats = [p[0] for p in all_points]
        lons = [p[1] for p in all_points]
        preview["bounds"] = {
            "minLat": min(lats),
            "maxLat": max(lats),
            "minLon": min(lons),
            "maxLon": max(lons),
        }

    return preview


def _safe_copy(src: Path, dest: Path) -> Path:
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)
    return dest


def _write_json_file(dest: Path, payload: Dict[str, Any]) -> Path:
    dest.parent.mkdir(parents=True, exist_ok=True)
    with dest.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return dest


def _safe_unlink(path: Optional[Path]) -> None:
    if not path:
        return
    try:
        path.unlink()
    except FileNotFoundError:
        return
    except Exception:
        return


def _safe_rmdir_if_empty(path: Optional[Path]) -> None:
    if not path:
        return
    try:
        path.rmdir()
    except FileNotFoundError:
        return
    except OSError:
        return
    except Exception:
        return


def _unique_dir(base: Path) -> Path:
    if not base.exists():
        return base
    for idx in range(1, 1000):
        candidate = base.parent / f"{base.name}_{idx:02d}"
        if not candidate.exists():
            return candidate
    return base.parent / f"{base.name}_{datetime.now().strftime('%H%M%S')}"


def _unique_file_path(base: Path) -> Path:
    if not base.exists():
        return base
    stem = base.stem
    suffix = base.suffix
    for idx in range(1, 1000):
        candidate = base.with_name(f"{stem}_{idx:02d}{suffix}")
        if not candidate.exists():
            return candidate
    return base.with_name(f"{stem}_{datetime.now().strftime('%H%M%S')}{suffix}")


def _bundle_agency_code() -> str:
    return os.environ.get("KU_AGENCY_CODE") or "SBC3"


def _sync_rtv_scenario_archive() -> int:
    RTV_SCENARIO_DIR.mkdir(parents=True, exist_ok=True)
    copied = 0
    scenario_files = sorted(
        (path for path in DB_ROOT.glob("Random_Scenario_*/*/Scenario/*.json") if path.is_file()),
        key=lambda path: (path.name, path.stat().st_mtime),
    )
    for scenario_path in scenario_files:
        if not scenario_path.is_file():
            continue
        dest = RTV_SCENARIO_DIR / scenario_path.name
        if dest.exists():
            try:
                if dest.read_bytes() == scenario_path.read_bytes():
                    continue
            except Exception:
                pass
        _safe_copy(scenario_path, dest)
        copied += 1
    return copied


def _area_mission_ids(imp_payload: Dict[str, Any]) -> set[int]:
    area_ids: set[int] = set()
    for mission in imp_payload.get("inputMissionList", []) or []:
        detail = mission.get("missionDetail") or {}
        if detail.get("areaList"):
            try:
                area_ids.add(int(mission.get("inputMissionID", 0) or 0))
            except Exception:
                continue
    return area_ids


def _sanitize_target_file(imp_path: Path, tgt_path: Path) -> int:
    imp_payload = _read_json_file(imp_path)
    area_ids = _area_mission_ids(imp_payload)
    tgt_payload = _read_json_file(tgt_path)
    target_list = list(tgt_payload.get("targetList") or [])
    filtered = [
        target
        for target in target_list
        if int(target.get("inputMissionID", 0) or 0) in area_ids
    ]
    removed = len(target_list) - len(filtered)
    if removed <= 0:
        return 0
    for idx, target in enumerate(filtered, start=1):
        target["targetID"] = idx
    tgt_payload["targetList"] = filtered
    _write_json_file(tgt_path, tgt_payload)
    return removed


def _save_generated_bundle(
    *,
    imp_path: Path,
    scn_path: Path,
    tgt_path: Path,
    scenario_payload: Dict[str, Any],
    scenario_name: str,
    seq: Optional[int],
) -> Dict[str, str]:
    seed_tag = f"{int(seq):04d}" if seq is not None else "generated"
    timestamp = datetime.now().strftime("%Y-%m-%dT%H%M%S")
    scenario_dir = _unique_dir(DB_ROOT / f"Random_Scenario_{seed_tag}_{timestamp}")
    db_root = scenario_dir / _bundle_agency_code()

    required_dirs = (
        "InputMissionPlan",
        "MissionReferenceInfo",
        "TargetInfo",
        "Scenario",
        "MissionPlan",
        "IndividualMissionPlan",
        "FlightPath",
        "MissionPlanOptionInfo",
        "DSS_Internal",
        "mission_output",
    )
    for sub in required_dirs:
        (db_root / sub).mkdir(parents=True, exist_ok=True)

    imp_copy = _safe_copy(imp_path, db_root / "InputMissionPlan" / imp_path.name)
    scn_copy = _safe_copy(scn_path, db_root / "MissionReferenceInfo" / scn_path.name)
    tgt_copy = _safe_copy(tgt_path, db_root / "TargetInfo" / tgt_path.name)
    scenario_copy = db_root / "Scenario" / f"{scenario_name}.json"
    _write_json_file(scenario_copy, scenario_payload)
    rtv_scenario_copy = _write_json_file(
        RTV_SCENARIO_DIR / f"{scenario_name}.json",
        scenario_payload,
    )

    _safe_unlink(imp_path)
    _safe_unlink(scn_path)
    _safe_unlink(tgt_path)
    _safe_rmdir_if_empty(imp_path.parent)
    _safe_rmdir_if_empty(scn_path.parent)
    _safe_rmdir_if_empty(tgt_path.parent)
    _safe_rmdir_if_empty(SCENARIO_DIR)
    return {
        "bundle_root": str(scenario_dir),
        "bundle_db_root": str(db_root),
        "imp": str(imp_copy),
        "scn": str(scn_copy),
        "tgt": str(tgt_copy),
        "scenario": str(scenario_copy),
        "rtv_scenario": str(rtv_scenario_copy),
    }


def _build_scenario_bundle(result: Dict[str, Any]) -> Dict[str, Any]:
    imp_path = Path(result["paths"]["input_mission_plan"]).resolve()
    scn_path = Path(result["paths"]["flight_reference"]).resolve()
    tgt_path = Path(result["paths"]["targets"]).resolve()
    _sanitize_target_file(imp_path, tgt_path)

    seq = None
    stem = imp_path.stem
    if stem.isdigit():
        seq = int(stem)
    else:
        match = re.search(r"(\d+)$", stem)
        if match:
            try:
                seq = int(match.group(1))
            except Exception:
                seq = None

    scenario_name = RTV.build_scenario.scenario_name_now(seq=seq)
    scenario = RTV.build_scenario.build_scenario(
        template_path=TEMPLATE_PATH,
        imp_path=imp_path,
        mr_path=scn_path,
        tgt_path=tgt_path,
        scenario_name=scenario_name,
        detect_pixel=config.SCENARIO_DETECT_PIXEL,
        recog_pixel=config.SCENARIO_RECOG_PIXEL,
    )

    bundle_paths = _save_generated_bundle(
        imp_path=imp_path,
        scn_path=scn_path,
        tgt_path=tgt_path,
        scenario_payload=scenario,
        scenario_name=scenario_name,
        seq=seq,
    )
    preview = _extract_preview(
        Path(bundle_paths["imp"]),
        Path(bundle_paths["scn"]),
        Path(bundle_paths["tgt"]),
    )
    return {
        "scenario": bundle_paths["scenario"],
        "scenarioName": scenario_name,
        "imp": bundle_paths["imp"],
        "scn": bundle_paths["scn"],
        "tgt": bundle_paths["tgt"],
        "bundleRoot": bundle_paths["bundle_root"],
        "bundleDbRoot": bundle_paths["bundle_db_root"],
        "rootScenario": bundle_paths["scenario"],
        "rtvScenario": bundle_paths["rtv_scenario"],
        "seq": seq,
        "preview": preview,
    }


r'''
Legacy PyQt desktop implementation kept below only as historical reference.
The active entrypoint now lives after this block and runs a local web server.

class GenerationWorker(QThread):
    finished = pyqtSignal(str)
    failed = pyqtSignal(str)
    status = pyqtSignal(str)

    def __init__(self, payload: Dict[str, Any], defaults: Dict[str, Any]) -> None:
        super().__init__()
        self.payload = payload
        self.defaults = defaults

    def run(self) -> None:
        try:
            config_values = self.payload.get("config", {})
            generation = self.payload.get("generation", {})
            for name, template in self.defaults.items():
                raw = config_values.get(name, _to_json_text(template))
                if name == "AUTO_MISSION_AREA":
                    area = _parse_value(str(raw), template)
                    if not isinstance(area, (list, tuple)) or len(area) != 2:
                        raise ValueError("자동 임무 생성 구역은 [[위도, 경도], [위도, 경도]] 형식이어야 합니다.")
                    sw, ne = area
                    config.AUTO_MISSION_AREA_SW = tuple(sw)
                    config.AUTO_MISSION_AREA_NE = tuple(ne)
                elif hasattr(config, name):
                    setattr(config, name, _parse_value(str(raw), template))
            _refresh_config_dependent_modules()

            generation_defaults = _default_generation_values()
            count = int(generation.get("count", generation_defaults["count"]))
            max_offset = float(generation.get("max_offset", generation_defaults["max_offset"]))
            target_types = _parse_int_list(
                str(generation.get("target_types", generation_defaults["target_types"]))
            )
            composite_route = bool(
                generation.get("composite_route", generation_defaults["composite_route"])
            )
            terrain_targeting = bool(
                generation.get("terrain_targeting", generation_defaults["terrain_targeting"])
            )

            if count < 1:
                raise ValueError("생성 횟수는 1 이상이어야 합니다.")
            _validate_generation_settings(max_offset)

            paths.set_db_root(DB_ROOT)

            self.status.emit("0201/0203/타깃 생성 중...")
            per_mission_attempts = max(
                1, int(getattr(config, "GENERATION_BATCH_ATTEMPTS_PER_MISSION", 3))
            )
            max_attempts = max(count * per_mission_attempts, count)
            attempts = 0
            results: List[Dict[str, Any]] = []
            errors: List[str] = []

            while len(results) < count and attempts < max_attempts:
                attempts += 1
                progress = f"{len(results) + 1}/{count}"
                self.status.emit(
                    f"0201/0203/?源??앹꽦 以?.. {progress} (??{attempts}/{max_attempts})"
                )
                try:
                    result = pipeline.generate_sequence(
                        max_start_offset_m=max_offset,
                        target_types=target_types,
                        composite_route=composite_route,
                        terrain_targeting=terrain_targeting,
                        save=True,
                        status_cb=self.status.emit,
                    )
                    results.append(result)
                    self.status.emit(f"?源??앹꽦 ?꾨즺 {len(results)}/{count}")
                except Exception as exc:
                    errors.append(str(exc))
                    self.status.emit(
                        f"?源??앹꽦 ?ㅽ뙣 ({len(results)}/{count}) - ?ъ떆??{attempts}/{max_attempts}: {exc}"
                    )

            if not results:
                detail = errors[-1] if errors else "unknown error"
                raise RuntimeError(f"Failed to generate missions after {max_attempts} attempts: {detail}")

            bundles = []
            for idx, result in enumerate(results, start=1):
                self.status.emit(f"Scenario build {idx}/{len(results)}...")
                bundles.append(_build_scenario_bundle(result))

            last_bundle = bundles[-1]
            payload = dict(last_bundle)
            payload["results"] = bundles
            payload["summary"] = {
                "requested_count": count,
                "generated_count": len(results),
                "attempt_count": attempts,
                "failed_attempt_count": max(0, attempts - len(results)),
                "partial": len(results) < count,
                "last_error": errors[-1] if errors else "",
            }
            self.finished.emit(json.dumps(payload))
            return

            last = results[-1]
            imp_path = Path(last["paths"]["input_mission_plan"]).resolve()
            scn_path = Path(last["paths"]["flight_reference"]).resolve()
            tgt_path = Path(last["paths"]["targets"]).resolve()

            self.status.emit("시나리오 생성 중...")
            seq = None
            stem = imp_path.stem
            if stem.isdigit():
                seq = int(stem)
            else:
                match = re.search(r"(\d+)$", stem)
                if match:
                    try:
                        seq = int(match.group(1))
                    except Exception:
                        seq = None
            scenario_name = RTV.build_scenario.scenario_name_now(seq=seq)
            scenario = RTV.build_scenario.build_scenario(
                template_path=TEMPLATE_PATH,
                imp_path=imp_path,
                mr_path=scn_path,
                tgt_path=tgt_path,
                scenario_name=scenario_name,
                detect_pixel=config.SCENARIO_DETECT_PIXEL,
                recog_pixel=config.SCENARIO_RECOG_PIXEL,
            )

            SCENARIO_DIR.mkdir(parents=True, exist_ok=True)
            out_path = SCENARIO_DIR / f"{scenario_name}.json"
            with out_path.open("w", encoding="utf-8") as fh:
                json.dump(scenario, fh, ensure_ascii=False, indent=2)

            preview = _extract_preview(imp_path, scn_path, tgt_path)
            payload = {
                "scenario": str(out_path),
                "imp": str(imp_path),
                "scn": str(scn_path),
                "tgt": str(tgt_path),
                "preview": preview,
            }
            self.finished.emit(json.dumps(payload))
        except Exception as exc:
            self.failed.emit(str(exc))


class Preview3DWindow(QDialog):
    def __init__(
        self,
        imp_path: Path,
        mission_id: int,
        dem_path: Path,
        out_dir: Path,
        *,
        view_elev: float,
        view_azim: float,
        z_max: float,
        buffer_m: float,
        grid_size: int,
    ) -> None:
        super().__init__()
        self.setWindowTitle("Mission 3D Preview")
        self.resize(820, 640)
        self.imp_path = imp_path
        self.mission_id = mission_id
        self.dem_path = dem_path
        self.out_dir = out_dir
        self.view_elev = view_elev
        self.view_azim = view_azim
        self.z_max = z_max
        self.buffer_m = buffer_m
        self.grid_size = grid_size

        self._layout = QVBoxLayout()
        self.setLayout(self._layout)

        controls = QHBoxLayout()
        btn_top = QPushButton("Top")
        btn_reset = QPushButton("Reset")
        controls.addStretch(1)
        controls.addWidget(btn_top)
        controls.addWidget(btn_reset)
        self._layout.addLayout(controls)

        btn_top.clicked.connect(self._set_top_view)
        btn_reset.clicked.connect(self._reset_view)

        self._canvas = None
        self._toolbar = None
        self._figure = None
        self._ax = None
        self._build_canvas()

    def _build_canvas(self) -> None:
        try:
            from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
            from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
        except Exception:
            label = QLabel("Matplotlib 3D renderer not available.")
            self._layout.addWidget(label)
            return

        fig_ax = dem_preview.build_mission_3d_figure(
            self.imp_path,
            self.mission_id,
            self.dem_path,
            buffer_m=self.buffer_m,
            grid_size=self.grid_size,
            view_elev=self.view_elev,
            view_azim=self.view_azim,
            z_max=self.z_max,
            line_offset_m=getattr(config, "DEM_3D_LINE_OFFSET_M", 200.0),
            surface_alpha=getattr(config, "DEM_3D_SURFACE_ALPHA", 0.9),
        )
        if not fig_ax:
            label = QLabel("3D preview failed to render.")
            self._layout.addWidget(label)
            return

        self._figure, self._ax = fig_ax
        self._proj_type = None
        if self._ax and hasattr(self._ax, "get_proj_type"):
            self._proj_type = self._ax.get_proj_type()
        self._canvas = FigureCanvas(self._figure)
        self._toolbar = NavigationToolbar(self._canvas, self)
        self._layout.addWidget(self._toolbar)
        self._layout.addWidget(self._canvas)
        self._canvas.draw()

    def _set_top_view(self) -> None:
        if not self._ax or not self._canvas:
            return
        self.view_elev = 90.0
        if hasattr(self._ax, "set_proj_type"):
            try:
                self._ax.set_proj_type("ortho")
            except Exception:
                pass
        self._ax.view_init(elev=self.view_elev, azim=self.view_azim)
        self._canvas.draw_idle()

    def _reset_view(self) -> None:
        if not self._ax or not self._canvas:
            return
        self.view_elev = getattr(config, "DEM_3D_VIEW_ELEV", 38.0)
        self.view_azim = getattr(config, "DEM_3D_VIEW_AZIM", 130.0)
        if hasattr(self._ax, "set_proj_type"):
            try:
                self._ax.set_proj_type(self._proj_type or "persp")
            except Exception:
                pass
        self._ax.view_init(elev=self.view_elev, azim=self.view_azim)
        self._canvas.draw_idle()


class Backend(QObject):
    generated = pyqtSignal(str)
    failed = pyqtSignal(str)
    status = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self.defaults = _default_config_values()
        self.generation_defaults = _default_generation_values()
        self.worker: Optional[GenerationWorker] = None
        self._preview_windows: List[Preview3DWindow] = []

    @pyqtSlot(result=str)
    def bootstrap(self) -> str:
        config_defaults = {k: _to_json_text(v) for k, v in self.defaults.items()}
        payload = {
            "config_schema": CONFIG_SCHEMA,
            "config_defaults": config_defaults,
            "generation_defaults": self.generation_defaults,
        }
        return json.dumps(payload)

    @pyqtSlot(str)
    def generate(self, payload_json: str) -> None:
        if self.worker and self.worker.isRunning():
            self.failed.emit("이미 생성 중입니다.")
            return
        try:
            payload = json.loads(payload_json)
        except Exception as exc:
            self.failed.emit(f"요청 데이터가 올바르지 않습니다: {exc}")
            return

        self.worker = GenerationWorker(payload, self.defaults)
        self.worker.finished.connect(self.generated)
        self.worker.failed.connect(self.failed)
        self.worker.status.connect(self.status)
        self.worker.start()

    @pyqtSlot(str)
    def preview3d(self, payload_json: str) -> None:
        try:
            payload = json.loads(payload_json)
            imp_path = Path(payload.get("imp_path", ""))
            mission_id = int(payload.get("mission_id", 0) or 0)
        except Exception as exc:
            self.status.emit(f"3D 요청 파싱 실패: {exc}")
            return

        if not imp_path.exists() or mission_id <= 0:
            self.status.emit("3D 프리뷰: 미션 ID 또는 파일 경로가 없습니다.")
            return

        dem_path = ROOT / config.DEM_CORRIDOR_FILE
        out_dir = ROOT / "_legacy_preview3d"
        window = Preview3DWindow(
            imp_path,
            mission_id,
            dem_path,
            out_dir,
            view_elev=getattr(config, "DEM_3D_VIEW_ELEV", 38.0),
            view_azim=getattr(config, "DEM_3D_VIEW_AZIM", 130.0),
            z_max=getattr(config, "DEM_3D_Z_MAX", 1500.0),
            buffer_m=getattr(config, "DEM_3D_BUFFER_M", 1500.0),
            grid_size=int(getattr(config, "DEM_3D_GRID_SIZE", 180)),
        )
        window.show()
        self._preview_windows.append(window)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        _ensure_app()
        super().__init__()
        # Import after QApplication is created to avoid Qt widget init before app.
        from PyQt5.QtWebEngineWidgets import QWebEngineView  # noqa: E402
        self.setWindowTitle("미션 시나리오 생성기")
        self.resize(1300, 950)

        self.view = QWebEngineView()
        self.setCentralWidget(self.view)
        self._configure_webengine_cache()

        self.backend = Backend()
        self.channel = QWebChannel()
        self.channel.registerObject("backend", self.backend)
        self.view.page().setWebChannel(self.channel)

        html_path = UI_DIR / "index.html"
        url = QUrl.fromLocalFile(str(html_path.resolve()))
        try:
            url.setQuery(f"v={html_path.stat().st_mtime_ns}")
        except Exception:
            url.setQuery("v=1")
        self.view.setUrl(url)

    def _configure_webengine_cache(self) -> None:
        cache_root = ROOT / "qtwebengine_cache"
        cache_dir = cache_root / "cache"
        storage_dir = cache_root / "storage"
        cache_dir.mkdir(parents=True, exist_ok=True)
        storage_dir.mkdir(parents=True, exist_ok=True)
        profile = self.view.page().profile()
        profile.setCachePath(str(cache_dir))
        profile.setPersistentStoragePath(str(storage_dir))


def main() -> None:
    app = _ensure_app()
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
'''


def _bootstrap_payload() -> Dict[str, Any]:
    config_defaults = {k: _to_json_text(v) for k, v in _default_config_values().items()}
    return {
        "config_schema": CONFIG_SCHEMA,
        "config_defaults": config_defaults,
        "generation_defaults": _default_generation_values(),
    }


def _run_generation_request(payload: Dict[str, Any], status_cb) -> Dict[str, Any]:
    config_values = payload.get("config", {})
    generation = payload.get("generation", {})
    defaults = _default_config_values()
    for name, template in defaults.items():
        raw = config_values.get(name, _to_json_text(template))
        if name == "AUTO_MISSION_AREA":
            area = _parse_value(str(raw), template)
            if not isinstance(area, (list, tuple)) or len(area) != 2:
                raise ValueError("AUTO_MISSION_AREA must be [[lat, lon], [lat, lon]].")
            sw, ne = area
            config.AUTO_MISSION_AREA_SW = tuple(sw)
            config.AUTO_MISSION_AREA_NE = tuple(ne)
        elif hasattr(config, name):
            setattr(config, name, _parse_value(str(raw), template))
    _refresh_config_dependent_modules()

    generation_defaults = _default_generation_values()
    count = int(generation.get("count", generation_defaults["count"]))
    max_offset = float(generation.get("max_offset", generation_defaults["max_offset"]))
    target_types = _parse_int_list(str(generation.get("target_types", generation_defaults["target_types"])))
    composite_route = bool(generation.get("composite_route", generation_defaults["composite_route"]))
    terrain_targeting = bool(generation.get("terrain_targeting", generation_defaults["terrain_targeting"]))

    if count < 1:
        raise ValueError("생성 횟수는 1 이상이어야 합니다.")
    _validate_generation_settings(max_offset)

    paths.set_db_root(DB_ROOT)
    status_cb("생성 준비 중...")

    per_mission_attempts = max(1, int(getattr(config, "GENERATION_BATCH_ATTEMPTS_PER_MISSION", 3)))
    max_attempts = max(count * per_mission_attempts, count)
    attempts = 0
    results: List[Dict[str, Any]] = []
    errors: List[str] = []

    while len(results) < count and attempts < max_attempts:
        attempts += 1
        progress = f"{len(results) + 1}/{count}"
        status_cb(f"생성 시도 {progress} ({attempts}/{max_attempts})")
        try:
            result = pipeline.generate_sequence(
                max_start_offset_m=max_offset,
                target_types=target_types,
                composite_route=composite_route,
                terrain_targeting=terrain_targeting,
                save=True,
                status_cb=status_cb,
            )
            results.append(result)
            status_cb(f"생성 완료 {len(results)}/{count}")
        except Exception as exc:
            errors.append(str(exc))
            status_cb(f"생성 실패 {attempts}/{max_attempts}: {exc}")

    if not results:
        detail = errors[-1] if errors else "unknown error"
        raise RuntimeError(f"Failed to generate missions after {max_attempts} attempts: {detail}")

    bundles = []
    for idx, result in enumerate(results, start=1):
        status_cb(f"Scenario build {idx}/{len(results)}...")
        bundles.append(_build_scenario_bundle(result))

    last_bundle = bundles[-1]
    response = dict(last_bundle)
    response["results"] = bundles
    response["summary"] = {
        "requested_count": count,
        "generated_count": len(results),
        "attempt_count": attempts,
        "failed_attempt_count": max(0, attempts - len(results)),
        "partial": len(results) < count,
        "last_error": errors[-1] if errors else "",
    }
    return response


def _resolve_dem_path() -> Path:
    dem_path = Path(str(config.DEM_CORRIDOR_FILE))
    if dem_path.is_absolute():
        return dem_path.resolve()
    return (ROOT / dem_path).resolve()


def _is_within(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
        return True
    except Exception:
        return False


def _render_preview3d_png_bytes(imp_path: Path, mission_id: int) -> bytes:
    fig_ax = dem_preview.build_mission_3d_figure(
        imp_path,
        mission_id,
        _resolve_dem_path(),
        buffer_m=getattr(config, "DEM_3D_BUFFER_M", 1500.0),
        grid_size=int(getattr(config, "DEM_3D_GRID_SIZE", 180)),
        view_elev=getattr(config, "DEM_3D_VIEW_ELEV", 38.0),
        view_azim=getattr(config, "DEM_3D_VIEW_AZIM", 130.0),
        z_max=getattr(config, "DEM_3D_Z_MAX", 1500.0),
        line_offset_m=getattr(config, "DEM_3D_LINE_OFFSET_M", 200.0),
        surface_alpha=getattr(config, "DEM_3D_SURFACE_ALPHA", 0.9),
    )
    if not fig_ax:
        raise RuntimeError("3D preview failed to render.")
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError(f"matplotlib unavailable: {exc}") from exc
    fig, _ax = fig_ax
    buffer = io.BytesIO()
    try:
        fig.savefig(buffer, format="png", dpi=140)
        return buffer.getvalue()
    finally:
        plt.close(fig)
        buffer.close()


class GenerationJob:
    def __init__(self, payload: Dict[str, Any], *, on_finish) -> None:
        self.id = uuid.uuid4().hex
        self.payload = payload
        self._on_finish = on_finish
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.state = "queued"
        self.created_at = time.time()
        self.updated_at = self.created_at
        self.status_messages: List[str] = []
        self.result: Optional[Dict[str, Any]] = None
        self.error = ""

    def start(self) -> None:
        self.add_status("요청 접수")
        with self._lock:
            self.state = "running"
            self.updated_at = time.time()
        self._thread.start()

    def add_status(self, message: str) -> None:
        with self._lock:
            self.status_messages.append(str(message))
            if len(self.status_messages) > 200:
                self.status_messages = self.status_messages[-200:]
            self.updated_at = time.time()

    def _run(self) -> None:
        try:
            result = _run_generation_request(self.payload, self.add_status)
            with self._lock:
                self.state = "completed"
                self.result = result
                self.updated_at = time.time()
        except Exception as exc:
            with self._lock:
                self.state = "failed"
                self.error = str(exc)
                self.updated_at = time.time()
        finally:
            self._on_finish(self.id)

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "job_id": self.id,
                "state": self.state,
                "latest_status": self.status_messages[-1] if self.status_messages else "",
                "status_messages": list(self.status_messages),
                "result": self.result,
                "error": self.error,
                "created_at": self.created_at,
                "updated_at": self.updated_at,
            }


class JobStore:
    def __init__(self) -> None:
        self._jobs: Dict[str, GenerationJob] = {}
        self._active_job_id: Optional[str] = None
        self._lock = threading.Lock()

    def start(self, payload: Dict[str, Any]) -> GenerationJob:
        with self._lock:
            if self._active_job_id:
                active = self._jobs.get(self._active_job_id)
                if active and active.snapshot()["state"] in {"queued", "running"}:
                    raise RuntimeError("이미 생성 작업이 진행 중입니다.")
            job = GenerationJob(payload, on_finish=self._mark_finished)
            self._jobs[job.id] = job
            self._active_job_id = job.id
        job.start()
        return job

    def _mark_finished(self, job_id: str) -> None:
        with self._lock:
            if self._active_job_id == job_id:
                self._active_job_id = None

    def get(self, job_id: str) -> Optional[GenerationJob]:
        with self._lock:
            return self._jobs.get(job_id)


_JOB_STORE = JobStore()


class RandomMissionHTTPServer(ThreadingHTTPServer):
    daemon_threads = True


class RequestHandler(BaseHTTPRequestHandler):
    server_version = "RandomMissionWeb/1.0"

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_json(self, status: int, payload: Dict[str, Any]) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _send_bytes(self, status: int, content_type: str, payload: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _read_json_body(self) -> Dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0") or "0")
        except Exception:
            length = 0
        raw = self.rfile.read(length) if length > 0 else b"{}"
        return json.loads(raw.decode("utf-8"))

    def _serve_file(self, path: Path, *, content_type: str) -> None:
        self._send_bytes(HTTPStatus.OK, content_type, path.read_bytes())

    def _serve_ui_asset(self, rel_path: str) -> None:
        rel = rel_path.lstrip("/") or "index.html"
        path = (UI_DIR / rel).resolve()
        if not _is_within(path, UI_DIR) or not path.exists() or not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if path.suffix == ".html":
            content_type = "text/html; charset=utf-8"
        elif path.suffix == ".js":
            content_type = "application/javascript; charset=utf-8"
        elif path.suffix == ".css":
            content_type = "text/css; charset=utf-8"
        elif path.suffix == ".png":
            content_type = "image/png"
        else:
            content_type = "application/octet-stream"
        self._serve_file(path, content_type=content_type)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path

        if path in {"/", "/index.html"}:
            self._serve_ui_asset("index.html")
            return
        if path.startswith("/ui/"):
            self._serve_ui_asset(path[len("/ui/"):])
            return
        if path == "/api/bootstrap":
            self._send_json(HTTPStatus.OK, _bootstrap_payload())
            return
        if path.startswith("/api/jobs/"):
            job_id = path.rsplit("/", 1)[-1]
            job = _JOB_STORE.get(job_id)
            if not job:
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "job not found"})
                return
            self._send_json(HTTPStatus.OK, job.snapshot())
            return
        if path == "/api/preview3d":
            query = parse_qs(parsed.query)
            imp_path_text = query.get("imp_path", [""])[0]
            mission_id_text = query.get("mission_id", [""])[0]
            try:
                imp_path = Path(unquote(imp_path_text)).resolve()
                mission_id = int(mission_id_text)
            except Exception as exc:
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": f"invalid preview params: {exc}"})
                return
            if mission_id <= 0 or not imp_path.exists() or not _is_within(imp_path, DB_ROOT):
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": "invalid IMP path or mission id"})
                return
            try:
                payload = _render_preview3d_png_bytes(imp_path, mission_id)
                self._send_bytes(HTTPStatus.OK, "image/png", payload)
            except Exception as exc:
                self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(exc)})
            return
        if path == "/healthz":
            self._send_json(HTTPStatus.OK, {"ok": True})
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/generate":
            try:
                payload = self._read_json_body()
                job = _JOB_STORE.start(payload)
                self._send_json(HTTPStatus.ACCEPTED, {"job_id": job.id})
            except RuntimeError as exc:
                self._send_json(HTTPStatus.CONFLICT, {"error": str(exc)})
            except Exception as exc:
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return
        self.send_error(HTTPStatus.NOT_FOUND)


def _pick_port(host: str, preferred_port: int) -> int:
    candidates = [preferred_port + i for i in range(0, 25)] if preferred_port > 0 else [0]
    for port in candidates:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.bind((host, port))
                return sock.getsockname()[1]
        except OSError:
            continue
    raise RuntimeError("No available port found for Random_mission web UI.")


def _open_browser_later(url: str) -> None:
    def _open() -> None:
        time.sleep(0.6)
        try:
            webbrowser.open(url)
        except Exception:
            return

    threading.Thread(target=_open, daemon=True).start()


def main() -> None:
    host = os.environ.get("RANDOM_MISSION_HOST", "127.0.0.1")
    preferred_port = int(os.environ.get("RANDOM_MISSION_PORT", "8765") or "8765")
    port = _pick_port(host, preferred_port)
    paths.set_db_root(DB_ROOT)
    _refresh_config_dependent_modules()
    _sync_rtv_scenario_archive()

    server = RandomMissionHTTPServer((host, port), RequestHandler)
    url = f"http://{host}:{port}/"
    print(f"Random_mission web UI listening on {url}")
    if os.environ.get("RANDOM_MISSION_OPEN_BROWSER", "0").lower() not in {"0", "false", "no"}:
        _open_browser_later(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
