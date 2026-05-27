from __future__ import annotations

"""
FlightReferenceInfo 생성기.

규칙 요약:
- 파일명: 0001.json 처럼 4자리 오름차순.
- missionReferencePackageID: 1부터 오름차순.
- timestamp/inputTimestamp: 2000-01-01 UTC 기준 ms.
- takeOverInfoList: UAV ID 4,5,6을 정삼각형(변 150m, 북향)으로 배치. 기준점은 시작 참조점 중 랜덤 선택.
- handOverInfoList: 각 UAV 시작점에서 동/서 랜덤 방향으로 300m 이동.
- rtbCoordinateList: handOver에서 선택한 방향의 반대편으로 300m 이동.
- flightAreaList: 자동임무 생성 구역 그대로, 고도 0~5000.
- prohibitedAreaList: 자동임무 생성 구역 바깥 서쪽에 200~400m 떨어진 위치에 반경 250m 오각형 2개, 고도 0~5000.
"""

import json
import math
import random
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

if __name__ == "__main__" and __package__ is None:
    # 패키지 외부 실행 시 상위 경로를 sys.path에 추가해 relative import 오류 방지
    import sys

    sys.path.append(str(Path(__file__).resolve().parent.parent))
    __package__ = "fpl_random"

from .areas import AUTO_MISSION_AREA, START_REFERENCE_POINTS, LatLon
from . import config, dem, paths
from .utils import now_ms_2000, offset_lat_lon

PROJECT_ROOT = Path(__file__).resolve().parent.parent  # 모듈 루트(FPL_Random).

# 생성 파라미터는 config.py에서 관리한다.


def _db_dir() -> Path:
    return paths.db_root() / "MissionReferenceInfo"


def _next_ids() -> Tuple[int, int]:
    """(missionReferencePackageID, file_seq)를 반환."""
    current = _existing_max_seq()
    seq = current + 1
    return seq, seq


def _existing_max_seq() -> int:
    max_seq = 0
    dir_path = _db_dir()
    roots = []
    if dir_path.exists():
        roots.extend(dir_path.glob("*.json"))
    bundle_root = paths.db_root()
    roots.extend(bundle_root.glob("Random_Scenario_*/*/MissionReferenceInfo/*.json"))
    for p in roots:
        seq = _extract_seq(p.stem)
        if seq is None:
            continue
        if seq > max_seq:
            max_seq = seq
    return max_seq


def _extract_seq(stem: str) -> int | None:
    if not stem:
        return None
    if stem.isdigit():
        return int(stem)
    match = re.search(r"(\d+)$", stem)
    if not match:
        return None
    try:
        return int(match.group(1))
    except Exception:
        return None


def _bootstrap_state_if_needed() -> None:
    # 상태 파일은 더 이상 사용하지 않으며, 기존 파일명에서 시퀀스를 계산한다.
    return None


def _triangle_vertices(center: LatLon) -> Tuple[LatLon, LatLon, LatLon]:
    """
    centroid를 기준으로 북향 정삼각형(변=config.SIDE_M) 좌표를 계산.
    좌표계: x=동(+), y=북(+)
    """
    h = math.sqrt(3) / 2 * config.SIDE_M
    v1 = (0.0, 2 * h / 3)  # 북쪽 꼭짓점
    v2 = (-config.SIDE_M / 2, -h / 3)
    v3 = (config.SIDE_M / 2, -h / 3)
    verts = []
    for east, north in (v1, v2, v3):
        lat, lon = offset_lat_lon(center.latitude, center.longitude, east, north)
        verts.append(LatLon(lat, lon))
    return tuple(verts)  # type: ignore


def _handover_point(start: LatLon, east_first: bool) -> LatLon:
    east_m = config.HANDOVER_OFFSET_M if east_first else -config.HANDOVER_OFFSET_M
    lat, lon = offset_lat_lon(start.latitude, start.longitude, east_m, 0.0)
    return LatLon(lat, lon)


def _rtb_point(start: LatLon, east_first: bool) -> LatLon:
    east_m = -config.RTB_OFFSET_M if east_first else config.RTB_OFFSET_M
    lat, lon = offset_lat_lon(start.latitude, start.longitude, east_m, 0.0)
    return LatLon(lat, lon)


def _agl_altitude(point: LatLon) -> int:
    return dem.altitude_agl_m(point.latitude, point.longitude, config.TAKEOVER_ALT_AGL_M)


def _prohibited_pentagon(rng: random.Random) -> List[Dict[str, float]]:
    sw = AUTO_MISSION_AREA.southwest
    ne = AUTO_MISSION_AREA.northeast
    mid_lat = (sw.latitude + ne.latitude) / 2
    mid_lon = (sw.longitude + ne.longitude) / 2

    # 구역 서쪽으로 200~400m + 절반 폭만큼 이동시켜 FlightArea와 겹치지 않도록 함.
    width_m = _lon_span_m(sw.latitude, ne.latitude, sw.longitude, ne.longitude)
    height_m = abs(ne.latitude - sw.latitude) * 111_320.0
    half_diag = math.hypot(width_m, height_m) / 2.0
    offset_base = (
        half_diag
        + config.PROHIBITED_RADIUS_M
        + rng.uniform(config.PROHIBITED_OFFSET_MIN, config.PROHIBITED_OFFSET_MAX)
    )
    bearing = math.radians(rng.uniform(0.0, 360.0))
    east = offset_base * math.sin(bearing)
    north = offset_base * math.cos(bearing)
    center_lat, center_lon = offset_lat_lon(mid_lat, mid_lon, east, north)

    pts: List[Dict[str, float]] = []
    start_bearing = rng.uniform(0, 360)
    for i in range(5):
        bearing = math.radians(start_bearing + i * 72.0)
        east = config.PROHIBITED_RADIUS_M * math.sin(bearing)
        north = config.PROHIBITED_RADIUS_M * math.cos(bearing)
        lat, lon = offset_lat_lon(center_lat, center_lon, east, north)
        pts.append({"latitude": lat, "longitude": lon})
    return pts


def _lon_span_m(lat0: float, lat1: float, lon0: float, lon1: float) -> float:
    mid_lat = (lat0 + lat1) / 2
    lon_diff = abs(lon1 - lon0)
    return lon_diff * 111_320.0 * math.cos(math.radians(mid_lat))


def generate(
    seed: int | None = None,
    *,
    base_point: Optional[LatLon] = None,
    handover_left: Optional[bool] = None,
) -> Dict:
    """
    FlightReferenceInfo JSON 객체 생성.
    """
    _bootstrap_state_if_needed()
    mission_id, file_seq = _next_ids()
    rng = random.Random(seed)

    base_point = base_point or rng.choice(START_REFERENCE_POINTS)
    uav_points = _triangle_vertices(base_point)

    # 전체 편대 기준으로 좌/우 랜덤 선택 (모든 UAV 동일 방향)
    if handover_left is None:
        handover_left = rng.choice([True, False])  # True=서쪽(좌), False=동쪽(우)

    # UAV IDs와 좌표 매핑

    take_over = []
    hand_over = []
    rtb_points = []
    for aircraft_id, pt in zip(config.UAV_IDS, uav_points):
        hand_pt = _handover_point(pt, east_first=not handover_left)  # east_first False->서쪽, True->동쪽
        rtb_pt = _rtb_point(pt, east_first=not handover_left)        # RTB는 반대 방향(함수 내부에서 부호 반전)
        take_alt = _agl_altitude(pt)
        hand_alt = _agl_altitude(hand_pt)
        rtb_alt = _agl_altitude(rtb_pt)
        take_over.append(
            {"aircraftID": aircraft_id, "coordinate": {"latitude": pt.latitude, "longitude": pt.longitude, "altitude": take_alt}}
        )
        hand_over.append(
            {"aircraftID": aircraft_id, "coordinate": {"latitude": hand_pt.latitude, "longitude": hand_pt.longitude, "altitude": hand_alt}}
        )
        rtb_points.append({"latitude": rtb_pt.latitude, "longitude": rtb_pt.longitude, "altitude": rtb_alt})

    # Flight area
    flight_area = {
        "flightAreaID": 1,
        "areaLatLonList": AUTO_MISSION_AREA.to_area_lat_lon_list(),
        "altitudeLimits": {"lowerLimit": 0, "upperLimit": 5000},
    }

    # Prohibited area
    prohibited_area_list = []
    for idx in range(config.PROHIBITED_AREA_COUNT):
        prohibited_area_list.append(
            {
                "prohibitedAreaID": idx + 1,
                "areaLatLonList": _prohibited_pentagon(rng),
                "altitudeLimits": {"lowerLimit": 0, "upperLimit": 5000},
            }
        )

    timestamp = now_ms_2000()

    return {
        "timestamp": timestamp,
        "missionReferencePackageID": mission_id,
        "inputTimestamp": timestamp,
        "takeOverInfoList": take_over,
        "handOverInfoList": hand_over,
        "rtbCoordinateList": rtb_points,
        "flightAreaList": [flight_area],
        "prohibitedAreaList": prohibited_area_list,
        "_meta": {"fileSeq": file_seq, "seed": seed},
    }


def save(payload: Dict) -> Path:
    """생성된 객체를 파일로 저장하고 경로를 반환."""
    dir_path = _db_dir()
    dir_path.mkdir(parents=True, exist_ok=True)
    meta = payload.get("_meta", {})
    file_seq = meta.get("fileSeq") or payload.get("missionReferencePackageID", 1)
    path = dir_path / f"{int(file_seq):04d}.json"
    payload = dict(payload)
    payload.pop("_meta", None)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path


if __name__ == "__main__":
    obj = generate()
    out = save(obj)
    print(f"generated: {out}")
