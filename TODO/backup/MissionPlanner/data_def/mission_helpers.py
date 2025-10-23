"""
mission_helpers.py
공통 유틸리티 · 지도-JS 브릿지 · 간단한 ‘임무 메타’ 다이얼로그
"""

import os, random, folium, json
from branca.colormap import linear
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import (QDialog, QGridLayout, QLabel, QComboBox,
                             QDialogButtonBox, QDoubleSpinBox)
from folium import CircleMarker   # 파일 맨 위 import
import math
import rasterio
from rasterio.transform import array_bounds
from data_def.id_allocator import next_individual_mission_id, next_path_id


_DEM_PATH = r"resources\n38_e127_1arc_v3.dt2"   # 실제 DEM 경로

def terrain_elev(lat: float, lon: float) -> float:
    """위·경도(°) → DEM 고도(m).  범위 밖이면 0."""
    with rasterio.open(_DEM_PATH, "r") as src:
        h, w = src.height, src.width
        tr   = src.transform
        left, bottom, right, top = array_bounds(h, w, tr)
        if not (bottom <= lat <= top and left <= lon <= right):
            return 0.0
        col_f, row_f = (~tr) * (lon, lat)
        row = int(max(0, min(h-1, round(row_f))))
        col = int(max(0, min(w-1, round(col_f))))
        return float(src.read(1)[row, col])
    
# ────────────────── 데이터·랜덤 헬퍼 ──────────────────
def rand_coord() -> dict:
    """임의 좌표 (위·경·고도) 하나 생성"""
    return {
        "latitude":  round(random.uniform(-90,  90), 6),
        "longitude": round(random.uniform(-180, 180), 6),
        "altitude":  round(random.uniform(50,  500), 1),
    }

def _corridor_polygon(path_ll, width_m):
    """
    path_ll : [(lat, lon), ...]   (최소 2점)
    width_m : 전체 폭 [m]
    반환     : corridor 바깥 경계 좌표 리스트 (Polygon)
    - 근사치 : 소규모 지역이므로 평면으로 간주
    """
    half = width_m / 2.0
    poly_left  = []
    poly_right = []

    for i in range(len(path_ll)-1):
        lat1, lon1 = path_ll[i]
        lat2, lon2 = path_ll[i+1]

        # 단위 벡터 (동-북)
        dx = (lon2 - lon1) * 111_000 * math.cos(math.radians((lat1+lat2)/2))
        dy = (lat2 - lat1) * 111_000
        L  = math.hypot(dx, dy)
        if L == 0:  # 동일 점
            continue
        ux, uy = dx/L, dy/L
        # 좌/우 수직 방향 (CW, CCW)
        px, py =  uy, -ux

        # 좌우 offset (단위: 위도/경도)
        dlat = (py * half) / 111_000
        dlon = (px * half) / (111_000 * math.cos(math.radians(lat1)))

        # 세그먼트 시작, 끝 점 offset
        left_start   = (lat1 + dlat, lon1 + dlon)
        right_start  = (lat1 - dlat, lon1 - dlon)
        left_end     = (lat2 + dlat, lon2 + dlon)
        right_end    = (lat2 - dlat, lon2 - dlon)

        if i == 0:
            poly_left.append(left_start)
            poly_right.append(right_start)
        poly_left.append(left_end)
        poly_right.append(right_end)

    return poly_left + poly_right[::-1]   # 폐곡선

def make_individual_mission(tmp_idx: int | None = None) -> dict:
    """
    ▣ 새 Individual Mission 기본 골격을 만든다.
      · IndividualMissionID  : 900 000 001~  (id_allocator.next_individual_mission_id)
      · PathID               : 0  (= 미정 → 이후 aircraftID 확정 시 next_path_id(aid)로 덮어쓰기)
    """
    im_id = next_individual_mission_id()

    return {
        "individualMissionID": im_id,          # uint32 순차 ID
        "isDone": False,
        "relatedMission": {
            "relatedMissionType": 0,
            "inputMissionID":    0,
            "priorMissionID":    0,
        },
        "individualMissionInfo": {
            "individualMissionType": 0,        # 0 = 미지정
            "patternType":          0,
            "autoZoomIn":           True,
            "coordinateList":       [],
            "lineList":             [],
            "areaList":             [],
            "targetID":             0,
        },
        "pathID": 0,                           # ★ aircraftID 확정 뒤에 덮어쓴다
    }

def add_mission_shapes(fmap, missions):
    color_scale = linear.Set1_09.scale(0, 8)  # 최대 9종 색
    ac_colors = {}

    # 현재 임무 목록을 순차적으로 돌며, 각 임무의 도형과 점들을 지도에 추가합니다.
    for m in missions:
        aid = m.get("aircraftID", 0)
        if aid not in ac_colors:
            ac_colors[aid] = color_scale(len(ac_colors))
        color = ac_colors[aid]

        info = m["individualMissionInfo"]
        mtype = info["individualMissionType"]

        # 1) 영역 수색 -> Polygon
        if mtype == 1 and info.get("areaList"):
            for area in info["areaList"]:
                coords = [(c["latitude"], c["longitude"]) for c in area["coordinateList"]]
                if len(coords) >= 3:  # Polygon은 최소 3점 이상이어야 함
                    folium.Polygon(locations=coords,
                                   color=color, weight=2,
                                   fill=True, fill_opacity=0.2,
                                   tooltip=f"A/C {aid} : Area").add_to(fmap)
                # 영역 수색에서 점 마커 추가
                for p in coords:
                    folium.CircleMarker(p, radius=4, color=color,
                                         fill=True, fill_opacity=0.9).add_to(fmap)

        # 2) 통로 정찰 -> Line + 폭 표시(간단하게 PolyLine 두께로 표현)
        elif mtype == 2 and info.get("lineList"):
            for line in info["lineList"]:
                # ── ① 좌표 추출 ─────────────────────────
                coords = [(c["latitude"], c["longitude"])
                        for c in line["coordinateList"]]

                if len(coords) < 2:          # 좌표가 2개 미만이면 skip
                    continue

                w_m = line["width"]

                # ── ② 폭(m) → Polygon 면적 생성 ────────
                corridor_poly = _corridor_polygon(coords, w_m)
                folium.Polygon(
                    locations=corridor_poly,
                    color=color, weight=1,
                    fill=True, fill_opacity=0.2,
                    tooltip=f"A/C {aid} : {w_m} m Corridor"
                ).add_to(fmap)

                # ── ③ 중심선 & 마커 시각화 ─────────────
                folium.PolyLine(coords, color=color,
                                weight=2, dash_array="4,4").add_to(fmap)
                for lat, lon in coords:
                    folium.CircleMarker([lat, lon], radius=4,
                                        color=color, fill=True,
                                        fill_opacity=0.9).add_to(fmap)

        # 3) 이동 -> 궤적 연결선
        if mtype == 3 and info.get("coordinateList"):
            coords = [(c["latitude"], c["longitude"]) for c in info["coordinateList"]]
            if len(coords) >= 2:  # Line으로 연결된 경로여야 하므로 최소 2점 이상이어야 함
                folium.PolyLine(locations=coords,
                                color=color, weight=3,
                                dash_array="5,10",
                                tooltip=f"A/C {aid} : Route").add_to(fmap)
            # 이동에서 점 마커 추가
            for p in coords:
                folium.CircleMarker(p, radius=4, color=color,
                                     fill=True, fill_opacity=0.9).add_to(fmap)

# ────────────────── 지도 ↔ Python 브릿지 ──────────────────
class MapBridge(QObject):
    pointClicked = pyqtSignal(float, float)

    @pyqtSlot(float, float)
    def sendPoint(self, lat, lon):
        self.pointClicked.emit(lat, lon)

bridge = MapBridge()  # 단일 인스턴스 사용

# ────────────────── 임무 메타 다이얼로그 ──────────────────
class MissionMetaDialog(QDialog):
    def __init__(self, next_id: int, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Mission Info")
        self.resize(300, 150)

        lay = QGridLayout(self)
        lay.addWidget(QLabel(f"IndividualMissionID: {next_id}"), 0, 0, 1, 2)

        lay.addWidget(QLabel("Mission Type"), 1, 0)
        self.cmb = QComboBox()
        self.cmb.addItems(["Area Search (4 pts)", "Corridor (3 pts)", "Move (5 pts)"])
        lay.addWidget(self.cmb, 1, 1)

        lay.addWidget(QLabel("Width (m)"), 2, 0)
        self.spin = QDoubleSpinBox()
        self.spin.setRange(1, 1000)
        self.spin.setValue(100)
        lay.addWidget(self.spin, 2, 1)
        self.spin.setEnabled(False)

        self.cmb.currentIndexChanged.connect(
            lambda i: self.spin.setEnabled(i == 1)
        )

        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        lay.addWidget(bb, 3, 0, 1, 2)

    def get_result(self):
        typ = self.cmb.currentIndex()
        need = {0: 4, 1: 3, 2: 5}[typ]
        width = self.spin.value() if typ == 1 else None
        return typ, need, width


# mission_helpers.py  (맨 아래에 추가)
from datetime import datetime, timezone

def now_ms_since_2000() -> int:
    """2000-01-01 00:00:00 UTC 기준 경과 millisecond"""
    epoch2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
    return int((datetime.utcnow().replace(tzinfo=timezone.utc) - epoch2000).total_seconds() * 1000)
