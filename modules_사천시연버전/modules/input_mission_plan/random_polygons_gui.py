# random_polygons_gui.py
# -*- coding: utf-8 -*-
"""
랜덤 LLA 다각형들을 Folium 지도로 시각화하는 PyQt5 GUI
- 원형 없음, 다각형만 생성
- 좌측: 맵(QWebEngineView), 우측: 파라미터/버튼
필요: PyQt5, PyQtWebEngine, folium
실행: python random_polygons_gui.py
"""
from __future__ import annotations
import os, math, random, tempfile
from typing import List, Tuple
from pathlib import Path

import folium
from PyQt5.QtCore import Qt, QUrl
from PyQt5.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QDoubleSpinBox,
    QSpinBox, QPushButton, QFormLayout
)
from PyQt5.QtWebEngineWidgets import QWebEngineView

# 좌표 변환 (프로젝트의 coord_transform.py와 동일식)
EARTH_RADIUS = 6378137.0

def xy_to_llh(x: float, y: float, lat0: float, lon0: float) -> Tuple[float, float]:
    d_lat = y / EARTH_RADIUS
    d_lon = x / (EARTH_RADIUS * math.cos(math.radians(lat0)))
    lat = math.degrees(d_lat) + lat0
    lon = math.degrees(d_lon) + lon0
    return lat, lon

class RandomPolygonGUI(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Random LLA Polygons – Demo GUI")
        self.resize(1200, 800)

        # 기본 파라미터
        self.center_lat = 37.5
        self.center_lon = 127.0
        self.span_e_m   = 15_000.0   # 동서 ±범위 [m]
        self.span_n_m   = 15_000.0   # 남북 ±범위 [m]
        self.n_polys    = 12
        self.v_min      = 3
        self.v_max      = 7
        self.r_min_m    = 250.0
        self.r_max_m    = 1200.0
        self.zoom_start = 12

        root = QHBoxLayout(self)

        # (1) 맵
        self.view = QWebEngineView()
        root.addWidget(self.view, 3)

        # (2) 컨트롤 패널
        side = QVBoxLayout()
        root.addLayout(side, 1)

        form = QFormLayout()
        side.addLayout(form)

        self.spin_lat = QDoubleSpinBox(); self.spin_lat.setDecimals(6); self.spin_lat.setRange(-90, 90); self.spin_lat.setValue(self.center_lat)
        self.spin_lon = QDoubleSpinBox(); self.spin_lon.setDecimals(6); self.spin_lon.setRange(-180, 180); self.spin_lon.setValue(self.center_lon)
        self.spin_span_e = QDoubleSpinBox(); self.spin_span_e.setRange(0.1, 100.0); self.spin_span_e.setSuffix(" km"); self.spin_span_e.setValue(self.span_e_m/1000.0)
        self.spin_span_n = QDoubleSpinBox(); self.spin_span_n.setRange(0.1, 100.0); self.spin_span_n.setSuffix(" km"); self.spin_span_n.setValue(self.span_n_m/1000.0)
        self.spin_npoly  = QSpinBox();      self.spin_npoly.setRange(1, 200); self.spin_npoly.setValue(self.n_polys)
        self.spin_vmin   = QSpinBox();      self.spin_vmin.setRange(3, 50);   self.spin_vmin.setValue(self.v_min)
        self.spin_vmax   = QSpinBox();      self.spin_vmax.setRange(3, 50);   self.spin_vmax.setValue(self.v_max)
        self.spin_rmin   = QDoubleSpinBox(); self.spin_rmin.setRange(10, 20_000); self.spin_rmin.setSuffix(" m"); self.spin_rmin.setValue(self.r_min_m)
        self.spin_rmax   = QDoubleSpinBox(); self.spin_rmax.setRange(10, 30_000); self.spin_rmax.setSuffix(" m"); self.spin_rmax.setValue(self.r_max_m)

        form.addRow("Center Lat", self.spin_lat)
        form.addRow("Center Lon", self.spin_lon)
        form.addRow("Span East ±", self.spin_span_e)
        form.addRow("Span North ±", self.spin_span_n)
        form.addRow("Polygons", self.spin_npoly)
        form.addRow("Vertices min", self.spin_vmin)
        form.addRow("Vertices max", self.spin_vmax)
        form.addRow("Radius min", self.spin_rmin)
        form.addRow("Radius max", self.spin_rmax)

        # 버튼들
        self.btn_generate = QPushButton("Generate Polygons")
        side.addWidget(self.btn_generate)
        side.addStretch(1)

        self.btn_generate.clicked.connect(self._on_generate)

        # 최초 맵 렌더
        self._on_generate()

    # ─────────────────── Logic ───────────────────
    def _on_generate(self):
        # 파라미터 갱신
        self.center_lat = self.spin_lat.value()
        self.center_lon = self.spin_lon.value()
        self.span_e_m   = self.spin_span_e.value() * 1000.0
        self.span_n_m   = self.spin_span_n.value() * 1000.0
        self.n_polys    = self.spin_npoly.value()
        self.v_min      = self.spin_vmin.value()
        self.v_max      = max(self.v_min, self.spin_vmax.value())
        self.r_min_m    = self.spin_rmin.value()
        self.r_max_m    = max(self.r_min_m, self.spin_rmax.value())

        # ★ areas 리스트: 입력 순서 = 방문 순서
        areas = self._generate_polygons(
            n_polys=self.n_polys,
            vmin=self.v_min, vmax=self.v_max,
            rmin=self.r_min_m, rmax=self.r_max_m
        )

        html_path = self._make_map_html(areas)
        self.view.setUrl(QUrl.fromLocalFile(html_path))

    def _rand_center(self) -> Tuple[float, float]:
        dx = random.uniform(-self.span_e_m, self.span_e_m)
        dy = random.uniform(-self.span_n_m, self.span_n_m)
        lat, lon = xy_to_llh(dx, dy, self.center_lat, self.center_lon)
        return lat, lon

    def _convex_poly_at(self, lat: float, lon: float, n_vert: int, rmin: float, rmax: float) -> List[Tuple[float, float]]:
        # 균일 각도 샘플 후 정렬 → 반경 랜덤 → 회전
        angs = sorted([random.uniform(0.0, 360.0) for _ in range(n_vert)])
        rads = [random.uniform(rmin, rmax) for _ in range(n_vert)]
        pts_xy = [(rads[i]*math.cos(math.radians(angs[i])),
                   rads[i]*math.sin(math.radians(angs[i]))) for i in range(n_vert)]
        rot = random.uniform(0.0, 180.0)
        c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
        pts_xy = [(x*c - y*s, x*s + y*c) for (x,y) in pts_xy]
        # ENU → LLA
        pts_lla = [xy_to_llh(x, y, lat, lon) for (x,y) in pts_xy]
        return pts_lla

    def _generate_polygons(self, n_polys: int, vmin: int, vmax: int,
                        rmin: float, rmax: float) -> list:
        """
        반환 형식 (입력 순서 = 방문 순서):
        [
        {
            "order": 1,                        # ★ 방문 순서(1-base)
            "name":  "AREA-01",
            "coordinateList": [                # folium/외부 모듈 재사용 쉽게 dict로 보관
            {"latitude": lat, "longitude": lon, "altitude": 100},
            ...
            ]
        },
        ...
        ]
        """
        areas = []
        for i in range(n_polys):
            lat0, lon0 = self._rand_center()
            n_vert = max(3, min(50, random.randint(vmin, vmax)))
            # 다각형 꼭짓점(LLA) 생성
            verts_lla = self._convex_poly_at(lat0, lon0, n_vert, rmin, rmax)

            coord_list = [
                {"latitude": lat, "longitude": lon, "altitude": 100.0}
                for (lat, lon) in verts_lla
            ]
            areas.append({
                "order": i + 1,                         # ★ 입력 순서 그대로
                "name": f"AREA-{i+1:02d}",
                "coordinateList": coord_list
            })
        return areas

    def _make_map_html(self, areas: list) -> str:
        import tempfile
        from pathlib import Path
        fmap = folium.Map(location=[self.center_lat, self.center_lon], zoom_start=self.zoom_start)

        palette = [
            "#e6194b","#3cb44b","#0082c8","#f58231","#911eb4","#46f0f0",
            "#f032e6","#d2f53c","#fabebe","#008080","#e6beff","#aa6e28",
            "#fffac8","#800000","#aaffc3","#808000","#ffd8b1","#000080",
        ]

        for i, area in enumerate(areas):
            coords_ll = [(p["latitude"], p["longitude"]) for p in area["coordinateList"]]
            color = palette[i % len(palette)]

            # 폴리곤 자체
            folium.Polygon(
                locations=coords_ll,
                color=color, weight=2,
                fill=True, fill_opacity=0.25,
                tooltip=f'{area["name"]} | order={area["order"]}'
            ).add_to(fmap)

            # ★ 방문 순서 라벨(숫자) – 지도에 바로 보이게
            cy, cx = self._centroid(coords_ll)
            folium.map.Marker(
                location=(cy, cx),
                icon=folium.DivIcon(
                    html=f'''
                    <div style="
                        background: rgba(0,0,0,0.65);
                        color: #fff;
                        border-radius: 12px;
                        padding: 2px 8px;
                        font-size: 12px;
                        border: 1px solid rgba(255,255,255,0.6);
                        text-align:center;
                    ">{area["order"]}</div>'''
                )
            ).add_to(fmap)

        tmp = Path(tempfile.gettempdir()) / "random_polygons_map.html"
        fmap.save(str(tmp))
        return str(tmp)
    
    def _centroid(self, coords_ll: list) -> tuple:
        """
        볼록 다각형 중심(간단 평균) – 시각화 라벨용.
        coords_ll: [(lat, lon), ...]
        """
        if not coords_ll:
            return (self.center_lat, self.center_lon)
        s_lat = sum(lat for lat, _ in coords_ll)
        s_lon = sum(lon for _, lon in coords_ll)
        n = len(coords_ll)
        return (s_lat / n, s_lon / n)

if __name__ == "__main__":
    app = QApplication([])
    gui = RandomPolygonGUI()
    gui.show()
    app.exec()
