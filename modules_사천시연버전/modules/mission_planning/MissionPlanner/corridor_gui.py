# corridor_gui.py
"""
Corridor 입력 GUI + 세그먼트 폭 기반 단일 통로(Union) 시각화
────────────────────────────────────────────────────────────
· Folium 지도에 3점 클릭 → 중심선 확보
· 세그먼트별 폭[m] 입력 → 각 세그먼트를 폭/2 버퍼한 네모를 생성
· 두 네모가 접하거나 겹치면 Shapely union 으로 하나의 Corridor 폴리곤으로 표시
· 'Save' 클릭 시 포인트·폭 JSON 을 콘솔로 출력
필수 : PyQt5-QtWebEngine, folium, numpy, shapely
"""

import os, sys, json, math, folium
from pathlib import Path
import numpy as np
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QUrl
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QDoubleSpinBox, QPushButton, QMessageBox
)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel

_PROJECT_ROOT = Path(__file__).resolve().parents[3]
_TEMP_DIR = _PROJECT_ROOT / "temp"


def _map_html_path() -> Path:
    _TEMP_DIR.mkdir(parents=True, exist_ok=True)
    return _TEMP_DIR / "map.html"


# ───────────── Map ↔ Python 브릿지 ─────────────
class MapBridge(QObject):
    pointClicked = pyqtSignal(float, float)

    @pyqtSlot(float, float)
    def sendPoint(self, lat, lon):
        self.pointClicked.emit(lat, lon)


# ───────────── 좌표 보조 함수 ─────────────
def offset_latlon(lat, lon, dx_east_m, dy_north_m):
    """위·경도 (deg)에 동(dx), 북(dy) 오프셋(m) 적용한 새 위·경도 반환"""
    dlat = dy_north_m / 111_320.0
    dlon = dx_east_m / (111_320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def rectangle_for_segment(p1, p2, width_m):
    """두 위·경도 점(p1→p2)과 폭[m]으로 네모 모서리 4점 위·경도 반환"""
    lat1, lon1 = p1
    lat2, lon2 = p2
    # Δ(N, E) [m]
    dy = (lat2 - lat1) * 111_320.0
    dx = (lon2 - lon1) * 111_320.0 * math.cos(math.radians((lat1 + lat2) / 2))
    vec = np.array([dx, dy])
    if np.linalg.norm(vec) == 0:
        return []
    vec_unit = vec / np.linalg.norm(vec)
    perp = np.array([-vec_unit[1], vec_unit[0]])   # (E, N)
    half_w = width_m / 2.0
    corners = []
    for base in [np.array([0, 0]), vec]:
        for sign in [+1, -1]:
            e, n = base + perp * half_w * sign
            corners.append(offset_latlon(lat1, lon1, e, n))
    # 순서: p1+perp, p1-perp, p2-perp, p2+perp
    return [corners[0], corners[1], corners[3], corners[2]]


# ───────────── 메인 GUI ─────────────
class CorridorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Corridor Input GUI")
        self.resize(1200, 800)

        self.points = []          # [(lat, lon)]
        self.bridge = MapBridge()
        self._map_html_path = _map_html_path()

        root = QHBoxLayout(self)

        # (1) Folium 지도
        self.map_view = QWebEngineView()
        root.addWidget(self.map_view, 3)

        # (2) 우측 패널
        side = QVBoxLayout()
        root.addLayout(side, 1)

        lbl1 = QLabel("Width (P1→P2) [m]")
        self.spin12 = QDoubleSpinBox()
        self.spin12.setRange(1, 5000)
        lbl2 = QLabel("Width (P2→P3) [m]")
        self.spin23 = QDoubleSpinBox()
        self.spin23.setRange(1, 5000)
        for sp in (self.spin12, self.spin23):
            sp.setEnabled(False)
            sp.valueChanged.connect(self._update_polygons)

        self.btn_save = QPushButton("Save Corridor")
        self.btn_save.setEnabled(False)
        self.btn_save.clicked.connect(self._save_corridor)

        side.addWidget(lbl1)
        side.addWidget(self.spin12)
        side.addWidget(lbl2)
        side.addWidget(self.spin23)
        side.addStretch(1)
        side.addWidget(self.btn_save)

        self._write_map_html()
        self._connect_bridge()

    # ───── Folium 지도 HTML 생성 ─────
    def _write_map_html(self):
        center = [37.5, 127.0]
        fmap = folium.Map(location=center, zoom_start=12)
        fmap.save(str(self._map_html_path))

        # JS: 클릭 이벤트 & QWebChannel
        js = """
<script src="qrc:///qtwebchannel/qwebchannel.js"></script>
<script>
new QWebChannel(qt.webChannelTransport, c=>{
  const br=c.objects.bridge;
  let mp=null;
  for(let k in window){ if(window[k] instanceof L.Map){ mp=window[k]; break; } }
  if(mp){
     mp.on('click', e=>{ br.sendPoint(e.latlng.lat, e.latlng.lng); });
  }
});
</script>
"""
        with open(self._map_html_path, "r+", encoding="utf-8") as f:
            html = f.read()
            f.seek(0)
            f.write(html.replace("</body>", js + "</body>"))
            f.truncate()

        self.map_view.setUrl(QUrl.fromLocalFile(str(self._map_html_path.resolve())))

    # ───── QWebChannel 연결 ─────
    def _connect_bridge(self):
        ch = QWebChannel(self.map_view.page())
        ch.registerObject("bridge", self.bridge)
        self.map_view.page().setWebChannel(ch)
        self.bridge.pointClicked.connect(self._handle_point)

    # ───── 포인트 클릭 처리 ─────
    def _handle_point(self, lat, lon):
        if len(self.points) >= 3:
            QMessageBox.information(self, "Info", "이미 3점을 선택했습니다.")
            return

        self.points.append((lat, lon))
        idx = len(self.points)
        print(f"Point {idx}/3 : {lat:.6f}, {lon:.6f}")

        js = f"""
(function(){{
  var m=null;
  for(var k in window){{ if(window[k] instanceof L.Map){{ m=window[k]; break; }} }}
  if(m){{
     L.circleMarker([{lat},{lon}],{{radius:4,color:'red',fill:true}}).addTo(m);
     window.__pts=window.__pts||[];
     window.__pts.push([{lat},{lon}]);
     if(window.__pts.length>=2){{
        L.polyline(window.__pts.slice(-2),{{color:'red'}}).addTo(m);
     }}
  }}
}})();
"""
        self.map_view.page().runJavaScript(js)

        if len(self.points) == 3:
            self.spin12.setEnabled(True)
            self.spin23.setEnabled(True)
            self.btn_save.setEnabled(True)
            QMessageBox.information(self, "Info", "세 점 입력 완료! 각 폭을 설정하세요.")

    # ───── Corridor 네모/Union 갱신 ─────
    def _update_polygons(self):
        if len(self.points) != 3:
            return
        w12, w23 = self.spin12.value(), self.spin23.value()
        if w12 <= 0 or w23 <= 0:
            return

        rect12 = rectangle_for_segment(self.points[0], self.points[1], w12)
        rect23 = rectangle_for_segment(self.points[1], self.points[2], w23)

        # Shapely union
        p1 = Polygon([(lon, lat) for lat, lon in rect12]) if rect12 else None
        p2 = Polygon([(lon, lat) for lat, lon in rect23]) if rect23 else None

        polys = [p for p in (p1, p2) if p]
        if not polys:
            return
        union = unary_union(polys)                     # ① 기본 합집합

        # ② 작은 버퍼-클로징으로 코너 틈 메우기
        gap_tol = 1.0          # ⟵ 단위: meter (필요시 조정)
        union = union.buffer(gap_tol, join_style=2)\
                     .buffer(-gap_tol, join_style=2)

        # ── 지도 갱신 ──────────────────────────────────────
        js_clear = """
(function(){
  if(window.__corr_polys){
     window.__corr_polys.forEach(p=>p.remove());
  }
  window.__corr_polys=[];
})();"""
        self.map_view.page().runJavaScript(js_clear)

        # 단일·다중 Polygon 처리
        union_polys = [union] if isinstance(union, Polygon) else list(union.geoms)
        for poly in union_polys:
            coords = [(lat, lon) for lon, lat in poly.exterior.coords]
            pts = ",".join(f"[{lat},{lon}]" for lat, lon in coords)
            js = f"""
(function(){{
  var m=null;
  for(var k in window){{ if(window[k] instanceof L.Map){{ m=window[k];break;}}}}
  if(m){{
     var pg=L.polygon([{pts}],{{color:'gray',weight:1,fillOpacity:0.25}});
     pg.addTo(m);
     window.__corr_polys=window.__corr_polys||[];
     window.__corr_polys.push(pg);
  }}
}})();"""
            self.map_view.page().runJavaScript(js)



    # ───── 저장 ─────
    def _save_corridor(self):
        if len(self.points) != 3:
            QMessageBox.warning(self, "Warning", "3점을 먼저 선택하세요.")
            return
        w12, w23 = self.spin12.value(), self.spin23.value()
        if w12 <= 0 or w23 <= 0:
            QMessageBox.warning(self, "Warning", "폭 값을 입력하세요.")
            return
        data = {"points": self.points, "widths": [w12, w23]}
        print("\n=== Corridor Data ===")
        print(json.dumps(data, indent=2, ensure_ascii=False))
        QMessageBox.information(self, "Saved", "콘솔에 Corridor 데이터가 출력되었습니다.")


# ───────────── 실행 ─────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = CorridorGUI()
    gui.show()
    sys.exit(app.exec_())
