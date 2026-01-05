# main_app.py  ― 2025-06-16  직접 실행용(Option B)
import sys, os, time, json, math
import folium
from itertools import cycle
import shutil, tempfile
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLineEdit, QPushButton, QTextEdit, QTabWidget, QFrame,
    QComboBox, QDialog, QPlainTextEdit, QFileDialog, QMessageBox, QLabel, QCheckBox
)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtWidgets import QApplication


HERE = Path(__file__).resolve()            # .../modules/mission_planning/MissionPlanner/main_MP.py
PKG_DIR = HERE.parent                      # .../modules/mission_planning/MissionPlanner
if str(PKG_DIR) not in sys.path:
    # 패키지(-m)로 실행 시에도 로컬 모듈(data_def, AnS, corridor_planner) 임포트 가능하게
    sys.path.insert(0, str(PKG_DIR))

_PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))


# ───────── data_def 패키지 ──────────
_app_guard = QApplication.instance() or QApplication([])

from data_def import (
    mission_helpers as mh,
    d0301, d0302, d0303,
    d0304,          # ← 100 m 분할
    search_speed,
)
from data_def.mission_helpers import now_ms_since_2000
from data_def.mission_helpers import terrain_elev
import config as mp_config

import corridor_planner as cp    # 기존 사용 코드가 있으면 그대로 유지
from data_def.id_allocator import next_path_id

from AnS import (
    run_divide_and_pattern,     # 0201+0203 → IMP(.json) 리스트
    build_mission_plan_0301,    # CMPK+MRPK+IMP → 0301 MissionPlan
    get_last_divide_and_pattern_metrics,
)


def gui_logger(widget):
    """텍스트 위젯에 실시간 로그를 남기기 위한 래퍼"""
    def _log(msg: str):
        widget.appendPlainText(msg)
        QApplication.processEvents()     # ★ 화면 즉시 갱신
    return _log

def _html_kv(title: str, dic: dict) -> str:
    rows = "\n".join(
        f"<tr><th align='left'>{k}</th><td>{v}</td></tr>"
        for k, v in dic.items()
    )
    return f"<b>{title}</b><br><table>{rows}</table>"

# ????????????????????????????????????????????????????????????????????
class MainGUI(QWidget):
    BRIDGE_THRESH_M = 150.0
    CRUISE_SP       = 40.0

    # ★ 저장 루트: <프로젝트루트>\database
    SAVE_DIR = _PROJECT_ROOT / "database"

    # 하위 폴더들
    DIR_0201 = SAVE_DIR / "InputMissionPlan"
    DIR_0203 = SAVE_DIR / "MissionReferenceInfo"

    def __init__(self) -> None:
        super().__init__()
        # ── 새로운 임무 저장 전에 기존 파일 전체 삭제 ──
        for d in (self.SAVE_DIR, self.DIR_0201, self.DIR_0203):
            d.mkdir(parents=True, exist_ok=True)

        # ── Window ───────────────────────────────────────────────
        self.setWindowTitle("Mission Plan Generator")
        self.resize(1600, 900)

        # ── 내부 상태 ────────────────────────────────────────────
        self.aircraft_pool: list[dict] = []
        self.aircraft_file_path: str | None = None
        self.next_air      = 1
        self.missions:     list[dict] = []
        self.next_im       = 1
        self.pending       = None
        self.pending_pts   = []
        self.flight_plans        = []   # 0303
        self.flight_plans_0304   = []   # 0304
        self.wp_alloc = d0303._WPAllocator()

        self.imp_id_map: dict[int, int] = {}
        self.plan_pkg_id: int | None = None
        self.pkg0201_id = None   # ← 0201 InputMissionPackageID
        self.pkg0203_id = None   # ← 0203 MissionReferencePackageID

        self._visible_aircrafts: set[int] = set(range(1, 7))
        self._visible_0201 = True
        self._visible_0203 = True

        self._cmpk_data: dict | None = None
        self._mrpk_data: dict | None = None

        self._btn_0201: QPushButton | None = None
        self._btn_0203: QPushButton | None = None

        self._last_compute_ms_0302 = 0.0
        self._last_compute_ms_0303 = 0.0
        self._last_compute_ms_0304 = 0.0
        self._uav_turn_step_deg = 15.0


        # ── 레이아웃 ────────────────────────────────────────────
        root = QHBoxLayout(self)

        # 지도 영역 ------------------------------------------------
        self.map_frame = QFrame()
        self.map_lay   = QVBoxLayout(self.map_frame)
        root.addWidget(self.map_frame, 2)
        self._build_map()

        # 탭 영역 --------------------------------------------------
        self.tabs = QTabWidget()
        root.addWidget(self.tabs, 1)
        self._build_tab_uav_manager()
        self._build_tab_initial()
        self._build_tab_0301()
        self._build_tab_0302()
        self._build_tab_0303()
        self._build_tab_0304()

        # ── “plannedMission/임무계획” 기본 탐색 폴더 ─────────────
        base = Path(__file__).resolve().parent       # missionPlanner 디렉터리
        self.default_dir = base / "plannedMission" / "임무계획"
        self.default_dir.mkdir(parents=True, exist_ok=True)

        # 지도 상태(중심/줌) 기억
        self._map_view_state: dict | None = None

    def _init_save_dirs(self):
        """
        plannedMission/database 하위 필요한 폴더 자동 생성
        """
        for p in [
            self.SAVE_DIR,
            self.DIR_0201,
            self.DIR_0203,
            self.DIR_0301,
            self.DIR_0302,
            self.DIR_0303,
        ]:
            try:
                p.mkdir(parents=True, exist_ok=True)
            except Exception as e:
                print(f"[WARN] create dir failed: {p} -> {e}")

    # ───────────────────────── 지도 관련 ──────────────────────────
    def _build_map(self):
        self._write_map_html()
        self.map_view = QWebEngineView()
        ch = QWebChannel(self.map_view.page())
        ch.registerObject("bridge", mh.bridge)
        self.map_view.page().setWebChannel(ch)
        self.map_view.setUrl(QUrl.fromLocalFile(os.path.join(os.getcwd(), "map.html")))
        self.map_view.loadFinished.connect(self._on_map_load_finished)
        self.map_lay.addWidget(self.map_view)
        mh.bridge.pointClicked.connect(self._handle_point)

    def _rebuild_map(self):
        self._capture_map_view_state(self._reload_map_content)

    def _reload_map_content(self):
        self._write_map_html()
        try:
            self.map_view.reload()
        except Exception:
            pass

    def _capture_map_view_state(self, callback=None):
        """현재 지도 화면(center/zoom)을 저장한 뒤 callback 실행."""
        if not hasattr(self, "map_view"):
            if callback:
                callback()
            return

        script = """
            (function() {
                var map = null;
                for (var k in window) {
                    if (window[k] instanceof L.Map) { map = window[k]; break; }
                }
                if (!map) { return null; }
                var c = map.getCenter();
                return {lat: c.lat, lng: c.lng, zoom: map.getZoom()};
            })();
        """
        def _store_view(result):
            if isinstance(result, dict) and "lat" in result and "lng" in result:
                self._map_view_state = result
            if callback:
                callback()
        try:
            self.map_view.page().runJavaScript(script, _store_view)
        except Exception:
            if callback:
                callback()

    def _on_map_load_finished(self, ok: bool):
        if not ok or not self._map_view_state:
            return
        lat = float(self._map_view_state.get("lat", 0.0))
        lng = float(self._map_view_state.get("lng", 0.0))
        zoom = int(self._map_view_state.get("zoom", 14))
        script = f"""
            (function() {{
                var map = null;
                for (var k in window) {{
                    if (window[k] instanceof L.Map) {{ map = window[k]; break; }}
                }}
                if (map) {{
                    map.setView([{lat}, {lng}], {zoom});
                }}
            }})();
        """
        try:
            self.map_view.page().runJavaScript(script)
        except Exception:
            pass

    def _write_map_html(self):
        state = getattr(self, "_map_view_state", None) or {}
        try:
            center = [
                float(state.get("lat", 38.128774)),
                float(state.get("lng", 127.318005)),
            ]
            zoom = int(state.get("zoom", 14))
        except Exception:
            center = [38.128774, 127.318005]
            zoom = 14

        fmap = folium.Map(location=center, zoom_start=zoom)
        
        _js_links = []
        hover_specs = []

        # 작업 구역(예시 사각형) ---------------------------------
        folium.Rectangle(
            [[38.110432, 127.295620], [38.147111, 127.340401]],
            color="blue", weight=1, fill=True, fill_opacity=0.1
        ).add_to(fmap)

        # ── 0201 Input Mission (CMPK) ───────────────────────────
        if self._visible_0201 and self._cmpk_data:
            color_area = "#3949ab"
            color_line = "#00838f"
            color_point = "#1a237e"

            for miss in self._cmpk_data.get("inputMissionList") or []:
                mid = miss.get("inputMissionID")
                detail = miss.get("missionDetail") or {}
                mission_type = miss.get("inputMissionType")
                label = f"0201 IM {mid}"

                for idx_area, area in enumerate(detail.get("areaList") or []):
                    coords = []
                    for coord in area.get("coordinateList") or []:
                        lat = coord.get("latitude")
                        lon = coord.get("longitude")
                        if lat is None or lon is None:
                            continue
                        coords.append((lat, lon))
                    if len(coords) < 3:
                        continue
                    cls = f"cmpk_area_{mid}_{idx_area}"
                    popup = folium.Popup(
                        _html_kv("0201 Area", {
                            "missionID": mid,
                            "type": mission_type,
                            "isHole": area.get("isHole"),
                        }), max_width=260
                    )
                    poly = folium.Polygon(
                        coords, color=color_area, weight=2,
                        fill=True, fill_opacity=0.2,
                        popup=popup,
                        **{"className": cls}
                    )
                    poly.add_to(fmap)
                    folium.Tooltip(f"{label} - Area", sticky=False).add_to(poly)
                    hover_specs.append({"cls": cls, "kind": "path", "strokeWidth": 4})

                for idx_line, line in enumerate(detail.get("lineList") or []):
                    coords = []
                    for coord in line.get("coordinateList") or []:
                        lat = coord.get("latitude")
                        lon = coord.get("longitude")
                        if lat is None or lon is None:
                            continue
                        coords.append((lat, lon))
                    if len(coords) < 2:
                        continue
                    cls = f"cmpk_line_{mid}_{idx_line}"
                    popup = folium.Popup(
                        _html_kv("0201 Corridor", {
                            "missionID": mid,
                            "type": mission_type,
                            "width(m)": line.get("width"),
                        }), max_width=240
                    )
                    seg = folium.PolyLine(
                        coords, color=color_line, weight=3, dash_array="6,4",
                        popup=popup,
                        **{"className": cls}
                    )
                    seg.add_to(fmap)
                    folium.Tooltip(f"{label} - Line", sticky=False).add_to(seg)
                    hover_specs.append({"cls": cls, "kind": "path", "strokeWidth": 4})

                for idx_pt, coord in enumerate(detail.get("coordinateList") or [], 1):
                    lat = coord.get("latitude")
                    lon = coord.get("longitude")
                    if lat is None or lon is None:
                        continue
                    cls = f"cmpk_point_{mid}_{idx_pt}"
                    popup = folium.Popup(
                        _html_kv("0201 Point", {
                            "missionID": mid,
                            "type": mission_type,
                            "alt(m)": coord.get("altitude"),
                        }), max_width=220
                    )
                    folium.CircleMarker(
                        [lat, lon], radius=4, color=color_point,
                        fill=True, fill_opacity=1,
                        popup=popup,
                        tooltip=f"{label} - P{idx_pt}",
                        **{"className": cls}
                    ).add_to(fmap)
                    hover_specs.append({"cls": cls, "kind": "circle", "baseRadius": 4, "radiusMul": 1.6, "strokeWidth": 4})

        # ── 0302 개별 임무 도형 ───────────────────────────────
        for miss in self.missions:
            aid   = miss["aircraftID"]
            if aid not in self._visible_aircrafts:   # ★ 추가
               continue
            info  = miss.get("individualMissionInfo", {})
            color = miss.get("color") or {   # d0302에서 넣어준 필드
                1:"#e6194b", 2:"#3cb44b", 3:"#0082c8",
                4:"#f58231", 5:"#911eb4", 6:"#46f0f0"
            }.get(aid, "gray")

            # ─ 영역(polygons) ---------------------------------
            if info.get("areaList"):
                coords = [(c["latitude"], c["longitude"])
                          for c in info["areaList"][0]["coordinateList"]]
                popup = folium.Popup(                     # ★ NEW
                    _html_kv("Area Mission", {
                        "aircraft": aid,
                        "missionID": miss.get("individualMissionID"),
                        "type": info.get("individualMissionType"),
                        "pattern": info.get("patternType")
                    }), max_width=250)
                folium.Polygon(
                    coords, color=color, weight=2,
                    fill=True, fill_opacity=0.15,
                    popup=popup                          # ★ NEW
                ).add_to(fmap)

            # ─ 통로·선형(lineList) ----------------------------
            elif info.get("lineList"):
                coords = [(c["latitude"], c["longitude"])
                          for c in info["lineList"][0]["coordinateList"]]
                popup = folium.Popup(_html_kv("Corridor", {
                    "aircraft": aid,
                    "width(m)": info["lineList"][0]["width"]
                }), max_width=220)
                folium.PolyLine(
                    coords, color=color, weight=3, dash_array="4,4",
                    popup=popup                          # ★ NEW
                ).add_to(fmap)

            # 3) 이동 / 은엄폐 구분 ----------------------------------
            elif info.get("coordinateList"):
                pts = [(c["latitude"], c["longitude"]) for c in info["coordinateList"]]

                # ── ① 은엄폐( type-9 / pattern-12 ) ? 단일 점 ─────────
                if info.get("individualMissionType") == 9 and info.get("patternType") == 12:
                    popup = folium.Popup(                                # ★ NEW
                        _html_kv("Cover-and-Hide", {
                            "aircraft":  aid,
                            "missionID": miss.get("individualMissionID")
                        }), max_width=200)

                    folium.CircleMarker(
                        pts[0], radius=4, color=color, fill=True, fill_opacity=1,
                        popup=popup,                                         # ★ NEW
                        tooltip=f"IM {miss['individualMissionID']}"          # (선택)
                    ).add_to(fmap)

                # ── ② 이동( type-7 / pattern-10 ) ? 선 + 각 점 ────────
                else:
                    # 선 자체에도 팝업 하나 달아두기
                    seg_popup = folium.Popup(                             # ★ NEW
                        _html_kv("Move-Route", {
                            "aircraft":  aid,
                            "missionID": miss.get("individualMissionID"),
                            "points":    len(pts)
                        }), max_width=220)

                    folium.PolyLine(
                        pts, color=color, weight=2,
                        popup=seg_popup                                    # ★ NEW
                    ).add_to(fmap)

                    # 경유지 마커에 개별 팝업 / 툴팁
                    for idx, pt in enumerate(pts, 1):
                        wp_popup = folium.Popup(                          # ★ NEW
                            _html_kv(f"WP {idx}", {
                                "lat": f"{pt[0]:.6f}",
                                "lon": f"{pt[1]:.6f}"
                            }), max_width=200)

                        folium.CircleMarker(
                            pt, radius=3, color=color, fill=True, fill_opacity=1,
                            popup=wp_popup,                               # ★ NEW
                            tooltip=f"WP {idx}"                           # (선택)
                        ).add_to(fmap)

        # ── 0303 FlightPlan (aircraft 4·5·6) ──────────────────────────────
        COLOR3 = {4: "red", 5: "blue", 6: "brown"}

        for fp in self.flight_plans:                     # ← d0303 결과
            aid = fp["aircraftID"]
            if aid not in self._visible_aircrafts:       # 가시성 토글
                continue
            c = COLOR3.get(aid, "gray")

            # ─ 1) 전체 경로(점선) -------------------------------------------------
            pts = [(w["coordinate"]["latitude"], w["coordinate"]["longitude"])
                for w in fp["waypointList"]]
            line_cls = f"path3_{aid}_{fp['pathID']}"
            folium.PolyLine(
                pts, color=c, weight=2, dash_array="4,4",
                **{"className": line_cls}
            ).add_to(fmap)

            # ─ 2) 각 WP + scan-line(모드-2 lineSearch) ----------------------------
            for w in fp["waypointList"]:
                wp_id = w["waypointID"]

                # 2-A. WP 마커
                popup = folium.Popup(_html_kv(f"WP {wp_id}", {
                            "ETA(s)": w["eta"],
                            "ECF":     w["ecf"],
                            "speed":   w["speed"],
                        }), max_width=220)

                folium.CircleMarker(
                    [w["coordinate"]["latitude"], w["coordinate"]["longitude"]],
                    radius=3, color=c, fill=True, fill_opacity=1,
                    popup=popup,
                    tooltip=f"WP {wp_id}",                 # ← enumerate idx → 실제 ID
                    **{"className": f"wp_{wp_id}"}
                ).add_to(fmap)

                # 2-B. scan-line(해당 WP의 좌표를 순차 연결)
                fp = w.get("filmingProperty")
                ls = (fp or {}).get("lineSearch", {})
                coords = ls.get("coordinateList", [])
                seq = []
                for coord in coords:
                    lat = coord.get("latitude")
                    lon = coord.get("longitude")
                    if lat is None or lon is None:
                        continue
                    seq.append((lat, lon))
                if len(seq) >= 2:
                    ls_cls = f"ls_{aid}_{wp_id}_0"
                    folium.PolyLine(
                        seq,
                        color=c, weight=3, opacity=0.9, dash_array="5,4",
                        **{"className": ls_cls}
                    ).add_to(fmap)

                    _js_links.append((wp_id, ls_cls))     # WP ↔ scan-line 매핑

                # 2-C. WP ↔ 전체 경로(path3_) 매핑
                _js_links.append((wp_id, line_cls))

                    
        # ── 0304 시각화 (aircraft 1·2·3) ──────────────────────────────────
        COLOR4 = {1: "green", 2: "orange", 3: "purple"}
        for fp in self.flight_plans_0304:
            aid = fp["aircraftID"]
            if aid not in self._visible_aircrafts:
                continue
            c   = COLOR4.get(aid, "green")

            pts      = [(w["coordinate"]["latitude"], w["coordinate"]["longitude"])
                        for w in fp["lahWaypointList"]]
            line_cls = f"path4_{aid}_{fp['pathID']}"
            folium.PolyLine(pts, color=c, weight=2,
                            **{"className": line_cls}).add_to(fmap)

            for w in fp["lahWaypointList"]:
                wp_id = w["waypointID"]
                popup = folium.Popup(_html_kv(f"WP {wp_id}", {
                            "ETA(s)": w["eta"],
                            "ECF":     w["ecf"],
                            "speed":   w["speed"],
                        }), max_width=220)

                folium.CircleMarker(
                    [w["coordinate"]["latitude"], w["coordinate"]["longitude"]],
                    radius=3, color=c, fill=True, fill_opacity=1,
                    popup=popup,
                    tooltip=f"WP {wp_id}",
                    **{"className": f"wp_{wp_id}"}
                ).add_to(fmap)

                _js_links.append((wp_id, line_cls))     # JS 연동

        # ── 0203 Mission Reference overlays ─────────────────────────
        if self._visible_0203 and self._mrpk_data:
            mrpk = self._mrpk_data
            take_color = "#1e88e5"
            hand_color = "#fb8c00"
            rtb_color = "#6a1b9a"

            for idx_take, item in enumerate(mrpk.get("takeOverInfoList") or [], 1):
                coord = item.get("coordinate") or {}
                lat = coord.get("latitude")
                lon = coord.get("longitude")
                if lat is None or lon is None:
                    continue
                aid = item.get("aircraftID")
                cls = f"mrpk_take_{aid}_{idx_take}"
                popup = folium.Popup(
                    _html_kv("0203 Take-Over", {
                        "aircraft": aid,
                        "alt(m)": coord.get("altitude"),
                    }), max_width=240
                )
                folium.CircleMarker(
                    [lat, lon], radius=6, color=take_color,
                    fill=True, fill_opacity=1,
                    popup=popup,
                    tooltip=f"TakeOver A/C {aid}",
                    **{"className": cls}
                ).add_to(fmap)
                hover_specs.append({"cls": cls, "kind": "circle", "baseRadius": 6, "radiusMul": 1.4, "strokeWidth": 4})

            for idx_hand, item in enumerate(mrpk.get("handOverInfoList") or [], 1):
                coord = item.get("coordinate") or {}
                lat = coord.get("latitude")
                lon = coord.get("longitude")
                if lat is None or lon is None:
                    continue
                aid = item.get("aircraftID")
                cls = f"mrpk_hand_{aid}_{idx_hand}"
                popup = folium.Popup(
                    _html_kv("0203 Hand-Over", {
                        "aircraft": aid,
                        "alt(m)": coord.get("altitude"),
                    }), max_width=240
                )
                folium.CircleMarker(
                    [lat, lon], radius=6, color=hand_color,
                    fill=True, fill_opacity=1,
                    popup=popup,
                    tooltip=f"HandOver A/C {aid}",
                    **{"className": cls}
                ).add_to(fmap)
                hover_specs.append({"cls": cls, "kind": "circle", "baseRadius": 6, "radiusMul": 1.4, "strokeWidth": 4})

            for idx_rtb, coord in enumerate(mrpk.get("rtbCoordinateList") or [], 1):
                lat = coord.get("latitude")
                lon = coord.get("longitude")
                if lat is None or lon is None:
                    continue
                cls = f"mrpk_rtb_{idx_rtb}"
                popup = folium.Popup(
                    _html_kv("0203 RTB", {
                        "index": idx_rtb,
                        "alt(m)": coord.get("altitude"),
                    }), max_width=220
                )
                folium.CircleMarker(
                    [lat, lon], radius=7, color=rtb_color,
                    fill=True, fill_opacity=0.95,
                    popup=popup,
                    tooltip=f"RTB #{idx_rtb}",
                    **{"className": cls}
                ).add_to(fmap)
                hover_specs.append({"cls": cls, "kind": "circle", "baseRadius": 7, "radiusMul": 1.3, "strokeWidth": 4})

            for idx_area, area in enumerate(mrpk.get("flightAreaList") or [], 1):
                coords = []
                for coord in area.get("areaLatLonList") or []:
                    lat = coord.get("latitude")
                    lon = coord.get("longitude")
                    if lat is None or lon is None:
                        continue
                    coords.append((lat, lon))
                if len(coords) < 3:
                    continue
                poly = folium.Polygon(
                    coords, color="#2196f3", weight=1.5,
                    fill=True, fill_opacity=0.1,
                )
                poly.add_to(fmap)
                limits = area.get("altitudeLimits") or {}
                label = area.get("flightAreaID", idx_area)
                tooltip_text = f"FlightArea {label} ({limits.get('lowerLimit', '-')}-{limits.get('upperLimit', '-')} m)"
                folium.Tooltip(tooltip_text, permanent=True, direction='center', opacity=0.75).add_to(poly)

            for idx_proh, area in enumerate(mrpk.get("prohibitedAreaList") or [], 1):
                coords = []
                for coord in area.get("areaLatLonList") or []:
                    lat = coord.get("latitude")
                    lon = coord.get("longitude")
                    if lat is None or lon is None:
                        continue
                    coords.append((lat, lon))
                if len(coords) < 3:
                    continue
                poly = folium.Polygon(
                    coords, color="#e53935", weight=1.5,
                    fill=True, fill_opacity=0.08,
                    dash_array="4,4",
                )
                poly.add_to(fmap)
                limits = area.get("altitudeLimits") or {}
                label = area.get("prohibitedAreaID", idx_proh)
                tooltip_text = f"Prohibited {label} ({limits.get('lowerLimit', '-')}-{limits.get('upperLimit', '-')} m)"
                folium.Tooltip(tooltip_text, permanent=True, direction='center', opacity=0.75).add_to(poly)

        # ── HTML 저장 + WebChannel 스크립트 삽입 ───────────────
        path = os.path.join(os.getcwd(), "map.html")
        fmap.save(path)
        js = f"""
                    <script src="qrc:///qtwebchannel/qwebchannel.js"></script>
                    <script>
                    new QWebChannel(qt.webChannelTransport, function(ch){{    
                        const bridge = ch.objects.bridge;

                        /* 1) 지도 객체 */
                        let mp = null;
                        for (let k in window) {{ if (window[k] instanceof L.Map) {{ mp = window[k]; break; }} }}

                        /* 2) WP ↔ Line 클래스 매핑 */
                        const rawLinks = {json.dumps(_js_links)};       /*  ← 파이썬 값만 한 쌍 */
                        const wp2lines = Object.create(null);
                        const line2wp  = Object.create(null);
                        rawLinks.forEach(([wp, cls]) => {{
                            (wp2lines[wp] = wp2lines[wp] || []).push(cls);
                            line2wp[cls] = wp;
                        }});

                        const hoverSpecs = {json.dumps(hover_specs)};

                        /* 3) 강조 / 해제 (legacy for waypoint lineage) */
                        function highlight(cls, on) {{
                            document.querySelectorAll('.' + cls).forEach(el => {{
                                el.style.stroke      = on ? 'yellow' : '';
                                el.style.strokeWidth = on ? '4'      : '';
                                el.style.opacity     = on ? '1.0'    : '0.7';
                            }});
                        }}

                        /* 4) Hover highlight for map elements */
                        function bindHover() {{
                            hoverSpecs.forEach(spec => {{
                                document.querySelectorAll('.' + spec.cls).forEach(el => {{
                                    if (el.dataset.hoverReady) {{ return; }}
                                    el.dataset.hoverReady = '1';
                                    const tag = (el.tagName || '').toLowerCase();
                                    const isCircle = tag === 'circle';

                                    el.addEventListener('mouseenter', () => {{
                                        if (el.dataset.origStroke === undefined) {{
                                            const strokeAttr = el.getAttribute('stroke');
                                            el.dataset.origStroke = strokeAttr !== null ? strokeAttr : '';
                                            el.dataset.origStrokeStyle = el.style.stroke || '';
                                        }}
                                        if (el.dataset.origStrokeWidth === undefined) {{
                                            const swAttr = el.getAttribute('stroke-width');
                                            el.dataset.origStrokeWidth = swAttr !== null ? swAttr : '';
                                            el.dataset.origStrokeWidthStyle = el.style.strokeWidth || '';
                                        }}
                                        const highlightColor = spec.highlight || '#ffeb3b';
                                        const highlightWidth = spec.strokeWidth || 4;
                                        el.setAttribute('stroke', highlightColor);
                                        el.style.stroke = highlightColor;
                                        el.setAttribute('stroke-width', highlightWidth);
                                        el.style.strokeWidth = highlightWidth;
                                        if (isCircle) {{
                                            if (el.dataset.origRadius === undefined) {{
                                                el.dataset.origRadius = el.getAttribute('r') || '';
                                            }}
                                            const base = parseFloat(el.dataset.origRadius || spec.baseRadius || 4);
                                            const ratio = spec.radiusMul || 1.6;
                                            el.setAttribute('r', (base * ratio).toString());
                                        }}
                                    }});

                                    el.addEventListener('mouseleave', () => {{
                                        const origStroke = el.dataset.origStroke;
                                        if (origStroke !== undefined) {{
                                            if (origStroke) {{
                                                el.setAttribute('stroke', origStroke);
                                            }} else {{
                                                el.removeAttribute('stroke');
                                            }}
                                            el.style.stroke = el.dataset.origStrokeStyle || '';
                                        }}
                                        const origWidth = el.dataset.origStrokeWidth;
                                        if (origWidth !== undefined) {{
                                            if (origWidth) {{
                                                el.setAttribute('stroke-width', origWidth);
                                            }} else {{
                                                el.removeAttribute('stroke-width');
                                            }}
                                            el.style.strokeWidth = el.dataset.origStrokeWidthStyle || '';
                                        }}
                                        if (isCircle && el.dataset.origRadius !== undefined) {{
                                            if (el.dataset.origRadius) {{
                                                el.setAttribute('r', el.dataset.origRadius);
                                            }}
                                        }}
                                    }});
                                }});
                            }});
                        }}

                        /* 5) 실제 바인드 ---------------------------------------------- */
                        function bindClicks() {{
                            /* 5-A. WP 마커 */
                            Object.keys(wp2lines).forEach(wp => {{
                                document.querySelectorAll('.wp_' + wp).forEach(mk => {{
                                    mk.onclick = () => activate(wp);
                                }});
                            }});

                            /* 5-B. Line 자체 */
                            Object.keys(line2wp).forEach(cls => {{
                                document.querySelectorAll('.' + cls).forEach(pl => {{
                                    pl.onclick = () => activate(line2wp[cls]);
                                }});
                            }});
                        }}

                        function wireAll() {{
                            bindClicks();
                            bindHover();
                        }}

                        /* 6) 맵 레이어가 추가될 때마다 & 첫 렌더 이후 바인드 ------------- */
                        if (mp) {{
                            mp.whenReady(() => setTimeout(wireAll, 0));
                            mp.on('layeradd',  () => setTimeout(wireAll, 0));
                            mp.on('click', e => bridge.sendPoint(e.latlng.lat, e.latlng.lng));
                        }}
                    }});
                    </script>
        """


        with open(path, "r+", encoding="utf-8") as f:
            html = f.read()
            if "qwebchannel.js" not in html:
                f.seek(0); f.write(html.replace("</body>", js + "</body>")); f.truncate()

    # ─────────────────────────── 탭 0301 ───────────────────────────
    def _build_tab_0301(self):
        tab  = QWidget(); form = QFormLayout(tab)

        # ---- 입력 위젯 -------------------------------------------------
        self.le_mpid  = QLineEdit("")                 # ?? 기본값 공백
        self.le_mpid.setPlaceholderText("auto")       #  ? 자동 부여 안내
        self.le_plid  = QLineEdit("1")
        self.le_ptime = QLineEdit("1.0")

        for lab, w in (("Mission Plan ID:", self.le_mpid),
                       ("Planner ID:",      self.le_plid),
                       ("Planning Time(s):",self.le_ptime)):
            form.addRow(lab, w)

        # ---- 항공기 관리 버튼 ------------------------------------------
        add = QPushButton("Add Aircraft");   add.clicked.connect(self._add_aircraft)
        clr = QPushButton("Clear Aircraft"); clr.clicked.connect(self._clear_aircraft)
        load = QPushButton("Load JSON");     load.clicked.connect(self._load_aircraft_json)
        h = QHBoxLayout(); h.addWidget(add); h.addWidget(clr); h.addWidget(load)
        form.addRow(h)

        # ---- 로그 창 ---------------------------------------------------
        self.log0301 = QTextEdit(); self.log0301.setReadOnly(True)
        form.addRow("Log:", self.log0301)

        self.tabs.addTab(tab, "0301")
        self._refresh_0301()


    def _load_aircraft_json(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select Aircraft JSON",
            str(self.default_dir),
            "JSON files (*.json)"
        )
        if not path:
            return

        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)

            # ── 리스트 추출 ────────────────────────────────────
            if isinstance(data, list):
                pool = data
            elif isinstance(data, dict) and "aircraftList" in data:
                pool = data["aircraftList"]
            else:
                raise ValueError(
                    "JSON must be one of\n"
                    "  ? [ {\"aircraftID\": …}, … ]\n"
                    "  ? { \"aircraftList\": [ … ] }"
                )

            # ── 상태 갱신 ────────────────────────────────────
            self.aircraft_pool      = pool
            self.aircraft_file_path = path
            self.next_air = max((ac["aircraftID"] for ac in pool), default=0) + 1
            self._refresh_air_combo()
            self._refresh_0301()

        except Exception as e:
            QMessageBox.warning(self, "Load Error", str(e))

    def _add_aircraft(self):
        aid = self.next_air; self.next_air = 1 if aid == 6 else aid + 1
        self.aircraft_pool.append({"aircraftID": aid}); self._refresh_0301(); self._refresh_air_combo()

    def _clear_aircraft(self):
        self.aircraft_pool.clear()
        self.aircraft_file_path = None
        self.next_air = 1
        self._refresh_0301()
        self._refresh_air_combo()

    def _get_uav_param_values(self) -> dict:
        return {
            "cruise_speed_mps": float(self.CRUISE_SP),
            "turn_step_deg": float(self._uav_turn_step_deg),
            "default_sweep_separation_m": float(mp_config.DEFAULT_SWEEP_SEPARATION_M),
            "search_speed_weight": float(mp_config.SEARCH_SPEED_WEIGHT),
            "fov_deg": float(d0303.FOV_DEG),
            "altitude_m": float(d0303.Altitude),
            "sweep_entry_offset_m": float(d0303.SWEEP_ENTRY_OFFSET_M),
            "sweep_merge_heading_deg": float(d0303.SWEEP_MERGE_HEADING_DEG),
            "sweep_line_interp_points": int(d0303.SWEEP_LINE_INTERP_POINTS),
            "min_sweep_len_m": float(d0303.MIN_SWEEP_LEN_M),
            "min_route_spacing_m": float(d0303.MIN_ROUTE_SPACING_M),
            "default_search_speed_multiplier": float(d0303.DEFAULT_SEARCH_SPEED_MULTIPLIER),
            "point_fov_deg": float(d0303.POINT_FOV_DEG),
            "entry_hold_fov_deg": float(d0303.ENTRY_HOLD_FOV_DEG),
            "entry_hold_gimbal_pitch": float(d0303.ENTRY_HOLD_GIMBAL_PITCH),
            "entry_hold_gimbal_yaw": float(d0303.ENTRY_HOLD_GIMBAL_YAW),
            "loiter_radius_m": float(d0303.LOITER_RADIUS_M),
            "loiter_direction": int(d0303.LOITER_DIRECTION),
            "loiter_time_s": float(d0303.LOITER_TIME_S),
            "loiter_speed_mps": float(d0303.LOITER_SPEED_MPS),
        }

    def _on_uav_algo_toggled(self, key: str, checked: bool) -> None:
        if not checked:
            return
        for other_key, cb in self._uav_algo_checks.items():
            if other_key != key and cb.isChecked():
                cb.setChecked(False)

    def _load_uav_params_into_form(self, values: dict) -> None:
        for spec in self._uav_param_specs:
            key = spec["key"]
            widget = self._uav_param_inputs.get(key)
            if widget is None:
                continue
            if key in values:
                widget.setText(str(values[key]))

    def _read_uav_params_from_form(self) -> dict | None:
        values: dict[str, float | int] = {}
        for spec in self._uav_param_specs:
            key = spec["key"]
            widget = self._uav_param_inputs.get(key)
            if widget is None:
                continue
            raw = widget.text().strip()
            if raw == "":
                QMessageBox.warning(self, "UAV Mission Manager", f"Missing value: {spec['label']}")
                return None
            try:
                if spec["type"] is int:
                    value = int(float(raw))
                else:
                    value = float(raw)
            except Exception:
                QMessageBox.warning(self, "UAV Mission Manager", f"Invalid value: {spec['label']}")
                return None
            min_value = spec.get("min")
            max_value = spec.get("max")
            if min_value is not None and value < min_value:
                QMessageBox.warning(self, "UAV Mission Manager", f"Value too small: {spec['label']}")
                return None
            if max_value is not None and value > max_value:
                QMessageBox.warning(self, "UAV Mission Manager", f"Value too large: {spec['label']}")
                return None
            values[key] = value
        return values

    def _apply_uav_params(self) -> None:
        algo_keys = [key for key, cb in self._uav_algo_checks.items() if cb.isChecked()]
        if len(algo_keys) != 1:
            QMessageBox.warning(self, "UAV Mission Manager", "Select exactly one algorithm.")
            return
        algo_key = algo_keys[0]
        if algo_key == "dtatrim":
            algo_name = "dtatrim"
        elif algo_key == "algo2":
            algo_name = "linear"
        else:
            algo_name = "algo3"

        values = self._read_uav_params_from_form()
        if values is None:
            return

        self.CRUISE_SP = float(values["cruise_speed_mps"])
        self._uav_turn_step_deg = float(values["turn_step_deg"])

        mp_config.DEFAULT_SWEEP_SEPARATION_M = float(values["default_sweep_separation_m"])
        mp_config.SEARCH_SPEED_WEIGHT = float(values["search_speed_weight"])
        search_speed._CFG_WEIGHT = float(values["search_speed_weight"])

        d0303.FOV_DEG = float(values["fov_deg"])
        d0303.Altitude = int(round(values["altitude_m"]))
        d0303.SWEEP_ENTRY_OFFSET_M = float(values["sweep_entry_offset_m"])
        d0303.SWEEP_MERGE_HEADING_DEG = float(values["sweep_merge_heading_deg"])
        d0303.SWEEP_LINE_INTERP_POINTS = int(values["sweep_line_interp_points"])
        d0303.MIN_SWEEP_LEN_M = float(values["min_sweep_len_m"])
        d0303.MIN_ROUTE_SPACING_M = float(values["min_route_spacing_m"])
        d0303.DEFAULT_SEARCH_SPEED_MULTIPLIER = float(values["default_search_speed_multiplier"])
        d0303.POINT_FOV_DEG = float(values["point_fov_deg"])
        d0303.ENTRY_HOLD_FOV_DEG = float(values["entry_hold_fov_deg"])
        d0303.ENTRY_HOLD_GIMBAL_PITCH = float(values["entry_hold_gimbal_pitch"])
        d0303.ENTRY_HOLD_GIMBAL_YAW = float(values["entry_hold_gimbal_yaw"])
        d0303.LOITER_RADIUS_M = float(values["loiter_radius_m"])
        d0303.LOITER_DIRECTION = int(values["loiter_direction"])
        d0303.LOITER_TIME_S = float(values["loiter_time_s"])
        d0303.LOITER_SPEED_MPS = float(values["loiter_speed_mps"])
        d0303.SWEEP_GEOMETRY = d0303.SweepConfig(
            separation_m=float(values["default_sweep_separation_m"]),
            fov_deg=float(values["fov_deg"]),
        )
        d0303.set_route_planner(algo_name)

        self._load_uav_params_into_form(self._get_uav_param_values())

    def _reset_uav_params(self) -> None:
        if not hasattr(self, "_uav_param_defaults"):
            return
        if hasattr(self, "_uav_algo_checks"):
            default_key = getattr(self, "_uav_algo_default", "algo2")
            for key, cb in self._uav_algo_checks.items():
                cb.setChecked(key == default_key)
        self._load_uav_params_into_form(self._uav_param_defaults)
        self._apply_uav_params()

    def _build_tab_uav_manager(self):
        tab = QWidget()
        form = QFormLayout(tab)

        algo_row = QHBoxLayout()
        algo_row_widget = QWidget()
        algo_row_widget.setLayout(algo_row)
        self._uav_algo_checks = {}
        for key, label in (
            ("dtatrim", "DTAutoTrim"),
            ("algo2", "Algorithm-2"),
            ("algo3", "Algorithm-3"),
        ):
            cb = QCheckBox(label)
            cb.setChecked(key == "algo2")
            cb.toggled.connect(lambda checked, k=key: self._on_uav_algo_toggled(k, checked))
            self._uav_algo_checks[key] = cb
            algo_row.addWidget(cb)
        self._uav_algo_default = "algo2"
        form.addRow("Algorithm", algo_row_widget)

        self._uav_param_specs = [
            {"key": "cruise_speed_mps", "label": "Cruise speed (m/s)", "type": float, "min": 0.1},
            {"key": "turn_step_deg", "label": "Turn step (deg)", "type": float, "min": 0.1},
            {"key": "default_sweep_separation_m", "label": "Sweep separation (m)", "type": float, "min": 0.1},
            {"key": "search_speed_weight", "label": "Search speed weight", "type": float, "min": 0.0},
            {"key": "fov_deg", "label": "Sweep FOV (deg)", "type": float, "min": 0.1},
            {"key": "altitude_m", "label": "Default altitude (m)", "type": float, "min": 0.0},
            {"key": "sweep_entry_offset_m", "label": "Sweep entry offset (m)", "type": float, "min": 0.0},
            {"key": "sweep_merge_heading_deg", "label": "Sweep merge heading (deg)", "type": float, "min": 0.0},
            {"key": "sweep_line_interp_points", "label": "Sweep line interp points", "type": int, "min": 2},
            {"key": "min_sweep_len_m", "label": "Min sweep length (m)", "type": float, "min": 0.0},
            {"key": "min_route_spacing_m", "label": "Min route spacing (m)", "type": float, "min": 0.0},
            {"key": "default_search_speed_multiplier", "label": "Default search speed multiplier", "type": float, "min": 0.0},
            {"key": "point_fov_deg", "label": "Point FOV (deg)", "type": float, "min": 0.0},
            {"key": "entry_hold_fov_deg", "label": "Entry hold FOV (deg)", "type": float, "min": 0.0},
            {"key": "entry_hold_gimbal_pitch", "label": "Entry hold gimbal pitch (deg)", "type": float},
            {"key": "entry_hold_gimbal_yaw", "label": "Entry hold gimbal yaw (deg)", "type": float},
            {"key": "loiter_radius_m", "label": "Loiter radius (m)", "type": float, "min": 0.0},
            {"key": "loiter_direction", "label": "Loiter direction (0=None,1=CW,2=CCW)", "type": int, "min": 0, "max": 2},
            {"key": "loiter_time_s", "label": "Loiter time (s)", "type": float, "min": 0.0},
            {"key": "loiter_speed_mps", "label": "Loiter speed (m/s)", "type": float, "min": 0.0},
        ]

        self._uav_param_inputs = {}
        for spec in self._uav_param_specs:
            le = QLineEdit()
            self._uav_param_inputs[spec["key"]] = le
            form.addRow(spec["label"], le)

        btn_apply = QPushButton("Apply")
        btn_apply.clicked.connect(self._apply_uav_params)
        btn_reset = QPushButton("Reset")
        btn_reset.clicked.connect(self._reset_uav_params)
        btn_row = QHBoxLayout()
        btn_row.addWidget(btn_apply)
        btn_row.addWidget(btn_reset)
        form.addRow(btn_row)

        self._uav_param_defaults = self._get_uav_param_values()
        self._load_uav_params_into_form(self._uav_param_defaults)

        self.tabs.insertTab(0, tab, "UAV Mission Manager")

# ─────────────────────────── 초기임무계획 탭 ───────────────────────────
    def _build_tab_initial(self):
        """
        ① 0201 / 0203 JSON 로드 (LED ← 빨강↔초록)
        ② Generate 0301~0304 전 과정 실행
        """
        tab  = QWidget(); form = QFormLayout(tab)

        # 내부 상태 ---------------------------------------------------
        self._cmpk_path = None
        self._mrpk_path = None

        def _led(color: str):
            lbl = QLabel(); lbl.setFixedSize(12, 12)
            lbl.setStyleSheet(f"background:{color}; border-radius:6px")
            return lbl
        self._led_cmpk = _led("#c00")     # red  ← not loaded
        self._led_mrpk = _led("#c00")

        # ------------- Load 0201 ------------------------------------
        btn_cmpk = QPushButton("Load 0201 협업기저임무")
        btn_cmpk.clicked.connect(self._load_0201_json)
        row1 = QHBoxLayout(); row1.addWidget(btn_cmpk); row1.addWidget(self._led_cmpk)
        form.addRow(row1)

        # ------------- Load 0203 ------------------------------------
        btn_mrpk = QPushButton("Load 0203 비행참조정보")
        btn_mrpk.clicked.connect(self._load_0203_json)
        row2 = QHBoxLayout(); row2.addWidget(btn_mrpk); row2.addWidget(self._led_mrpk)
        form.addRow(row2)

        # ------------- Generate All ---------------------------------
        btn_gen = QPushButton("Generate 0301 ~ 0304")
        btn_gen.clicked.connect(self._generate_all)
        form.addRow(btn_gen)

        # -----  (신규) 가시성 토글 스위치 -------------------------------
        vis_row = QHBoxLayout()

        self._vis_btns = {}            # aid(int) → QPushButton
        for aid, label in [(1,"LAH1"),(2,"LAH2"),(3,"LAH3"),
                           (4,"UAV4"),(5,"UAV5"),(6,"UAV6")]:
            btn = QPushButton(label)
            btn.setCheckable(True); btn.setChecked(True)
            btn.setStyleSheet("QPushButton:checked{background:#4caf50;color:white}"
                              "QPushButton{background:#ddd}")
            btn.toggled.connect(lambda state, a=aid: self._toggle_aircraft(a, state))
            vis_row.addWidget(btn)
            self._vis_btns[aid] = btn

        for tag, label in (("0201", "0201"), ("0203", "0203")):
            btn_ds = QPushButton(label)
            btn_ds.setCheckable(True); btn_ds.setChecked(True)
            btn_ds.setStyleSheet("QPushButton:checked{background:#4caf50;color:white}"
                              "QPushButton{background:#ddd}")
            btn_ds.toggled.connect(lambda state, key=tag: self._toggle_dataset(key, state))
            vis_row.addWidget(btn_ds)
            if tag == "0201":
                self._btn_0201 = btn_ds
            else:
                self._btn_0203 = btn_ds


        form.addRow("Show/Hide:", vis_row)

        # ------------- Log window -----------------------------------
        self.log_init = QPlainTextEdit(); self.log_init.setReadOnly(True)
        form.addRow(self.log_init)

        # 탭 0 (맨 왼쪽)에 삽입
        self.tabs.insertTab(1, tab, "초기임무계획")

    # ─────────────────── 가시성 토글 핸들러 ────────────────────
    def _toggle_aircraft(self, aid: int, state: bool):
        if state:
            self._visible_aircrafts.add(aid)
        else:
            self._visible_aircrafts.discard(aid)
        self._rebuild_map()            # 지도 다시 그리기


    def _toggle_dataset(self, tag: str, state: bool):
        if tag == "0201":
            self._visible_0201 = state
        elif tag == "0203":
            self._visible_0203 = state
        else:
            return
        self._rebuild_map()

    def _generate_all(self):
        if not (self._cmpk_path and self._mrpk_path):
            QMessageBox.warning(self, "경고", "0201·0203 JSON을 모두 로드하세요")
            return

        out_root = self.SAVE_DIR / "mission_output"
        out_root.mkdir(parents=True, exist_ok=True)

        self.log_init.appendPlainText("=== Pipeline 시작 ===")
        plan_start = time.perf_counter()
        compute_ms = 0.0

        # ── 1. 0201+0203 → IMP(0302) ---------------------------------
        try:
            imp_paths = run_divide_and_pattern(
                cmpk_path = self._cmpk_path,
                ref_path  = self._mrpk_path,
                out_dir   = str(out_root),
                log       = gui_logger(self.log_init),   # ★ 변경
            )
            if not imp_paths:
                raise RuntimeError("IMP 생성 결과가 없습니다.")
            self.log_init.appendPlainText(
                f"[OK] IMP {len(imp_paths)} 개 생성 완료")
            metrics = get_last_divide_and_pattern_metrics()
            if metrics:
                dp_total_s = float(metrics.get('total_s', 0.0) or 0.0)
                dp_excluded_s = float(metrics.get('load_s', 0.0) or 0.0)
                dp_excluded_s += float(metrics.get('lah_save_s', 0.0) or 0.0)
                dp_excluded_s += float(metrics.get('uav_imp_s', 0.0) or 0.0)
                dp_compute_s = max(0.0, dp_total_s - dp_excluded_s)
                dp_compute_ms = dp_compute_s * 1000.0
                compute_ms += dp_compute_ms
                self.log_init.appendPlainText(
                    '[INFO] compute-only divide_and_pattern: '
                    f'{dp_compute_ms:.1f} ms (total={dp_total_s:.2f}s, excluded={dp_excluded_s:.2f}s)'
                )
        except Exception as e:
            self.log_init.appendPlainText(f"[ERR] divide/pattern 실패: {e}")
            return

        # ── 2. MissionPlan 0301 --------------------------------------
        mp_path = out_root / f"MissionPlan_{int(time.time()*1000)}.json"
        try:
            build_t0 = time.perf_counter()
            build_mission_plan_0301(
                self._cmpk_path, self._mrpk_path, imp_paths, str(mp_path)
            )
            build_ms = (time.perf_counter() - build_t0) * 1000.0
            compute_ms += build_ms
            self.log_init.appendPlainText(f"[TIME] 0301 build: {build_ms:.1f} ms")

            # ▼ 0301 MissionPlan 에서 aircraftID → individualMissionPackageID 매핑 추출
            with open(mp_path, encoding="utf-8") as f:
                _mp = json.load(f)
            self.imp_id_map = {
                a["aircraftID"]: a["individualMissionPackageID"]
                for a in _mp.get("aircraftList", [])
            }

            self.log_init.appendPlainText(f"[OK] 0301 저장 → {mp_path}")
        except Exception as e:
            self.log_init.appendPlainText(f"[ERR] 0301 생성 실패: {e}")
            return


        # ── 3. IMP 로 GUI self.missions 에 주입 -----------------------
        self.missions.clear()
        max_mid_num = 0
        for imp in imp_paths:
            with open(imp, encoding="utf-8") as f:
                pkg = json.load(f)
            aid = pkg["aircraftID"]
            for im in pkg["individualMissionList"]:
                im["aircraftID"] = aid
                self.missions.append(im)
                try:
                    num = int(im.get("individualMissionID", 0))
                    max_mid_num = max(max_mid_num, num)
                except: pass
        self.next_im = max_mid_num + 1
        self._refresh_0302()          # 기존 함수 재활용 → 0302 로그 창 갱신

        # ── 4. 0303 / 0304 자동 생성(기존 버튼 로직 재사용) ------------
        self._refresh_0303()
        self._refresh_0304()
        compute_ms += (self._last_compute_ms_0302 + self._last_compute_ms_0303 + self._last_compute_ms_0304)
        total_030x_ms = (self._last_compute_ms_0302 + self._last_compute_ms_0303 + self._last_compute_ms_0304)
        self.log_init.appendPlainText(
            f"[INFO] compute-only 0302/0303/0304: {total_030x_ms:.1f} ms "
            f"(0302={self._last_compute_ms_0302:.1f}, 0303={self._last_compute_ms_0303:.1f}, 0304={self._last_compute_ms_0304:.1f})"
        )

        # ── 4-1. planningTime을 전체 파이프라인 경과(ms)로 갱신 ---------
        try:
            elapsed_ms = (time.perf_counter() - plan_start) * 1000.0
            if compute_ms <= 0.0:
                compute_ms = elapsed_ms
            mp_data = json.loads(mp_path.read_text(encoding="utf-8"))
            mp_data["planningTime"] = float(compute_ms)
            mp_path.write_text(json.dumps(mp_data, ensure_ascii=False, indent=2), encoding="utf-8")
            self.log_init.appendPlainText(
                f"[INFO] planningTime(ComputeOnly) ????: {compute_ms:.1f} ms (total={elapsed_ms:.1f} ms)"
            )
        except Exception as e:
            self.log_init.appendPlainText(f"[WARN] planningTime update failed: {e}")

        # ── 5. 결과 로그 & 0301 텍스트 창 갱신 ------------------------
        self.log_init.appendPlainText(f"[OK] 0301 MissionPlan 저장 → {mp_path}")

        if hasattr(self, "log0301"):                  # 0301 탭이 있으면 표시
            with open(mp_path, encoding="utf-8") as f:
                self.log0301.setPlainText(f.read())

        self.log_init.appendPlainText("=== Pipeline 완료 ===")

        # ▶? 모든 기체 표시 상태를 ON 으로 초기화
        self._visible_aircrafts = set(range(1, 7))
        for btn in self._vis_btns.values():
            btn.setChecked(True)

        self._visible_0201 = True
        self._visible_0203 = True
        if self._btn_0201:
            self._btn_0201.setChecked(True)
        if self._btn_0203:
            self._btn_0203.setChecked(True)
        
    # ─────────────── 0201 / 0203 Load 핸들러 ───────────────
    def _set_led(self, led: QLabel, ok: bool):
        led.setStyleSheet(
            f"background:{'#0c0' if ok else '#c00'}; border-radius:6px")

    # ─────────────────────────────────────────────────────────────
    # 0201 협업기저임무패키지(CMPK) 로드
    #   · 파일명 "1.json" ? 1  →  inputMissionPackageID 로 저장
    #   · LED 초록 & 로그 기록
    # ─────────────────────────────────────────────────────────────
    def _load_0201_json(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "0201 CMPK JSON",
            str(self.DIR_0201),
            "JSON files (*.json)",
        )
        if not path:
            return

        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            QMessageBox.warning(self, "0201 로드 실패", str(e))
            return

        try:
            self.pkg0201_id = int(Path(path).stem)
        except ValueError:
            QMessageBox.warning(self, "0201 로드 실패",
                                "파일 이름이 '정수.json' 형태여야 합니다.")
            return

        self._cmpk_path = path
        self._cmpk_data = data
        self._led_cmpk.setStyleSheet("background:#0c0; border-radius:6px")
        self.log_init.appendPlainText(f"[OK] CMPK loaded  →  {path}")
        self._rebuild_map()

    def _load_0203_json(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "0203 MRPK JSON",
            str(self.DIR_0203),
            "JSON files (*.json)",
        )
        if not path:
            return

        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            QMessageBox.warning(self, "0203 로드 실패", str(e))
            return

        try:
            self.pkg0203_id = int(Path(path).stem)
        except ValueError:
            QMessageBox.warning(self, "0203 로드 실패",
                                "파일 이름이 '정수.json' 형태여야 합니다.")
            return

        self._mrpk_path = path
        self._mrpk_data = data
        self._led_mrpk.setStyleSheet("background:#0c0; border-radius:6px")
        self.log_init.appendPlainText(f"[OK] MRPK loaded  →  {path}")
        self._rebuild_map()

    def _refresh_0301(self):
        # 항공기 없는 경우 안내
        if not self.aircraft_pool:
            self.log0301.setPlainText("?  먼저 A/C 를 추가하거나 JSON 목록을 불러오세요.")
            return

        # 0201 / 0203 아직 안 불렀다면 중단
        if self.pkg0201_id is None or self.pkg0203_id is None:
            self.log0301.setPlainText("?  0201·0203 파일을 먼저 불러와야 합니다.")
            return

        # MissionPlanID 입력값 처리
        mpid_raw = self.le_mpid.text().strip()
        mpid = None if mpid_raw == "" else mpid_raw     # str 그대로 ? d0301 내부에서 변환

        try:
            msg = d0301.build_mission_plan(
                aircraft_pool               = self.aircraft_pool,
                input_mission_package_id    = self.pkg0201_id,
                mission_reference_package_id= self.pkg0203_id,
                mission_plan_id             = mpid,
                planner_id                  = int(self.le_plid.text() or 0),
                planning_time_s             = float(self.le_ptime.text() or 0),
            )
        except Exception as e:
            QMessageBox.warning(self, "0301 생성 실패", str(e)); return

        # 상태·로그 갱신
        self.plan_pkg_id = msg["aircraftList"][0]["individualMissionPackageID"]
        self.imp_id_map  = {a["aircraftID"]: a["individualMissionPackageID"]
                            for a in msg["aircraftList"]}
        self.log0301.setText(json.dumps(msg, indent=2))


    # ─────────────────────────── 탭 0302 ───────────────────────────
    def _build_tab_0302(self):
        tab = QWidget(); form = QFormLayout(tab)

        # ── 항공기 선택 콤보 ───────────────────────────────────
        self.cmb_air = QComboBox()
        self._refresh_air_combo()
        form.addRow("Aircraft ID:", self.cmb_air)

        # ── 버튼들 ─────────────────────────────────────────────
        new  = QPushButton("New Mission")       # 수동 추가
        new.clicked.connect(self._new_mission)

        clr  = QPushButton("Clear Missions")    # 전체 삭제
        clr.clicked.connect(self._clear_missions)

        load = QPushButton("Load 0302 JSON(s)") # 여러 개 불러오기
        load.clicked.connect(self._load_0302_jsons)

        h = QHBoxLayout()
        for b in (new, clr, load): h.addWidget(b)
        form.addRow(h)

        # ── 로그창 ─────────────────────────────────────────────
        self.log0302 = QTextEdit(); self.log0302.setReadOnly(True)
        form.addRow("Log:", self.log0302)

        self.tabs.addTab(tab, "0302")
        self._refresh_0302()

    def _load_0302_jsons(self):
        paths, _ = QFileDialog.getOpenFileNames(
            self, "Select 0302 JSON files",
            str(self.default_dir),
            "JSON files (*.json)"
        )
        if not paths:
            return

        try:
            self.missions.clear()
            max_mid_num = 0

            for p in paths:
                with open(p, encoding="utf-8") as f:
                    pkg = json.load(f)

                if ("aircraftID" not in pkg or
                    "individualMissionList" not in pkg):
                    raise ValueError(f"{os.path.basename(p)}: invalid 0302 format")

                aid = pkg["aircraftID"]
                for im in pkg["individualMissionList"]:
                    im_cp = dict(im)
                    im_cp["aircraftID"] = aid
                    self.missions.append(im_cp)

                    # ── ID 숫자 부분만 추출해 비교 ──────────────
                    iid_raw = im_cp.get("individualMissionID", 0)
                    try:
                        iid_num = int(iid_raw)
                    except (ValueError, TypeError):
                        iid_num = 0                     # 문자열·None → 0
                    max_mid_num = max(max_mid_num, iid_num)

            self.next_im = max_mid_num + 1
            self._refresh_0302()
            self._rebuild_map()
            QMessageBox.information(
                self, "Load Complete",
                f"Loaded {len(paths)} file(s), total {len(self.missions)} missions.")
        except Exception as e:
            QMessageBox.warning(self, "Load Error", str(e))

    def _refresh_air_combo(self):
        self.cmb_air.clear(); self.cmb_air.addItems([str(a["aircraftID"]) for a in self.aircraft_pool])

    def _new_mission(self):
        if not self.cmb_air.currentText(): return
        dlg = mh.MissionMetaDialog(self.next_im, self)
        if dlg.exec_() == QDialog.Accepted:
            self.pending = dlg.get_result(); self.pending_pts = []
            typ, need, _ = self.pending; self.log0302.append(f"Select {need} points (type {typ})")

    def _clear_missions(self):
        self.missions.clear(); self.next_im = 1; self._refresh_0302(); self._rebuild_map()

    def _refresh_0302(self):
        t0 = time.perf_counter()
        pkg_list = d0302.build_mission_packages(
            self.missions,                 # IM 원본 리스트
            cmpk_id      = self.pkg0201_id,  # 0201 파일명 숫자
            plan_pkg_map = self.imp_id_map,  # aircraftID → IMP ID
        )
        self._last_compute_ms_0302 = (time.perf_counter() - t0) * 1000.0
        self.log0302.setText(json.dumps(pkg_list, indent=2, ensure_ascii=False))
        self._rebuild_map()



    # ─────────────────────────── 탭 0303 ───────────────────────────
    def _build_tab_0303(self):
        tab = QWidget(); lay = QVBoxLayout(tab)
        gen = QPushButton("Generate 0303 FlightPlan"); gen.clicked.connect(self._refresh_0303)
        self.log0303 = QPlainTextEdit(); self.log0303.setReadOnly(True)
        lay.addWidget(gen); lay.addWidget(self.log0303); self.tabs.addTab(tab, "0303")

    def _refresh_0303(self):
        t0 = time.perf_counter()
        fp = d0303.build_flight_plans(
            self.missions,
            self.wp_alloc,
            self.CRUISE_SP,
            turn_step_deg=self._uav_turn_step_deg,
            ref0203=self._mrpk_data,
        )
        self._last_compute_ms_0303 = (time.perf_counter() - t0) * 1000.0
        self.flight_plans = fp.copy()
        self.log0303.setPlainText(json.dumps(fp, indent=2, ensure_ascii=False))
        self._rebuild_map()

    # ─────────────────────────── 탭 0304 ───────────────────────────
    def _build_tab_0304(self):
        tab = QWidget(); lay = QVBoxLayout(tab)

        # ── 버튼 영역 ───────────────────────────────────
        btn_gen  = QPushButton("Generate 0304 FlightPlan")
        btn_gen.clicked.connect(self._refresh_0304)

        btn_save = QPushButton("Save Missions")
        btn_save.clicked.connect(self._save_all_missions)

        btn_check = QPushButton("Check Missions")      # ★ 새 버튼
        btn_check.clicked.connect(self._check_missions)

        h_btn = QHBoxLayout()
        for b in (btn_gen, btn_save, btn_check):
            h_btn.addWidget(b)
        lay.addLayout(h_btn)

        # ── 로그창 ─────────────────────────────────────
        self.log0304 = QPlainTextEdit(); self.log0304.setReadOnly(True)
        lay.addWidget(self.log0304)

        self.tabs.addTab(tab, "0304")

    # ─────────────────── 임무계획 유효성 검증 ────────────────────
    def _check_missions(self, check_saved: bool = True):
        """
        0201?0203?0301?0302?0303?0304 (메모리) + 디스크 저장본
        모두를 한 번에 검증하고, MEM_… / DISK_… 태그로 구분해
        로그에 남긴다.
        """
        import json
        from pathlib import Path
        from PyQt5.QtWidgets import QMessageBox

        self.log0304.appendPlainText("\n===== CHECK MISSIONS =====")
        errors, warns, oks = [], [], []

        # ─────────────────── 타입 검증 헬퍼 (FIXED) ────────────────────
        def _check_json_types(self, imp_pkgs: list, tag: str):
            """
            imp_pkgs : 0302 IndividualMissionPackage 리스트
            ? 각 IM 의 isDone / isHole / altitude 자료형을 점검한다.
            """
            import numbers, math
            local_errs = 0

            for pkg in imp_pkgs:                                   # ─ 패키지 루프
                for im in pkg.get("individualMissionList", []):    # ─ IM 루프
                    mid = im.get("individualMissionID", "?")

                    # 1) isDone → bool
                    if not isinstance(im.get("isDone"), bool):
                        errors.append(f"{tag} mid={mid}: isDone must be bool")
                        local_errs += 1

                    info = im.get("individualMissionInfo", {})

                    # 2) isHole → bool
                    for area in info.get("areaList", []):
                        if "isHole" in area and not isinstance(area["isHole"], bool):
                            errors.append(f"{tag} mid={mid}: isHole must be bool")
                            local_errs += 1

                    # 3) altitude → 숫자
                    blocks = info.get("areaList") or info.get("lineList") or []
                    for blk in blocks:
                        for p in blk.get("coordinateList", []):
                            alt = p.get("altitude")
                            if alt is not None and (not isinstance(alt, numbers.Real) or math.isnan(alt)):
                                errors.append(f"{tag} mid={mid}: altitude must be number")
                                local_errs += 1

            return local_errs
        # ─────────────────────────────── 공통 헬퍼 ────────────────────────────────
        # ─────────────────────────────── 공통 헬퍼 ────────────────────────────────
        def _check_fps(fp_list: list, missions_src: list, tag: str):
            """
            FlightPath 리스트 검사 → (err_cnt, warn_cnt)
            · WP 중복, ECF 단조 증가, nextWaypointID 체인
            · operationMode 별 필수 필드
              - mode 4(HOLD) → aircraftFixed.gimbalPitch/gimbalYaw 포함
            """
            local_errs, local_warns = 0, 0
            wp_global = set()
            miss_path_ids = {im["pathID"] for im in missions_src}

            for fp in fp_list:
                path_id = fp.get("pathID")
                wps     = fp.get("waypointList", [])
                if not wps:
                    errors.append(f"{tag} pathID {path_id}: waypointList empty")
                    local_errs += 1
                    continue

                # 0302와 pathID 매칭
                if path_id not in miss_path_ids:
                    warns.append(f"{tag} pathID {path_id} not found in 0302 missions")
                    local_warns += 1

                # waypointID 중복 및 ECF 단조 증가
                last_ecf = -1.0
                for wp in wps:
                    wid = wp["waypointID"]
                    if wid in wp_global:
                        errors.append(f"{tag} duplicate waypointID {wid}")
                        local_errs += 1
                    wp_global.add(wid)

                    if wp["ecf"] + 1e-6 < last_ecf:
                        errors.append(f"{tag} waypointID {wid}: ECF not increasing")
                        local_errs += 1
                    last_ecf = wp["ecf"]

                    # ── mode-specific 필드 검사 ─────────────────────────
                    fp_prop = wp.get("filmingProperty", {})
                    mode    = fp_prop.get("operationMode", 0)

                    need_top = {
                        1: ["coordinateOrientation"],
                        2: ["lineSearch"],
                        3: ["autoTracking"],
                        4: ["aircraftFixed"],   # HOLD
                        5: ["autoScan"],
                    }.get(mode, [])

                    for f in need_top:
                        if f not in fp_prop:
                            errors.append(f"{tag} WP {wid}: mode-{mode} needs {f}")
                            local_errs += 1

                    # ── mode 4: aircraftFixed 하위 필드(gimbalPitch/gimbalYaw) ──
                    if mode == 4 and "aircraftFixed" in fp_prop:
                        af = fp_prop["aircraftFixed"]
                        for sub in ("gimbalPitch", "gimbalYaw"):
                            if sub not in af:
                                errors.append(f"{tag} WP {wid}: aircraftFixed missing {sub}")
                                local_errs += 1

                # nextWaypointID 체인 검증
                chain = {w["waypointID"]: w["nextWaypointID"] for w in wps}
                visited, cur = set(), wps[0]["waypointID"]
                while cur and cur not in visited:
                    visited.add(cur)
                    cur = chain.get(cur, 0)
                if len(visited) != len(wps):
                    warns.append(f"{tag} pathID {path_id}: nextWaypointID chain broken")
                    local_warns += 1

            if local_errs == 0:
                oks.append(f"{tag}: FlightPath OK ({len(fp_list)} paths)")
            return local_errs, local_warns






        # ───────────────────── 1) CMPK / MRPK 파일 로드 ─────────────────────
        for _tag, _path in (("0201", self._cmpk_path), ("0203", self._mrpk_path)):
            try:
                with open(_path, encoding="utf-8") as f: json.load(f)
                oks.append(f"{_tag}: file loaded OK")
            except Exception as e:
                errors.append(f"{_tag} validation failed - {e}")

        # ───────────────────── 2) 메모리-상 패키지 ─────────────────────
        try:
            mp_mem = json.loads(self.log0301.toPlainText())
            d0301._validate_mission_plan(mp_mem)
            oks.append("MEM_0301: MissionPlan OK")
        except Exception as e:
            errors.append(f"MEM_0301 validation failed - {e}")

        try:
            imp_mem = json.loads(self.log0302.toPlainText())
            cmpk_id = self.pkg0201_id or 0
            d0302._validate_mission_packages(imp_mem, self.imp_id_map, cmpk_id)
            # ▶ 타입 검사 추가
            _check_json_types(self, imp_mem, "MEM_0302")
            oks.append("MEM_0302: IndividualMissionPlan OK")
        except Exception as e:
            errors.append(f"MEM_0302 validation failed - {e}")

        _check_fps(self.flight_plans,     self.missions, "MEM_0303")
        _check_fps(self.flight_plans_0304, self.missions, "MEM_0304")

        # ───────────────────── 3) 디스크 저장본 (옵션) ─────────────────────
        if check_saved:
            dir_mp  = self.SAVE_DIR / "MissionPlan"
            dir_imp = self.SAVE_DIR / "IndividualMissionPlan"
            dir_fp  = self.SAVE_DIR / "FlightPath"

            # 3-A. MissionPlan
            for p in sorted(dir_mp.glob("*.json")):
                try:
                    with p.open(encoding="utf-8") as f: mp = json.load(f)
                    d0301._validate_mission_plan(mp)
                    oks.append(f"DISK_0301 {p.name}: OK")
                except Exception as e:
                    errors.append(f"DISK_0301 {p.name}: {e}")

            # 3-B. IMP + missions 집계
            imp_pkgs_disk, missions_disk, plan_pkg_map_disk = [], [], {}
            latest_mp = max(dir_mp.glob("*.json"), default=None, key=lambda p: p.stat().st_mtime)
            if latest_mp:
                with latest_mp.open(encoding="utf-8") as f:
                    _lmp = json.load(f)
                plan_pkg_map_disk = {a["aircraftID"]: a["individualMissionPackageID"]
                                     for a in _lmp.get("aircraftList", [])}

            for p in sorted(dir_imp.glob("*.json")):
                with p.open(encoding="utf-8") as f: pkg = json.load(f)
                imp_pkgs_disk.append(pkg)
                aid = pkg["aircraftID"]
                for im in pkg.get("individualMissionList", []):
                    im_cp = dict(im); im_cp["aircraftID"] = aid
                    missions_disk.append(im_cp)

            if imp_pkgs_disk:
                try:
                    cmpk_id = self.pkg0201_id or 0
                    d0302._validate_mission_packages(imp_pkgs_disk, plan_pkg_map_disk, cmpk_id)
                    oks.append(f"DISK_0302 {len(imp_pkgs_disk)} pkg: OK")
                except Exception as e:
                    errors.append(f"DISK_0302 packages: {e}")

            # 3-C. FlightPath
            fp_disk = [json.load(p.open(encoding="utf-8")) for p in dir_fp.glob("*.json")]
            if fp_disk:
                _check_fps(fp_disk, missions_disk, "DISK_FP")

        # ───────────────────── 4) 로그 출력 ─────────────────────
        for line in oks:   self.log0304.appendPlainText(f"[OK  ] {line}")
        for w in warns:    self.log0304.appendPlainText(f"[WARN] {w}")
        for e in errors:   self.log0304.appendPlainText(f"[ERR ] {e}")

        if errors:
            QMessageBox.critical(self, "Check Missions",
                                 f"{len(errors)} error(s) ? {len(warns)} warning(s)")
        elif warns:
            QMessageBox.warning(self, "Check Missions",
                                 f"All critical tests passed with {len(warns)} warning(s).")
        else:
            QMessageBox.information(self, "Check Missions", "All checks passed!")

    def _refresh_0304(self):
        t0 = time.perf_counter()
        fp = d0304.build_lah_flight_plans_fixed(
            self.missions,
            cruise_speed = self.CRUISE_SP,
            wp_alloc     = self.wp_alloc,
        )
        self._last_compute_ms_0304 = (time.perf_counter() - t0) * 1000.0

        self.flight_plans_0304 = fp
        self.log0304.setPlainText(json.dumps(fp, indent=2, ensure_ascii=False))
        self._rebuild_map()

    # ─────────────────── 공통 핸들러 / 유틸 ────────────────────
    def _handle_point(self, lat: float, lon: float):
        if not self.pending:
            return
        typ, need, width = self.pending
        alt = terrain_elev(lat, lon) + 200.0
        self.pending_pts.append({"latitude": lat,
                                "longitude": lon,
                                "altitude": int(round(alt))})   # ← int 처리로 변경
        self.map_view.page().runJavaScript(
            f"(function(){{var m=null;for(var k in window)if(window[k] instanceof L.Map){{m=window[k];break;}}"
            f"if(m)L.circleMarker([{lat},{lon}],{{radius:4,color:'red',fill:true}}).addTo(m);}})();"
        )
        cur = len(self.pending_pts)
        self.log0302.append(f"Point {cur}/{need} selected")
        if cur == need:
            self._save_pending()

    def _save_pending(self):
        if not self.pending:
            return

        # ── 내부 상태 --------------------------------------------------
        typ, _, width = self.pending          # typ: 0-Area, 1-Corridor, 2-Move
        mid   = self.next_im
        aid   = int(self.cmb_air.currentText())

        miss = mh.make_individual_mission(mid)
        miss["aircraftID"] = aid
        miss["pathID"]     = next_path_id(aid)

        # 0301 Plan Package ID 연결
        if self.plan_pkg_id is not None:
            miss["individualMissionPlanPackageID"] = self.plan_pkg_id

        # ── IM-Type / Pattern Type 매핑 -------------------------------
        #   0:Area → 3/0,   1:Corridor → 6/4,   2:Move → 7/0
        TYPE_MAP = {0: (3, 0), 1: (6, 4), 2: (7, 0)}
        im_type, pat_type = TYPE_MAP[typ]

        info = miss["individualMissionInfo"]
        info["individualMissionType"] = im_type
        info["patternType"]           = pat_type

        # ── 좌표 저장 --------------------------------------------------
        if typ == 0:                              # Area (polygon)
            info["areaList"] = [{
                "isHole": False,
                "coordinateList": self.pending_pts
            }]
        elif typ == 1:                            # Corridor (lineList)
            info["lineList"] = [{
                "width": width,
                "coordinateList": self.pending_pts
            }]
        else:                                     # Move (coordinateList)
            info["coordinateList"] = self.pending_pts

        # ── 목록 반영 & UI 갱신 ----------------------------------------
        self.missions.append(miss)
        self.next_im += 1
        self.pending, self.pending_pts = None, []
        self.log0302.append("? Mission saved")
        self._refresh_0302()
        self._rebuild_map()


    def _save_all_missions(self):
        """
        ? 저장 폴더: SAVE_DIR/database 하위(구성에 따라) 또는 SAVE_DIR 하위
        - MissionPlan/<MissionPlanID>.json
        - IndividualMissionPlan/<IMP_ID>.json
        - FlightPath/<PathID>.json
        ? 새로운 임무 저장 전, 위 3개 폴더의 기존 *.json 전부 삭제(클린 세이브)
        ? Save 후 mission_output 임시폴더 자동 삭제
        """
        import json, shutil

        # ── 1) 최종 저장 폴더 준비 ─────────────────────────────
        dir_mp  = self.SAVE_DIR / "MissionPlan"
        dir_imp = self.SAVE_DIR / "IndividualMissionPlan"
        dir_fp  = self.SAVE_DIR / "FlightPath"
        for d in (dir_mp, dir_imp, dir_fp):
            d.mkdir(parents=True, exist_ok=True)

        # ── 2) 기존 임무 파일 전부 삭제(클린 세이브) ────────────
        for d in (dir_mp, dir_imp, dir_fp):
            try:
                removed = 0
                for p in d.glob("*.json"):
                    try:
                        p.unlink()
                        removed += 1
                    except Exception as ie:
                        self.log0304.appendPlainText(f"[WARN] 삭제 실패: {p} → {ie}")
                if removed:
                    self.log0304.appendPlainText(f"[INFO] 기존 임무 정리: {d} ({removed}개 삭제)")
            except Exception as e:
                self.log0304.appendPlainText(f"[WARN] 기존 파일 정리 실패: {d} → {e}")

        # ── 3) 0301 MissionPlan 1개 저장 ─────────────────────
        try:
            mp_data = json.loads(self.log0301.toPlainText())
            mp_id   = str(mp_data.get("missionPlanID") or mp_data.get("MissionPlanID"))
            (dir_mp / f"{mp_id}.json").write_text(
                json.dumps(mp_data, indent=2, ensure_ascii=False), encoding="utf-8")
        except Exception as e:
            self.log0304.appendPlainText(f"[WARN] 0301 저장 실패: {e}")

        # ── 4) 0302 IndividualMissionPlan 여러 개 저장 ───────
        imp_cnt = 0
        try:
            for pkg in json.loads(self.log0302.toPlainText()):
                imp_id = str(pkg.get("individualMissionPackageID")
                            or pkg.get("individualMissionPlanPackageID"))
                (dir_imp / f"{imp_id}.json").write_text(
                    json.dumps(pkg, indent=2, ensure_ascii=False), encoding="utf-8")
                imp_cnt += 1
        except Exception as e:
            self.log0304.appendPlainText(f"[WARN] 0302 저장 실패: {e}")

        # ── 5) 0303?0304 FlightPath 저장 ─────────────────────
        fp_cnt = 0
        for lst in (self.flight_plans, self.flight_plans_0304):
            for fp in lst:
                try:
                    pid = str(fp["pathID"])
                    (dir_fp / f"{pid}.json").write_text(
                        json.dumps(fp, indent=2, ensure_ascii=False), encoding="utf-8")
                    fp_cnt += 1
                except Exception as e:
                    self.log0304.appendPlainText(f"[WARN] PathID {pid} 저장 실패: {e}")

        # ── 6) 임시 mission_output 폴더 자동 삭제 ─────────────
        tmp_dir = self.SAVE_DIR / "mission_output"
        try:
            if tmp_dir.exists():
                shutil.rmtree(tmp_dir)
                self.log0304.appendPlainText(f"[INFO] 임시폴더 삭제: {tmp_dir}")
        except Exception as e:
            self.log0304.appendPlainText(f"[WARN] 임시폴더 삭제 실패: {e}")

        # ── 7) 완료 로그 ─────────────────────────────────────
        self.log0304.appendPlainText(
            f"? 저장 완료  →  MissionPlan 1, IndividualMission {imp_cnt}, FlightPath {fp_cnt}"
        )



# ????????????????????????????????????????????????????????????????????
def main():
    import sys, os
    from PyQt5.QtWidgets import QApplication

    if os.name == "nt":
        import multiprocessing
        multiprocessing.freeze_support()

    # 이미 위에서 가드 인스턴스를 만들었으므로, 여기서는 재사용
    app = QApplication.instance() or QApplication(sys.argv)

    win = MainGUI()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

