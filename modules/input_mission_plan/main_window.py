# main_window.py
# 좌: Folium 지도(QWebEngineView) / 우: 0201 입력 패널(패키지/센서/항공기/미션)
import os, json
from PyQt5.QtCore import QUrl, Qt
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QSplitter,
    QLabel, QFrame, QStatusBar, QGroupBox, QPushButton, QComboBox,
    QCheckBox, QDoubleSpinBox, QListWidget, QFileDialog, QMessageBox, QGridLayout
)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel

from bridge import MapBridge
from map_utils import write_map_html

# ── 외부 유틸 (앞서 제공한 파일) ──────────────────────────────────────────
# - cmpk_0201_id.py : 패키지/미션 ID 부여 & 검증
# - cmpk_0201_time.py : timestamp(ms since 2000) 부여 & 검증
from cmpk_0201_id import (
    assign_package_id_inplace, assign_mission_ids_inplace, validate_sequential_ids
)
from cmpk_0201_time import assign_timestamp_inplace, validate_timestamp

# ── 고정 매핑/옵션 ───────────────────────────────────────────────────────
PACK_TYPES = [
    (1, "대기갑항공타격작전"),
    (2, "지상작전부대 기동여건 보장 작전"),
    (3, "공중강습작전대 엄호 작전"),
    (4, "항공지원작전-중요시설 방호"),
    (5, "도시지역 작전"),
]
MAIN_SENSORS = [(0, "0"), (1, "1"), (2, "2")]  # 0~2
AIRCRAFT_IDS = [
    (1, "지휘기(1)"),
    (2, "편대기1(2)"),
    (3, "편대기2(3)"),
    (4, "UAV#1(4)"),
    (5, "UAV#2(5)"),
    (6, "UAV#3(6)"),
]
# 미션 타입: 한 미션당 하나의 타입만 선택 (요청 사항)
MISSION_TYPES = [
    (1, "coordinateList (점/웨이포인트)"),
    (2, "lineList (선/회랑)"),
    (3, "areaList (면/다각형)"),
]

class CMPK0201Window(QMainWindow):
    """
    클래스: CMPK0201Window
    목적: '0201 협업기저임무' 입력 GUI
         - 좌측: Folium 지도
         - 우측: 패키지/센서/항공기/미션 입력 패널
    JSON 스키마(중요, lower camelCase):
      top-level:
        timestamp (ms since 2000-01-01 UTC)
        inputMissionPackageID (1..N, 전역 유일)
        inputMissionPackageType (1..5)
        mainSensor (0..2)
        availableAircraftList: [{aircraftID}]
        inputMissionList: [{
            inputMissionID (1..M, 패키지 내부 유일)
            inputMissionType (1=coordinate, 2=line, 3=area)
            isDone (bool)
            missionDetail: { coordinateList | lineList | areaList }
        }]
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("0201 협업기저임무 계획 GUI")
        self.resize(1500, 940)
        self._init_state()
        self._build_ui()
        self._load_map()

    # ─────────────────────────────────────────────────────
    # 메서드: _init_state — 내부 상태 초기화
    def _init_state(self):
        # 작업 중 CMPK 모델(저장 전까지 메모리에서 유지)
        self.cmpk = {
            # "timestamp": ... 저장시 부여
            # "inputMissionPackageID": ... 버튼으로 고유 발급
            "inputMissionPackageType": PACK_TYPES[0][0],
            "mainSensor": MAIN_SENSORS[0][0],
            "availableAircraftList": [{"aircraftID": i} for i, _ in AIRCRAFT_IDS],  # default 모두 선택
            "inputMissionList": [],
        }
        # 임시 입력 버퍼(현재 미션 작성 중 자료)
        self.curMissionType = 1  # 1: coordinate, 2: line, 3: area
        self.curPoints = []              # coordinateList용: [{lat,lon,alt}, ...]
        self.curLineSegments = []        # lineList용: [ {width, coordinateList:[...]}, ... ]
        self._segmentOpen = False
        self._segmentPoints = []
        self.curAreaPolygons = []        # areaList용: [ {isHole, coordinateList:[...]}, ... ]
        self._polygonOpen = False
        self._polygonPoints = []
        self.captureEnabled = False

    # ─────────────────────────────────────────────────────
    def _build_ui(self):
        from PyQt5.QtCore import QTimer  # ★ 추가: 초기 비율 강제 적용용

        self.setStatusBar(QStatusBar(self))
        center = QWidget(self)
        root = QHBoxLayout(center)
        root.setContentsMargins(6, 6, 6, 6)
        self.setCentralWidget(center)

        splitter = QSplitter(self)
        splitter.setChildrenCollapsible(False)
        root.addWidget(splitter)

        # (좌) 지도
        self.map_view = QWebEngineView(self)
        splitter.addWidget(self.map_view)

        # (우) 입력 패널
        side = QWidget(self)
        side_lay = QVBoxLayout(side)
        side_lay.setContentsMargins(8, 8, 8, 8)

        # ── 패키지 그룹 ─────────────────────────────
        grp_pkg = QGroupBox("Package")
        gl = QGridLayout(grp_pkg)

        self.lbl_pkg_id = QLabel("inputMissionPackageID: -")
        self.btn_pkg_id = QPushButton("고유 ID 발급")
        self.btn_pkg_id.clicked.connect(self._on_assign_pkg_id)

        self.cmb_pkg_type = QComboBox()
        for v, text in PACK_TYPES:
            self.cmb_pkg_type.addItem(text, v)
        self.cmb_pkg_type.currentIndexChanged.connect(self._on_pkg_type_changed)

        self.cmb_main_sensor = QComboBox()
        for v, text in MAIN_SENSORS:
            self.cmb_main_sensor.addItem(text, v)
        self.cmb_main_sensor.currentIndexChanged.connect(self._on_main_sensor_changed)

        gl.addWidget(self.lbl_pkg_id,            0, 0, 1, 2)
        gl.addWidget(QLabel("inputMissionPackageType:"), 1, 0)
        gl.addWidget(self.cmb_pkg_type,          1, 1)
        gl.addWidget(QLabel("mainSensor (0~2):"),2, 0)
        gl.addWidget(self.cmb_main_sensor,       2, 1)
        side_lay.addWidget(grp_pkg)

        # ── 항공기 그룹 ─────────────────────────────
        grp_ac = QGroupBox("Available Aircraft (기본: 전부 사용)")
        ac_lay = QGridLayout(grp_ac)
        self.chk_ac = {}  # id -> QCheckBox
        for idx, (aid, text) in enumerate(AIRCRAFT_IDS):
            cb = QCheckBox(text)
            cb.setChecked(True)
            cb.stateChanged.connect(self._on_aircraft_changed)
            self.chk_ac[aid] = cb
            r = idx // 2; c = idx % 2
            ac_lay.addWidget(cb, r, c)
        side_lay.addWidget(grp_ac)

        # ── 미션 입력 그룹 ──────────────────────────
        grp_mis = QGroupBox("Mission (한 미션 = 하나의 InputMissionType)")
        ml = QGridLayout(grp_mis)

        self.lbl_next_mid = QLabel("다음 inputMissionID: 1")
        self.cmb_mis_type = QComboBox()
        for v, text in MISSION_TYPES:
            self.cmb_mis_type.addItem(text, v)
        self.cmb_mis_type.currentIndexChanged.connect(self._on_mission_type_changed)

        self.spin_alt = QDoubleSpinBox()
        self.spin_alt.setDecimals(1)
        self.spin_alt.setRange(-10000.0, 50000.0)
        self.spin_alt.setValue(120.0)
        self.spin_alt.setSuffix(" m  (좌표 고도 기본값)")

        # Line 옵션
        self.spin_width = QDoubleSpinBox()
        self.spin_width.setDecimals(1)
        self.spin_width.setRange(0.0, 100000.0)
        self.spin_width.setValue(120.0)
        self.spin_width.setSuffix(" m  (lineList width)")

        # Area 옵션
        self.chk_is_hole = QCheckBox("isHole (다각형 제외영역)")
        self.chk_is_hole.setChecked(False)

        # 캡처/세그/폴리곤 제어
        self.btn_capture = QPushButton("지도 클릭 캡처: OFF")
        self.btn_capture.setCheckable(True)
        self.btn_capture.toggled.connect(self._toggle_capture)

        self.btn_seg_start = QPushButton("선분 시작")
        self.btn_seg_end   = QPushButton("선분 종료(등록)")
        self.btn_poly_start= QPushButton("다각형 시작")
        self.btn_poly_end  = QPushButton("다각형 종료(등록)")

        self.btn_seg_start.clicked.connect(self._start_line_segment)
        self.btn_seg_end.clicked.connect(self._end_line_segment)
        self.btn_poly_start.clicked.connect(self._start_area_polygon)
        self.btn_poly_end.clicked.connect(self._close_area_polygon)

        # 프리뷰
        self.lst_preview = QListWidget()

        # 커밋/저장
        self.btn_commit_mission = QPushButton("현재 미션 등록 (Commit)")
        self.btn_commit_mission.clicked.connect(self._on_commit_mission)

        self.btn_save = QPushButton("0201 저장(JSON)")
        self.btn_save.clicked.connect(self._save_package)

        # 배치
        row = 0
        ml.addWidget(self.lbl_next_mid,      row, 0, 1, 2); row += 1
        ml.addWidget(QLabel("inputMissionType:"), row, 0); ml.addWidget(self.cmb_mis_type, row, 1); row += 1
        ml.addWidget(QLabel("기본 altitude:"), row, 0); ml.addWidget(self.spin_alt, row, 1); row += 1

        # line/area 옵션
        ml.addWidget(QLabel("lineList width:"), row, 0); ml.addWidget(self.spin_width, row, 1); row += 1
        ml.addWidget(self.chk_is_hole, row, 0, 1, 2); row += 1

        # 캡처/세그/폴리곤 버튼
        ml.addWidget(self.btn_capture,   row, 0, 1, 2); row += 1
        ml.addWidget(self.btn_seg_start, row, 0); ml.addWidget(self.btn_seg_end, row, 1); row += 1
        ml.addWidget(self.btn_poly_start,row, 0); ml.addWidget(self.btn_poly_end, row, 1); row += 1

        # 미션 프리뷰 & 커밋/저장
        ml.addWidget(QLabel("프리뷰(현재 미션 입력상태):"), row, 0, 1, 2); row += 1
        ml.addWidget(self.lst_preview, row, 0, 1, 2); row += 1
        ml.addWidget(self.btn_commit_mission, row, 0, 1, 2); row += 1
        ml.addWidget(self.btn_save, row, 0, 1, 2); row += 1

        side_lay.addWidget(grp_mis)

        # 빈 공간
        spacer = QFrame(self); spacer.setFrameShape(QFrame.StyledPanel)
        side_lay.addWidget(spacer, 1)

        splitter.addWidget(side)

        # ★ 변경 1: 우측 패널 최소 폭 설정(입력 가독성 유지)
        side.setMinimumWidth(400)  # 380~420 사이 취향대로 조절

        # ★ 변경 2: 기본 스트레치 비율(지도:패널 ≈ 80:20)
        splitter.setStretchFactor(0, 5)   # 지도
        splitter.setStretchFactor(1, 1)   # 패널

        # ★ 변경 3: 레이아웃 완성 직후 픽셀 기반으로 정확히 80:20 재적용
        def _apply_initial_sizes():
            total = max(splitter.size().width(), self.width())
            if total <= 0:
                total = 1400  # 초기 안전값
            left = int(total * 0.80)
            right = max(side.minimumWidth(), total - left)
            splitter.setSizes([left, right])
        QTimer.singleShot(0, _apply_initial_sizes)

        # WebChannel 브릿지
        self.bridge = MapBridge()
        ch = QWebChannel(self.map_view.page())
        ch.registerObject("bridge", self.bridge)
        self.map_view.page().setWebChannel(ch)
        self.bridge.pointClicked.connect(self._on_map_click)

        # 초기 상태 반영
        self._refresh_next_ids()
        self._on_mission_type_changed()


    # ─────────────────────────────────────────────────────
    # 메서드: _load_map — Folium 지도 로드
    def _load_map(self):
        """
        메서드: _load_map
        역할: Folium 지도를 생성하여 QWebEngineView에 로드 + loadFinished 시그널 연결
        """
        from PyQt5.QtCore import QUrl
        import os
        html_path = write_map_html("map.html", center=(37.5, 127.0), zoom=12)
        self._map_ready = False
        self.map_view.setUrl(QUrl.fromLocalFile(os.path.abspath(html_path)))
        self.map_view.loadFinished.connect(self._on_map_loaded)
        self.statusBar().showMessage("지도 로드 중…")

    def _on_map_loaded(self, ok: bool):
        """
        메서드: _on_map_loaded
        역할: 지도(HTML)가 로드되어 JS 유틸이 준비되었음을 표시
        """
        self._map_ready = bool(ok)
        self.statusBar().showMessage("지도 로드 완료" if ok else "지도 로드 실패")

    def _js(self, code: str):
        """
        메서드: _js
        역할: 현재 페이지에서 JS 실행(지도 로드 전이면 지연 재시도)
        """
        if not getattr(self, "_map_ready", False):
            # 로드가 아직이면 약간 지연 후 재시도
            from PyQt5.QtCore import QTimer
            QTimer.singleShot(100, lambda: self._js(code))
            return
        self.map_view.page().runJavaScript(code)

    # ─────────────────────────────────────────────────────
    # 메서드: _on_map_click — 지도 클릭 시 현재 모드에 따라 좌표 추가
    def _on_map_click(self, lat: float, lon: float):
        """
        메서드: _on_map_click
        역할: 지도 클릭 좌표를 수집 + 즉시 지도에 시각화
        """
        if not self.captureEnabled:
            return
        alt = float(self.spin_alt.value())
        pt = {"latitude": float(lat), "longitude": float(lon), "altitude": alt}

        if self.curMissionType == 1:  # coordinateList
            self.curPoints.append(pt)
            self._preview_append(f"[POINT] ({lat:.6f}, {lon:.6f}, {alt:g})")
            self._js(f"window.__cmpk && window.__cmpk.draw.addPoint({lat}, {lon}, {alt});")

        elif self.curMissionType == 2:  # lineList
            if not self._segmentOpen:
                from PyQt5.QtWidgets import QMessageBox
                QMessageBox.information(self, "알림", "먼저 '선분 시작'을 누르세요.")
                return
            self._segmentPoints.append(pt)
            self._preview_append(f"[LINE-SEG] + ({lat:.6f}, {lon:.6f}, {alt:g})")
            self._js(f"window.__cmpk && window.__cmpk.draw.lineAdd({lat}, {lon}, {alt});")

        else:  # areaList
            if not self._polygonOpen:
                from PyQt5.QtWidgets import QMessageBox
                QMessageBox.information(self, "알림", "먼저 '다각형 시작'을 누르세요.")
                return
            self._polygonPoints.append(pt)
            self._preview_append(f"[AREA-POLY] + ({lat:.6f}, {lon:.6f}, {alt:g})")
            self._js(f"window.__cmpk && window.__cmpk.draw.areaAdd({lat}, {lon}, {alt});")

        self.statusBar().showMessage(f"Map clicked: lat={lat:.6f}, lon={lon:.6f}")

    # ─────────────────────────────────────────────────────
    # 메서드: _on_assign_pkg_id — inputMissionPackageID 고유 발급
    def _on_assign_pkg_id(self):
        assign_package_id_inplace(self.cmpk)
        self.lbl_pkg_id.setText(f"inputMissionPackageID: {self.cmpk['inputMissionPackageID']}")
        self.statusBar().showMessage("고유 패키지 ID 발급 완료")

    # ─────────────────────────────────────────────────────
    # 메서드: _on_pkg_type_changed — 패키지 타입 선택 반영
    def _on_pkg_type_changed(self):
        self.cmpk["inputMissionPackageType"] = int(self.cmb_pkg_type.currentData())

    # ─────────────────────────────────────────────────────
    # 메서드: _on_main_sensor_changed — mainSensor 선택 반영
    def _on_main_sensor_changed(self):
        self.cmpk["mainSensor"] = int(self.cmb_main_sensor.currentData())

    # ─────────────────────────────────────────────────────
    # 메서드: _on_aircraft_changed — 사용 항공기 체크 반영
    def _on_aircraft_changed(self):
        used = []
        for aid, cb in self.chk_ac.items():
            if cb.isChecked():
                used.append({"aircraftID": aid})
        if not used:
            # 최소 1대는 선택되도록 경고만 표시(데이터는 비워둘 수 있음)
            self.statusBar().showMessage("경고: 선택된 항공기가 없습니다.", 5000)
        self.cmpk["availableAircraftList"] = used

    # ─────────────────────────────────────────────────────
    # 메서드: _on_mission_type_changed — 미션 타입 전환(1=coordinate,2=line,3=area)
    def _on_mission_type_changed(self):
        self.curMissionType = int(self.cmb_mis_type.currentData())
        # UI 활성/비활성
        is_line = self.curMissionType == 2
        is_area = self.curMissionType == 3
        self.spin_width.setEnabled(is_line)
        self.chk_is_hole.setEnabled(is_area)
        self.btn_seg_start.setEnabled(is_line)
        self.btn_seg_end.setEnabled(is_line)
        self.btn_poly_start.setEnabled(is_area)
        self.btn_poly_end.setEnabled(is_area)
        # 버퍼 초기화
        self.curPoints.clear()
        self.curLineSegments.clear()
        self._segmentOpen = False
        self._segmentPoints = []
        self.curAreaPolygons.clear()
        self._polygonOpen = False
        self._polygonPoints = []
        self.lst_preview.clear()

    # ─────────────────────────────────────────────────────
    # 메서드: _toggle_capture — 지도 클릭 캡처 ON/OFF
    def _toggle_capture(self, on: bool):
        self.captureEnabled = bool(on)
        self.btn_capture.setText(f"지도 클릭 캡처: {'ON' if on else 'OFF'}")
        self.statusBar().showMessage("지도 클릭을 좌표로 수집합니다." if on else "캡처 해제")

    # ─────────────────────────────────────────────────────
    # 메서드: _start_line_segment — lineList용 선분 시작
    def _start_line_segment(self):
        if self.curMissionType != 2:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.information(self, "알림", "미션 타입을 'lineList'로 설정하세요.")
            return
        if self._segmentOpen:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.information(self, "알림", "이미 선분 입력 중입니다.")
            return
        self._segmentOpen = True
        self._segmentPoints = []
        self._preview_append("[LINE-SEG] 시작")
        width = float(self.spin_width.value())
        self._js(f"window.__cmpk && window.__cmpk.draw.lineStart({width});")

    # ─────────────────────────────────────────────────────
    # 메서드: _end_line_segment — lineList용 선분 종료(등록)
    def _end_line_segment(self):
        if self.curMissionType != 2 or not self._segmentOpen:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.information(self, "알림", "시작되지 않은 선분입니다.")
            return
        if len(self._segmentPoints) < 2:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(self, "오류", "선분에는 2점 이상이 필요합니다.")
            return
        seg = {
            "width": float(self.spin_width.value()),
            "coordinateList": list(self._segmentPoints)
        }
        self.curLineSegments.append(seg)
        self._segmentOpen = False
        self._segmentPoints = []
        self._preview_append(f"[LINE-SEG] 종료/등록 (총 {len(seg['coordinateList'])}점, width={seg['width']})")
        self._js("window.__cmpk && window.__cmpk.draw.lineEnd();")

    # ─────────────────────────────────────────────────────
    # 메서드: _start_area_polygon — areaList용 다각형 시작
    def _start_area_polygon(self):
        if self.curMissionType != 3:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.information(self, "알림", "미션 타입을 'areaList'로 설정하세요.")
            return
        if self._polygonOpen:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.information(self, "알림", "이미 다각형 입력 중입니다.")
            return
        self._polygonOpen = True
        self._polygonPoints = []
        self._preview_append("[AREA-POLY] 시작")
        is_hole = bool(self.chk_is_hole.isChecked())
        self._js(f"window.__cmpk && window.__cmpk.draw.areaStart({str(is_hole).lower()});")

    # ─────────────────────────────────────────────────────
    # 메서드: _close_area_polygon — areaList용 다각형 종료(등록)
    def _close_area_polygon(self):
        if self.curMissionType != 3 or not self._polygonOpen:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.information(self, "알림", "시작되지 않은 다각형입니다.")
            return
        if len(self._polygonPoints) < 3:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(self, "오류", "다각형에는 3점 이상이 필요합니다.")
            return
        poly = {
            "isHole": bool(self.chk_is_hole.isChecked()),
            "coordinateList": list(self._polygonPoints)
        }
        self.curAreaPolygons.append(poly)
        self._polygonOpen = False
        self._polygonPoints = []
        self._preview_append(f"[AREA-POLY] 종료/등록 (총 {len(poly['coordinateList'])}점, isHole={poly['isHole']})")
        self._js("window.__cmpk && window.__cmpk.draw.areaEnd();")

    # ─────────────────────────────────────────────────────
    # 메서드: _on_commit_mission — 현재 입력 버퍼를 하나의 미션으로 등록
    def _on_commit_mission(self):
        # 디테일 유효성 확인
        from PyQt5.QtWidgets import QMessageBox
        if self.curMissionType == 1 and len(self.curPoints) == 0:
            QMessageBox.warning(self, "오류", "좌표(POINT)를 하나 이상 입력하세요.")
            return
        if self.curMissionType == 2 and len(self.curLineSegments) == 0:
            QMessageBox.warning(self, "오류", "선분(LINE-SEG)을 하나 이상 등록하세요.")
            return
        if self.curMissionType == 3 and len(self.curAreaPolygons) == 0:
            QMessageBox.warning(self, "오류", "다각형(AREA-POLY)을 하나 이상 등록하세요.")
            return

        next_id = len(self.cmpk.get("inputMissionList", [])) + 1
        mission = {
            "inputMissionID": next_id,
            "inputMissionType": int(self.curMissionType),
            "isDone": False,
            "missionDetail": {}
        }
        if self.curMissionType == 1:
            mission["missionDetail"]["coordinateList"] = list(self.curPoints)
        elif self.curMissionType == 2:
            mission["missionDetail"]["lineList"] = list(self.curLineSegments)
        else:
            mission["missionDetail"]["areaList"] = list(self.curAreaPolygons)

        self.cmpk["inputMissionList"].append(mission)

        # 지도에 남기기: preview -> committed
        self._js("window.__cmpk && window.__cmpk.preview.commit();")

        # 버퍼 초기화 & 라벨 갱신
        self._on_mission_type_changed()
        self._refresh_next_ids()
        self.statusBar().showMessage(f"미션 등록 완료 (inputMissionID={next_id})", 5000)

    # ─────────────────────────────────────────────────────
    # 메서드: _save_package — 저장 직전 필수 보정 + JSON 저장
    def _save_package(self):
        # 필수: timestamp(ms since 2000), ID 시퀀스 보정/검증
        assign_timestamp_inplace(self.cmpk, force=False)
        ts_errs = validate_timestamp(self.cmpk)
        if ts_errs:
            QMessageBox.warning(self, "Timestamp 오류", "\n".join(ts_errs))
            return

        # 패키지 ID 미발급 시 경고
        if "inputMissionPackageID" not in self.cmpk or not isinstance(self.cmpk["inputMissionPackageID"], int):
            ret = QMessageBox.question(self, "패키지 ID 없음", "고유 패키지 ID가 없습니다. 지금 발급할까요?",
                                       QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
            if ret == QMessageBox.Yes:
                self._on_assign_pkg_id()
            else:
                return

        # 미션 ID 1..N 연속성 보장
        assign_mission_ids_inplace(self.cmpk)

        # ID 규칙 검증(패키지 외부 중복 체크는 생략 또는 외부에서 주입)
        id_errs = validate_sequential_ids(self.cmpk, existing_package_ids=None)
        if id_errs:
            QMessageBox.warning(self, "ID 규칙 오류", "\n".join(id_errs))
            return

        # 파일 저장
        path, _ = QFileDialog.getSaveFileName(self, "0201 저장", "0201.json", "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.cmpk, f, ensure_ascii=False, indent=2)
            self.statusBar().showMessage(f"저장 완료: {path}")
            QMessageBox.information(self, "완료", "0201 JSON 저장 완료")
        except Exception as e:
            QMessageBox.critical(self, "저장 실패", str(e))

    # ─────────────────────────────────────────────────────
    # 메서드: _refresh_next_ids — 우측 라벨 갱신
    def _refresh_next_ids(self):
        next_mid = len(self.cmpk.get("inputMissionList", [])) + 1
        self.lbl_next_mid.setText(f"다음 inputMissionID: {next_mid}")

    # ─────────────────────────────────────────────────────
    # 메서드: _preview_append — 프리뷰 리스트에 메시지 추가
    def _preview_append(self, text: str):
        self.lst_preview.addItem(text)
        self.lst_preview.scrollToBottom()
