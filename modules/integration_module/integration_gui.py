# -*- coding: utf-8 -*-
# modules/integration_module/integration_gui.py
# Integration GUI – 연동 모듈 전용 GUI
from __future__ import annotations

import sys, os, threading, json, re, time, random
from datetime import datetime, timezone
os.environ["KU_ROLE"] = "integration"
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]  # .../KU_LAHMUMT
for _p in (_ROOT, _ROOT / "modules", _ROOT / "modules" / "common"):
    _ps = str(_p)
    if _p.exists() and _ps not in sys.path:
        sys.path.insert(0, _ps)

from modules.common.qt_env import ensure_qt_platform
ensure_qt_platform()
from modules.common.process_console import emit_process_log, ensure_console, install_process_file_logging

ensure_console(os.getenv("KU_CONSOLE_TITLE", "KU Integration Console"))
install_process_file_logging("integration")

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, pyqtSignal, QTimer, Qt, QEvent, QObject, QPointF, QRect
from PyQt5.QtGui import QPainter, QColor, QPen, QFontMetrics, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QWidget, QLabel,
    QHBoxLayout, QVBoxLayout, QSlider, QLineEdit, QPushButton, QFileDialog,
    QGroupBox, QMessageBox, QSizePolicy, QTableWidget, QHeaderView, QTableWidgetItem,
    QCheckBox, QDialog, QFormLayout, QDialogButtonBox, QComboBox, QSpinBox, QDoubleSpinBox,
    QStyle, QStyleOptionSlider,
)

class ModeTickLabels(QWidget):
    def __init__(self, slider, labels, parent=None):
        super().__init__(parent)
        self._slider = slider
        self._labels = list(labels)
        self._pad = 0
        self._font = QFont("Malgun Gothic")
        self._font.setPointSize(8)
        metrics = QFontMetrics(self._font)
        self.setFixedHeight(max(30, metrics.height() * 2 + 2))
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self._slider.installEventFilter(self)
        self._sync_width()

    def _calc_pad(self):
        metrics = QFontMetrics(self._font)
        max_width = 0
        for label in self._labels:
            lines = str(label).splitlines() or [""]
            width = max(metrics.horizontalAdvance(line) for line in lines)
            if width > max_width:
                max_width = width
        rect_width = max(max_width + 6, 24)
        return int(rect_width / 2) + 2

    def _sync_width(self):
        self._pad = self._calc_pad()
        self.setFixedWidth(self._slider.width() + self._pad * 2)
        self.update()

    def eventFilter(self, obj, event):
        if obj is self._slider and event.type() == QEvent.Resize:
            self._sync_width()
        return super().eventFilter(obj, event)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.TextAntialiasing)
        painter.setPen(QColor("#6b7280"))
        painter.setFont(self._font)
        metrics = QFontMetrics(self._font)
        option = QStyleOptionSlider()
        self._slider.initStyleOption(option)
        style = self._slider.style()
        groove = style.subControlRect(QStyle.CC_Slider, option, QStyle.SC_SliderGroove, self._slider)
        handle = style.subControlRect(QStyle.CC_Slider, option, QStyle.SC_SliderHandle, self._slider)
        min_val = self._slider.minimum()
        max_val = self._slider.maximum()
        count = len(self._labels)
        if count < 2 or max_val == min_val:
            return
        step = (max_val - min_val) / (count - 1)
        available = groove.width() - handle.width()
        if available < 0:
            available = 0
        for idx, label in enumerate(self._labels):
            val = min_val + int(round(step * idx))
            pos = style.sliderPositionFromValue(min_val, max_val, val, available, option.upsideDown)
            x = self._pad + groove.x() + (handle.width() // 2) + pos
            lines = str(label).splitlines() or [""]
            text_width = max(metrics.horizontalAdvance(line) for line in lines)
            rect_width = max(text_width + 6, 24)
            x0 = int(x - rect_width / 2)
            rect = QRect(int(x0), 0, int(rect_width), self.height())
            painter.drawText(rect, Qt.AlignHCenter | Qt.AlignTop, label)
        painter.end()



# ───────── Qt 경고 필터 ─────────
def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message:
        return
    sys.stderr.write(message + "\n")
qInstallMessageHandler(_qt_silent_handler)

# ───────── 경로 부트스트랩 ─────────
def _bootstrap_paths():
    here = Path(__file__).resolve()
    modules_dir = here.parents[1]                # .../modules
    root = modules_dir.parent                    # .../<project root>
    common_dir = modules_dir / "common"
    for p in (modules_dir / "integration_module", common_dir, root):
        p_str = str(p)
        if p.exists() and p_str not in sys.path:
            sys.path.insert(0, p_str)
    try: os.chdir(root)
    except Exception: pass
    return root, common_dir
PROJECT_ROOT, COMMON_DIR = _bootstrap_paths()
from modules.common.settings_paths import fusion_runtime_working_dir

# ───────── 설정/라이선스 정규화 ─────────
def _ensure_fusion_configs():
    from modules.common.settings_paths import ensure_fusion_license_file, ensure_fusion_settings_file

    dst = ensure_fusion_settings_file(project_root=PROJECT_ROOT, common_dir=COMMON_DIR)
    if dst is None:
        raise FileNotFoundError("nFusionSettings.json/FusionSettings.json 이 없습니다.")
    ensure_fusion_license_file(project_root=PROJECT_ROOT, common_dir=COMMON_DIR)
    return str(dst)

# ───────── nFusion DLL/어셈블리 로드 ─────────
from dll_files.nFusionImports import *  # FusionNodeIoc, NodeMessenger, clr

def _load_msglib_and_deps():
    _clr = globals().get("clr", None)
    if _clr is None:
        try:
            from dll_files.nFusionImports import clr as _clr  # type: ignore
        except Exception:
            import clr as _clr  # type: ignore
    msg_dir = COMMON_DIR / "msg_files"
    stem = msg_dir / "MessageLibrary"
    try: _clr.AddReference(str(stem))
    except Exception: _clr.AddReference(str(stem.with_suffix(".dll")))
    for s in ("K4586Model", "K4586Model.Assist", "MiscUtil"):
        dll = msg_dir / (s + ".dll")
        if dll.exists():
            try: _clr.AddReference(str(dll.with_suffix("")))
            except Exception:
                try: _clr.AddReference(str(dll))
                except Exception: pass
_settings_path = _ensure_fusion_configs()
_ = _load_msglib_and_deps()

# 공용 수신 등록 (버스 핸들러들 로드)
from receive import *  # noqa

# 탭
from Tabs.integration_tab import IntegrationTab

# 상태 OK(=0102) 헬퍼
from modules.common.status_reporter import send_status_ok

from push_center import push_message
from modules.common import db_paths
from receive_center import register_listener


# ───────── 유틸: ms since 2000-01-01 ─────────
_EPOCH_2000_MS = 946684800000  # 2000-01-01 00:00:00 UTC
def _now_ms_since_2000() -> int:
    return int(time.time() * 1000) - _EPOCH_2000_MS


# ───────── 비주기 전송 다이얼로그들 ─────────
class _Dlg0202_PriorMissionInfo(QDialog):
    """0202 선행임무정보 입력."""
    def __init__(self, parent=None, default_source="DSC"):
        super().__init__(parent)
        self.setWindowTitle("0202 선행임무정보 보내기")
        lay = QFormLayout(self)
        self.ed_src = QLineEdit(default_source); lay.addRow("Source", self.ed_src)
        self.sp_mission_id = QSpinBox(); self.sp_mission_id.setRange(1, 2**31 - 1); lay.addRow("PriorMissionID", self.sp_mission_id)
        self.cb_type = QComboBox(); self.cb_type.addItem("1: 좌표지향", 1); self.cb_type.addItem("2: 표적추적", 2); lay.addRow("MissionType", self.cb_type)
        # type=1: 좌표
        self.sb_lat = QDoubleSpinBox(); self.sb_lat.setRange(-90.0, 90.0); self.sb_lat.setDecimals(6)
        self.sb_lon = QDoubleSpinBox(); self.sb_lon.setRange(-180.0, 180.0); self.sb_lon.setDecimals(6)
        self.sp_alt = QSpinBox(); self.sp_alt.setRange(0, 50000)
        lay.addRow("Latitude", self.sb_lat); lay.addRow("Longitude", self.sb_lon); lay.addRow("Altitude(m)", self.sp_alt)
        # type=2: 표적
        self.sp_target = QSpinBox(); self.sp_target.setRange(1, 2**31 - 1); lay.addRow("TargetID", self.sp_target)
        def _toggle(_=None):
            # 항상 편집 가능. MissionType에 따라 payload 구성만 달라짐.
            return
        self.cb_type.currentIndexChanged.connect(_toggle); _toggle()
        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=self)
        btns.accepted.connect(self.accept); btns.rejected.connect(self.reject); lay.addRow(btns)
    def build_body(self, ts_ms: int) -> dict:
        src = (self.ed_src.text() or "DSC").strip()
        pid = int(self.sp_mission_id.value()); mtype = int(self.cb_type.currentData())
        entry = {"priorMissionID": pid, "missionType": mtype}
        if mtype == 1:
            entry["coordinateOrientation"] = {
                "coordinate": {"latitude": float(self.sb_lat.value()),
                               "longitude": float(self.sb_lon.value()),
                               "altitude": int(self.sp_alt.value())}
            }
        else:
            entry["targetOrientation"] = {"targetID": int(self.sp_target.value())}
        return {"timestamp": int(ts_ms), "source": src, "priorMissionList": [entry]}


class _Dlg0801_InitialPlanCommand(QDialog):
    """0801 운용자 임무재계획 명령 입력."""
    def __init__(self, parent=None, default_source="DSC"):
        super().__init__(parent)
        self.setWindowTitle("0801 임무재계획 명령 보내기")
        lay = QFormLayout(self)
        self.ed_src = QLineEdit(default_source); lay.addRow("Source", self.ed_src)

        self.ed_replan_ts = QLineEdit(self)
        self.ed_replan_ts.setPlaceholderText("ms since 2000-01-01 (기본: 현재 시뮬레이션 시각)")
        lay.addRow("OperatorReplanRequestTime", self.ed_replan_ts)

        self.sp_input_pkg = QSpinBox(); self.sp_input_pkg.setRange(0, 2**31 - 1); lay.addRow("InputMissionPackageID", self.sp_input_pkg)
        self.sp_ref_pkg = QSpinBox(); self.sp_ref_pkg.setRange(0, 2**31 - 1); lay.addRow("MissionReferencePackageID", self.sp_ref_pkg)
        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=self)
        btns.accepted.connect(self.accept); btns.rejected.connect(self.reject); lay.addRow(btns)

    def prefill(self, ts_ms: int):
        try:
            self.ed_replan_ts.setText(str(int(ts_ms)))
        except Exception:
            self.ed_replan_ts.clear()

    def build_body(self, ts_ms: int) -> dict:
        src = (self.ed_src.text() or "DSC").strip()
        try:
            op_ts = int((self.ed_replan_ts.text() or "").strip() or int(ts_ms))
        except Exception:
            op_ts = int(ts_ms)
        return {
            "timestamp": int(ts_ms),
            "source": src,
            "operatorReplanRequestTime": op_ts,
            "inputMissionPackageID": int(self.sp_input_pkg.value()),
            "missionReferencePackageID": int(self.sp_ref_pkg.value()),
        }


class _Dlg0802_MandatoryCommand(QDialog):
    """0802 강제명령 입력."""
    def __init__(self, parent=None, default_source="DSC"):
        super().__init__(parent)
        self.setWindowTitle("0802 강제명령 보내기")
        lay = QFormLayout(self)
        self.ed_src = QLineEdit(default_source); lay.addRow("Source", self.ed_src)
        self.cb_air = QComboBox()
        self.cb_air.addItem("4: 무인기1", 4); self.cb_air.addItem("5: 무인기2", 5); self.cb_air.addItem("6: 무인기3", 6)
        lay.addRow("AircraftID", self.cb_air)
        self.cb_type = QComboBox()
        self.cb_type.addItem("1: 강제 대기", 1); self.cb_type.addItem("2: 강제 귀환", 2); self.cb_type.addItem("3: 강제 임무복귀", 3)
        lay.addRow("MandatoryType", self.cb_type)
        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=self)
        btns.accepted.connect(self.accept); btns.rejected.connect(self.reject); lay.addRow(btns)
    def build_body(self, ts_ms: int) -> dict:
        src = (self.ed_src.text() or "DSC").strip()
        return {
            "timestamp": int(ts_ms),
            "source": src,
            "aircraftID": int(self.cb_air.currentData()),
            "mandatoryType": int(self.cb_type.currentData()),
        }


class _Dlg0503_SystemRecommend(QDialog):
    """0503(협업기저임무 완료) 수신 시, Next/Repeat 를 빠르게 선택하는 다이얼로그."""

    ACTION_CANCEL = 0
    ACTION_NEXT = 1
    ACTION_REPEAT = 2

    def __init__(
        self,
        parent=None,
        *,
        system_recommend: int | None = None,
        current_input_id: int | None = None,
        next_input_id: int | None = None,
    ):
        super().__init__(parent)
        self.setModal(True)
        self.setObjectName("Dlg0503Recommend")
        self.setWindowTitle("Mission Recommend (0503)")
        self.setMinimumWidth(520)
        try:
            self.setWindowFlag(Qt.WindowContextHelpButtonHint, False)
        except Exception:
            pass

        rec_map = {
            1: "1: 다음 협업기저임무 추천",
            2: "2: 현재 협업기저임무 재수행 추천",
            3: "3: 모든 협업기저임무 완료",
        }
        rec_text = rec_map.get(int(system_recommend)) if system_recommend is not None else None
        if not rec_text:
            rec_text = f"systemRecommend={system_recommend}" if system_recommend is not None else "systemRecommend=unknown"

        title = QLabel("협업기저임무 완료")
        title.setObjectName("Title")
        subtitle = QLabel(f"(0503) {rec_text}")
        subtitle.setObjectName("Subtitle")

        if current_input_id is not None and next_input_id is not None:
            body_txt = f"현재 임무: {current_input_id}\n다음 임무: {next_input_id}"
        elif current_input_id is not None:
            body_txt = f"현재 임무: {current_input_id}"
        else:
            body_txt = "다음 행동을 선택하세요."
        body = QLabel(body_txt)
        body.setObjectName("Body")
        body.setWordWrap(True)

        btn_next = QPushButton("Next Mission\n(0803 execute=1)")
        btn_next.setObjectName("NextBtn")
        btn_repeat = QPushButton("Repeat Mission\n(0803 execute=2)")
        btn_repeat.setObjectName("RepeatBtn")

        # 추천 행동 강조/기본 포커스
        try:
            if int(system_recommend) == 2:
                btn_repeat.setProperty("recommended", True)
                btn_repeat.setDefault(True)
            else:
                btn_next.setProperty("recommended", True)
                btn_next.setDefault(True)
        except Exception:
            btn_next.setDefault(True)

        btn_next.clicked.connect(lambda: self.done(self.ACTION_NEXT))
        btn_repeat.clicked.connect(lambda: self.done(self.ACTION_REPEAT))

        btn_row = QWidget(self)
        btn_lay = QHBoxLayout(btn_row)
        btn_lay.setContentsMargins(0, 0, 0, 0)
        btn_lay.setSpacing(12)
        btn_lay.addWidget(btn_next)
        btn_lay.addWidget(btn_repeat)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 16)
        lay.setSpacing(10)
        lay.addWidget(title)
        lay.addWidget(subtitle)
        lay.addSpacing(4)
        lay.addWidget(body)
        lay.addSpacing(12)
        lay.addWidget(btn_row)

        # 미니멀 QSS(기존 Scenario GUI 기능은 유지하고, 빠른 선택만 제공)
        self.setStyleSheet(
            """
            #Dlg0503Recommend {
                background: #0b1220;
                border: 1px solid rgba(148,163,184,0.25);
                border-radius: 14px;
            }
            QLabel#Title { color: #e5e7eb; font-size: 18px; font-weight: 700; }
            QLabel#Subtitle { color: #93c5fd; font-size: 12px; }
            QLabel#Body { color: #cbd5e1; font-size: 12px; line-height: 1.35; }
            QPushButton {
                border: 1px solid rgba(148,163,184,0.25);
                border-radius: 12px;
                padding: 12px 14px;
                font-weight: 700;
                color: #e5e7eb;
                background: rgba(15,23,42,0.65);
            }
            QPushButton#NextBtn { background: rgba(37,99,235,0.92); border: 1px solid rgba(37,99,235,1.0); }
            QPushButton#NextBtn:hover { background: rgba(29,78,216,0.95); }
            QPushButton#RepeatBtn { background: rgba(14,165,233,0.92); border: 1px solid rgba(14,165,233,1.0); }
            QPushButton#RepeatBtn:hover { background: rgba(2,132,199,0.95); }
            QPushButton[recommended="true"] { border: 2px solid rgba(251,191,36,0.95); }
            """
        )

    def reject(self) -> None:
        self.done(self.ACTION_CANCEL)


# ───────── 0401 이동 미니맵 위젯 ─────────
class _MiniMap(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._hist = {}   # id -> list[(lat, lon)]
        self._types = {}  # id -> 'LAH' | 'UAV'
        self.setMinimumHeight(180)

        self._timer = QTimer(self)
        self._timer.setInterval(200)
        self._timer.timeout.connect(self.update)
        self._timer.start()

    def clear(self):
        self._hist.clear()
        self._types.clear()
        self.update()


    def _color_for(self, aid: int) -> QColor:
        typ = self._types.get(int(aid))
        if typ == "LAH":  # 유인(LAH)
            return QColor("#dc2626")  # red
        if typ == "UAV":  # 무인(UAV)
            return QColor("#2563eb")  # blue
        base = (hash(int(aid)) % 360)
        return QColor.fromHsl(base, 160, 140)

    def set_pos(self, aid: int, lat: float, lon: float, type_hint=None):
        if type_hint:
            self._types[int(aid)] = type_hint
        lst = self._hist.setdefault(int(aid), [])
        lst.append((float(lat), float(lon)))
        if len(lst) > 120:
            del lst[:-120]

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#f8fafc"))
        if not self._hist:
            p.setPen(QColor("#9aa0a6"))
            p.drawText(self.rect(), Qt.AlignCenter, "0401 수신 대기…")
            return

        # bounds
        all_pts = [pt for lst in self._hist.values() for pt in lst]
        lats = [x for x, _ in all_pts]; lons = [y for _, y in all_pts]
        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)
        pad = 1e-6
        if abs(max_lat - min_lat) < pad: max_lat += pad; min_lat -= pad
        if abs(max_lon - min_lon) < pad: max_lon += pad; min_lon -= pad

        w = self.width() - 16; h = self.height() - 16
        ox, oy = 8, 8

        p.setPen(QPen(QColor("#cbd5e1"), 1))
        p.drawRect(ox, oy, w, h)

        def map_pt(lat, lon):
            x = ox + (lon - min_lon) / (max_lon - min_lon) * w
            y = oy + h - (lat - min_lat) / (max_lat - min_lat) * h
            return x, y

        # trails
        for aid, lst in self._hist.items():
            if len(lst) < 1: continue
            col = self._color_for(aid)
            pen = QPen(col, 2); pen.setCosmetic(True)
            p.setPen(pen)
            prev = None
            for lat, lon in lst[-60:]:
                x, y = map_pt(lat, lon)
                if prev:
                    p.drawLine(QPointF(prev[0], prev[1]), QPointF(x, y))  # ← QPointF 로 타입 고정
                prev = (x, y)
            p.setBrush(col)
            p.drawEllipse(int(prev[0])-4, int(prev[1])-4, 8, 8)
        p.end()


# ───────── 0401 트래커 패널(표+미니맵) ─────────
class _Agent0401Panel(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("0401 이동 모니터", parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self._map = _MiniMap(self)

        self._tbl = QTableWidget(0, 5)
        self._tbl.setHorizontalHeaderLabels(["ID", "Lat", "Lon", "Alt", "Speed"])
        self._tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._tbl.verticalHeader().setVisible(False)
        self._tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        self._tbl.setFixedHeight(150)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 10)
        lay.setSpacing(8)
        lay.addWidget(self._map)
        lay.addWidget(self._tbl)

        self._row_of = {}  # id -> row

        try:
            # 버스 수신 → mark_received(msg_id, raw) 로 넘어옴
            from receive_center import register_listener
            register_listener("0401", self)
        except Exception:
            pass

    # ---- utils ----
    @staticmethod
    def _get_ci(d: dict, k: str):
        kl = k.lower()
        for kk, vv in d.items():
            if kk.lower() == kl:
                return vv
        return None

    def _extract_states(self, obj: dict):
        """
        (aid, lat, lon, alt, spd, type_label) 튜플 목록 반환
        type_label: 'LAH'|'UAV'|None
        """
        out = []
        if not isinstance(obj, dict):
            return out

        def _ci(d, k):
            if not isinstance(d, dict): return None
            kl = k.lower()
            for kk, vv in d.items():
                if kk.lower() == kl: return vv
            return None

        lst = _ci(obj, "AgentStateList")
        if isinstance(lst, list) and lst:
            for it in lst:
                if not isinstance(it, dict): continue
                aid = _ci(it, "AircraftID") or _ci(it, "agentID") or _ci(it, "vehicleID") or _ci(it, "id")
                coord = _ci(it, "Coordinate") or {}
                lat = _ci(coord, "Latitude") or _ci(coord, "latitude")
                lon = _ci(coord, "Longitude") or _ci(coord, "longitude")
                alt = _ci(coord, "Altitude") or _ci(coord, "altitude")
                vel = _ci(it, "Velocity") or {}
                spd = _ci(vel, "Speed") or _ci(vel, "speed")
                is_unm = _ci(it, "IsUnmaned")  # 원문 키 철자 유지
                # 타입 라벨
                typ = None
                try:
                    if is_unm is not None:
                        typ = "UAV" if int(is_unm) == 1 else "LAH"
                except Exception:
                    pass
                try:
                    if aid is not None and lat is not None and lon is not None:
                        out.append((int(aid), float(lat), float(lon),
                                    (None if alt is None else float(alt)),
                                    (None if spd is None else float(spd)),
                                    typ))
                except Exception:
                    continue
            return out

        # 단일형 백업
        aid=lat=lon=alt=spd=None

        # 2) 단일형(하위 키 전역 탐색)
        def walk(n):
            if isinstance(n, dict):
                for k, v in n.items():
                    yield k, v
                    yield from walk(v)
            elif isinstance(n, list):
                for x in n:
                    yield from walk(x)

        aid = None; lat = None; lon = None; alt = None; spd = None
        for k, v in walk(obj):
            kl = k.lower()
            try:
                if aid is None and kl in ("aircraftid","agentid","vehicleid","id"): aid = int(v)
                elif lat is None and kl == "latitude": lat = float(v)
                elif lon is None and kl == "longitude": lon = float(v)
                elif alt is None and kl in ("altitude","alt"): alt = float(v)
                elif spd is None and kl in ("speed","groundspeed"): spd = float(v)
            except Exception:
                continue
        if aid is not None and lat is not None and lon is not None:
            out.append((int(aid), float(lat), float(lon), alt, spd))
        return out

    # 버스 수신(등록됨) → UI 쓰레드에서 호출됨
    def mark_received(self, msg_id: str, raw: bytes | None = None):
        if str(msg_id).zfill(4) != "0401":
            return
        obj = self._parse_json(raw)
        if not isinstance(obj, dict):
            return

        for aid, lat, lon, alt, spd, typ in self._extract_states(obj):
            self._map.set_pos(aid, lat, lon, typ)    # ← 타입 전달
            self._upsert_row(aid, lat, lon, alt, spd)

    # ---- helpers ----
    def _parse_json(self, raw):
        if not raw: return None
        try:
            txt = raw.decode("utf-8", "ignore")
            return json.loads(txt)
        except Exception:
            try:
                m = re.search(r"\{.*\}", txt, flags=re.S)
                return json.loads(m.group(0)) if m else None
            except Exception:
                return None

    def _upsert_row(self, aid, lat, lon, alt, spd):
        row = self._row_of.get(aid, -1)
        if row < 0:
            row = self._tbl.rowCount()
            self._tbl.insertRow(row)
            self._tbl.setItem(row, 0, QTableWidgetItem(str(aid)))
            self._tbl.setItem(row, 1, QTableWidgetItem(f"{lat:.6f}"))
            self._tbl.setItem(row, 2, QTableWidgetItem(f"{lon:.6f}"))
            self._tbl.setItem(row, 3, QTableWidgetItem("-" if alt is None else f"{alt:.1f}"))
            self._tbl.setItem(row, 4, QTableWidgetItem("-" if spd is None else f"{spd:.1f}"))
            self._row_of[aid] = row
        else:
            self._tbl.item(row, 1).setText(f"{lat:.6f}")
            self._tbl.item(row, 2).setText(f"{lon:.6f}")
            if alt is not None: self._tbl.item(row, 3).setText(f"{alt:.1f}")
            if spd is not None: self._tbl.item(row, 4).setText(f"{spd:.1f}")


# ───────── 좌측 상단 기능 패널(0401/0402 각각 파일 선택 + 램프 + 시작 버튼 + 0401 모니터) ─────────
class _MissionSidePanel(QGroupBox):
    def __init__(self, on_log=None, parent=None):
        super().__init__("임무 수행 보조", parent)
        self._on_log = on_log
        lay = QVBoxLayout(self); lay.setContentsMargins(10,8,10,10); lay.setSpacing(10)

        # 0401 행
        row1 = QWidget(self); r1 = QHBoxLayout(row1); r1.setContentsMargins(0,0,0,0); r1.setSpacing(8)
        self._lamp_0401 = QLabel(row1); self._set_lamp(self._lamp_0401, False)
        lbl1 = QLabel("0401 Log", row1)
        self._path_0401 = QLineEdit(row1); self._path_0401.setReadOnly(True); self._path_0401.setPlaceholderText("선택된 파일 없음")
        btn1 = QPushButton("찾아보기", row1); btn1.setMinimumWidth(84)
        btn1.clicked.connect(lambda: self._browse("0401"))
        r1.addWidget(self._lamp_0401); r1.addWidget(lbl1); r1.addWidget(self._path_0401); r1.addWidget(btn1)
        lay.addWidget(row1)

        self._chk_force_wp0 = QCheckBox("wp id 강제 0 할당", self)
        self._chk_force_wp0.setToolTip("0401 송출 시 currentWaypointID.waypointID 값을 0으로 덮어씁니다.")
        lay.addWidget(self._chk_force_wp0)

        block_row = QHBoxLayout()
        block_row.setContentsMargins(0, 0, 0, 0)
        block_row.setSpacing(8)
        self._block_checks: dict[int, QCheckBox] = {}
        for aid, label in (
            (4, "무인기 1 차단"),
            (5, "무인기 2 차단"),
            (6, "무인기 3 차단"),
        ):
            chk = QCheckBox(label, self)
            block_row.addWidget(chk)
            self._block_checks[aid] = chk
        block_row.addStretch(1)
        lay.addLayout(block_row)

        # 0402 행
        row2 = QWidget(self); r2 = QHBoxLayout(row2); r2.setContentsMargins(0,0,0,0); r2.setSpacing(8)
        self._lamp_0402 = QLabel(row2); self._set_lamp(self._lamp_0402, False)
        lbl2 = QLabel("0402 Log", row2)
        self._path_0402 = QLineEdit(row2); self._path_0402.setReadOnly(True); self._path_0402.setPlaceholderText("선택된 파일 없음")
        btn2 = QPushButton("찾아보기", row2); btn2.setMinimumWidth(84)
        btn2.clicked.connect(lambda: self._browse("0402"))
        r2.addWidget(self._lamp_0402); r2.addWidget(lbl2); r2.addWidget(self._path_0402); r2.addWidget(btn2)
        lay.addWidget(row2)

        # 임무 데이터 기반 덮어쓰기 옵션
        self._chk_use_mission = QCheckBox("임무 데이터 기반 0401 덮어쓰기", self)
        self._chk_use_mission.setToolTip("임무 계획에 저장된 각 무인기의 비행 경로를 따라 currentWaypointID와 연료를 자동으로 갱신합니다.")
        lay.addWidget(self._chk_use_mission)

        # 비주기 명령 버튼 묶음
        self.grp_manual = QGroupBox("비주기 명령", self)
        manual_layout = QVBoxLayout(self.grp_manual); manual_layout.setContentsMargins(8,6,8,6); manual_layout.setSpacing(6)
        self.btn_0202 = QPushButton("0202 선행임무정보 보내기", self.grp_manual); manual_layout.addWidget(self.btn_0202)
        self.btn_0801 = QPushButton("0801 임무재계획 명령 보내기", self.grp_manual); manual_layout.addWidget(self.btn_0801)
        self.btn_0802 = QPushButton("0802 강제명령 보내기", self.grp_manual); manual_layout.addWidget(self.btn_0802)
        self.btn_0803_next = QPushButton("0803 \uB2E4\uC74C \uD611\uC5C5\uAE30\uC800\uC784\uBB34 (execute=1)", self.grp_manual); manual_layout.addWidget(self.btn_0803_next)
        self.btn_0803_repeat = QPushButton("0803 \uC7AC\uC218\uD589 (execute=2)", self.grp_manual); manual_layout.addWidget(self.btn_0803_repeat)
        self.btn_onmission2_last_wp = QPushButton("현재 협업기저임무 onMission=2 (마지막 WP)", self.grp_manual); manual_layout.addWidget(self.btn_onmission2_last_wp)
        self.cmb_0702_option = QComboBox(self.grp_manual)
        self.cmb_0702_option.setEnabled(False)
        manual_layout.addWidget(self.cmb_0702_option)
        self.cmb_0702_ignore = QComboBox(self.grp_manual)
        self.cmb_0702_ignore.addItem("ignore=0 (자동선택 없음)", 0)
        self.cmb_0702_ignore.addItem("ignore=1 (기존 임무 유지)", 1)
        self.cmb_0702_ignore.addItem("ignore=2 (신규 임무 선택)", 2)
        self.cmb_0702_ignore.setCurrentIndex(2)
        self.cmb_0702_ignore.setEnabled(False)
        manual_layout.addWidget(self.cmb_0702_ignore)
        self.chk_0702_auto = QCheckBox("autoExecution", self.grp_manual)
        self.chk_0702_auto.setChecked(False)
        self.chk_0702_auto.setEnabled(False)
        manual_layout.addWidget(self.chk_0702_auto)
        self.btn_0702 = QPushButton("0702 의사결정 결과 보내기", self.grp_manual)
        self.btn_0702.setEnabled(False)
        manual_layout.addWidget(self.btn_0702)
        lay.addWidget(self.grp_manual)

        # 시작 버튼
        self.btn_start = QPushButton("임무 수행 모사 시작", self)
        self.btn_start.setMinimumHeight(90)
        self.btn_start.setMaximumHeight(110)
        self.btn_start.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        lay.addWidget(self.btn_start, 0, Qt.AlignTop)

        # 0401 이동 모니터
        self.tracker_0401 = _Agent0401Panel(self)
        lay.addWidget(self.tracker_0401)

        lay.addStretch(1)
        self.setMinimumWidth(360)

        self._paths_0401: list[str] = []
        self._paths_0402: list[str] = []

    def _set_lamp(self, w: QLabel, on: bool):
        w.setFixedSize(12,12)
        w.setStyleSheet(f"border-radius:6px; background:{'#16a34a' if on else '#dc2626'}; border:1px solid rgba(0,0,0,0.1);")

    def _browse(self, kind: str):
        paths, _ = QFileDialog.getOpenFileNames(self, f"{kind} 파일 선택", "", "NDJSON/JSON (*.ndjson *.json);;모든 파일 (*.*)")
        if not paths:
            return
        if kind == "0401":
            self._paths_0401 = list(paths)
            self._path_0401.setText(paths[0] if len(paths)==1 else f"{len(paths)}개 파일 선택")
            self._set_lamp(self._lamp_0401, True)
            try: self.tracker_0401._map.clear()  # ← 새 파일 선택 시 히스토리 초기화
            except Exception: pass
        else:
            self._paths_0402 = list(paths)
            self._path_0402.setText(paths[0] if len(paths)==1 else f"{len(paths)}개 파일 선택")
            self._set_lamp(self._lamp_0402, True)
        if callable(self._on_log): self._on_log(f"[FILES] {kind} → {', '.join(paths)}")

    def selected_paths_0401(self) -> list[str]:
        return list(self._paths_0401)

    def selected_paths_0402(self) -> list[str]:
        return list(self._paths_0402)

    def use_mission_overlay(self) -> bool:
        return bool(self._chk_use_mission.isChecked())

    def force_wp_zero(self) -> bool:
        try:
            return bool(self._chk_force_wp0.isChecked())
        except Exception:
            return False

    def blocked_uav_ids(self) -> set[int]:
        try:
            return {
                aid for aid, chk in self._block_checks.items() if chk.isChecked()
            }
        except Exception:
            return set()

    def update_decision_options(self, options: list[dict]) -> None:
        self.cmb_0702_option.clear()
        if not options:
            self.cmb_0702_option.setEnabled(False)
            self.cmb_0702_ignore.setEnabled(False)
            self.chk_0702_auto.setEnabled(False)
            self.btn_0702.setEnabled(False)
            return

        for opt in options:
            if not isinstance(opt, dict):
                continue
            option_id = opt.get("optionID")
            mission_plan_id = opt.get("missionPlanID")
            label = f"Option {option_id} → Plan {mission_plan_id}"
            self.cmb_0702_option.addItem(label, mission_plan_id)

        if self.cmb_0702_option.count() > 0:
            self.cmb_0702_option.setCurrentIndex(0)
            self.cmb_0702_ignore.setCurrentIndex(2)
            self.chk_0702_auto.setChecked(False)
            self.cmb_0702_ignore.setEnabled(True)
            self.chk_0702_auto.setEnabled(True)
            self.cmb_0702_option.setEnabled(True)
            self.btn_0702.setEnabled(True)
        else:
            self.cmb_0702_option.setEnabled(False)
            self.cmb_0702_ignore.setEnabled(False)
            self.chk_0702_auto.setEnabled(False)
            self.btn_0702.setEnabled(False)

    def selected_decision_plan_id(self) -> int | None:
        data = self.cmb_0702_option.currentData()
        if data is None:
            return None
        try:
            return int(data)
        except (TypeError, ValueError):
            return None

    def selected_decision_ignore(self) -> int:
        data = self.cmb_0702_ignore.currentData()
        try:
            return int(data)
        except (TypeError, ValueError):
            return 0

    def decision_auto_execution(self) -> bool:
        return bool(self.chk_0702_auto.isChecked())

    def set_extra_buttons_enabled(self, enabled: bool):
        try:
            self.grp_manual.setEnabled(bool(enabled))
        except Exception:
            pass



class _MissionPlanWaypointOverlay:
    """MissionPlan/IndividualMissionPlan/FlightPath을 이용해 0401 메시지를 보정한다."""

    def __init__(self, logger):
        self._log = logger
        self._missions_by_input = {}
        self._input_order = []
        self._current_input_index = -1
        self._current_input_id = None
        self._cursor = {}
        self._ticks = {}
        self._fuel = {}
        self._fuel_step = {}
        self._plan_key = None
        self._input_package_id = None
        self._prepared = False
        self._mission_done = False
        self._current_mission_start_idx = 0
        self._current_mission_base_ts = 0.0
        try:
            self._advance_every = max(1, int(os.environ.get("KU_0401_WP_STEP", "10")))
        except Exception:
            self._advance_every = 20

    def _log_info(self, text: str) -> None:
        if callable(self._log):
            self._log(text)

    def disable(self) -> None:
        self._prepared = False
        self._missions_by_input.clear()
        self._input_order = []
        self._current_input_index = -1
        self._current_input_id = None
        self._cursor.clear()
        self._ticks.clear()
        self._fuel.clear()
        self._fuel_step.clear()
        self._mission_done = False
        self._plan_key = None
        self._input_package_id = None

    def prepare(self, *, force: bool = False) -> bool:
        mission_plan_path = self._find_latest_mission_plan()
        if mission_plan_path is None:
            self._log_info("[REPLAY] MissionPlan 파일을 찾을 수 없습니다.")
            self.disable()
            return False

        plan_key = (str(mission_plan_path), mission_plan_path.stat().st_mtime)
        if not force and self._prepared and self._plan_key == plan_key:
            self._reset_progress()
            return True

        if not self._load_from_plan(mission_plan_path):
            self.disable()
            return False

        self._plan_key = plan_key
        self._prepared = True
        self._reset_progress()

        summary_items = []
        for input_id in self._input_order:
            mapping = self._missions_by_input.get(input_id) or {}
            summary_items.append(f"{input_id}({len(mapping)}대)")
        summary = ", ".join(summary_items) if summary_items else "없음"
        self._log_info(f"[REPLAY] 임무 기반 0401 덮어쓰기 준비 완료 (입력임무: {summary}, step={self._advance_every})")
        return True

    def has_inputs(self) -> bool:
        return bool(self._input_order)

    def current_input_id(self):
        return self._current_input_id

    def peek_next_input_id(self):
        next_idx = self._current_input_index + 1
        if 0 <= next_idx < len(self._input_order):
            return self._input_order[next_idx]
        return None

    def advance_to_next_input(self):
        if not self._prepared or not self._input_order:
            return None
        if self._current_input_index + 1 >= len(self._input_order):
            self._current_input_id = None
            self._mission_done = True
            return None
        self._current_input_index += 1
        self._current_input_id = self._input_order[self._current_input_index]
        seq_map = self._missions_by_input.get(self._current_input_id, {})
        self._cursor = {aid: 0 for aid in seq_map}
        self._ticks = {aid: 0 for aid in seq_map}
        self._mission_done = False
        for aid in seq_map:
            self._fuel.pop(aid, None)
            self._fuel_step.pop(aid, None)
        return self._current_input_id

    def set_current_input_id(self, input_id: int | None, *, reset: bool = True) -> int | None:
        if not self._prepared or input_id is None:
            return None
        try:
            input_id_int = int(input_id)
        except Exception:
            return None
        if input_id_int not in self._input_order:
            return None
        try:
            idx = self._input_order.index(input_id_int)
        except ValueError:
            return None
        self._current_input_index = int(idx)
        self._current_input_id = int(input_id_int)
        if reset:
            seq_map = self._missions_by_input.get(self._current_input_id, {})
            self._cursor = {aid: 0 for aid in seq_map}
            self._ticks = {aid: 0 for aid in seq_map}
            self._mission_done = False
            for aid in seq_map:
                self._fuel.pop(aid, None)
                self._fuel_step.pop(aid, None)
        return self._current_input_id

    @staticmethod
    def _select_next_pending_input_id(input_plan: dict) -> int | None:
        if not isinstance(input_plan, dict):
            return None
        mission_list = input_plan.get("inputMissionList") or []
        last_id: int | None = None
        for item in mission_list:
            if not isinstance(item, dict):
                continue
            try:
                input_id = int(item.get("inputMissionID"))
            except Exception:
                continue
            last_id = input_id
            if not bool(item.get("isDone")):
                return input_id
        return last_id

    def resolve_current_input_id_from_db(self, *, reset: bool = True) -> int | None:
        if not self._prepared:
            return None
        pkg_id = self._input_package_id
        if pkg_id is None:
            return None
        input_plan = self._read_input_plan(int(pkg_id))
        if not input_plan:
            return None
        candidate = self._select_next_pending_input_id(input_plan)
        if candidate is None:
            return None
        return self.set_current_input_id(int(candidate), reset=reset)

    def has_next_input(self) -> bool:
        return self._prepared and (self._current_input_index + 1) < len(self._input_order)

    def reset_current_input(self) -> None:
        if not self._prepared or self._current_input_id is None:
            return
        seq_map = self._missions_by_input.get(self._current_input_id, {})
        self._cursor = {aid: 0 for aid in seq_map}
        self._ticks = {aid: 0 for aid in seq_map}
        for aid in seq_map:
            self._fuel.pop(aid, None)
            self._fuel_step.pop(aid, None)
        self._mission_done = False

    def last_waypoints_for_current(self) -> dict[int, int]:
        if self._current_input_id is None:
            return {}
        seq_map = self._missions_by_input.get(self._current_input_id, {})
        out: dict[int, int] = {}
        for aid, seq in seq_map.items():
            if not seq:
                continue
            try:
                out[int(aid)] = int(seq[-1])
            except Exception:
                continue
        return out

    def apply(self, body: dict) -> bool:
        if not self._prepared or self._current_input_id is None:
            return False
        seq_map = self._missions_by_input.get(self._current_input_id, {})
        if not seq_map:
            if not self._mission_done:
                self._mission_done = True
                return True
            return True

        agent_list = None
        if isinstance(body, dict):
            for key in ("agentStateList", "AgentStateList"):
                val = body.get(key)
                if isinstance(val, list):
                    agent_list = val
                    break
        if not isinstance(agent_list, list):
            return False

        advance_every = max(1, int(self._advance_every))
        for agent in agent_list:
            if not isinstance(agent, dict):
                continue
            aid = self._get_int(agent, "aircraftID")
            if aid is None:
                continue
            seq = seq_map.get(aid)
            if not seq:
                continue
            idx = self._cursor.get(aid, 0)
            tick = self._ticks.get(aid, 0) + 1
            if idx < len(seq):
                if tick >= advance_every:
                    idx += 1
                    tick = 0
                if idx > len(seq):
                    idx = len(seq)
            else:
                idx = len(seq)
                tick = 0
            self._cursor[aid] = idx
            self._ticks[aid] = tick
            if seq:
                cur_idx = min(idx, len(seq) - 1)
                is_unmanned = self._get_int(agent, "isUnmanned")
                if is_unmanned is None:
                    is_unmanned = 1 if aid >= 4 else 0
                on_mission = 1
                if int(is_unmanned) == 1:
                    info = self._ensure_unmanned_info(agent)
                    current = info.get("currentWaypointID")
                    if not isinstance(current, dict):
                        current = {}
                        info["currentWaypointID"] = current
                    current["waypointID"] = seq[cur_idx]
                    info["onMission"] = on_mission
                agent["onMission"] = on_mission
            self._apply_fuel_decay(agent, aid)

        completed = True
        for aid, seq in seq_map.items():
            if seq and self._cursor.get(aid, 0) < len(seq):
                completed = False
                break
        if completed and not self._mission_done:
            self._mission_done = True
            return True
        return False

    def _reset_progress(self) -> None:
        self._current_input_index = -1
        self._current_input_id = None
        self._cursor = {}
        self._ticks = {}
        self._fuel.clear()
        self._fuel_step.clear()
        self._mission_done = False
        self._current_mission_start_idx = 0
        self._current_mission_base_ts = 0.0

    def _find_latest_mission_plan(self):
        try:
            mission_plan_dir = db_paths.get_db_subpath("MissionPlan")
        except Exception:
            return None
        try:
            entries = list(Path(mission_plan_dir).glob("*.json"))
        except Exception:
            entries = []
        if not entries:
            return None
        return max(entries, key=lambda p: p.stat().st_mtime)

    def reset_persisted_progress_for_latest_plan(self) -> dict[str, int | None]:
        summary: dict[str, int | None] = {
            "missionPlanID": None,
            "inputMissionPackageID": None,
            "inputPlans": 0,
            "inputMissions": 0,
            "individualPlans": 0,
            "individualMissions": 0,
            "flightPaths": 0,
            "waypoints": 0,
        }
        plan_path = self._find_latest_mission_plan()
        if plan_path is None:
            return summary
        summary["missionPlanID"] = self._safe_int(plan_path.stem)
        plan_data = self._read_json_file(plan_path, "MissionPlan")
        if not isinstance(plan_data, dict):
            return summary

        input_pkg_id = self._safe_int(
            self._get_case_insensitive(plan_data, "inputMissionPackageID")
            or self._get_case_insensitive(plan_data, "InputMissionPackageID")
        )
        summary["inputMissionPackageID"] = input_pkg_id
        if input_pkg_id is not None:
            input_path = self._db_json_path("InputMissionPlan", int(input_pkg_id))
            input_data = self._read_json_file(input_path, "InputMissionPlan") if input_path else None
            if isinstance(input_data, dict):
                changed_count = 0
                for mission in self._get_case_insensitive(input_data, "inputMissionList") or []:
                    if isinstance(mission, dict) and self._reset_done_flag(mission):
                        changed_count += 1
                if changed_count and self._write_json_file(input_path, input_data, "InputMissionPlan"):
                    summary["inputPlans"] = int(summary["inputPlans"] or 0) + 1
                    summary["inputMissions"] = int(summary["inputMissions"] or 0) + changed_count

        individual_package_ids: set[int] = set()
        for entry in self._get_case_insensitive(plan_data, "aircraftList") or []:
            if not isinstance(entry, dict):
                continue
            imp_id = self._get_int(entry, "individualMissionPackageID")
            if imp_id is not None:
                individual_package_ids.add(int(imp_id))

        flight_path_ids: set[int] = set()
        for imp_id in sorted(individual_package_ids):
            imp_path = self._db_json_path("IndividualMissionPlan", int(imp_id))
            imp_data = self._read_json_file(imp_path, "IndividualMissionPlan") if imp_path else None
            if not isinstance(imp_data, dict):
                continue
            changed_count = 0
            for mission in self._get_case_insensitive(imp_data, "individualMissionList") or []:
                if not isinstance(mission, dict):
                    continue
                path_id = self._get_int(mission, "pathID")
                if path_id is not None:
                    flight_path_ids.add(int(path_id))
                if self._reset_done_flag(mission):
                    changed_count += 1
            if changed_count and self._write_json_file(imp_path, imp_data, "IndividualMissionPlan"):
                summary["individualPlans"] = int(summary["individualPlans"] or 0) + 1
                summary["individualMissions"] = int(summary["individualMissions"] or 0) + changed_count

        for path_id in sorted(flight_path_ids):
            fp_path = self._db_json_path("FlightPath", int(path_id))
            fp_data = self._read_json_file(fp_path, "FlightPath") if fp_path else None
            if not isinstance(fp_data, dict):
                continue
            changed_count = 0
            seen_keys: set[str] = set()
            lower_map = {str(key).lower(): str(key) for key in fp_data.keys()}
            for expected_key in ("waypointList", "lahWaypointList", "uavWaypointList"):
                actual_key = expected_key if expected_key in fp_data else lower_map.get(expected_key.lower())
                if not actual_key or actual_key in seen_keys:
                    continue
                seen_keys.add(actual_key)
                waypoints = fp_data.get(actual_key)
                if not isinstance(waypoints, list):
                    continue
                for waypoint in waypoints:
                    if isinstance(waypoint, dict) and self._reset_done_flag(waypoint):
                        changed_count += 1
            if changed_count and self._write_json_file(fp_path, fp_data, "FlightPath"):
                summary["flightPaths"] = int(summary["flightPaths"] or 0) + 1
                summary["waypoints"] = int(summary["waypoints"] or 0) + changed_count

        return summary

    def _db_json_path(self, folder: str, file_id: int) -> Path | None:
        try:
            return Path(db_paths.get_db_subpath(str(folder), f"{int(file_id)}.json"))
        except Exception:
            return None

    def _read_json_file(self, path: Path | None, label: str):
        if path is None:
            return None
        try:
            return json.loads(Path(path).read_text(encoding="utf-8"))
        except Exception as exc:
            self._log_info(f"[REPLAY] {label} 로드 실패({Path(path).name}): {exc}")
            return None

    def _write_json_file(self, path: Path | None, payload: dict, label: str) -> bool:
        if path is None:
            return False
        tmp_path = Path(path).with_name(f"{Path(path).name}.tmp")
        try:
            tmp_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            tmp_path.replace(path)
            return True
        except Exception as exc:
            self._log_info(f"[REPLAY] {label} 저장 실패({Path(path).name}): {exc}")
            try:
                tmp_path.unlink(missing_ok=True)
            except Exception:
                pass
            return False

    @staticmethod
    def _get_case_insensitive(data: dict, key: str):
        if not isinstance(data, dict):
            return None
        key_lower = str(key).lower()
        for actual_key, value in data.items():
            if str(actual_key).lower() == key_lower:
                return value
        return None

    @staticmethod
    def _reset_done_flag(data: dict) -> bool:
        changed = False
        for key in ("isDone", "IsDone"):
            if key in data and bool(data.get(key)):
                data[key] = False
                changed = True
        return changed

    def _load_from_plan(self, plan_path: Path) -> bool:
        try:
            data = json.loads(plan_path.read_text(encoding="utf-8"))
        except Exception as exc:
            self._log_info(f"[REPLAY] MissionPlan 로드 실패: {exc}")
            return False


        missions_by_input: dict[int, dict[int, list[int]]] = {}
        for entry in data.get("aircraftList", []):
            if not isinstance(entry, dict):
                continue
            aid = self._safe_int(entry.get("aircraftID"))
            imp_id = self._safe_int(entry.get("individualMissionPackageID"))
            if aid is None or imp_id is None:
                continue
            imp_data = self._read_individual_plan(imp_id)
            if not imp_data:
                continue
            for mission in imp_data.get("individualMissionList", []):
                if not isinstance(mission, dict):
                    continue
                if bool(mission.get("isDone")):
                    continue
                related = mission.get("relatedMission") or {}
                input_id = self._safe_int(related.get("inputMissionID"))
                if input_id is None:
                    continue
                path_id = self._safe_int(mission.get("pathID"))
                if path_id is None:
                    missions_by_input.setdefault(input_id, {})
                    continue
                seq = self._read_waypoint_ids(path_id)
                if not seq:
                    missions_by_input.setdefault(input_id, {})
                    continue
                mapping = missions_by_input.setdefault(input_id, {})
                mapping.setdefault(aid, [])
                mapping[aid].extend(seq)

        input_order: list[int] = []
        input_pkg_id = self._safe_int(data.get("inputMissionPackageID") or data.get("InputMissionPackageID"))
        self._input_package_id = input_pkg_id
        if input_pkg_id is not None:
            input_plan = self._read_input_plan(input_pkg_id)
            if input_plan:
                seen: set[int] = set()
                for item in input_plan.get("inputMissionList") or []:
                    if not isinstance(item, dict):
                        continue
                    input_id = self._safe_int(item.get("inputMissionID"))
                    if input_id is None or input_id in seen:
                        continue
                    seen.add(input_id)
                    input_order.append(input_id)
                    missions_by_input.setdefault(input_id, {})

        if not missions_by_input and not input_order:
            self._log_info("[REPLAY] MissionPlan has no usable input missions.")
            return False
        if input_order:
            self._missions_by_input = {k: missions_by_input.get(k, {}) for k in input_order}
            self._input_order = input_order
        else:
            self._missions_by_input = {k: missions_by_input[k] for k in sorted(missions_by_input)}
            self._input_order = list(self._missions_by_input.keys())
        return True

    def _read_individual_plan(self, imp_id: int):
        try:
            imp_path = db_paths.get_db_subpath("IndividualMissionPlan", f"{imp_id}.json")
        except Exception:
            return None
        imp_path = Path(imp_path)
        if not imp_path.exists():
            self._log_info(f"[REPLAY] IndividualMissionPlan 파일이 없습니다: {imp_id}")
            return None
        try:
            return json.loads(imp_path.read_text(encoding="utf-8"))
        except Exception as exc:
            self._log_info(f"[REPLAY] IndividualMissionPlan 로드 실패({imp_id}): {exc}")
            return None

    def _read_input_plan(self, package_id: int):
        try:
            plan_path = db_paths.get_db_subpath("InputMissionPlan", f"{package_id}.json")
        except Exception:
            return None
        plan_path = Path(plan_path)
        if not plan_path.exists():
            self._log_info(f"[REPLAY] InputMissionPlan file not found: {package_id}")
            return None
        try:
            return json.loads(plan_path.read_text(encoding="utf-8"))
        except Exception as exc:
            self._log_info(f"[REPLAY] InputMissionPlan load failed ({package_id}): {exc}")
            return None

    def _collect_waypoints(self, imp_id: int) -> list[int]:
        data = self._read_individual_plan(imp_id)
        if not data:
            return []
        seq: list[int] = []
        for mission in data.get("individualMissionList", []):
            if not isinstance(mission, dict):
                continue
            if bool(mission.get("isDone")):
                continue
            path_id = self._safe_int(mission.get("pathID"))
            if path_id is None:
                continue
            seq.extend(self._read_waypoint_ids(path_id))
        return seq

    def _read_waypoint_ids(self, path_id: int) -> list[int]:
        try:
            fp_path = db_paths.get_db_subpath("FlightPath", f"{path_id}.json")
        except Exception:
            return []
        fp_path = Path(fp_path)
        if not fp_path.exists():
            self._log_info(f"[REPLAY] FlightPath 파일이 없습니다: {path_id}")
            return []
        try:
            data = json.loads(fp_path.read_text(encoding="utf-8"))
        except Exception as exc:
            self._log_info(f"[REPLAY] FlightPath 로드 실패({path_id}): {exc}")
            return []
        waypoints = data.get("waypointList")
        if waypoints is None:
            lower_map = {str(k).lower(): k for k in data.keys()}
            for key in ("waypointlist", "lahwaypointlist", "uavwaypointlist"):
                actual = lower_map.get(key)
                if actual is not None:
                    waypoints = data.get(actual)
                    break
        out: list[int] = []
        for wp in waypoints or []:
            if not isinstance(wp, dict):
                continue
            wid = self._safe_int(wp.get("waypointID") or wp.get("WaypointID"))
            if wid is None:
                continue
            out.append(wid)
        return out

    def _apply_fuel_decay(self, agent: dict, aid: int) -> None:
        fuel_raw = agent.get("fuel")
        try:
            fuel_current = float(fuel_raw)
        except (TypeError, ValueError):
            return
        base = self._fuel.get(aid)
        if base is None:
            base = fuel_current
            self._fuel[aid] = base
        step = self._fuel_step.get(aid)
        if step is None:
            step = max(base * 0.0005, 0.02)
            self._fuel_step[aid] = step
        new_val = max(0.0, self._fuel[aid] - step)
        self._fuel[aid] = new_val
        agent["fuel"] = round(new_val, 2)
        info = self._ensure_unmanned_info(agent)
        warn = 0
        if new_val <= 10:
            warn = 2
        elif new_val <= 20:
            warn = 1
        info["fuelWarning"] = warn

    def _safe_int(self, value) -> int | None:
        try:
            if value is None:
                return None
            return int(value)
        except (TypeError, ValueError):
            return None

    def _get_int(self, data: dict, key: str) -> int | None:
        for k, v in data.items():
            if k.lower() == key.lower():
                return self._safe_int(v)
        return None

    def _ensure_unmanned_info(self, agent: dict) -> dict:
        for key in ("unmannedInfo", "UnmannedInfo"):
            info = agent.get(key)
            if isinstance(info, dict):
                if key != "unmannedInfo":
                    agent["unmannedInfo"] = info
                return info
        info = {}
        agent["unmannedInfo"] = info
        return info

class _ReplayManager(QObject):
    def __init__(self, tab: IntegrationTab, messenger, logger, side_panel: _MissionSidePanel):
        super().__init__(tab)
        self._tab = tab
        self._msgr = messenger
        self._log = logger
        self._side = side_panel
        self._tracker = side_panel.tracker_0401
        self._overlay = _MissionPlanWaypointOverlay(logger)
        self._overlay_active = False
        self._timers = []
        self._rows_0401 = []
        self._row_idx_0401 = 0
        self._prev_sim_ts_0401 = 0.0
        self._mission_timer = None
        self._awaiting_user = False
        self._pending_target_ts = 0.0
        self._anchor_0401 = 0.0
        self._anchor_all = 0.0
        self._current_mission_start_idx = 0
        self._current_mission_base_ts = 0.0
        self._anchor_ms: float | None = None
        self._t0_mono: float | None = None
        self._completion_timer = None
        self._completion_last_wp: dict[int, int] = {}
        self._last_positions: dict[int, tuple[float, float, float]] = {}
        self._completion_on_mission = 1
        self._completion_ticks_left: int | None = None
        self._completion_after = None

    def stop(self):
        for t in self._timers:
            try:
                t.stop()
            except Exception:
                pass
        self._timers.clear()
        self._anchor_ms = None
        self._t0_mono = None
        if self._mission_timer is not None:
            try:
                self._mission_timer.stop()
            except Exception:
                pass
            self._mission_timer = None
        self._rows_0401 = []
        self._row_idx_0401 = 0
        self._awaiting_user = False
        self._pending_target_ts = 0.0
        self._prev_sim_ts_0401 = 0.0
        self._anchor_0401 = 0.0
        self._anchor_all = 0.0
        self._last_positions.clear()
        self._overlay_active = False
        if callable(self._log):
            self._log("[REPLAY] 재생 중단")
        self._stop_completion_sim()

    def _start_clock(self, anchor_ms: float):
        self._anchor_ms = float(anchor_ms)
        self._t0_mono = time.monotonic()
        if callable(self._log):
            self._log(f"[CLOCK] sim anchor={self._anchor_ms}")

    def now_timestamp_ms(self) -> int:
        if self._anchor_ms is not None and self._t0_mono is not None:
            return int(self._anchor_ms + (time.monotonic() - self._t0_mono) * 1000.0)
        return _now_ms_since_2000()

    def _row_of(self, msg_id: str) -> int:
        try:
            tbl = self._tab.tbl_tx
            for r in range(tbl.rowCount()):
                if tbl.item(r, 0).text().strip() == msg_id:
                    return r
        except Exception:
            pass
        return -1

    def _extract_ts(self, obj) -> float | None:
        keys = ("timestamp", "time", "ts", "Timestamp", "Time", "Ts")

        def walk(node):
            if isinstance(node, dict):
                for k, v in node.items():
                    yield k, v
                    yield from walk(v)
            elif isinstance(node, list):
                for item in node:
                    yield from walk(item)

        for key, val in walk(obj):
            try:
                if any(key.lower() == kk.lower() for kk in keys):
                    return float(val)
            except Exception:
                pass
        return None

    def _iter_ndjson(self, path: str):
        with open(path, "r", encoding="utf-8", errors="ignore") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    yield json.loads(line)
                except Exception:
                    match = re.search(r"\{.*\}", line)
                    if match:
                        try:
                            yield json.loads(match.group(0))
                        except Exception:
                            pass

    def _stop_periodic_for(self, msg_id: str):
        try:
            timers = getattr(self._tab, "periodic_timers", {})
            timer = timers.get(msg_id)
            if timer:
                try:
                    timer.stop()
                except Exception:
                    pass
                try:
                    timer.deleteLater()
                except Exception:
                    pass
                try:
                    del timers[msg_id]
                except Exception:
                    pass
            try:
                tbl = getattr(self._tab, "tbl_tx", None)
                if tbl is not None:
                    for row in range(tbl.rowCount()):
                        code_item = tbl.item(row, 0)
                        if code_item and code_item.text().strip() == msg_id:
                            state_item = tbl.item(row, 2)
                            if state_item:
                                state_item.setText("전송 정지(리플레이)")
                            break
            except Exception:
                pass
        except Exception:
            pass

    def _reset_db_progress_on_start_enabled(self) -> bool:
        value = os.environ.get("KU_REPLAY_RESET_DB_ON_START", "1").strip().lower()
        return value not in {"0", "false", "no", "off"}

    def start_from_two(self, paths_0401: list[str], paths_0402: list[str]):
        try:
            self._stop_periodic_for("0401")
            self._stop_periodic_for("0402")
        except Exception:
            pass

        self.stop()
        try:
            self._tracker._map.clear()
        except Exception:
            pass
        self._last_positions.clear()

        if self._reset_db_progress_on_start_enabled():
            try:
                reset_summary = self._overlay.reset_persisted_progress_for_latest_plan()
                plan_id = reset_summary.get("missionPlanID")
                if plan_id is not None and callable(self._log):
                    total_reset = sum(
                        int(reset_summary.get(key) or 0)
                        for key in ("inputMissions", "individualMissions", "waypoints")
                    )
                    status = "초기화" if total_reset else "초기화 확인"
                    self._log(
                        "[REPLAY] SIM DB 완료 상태 "
                        f"{status}: MissionPlan={plan_id}, "
                        f"InputMissionPlan={reset_summary.get('inputMissionPackageID')}, "
                        f"inputDone={reset_summary.get('inputMissions')}, "
                        f"individualDone={reset_summary.get('individualMissions')}, "
                        f"waypointDone={reset_summary.get('waypoints')}"
                    )
            except Exception as exc:
                if callable(self._log):
                    self._log(f"[REPLAY] SIM DB 완료 상태 초기화 실패: {exc}")

        self._overlay_active = False
        if self._side.use_mission_overlay():
            if self._overlay.prepare(force=True) and self._overlay.has_inputs():
                self._overlay_active = True
            else:
                if callable(self._log):
                    self._log("[REPLAY] 임무 기반 덮어쓰기를 사용할 수 없어 기본 모드로 진행합니다.")
                self._overlay.disable()
        else:
            self._overlay.disable()

        buckets = {"0401": [], "0402": []}
        for path in paths_0401:
            for obj in self._iter_ndjson(path):
                buckets["0401"].append((self._extract_ts(obj), obj))
        for path in paths_0402:
            for obj in self._iter_ndjson(path):
                buckets["0402"].append((self._extract_ts(obj), obj))

        if not buckets["0401"] and not buckets["0402"]:
            if callable(self._log):
                self._log("[REPLAY] 선택된 0401/0402 레코드가 없습니다.")
            return

        def first_ts(rows):
            if not rows:
                return None
            rows_sorted = sorted(rows, key=lambda x: (float('inf') if x[0] is None else x[0]))
            for ts, _ in rows_sorted:
                if ts is not None:
                    return float(ts)
            return None

        anchor_0401 = first_ts(buckets["0401"])
        anchor = anchor_0401 if anchor_0401 is not None else first_ts(buckets["0402"])
        if anchor is None:
            anchor = 0.0
        if callable(self._log):
            self._log(f"[REPLAY] anchor = {anchor} (0401_first={anchor_0401})")
        self._start_clock(anchor)

        rows_0401 = sorted(buckets["0401"], key=lambda x: (float('inf') if x[0] is None else x[0]))
        rows_0402 = sorted(buckets["0402"], key=lambda x: (float('inf') if x[0] is None else x[0]))

        self._anchor_all = anchor
        self._anchor_0401 = anchor_0401 if anchor_0401 is not None else anchor

        if self._overlay_active and rows_0401:
            if self._overlay.advance_to_next_input() is None:
                self._overlay_active = False
            else:
                self._rows_0401 = rows_0401
                self._row_idx_0401 = 0
                self._prev_sim_ts_0401 = anchor
                self._awaiting_user = False
                if callable(self._log):
                    self._log(f"[REPLAY] 협업기저임무 {self._overlay.current_input_id()}부터 재생합니다.")
                self._schedule_next_overlay_row(initial=True)
        if not self._overlay_active:
            self._fallback_schedule(rows_0401, anchor, anchor_0401)

        self._schedule_0402_rows(rows_0402, anchor)

    def _fallback_schedule(self, rows_0401, anchor, anchor_0401):
        step = 200
        if rows_0401:
            last_sim_ts = anchor
            for ts, obj in rows_0401:
                sim_ts = ts if ts is not None else (last_sim_ts + step)
                if sim_ts < last_sim_ts:
                    sim_ts = last_sim_ts
                delay = int(max(0, sim_ts - anchor))
                last_sim_ts = sim_ts
                self._schedule_send("0401", obj, delay)
            if callable(self._log):
                self._log(f"[REPLAY] 0401 {len(rows_0401)}건 재생 완료 (5Hz, anchor={anchor})")

    def _schedule_0402_rows(self, rows_0402, anchor):
        if not rows_0402:
            return
        count = 0
        for ts, obj in rows_0402:
            if ts is None:
                if callable(self._log):
                    self._log("[REPLAY] 0402: ts가 없어 skip")
                continue
            delay = int(max(0, ts - anchor))
            self._schedule_send("0402", obj, delay)
            count += 1
        if callable(self._log):
            self._log(f"[REPLAY] 0402 {count}건 재생 완료 (event, anchor={anchor})")

    def _schedule_send(self, msg_id: str, body: dict, delay_ms: int):
        row = self._row_of(msg_id)

        def do_send():
            self._emit_message(msg_id, body, row)

        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.setInterval(int(max(0, delay_ms)))
        timer.timeout.connect(do_send)
        timer.start()
        self._timers.append(timer)

    def _iter_agent_states(self, obj):
        def get_ci(d, k):
            if not isinstance(d, dict):
                return None
            kl = k.lower()
            for kk, vv in d.items():
                if kk.lower() == kl:
                    return vv
            return None

        lst = get_ci(obj, "AgentStateList")
        if isinstance(lst, list) and lst:
            for item in lst:
                aid = get_ci(item, "AircraftID") or get_ci(item, "agentID") or get_ci(item, "vehicleID") or get_ci(item, "id")
                coord = get_ci(item, "Coordinate") or {}
                lat = get_ci(coord, "Latitude") or get_ci(coord, "latitude")
                lon = get_ci(coord, "Longitude") or get_ci(coord, "longitude")
                typ = None
                try:
                    is_unm = get_ci(item, "IsUnmaned")
                    if is_unm is not None:
                        typ = "UAV" if int(is_unm) == 1 else "LAH"
                except Exception:
                    pass
                if aid is None or lat is None or lon is None:
                    continue
                yield int(aid), float(lat), float(lon), typ
        else:
            aid = lat = lon = None

            def walk(node):
                if isinstance(node, dict):
                    for k, v in node.items():
                        yield k, v
                        yield from walk(v)
                elif isinstance(node, list):
                    for child in node:
                        yield from walk(child)

            for key, value in walk(obj):
                kl = key.lower()
                try:
                    if aid is None and kl in ("aircraftid", "agentid", "vehicleid", "id"):
                        aid = int(value)
                    elif lat is None and kl == "latitude":
                        lat = float(value)
                    elif lon is None and kl == "longitude":
                        lon = float(value)
                except Exception:
                    pass
            if aid is not None and lat is not None and lon is not None:
                yield aid, lat, lon, None

    def _emit_message(self, msg_id: str, body: dict, row: int):
        if msg_id == "0401":
            if self._side.force_wp_zero():
                self._force_wp_zero(body)
            self._apply_uav_blocks(body)
            now_ms = _now_ms_since_2000()
            self._apply_current_timestamp(body, now_ms)
            self._apply_last_signal_time(body, now_ms)
            try:
                for aid, lat, lon, typ in self._iter_agent_states(body):
                    self._tracker._map.set_pos(aid, lat, lon, typ)
                    self._tracker._upsert_row(aid, lat, lon, None, None)
                    self._last_positions[int(aid)] = (float(lat), float(lon), 0.0)

            except Exception:
                pass
        try:
            push_message(
                msg_id,
                self._msgr,
                on_done=(lambda mid, raw: self._tab._mark_single_sent(row, mid, raw)) if row >= 0 else None,
                body_dict=body,
            )
        except Exception as exc:
            if callable(self._log):
                self._log(f"[REPLAY] {msg_id} push 실패: {exc}")


    def send_current_onmission2_last_wp(self, *, ticks: int = 3) -> bool:
        """현재 보고 있는 협업기저임무의 마지막 WP에 onMission=2를 송신한다."""
        self._stop_completion_sim()

        prepared = False
        try:
            prepared = bool(getattr(self._overlay, "_prepared", False))
        except Exception:
            prepared = False
        if not prepared:
            try:
                if not self._overlay.prepare(force=True):
                    if callable(self._log):
                        self._log("[REPLAY] onMission=2 송신 실패: MissionPlan 준비 실패")
                    return False
            except Exception as exc:
                if callable(self._log):
                    self._log(f"[REPLAY] onMission=2 송신 실패: {exc}")
                return False

        current_id = None
        try:
            current_id = self._overlay.current_input_id()
        except Exception:
            current_id = None
        if current_id is None or not self._overlay_active:
            try:
                resolved = self._overlay.resolve_current_input_id_from_db(reset=True)
                if resolved is not None:
                    current_id = int(resolved)
            except Exception:
                pass
        if current_id is None:
            if callable(self._log):
                self._log("[REPLAY] onMission=2 send skipped: no current input mission")
            return False

        if current_id is None:
            if callable(self._log):
                self._log("[REPLAY] onMission=2 송신 실패: 현재 input mission이 없습니다")
            return False

        try:
            last_wp = self._overlay.last_waypoints_for_current()
        except Exception:
            last_wp = {}
        if not last_wp:
            if callable(self._log):
                self._log("[REPLAY] onMission=2 송신 실패: 마지막 WP를 찾지 못했습니다")
            return False

        try:
            ticks_int = max(1, int(ticks))
        except Exception:
            ticks_int = 1

        self._start_completion_sim(on_mission=2, ticks=ticks_int)
        if callable(self._log):
            try:
                summary = ", ".join(f"{aid}:{wp}" for aid, wp in sorted(last_wp.items()))
            except Exception:
                summary = str(last_wp)
            self._log(f"[REPLAY] onMission=2 송신 시작 (input={current_id}, lastWP={summary}, ticks={ticks_int})")
        return True

    def _start_completion_sim(
        self,
        *,
        on_mission: int = 1,
        ticks: int | None = None,
        on_done=None,
    ) -> None:
        self._stop_completion_sim()
        try:
            self._completion_last_wp = self._overlay.last_waypoints_for_current()
        except Exception:
            self._completion_last_wp = {}
        self._completion_on_mission = int(on_mission)
        self._completion_ticks_left = int(ticks) if ticks is not None else None
        self._completion_after = on_done
        self._completion_timer = QTimer(self)
        self._completion_timer.setInterval(200)
        self._completion_timer.timeout.connect(self._send_completion_tick)
        self._completion_timer.start()

    def _stop_completion_sim(self) -> None:
        if self._completion_timer is not None:
            try:
                self._completion_timer.stop()
            except Exception:
                pass
        self._completion_timer = None
        self._completion_last_wp = {}
        self._completion_on_mission = 1
        self._completion_ticks_left = None
        self._completion_after = None

    def _send_completion_tick(self) -> None:
        body = self._build_completion_body()
        row = self._row_of("0401")
        self._emit_message("0401", body, row)
        if self._completion_ticks_left is not None:
            self._completion_ticks_left -= 1
            if self._completion_ticks_left <= 0:
                callback = self._completion_after
                self._stop_completion_sim()
                if callable(callback):
                    callback()

    def _build_completion_body(self) -> dict:
        agent_list = []
        for aid in range(1, 7):
            is_unmanned = aid >= 4
            lat, lon, alt = self._random_coord(aid, is_unmanned)
            state = {
                "aircraftID": int(aid),
                "isUnmanned": 1 if is_unmanned else 0,
                "health": 1,
                "coordinate": {
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": alt,
                },
                "velocity": {
                    "speed": 0.0,
                    "heading": 0.0,
                },
                "attitude": {
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                },
            }
            on_mission = int(self._completion_on_mission)
            if is_unmanned:
                last_wp = self._completion_last_wp.get(aid, 0)
                state["onMission"] = on_mission
                state["unmannedInfo"] = {
                    "currentWaypointID": {"waypointID": int(last_wp) if last_wp else 0},
                    "onMission": on_mission,
                }
            elif on_mission == 1:
                state["onMission"] = 1
            agent_list.append(state)
        return {"source": "INT", "agentStateList": agent_list}

    def _random_coord(self, aid: int, is_unmanned: bool) -> tuple[float, float, float]:
        base = self._last_positions.get(int(aid))
        if base:
            lat, lon, alt = base
        else:
            lat = 37.7 + random.uniform(-0.05, 0.05)
            lon = 128.1 + random.uniform(-0.05, 0.05)
            alt = 1000.0 if is_unmanned else 300.0
        lat += random.uniform(-0.0005, 0.0005)
        lon += random.uniform(-0.0005, 0.0005)
        return round(lat, 6), round(lon, 6), float(alt)
    def _push_0803(self, execute: int) -> None:
        timestamp = int(
            (datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)).total_seconds() * 1000
        )
        body_0803 = {
            "timestamp": timestamp,
            "source": "CSP",
            "execute": int(execute),
        }
        row = self._row_of("0803")
        try:
            push_message(
                "0803",
                self._msgr,
                on_done=(lambda mid, raw: self._tab._mark_single_sent(row, mid, raw)) if row >= 0 else None,
                body_dict=body_0803,
            )
            if callable(self._log):
                self._log(f"[REPLAY] 0803 send ok (execute={execute})")
        except Exception as exc:
            if callable(self._log):
                self._log(f"[REPLAY] 0803 push failed: {exc}")

    def _match_key(self, container: dict, target: str):
        if not isinstance(container, dict):
            return None, None
        target_lower = target.lower()
        for key, value in container.items():
            if isinstance(key, str) and key.lower() == target_lower:
                return key, value
        return None, None

    def _force_wp_zero(self, body: dict) -> None:
        if not isinstance(body, dict):
            return

        _, agent_states = self._match_key(body, "agentStateList")
        if not isinstance(agent_states, list):
            return

        for agent in agent_states:
            if not isinstance(agent, dict):
                continue
            _, unmanned_info = self._match_key(agent, "unmannedInfo")
            if not isinstance(unmanned_info, dict):
                continue
            key_cwp, current_wp = self._match_key(unmanned_info, "currentWaypointID")
            if key_cwp is None or not isinstance(current_wp, dict):
                unmanned_info["currentWaypointID"] = {"waypointID": 0}
                continue
            key_wp, _ = self._match_key(current_wp, "waypointID")
            if key_wp is None:
                current_wp["waypointID"] = 0
            else:
                current_wp[key_wp] = 0

    def _apply_uav_blocks(self, body: dict) -> None:
        blocked = self._side.blocked_uav_ids()
        if not blocked or not isinstance(body, dict):
            return

        key_list, agent_states = self._match_key(body, "agentStateList")
        if key_list is None or not isinstance(agent_states, list):
            return

        filtered = [
            agent
            for agent in agent_states
            if not self._is_agent_blocked(agent, blocked)
        ]

        body[key_list] = filtered

    def _apply_current_timestamp(self, body: dict, now_ms: int) -> None:
        if not isinstance(body, dict):
            return
        key_ts, _ = self._match_key(body, "timestamp")
        if key_ts is None:
            body["timestamp"] = int(now_ms)
        else:
            body[key_ts] = int(now_ms)

    def _apply_last_signal_time(self, body: dict, now_ms: int | None = None) -> None:
        if not isinstance(body, dict):
            return
        key_list, agent_states = self._match_key(body, "agentStateList")
        if key_list is None or not isinstance(agent_states, list):
            return
        if now_ms is None:
            now_ms = _now_ms_since_2000()
        last_signal = int(now_ms)
        for agent in agent_states:
            if not isinstance(agent, dict):
                continue
            key_ls, _ = self._match_key(agent, "lastSignalTime")
            if key_ls is None:
                agent["lastSignalTime"] = last_signal
            else:
                agent[key_ls] = last_signal

    def _is_agent_blocked(self, agent: dict, blocked: set[int]) -> bool:
        if not isinstance(agent, dict):
            return False
        _, value = self._match_key(agent, "aircraftID")
        if value is None:
            _, value = self._match_key(agent, "agentID")
        if value is None and "aircraftID" in agent:
            value = agent.get("aircraftID")
        try:
            return int(value) in blocked
        except (TypeError, ValueError):
            return False

    def _schedule_next_overlay_row(self, initial: bool = False):
        if self._awaiting_user:
            return
        if self._row_idx_0401 >= len(self._rows_0401):
            self._prompt_next_input_mission()
            return
        if initial:
            self._current_mission_start_idx = self._row_idx_0401
            self._current_mission_base_ts = self._prev_sim_ts_0401
        delay, target_ts = self._compute_delay_for_overlay_row(self._row_idx_0401, initial)
        self._pending_target_ts = target_ts
        if self._mission_timer is None:
            self._mission_timer = QTimer(self)
            self._mission_timer.setSingleShot(True)
            self._mission_timer.timeout.connect(self._send_overlay_row)
        else:
            self._mission_timer.stop()
        self._mission_timer.setInterval(int(max(0, delay)))
        self._mission_timer.start()

    def _compute_delay_for_overlay_row(self, idx: int, initial: bool):
        ts, _ = self._rows_0401[idx]
        step = 200
        prev = self._prev_sim_ts_0401
        if ts is None:
            target = prev + step
        else:
            target = ts
            if target < prev:
                target = prev
        delay = target - prev if (idx > 0 or not initial) else max(0, target - prev)
        if initial and idx == 0 and delay < 0:
            delay = 0
        return delay, target

    def _send_overlay_row(self):
        if self._awaiting_user:
            return
        if self._row_idx_0401 >= len(self._rows_0401):
            self._prompt_next_input_mission()
            return
        ts, body = self._rows_0401[self._row_idx_0401]
        self._prev_sim_ts_0401 = self._pending_target_ts
        finished = self._emit_overlay_entry(body)
        self._row_idx_0401 += 1
        if finished:
            self._prompt_next_input_mission()
        else:
            self._schedule_next_overlay_row()

    def _emit_overlay_entry(self, body: dict) -> bool:
        finished = False
        if self._overlay_active:
            try:
                finished = self._overlay.apply(body)
            except Exception as exc:
                self._overlay_active = False
                if callable(self._log):
                    self._log(f"[REPLAY] 임무 덮어쓰기 중 오류: {exc}")
                finished = False
        row = self._row_of("0401")
        self._emit_message("0401", body, row)
        return finished

    def _prompt_0503_recommend(self, system_recommend: int) -> None:
        """
        0503(systemRecommend) 수신 시 Sim에서 빠르게 Next/Repeat 선택.

        - overlay(replay) 모드면: 기존 Scenario GUI 동작(overlay advance/repeat)을 그대로 수행
        - overlay 미사용이면: 0803만 전송
        """
        if self._awaiting_user:
            return

        current_id = self._overlay.current_input_id() if self._overlay_active else None
        next_id = self._overlay.peek_next_input_id() if self._overlay_active else None

        # replay 모드에서 이미 마지막 입력임무라면, 기존 로직과 동일하게 완료 시뮬레이션으로 전환
        if self._overlay_active and next_id is None:
            self._overlay_active = False
            self._awaiting_user = False
            self._start_completion_sim()
            if callable(self._log):
                self._log("[REPLAY] Last input mission reached; start completion 0401 simulation")
            return

        self._awaiting_user = True
        dlg = _Dlg0503_SystemRecommend(
            self._tab,
            system_recommend=int(system_recommend),
            current_input_id=current_id,
            next_input_id=next_id,
        )
        action = int(dlg.exec_())

        if action == _Dlg0503_SystemRecommend.ACTION_NEXT:
            self._push_0803(1)
            self._awaiting_user = False
            if self._overlay_active:
                new_id = self._overlay.advance_to_next_input()
                if new_id is None:
                    self._overlay_active = False
                    if callable(self._log):
                        self._log("[REPLAY] 모든 협업기저임무 재생 완료")
                    return
                if callable(self._log):
                    self._log(f"[REPLAY] 협업기저임무 {new_id} 재생 시작")
                self._schedule_next_overlay_row(initial=True)
            return

        if action == _Dlg0503_SystemRecommend.ACTION_REPEAT:
            self._push_0803(2)
            self._awaiting_user = False
            if self._overlay_active:
                if not self.repeat_current_input():
                    if callable(self._log):
                        self._log("[REPLAY] repeat unavailable; stopping replay")
                    self.stop()
            return

        # cancel/close: 대기 해제(기존 Scenario GUI는 그대로, 사용자는 좌측 버튼으로 0803 수동 전송 가능)
        self._awaiting_user = False

    def _prompt_next_input_mission(self):
        if not self._overlay_active:
            return
        if self._awaiting_user:
            return
        self._awaiting_user = True
        current_id = self._overlay.current_input_id()
        next_id = self._overlay.peek_next_input_id()
        if next_id is None:
            self._overlay_active = False
            self._awaiting_user = False
            self._start_completion_sim()
            if callable(self._log):
                self._log("[REPLAY] Last input mission reached; start completion 0401 simulation")
            return

        box = QMessageBox(self._tab)
        box.setWindowTitle("Input Mission Complete")
        box.setText(f"Input mission {current_id} finished.\nProceed to the next mission?")
        next_button = box.addButton("Next mission (execute=1)", QMessageBox.AcceptRole)
        complete_button = box.addButton("OnMission=2 then Next", QMessageBox.ActionRole)
        repeat_button = box.addButton("Repeat mission (execute=2)", QMessageBox.ActionRole)
        stop_button = box.addButton("Stop", QMessageBox.RejectRole)
        box.exec_()
        clicked = box.clickedButton()
        if clicked == next_button:
            self._push_0803(1)
            self._awaiting_user = False
            new_id = self._overlay.advance_to_next_input()
            if new_id is None:
                self._overlay_active = False
                if callable(self._log):
                    self._log("[REPLAY] 모든 협업기저임무 재생 완료")
                return
            if callable(self._log):
                self._log(f"[REPLAY] 협업기저임무 {new_id} 재생 시작")
            self._schedule_next_overlay_row(initial=True)
        elif clicked == complete_button:
            self._push_0803(1)
            self._awaiting_user = False

            def _advance():
                new_id = self._overlay.advance_to_next_input()
                if new_id is None:
                    self._overlay_active = False
                    if callable(self._log):
                        self._log("[REPLAY] all input missions completed")
                    return
                if callable(self._log):
                    self._log(f"[REPLAY] input mission {new_id} start")
                self._schedule_next_overlay_row(initial=True)



            self._start_completion_sim(on_mission=2, ticks=10, on_done=_advance)
            return

        elif clicked == repeat_button:
            self._push_0803(2)
            self._awaiting_user = False
            if not self.repeat_current_input():
                if callable(self._log):
                    self._log("[REPLAY] repeat unavailable; stopping replay")
                self.stop()
            return
        else:
            if callable(self._log):
                self._log("[REPLAY] 사용자 요청으로 재생을 중단합니다.")
            self.stop()

    def repeat_current_input(self) -> bool:
        self._stop_completion_sim()
        if not self._overlay_active:
            if self._overlay and self._overlay.current_input_id() is not None and self._overlay.has_inputs():
                self._overlay_active = True
            else:
                if callable(self._log):
                    self._log("[REPLAY] repeat ignored: overlay inactive")
                return False
        if self._overlay.current_input_id() is None:
            if callable(self._log):
                self._log("[REPLAY] repeat ignored: no current input")
            return False
        if not self._rows_0401:
            if callable(self._log):
                self._log("[REPLAY] repeat ignored: no 0401 rows")
            return False
        if self._current_mission_start_idx >= len(self._rows_0401):
            if callable(self._log):
                self._log("[REPLAY] repeat ignored: invalid start index")
            return False

        if self._mission_timer is not None:
            try:
                self._mission_timer.stop()
            except Exception:
                pass
        self._awaiting_user = False
        self._pending_target_ts = self._current_mission_base_ts
        self._row_idx_0401 = max(0, int(self._current_mission_start_idx))
        self._prev_sim_ts_0401 = float(self._current_mission_base_ts)
        try:
            self._overlay.reset_current_input()
        except Exception:
            pass
        if callable(self._log):
            self._log(f"[REPLAY] repeat input {self._overlay.current_input_id()} from row {self._row_idx_0401}")
        self._schedule_next_overlay_row(initial=True)
        return True

class MainWindow(QMainWindow):
    ctrl_payload = pyqtSignal(dict)  # 백그라운드 → UI 스레드

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle('통합모듈(INT)')
        self.resize(1380, 780)

        self._power_on = True
        self._last_ctrl_ts = {}

        # ── 중앙 탭
        tabs = QTabWidget()
        self._tab = IntegrationTab(messenger=NodeMessenger)
        tabs.addTab(self._tab, "Integration CSC")

        # ── 상단 모드 슬라이더
        top = QWidget(); top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(8,4,8,4); top_layout.addStretch(1)
        self.mode_slider = QSlider(Qt.Horizontal); self.mode_slider.setRange(0, 3)
        self.mode_slider.setSingleStep(1); self.mode_slider.setTickInterval(1)
        self.mode_slider.setTickPosition(QSlider.TicksBelow); self.mode_slider.setFixedWidth(420)
        self.mode_slider.valueChanged.connect(self._on_mode_slider_changed)
        slider_wrap = QWidget()
        slider_layout = QVBoxLayout(slider_wrap)
        slider_layout.setContentsMargins(0, 0, 0, 0)
        slider_layout.setSpacing(2)
        slider_layout.addWidget(self.mode_slider, 0, Qt.AlignHCenter)
        self.mode_hint = ModeTickLabels(
            self.mode_slider,
            ["0\n초기화", "1\n대기", "2\n초기임무계획", "3\n임무수행"],
            slider_wrap,
        )
        slider_layout.addWidget(self.mode_hint, 0, Qt.AlignHCenter)
        self.mode_now = QLabel("초기화 모드"); self.mode_now.setStyleSheet("font-weight:600; padding-left:8px;")
        self.mode_now.setFixedWidth(140)
        self.mode_now.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        lbl = QLabel("모드:"); lbl.setStyleSheet("color:#789; padding-right:6px;")
        top_layout.addWidget(lbl); top_layout.addWidget(slider_wrap); top_layout.addWidget(self.mode_now)

        # ── 좌측 보조 패널
        self._side = _MissionSidePanel(on_log=self._append_log_line, parent=self)
        self._side.btn_start.clicked.connect(self._on_click_start_sim)
        self._side.btn_0202.clicked.connect(self._act_send_0202)
        self._side.btn_0801.clicked.connect(self._act_send_0801)
        self._side.btn_0802.clicked.connect(self._act_send_0802)
        self._side.btn_0803_next.clicked.connect(self._act_send_0803_next)
        self._side.btn_0803_repeat.clicked.connect(self._act_send_0803_repeat)
        self._side.btn_onmission2_last_wp.clicked.connect(self._act_send_onmission2_last_wp)
        self._side.btn_0702.clicked.connect(self._act_send_0702)
        self._latest_decision_options: list[dict] = []
        self._last_selected_plan_id: int | None = None
        try:
            register_listener("0701", self._on_receive_0701)
        except Exception:
            self._append_log_line("[WARN] register_listener(0701) 실패")
        try:
            register_listener("0503", self._on_receive_0503)
        except Exception:
            self._append_log_line("[WARN] register_listener(0503) failed")

        # (우측) 기존 상단/탭을 세로 배치
        right = QWidget(); v = QVBoxLayout(right); v.setContentsMargins(0,0,0,0)
        v.addWidget(top); v.addWidget(tabs)

        # (중앙) 좌측 패널 + 우측 기존 영역을 가로 배치
        center = QWidget(); h = QHBoxLayout(center); h.setContentsMargins(0,0,0,0); h.setSpacing(10)
        h.addWidget(self._side, 0, Qt.AlignTop); h.addWidget(right, 1)
        self.setCentralWidget(center)

        # 리플레이 매니저
        self._replay = _ReplayManager(self._tab, NodeMessenger, self._append_log_line, self._side)

        # 초기 모드
        self._set_mode_slider_by_text("초기화 모드")
        self._apply_power_state()
        self._rx_ready = False
        self._rx_setup()

    def _install_power_gate_hooks(self):
        try:
            tab = self._tab
            tbl = getattr(tab, "tbl_tx", None)
            if tbl is not None:
                class _PG(QObject):
                    def __init__(self, host): super().__init__(host); self.host = host
                    def eventFilter(self, obj, ev):
                        if not self.host._power_on and ev.type() in (
                            QEvent.MouseButtonPress, QEvent.MouseButtonRelease,
                            QEvent.MouseButtonDblClick, QEvent.KeyPress, QEvent.KeyRelease
                        ):
                            return True
                        return False
                self._pg_filter = _PG(self)
                tbl.installEventFilter(self._pg_filter)
        except Exception:
            pass

    def _apply_power_state(self):
        on = bool(self._power_on)
        try:
            self._update_tx_enabled(on)
            if not on:
                self._replay.stop()
                self._stop_all_periodic()
        except Exception:
            pass

    def _update_tx_enabled(self, enabled: bool):
        try:
            tab = self._tab
            tbl = getattr(tab, "tbl_tx", None)
            if tbl is None:
                self._side.set_extra_buttons_enabled(enabled)
                return
            tbl.setEnabled(enabled)
            for r in range(tbl.rowCount()):
                w = tbl.cellWidget(r, 3)   # '발신' 버튼 컬럼
                if w is not None and hasattr(w, "setEnabled"):
                    w.setEnabled(enabled)
            self._side.set_extra_buttons_enabled(enabled)
        except Exception:
            pass

    def _stop_all_periodic(self):
        try:
            timers = getattr(self._tab, "periodic_timers", {})
            for _, t in list(timers.items()):
                try: t.stop()
                except Exception: pass
            try: timers.clear()
            except Exception: pass
        except Exception:
            pass

    # ───────── 모드/슬라이더 ─────────
    def _on_mode_slider_changed(self, val: int):
        labels = ["초기화 모드", "대기모드", "초기 임무 계획", "임무 수행"]
        try: self.mode_now.setText(labels[int(val)])
        except Exception: pass
        self._power_on = True
        self._apply_power_state()

    def _set_mode_slider_by_text(self, text: str):
        labels = ["초기화 모드", "대기모드", "초기 임무 계획", "임무 수행"]
        norm = re.sub(r"\s+", "", str(text)).lower()
        mapping = {
            "전원off":0,"off":0,"poweroff":0,
            "전원on":0,"on":0,"poweron":0,
            "0":0,
            "초기화":0,"초기화모드":0,"초기화mode":0,
            "1":1,"대기모드":1,"대기":1,"standby":1,
            "2":2,"초기임무계획":2,"초기임무계획모드":2,"initplan":2,"initial":2,
            "3":3,"임무수행":3,"execution":3,
        }
        val = mapping.get(norm, 1)
        try:
            if getattr(self, "mode_slider", None):
                if self.mode_slider.value() != val:
                    self.mode_slider.blockSignals(True); self.mode_slider.setValue(val); self.mode_slider.blockSignals(False)
            if getattr(self, "mode_now", None): self.mode_now.setText(labels[val])
        except Exception: pass
        self._power_on = True

    # ───────── 버스 초기화 ─────────
    def _rx_setup(self):
        if getattr(self, "_rx_ready", False):
            return
        try:
            with fusion_runtime_working_dir(project_root=PROJECT_ROOT):
                FusionNodeIoc.Configure()
                NodeMessenger.Initialize("INT_ReceiveNode")
                NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
                NodeMessenger.InitAllSubscriberFromAssembly()
                NodeMessenger.RegistAllProviderFromFusionNodeIoc()
            self._rx_ready = True
        except Exception as e:
            self._append_log_line(f"[BUS] init 실패: {e}")
            self._rx_ready = False

    def _parse_payload_any(self, payload):
        if payload is None:
            return None
        if isinstance(payload, dict):
            return payload
        if isinstance(payload, list) and payload:
            payload = payload[-1]
            if isinstance(payload, dict):
                return payload
        if isinstance(payload, bytes):
            return self._parse_json_body(payload)
        if isinstance(payload, str):
            try:
                return self._parse_json_body(payload.encode("utf-8", "ignore"))
            except Exception:
                return None
        try:
            return self._parse_json_body(bytes(payload))
        except Exception:
            return None

    def _on_receive_0503(self, _msg_id: str, payload: object | None):
        data = self._parse_payload_any(payload)
        if not isinstance(data, dict):
            return
        rec = data.get("systemRecommend") or data.get("SystemRecommend")
        try:
            rec_val = int(rec)
        except Exception:
            return
        self._append_log_line(f"[0503] systemRecommend={rec_val}")
        if rec_val == 3:
            QTimer.singleShot(0, lambda: QMessageBox.information(self, "Mission Complete", "All input missions completed."))
            return
        QTimer.singleShot(0, lambda rv=rec_val: self._replay._prompt_0503_recommend(rv))

    def _handle_ctrl_payload(self, payload: dict):
        try: cmd = str(payload.get("cmd") or "")
        except Exception: return

        key = f"{cmd}:{payload.get('text') or payload.get('status')}"
        now = time.monotonic(); last = getattr(self, "_last_ctrl_ts", {}).get(key, 0.0)
        if (now - last) < 1.0: return
        self._last_ctrl_ts[key] = now

        if not self._power_on and cmd not in ("mode",):
            self._append_log_line(f"[BLOCK] Power OFF → CTRL '{cmd}' 무시")
            return

        if cmd == "self_check":
            try:
                status = int(payload.get("status", 1))
            except Exception:
                status = 1
            if status == 1:
                send_status_ok("INT")
        elif cmd == "mode":
            text = str(payload.get("text") or "").strip()
            self._append_log_line(f"[CTRL] MODE change request: {text}")
            self._set_mode_slider_by_text(text)

    # ───────── 로깅 ─────────
    def _parse_json_body(self, raw: bytes | None):
        if not raw:
            return None
        try:
            txt = raw.decode("utf-8", "ignore")
        except Exception:
            txt = ""
        try:
            return json.loads(txt)
        except Exception:
            try:
                m = re.search(r"\{.*\}", txt, flags=re.S)
                return json.loads(m.group(0)) if m else None
            except Exception:
                return None

    def _append_log_line(self, text: str):
        try:
            emit_process_log("integration", str(text))
        except Exception:
            pass
        try:
            if getattr(self, "_tab", None) and hasattr(self._tab, "append_log"):
                QTimer.singleShot(0, lambda t=str(text): self._tab.append_log(t))
                try:
                    print(text)
                except Exception:
                    pass
                return
        except Exception:
            pass
        try:
            print(text)
        except Exception:
            pass

    def _sim_now(self) -> int:
        try:
            return int(self._replay.now_timestamp_ms())
        except Exception:
            return _now_ms_since_2000()

    def _send_and_mark(self, msg_id: str, body: dict):
        if not getattr(self, "_rx_ready", False):
            self._rx_setup()
        if not getattr(self, "_rx_ready", False):
            self._append_log_line(f"[SEND] {msg_id} 실패: NodeMessenger init failed")
            return
        row = self._replay._row_of(msg_id)
        self._append_log_line(f"[SEND] : {msg_id}")
        try:
            self._append_log_line(f"[{msg_id}] BODY  : {json.dumps(body, ensure_ascii=False)}")
        except Exception:
            pass

        def _on_done(mid, raw):
            if row >= 0:
                try:
                    self._tab._mark_single_sent(row, mid, raw)
                except Exception:
                    pass
            self._append_log_line(f"[{mid}] PUSH 완료")

        try:
            push_message(msg_id, NodeMessenger, on_done=_on_done, body_dict=body)
        except Exception as exc:
            self._append_log_line(f"[SEND] {msg_id} 실패: {exc}")

    def _on_receive_0701(self, msg_id: str, payload: object) -> None:
        option_list: list[dict] = []
        if isinstance(payload, dict):
            raw_list = payload.get("optionList") or []
            if isinstance(raw_list, list):
                option_list = [opt for opt in raw_list if isinstance(opt, dict)]

        self._latest_decision_options = option_list
        self._side.update_decision_options(option_list)

        if option_list:
            plan_ids = [opt.get("missionPlanID") for opt in option_list if isinstance(opt, dict)]
            self._append_log_line(f"[0701] 옵션 {len(option_list)}건 수신: {plan_ids}")
        else:
            self._append_log_line("[0701] 옵션 정보가 비어 있어 버튼을 비활성화합니다.")

    def _act_send_0803_next(self):
        try:
            self._replay._push_0803(1)
        except Exception as exc:
            self._append_log_line(f"[0803] send failed: {exc}")

    def _act_send_0803_repeat(self):
        try:
            self._replay._push_0803(2)
        except Exception as exc:
            self._append_log_line(f"[0803] send failed: {exc}")

    def _act_send_onmission2_last_wp(self):
        try:
            ok = bool(self._replay.send_current_onmission2_last_wp(ticks=3))
        except Exception as exc:
            self._append_log_line(f"[0401] onMission=2 send failed: {exc}")
            return
        if not ok:
            self._append_log_line("[0401] onMission=2 send skipped (overlay/current mission unavailable)")

    def _act_send_0702(self):
        if not self._latest_decision_options:
            QMessageBox.warning(self, "0702", "최근 수신한 옵션 정보가 없습니다.")
            return

        ignore_value = self._side.selected_decision_ignore()
        selected_plan_id = self._side.selected_decision_plan_id()

        if ignore_value == 2:
            if selected_plan_id is None:
                QMessageBox.warning(self, "0702", "선택된 옵션이 없습니다.")
                return
            mission_plan_id = int(selected_plan_id)
            self._last_selected_plan_id = mission_plan_id
        elif ignore_value == 1:
            if self._last_selected_plan_id is None:
                QMessageBox.warning(self, "0702", "기존 임무 계획 ID를 찾을 수 없습니다.")
                return
            mission_plan_id = int(self._last_selected_plan_id)
        else:
            mission_plan_id = int(selected_plan_id or 0)

        ts = self._sim_now()
        body = {
            "timestamp": ts,
            "source": "MOB",
            "autoExecution": bool(self._side.decision_auto_execution()),
            "ignore": int(ignore_value),
            "missionPlanID": mission_plan_id,
        }
        self._send_and_mark("0702", body)

    def _act_send_0202(self):
        dlg = _Dlg0202_PriorMissionInfo(self, default_source="DSC")
        if dlg.exec_() == QDialog.Accepted:
            ts = self._sim_now()
            self._send_and_mark("0202", dlg.build_body(ts))

    def _act_send_0801(self):
        dlg = _Dlg0801_InitialPlanCommand(self, default_source="DSC")
        ts = self._sim_now()
        dlg.prefill(ts)
        if dlg.exec_() == QDialog.Accepted:
            self._send_and_mark("0801", dlg.build_body(ts))

    def _act_send_0802(self):
        dlg = _Dlg0802_MandatoryCommand(self, default_source="DSC")
        if dlg.exec_() == QDialog.Accepted:
            ts = self._sim_now()
            self._send_and_mark("0802", dlg.build_body(ts))

    # ───────── 임무 수행 모사 시작 핸들러 ─────────
    def _on_click_start_sim(self):
        p0401 = []; p0402 = []
        try: p0401 = self._side.selected_paths_0401(); p0402 = self._side.selected_paths_0402()
        except Exception: pass
        if not p0401 and not p0402:
            self._append_log_line("[REPLAY] 파일이 선택되지 않았습니다.")
            return
        self._append_log_line(f"[REPLAY] 시작: 0401={len(p0401)} / 0402={len(p0402)}")
        self._replay.start_from_two(p0401, p0402)
        try:
            QMessageBox.information(self, "안내", "파일 기반 임무 모사를 시작합니다.\n(0401은 ts 간격 재현, 없으면 5Hz / 0402는 이벤트성)")
        except Exception:
            pass


# ───────── 엔트리 ─────────
if __name__ == "__main__":
    from PyQt5.QtWidgets import QLabel, QHBoxLayout, QVBoxLayout  # noqa: F401  (위에서 사용)
    app = QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec_())



