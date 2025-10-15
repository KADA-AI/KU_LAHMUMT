# -*- coding: utf-8 -*-
# modules/integration_module/integration_gui.py
# Integration GUI – 연동 모듈 전용 GUI
from __future__ import annotations

import sys, os, threading, json, re, time
os.environ["KU_ROLE"] = "integration"
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]  # .../KU_LAHMUMT
for _p in (_ROOT, _ROOT / "modules", _ROOT / "modules" / "common"):
    _ps = str(_p)
    if _p.exists() and _ps not in sys.path:
        sys.path.insert(0, _ps)

from modules.common.qt_env import ensure_qt_platform
ensure_qt_platform()

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, pyqtSignal, QTimer, Qt, QEvent, QObject, QPointF
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QWidget, QLabel, QHBoxLayout, QVBoxLayout, QSlider, QLineEdit, QPushButton, QFileDialog, QGroupBox, QMessageBox, QSizePolicy, QTableWidget, QHeaderView, QTableWidgetItem
from PyQt5.QtGui import QPainter, QColor, QPen

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

# ───────── 설정/라이선스 정규화 ─────────
def _ensure_fusion_configs():
    cands = [
        PROJECT_ROOT / "nFusionSettings.json",
        COMMON_DIR   / "nFusionSettings.json",
        PROJECT_ROOT / "FusionSettings.json",
        COMMON_DIR   / "FusionSettings.json",
        PROJECT_ROOT / "nFusion" / "FusionSettings.json",
    ]
    src = next((p for p in cands if p.exists()), None)
    if src is None:
        raise FileNotFoundError("nFusionSettings.json/FusionSettings.json 이 없습니다.")
    dst = PROJECT_ROOT / "nFusionSettings.json"
    if src != dst:
        dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")

    lcands = [
        PROJECT_ROOT / "nFusionLicense.lic",
        COMMON_DIR   / "nFusionLicense.lic",
        PROJECT_ROOT / "nFusion" / "nFusionLicense.lic",
    ]
    lsrc = next((p for p in lcands if p.exists()), None)
    if lsrc:
        ldst = PROJECT_ROOT / "nFusionLicense.lic"
        if lsrc != ldst:
            ldst.write_text(lsrc.read_text(encoding="utf-8"), encoding="utf-8")
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
from modules.common.ctrl_listener import start_ctrl_listener, env_ctrl_port

from push_center import push_message


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

        # 0402 행
        row2 = QWidget(self); r2 = QHBoxLayout(row2); r2.setContentsMargins(0,0,0,0); r2.setSpacing(8)
        self._lamp_0402 = QLabel(row2); self._set_lamp(self._lamp_0402, False)
        lbl2 = QLabel("0402 Log", row2)
        self._path_0402 = QLineEdit(row2); self._path_0402.setReadOnly(True); self._path_0402.setPlaceholderText("선택된 파일 없음")
        btn2 = QPushButton("찾아보기", row2); btn2.setMinimumWidth(84)
        btn2.clicked.connect(lambda: self._browse("0402"))
        r2.addWidget(self._lamp_0402); r2.addWidget(lbl2); r2.addWidget(self._path_0402); r2.addWidget(btn2)
        lay.addWidget(row2)

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


# ───────── 리플레이 매니저 ─────────
class _ReplayManager(QObject):
    def __init__(self, tab: IntegrationTab, messenger, logger, tracker: _Agent0401Panel):
        super().__init__(tab)
        self._tab = tab
        self._msgr = messenger
        self._log = logger
        self._tracker = tracker
        self._timers: list[QTimer] = []

    def stop(self):
        for t in self._timers:
            try: t.stop()
            except Exception: pass
        self._timers.clear()
        if callable(self._log): self._log("[REPLAY] 중지됨")

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
        keys = ("timestamp","time","ts","Timestamp","Time","Ts")
        def walk(n):
            if isinstance(n, dict):
                for k,v in n.items():
                    yield k,v
                    yield from walk(v)
            elif isinstance(n, list):
                for it in n:
                    yield from walk(it)
        for k,v in walk(obj):
            try:
                if any(k.lower()==kk.lower() for kk in keys):
                    return float(v)
            except Exception:
                pass
        return None

    def _iter_ndjson(self, path: str):
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                line=line.strip()
                if not line: continue
                try:
                    yield json.loads(line)
                except Exception:
                    m = re.search(r"\{.*\}", line)
                    if m:
                        try: yield json.loads(m.group(0))
                        except Exception: pass

    # modules/integration_module/integration_gui.py
    # (_ReplayManager.start_from_two 교체)

    def _stop_periodic_for(self, msg_id: str):
        try:
            timers = getattr(self._tab, "periodic_timers", {})
            t = timers.get(msg_id)
            if t:
                try: t.stop()
                except Exception: pass
                try: t.deleteLater()
                except Exception: pass
                try: del timers[msg_id]
                except Exception: pass
            # TX 테이블 상태 표시도 정리(있을 때만)
            try:
                tbl = getattr(self._tab, "tbl_tx", None)
                if tbl is not None:
                    for r in range(tbl.rowCount()):
                        it_code = tbl.item(r, 0)
                        if it_code and it_code.text().strip() == msg_id:
                            it_state = tbl.item(r, 2)
                            if it_state:
                                it_state.setText("전송 정지(리플레이)")
                            break
            except Exception:
                pass
        except Exception:
            pass

    def start_from_two(self, paths_0401: list[str], paths_0402: list[str]):
        """0402를 0401의 최솟값 ts(=anchor) 기준으로 상대 지연을 잡아 재생."""
        # 중복 방지: 기존 주기 전송 중지
        try:
            self._stop_periodic_for("0401")
            self._stop_periodic_for("0402")
        except Exception:
            pass

        self.stop()
        try: self._tracker._map.clear()
        except Exception: pass

        buckets = {"0401": [], "0402": []}
        for p in paths_0401:
            for obj in self._iter_ndjson(p):
                buckets["0401"].append((self._extract_ts(obj), obj))
        for p in paths_0402:
            for obj in self._iter_ndjson(p):
                buckets["0402"].append((self._extract_ts(obj), obj))

        if not buckets["0401"] and not buckets["0402"]:
            if callable(self._log): self._log("[REPLAY] 선택된 0401/0402 레코드가 없습니다.")
            return

        # ---- anchor: 0401의 최솟값 ts (없으면 0402의 최솟값으로 대체) ----
        def first_ts(rows):
            if not rows: return None
            rows_sorted = sorted(rows, key=lambda x: (float('inf') if x[0] is None else x[0]))
            for ts, _ in rows_sorted:
                if ts is not None:
                    return float(ts)
            return None

        anchor_0401 = first_ts(buckets["0401"])
        anchor = anchor_0401 if anchor_0401 is not None else first_ts(buckets["0402"])
        if anchor is None:
            anchor = 0.0   # 전부 ts 없으면 0 기준 보간

        if callable(self._log):
            self._log(f"[REPLAY] anchor = {anchor} (0401_first={anchor_0401})")

        # ---- 각 스트림 예약: sim_ts - anchor ----
        for msg_id, rows in buckets.items():
            if not rows:
                continue
            rows.sort(key=lambda x: (float('inf') if x[0] is None else x[0]))

            if msg_id == "0401":
                # 0401: 5Hz(200ms) 복원, ts 없으면 보간
                step = 200  # ms
                last_sim_ts = anchor
                for ts, obj in rows:
                    sim_ts = ts if (ts is not None) else (last_sim_ts + step)
                    if sim_ts < last_sim_ts:
                        sim_ts = last_sim_ts
                    delay = int(max(0, sim_ts - anchor))
                    last_sim_ts = sim_ts
                    self._schedule_send(msg_id, obj, delay)
                if callable(self._log):
                    self._log(f"[REPLAY] 0401 {len(rows)}건 예약 완료 (5Hz, anchor={anchor})")

            elif msg_id == "0402":
                # 0402: 비주기 이벤트 → ts가 있는 건만 anchor 기준으로 스케줄
                cnt = 0
                for ts, obj in rows:
                    if ts is None:
                        if callable(self._log):
                            self._log("[REPLAY] 0402: ts 없음 → skip")
                        continue
                    delay = int(max(0, ts - anchor))  # ← 핵심: 0401 최솟값 기준
                    self._schedule_send(msg_id, obj, delay)
                    cnt += 1
                if callable(self._log):
                    self._log(f"[REPLAY] 0402 {cnt}건 예약 완료 (event, anchor={anchor})")


    # modules/integration_module/integration_gui.py  (_ReplayManager._schedule_send 내부 iter_states/do_send 교체)
    def _schedule_send(self, msg_id: str, body: dict, delay_ms: int):
        row = self._row_of(msg_id)

        def get_ci(d, k):
            if not isinstance(d, dict): return None
            kl = k.lower()
            for kk, vv in d.items():
                if kk.lower() == kl: return vv
            return None

        def iter_states(obj):
            """0401 본문에서 (aid, lat, lon, type_label) 생성"""
            lst = get_ci(obj, "AgentStateList")
            if isinstance(lst, list) and lst:
                for it in lst:
                    aid = get_ci(it, "AircraftID") or get_ci(it, "agentID") or get_ci(it, "vehicleID") or get_ci(it, "id")
                    coord = get_ci(it, "Coordinate") or {}
                    lat = get_ci(coord, "Latitude") or get_ci(coord, "latitude")
                    lon = get_ci(coord, "Longitude") or get_ci(coord, "longitude")
                    typ = None
                    try:
                        is_unm = get_ci(it, "IsUnmaned")
                        if is_unm is not None:
                            typ = "UAV" if int(is_unm) == 1 else "LAH"
                    except Exception:
                        pass
                    if aid is None or lat is None or lon is None: continue
                    yield int(aid), float(lat), float(lon), typ
            else:
                # 단일형
                aid=lat=lon=None
                def walk(n):
                    if isinstance(n, dict):
                        for k,v in n.items():
                            yield k,v
                            yield from walk(v)
                    elif isinstance(n, list):
                        for x in n: yield from walk(x)
                for k,v in walk(obj):
                    kl=k.lower()
                    try:
                        if aid is None and kl in ("aircraftid","agentid","vehicleid","id"): aid=int(v)
                        elif lat is None and kl=="latitude": lat=float(v)
                        elif lon is None and kl=="longitude": lon=float(v)
                    except Exception:
                        pass
                if aid is not None and lat is not None and lon is not None:
                    yield aid, lat, lon, None

        def do_send():
            if msg_id == "0401":
                # 리플레이 경로에서도 지도+표 동기 갱신
                try:
                    for aid, lat, lon, typ in iter_states(body):
                        self._tracker._map.set_pos(aid, lat, lon, typ)
                        self._tracker._upsert_row(aid, lat, lon, None, None)
                except Exception:
                    pass
            try:
                push_message(
                    msg_id, self._msgr,
                    on_done=(lambda mid, raw: self._tab._mark_single_sent(row, mid, raw)) if row >= 0 else None,
                    body_dict=body
                )
            except Exception as e:
                if callable(self._log): self._log(f"[REPLAY] {msg_id} push 실패: {e}")



        t = QTimer(self)
        t.setSingleShot(True)
        t.setInterval(int(delay_ms))
        t.timeout.connect(do_send)
        t.start()
        self._timers.append(t)


class MainWindow(QMainWindow):
    ctrl_payload = pyqtSignal(dict)  # 백그라운드 → UI 스레드

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle("연동 모듈 GUI")
        self.resize(1380, 780)

        self._power_on = False
        self._last_ctrl_ts = {}

        # ── 중앙 탭
        tabs = QTabWidget()
        self._tab = IntegrationTab(messenger=NodeMessenger)
        tabs.addTab(self._tab, "Integration CSC")

        # ── 상단 모드 슬라이더
        top = QWidget(); top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(8,4,8,4); top_layout.addStretch(1)
        self.mode_slider = QSlider(Qt.Horizontal); self.mode_slider.setRange(0,4)
        self.mode_slider.setSingleStep(1); self.mode_slider.setTickInterval(1)
        self.mode_slider.setTickPosition(QSlider.TicksBelow); self.mode_slider.setFixedWidth(420)
        self.mode_slider.valueChanged.connect(self._on_mode_slider_changed)
        self.mode_now = QLabel("대기모드"); self.mode_now.setStyleSheet("font-weight:600; padding-left:8px;")
        lbl = QLabel("모드:"); lbl.setStyleSheet("color:#789; padding-right:6px;")
        top_layout.addWidget(lbl); top_layout.addWidget(self.mode_slider); top_layout.addWidget(self.mode_now)

        # ── 좌측 보조 패널
        self._side = _MissionSidePanel(on_log=self._append_log_line, parent=self)
        self._side.btn_start.clicked.connect(self._on_click_start_sim)

        # (우측) 기존 상단/탭을 세로 배치
        right = QWidget(); v = QVBoxLayout(right); v.setContentsMargins(0,0,0,0)
        v.addWidget(top); v.addWidget(tabs)

        # (중앙) 좌측 패널 + 우측 기존 영역을 가로 배치
        center = QWidget(); h = QHBoxLayout(center); h.setContentsMargins(0,0,0,0); h.setSpacing(10)
        h.addWidget(self._side, 0, Qt.AlignTop); h.addWidget(right, 1)
        self.setCentralWidget(center)

        # 리플레이 매니저
        self._replay = _ReplayManager(self._tab, NodeMessenger, self._append_log_line, self._side.tracker_0401)

        # 초기 OFF
        self._set_mode_slider_by_text("전원 OFF")
        self._apply_power_state()

        # 버스 초기화 + CTRL 리스너 + UDP 제어 수신
        self.ctrl_payload.connect(self._handle_ctrl_payload)
        threading.Thread(target=self._rx_setup, daemon=True).start()
        self._start_control_udp()

        # GUI 표시 후 상태 OK 송신(모듈 코드: INT)
        QTimer.singleShot(800, lambda: send_status_ok("INT"))

        # 외부 self_check=1 수신 시에도 상태 OK 송신
        start_ctrl_listener(env_ctrl_port(45985), lambda payload: (
            send_status_ok("INT") if (payload or {}).get("cmd") == "self_check" and int((payload or {}).get("status", 0)) == 1 else None
        ))

        # Power OFF 시 UI 입력 차단 가드 설치
        self._install_power_gate_hooks()

    # ───────── Power Gate ─────────
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
                return
            tbl.setEnabled(enabled)
            for r in range(tbl.rowCount()):
                w = tbl.cellWidget(r, 3)   # '발신' 버튼 컬럼
                if w is not None and hasattr(w, "setEnabled"):
                    w.setEnabled(enabled)
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
        labels = ["전원 OFF", "전원 ON", "대기모드", "초기 임무 계획", "임무 수행"]
        try: self.mode_now.setText(labels[int(val)])
        except Exception: pass
        self._power_on = (int(val) != 0)
        self._apply_power_state()

    def _set_mode_slider_by_text(self, text: str):
        labels = ["전원 OFF", "전원 ON", "대기모드", "초기 임무 계획", "임무 수행"]
        norm = re.sub(r"\s+", "", str(text)).lower()
        mapping = {
            "전원off":0,"off":0,"poweroff":0,"0":0,
            "전원on":1,"on":1,"poweron":1,"1":1,
            "대기모드":2,"대기":2,"standby":2,"2":2,
            "초기임무계획":3,"초기임무계획모드":3,"initplan":3,"initial":3,"3":3,
            "임무수행":4,"execution":4,"4":4,
        }
        val = mapping.get(norm, 2)
        try:
            if getattr(self, "mode_slider", None):
                if self.mode_slider.value() != val:
                    self.mode_slider.blockSignals(True); self.mode_slider.setValue(val); self.mode_slider.blockSignals(False)
            if getattr(self, "mode_now", None): self.mode_now.setText(labels[val])
        except Exception: pass
        self._power_on = (int(val) != 0)

    # ───────── 버스 초기화 ─────────
    def _rx_setup(self):
        try:
            FusionNodeIoc.Configure()
            NodeMessenger.Initialize("INT_ReceiveNode")
            NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
            NodeMessenger.InitAllSubscriberFromAssembly()
            NodeMessenger.RegistAllProviderFromFusionNodeIoc()
        except Exception as e:
            self._append_log_line(f"[BUS] init 실패: {e}")

    # ───────── UDP 제어 수신 ─────────
    def _start_control_udp(self):
        import socket
        if getattr(self, "_ctrl_udp_started", False): return
        self._ctrl_udp_started = True

        port = int(os.getenv("KU_CTRL_PORT", "45985"))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", port))
            self._append_log_line(f"CTRL UDP 수신 대기 시작 (127.0.0.1:{port})")
        except Exception as e:
            self._append_log_line(f"CTRL UDP 바인드 실패: {e}")
            return

        def loop():
            while True:
                try:
                    data, _ = sock.recvfrom(8192)
                    payload = json.loads(data.decode("utf-8", "ignore"))
                    self.ctrl_payload.emit(payload)
                except Exception:
                    pass
        threading.Thread(target=loop, daemon=True).start()

    # ───────── CTRL 처리 ─────────
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
    def _append_log_line(self, text: str):
        try:
            if getattr(self, "_tab", None) and hasattr(self._tab, "append_log"):
                self._tab.append_log(text); return
        except Exception:
            pass
        try: print(text)
        except Exception: pass

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
