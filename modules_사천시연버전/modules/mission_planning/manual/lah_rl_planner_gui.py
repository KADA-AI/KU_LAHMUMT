"""
LAH 경로계획 GUI (Hex Grid 기반)
──────────────────────────────────
- 육각형 맵 자동 생성 (DEM 좌표 기반 자동 선택)
- 기존 LAH A* 경로계획 / RL PPO 경로계획 전환
- 고도 600m 기본, 간격 기반 묶기 유지
"""
from __future__ import annotations

import heapq
import math
import os
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

from PyQt5.QtCore import Qt, pyqtSignal, QPointF
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen, QPolygonF, QFont
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

# ── Path setup ──────────────────────────────────────────────────
_FILE_DIR = Path(__file__).resolve().parent
_MISSION_ROOT = _FILE_DIR.parent
_PROJECT_ROOT = _MISSION_ROOT.parents[1]  # DSS_KU/
_RESOURCE_DIR = _PROJECT_ROOT / "resource"
_BUNDLE_ROOT = _MISSION_ROOT / "MissionPlanner" / "portable_mission_bundle"
_MODEL_PATH = _BUNDLE_ROOT / "models" / "latest_model.zip"

# portable_mission 패키지의 __init__.py 가 Flask 를 import 하므로,
# __init__.py 를 건너뛰고 패키지를 수동 등록한 뒤 하위 모듈을 import 한다.
import types as _types
import importlib as _il

_PM_PKG = "portable_mission"
_PM_DIR = _BUNDLE_ROOT / _PM_PKG
if str(_BUNDLE_ROOT) not in sys.path:
    sys.path.insert(0, str(_BUNDLE_ROOT))
# 빈 패키지로 등록 (Flask 의존 __init__.py 우회)
if _PM_PKG not in sys.modules:
    _pkg = _types.ModuleType(_PM_PKG)
    _pkg.__path__ = [str(_PM_DIR)]
    _pkg.__package__ = _PM_PKG
    sys.modules[_PM_PKG] = _pkg

from portable_mission.hex_utils import (  # noqa: E402
    DIRECTION_SEQUENCE,
    offset_to_axial,
    step_neighbor,
)
from portable_mission.terrain import crop_dem_by_normalized_roi  # noqa: E402
from portable_mission.env import PortableMissionEnv  # noqa: E402

# ── Defaults ────────────────────────────────────────────────────
DEFAULT_LAT = 38.057393
DEFAULT_LON = 127.410630
DEFAULT_ALTITUDE_M = 600.0
DEFAULT_HEX_STEP = 25
DEFAULT_AREA_KM = 10.0

# ── Utilities ───────────────────────────────────────────────────


def auto_select_dem(lat: float, lon: float) -> Optional[Path]:
    """좌표 기반 SRTM DEM 타일 자동 선택."""
    tile_lat = int(math.floor(lat))
    tile_lon = int(math.floor(lon))
    candidates = [
        f"n{tile_lat}_e{tile_lon}_1arc_v3.tif",
        "Jipo_48km.tif",
        "Hongik_48km.tif",
        "Inje_48km.tif",
    ]
    for name in candidates:
        p = _RESOURCE_DIR / name
        if p.exists():
            return p
    return None


def compute_roi_normalized(
    dem_path: Path, center_lat: float, center_lon: float, area_km: float
) -> Dict[str, float]:
    """중심 좌표·영역 크기 → 정규화 ROI {x0, y0, x1, y1}."""
    import rasterio

    with rasterio.open(str(dem_path)) as ds:
        bounds = ds.bounds

    half_lat = (area_km / 2.0) / 111.0
    cos_lat = max(0.01, math.cos(math.radians(center_lat)))
    half_lon = (area_km / 2.0) / (111.0 * cos_lat)

    dem_w = bounds.right - bounds.left
    dem_h = bounds.top - bounds.bottom

    x0 = (center_lon - half_lon - bounds.left) / dem_w
    x1 = (center_lon + half_lon - bounds.left) / dem_w
    y0 = 1.0 - (center_lat + half_lat - bounds.bottom) / dem_h
    y1 = 1.0 - (center_lat - half_lat - bounds.bottom) / dem_h

    x0, x1 = sorted([max(0.0, min(1.0, x0)), max(0.0, min(1.0, x1))])
    y0, y1 = sorted([max(0.0, min(1.0, y0)), max(0.0, min(1.0, y1))])
    return {"x0": x0, "y0": y0, "x1": x1, "y1": y1}


def _hex_dist(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    q1, r1 = offset_to_axial(a[0], a[1])
    q2, r2 = offset_to_axial(b[0], b[1])
    dq, dr = q2 - q1, r2 - r1
    return max(abs(dq), abs(dr), abs(-dq - dr))


def hex_astar(
    occupancy: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
) -> List[Tuple[int, int]]:
    """A* 최단경로 (Hex grid, 장애물 회피)."""
    nrows, ncols = occupancy.shape
    open_set: list = [(_hex_dist(start, goal), 0, start)]
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    g_score: Dict[Tuple[int, int], int] = {start: 0}
    closed: Set[Tuple[int, int]] = set()

    while open_set:
        _, cost, current = heapq.heappop(open_set)
        if current in closed:
            continue
        closed.add(current)
        if current == goal:
            path: list = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return list(reversed(path))
        row, col = current
        for d in DIRECTION_SEQUENCE:
            nr, nc = step_neighbor(row, col, d)
            if 0 <= nr < nrows and 0 <= nc < ncols and not occupancy[nr, nc] and (nr, nc) not in closed:
                new_g = g_score[current] + 1
                if new_g < g_score.get((nr, nc), 999_999):
                    came_from[(nr, nc)] = current
                    g_score[(nr, nc)] = new_g
                    heapq.heappush(open_set, (new_g + _hex_dist((nr, nc), goal), new_g, (nr, nc)))
    return []


# ── Hex Canvas ──────────────────────────────────────────────────


class HexGridCanvas(QWidget):
    """육각형 그리드 시각화 및 셀 선택 캔버스."""

    cellLeftClicked = pyqtSignal(int, int)
    cellRightClicked = pyqtSignal(int, int)
    hoverChanged = pyqtSignal(int, int)  # row, col

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.occupancy: Optional[np.ndarray] = None
        self.grid_shape: Tuple[int, int] = (0, 0)
        self.cell_size = 12.0
        self.margin = 30
        self.start_cell: Optional[Tuple[int, int]] = None
        self.goal_cells: List[Tuple[int, int]] = []
        self.path_cells: Set[Tuple[int, int]] = set()
        self._centers: Dict[Tuple[int, int], Tuple[float, float]] = {}
        self._hover: Optional[Tuple[int, int]] = None
        self.setMouseTracking(True)

    # -- public API --

    def set_grid(self, occupancy: np.ndarray) -> None:
        self.occupancy = occupancy
        self.grid_shape = occupancy.shape
        self.clear_selection()
        self._rebuild_layout()

    def set_path(self, cells: List[Tuple[int, int]]) -> None:
        self.path_cells = set(cells)
        self.update()

    def clear_selection(self) -> None:
        self.start_cell = None
        self.goal_cells = []
        self.path_cells = set()
        self.update()

    # -- layout --

    def _rebuild_layout(self) -> None:
        if self.occupancy is None:
            return
        nrows, ncols = self.grid_shape
        s = self.cell_size
        horiz = math.sqrt(3) * s
        vert = 1.5 * s
        self._centers = {}
        for r in range(nrows):
            for c in range(ncols):
                cx = self.margin + c * horiz + (r % 2) * (horiz / 2.0)
                cy = self.margin + r * vert
                self._centers[(r, c)] = (cx, cy)
        w = int(self.margin * 2 + ncols * horiz + horiz)
        h = int(self.margin * 2 + nrows * vert + s)
        self.setMinimumSize(w, h)
        self.resize(w, h)
        self.update()

    # -- rendering --

    _COLOR_SAFE = QColor(195, 215, 195)
    _COLOR_BLOCKED = QColor(60, 60, 60)
    _COLOR_START = QColor(30, 180, 30)
    _COLOR_GOAL = QColor(220, 50, 50)
    _COLOR_PATH = QColor(255, 200, 40)
    _COLOR_HOVER = QColor(160, 200, 255)
    _PEN_GRID = QPen(QColor(120, 120, 120), 0.5)

    def paintEvent(self, event) -> None:  # noqa: N802
        if self.occupancy is None:
            return
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        nrows, ncols = self.grid_shape
        s = self.cell_size
        path = self.path_cells

        for r in range(nrows):
            for c in range(ncols):
                center = self._centers.get((r, c))
                if center is None:
                    continue
                cx, cy = center
                cell = (r, c)

                # pick colour
                if cell == self.start_cell:
                    fill = self._COLOR_START
                elif cell in self.goal_cells:
                    fill = self._COLOR_GOAL
                elif cell in path:
                    fill = self._COLOR_PATH
                elif cell == self._hover and not self.occupancy[r, c]:
                    fill = self._COLOR_HOVER
                elif self.occupancy[r, c]:
                    fill = self._COLOR_BLOCKED
                else:
                    fill = self._COLOR_SAFE

                painter.setBrush(QBrush(fill))
                painter.setPen(self._PEN_GRID)
                painter.drawPolygon(self._hex_poly(cx, cy, s))

        # draw labels on start / goal
        painter.setPen(QPen(Qt.white))
        font = QFont()
        font.setPointSize(max(6, int(s * 0.6)))
        font.setBold(True)
        painter.setFont(font)
        if self.start_cell and self.start_cell in self._centers:
            cx, cy = self._centers[self.start_cell]
            painter.drawText(QPointF(cx - s * 0.3, cy + s * 0.25), "S")
        for i, g in enumerate(self.goal_cells):
            if g in self._centers:
                cx, cy = self._centers[g]
                painter.drawText(QPointF(cx - s * 0.3, cy + s * 0.25), f"G{i + 1}")

        painter.end()

    @staticmethod
    def _hex_poly(cx: float, cy: float, s: float) -> QPolygonF:
        pts = []
        for i in range(6):
            a = math.radians(60 * i - 30)
            pts.append(QPointF(cx + s * math.cos(a), cy + s * math.sin(a)))
        return QPolygonF(pts)

    # -- interaction --

    def _cell_at(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        best, best_d = None, float("inf")
        for (r, c), (cx, cy) in self._centers.items():
            d = (x - cx) ** 2 + (y - cy) ** 2
            if d < best_d:
                best_d = d
                best = (r, c)
        if best is not None and best_d < (self.cell_size * 1.2) ** 2:
            return best
        return None

    def mousePressEvent(self, event) -> None:  # noqa: N802
        cell = self._cell_at(event.pos().x(), event.pos().y())
        if cell is None:
            return
        if event.button() == Qt.LeftButton:
            self.cellLeftClicked.emit(cell[0], cell[1])
        elif event.button() == Qt.RightButton:
            self.cellRightClicked.emit(cell[0], cell[1])

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        cell = self._cell_at(event.pos().x(), event.pos().y())
        if cell != self._hover:
            self._hover = cell
            if cell is not None:
                self.hoverChanged.emit(cell[0], cell[1])
            self.update()

    def wheelEvent(self, event) -> None:  # noqa: N802
        delta = event.angleDelta().y()
        if delta > 0:
            self.cell_size = min(40.0, self.cell_size + 1)
        else:
            self.cell_size = max(4.0, self.cell_size - 1)
        self._rebuild_layout()


# ── Main Window ─────────────────────────────────────────────────


class LAHPlannerWindow(QMainWindow):
    """LAH 경로계획 GUI — Hex Grid 기반 A*/RL 전환."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("LAH 경로계획 (Hex Grid)")
        self.resize(1200, 800)

        self.env: Optional[PortableMissionEnv] = None
        self._model = None
        self._tmp_dem: Optional[str] = None
        self._dem_name: str = ""

        self._build_ui()
        self.statusBar().showMessage("좌표를 입력하고 '맵 생성' 버튼을 누르세요.")

    # ── UI construction ──

    def _build_ui(self) -> None:
        central = QWidget(self)
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        # -- left panel --
        left = QVBoxLayout()
        left.setSpacing(8)

        # coordinate group
        cg = QGroupBox("좌표 / 영역")
        cf = QFormLayout()
        self.lat_spin = QDoubleSpinBox()
        self.lat_spin.setRange(33.0, 43.0)
        self.lat_spin.setDecimals(6)
        self.lat_spin.setValue(DEFAULT_LAT)
        self.lon_spin = QDoubleSpinBox()
        self.lon_spin.setRange(124.0, 132.0)
        self.lon_spin.setDecimals(6)
        self.lon_spin.setValue(DEFAULT_LON)
        self.area_spin = QDoubleSpinBox()
        self.area_spin.setRange(1.0, 50.0)
        self.area_spin.setDecimals(1)
        self.area_spin.setValue(DEFAULT_AREA_KM)
        self.area_spin.setSuffix(" km")
        cf.addRow("위도:", self.lat_spin)
        cf.addRow("경도:", self.lon_spin)
        cf.addRow("영역:", self.area_spin)
        cg.setLayout(cf)
        left.addWidget(cg)

        # settings group
        sg = QGroupBox("설정")
        sf = QFormLayout()
        self.alt_spin = QSpinBox()
        self.alt_spin.setRange(100, 2000)
        self.alt_spin.setValue(int(DEFAULT_ALTITUDE_M))
        self.alt_spin.setSuffix(" m")
        self.step_spin = QSpinBox()
        self.step_spin.setRange(5, 100)
        self.step_spin.setValue(DEFAULT_HEX_STEP)
        sf.addRow("고도:", self.alt_spin)
        sf.addRow("Hex Step:", self.step_spin)
        sg.setLayout(sf)
        left.addWidget(sg)

        self.btn_build = QPushButton("맵 생성")
        self.btn_build.setFixedHeight(38)
        self.btn_build.setStyleSheet("font-weight: bold;")
        self.btn_build.clicked.connect(self._on_build_map)
        left.addWidget(self.btn_build)

        # mode switch
        mg = QGroupBox("경로계획 모드")
        ml = QVBoxLayout()
        self.radio_astar = QRadioButton("기존 LAH 경로계획 (A*)")
        self.radio_rl = QRadioButton("RL 경로계획 (PPO)")
        self.radio_astar.setChecked(True)
        ml.addWidget(self.radio_astar)
        ml.addWidget(self.radio_rl)
        mg.setLayout(ml)
        left.addWidget(mg)

        # selection info
        self.info_label = QLabel("좌클릭: 시작점  |  우클릭: 목표점 추가/제거")
        self.info_label.setWordWrap(True)
        self.info_label.setStyleSheet("color:#555; padding:4px;")
        left.addWidget(self.info_label)

        self.btn_run = QPushButton("경로 생성")
        self.btn_run.setFixedHeight(38)
        self.btn_run.setStyleSheet("font-weight: bold;")
        self.btn_run.setEnabled(False)
        self.btn_run.clicked.connect(self._on_run)
        left.addWidget(self.btn_run)

        btn_row = QHBoxLayout()
        self.btn_clear = QPushButton("선택 초기화")
        self.btn_clear.clicked.connect(self._on_clear)
        btn_row.addWidget(self.btn_clear)
        left.addLayout(btn_row)

        self.result_label = QLabel("")
        self.result_label.setWordWrap(True)
        self.result_label.setStyleSheet(
            "padding:8px; background:#f4f4f4; border:1px solid #ddd; border-radius:4px;"
        )
        left.addWidget(self.result_label)

        self.hover_label = QLabel("")
        self.hover_label.setStyleSheet("color:#888; font-size:11px; padding:2px;")
        left.addWidget(self.hover_label)

        left.addStretch(1)

        left_w = QWidget()
        left_w.setLayout(left)
        left_w.setFixedWidth(280)
        root.addWidget(left_w)

        # -- right: canvas --
        self.canvas = HexGridCanvas()
        self.canvas.cellLeftClicked.connect(self._on_left_click)
        self.canvas.cellRightClicked.connect(self._on_right_click)
        self.canvas.hoverChanged.connect(self._on_hover)

        scroll = QScrollArea()
        scroll.setWidget(self.canvas)
        scroll.setWidgetResizable(False)
        root.addWidget(scroll, 1)

    # ── Map build ──

    def _on_build_map(self) -> None:
        lat = self.lat_spin.value()
        lon = self.lon_spin.value()
        area_km = self.area_spin.value()
        alt = float(self.alt_spin.value())
        step = self.step_spin.value()

        dem_path = auto_select_dem(lat, lon)
        if dem_path is None:
            QMessageBox.critical(
                self,
                "DEM 없음",
                f"위도 {lat:.2f}, 경도 {lon:.2f} 에 해당하는 DEM 파일이 없습니다.\n"
                f"resource/ 폴더를 확인하세요.",
            )
            return

        self._dem_name = dem_path.name
        self.statusBar().showMessage(f"DEM 로딩: {dem_path.name} ...")
        QApplication.processEvents()

        try:
            roi = compute_roi_normalized(dem_path, lat, lon, area_km)
        except Exception as exc:
            QMessageBox.critical(self, "ROI 계산 실패", str(exc))
            return

        # crop to temp file
        self._cleanup_tmp()
        fd, tmp = tempfile.mkstemp(suffix=".tif")
        os.close(fd)
        self._tmp_dem = tmp

        try:
            crop_dem_by_normalized_roi(str(dem_path), roi, tmp)
        except Exception as exc:
            QMessageBox.critical(self, "DEM 크롭 실패", str(exc))
            return

        try:
            self.env = PortableMissionEnv(
                dem_path=tmp,
                altitude_m=alt,
                hex_step=step,
                max_steps=3000,
                max_goals=20,
            )
        except Exception as exc:
            QMessageBox.critical(self, "환경 생성 실패", str(exc))
            return

        self.canvas.set_grid(self.env.occupancy)
        self.btn_run.setEnabled(True)
        self.result_label.setText("")

        nrows, ncols = self.env.grid_shape
        safe = self.env.safe_count
        blocked = self.env.blocked_count
        self.statusBar().showMessage(
            f"맵 생성 완료  |  {nrows}x{ncols} 그리드  |  안전 {safe}  |  장애물 {blocked}  |  DEM: {dem_path.name}"
        )

    # ── Cell selection ──

    def _on_left_click(self, row: int, col: int) -> None:
        if self.env is None:
            return
        if self.env.occupancy[row, col]:
            self.statusBar().showMessage(f"({row}, {col}) 은 장애물입니다.")
            return
        self.canvas.start_cell = (row, col)
        self.canvas.path_cells = set()
        self.canvas.update()
        self._refresh_info()

    def _on_right_click(self, row: int, col: int) -> None:
        if self.env is None:
            return
        if self.env.occupancy[row, col]:
            self.statusBar().showMessage(f"({row}, {col}) 은 장애물입니다.")
            return
        cell = (row, col)
        if cell in self.canvas.goal_cells:
            self.canvas.goal_cells.remove(cell)
        else:
            self.canvas.goal_cells.append(cell)
        self.canvas.path_cells = set()
        self.canvas.update()
        self._refresh_info()

    def _on_hover(self, row: int, col: int) -> None:
        if self.env is None:
            return
        blocked = "장애물" if self.env.occupancy[row, col] else "안전"
        self.hover_label.setText(f"셀 ({row}, {col})  [{blocked}]")

    def _refresh_info(self) -> None:
        parts: list[str] = []
        s = self.canvas.start_cell
        if s:
            parts.append(f"시작: ({s[0]}, {s[1]})")
        for i, g in enumerate(self.canvas.goal_cells):
            parts.append(f"목표{i + 1}: ({g[0]}, {g[1]})")
        self.info_label.setText("\n".join(parts) if parts else "좌클릭: 시작점  |  우클릭: 목표점")

    def _on_clear(self) -> None:
        self.canvas.clear_selection()
        self.result_label.setText("")
        self._refresh_info()

    # ── Run planning ──

    def _on_run(self) -> None:
        if self.env is None:
            return
        start = self.canvas.start_cell
        goals = self.canvas.goal_cells
        if not start:
            QMessageBox.warning(self, "시작점 없음", "좌클릭으로 시작점을 선택하세요.")
            return
        if not goals:
            QMessageBox.warning(self, "목표점 없음", "우클릭으로 목표점을 추가하세요.")
            return

        if self.radio_astar.isChecked():
            self._run_astar(start, goals)
        else:
            self._run_rl(start, goals)

    def _run_astar(self, start: Tuple[int, int], goals: List[Tuple[int, int]]) -> None:
        self.statusBar().showMessage("A* 경로 탐색 중...")
        QApplication.processEvents()

        full_path: list[Tuple[int, int]] = [start]
        current = start
        remaining = list(goals)

        while remaining:
            nearest = min(remaining, key=lambda g: _hex_dist(current, g))
            remaining.remove(nearest)
            segment = hex_astar(self.env.occupancy, current, nearest)
            if not segment:
                QMessageBox.warning(
                    self,
                    "경로 없음",
                    f"({current[0]},{current[1]}) → ({nearest[0]},{nearest[1]})\n경로를 찾을 수 없습니다.",
                )
                return
            full_path.extend(segment[1:])
            current = nearest

        self.canvas.set_path(full_path)
        self.result_label.setText(
            f"<b>[A*] 경로 생성 완료</b><br>"
            f"총 <b>{len(full_path)}</b> 셀<br>"
            f"목표: {len(goals)}개 도달"
        )
        self.statusBar().showMessage(f"A* 완료  |  {len(full_path)} 셀")

    def _run_rl(self, start: Tuple[int, int], goals: List[Tuple[int, int]]) -> None:
        if not _MODEL_PATH.exists():
            QMessageBox.critical(
                self,
                "모델 없음",
                f"PPO 모델 파일이 없습니다:\n{_MODEL_PATH}",
            )
            return

        self.statusBar().showMessage("RL 모델 로딩 중...")
        QApplication.processEvents()

        try:
            if self._model is None:
                from stable_baselines3 import PPO

                self._model = PPO.load(str(_MODEL_PATH), device="cpu")

            self.statusBar().showMessage("RL 경로 생성 중...")
            QApplication.processEvents()

            obs, _info = self.env.configure_manual_episode(start, goals)

            path_cells: list[Tuple[int, int]] = [start]
            total_reward = 0.0
            done = False
            steps = 0
            goals_reached = 0

            while not done:
                action, _ = self._model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = self.env.step(int(action))
                total_reward += reward
                steps += 1
                done = terminated or truncated
                cell = getattr(self.env, "current_cell", None)
                if cell is not None:
                    path_cells.append(tuple(cell))
                goals_reached = getattr(self.env, "current_goal_idx", goals_reached)

            reason = info.get("termination_reason", "unknown")

            self.canvas.set_path(path_cells)
            self.result_label.setText(
                f"<b>[RL/PPO] 경로 생성 완료</b><br>"
                f"스텝: <b>{steps}</b>  |  셀: <b>{len(path_cells)}</b><br>"
                f"목표: {goals_reached}/{len(goals)}개 도달<br>"
                f"보상: {total_reward:.2f}<br>"
                f"종료: {reason}"
            )
            self.statusBar().showMessage(f"RL 완료  |  {steps} 스텝  |  {reason}")

        except Exception as exc:
            import traceback

            traceback.print_exc()
            QMessageBox.critical(self, "RL 실행 실패", str(exc))

    # ── Cleanup ──

    def _cleanup_tmp(self) -> None:
        if self._tmp_dem and Path(self._tmp_dem).exists():
            try:
                os.unlink(self._tmp_dem)
            except Exception:
                pass
        self._tmp_dem = None

    def closeEvent(self, event) -> None:  # noqa: N802
        self._cleanup_tmp()
        super().closeEvent(event)


# ── Entry point ─────────────────────────────────────────────────


def main() -> None:
    app = QApplication(sys.argv)
    win = LAHPlannerWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
