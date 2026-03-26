# -*- coding: utf-8 -*-
# monitoring_gui.py - Monitoring (MSM) GUI (send/receive only)
from __future__ import annotations

import os
import sys
import threading
import json
import re
import time
import traceback
from pathlib import Path
from typing import Callable

os.environ["KU_ROLE"] = "monitoring"

_ROOT = Path(__file__).resolve().parents[2]  # .../KU_LAHMUMT
for _p in (_ROOT, _ROOT / "modules", _ROOT / "modules" / "common"):
    _ps = str(_p)
    if _p.exists() and _ps not in sys.path:
        sys.path.insert(0, _ps)

from modules.common.qt_env import ensure_qt_platform
ensure_qt_platform()
from modules.common.gui_style import load_shared_stylesheet, polish_tabs, position_window_from_env
from modules.common.process_console import emit_process_log, ensure_console, install_process_file_logging

ensure_console(os.getenv("KU_CONSOLE_TITLE", "KU Monitoring Console"))
install_process_file_logging("monitoring")

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, QTimer, Qt, QEvent, QObject, QRect
from PyQt5.QtGui import QPainter, QColor, QFontMetrics, QFont
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QTabWidget,
    QWidget,
    QLabel,
    QHBoxLayout,
    QVBoxLayout,
    QSlider,
    QStyle,
    QStyleOptionSlider,
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



def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message:
        return
    sys.stderr.write(message + "\n")


qInstallMessageHandler(_qt_silent_handler)


def _bootstrap_paths():
    here = Path(__file__).resolve()
    modules_dir = here.parents[1]  # .../modules
    root = modules_dir.parent
    common_dir = modules_dir / "common"
    for p in (modules_dir / "monitoring", common_dir, root):
        p_str = str(p)
        if p.exists() and p_str not in sys.path:
            sys.path.insert(0, p_str)
    try:
        os.chdir(root)
    except Exception:
        pass
    return root, common_dir


PROJECT_ROOT, COMMON_DIR = _bootstrap_paths()

from modules.common import db_paths
from modules.common import agent_status_snapshot
from modules.common.fusion_files import copy_file_with_retry
from modules.monitoring.gui.tabs.mission_schedule_tab import MissionScheduleTab
from modules.monitoring.gui.tabs.quality_monitor_tab import QualityMonitorTab
from modules.monitoring.gui.tabs.replan_management_tab import ReplanManagementTab
from modules.monitoring.gui.tabs.monitoring_visualization_tab import (
    MonitoringVisualizationTab,
    RealtimeRiskPredictionTab,
)
from modules.monitoring.gui.tabs.turn_radius_monitor_tab import TurnRadiusMonitorTab
from modules.monitoring.logic.init_replan import (
    allocate_mission_plan_ids,
    build_replan_context,
    collect_input_mission_ids,
)
from modules.monitoring.logic.mission_update import (
    collect_available_aircraft_ids,
    extract_0401_agent_states,
    extract_0802_command,
    extract_0702_decision,
    extract_0803_execute,
    extract_0903_info,
    format_timestamp_ms,
    mission_plan_json_path,
    parse_payload,
)
from modules.monitoring.logic.collab_reexecute import CollabReexecuteCoordinator
from modules.monitoring.logic.fuel_warning import FuelWarningCoordinator
from modules.monitoring.logic.forced_command_replan import ForcedCommandReplanCoordinator
from modules.monitoring.logic.imaging_schedule_replan import ImagingScheduleReplanCoordinator
from modules.monitoring.logic.next_collab_replan import NextCollabMissionReplanCoordinator
from modules.monitoring.logic.quality_speed_replan import QualitySpeedReplanCoordinator
from modules.monitoring.logic.input_refresh_replan import InputRefreshReplanCoordinator
from modules.monitoring.logic.path_deviation_replan import PathDeviationReplanCoordinator
from modules.monitoring.logic.prior_mission_replan import PriorMissionReplanCoordinator
from modules.monitoring.logic.replan_runtime_settings import (
    get_dl_risk_settings,
    get_input_refresh_settings,
    get_replan_toggle,
    update_replan_toggle,
)
from modules.monitoring.logic.rtb_replan import RtbReplanCoordinator
from modules.monitoring.logic.target_detection_replan import TargetDetectionCoordinator
from modules.monitoring.logic.target_info import mark_targets_as_ignored
from modules.monitoring.utils.vehicle_status import write_vehicle_status
from Tabs.csc_tab_base import _now_ms_since_2000
try:
    from modules.mission_planning.runtime.attack_assignment_state import (
        commit_pending_manned_assignment,
    )
except Exception:
    def commit_pending_manned_assignment(_mission_plan_id: int | None) -> int | None:
        return None

try:
    from modules.monitoring.logic.risk_analysis import RealTimeInferrer, evaluate_risk_thresholds
except Exception:
    RealTimeInferrer = None
    evaluate_risk_thresholds = None


def _ensure_fusion_configs():
    cands = [
        PROJECT_ROOT / "nFusionSettings.json",
        COMMON_DIR / "nFusionSettings.json",
        PROJECT_ROOT / "FusionSettings.json",
        COMMON_DIR / "FusionSettings.json",
        PROJECT_ROOT / "nFusion" / "FusionSettings.json",
    ]
    src = next((p for p in cands if p.exists()), None)
    if src is None:
        raise FileNotFoundError("nFusionSettings.json/FusionSettings.json is missing.")
    dst = PROJECT_ROOT / "nFusionSettings.json"
    if src != dst:
        copy_file_with_retry(src, dst)

    lcands = [
        PROJECT_ROOT / "nFusionLicense.lic",
        COMMON_DIR / "nFusionLicense.lic",
        PROJECT_ROOT / "nFusion" / "nFusionLicense.lic",
    ]
    lsrc = next((p for p in lcands if p.exists()), None)
    if lsrc:
        ldst = PROJECT_ROOT / "nFusionLicense.lic"
        if lsrc != ldst:
            copy_file_with_retry(lsrc, ldst)
    return str(dst)


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

    def _already_loaded(exc: Exception) -> bool:
        return "already loaded" in str(exc).lower()

    try:
        _clr.AddReference(str(stem))
    except Exception as exc:
        if not _already_loaded(exc):
            try:
                _clr.AddReference(str(stem.with_suffix(".dll")))
            except Exception as exc2:
                if not _already_loaded(exc2):
                    raise
    for s in ("K4586Model", "K4586Model.Assist", "MiscUtil"):
        dll = msg_dir / (s + ".dll")
        if dll.exists():
            try:
                _clr.AddReference(str(dll.with_suffix("")))
            except Exception:
                try:
                    _clr.AddReference(str(dll))
                except Exception:
                    pass


_settings_path = _ensure_fusion_configs()
_ = _load_msglib_and_deps()

from receive import *  # noqa
from receive_center import register_listener
from Tabs.mission_monitoring_tab import MissionMonitoringTab

_MODE_LABELS = ["초기화 모드", "대기모드", "초기 임무 계획", "임무 수행"]


class MainWindow(QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle("모니터링(MSM)")
        default_w, default_h = 1100, 700
        try:
            screen = QApplication.primaryScreen()
            if screen is not None:
                geom = screen.availableGeometry()
                default_w = min(default_w, int(geom.width() * 0.96))
                default_h = min(default_h, int(geom.height() * 0.90))
        except Exception:
            pass
        self.resize(default_w, default_h)

        self._power_on = True
        self._auto_initplan_triggered = False
        self._system_mode_code = None
        self._external_mode_code: int | None = None
        self._mode_manual_override = False
        self._mode_update_source: str | None = None
        self._last_ignored_external_mode_code: int | None = None
        self._send_0501_timer = None
        self._last_0501_attempt_monotonic = 0.0
        self._last_0501_payload_monotonic = 0.0
        self._last_0501_send_monotonic = 0.0
        self._last_0501_watchdog_state = "init"
        self._current_mission_plan_id: int | None = None
        self._active_0401_notice_keys: set[tuple[int, str]] = set()
        self._sent_0502_plans: set[int] = set()
        self._replan_option_meta_by_plan_id: dict[int, dict[str, object]] = {}
        self._availability_base_ids: set[int] = set()
        self._forced_availability_override: dict[int, bool] = {}
        self._rtb_availability_override: dict[int, bool] = {}
        self._availability_seen: bool = False
        self._last_0201_type_warning_key: tuple[tuple[str, ...], tuple[str, ...]] | None = None
        self._dl_enabled = False
        self._dl_visual_enabled = False
        self._dl_debug = False
        self._dl_inferrer = None
        self._dl_status = "INIT"
        self._dl_last_data_ts = 0.0
        self._dl_last_infer_ts = 0.0
        self._dl_last_status_log_ts = 0.0
        self._dl_replan_enabled = False
        self._dl_replan_last_ts = 0.0
        self._dl_timer = None
        self._dl_lock = threading.Lock()
        self._input_refresh_replan_enabled = get_replan_toggle("input_refresh", True)
        self._prior_mission_replan_enabled = get_replan_toggle("prior_mission", True)
        self._target_detection_replan_enabled = get_replan_toggle("target_detection", True)
        self._forced_command_replan_enabled = get_replan_toggle("forced_command", True)
        self._rtb_replan_enabled = get_replan_toggle("rtb", True)
        self._path_deviation_trigger_enabled = get_replan_toggle("path_deviation", True)
        self._schedule_replan_trigger_enabled = get_replan_toggle("imaging_schedule", False)
        self._next_collab_replan_trigger_enabled = get_replan_toggle("next_collab", False)
        self._fuel_threshold_logic_enabled = get_replan_toggle("fuel_threshold", False)
        self._quality_monitor_enabled = get_replan_toggle("quality_monitor", True)
        self._quality_speed_replan_enabled = get_replan_toggle("quality_speed", False)
        self._dl_replan_user_enabled = get_replan_toggle("dl_risk", False)
        self._init_0501_watchdog()

        tabs = QTabWidget()
        polish_tabs(tabs)
        self._tab = MissionMonitoringTab(messenger=NodeMessenger)
        self._csc_tab_index = tabs.addTab(self._tab, "모니터링 CSC")
        self._viz_tab = MonitoringVisualizationTab()
        self._viz_tab.set_recommend_callback(self._on_0503_recommend)
        self._viz_tab.set_notice_callback(self._on_notice)
        self._viz_tab.set_log_callback(self._append_log_line)
        self._schedule_tab = MissionScheduleTab()
        self._schedule_tab.set_log_callback(self._append_log_line)
        self._schedule_tab.set_path_trigger_toggle_callback(self._on_path_deviation_trigger_toggled)
        self._schedule_tab.set_path_trigger_enabled(self._path_deviation_trigger_enabled)
        self._schedule_tab.set_schedule_trigger_toggle_callback(self._on_schedule_replan_trigger_toggled)
        self._schedule_tab.set_schedule_trigger_enabled(self._schedule_replan_trigger_enabled)
        self._schedule_tab.set_next_collab_trigger_toggle_callback(self._on_next_collab_replan_trigger_toggled)
        self._schedule_tab.set_next_collab_trigger_enabled(self._next_collab_replan_trigger_enabled)
        self._schedule_tab.set_fuel_threshold_toggle_callback(self._on_fuel_threshold_logic_toggled)
        self._schedule_tab.set_fuel_threshold_enabled(self._fuel_threshold_logic_enabled)
        self._dl_risk_tab = RealtimeRiskPredictionTab()
        self._replan_management_tab = ReplanManagementTab()
        self._replan_management_tab.set_log_callback(self._append_log_line)
        self._quality_tab = QualityMonitorTab()
        self._quality_tab.set_log_callback(self._append_log_line)
        self._quality_tab.set_monitor_toggle_callback(self._on_quality_monitor_toggled)
        self._quality_tab.set_replan_toggle_callback(self._on_quality_speed_replan_toggled)
        self._quality_tab.set_monitor_enabled(self._quality_monitor_enabled)
        self._quality_tab.set_replan_enabled(self._quality_speed_replan_enabled)
        self._replan_management_tab.set_all_toggle_callbacks(self._build_replan_management_toggle_callbacks())
        self._replan_management_tab.set_all_toggle_states(
            {
                "input_refresh": self._input_refresh_replan_enabled,
                "prior_mission": self._prior_mission_replan_enabled,
                "dl_risk": self._dl_replan_user_enabled,
                "target_detection": self._target_detection_replan_enabled,
                "forced_command": self._forced_command_replan_enabled,
                "rtb": self._rtb_replan_enabled,
                "path_deviation": self._path_deviation_trigger_enabled,
                "quality_monitor": self._quality_monitor_enabled,
                "quality_speed": self._quality_speed_replan_enabled,
                "imaging_schedule": self._schedule_replan_trigger_enabled,
                "next_collab": self._next_collab_replan_trigger_enabled,
                "fuel_threshold": self._fuel_threshold_logic_enabled,
            },
            emit=False,
        )
        self._reexecute_coord = CollabReexecuteCoordinator(
            now_fn=_now_ms_since_2000,
            logger=self._append_log_line,
        )
        self._fuel_coord = FuelWarningCoordinator(now_fn=_now_ms_since_2000)
        self._fuel_coord.set_threshold_logic_enabled(self._fuel_threshold_logic_enabled)
        self._input_refresh_coord = InputRefreshReplanCoordinator(
            now_fn=_now_ms_since_2000,
            logger=self._append_log_line,
        )
        self._prior_mission_coord = PriorMissionReplanCoordinator(
            now_fn=_now_ms_since_2000,
            logger=self._append_log_line,
        )
        self._target_detection_coord = TargetDetectionCoordinator(
            now_fn=_now_ms_since_2000,
            logger=self._append_log_line,
        )
        self._forced_command_coord = ForcedCommandReplanCoordinator(
            now_fn=_now_ms_since_2000,
            logger=self._append_log_line,
        )
        self._rtb_replan_coord = RtbReplanCoordinator(
            now_fn=_now_ms_since_2000,
            logger=self._append_log_line,
        )
        self._path_deviation_coord = PathDeviationReplanCoordinator(
            now_fn=_now_ms_since_2000,
            logger=self._append_log_line,
        )
        self._next_collab_replan_coord = NextCollabMissionReplanCoordinator(
            now_fn=_now_ms_since_2000,
            logger=self._append_log_line,
        )
        self._imaging_schedule_coord = ImagingScheduleReplanCoordinator(
            now_fn=_now_ms_since_2000,
            logger=self._append_log_line,
        )
        self._quality_speed_coord = QualitySpeedReplanCoordinator(
            now_fn=_now_ms_since_2000,
            logger=self._append_log_line,
        )
        self._replan_management_tab_index = tabs.addTab(self._replan_management_tab, "임무 재계획 관리")
        self._viz_tab_index = tabs.addTab(self._viz_tab, "모니터링 시각화")
        self._dl_risk_tab_index = tabs.addTab(self._dl_risk_tab, "실시간 위험도 예측")
        self._schedule_tab_index = tabs.addTab(self._schedule_tab, "스케줄 모니터")
        self._quality_tab_index = tabs.addTab(self._quality_tab, "촬영품질")
        self._turn_radius_tab = TurnRadiusMonitorTab()
        self._turn_radius_tab_index = tabs.addTab(self._turn_radius_tab, "경로추종 모니터링")
        tabs.currentChanged.connect(self._on_tab_changed)
        self._on_tab_changed(tabs.currentIndex())

        top = QWidget()
        top.setObjectName("TopBar")
        top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(4, 2, 4, 2)
        top_layout.setSpacing(12)
        top_layout.addStretch(1)
        self.mode_slider = QSlider(Qt.Horizontal)
        self.mode_slider.setRange(0, 3)
        self.mode_slider.setSingleStep(1)
        self.mode_slider.setTickInterval(1)
        self.mode_slider.setTickPosition(QSlider.TicksBelow)
        self.mode_slider.setFixedWidth(420)
        self.mode_slider.valueChanged.connect(self._on_mode_slider_changed)
        slider_wrap = QWidget()
        slider_wrap.setObjectName("ModePanel")
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
        self.mode_now = QLabel("초기화 모드")
        self.mode_now.setObjectName("ModeStatusLabel")
        self.mode_now.setFixedWidth(140)
        self.mode_now.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        lbl = QLabel("모드:")
        lbl.setObjectName("ModeCaptionLabel")
        top_layout.addWidget(lbl)
        top_layout.addWidget(slider_wrap)
        top_layout.addWidget(self.mode_now)

        center = QWidget()
        vbox = QVBoxLayout(center)
        vbox.setContentsMargins(12, 12, 12, 12)
        vbox.setSpacing(10)
        vbox.addWidget(top)
        vbox.addWidget(tabs)
        self.setCentralWidget(center)

        self._install_power_gate_hooks()
        self._init_db_root_sync()
        self._set_mode_slider_by_text("초기화 모드")
        self._apply_power_state()

        threading.Thread(target=self._rx_setup, daemon=True).start()
        self._init_0102_autostart()

        self._install_0101_mode_listener()
        self._start_0101_rx_poller()
        self._install_0903_listener()
        self._start_0903_rx_poller()
        self._install_0702_listener()
        self._start_0702_rx_poller()
        self._install_0201_listener()
        self._start_0201_rx_poller()
        self._install_0202_listener()
        self._start_0202_rx_poller()
        self._install_0401_listener()
        self._start_0401_rx_poller()
        self._install_0402_listener()
        self._start_0402_rx_poller()
        self._install_0802_listener()
        self._start_0802_rx_poller()
        self._install_0803_listener()
        self._start_0803_rx_poller()
        self._start_forced_hold_timer()
        self._init_dl_inference()

    def _append_log_line(self, text: str) -> None:
        try:
            emit_process_log("monitoring", str(text))
        except Exception:
            pass
        try:
            if getattr(self, "_tab", None) and hasattr(self._tab, "append_log"):
                self._tab.append_log(text)
                return
        except Exception:
            pass

    def _persist_replan_toggle(self, key: str, enabled: bool) -> None:
        try:
            update_replan_toggle(str(key), bool(enabled))
        except Exception as exc:
            self._append_log_line(f"[REPLANCFG] toggle persist failed ({key}): {exc}")

    def _sync_replan_management_toggle(self, key: str, enabled: bool) -> None:
        tab = getattr(self, "_replan_management_tab", None)
        if tab is None or not hasattr(tab, "set_toggle_state"):
            return
        try:
            tab.set_toggle_state(str(key), bool(enabled), emit=False)
        except Exception:
            pass

    def _build_replan_management_toggle_callbacks(self) -> dict[str, Callable[[bool, dict[str, Any]], None]]:
        return {
            "input_refresh": lambda enabled, _state: self._on_input_refresh_replan_toggled(enabled),
            "prior_mission": lambda enabled, _state: self._on_prior_mission_replan_toggled(enabled),
            "dl_risk": lambda enabled, _state: self._on_dl_risk_replan_toggled(enabled),
            "target_detection": lambda enabled, _state: self._on_target_detection_replan_toggled(enabled),
            "forced_command": lambda enabled, _state: self._on_forced_command_replan_toggled(enabled),
            "rtb": lambda enabled, _state: self._on_rtb_replan_toggled(enabled),
            "path_deviation": lambda enabled, _state: self._on_path_deviation_trigger_toggled(enabled),
            "quality_monitor": lambda enabled, _state: self._on_quality_monitor_toggled(enabled),
            "quality_speed": lambda enabled, _state: self._on_quality_speed_replan_toggled(enabled),
            "imaging_schedule": lambda enabled, _state: self._on_schedule_replan_trigger_toggled(enabled),
            "next_collab": lambda enabled, _state: self._on_next_collab_replan_trigger_toggled(enabled),
            "fuel_threshold": lambda enabled, _state: self._on_fuel_threshold_logic_toggled(enabled),
        }

    def _on_path_deviation_trigger_toggled(self, enabled: bool) -> None:
        self._path_deviation_trigger_enabled = bool(enabled)
        self._sync_replan_management_toggle("path_deviation", self._path_deviation_trigger_enabled)
        schedule_tab = getattr(self, "_schedule_tab", None)
        if schedule_tab is not None and hasattr(schedule_tab, "set_path_trigger_enabled"):
            try:
                if bool(schedule_tab._path_trigger_enabled) != self._path_deviation_trigger_enabled:  # type: ignore[attr-defined]
                    schedule_tab.set_path_trigger_enabled(self._path_deviation_trigger_enabled)
            except Exception:
                pass
        self._persist_replan_toggle("path_deviation", self._path_deviation_trigger_enabled)
        state_text = "ON" if self._path_deviation_trigger_enabled else "OFF"
        self._append_log_line(f"[PATHDEV] monitoring trigger toggled -> {state_text}")

    def _on_schedule_replan_trigger_toggled(self, enabled: bool) -> None:
        self._schedule_replan_trigger_enabled = bool(enabled)
        self._sync_replan_management_toggle("imaging_schedule", self._schedule_replan_trigger_enabled)
        schedule_tab = getattr(self, "_schedule_tab", None)
        if schedule_tab is not None and hasattr(schedule_tab, "set_schedule_trigger_enabled"):
            try:
                if bool(schedule_tab._schedule_trigger_enabled) != self._schedule_replan_trigger_enabled:  # type: ignore[attr-defined]
                    schedule_tab.set_schedule_trigger_enabled(self._schedule_replan_trigger_enabled)
            except Exception:
                pass
        self._persist_replan_toggle("imaging_schedule", self._schedule_replan_trigger_enabled)
        state_text = "ON" if self._schedule_replan_trigger_enabled else "OFF"
        self._append_log_line(f"[SCHED] monitoring trigger toggled -> {state_text}")

    def _on_next_collab_replan_trigger_toggled(self, enabled: bool) -> None:
        self._next_collab_replan_trigger_enabled = bool(enabled)
        self._sync_replan_management_toggle("next_collab", self._next_collab_replan_trigger_enabled)
        schedule_tab = getattr(self, "_schedule_tab", None)
        if schedule_tab is not None and hasattr(schedule_tab, "set_next_collab_trigger_enabled"):
            try:
                if bool(schedule_tab._next_collab_trigger_enabled) != self._next_collab_replan_trigger_enabled:  # type: ignore[attr-defined]
                    schedule_tab.set_next_collab_trigger_enabled(self._next_collab_replan_trigger_enabled)
            except Exception:
                pass
        self._persist_replan_toggle("next_collab", self._next_collab_replan_trigger_enabled)
        state_text = "ON" if self._next_collab_replan_trigger_enabled else "OFF"
        self._append_log_line(f"[NEXTCOLLAB] monitoring trigger toggled -> {state_text}")

    def _on_fuel_threshold_logic_toggled(self, enabled: bool) -> None:
        self._fuel_threshold_logic_enabled = bool(enabled)
        self._sync_replan_management_toggle("fuel_threshold", self._fuel_threshold_logic_enabled)
        schedule_tab = getattr(self, "_schedule_tab", None)
        if schedule_tab is not None and hasattr(schedule_tab, "set_fuel_threshold_enabled"):
            try:
                if bool(schedule_tab._fuel_threshold_enabled) != self._fuel_threshold_logic_enabled:  # type: ignore[attr-defined]
                    schedule_tab.set_fuel_threshold_enabled(self._fuel_threshold_logic_enabled)
            except Exception:
                pass
        self._persist_replan_toggle("fuel_threshold", self._fuel_threshold_logic_enabled)
        fuel_coord = getattr(self, "_fuel_coord", None)
        if fuel_coord is not None and hasattr(fuel_coord, "set_threshold_logic_enabled"):
            try:
                fuel_coord.set_threshold_logic_enabled(self._fuel_threshold_logic_enabled)
            except Exception:
                pass
        state_text = "ON" if self._fuel_threshold_logic_enabled else "OFF"
        mode_text = "10/20% threshold + 0401" if self._fuel_threshold_logic_enabled else "0401 fuelWarning only"
        self._append_log_line(f"[FUEL] threshold auto-judge toggled -> {state_text} ({mode_text})")

    def _on_quality_monitor_toggled(self, enabled: bool) -> None:
        self._quality_monitor_enabled = bool(enabled)
        self._sync_replan_management_toggle("quality_monitor", self._quality_monitor_enabled)
        quality_tab = getattr(self, "_quality_tab", None)
        if quality_tab is not None and hasattr(quality_tab, "set_monitor_enabled"):
            try:
                if bool(quality_tab.is_monitor_enabled()) != self._quality_monitor_enabled:
                    quality_tab.set_monitor_enabled(self._quality_monitor_enabled)
            except Exception:
                pass
        self._persist_replan_toggle("quality_monitor", self._quality_monitor_enabled)
        state_text = "ON" if self._quality_monitor_enabled else "OFF"
        self._append_log_line(f"[QUALITY] monitoring trigger toggled -> {state_text}")

    def _on_quality_speed_replan_toggled(self, enabled: bool) -> None:
        self._quality_speed_replan_enabled = bool(enabled)
        self._sync_replan_management_toggle("quality_speed", self._quality_speed_replan_enabled)
        quality_tab = getattr(self, "_quality_tab", None)
        if quality_tab is not None and hasattr(quality_tab, "set_replan_enabled"):
            try:
                if bool(quality_tab.is_replan_enabled()) != self._quality_speed_replan_enabled:
                    quality_tab.set_replan_enabled(self._quality_speed_replan_enabled)
            except Exception:
                pass
        self._persist_replan_toggle("quality_speed", self._quality_speed_replan_enabled)
        state_text = "ON" if self._quality_speed_replan_enabled else "OFF"
        self._append_log_line(f"[QUALITY] speed replan trigger toggled -> {state_text}")

    def _on_input_refresh_replan_toggled(self, enabled: bool) -> None:
        self._input_refresh_replan_enabled = bool(enabled)
        self._sync_replan_management_toggle("input_refresh", self._input_refresh_replan_enabled)
        self._persist_replan_toggle("input_refresh", self._input_refresh_replan_enabled)
        state_text = "ON" if self._input_refresh_replan_enabled else "OFF"
        self._append_log_line(f"[REINPUT] monitoring trigger toggled -> {state_text}")

    def _on_prior_mission_replan_toggled(self, enabled: bool) -> None:
        self._prior_mission_replan_enabled = bool(enabled)
        self._sync_replan_management_toggle("prior_mission", self._prior_mission_replan_enabled)
        self._persist_replan_toggle("prior_mission", self._prior_mission_replan_enabled)
        state_text = "ON" if self._prior_mission_replan_enabled else "OFF"
        self._append_log_line(f"[PRIOR] monitoring trigger toggled -> {state_text}")

    def _on_target_detection_replan_toggled(self, enabled: bool) -> None:
        self._target_detection_replan_enabled = bool(enabled)
        self._sync_replan_management_toggle("target_detection", self._target_detection_replan_enabled)
        self._persist_replan_toggle("target_detection", self._target_detection_replan_enabled)
        state_text = "ON" if self._target_detection_replan_enabled else "OFF"
        self._append_log_line(f"[0402] target detection trigger toggled -> {state_text}")

    def _on_forced_command_replan_toggled(self, enabled: bool) -> None:
        self._forced_command_replan_enabled = bool(enabled)
        self._sync_replan_management_toggle("forced_command", self._forced_command_replan_enabled)
        self._persist_replan_toggle("forced_command", self._forced_command_replan_enabled)
        if not self._forced_command_replan_enabled:
            self._forced_availability_override = {}
            try:
                self._apply_forced_availability(stage="0802-toggle")
            except Exception:
                pass
        state_text = "ON" if self._forced_command_replan_enabled else "OFF"
        self._append_log_line(f"[0802] forced-command trigger toggled -> {state_text}")

    def _on_rtb_replan_toggled(self, enabled: bool) -> None:
        self._rtb_replan_enabled = bool(enabled)
        self._sync_replan_management_toggle("rtb", self._rtb_replan_enabled)
        self._persist_replan_toggle("rtb", self._rtb_replan_enabled)
        if not self._rtb_replan_enabled:
            self._rtb_availability_override = {}
            try:
                self._apply_forced_availability(stage="rtb-toggle")
            except Exception:
                pass
        state_text = "ON" if self._rtb_replan_enabled else "OFF"
        self._append_log_line(f"[RTB] monitoring trigger toggled -> {state_text}")

    def _on_dl_risk_replan_toggled(self, enabled: bool) -> None:
        self._dl_replan_user_enabled = bool(enabled)
        self._sync_replan_management_toggle("dl_risk", self._dl_replan_user_enabled)
        self._persist_replan_toggle("dl_risk", self._dl_replan_user_enabled)
        self._dl_replan_enabled = bool(self._dl_enabled and self._dl_replan_user_enabled)
        self._update_dl_visual_panel(replan_enabled=self._dl_replan_enabled)
        state_text = "ON" if self._dl_replan_enabled else "OFF"
        self._append_log_line(f"[DL] risk replan trigger toggled -> {state_text}")

    def _update_dl_visual_panel(self, **kwargs) -> None:
        tab = getattr(self, "_dl_risk_tab", None)
        if tab is None or not hasattr(tab, "update_dl_panel"):
            return
        try:
            tab.update_dl_panel(**kwargs)
        except Exception:
            pass

    def _env_true(self, key: str) -> bool:
        return os.getenv(key, "").strip().lower() in {"1", "true", "yes", "y", "on"}

    def _init_dl_inference(self) -> None:
        self._dl_visual_enabled = not self._env_true("MSM_DNN_VIS_DISABLE")
        self._dl_enabled = (
            self._env_true("MSM_DNN_ENABLE")
            or self._env_true("MSM_DNN_DEBUG")
            or self._dl_visual_enabled
        )
        self._dl_debug = self._env_true("MSM_DNN_DEBUG")
        if not hasattr(self, "_dl_replan_user_enabled"):
            self._dl_replan_user_enabled = get_replan_toggle(
                "dl_risk",
                self._env_true("MSM_DNN_REPLAN_ENABLE") and not self._env_true("MSM_DNN_REPLAN_DISABLE"),
            )
        self._dl_replan_enabled = bool(self._dl_enabled and self._dl_replan_user_enabled)
        self._update_dl_visual_panel(
            status="DISABLED",
            enabled=self._dl_enabled,
            replan_enabled=self._dl_replan_enabled,
            buffer_len=0,
            min_buffer=0,
            base_ready=False,
        )
        if not self._dl_enabled:
            return
        if RealTimeInferrer is None:
            self._append_log_line("[DL] RealTimeInferrer import failed (torch missing?)")
            self._update_dl_visual_panel(
                status="UNAVAILABLE",
                enabled=True,
                replan_enabled=False,
            )
            return

        model_dir = Path(__file__).resolve().parent / "models" / "checkpoints"
        context_path = str(model_dir / "mission_context.pt")
        model_path = str(model_dir / "main_model.pth")
        try:
            self._dl_inferrer = RealTimeInferrer(context_path, model_path)
            self._append_log_line("[DL] DNN inferrer initialized")
            min_buffer = 0
            base_ready = False
            try:
                min_buffer = int(getattr(self._dl_inferrer, "min_buffer_size", 0) or 0)
            except Exception:
                min_buffer = 0
            try:
                base_ready = getattr(self._dl_inferrer, "base_coord", None) is not None
            except Exception:
                base_ready = False
            self._update_dl_visual_panel(
                status="WARMUP",
                enabled=True,
                replan_enabled=self._dl_replan_enabled,
                buffer_len=0,
                min_buffer=min_buffer,
                base_ready=base_ready,
            )
        except Exception as exc:
            self._append_log_line(f"[DL] DNN init failed: {exc}")
            self._dl_inferrer = None
            self._update_dl_visual_panel(
                status="ERROR",
                enabled=True,
                replan_enabled=False,
            )
            return

        self._dl_timer = QTimer(self)
        self._dl_timer.setInterval(1000)
        self._dl_timer.timeout.connect(self._tick_dl_status)
        self._dl_timer.start()

    def _update_dl_inference(self, raw_body: dict | None) -> None:
        if not self._dl_enabled or self._dl_inferrer is None or not isinstance(raw_body, dict):
            return
        if self._current_mission_plan_id is None and not getattr(self._dl_inferrer, "mission_file", None):
            return
        if not self._dl_lock.acquire(blocking=False):
            return
        try:
            now = time.time()
            self._dl_last_data_ts = now
            if self._current_mission_plan_id is not None:
                try:
                    mp_path = mission_plan_json_path(self._current_mission_plan_id)
                    if mp_path:
                        self._dl_inferrer.mission_file = str(mp_path)
                except Exception:
                    pass
            mean, std = self._dl_inferrer.infer_one_step(raw_body)
            if mean:
                self._dl_last_infer_ts = now
                risky_indices: list[int] = []
                if self._dl_debug:
                    mean_dbg = [round(float(v), 3) for v in mean]
                    std_dbg = [round(float(v), 3) for v in (std or [])]
                    self._append_log_line(f"[DL] risk mean={mean_dbg} std={std_dbg}")
                if evaluate_risk_thresholds:
                    risky_indices = evaluate_risk_thresholds(mean, std)
                    if self._dl_debug:
                        self._append_log_line(f"[DL] risk indices={risky_indices}")
                    if risky_indices:
                        self._maybe_trigger_dl_replan(mean, risky_indices, raw_body)
                aircraft_ids: list[int] = []
                try:
                    agents = sorted(
                        raw_body.get("agentStateList", []),
                        key=lambda x: x.get("aircraftID", 0),
                    )
                    for agent in agents[:6]:
                        aid = agent.get("aircraftID")
                        if aid is None:
                            continue
                        aircraft_ids.append(int(aid))
                except Exception:
                    aircraft_ids = []
                if not aircraft_ids:
                    aircraft_ids = [1, 2, 3, 4, 5, 6]
                msg_ts = None
                try:
                    msg_ts = int(raw_body.get("timestamp"))
                except Exception:
                    msg_ts = None
                self._update_dl_visual_panel(
                    status=self._dl_status or "RUNNING",
                    enabled=self._dl_enabled,
                    replan_enabled=self._dl_replan_enabled,
                    mean=mean,
                    std=std or [],
                    risky_indices=risky_indices,
                    aircraft_ids=aircraft_ids,
                    timestamp_ms=msg_ts,
                )
        except Exception as exc:
            self._append_log_line(f"[DL] inference error: {exc}")
        finally:
            self._dl_lock.release()

    def _maybe_trigger_dl_replan(self, mean, risky_indices, raw_body: dict) -> None:
        if not self._dl_replan_enabled:
            if self._dl_debug:
                self._append_log_line("[DL] replan disabled")
            return
        if self._system_mode_code not in (3, 4):
            if self._dl_debug:
                self._append_log_line(
                    f"[DL] replan skipped: mode={self._system_mode_code} (need 3/4)"
                )
            return

        now = time.time()
        cooldown = float(get_dl_risk_settings().get("cooldown_sec", 10.0))
        if self._dl_replan_last_ts and (now - self._dl_replan_last_ts) < cooldown:
            if self._dl_debug:
                elapsed = now - self._dl_replan_last_ts
                self._append_log_line(
                    f"[DL] replan cooldown active ({elapsed:.1f}s/{cooldown}s)"
                )
            return

        risk_score = 0.0
        try:
            risk_score = float(max(mean)) if mean else 0.0
        except Exception:
            risk_score = 0.0

        risky_aircraft_ids: list[int] = []
        try:
            agents = sorted(
                raw_body.get("agentStateList", []),
                key=lambda x: x.get("aircraftID", 0),
            )
            for idx in risky_indices:
                if idx < len(agents):
                    try:
                        risky_aircraft_ids.append(int(agents[idx].get("aircraftID")))
                    except Exception:
                        pass
        except Exception:
            pass

        coord = getattr(self, "_prior_mission_coord", None)
        if coord is None:
            return
        payloads, logs = coord.on_risk_update(
            risk_score,
            system_mode=self._system_mode_code,
            current_mission_plan_id=self._current_mission_plan_id,
            risky_aircraft_ids=risky_aircraft_ids,
        )
        for line in logs:
            self._append_log_line(line)
        for body in payloads:
            if isinstance(body, dict):
                self._send_0902(body)
        if payloads:
            self._dl_replan_last_ts = now

    def _tick_dl_status(self) -> None:
        if not self._dl_enabled or self._dl_inferrer is None:
            return
        now = time.time()
        hb_sec = float(os.getenv("MSM_DNN_HEARTBEAT_SEC", "5"))
        no_data_sec = float(os.getenv("MSM_DNN_NO_DATA_SEC", "10"))
        status_log_sec = float(os.getenv("MSM_DNN_STATUS_LOG_SEC", "5"))

        buffer_len = 0
        min_buffer = 0
        base_ready = True
        if hasattr(self._dl_inferrer, "buffer"):
            try:
                buffer_len = len(self._dl_inferrer.buffer)
            except Exception:
                buffer_len = 0
        if hasattr(self._dl_inferrer, "min_buffer_size"):
            try:
                min_buffer = int(self._dl_inferrer.min_buffer_size)
            except Exception:
                min_buffer = 0
        if hasattr(self._dl_inferrer, "base_coord"):
            base_ready = self._dl_inferrer.base_coord is not None

        if self._dl_last_data_ts == 0.0 or (now - self._dl_last_data_ts) > no_data_sec:
            status = "NO_DATA"
        elif not base_ready or (min_buffer and buffer_len < min_buffer):
            status = "WARMUP"
        elif self._dl_last_infer_ts and (now - self._dl_last_infer_ts) <= hb_sec:
            status = "RUNNING"
        else:
            status = "STALE"

        if status != self._dl_status:
            self._dl_status = status
            self._append_log_line(f"[DL] realtime status -> {status}")
            self._dl_last_status_log_ts = now
        elif self._dl_debug and (now - self._dl_last_status_log_ts) >= status_log_sec:
            data_age = now - self._dl_last_data_ts if self._dl_last_data_ts else -1
            infer_age = now - self._dl_last_infer_ts if self._dl_last_infer_ts else -1
            self._append_log_line(
                "[DL] heartbeat: "
                f"status={status}, last_data={data_age:.1f}s, last_infer={infer_age:.1f}s, "
                f"buffer={buffer_len}/{min_buffer}, base_ready={base_ready}"
            )
            self._dl_last_status_log_ts = now

        data_age = None
        infer_age = None
        try:
            if self._dl_last_data_ts:
                data_age = max(0.0, now - self._dl_last_data_ts)
        except Exception:
            data_age = None
        try:
            if self._dl_last_infer_ts:
                infer_age = max(0.0, now - self._dl_last_infer_ts)
        except Exception:
            infer_age = None
        self._update_dl_visual_panel(
            status=self._dl_status,
            enabled=self._dl_enabled,
            replan_enabled=self._dl_replan_enabled,
            data_age_sec=data_age,
            infer_age_sec=infer_age,
            buffer_len=buffer_len,
            min_buffer=min_buffer,
            base_ready=base_ready,
        )

    def _on_tab_changed(self, index: int) -> None:
        viz = getattr(self, "_viz_tab", None)
        try:
            if viz is not None and hasattr(viz, "set_ui_updates_enabled"):
                viz.set_ui_updates_enabled(index == getattr(self, "_viz_tab_index", -1))
            dl_tab = getattr(self, "_dl_risk_tab", None)
            if dl_tab is not None and hasattr(dl_tab, "set_ui_updates_enabled"):
                dl_tab.set_ui_updates_enabled(index == getattr(self, "_dl_risk_tab_index", -1))
            quality_tab = getattr(self, "_quality_tab", None)
            if quality_tab is not None and hasattr(quality_tab, "set_ui_updates_enabled"):
                quality_tab.set_ui_updates_enabled(index == getattr(self, "_quality_tab_index", -1))
            turn_tab = getattr(self, "_turn_radius_tab", None)
            if turn_tab is not None and hasattr(turn_tab, "set_ui_updates_enabled"):
                turn_tab.set_ui_updates_enabled(index == getattr(self, "_turn_radius_tab_index", -1))
        except Exception:
            pass

    def _update_0702_status(self, *, status: str, detail: str | None = None) -> None:
        viz = getattr(self, "_viz_tab", None)
        if viz is None or not hasattr(viz, "update_0702_status"):
            return
        try:
            viz.update_0702_status(status=status, detail=detail)
        except Exception:
            pass

    def _install_power_gate_hooks(self) -> None:
        """Power OFF 시 TX만 막음. RX는 항상 통과."""
        try:
            tab = self._tab
            tbl_tx = getattr(tab, "tbl_tx", None)

            if tbl_tx is not None:
                class _PG(QObject):
                    def __init__(self, host):
                        super().__init__(host)
                        self.host = host

                    def eventFilter(self, obj, ev):
                        if not self.host._power_on and ev.type() in (
                            QEvent.MouseButtonPress,
                            QEvent.MouseButtonRelease,
                            QEvent.MouseButtonDblClick,
                            QEvent.KeyPress,
                            QEvent.KeyRelease,
                        ):
                            return True
                        return False

                self._pg_filter_tx = _PG(self)
                tbl_tx.installEventFilter(self._pg_filter_tx)

            if hasattr(tab, "_on_tx_button_clicked"):
                self._orig_tx_click = tab._on_tx_button_clicked

                def _wrapped_tx_click(row):
                    if not self._power_on:
                        self._append_log_line("[BLOCK] Power OFF 시 TX 버튼 차단")
                        return
                    return self._orig_tx_click(row)

                tab._on_tx_button_clicked = _wrapped_tx_click
        except Exception:
            pass

    def _apply_power_state(self) -> None:
        on = bool(self._power_on)
        try:
            self._update_tx_table_enabled(on)
            self._update_rx_table_enabled(True)
            if not on:
                self._stop_all_periodic()
                self._stop_0501_sender()
        except Exception:
            pass

    def _update_0501_state(self, mode_code: int | None) -> None:
        viz = getattr(self, "_viz_tab", None)
        if viz is not None and hasattr(viz, "set_system_mode"):
            try:
                viz.set_system_mode(mode_code)
            except Exception:
                pass
        if mode_code == 3:
            self._start_0501_sender()
        else:
            self._stop_0501_sender()

    def _start_0501_sender(self) -> None:
        if getattr(self, "_send_0501_timer", None) is None:
            self._send_0501_timer = QTimer(self)
            self._send_0501_timer.setInterval(200)
            self._send_0501_timer.timeout.connect(self._send_0501_tick)
        if not self._send_0501_timer.isActive():
            self._send_0501_timer.start()
            self._record_0501_diag("sender_started")
            self._send_0501_tick()

    def _stop_0501_sender(self) -> None:
        timer = getattr(self, "_send_0501_timer", None)
        if timer is not None and timer.isActive():
            timer.stop()
            self._record_0501_diag("sender_stopped")

    def _init_0501_watchdog(self) -> None:
        self._send_0501_watchdog_timer = QTimer(self)
        self._send_0501_watchdog_timer.setInterval(1000)
        self._send_0501_watchdog_timer.timeout.connect(self._check_0501_health)
        self._send_0501_watchdog_timer.start()

    def _capture_0501_context(self) -> dict[str, object]:
        viz = getattr(self, "_viz_tab", None)
        view = getattr(viz, "_mission_view", None) if viz is not None else None
        if isinstance(view, dict):
            uav_entries = view.get("uav_entries") or []
            plan_id = view.get("mission_plan_id")
            input_ids = [entry.get("input_id") for entry in view.get("input_missions") or [] if isinstance(entry, dict)]
        else:
            uav_entries = []
            plan_id = None
            input_ids = []
        try:
            latest_status_ts = viz.get_latest_status_timestamp_ms() if viz is not None else None
        except Exception:
            latest_status_ts = None
        timer = getattr(self, "_send_0501_timer", None)
        return {
            "power_on": bool(getattr(self, "_power_on", True)),
            "mode_code": self._system_mode_code,
            "mission_plan_id": plan_id,
            "has_view": isinstance(view, dict) and bool(view),
            "uav_entry_count": len(uav_entries),
            "input_ids": input_ids,
            "latest_status_timestamp_ms": latest_status_ts,
            "sender_active": bool(timer is not None and timer.isActive()),
        }

    def _record_0501_diag(self, event: str, **detail: object) -> None:
        payload = {
            "timestamp": _now_ms_since_2000(),
            "event": str(event),
            "detail": {
                **self._capture_0501_context(),
                **detail,
            },
        }
        try:
            base = db_paths.get_db_subpath("DSS_Internal", "monitoring_diagnostics")
            base.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
            suffix = f"{int(time.time_ns() % 1_000_000_000):09d}"
            path = base / f"msm_0501_{stamp}_{suffix}.json"
            path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            latest = base / "latest_msm_0501_diag.json"
            latest.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception:
            pass

    def _check_0501_health(self) -> None:
        if not getattr(self, "_power_on", True):
            self._last_0501_watchdog_state = "power_off"
            return
        if self._system_mode_code != 3:
            self._last_0501_watchdog_state = f"mode_{self._system_mode_code}"
            return
        timer = getattr(self, "_send_0501_timer", None)
        if timer is None or not timer.isActive():
            if self._last_0501_watchdog_state != "inactive":
                self._record_0501_diag("watchdog_sender_inactive")
                self._last_0501_watchdog_state = "inactive"
            self._start_0501_sender()
            return
        last_ok = max(
            float(getattr(self, "_last_0501_payload_monotonic", 0.0) or 0.0),
            float(getattr(self, "_last_0501_send_monotonic", 0.0) or 0.0),
        )
        if last_ok > 0.0:
            age_sec = time.monotonic() - last_ok
            if age_sec > 1.0:
                if self._last_0501_watchdog_state != "stalled":
                    self._record_0501_diag("watchdog_sender_stalled", age_sec=round(age_sec, 3))
                    self._last_0501_watchdog_state = "stalled"
                self._send_0501_tick()
                return
        if self._last_0501_watchdog_state != "ok":
            self._record_0501_diag("watchdog_sender_recovered")
            self._last_0501_watchdog_state = "ok"

    def _send_0501_tick(self) -> None:
        self._last_0501_attempt_monotonic = time.monotonic()
        try:
            if not getattr(self, "_power_on", True):
                return
            viz = getattr(self, "_viz_tab", None)
            if viz is None or not hasattr(viz, "build_0501_payload"):
                self._record_0501_diag("missing_visualization_tab")
                return
            ts_for_0501 = None
            try:
                if hasattr(viz, "get_latest_status_timestamp_ms"):
                    ts_for_0501 = viz.get_latest_status_timestamp_ms()
            except Exception:
                ts_for_0501 = None
            if ts_for_0501 is None:
                ts_for_0501 = _now_ms_since_2000()
            payload = viz.build_0501_payload(timestamp_ms=ts_for_0501, source="MSM")
            if not payload:
                self._record_0501_diag("empty_0501_payload")
                return
            self._last_0501_payload_monotonic = time.monotonic()
            try:
                from push_center import push_message
            except Exception as exc:
                self._append_log_line(f"[0501] push import failed: {exc}")
                self._record_0501_diag("push_import_failed", error=str(exc))
                return
            row = self._find_tx_row("0501")
            tab = getattr(self, "_tab", None)
            on_done = None
            if row >= 0 and tab is not None and hasattr(tab, "_mark_single_sent"):
                on_done = (lambda mid, raw: tab._mark_single_sent(row, mid, raw))
            ok = push_message("0501", NodeMessenger, body_dict=payload, on_done=on_done)
            if not ok:
                self._append_log_line("[0501] send failed")
                self._record_0501_diag("send_failed")
                return
            self._last_0501_send_monotonic = time.monotonic()
        except Exception as exc:
            self._append_log_line(f"[0501] send error: {exc}")
            self._record_0501_diag(
                "send_exception",
                error=str(exc),
                traceback=traceback.format_exc(),
            )

    # --- DB root sync ---
    def _init_db_root_sync(self) -> None:
        self._db_root = None
        self._refresh_db_root(log_first=True)
        self._db_root_timer = QTimer(self)
        self._db_root_timer.setInterval(1000)
        self._db_root_timer.timeout.connect(self._refresh_db_root)
        self._db_root_timer.start()

    def _refresh_db_root(self, log_first: bool = False) -> None:
        try:
            root = db_paths.get_active_db_root()
        except Exception as exc:
            self._append_log_line(f"[PATH] DB root check failed: {exc}")
            return
        root_str = str(root)
        if log_first or root_str != getattr(self, "_db_root", None):
            self._db_root = root_str
            self._append_log_line(f"[PATH] DB root -> {root_str}")

    # --- 0102 self-check auto send ---
    def _init_0102_autostart(self) -> None:
        # Delay to let UI/bus/self-check settle before auto send/enable.
        QTimer.singleShot(2000, self._start_0102_autostart)

    def _start_0102_autostart(self, _retry: int = 0) -> None:
        if not self._power_on:
            return
        if not self._ensure_0102_periodic():
            if _retry < 10:
                QTimer.singleShot(300, lambda: self._start_0102_autostart(_retry + 1))
            return
        self._send_self_check_0102()

    def _ensure_0102_periodic(self) -> bool:
        tab = getattr(self, "_tab", None)
        if tab is None:
            return False
        if "0102" in getattr(tab, "periodic_timers", {}):
            return True
        row = self._find_tx_row("0102")
        if row < 0:
            self._append_log_line("[0102] TX table row not found")
            return False
        try:
            if hasattr(tab, "send_tx_row"):
                return bool(tab.send_tx_row(row, interactive=False))
            tab._on_tx_button_clicked(row)
            return True
        except Exception as exc:
            self._append_log_line(f"[0102] Auto-start failed: {exc}")
            return False

    def _send_self_check_0102(self) -> bool:
        try:
            from push_center import push_message
        except Exception as exc:
            self._append_log_line(f"[0102] push import failed: {exc}")
            return False
        body = {
            "timestamp": _now_ms_since_2000(),
            "status": 1,
            "source": "MSM",
        }
        try:
            ok = push_message("0102", NodeMessenger, body_dict=body)
            if ok:
                self._append_log_line("[0102] status=1 sent")
            else:
                self._append_log_line("[0102] send failed")
            return bool(ok)
        except Exception as exc:
            self._append_log_line(f"[0102] send error: {exc}")
            return False

    # --- 0902 auto replan (init plan mode) ---
    def _handle_initplan_transition(self, code: int) -> None:
        if code == 2:
            if not self._auto_initplan_triggered:
                self._auto_initplan_triggered = True
                self._auto_prepare_replan()
        else:
            self._auto_initplan_triggered = False

    def _auto_prepare_replan(self) -> None:
        if not getattr(self, "_power_on", True):
            return
        tab = getattr(self, "_tab", None)
        if tab is None or not hasattr(tab, "send_replan_request"):
            self._append_log_line("[0902] auto send hook missing")
            return
        self._seed_initial_availability_from_input_plan()
        mission_ids = collect_input_mission_ids()
        if not mission_ids:
            self._append_log_line("[0902] inputMissionID list empty; abort")
            return
        plan_ids = allocate_mission_plan_ids(1)
        if not plan_ids:
            self._append_log_line("[0902] missionPlanID allocation failed; abort")
            return
        context = build_replan_context(mission_ids, plan_ids)
        try:
            ok = bool(tab.send_replan_request(context, reason="초기임무재계획"))
        except Exception as exc:
            self._append_log_line(f"[0902] auto send failed: {exc}")
            return
        if ok:
            self._append_log_line("[0902] auto replan request sent (init plan mode)")
        else:
            self._append_log_line("[0902] auto replan request failed")

    def _seed_initial_availability_from_input_plan(self) -> None:
        available_ids = collect_available_aircraft_ids(None)
        if not available_ids:
            self._append_log_line("[STATUS] init availability seed skipped: InputMissionPlan aircraft list empty")
            return
        self._availability_base_ids = {int(aid) for aid in available_ids if int(aid) > 0}
        self._forced_availability_override = {}
        self._rtb_availability_override = {}
        self._availability_seen = True
        self._apply_forced_availability(stage="0201")
        summary = ", ".join(str(aid) for aid in sorted(self._availability_base_ids))
        self._append_log_line(f"[STATUS] init availability seeded from InputMissionPlan: [{summary}]")

    def _find_tx_row(self, msg_id: str) -> int:
        tab = getattr(self, "_tab", None)
        tbl = getattr(tab, "tbl_tx", None) if tab else None
        if tbl is None:
            return -1
        for r in range(tbl.rowCount()):
            item = tbl.item(r, 0)
            if item and item.text().strip() == msg_id:
                return r
        return -1

    def _update_tx_table_enabled(self, enabled: bool) -> None:
        try:
            tab = self._tab
            tbl = getattr(tab, "tbl_tx", None)
            if tbl is None:
                return
            tbl.setEnabled(enabled)
            for r in range(tbl.rowCount()):
                w = tbl.cellWidget(r, 3)
                if w is not None and hasattr(w, "setEnabled"):
                    w.setEnabled(enabled)
        except Exception:
            pass

    def _update_rx_table_enabled(self, enabled: bool) -> None:
        try:
            tab = self._tab
            tbl = getattr(tab, "tbl_rx", None)
            if tbl is None:
                return
            tbl.setEnabled(enabled)
            cols = getattr(tbl, "columnCount", lambda: 4)()
            for r in range(tbl.rowCount()):
                for c in range(cols):
                    w = tbl.cellWidget(r, c)
                    if w is not None and hasattr(w, "setEnabled"):
                        w.setEnabled(enabled)
        except Exception:
            pass

    def _stop_all_periodic(self) -> None:
        try:
            tab = self._tab
            timers = getattr(tab, "periodic_timers", {})
            for _, t in list(timers.items()):
                try:
                    t.stop()
                except Exception:
                    pass
            try:
                timers.clear()
            except Exception:
                pass
            self._append_log_line("[POWER] periodic TX 정지")
        except Exception:
            pass
        self._stop_0501_sender()

    def _on_0503_recommend(self, recommend: int, input_id: int | None = None) -> None:
        self._send_0503(recommend, input_id)

    def _send_0503(self, recommend: int, input_id: int | None = None) -> bool:
        viz = getattr(self, "_viz_tab", None)
        try:
            from push_center import push_message
        except Exception as exc:
            self._append_log_line(f"[0503] push import failed: {exc}")
            try:
                if viz is not None and hasattr(viz, "note_completion_recommendation_failed"):
                    viz.note_completion_recommendation_failed(recommend, input_id)
            except Exception:
                pass
            return False

        payload = {
            "timestamp": _now_ms_since_2000(),
            "source": "MSM",
            "systemRecommend": int(recommend),
        }
        tab = getattr(self, "_tab", None)
        row = self._find_tx_row("0503")
        on_done = None
        if row >= 0 and tab is not None and hasattr(tab, "_mark_single_sent"):
            on_done = (lambda mid, raw: tab._mark_single_sent(row, mid, raw))
        try:
            ok = push_message("0503", NodeMessenger, body_dict=payload, on_done=on_done)
            if not ok:
                self._append_log_line("[0503] send failed")
                try:
                    if viz is not None and hasattr(viz, "note_completion_recommendation_failed"):
                        viz.note_completion_recommendation_failed(recommend, input_id)
                except Exception:
                    pass
                return False
        except Exception as exc:
            self._append_log_line(f"[0503] send error: {exc}")
            try:
                if viz is not None and hasattr(viz, "note_completion_recommendation_failed"):
                    viz.note_completion_recommendation_failed(recommend, input_id)
            except Exception:
                pass
            return False
        try:
            if viz is not None and hasattr(viz, "note_completion_recommendation_sent"):
                viz.note_completion_recommendation_sent(recommend, input_id)
        except Exception:
            pass
        try:
            input_text = f", inputID={int(input_id)}" if input_id is not None else ""
        except Exception:
            input_text = ""
        self._append_log_line(f"[0503] recommend={int(recommend)}{input_text} sent")
        if int(recommend) == 3:
            self._send_0502()
        return True

    def _send_0502(self) -> None:
        try:
            from push_center import push_message
        except Exception as exc:
            self._append_log_line(f"[0502] push import failed: {exc}")
            return

        plan_id = self._current_mission_plan_id or self._current_plan_id_from_viz()
        if plan_id is not None:
            try:
                plan_id_int = int(plan_id)
            except Exception:
                plan_id_int = None
            if plan_id_int is not None and plan_id_int in self._sent_0502_plans:
                return
        else:
            plan_id_int = None

        payload = {
            "timestamp": _now_ms_since_2000(),
            "source": "MSM",
        }

        tab = getattr(self, "_tab", None)
        row = self._find_tx_row("0502")
        on_done = None
        if row >= 0 and tab is not None and hasattr(tab, "_mark_single_sent"):
            on_done = (lambda mid, raw: tab._mark_single_sent(row, mid, raw))

        try:
            ok = push_message("0502", NodeMessenger, body_dict=payload, on_done=on_done)
            if not ok:
                self._append_log_line("[0502] send failed")
                return
        except Exception as exc:
            self._append_log_line(f"[0502] send error: {exc}")
            return

        if plan_id_int is not None:
            self._sent_0502_plans.add(plan_id_int)
        self._append_log_line("[0502] mission end request sent")

    def _send_0504(self, payload: dict) -> None:
        try:
            from push_center import push_message
        except Exception as exc:
            self._append_log_line(f"[0504] push import failed: {exc}")
            return

        body = dict(payload or {})
        body.setdefault("timestamp", _now_ms_since_2000())
        body.setdefault("source", "MSM")
        try:
            body["aircraftID"] = int(body.get("aircraftID") or 0)
        except Exception:
            body["aircraftID"] = 0
        try:
            body["fuelLevel"] = int(body.get("fuelLevel") or 0)
        except Exception:
            body["fuelLevel"] = 0

        tab = getattr(self, "_tab", None)
        row = self._find_tx_row("0504")
        on_done = None
        if row >= 0 and tab is not None and hasattr(tab, "_mark_single_sent"):
            on_done = (lambda mid, raw: tab._mark_single_sent(row, mid, raw))
        try:
            ok = push_message("0504", NodeMessenger, body_dict=body, on_done=on_done)
            if not ok:
                self._append_log_line("[0504] send failed")
        except Exception as exc:
            self._append_log_line(f"[0504] send error: {exc}")

    def _send_0902(self, payload: dict) -> None:
        try:
            from push_center import push_message
        except Exception as exc:
            self._append_log_line(f"[0902] push import failed: {exc}")
            return

        tab = getattr(self, "_tab", None)
        row = self._find_tx_row("0902")
        on_done = None
        if row >= 0 and tab is not None and hasattr(tab, "_mark_single_sent"):
            on_done = (lambda mid, raw: tab._mark_single_sent(row, mid, raw))

        try:
            ok = push_message("0902", NodeMessenger, body_dict=payload, on_done=on_done)
        except Exception as exc:
            self._append_log_line(f"[0902] send error: {exc}")
            return
        if not ok:
            self._append_log_line("[0902] send failed")
            return

        level = payload.get("replanLevel")
        reason = payload.get("replanRequest") or payload.get("replanReason")
        options = payload.get("pendingOptionList") or payload.get("optionList") or []
        plan_ids: list[int] = []
        for item in options:
            if not isinstance(item, dict):
                continue
            try:
                plan_id = int(item.get("missionPlanID"))
            except Exception:
                continue
            plan_ids.append(plan_id)
            self._replan_option_meta_by_plan_id[plan_id] = {
                "optionName": str(item.get("optionName") or ""),
                "replanLevel": payload.get("replanLevel"),
                "replanRequest": payload.get("replanRequest") or payload.get("replanReason"),
                "replanDetail": dict(payload.get("replanDetail") or {})
                if isinstance(payload.get("replanDetail"), dict)
                else {},
                "timestamp": payload.get("timestamp"),
            }
        if not plan_ids:
            for item in payload.get("missionPlanIDList") or []:
                if isinstance(item, dict):
                    value = item.get("missionPlanID")
                else:
                    value = item
                try:
                    plan_id = int(value)
                except Exception:
                    continue
                plan_ids.append(plan_id)
                self._replan_option_meta_by_plan_id[plan_id] = {
                    "optionName": "",
                    "replanLevel": payload.get("replanLevel"),
                    "replanRequest": payload.get("replanRequest") or payload.get("replanReason"),
                    "replanDetail": dict(payload.get("replanDetail") or {})
                    if isinstance(payload.get("replanDetail"), dict)
                    else {},
                    "timestamp": payload.get("timestamp"),
                }
        plan_summary = ", ".join(str(pid) for pid in plan_ids) if plan_ids else "-"
        self._append_log_line(
            f"[0902] replan request sent (level={level}, reason={reason}, planIds={plan_summary})"
        )

    @staticmethod
    def _format_aircraft_notice_prefix(aircraft_id: int | None) -> str:
        try:
            aid = int(aircraft_id) if aircraft_id is not None else 0
        except Exception:
            aid = 0
        if 1 <= aid <= 3:
            return f"유인기 {aid}번"
        if 4 <= aid <= 6:
            return f"무인기 {aid - 3}번"
        if aid > 0:
            return f"항공기 {aid}번"
        return "미상 항공기"

    def _format_datalink_notice_text(
        self,
        *,
        manned_aircraft_id: int | None,
        unmanned_aircraft_id: int | None,
    ) -> str:
        try:
            manned_id = int(manned_aircraft_id) if manned_aircraft_id is not None else 0
        except Exception:
            manned_id = 0
        try:
            unmanned_id = int(unmanned_aircraft_id) if unmanned_aircraft_id is not None else 0
        except Exception:
            unmanned_id = 0
        if 1 <= manned_id <= 3:
            manned_label = f"유인기 {manned_id}"
        else:
            manned_label = self._format_aircraft_notice_prefix(manned_aircraft_id)
        if 4 <= unmanned_id <= 6:
            unmanned_label = f"무인기 {unmanned_id - 3}"
        else:
            unmanned_label = self._format_aircraft_notice_prefix(unmanned_aircraft_id)
        return f"{manned_label} - {unmanned_label} 통신두절"

    def _sync_0401_fault_notices(
        self,
        agent_states: list[dict[str, object]] | None,
    ) -> None:
        active_keys: set[tuple[int, str]] = set()
        for state in agent_states or []:
            if not isinstance(state, dict):
                continue
            try:
                aircraft_id = int(state.get("aircraft_id"))
            except Exception:
                continue
            if aircraft_id < 4 or aircraft_id > 6:
                continue
            if not self._is_aircraft_in_current_plan(aircraft_id):
                continue

            try:
                health = int(state.get("health")) if state.get("health") is not None else None
            except Exception:
                health = None
            try:
                payload_health = (
                    int(state.get("payload_health"))
                    if state.get("payload_health") is not None
                    else None
                )
            except Exception:
                payload_health = None

            if health == 2:
                notice_key = (aircraft_id, "aircraft_fault")
                active_keys.add(notice_key)
                if notice_key not in self._active_0401_notice_keys:
                    self._send_0001_notice(
                        f"{self._format_aircraft_notice_prefix(aircraft_id)} 고장"
                    )
            elif payload_health == 2:
                notice_key = (aircraft_id, "payload_fault")
                active_keys.add(notice_key)
                if notice_key not in self._active_0401_notice_keys:
                    self._send_0001_notice(
                        f"{self._format_aircraft_notice_prefix(aircraft_id)} 임무장비 고장"
                    )

            pair_statuses = state.get("datalink_connected_by_manned")
            pair_notice_sent = False
            if isinstance(pair_statuses, dict):
                for raw_manned_id, connected in sorted(pair_statuses.items()):
                    try:
                        manned_aircraft_id = int(raw_manned_id)
                    except Exception:
                        continue
                    if connected is not False:
                        continue
                    notice_key = (aircraft_id, f"communication_loss_{manned_aircraft_id}")
                    active_keys.add(notice_key)
                    if notice_key not in self._active_0401_notice_keys:
                        self._send_0001_notice(
                            self._format_datalink_notice_text(
                                manned_aircraft_id=manned_aircraft_id,
                                unmanned_aircraft_id=aircraft_id,
                            )
                        )
                    pair_notice_sent = True

            if not pair_notice_sent and state.get("datalink_connected") is False:
                notice_key = (aircraft_id, "communication_loss")
                active_keys.add(notice_key)
                if notice_key not in self._active_0401_notice_keys:
                    self._send_0001_notice(
                        f"{self._format_aircraft_notice_prefix(aircraft_id)} 통신두절"
                    )

        self._active_0401_notice_keys = active_keys

    def _plan_option_meta(self, mission_plan_id: int) -> dict[str, object] | None:
        meta = self._replan_option_meta_by_plan_id.get(int(mission_plan_id))
        return dict(meta) if isinstance(meta, dict) else None

    def _apply_attack_exclusion_ignore(self, mission_plan_id: int) -> bool:
        meta = self._plan_option_meta(mission_plan_id)
        if not isinstance(meta, dict):
            return False
        option_name = "".join(str(meta.get("optionName") or "").split()).lower()
        if option_name != "공격배제":
            return False
        detail = meta.get("replanDetail")
        if not isinstance(detail, dict):
            return True
        try:
            target_id = int(detail.get("targetID") or 0)
        except Exception:
            target_id = 0
        if target_id <= 0:
            return True
        try:
            watcher_id = detail.get("watcherID")
            mark_targets_as_ignored(
                [
                    {
                        "targetID": int(target_id),
                        "watcherID": int(watcher_id) if watcher_id is not None else None,
                    }
                ]
            )
            self._append_log_line(
                f"[0702] 공격 배제 선택 -> targetID={target_id} isIgnored=1 반영"
            )
        except Exception as exc:
            self._append_log_line(
                f"[0702] 공격 배제 ignore 반영 실패: {exc}"
            )
        return True

    def _commit_pending_attack_slot(self, mission_plan_id: int) -> None:
        try:
            aircraft_id = commit_pending_manned_assignment(mission_plan_id)
        except Exception as exc:
            self._append_log_line(f"[0702] 공격 슬롯 점유 반영 실패: {exc}")
            return
        if aircraft_id is not None:
            self._append_log_line(
                f"[0702] 공격 임무 확정 -> aircraftID={aircraft_id} 슬롯 점유"
            )

    def _on_notice(self, contents: str) -> None:
        self._send_0001_notice(contents)

    def _current_plan_id_from_viz(self) -> int | None:
        viz = getattr(self, "_viz_tab", None)
        view = getattr(viz, "_mission_view", None) if viz is not None else None
        if not isinstance(view, dict):
            return None
        plan_id = view.get("mission_plan_id")
        try:
            return int(plan_id) if plan_id is not None else None
        except Exception:
            return None

    def _current_plan_aircraft_ids(self) -> set[int]:
        ids: set[int] = set()
        plan_id = self._current_mission_plan_id
        if plan_id is None:
            plan_id = self._current_plan_id_from_viz()
        if plan_id is not None:
            try:
                path = mission_plan_json_path(plan_id)
            except Exception:
                path = None
            if path is not None and path.exists():
                try:
                    payload = json.loads(path.read_text(encoding="utf-8"))
                except Exception:
                    payload = {}
                for entry in payload.get("aircraftList") or []:
                    if not isinstance(entry, dict):
                        continue
                    try:
                        aid = int(entry.get("aircraftID"))
                    except Exception:
                        continue
                    ids.add(aid)
        viz = getattr(self, "_viz_tab", None)
        view = getattr(viz, "_mission_view", None) if viz is not None else None
        if isinstance(view, dict):
            for entry in view.get("uav_entries") or []:
                if not isinstance(entry, dict):
                    continue
                try:
                    aid = int(entry.get("aircraft_id"))
                except Exception:
                    continue
                ids.add(aid)
        return ids

    def _effective_available_ids(self) -> set[int]:
        base = set(int(v) for v in (self._availability_base_ids or set()))
        overrides = self._availability_overrides()
        if not base and not overrides:
            return set()
        effective = set(base)
        for aid, forced_available in overrides.items():
            try:
                aid_int = int(aid)
            except Exception:
                continue
            if bool(forced_available):
                effective.add(aid_int)
            else:
                effective.discard(aid_int)
        return effective

    def _bootstrap_availability_from_agent_states(
        self,
        agent_states: list[dict[str, object]] | None,
    ) -> None:
        if self._availability_base_ids:
            return
        derived = {
            int(aid)
            for aid in (collect_available_aircraft_ids(None) or [])
            if int(aid) > 0
        }
        for state in agent_states or []:
            if not isinstance(state, dict):
                continue
            try:
                aircraft_id = int(state.get("aircraft_id"))
            except Exception:
                continue
            if aircraft_id <= 0:
                continue
            derived.add(aircraft_id)
        if not derived:
            return
        self._availability_base_ids = set(derived)

    def _availability_state_for(self, aircraft_id: int) -> bool | None:
        """Return True/False when availability is known; None when unknown."""
        overrides = self._availability_overrides()
        if aircraft_id in overrides:
            return bool(overrides[aircraft_id])
        base = set(int(v) for v in (self._availability_base_ids or set()))
        if base:
            return aircraft_id in base
        if getattr(self, "_availability_seen", False):
            return False
        return None

    def _is_aircraft_unavailable(self, aircraft_id: int) -> bool:
        try:
            aid = int(aircraft_id)
        except Exception:
            return True
        plan_aircraft = self._current_plan_aircraft_ids()
        if plan_aircraft and aid not in plan_aircraft:
            return True
        availability = self._availability_state_for(aid)
        return availability is False

    def _is_aircraft_in_current_plan(self, aircraft_id: int) -> bool:
        try:
            aid = int(aircraft_id)
        except Exception:
            return False
        plan_aircraft = self._current_plan_aircraft_ids()
        if not plan_aircraft:
            return True
        return aid in plan_aircraft

    def _forced_hold_availability(self, aircraft_id: int) -> bool | None:
        """Return current availability state for delayed-hold replan decisions."""
        if self._is_aircraft_unavailable(aircraft_id):
            return False
        return True

    def _availability_overrides(self) -> dict[int, bool]:
        merged: dict[int, bool] = {}
        for source in (
            dict(self._forced_availability_override or {}),
            dict(self._rtb_availability_override or {}),
        ):
            for aid, value in source.items():
                try:
                    aid_int = int(aid)
                except Exception:
                    continue
                merged[aid_int] = bool(value)
        return merged

    def _send_0001_notice(self, contents: str) -> None:
        text = str(contents or "").strip()
        if not text:
            return

        try:
            from push_center import push_message
        except Exception as exc:
            self._append_log_line(f"[0001] push import failed: {exc}")
            return

        payload = {
            "timestamp": _now_ms_since_2000(),
            "source": "MSM",
            "contents": text,
        }

        tab = getattr(self, "_tab", None)
        row = self._find_tx_row("0001")
        on_done = None
        if row >= 0 and tab is not None and hasattr(tab, "_mark_single_sent"):
            on_done = (lambda mid, raw: tab._mark_single_sent(row, mid, raw))

        try:
            ok = push_message("0001", NodeMessenger, body_dict=payload, on_done=on_done)
            if ok:
                self._append_log_line(f"[0001] notice sent: {text}")
            else:
                self._append_log_line("[0001] send failed")
        except Exception as exc:
            self._append_log_line(f"[0001] send error: {exc}")

    def _on_mode_slider_changed(self, val: int) -> None:
        source = getattr(self, "_mode_update_source", None)
        manual_change = source != "external"
        try:
            self.mode_now.setText(_MODE_LABELS[int(val)])
        except Exception:
            pass
        self._power_on = True
        self._apply_power_state()
        if manual_change:
            prev = self._system_mode_code
            if (not self._mode_manual_override) or (prev != val):
                self._append_log_line(f"[MODE] manual override -> code={val}")
            self._mode_manual_override = True
            self._last_ignored_external_mode_code = None
        self._system_mode_code = val
        self._update_0501_state(val)
        # Slider should drive the same transitions as external 0101.
        try:
            self._handle_initplan_transition(val)
        except Exception:
            pass

    def _set_mode_slider_by_text(self, text: str) -> None:
        norm = "".join(str(text).split()).lower()
        mapping = {
            "전원off": 0,
            "off": 0,
            "poweroff": 0,
            "전원on": 0,
            "on": 0,
            "poweron": 0,
            "0": 0,
            "초기화": 0,
            "초기화모드": 0,
            "초기화mode": 0,
            "1": 1,
            "대기모드": 1,
            "대기": 1,
            "standby": 1,
            "2": 2,
            "초기임무계획": 2,
            "초기임무계획모드": 2,
            "initplan": 2,
            "initial": 2,
            "3": 3,
            "임무수행": 3,
            "임무수행모드": 3,
            "execution": 3,
            "execute": 3,
            "mission": 3,
            "missionexecution": 3,
        }
        val = mapping.get(norm, 1)
        try:
            if self.mode_slider.value() != val:
                self.mode_slider.blockSignals(True)
                self.mode_slider.setValue(val)
                self.mode_slider.blockSignals(False)
            self.mode_now.setText(_MODE_LABELS[val])
        except Exception:
            pass
        self._power_on = True
        self._apply_power_state()

    def _install_0101_mode_listener(self) -> None:
        """Receive 0101 systemMode and reflect it in the slider."""
        class _Rx0101:
            def __init__(self, host):
                self.host = host

            def mark_received(self, msg_id: str, raw: bytes | None = None):
                try:
                    self.host._on_rx_0101(raw)
                except Exception:
                    pass

        try:
            self._rx0101 = _Rx0101(self)
            register_listener("0101", self._rx0101)
        except Exception as exc:
            self._append_log_line(f"[0101] 리스너 등록 실패: {exc}")

    def _install_0903_listener(self) -> None:
        def _rx_0903(_msg_id: str, payload: object | None):
            try:
                self._on_rx_0903(payload)
            except Exception:
                pass

        try:
            self._rx0903_handler = _rx_0903
            register_listener("0903", self._rx0903_handler)
        except Exception as exc:
            self._append_log_line(f"[0903] 리스너 등록 실패: {exc}")

    def _install_0702_listener(self) -> None:
        def _rx_0702(_msg_id: str, payload: object | None):
            try:
                self._on_rx_0702(payload)
            except Exception:
                pass

        try:
            self._rx0702_handler = _rx_0702
            register_listener("0702", self._rx0702_handler)
        except Exception as exc:
            self._append_log_line(f"[0702] listener registration failed: {exc}")

    def _install_0201_listener(self) -> None:
        def _rx_0201(_msg_id: str, payload: object | None):
            try:
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    self._last_0201_raw = raw_latest
                    try:
                        self._last_0201_ms = int(time.time() * 1000)
                    except Exception:
                        pass
                self._on_rx_0201(payload)
            except Exception:
                pass

        try:
            self._rx0201_handler = _rx_0201
            register_listener("0201", self._rx0201_handler)
        except Exception as exc:
            self._append_log_line(f"[0201] 리스너 등록 실패: {exc}")

    def _install_0202_listener(self) -> None:
        def _rx_0202(_msg_id: str, payload: object | None):
            try:
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    self._last_0202_raw = raw_latest
                    try:
                        self._last_0202_ms = int(time.time() * 1000)
                    except Exception:
                        pass
                self._on_rx_0202(payload)
            except Exception:
                pass

        try:
            self._rx0202_handler = _rx_0202
            register_listener("0202", self._rx0202_handler)
        except Exception as exc:
            self._append_log_line(f"[0202] listener registration failed: {exc}")

    def _install_0401_listener(self) -> None:
        def _rx_0401(_msg_id: str, payload: object | None):
            try:
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    last_raw = getattr(self, "_last_0401_raw", None)
                    if last_raw is not None and raw_latest == last_raw:
                        return
                    self._last_0401_raw = raw_latest
                    self._on_rx_0401(raw_latest)
                    return
                self._on_rx_0401(payload)
            except Exception:
                pass

        try:
            self._rx0401_handler = _rx_0401
            register_listener("0401", self._rx0401_handler)
        except Exception as exc:
            self._append_log_line(f"[0401] 리스너 등록 실패: {exc}")

    def _install_0402_listener(self) -> None:
        def _rx_0402(_msg_id: str, payload: object | None):
            try:
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    self._last_0402_raw = raw_latest
                try:
                    payload_ms = self._latest_payload_ms(payload)
                except Exception:
                    payload_ms = None
                if payload_ms is not None:
                    self._last_0402_ms = int(payload_ms)
                elif raw_latest:
                    try:
                        self._last_0402_ms = int(time.time() * 1000)
                    except Exception:
                        pass
                self._on_rx_0402(raw_latest if raw_latest else payload)
            except Exception:
                pass

        try:
            self._rx0402_handler = _rx_0402
            register_listener("0402", self._rx0402_handler)
        except Exception as exc:
            self._append_log_line(f"[0402] 리스너 등록 실패: {exc}")

    def _install_0802_listener(self) -> None:
        def _rx_0802(_msg_id: str, payload: object | None):
            try:
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    self._last_0802_raw = raw_latest
                    try:
                        self._last_0802_ms = int(time.time() * 1000)
                    except Exception:
                        pass
                self._on_rx_0802(payload)
            except Exception:
                pass

        try:
            self._rx0802_handler = _rx_0802
            register_listener("0802", self._rx0802_handler)
        except Exception as exc:
            self._append_log_line(f"[0802] listener registration failed: {exc}")

    def _install_0803_listener(self) -> None:
        def _rx_0803(_msg_id: str, payload: object | None):
            try:
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    last_raw = getattr(self, "_last_0803_raw", None)
                    if last_raw is not None and raw_latest == last_raw:
                        return
                    self._last_0803_raw = raw_latest
                    self._on_rx_0803(raw_latest)
                    return
                self._on_rx_0803(payload)
            except Exception:
                pass

        try:
            self._rx0803_handler = _rx_0803
            register_listener("0803", self._rx0803_handler)
        except Exception as exc:
            self._append_log_line(f"[0803] 리스너 등록 실패: {exc}")

    def _on_rx_0903(self, payload: object | None) -> None:
        ts, mpid, source, _body = extract_0903_info(payload)
        if mpid != self._current_mission_plan_id:
            self._current_mission_plan_id = mpid
            self._active_0401_notice_keys.clear()
        if mpid is None:
            return
        viz = getattr(self, "_viz_tab", None)
        if viz is None or not hasattr(viz, "update_0903"):
            viz = None
        try:
            if viz is not None:
                viz.update_0903(timestamp_ms=ts, mission_plan_id=mpid, source=source)
        except Exception:
            pass
        schedule_tab = getattr(self, "_schedule_tab", None)
        try:
            if schedule_tab is not None and hasattr(schedule_tab, "update_0903"):
                schedule_tab.update_0903(timestamp_ms=ts, mission_plan_id=mpid, source=source)
        except Exception:
            pass
        quality_tab = getattr(self, "_quality_tab", None)
        try:
            if quality_tab is not None and hasattr(quality_tab, "update_0903"):
                quality_tab.update_0903(timestamp_ms=ts, mission_plan_id=mpid, source=source)
        except Exception:
            pass
        try:
            quality_speed_coord = getattr(self, "_quality_speed_coord", None)
            if quality_speed_coord is not None and hasattr(quality_speed_coord, "apply_mission_plan_decision"):
                quality_speed_coord.apply_mission_plan_decision(mpid)
        except Exception:
            pass

    def _on_rx_0702(self, payload: object | None) -> None:
        raw_latest = self._unwrap_payload(payload)
        if raw_latest:
            # Prevent double-processing when both listener and poller fire.
            self._last_0702_raw = raw_latest
            parse_target: object | None = raw_latest
        else:
            parse_target = payload

        ts, ignore_val, mission_plan_id, source, _body = extract_0702_decision(parse_target)
        decision_key = (ts, ignore_val, mission_plan_id)
        ts_text = format_timestamp_ms(ts)

        if ignore_val != 2:
            self._clear_pending_0702()
            # Dedupe identical "keep" decisions to reduce log noise.
            if decision_key == getattr(self, "_last_0702_key", None):
                return
            self._last_0702_key = decision_key
            if ignore_val == 1:
                self._append_log_line("[0702] ignore=1 -> keep current mission plan")
                detail = f"시간: {ts_text}"
                if source:
                    detail = f"{detail}\nsource: {source}"
                if mission_plan_id:
                    detail = f"{detail}\nmissionPlanID: {mission_plan_id}"
                self._update_0702_status(status="유지(ignore=1)", detail=detail)
            else:
                self._append_log_line(
                    f"[0702] ignore={ignore_val} -> no mission plan switch"
                )
                detail = f"시간: {ts_text}"
                if source:
                    detail = f"{detail}\nsource: {source}"
                if mission_plan_id:
                    detail = f"{detail}\nmissionPlanID: {mission_plan_id}"
                self._update_0702_status(status=f"ignore={ignore_val}", detail=detail)
            return

        try:
            plan_id_int = int(mission_plan_id) if mission_plan_id is not None else None
        except Exception:
            plan_id_int = None
        if plan_id_int is None or plan_id_int <= 0:
            self._append_log_line("[0702] ignore=2 but missionPlanID is missing/invalid")
            detail = f"시간: {ts_text}"
            if source:
                detail = f"{detail}\nsource: {source}"
            self._update_0702_status(status="오류(ignore=2)", detail=detail)
            return

        applied = self._try_apply_0702_plan(
            plan_id_int,
            timestamp_ms=ts,
            source=source,
            decision_key=decision_key,
            pending_resolved=False,
        )
        if applied:
            return

        pending_same = (
            getattr(self, "_pending_0702_plan_id", None) == plan_id_int
            and getattr(self, "_pending_0702_key", None) == decision_key
        )
        self._pending_0702_plan_id = plan_id_int
        self._pending_0702_ts = ts
        self._pending_0702_source = source
        self._pending_0702_key = decision_key
        if not pending_same:
            self._append_log_line(
                f"[0702] missionPlanID={plan_id_int} not ready; will apply when DB is updated"
            )
            detail = f"시간: {ts_text}\nmissionPlanID: {plan_id_int}"
            if source:
                detail = f"{detail}\nsource: {source}"
            self._update_0702_status(status="대기중(ignore=2)", detail=detail)

    def _clear_pending_0702(self) -> None:
        self._pending_0702_plan_id = None
        self._pending_0702_ts = None
        self._pending_0702_source = None
        self._pending_0702_key = None

    def _maybe_apply_pending_0702(self) -> None:
        plan_id = getattr(self, "_pending_0702_plan_id", None)
        if not plan_id:
            return
        ts = getattr(self, "_pending_0702_ts", None)
        source = getattr(self, "_pending_0702_source", None)
        decision_key = getattr(self, "_pending_0702_key", None) or (ts, 2, plan_id)
        self._try_apply_0702_plan(
            int(plan_id),
            timestamp_ms=ts,
            source=source,
            decision_key=decision_key,
            pending_resolved=True,
        )

    def _try_apply_0702_plan(
        self,
        plan_id: int,
        *,
        timestamp_ms: int | None,
        source: str | None,
        decision_key: tuple[int | None, int | None, int | None],
        pending_resolved: bool,
    ) -> bool:
        plan_path = mission_plan_json_path(plan_id)
        if plan_path is None or not plan_path.exists():
            return False

        if decision_key == getattr(self, "_last_0702_key", None) and plan_id == self._current_mission_plan_id:
            self._clear_pending_0702()
            return True

        if plan_id != self._current_mission_plan_id:
            self._current_mission_plan_id = plan_id
            self._active_0401_notice_keys.clear()

        suffix = " (pending resolved)" if pending_resolved else ""
        src_text = f", source={source}" if source else ""
        self._append_log_line(
            f"[0702] ignore=2 -> apply missionPlanID={plan_id}{src_text}{suffix}"
        )
        is_attack_exclusion = self._apply_attack_exclusion_ignore(plan_id)
        if not is_attack_exclusion:
            self._commit_pending_attack_slot(plan_id)
        detail = f"시간: {format_timestamp_ms(timestamp_ms)}\nmissionPlanID: {plan_id}"
        if source:
            detail = f"{detail}\nsource: {source}"
        if pending_resolved:
            detail = f"{detail}\n상태: pending 해소 후 적용"
        self._update_0702_status(status="적용됨(ignore=2)", detail=detail)

        viz = getattr(self, "_viz_tab", None)
        if viz is not None:
            try:
                if hasattr(viz, "apply_mission_plan_decision"):
                    viz.apply_mission_plan_decision(mission_plan_id=plan_id)
                elif hasattr(viz, "update_0903"):
                    viz.update_0903(timestamp_ms=timestamp_ms, mission_plan_id=plan_id, source=source)
            except Exception as exc:
                self._append_log_line(f"[0702] mission plan apply failed: {exc}")
        schedule_tab = getattr(self, "_schedule_tab", None)
        if schedule_tab is not None:
            try:
                if hasattr(schedule_tab, "apply_mission_plan_decision"):
                    schedule_tab.apply_mission_plan_decision(mission_plan_id=plan_id)
                elif hasattr(schedule_tab, "update_0903"):
                    schedule_tab.update_0903(timestamp_ms=timestamp_ms, mission_plan_id=plan_id, source=source)
            except Exception as exc:
                self._append_log_line(f"[0702] schedule tab apply failed: {exc}")
        quality_tab = getattr(self, "_quality_tab", None)
        if quality_tab is not None:
            try:
                if hasattr(quality_tab, "apply_mission_plan_decision"):
                    quality_tab.apply_mission_plan_decision(mission_plan_id=plan_id)
                elif hasattr(quality_tab, "update_0903"):
                    quality_tab.update_0903(timestamp_ms=timestamp_ms, mission_plan_id=plan_id, source=source)
            except Exception as exc:
                self._append_log_line(f"[0702] quality tab apply failed: {exc}")
        try:
            quality_speed_coord = getattr(self, "_quality_speed_coord", None)
            if quality_speed_coord is not None and hasattr(quality_speed_coord, "apply_mission_plan_decision"):
                quality_speed_coord.apply_mission_plan_decision(plan_id)
        except Exception as exc:
            self._append_log_line(f"[0702] quality-speed coord apply failed: {exc}")

        self._last_0702_key = decision_key
        self._clear_pending_0702()
        return True

    @staticmethod
    def _infer_input_mission_type(detail: object) -> int | None:
        if not isinstance(detail, dict):
            return None
        if detail.get("lineList"):
            return 1
        if detail.get("areaList"):
            return 2
        return None

    @staticmethod
    def _format_0201_type_zero_id(mission_id: object, inferred_type: int | None) -> str:
        mid_text = str(mission_id) if mission_id is not None else "?"
        if int(inferred_type or 0) == 1:
            return f"{mid_text}(line)"
        if int(inferred_type or 0) == 2:
            return f"{mid_text}(area)"
        return mid_text

    @staticmethod
    def _summarize_notice_ids(values: list[str], limit: int = 5) -> str:
        if not values:
            return "-"
        summary = ", ".join(values[:limit])
        remain = len(values) - limit
        if remain > 0:
            summary += f" 외 {remain}건"
        return summary

    def _warn_0201_type_zero(self, payload: object | None) -> None:
        raw_body = parse_payload(payload)
        if not raw_body:
            raw_body = parse_payload(self._unwrap_payload(payload))
        if not isinstance(raw_body, dict):
            return
        mission_list = raw_body.get("inputMissionList")
        if not isinstance(mission_list, list):
            return

        auto_fixed_ids: list[str] = []
        invalid_ids: list[str] = []
        for mission in mission_list:
            if not isinstance(mission, dict):
                continue
            if bool(mission.get("isDone")):
                continue
            mtype_raw = mission.get("inputMissionType")
            try:
                mission_type = int(mtype_raw)
            except Exception:
                mission_type = None
            if mission_type not in (None, 0):
                continue
            mission_id = mission.get("inputMissionID")
            inferred_type = self._infer_input_mission_type(mission.get("missionDetail"))
            if inferred_type is not None:
                auto_fixed_ids.append(
                    self._format_0201_type_zero_id(mission_id, inferred_type)
                )
            else:
                invalid_ids.append(str(mission_id) if mission_id is not None else "?")

        warning_key = (tuple(auto_fixed_ids), tuple(invalid_ids))
        if not auto_fixed_ids and not invalid_ids:
            self._last_0201_type_warning_key = None
            return
        if warning_key == getattr(self, "_last_0201_type_warning_key", None):
            return
        self._last_0201_type_warning_key = warning_key

        if auto_fixed_ids:
            notice = (
                "0201 임무 type 이상 경고: "
                + self._summarize_notice_ids(auto_fixed_ids)
                + " -> 임무계획은 계속 진행하며 도형 기준 자동보정"
            )
            self._append_log_line("[WARN] " + notice)
            self._send_0001_notice(notice)

        if invalid_ids:
            notice = (
                "0201 임무 type 경고: "
                + self._summarize_notice_ids(invalid_ids)
                + " -> 임무계획은 계속 진행하되 후속 검증 필요"
            )
            self._append_log_line("[WARN] " + notice)
            self._send_0001_notice(notice)

    def _on_rx_0201(self, payload: object | None, *, is_new_arrival: bool = True) -> None:
        try:
            self._warn_0201_type_zero(payload)
        except Exception as exc:
            self._append_log_line(f"[WARN] 0201 type warning check failed: {exc}")
        available_ids = collect_available_aircraft_ids(payload)
        if available_ids:
            self._availability_base_ids = {int(aid) for aid in available_ids or []}
            self._availability_seen = True
            self._apply_forced_availability(stage="0201")
        reexecute_active = False
        try:
            coord = getattr(self, "_reexecute_coord", None)
            if coord is not None:
                try:
                    reexecute_active = bool(coord.is_active())
                except Exception:
                    reexecute_active = False
                replan_payload, logs = coord.on_input_plan(
                    payload,
                    has_new_arrival=bool(is_new_arrival),
                )
                for line in logs:
                    self._append_log_line(line)
                if replan_payload:
                    self._send_0902(replan_payload)
                try:
                    reexecute_active = bool(reexecute_active or coord.is_active())
                except Exception:
                    pass
        except Exception as exc:
            self._append_log_line(f"[0902] reexecute-on-0201 error: {exc}")
        try:
            refresh_coord = getattr(self, "_input_refresh_coord", None)
            if refresh_coord is None:
                return
            input_refresh_cfg = get_input_refresh_settings()
            block_when_reexecute_active = bool(input_refresh_cfg.get("block_when_reexecute_active", True))
            refresh_blocked = bool(reexecute_active and block_when_reexecute_active)
            if not self._input_refresh_replan_enabled:
                self._append_log_line("[REINPUT] monitoring trigger OFF -> skip 0902 replan on 0201")
                return
            if refresh_blocked:
                self._append_log_line("[REINPUT] skipped: reexecute-wait mode is active")
            elif self._system_mode_code not in (3, 4):
                self._append_log_line(
                    f"[REINPUT] skipped: mode={self._system_mode_code} (need 3/4)"
                )
            replan_payload, logs = refresh_coord.on_input_plan(
                payload,
                system_mode=self._system_mode_code,
                blocked=refresh_blocked,
            )
            for line in logs:
                self._append_log_line(line)
            if replan_payload:
                self._send_0902(replan_payload)
        except Exception as exc:
            self._append_log_line(f"[0902] input-refresh-on-0201 error: {exc}")

    def _on_rx_0202(self, payload: object | None) -> None:
        try:
            coord = getattr(self, "_prior_mission_coord", None)
            if coord is None:
                return
            if not self._prior_mission_replan_enabled:
                self._append_log_line("[PRIOR] monitoring trigger OFF -> skip 0902 replan on 0202")
                return
            if self._system_mode_code not in (3, 4):
                self._append_log_line(
                    f"[PRIOR] skipped: mode={self._system_mode_code} (need 3/4)"
                )
                return
            replan_payloads, logs = coord.on_prior_mission(
                payload,
                system_mode=self._system_mode_code,
                current_mission_plan_id=self._current_mission_plan_id,
            )
            for line in logs:
                self._append_log_line(line)
            for body in replan_payloads:
                if isinstance(body, dict):
                    self._send_0902(body)
        except Exception as exc:
            self._append_log_line(f"[0902] prior-mission-on-0202 error: {exc}")

    def _on_rx_0401(self, payload: object | None) -> None:
        raw_body = None
        try:
            raw_body = parse_payload(payload)
            if not raw_body:
                raw_body = parse_payload(self._unwrap_payload(payload))
            if raw_body:
                agent_status_snapshot.save_agent_status_snapshot(raw_body)
        except Exception as exc:
            self._append_log_line(f"[0401] snapshot save failed: {exc}")
        try:
            self._update_dl_inference(raw_body)
        except Exception:
            pass
        try:
            turn_tab = getattr(self, "_turn_radius_tab", None)
            if turn_tab is not None and hasattr(turn_tab, "ingest_0401"):
                turn_tab.ingest_0401(raw_body)
        except Exception:
            pass
        ts, states = extract_0401_agent_states(payload)
        try:
            self._bootstrap_availability_from_agent_states(states)
        except Exception:
            pass
        try:
            self._sync_0401_fault_notices(states)
        except Exception as exc:
            self._append_log_line(f"[0001] 0401 abnormal notice update failed: {exc}")
        fuel_state_map: dict[int, str] = {}
        try:
            fuel_coord = getattr(self, "_fuel_coord", None)
            if fuel_coord is not None:
                warnings, fuel_state_map = fuel_coord.update(
                    agent_states=states,
                    timestamp_ms=ts,
                    source="MSM",
                )
                if getattr(self, "_power_on", True):
                    for warning_body in warnings:
                        self._send_0504(warning_body)
        except Exception as exc:
            self._append_log_line(f"[0504] fuel warning update failed: {exc}")
        viz = getattr(self, "_viz_tab", None)
        schedule_tab = getattr(self, "_schedule_tab", None)
        quality_tab = getattr(self, "_quality_tab", None)
        try:
            if viz is not None and hasattr(viz, "update_agent_status"):
                viz.update_agent_status(
                    timestamp_ms=ts,
                    agent_states=states,
                    fuel_state_map=fuel_state_map,
                )
        except Exception:
            pass
        try:
            if schedule_tab is not None and hasattr(schedule_tab, "update_agent_status"):
                schedule_tab.update_agent_status(
                    timestamp_ms=ts,
                    agent_states=states,
                    fuel_state_map=fuel_state_map,
                )
        except Exception:
            pass
        try:
            if quality_tab is not None and hasattr(quality_tab, "update_agent_status"):
                quality_tab.update_agent_status(
                    timestamp_ms=ts,
                    agent_states=states,
                    fuel_state_map=fuel_state_map,
                )
        except Exception:
            pass
        if viz is None or not hasattr(viz, "update_agent_status"):
            return
        try:
            if hasattr(viz, "pop_completion_recommendations"):
                for recommend, input_id in viz.pop_completion_recommendations():
                    self._send_0503(recommend, input_id)
        except Exception:
            pass
        try:
            rtb_coord = getattr(self, "_rtb_replan_coord", None)
            if rtb_coord is not None and self._rtb_replan_enabled:
                forced_coord = getattr(self, "_forced_command_coord", None)
                suppressed_aircraft = ()
                if forced_coord is not None and hasattr(forced_coord, "get_rtb_suppressed_aircraft"):
                    try:
                        suppressed_aircraft = forced_coord.get_rtb_suppressed_aircraft()
                    except Exception:
                        suppressed_aircraft = ()
                replan_payloads, logs = rtb_coord.on_agent_states(
                    states,
                    timestamp_ms=ts,
                    system_mode=self._system_mode_code,
                    current_mission_plan_id=self._current_mission_plan_id,
                    aircraft_filter=self._is_aircraft_in_current_plan,
                    suppressed_aircraft=suppressed_aircraft,
                )
                for line in logs:
                    self._append_log_line(line)
                self._rtb_availability_override = rtb_coord.get_availability_overrides()
                if self._rtb_availability_override:
                    self._apply_forced_availability(stage="0401")
                for body in replan_payloads:
                    if isinstance(body, dict):
                        self._send_0902(body)
        except Exception as exc:
            self._append_log_line(f"[0902] rtb-on-0401 error: {exc}")
        path_dev_suppressed_aircraft: set[int] = set()
        try:
            path_dev_coord = getattr(self, "_path_deviation_coord", None)
            turn_tab = getattr(self, "_turn_radius_tab", None)
            turn_views = None
            if turn_tab is not None and hasattr(turn_tab, "build_views"):
                turn_views = turn_tab.build_views()
            if path_dev_coord is not None:
                replan_payloads, logs = path_dev_coord.on_turn_monitor_views(
                    turn_views,
                    enabled=self._path_deviation_trigger_enabled,
                    system_mode=self._system_mode_code,
                    current_mission_plan_id=self._current_mission_plan_id,
                    aircraft_filter=self._is_aircraft_in_current_plan,
                )
                for line in logs:
                    self._append_log_line(line)
                for body in replan_payloads:
                    if isinstance(body, dict):
                        detail = body.get("replanDetail")
                        if isinstance(detail, dict):
                            try:
                                aid = int(detail.get("aircraftID"))
                            except Exception:
                                aid = None
                            if aid is not None and aid > 0:
                                path_dev_suppressed_aircraft.add(aid)
                        self._send_0902(body)
        except Exception as exc:
            self._append_log_line(f"[0902] path-deviation-on-0401 error: {exc}")
        quality_suppressed_aircraft: set[int] = set(path_dev_suppressed_aircraft)
        try:
            quality_coord = getattr(self, "_quality_speed_coord", None)
            quality_monitor_enabled = bool(self._quality_monitor_enabled)
            quality_replan_enabled = bool(self._quality_speed_replan_enabled)
            if quality_coord is not None:
                replan_payloads, logs = quality_coord.on_agent_states(
                    states,
                    enabled=(quality_monitor_enabled and quality_replan_enabled),
                    system_mode=self._system_mode_code,
                    current_mission_plan_id=self._current_mission_plan_id,
                    aircraft_filter=self._is_aircraft_in_current_plan,
                    suppressed_aircraft=quality_suppressed_aircraft,
                )
                for line in logs:
                    self._append_log_line(line)
                for body in replan_payloads:
                    if isinstance(body, dict):
                        detail = body.get("replanDetail")
                        if isinstance(detail, dict):
                            try:
                                aid = int(detail.get("aircraftID"))
                            except Exception:
                                aid = None
                            if aid is not None and aid > 0:
                                quality_suppressed_aircraft.add(aid)
                        self._send_0902(body)
        except Exception as exc:
            self._append_log_line(f"[0902] quality-speed-on-0401 error: {exc}")
        try:
            imaging_coord = getattr(self, "_imaging_schedule_coord", None)
            if imaging_coord is not None:
                replan_payloads, logs = imaging_coord.on_agent_states(
                    states,
                    enabled=self._schedule_replan_trigger_enabled,
                    system_mode=self._system_mode_code,
                    current_mission_plan_id=self._current_mission_plan_id,
                    aircraft_filter=self._is_aircraft_in_current_plan,
                    suppressed_aircraft=quality_suppressed_aircraft,
                )
                for line in logs:
                    self._append_log_line(line)
                for body in replan_payloads:
                    if isinstance(body, dict):
                        self._send_0902(body)
        except Exception as exc:
            self._append_log_line(f"[0902] imaging-schedule-on-0401 error: {exc}")
        try:
            if self._forced_availability_override or self._rtb_availability_override:
                self._apply_forced_availability(stage="0401")
        except Exception:
            pass

    def _on_rx_0402(self, payload: object | None) -> None:
        try:
            coord = getattr(self, "_target_detection_coord", None)
            if coord is None:
                return
            if not self._target_detection_replan_enabled:
                self._append_log_line("[0402] monitoring trigger OFF -> skip 0902 replan on 0402")
                return
            replan_payloads, logs = coord.on_situation_awareness(
                payload,
                system_mode=self._system_mode_code,
                current_mission_plan_id=self._current_mission_plan_id,
            )
            for line in logs:
                self._append_log_line(line)
            for body in replan_payloads:
                if isinstance(body, dict):
                    self._send_0902(body)
        except Exception as exc:
            self._append_log_line(f"[0902] target-detect-on-0402 error: {exc}")

    def _on_rx_0802(self, payload: object | None) -> None:
        try:
            ts, aircraft_id, mandatory_type, _source, _body = extract_0802_command(payload)
            coord = getattr(self, "_forced_command_coord", None)
            def _send_unavailable_notice(cmd_type: int | None, aircraft_id_value: int | None) -> None:
                try:
                    aid_text = f"{int(aircraft_id_value):04d}"
                except Exception:
                    aid_text = "0000"
                if cmd_type == 2:
                    self._send_0001_notice(
                        f"{aid_text} 비행체 강제귀환 불가로 강제귀환 무효"
                    )
                elif cmd_type == 3:
                    self._send_0001_notice(
                        f"{aid_text} 비행체 임무복귀 불가로 강제임무복귀 무효"
                    )
                elif cmd_type == 1:
                    self._send_0001_notice(
                        f"{aid_text} 비행체 비가용으로 강제대기 무효"
                    )
                else:
                    self._send_0001_notice(
                        f"{aid_text} 비행체 비가용으로 강제명령 무효"
                    )

            if coord is not None and coord.is_permanent_return(aircraft_id):
                _send_unavailable_notice(mandatory_type, aircraft_id)
                self._append_log_line(
                    "[0802] command ignored: aircraft permanently returned"
                )
                return

            try:
                aid_int = int(aircraft_id) if aircraft_id is not None else None
            except Exception:
                aid_int = None
            if aid_int is None:
                _send_unavailable_notice(mandatory_type, aid_int)
                self._append_log_line(
                    "[0802] command ignored: invalid aircraft id"
                )
                return
            if not self._is_aircraft_in_current_plan(int(aid_int)):
                _send_unavailable_notice(mandatory_type, aid_int)
                self._append_log_line(
                    "[0802] command ignored: aircraft not in current plan"
                )
                return
            viz = getattr(self, "_viz_tab", None)
            if (
                viz is not None
                and hasattr(viz, "set_forced_wait")
                and mandatory_type in (1, 3)
            ):
                try:
                    viz.set_forced_wait(
                        aircraft_id=aircraft_id,
                        paused=bool(int(mandatory_type) == 1),
                        timestamp_ms=ts,
                    )
                except Exception:
                    pass
            if coord is None:
                return
            if not self._forced_command_replan_enabled:
                self._append_log_line("[0802] monitoring trigger OFF -> skip forced-command replan handling")
                return
            replan_payloads, logs = coord.on_forced_command(
                payload,
                system_mode=self._system_mode_code,
                current_mission_plan_id=self._current_mission_plan_id,
            )
            for line in logs:
                self._append_log_line(line)
            self._forced_availability_override = coord.get_availability_overrides()
            self._apply_forced_availability(stage="0802")
            for body in replan_payloads:
                if isinstance(body, dict):
                    self._send_0902(body)
        except Exception as exc:
            self._append_log_line(f"[0902] forced-command-on-0802 error: {exc}")

    def _on_rx_0803(self, payload: object | None) -> None:
        _ts, execute, _source, _body = extract_0803_execute(payload)
        if execute is None:
            return
        try:
            coord = getattr(self, "_reexecute_coord", None)
            if coord is not None:
                for line in coord.on_execute(execute):
                    self._append_log_line(line)
        except Exception as exc:
            self._append_log_line(f"[0902] reexecute-on-0803 error: {exc}")

        next_collab_trigger_enabled = bool(getattr(self, "_next_collab_replan_trigger_enabled", False))
        if int(execute) == 1 and next_collab_trigger_enabled:
            try:
                coord = getattr(self, "_next_collab_replan_coord", None)
                viz = getattr(self, "_viz_tab", None)
                turn_tab = getattr(self, "_turn_radius_tab", None)
                context_payload = None
                if viz is not None and hasattr(viz, "build_execute_next_replan_context"):
                    context_payload = viz.build_execute_next_replan_context()
                turn_views = None
                if turn_tab is not None and hasattr(turn_tab, "build_views"):
                    turn_views = turn_tab.build_views()
                if coord is not None:
                    replan_payload, logs = coord.on_execute_next(
                        context_payload,
                        turn_views=turn_views,
                        current_mission_plan_id=self._current_mission_plan_id,
                        system_mode=self._system_mode_code,
                    )
                    for line in logs:
                        self._append_log_line(line)
                    if isinstance(replan_payload, dict):
                        self._send_0902(replan_payload)
                        return
            except Exception as exc:
                self._append_log_line(f"[0902] next-collab-on-0803 error: {exc}")
            self._append_log_line("[NEXTCOLLAB] 0902 next-collab skipped -> falling back to normal execute=1 handling")
        if int(execute) == 1 and not next_collab_trigger_enabled:
            self._append_log_line("[NEXTCOLLAB] monitoring trigger OFF -> skip 0902 replan on 0803 execute=1")

        viz = getattr(self, "_viz_tab", None)
        if viz is None or not hasattr(viz, "handle_execute_command"):
            return
        try:
            viz.handle_execute_command(execute=execute)
        except Exception:
            pass

    def _unwrap_payload(self, payload) -> bytes:
        tab = getattr(self, "_tab", None)
        if tab and hasattr(tab, "_latest_payload_bytes"):
            latest = tab._latest_payload_bytes(payload)
            if isinstance(latest, bytes):
                return latest
        if isinstance(payload, list):
            payload = payload[-1] if payload else b""
        if isinstance(payload, dict):
            payload = payload.get("raw")
        if payload is None:
            return b""
        if isinstance(payload, bytes):
            return payload
        if isinstance(payload, str):
            return payload.encode("utf-8", "ignore")
        try:
            return bytes(payload)
        except Exception:
            return b""

    def _latest_payload_ms(self, payload) -> int | None:
        """Extract the latest arrival timestamp (ms) from RX history when available."""
        try:
            if isinstance(payload, list) and payload:
                last = payload[-1]
                if isinstance(last, dict) and "ms" in last:
                    return int(last.get("ms"))
                if isinstance(last, (tuple, list)) and len(last) >= 1:
                    return int(last[0])
        except Exception:
            return None
        return None

    def _on_rx_0101(self, raw: bytes | None):
        raw_latest = self._unwrap_payload(raw)
        txt = raw_latest.decode("utf-8", "ignore")
        m = re.search(r"\{.*\}", txt, flags=re.S)
        jtxt = m.group(0) if m else txt.strip()
        try:
            body = json.loads(jtxt) if jtxt.startswith("{") else {}
        except Exception:
            body = {}

        code = self._extract_mode_code(body)
        if code is None:
            mm = re.search(r'"systemMode"\s*:\s*([0-9]+)', txt)
            if mm:
                try:
                    code = int(mm.group(1))
                except Exception:
                    code = None

        if code is None:
            return
        if not self._apply_system_mode_code(code):
            self._append_log_line(f"[MODE] 미지원 코드({code})")

    def _extract_mode_code(self, body: dict) -> int | None:
        if not isinstance(body, dict):
            return None
        low = {str(k).lower(): body[k] for k in body.keys() if k is not None}
        for key in ("systemmode", "mode", "modecode", "state"):
            if key in low:
                v = low[key]
                if isinstance(v, bool):
                    return 1 if v else 0
                try:
                    return int(v)
                except Exception:
                    try:
                        return int(float(str(v).strip()))
                    except Exception:
                        return None
        return None

    def _apply_system_mode_code(self, code: int) -> bool:
        if code not in (0, 1, 2, 3):
            return False
        self._external_mode_code = int(code)
        val = int(code)
        # If the user has manually overridden the mode, do not let a stale
        # external 0101 keep forcing us back to standby.
        if getattr(self, "_mode_manual_override", False):
            try:
                slider_val = int(self.mode_slider.value())
            except Exception:
                slider_val = self._system_mode_code
            if slider_val is not None and int(val) != int(slider_val):
                last_ignored = getattr(self, "_last_ignored_external_mode_code", None)
                if last_ignored != int(val):
                    self._append_log_line(
                        f"[MODE] external code={val} ignored (manual slider={slider_val})"
                    )
                    self._last_ignored_external_mode_code = int(val)
                return True
            # External mode matches the slider; release manual override.
            self._mode_manual_override = False
            self._last_ignored_external_mode_code = None
        try:
            self._mode_update_source = "external"
            self.mode_slider.blockSignals(True)
            self.mode_slider.setValue(val)
            self.mode_slider.blockSignals(False)
            self._on_mode_slider_changed(val)
            self._mode_update_source = None
            self._system_mode_code = code
            self._update_0501_state(code)
            self._handle_initplan_transition(code)
            return True
        except Exception:
            self._mode_update_source = None
            return False

    def _start_0101_rx_poller(self) -> None:
        self._last_0101_raw = None
        self._poll_0101_timer = QTimer(self)
        self._poll_0101_timer.setInterval(250)
        self._poll_0101_timer.timeout.connect(self._poll_0101_in_rx_table)
        self._poll_0101_timer.start()

    def _start_0903_rx_poller(self) -> None:
        self._last_0903_raw = None
        self._poll_0903_timer = QTimer(self)
        self._poll_0903_timer.setInterval(250)
        self._poll_0903_timer.timeout.connect(self._poll_0903_in_rx_table)
        self._poll_0903_timer.start()

    def _start_0702_rx_poller(self) -> None:
        self._last_0702_raw = None
        self._last_0702_key = None
        self._pending_0702_plan_id = None
        self._pending_0702_ts = None
        self._pending_0702_source = None
        self._pending_0702_key = None
        self._poll_0702_timer = QTimer(self)
        self._poll_0702_timer.setInterval(250)
        self._poll_0702_timer.timeout.connect(self._poll_0702_in_rx_table)
        self._poll_0702_timer.start()

    def _start_0201_rx_poller(self) -> None:
        if not hasattr(self, "_last_0201_raw"):
            self._last_0201_raw = None
        if not hasattr(self, "_last_0201_ms"):
            self._last_0201_ms = None
        self._poll_0201_timer = QTimer(self)
        self._poll_0201_timer.setInterval(250)
        self._poll_0201_timer.timeout.connect(self._poll_0201_in_rx_table)
        self._poll_0201_timer.start()

    def _start_0202_rx_poller(self) -> None:
        if not hasattr(self, "_last_0202_raw"):
            self._last_0202_raw = None
        if not hasattr(self, "_last_0202_ms"):
            self._last_0202_ms = None
        self._poll_0202_timer = QTimer(self)
        self._poll_0202_timer.setInterval(250)
        self._poll_0202_timer.timeout.connect(self._poll_0202_in_rx_table)
        self._poll_0202_timer.start()

    def _start_0401_rx_poller(self) -> None:
        self._last_0401_raw = None
        self._poll_0401_timer = QTimer(self)
        self._poll_0401_timer.setInterval(250)
        self._poll_0401_timer.timeout.connect(self._poll_0401_in_rx_table)
        self._poll_0401_timer.start()

    def _start_0402_rx_poller(self) -> None:
        if not hasattr(self, "_last_0402_raw"):
            self._last_0402_raw = None
        if not hasattr(self, "_last_0402_ms"):
            self._last_0402_ms = None
        self._poll_0402_timer = QTimer(self)
        self._poll_0402_timer.setInterval(250)
        self._poll_0402_timer.timeout.connect(self._poll_0402_in_rx_table)
        self._poll_0402_timer.start()

    def _start_0802_rx_poller(self) -> None:
        if not hasattr(self, "_last_0802_raw"):
            self._last_0802_raw = None
        if not hasattr(self, "_last_0802_ms"):
            self._last_0802_ms = None
        self._poll_0802_timer = QTimer(self)
        self._poll_0802_timer.setInterval(250)
        self._poll_0802_timer.timeout.connect(self._poll_0802_in_rx_table)
        self._poll_0802_timer.start()

    def _start_forced_hold_timer(self) -> None:
        self._forced_hold_timer = QTimer(self)
        self._forced_hold_timer.setInterval(250)
        self._forced_hold_timer.timeout.connect(self._poll_forced_hold_deadlines)
        self._forced_hold_timer.start()

    def _start_0803_rx_poller(self) -> None:
        self._last_0803_raw = None
        self._poll_0803_timer = QTimer(self)
        self._poll_0803_timer.setInterval(250)
        self._poll_0803_timer.timeout.connect(self._poll_0803_in_rx_table)
        self._poll_0803_timer.start()

    def _poll_0101_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0101":
                    target_row = r
                    break
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest or (self._last_0101_raw is not None and raw_latest == self._last_0101_raw):
                return

            txt = raw_latest.decode("utf-8", "ignore")
            m = re.search(r"\{.*\}", txt, flags=re.S)
            jtxt = m.group(0) if m else txt.strip()
            body = {}
            try:
                if jtxt.startswith("{"):
                    body = json.loads(jtxt)
            except Exception:
                body = {}

            code = self._extract_mode_code(body)
            if code is None:
                mm = re.search(r'"systemMode"\s*:\s*([0-9]+)', txt)
                if mm:
                    try:
                        code = int(mm.group(1))
                    except Exception:
                        code = None

            if code is not None:
                if self._apply_system_mode_code(code):
                    self._last_0101_raw = raw_latest
        except Exception:
            pass

    def _poll_0903_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0903":
                    target_row = r
                    break
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest or (self._last_0903_raw is not None and raw_latest == self._last_0903_raw):
                return
            self._last_0903_raw = raw_latest
            self._on_rx_0903(raw_latest)
        except Exception:
            pass

    def _poll_0702_in_rx_table(self) -> None:
        try:
            self._maybe_apply_pending_0702()
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0702":
                    target_row = r
                    break
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest or (self._last_0702_raw is not None and raw_latest == self._last_0702_raw):
                return
            self._last_0702_raw = raw_latest
            self._on_rx_0702(raw_latest)
        except Exception:
            pass

    def _poll_0201_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0201":
                    target_row = r
                    break
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest:
                return
            latest_ms = self._latest_payload_ms(raw_payload)
            reexecute_pending = False
            try:
                coord = getattr(self, "_reexecute_coord", None)
                if coord is not None:
                    reexecute_pending = bool(coord.is_active())
            except Exception:
                reexecute_pending = False
            last_ms = getattr(self, "_last_0201_ms", None)
            new_arrival = latest_ms is not None and (last_ms is None or latest_ms > last_ms)
            same_payload = self._last_0201_raw is not None and raw_latest == self._last_0201_raw

            if (not new_arrival) and same_payload and (not reexecute_pending):
                return
            if same_payload and reexecute_pending:
                self._append_log_line(
                    "[0201] reexecute pending -> processing latest 0201 even if unchanged"
                )
            elif same_payload and new_arrival and (not reexecute_pending):
                self._append_log_line(
                    "[0201] new arrival detected -> processing even if payload is unchanged"
                )
            self._last_0201_raw = raw_latest
            if latest_ms is not None:
                self._last_0201_ms = int(latest_ms)
            self._on_rx_0201(raw_latest, is_new_arrival=bool(new_arrival))
        except Exception:
            pass

    def _poll_0202_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0202":
                    target_row = r
                    break
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest:
                return
            latest_ms = self._latest_payload_ms(raw_payload)
            last_ms = getattr(self, "_last_0202_ms", None)
            new_arrival = latest_ms is not None and (last_ms is None or latest_ms > last_ms)
            same_payload = self._last_0202_raw is not None and raw_latest == self._last_0202_raw
            if (not new_arrival) and same_payload:
                return
            if same_payload and new_arrival:
                self._append_log_line(
                    "[0202] new arrival detected -> processing even if payload is unchanged"
                )
            self._last_0202_raw = raw_latest
            if latest_ms is not None:
                self._last_0202_ms = int(latest_ms)
            self._on_rx_0202(raw_latest)
        except Exception:
            pass

    def _poll_0401_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0401":
                    target_row = r
                    break
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest or (self._last_0401_raw is not None and raw_latest == self._last_0401_raw):
                return
            self._last_0401_raw = raw_latest
            self._on_rx_0401(raw_latest)
        except Exception:
            pass

    def _poll_0402_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0402":
                    target_row = r
                    break
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest:
                return
            latest_ms = self._latest_payload_ms(raw_payload)
            last_ms = getattr(self, "_last_0402_ms", None)
            new_arrival = latest_ms is not None and (last_ms is None or latest_ms > last_ms)
            same_payload = self._last_0402_raw is not None and raw_latest == self._last_0402_raw
            # If message timestamp is not newer, treat it as already handled
            # even when raw representation differs between listener/poller paths.
            if latest_ms is not None and last_ms is not None and latest_ms <= last_ms:
                return
            if (not new_arrival) and same_payload:
                return
            if same_payload and new_arrival:
                self._append_log_line(
                    "[0402] new arrival detected -> processing even if payload is unchanged"
                )
            self._last_0402_raw = raw_latest
            if latest_ms is not None:
                self._last_0402_ms = int(latest_ms)
            self._on_rx_0402(raw_latest)
        except Exception:
            pass

    def _poll_0802_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0802":
                    target_row = r
                    break
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest:
                return
            latest_ms = self._latest_payload_ms(raw_payload)
            last_ms = getattr(self, "_last_0802_ms", None)
            new_arrival = latest_ms is not None and (last_ms is None or latest_ms > last_ms)
            same_payload = self._last_0802_raw is not None and raw_latest == self._last_0802_raw
            if (not new_arrival) and same_payload:
                return
            if same_payload and new_arrival:
                self._append_log_line(
                    "[0802] new arrival detected -> processing even if payload is unchanged"
                )
            self._last_0802_raw = raw_latest
            if latest_ms is not None:
                self._last_0802_ms = int(latest_ms)
            self._on_rx_0802(raw_latest)
        except Exception:
            pass

    def _poll_0803_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0803":
                    target_row = r
                    break
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest or (self._last_0803_raw is not None and raw_latest == self._last_0803_raw):
                return
            self._last_0803_raw = raw_latest
            self._on_rx_0803(raw_latest)
        except Exception:
            pass

    def _poll_forced_hold_deadlines(self) -> None:
        try:
            coord = getattr(self, "_forced_command_coord", None)
            if coord is None or not self._forced_command_replan_enabled:
                return
            payloads, logs = coord.poll_due_holds(
                system_mode=self._system_mode_code,
                current_mission_plan_id=self._current_mission_plan_id,
                availability_check=self._forced_hold_availability,
            )
            for line in logs:
                self._append_log_line(line)
            if payloads:
                self._forced_availability_override = coord.get_availability_overrides()
                self._apply_forced_availability(stage="0802")
            for body in payloads:
                if isinstance(body, dict):
                    self._send_0902(body)
        except Exception:
            pass

    def _apply_forced_availability(self, *, stage: str) -> None:
        base = set(int(v) for v in (self._availability_base_ids or set()))
        overrides = self._availability_overrides()
        effective = set(base)
        for aid, forced_available in overrides.items():
            try:
                aid_int = int(aid)
            except Exception:
                continue
            if bool(forced_available):
                effective.add(aid_int)
            else:
                effective.discard(aid_int)

        if getattr(self, "_availability_seen", False) or overrides:
            try:
                write_vehicle_status(sorted(effective))
            except Exception as exc:
                self._append_log_line(f"[STATUS] VehicleStatus write failed: {exc}")

        viz = getattr(self, "_viz_tab", None)
        if viz is None or not hasattr(viz, "update_availability"):
            return
        if self._forced_availability_override:
            stage_to_use = "0802"
        elif self._rtb_availability_override:
            stage_to_use = "0401"
        else:
            stage_to_use = stage
        try:
            viz.update_availability(sorted(effective), stage=stage_to_use)
        except Exception:
            pass

    def _rx_setup(self):
        try:
            FusionNodeIoc.Configure()
            NodeMessenger.Initialize("MSM_ReceiveNode")
            NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
            NodeMessenger.InitAllSubscriberFromAssembly()
            NodeMessenger.RegistAllProviderFromFusionNodeIoc()
        except Exception as exc:
            try:
                sys.stderr.write(f"[WARN] MSM bus init failed: {exc}\n")
            except Exception:
                pass


if __name__ == "__main__":
    app = QApplication(sys.argv)
    load_shared_stylesheet(app, PROJECT_ROOT)
    win = MainWindow()
    win.show()
    position_window_from_env(app, win)
    sys.exit(app.exec_())
