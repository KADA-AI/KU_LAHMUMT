# -*- coding: utf-8 -*-
# monitoring_gui.py - Monitoring (MSM) GUI (send/receive only)
from __future__ import annotations

import os
import sys
import threading
import json
import copy
import re
import time
import traceback
import faulthandler
import atexit
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable

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
from modules.common.qt_safety import set_text_if_changed, set_tooltip_if_changed
from modules.common.string_limits import limit_utf8_bytes

ensure_console(os.getenv("KU_CONSOLE_TITLE", "KU Monitoring Console"))
install_process_file_logging("monitoring")

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, QTimer, Qt, QEvent, QObject, QRect, QThread, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QFontMetrics, QFont
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QTabWidget,
    QWidget,
    QLabel,
    QHBoxLayout,
    QVBoxLayout,
    QPushButton,
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


class VisualizationGateWidget(QWidget):
    def __init__(
        self,
        child: QWidget,
        title: str,
        *,
        enabled_by_default: bool = False,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._child = child
        self._title = str(title)
        self._tab_active = False
        self._visualization_enabled = bool(enabled_by_default)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        header = QWidget(self)
        header.setObjectName("VisualizationGateHeader")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(12, 8, 12, 8)
        header_layout.setSpacing(10)

        self._status_label = QLabel("", header)
        self._status_label.setObjectName("VisualizationGateStatus")
        self._toggle_button = QPushButton("", header)
        self._toggle_button.setObjectName("VisualizationGateToggle")
        self._toggle_button.setCheckable(True)
        self._toggle_button.clicked.connect(self._on_toggle_clicked)

        header_layout.addWidget(self._status_label)
        header_layout.addStretch(1)
        header_layout.addWidget(self._toggle_button)
        root.addWidget(header)
        root.addWidget(child, 1)

        self.setStyleSheet(
            """
            QWidget#VisualizationGateHeader {
                background: #f8fafc;
                border-bottom: 1px solid #dbe5ef;
            }
            QLabel#VisualizationGateStatus {
                color: #334155;
                font-size: 12px;
                font-weight: 600;
            }
            QPushButton#VisualizationGateToggle {
                background: #1d4ed8;
                color: #ffffff;
                border: 1px solid #1d4ed8;
                border-radius: 8px;
                padding: 6px 12px;
                font-size: 12px;
                font-weight: 700;
            }
            QPushButton#VisualizationGateToggle:checked {
                background: #475569;
                border-color: #475569;
            }
            """
        )
        self._sync_controls()
        self._apply_child_state()

    def set_tab_active(self, active: bool) -> None:
        self._tab_active = bool(active)
        self._apply_child_state()

    def set_visualization_enabled(self, enabled: bool) -> None:
        self._visualization_enabled = bool(enabled)
        self._sync_controls()
        self._apply_child_state()

    def visualization_enabled(self) -> bool:
        return bool(self._visualization_enabled)

    def _on_toggle_clicked(self, checked: bool) -> None:
        self.set_visualization_enabled(bool(checked))

    def _sync_controls(self) -> None:
        self._toggle_button.blockSignals(True)
        self._toggle_button.setChecked(self._visualization_enabled)
        self._toggle_button.setText("시각화 끄기" if self._visualization_enabled else "시각화 켜기")
        self._toggle_button.blockSignals(False)
        state = "ON" if self._visualization_enabled else "OFF"
        active = "현재 탭" if self._tab_active else "대기"
        self._status_label.setText(f"{self._title} 시각화 {state} / {active}")

    def _apply_child_state(self) -> None:
        requested = bool(self._tab_active and self._visualization_enabled)
        child = self._child
        try:
            setter = getattr(child, "set_visualization_enabled", None)
            if callable(setter):
                setter(requested)
                self._sync_controls()
                return
            setter = getattr(child, "set_ui_updates_enabled", None)
            if callable(setter):
                setter(requested)
        except Exception:
            pass
        self._sync_controls()

_QT_MESSAGE_LOCK = threading.Lock()
_QT_MESSAGE_LAST_LOG: dict[str, float] = {}


def _write_monitoring_diag_line(filename: str, line: str) -> None:
    try:
        from modules.common import db_paths as _diag_db_paths

        base = _diag_db_paths.get_db_subpath("DSS_Internal", "monitoring_diagnostics")
    except Exception:
        base = Path.cwd() / "DSS_Internal" / "monitoring_diagnostics"
    try:
        base.mkdir(parents=True, exist_ok=True)
        with (base / filename).open("a", encoding="utf-8", buffering=1, errors="replace") as handle:
            handle.write(line.rstrip("\n") + "\n")
    except Exception:
        pass


def _qt_silent_handler(mode: QtMsgType, context, message: str):
    text = str(message or "")
    if "Cannot queue arguments of type" in text:
        return
    try:
        sys.stderr.write("[QT] " + text + "\n")
    except Exception:
        pass

    now = time.monotonic()
    key = text[:200]
    with _QT_MESSAGE_LOCK:
        last = float(_QT_MESSAGE_LAST_LOG.get(key, 0.0) or 0.0)
        if now - last < 1.0:
            return
        _QT_MESSAGE_LAST_LOG[key] = now
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    mode_name = getattr(mode, "name", str(mode))
    file_name = getattr(context, "file", "") or ""
    line_no = getattr(context, "line", 0) or 0
    function_name = getattr(context, "function", "") or ""
    _write_monitoring_diag_line(
        "qt_messages.log",
        f"[{stamp}] mode={mode_name} file={file_name}:{line_no} func={function_name} message={text}",
    )


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

from modules.common import db_paths, prior_replan_store
from modules.common import agent_status_snapshot
try:
    from modules.common import replan_perf
except Exception:
    _COMMON_PERF_DIR = COMMON_DIR if (COMMON_DIR / "replan_perf.py").exists() else None
    if _COMMON_PERF_DIR is not None and str(_COMMON_PERF_DIR) not in sys.path:
        sys.path.insert(0, str(_COMMON_PERF_DIR))
    import replan_perf  # type: ignore
from modules.common.ctrl_listener import start_ctrl_listener, env_ctrl_port
from modules.common.settings_paths import (
    ensure_fusion_license_file,
    ensure_fusion_settings_file,
    fusion_runtime_working_dir,
)
from modules.common.gui_process_control import (
    apply_initial_visibility,
    handle_window_control,
    hide_instead_of_close,
)
from modules.monitoring.gui.tabs.mission_schedule_tab import MissionScheduleTab
from modules.monitoring.gui.tabs.quality_monitor_tab import QualityMonitorTab
from modules.monitoring.gui.tabs.replan_management_tab import ReplanManagementTab
from modules.monitoring.gui.tabs.replan_queue_tab import ReplanQueueTab
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
from modules.monitoring.logic.payload_signature import payload_signature_context
from modules.monitoring.logic.replan_dispatch_context import Replan0401DispatchContext
from modules.monitoring.logic.collab_reexecute import CollabReexecuteCoordinator
from modules.monitoring.logic.fuel_warning import FuelWarningCoordinator
from modules.monitoring.logic.forced_command_replan import ForcedCommandReplanCoordinator
from modules.monitoring.logic.imaging_schedule_replan import ImagingScheduleReplanCoordinator
from modules.monitoring.logic.line_scan_progress_monitor import LineScanProgressWorker
from modules.monitoring.logic.next_collab_replan import (
    NextCollabMissionReplanCoordinator,
    _centroid_coordinate,
    _coerce_float,
    _normalize_coordinate,
    _planner_turn_radius_scale,
)
from modules.monitoring.logic.quality_speed_replan import QualitySpeedReplanCoordinator
from modules.monitoring.logic.input_refresh_replan import InputRefreshReplanCoordinator
from modules.monitoring.logic.current_remaining_replan import (
    resolve_remaining_entry_aircraft_list,
)
from modules.monitoring.logic.path_deviation_replan import PathDeviationReplanCoordinator
from modules.monitoring.logic.prior_mission_replan import PriorMissionReplanCoordinator
from modules.monitoring.logic.replan_queue_manager import ReplanQueueManager
from modules.monitoring.logic.replan_runtime_settings import (
    get_dl_risk_settings,
    get_input_refresh_settings,
    get_target_detection_settings,
    get_replan_toggle,
    update_replan_toggle,
)
from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float
from modules.monitoring.logic.rtb_replan import RtbReplanCoordinator
from modules.monitoring.logic.target_detection_replan import (
    TargetDetectionCoordinator,
    build_target_bundle_from_target_info,
    destroyed_target_ids_from_message,
)
from modules.monitoring.logic.target_info import (
    load_target_info,
    mark_targets_as_ignored,
    mark_targets_as_used,
)
from modules.monitoring.utils.vehicle_status import write_vehicle_status
from Tabs.csc_tab_base import _now_ms_since_2000
try:
    from modules.mission_planning.runtime.attack_assignment_state import (
        clear_pending_manned_assignments,
        commit_pending_manned_assignment,
        commit_pending_manned_assignments,
        release_manned_used,
    )
except Exception:
    def clear_pending_manned_assignments(_mission_plan_ids: object) -> list[int]:
        return []

    def commit_pending_manned_assignment(_mission_plan_id: int | None) -> int | None:
        return None

    def commit_pending_manned_assignments(_mission_plan_id: int | None) -> list[int]:
        return []

    def release_manned_used(
        _input_package_id: int | None,
        _aircraft_ids: object = None,
    ) -> list[int]:
        return []

try:
    from modules.monitoring.logic.risk_analysis import RealTimeInferrer, evaluate_risk_thresholds
except Exception:
    RealTimeInferrer = None
    evaluate_risk_thresholds = None


def _ensure_fusion_configs():
    dst = ensure_fusion_settings_file(project_root=PROJECT_ROOT, common_dir=COMMON_DIR)
    if dst is None:
        raise FileNotFoundError("nFusionSettings.json/FusionSettings.json is missing.")
    ensure_fusion_license_file(project_root=PROJECT_ROOT, common_dir=COMMON_DIR)
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
_ANTI_ARMOR_AIR_STRIKE_INPUT_PACKAGE_TYPE = 1
_INPUT_0201_REVIEW_0204_SENT_FLAG = "inputMissionPackageReview0204Sent"


def _optional_int_value(value: Any) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except Exception:
        return None


def _dict_key_ci(container: dict | None, *names: str) -> str | None:
    if not isinstance(container, dict):
        return None
    by_lower = {str(key).lower(): str(key) for key in container.keys()}
    for name in names:
        key = by_lower.get(str(name).lower())
        if key is not None:
            return key
    return None


def _get_dict_ci(container: dict | None, *names: str) -> Any:
    key = _dict_key_ci(container, *names)
    if key is None or not isinstance(container, dict):
        return None
    return container.get(key)


class MainWindow(QMainWindow):
    _ui_invoke_sig = pyqtSignal(object, object, object)
    ctrl_payload = pyqtSignal(dict)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ui_invoke_sig.connect(self._run_ui_invoke)
        self.ctrl_payload.connect(self._handle_ctrl_payload)
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
        self._input_0201_review_lock = threading.RLock()
        # A source package ID can legitimately arrive again in a later initial-plan
        # cycle.  Cache by the actual RX occurrence as well as the source ID so a
        # duplicate mode callback is idempotent without making type-1 review a
        # once-per-process operation.
        self._reviewed_0201_by_arrival: dict[tuple[int, int], dict[str, Any]] = {}
        self._last_rx_0201_arrival_seq = 0
        self._last_rx_0201_package_id: int | None = None
        self._last_rx_0201_payload: dict[str, Any] | None = None
        self._new_target_reviewed_0201_by_source_id: dict[int, dict[str, Any]] = {}
        self._reviewed_0201_generated_ids: set[int] = set()
        self._system_mode_code = None
        self._external_mode_code: int | None = None
        self._mode_manual_override = False
        self._mode_update_source: str | None = None
        self._last_ignored_external_mode_code: int | None = None
        self._bus_ready = False
        self._send_0501_timer = None
        self._last_0501_attempt_monotonic = 0.0
        self._last_0501_payload_monotonic = 0.0
        self._last_0501_send_monotonic = 0.0
        self._last_0501_watchdog_state = "init"
        self._last_0501_timestamp_ms = 0
        self._last_0501_timestamp_error_ms: int | None = None
        self._0501_timestamp_error_style_state: str | None = None
        self._0501_timestamp_lock = threading.Lock()
        self._message_row_cache: dict[tuple[str, str], tuple[int, int, int]] = {}
        self._send_0501_lock = threading.Lock()
        self._nfusion_push_lock = self._send_0501_lock
        self._hb_0102_enabled = False
        self._hb_0102_interval_sec = 0.2
        self._hb_0102_stop = threading.Event()
        self._hb_0102_thread = None
        self._last_0102_send_monotonic = 0.0
        self._hb_0501_enabled = False
        self._hb_0501_interval_sec = 0.2
        self._hb_0501_stop = threading.Event()
        self._hb_0501_thread = None
        self._cached_0501_payload = None
        self._cached_0501_payload_lock = threading.Lock()
        self._cached_0501_plan_id = 0
        self._cached_0501_input_id = 0
        self._send_0501_timer_active = False
        self._throttled_log_last: dict[str, float] = {}
        self._current_mission_plan_id: int | None = None
        self._plan_apply_generation = 0
        self._replan_queue_drain_scheduled = False
        self._replan_queue_draining = False
        self._replan_queue_drain_not_before_ms = 0
        self._active_0401_notice_keys: set[tuple[int, str]] = set()
        self._path_deviation_guard_until_ms = 0
        self._path_deviation_guard_notice_key: tuple[object, ...] | None = None
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
        self._0401_pending_lock = threading.Lock()
        self._0401_pending_payload = None
        self._0401_pending_raw_body: dict[str, Any] | None = None
        self._0401_pending_signature: bytes | None = None
        self._0401_pending_scheduled = False
        self._0401_last_signature: bytes | None = None
        self._0401_coalesce_ms = int(max(20, min(500, self._env_float("MSM_0401_COALESCE_MS", 80.0))))
        self._0401_drain_timer = None
        self._0402_pending_lock = threading.Lock()
        self._0402_pending_payload = None
        self._0402_pending_terminal_payloads: list[object] = []
        self._0402_destroyed_target_ids_seen: set[int] = set()
        self._0402_pending_scheduled = False
        self._0402_last_signature: bytes | None = None
        self._0402_coalesce_ms = int(max(20, min(1000, self._env_float("MSM_0402_COALESCE_MS", 120.0))))
        self._0402_drain_timer = None
        self._input_refresh_replan_enabled = get_replan_toggle("input_refresh", True)
        self._prior_mission_replan_enabled = get_replan_toggle("prior_mission", True)
        self._target_detection_replan_enabled = get_replan_toggle("target_detection", True)
        self._post_attack_rejoin_enabled = get_replan_toggle("post_attack_rejoin", True)
        self._forced_command_replan_enabled = get_replan_toggle("forced_command", True)
        self._rtb_replan_enabled = get_replan_toggle("rtb", True)
        self._path_deviation_trigger_enabled = get_replan_toggle("path_deviation", True)
        self._schedule_replan_trigger_enabled = get_replan_toggle("imaging_schedule", False)
        self._next_collab_replan_trigger_enabled = get_replan_toggle("next_collab", False)
        self._fuel_threshold_logic_enabled = get_replan_toggle("fuel_threshold", False)
        self._quality_monitor_enabled = get_replan_toggle("quality_monitor", True)
        self._quality_speed_replan_enabled = get_replan_toggle("quality_speed", False)
        self._dl_replan_user_enabled = get_replan_toggle("dl_risk", False)
        self._fatal_log_handle = None
        self._init_0401_trace()
        self._enable_monitoring_faulthandler()
        self._install_monitoring_exception_hooks()
        self._start_monitoring_lifecycle_heartbeat()
        self._init_0501_watchdog()

        tabs = QTabWidget()
        polish_tabs(tabs)
        try:
            tabs.setUsesScrollButtons(True)
            bar = tabs.tabBar()
            if bar is not None:
                bar.setUsesScrollButtons(True)
                bar.setExpanding(False)
                bar.setElideMode(Qt.ElideRight)
        except Exception:
            pass
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
        self._replan_queue_tab = ReplanQueueTab()
        self._replan_queue_tab.set_log_callback(self._append_log_line)
        self._replan_queue_manager = ReplanQueueManager(
            now_fn=_now_ms_since_2000,
            logger=self._append_log_line,
        )
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
                "post_attack_rejoin": self._post_attack_rejoin_enabled,
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
        self._start_line_scan_progress_worker()
        self._start_area_snapshot_worker()
        self._replan_management_gate = VisualizationGateWidget(self._replan_management_tab, "임무 재계획 관리")
        self._replan_queue_gate = VisualizationGateWidget(self._replan_queue_tab, "재계획 Queue")
        self._dl_risk_gate = VisualizationGateWidget(self._dl_risk_tab, "실시간 위험도 예측")
        self._schedule_gate = VisualizationGateWidget(self._schedule_tab, "스케줄 모니터")
        self._quality_gate = VisualizationGateWidget(self._quality_tab, "촬영품질")
        self._turn_radius_tab = TurnRadiusMonitorTab()
        self._turn_radius_gate = VisualizationGateWidget(self._turn_radius_tab, "경로추종 모니터링")
        self._replan_management_tab_index = tabs.addTab(self._replan_management_gate, "임무 재계획 관리")
        self._replan_queue_tab_index = tabs.addTab(self._replan_queue_gate, "재계획 Queue")
        self._viz_tab_index = tabs.addTab(self._viz_tab, "모니터링 시각화")
        self._dl_risk_tab_index = tabs.addTab(self._dl_risk_gate, "실시간 위험도 예측")
        self._schedule_tab_index = tabs.addTab(self._schedule_gate, "스케줄 모니터")
        self._quality_tab_index = tabs.addTab(self._quality_gate, "촬영품질")
        self._turn_radius_tab_index = tabs.addTab(self._turn_radius_gate, "경로추종 모니터링")
        tabs.currentChanged.connect(self._on_tab_changed)
        self._on_tab_changed(tabs.currentIndex())

        top = QWidget()
        top.setObjectName("TopBar")
        top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(4, 2, 4, 2)
        top_layout.setSpacing(12)
        self._0501_timestamp_error_label = QLabel("0501 시간오차: 대기")
        self._0501_timestamp_error_label.setObjectName("TimestampErrorLabel")
        self._0501_timestamp_error_label.setFixedWidth(210)
        self._0501_timestamp_error_label.setMinimumHeight(22)
        self._0501_timestamp_error_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._0501_timestamp_error_label.setToolTip(
            "0501 payload timestamp와 송신 호출 시각의 차이(now - timestamp)"
        )
        self._style_0501_timestamp_error_label("idle")
        top_layout.addWidget(self._0501_timestamp_error_label, 0, Qt.AlignLeft | Qt.AlignTop)
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
        self._ctrl_thread = None
        try:
            port = env_ctrl_port(45982)
            self._ctrl_thread = start_ctrl_listener(port, lambda payload: self.ctrl_payload.emit(payload))
            self._append_log_line(f"[CTRL] listener started @ 127.0.0.1:{port}")
        except Exception as exc:
            self._append_log_line(f"[CTRL] listener start failed: {exc}")
        self._set_mode_slider_by_text("초기화 모드")
        self._apply_power_state()
        self._init_0401_dispatcher()
        self._init_0402_dispatcher()

        threading.Thread(target=self._rx_setup, daemon=True).start()
        self._init_0102_autostart()

        self._install_0101_mode_listener()
        self._start_0101_rx_poller()
        self._install_0305_listener()
        self._start_0305_rx_poller()
        self._install_0701_listener()
        self._start_0701_rx_poller()
        self._install_0001_listener()
        self._start_0001_rx_poller()
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
        self._start_replan_queue_timer()
        self._refresh_replan_queue_snapshot()
        self._init_dl_inference()

    def _append_log_line(self, text: str) -> None:
        if not self._is_ui_thread():
            replan_perf.add("monitoring.log.append_request", ui_thread=0, queued_to_ui=1)
            self._invoke_on_ui_thread(self._append_log_line, str(text))
            return
        perf_start = replan_perf.start_timer()
        replan_perf.add("monitoring.log.append_request", ui_thread=1, queued_to_ui=0)
        try:
            emit_process_log("monitoring", str(text))
        except Exception:
            pass
        try:
            if getattr(self, "_tab", None) and hasattr(self._tab, "append_log"):
                self._tab.append_log(text)
                replan_perf.add_elapsed("monitoring.log.append_ui", perf_start, appended=1)
                return
        except Exception:
            pass
        replan_perf.add_elapsed("monitoring.log.append_ui", perf_start, appended=0)

    def _append_throttled_log_line(
        self,
        key: str,
        text: str,
        *,
        min_interval_sec: float = 5.0,
    ) -> bool:
        now = time.monotonic()
        try:
            last_map = getattr(self, "_throttled_log_last", None)
            if not isinstance(last_map, dict):
                last_map = {}
                self._throttled_log_last = last_map
            last = float(last_map.get(str(key), 0.0) or 0.0)
            if last > 0.0 and now - last < max(0.0, float(min_interval_sec)):
                return False
            last_map[str(key)] = now
        except Exception:
            pass
        self._append_log_line(text)
        return True

    def _handle_ctrl_payload(self, payload: dict) -> None:
        if handle_window_control(self, payload, role="monitor", log=self._append_log_line):
            return
        if not isinstance(payload, dict):
            return
        cmd = str(payload.get("cmd") or "").strip().lower()
        if cmd in {"db_root", "debug_db_root", "log_db_root"}:
            self._refresh_db_root(log_first=True)
            return
        if cmd == "self_check":
            try:
                raw_status = payload.get("status", payload.get("on", 1))
                if isinstance(raw_status, str):
                    status = 0 if raw_status.strip().lower() in {"0", "false", "off"} else 1
                else:
                    status = 1 if bool(raw_status) else 0
            except Exception:
                status = 1
            self._append_log_line(f"[CTRL] self_check status={status}")
            self._ensure_0102(status == 1)
            self._send_self_check_0102(status=status)
            return
        if cmd in {"power", "power_on", "poweroff", "mode", "system_mode"}:
            self._append_log_line(f"[CTRL] {payload}")
            return

    def _log_suppressed_exception(
        self,
        key: str,
        prefix: str,
        exc: BaseException,
        *,
        diag_event: str | None = None,
    ) -> None:
        if self._append_throttled_log_line(str(key), f"{prefix}: {exc}"):
            if diag_event:
                try:
                    self._record_0501_diag(
                        diag_event,
                        error=str(exc),
                        traceback=traceback.format_exc(),
                    )
                except Exception:
                    pass

    def _is_ui_thread(self) -> bool:
        try:
            return QThread.currentThread() is self.thread()
        except Exception:
            return False

    def _invoke_on_ui_thread(self, fn, *args, **kwargs) -> None:
        if not callable(fn):
            return
        if self._is_ui_thread():
            fn(*args, **kwargs)
            return
        try:
            self._ui_invoke_sig.emit(fn, tuple(args), dict(kwargs))
        except Exception:
            pass

    def _run_ui_invoke(self, fn: object, args: object, kwargs: object) -> None:
        if not callable(fn):
            return
        call_args = tuple(args) if isinstance(args, tuple) else tuple(args or ())
        call_kwargs = dict(kwargs) if isinstance(kwargs, dict) else {}
        try:
            fn(*call_args, **call_kwargs)
        except Exception as exc:
            try:
                emit_process_log("monitoring", f"[UI] invoke failed: {exc}")
            except Exception:
                pass

    def _start_line_scan_progress_worker(self) -> None:
        self._line_scan_progress_enabled = not self._env_true("MSM_LINE_SCAN_PROGRESS_DISABLE")
        self._line_scan_progress_worker = None
        if not self._line_scan_progress_enabled:
            return
        try:
            min_update_ms = max(50, int(self._env_float("MSM_LINE_SCAN_UPDATE_INTERVAL_MS", 200.0)))
            persist_ms = max(100, int(self._env_float("MSM_LINE_SCAN_PERSIST_INTERVAL_MS", 500.0)))
            worker = LineScanProgressWorker(
                logger=self._append_log_line,
                min_update_interval_ms=min_update_ms,
                persist_interval_ms=persist_ms,
            )
            self._line_scan_progress_worker = worker
            worker.start()
        except Exception as exc:
            self._line_scan_progress_enabled = False
            self._append_log_line(f"[LINE] progress worker start failed: {exc}")

    def _line_scan_apply_mission_plan(self, mission_plan_id: int | None) -> None:
        worker = getattr(self, "_line_scan_progress_worker", None)
        if worker is None:
            return
        try:
            worker.apply_mission_plan(mission_plan_id)
        except Exception as exc:
            self._append_log_line(f"[LINE] plan queue failed: {exc}")

    def _line_scan_reset_input_coverage(self, input_mission_id: int | None) -> None:
        worker = getattr(self, "_line_scan_progress_worker", None)
        if worker is None or input_mission_id is None:
            return
        try:
            worker.reset_input_coverage(int(input_mission_id))
        except Exception as exc:
            self._append_log_line(f"[LINE] input coverage reset queue failed: {exc}")

    def _line_scan_submit_agent_status(
        self,
        *,
        timestamp_ms: int | None,
        agent_states: list[dict[str, Any]],
    ) -> None:
        worker = getattr(self, "_line_scan_progress_worker", None)
        if worker is None:
            return
        try:
            worker.submit_agent_status(timestamp_ms=timestamp_ms, agent_states=agent_states)
        except Exception as exc:
            self._append_log_line(f"[LINE] 0401 queue failed: {exc}")

    def _stop_line_scan_progress_worker(self) -> None:
        worker = getattr(self, "_line_scan_progress_worker", None)
        if worker is None:
            return
        try:
            worker.stop()
        except Exception:
            pass
        self._line_scan_progress_worker = None

    def _start_area_snapshot_worker(self) -> None:
        area_snapshot_env = os.getenv("MSM_AREA_SNAPSHOT_ENABLE", "").strip()
        self._area_snapshot_enabled = True if not area_snapshot_env else self._env_true("MSM_AREA_SNAPSHOT_ENABLE")
        self._area_snapshot_lock = threading.Lock()
        self._area_snapshot_event = threading.Event()
        self._area_snapshot_stop = threading.Event()
        self._area_snapshot_plan_jobs: list[dict[str, Any]] = []
        self._area_snapshot_latest_status: tuple[int | None, list[dict[str, Any]]] | None = None
        self._area_snapshot_thread = None
        if not self._area_snapshot_enabled:
            return
        thread = threading.Thread(
            target=self._run_area_snapshot_worker,
            name="MSM-AreaSnapshot",
            daemon=True,
        )
        self._area_snapshot_thread = thread
        thread.start()

    def _area_snapshot_log(self, message: str) -> None:
        try:
            self._invoke_on_ui_thread(self._append_log_line, str(message))
        except Exception:
            try:
                emit_process_log("monitoring", str(message))
            except Exception:
                pass

    def _run_area_snapshot_worker(self) -> None:
        try:
            from modules.monitoring.logic.mission_area_progress_monitor import (
                MissionProgressAreaSnapshotMonitor,
            )

            interval_ms = max(200, int(self._env_float("MSM_AREA_SNAPSHOT_INTERVAL_MS", 1000.0)))
            update_interval_sec = (
                max(200, int(self._env_float("MSM_AREA_SNAPSHOT_UPDATE_INTERVAL_MS", 200.0)))
                / 1000.0
            )
            monitor = MissionProgressAreaSnapshotMonitor(snapshot_persist_interval_ms=interval_ms)
        except Exception as exc:
            self._area_snapshot_log(f"[AREA] snapshot worker init failed: {exc}")
            return
        stop = getattr(self, "_area_snapshot_stop", None)
        event = getattr(self, "_area_snapshot_event", None)
        if stop is None or event is None:
            return
        last_status_update_monotonic = 0.0
        while not stop.is_set():
            event.wait(0.5)
            event.clear()
            while not stop.is_set():
                with self._area_snapshot_lock:
                    plan_jobs = list(self._area_snapshot_plan_jobs)
                    self._area_snapshot_plan_jobs.clear()
                    latest_status = self._area_snapshot_latest_status
                    self._area_snapshot_latest_status = None
                if not plan_jobs and latest_status is None:
                    break
                for job in plan_jobs:
                    try:
                        reset_input_id = job.get("reset_input_id")
                        if reset_input_id is not None:
                            # 재수행(0803 execute=2): 해당 input의 이월 커버리지를
                            # 비워 처음부터 다시 적립되게 한다.
                            reset_count = monitor.reset_input_coverage(reset_input_id)
                            self._area_snapshot_log(
                                f"[AREA] reexecute -> input {reset_input_id} coverage reset "
                                f"({reset_count} states)"
                            )
                            continue
                        plan_id = job.get("plan_id")
                        if job.get("prefer_apply"):
                            monitor.apply_mission_plan_decision(mission_plan_id=plan_id)
                        else:
                            monitor.update_0903(
                                timestamp_ms=job.get("timestamp_ms"),
                                mission_plan_id=plan_id,
                                source=job.get("source"),
                            )
                    except Exception as exc:
                        self._area_snapshot_log(f"[AREA] plan snapshot update failed: {exc}")
                if latest_status is None:
                    continue
                now = time.monotonic()
                wait_sec = float(update_interval_sec) - (now - float(last_status_update_monotonic))
                if wait_sec > 0.0:
                    stop.wait(wait_sec)
                    with self._area_snapshot_lock:
                        newer_status = self._area_snapshot_latest_status
                        self._area_snapshot_latest_status = None
                    if newer_status is not None:
                        latest_status = newer_status
                try:
                    timestamp_ms, agent_states = latest_status
                    monitor.update_agent_status(
                        timestamp_ms=timestamp_ms,
                        agent_states=agent_states,
                        fuel_state_map=None,
                    )
                    last_status_update_monotonic = time.monotonic()
                except Exception as exc:
                    self._area_snapshot_log(f"[AREA] 0401 snapshot update failed: {exc}")

    def _queue_area_snapshot_plan_update(
        self,
        *,
        plan_id: int | None,
        timestamp_ms: int | None,
        source: str | None,
        prefer_apply: bool,
    ) -> None:
        if not getattr(self, "_area_snapshot_enabled", False):
            return
        if plan_id is None:
            return
        try:
            job = {
                "plan_id": int(plan_id),
                "timestamp_ms": int(timestamp_ms) if timestamp_ms is not None else None,
                "source": source,
                "prefer_apply": bool(prefer_apply),
            }
        except Exception:
            return
        try:
            with self._area_snapshot_lock:
                self._area_snapshot_plan_jobs.append(job)
                if len(self._area_snapshot_plan_jobs) > 16:
                    self._area_snapshot_plan_jobs = self._area_snapshot_plan_jobs[-16:]
            self._area_snapshot_event.set()
        except Exception:
            pass

    def _queue_area_snapshot_input_reset(self, input_mission_id: int | None) -> None:
        if not getattr(self, "_area_snapshot_enabled", False):
            return
        if input_mission_id is None:
            return
        try:
            job = {"reset_input_id": int(input_mission_id)}
        except Exception:
            return
        try:
            with self._area_snapshot_lock:
                self._area_snapshot_plan_jobs.append(job)
            self._area_snapshot_event.set()
        except Exception:
            pass

    def _queue_area_snapshot_status_update(
        self,
        *,
        timestamp_ms: int | None,
        agent_states: list[dict[str, Any]],
    ) -> None:
        if not getattr(self, "_area_snapshot_enabled", False):
            return
        try:
            keep_keys = {
                "aircraft_id",
                "current_waypoint_id",
                "flying",
                "filming",
                "flight_mode",
                "sensor_operation_mode",
                "sensor_center_coordinate",
                "coordinate",
                "footprint_corners",
                "boundary_guard_set_id",
                "boundary_guard_cycle_count",
                "boundary_guard_loop_active",
            }
            rows = [
                {key: item.get(key) for key in keep_keys if key in item}
                for item in (agent_states or [])
                if isinstance(item, dict)
            ]
            ts = int(timestamp_ms) if timestamp_ms is not None else None
            with self._area_snapshot_lock:
                self._area_snapshot_latest_status = (ts, rows)
            self._area_snapshot_event.set()
        except Exception:
            pass

    def _stop_area_snapshot_worker(self) -> None:
        if not getattr(self, "_area_snapshot_enabled", False):
            return
        try:
            self._area_snapshot_stop.set()
            self._area_snapshot_event.set()
        except Exception:
            return
        try:
            thread = getattr(self, "_area_snapshot_thread", None)
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
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
            "post_attack_rejoin": lambda enabled, _state: self._on_post_attack_rejoin_toggled(enabled),
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

    def _on_post_attack_rejoin_toggled(self, enabled: bool) -> None:
        self._post_attack_rejoin_enabled = bool(enabled)
        self._sync_replan_management_toggle("post_attack_rejoin", self._post_attack_rejoin_enabled)
        self._persist_replan_toggle("post_attack_rejoin", self._post_attack_rejoin_enabled)
        state_text = "ON" if self._post_attack_rejoin_enabled else "OFF"
        self._append_log_line(f"[POSTATTACK] post-attack rejoin trigger toggled -> {state_text}")

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

    def _env_float(self, key: str, default: float) -> float:
        try:
            return float(os.getenv(key, str(default)))
        except Exception:
            return float(default)

    def _monitoring_diag_dir(self) -> Path:
        base = db_paths.get_db_subpath("DSS_Internal", "monitoring_diagnostics")
        base.mkdir(parents=True, exist_ok=True)
        return base

    def _enable_monitoring_faulthandler(self) -> None:
        try:
            path = self._monitoring_diag_dir() / f"monitoring_fatal_{os.getpid()}.log"
            handle = path.open("a", encoding="utf-8", buffering=1, errors="replace")
            handle.write(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] faulthandler enabled\n")
            faulthandler.enable(file=handle, all_threads=True)
            self._fatal_log_handle = handle
        except Exception as exc:
            try:
                emit_process_log("monitoring", f"[TRACE] faulthandler enable failed: {exc}")
            except Exception:
                pass

    def _write_monitoring_fatal_line(self, message: str) -> None:
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{stamp}] {message}"
        try:
            handle = getattr(self, "_fatal_log_handle", None)
            if handle is not None:
                handle.write(line.rstrip("\n") + "\n")
                handle.flush()
        except Exception:
            pass
        try:
            _write_monitoring_diag_line("monitoring_fatal_events.log", line)
        except Exception:
            pass

    def _write_monitoring_lifecycle_event(self, event: str, **extra: object) -> None:
        payload = {
            "event": str(event),
            "pid": int(os.getpid()),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "time_unix": time.time(),
            "mode_code": getattr(self, "_system_mode_code", None),
            "mission_plan_id": getattr(self, "_current_mission_plan_id", None),
            "thread_count": threading.active_count(),
        }
        try:
            with self._0401_trace_lock:
                payload["last_0401_state"] = dict(getattr(self, "_0401_trace_state", {}) or {})
        except Exception:
            pass
        try:
            last_0102 = float(
                getattr(self, "_last_0102_send_monotonic", 0.0)
                or getattr(self, "_hb_0102_last_send_monotonic", 0.0)
                or 0.0
            )
            payload["last_0102_send_age_sec"] = round(
                time.monotonic() - last_0102,
                3,
            ) if last_0102 > 0.0 else None
        except Exception:
            pass
        try:
            last_0501 = float(getattr(self, "_last_0501_send_monotonic", 0.0) or 0.0)
            payload["last_0501_send_age_sec"] = round(time.monotonic() - last_0501, 3) if last_0501 > 0.0 else None
        except Exception:
            pass
        payload.update({str(k): self._json_safe(v) for k, v in extra.items()})
        try:
            _write_monitoring_diag_line(
                f"monitoring_lifecycle_{os.getpid()}.jsonl",
                json.dumps(payload, ensure_ascii=False, default=str),
            )
        except Exception:
            pass

    def _install_monitoring_exception_hooks(self) -> None:
        if getattr(self, "_monitoring_exception_hooks_installed", False):
            return
        self._monitoring_exception_hooks_installed = True
        previous_excepthook = sys.excepthook
        previous_unraisablehook = getattr(sys, "unraisablehook", None)
        previous_threading_hook = getattr(threading, "excepthook", None)

        def _write_exception(kind: str, exc_type, exc_value, exc_tb, *, thread_name: str | None = None) -> None:
            try:
                trace_text = "".join(traceback.format_exception(exc_type, exc_value, exc_tb))
            except Exception:
                trace_text = f"{exc_type}: {exc_value}"
            detail = {
                "kind": kind,
                "thread": thread_name,
                "exception": f"{getattr(exc_type, '__name__', exc_type)}: {exc_value}",
                "traceback": trace_text,
            }
            self._write_monitoring_lifecycle_event("unhandled_exception", **detail)
            self._write_monitoring_fatal_line(
                f"unhandled_exception kind={kind} thread={thread_name} exception={detail['exception']}\n"
                f"{detail['traceback']}"
            )

        def _sys_excepthook(exc_type, exc_value, exc_tb):
            _write_exception("sys.excepthook", exc_type, exc_value, exc_tb)
            if callable(previous_excepthook) and previous_excepthook is not _sys_excepthook:
                previous_excepthook(exc_type, exc_value, exc_tb)

        def _threading_excepthook(args):
            thread_obj = getattr(args, "thread", None)
            _write_exception(
                "threading.excepthook",
                getattr(args, "exc_type", None),
                getattr(args, "exc_value", None),
                getattr(args, "exc_traceback", None),
                thread_name=getattr(thread_obj, "name", None),
            )
            if callable(previous_threading_hook) and previous_threading_hook is not _threading_excepthook:
                previous_threading_hook(args)

        def _unraisablehook(args):
            exc_type = getattr(args, "exc_type", None)
            exc_value = getattr(args, "exc_value", None)
            exc_tb = getattr(args, "exc_traceback", None)
            object_text = repr(getattr(args, "object", None))
            _write_exception("sys.unraisablehook", exc_type, exc_value, exc_tb, thread_name=object_text[:160])
            if callable(previous_unraisablehook) and previous_unraisablehook is not _unraisablehook:
                previous_unraisablehook(args)

        sys.excepthook = _sys_excepthook
        if previous_threading_hook is not None:
            threading.excepthook = _threading_excepthook
        if previous_unraisablehook is not None:
            sys.unraisablehook = _unraisablehook
        self._write_monitoring_lifecycle_event("hooks_installed")

    def _start_monitoring_lifecycle_heartbeat(self) -> None:
        if getattr(self, "_monitoring_lifecycle_thread", None) is not None:
            return
        stop = threading.Event()
        self._monitoring_lifecycle_stop = stop
        self._write_monitoring_lifecycle_event("start")

        def _on_atexit() -> None:
            self._write_monitoring_lifecycle_event("atexit")
            self._write_monitoring_fatal_line("atexit reached")

        try:
            atexit.register(_on_atexit)
            self._monitoring_atexit_hook = _on_atexit
        except Exception:
            pass

        def _run() -> None:
            interval = max(1.0, self._env_float("MSM_MONITORING_LIFECYCLE_HEARTBEAT_SEC", 5.0))
            while not stop.wait(interval):
                self._write_monitoring_lifecycle_event("heartbeat")

        thread = threading.Thread(
            target=_run,
            name="MSM-LIFECYCLE",
            daemon=True,
        )
        self._monitoring_lifecycle_thread = thread
        thread.start()

    def _init_0401_trace(self) -> None:
        self._0401_trace_enabled = not self._env_true("MSM_0401_TRACE_DISABLE")
        self._0401_trace_slow_sec = max(0.01, self._env_float("MSM_0401_TRACE_SLOW_SEC", 0.15))
        self._0401_trace_stall_sec = max(0.2, self._env_float("MSM_0401_TRACE_STALL_SEC", 1.0))
        self._0401_trace_lock = threading.Lock()
        self._0401_trace_seq = 0
        self._0401_trace_state: dict[str, object] = {
            "active": False,
            "seq": 0,
            "phase": "idle",
            "phase_status": "idle",
            "phase_started_monotonic": 0.0,
            "handler_started_monotonic": 0.0,
            "updated_monotonic": time.monotonic(),
        }
        self._0401_trace_watchdog_stop = threading.Event()
        self._0401_trace_watchdog_thread = None
        if not self._0401_trace_enabled:
            return
        th = threading.Thread(
            target=self._run_0401_trace_watchdog,
            name="MSM-TRACE-WD",
            daemon=True,
        )
        self._0401_trace_watchdog_thread = th
        th.start()

    def _json_safe(self, value: object) -> object:
        try:
            json.dumps(value, ensure_ascii=False, default=str)
            return value
        except Exception:
            return str(value)

    def _payload_trace_meta(self, payload: object | None) -> dict[str, object]:
        meta: dict[str, object] = {"payload_type": type(payload).__name__}
        try:
            if payload is None:
                meta["payload_empty"] = True
            elif isinstance(payload, (bytes, bytearray)):
                meta["payload_bytes"] = len(payload)
            elif isinstance(payload, str):
                meta["payload_chars"] = len(payload)
            elif isinstance(payload, list):
                meta["payload_items"] = len(payload)
            elif isinstance(payload, dict):
                meta["payload_keys"] = sorted(str(k) for k in payload.keys())[:12]
        except Exception:
            pass
        return meta

    def _write_0401_trace_payload(self, payload: dict[str, object], *, append: bool = False) -> None:
        if not getattr(self, "_0401_trace_enabled", False):
            return
        try:
            base = self._monitoring_diag_dir()
            latest = base / "latest_msm_0401_trace.json"
            latest.write_text(json.dumps(payload, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
            if append:
                trace_path = base / "msm_0401_trace.jsonl"
                with trace_path.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(payload, ensure_ascii=False, default=str) + "\n")
        except Exception:
            pass

    def _set_0401_trace_state(
        self,
        *,
        seq: int,
        phase: str,
        status: str,
        active: bool,
        detail: dict[str, object] | None = None,
    ) -> dict[str, object]:
        now_mono = time.monotonic()
        now_ms = int(_now_ms_since_2000())
        safe_detail = self._json_safe(detail or {})
        with self._0401_trace_lock:
            previous = dict(getattr(self, "_0401_trace_state", {}) or {})
            handler_started = float(previous.get("handler_started_monotonic", 0.0) or 0.0)
            if status == "handler_begin" or handler_started <= 0.0:
                handler_started = now_mono
            phase_started = now_mono if status in {"begin", "handler_begin"} else float(
                previous.get("phase_started_monotonic", now_mono) or now_mono
            )
            state = {
                "event": status,
                "seq": int(seq),
                "phase": str(phase),
                "phase_status": str(status),
                "active": bool(active),
                "timestamp_ms_since_2000": now_ms,
                "wall_time_unix": time.time(),
                "monotonic": now_mono,
                "phase_started_monotonic": phase_started,
                "handler_started_monotonic": handler_started,
                "mode_code": self._system_mode_code,
                "mission_plan_id": self._current_mission_plan_id,
                "detail": safe_detail,
            }
            self._0401_trace_state = state
        self._write_0401_trace_payload(state, append=status in {"handler_begin", "handler_end", "exception", "slow"})
        return state

    def _begin_0401_handler(self, payload: object | None) -> int:
        if not getattr(self, "_0401_trace_enabled", False):
            return 0
        with self._0401_trace_lock:
            self._0401_trace_seq = int(getattr(self, "_0401_trace_seq", 0) or 0) + 1
            seq = int(self._0401_trace_seq)
        self._set_0401_trace_state(
            seq=seq,
            phase="handler",
            status="handler_begin",
            active=True,
            detail=self._payload_trace_meta(payload),
        )
        return seq

    def _end_0401_handler(
        self,
        seq: int,
        *,
        raw_body: object | None,
        timestamp_ms: int | None,
        state_count: int,
    ) -> None:
        if not getattr(self, "_0401_trace_enabled", False) or not seq:
            return
        with self._0401_trace_lock:
            previous = dict(getattr(self, "_0401_trace_state", {}) or {})
        started = float(previous.get("handler_started_monotonic", 0.0) or 0.0)
        elapsed = time.monotonic() - started if started > 0.0 else 0.0
        detail = {
            "elapsed_sec": round(elapsed, 6),
            "raw_body_type": type(raw_body).__name__ if raw_body is not None else None,
            "message_timestamp_ms": timestamp_ms,
            "state_count": int(state_count),
        }
        self._set_0401_trace_state(
            seq=seq,
            phase="handler",
            status="handler_end",
            active=False,
            detail=detail,
        )
        slow_sec = float(getattr(self, "_0401_trace_slow_sec", 0.15) or 0.15)
        if elapsed >= max(slow_sec * 2.0, 0.3):
            self._append_throttled_log_line(
                f"0401_handler_slow_{seq}",
                f"[0401TRACE] handler slow seq={seq} elapsed={elapsed:.3f}s ts={timestamp_ms} states={state_count}",
                min_interval_sec=0.0,
            )

    def _record_0401_handler_exception(self, seq: int, exc: BaseException) -> None:
        if not getattr(self, "_0401_trace_enabled", False) or not seq:
            return
        self._set_0401_trace_state(
            seq=seq,
            phase="handler",
            status="exception",
            active=False,
            detail={
                "error": f"{type(exc).__name__}: {exc}",
                "traceback": traceback.format_exc(),
            },
        )

    @contextmanager
    def _trace_0401_phase(self, seq: int, phase: str, **detail: object):
        if not getattr(self, "_0401_trace_enabled", False) or not seq:
            yield
            return
        started = time.monotonic()
        self._set_0401_trace_state(
            seq=seq,
            phase=phase,
            status="begin",
            active=True,
            detail=detail,
        )
        exc_text = None
        try:
            yield
        except BaseException as exc:
            exc_text = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            elapsed = time.monotonic() - started
            end_detail: dict[str, object] = {"elapsed_sec": round(elapsed, 6)}
            if exc_text:
                end_detail["error"] = exc_text
            status = "exception" if exc_text else "end"
            self._set_0401_trace_state(
                seq=seq,
                phase=phase,
                status=status,
                active=True,
                detail=end_detail,
            )
            slow_sec = float(getattr(self, "_0401_trace_slow_sec", 0.15) or 0.15)
            if elapsed >= slow_sec:
                payload = {
                    "event": "slow",
                    "seq": int(seq),
                    "phase": str(phase),
                    "elapsed_sec": round(elapsed, 6),
                    "timestamp_ms_since_2000": int(_now_ms_since_2000()),
                    "mode_code": self._system_mode_code,
                    "mission_plan_id": self._current_mission_plan_id,
                    "detail": end_detail,
                }
                self._write_0401_trace_payload(payload, append=True)
                self._append_throttled_log_line(
                    f"0401_phase_slow_{phase}",
                    f"[0401TRACE] slow phase={phase} elapsed={elapsed:.3f}s seq={seq}",
                    min_interval_sec=2.0,
                )

    def _dump_monitoring_stacks(self, *, reason: str, seq: int | None = None, phase: str | None = None, age_sec: float | None = None) -> None:
        try:
            safe_phase = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(phase or "none"))[:80]
            stamp = time.strftime("%Y%m%dT%H%M%S", time.localtime())
            suffix = f"{int(time.time_ns() % 1_000_000_000):09d}"
            path = self._monitoring_diag_dir() / f"msm_stack_{reason}_seq{seq or 0}_{safe_phase}_{stamp}_{suffix}.log"
            with path.open("w", encoding="utf-8") as f:
                f.write(f"reason={reason}\nseq={seq}\nphase={phase}\nage_sec={age_sec}\n")
                f.write(f"mode_code={self._system_mode_code}\nmission_plan_id={self._current_mission_plan_id}\n\n")
                try:
                    faulthandler.dump_traceback(file=f, all_threads=True)
                except Exception:
                    f.write("\nfaulthandler failed; falling back to sys._current_frames()\n")
                    frames = sys._current_frames()
                    for thread_id, frame in frames.items():
                        f.write(f"\n--- thread {thread_id} ---\n")
                        f.write("".join(traceback.format_stack(frame)))
            try:
                emit_process_log("monitoring", f"[TRACE] stack dump written: {path}")
            except Exception:
                pass
        except Exception:
            pass

    def _run_0401_trace_watchdog(self) -> None:
        last_0401_bucket: tuple[int, str, int] | None = None
        last_0501_bucket = 0
        while not self._0401_trace_watchdog_stop.wait(0.25):
            now = time.monotonic()
            try:
                with self._0401_trace_lock:
                    state = dict(getattr(self, "_0401_trace_state", {}) or {})
                if state.get("active"):
                    seq = int(state.get("seq", 0) or 0)
                    phase = str(state.get("phase") or "unknown")
                    started = float(state.get("phase_started_monotonic", 0.0) or 0.0)
                    age = now - started if started > 0.0 else 0.0
                    stall_sec = float(getattr(self, "_0401_trace_stall_sec", 1.0) or 1.0)
                    if age >= stall_sec:
                        bucket = (seq, phase, int(age / max(stall_sec, 1.0)))
                        if bucket != last_0401_bucket:
                            last_0401_bucket = bucket
                            payload = {
                                "event": "stalled",
                                "seq": seq,
                                "phase": phase,
                                "age_sec": round(age, 6),
                                "timestamp_ms_since_2000": int(_now_ms_since_2000()),
                                "mode_code": self._system_mode_code,
                                "mission_plan_id": self._current_mission_plan_id,
                                "state": state,
                            }
                            self._write_0401_trace_payload(payload, append=True)
                            try:
                                emit_process_log(
                                    "monitoring",
                                    f"[0401TRACE] stalled seq={seq} phase={phase} age={age:.3f}s",
                                )
                            except Exception:
                                pass
                            self._dump_monitoring_stacks(reason="0401_stall", seq=seq, phase=phase, age_sec=age)

                if self._system_mode_code == 3 and bool(getattr(self, "_power_on", True)):
                    last_ok = max(
                        float(getattr(self, "_last_0501_payload_monotonic", 0.0) or 0.0),
                        float(getattr(self, "_last_0501_send_monotonic", 0.0) or 0.0),
                    )
                    if last_ok > 0.0:
                        age_0501 = now - last_ok
                        if age_0501 >= 1.2:
                            bucket_0501 = int(age_0501 / 1.0)
                            if bucket_0501 != last_0501_bucket:
                                last_0501_bucket = bucket_0501
                                payload = {
                                    "event": "0501_stalled",
                                    "age_sec": round(age_0501, 6),
                                    "timestamp_ms_since_2000": int(_now_ms_since_2000()),
                                    "mode_code": self._system_mode_code,
                                    "mission_plan_id": self._current_mission_plan_id,
                                    "last_0401_state": state,
                                    "0501_context": self._capture_0501_context(),
                                }
                                self._write_0401_trace_payload(payload, append=True)
                                try:
                                    emit_process_log(
                                        "monitoring",
                                        f"[0501TRACE] no send age={age_0501:.3f}s last0401={state.get('phase')}",
                                    )
                                except Exception:
                                    pass
                                if bucket_0501 in (1, 3, 5):
                                    self._dump_monitoring_stacks(
                                        reason="0501_stall",
                                        seq=int(state.get("seq", 0) or 0),
                                        phase=str(state.get("phase") or "unknown"),
                                        age_sec=age_0501,
                                    )
                        else:
                            last_0501_bucket = 0
            except Exception:
                pass

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
        self._queue_replan_payloads(payloads, source="dl_risk")
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
            gate_specs = (
                ("_replan_management_gate", "_replan_management_tab", "_replan_management_tab_index"),
                ("_replan_queue_gate", "_replan_queue_tab", "_replan_queue_tab_index"),
                ("_dl_risk_gate", "_dl_risk_tab", "_dl_risk_tab_index"),
                ("_schedule_gate", "_schedule_tab", "_schedule_tab_index"),
                ("_quality_gate", "_quality_tab", "_quality_tab_index"),
                ("_turn_radius_gate", "_turn_radius_tab", "_turn_radius_tab_index"),
            )
            for gate_name, tab_name, index_name in gate_specs:
                active = index == getattr(self, index_name, -1)
                gate = getattr(self, gate_name, None)
                if gate is not None and hasattr(gate, "set_tab_active"):
                    gate.set_tab_active(active)
                    continue
                tab = getattr(self, tab_name, None)
                if tab is not None and hasattr(tab, "set_ui_updates_enabled"):
                    tab.set_ui_updates_enabled(False)
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
            self._send_0501_timer.setTimerType(Qt.PreciseTimer)
            self._send_0501_timer.setInterval(200)
            self._send_0501_timer.timeout.connect(self._send_0501_tick)
        self._set_0501_heartbeat_enabled(True)
        if not self._send_0501_timer.isActive():
            self._send_0501_timer.start()
            self._record_0501_diag("sender_started")
            self._send_0501_tick()
        try:
            self._send_0501_timer_active = bool(self._send_0501_timer.isActive())
        except Exception:
            self._send_0501_timer_active = True

    def _stop_0501_sender(self) -> None:
        self._set_0501_heartbeat_enabled(False)
        timer = getattr(self, "_send_0501_timer", None)
        if timer is not None and timer.isActive():
            timer.stop()
            self._record_0501_diag("sender_stopped")
        self._send_0501_timer_active = False
        self._update_0501_timestamp_error_label(idle_text="대기")

    def _init_0501_watchdog(self) -> None:
        self._send_0501_watchdog_timer = QTimer(self)
        self._send_0501_watchdog_timer.setInterval(1000)
        self._send_0501_watchdog_timer.timeout.connect(self._check_0501_health)
        self._send_0501_watchdog_timer.start()

    def _set_0501_heartbeat_enabled(self, enabled: bool) -> None:
        self._hb_0501_enabled = bool(enabled)
        if self._hb_0501_enabled:
            self._start_0501_heartbeat_worker_if_needed()

    def _start_0501_heartbeat_worker_if_needed(self) -> None:
        th = getattr(self, "_hb_0501_thread", None)
        if th is not None and th.is_alive():
            return
        self._hb_0501_stop.clear()
        self._hb_0501_thread = threading.Thread(
            target=self._run_0501_heartbeat_worker,
            name="MSM-0501-HB",
            daemon=True,
        )
        self._hb_0501_thread.start()

    def _cache_0501_payload(self, payload: dict | None) -> None:
        if not isinstance(payload, dict):
            return
        try:
            cached = copy.deepcopy(payload)
        except Exception:
            cached = dict(payload)
        lock = getattr(self, "_cached_0501_payload_lock", None)
        if lock is None:
            self._cached_0501_payload = cached
            self._cached_0501_plan_id = self._payload_int(cached, "currentMissionPlanID")
            self._cached_0501_input_id = self._payload_int(cached, "currentInputMissionID")
            return
        with lock:
            self._cached_0501_payload = cached
            self._cached_0501_plan_id = self._payload_int(cached, "currentMissionPlanID")
            self._cached_0501_input_id = self._payload_int(cached, "currentInputMissionID")

    @staticmethod
    def _payload_int(payload: object, key: str, default: int = 0) -> int:
        if not isinstance(payload, dict):
            return int(default)
        try:
            return int(payload.get(key) or default)
        except Exception:
            return int(default)

    def _snapshot_cached_0501_payload(self) -> dict | None:
        lock = getattr(self, "_cached_0501_payload_lock", None)
        try:
            if lock is None:
                payload = getattr(self, "_cached_0501_payload", None)
            else:
                with lock:
                    payload = getattr(self, "_cached_0501_payload", None)
            if not isinstance(payload, dict):
                return None
            snapshot = copy.deepcopy(payload)
        except Exception:
            return None

        def _clear_cached(reason: str) -> None:
            try:
                if lock is None:
                    self._cached_0501_payload = None
                else:
                    with lock:
                        self._cached_0501_payload = None
            except Exception:
                pass
            try:
                emit_process_log(
                    "monitoring",
                    "[0501] cached heartbeat payload discarded: " + str(reason),
                )
            except Exception:
                pass

        try:
            payload_plan_id = int(snapshot.get("currentMissionPlanID") or 0)
        except Exception:
            payload_plan_id = 0
        try:
            current_plan_id = int(getattr(self, "_current_mission_plan_id", None) or 0)
        except Exception:
            current_plan_id = 0
        if payload_plan_id > 0 and current_plan_id > 0 and payload_plan_id != current_plan_id:
            _clear_cached(f"plan {payload_plan_id} != current {current_plan_id}")
            return None

        try:
            payload_input_id = int(snapshot.get("currentInputMissionID") or 0)
        except Exception:
            payload_input_id = 0
        if self._is_ui_thread():
            current_input_id = 0
            viz = getattr(self, "_viz_tab", None)
            method = getattr(viz, "get_current_input_mission_id", None) if viz is not None else None
            if callable(method):
                try:
                    current_input_id = int(method(allow_pending_fallback=False) or 0)
                except TypeError:
                    try:
                        current_input_id = int(method() or 0)
                    except Exception:
                        current_input_id = 0
                except Exception:
                    current_input_id = 0
            if payload_input_id > 0 and current_input_id > 0 and payload_input_id != current_input_id:
                _clear_cached(f"input {payload_input_id} != current {current_input_id}")
                return None
        return snapshot

    def _stamp_0501_payload(self, payload: dict) -> tuple[dict, int]:
        try:
            stamped = copy.deepcopy(payload)
        except Exception:
            stamped = dict(payload)
        ts = self._next_0501_timestamp_ms()
        stamped["timestamp"] = int(ts)
        return stamped, int(ts)

    def _run_0501_heartbeat_worker(self) -> None:
        try:
            from push_center import push_message
        except Exception as exc:
            try:
                emit_process_log("monitoring", f"[0501] heartbeat import failed: {exc}")
            except Exception:
                pass
            return

        try:
            interval = max(0.05, float(getattr(self, "_hb_0501_interval_sec", 0.2) or 0.2))
        except Exception:
            interval = 0.2
        next_due = time.monotonic() + interval
        last_warn = 0.0
        last_failover_log = 0.0

        while not self._hb_0501_stop.is_set():
            if (
                not bool(getattr(self, "_hb_0501_enabled", False))
                or not bool(getattr(self, "_power_on", True))
                or int(getattr(self, "_system_mode_code", -1) or -1) != 3
            ):
                self._hb_0501_stop.wait(0.1)
                next_due = time.monotonic() + interval
                continue

            now = time.monotonic()
            if now < next_due:
                self._hb_0501_stop.wait(min(0.05, next_due - now))
                continue
            if now - next_due > interval:
                next_due = now + interval
            else:
                next_due += interval

            last_send = float(getattr(self, "_last_0501_send_monotonic", 0.0) or 0.0)
            if last_send > 0.0 and now - last_send < max(interval * 1.5, 0.35):
                continue

            cached = self._snapshot_cached_0501_payload()
            if not cached:
                if now - last_warn >= 5.0:
                    try:
                        emit_process_log("monitoring", "[0501] heartbeat waiting for cached payload")
                    except Exception:
                        pass
                    last_warn = now
                continue

            try:
                body, _payload_ts = self._stamp_0501_payload(cached)
                lock = getattr(self, "_send_0501_lock", None)
                if lock is None:
                    push_started = time.monotonic()
                    ok = push_message("0501", NodeMessenger, body_dict=body)
                else:
                    acquired = lock.acquire(blocking=False)
                    if not acquired:
                        if now - last_warn >= 2.0:
                            try:
                                emit_process_log("monitoring", "[0501] heartbeat skipped: push lock busy")
                            except Exception:
                                pass
                            last_warn = now
                        continue
                    try:
                        push_started = time.monotonic()
                        ok = push_message("0501", NodeMessenger, body_dict=body)
                    finally:
                        lock.release()
                push_elapsed = time.monotonic() - push_started
                if push_elapsed >= 0.2 and now - last_warn >= 2.0:
                    try:
                        emit_process_log("monitoring", f"[0501] heartbeat slow push elapsed={push_elapsed:.3f}s")
                    except Exception:
                        pass
                    last_warn = now
                if ok:
                    sent_at = time.monotonic()
                    self._last_0501_payload_monotonic = sent_at
                    self._last_0501_send_monotonic = sent_at
                    if sent_at - last_failover_log >= 5.0:
                        try:
                            emit_process_log("monitoring", "[0501] heartbeat failover send active")
                        except Exception:
                            pass
                        last_failover_log = sent_at
            except Exception as exc:
                if now - last_warn >= 5.0:
                    try:
                        emit_process_log("monitoring", f"[0501] heartbeat send failed: {exc}")
                    except Exception:
                        pass
                    last_warn = now
                next_due = time.monotonic() + max(interval, 0.5)

    def _capture_0501_context(self) -> dict[str, object]:
        if not self._is_ui_thread():
            snapshot = None
            lock = getattr(self, "_cached_0501_payload_lock", None)
            try:
                if lock is None:
                    snapshot = getattr(self, "_cached_0501_payload", None)
                else:
                    with lock:
                        snapshot = getattr(self, "_cached_0501_payload", None)
                if isinstance(snapshot, dict):
                    snapshot = copy.deepcopy(snapshot)
            except Exception:
                snapshot = None
            plan_id = self._payload_int(snapshot, "currentMissionPlanID")
            if plan_id <= 0:
                try:
                    plan_id = int(getattr(self, "_current_mission_plan_id", None) or 0)
                except Exception:
                    plan_id = 0
            input_id = self._payload_int(snapshot, "currentInputMissionID")
            input_ids = [int(input_id)] if input_id > 0 else []
            latest_status_ts = self._payload_int(snapshot, "timestamp")
            sender_active = bool(
                getattr(self, "_send_0501_timer_active", False)
                or getattr(self, "_hb_0501_enabled", False)
            )
            return {
                "power_on": bool(getattr(self, "_power_on", True)),
                "mode_code": self._system_mode_code,
                "mission_plan_id": plan_id or None,
                "has_view": bool(snapshot),
                "uav_entry_count": 0,
                "input_ids": input_ids,
                "latest_status_timestamp_ms": latest_status_ts or None,
                "sender_active": sender_active,
            }
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
        try:
            if not getattr(self, "_power_on", True):
                self._last_0501_watchdog_state = "power_off"
                return
            if self._system_mode_code != 3:
                self._last_0501_watchdog_state = f"mode_{self._system_mode_code}"
                return
            self._set_0501_heartbeat_enabled(True)
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
        except Exception as exc:
            self._log_suppressed_exception(
                "0501_watchdog_exception",
                "[0501] watchdog failed",
                exc,
                diag_event="watchdog_exception",
            )

    def _style_0501_timestamp_error_label(self, state: str) -> None:
        label = getattr(self, "_0501_timestamp_error_label", None)
        if label is None:
            return
        state_key = str(state or "idle")
        if getattr(self, "_0501_timestamp_error_style_state", None) == state_key:
            return
        color = {
            "ok": "#166534",
            "warn": "#b45309",
            "bad": "#b91c1c",
            "idle": "#64748b",
        }.get(state_key, "#64748b")
        label.setStyleSheet(
            "QLabel#TimestampErrorLabel {"
            f" color: {color};"
            " font-size: 11px;"
            " font-weight: 700;"
            " padding-left: 4px;"
            "}"
        )
        self._0501_timestamp_error_style_state = state_key

    def _update_0501_timestamp_error_label(
        self,
        timestamp_ms: int | None = None,
        *,
        now_ms: int | None = None,
        idle_text: str | None = None,
    ) -> None:
        if not self._is_ui_thread():
            self._invoke_on_ui_thread(
                self._update_0501_timestamp_error_label,
                timestamp_ms,
                now_ms=now_ms,
                idle_text=idle_text,
            )
            return
        label = getattr(self, "_0501_timestamp_error_label", None)
        if label is None:
            return
        if idle_text is not None:
            self._last_0501_timestamp_error_ms = None
            set_text_if_changed(label, f"0501 시간오차: {idle_text}")
            set_tooltip_if_changed(
                label,
                "0501 payload timestamp와 송신 호출 시각의 차이(now - timestamp)"
            )
            self._style_0501_timestamp_error_label("idle")
            return
        try:
            ts = int(timestamp_ms) if timestamp_ms is not None else None
            now = int(now_ms) if now_ms is not None else int(_now_ms_since_2000())
        except Exception:
            self._last_0501_timestamp_error_ms = None
            set_text_if_changed(label, "0501 시간오차: 계산 실패")
            self._style_0501_timestamp_error_label("bad")
            return
        if ts is None or ts <= 0:
            self._last_0501_timestamp_error_ms = None
            set_text_if_changed(label, "0501 시간오차: timestamp 없음")
            self._style_0501_timestamp_error_label("bad")
            return
        error_ms = int(now - ts)
        self._last_0501_timestamp_error_ms = error_ms
        abs_error = abs(error_ms)
        state = "ok" if abs_error <= 50 else "warn" if abs_error <= 200 else "bad"
        set_text_if_changed(label, f"0501 시간오차: {error_ms:+d} ms")
        set_tooltip_if_changed(
            label,
            "0501 timestamp 실시간성 확인\n"
            f"now - payload.timestamp = {error_ms:+d} ms\n"
            f"payload.timestamp = {ts}\n"
            f"send-call now = {now}"
        )
        self._style_0501_timestamp_error_label(state)

    def _next_0501_timestamp_ms(self) -> int:
        lock = getattr(self, "_0501_timestamp_lock", None)
        if lock is None:
            ts = int(_now_ms_since_2000())
            last_ts = int(getattr(self, "_last_0501_timestamp_ms", 0) or 0)
            if ts <= last_ts:
                ts = last_ts + 1
            self._last_0501_timestamp_ms = int(ts)
            return int(ts)
        with lock:
            ts = int(_now_ms_since_2000())
            last_ts = int(getattr(self, "_last_0501_timestamp_ms", 0) or 0)
            if ts <= last_ts:
                ts = last_ts + 1
            self._last_0501_timestamp_ms = int(ts)
            return int(ts)

    def _next_plan_apply_generation(self) -> int:
        generation = int(getattr(self, "_plan_apply_generation", 0) or 0) + 1
        self._plan_apply_generation = generation
        return generation

    def _kick_0501_sender(self, *, min_interval_sec: float = 0.0) -> None:
        if not getattr(self, "_power_on", True):
            return
        if self._system_mode_code != 3:
            return
        try:
            min_interval = max(0.0, float(min_interval_sec))
        except Exception:
            min_interval = 0.0
        if min_interval > 0.0:
            last_sent = float(getattr(self, "_last_0501_send_monotonic", 0.0) or 0.0)
            if last_sent > 0.0 and time.monotonic() - last_sent < min_interval:
                return
        try:
            timer = getattr(self, "_send_0501_timer", None)
            if timer is None or not timer.isActive():
                self._start_0501_sender()
                return
            self._send_0501_tick()
        except Exception as exc:
            self._append_log_line(f"[0501] kick failed: {exc}")

    def _defer_plan_tab_updates(
        self,
        *,
        plan_id: int,
        timestamp_ms: int | None,
        source: str | None,
        generation: int,
        prefer_apply: bool,
        log_prefix: str,
        tab_jobs: tuple[tuple[str, str, int], ...],
    ) -> None:
        for tab_attr, label, delay_ms in tab_jobs:
            self._schedule_deferred_plan_tab_update(
                tab_attr=tab_attr,
                label=label,
                plan_id=plan_id,
                timestamp_ms=timestamp_ms,
                source=source,
                generation=generation,
                prefer_apply=prefer_apply,
                delay_ms=delay_ms,
                log_prefix=log_prefix,
            )

    def _schedule_deferred_plan_tab_update(
        self,
        *,
        tab_attr: str,
        label: str,
        plan_id: int,
        timestamp_ms: int | None,
        source: str | None,
        generation: int,
        prefer_apply: bool,
        delay_ms: int,
        log_prefix: str,
    ) -> None:
        try:
            delay = max(0, int(delay_ms))
        except Exception:
            delay = 0
        try:
            QTimer.singleShot(
                delay,
                lambda: self._run_deferred_plan_tab_update(
                    tab_attr=tab_attr,
                    label=label,
                    plan_id=plan_id,
                    timestamp_ms=timestamp_ms,
                    source=source,
                    generation=generation,
                    prefer_apply=prefer_apply,
                    log_prefix=log_prefix,
                ),
            )
        except Exception as exc:
            self._append_log_line(f"{log_prefix} {label} defer failed: {exc}")

    def _run_deferred_plan_tab_update(
        self,
        *,
        tab_attr: str,
        label: str,
        plan_id: int,
        timestamp_ms: int | None,
        source: str | None,
        generation: int,
        prefer_apply: bool,
        log_prefix: str,
    ) -> None:
        if generation != int(getattr(self, "_plan_apply_generation", 0) or 0):
            return
        try:
            if int(plan_id) != int(self._current_mission_plan_id):
                return
        except Exception:
            return
        tab = getattr(self, tab_attr, None)
        if tab is None:
            return
        methods = (
            ("apply_mission_plan_decision", "update_0903")
            if prefer_apply
            else ("update_0903", "apply_mission_plan_decision")
        )
        for method_name in methods:
            method = getattr(tab, method_name, None)
            if not callable(method):
                continue
            try:
                if method_name == "apply_mission_plan_decision":
                    method(mission_plan_id=plan_id)
                else:
                    method(timestamp_ms=timestamp_ms, mission_plan_id=plan_id, source=source)
                return
            except Exception as exc:
                self._append_log_line(f"{log_prefix} {label} apply failed: {exc}")
                return

    def _send_0501_tick(self) -> None:
        self._last_0501_attempt_monotonic = time.monotonic()
        try:
            if not getattr(self, "_power_on", True):
                self._update_0501_timestamp_error_label(idle_text="전원 OFF")
                return
            viz = getattr(self, "_viz_tab", None)
            if viz is None or not hasattr(viz, "build_0501_payload"):
                self._update_0501_timestamp_error_label(idle_text="탭 없음")
                self._record_0501_diag("missing_visualization_tab")
                return
            ts_for_0501 = self._next_0501_timestamp_ms()
            payload = viz.build_0501_payload(timestamp_ms=ts_for_0501, source="MSM")
            if not payload:
                self._update_0501_timestamp_error_label(idle_text="payload 없음")
                self._record_0501_diag("empty_0501_payload")
                return
            self._cache_0501_payload(payload)
            payload_ts = ts_for_0501
            try:
                if isinstance(payload, dict) and payload.get("timestamp") is not None:
                    payload_ts = int(payload.get("timestamp"))
            except Exception:
                payload_ts = ts_for_0501
            self._last_0501_payload_monotonic = time.monotonic()
            try:
                from push_center import push_message
            except Exception as exc:
                self._append_log_line(f"[0501] push import failed: {exc}")
                self._update_0501_timestamp_error_label(idle_text="push import 실패")
                self._record_0501_diag("push_import_failed", error=str(exc))
                return
            row = self._find_tx_row("0501")
            tab = getattr(self, "_tab", None)
            on_done = None
            if row >= 0 and tab is not None and hasattr(tab, "_mark_single_sent"):
                on_done = (lambda mid, raw: tab._mark_single_sent(row, mid, raw))
            send_call_now_ms = int(_now_ms_since_2000())
            lock = getattr(self, "_send_0501_lock", None)
            acquired = False
            if lock is not None:
                acquired = lock.acquire(blocking=False)
                if not acquired:
                    if self._append_throttled_log_line(
                        "0501_send_lock_busy",
                        "[0501] send skipped: push lock busy",
                        min_interval_sec=2.0,
                    ):
                        self._record_0501_diag("send_lock_busy")
                    return
            try:
                push_started = time.monotonic()
                ok = push_message("0501", NodeMessenger, body_dict=payload, on_done=on_done)
            finally:
                if lock is not None and acquired:
                    lock.release()
            push_elapsed = time.monotonic() - push_started
            if push_elapsed >= 0.2:
                self._append_throttled_log_line(
                    "0501_slow_push",
                    f"[0501] slow push elapsed={push_elapsed:.3f}s",
                    min_interval_sec=2.0,
                )
            if not ok:
                self._append_log_line("[0501] send failed")
                self._update_0501_timestamp_error_label(idle_text="송신 실패")
                self._record_0501_diag("send_failed")
                return
            self._last_0501_send_monotonic = time.monotonic()
            self._update_0501_timestamp_error_label(payload_ts, now_ms=send_call_now_ms)
        except Exception as exc:
            self._append_log_line(f"[0501] send error: {exc}")
            self._update_0501_timestamp_error_label(idle_text="송신 예외")
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
    def _is_0102_push_enabled(self) -> bool:
        raw = str(os.getenv("KU_MSM_0102_PUSH_ENABLED", "0") or "0").strip().lower()
        return raw in {"1", "true", "yes", "y", "on"}

    def _init_0102_autostart(self) -> None:
        if not self._is_0102_push_enabled():
            self._append_log_line("[0102] auto heartbeat disabled (set KU_MSM_0102_PUSH_ENABLED=1 to enable)")
            try:
                emit_process_log("monitoring", "[0102] auto heartbeat skipped")
            except Exception:
                pass
            return
        # Delay to let UI/bus/self-check settle before auto send/enable.
        QTimer.singleShot(2000, self._start_0102_autostart)

    def _start_0102_autostart(self, _retry: int = 0) -> None:
        if not self._power_on:
            return
        if not getattr(self, "_bus_ready", False):
            if _retry == 0:
                self._append_log_line("[0102] NodeMessenger 초기화 대기 중 - heartbeat 시작 보류")
            if _retry < 30:
                QTimer.singleShot(300, lambda: self._start_0102_autostart(_retry + 1))
                return
            self._append_log_line("[WARN] NodeMessenger 준비 지연 - 0102 heartbeat 강제 시작")
        if not self._ensure_0102(True):
            if _retry < 10:
                QTimer.singleShot(300, lambda: self._start_0102_autostart(_retry + 1))
            return
        self._send_self_check_0102(status=1)

    def _ensure_0102_periodic(self) -> bool:
        return self._ensure_0102(True)

    def _set_0102_tx_state(self, running: bool) -> None:
        try:
            row = self._find_tx_row("0102")
            if row < 0:
                return
            state_item = self._tab.tbl_tx.item(row, 2)
            if state_item is None:
                return
            if running:
                state_item.setText("주기송신(5Hz/HB)")
                state_item.setForeground(QColor("blue"))
            else:
                state_item.setText("전송 정지")
        except Exception:
            pass

    def _build_0102_body(self, *, status: int = 1) -> dict:
        return {
            "timestamp": _now_ms_since_2000(),
            "status": int(status),
            "source": "MSM",
        }

    def _stop_tab_periodic_0102_if_running(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            timers = getattr(tab, "periodic_timers", {}) if tab is not None else {}
            timer = timers.get("0102")
            if timer is not None:
                timer.stop()
                timer.deleteLater()
                del timers["0102"]
            templates = getattr(tab, "_periodic_payload_templates", None)
            if isinstance(templates, dict):
                templates.pop("0102", None)
        except Exception:
            pass

    def _ensure_0102(self, on: bool) -> bool:
        if on and not self._is_0102_push_enabled():
            self._append_log_line("[0102] heartbeat disabled (set KU_MSM_0102_PUSH_ENABLED=1 to enable)")
            self._set_0102_heartbeat_enabled(False)
            return False
        if on and not self._power_on:
            self._append_log_line("[BLOCK] Power OFF - 0102 heartbeat 차단")
            return False
        row = self._find_tx_row("0102")
        if row < 0:
            self._append_log_line("[0102] TX table row not found")
            return False
        try:
            self._set_0102_heartbeat_enabled(bool(on))
            return True
        except Exception as exc:
            self._append_log_line(f"[0102] heartbeat control failed: {exc}")
            return False

    def _set_0102_heartbeat_enabled(self, enabled: bool) -> None:
        if enabled and not self._is_0102_push_enabled():
            enabled = False
        self._hb_0102_enabled = bool(enabled)
        self._stop_tab_periodic_0102_if_running()
        if self._hb_0102_enabled:
            self._start_0102_heartbeat_worker_if_needed()
            self._set_0102_tx_state(True)
        else:
            self._set_0102_tx_state(False)
            try:
                self._hb_0102_stop.set()
            except Exception:
                pass

    def _start_0102_heartbeat_worker_if_needed(self) -> None:
        th = getattr(self, "_hb_0102_thread", None)
        if th is not None and th.is_alive():
            return
        self._hb_0102_stop.clear()
        self._hb_0102_thread = threading.Thread(
            target=self._run_0102_heartbeat_worker,
            name="MSM-0102-HB",
            daemon=True,
        )
        self._hb_0102_thread.start()
        try:
            emit_process_log("monitoring", "[0102] heartbeat worker started (5Hz)")
        except Exception:
            pass

    def _push_0102_body(self, push_message, body: dict, *, wait_sec: float = 0.0) -> tuple[bool, bytes | None, str]:
        raw_holder: dict[str, bytes | None] = {"raw": None}

        def _on_done(_mid: str, raw: bytes | None) -> None:
            raw_holder["raw"] = raw

        lock = getattr(self, "_nfusion_push_lock", None)
        acquired = False
        if lock is not None:
            try:
                wait = max(0.0, float(wait_sec))
                acquired = lock.acquire(timeout=wait) if wait > 0.0 else lock.acquire(blocking=False)
            except Exception:
                acquired = False
            if not acquired:
                return False, None, "push lock busy"
        try:
            ok = bool(push_message("0102", NodeMessenger, body_dict=body, on_done=_on_done))
            return ok, raw_holder.get("raw"), "" if ok else "push_message returned False"
        except Exception as exc:
            return False, None, str(exc)
        finally:
            if lock is not None and acquired:
                try:
                    lock.release()
                except Exception:
                    pass

    def _mark_0102_sent(self, raw: bytes | None) -> None:
        tab = getattr(self, "_tab", None)
        if tab is not None and hasattr(tab, "mark_sent"):
            try:
                tab.mark_sent("0102", raw)
            except Exception:
                pass

    def _run_0102_heartbeat_worker(self) -> None:
        try:
            from push_center import push_message
        except Exception as exc:
            try:
                emit_process_log("monitoring", f"[0102] heartbeat import failed: {exc}")
            except Exception:
                pass
            return

        try:
            interval = max(0.05, float(getattr(self, "_hb_0102_interval_sec", 0.2) or 0.2))
        except Exception:
            interval = 0.2
        next_due = time.monotonic() + interval
        last_warn = 0.0

        while not self._hb_0102_stop.is_set():
            if (
                not bool(getattr(self, "_hb_0102_enabled", False))
                or not bool(getattr(self, "_power_on", True))
                or not bool(getattr(self, "_bus_ready", False))
            ):
                self._hb_0102_stop.wait(0.1)
                next_due = time.monotonic() + interval
                continue

            now = time.monotonic()
            if now < next_due:
                self._hb_0102_stop.wait(min(0.05, next_due - now))
                continue
            if now - next_due > interval:
                next_due = now + interval
            else:
                next_due += interval

            body = self._build_0102_body(status=1)
            push_started = time.monotonic()
            ok, raw, reason = self._push_0102_body(push_message, body, wait_sec=0.2)
            elapsed = time.monotonic() - push_started
            if ok:
                self._last_0102_send_monotonic = time.monotonic()
                self._invoke_on_ui_thread(self._mark_0102_sent, raw)
                if elapsed >= 0.2 and time.monotonic() - last_warn >= 5.0:
                    try:
                        emit_process_log("monitoring", f"[0102] heartbeat slow push elapsed={elapsed:.3f}s")
                    except Exception:
                        pass
                    last_warn = time.monotonic()
            elif time.monotonic() - last_warn >= 5.0:
                try:
                    emit_process_log("monitoring", f"[0102] heartbeat send skipped: {reason}")
                except Exception:
                    pass
                last_warn = time.monotonic()
        try:
            emit_process_log("monitoring", "[0102] heartbeat worker stopped")
        except Exception:
            pass

    def _stop_0102_sender(self) -> None:
        self._set_0102_heartbeat_enabled(False)

    def _send_self_check_0102(self, status: int = 1, _retry: int = 0) -> bool:
        try:
            status_int = int(status)
        except Exception:
            status_int = 1
        if status_int == 1 and not self._is_0102_push_enabled():
            self._append_log_line("[0102] self-check push skipped (set KU_MSM_0102_PUSH_ENABLED=1 to enable)")
            return False
        if _retry == 0:
            try:
                self._ensure_0102(status_int == 1)
            except Exception:
                pass
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF - 0102 단발 송신 차단")
            return False
        if not getattr(self, "_bus_ready", False):
            if _retry == 0:
                self._append_log_line("[0102] NodeMessenger 초기화 대기 중 - 단발 송신 보류")
            if _retry < 10:
                QTimer.singleShot(300, lambda: self._send_self_check_0102(status=status_int, _retry=_retry + 1))
                return False
            self._append_log_line("[WARN] NodeMessenger 준비 지연 - 0102 단발 강제 송신")
        try:
            from push_center import push_message
        except Exception as exc:
            self._append_log_line(f"[0102] push import failed: {exc}")
            return False
        body = self._build_0102_body(status=status_int)
        try:
            ok, raw, reason = self._push_0102_body(push_message, body, wait_sec=0.0)
            if ok:
                self._last_0102_send_monotonic = time.monotonic()
                self._mark_0102_sent(raw)
                self._append_log_line(f"[0102] status={status_int} sent")
            else:
                self._append_log_line(f"[0102] send failed: {reason}")
            return bool(ok)
        except Exception as exc:
            self._append_log_line(f"[0102] send error: {exc}")
            return False

    # --- 0902 auto replan (init plan mode) ---
    @staticmethod
    def _input_0201_package_id_from_payload(payload: Any) -> int | None:
        if not isinstance(payload, dict):
            return None
        value = _get_dict_ci(payload, "inputMissionPackageID", "InputMissionPackageID", "inputMissionPackageId")
        package_id = _optional_int_value(value)
        return int(package_id) if package_id is not None and int(package_id) > 0 else None

    @staticmethod
    def _input_0201_package_type_from_payload(payload: Any) -> int | None:
        if not isinstance(payload, dict):
            return None
        value = _get_dict_ci(payload, "inputMissionPackageType", "InputMissionPackageType", "inputMissionPackageTYPE")
        return _optional_int_value(value)

    @staticmethod
    def _input_0201_has_core_payload(payload: Any) -> bool:
        if not isinstance(payload, dict):
            return False
        missions = _get_dict_ci(payload, "inputMissionList", "InputMissionList")
        aircraft = _get_dict_ci(payload, "availableAircraftList", "AvailableAircraftList")
        return bool(missions) or bool(aircraft)

    @staticmethod
    def _numeric_json_ids(directory: Path) -> set[int]:
        ids: set[int] = set()
        try:
            for path in Path(directory).glob("*.json"):
                if path.is_file() and path.stem.isdigit():
                    ids.add(int(path.stem))
        except Exception:
            pass
        return ids

    @staticmethod
    def _input_mission_ids_from_payload(payload: Any) -> list[int]:
        if not isinstance(payload, dict):
            return []
        missions = _get_dict_ci(payload, "inputMissionList", "InputMissionList")
        if not isinstance(missions, list):
            return []
        mission_ids: list[int] = []
        for mission in missions:
            if not isinstance(mission, dict):
                continue
            mission_id = _optional_int_value(_get_dict_ci(mission, "inputMissionID", "InputMissionID"))
            if mission_id is not None and mission_id > 0:
                mission_ids.append(int(mission_id))
        return sorted({int(value) for value in mission_ids})

    def _input_mission_ids_from_path(self, path_value: Any) -> list[int]:
        if not path_value:
            return []
        try:
            path = Path(str(path_value))
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            self._append_log_line(f"[0902] reviewed 0201 mission ID read failed: {exc}")
            return []
        return self._input_mission_ids_from_payload(payload)

    @staticmethod
    def _is_msm_reviewed_0201_payload(payload: Any) -> bool:
        """True for InputMissionPlan payloads MSM generated via the type-1 review.

        These carry ``reviewSource=MSM`` / ``reviewedFromInputMissionPackageID``
        provenance markers. They keep inputMissionPackageType=1, so without this
        check a stale review artifact would be re-reviewed as if it were a newly
        received type-1 0201 (re-sending 0204 in non-type-1 scenarios).
        """
        if not isinstance(payload, dict):
            return False
        source = str(_get_dict_ci(payload, "reviewSource", "ReviewSource") or "").strip().upper()
        if source == "MSM":
            return True
        reviewed_from = _optional_int_value(
            _get_dict_ci(
                payload,
                "reviewedFromInputMissionPackageID",
                "ReviewedFromInputMissionPackageID",
            )
        )
        return reviewed_from is not None and reviewed_from > 0

    def _latest_0201_payload_for_review(self) -> tuple[int, dict[str, Any], Path] | None:
        try:
            input_dir = db_paths.get_db_subpath("InputMissionPlan")
        except Exception as exc:
            self._append_log_line(f"[0201-REVIEW] InputMissionPlan path unavailable: {exc}")
            return None

        def _load(path: Path) -> dict[str, Any] | None:
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
            except Exception as exc:
                self._append_log_line(f"[0201-REVIEW] source load failed ({path.name}): {exc}")
                return None
            return payload if isinstance(payload, dict) else None

        # 1) The 0201 actually received in this scenario is the review subject.
        #    Never pick "highest file ID" first - a stale type-1 file (or an MSM
        #    review artifact, which stays type=1) from an earlier scenario can
        #    outrank the current package and would wrongly trigger 0204.
        last_rx_id = _optional_int_value(getattr(self, "_last_rx_0201_package_id", None))
        if last_rx_id is not None and last_rx_id > 0:
            rx_path = Path(input_dir) / f"{int(last_rx_id)}.json"
            # The same package ID may be received again with changed content.  The
            # RX callback is authoritative for this occurrence; reading disk first
            # could pick the previous cycle's file during the RX-to-0101 race.
            payload = None
            cached_payload = getattr(self, "_last_rx_0201_payload", None)
            if isinstance(cached_payload, dict) and (
                self._input_0201_package_id_from_payload(cached_payload) == int(last_rx_id)
            ):
                payload = copy.deepcopy(cached_payload)
            if payload is None and rx_path.is_file():
                payload = _load(rx_path)
            if payload is not None and not self._is_msm_reviewed_0201_payload(payload):
                source_package_id = self._input_0201_package_id_from_payload(payload) or int(last_rx_id)
                return int(source_package_id), payload, rx_path

        # 2) Fallback (e.g. monitoring restarted after the 0201 arrived): most
        #    recently written non-review file, by mtime - not by numeric ID.
        candidates: list[tuple[float, int, Path]] = []
        try:
            for path in Path(input_dir).glob("*.json"):
                if not (path.is_file() and path.stem.isdigit()):
                    continue
                try:
                    mtime = float(path.stat().st_mtime)
                except Exception:
                    mtime = 0.0
                candidates.append((mtime, int(path.stem), path))
        except Exception as exc:
            self._append_log_line(f"[0201-REVIEW] InputMissionPlan scan failed: {exc}")
            return None
        if not candidates:
            return None

        candidates.sort(key=lambda item: (item[0], item[1]), reverse=True)
        for _mtime, file_id, path in candidates:
            payload = _load(path)
            if payload is None:
                continue
            if self._is_msm_reviewed_0201_payload(payload):
                continue
            source_package_id = self._input_0201_package_id_from_payload(payload) or int(file_id)
            return int(source_package_id), payload, path
        return None

    def _reserve_reviewed_0201_package_id(self, input_dir: Path, source_package_id: int) -> int:
        used_ids = self._numeric_json_ids(input_dir)
        try:
            used_ids.update(int(value) for value in getattr(self, "_reviewed_0201_generated_ids", set()))
        except Exception:
            pass
        used_ids.add(int(source_package_id))
        # Reviewed packages are revisions, so always append after the current
        # maximum instead of filling an old numeric hole.  This keeps repeated
        # 0201/0203 test cycles monotonic and avoids colliding with prior output.
        candidate = max(int(source_package_id), max(used_ids, default=0)) + 1
        while candidate in used_ids or (Path(input_dir) / f"{candidate}.json").exists():
            candidate += 1
        self._reviewed_0201_generated_ids.add(int(candidate))
        return int(candidate)

    def _record_external_0201_review_anchor(
        self,
        payload: object | None,
        *,
        is_new_arrival: bool,
    ) -> dict[str, Any]:
        """Record one externally received 0201 occurrence for type-1 review.

        RX table values are normally bytes.  Normalizing here is important: the
        review picker and its provenance checks operate on dictionaries.  An
        unchanged payload with a newer RX-history timestamp is still a new test
        cycle, while an unchanged reexecute poll is not.
        """

        normalized = parse_payload(payload)
        if not self._input_0201_has_core_payload(normalized):
            normalized = parse_payload(self._unwrap_payload(payload))
        if not isinstance(normalized, dict) or not normalized:
            return {}
        package_id = self._input_0201_package_id_from_payload(normalized)
        if package_id is None or self._is_msm_reviewed_0201_payload(normalized):
            return normalized

        previous_id = _optional_int_value(getattr(self, "_last_rx_0201_package_id", None))
        previous_payload = getattr(self, "_last_rx_0201_payload", None)
        occurrence_changed = bool(
            is_new_arrival
            or previous_id != int(package_id)
            or not isinstance(previous_payload, dict)
            or previous_payload != normalized
        )
        if occurrence_changed:
            self._last_rx_0201_arrival_seq = int(
                getattr(self, "_last_rx_0201_arrival_seq", 0) or 0
            ) + 1
        self._last_rx_0201_package_id = int(package_id)
        self._last_rx_0201_payload = copy.deepcopy(normalized)
        return normalized

    def _build_reviewed_0201_payload(
        self,
        source_payload: dict[str, Any],
        *,
        new_package_id: int,
    ) -> dict[str, Any]:
        timestamp = int(_now_ms_since_2000())
        from modules.monitoring.logic.anti_armor_air_strike_review import (
            build_anti_armor_air_strike_review_payload,
        )

        result = build_anti_armor_air_strike_review_payload(
            source_payload,
            new_package_id=int(new_package_id),
            timestamp_ms=int(timestamp),
        )
        self._last_0201_review_summary = dict(result.summary)
        return result.payload

    def _push_review_0204_payload(self, input_mission_package_id: int) -> bool:
        try:
            from push_center import push_message
        except Exception as exc:
            self._append_log_line(f"[0204] push import failed: {exc}")
            return False

        body = {
            "timestamp": int(_now_ms_since_2000()),
            "source": "MSM",
            "inputMissionPackageID": int(input_mission_package_id),
        }
        row = self._find_tx_row("0204")
        tab = getattr(self, "_tab", None)
        on_done = None
        if row >= 0 and tab is not None and hasattr(tab, "_mark_single_sent"):
            on_done = lambda mid, raw: tab._mark_single_sent(row, mid, raw)
        try:
            ok = bool(push_message("0204", NodeMessenger, body_dict=body, on_done=on_done))
        except Exception as exc:
            self._append_log_line(f"[0204] send error: {exc}")
            return False
        if ok:
            self._append_log_line(f"[0204] reviewed inputMissionPackageID={int(input_mission_package_id)} sent")
        else:
            self._append_log_line(f"[0204] reviewed inputMissionPackageID={int(input_mission_package_id)} send failed")
        return bool(ok)

    def _review_type1_0201_for_initial_replan(self) -> dict[str, Any] | None:
        lock = getattr(self, "_input_0201_review_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._input_0201_review_lock = lock

        with lock:
            latest = self._latest_0201_payload_for_review()
            if latest is None:
                return None
            source_package_id, source_payload, source_path = latest

            package_type = self._input_0201_package_type_from_payload(source_payload)
            if package_type != _ANTI_ARMOR_AIR_STRIKE_INPUT_PACKAGE_TYPE:
                return None

            if not self._input_0201_has_core_payload(source_payload):
                self._append_log_line(
                    f"[0201-REVIEW] skipped: source package {int(source_package_id)} has no mission payload"
                )
                return {"applicable": True, "sent": False, "error": "missing_core_payload"}

            from modules.monitoring.logic.anti_armor_air_strike_review import (
                is_anti_armor_air_strike_review_source,
            )

            if not is_anti_armor_air_strike_review_source(source_payload):
                mission_count = len(source_payload.get("inputMissionList") or [])
                self._append_log_line(
                    "[0201-REVIEW] type=1 generic line/area package; "
                    f"skip special anti-armor review and continue normal initial planning "
                    f"(package={int(source_package_id)}, missions={int(mission_count)})"
                )
                return None

            reviewed_by_arrival = getattr(self, "_reviewed_0201_by_arrival", None)
            if not isinstance(reviewed_by_arrival, dict):
                reviewed_by_arrival = {}
                self._reviewed_0201_by_arrival = reviewed_by_arrival
            arrival_seq = int(getattr(self, "_last_rx_0201_arrival_seq", 0) or 0)
            cache_key = (int(arrival_seq), int(source_package_id))
            cached = reviewed_by_arrival.get(cache_key)
            if isinstance(cached, dict):
                reviewed_package_id = _optional_int_value(cached.get("reviewed_package_id"))
                if reviewed_package_id is not None and reviewed_package_id > 0:
                    return {
                        "applicable": True,
                        "sent": True,
                        "inputMissionPackageID": int(reviewed_package_id),
                        "sourceInputMissionPackageID": int(source_package_id),
                        "path": cached.get("path"),
                    }

            try:
                input_dir = db_paths.get_db_subpath("InputMissionPlan")
                input_dir.mkdir(parents=True, exist_ok=True)
            except Exception as exc:
                self._append_log_line(f"[0201-REVIEW] InputMissionPlan path unavailable: {exc}")
                return {"applicable": True, "sent": False, "error": "path_unavailable"}

            new_package_id = self._reserve_reviewed_0201_package_id(input_dir, int(source_package_id))
            try:
                reviewed_payload = self._build_reviewed_0201_payload(
                    source_payload,
                    new_package_id=int(new_package_id),
                )
            except Exception as exc:
                try:
                    self._reviewed_0201_generated_ids.discard(int(new_package_id))
                except Exception:
                    pass
                self._append_log_line(
                    f"[0201-REVIEW] type=1 transform failed "
                    f"({int(source_package_id)}->{int(new_package_id)}): {exc}"
                )
                return {"applicable": True, "sent": False, "error": "transform_failed"}
            output_path = Path(input_dir) / f"{int(new_package_id)}.json"
            try:
                output_path.write_text(
                    json.dumps(reviewed_payload, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
            except Exception as exc:
                try:
                    self._reviewed_0201_generated_ids.discard(int(new_package_id))
                except Exception:
                    pass
                self._append_log_line(
                    f"[0201-REVIEW] write failed ({int(source_package_id)}->{int(new_package_id)}): {exc}"
                )
                return {"applicable": True, "sent": False, "error": "write_failed"}

            if not self._push_review_0204_payload(int(new_package_id)):
                try:
                    output_path.unlink(missing_ok=True)
                except Exception:
                    pass
                try:
                    self._reviewed_0201_generated_ids.discard(int(new_package_id))
                except Exception:
                    pass
                return {"applicable": True, "sent": False, "error": "0204_send_failed"}

            reviewed_by_arrival[cache_key] = {
                "arrival_seq": int(arrival_seq),
                "source_package_id": int(source_package_id),
                "reviewed_package_id": int(new_package_id),
                "source_path": str(source_path),
                "path": str(output_path),
            }
            summary = getattr(self, "_last_0201_review_summary", {}) or {}
            if isinstance(summary, dict) and summary:
                dem_name = Path(str(summary.get("demPath") or "")).name or "-"
                self._append_log_line(
                    "[0201-REVIEW] type=1 transformed missions "
                    f"{summary.get('sourceMissionCount')} -> {summary.get('reviewedMissionCount')}, "
                    f"battleVertices={summary.get('battleAreaVertices')}, "
                    f"enemyCount={summary.get('enemyCount')}, "
                    f"dem={dem_name}, "
                    f"elapsed={summary.get('analysisElapsedS')}s"
                )
            self._append_log_line(
                f"[0201-REVIEW] type=1 reviewed package generated "
                f"{int(source_package_id)} -> {int(new_package_id)}"
            )
            return {
                "applicable": True,
                "sent": True,
                "inputMissionPackageID": int(new_package_id),
                "sourceInputMissionPackageID": int(source_package_id),
                "path": str(output_path),
            }

    def _review_type1_new_target_0201_for_input_refresh(
        self,
        payload: object | None,
    ) -> dict[str, Any] | None:
        source_payload = parse_payload(payload)
        if not self._input_0201_has_core_payload(source_payload):
            source_payload = parse_payload(self._unwrap_payload(payload))
        if not isinstance(source_payload, dict) or self._is_msm_reviewed_0201_payload(source_payload):
            return None

        from modules.monitoring.logic.anti_armor_air_strike_review import (
            build_anti_armor_new_target_refresh_payload,
            is_anti_armor_new_target_refresh_payload,
        )

        if not is_anti_armor_new_target_refresh_payload(source_payload):
            return None
        source_package_id = self._input_0201_package_id_from_payload(source_payload)
        if source_package_id is None:
            self._append_log_line("[REINPUT][0201-REVIEW] new-target source package ID is missing")
            return {"applicable": True, "sent": False, "error": "missing_package_id"}

        lock = getattr(self, "_input_0201_review_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._input_0201_review_lock = lock

        with lock:
            reviewed_by_source = getattr(self, "_new_target_reviewed_0201_by_source_id", None)
            if not isinstance(reviewed_by_source, dict):
                reviewed_by_source = {}
                self._new_target_reviewed_0201_by_source_id = reviewed_by_source
            cached = reviewed_by_source.get(int(source_package_id))
            if isinstance(cached, dict):
                reviewed_package_id = _optional_int_value(cached.get("reviewed_package_id"))
                reviewed_payload = cached.get("payload")
                if (
                    reviewed_package_id is not None
                    and reviewed_package_id > 0
                    and isinstance(reviewed_payload, dict)
                ):
                    return {
                        "applicable": True,
                        "sent": True,
                        "inputMissionPackageID": int(reviewed_package_id),
                        "sourceInputMissionPackageID": int(source_package_id),
                        "path": cached.get("path"),
                        "payload": copy.deepcopy(reviewed_payload),
                    }

            try:
                input_dir = db_paths.get_db_subpath("InputMissionPlan")
                input_dir.mkdir(parents=True, exist_ok=True)
            except Exception as exc:
                self._append_log_line(f"[REINPUT][0201-REVIEW] InputMissionPlan path unavailable: {exc}")
                return {"applicable": True, "sent": False, "error": "path_unavailable"}

            new_package_id = self._reserve_reviewed_0201_package_id(
                input_dir,
                int(source_package_id),
            )
            try:
                result = build_anti_armor_new_target_refresh_payload(
                    source_payload,
                    new_package_id=int(new_package_id),
                    timestamp_ms=int(_now_ms_since_2000()),
                )
                reviewed_payload = result.payload
                self._last_0201_review_summary = dict(result.summary)
            except Exception as exc:
                self._reviewed_0201_generated_ids.discard(int(new_package_id))
                self._append_log_line(
                    f"[REINPUT][0201-REVIEW] new-target transform failed "
                    f"({int(source_package_id)}->{int(new_package_id)}): {exc}"
                )
                return {"applicable": True, "sent": False, "error": "transform_failed"}

            output_path = Path(input_dir) / f"{int(new_package_id)}.json"
            try:
                output_path.write_text(
                    json.dumps(reviewed_payload, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
            except Exception as exc:
                self._reviewed_0201_generated_ids.discard(int(new_package_id))
                self._append_log_line(
                    f"[REINPUT][0201-REVIEW] new-target write failed "
                    f"({int(source_package_id)}->{int(new_package_id)}): {exc}"
                )
                return {"applicable": True, "sent": False, "error": "write_failed"}

            if not self._push_review_0204_payload(int(new_package_id)):
                try:
                    output_path.unlink(missing_ok=True)
                except Exception:
                    pass
                self._reviewed_0201_generated_ids.discard(int(new_package_id))
                return {"applicable": True, "sent": False, "error": "0204_send_failed"}

            reviewed_by_source[int(source_package_id)] = {
                "source_package_id": int(source_package_id),
                "reviewed_package_id": int(new_package_id),
                "path": str(output_path),
                "payload": copy.deepcopy(reviewed_payload),
            }
            summary = getattr(self, "_last_0201_review_summary", {}) or {}
            source_count = summary.get("sourceMissionCount", 11) if isinstance(summary, dict) else 11
            reviewed_count = summary.get("reviewedMissionCount", 16) if isinstance(summary, dict) else 16
            self._append_log_line(
                f"[REINPUT][0201-REVIEW] type=1 new target expanded "
                f"{source_count} -> {reviewed_count} missions, "
                f"package {int(source_package_id)} -> {int(new_package_id)}"
            )
            return {
                "applicable": True,
                "sent": True,
                "inputMissionPackageID": int(new_package_id),
                "sourceInputMissionPackageID": int(source_package_id),
                "path": str(output_path),
                "payload": copy.deepcopy(reviewed_payload),
            }

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
        review_info = self._review_type1_0201_for_initial_replan()
        if isinstance(review_info, dict) and review_info.get("applicable") and not review_info.get("sent"):
            error = str(review_info.get("error") or "unknown")
            self._append_log_line(f"[0902] auto replan aborted: type=1 0201 review 0204 not sent ({error})")
            return
        self._seed_initial_availability_from_input_plan(review_info=review_info)
        mission_ids = []
        if isinstance(review_info, dict) and review_info.get("sent"):
            mission_ids = self._input_mission_ids_from_path(review_info.get("path"))
            if mission_ids:
                self._append_log_line(
                    f"[0902] inputMissionIDList sourced from reviewed 0201: {mission_ids}"
                )
        if not mission_ids:
            mission_ids = collect_input_mission_ids()
        if not mission_ids:
            self._append_log_line("[0902] inputMissionID list empty; abort")
            return
        plan_ids = allocate_mission_plan_ids(1)
        if not plan_ids:
            self._append_log_line("[0902] missionPlanID allocation failed; abort")
            return
        context = build_replan_context(mission_ids, plan_ids)
        ts = _now_ms_since_2000()
        payload = {
            "timestamp": ts,
            "source": "MSM",
            "replanRequestTime": {"replanRequestTimestamp": ts},
            "replanLevel": int(context.get("replan_level", 1)),
            "inputMissionIDList": [
                {"inputMissionID": int(mid)}
                for mid in context.get("mission_ids", [])
                if mid is not None
            ],
            "replanRequest": "초기임무재계획",
            "optionList": [
                {
                    "optionID": idx,
                    "optionName": int(code),
                    "missionPlanID": int(plan_id),
                }
                for idx, (plan_id, code) in enumerate(
                    zip(
                        context.get("plan_ids", []),
                        context.get("option_names", []) or [1] * max(1, len(context.get("plan_ids", []))),
                    ),
                    start=1,
                )
            ],
        }
        if isinstance(review_info, dict) and review_info.get("sent"):
            reviewed_id = _optional_int_value(review_info.get("inputMissionPackageID"))
            if reviewed_id is not None and reviewed_id > 0:
                payload["inputMissionPackageID"] = int(reviewed_id)
                payload[_INPUT_0201_REVIEW_0204_SENT_FLAG] = True
                review_detail = {
                    "inputMissionPackageID": int(reviewed_id),
                    _INPUT_0201_REVIEW_0204_SENT_FLAG: True,
                }
                source_id = _optional_int_value(review_info.get("sourceInputMissionPackageID"))
                if source_id is not None and source_id > 0:
                    payload["sourceInputMissionPackageID"] = int(source_id)
                    review_detail["sourceInputMissionPackageID"] = int(source_id)
                payload["replanDetail"] = review_detail
        self._queue_replan_payloads([payload], source="auto_init")
        self._append_log_line("[0902] auto replan request queued (init plan mode)")

    def _seed_initial_availability_from_input_plan(self, *, review_info: dict[str, Any] | None = None) -> None:
        payload_hint: dict[str, Any] | None = None
        if isinstance(review_info, dict) and review_info.get("sent"):
            reviewed_id = _optional_int_value(review_info.get("inputMissionPackageID"))
            if reviewed_id is not None and reviewed_id > 0:
                payload_hint = {"inputMissionPackageID": int(reviewed_id)}
        available_ids = collect_available_aircraft_ids(payload_hint)
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

    def _find_message_row(self, table_attr: str, msg_id: str) -> int:
        perf_start = replan_perf.start_timer()
        tab = getattr(self, "_tab", None)
        tbl = getattr(tab, table_attr, None) if tab else None
        if tbl is None:
            replan_perf.add_elapsed(
                "monitoring.message_table.row_lookup",
                perf_start,
                missing_table=1,
                table_rx=1 if table_attr == "tbl_rx" else 0,
                table_tx=1 if table_attr == "tbl_tx" else 0,
            )
            return -1
        msg_key = str(msg_id or "").strip()
        try:
            row_count = int(tbl.rowCount())
        except Exception:
            row_count = 0
        cache = getattr(self, "_message_row_cache", None)
        if not isinstance(cache, dict):
            cache = {}
            self._message_row_cache = cache
        cache_key = (str(table_attr), msg_key)
        cached = cache.get(cache_key)
        if isinstance(cached, tuple) and len(cached) == 3:
            table_id, cached_row_count, cached_row = cached
            if int(table_id) == id(tbl) and int(cached_row_count) == row_count:
                row = int(cached_row)
                if 0 <= row < row_count:
                    item = tbl.item(row, 0)
                    if item and item.text().strip() == msg_key:
                        replan_perf.add_elapsed(
                            "monitoring.message_table.row_lookup",
                            perf_start,
                            cache_hit=1,
                            rows=row_count,
                            table_rx=1 if table_attr == "tbl_rx" else 0,
                            table_tx=1 if table_attr == "tbl_tx" else 0,
                        )
                        return row
        for r in range(row_count):
            item = tbl.item(r, 0)
            if item and item.text().strip() == msg_key:
                cache[cache_key] = (id(tbl), row_count, int(r))
                replan_perf.add_elapsed(
                    "monitoring.message_table.row_lookup",
                    perf_start,
                    cache_miss=1,
                    found=1,
                    rows=row_count,
                    scanned_rows=r + 1,
                    table_rx=1 if table_attr == "tbl_rx" else 0,
                    table_tx=1 if table_attr == "tbl_tx" else 0,
                )
                return r
        cache.pop(cache_key, None)
        replan_perf.add_elapsed(
            "monitoring.message_table.row_lookup",
            perf_start,
            cache_miss=1,
            found=0,
            rows=row_count,
            scanned_rows=row_count,
            table_rx=1 if table_attr == "tbl_rx" else 0,
            table_tx=1 if table_attr == "tbl_tx" else 0,
        )
        return -1

    def _find_tx_row(self, msg_id: str) -> int:
        return self._find_message_row("tbl_tx", msg_id)

    def _find_rx_row(self, msg_id: str) -> int:
        row = self._find_message_row("tbl_rx", msg_id)
        replan_perf.add(
            f"monitoring.rx.poller.scan.{msg_id}",
            found=1 if row >= 0 else 0,
            missing=1 if row < 0 else 0,
        )
        return row

    def _payload_size_bytes(self, payload: object | None) -> int:
        if payload is None:
            return 0
        if isinstance(payload, (bytes, bytearray)):
            return len(payload)
        if isinstance(payload, str):
            return len(payload.encode("utf-8", "ignore"))
        try:
            return len(json.dumps(payload, ensure_ascii=False, default=str).encode("utf-8"))
        except Exception:
            return 0

    def _record_rx_listener_event(self, msg_id: str, payload: object | None) -> None:
        replan_perf.add(
            f"monitoring.rx.listener.{msg_id}",
            payload_bytes=self._payload_size_bytes(payload),
            raw_payload=1 if isinstance(payload, (bytes, bytearray, str)) else 0,
        )

    def _record_rx_enqueue_event(self, msg_id: str, **counters: object) -> None:
        replan_perf.add(f"monitoring.rx.enqueue.{msg_id}", **counters)

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
        self._stop_0102_sender()
        self._stop_0501_sender()

    def _on_0503_recommend(self, recommend: int, input_id: int | None = None) -> None:
        self._send_0503(recommend, input_id)

    def _target_detection_option_blocker_for_0503(self) -> dict[str, Any] | None:
        manager = getattr(self, "_replan_queue_manager", None)
        if manager is None or not hasattr(
            manager,
            "find_target_detection_option_decision_blocker",
        ):
            return None
        try:
            blocker = manager.find_target_detection_option_decision_blocker()
        except Exception as exc:
            self._append_log_line(f"[0503] target-option queue check failed: {exc}")
            return None
        return dict(blocker) if isinstance(blocker, dict) else None

    def _send_0503(self, recommend: int, input_id: int | None = None) -> bool:
        try:
            recommend_int = int(recommend)
        except Exception:
            recommend_int = None
        if recommend_int in {1, 3}:
            blocker = self._target_detection_option_blocker_for_0503()
            if blocker is not None:
                queue_id = blocker.get("queue_id")
                reason = str(blocker.get("reason") or "-").strip() or "-"
                try:
                    input_text = f", inputID={int(input_id)}" if input_id is not None else ""
                except Exception:
                    input_text = ""
                queue_text = f", queueID={queue_id}" if queue_id is not None else ""
                log_text = (
                    f"[0503] deferred recommend={recommend_int}{input_text}: "
                    f"target option decision pending{queue_text}, reason={reason}"
                )
                throttled_log = getattr(self, "_append_throttled_log_line", None)
                if callable(throttled_log):
                    throttled_log(
                        f"0503_target_option_pending:{recommend_int}:{input_id}:{queue_id}",
                        log_text,
                    )
                else:
                    self._append_log_line(log_text)
                # This is a defer, not a send failure.  Keep the visualization's
                # observed completion pending so ignore=1 can retry it after the
                # target option decision; ignore=2 resets it with the new plan.
                return False

        viz = getattr(self, "_viz_tab", None)
        try:
            recommend_ok = True
            recommend_reason = ""
            if viz is not None and hasattr(viz, "validate_completion_recommendation"):
                recommend_ok, recommend_reason = viz.validate_completion_recommendation(recommend, input_id)
            if not recommend_ok:
                try:
                    input_text = f", inputID={int(input_id)}" if input_id is not None else ""
                except Exception:
                    input_text = ""
                reason_text = f" ({recommend_reason})" if recommend_reason else ""
                self._append_log_line(
                    f"[0503] blocked recommend={int(recommend)}{input_text}{reason_text}"
                )
                return False
        except Exception as exc:
            self._append_log_line(f"[0503] validation error: {exc}")
            return False
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

    def _refresh_replan_queue_snapshot(self) -> None:
        if not self._is_ui_thread():
            self._invoke_on_ui_thread(self._refresh_replan_queue_snapshot)
            return
        tab = getattr(self, "_replan_queue_tab", None)
        manager = getattr(self, "_replan_queue_manager", None)
        if tab is None or manager is None:
            return
        try:
            build_start = replan_perf.start_timer()
            snapshot = manager.build_snapshot()
            replan_perf.add_elapsed("monitoring.replan_queue.snapshot_build", build_start)
            set_start = replan_perf.start_timer()
            tab.set_snapshot(snapshot)
            replan_perf.add_elapsed("monitoring.replan_queue.tab_set_snapshot", set_start)
        except Exception:
            pass

    def _current_dispatch_plan_id(self) -> int | None:
        plan_id = self._current_mission_plan_id
        if plan_id is None:
            plan_id = self._current_plan_id_from_viz()
        try:
            return int(plan_id) if plan_id is not None else None
        except Exception:
            return None

    def _normalized_replan_option_name(self, value: object) -> str:
        return "".join(str(value or "").split()).lower()

    @staticmethod
    def _extract_positive_int(value: object) -> int | None:
        if value is None:
            return None
        try:
            parsed = int(value)
        except Exception:
            return None
        return parsed if parsed > 0 else None

    def _queue_has_target_detection_history(self) -> bool:
        manager = getattr(self, "_replan_queue_manager", None)
        if manager is None:
            return False
        try:
            snapshot = manager.build_snapshot()
        except Exception:
            return False
        for item in snapshot.get("history") or []:
            if not isinstance(item, dict):
                continue
            if str(item.get("source_tag") or "").strip() == "target_detection":
                return True
        return False

    def _path_deviation_option_pending_guard(self) -> tuple[tuple[object, ...], str] | None:
        pending_plan_id = None
        pending_plan_raw = getattr(self, "_pending_0702_plan_id", None)
        try:
            if pending_plan_raw is not None:
                pending_plan_id = int(pending_plan_raw)
        except Exception:
            pending_plan_id = None
        if pending_plan_id is not None and pending_plan_id <= 0:
            pending_plan_id = None
        if pending_plan_id is not None:
            return (
                ("pending_0702_apply", int(pending_plan_id)),
                f"[0401] path-deviation trigger suppressed: 0702 apply pending (missionPlanID={pending_plan_id})",
            )

        manager = getattr(self, "_replan_queue_manager", None)
        if manager is None:
            return None
        try:
            snapshot = manager.build_snapshot()
        except Exception:
            return None
        active = snapshot.get("active")
        if not isinstance(active, dict):
            return None
        stage = str(active.get("stage") or "").strip()
        if stage not in {"options_requested", "options_sent"}:
            return None
        queue_id = 0
        try:
            queue_id_raw = active.get("queue_id")
            if queue_id_raw is not None:
                queue_id = max(0, int(queue_id_raw))
        except Exception:
            queue_id = 0
        source_tag = str(active.get("source_tag") or "").strip() or "-"
        stage_label = str(active.get("stage_label") or stage or "-").strip()
        return (
            ("option_pending", int(queue_id), stage, source_tag),
            "[0401] path-deviation trigger suppressed: option pending until 0702 "
            f"(queueID={queue_id}, source={source_tag}, stage={stage_label})",
        )

    def _path_deviation_cooldown_guard(self, *, now_ms: int) -> tuple[tuple[object, ...], str] | None:
        guard_until_ms = 0
        try:
            raw_guard_until_ms = getattr(self, "_path_deviation_guard_until_ms", None)
            if raw_guard_until_ms is not None:
                guard_until_ms = max(0, int(raw_guard_until_ms))
        except Exception:
            guard_until_ms = 0
        remaining_ms = int(guard_until_ms) - int(now_ms)
        if remaining_ms <= 0:
            return None
        return (
            ("post_0702_cooldown", int(guard_until_ms)),
            "[0401] path-deviation trigger suppressed: within 3s after 0702 apply "
            f"(remaining={remaining_ms}ms)",
        )

    def _path_deviation_trigger_guard(self) -> tuple[tuple[object, ...], str] | None:
        pending_guard = self._path_deviation_option_pending_guard()
        if pending_guard is not None:
            return pending_guard
        return self._path_deviation_cooldown_guard(now_ms=int(_now_ms_since_2000()))

    def _update_path_deviation_guard_notice(
        self,
        guard: tuple[tuple[object, ...], str] | None,
    ) -> None:
        if guard is None:
            self._path_deviation_guard_notice_key = None
            return
        notice_key, message = guard
        if notice_key == getattr(self, "_path_deviation_guard_notice_key", None):
            return
        self._path_deviation_guard_notice_key = notice_key
        self._append_log_line(str(message))

    def _current_plan_is_attack_specialized(self) -> bool:
        plan_id = self._current_dispatch_plan_id()
        if plan_id is None:
            return False
        meta = self._plan_option_meta(plan_id)
        if not isinstance(meta, dict):
            return False
        option_name = self._normalized_replan_option_name(meta.get("optionName"))
        if option_name == "공격특화":
            return True
        detail = meta.get("replanDetail")
        if isinstance(detail, dict):
            trigger = str(detail.get("trigger") or detail.get("triggerType") or "").strip()
            if trigger == "0402" and option_name in {"공격특화", "공격배제"}:
                return True
        return False

    def _prepare_0902_payload_for_dispatch(self, payload: dict) -> tuple[dict, list[int]]:
        prepared = copy.deepcopy(payload) if isinstance(payload, dict) else {}
        bundle_target_ids: list[int] = []
        detail = prepared.get("replanDetail")
        if not isinstance(detail, dict):
            detail = {}

        trigger = str(detail.get("trigger") or detail.get("triggerType") or "").strip()
        trigger_type = str(detail.get("triggerType") or "").strip()
        prior_mission_id = self._extract_positive_int(detail.get("priorMissionID"))
        try:
            replan_level = int(prepared.get("replanLevel"))
        except (TypeError, ValueError):
            replan_level = 0
        prior_mission_list = prepared.get("priorMissionList")
        is_prior_start = (
            replan_level == 4
            and prior_mission_id is not None
            and isinstance(prior_mission_list, list)
            and bool(prior_mission_list)
        )
        if is_prior_start:
            current_plan_id = self._current_dispatch_plan_id()
            previous_source_plan_id = self._extract_positive_int(
                detail.get("sourceMissionPlanID") or prepared.get("sourceMissionPlanID")
            )
            if current_plan_id is not None:
                prepared["sourceMissionPlanID"] = int(current_plan_id)
                prepared["currentMissionPlanID"] = int(current_plan_id)
                detail["sourceMissionPlanID"] = int(current_plan_id)
                detail["currentMissionPlanID"] = int(current_plan_id)
                option_list = prepared.get("pendingOptionList") or prepared.get("optionList") or []
                for option in option_list:
                    if not isinstance(option, dict):
                        continue
                    option_plan_id = self._extract_positive_int(option.get("missionPlanID"))
                    if option_plan_id is None:
                        continue
                    try:
                        prior_replan_store.save_detail(int(option_plan_id), dict(detail))
                    except Exception:
                        pass
                if (
                    previous_source_plan_id is not None
                    and int(previous_source_plan_id) != int(current_plan_id)
                ):
                    self._append_log_line(
                        "[RQUEUE] prior-mission replan sourcePlan rebound: "
                        f"{previous_source_plan_id} -> {current_plan_id} (applied plan)"
                    )
        elif trigger == "0402":
            current_plan_id = self._current_dispatch_plan_id()
            previous_source_plan_id = self._extract_positive_int(
                detail.get("sourceMissionPlanID") or prepared.get("sourceMissionPlanID")
            )
            if current_plan_id is not None:
                prepared["sourceMissionPlanID"] = int(current_plan_id)
                prepared["currentMissionPlanID"] = int(current_plan_id)
                detail["sourceMissionPlanID"] = int(current_plan_id)
                detail["currentMissionPlanID"] = int(current_plan_id)
                if (
                    trigger_type != "attackClosedDestroyed"
                    and previous_source_plan_id is not None
                    and int(previous_source_plan_id) != int(current_plan_id)
                ):
                    self._append_log_line(
                        f"[RQUEUE] 0402 target replan sourcePlan rebound: "
                        f"{previous_source_plan_id} -> {current_plan_id} (applied plan)"
                    )

            if trigger_type == "attackClosedDestroyed":
                prepared["replanDetail"] = detail
                return prepared, bundle_target_ids

            preferred_target_id = self._extract_positive_int(detail.get("targetID") or detail.get("targetId"))
            follow_up_attack = self._current_plan_is_attack_specialized()
            requested_bundle_ids: list[int] = []
            requested_bundle_raw = detail.get("attackTargetList")
            if not isinstance(requested_bundle_raw, list) or not requested_bundle_raw:
                requested_bundle_raw = detail.get("targetBundle")
            if isinstance(requested_bundle_raw, list):
                for item in requested_bundle_raw:
                    if not isinstance(item, dict):
                        continue
                    target_id = self._extract_positive_int(item.get("targetID") or item.get("targetId"))
                    if target_id is not None and target_id not in requested_bundle_ids:
                        requested_bundle_ids.append(target_id)
            try:
                target_info = load_target_info()
            except Exception:
                target_info = {}
            bundle_limit = 3 if follow_up_attack else max(1, min(3, len(requested_bundle_ids) or 1))
            bundle = build_target_bundle_from_target_info(
                target_info,
                preferred_target_id=preferred_target_id,
                limit=bundle_limit,
            )
            if not bundle and isinstance(requested_bundle_raw, list):
                bundle = [dict(item) for item in requested_bundle_raw if isinstance(item, dict)][:bundle_limit]
            if bundle:
                detail["targetBundle"] = bundle
                detail["targetBundleCount"] = len(bundle)
                detail["targetBundleMode"] = (
                    "follow_up" if follow_up_attack else ("bundle" if len(bundle) > 1 else "single")
                )
                detail["followUpAttackMode"] = bool(follow_up_attack)
                detail["maxTrackedTargets"] = 3
                detail["attackTargetList"] = list(bundle)
                detail["attackTargetCount"] = len(bundle)
                bundle_ids: list[int] = []
                for item in bundle:
                    if not isinstance(item, dict):
                        continue
                    target_id = self._extract_positive_int(item.get("targetID"))
                    if target_id is not None and target_id not in bundle_ids:
                        bundle_ids.append(target_id)
                if bundle_ids:
                    detail["attackTargetIDs"] = list(bundle_ids)
                prepared["targetBundle"] = bundle
                if follow_up_attack:
                    bundle_target_ids = []
                    for item in bundle:
                        target_id = self._extract_positive_int(item.get("targetID"))
                        if target_id is not None and target_id not in bundle_target_ids:
                            bundle_target_ids.append(target_id)
            else:
                detail.pop("targetBundle", None)
                detail.pop("targetBundleCount", None)
                detail.pop("targetBundleMode", None)
                detail["followUpAttackMode"] = False
        elif trigger in {"0401", "pathDeviation"}:
            current_plan_id = self._current_dispatch_plan_id()
            if current_plan_id is not None:
                prepared["sourceMissionPlanID"] = int(current_plan_id)
                prepared["currentMissionPlanID"] = int(current_plan_id)
                detail["sourceMissionPlanID"] = int(current_plan_id)
                detail["currentMissionPlanID"] = int(current_plan_id)
        prepared["replanDetail"] = detail
        return prepared, bundle_target_ids

    def _queue_replan_payloads(self, payloads: list[dict] | tuple[dict, ...], *, source: str) -> None:
        manager = getattr(self, "_replan_queue_manager", None)
        valid_payloads = [dict(item) for item in (payloads or []) if isinstance(item, dict)]
        if not valid_payloads:
            return
        if manager is None:
            for payload in valid_payloads:
                self._push_0902_now(payload)
            return
        logs = manager.enqueue_payloads(valid_payloads, source=source)
        for line in logs:
            self._append_log_line(line)
        self._refresh_replan_queue_snapshot()
        self._schedule_replan_queue_drain()

    @staticmethod
    def _is_post_attack_close_payload(payload: object) -> bool:
        if not isinstance(payload, dict):
            return False
        detail = payload.get("replanDetail")
        if not isinstance(detail, dict):
            return False
        trigger_type = str(detail.get("triggerType") or "").strip()
        return trigger_type == "attackClosedDestroyed"

    def _queue_0402_replan_payloads(self, payloads: list[dict] | tuple[dict, ...]) -> None:
        post_attack_payloads: list[dict] = []
        target_payloads: list[dict] = []
        other_payloads: list[dict] = []
        for payload in payloads or []:
            if not isinstance(payload, dict):
                continue
            if self._is_post_attack_close_payload(payload):
                post_attack_payloads.append(dict(payload))
            else:
                detail = payload.get("replanDetail")
                trigger = str((detail or {}).get("trigger") or "").strip() if isinstance(detail, dict) else ""
                if trigger == "0402":
                    target_payloads.append(dict(payload))
                else:
                    other_payloads.append(dict(payload))

        if post_attack_payloads and target_payloads:
            self._append_log_line(
                "[RQUEUE] post-attack close queued before target attack replan; "
                "target replan will dispatch from the applied plan"
            )
        if post_attack_payloads:
            self._queue_replan_payloads(post_attack_payloads, source="post_attack_rejoin")
        if target_payloads:
            self._queue_replan_payloads(target_payloads, source="target_detection")
        if other_payloads:
            self._queue_replan_payloads(other_payloads, source="target_detection")

    def _maybe_resume_deferred_attack_replans(self, reason: str) -> None:
        if not bool(getattr(self, "_target_detection_replan_enabled", False)):
            return
        coord = getattr(self, "_target_detection_coord", None)
        if coord is None or not hasattr(coord, "resume_deferred_attacks"):
            return
        try:
            replan_payloads, logs = coord.resume_deferred_attacks(
                system_mode=self._system_mode_code,
                current_mission_plan_id=self._current_mission_plan_id,
            )
        except Exception as exc:
            self._append_log_line(f"[0402] deferred attack resume failed ({reason}): {exc}")
            return
        for line in logs:
            self._append_log_line(line)
        if not replan_payloads:
            return
        self._append_log_line(
            f"[0402] deferred attack resume requested after {reason}: count={len(replan_payloads)}"
        )
        self._queue_0402_replan_payloads(replan_payloads)

    def _schedule_replan_queue_drain(self, delay_ms: int = 0) -> None:
        if not self._is_ui_thread():
            self._invoke_on_ui_thread(lambda: self._schedule_replan_queue_drain(delay_ms=delay_ms))
            return
        try:
            delay_value = max(0, int(delay_ms))
        except Exception:
            delay_value = 0
        if delay_value > 0:
            hold_until = int(time.time() * 1000) + int(delay_value)
            self._replan_queue_drain_not_before_ms = max(
                int(getattr(self, "_replan_queue_drain_not_before_ms", 0) or 0),
                int(hold_until),
            )
        # Avoid re-entrant 0902 sends while we are still inside an incoming
        # message callback (notably 0001 option-suppression after 0402 bursts).
        if self._replan_queue_draining:
            self._replan_queue_drain_scheduled = True
            return
        if self._replan_queue_drain_scheduled:
            return
        self._replan_queue_drain_scheduled = True
        hold_remaining_ms = max(
            0,
            int(getattr(self, "_replan_queue_drain_not_before_ms", 0) or 0) - int(time.time() * 1000),
        )
        QTimer.singleShot(max(delay_value, hold_remaining_ms), self._drain_replan_queue)

    def _drain_replan_queue(self) -> None:
        if not self._is_ui_thread():
            self._invoke_on_ui_thread(self._drain_replan_queue)
            return
        if self._replan_queue_draining:
            self._replan_queue_drain_scheduled = True
            return
        manager = getattr(self, "_replan_queue_manager", None)
        if manager is None:
            self._replan_queue_drain_scheduled = False
            return
        hold_remaining_ms = max(
            0,
            int(getattr(self, "_replan_queue_drain_not_before_ms", 0) or 0) - int(time.time() * 1000),
        )
        if hold_remaining_ms > 0:
            self._replan_queue_drain_scheduled = True
            QTimer.singleShot(hold_remaining_ms, self._drain_replan_queue)
            return
        self._replan_queue_drain_scheduled = False
        self._replan_queue_draining = True
        try:
            dispatched = manager.try_activate_next()
            self._refresh_replan_queue_snapshot()
            if dispatched is None:
                return
            queue_id, payload = dispatched
            prepared_payload, bundle_target_ids = self._prepare_0902_payload_for_dispatch(payload)
            sent = self._push_0902_now(prepared_payload, already_prepared=True)
            logs = manager.mark_dispatch_result(queue_id, sent=bool(sent), error=None if sent else "0902 send failed")
            for line in logs:
                self._append_log_line(line)
            if sent and bundle_target_ids and str((prepared_payload.get("replanDetail") or {}).get("targetBundleMode") or "") == "follow_up":
                try:
                    drop_logs = manager.drop_queued_target_detection_targets(bundle_target_ids)
                except Exception as exc:
                    drop_logs = [f"[RQUEUE] bundle drop failed: {exc}"]
                for line in drop_logs:
                    self._append_log_line(line)
                bundle_entries = (prepared_payload.get("replanDetail") or {}).get("targetBundle")
                if isinstance(bundle_entries, list) and bundle_entries:
                    try:
                        mark_targets_as_used(bundle_entries)
                        self._append_log_line(
                            f"[RQUEUE] bundle targets marked used: count={len(bundle_entries)}"
                        )
                    except Exception as exc:
                        self._append_log_line(f"[RQUEUE] bundle mark-used failed: {exc}")
                self._refresh_replan_queue_snapshot()
            self._refresh_replan_queue_snapshot()
            if not sent:
                self._schedule_replan_queue_drain()
        except Exception as exc:
            self._append_log_line(f"[RQUEUE] drain error: {exc}")
        finally:
            self._replan_queue_draining = False
            if self._replan_queue_drain_scheduled:
                hold_remaining_ms = max(
                    0,
                    int(getattr(self, "_replan_queue_drain_not_before_ms", 0) or 0) - int(time.time() * 1000),
                )
                QTimer.singleShot(hold_remaining_ms, self._drain_replan_queue)

    def _push_0902_now(self, payload: dict, *, already_prepared: bool = False) -> bool:
        try:
            from push_center import push_message
        except Exception as exc:
            self._append_log_line(f"[0902] push import failed: {exc}")
            return False

        prepared_payload = payload if already_prepared else self._prepare_0902_payload_for_dispatch(payload)[0]

        tab = getattr(self, "_tab", None)
        row = self._find_tx_row("0902")
        on_done = None
        if row >= 0 and tab is not None and hasattr(tab, "_mark_single_sent"):
            on_done = (lambda mid, raw: tab._mark_single_sent(row, mid, raw))

        try:
            ok = push_message("0902", NodeMessenger, body_dict=prepared_payload, on_done=on_done)
        except Exception as exc:
            self._append_log_line(f"[0902] send error: {exc}")
            return False
        if not ok:
            self._append_log_line("[0902] send failed")
            return False

        level = prepared_payload.get("replanLevel")
        reason = prepared_payload.get("replanRequest") or prepared_payload.get("replanReason")
        options = prepared_payload.get("pendingOptionList") or prepared_payload.get("optionList") or []
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
                "replanDetail": dict(prepared_payload.get("replanDetail") or {})
                if isinstance(prepared_payload.get("replanDetail"), dict)
                else {},
                "timestamp": prepared_payload.get("timestamp"),
            }
        if not plan_ids:
            for item in prepared_payload.get("missionPlanIDList") or []:
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
                    "replanDetail": dict(prepared_payload.get("replanDetail") or {})
                    if isinstance(prepared_payload.get("replanDetail"), dict)
                    else {},
                    "timestamp": prepared_payload.get("timestamp"),
                }
        plan_summary = ", ".join(str(pid) for pid in plan_ids) if plan_ids else "-"
        self._append_log_line(
            f"[0902] replan request sent (level={level}, reason={reason}, planIds={plan_summary})"
        )
        return True

    def _send_0902(self, payload: dict) -> None:
        self._queue_replan_payloads([payload], source="manual")

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
            raw_aid = state.get("aircraft_id")
            if raw_aid is None:
                continue
            try:
                aircraft_id = int(raw_aid)
            except (TypeError, ValueError):
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

    def _clear_all_attack_tracking_for_exclusion(self) -> list[dict[str, int | None]]:
        try:
            from modules.mission_planning.runtime.state.attack_tracking import (
                clear_tracking_assignment,
                list_active_tracking_assignments,
            )
        except Exception as exc:
            self._append_log_line(f"[0702] 공격 배제 추적 해제 모듈 로드 실패: {exc}")
            return []

        cleared_ids: list[int] = []
        tracking_targets: list[dict[str, int | None]] = []
        try:
            for assignment in list_active_tracking_assignments():
                if not isinstance(assignment, dict):
                    continue
                aircraft_id = self._extract_positive_int(assignment.get("aircraft_id"))
                assigned_target_id = self._extract_positive_int(assignment.get("target_id"))
                if aircraft_id is None:
                    continue
                if assigned_target_id is not None:
                    tracking_targets.append(
                        {
                            "targetID": int(assigned_target_id),
                            "watcherID": int(aircraft_id),
                        }
                    )
                clear_tracking_assignment(int(aircraft_id))
                cleared_ids.append(int(aircraft_id))
        except Exception as exc:
            self._append_log_line(f"[0702] 공격 배제 추적 해제 실패: {exc}")
            return tracking_targets

        if cleared_ids:
            aircraft_text = ",".join(str(aid) for aid in sorted(set(cleared_ids)))
            self._append_log_line(
                f"[0702] 전체 공격 배제 선택 -> 모든 UAV 추적 해제 반영 (aircraftID={aircraft_text})"
            )
        return tracking_targets

    def _release_attack_slots_for_exclusion(self, mission_plan_id: int) -> None:
        try:
            plan_path = mission_plan_json_path(int(mission_plan_id))
            if plan_path is None or not plan_path.exists():
                return
            plan_data = json.loads(plan_path.read_text(encoding="utf-8"))
            input_package_id = self._extract_positive_int(
                (plan_data or {}).get("inputMissionPackageID")
                if isinstance(plan_data, dict)
                else None
            )
            if input_package_id is None:
                return
            released_ids = release_manned_used(int(input_package_id))
            if released_ids:
                self._append_log_line(
                    "[0702] 전체 공격 배제 선택 -> 유인기 공격 슬롯 해제 "
                    f"(inputMissionPackageID={input_package_id}, aircraftID={released_ids})"
                )
        except Exception as exc:
            self._append_log_line(f"[0702] 공격 배제 유인기 슬롯 해제 실패: {exc}")

        try:
            attack_option_plan_ids = [
                int(plan_id)
                for plan_id, candidate_meta in self._replan_option_meta_by_plan_id.items()
                if isinstance(candidate_meta, dict)
                and self._normalized_replan_option_name(candidate_meta.get("optionName")) == "공격특화"
            ]
            cleared_plan_ids = clear_pending_manned_assignments(attack_option_plan_ids)
            if cleared_plan_ids:
                self._append_log_line(
                    "[0702] 전체 공격 배제 선택 -> 미선택 공격안 슬롯 예약 해제 "
                    f"(planIDs={sorted(cleared_plan_ids)})"
                )
        except Exception as exc:
            self._append_log_line(f"[0702] 공격 배제 pending 슬롯 정리 실패: {exc}")

    def _apply_attack_exclusion_ignore(self, mission_plan_id: int) -> bool:
        meta = self._plan_option_meta(mission_plan_id)
        if not isinstance(meta, dict):
            return False
        option_name = self._normalized_replan_option_name(meta.get("optionName"))
        if option_name != "공격배제":
            return False
        detail = meta.get("replanDetail")
        if not isinstance(detail, dict):
            detail = {}
        target_entries: list[dict[str, int | None]] = []
        seen_target_ids: set[int] = set()
        bundle = detail.get("targetBundle")
        if isinstance(bundle, list):
            for item in bundle:
                if not isinstance(item, dict):
                    continue
                target_id = self._extract_positive_int(item.get("targetID"))
                if target_id is None or target_id in seen_target_ids:
                    continue
                seen_target_ids.add(target_id)
                target_entries.append(
                    {
                        "targetID": int(target_id),
                        "watcherID": self._extract_positive_int(item.get("watcherID")),
                    }
                )
        if not target_entries:
            target_id = self._extract_positive_int(detail.get("targetID") or detail.get("targetId"))
            if target_id is not None:
                target_entries.append(
                    {
                        "targetID": int(target_id),
                        "watcherID": self._extract_positive_int(detail.get("watcherID") or detail.get("watcherId")),
                    }
                )

        # "공격 배제"는 현재 후보만이 아니라 진행 중인 공격 전체를 취소한다.
        # 이미 공격에 사용된 표적과 활성 추적 표적도 함께 무시 처리해야 이후
        # 0402에서 같은 표적이 다시 공격 후보로 살아나지 않는다.
        target_info = load_target_info()
        target_map = target_info.get("targetList") if isinstance(target_info, dict) else None
        if isinstance(target_map, dict):
            for item in target_map.values():
                if not isinstance(item, dict):
                    continue
                target_id = self._extract_positive_int(item.get("targetID"))
                if target_id is None or bool(item.get("isDestroyed")):
                    continue
                try:
                    is_used = int(item.get("isUsed") or 0) != 0
                except Exception:
                    is_used = False
                if not is_used:
                    continue
                target_entries.append(
                    {
                        "targetID": int(target_id),
                        "watcherID": self._extract_positive_int(item.get("watcherID")),
                    }
                )
        target_entries.extend(self._clear_all_attack_tracking_for_exclusion())
        deduped_targets: dict[int, dict[str, int | None]] = {}
        for item in target_entries:
            if not isinstance(item, dict):
                continue
            target_id = self._extract_positive_int(item.get("targetID"))
            if target_id is None:
                continue
            existing = deduped_targets.get(int(target_id))
            if existing is None or existing.get("watcherID") is None:
                deduped_targets[int(target_id)] = {
                    "targetID": int(target_id),
                    "watcherID": self._extract_positive_int(item.get("watcherID")),
                }
        target_entries = list(deduped_targets.values())
        try:
            mark_targets_as_ignored(target_entries)
            self._append_log_line(
                f"[0702] 전체 공격 배제 선택 -> 공격/추적 표적 isIgnored=1 반영 (count={len(target_entries)})"
            )
        except Exception as exc:
            self._append_log_line(
                f"[0702] 공격 배제 ignore 반영 실패: {exc}"
            )
        self._release_attack_slots_for_exclusion(int(mission_plan_id))
        return True

    def _commit_pending_attack_slot(self, mission_plan_id: int) -> None:
        try:
            aircraft_ids = commit_pending_manned_assignments(mission_plan_id)
        except Exception as exc:
            self._append_log_line(f"[0702] 공격 슬롯 점유 반영 실패: {exc}")
            return
        if not aircraft_ids:
            aircraft_id = commit_pending_manned_assignment(mission_plan_id)
            aircraft_ids = [aircraft_id] if aircraft_id is not None else []
        for aircraft_id in aircraft_ids:
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
            raw_aid = state.get("aircraft_id")
            if raw_aid is None:
                continue
            try:
                aircraft_id = int(raw_aid)
            except (TypeError, ValueError):
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
        text = limit_utf8_bytes(contents)
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
                    self.host._record_rx_listener_event(str(msg_id or "0101"), raw)
                    self.host._invoke_on_ui_thread(self.host._on_rx_0101, raw)
                except Exception:
                    pass

        try:
            self._rx0101 = _Rx0101(self)
            register_listener("0101", self._rx0101)
        except Exception as exc:
            self._append_log_line(f"[0101] 리스너 등록 실패: {exc}")

    def _install_0305_listener(self) -> None:
        def _rx_0305(_msg_id: str, payload: object | None):
            try:
                self._record_rx_listener_event("0305", payload)
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    last_raw = getattr(self, "_last_0305_raw", None)
                    if last_raw is not None and raw_latest == last_raw:
                        return
                    self._last_0305_raw = raw_latest
                    self._invoke_on_ui_thread(self._on_rx_0305, raw_latest)
                    return
                self._invoke_on_ui_thread(self._on_rx_0305, payload)
            except Exception:
                pass

        try:
            self._rx0305_handler = _rx_0305
            register_listener("0305", self._rx0305_handler)
        except Exception as exc:
            self._append_log_line(f"[0305] 리스너 등록 실패: {exc}")

    def _install_0701_listener(self) -> None:
        def _rx_0701(_msg_id: str, payload: object | None):
            try:
                self._record_rx_listener_event("0701", payload)
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    last_raw = getattr(self, "_last_0701_raw", None)
                    if last_raw is not None and raw_latest == last_raw:
                        return
                    self._last_0701_raw = raw_latest
                    self._invoke_on_ui_thread(self._on_rx_0701, raw_latest)
                    return
                self._invoke_on_ui_thread(self._on_rx_0701, payload)
            except Exception:
                pass

        try:
            self._rx0701_handler = _rx_0701
            register_listener("0701", self._rx0701_handler)
        except Exception as exc:
            self._append_log_line(f"[0701] 리스너 등록 실패: {exc}")

    def _install_0001_listener(self) -> None:
        def _rx_0001(_msg_id: str, payload: object | None):
            try:
                self._record_rx_listener_event("0001", payload)
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    last_raw = getattr(self, "_last_0001_raw", None)
                    if last_raw is not None and raw_latest == last_raw:
                        return
                    self._last_0001_raw = raw_latest
                    self._invoke_on_ui_thread(self._on_rx_0001, raw_latest)
                    return
                self._invoke_on_ui_thread(self._on_rx_0001, payload)
            except Exception:
                pass

        try:
            self._rx0001_handler = _rx_0001
            register_listener("0001", self._rx0001_handler)
        except Exception as exc:
            self._append_log_line(f"[0001] 리스너 등록 실패: {exc}")

    def _install_0903_listener(self) -> None:
        def _rx_0903(_msg_id: str, payload: object | None):
            try:
                self._record_rx_listener_event("0903", payload)
                self._invoke_on_ui_thread(self._on_rx_0903, payload)
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
                self._record_rx_listener_event("0702", payload)
                self._invoke_on_ui_thread(self._on_rx_0702, payload)
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
                self._record_rx_listener_event("0201", payload)
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    self._last_0201_raw = raw_latest
                    try:
                        self._last_0201_ms = int(time.time() * 1000)
                    except Exception:
                        pass
                self._invoke_on_ui_thread(self._on_rx_0201, payload)
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
                self._record_rx_listener_event("0202", payload)
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    self._last_0202_raw = raw_latest
                    try:
                        self._last_0202_ms = int(time.time() * 1000)
                    except Exception:
                        pass
                self._invoke_on_ui_thread(self._on_rx_0202, payload)
            except Exception:
                pass

        try:
            self._rx0202_handler = _rx_0202
            register_listener("0202", self._rx0202_handler)
        except Exception as exc:
            self._append_log_line(f"[0202] listener registration failed: {exc}")

    def _init_0401_dispatcher(self) -> None:
        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.setInterval(int(getattr(self, "_0401_coalesce_ms", 80) or 80))
        timer.timeout.connect(self._drain_0401_payload)
        self._0401_drain_timer = timer

    def _init_0402_dispatcher(self) -> None:
        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.setInterval(int(getattr(self, "_0402_coalesce_ms", 120) or 120))
        timer.timeout.connect(self._drain_0402_payload)
        self._0402_drain_timer = timer

    def _payload_signature(self, payload: object | None) -> bytes | None:
        signature, _raw_body = self._payload_signature_context(payload)
        return signature

    def _payload_signature_context(self, payload: object | None) -> tuple[bytes | None, dict[str, Any] | None]:
        return payload_signature_context(payload)

    def _build_0401_dispatch_context(
        self,
        *,
        payload: object | None,
        raw_body: dict[str, Any] | None,
        canonical_signature: bytes | None,
        timestamp_ms: int | None,
        agent_states: list[dict[str, Any]] | None,
    ) -> Replan0401DispatchContext:
        settings_snapshot = {
            "system_mode": int(getattr(self, "_system_mode_code", 0) or 0),
            "current_mission_plan_id": getattr(self, "_current_mission_plan_id", None),
            "coalesce_ms": int(getattr(self, "_0401_coalesce_ms", 80) or 80),
            "toggles": {
                "prior_mission": bool(getattr(self, "_prior_mission_replan_enabled", False)),
                "rtb": bool(getattr(self, "_rtb_replan_enabled", False)),
                "path_deviation": bool(getattr(self, "_path_deviation_trigger_enabled", False)),
                "quality_monitor": bool(getattr(self, "_quality_monitor_enabled", False)),
                "quality_speed": bool(getattr(self, "_quality_speed_replan_enabled", False)),
                "imaging_schedule": bool(getattr(self, "_schedule_replan_trigger_enabled", False)),
            },
        }
        return Replan0401DispatchContext(
            raw_payload=payload,
            parsed_body=dict(raw_body) if isinstance(raw_body, dict) else None,
            canonical_signature=canonical_signature,
            timestamp_ms=timestamp_ms,
            agent_states=list(agent_states or []),
            settings_snapshot=settings_snapshot,
        )

    def _enqueue_0401_payload(self, payload: object | None) -> None:
        signature, raw_body = self._payload_signature_context(payload)
        with self._0401_pending_lock:
            if signature is not None and signature == self._0401_last_signature:
                self._record_rx_enqueue_event("0401", duplicate_signature=1, skipped=1)
                return
            self._0401_last_signature = signature
            self._0401_pending_payload = payload
            self._0401_pending_raw_body = raw_body
            self._0401_pending_signature = signature
            if self._0401_pending_scheduled:
                self._record_rx_enqueue_event("0401", coalesced_pending=1, scheduled=1)
                return
            self._0401_pending_scheduled = True
        self._record_rx_enqueue_event("0401", accepted=1, scheduled=1)
        self._invoke_on_ui_thread(self._schedule_0401_drain)

    def _schedule_0401_drain(self) -> None:
        timer = getattr(self, "_0401_drain_timer", None)
        if timer is None:
            self._drain_0401_payload()
            return
        if not timer.isActive():
            timer.start(int(getattr(self, "_0401_coalesce_ms", 80) or 80))

    def _drain_0401_payload(self) -> None:
        with self._0401_pending_lock:
            payload = self._0401_pending_payload
            raw_body = self._0401_pending_raw_body
            signature = self._0401_pending_signature
            self._0401_pending_payload = None
            self._0401_pending_raw_body = None
            self._0401_pending_signature = None
            self._0401_pending_scheduled = False
        if payload is None:
            return
        self._on_rx_0401(payload, raw_body=raw_body, canonical_signature=signature)
        should_reschedule = False
        with self._0401_pending_lock:
            if self._0401_pending_payload is not None and not self._0401_pending_scheduled:
                self._0401_pending_scheduled = True
                should_reschedule = True
        if should_reschedule:
            self._schedule_0401_drain()

    def _enqueue_0402_payload(self, payload: object | None) -> None:
        signature = self._payload_signature(payload)
        destroyed_target_ids = destroyed_target_ids_from_message(payload)
        with self._0402_pending_lock:
            if signature is not None and signature == self._0402_last_signature:
                self._record_rx_enqueue_event("0402", duplicate_signature=1, skipped=1)
                return
            self._0402_last_signature = signature
            new_destroyed_target_ids = (
                set(destroyed_target_ids) - self._0402_destroyed_target_ids_seen
            )
            if new_destroyed_target_ids:
                # Destruction is a terminal state transition and must not be
                # overwritten by a newer routine 0402 sample during coalescing.
                self._0402_destroyed_target_ids_seen.update(new_destroyed_target_ids)
                self._0402_pending_terminal_payloads.append(payload)
            else:
                self._0402_pending_payload = payload
            if self._0402_pending_scheduled:
                self._record_rx_enqueue_event(
                    "0402",
                    coalesced_pending=0 if new_destroyed_target_ids else 1,
                    terminal_pending=1 if new_destroyed_target_ids else 0,
                    scheduled=1,
                )
                return
            self._0402_pending_scheduled = True
        self._record_rx_enqueue_event(
            "0402",
            accepted=1,
            terminal_pending=1 if new_destroyed_target_ids else 0,
            scheduled=1,
        )
        self._invoke_on_ui_thread(self._schedule_0402_drain)

    def _schedule_0402_drain(self) -> None:
        timer = getattr(self, "_0402_drain_timer", None)
        if timer is None:
            self._drain_0402_payload()
            return
        if not timer.isActive():
            timer.start(int(getattr(self, "_0402_coalesce_ms", 120) or 120))

    def _drain_0402_payload(self) -> None:
        with self._0402_pending_lock:
            if self._0402_pending_terminal_payloads:
                payload = self._0402_pending_terminal_payloads.pop(0)
            else:
                payload = self._0402_pending_payload
                self._0402_pending_payload = None
            self._0402_pending_scheduled = False
        if payload is None:
            return
        self._on_rx_0402(payload)
        should_reschedule = False
        with self._0402_pending_lock:
            if (
                self._0402_pending_terminal_payloads
                or self._0402_pending_payload is not None
            ) and not self._0402_pending_scheduled:
                self._0402_pending_scheduled = True
                should_reschedule = True
        if should_reschedule:
            self._schedule_0402_drain()

    def _install_0401_listener(self) -> None:
        def _rx_0401(_msg_id: str, payload: object | None):
            try:
                self._record_rx_listener_event("0401", payload)
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    self._last_0401_raw = raw_latest
                    self._enqueue_0401_payload(raw_latest)
                else:
                    self._enqueue_0401_payload(payload)
            except Exception:
                pass

        try:
            _rx_0401.receive_coalesce_messages = {"0401": self._0401_coalesce_ms}
            self._rx0401_handler = _rx_0401
            register_listener("0401", self._rx0401_handler)
        except Exception as exc:
            self._append_log_line(f"[0401] 리스너 등록 실패: {exc}")

    def _install_0402_listener(self) -> None:
        def _rx_0402(_msg_id: str, payload: object | None):
            try:
                self._record_rx_listener_event("0402", payload)
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
                self._enqueue_0402_payload(raw_latest if raw_latest else payload)
            except Exception:
                pass

        try:
            # Do not coalesce in receive_center: this class owns a terminal-aware
            # queue that preserves newly destroyed targets while still merging
            # ordinary high-rate 0402 samples.
            self._rx0402_handler = _rx_0402
            register_listener("0402", self._rx0402_handler)
        except Exception as exc:
            self._append_log_line(f"[0402] 리스너 등록 실패: {exc}")

    def _install_0802_listener(self) -> None:
        def _rx_0802(_msg_id: str, payload: object | None):
            try:
                self._record_rx_listener_event("0802", payload)
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    self._last_0802_raw = raw_latest
                    try:
                        self._last_0802_ms = int(time.time() * 1000)
                    except Exception:
                        pass
                self._invoke_on_ui_thread(self._on_rx_0802, payload)
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
                self._record_rx_listener_event("0803", payload)
                raw_latest = self._unwrap_payload(payload)
                if raw_latest:
                    last_raw = getattr(self, "_last_0803_raw", None)
                    if last_raw is not None and raw_latest == last_raw:
                        return
                    self._last_0803_raw = raw_latest
                    self._invoke_on_ui_thread(self._on_rx_0803, raw_latest)
                    return
                self._invoke_on_ui_thread(self._on_rx_0803, payload)
            except Exception:
                pass

        try:
            self._rx0803_handler = _rx_0803
            register_listener("0803", self._rx0803_handler)
        except Exception as exc:
            self._append_log_line(f"[0803] 리스너 등록 실패: {exc}")

    def _on_rx_0305(self, payload: object | None) -> None:
        manager = getattr(self, "_replan_queue_manager", None)
        if manager is None:
            return
        try:
            released, logs = manager.handle_0305(payload)
            for line in logs:
                self._append_log_line(line)
            self._recover_suppressed_target_replan(manager, "0305")
            self._refresh_replan_queue_snapshot()
            if released:
                self._schedule_replan_queue_drain()
        except Exception as exc:
            self._append_log_line(f"[RQUEUE] 0305 handling error: {exc}")

    def _on_rx_0701(self, payload: object | None) -> None:
        manager = getattr(self, "_replan_queue_manager", None)
        if manager is None:
            return
        try:
            released, logs = manager.handle_0701(payload)
            for line in logs:
                self._append_log_line(line)
            self._refresh_replan_queue_snapshot()
            if released:
                self._schedule_replan_queue_drain()
        except Exception as exc:
            self._append_log_line(f"[RQUEUE] 0701 handling error: {exc}")

    def _on_rx_0001(self, payload: object | None) -> None:
        manager = getattr(self, "_replan_queue_manager", None)
        if manager is None:
            return
        try:
            released, logs = manager.handle_0001(payload)
            for line in logs:
                self._append_log_line(line)
            self._recover_suppressed_target_replan(manager, "0001")
            self._refresh_replan_queue_snapshot()
            if released:
                self._schedule_replan_queue_drain()
        except Exception as exc:
            self._append_log_line(f"[RQUEUE] 0001 handling error: {exc}")

    def _extract_target_ids_from_replan_queue_item(self, item: object) -> set[int]:
        target_ids: set[int] = set()
        payload = getattr(item, "payload", None)
        detail = payload.get("replanDetail") if isinstance(payload, dict) and isinstance(payload.get("replanDetail"), dict) else {}

        raw_target_id = self._extract_positive_int(getattr(item, "target_id", None))
        if raw_target_id is not None:
            target_ids.add(int(raw_target_id))

        for raw in detail.get("attackTargetIDs") or []:
            target_id = self._extract_positive_int(raw)
            if target_id is not None:
                target_ids.add(int(target_id))

        for container in (
            detail.get("targetBundle"),
            detail.get("attackTargetList"),
            payload.get("targetBundle") if isinstance(payload, dict) else None,
            payload.get("attackTargetList") if isinstance(payload, dict) else None,
        ):
            if not isinstance(container, list):
                continue
            for entry in container:
                if not isinstance(entry, dict):
                    continue
                target_id = self._extract_positive_int(entry.get("targetID") or entry.get("targetId"))
                if target_id is not None:
                    target_ids.add(int(target_id))

        return target_ids

    def _recover_suppressed_target_replan(self, manager: object, signal_name: str) -> None:
        reset_target_ids = self._unmark_suppressed_target(manager)
        if not reset_target_ids:
            return

        coord = getattr(self, "_target_detection_coord", None)
        if coord is None:
            return
        try:
            clearer = getattr(coord, "clear_target_trigger_history", None)
            if callable(clearer):
                cleared_ids = clearer(reset_target_ids)
                if cleared_ids:
                    self._append_log_line(
                        "[RQUEUE] target trigger cooldown cleared for targetIDs="
                        + ",".join(str(target_id) for target_id in sorted(cleared_ids))
                        + f" (option_suppressed/{signal_name})"
                    )
        except Exception as exc:
            self._append_log_line(f"[RQUEUE] target trigger cooldown clear failed: {exc}")

        if not bool(getattr(self, "_target_detection_replan_enabled", False)):
            return
        if not hasattr(coord, "on_situation_awareness"):
            return
        try:
            replan_payloads, logs = coord.on_situation_awareness(
                None,
                system_mode=self._system_mode_code,
                current_mission_plan_id=self._current_mission_plan_id,
            )
        except Exception as exc:
            self._append_log_line(f"[RQUEUE] suppressed target retry build failed: {exc}")
            return
        for line in logs:
            self._append_log_line(line)
        if not replan_payloads:
            return
        self._append_log_line(
            "[RQUEUE] target replan retry queued after option_suppressed: targetIDs="
            + ",".join(str(target_id) for target_id in sorted(reset_target_ids))
        )
        self._queue_0402_replan_payloads(replan_payloads)

    def _unmark_suppressed_target(self, manager: object) -> set[int]:
        """option_suppressed로 완료된 항목의 표적 isUsed를 해제하여 다음 재계획 번들에 포함되게 한다."""
        reset_ids: set[int] = set()
        try:
            history = getattr(manager, "_history", None)
            if not history:
                return reset_ids
            last = history[0]
            if str(getattr(last, "status", "")) != "option_suppressed":
                return reset_ids
            plan_ids = [
                int(plan_id)
                for plan_id in (getattr(last, "plan_ids", None) or [])
                if self._extract_positive_int(plan_id) is not None
            ]
            cleared_plan_ids = clear_pending_manned_assignments(plan_ids)
            if cleared_plan_ids:
                self._append_log_line(
                    "[RQUEUE] pending attack slot assignment cleared for suppressed planIDs="
                    + ",".join(str(plan_id) for plan_id in sorted(cleared_plan_ids))
            )
            suppressed_target_ids = self._extract_target_ids_from_replan_queue_item(last)
            if not suppressed_target_ids:
                return reset_ids

            protected_target_ids: set[int] = set()
            active = getattr(manager, "_active", None)
            if active is not None and str(getattr(active, "source_tag", "")) == "target_detection":
                protected_target_ids.update(self._extract_target_ids_from_replan_queue_item(active))
            for queued in getattr(manager, "_queue", []) or []:
                if str(getattr(queued, "source_tag", "")) != "target_detection":
                    continue
                protected_target_ids.update(self._extract_target_ids_from_replan_queue_item(queued))

            reset_target_ids = {
                int(target_id)
                for target_id in suppressed_target_ids
                if int(target_id) not in protected_target_ids
            }
            if not reset_target_ids:
                return reset_ids

            info = load_target_info()
            target_map = info.get("targetList")
            if not isinstance(target_map, dict):
                return reset_ids
            changed = False
            for entry in target_map.values():
                if not isinstance(entry, dict):
                    continue
                entry_tid = self._extract_positive_int(entry.get("targetID"))
                if entry_tid is None:
                    continue
                if entry_tid in reset_target_ids and entry.get("isUsed") == 1:
                    entry["isUsed"] = 0
                    reset_ids.add(int(entry_tid))
                    changed = True
            if changed:
                from modules.monitoring.logic.target_info import save_target_info
                save_target_info(info)
                self._append_log_line(
                    "[RQUEUE] target isUsed reset for targetIDs="
                    + ",".join(str(target_id) for target_id in sorted(reset_ids))
                    + " (option_suppressed)"
                )
        except Exception as exc:
            self._append_log_line(f"[RQUEUE] target unmark failed: {exc}")
        return reset_ids

    def _on_rx_0903(self, payload: object | None) -> None:
        ts, mpid, source, _body = extract_0903_info(payload)
        if mpid != self._current_mission_plan_id:
            self._current_mission_plan_id = mpid
            self._active_0401_notice_keys.clear()
        if mpid is None:
            return
        generation = self._next_plan_apply_generation()
        self._line_scan_apply_mission_plan(mpid)
        self._queue_area_snapshot_plan_update(
            plan_id=mpid,
            timestamp_ms=ts,
            source=source,
            prefer_apply=False,
        )
        viz = getattr(self, "_viz_tab", None)
        if viz is None or not hasattr(viz, "update_0903"):
            viz = None
        viz_applied = False
        try:
            if viz is not None:
                viz.update_0903(timestamp_ms=ts, mission_plan_id=mpid, source=source)
                viz_applied = True
        except Exception:
            pass
        if viz_applied:
            self._kick_0501_sender()
        self._line_scan_apply_mission_plan(mpid)
        self._queue_area_snapshot_plan_update(
            plan_id=mpid,
            timestamp_ms=ts,
            source=source,
            prefer_apply=True,
        )
        self._defer_plan_tab_updates(
            plan_id=mpid,
            timestamp_ms=ts,
            source=source,
            generation=generation,
            prefer_apply=False,
            log_prefix="[0903]",
            tab_jobs=(
                ("_schedule_tab", "schedule tab", 20),
                ("_quality_tab", "quality tab", 100),
            ),
        )
        try:
            quality_speed_coord = getattr(self, "_quality_speed_coord", None)
            if quality_speed_coord is not None and hasattr(quality_speed_coord, "apply_mission_plan_decision"):
                quality_speed_coord.apply_mission_plan_decision(mpid)
        except Exception:
            pass
        try:
            manager = getattr(self, "_replan_queue_manager", None)
            if manager is not None:
                released, logs = manager.handle_0903(payload)
                for line in logs:
                    self._append_log_line(line)
                self._refresh_replan_queue_snapshot()
                if released:
                    self._schedule_replan_queue_drain()
                self._maybe_resume_deferred_attack_replans("0903")
        except Exception as exc:
            self._append_log_line(f"[RQUEUE] 0903 handling error: {exc}")

    def _handle_replan_queue_0702(self, payload: object | None) -> None:
        manager = getattr(self, "_replan_queue_manager", None)
        if manager is None:
            return
        try:
            released, logs = manager.handle_0702(payload)
            for line in logs:
                self._append_log_line(line)
            self._refresh_replan_queue_snapshot()
            if released:
                self._schedule_replan_queue_drain(delay_ms=1000)
            self._maybe_resume_deferred_attack_replans("0702")
        except Exception as exc:
            self._append_log_line(f"[RQUEUE] 0702 handling error: {exc}")

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
            self._handle_replan_queue_0702(payload)
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
            self._handle_replan_queue_0702(payload)
            return

        manager = getattr(self, "_replan_queue_manager", None)
        if manager is not None and hasattr(manager, "validate_0702_decision"):
            try:
                accepted, validation_logs = manager.validate_0702_decision(
                    mission_plan_id=plan_id_int,
                    ignore_value=ignore_val,
                )
            except Exception as exc:
                accepted, validation_logs = True, [f"[RQUEUE] 0702 validation failed: {exc}"]
            for line in validation_logs:
                self._append_log_line(line)
            if not accepted:
                detail = f"시간: {ts_text}\nmissionPlanID: {plan_id_int}"
                if source:
                    detail = f"{detail}\nsource: {source}"
                self._update_0702_status(status="무시됨(stale)", detail=detail)
                self._refresh_replan_queue_snapshot()
                return

        applied = self._try_apply_0702_plan(
            plan_id_int,
            timestamp_ms=ts,
            source=source,
            decision_key=decision_key,
            pending_resolved=False,
        )
        if applied:
            self._handle_replan_queue_0702(payload)
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
        manager = getattr(self, "_replan_queue_manager", None)
        if manager is not None and hasattr(manager, "validate_0702_decision"):
            try:
                accepted, validation_logs = manager.validate_0702_decision(
                    mission_plan_id=int(plan_id),
                    ignore_value=2,
                )
            except Exception as exc:
                accepted, validation_logs = True, [f"[RQUEUE] pending 0702 validation failed: {exc}"]
            for line in validation_logs:
                self._append_log_line(line)
            if not accepted:
                self._update_0702_status(
                    status="무시됨(stale)",
                    detail=f"시간: {format_timestamp_ms(ts)}\nmissionPlanID: {int(plan_id)}",
                )
                self._clear_pending_0702()
                self._refresh_replan_queue_snapshot()
                return
        applied = self._try_apply_0702_plan(
            int(plan_id),
            timestamp_ms=ts,
            source=source,
            decision_key=decision_key,
            pending_resolved=True,
        )
        if applied:
            self._handle_replan_queue_0702(
                {
                    "timestamp": ts,
                    "source": source,
                    "ignore": 2,
                    "missionPlanID": int(plan_id),
                }
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
        generation = self._next_plan_apply_generation()

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

        # 0702로 확정되는 공격 계획도 현재 수행 계획이다. 기존에는 0903에서만
        # 두 촬영 진행 worker를 갱신해, 공격 중에는 떠난 UAV를 포함한 이전
        # 3대 편성의 Line/Area 상태가 계속 남았다. 확정 즉시 실제 재할당 편성
        # (예: 3대 -> 2대)을 authoritative state로 전환한다.
        self._line_scan_apply_mission_plan(plan_id)
        self._queue_area_snapshot_plan_update(
            plan_id=plan_id,
            timestamp_ms=timestamp_ms,
            source=source,
            prefer_apply=True,
        )

        viz = getattr(self, "_viz_tab", None)
        viz_applied = False
        if viz is not None:
            try:
                if hasattr(viz, "apply_mission_plan_decision"):
                    viz.apply_mission_plan_decision(mission_plan_id=plan_id)
                    viz_applied = True
                elif hasattr(viz, "update_0903"):
                    viz.update_0903(timestamp_ms=timestamp_ms, mission_plan_id=plan_id, source=source)
                    viz_applied = True
            except Exception as exc:
                self._append_log_line(f"[0702] mission plan apply failed: {exc}")
        if viz_applied:
            self._kick_0501_sender()
        self._defer_plan_tab_updates(
            plan_id=plan_id,
            timestamp_ms=timestamp_ms,
            source=source,
            generation=generation,
            prefer_apply=True,
            log_prefix="[0702]",
            tab_jobs=(
                ("_schedule_tab", "schedule tab", 20),
                ("_quality_tab", "quality tab", 60),
            ),
        )
        try:
            quality_speed_coord = getattr(self, "_quality_speed_coord", None)
            if quality_speed_coord is not None and hasattr(quality_speed_coord, "apply_mission_plan_decision"):
                quality_speed_coord.apply_mission_plan_decision(plan_id)
        except Exception as exc:
            self._append_log_line(f"[0702] quality-speed coord apply failed: {exc}")

        self._path_deviation_guard_until_ms = int(_now_ms_since_2000()) + 3000
        self._path_deviation_guard_notice_key = None

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
            notice = "0201 type 자동보정: " + self._summarize_notice_ids(auto_fixed_ids)
            self._append_log_line("[WARN] " + notice)
            self._send_0001_notice(notice)

        if invalid_ids:
            notice = "0201 type 확인필요: " + self._summarize_notice_ids(invalid_ids)
            self._append_log_line("[WARN] " + notice)
            self._send_0001_notice(notice)

    def _on_rx_0201(self, payload: object | None, *, is_new_arrival: bool = True) -> None:
        previous_input_payload = copy.deepcopy(
            getattr(self, "_last_rx_0201_payload", None)
        )
        try:
            # Anchor the actual RX occurrence before a following 0101(mode=2)
            # starts review.  This also handles repeated identical package IDs.
            self._record_external_0201_review_anchor(
                payload,
                is_new_arrival=bool(is_new_arrival),
            )
        except Exception:
            pass
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
        reexecute_dispatched = False
        reexecute_consumed_input = False
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
                    try:
                        replan_payload = self._attach_current_remaining_context(dict(replan_payload))
                    except Exception as exc:
                        self._append_log_line(f"[CURRENT] reexecute context attach failed: {exc}")
                    reexecute_dispatched = True
                    self._queue_replan_payloads([replan_payload], source="reexecute_0201")
                try:
                    reexecute_consumed_input = bool(coord.has_dispatched_input_plan(payload))
                except Exception:
                    reexecute_consumed_input = False
                try:
                    reexecute_active = bool(reexecute_active or reexecute_dispatched or coord.is_active())
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
            refresh_blocked = bool(
                reexecute_dispatched
                or reexecute_consumed_input
                or (reexecute_active and block_when_reexecute_active)
            )
            if not self._input_refresh_replan_enabled:
                self._append_log_line("[REINPUT] monitoring trigger OFF -> skip 0902 replan on 0201")
                return
            if refresh_blocked:
                if reexecute_dispatched or reexecute_consumed_input:
                    self._append_log_line("[REINPUT] skipped: reexecute 0201 already handled")
                else:
                    self._append_log_line("[REINPUT] skipped: reexecute-wait mode is active")
            elif self._system_mode_code not in (3, 4):
                self._append_log_line(
                    f"[REINPUT] skipped: mode={self._system_mode_code} (need 3/4)"
                )

            refresh_payload = payload
            refresh_review_info = None
            target_order_change_info = None
            if not refresh_blocked and self._system_mode_code in (3, 4):
                from modules.monitoring.logic.anti_armor_air_strike_review import (
                    detect_anti_armor_target_order_change,
                )

                def _normalized_input_plan(value: object | None) -> dict[str, Any]:
                    parsed = parse_payload(value)
                    if not self._input_0201_has_core_payload(parsed):
                        parsed = parse_payload(self._unwrap_payload(value))
                    return parsed if isinstance(parsed, dict) else {}

                previous_plan = _normalized_input_plan(previous_input_payload)
                current_plan = _normalized_input_plan(payload)
                target_order_change_info = detect_anti_armor_target_order_change(
                    previous_plan,
                    current_plan,
                )
                if isinstance(target_order_change_info, dict):
                    self._append_log_line(
                        "[REINPUT][TYPE1] target order changed -> preserve incoming mission order "
                        f"(targets={int(target_order_change_info.get('targetCount') or 0)})"
                    )
                else:
                    refresh_review_info = self._review_type1_new_target_0201_for_input_refresh(payload)
                    if isinstance(refresh_review_info, dict) and refresh_review_info.get("applicable"):
                        if not refresh_review_info.get("sent"):
                            error = str(refresh_review_info.get("error") or "unknown")
                            self._append_log_line(
                                f"[REINPUT][0902] aborted: new-target 0204 not sent ({error})"
                            )
                            return
                        reviewed_payload = refresh_review_info.get("payload")
                        if not isinstance(reviewed_payload, dict):
                            self._append_log_line(
                                "[REINPUT][0902] aborted: reviewed new-target 0201 payload is unavailable"
                            )
                            return
                        refresh_payload = reviewed_payload

            order_change_reason = None
            order_change_detail = None
            if isinstance(target_order_change_info, dict):
                order_change_reason = str(target_order_change_info.get("reason") or "").strip()
                order_change_detail = {
                    "changeKind": "antiArmorTargetOrderChange",
                    "targetOrderChanged": True,
                    "targetCount": int(target_order_change_info.get("targetCount") or 0),
                    "previousTargetInputMissionIDs": list(
                        target_order_change_info.get("previousTargetInputMissionIDs") or []
                    ),
                    "currentTargetInputMissionIDs": list(
                        target_order_change_info.get("currentTargetInputMissionIDs") or []
                    ),
                }

            refresh_current_input_id = None
            viz = getattr(self, "_viz_tab", None)
            current_input_getter = getattr(viz, "get_current_input_mission_id", None)
            if callable(current_input_getter):
                try:
                    refresh_current_input_id = current_input_getter(
                        allow_pending_fallback=False
                    )
                except TypeError:
                    refresh_current_input_id = current_input_getter()
                except Exception as exc:
                    self._append_log_line(
                        f"[REINPUT] current input lookup failed: {exc}"
                    )
            try:
                refresh_current_input_id = int(refresh_current_input_id)
            except (TypeError, ValueError):
                refresh_current_input_id = None
            if refresh_current_input_id is not None and refresh_current_input_id <= 0:
                refresh_current_input_id = None
            if refresh_current_input_id is not None:
                self._append_log_line(
                    "[REINPUT] current mission progress attached: "
                    f"inputMissionID={int(refresh_current_input_id)}"
                )
            else:
                self._append_log_line(
                    "[REINPUT] current mission ID unavailable; "
                    "planner will use exact started-snapshot fallback only"
                )

            replan_payload, logs = refresh_coord.on_input_plan(
                refresh_payload,
                system_mode=self._system_mode_code,
                blocked=refresh_blocked,
                current_mission_plan_id=self._current_mission_plan_id,
                current_input_mission_id=refresh_current_input_id,
                replan_reason=order_change_reason,
                replan_detail=order_change_detail,
            )
            for line in logs:
                self._append_log_line(line)
            if replan_payload:
                if isinstance(refresh_review_info, dict) and refresh_review_info.get("sent"):
                    reviewed_id = _optional_int_value(refresh_review_info.get("inputMissionPackageID"))
                    source_id = _optional_int_value(refresh_review_info.get("sourceInputMissionPackageID"))
                    replan_payload[_INPUT_0201_REVIEW_0204_SENT_FLAG] = True
                    if reviewed_id is not None and reviewed_id > 0:
                        replan_payload["inputMissionPackageID"] = int(reviewed_id)
                    if source_id is not None and source_id > 0:
                        replan_payload["sourceInputMissionPackageID"] = int(source_id)
                    detail = replan_payload.get("replanDetail")
                    if not isinstance(detail, dict):
                        detail = {}
                        replan_payload["replanDetail"] = detail
                    detail[_INPUT_0201_REVIEW_0204_SENT_FLAG] = True
                    detail["reviewKind"] = "antiArmorNewTargetRefresh"
                    if reviewed_id is not None and reviewed_id > 0:
                        detail["inputMissionPackageID"] = int(reviewed_id)
                    if source_id is not None and source_id > 0:
                        detail["sourceInputMissionPackageID"] = int(source_id)
                self._queue_replan_payloads([replan_payload], source="input_refresh")
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
            self._queue_replan_payloads(replan_payloads, source="prior_mission")
        except Exception as exc:
            self._append_log_line(f"[0902] prior-mission-on-0202 error: {exc}")

    def _on_rx_0401(
        self,
        payload: object | None,
        *,
        raw_body: dict[str, Any] | None = None,
        canonical_signature: bytes | None = None,
    ) -> None:
        seq = self._begin_0401_handler(payload)
        raw_body = dict(raw_body) if isinstance(raw_body, dict) else None
        ts: int | None = None
        states = []
        dispatch_context: Replan0401DispatchContext | None = None
        try:
            with self._trace_0401_phase(seq, "parse", reused=bool(raw_body)):
                if raw_body:
                    replan_perf.add("monitoring.0401.parse", reused=1)
                else:
                    parse_start = replan_perf.start_timer()
                    try:
                        raw_body = parse_payload(payload)
                        if not raw_body:
                            raw_body = parse_payload(self._unwrap_payload(payload))
                        replan_perf.add_elapsed(
                            "monitoring.0401.parse",
                            parse_start,
                            parsed=1 if raw_body else 0,
                        )
                    except Exception as exc:
                        replan_perf.add_elapsed("monitoring.0401.parse", parse_start, error=1)
                        self._append_log_line(f"[0401] parse failed: {exc}")

            with self._trace_0401_phase(seq, "extract_states"):
                ts, states = extract_0401_agent_states(raw_body or payload)
                if canonical_signature is None and raw_body:
                    canonical_signature, _parsed_for_signature = self._payload_signature_context(raw_body)
                dispatch_context = self._build_0401_dispatch_context(
                    payload=payload,
                    raw_body=raw_body,
                    canonical_signature=canonical_signature,
                    timestamp_ms=ts,
                    agent_states=list(states or []),
                )
                try:
                    self._latest_0401_raw_body = raw_body if isinstance(raw_body, dict) else None
                    self._latest_0401_agent_states = list(states or [])
                except Exception:
                    pass
                self._line_scan_submit_agent_status(
                    timestamp_ms=ts,
                    agent_states=list(states or []),
                )
                self._queue_area_snapshot_status_update(
                    timestamp_ms=ts,
                    agent_states=list(states or []),
                )

            with self._trace_0401_phase(seq, "availability_bootstrap", state_count=len(states)):
                try:
                    self._bootstrap_availability_from_agent_states(states)
                except Exception as exc:
                    self._log_suppressed_exception(
                        "0401_availability_bootstrap",
                        "[0401] availability bootstrap failed",
                        exc,
                    )

            with self._trace_0401_phase(seq, "fault_notices"):
                try:
                    self._sync_0401_fault_notices(states)
                except Exception as exc:
                    self._append_log_line(f"[0001] 0401 abnormal notice update failed: {exc}")

            fuel_state_map: dict[int, str] = {}
            with self._trace_0401_phase(seq, "fuel_coord"):
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

            with self._trace_0401_phase(seq, "prior_close"):
                try:
                    coord = getattr(self, "_prior_mission_coord", None)
                    if coord is not None and self._prior_mission_replan_enabled:
                        replan_payloads, logs = coord.on_agent_states(
                            states,
                            system_mode=self._system_mode_code,
                            current_mission_plan_id=self._current_mission_plan_id,
                            dispatch_context=dispatch_context,
                        )
                        for line in logs:
                            self._append_log_line(line)
                        self._queue_replan_payloads(replan_payloads, source="prior_mission")
                except Exception as exc:
                    self._append_log_line(f"[0902] prior-close-on-0401 error: {exc}")

            viz = getattr(self, "_viz_tab", None)
            schedule_tab = getattr(self, "_schedule_tab", None)
            quality_tab = getattr(self, "_quality_tab", None)
            viz_updated = False
            with self._trace_0401_phase(seq, "visualization_update"):
                try:
                    if viz is not None and hasattr(viz, "update_agent_status"):
                        viz.update_agent_status(
                            timestamp_ms=ts,
                            agent_states=states,
                            fuel_state_map=fuel_state_map,
                        )
                        viz_updated = True
                except Exception as exc:
                    self._log_suppressed_exception(
                        "0401_visualization_update",
                        "[0401] visualization update failed",
                        exc,
                        diag_event="0401_visualization_update_failed",
                    )

            if viz_updated:
                with self._trace_0401_phase(seq, "0501_kick"):
                    self._kick_0501_sender(min_interval_sec=0.12)

            with self._trace_0401_phase(seq, "snapshot_save", has_raw=bool(raw_body)):
                try:
                    if raw_body:
                        agent_status_snapshot.save_agent_status_snapshot(raw_body)
                except Exception as exc:
                    self._append_log_line(f"[0401] snapshot save failed: {exc}")

            with self._trace_0401_phase(seq, "dl_update"):
                try:
                    self._update_dl_inference(raw_body)
                except Exception as exc:
                    self._log_suppressed_exception(
                        "0401_dl_update",
                        "[DL] update dispatch failed",
                        exc,
                        diag_event="0401_dl_update_failed",
                    )

            with self._trace_0401_phase(seq, "turn_tab"):
                try:
                    turn_tab = getattr(self, "_turn_radius_tab", None)
                    if turn_tab is not None and hasattr(turn_tab, "ingest_0401"):
                        turn_tab.ingest_0401(raw_body)
                except Exception as exc:
                    self._log_suppressed_exception(
                        "0401_turn_tab",
                        "[0401] turn-radius tab update failed",
                        exc,
                    )

            with self._trace_0401_phase(seq, "schedule_tab"):
                try:
                    if schedule_tab is not None and hasattr(schedule_tab, "update_agent_status"):
                        schedule_tab.update_agent_status(
                            timestamp_ms=ts,
                            agent_states=states,
                            fuel_state_map=fuel_state_map,
                        )
                except Exception as exc:
                    self._log_suppressed_exception(
                        "0401_schedule_tab",
                        "[0401] schedule tab update failed",
                        exc,
                    )

            with self._trace_0401_phase(seq, "quality_tab"):
                try:
                    if quality_tab is not None and hasattr(quality_tab, "update_agent_status"):
                        quality_tab.update_agent_status(
                            timestamp_ms=ts,
                            agent_states=states,
                            fuel_state_map=fuel_state_map,
                        )
                except Exception as exc:
                    self._log_suppressed_exception(
                        "0401_quality_tab",
                        "[0401] quality tab update failed",
                        exc,
                    )

            if viz is None or not hasattr(viz, "update_agent_status"):
                return

            with self._trace_0401_phase(seq, "completion_recommendation"):
                try:
                    if hasattr(viz, "pop_completion_recommendations"):
                        for recommend, input_id in viz.pop_completion_recommendations():
                            self._send_0503(recommend, input_id)
                except Exception as exc:
                    self._log_suppressed_exception(
                        "0503_completion_recommendation",
                        "[0503] completion recommendation failed",
                        exc,
                        diag_event="0503_completion_recommendation_failed",
                    )

            with self._trace_0401_phase(seq, "rtb_coord"):
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
                        replan_payloads, logs, notices = rtb_coord.on_agent_states(
                            states,
                            timestamp_ms=ts,
                            system_mode=self._system_mode_code,
                            current_mission_plan_id=self._current_mission_plan_id,
                            aircraft_filter=self._is_aircraft_in_current_plan,
                            suppressed_aircraft=suppressed_aircraft,
                            dispatch_context=dispatch_context,
                        )
                        for line in logs:
                            self._append_log_line(line)
                        for notice in notices:
                            self._send_0001_notice(notice)
                        self._rtb_availability_override = rtb_coord.get_availability_overrides()
                        if self._rtb_availability_override:
                            self._apply_forced_availability(stage="0401")
                        replan_payloads = [
                            self._attach_current_remaining_context(body)
                            for body in replan_payloads
                            if isinstance(body, dict)
                        ]
                        self._queue_replan_payloads(replan_payloads, source="rtb")
                except Exception as exc:
                    self._append_log_line(f"[0902] rtb-on-0401 error: {exc}")

            path_dev_suppressed_aircraft: set[int] = set()
            with self._trace_0401_phase(seq, "path_deviation"):
                try:
                    path_dev_coord = getattr(self, "_path_deviation_coord", None)
                    turn_tab = getattr(self, "_turn_radius_tab", None)
                    turn_views = None
                    if turn_tab is not None and hasattr(turn_tab, "build_views"):
                        turn_views = turn_tab.build_views(include_paths=False)
                    if path_dev_coord is not None:
                        path_dev_guard = self._path_deviation_trigger_guard()
                        self._update_path_deviation_guard_notice(path_dev_guard)
                        if path_dev_guard is None:
                            current_input_id_for_path_deviation = None
                            if viz is not None and hasattr(viz, "get_current_input_mission_id"):
                                try:
                                    current_input_id_for_path_deviation = viz.get_current_input_mission_id(
                                        allow_pending_fallback=False
                                    )
                                except Exception:
                                    current_input_id_for_path_deviation = None
                            replan_payloads, logs = path_dev_coord.on_turn_monitor_views(
                                turn_views,
                                enabled=self._path_deviation_trigger_enabled,
                                system_mode=self._system_mode_code,
                                current_mission_plan_id=self._current_mission_plan_id,
                                current_input_id=current_input_id_for_path_deviation,
                                aircraft_filter=self._is_aircraft_in_current_plan,
                                dispatch_context=dispatch_context,
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
                            self._queue_replan_payloads(replan_payloads, source="path_deviation")
                except Exception as exc:
                    self._append_log_line(f"[0902] path-deviation-on-0401 error: {exc}")

            quality_suppressed_aircraft: set[int] = set(path_dev_suppressed_aircraft)
            with self._trace_0401_phase(seq, "quality_speed"):
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
                            dispatch_context=dispatch_context,
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
                        self._queue_replan_payloads(replan_payloads, source="quality_speed")
                except Exception as exc:
                    self._append_log_line(f"[0902] quality-speed-on-0401 error: {exc}")

            with self._trace_0401_phase(seq, "imaging_schedule"):
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
                            dispatch_context=dispatch_context,
                        )
                        for line in logs:
                            self._append_log_line(line)
                        self._queue_replan_payloads(replan_payloads, source="imaging_schedule")
                except Exception as exc:
                    self._append_log_line(f"[0902] imaging-schedule-on-0401 error: {exc}")

            with self._trace_0401_phase(seq, "forced_availability_apply"):
                try:
                    if self._forced_availability_override or self._rtb_availability_override:
                        self._apply_forced_availability(stage="0401")
                except Exception:
                    pass
        except Exception as exc:
            self._record_0401_handler_exception(seq, exc)
            self._append_log_line(f"[0401] handler failed: {exc}")
        finally:
            try:
                state_count = len(states) if states is not None else 0
            except Exception:
                state_count = 0
            self._end_0401_handler(
                seq,
                raw_body=raw_body,
                timestamp_ms=ts,
                state_count=state_count,
            )

    def _on_rx_0402(self, payload: object | None) -> None:
        try:
            coord = getattr(self, "_target_detection_coord", None)
            if coord is None:
                return
            if not self._target_detection_replan_enabled and not self._post_attack_rejoin_enabled:
                self._append_log_line("[0402] monitoring triggers OFF -> skip 0902 replan on 0402")
                return
            replan_payloads, logs = coord.on_situation_awareness(
                payload,
                system_mode=self._system_mode_code,
                current_mission_plan_id=self._current_mission_plan_id,
            )
            for line in logs:
                self._append_log_line(line)
            self._queue_0402_replan_payloads(replan_payloads)
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
                    self._send_0001_notice(f"{aid_text} 강제귀환 불가")
                elif cmd_type == 3:
                    self._send_0001_notice(f"{aid_text} 강제복귀 불가")
                elif cmd_type == 1:
                    self._send_0001_notice(f"{aid_text} 강제대기 불가")
                else:
                    self._send_0001_notice(f"{aid_text} 강제명령 불가")

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
            replan_payloads = [
                self._attach_current_remaining_context(body)
                for body in replan_payloads
                if isinstance(body, dict)
            ]
            self._queue_replan_payloads(replan_payloads, source="forced_command")
        except Exception as exc:
            self._append_log_line(f"[0902] forced-command-on-0802 error: {exc}")

    def _replan_queue_blocker_for_0803(self) -> dict[str, Any] | None:
        manager = getattr(self, "_replan_queue_manager", None)
        if manager is None or not hasattr(manager, "find_0803_option_decision_blocker"):
            return None
        try:
            blocker = manager.find_0803_option_decision_blocker()
        except Exception as exc:
            self._append_log_line(f"[0803] option-decision queue check failed: {exc}")
            return None
        return dict(blocker) if isinstance(blocker, dict) else None

    def _on_rx_0803(self, payload: object | None) -> None:
        _ts, execute, _source, _body = extract_0803_execute(payload)
        if execute is None:
            return
        # The listener and the RX-table poller can surface the same nFusion
        # message with different raw byte representations.  Raw-byte checks in
        # those ingress paths therefore are not sufficient.  Deduplicate the
        # normalized 0803 event before it can re-arm execute=2 state or repeat
        # any visualization/progress side effects.
        try:
            event_signature = json.dumps(
                {
                    "timestamp": _ts,
                    "execute": execute,
                    "source": _source,
                    "body": _body,
                },
                ensure_ascii=False,
                sort_keys=True,
                default=str,
            )
        except Exception:
            event_signature = repr((_ts, execute, _source, _body))
        now_wall_ms = int(time.time() * 1000)
        last_event_signature = getattr(self, "_last_0803_event_signature", None)
        try:
            last_event_signature_ms = int(
                getattr(self, "_last_0803_event_signature_ms", 0) or 0
            )
        except Exception:
            last_event_signature_ms = 0
        if (
            last_event_signature == event_signature
            and last_event_signature_ms > 0
            and now_wall_ms - last_event_signature_ms <= 5000
        ):
            self._append_log_line(f"[0803] duplicate execute={int(execute)} ignored")
            return
        self._last_0803_event_signature = event_signature
        self._last_0803_event_signature_ms = now_wall_ms
        reexecute_handled = False
        next_collab_trigger_enabled = bool(getattr(self, "_next_collab_replan_trigger_enabled", False))
        if int(execute) == 1 and next_collab_trigger_enabled:
            try:
                try:
                    next_collab_0803_signature = json.dumps(
                        {
                            "timestamp": _ts,
                            "execute": execute,
                            "source": _source,
                            "body": _body,
                        },
                        ensure_ascii=False,
                        sort_keys=True,
                        default=str,
                    )
                except Exception:
                    next_collab_0803_signature = repr((_ts, execute, _source, _body))
                now_wall_ms = int(time.time() * 1000)
                last_signature = getattr(self, "_last_next_collab_0803_signature", None)
                last_signature_ms = getattr(self, "_last_next_collab_0803_signature_ms", 0)
                try:
                    last_signature_ms_int = int(last_signature_ms or 0)
                except Exception:
                    last_signature_ms_int = 0
                if (
                    last_signature == next_collab_0803_signature
                    and last_signature_ms_int > 0
                    and now_wall_ms - last_signature_ms_int <= 5000
                ):
                    self._append_log_line("[0803] duplicate next-collab execute ignored")
                    return
                blocker = self._replan_queue_blocker_for_0803()
                if blocker is not None:
                    reason = str(blocker.get("reason") or "-").strip() or "-"
                    source_label = str(blocker.get("source_label") or "").strip()
                    queue_id = blocker.get("queue_id")
                    queue_text = f"#{queue_id} " if queue_id is not None else ""
                    label_text = f"{source_label}: " if source_label else ""
                    notice = f"임무계획이 진행중입니다. 진행중인 재계획 사유: {label_text}{reason}"
                    self._send_0001_notice(notice)
                    self._append_log_line(
                        "[0803] next-collab replan blocked: option decision pending "
                        f"({queue_text}{label_text}{reason})"
                    )
                    self._last_next_collab_0803_signature = next_collab_0803_signature
                    self._last_next_collab_0803_signature_ms = now_wall_ms
                    return
                try:
                    reexecute_coord = getattr(self, "_reexecute_coord", None)
                    if reexecute_coord is not None:
                        for line in reexecute_coord.on_execute(execute):
                            self._append_log_line(line)
                    reexecute_handled = True
                except Exception as exc:
                    reexecute_handled = True
                    self._append_log_line(f"[0902] reexecute-on-0803 error: {exc}")
                coord = getattr(self, "_next_collab_replan_coord", None)
                viz = getattr(self, "_viz_tab", None)
                turn_tab = getattr(self, "_turn_radius_tab", None)
                context_payload = None
                if viz is not None and hasattr(viz, "build_execute_next_replan_context"):
                    reexecute_source_input_id = None
                    reexecute_clone_input_id = None
                    try:
                        clone_mapping = (
                            reexecute_coord.current_clone_mapping()
                            if reexecute_coord is not None
                            and hasattr(reexecute_coord, "current_clone_mapping")
                            else None
                        )
                        if clone_mapping is not None:
                            reexecute_source_input_id = int(clone_mapping[0])
                            reexecute_clone_input_id = int(clone_mapping[1])
                    except Exception as exc:
                        self._append_log_line(
                            f"[0803] reexecute clone mapping lookup failed: {exc}"
                        )
                    context_payload = viz.build_execute_next_replan_context(
                        reexecute_source_input_id=reexecute_source_input_id,
                        reexecute_clone_input_id=reexecute_clone_input_id,
                    )
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
                        try:
                            if (
                                viz is not None
                                and hasattr(viz, "note_execute_next_transition")
                                and isinstance(context_payload, dict)
                            ):
                                viz.note_execute_next_transition(
                                    current_input_id=context_payload.get("current_input_mission_id"),
                                    target_input_id=context_payload.get("target_input_mission_id"),
                                )
                        except Exception as exc:
                            self._append_log_line(f"[0803] execute-next transition note failed: {exc}")
                        self._last_next_collab_0803_signature = next_collab_0803_signature
                        self._last_next_collab_0803_signature_ms = now_wall_ms
                        self._queue_replan_payloads([replan_payload], source="next_collab")
                        return
            except Exception as exc:
                self._append_log_line(f"[0902] next-collab-on-0803 error: {exc}")
            self._append_log_line("[NEXTCOLLAB] 0902 next-collab skipped -> falling back to normal execute=1 handling")
        if not reexecute_handled:
            try:
                coord = getattr(self, "_reexecute_coord", None)
                if coord is not None:
                    for line in coord.on_execute(execute):
                        self._append_log_line(line)
            except Exception as exc:
                self._append_log_line(f"[0902] reexecute-on-0803 error: {exc}")

        if int(execute) == 1 and not next_collab_trigger_enabled:
            self._append_log_line("[NEXTCOLLAB] monitoring trigger OFF -> skip 0902 replan on 0803 execute=1")

        viz = getattr(self, "_viz_tab", None)
        if viz is None or not hasattr(viz, "handle_execute_command"):
            return
        if int(execute) == 2:
            # 재수행: 현재 input의 area 커버리지 이월분을 초기화해 재적립되게 한다.
            try:
                tracker = getattr(viz, "_progress_tracker", None)
                repeat_input_id = (
                    tracker.get_active_input_id()
                    if tracker is not None and hasattr(tracker, "get_active_input_id")
                    else None
                )
                if repeat_input_id is not None:
                    self._queue_area_snapshot_input_reset(int(repeat_input_id))
                    self._line_scan_reset_input_coverage(int(repeat_input_id))
                    self._append_log_line(
                        f"[AREA] reexecute -> input {int(repeat_input_id)} coverage reset queued"
                    )
            except Exception as exc:
                self._append_log_line(f"[AREA] reexecute coverage reset failed: {exc}")
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

    def _start_0305_rx_poller(self) -> None:
        self._last_0305_raw = None
        self._poll_0305_timer = QTimer(self)
        self._poll_0305_timer.setInterval(250)
        self._poll_0305_timer.timeout.connect(self._poll_0305_in_rx_table)
        self._poll_0305_timer.start()

    def _start_0701_rx_poller(self) -> None:
        self._last_0701_raw = None
        self._poll_0701_timer = QTimer(self)
        self._poll_0701_timer.setInterval(250)
        self._poll_0701_timer.timeout.connect(self._poll_0701_in_rx_table)
        self._poll_0701_timer.start()

    def _start_0001_rx_poller(self) -> None:
        self._last_0001_raw = None
        self._poll_0001_timer = QTimer(self)
        self._poll_0001_timer.setInterval(250)
        self._poll_0001_timer.timeout.connect(self._poll_0001_in_rx_table)
        self._poll_0001_timer.start()

    def _start_0903_rx_poller(self) -> None:
        self._last_0903_raw = None
        self._poll_0903_timer = QTimer(self)
        self._poll_0903_timer.setInterval(250)
        self._poll_0903_timer.timeout.connect(self._poll_0903_in_rx_table)
        self._poll_0903_timer.start()

    def _start_replan_queue_timer(self) -> None:
        self._replan_queue_timer = QTimer(self)
        self._replan_queue_timer.setInterval(500)
        self._replan_queue_timer.timeout.connect(self._poll_replan_queue_state)
        self._replan_queue_timer.start()

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
            target_row = self._find_rx_row("0101")
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

    def _poll_0305_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = self._find_rx_row("0305")
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest or (self._last_0305_raw is not None and raw_latest == self._last_0305_raw):
                return
            self._last_0305_raw = raw_latest
            self._on_rx_0305(raw_latest)
        except Exception:
            pass

    def _poll_0701_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = self._find_rx_row("0701")
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest or (self._last_0701_raw is not None and raw_latest == self._last_0701_raw):
                return
            self._last_0701_raw = raw_latest
            self._on_rx_0701(raw_latest)
        except Exception:
            pass

    def _poll_0001_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = self._find_rx_row("0001")
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest or (self._last_0001_raw is not None and raw_latest == self._last_0001_raw):
                return
            self._last_0001_raw = raw_latest
            self._on_rx_0001(raw_latest)
        except Exception:
            pass

    def _poll_0903_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = self._find_rx_row("0903")
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

    def _poll_replan_queue_state(self) -> None:
        manager = getattr(self, "_replan_queue_manager", None)
        if manager is None:
            return
        try:
            released, logs = manager.poll_due_transitions()
            for line in logs:
                self._append_log_line(line)
            self._refresh_replan_queue_snapshot()
            if released:
                self._schedule_replan_queue_drain()
        except Exception:
            pass

    def _poll_0702_in_rx_table(self) -> None:
        try:
            self._maybe_apply_pending_0702()
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = self._find_rx_row("0702")
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
            target_row = self._find_rx_row("0201")
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
            same_payload = self._last_0201_raw is not None and raw_latest == self._last_0201_raw
            raw_changed = not same_payload
            new_arrival = (
                latest_ms is not None and (last_ms is None or latest_ms > last_ms)
            ) or (latest_ms is None and raw_changed)

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
            target_row = self._find_rx_row("0202")
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
            target_row = self._find_rx_row("0401")
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest or (self._last_0401_raw is not None and raw_latest == self._last_0401_raw):
                return
            self._last_0401_raw = raw_latest
            self._enqueue_0401_payload(raw_latest)
        except Exception:
            pass

    def _poll_0402_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = self._find_rx_row("0402")
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
            self._enqueue_0402_payload(raw_latest)
        except Exception:
            pass

    def _poll_0802_in_rx_table(self) -> None:
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = self._find_rx_row("0802")
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
            target_row = self._find_rx_row("0803")
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
                normalized_payloads: list[dict[str, Any]] = []
                for payload in payloads:
                    if not isinstance(payload, dict):
                        continue
                    try:
                        normalized_payloads.append(self._attach_current_remaining_context(dict(payload)))
                    except Exception as exc:
                        self._append_log_line(f"[CURRENT] forced_hold_due context attach failed: {exc}")
                        normalized_payloads.append(dict(payload))
                payloads = normalized_payloads
            self._queue_replan_payloads(payloads, source="forced_hold_due")
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

    def _attach_current_remaining_context(self, payload: dict[str, Any]) -> dict[str, Any]:
        body = dict(payload or {})
        detail = body.get("replanDetail") if isinstance(body.get("replanDetail"), dict) else None
        if not isinstance(detail, dict):
            return body
        trigger = str(detail.get("trigger") or "").strip()
        trigger_type = str(detail.get("triggerType") or "").strip()
        is_reexecute_refresh = trigger == "0201" and trigger_type == "collabReexecuteInputRefresh"
        if trigger not in {"0401", "0802"} and not is_reexecute_refresh:
            return body
        if trigger_type in {"pathDeviation", "qualityMonitorSep", "imagingScheduleDeviation", "nextCollaborativeMission"}:
            return body
        def _first_payload_input_id() -> int | None:
            for row in body.get("inputMissionIDList") or []:
                if not isinstance(row, dict):
                    continue
                try:
                    input_id = int(row.get("inputMissionID"))
                except BaseException:
                    continue
                if input_id > 0:
                    return int(input_id)
            return None

        def _active_current_input_id() -> int | None:
            viz = getattr(self, "_viz_tab", None)
            if viz is None:
                return None
            for method_name in ("get_current_input_mission_id",):
                method = getattr(viz, method_name, None)
                if not callable(method):
                    continue
                try:
                    value = method(allow_pending_fallback=False)
                except TypeError:
                    try:
                        value = method()
                    except BaseException:
                        value = None
                except BaseException:
                    value = None
                try:
                    input_id = int(value) if value is not None else None
                except BaseException:
                    input_id = None
                if input_id is not None and input_id > 0:
                    return int(input_id)
            return None

        def _available_uav_ids() -> list[int]:
            out: list[int] = []
            for aid in self._effective_available_ids():
                try:
                    aircraft_id = int(aid)
                except BaseException:
                    continue
                if aircraft_id in (4, 5, 6):
                    out.append(int(aircraft_id))
            return sorted(set(out))

        def _current_source_plan_id() -> int | None:
            for raw_value in (
                detail.get("sourceMissionPlanID"),
                detail.get("currentMissionPlanID"),
                body.get("sourceMissionPlanID"),
                body.get("currentMissionPlanID"),
                getattr(self, "_current_mission_plan_id", None),
            ):
                try:
                    plan_id = int(raw_value)
                except BaseException:
                    continue
                if plan_id > 0:
                    return int(plan_id)
            return None

        def _snapshot_agent_states() -> list[dict[str, Any]]:
            def _raw_agent_rows(payload: object | None) -> list[dict[str, Any]]:
                body = parse_payload(payload)
                if not isinstance(body, dict):
                    return []
                rows = (
                    body.get("agentStateList")
                    or body.get("AgentStateList")
                    or body.get("uavStates")
                    or body.get("UavStates")
                    or []
                )
                if not isinstance(rows, list):
                    return []
                return [row for row in rows if isinstance(row, dict)]

            raw_body = getattr(self, "_latest_0401_raw_body", None)
            raw_rows = _raw_agent_rows(raw_body)
            if raw_rows:
                return raw_rows
            raw_latest = getattr(self, "_last_0401_raw", None)
            if raw_latest is not None:
                raw_rows = _raw_agent_rows(raw_latest)
                if raw_rows:
                    return raw_rows
            cached_rows = getattr(self, "_latest_0401_agent_states", None)
            if isinstance(cached_rows, list) and cached_rows:
                return [row for row in cached_rows if isinstance(row, dict)]
            return []

        def _state_containers(row: dict[str, Any]) -> list[dict[str, Any]]:
            containers: list[dict[str, Any]] = [row]
            for key in ("unmannedInfo", "UnmannedInfo", "unmanned_info"):
                nested = row.get(key)
                if isinstance(nested, dict):
                    containers.append(nested)
            return containers

        def _state_aircraft_id(row: dict[str, Any]) -> int | None:
            for key in ("aircraftID", "aircraftId", "AircraftID", "aircraft_id"):
                try:
                    aircraft_id = int(row.get(key))
                except BaseException:
                    continue
                if aircraft_id > 0:
                    return int(aircraft_id)
            return None

        def _state_coordinate(row: dict[str, Any]) -> dict[str, float] | None:
            for container in _state_containers(row):
                coord = _normalize_coordinate(
                    container.get("coordinate")
                    or container.get("Coordinate")
                    or container.get("coord")
                )
                if coord is not None:
                    return coord
                coord = _normalize_coordinate(container)
                if coord is not None:
                    return coord
            return None

        def _state_number_from_sources(
            row: dict[str, Any],
            keys: tuple[str, ...],
        ) -> float | None:
            for container in _state_containers(row):
                velocity = container.get("velocity") or container.get("Velocity")
                for source in (velocity, container):
                    if not isinstance(source, dict):
                        continue
                    for key in keys:
                        value = _coerce_float(source.get(key))
                        if value is not None:
                            return float(value)
            return None

        def _state_heading_deg(row: dict[str, Any]) -> float | None:
            return _state_number_from_sources(
                row,
                (
                    "headingDeg",
                    "heading_deg",
                    "HeadingDeg",
                    "heading",
                    "Heading",
                ),
            )

        def _state_heading_rad(row: dict[str, Any]) -> float | None:
            return _state_number_from_sources(
                row,
                (
                    "headingRad",
                    "heading_rad",
                    "HeadingRad",
                    "headingRadian",
                    "heading_radian",
                ),
            )

        def _state_speed_mps(row: dict[str, Any]) -> float | None:
            return _state_number_from_sources(
                row,
                ("speedMps", "speed_mps", "SpeedMps", "speed", "Speed"),
            )

        def _attach_reexecute_snapshot_context() -> bool:
            active_input_id = _active_current_input_id()
            first_input_id = _first_payload_input_id()
            attach_input_id = first_input_id if first_input_id is not None else active_input_id
            available_uavs = _available_uav_ids()
            aircraft_text = ",".join(str(v) for v in available_uavs) if available_uavs else "-"
            self._append_log_line(
                "[CURRENT] reexecute attach start: "
                f"currentInputMissionID={attach_input_id}, firstPayloadInputID={first_input_id}, "
                f"aircraft={aircraft_text}"
            )
            if attach_input_id is None:
                self._append_log_line("[CURRENT] reexecute snapshot context skipped: no inputMissionID")
                return False
            if not available_uavs:
                self._append_log_line("[CURRENT] reexecute snapshot context skipped: no available UAVs")
                return False

            rows = _snapshot_agent_states()
            self._append_log_line(
                f"[CURRENT] reexecute cached 0401 rows loaded: count={len(rows)}"
            )
            states_by_aircraft: dict[int, dict[str, Any]] = {}
            for row in rows:
                aircraft_id = _state_aircraft_id(row)
                if aircraft_id is None:
                    continue
                states_by_aircraft[int(aircraft_id)] = row
            if not states_by_aircraft:
                self._append_log_line("[CURRENT] reexecute snapshot context skipped: no 0401 snapshot rows")
                return False

            turn_radius_scale = _planner_turn_radius_scale()
            entries: list[dict[str, Any]] = []
            coords_for_centroid: list[dict[str, float]] = []
            for aircraft_id in available_uavs:
                row = states_by_aircraft.get(int(aircraft_id))
                if row is None:
                    self._append_log_line(
                        f"[CURRENT] reexecute entry skipped aircraft {aircraft_id}: no 0401 snapshot state"
                    )
                    continue
                position_coord = _state_coordinate(row)
                if position_coord is None:
                    self._append_log_line(
                        f"[CURRENT] reexecute entry skipped aircraft {aircraft_id}: no 0401 coordinate"
                    )
                    continue
                heading_deg = _state_heading_deg(row)
                eta_s = 0.0
                entry_coord = dict(position_coord)
                altitude = _coerce_float(entry_coord.get("altitude"))
                if altitude is not None:
                    entry_coord["altitude"] = int(round(float(altitude)))
                coords_for_centroid.append(entry_coord)
                source = "snapshotCurrentPosition0401"
                entry: dict[str, Any] = {
                    "aircraftID": int(aircraft_id),
                    "coordinate": entry_coord,
                    "source": source,
                }
                if heading_deg is not None:
                    entry["headingDeg"] = float(heading_deg) % 360.0
                if eta_s is not None:
                    entry["etaS"] = float(eta_s)
                entries.append(entry)

            if not entries:
                self._append_log_line("[CURRENT] reexecute snapshot context skipped: entryAircraftList empty")
                return False

            detail["currentRemainingCollaborativeReplan"] = True
            detail["currentInputMissionID"] = int(attach_input_id)
            detail["entryStrategy"] = "turn_projection"
            detail["turnRadiusScale"] = float(turn_radius_scale)
            detail["entryAircraftList"] = entries
            detail.pop("currentRemainingApplyOptionOrdinals", None)
            detail.pop("currentRemainingApplyOptions", None)
            detail.pop("currentRemainingHybridOptionOrdinals", None)

            source_plan_id = _current_source_plan_id()
            if source_plan_id is not None:
                detail["sourceMissionPlanID"] = int(source_plan_id)
                detail["currentMissionPlanID"] = int(source_plan_id)
                body["sourceMissionPlanID"] = int(source_plan_id)
                body["currentMissionPlanID"] = int(source_plan_id)
                self._append_log_line(
                    f"[CURRENT] reexecute source missionPlanID attached: {int(source_plan_id)}"
                )
            else:
                self._append_log_line(
                    "[CURRENT] reexecute source missionPlanID unavailable; mission planner will fallback"
                )

            representative_entry = _centroid_coordinate(coords_for_centroid)
            if representative_entry is not None:
                detail["representativeEntryCoordinate"] = dict(representative_entry)
            body["replanDetail"] = detail
            self._append_log_line(
                f"[CURRENT] reexecute snapshot entries resolved: count={len(entries)}"
            )
            self._append_log_line(
                "[CURRENT] reexecute current-position hybrid attached for all options"
            )
            return True

        if is_reexecute_refresh:
            try:
                if _attach_reexecute_snapshot_context():
                    return body
            except BaseException as exc:
                self._append_log_line(f"[CURRENT] reexecute snapshot context fatal guard: {exc}")
                try:
                    self._append_log_line(traceback.format_exc(limit=4).rstrip())
                except BaseException:
                    pass
            self._append_log_line(
                "[CURRENT] reexecute snapshot context unavailable; queueing generic reexecute request"
            )
            return body

        viz = getattr(self, "_viz_tab", None)
        if viz is None or not hasattr(viz, "build_current_remaining_replan_context"):
            return body
        else:
            context_payload = None

        try:
            affected_aircraft_id = int(detail.get("aircraftID")) if detail.get("aircraftID") is not None else None
        except Exception:
            affected_aircraft_id = None
        if context_payload is None:
            try:
                context_payload = viz.build_current_remaining_replan_context(
                    affected_aircraft_id=affected_aircraft_id,
                    available_aircraft_ids=sorted(self._effective_available_ids()),
                )
            except Exception as exc:
                self._append_log_line(f"[CURRENT] context build failed: {exc}")
                return body
        if not isinstance(context_payload, dict):
            return body
        turn_tab = getattr(self, "_turn_radius_tab", None)
        turn_views = None
        if turn_tab is not None and hasattr(turn_tab, "build_views"):
            try:
                turn_views = turn_tab.build_views(include_paths=False)
            except Exception:
                turn_views = None
        try:
            entries, representative_entry, turn_radius_scale, logs = resolve_remaining_entry_aircraft_list(
                aircraft_ids=[int(v) for v in (context_payload.get("target_aircraft_ids") or [])],
                turn_views=turn_views,
                entry_strategy=str(context_payload.get("entry_strategy") or "turn_projection"),
                log_prefix="[CURRENT]",
            )
        except Exception as exc:
            self._append_log_line(f"[CURRENT] entry resolution failed: {exc}")
            return body
        for line in logs:
            self._append_log_line(line)
        if not entries:
            return body
        detail["currentRemainingCollaborativeReplan"] = True
        detail["currentInputMissionID"] = int(context_payload.get("current_input_mission_id"))
        detail["entryStrategy"] = str(context_payload.get("entry_strategy") or "turn_projection")
        detail["turnRadiusScale"] = float(turn_radius_scale)
        detail["entryAircraftList"] = entries
        if representative_entry is not None:
            detail["representativeEntryCoordinate"] = dict(representative_entry)
        body["replanDetail"] = detail
        return body

    def _rx_setup(self):
        try:
            with fusion_runtime_working_dir(project_root=PROJECT_ROOT):
                FusionNodeIoc.Configure()
                NodeMessenger.Initialize("MSM_ReceiveNode")
                NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
                NodeMessenger.InitAllSubscriberFromAssembly()
                NodeMessenger.RegistAllProviderFromFusionNodeIoc()
            self._bus_ready = True
            try:
                self._append_log_line("[BUS] MSM NodeMessenger 초기화 완료")
            except Exception:
                pass
        except Exception as exc:
            self._bus_ready = False
            try:
                sys.stderr.write(f"[WARN] MSM bus init failed: {exc}\n")
            except Exception:
                pass

    def closeEvent(self, event):
        if hide_instead_of_close(self, event, log=self._append_log_line):
            return
        try:
            self._hb_0102_enabled = False
            self._hb_0102_stop.set()
        except Exception:
            pass
        try:
            self._hb_0501_enabled = False
            self._hb_0501_stop.set()
        except Exception:
            pass
        try:
            self._0401_trace_watchdog_stop.set()
        except Exception:
            pass
        try:
            self._stop_line_scan_progress_worker()
        except Exception:
            pass
        try:
            self._stop_area_snapshot_worker()
        except Exception:
            pass
        try:
            self._write_monitoring_lifecycle_event("graceful_close")
            self._write_monitoring_fatal_line("graceful_close reached")
            stop = getattr(self, "_monitoring_lifecycle_stop", None)
            if stop is not None:
                stop.set()
        except Exception:
            pass
        try:
            faulthandler.disable()
        except Exception:
            pass
        try:
            handle = getattr(self, "_fatal_log_handle", None)
            if handle is not None:
                handle.close()
        except Exception:
            pass
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    load_shared_stylesheet(app, PROJECT_ROOT)
    win = MainWindow()
    apply_initial_visibility(app, win, position_window_from_env)
    sys.exit(app.exec_())
