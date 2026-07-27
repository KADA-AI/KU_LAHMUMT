# -*- coding: utf-8 -*-
# mission_planning_gui.py – 임무 할당·계획수립 전용 GUI (S110 플로우 대응)
from __future__ import annotations

import sys, os, threading, json, re, time, shutil, copy, traceback, math, importlib
import faulthandler
import concurrent.futures
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Set, List

_SCRIPT_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_SCRIPT_PROJECT_ROOT_STR = str(_SCRIPT_PROJECT_ROOT)
if _SCRIPT_PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, _SCRIPT_PROJECT_ROOT_STR)

from modules.mission_planning.replanning.reexecute_lah_role import (
    has_reusable_lah_role_geometry,
    rebind_reexecute_lah_role_mission,
    resolve_reexecute_lah_template_input_id,
)
from modules.mission_planning.pipelines.handover_terminal import (
    mark_handover_terminal_missions,
)

_PROCESS_LOG_NAME = "mission_planning"
_DEFAULT_CONSOLE_TITLE = "KU Mission Planning Console"


def _install_early_process_logging() -> None:
    try:
        from modules.common.process_console import ensure_console, install_process_file_logging, emit_process_log

        ensure_console(os.getenv("KU_CONSOLE_TITLE", _DEFAULT_CONSOLE_TITLE))
        install_process_file_logging(_PROCESS_LOG_NAME)
    except Exception:
        return

    if getattr(sys, "_mission_planning_crash_logging_installed", False):
        return
    setattr(sys, "_mission_planning_crash_logging_installed", True)

    original_excepthook = sys.excepthook

    def _log_unhandled(exc_type, exc, tb) -> None:
        try:
            trace_text = "".join(traceback.format_exception(exc_type, exc, tb)).rstrip()
            emit_process_log(_PROCESS_LOG_NAME, "[CRASH] unhandled exception\n" + trace_text)
        except Exception:
            pass

    def _excepthook(exc_type, exc, tb) -> None:
        _log_unhandled(exc_type, exc, tb)
        try:
            original_excepthook(exc_type, exc, tb)
        except Exception:
            pass

    sys.excepthook = _excepthook

    if hasattr(threading, "excepthook"):
        original_threading_excepthook = threading.excepthook

        def _threading_excepthook(args) -> None:
            try:
                thread_name = getattr(getattr(args, "thread", None), "name", "unknown")
                trace_text = "".join(
                    traceback.format_exception(args.exc_type, args.exc_value, args.exc_traceback)
                ).rstrip()
                emit_process_log(
                    _PROCESS_LOG_NAME,
                    f"[CRASH] unhandled thread exception thread={thread_name}\n{trace_text}",
                )
            except Exception:
                pass
            try:
                original_threading_excepthook(args)
            except Exception:
                pass

        threading.excepthook = _threading_excepthook

    if hasattr(sys, "unraisablehook"):
        original_unraisablehook = sys.unraisablehook

        def _unraisablehook(args) -> None:
            try:
                trace_text = "".join(
                    traceback.format_exception(
                        type(args.exc_value),
                        args.exc_value,
                        args.exc_traceback,
                    )
                ).rstrip()
                emit_process_log(
                    _PROCESS_LOG_NAME,
                    f"[CRASH] unraisable exception object={args.object!r}\n{trace_text}",
                )
            except Exception:
                pass
            try:
                original_unraisablehook(args)
            except Exception:
                pass

        sys.unraisablehook = _unraisablehook


# 유지할 fatal 로그 핸들 (GC 로 닫히지 않도록 모듈 전역에 보관)
_MISSION_FAULT_LOG_HANDLE = None


def _install_early_faulthandler() -> None:
    """네이티브 크래시(0xC0000005 access violation 등)의 스택을 파일로 남긴다.

    monitoring 모듈과 달리 mission_planning 은 그동안 faulthandler 계측이 전혀
    없어, CLR 버스/네이티브 확장에서 발생한 access violation 이 아무 기록 없이
    프로세스를 죽였다. 여기서 전 스레드 스택을 파일에 남겨 원인을 특정한다.
    """
    global _MISSION_FAULT_LOG_HANDLE
    if getattr(sys, "_mission_planning_faulthandler_installed", False):
        return

    handle = None
    try:
        from modules.common import db_paths

        base = db_paths.get_db_subpath("DSS_Internal", "mission_planning_diagnostics")
        base.mkdir(parents=True, exist_ok=True)
        handle = (base / f"mission_planning_fatal_{os.getpid()}.log").open(
            "a", encoding="utf-8", buffering=1, errors="replace"
        )
    except Exception:
        # db_root 해석 실패 시에도 최소한 stderr 로는 덤프되도록 폴백한다.
        handle = None

    try:
        if handle is not None:
            handle.write(
                f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] faulthandler enabled pid={os.getpid()}\n"
            )
            faulthandler.enable(file=handle, all_threads=True)
            _MISSION_FAULT_LOG_HANDLE = handle
        else:
            faulthandler.enable(all_threads=True)
        setattr(sys, "_mission_planning_faulthandler_installed", True)
    except Exception as exc:
        try:
            from modules.common.process_console import emit_process_log

            emit_process_log("mission_planning", f"[TRACE] faulthandler enable failed: {exc}")
        except Exception:
            pass


_install_early_process_logging()
_install_early_faulthandler()


def _preload_gdal_before_qt() -> None:
    """osgeo(GDAL) 네이티브 DLL 을 PyQt5 보다 먼저 로드한다.

    이 프로세스에서 PyQt5 가 먼저 로드된 뒤 osgeo.gdal 을 import 하면
    (플래너 워밍업의 attack pipeline -> lah_attack_assistance 경로)
    DLL 의존성 충돌로 access violation(0xC0000005)이 나며 프로세스가
    즉사한다. 역순(osgeo -> PyQt5)은 안전함이 검증되었으므로 Qt import
    전에 선로드한다. GDAL 미설치 환경에서는 조용히 건너뛴다.
    """
    try:
        from osgeo import gdal  # noqa: F401
    except Exception:
        pass


_preload_gdal_before_qt()

from modules.mission_planning.app.bootstrap import (
    configure_mission_process_console,
    configure_mission_role,
)
from modules.mission_planning.app.message_handlers.system_mode import (
    MODE_LABELS,
    build_0102_body as _build_0102_body_payload,
    extract_mode_code as _extract_mode_code_from_body,
    extract_system_mode_code,
    normalize_0102_body_template,
    parse_payload_body as _parse_system_mode_payload_body,
    resolve_mode_code_from_text,
)
from modules.mission_planning.app.message_handlers.input_packages import (
    build_input_banner_info as _build_latest_input_banner_info,
    extract_payload_source as _extract_input_payload_source,
    prepare_cached_payload_for_file,
)
from modules.mission_planning.app.message_handlers.replan_requests import (
    extract_replan_request_selection,
    parse_replan_payload,
    replan_delay_policy,
)
from modules.mission_planning.app.delivery.mission_plan_delivery import (
    merge_post_delivery_snapshot_carry_forward,
    merge_post_delivery_waypoint_mark,
    normalize_post_delivery_snapshot_carry_forward,
    normalize_post_delivery_waypoint_mark,
    post_0301_delivery_delays,
    sort_plan_delivery_entries,
)

configure_mission_role()  # MMR

from shapely.geometry import GeometryCollection, LineString, MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from modules.common.process_console import (
    emit_process_lifecycle_event,
    emit_process_log,
)
from modules.mission_planning.mission_control.planner_runtime import (
    PLANNER_RUNTIME_WATCH_RELATIVE_PATHS,
    current_remaining_hybrid_global_lock_enabled,
    ensure_mission_planner_import_paths as _ensure_mission_planner_import_paths_impl,
    file_signature as _planner_runtime_file_signature,
    planner_runtime_source_signature as _planner_runtime_source_signature_impl,
    refresh_live_planning_helpers as _refresh_live_planning_helpers_impl,
    reload_planning_module as _reload_planning_module_impl,
)
from modules.mission_planning.mission_control.plan_metrics import (
    classify_replan_timing_context,
    count_replan_options,
)
from modules.mission_planning.runtime.cache.source_artifacts import (
    SourceArtifactCache,
    use_source_artifact_cache,
)
from modules.mission_planning.runtime.replan_timing_history import (
    record_replan_timing as _record_replan_timing_history,
)
from modules.mission_planning.replanning.dispatcher import (
    is_post_attack_rejoin_detail,
    is_prior_post_rejoin_detail,
    should_use_attack_pipeline,
    should_use_post_attack_rejoin_pipeline,
    should_use_prior_post_rejoin_pipeline,
)
from modules.common.string_limits import limit_utf8_bytes

configure_mission_process_console()

try:
    from .pipelines.mission_planning_attack_helpers import (
        apply_attack_customizations,
        build_attack_context_from_replan_detail,
        compute_attack_waypoint,
        load_attack_context,
    )
    from .replanning.triggers.remaining_hybrid.current import (
        CurrentRemainingHybridRequest,
        build_current_remaining_hybrid,
        filter_generic_flightpath_missions_for_hybrid,
        merge_current_remaining_hybrid,
        validate_current_remaining_hybrid_paths,
        validate_current_remaining_hybrid_request,
    )
    from .replanning.line_entry_context import build_line_entry_context_map_from_entry_rows
    from .replanning.triggers.recon_specialized.pipeline import (
        build_recon_specialized_runtime_payload,
        is_recon_specialized_option,
    )
    from .ui.mission_planning_gui_env import (
        _bootstrap_paths,
        _ensure_fusion_configs,
        _load_msglib_and_deps,
        _now_ms_since_2000,
        _sanitize_reason,
        _z4,
    )
    from .runtime.logging.pipeline_events import PipelineLogManager, emit_replan_checkpoint
    from .runtime.logging.plan_file_logger import MissionPlanFileLogger
    from .runtime.debug_artifacts import write_debug_json
    from .runtime.json_io import serialize_json_payload, write_json, write_json_bytes
    from .runtime.validation.replan_payloads import (
        ReplanValidationError,
        collect_missing_flight_path_repairs,
        sync_flight_plan_individual_mission_ids,
        validate_mission_flightpath_links,
        validate_replan_payloads,
        validate_unique_flightpath_ids,
    )
    from .runtime.cache.source_artifacts import (
        SourceArtifactCache,
        call_with_source_artifact_cache,
        use_source_artifact_cache,
    )
    from .runtime.cache.initial_plan_templates import (
        get_initial_plan_template,
        make_initial_plan_template_key,
        put_initial_plan_template,
    )
    from .runtime.aircraft_parallel_0303 import build_0303_flight_plans_aircraft_parallel
    from .ui import MissionAlgoConfigTab, MissionIdRelationshipTab
    from .MissionPlanner.runtime_settings import (
        DEFAULT_AREA_SPLIT_MODE,
        DEFAULT_AREA_SWEEP_MODE,
        DEFAULT_UAV_PLAN_MODE,
        MANUAL_FOV_ROLLBACK_KEY,
        PERSISTED_RUNTIME_VALUE_KEYS,
        apply_runtime_camera_adjusted_fov_deg,
        canonicalize_runtime_payload,
        clear_runtime_camera_fov_adjustment_logs,
        pop_runtime_camera_fov_adjustment_logs,
        get_runtime_prior_float,
        get_runtime_prior_int,
        get_runtime_prior_mission_profile,
        load_runtime_settings,
        runtime_override as runtime_settings_override,
        settings_path as runtime_settings_path,
    )
    from modules.monitoring.logic.replan_runtime_settings import get_target_detection_settings
    from modules.mission_planning.runtime.state.attack_assignment import (
        get_last_assigned_manned_id,
        set_last_assigned_manned_id,
    )
    from .planning_modes import mission_mode_context, resolve_mission_planning_mode
except Exception:
    from modules.mission_planning.pipelines.mission_planning_attack_helpers import (
        apply_attack_customizations,
        build_attack_context_from_replan_detail,
        compute_attack_waypoint,
        load_attack_context,
    )
    from modules.mission_planning.replanning.triggers.remaining_hybrid.current import (
        CurrentRemainingHybridRequest,
        build_current_remaining_hybrid,
        filter_generic_flightpath_missions_for_hybrid,
        merge_current_remaining_hybrid,
        validate_current_remaining_hybrid_paths,
        validate_current_remaining_hybrid_request,
    )
    from modules.mission_planning.replanning.line_entry_context import (
        build_line_entry_context_map_from_entry_rows,
    )
    from modules.mission_planning.replanning.triggers.recon_specialized.pipeline import (
        build_recon_specialized_runtime_payload,
        is_recon_specialized_option,
    )
    from modules.mission_planning.ui.mission_planning_gui_env import (
        _bootstrap_paths,
        _ensure_fusion_configs,
        _load_msglib_and_deps,
        _now_ms_since_2000,
        _sanitize_reason,
        _z4,
    )
    from modules.mission_planning.runtime.logging.pipeline_events import (
        PipelineLogManager,
        emit_replan_checkpoint,
    )
    from modules.mission_planning.runtime.logging.plan_file_logger import MissionPlanFileLogger
    from modules.mission_planning.runtime.debug_artifacts import write_debug_json
    from modules.mission_planning.runtime.json_io import serialize_json_payload, write_json, write_json_bytes
    from modules.mission_planning.runtime.validation.replan_payloads import (
        ReplanValidationError,
        collect_missing_flight_path_repairs,
        sync_flight_plan_individual_mission_ids,
        validate_mission_flightpath_links,
        validate_replan_payloads,
        validate_unique_flightpath_ids,
    )
    from modules.mission_planning.runtime.cache.source_artifacts import (
        SourceArtifactCache,
        call_with_source_artifact_cache,
        use_source_artifact_cache,
    )
    from modules.mission_planning.runtime.cache.initial_plan_templates import (
        get_initial_plan_template,
        make_initial_plan_template_key,
        put_initial_plan_template,
    )
    from modules.mission_planning.runtime.aircraft_parallel_0303 import build_0303_flight_plans_aircraft_parallel
    from modules.mission_planning.ui import MissionAlgoConfigTab, MissionIdRelationshipTab
    from modules.mission_planning.MissionPlanner.runtime_settings import (
        DEFAULT_AREA_SPLIT_MODE,
        DEFAULT_AREA_SWEEP_MODE,
        DEFAULT_UAV_PLAN_MODE,
        MANUAL_FOV_ROLLBACK_KEY,
        PERSISTED_RUNTIME_VALUE_KEYS,
        apply_runtime_camera_adjusted_fov_deg,
        canonicalize_runtime_payload,
        clear_runtime_camera_fov_adjustment_logs,
        pop_runtime_camera_fov_adjustment_logs,
        get_runtime_prior_float,
        get_runtime_prior_int,
        get_runtime_prior_mission_profile,
        load_runtime_settings,
        runtime_override as runtime_settings_override,
        settings_path as runtime_settings_path,
    )
    from modules.monitoring.logic.replan_runtime_settings import get_target_detection_settings
    from modules.mission_planning.runtime.state.attack_assignment import (
        get_last_assigned_manned_id,
        set_last_assigned_manned_id,
    )
    from modules.mission_planning.planning_modes import (
        mission_mode_context,
        resolve_mission_planning_mode,
    )

PROJECT_ROOT, COMMON_DIR = _bootstrap_paths(Path(__file__))
from modules.common.settings_paths import fusion_runtime_working_dir

_TEMP_DIR = PROJECT_ROOT / "temp"
_CURRENT_REMAINING_HYBRID_BUILD_LOCK = threading.RLock()


def _current_remaining_hybrid_global_lock_enabled() -> bool:
    return current_remaining_hybrid_global_lock_enabled()


_PLANNER_RUNTIME_WATCH_RELATIVE_PATHS = PLANNER_RUNTIME_WATCH_RELATIVE_PATHS


def _ensure_temp_dir() -> Path:
    _TEMP_DIR.mkdir(parents=True, exist_ok=True)
    return _TEMP_DIR


def _file_sig(path: Path) -> tuple[int, int] | None:
    return _planner_runtime_file_signature(path)


def _planner_runtime_source_signature() -> tuple[tuple[str, tuple[int, int] | None], ...]:
    return _planner_runtime_source_signature_impl(PROJECT_ROOT)


def _reload_planning_module(module_name: str):
    return _reload_planning_module_impl(module_name)


def _refresh_live_planning_helpers() -> None:
    _refresh_live_planning_helpers_impl(globals())


def _ensure_mission_planner_import_paths() -> None:
    _ensure_mission_planner_import_paths_impl(PROJECT_ROOT)

from modules.common.qt_env import ensure_qt_platform
ensure_qt_platform()
from modules.common.gui_style import load_shared_stylesheet, polish_tabs, position_window_from_env

from PyQt5.QtCore import (
    qInstallMessageHandler, QtMsgType, pyqtSignal, QTimer, Qt, QEvent, QObject, QRect, QThread
)
from PyQt5.QtGui import QKeySequence, QPainter, QColor, QFontMetrics, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QShortcut,
    QWidget, QLabel, QHBoxLayout, QVBoxLayout, QSlider, QPushButton, QCheckBox,
    QStyle, QStyleOptionSlider, QDialog, QMessageBox, QFormLayout,
)
from modules.mission_planning.app.visualization.mission_visualization_tab import MissionVisualizationTab


def _preimport_heavy_native_runtimes() -> None:
    """무거운 네이티브 런타임(torch / stable_baselines3)을 메인 스레드에서 선로드한다.

    mission_planning 은 startup 시 ``Planner-Warmup`` 백그라운드 스레드가
    ``AnS.mission_pipeline`` -> ``from stable_baselines3 import PPO`` 경로로 torch/SB3
    네이티브 스택을 *처음* import 한다. 이것이 ``MMR-RX-Setup`` 스레드의 nFusion
    CLR(.NET) 초기화와 동시에 실행되면 두 네이티브 런타임 초기화가 경합해 프로세스가
    access violation(0xC0000005)으로 즉사한다(대시보드 로그의 mission GUI exit code
    3221225477 이 전부 이 크래시였다). monitoring 은 torch 스택을 모듈 최상위(=메인
    스레드)에서 로드하므로 이 경합이 없다.

    여기서 메인 스레드에서(그리고 백그라운드 스레드가 뜨기 전에) 먼저 로드해 두면
    이후 워밍업 스레드의 import 는 캐시 히트가 되어 동시 네이티브 초기화가 사라진다.
    미설치 환경에서는 조용히 건너뛴다.
    """
    for module_name in ("torch", "stable_baselines3"):
        try:
            importlib.import_module(module_name)
        except Exception:
            pass


_preimport_heavy_native_runtimes()


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
    text = str(message)
    try:
        emit_process_log("mission_planning", f"[QT] {text}")
    except Exception:
        pass
    try:
        sys.stderr.write(text + "\n")
    except Exception:
        pass
qInstallMessageHandler(_qt_silent_handler)

# ───────── 경로 부트스트랩 ─────────
from modules.common.status_reporter import send_status_ok
from modules.common import db_paths
from modules.common import mission_area_replan_store
from modules.common import imaging_schedule_replan_store
from modules.common import path_deviation_replan_store
from modules.common.ctrl_listener import start_ctrl_listener, env_ctrl_port
from modules.common.gui_process_control import (
    apply_initial_visibility,
    handle_window_control,
    hide_instead_of_close,
)
from modules.common.message_payload_dialog import JsonPayloadBatchDialog
from modules.common.option_codes import (
    DEFAULT_OPTION_CODE_SEQUENCE,
    ensure_option_code_sequence,
    is_option_code_value,
    normalize_option_code,
    option_code_to_label,
)
from modules.mission_planning.replanning.input_refresh_progress import (
    attach_input_refresh_current_input_id,
    infer_started_input_mission_id,
    input_refresh_current_input_id,
    input_refresh_snapshot_whitelist,
    is_input_refresh_context,
    parallel_snapshot_safety_reasons,
)
from receive_center import register_listener, unregister_listener   # ★ 0101 모드 수신 리스너
try:
    from .runtime.cache.latest_input import (
        reset_latest_inputs,
        update_from_payload as cache_update_from_payload,
        get_latest_package_id,
        get_latest_snapshot,
        describe_latest_ids,
        resolve_path_from_cache,
    )
    from .runtime.next_collab_replan_runtime import (
        is_next_collab_reason_text as runtime_is_next_collab_reason_text,
        load_next_collab_detail,
        record_next_collab_event,
    )
    from .replanning.triggers.prior.pipeline import (
        run_prior_mission_pipeline,
        warm_prior_mission_pipeline,
        run_prior_post_rejoin_pipeline,
        warm_prior_post_rejoin_pipeline,
    )
    from .replanning.triggers.imaging_schedule.pipeline import (
        run_imaging_schedule_replan_pipeline,
        warm_imaging_schedule_replan_pipeline,
    )
    from .replanning.triggers.path_deviation.pipeline import (
        run_path_deviation_replan_pipeline,
        warm_path_deviation_replan_pipeline,
    )
    from .replanning.triggers.next_collab.pipeline import (
        run_next_collab_replan_pipeline,
        warm_next_collab_replan_pipeline,
    )
    from .replanning.triggers.attack.pipeline import (
        run_attack_exclusion_pipeline,
        run_attack_plan_pipeline,
        warm_attack_plan_pipeline,
    )
    from .replanning.triggers.post_attack.pipeline import (
        run_post_attack_rejoin_pipeline,
        warm_post_attack_rejoin_pipeline,
    )
except Exception:
    from modules.mission_planning.runtime.cache.latest_input import (
        reset_latest_inputs,
        update_from_payload as cache_update_from_payload,
        get_latest_package_id,
        get_latest_snapshot,
        describe_latest_ids,
        resolve_path_from_cache,
    )
    from modules.mission_planning.runtime.next_collab_replan_runtime import (
        is_next_collab_reason_text as runtime_is_next_collab_reason_text,
        load_next_collab_detail,
        record_next_collab_event,
    )
    from modules.mission_planning.replanning.triggers.prior.pipeline import (
        run_prior_mission_pipeline,
        warm_prior_mission_pipeline,
        run_prior_post_rejoin_pipeline,
        warm_prior_post_rejoin_pipeline,
    )
    from modules.mission_planning.replanning.triggers.imaging_schedule.pipeline import (
        run_imaging_schedule_replan_pipeline,
        warm_imaging_schedule_replan_pipeline,
    )
    from modules.mission_planning.replanning.triggers.path_deviation.pipeline import (
        run_path_deviation_replan_pipeline,
        warm_path_deviation_replan_pipeline,
    )
    from modules.mission_planning.replanning.triggers.next_collab.pipeline import (
        run_next_collab_replan_pipeline,
        warm_next_collab_replan_pipeline,
    )
    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        run_attack_exclusion_pipeline,
        run_attack_plan_pipeline,
        warm_attack_plan_pipeline,
    )
    from modules.mission_planning.replanning.triggers.post_attack.pipeline import (
        run_post_attack_rejoin_pipeline,
        warm_post_attack_rejoin_pipeline,
    )

# ───────── nFusion 설정/라이선스 정규화 + MessageLibrary 로드 ─────────
from dll_files.nFusionImports import *  # FusionNodeIoc, NodeMessenger, clr 등

_settings_path = _ensure_fusion_configs(PROJECT_ROOT, COMMON_DIR)
_ = _load_msglib_and_deps(COMMON_DIR)

# 수신 등록 모듈(내부에서 각 탭의 RECEIVE 등록을 수행)
from receive import *  # noqa

# 탭
from Tabs.assignment_planning_tab import AssignmentPlanningTab
from datetime import datetime, timezone


_PATH_DEVIATION_REASON_KEYWORDS = (
    "\uacbd\ub85c \ubbf8\ucd94\uc885",
    "\uacbd\ub85c \uc2a4\ucf00\uc904 \ubbf8\uc900\uc218",
)
_IMAGING_SCHEDULE_REASON_KEYWORDS = (
    "\ucd2c\uc601 \uc2a4\ucf00\uc904 \ubbf8\uc900\uc218",
)
_QUALITY_SPEED_REASON_KEYWORDS = (
    "\ucd2c\uc601 \ud488\uc9c8 \uac1c\uc120",
)
_SNAPSHOT_SKIP_TRIGGER_TYPES = {
    "collabReexecuteInputRefresh",
}
def _is_path_deviation_reason_text(value: object | None) -> bool:
    text = str(value or "")
    return any(keyword in text for keyword in _PATH_DEVIATION_REASON_KEYWORDS)


def _is_imaging_schedule_reason_text(value: object | None) -> bool:
    text = str(value or "")
    return any(keyword in text for keyword in _IMAGING_SCHEDULE_REASON_KEYWORDS)


def _safe_int_value(value: Any) -> Optional[int]:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _normalize_mission_plan_ids(values: Any) -> list[int]:
    """Return unique positive MissionPlan IDs without trusting caller state."""

    if values is None:
        return []
    if isinstance(values, (str, bytes)) or not isinstance(values, (list, tuple, set)):
        values = [values]
    normalized: list[int] = []
    seen: set[int] = set()
    for value in values:
        plan_id = _safe_int_value(value)
        if plan_id is None or plan_id <= 0 or plan_id in seen:
            continue
        seen.add(plan_id)
        normalized.append(plan_id)
    return normalized


def _replan_elapsed_ms(ctx: Any, *, event_perf: Optional[float] = None) -> Optional[float]:
    """Measure request-local elapsed time from the captured 0902 callback boundary."""

    if not isinstance(ctx, dict):
        return None
    timing = ctx.get("_replan_timing")
    if not isinstance(timing, dict):
        return None
    try:
        base_perf = float(timing.get("base_perf"))
        checkpoint_perf = time.perf_counter() if event_perf is None else float(event_perf)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(base_perf) or not math.isfinite(checkpoint_perf):
        return None
    return round(max(0.0, (checkpoint_perf - base_perf) * 1000.0), 3)


def _sync_mission_plan_planning_time(plan_ids: Any, planning_time_ms: Any) -> Dict[str, Any]:
    """Atomically synchronize MissionPlan.planningTime for one delivered request.

    ``planningTime`` remains milliseconds for compatibility with the existing
    0301 files and logAnalyzer.  The caller decides whether the value is the
    pre-0301 provisional checkpoint or the exact 0305 status=2 checkpoint.
    """

    normalized_ids = _normalize_mission_plan_ids(plan_ids)
    result: Dict[str, Any] = {
        "requestedPlanIDs": normalized_ids,
        "updatedPlanIDs": [],
        "unchangedPlanIDs": [],
        "missingPlanIDs": [],
        "errors": [],
    }
    try:
        normalized_ms = round(float(planning_time_ms), 3)
    except (TypeError, ValueError):
        result["errors"].append("planningTime is not numeric")
        return result
    if not math.isfinite(normalized_ms) or normalized_ms < 0.0:
        result["errors"].append("planningTime must be a finite non-negative millisecond value")
        return result
    result["planningTimeMs"] = normalized_ms

    for plan_id in normalized_ids:
        try:
            plan_path = db_paths.get_db_subpath("MissionPlan", f"{plan_id}.json")
            if not plan_path.exists():
                result["missingPlanIDs"].append(plan_id)
                continue
            payload = json.loads(plan_path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("MissionPlan root must be an object")
            payload_id = _safe_int_value(payload.get("missionPlanID"))
            if payload_id is not None and payload_id > 0 and payload_id != plan_id:
                raise ValueError(f"missionPlanID mismatch: file={plan_id}, payload={payload_id}")
            payload["planningTime"] = float(normalized_ms)
            written = write_json(
                plan_path,
                payload,
                pretty=True,
                ensure_ascii=False,
                skip_if_unchanged=True,
            )
            bucket = "updatedPlanIDs" if written else "unchangedPlanIDs"
            result[bucket].append(plan_id)
        except Exception as exc:
            result["errors"].append(f"{plan_id}: {exc}")
    return result


def _load_source_plan_package_ids(source_plan_id: Any) -> Dict[str, int]:
    """Resolve the 0201/0203 IDs owned by an already-applied source plan."""

    normalized_plan_id = _safe_int_value(source_plan_id)
    if normalized_plan_id is None or normalized_plan_id <= 0:
        return {}
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{normalized_plan_id}.json")
        if not plan_path.exists():
            return {}
        payload = json.loads(plan_path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    if not isinstance(payload, dict):
        return {}

    resolved: Dict[str, int] = {}
    for key in ("inputMissionPackageID", "missionReferencePackageID"):
        value = _safe_int_value(payload.get(key))
        if value is not None and value > 0:
            resolved[key] = int(value)
    if resolved:
        resolved["sourceMissionPlanID"] = int(normalized_plan_id)
    return resolved


def _pick_latest_package_json(directory: Path) -> Optional[Path]:
    """Pick the highest numeric package ID, with mtime as a stable fallback."""

    candidates = [path for path in directory.glob("*.json") if path.is_file()]
    if not candidates:
        return None

    def _sort_key(path: Path) -> tuple[int, int, int, str]:
        package_id = _safe_int_value(path.stem)
        try:
            modified_ns = int(path.stat().st_mtime_ns)
        except Exception:
            modified_ns = 0
        return (
            1 if package_id is not None and package_id > 0 else 0,
            int(package_id or -1),
            modified_ns,
            path.name,
        )

    return max(candidates, key=_sort_key)


_TRACKED_UAV_IDS = frozenset({4, 5, 6})
_INPUT_0201_REVIEW_0204_SENT_FLAG = "inputMissionPackageReview0204Sent"
_NO_AVAILABLE_UAV_NOTICE = "현재 가용한 무인기가 없습니다."


def _load_vehicle_status_available_ids() -> tuple[bool, set[int], set[int], Optional[Path]]:
    try:
        status_path = db_paths.get_db_subpath("VehicleStatus", "status.json")
    except Exception:
        return False, set(), set(), None
    if not status_path.exists():
        return False, set(), status_path
    try:
        payload = json.loads(status_path.read_text(encoding="utf-8"))
    except Exception:
        return False, set(), status_path
    raw_available = payload.get("available")
    if not isinstance(raw_available, list):
        return False, set(), status_path
    available_ids: set[int] = set()
    for item in raw_available:
        value = _safe_int_value(item)
        if value is None or value <= 0:
            continue
        available_ids.add(int(value))
    return True, available_ids, {aid for aid in available_ids if aid in _TRACKED_UAV_IDS}, status_path


def _has_nonempty_remaining_detail(detail: Any) -> bool:
    if not isinstance(detail, dict):
        return False
    line_list = detail.get("lineList")
    if isinstance(line_list, list) and line_list:
        return True
    area_list = detail.get("areaList")
    if isinstance(area_list, list) and area_list:
        return True
    coord_list = detail.get("coordinateList")
    return isinstance(coord_list, list) and len(coord_list) >= 2


def _normalize_coord_dict(value: Any) -> Optional[Dict[str, Any]]:
    if not isinstance(value, dict):
        return None
    try:
        lat = float(value.get("latitude"))
        lon = float(value.get("longitude"))
    except Exception:
        return None
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    out: Dict[str, Any] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    try:
        alt = float(value.get("altitude", 0.0) or 0.0)
    except Exception:
        alt = 0.0
    out["altitude"] = int(round(float(alt)))
    return out


def _normalize_coord_list(coords: Any, *, min_len: int = 0) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    if not isinstance(coords, list):
        return out
    for item in coords:
        coord = _normalize_coord_dict(item)
        if coord is None:
            continue
        out.append(coord)
    if len(out) < int(min_len):
        return []
    return out


def _dedupe_coord_path(coords: List[Dict[str, Any]], *, closed: bool = False) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    last_key: tuple[int, int, int] | None = None
    for item in coords:
        coord = _normalize_coord_dict(item)
        if coord is None:
            continue
        key = (
            int(round(float(coord["latitude"]) * 1_000_000.0)),
            int(round(float(coord["longitude"]) * 1_000_000.0)),
            int(round(float(coord.get("altitude", 0) or 0))),
        )
        if key == last_key:
            continue
        last_key = key
        out.append(coord)
    if closed and len(out) >= 2:
        first = out[0]
        last = out[-1]
        if (
            abs(float(first["latitude"]) - float(last["latitude"])) <= 1e-8
            and abs(float(first["longitude"]) - float(last["longitude"])) <= 1e-8
        ):
            out.pop()
    return out


def _coord_distance_sq(left: Dict[str, Any] | None, right: Dict[str, Any] | None) -> float:
    if not isinstance(left, dict) or not isinstance(right, dict):
        return float("inf")
    try:
        d_lat = float(left["latitude"]) - float(right["latitude"])
        d_lon = float(left["longitude"]) - float(right["longitude"])
    except Exception:
        return float("inf")
    return (d_lat * d_lat) + (d_lon * d_lon)


def _mission_geometry_bucket(mission: Dict[str, Any]) -> Optional[str]:
    if not isinstance(mission, dict):
        return None
    try:
        mtype = int(mission.get("inputMissionType", 0) or 0)
    except Exception:
        mtype = 0
    if mtype in (1, 7):
        return "line"
    if mtype in (2, 3, 4, 5, 6):
        return "area"
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    if isinstance(detail.get("lineList"), list) and detail.get("lineList"):
        return "line"
    if isinstance(detail.get("areaList"), list) and detail.get("areaList"):
        return "area"
    coord_list = detail.get("coordinateList")
    if isinstance(coord_list, list) and len(coord_list) >= 2:
        return "line"
    return None


def _extract_line_segments_from_detail(detail: Dict[str, Any]) -> tuple[List[Dict[str, Any]], float | None]:
    segments: List[Dict[str, Any]] = []
    width_values: List[float] = []
    if not isinstance(detail, dict):
        return segments, None
    line_list = detail.get("lineList")
    if isinstance(line_list, list):
        for row in line_list:
            if not isinstance(row, dict):
                continue
            coords = _dedupe_coord_path(_normalize_coord_list(row.get("coordinateList"), min_len=2), closed=False)
            if len(coords) < 2:
                continue
            try:
                width_val = float(row.get("width", 0.0) or 0.0)
            except Exception:
                width_val = 0.0
            if width_val > 0.0:
                width_values.append(float(width_val))
            segments.append(
                {
                    "coordinateList": coords,
                    "width": max(0, min(50000, int(round(float(width_val))))) if width_val > 0.0 else None,
                }
            )
    if not segments:
        coords = _dedupe_coord_path(_normalize_coord_list(detail.get("coordinateList"), min_len=2), closed=False)
        if len(coords) >= 2:
            segments.append({"coordinateList": coords, "width": None})
    width_hint = max(width_values) if width_values else None
    return segments, width_hint


def _resolve_source_line_metadata(
    detail: Dict[str, Any],
    *,
    fallback_to_current: bool = False,
) -> tuple[float | None, List[Dict[str, Any]]]:
    width_m: float | None = None
    if isinstance(detail, dict):
        try:
            raw_width = detail.get("sourceLineWidthM")
            if raw_width is not None:
                parsed_width = float(raw_width)
                if parsed_width > 0.0:
                    width_m = float(parsed_width)
        except Exception:
            width_m = None

    coord_candidates: List[Any] = []
    if isinstance(detail, dict):
        coord_candidates.append(detail.get("sourceCoordinateList"))
        source_line_list = detail.get("sourceLineList")
        if isinstance(source_line_list, list):
            for row in source_line_list:
                if isinstance(row, dict):
                    coord_candidates.append(row.get("coordinateList"))
        if fallback_to_current:
            line_list = detail.get("lineList")
            if isinstance(line_list, list):
                for row in line_list:
                    if isinstance(row, dict):
                        coord_candidates.append(row.get("coordinateList"))
            coord_candidates.append(detail.get("coordinateList"))

    source_coords: List[Dict[str, Any]] = []
    for candidate in coord_candidates:
        coords = _dedupe_coord_path(_normalize_coord_list(candidate, min_len=2), closed=False)
        if len(coords) >= 2:
            source_coords = coords
            break

    if fallback_to_current and (width_m is None or width_m <= 0.0):
        _, width_hint = _extract_line_segments_from_detail(detail)
        if width_hint is not None and width_hint > 0.0:
            width_m = float(width_hint)

    return (float(width_m) if width_m is not None and width_m > 0.0 else None), source_coords


def _line_merge_reference_ll(coords: List[Dict[str, Any]]) -> tuple[float, float]:
    lat_values = [float(item["latitude"]) for item in coords if isinstance(item, dict) and item.get("latitude") is not None]
    lon_values = [float(item["longitude"]) for item in coords if isinstance(item, dict) and item.get("longitude") is not None]
    if not lat_values or not lon_values:
        return 0.0, 0.0
    return (
        float(sum(lat_values) / float(len(lat_values))),
        float(sum(lon_values) / float(len(lon_values))),
    )


def _coord_to_local_xy_m(
    coord: Dict[str, Any],
    *,
    ref_lat: float,
    ref_lon: float,
) -> tuple[float, float]:
    lat = float(coord.get("latitude", 0.0) or 0.0)
    lon = float(coord.get("longitude", 0.0) or 0.0)
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(math.radians(float(ref_lat)))
    return (
        (lon - float(ref_lon)) * float(m_per_deg_lon),
        (lat - float(ref_lat)) * float(m_per_deg_lat),
    )


def _coord_path_xy_m(
    coords: List[Dict[str, Any]],
    *,
    ref_lat: float,
    ref_lon: float,
) -> List[tuple[float, float]]:
    return [
        _coord_to_local_xy_m(coord, ref_lat=float(ref_lat), ref_lon=float(ref_lon))
        for coord in coords
        if isinstance(coord, dict)
    ]


def _coord_path_length_m(
    coords: List[Dict[str, Any]],
    *,
    ref_lat: float,
    ref_lon: float,
) -> float:
    points_xy = _coord_path_xy_m(coords, ref_lat=float(ref_lat), ref_lon=float(ref_lon))
    if len(points_xy) < 2:
        return 0.0
    total = 0.0
    for idx in range(1, len(points_xy)):
        x0, y0 = points_xy[idx - 1]
        x1, y1 = points_xy[idx]
        total += math.hypot(float(x1) - float(x0), float(y1) - float(y0))
    return float(total)


def _coord_path_midpoint_xy_m(
    coords: List[Dict[str, Any]],
    *,
    ref_lat: float,
    ref_lon: float,
) -> tuple[float, float] | None:
    points_xy = _coord_path_xy_m(coords, ref_lat=float(ref_lat), ref_lon=float(ref_lon))
    if not points_xy:
        return None
    if len(points_xy) == 1:
        return points_xy[0]

    seg_lengths: List[float] = []
    total = 0.0
    for idx in range(1, len(points_xy)):
        x0, y0 = points_xy[idx - 1]
        x1, y1 = points_xy[idx]
        seg_len = math.hypot(float(x1) - float(x0), float(y1) - float(y0))
        seg_lengths.append(float(seg_len))
        total += float(seg_len)
    if total <= 1e-6:
        return points_xy[len(points_xy) // 2]

    target = float(total) / 2.0
    walked = 0.0
    for idx, seg_len in enumerate(seg_lengths, start=1):
        if walked + float(seg_len) < target:
            walked += float(seg_len)
            continue
        x0, y0 = points_xy[idx - 1]
        x1, y1 = points_xy[idx]
        ratio = 0.0 if seg_len <= 1e-6 else (target - walked) / float(seg_len)
        return (
            float(x0) + (float(x1) - float(x0)) * float(ratio),
            float(y0) + (float(y1) - float(y0)) * float(ratio),
        )
    return points_xy[-1]


def _merge_line_segments_to_detail(details: List[Dict[str, Any]]) -> Dict[str, Any]:
    segments: List[Dict[str, Any]] = []
    width_values: List[float] = []
    source_width_values: List[float] = []
    source_coordinate_rows: List[List[Dict[str, Any]]] = []
    for detail in details:
        seg_rows, width_hint = _extract_line_segments_from_detail(detail)
        segments.extend(seg_rows)
        if width_hint is not None and width_hint > 0.0:
            width_values.append(float(width_hint))
        source_width_m, source_coords = _resolve_source_line_metadata(detail, fallback_to_current=False)
        if source_width_m is not None and source_width_m > 0.0:
            source_width_values.append(float(source_width_m))
        if len(source_coords) >= 2:
            source_coordinate_rows.append(copy.deepcopy(source_coords))

    if not segments:
        return {"coordinateList": [], "lineList": [], "areaList": []}

    all_coords = [
        dict(coord)
        for seg in segments
        for coord in (seg.get("coordinateList") or [])
        if isinstance(coord, dict)
    ]
    ref_lat, ref_lon = _line_merge_reference_ll(all_coords)
    segment_rows: List[Dict[str, Any]] = []
    for seg in segments:
        coords = _dedupe_coord_path(list(seg.get("coordinateList") or []), closed=False)
        if len(coords) < 2:
            continue
        width_m = seg.get("width")
        try:
            width_val = float(width_m) if width_m is not None else 0.0
        except Exception:
            width_val = 0.0
        if width_val <= 0.0:
            width_val = max(width_values) if width_values else 1.0
        segment_rows.append(
            {
                "coordinateList": coords,
                "width": max(0, min(50000, int(round(float(width_val))))),
                "length_m": _coord_path_length_m(coords, ref_lat=float(ref_lat), ref_lon=float(ref_lon)),
                "midpoint_xy": _coord_path_midpoint_xy_m(coords, ref_lat=float(ref_lat), ref_lon=float(ref_lon)),
            }
        )

    if not segment_rows:
        return {"coordinateList": [], "lineList": [], "areaList": []}

    representative = max(
        segment_rows,
        key=lambda row: (
            float(row.get("length_m", 0.0) or 0.0),
            len(row.get("coordinateList") or []),
        ),
    )
    centerline_candidate = representative
    source_coords: List[Dict[str, Any]] = []
    if source_coordinate_rows:
        source_coords = max(
            source_coordinate_rows,
            key=lambda coords: (
                _coord_path_length_m(coords, ref_lat=float(ref_lat), ref_lon=float(ref_lon)),
                len(coords),
            ),
        )
    if len(source_coords) >= 2:
        try:
            source_line = LineString(
                _coord_path_xy_m(source_coords, ref_lat=float(ref_lat), ref_lon=float(ref_lon))
            )
        except Exception:
            source_line = None
        if source_line is not None and not source_line.is_empty:
            try:
                centerline_candidate = min(
                    segment_rows,
                    key=lambda row: (
                        float(
                            source_line.distance(
                                Point(row["midpoint_xy"])
                            )
                        )
                        if row.get("midpoint_xy") is not None
                        else float("inf"),
                        -float(row.get("length_m", 0.0) or 0.0),
                        -len(row.get("coordinateList") or []),
                    ),
                )
            except Exception:
                centerline_candidate = representative
    merged_coords = _dedupe_coord_path(
        copy.deepcopy(centerline_candidate.get("coordinateList") or []),
        closed=False,
    )
    if len(merged_coords) < 2:
        return {"coordinateList": [], "lineList": [], "areaList": []}

    width = float(representative.get("width", 1.0) or 1.0)
    source_width_m = max(source_width_values) if source_width_values else None
    if source_width_m is not None and source_width_m > 0.0:
        width = float(source_width_m)
    else:
        rep_xy = _coord_path_xy_m(merged_coords, ref_lat=float(ref_lat), ref_lon=float(ref_lon))
        if len(rep_xy) >= 2:
            try:
                rep_line = LineString(rep_xy)
            except Exception:
                rep_line = None
            if rep_line is not None and not rep_line.is_empty:
                half_width = float(width) / 2.0
                for row in segment_rows:
                    midpoint_xy = row.get("midpoint_xy")
                    if midpoint_xy is None:
                        continue
                    try:
                        offset_m = float(rep_line.distance(Point(midpoint_xy)))
                    except Exception:
                        offset_m = 0.0
                    half_width = max(
                        float(half_width),
                        float(offset_m) + (float(row.get("width", 1.0) or 1.0) / 2.0),
                    )
                width = max(float(width), float(half_width) * 2.0)

    merged_detail = {
        "coordinateList": copy.deepcopy(merged_coords),
        "lineList": [
            {
                "width": max(0, min(50000, int(round(float(width))))),
                "coordinateList": copy.deepcopy(merged_coords),
            }
        ],
        "areaList": [],
    }
    if source_width_m is not None and source_width_m > 0.0:
        merged_detail["sourceLineWidthM"] = float(source_width_m)
    if len(source_coords) >= 2:
        merged_detail["sourceCoordinateList"] = copy.deepcopy(source_coords)
    return merged_detail


def _coord_list_to_polygon(coords: Any) -> Optional[Polygon]:
    coord_list = _dedupe_coord_path(_normalize_coord_list(coords, min_len=3), closed=True)
    if len(coord_list) < 3:
        return None
    xy = [
        (float(item["longitude"]), float(item["latitude"]))
        for item in coord_list
    ]
    try:
        poly = Polygon(xy)
    except Exception:
        return None
    if poly.is_empty:
        return None
    if not poly.is_valid:
        poly = poly.buffer(0)
    if poly.is_empty:
        return None
    if isinstance(poly, Polygon):
        return poly
    if isinstance(poly, MultiPolygon):
        polys = [child for child in poly.geoms if isinstance(child, Polygon) and not child.is_empty]
        if polys:
            return max(polys, key=lambda child: float(child.area or 0.0))
    return None


def _iter_polygons(geometry: BaseGeometry | None) -> List[Polygon]:
    if geometry is None or geometry.is_empty:
        return []
    if isinstance(geometry, Polygon):
        return [geometry]
    if isinstance(geometry, MultiPolygon):
        return [poly for poly in geometry.geoms if isinstance(poly, Polygon) and not poly.is_empty]
    if isinstance(geometry, GeometryCollection):
        out: List[Polygon] = []
        for child in geometry.geoms:
            out.extend(_iter_polygons(child))
        return out
    return []


def _collapse_area_geometry_to_single_polygon(geometry: BaseGeometry | None) -> Optional[Polygon]:
    polygons = _iter_polygons(geometry)
    if not polygons:
        return None
    merged = unary_union(polygons)
    if isinstance(merged, Polygon):
        return merged
    if isinstance(merged, MultiPolygon):
        try:
            hull = merged.convex_hull
            if isinstance(hull, Polygon) and not hull.is_empty:
                return hull
        except Exception:
            pass
        return max(polygons, key=lambda poly: float(poly.area or 0.0))
    if isinstance(merged, GeometryCollection):
        polys = _iter_polygons(merged)
        if polys:
            return max(polys, key=lambda poly: float(poly.area or 0.0))
    return None


def _polygon_ring_to_coord_list(coords: Any, altitude: int) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    if coords is None:
        return out
    for lon_val, lat_val in list(coords)[:-1]:
        out.append(
            {
                "latitude": float(lat_val),
                "longitude": float(lon_val),
                "altitude": int(altitude),
            }
        )
    return _dedupe_coord_path(out, closed=True)


def _merge_area_segments_to_detail(details: List[Dict[str, Any]]) -> Dict[str, Any]:
    outer_polys: List[Polygon] = []
    hole_polys: List[Polygon] = []
    altitude = 0
    for detail in details:
        if not isinstance(detail, dict):
            continue
        area_list = detail.get("areaList")
        if isinstance(area_list, list) and area_list:
            for row in area_list:
                if not isinstance(row, dict):
                    continue
                poly = _coord_list_to_polygon(row.get("coordinateList"))
                if poly is None:
                    continue
                coord_list = _normalize_coord_list(row.get("coordinateList"))
                if coord_list:
                    altitude = int(coord_list[0].get("altitude", altitude) or altitude)
                if bool(row.get("isHole")):
                    hole_polys.append(poly)
                else:
                    outer_polys.append(poly)
            continue
        poly = _coord_list_to_polygon(detail.get("coordinateList"))
        if poly is not None:
            coord_list = _normalize_coord_list(detail.get("coordinateList"))
            if coord_list:
                altitude = int(coord_list[0].get("altitude", altitude) or altitude)
            outer_polys.append(poly)

    if not outer_polys:
        return {"coordinateList": [], "lineList": [], "areaList": []}

    geometry: BaseGeometry = unary_union(outer_polys)
    if hole_polys:
        try:
            geometry = geometry.difference(unary_union(hole_polys))
        except Exception:
            pass
    polygon = _collapse_area_geometry_to_single_polygon(geometry)
    if polygon is None or polygon.is_empty:
        return {"coordinateList": [], "lineList": [], "areaList": []}

    outer_coords = _polygon_ring_to_coord_list(polygon.exterior.coords, altitude)
    if len(outer_coords) < 3:
        return {"coordinateList": [], "lineList": [], "areaList": []}

    area_rows: List[Dict[str, Any]] = [
        {
            "isHole": False,
            "coordinateList": copy.deepcopy(outer_coords),
        }
    ]
    for interior in polygon.interiors:
        hole_coords = _polygon_ring_to_coord_list(interior.coords, altitude)
        if len(hole_coords) < 3:
            continue
        area_rows.append(
            {
                "isHole": True,
                "coordinateList": hole_coords,
            }
        )
    return {
        "coordinateList": [],
        "lineList": [],
        "areaList": area_rows,
    }


def _collapse_input_missions_for_replan(
    payload: Dict[str, Any],
    *,
    mission_whitelist: Set[int] | None = None,
) -> Dict[str, Any]:
    mission_list = payload.get("inputMissionList") if isinstance(payload, dict) else None
    if not isinstance(mission_list, list):
        return {"mutated": False, "groupCount": 0, "removedInputMissionIDs": [], "normalizedInputMissionIDs": []}

    whitelist = {int(value) for value in (mission_whitelist or set())}
    new_list: List[Dict[str, Any]] = []
    mutated = False
    normalized_ids: List[int] = []
    for mission in mission_list:
        if not isinstance(mission, dict):
            new_list.append(mission)
            continue

        input_id = _safe_int_value(mission.get("inputMissionID"))
        if bool(mission.get("isDone")) or (whitelist and (input_id is None or input_id not in whitelist)):
            new_list.append(mission)
            continue

        bucket = _mission_geometry_bucket(mission)
        detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
        if not bucket or not isinstance(detail, dict):
            new_list.append(mission)
            continue

        should_collapse = False
        if bucket == "line":
            seg_rows, _ = _extract_line_segments_from_detail(detail)
            source_width_m, source_coords = _resolve_source_line_metadata(detail, fallback_to_current=False)
            should_collapse = (
                len(seg_rows) > 1
                or (source_width_m is not None and source_width_m > 0.0)
                or len(source_coords) >= 2
            )
            merged_detail = _merge_line_segments_to_detail([detail]) if should_collapse else detail
        else:
            outer_area_count = 0
            area_list = detail.get("areaList")
            if isinstance(area_list, list) and area_list:
                for row in area_list:
                    if not isinstance(row, dict) or bool(row.get("isHole")):
                        continue
                    if _coord_list_to_polygon(row.get("coordinateList")) is not None:
                        outer_area_count += 1
            elif _coord_list_to_polygon(detail.get("coordinateList")) is not None:
                outer_area_count = 1
            should_collapse = outer_area_count > 1
            merged_detail = _merge_area_segments_to_detail([detail]) if should_collapse else detail

        if not should_collapse:
            new_list.append(mission)
            continue

        updated_mission = copy.deepcopy(mission)
        updated_mission["missionDetail"] = merged_detail
        updated_mission["isDone"] = not _has_nonempty_remaining_detail(merged_detail)
        if updated_mission != mission:
            mutated = True
            if input_id is not None:
                normalized_ids.append(int(input_id))
        new_list.append(updated_mission)

    if new_list != mission_list:
        payload["inputMissionList"] = new_list
        mutated = True

    return {
        "mutated": bool(mutated),
        "groupCount": len(sorted({int(value) for value in normalized_ids})),
        "removedInputMissionIDs": [],
        "normalizedInputMissionIDs": sorted({int(value) for value in normalized_ids}),
    }


def _should_apply_remaining_snapshot(
    *,
    ctx: Dict[str, Any],
    staged: Dict[str, Any],
    source_plan_id: int | None,
) -> tuple[bool, str]:
    if source_plan_id is None or source_plan_id <= 0:
        return False, "source plan unavailable"

    if is_input_refresh_context(ctx, staged):
        current_input_id = input_refresh_current_input_id(ctx, staged)
        if current_input_id is None:
            return False, "inputRefresh current input unavailable"
        return True, f"inputRefresh current mission {int(current_input_id)}"

    detail_candidates = []
    for container in (ctx, staged):
        detail = container.get("replan_detail") if isinstance(container, dict) else None
        if isinstance(detail, dict):
            detail_candidates.append(detail)

    for detail in detail_candidates:
        trigger = str(detail.get("trigger") or "").strip()
        trigger_type = str(detail.get("triggerType") or "").strip()
        if trigger == "0201":
            return False, f"fresh {trigger_type or '0201'} trigger"
        if trigger_type in _SNAPSHOT_SKIP_TRIGGER_TYPES:
            return False, f"fresh {trigger_type} trigger"

    reason_candidates = [
        str((container or {}).get("reason") or "").strip()
        for container in (ctx, staged)
        if isinstance(container, dict)
    ]
    if any(reason == "협업기저임무 재수행 요청" for reason in reason_candidates):
        return False, "collab reexecute refresh"

    return True, "source mission snapshot available"


def _remaining_snapshot_audit_context(
    *,
    ctx: Dict[str, Any],
    staged: Dict[str, Any],
) -> str:
    detail_candidates = []
    for container in (ctx, staged):
        detail = container.get("replan_detail") if isinstance(container, dict) else None
        if isinstance(detail, dict):
            detail_candidates.append(detail)

    for detail in detail_candidates:
        if not bool(detail.get("currentRemainingCollaborativeReplan")):
            continue
        trigger = str(detail.get("trigger") or "").strip()
        trigger_type = str(detail.get("triggerType") or "").strip()
        if trigger == "0201" and trigger_type == "collabReexecuteInputRefresh":
            return "mission_planning_gui_reexecute_first_snapshot_apply"
        return "mission_planning_gui_current_remaining_snapshot_apply"
    return "mission_planning_gui_apply_remaining_snapshot"


def _find_input_mission_entry(
    payload: Dict[str, Any],
    input_mission_id: int,
) -> Optional[Dict[str, Any]]:
    mission_list = payload.get("inputMissionList") if isinstance(payload, dict) else None
    if not isinstance(mission_list, list):
        return None
    for mission in mission_list:
        if not isinstance(mission, dict):
            continue
        if _safe_int_value(mission.get("inputMissionID")) == int(input_mission_id):
            return mission
    return None


def _mark_handover_terminal_missions_from_path(
    missions: List[Dict[str, Any]],
    input_plan_path: Path,
) -> int:
    """Attach regionType=2 direct-transit metadata; 0203 HO stays reference-only."""

    try:
        with Path(input_plan_path).open("r", encoding="utf-8-sig") as stream:
            input_plan = json.load(stream)
    except Exception:
        return 0
    return mark_handover_terminal_missions(missions, input_plan)


def _find_next_pending_input_mission_entry(
    payload: Dict[str, Any],
    current_input_id: int,
    *,
    mission_whitelist: Set[int] | None = None,
) -> Optional[Dict[str, Any]]:
    mission_list = payload.get("inputMissionList") if isinstance(payload, dict) else None
    if not isinstance(mission_list, list):
        return None
    whitelist = {int(value) for value in (mission_whitelist or set())}
    seen_current = False
    for mission in mission_list:
        if not isinstance(mission, dict):
            continue
        input_id = _safe_int_value(mission.get("inputMissionID"))
        if input_id is None or input_id <= 0:
            continue
        if whitelist and int(input_id) not in whitelist:
            continue
        if bool(mission.get("isDone")):
            continue
        if int(input_id) == int(current_input_id):
            seen_current = True
            continue
        if seen_current:
            return mission
    return None


def _centroid_coord_list(coords: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not coords:
        return None
    lat_vals = [float(item["latitude"]) for item in coords if isinstance(item, dict) and "latitude" in item]
    lon_vals = [float(item["longitude"]) for item in coords if isinstance(item, dict) and "longitude" in item]
    if not lat_vals or not lon_vals:
        return None
    out: Dict[str, Any] = {
        "latitude": sum(lat_vals) / float(len(lat_vals)),
        "longitude": sum(lon_vals) / float(len(lon_vals)),
    }
    alt_vals = [
        float(item.get("altitude", 0.0) or 0.0)
        for item in coords
        if isinstance(item, dict) and item.get("altitude") is not None
    ]
    if alt_vals:
        out["altitude"] = int(round(sum(alt_vals) / float(len(alt_vals))))
    return out


def _build_current_remaining_hybrid_request(
    *,
    ctx: Dict[str, Any],
    staged: Dict[str, Any],
    cmpk_data: Dict[str, Any],
    source_plan_id: int | None,
    mission_whitelist: Set[int] | None = None,
) -> Optional[CurrentRemainingHybridRequest]:
    if source_plan_id is None or source_plan_id <= 0:
        return None

    detail_candidates = []
    for container in (ctx, staged):
        detail = container.get("replan_detail") if isinstance(container, dict) else None
        if isinstance(detail, dict):
            detail_candidates.append(detail)

    detail = next(
        (
            item
            for item in detail_candidates
            if bool(item.get("currentRemainingCollaborativeReplan"))
            and isinstance(item.get("entryAircraftList"), list)
        ),
        None,
    )
    if not isinstance(detail, dict):
        return None
    trigger = str(detail.get("trigger") or "").strip()
    trigger_type = str(detail.get("triggerType") or "").strip()
    planner_mode = (
        "reexecute_first_mission"
        if trigger == "0201" and trigger_type == "collabReexecuteInputRefresh"
        else "current_remaining"
    )
    source_template_input_id = None
    if planner_mode == "reexecute_first_mission":
        source_template_input_id = _safe_int_value(
            detail.get("reexecuteSourceInputMissionID")
            or detail.get("sourceInputMissionID")
            or detail.get("originalInputMissionID")
        )

    current_input_id = _safe_int_value(detail.get("currentInputMissionID"))
    if current_input_id is None or current_input_id <= 0:
        return None
    whitelist = {int(value) for value in (mission_whitelist or set())}
    if whitelist and int(current_input_id) not in whitelist:
        return None

    current_mission = _find_input_mission_entry(cmpk_data, int(current_input_id))
    if not isinstance(current_mission, dict):
        return None
    if bool(current_mission.get("isDone")):
        return None
    if _mission_geometry_bucket(current_mission) not in {"line", "area"}:
        return None

    entry_coord_map: Dict[int, Dict[str, Any]] = {}
    heading_map: Dict[int, float] = {}
    for row in detail.get("entryAircraftList") or []:
        if not isinstance(row, dict):
            continue
        aircraft_id = _safe_int_value(row.get("aircraftID"))
        if aircraft_id is None or aircraft_id not in (4, 5, 6):
            continue
        coord = _normalize_coord_dict(row.get("coordinate"))
        if coord is None:
            continue
        entry_coord_map[int(aircraft_id)] = coord
        try:
            heading_val = row.get("headingDeg")
            if heading_val is None:
                heading_val = row.get("heading")
            if heading_val is not None:
                heading_map[int(aircraft_id)] = float(heading_val) % 360.0
        except Exception:
            continue
    if not entry_coord_map:
        return None
    entry_aircraft_context_map = build_line_entry_context_map_from_entry_rows(
        detail.get("entryAircraftList") or []
    )

    representative_entry = _normalize_coord_dict(detail.get("representativeEntryCoordinate"))
    if representative_entry is None:
        representative_entry = _centroid_coord_list(list(entry_coord_map.values()))

    try:
        turn_radius_scale = float(detail.get("turnRadiusScale") or 1.2)
    except Exception:
        turn_radius_scale = 1.2
    if turn_radius_scale <= 0.0:
        turn_radius_scale = 1.2

    apply_option_ordinals: Set[int] | None = None
    raw_apply_ordinals = (
        detail.get("currentRemainingApplyOptionOrdinals")
        or detail.get("currentRemainingApplyOptions")
        or detail.get("currentRemainingHybridOptionOrdinals")
    )
    if isinstance(raw_apply_ordinals, list):
        raw_rows = raw_apply_ordinals
    elif raw_apply_ordinals is None:
        raw_rows = []
    else:
        raw_rows = [raw_apply_ordinals]
    parsed_ordinals: Set[int] = set()
    for raw_value in raw_rows:
        try:
            ordinal = int(raw_value)
        except BaseException:
            continue
        if ordinal > 0:
            parsed_ordinals.add(int(ordinal))
    if parsed_ordinals:
        apply_option_ordinals = parsed_ordinals

    next_input_mission = _find_next_pending_input_mission_entry(
        cmpk_data,
        int(current_input_id),
        mission_whitelist=mission_whitelist,
    )
    planning_mode_ctx = mission_mode_context(mode=resolve_mission_planning_mode(cmpk_data))
    return CurrentRemainingHybridRequest(
        source_plan_id=int(source_plan_id),
        current_input_id=int(current_input_id),
        current_input_mission=copy.deepcopy(current_mission),
        next_input_mission=copy.deepcopy(next_input_mission) if isinstance(next_input_mission, dict) else None,
        entry_coord_map={int(aid): copy.deepcopy(coord) for aid, coord in entry_coord_map.items()},
        heading_map={int(aid): float(val) for aid, val in heading_map.items()},
        representative_entry=copy.deepcopy(representative_entry) if isinstance(representative_entry, dict) else None,
        turn_radius_scale=float(turn_radius_scale),
        apply_option_ordinals=apply_option_ordinals,
        planner_mode=planner_mode,
        source_template_input_id=source_template_input_id,
        entry_aircraft_context_map={
            int(aid): copy.deepcopy(row)
            for aid, row in entry_aircraft_context_map.items()
            if isinstance(row, dict)
        },
        planning_mode=dict(planning_mode_ctx),
    )


def _snapshot_apply_whitelist_for_current_remaining_hybrid(
    *,
    ctx: Dict[str, Any],
    staged: Dict[str, Any],
    cmpk_data: Dict[str, Any],
    mission_whitelist: Set[int] | None = None,
) -> Set[int]:
    whitelist = {int(value) for value in (mission_whitelist or set())}
    if not isinstance(cmpk_data, dict):
        return whitelist

    input_refresh_scope = input_refresh_snapshot_whitelist(
        ctx=ctx,
        staged=staged,
        mission_whitelist=mission_whitelist,
    )
    if input_refresh_scope is not None:
        return {int(value) for value in input_refresh_scope}

    detail_candidates = []
    for container in (ctx, staged):
        detail = container.get("replan_detail") if isinstance(container, dict) else None
        if isinstance(detail, dict):
            detail_candidates.append(detail)

    detail = next(
        (
            item
            for item in detail_candidates
            if bool(item.get("currentRemainingCollaborativeReplan"))
        ),
        None,
    )
    if not isinstance(detail, dict):
        return whitelist

    current_input_id = _safe_int_value(detail.get("currentInputMissionID"))
    if current_input_id is None or current_input_id <= 0:
        return whitelist

    mission_list = cmpk_data.get("inputMissionList")
    if not isinstance(mission_list, list):
        return whitelist

    ordered_ids: List[int] = []
    for mission in mission_list:
        if not isinstance(mission, dict):
            continue
        input_id = _safe_int_value(mission.get("inputMissionID"))
        if input_id is None or input_id <= 0:
            continue
        if whitelist and int(input_id) not in whitelist:
            continue
        ordered_ids.append(int(input_id))

    if int(current_input_id) not in ordered_ids:
        return whitelist

    current_idx = ordered_ids.index(int(current_input_id))
    scoped_whitelist = {int(value) for value in ordered_ids[: current_idx + 1]}
    return scoped_whitelist or whitelist


def _override_input_missions_with_remaining_snapshot(
    payload: Dict[str, Any],
    *,
    source_plan_id: int | None,
    mission_whitelist: Set[int] | None = None,
    audit_context: str = "mission_planning_gui_apply_remaining_snapshot",
) -> Dict[str, Any]:
    if not isinstance(payload, dict):
        return {"applied": 0, "marked_done": 0, "snapshotMissionCount": 0}
    if source_plan_id is None or source_plan_id <= 0:
        return {"applied": 0, "marked_done": 0, "snapshotMissionCount": 0}

    mission_list = payload.get("inputMissionList")
    if not isinstance(mission_list, list):
        return {"applied": 0, "marked_done": 0, "snapshotMissionCount": 0}

    whitelist = {int(value) for value in (mission_whitelist or set())}
    snapshot_map: Dict[int, Dict[str, Any]] = {}
    snapshot_map_plan_ids: Dict[int, int] = {}
    snapshot_map_exact: Dict[int, bool] = {}
    snapshot_map_audited_inputs: Set[int] = set()
    snapshot_plan_ids: Set[int] = set()
    snapshot = mission_area_replan_store.load_snapshot(int(source_plan_id))
    if isinstance(snapshot, dict):
        snapshot_plan_id = _safe_int_value(snapshot.get("missionPlanID"))
        if snapshot_plan_id is not None and snapshot_plan_id > 0:
            snapshot_plan_ids.add(int(snapshot_plan_id))
        for item in snapshot.get("missions") or []:
            if not isinstance(item, dict):
                continue
            input_id = _safe_int_value(item.get("inputMissionID"))
            if input_id is None or input_id <= 0:
                continue
            snapshot_map[int(input_id)] = item
            snapshot_map_exact[int(input_id)] = True
            if snapshot_plan_id is not None and snapshot_plan_id > 0:
                snapshot_map_plan_ids[int(input_id)] = int(snapshot_plan_id)

    applied = 0
    marked_done = 0
    context_text = str(audit_context or "mission_planning_gui_apply_remaining_snapshot")
    for mission in mission_list:
        if not isinstance(mission, dict):
            continue
        input_id = _safe_int_value(mission.get("inputMissionID"))
        if input_id is None or input_id <= 0:
            continue
        if whitelist and int(input_id) not in whitelist:
            continue
        snapshot_entry = snapshot_map.get(int(input_id))
        if not isinstance(snapshot_entry, dict):
            snapshot_info = mission_area_replan_store.load_snapshot_entry(
                int(source_plan_id),
                int(input_id),
                allow_latest=True,
                audit_context=context_text,
            )
            if isinstance(snapshot_info, dict):
                fallback_entry = snapshot_info.get("entry")
                if isinstance(fallback_entry, dict):
                    snapshot_entry = fallback_entry
                    snapshot_map[int(input_id)] = fallback_entry
                    fallback_plan_id = _safe_int_value(snapshot_info.get("snapshotMissionPlanID"))
                    if fallback_plan_id is not None and fallback_plan_id > 0:
                        snapshot_plan_ids.add(int(fallback_plan_id))
                        snapshot_map_plan_ids[int(input_id)] = int(fallback_plan_id)
                    snapshot_map_exact[int(input_id)] = bool(snapshot_info.get("exact"))
                    snapshot_map_audited_inputs.add(int(input_id))
            if not isinstance(snapshot_entry, dict):
                continue
        elif int(input_id) not in snapshot_map_audited_inputs:
            mission_area_replan_store.audit_snapshot_entry_access(
                snapshot_entry,
                requested_mission_plan_id=int(source_plan_id),
                snapshot_mission_plan_id=snapshot_map_plan_ids.get(int(input_id), int(source_plan_id)),
                audit_context=context_text,
                event="snapshot_entry_exact",
            )
            snapshot_map_audited_inputs.add(int(input_id))

        reject_reason = mission_area_replan_store.snapshot_entry_replan_reject_reason(
            snapshot_entry,
            exact=snapshot_map_exact.get(int(input_id)),
        )
        if reject_reason == "area_snapshot_latest_fallback_not_allowed":
            mission_area_replan_store.audit_snapshot_entry_rejected(
                snapshot_entry,
                requested_mission_plan_id=int(source_plan_id),
                snapshot_mission_plan_id=snapshot_map_plan_ids.get(int(input_id), int(source_plan_id)),
                audit_context=context_text,
                reason=str(reject_reason),
            )
            mission["isDone"] = True
            marked_done += 1
            continue

        remaining_detail = mission_area_replan_store.coverage_replan_pending_remaining_detail(
            snapshot_entry
        )
        depth_contract = mission_area_replan_store.coverage_depth_replan_contract(
            snapshot_entry
        )
        depth_unresolved = bool(
            depth_contract
            and not depth_contract.get("coverageDepthSatisfied")
            and int(depth_contract.get("coverageDepthUnresolvedGeometryCount") or 0) > 0
        )
        is_done = (
            bool(depth_contract.get("coverageDepthSatisfied"))
            if depth_contract
            else bool(snapshot_entry.get("isDone"))
            or not _has_nonempty_remaining_detail(remaining_detail)
        )
        if is_done:
            mission["isDone"] = True
            marked_done += 1
            continue
        if depth_unresolved and not _has_nonempty_remaining_detail(remaining_detail):
            mission_area_replan_store.audit_snapshot_entry_rejected(
                snapshot_entry,
                requested_mission_plan_id=int(source_plan_id),
                snapshot_mission_plan_id=snapshot_map_plan_ids.get(int(input_id), int(source_plan_id)),
                audit_context=context_text,
                reason="area_coverage_depth_geometry_unresolved",
            )
            continue

        if reject_reason:
            mission_area_replan_store.audit_snapshot_entry_rejected(
                snapshot_entry,
                requested_mission_plan_id=int(source_plan_id),
                snapshot_mission_plan_id=snapshot_map_plan_ids.get(int(input_id), int(source_plan_id)),
                audit_context=context_text,
                reason=str(reject_reason),
            )
            mission["isDone"] = True
            marked_done += 1
            continue

        mission_detail = mission.get("missionDetail")
        if not isinstance(mission_detail, dict):
            mission_detail = {}
        else:
            mission_detail = dict(mission_detail)
        source_line_width_m, source_coordinate_list = _resolve_source_line_metadata(
            mission_detail,
            fallback_to_current=True,
        )
        if source_line_width_m is None or source_line_width_m <= 0.0:
            try:
                snapshot_source_width_m = float(snapshot_entry.get("sourceLineWidthM", 0.0) or 0.0)
            except Exception:
                snapshot_source_width_m = 0.0
            if snapshot_source_width_m > 0.0:
                source_line_width_m = float(snapshot_source_width_m)
        if len(source_coordinate_list) < 2:
            source_coordinate_list = _dedupe_coord_path(
                _normalize_coord_list(snapshot_entry.get("sourceCoordinateList"), min_len=2),
                closed=False,
            )

        coordinate_list = remaining_detail.get("coordinateList") if isinstance(remaining_detail, dict) else []
        line_list = remaining_detail.get("lineList") if isinstance(remaining_detail, dict) else []
        area_list = remaining_detail.get("areaList") if isinstance(remaining_detail, dict) else []
        area_segment_list = (
            remaining_detail.get("areaSegmentList")
            if isinstance(remaining_detail, dict) and isinstance(remaining_detail.get("areaSegmentList"), list)
            else []
        )
        mission_type = str(snapshot_entry.get("missionType") or "").strip().lower()

        if mission_type == "line":
            mission_detail["lineList"] = copy.deepcopy(line_list if isinstance(line_list, list) else [])
            mission_detail["coordinateList"] = copy.deepcopy(
                coordinate_list if isinstance(coordinate_list, list) else []
            )
            mission_detail.pop("areaList", None)
            if source_line_width_m is not None and source_line_width_m > 0.0:
                mission_detail["sourceLineWidthM"] = float(source_line_width_m)
            if len(source_coordinate_list) >= 2:
                mission_detail["sourceCoordinateList"] = copy.deepcopy(source_coordinate_list)
        elif depth_contract:
            assignment_detail = mission_area_replan_store.area_assignment_detail(
                snapshot_entry,
                fallback=mission_detail,
            )
            if assignment_detail is not None:
                mission_area_replan_store.apply_area_assignment_geometry(
                    mission_detail,
                    assignment_detail,
                )
            mission_detail["areaCoverageWorkloadDetail"] = copy.deepcopy(
                remaining_detail
                if isinstance(remaining_detail, dict)
                else {"coordinateList": [], "lineList": [], "areaList": []}
            )
            mission_detail.pop("sourceLineWidthM", None)
            mission_detail.pop("sourceCoordinateList", None)
        else:
            mission_detail["areaList"] = copy.deepcopy(area_list if isinstance(area_list, list) else [])
            if area_segment_list:
                mission_detail["areaSegmentList"] = copy.deepcopy(area_segment_list)
                mission_detail["areaSegmentPolicy"] = str(
                    remaining_detail.get("areaSegmentPolicy") or "planned_sweep_row_remaining"
                )
            else:
                mission_detail.pop("areaSegmentList", None)
                mission_detail.pop("areaSegmentPolicy", None)
            if (isinstance(area_list, list) and area_list) or area_segment_list:
                mission_detail["coordinateList"] = []
            else:
                mission_detail["coordinateList"] = copy.deepcopy(
                    coordinate_list if isinstance(coordinate_list, list) else []
                )
            mission_detail.pop("lineList", None)
            mission_detail.pop("sourceLineWidthM", None)
            mission_detail.pop("sourceCoordinateList", None)

        mission["missionDetail"] = mission_detail
        contracts = mission_area_replan_store.apply_area_coverage_replan_contracts(
            mission_detail,
            snapshot_entry,
        )
        if contracts.get("passes") or contracts.get("depth"):
            mission_area_replan_store.apply_area_coverage_replan_contracts(
                mission,
                snapshot_entry,
            )
        mission["isDone"] = False
        applied += 1

    return {
        "applied": int(applied),
        "marked_done": int(marked_done),
        "snapshotMissionCount": len(snapshot_map),
        "snapshotPlanID": int(source_plan_id),
        "snapshotPlanIDs": sorted(snapshot_plan_ids),
    }


def _is_quality_speed_reason_text(value: object | None) -> bool:
    text = str(value or "")
    return any(keyword in text for keyword in _QUALITY_SPEED_REASON_KEYWORDS)


def _is_quality_speed_trigger_type(value: object | None) -> bool:
    return str(value or "").strip() == "qualityMonitorSep"


def _plan_meta_has_quality_speed(meta_map: object | None) -> bool:
    if not isinstance(meta_map, dict):
        return False
    for meta in meta_map.values():
        if not isinstance(meta, dict):
            continue
        if _is_quality_speed_trigger_type(meta.get("triggerType")):
            return True
        detail = meta.get("replanDetail")
        if isinstance(detail, dict) and _is_quality_speed_trigger_type(detail.get("triggerType")):
            return True
    return False


def _count_replan_options(ctx: object | None, payload: object | None = None) -> int:
    return count_replan_options(ctx, payload)


def _classify_replan_timing_context(
    ctx: object | None,
    payload: object | None = None,
) -> Dict[str, Any]:
    return classify_replan_timing_context(
        ctx,
        payload,
        is_path_deviation_reason_text=_is_path_deviation_reason_text,
        is_quality_speed_reason_text=_is_quality_speed_reason_text,
        is_imaging_schedule_reason_text=_is_imaging_schedule_reason_text,
    )


def _post_0301_delivery_delays(*, plan_count: int, force_direct: bool) -> tuple[int, int, str]:
    return post_0301_delivery_delays(plan_count=plan_count, force_direct=force_direct)


def _is_next_collab_reason_text(value: object | None) -> bool:
    return runtime_is_next_collab_reason_text(value)


def _sort_plan_delivery_entries(
    plan_ids: object | None,
    option_names: object | None,
) -> tuple[list[int], list[str]]:
    return sort_plan_delivery_entries(plan_ids, option_names)


# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    # 백그라운드 → UI 스레드용 신호
    ctrl_payload   = pyqtSignal(dict)   # 제어
    log_sig        = pyqtSignal(str)    # 로그
    pipeline_log_sig = pyqtSignal(dict) # pipeline log fan-out
    planning_metric_sig = pyqtSignal(dict)
    start_push_seq = pyqtSignal()       # 0301/0305/0901 순차 푸시 트리거
    resume_deferred_replan_sig = pyqtSignal()
    visual_refresh = pyqtSignal()
    id_tab_update_sig = pyqtSignal(object)
    tab_mark_sent_sig = pyqtSignal(str, object)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._emit_lifecycle("window_init_start", component="gui", outcome="ok")
        self.setWindowTitle('임무계획(MMR)')
        self.resize(1100, 700)

        # 파워/상태
        self._power_on = True
        self._self_check_sent = False
        self._last_ctrl_ts = {}     # 디듀프
        self._rx_counts = {}

        # 파이프라인 컨텍스트
        self._initplan_running = False
        self._last_mission_plan_id = None
        self._last_mission_plan_ids = []
        self._staged_plan_context: dict = {}
        self._active_plan_context: dict = {}
        self._deferred_replan_requests: list[dict[str, Any]] = []
        self._pending_plan_push: dict | None = None
        self._scheduled_0301_plan_ids: list[int] = []
        self._replan_delay_timer: QTimer | None = None
        self._post_0301_delivery: dict | None = None
        self._post_0301_timer = QTimer(self)
        self._post_0301_timer.setSingleShot(True)
        self._post_0301_timer.timeout.connect(
            lambda: self._try_flush_post_0301_delivery(
                trigger="timeout",
                force=True,
            )
        )
        self._session_scope = self._create_empty_scope()
        self._plan_status = "임무계획 전"
        self._option_id_counter = 0
        self._bus_ready = False
        self._attack_delivery_buffer: list[dict] = []
        self._hb_0102_enabled = False
        self._hb_0102_interval_sec = 0.2  # 5Hz
        self._hb_0102_stop = threading.Event()
        self._hb_0102_thread: Optional[threading.Thread] = None
        self._hb_0102_timer = QTimer(self)
        self._hb_0102_timer.setInterval(max(1, int(self._hb_0102_interval_sec * 1000)))
        self._hb_0102_timer.timeout.connect(self._send_0102_heartbeat_tick)
        self._hb_0102_last_warn = 0.0
        self._hb_0102_last_tick_monotonic = 0.0
        self._hb_0102_last_success_monotonic = 0.0
        self._hb_0102_body_template: dict[str, Any] = {"source": "MMR", "status": 1}
        self._planning_timer_started_at: Optional[float] = None
        self._planning_timer_reason: str = "-"
        self._last_planning_elapsed_ms: Optional[float] = None
        self._last_planning_status: str = "idle"
        self._planner_runtime_lock = threading.RLock()
        self._planner_runtime_cache: Optional[Dict[str, Any]] = None
        self._planner_runtime_warmup_running = False
        self._planner_runtime_warmup_pending: Optional[str] = None
        self._planner_runtime_ready_logged = False
        self._planner_runtime_ready_event = threading.Event()
        self._planner_runtime_source_signature_seen: Optional[tuple] = None
        self._terrain_runtime_warmup_lock = threading.Lock()
        self._terrain_runtime_warmup_running = False
        self._replan_terrain_warmup_lock = threading.Lock()
        self._replan_terrain_warmup_running = False
        self._0303_process_warmup_lock = threading.Lock()
        self._0303_process_warmup_running = False

        reset_latest_inputs()
        self._last_logged_input_ids = {"0201": None, "0203": None}
        self._review_0204_sent_package_ids: set[int] = set()
        self._input_listener_refs: list[tuple[str, callable]] = []
        self._install_input_listeners()

        # ── 중앙 탭(AssignmentPlanningTab)
        tabs = QTabWidget()
        polish_tabs(tabs)
        self._tabs = tabs
        self._tab = AssignmentPlanningTab(messenger=NodeMessenger)
        self._tab.set_replan_callback(self._handle_replan_received)

        self._install_power_gate_hooks()       # Power OFF 가드
        self._install_0301_override()          # 0301 전송 커스텀
        tabs.addTab(self._tab, "임무 할당·계획수립 CSC")
        self._algo_tab = MissionAlgoConfigTab(
            runtime_settings_path(),
            on_apply=self._on_algo_settings_applied,
        )
        tabs.addTab(self._algo_tab, "알고리즘 설정")
        self._id_tab = MissionIdRelationshipTab()
        tabs.addTab(self._id_tab, "임무 관계도")
        self._id_tab_index = tabs.indexOf(self._id_tab)

        # LAH Hex RL 경로계획 탭 (별도 윈도우 실행 버튼)
        try:
            _lah_rl_tab = QWidget()
            _lah_rl_layout = QVBoxLayout(_lah_rl_tab)
            _lah_rl_layout.setContentsMargins(20, 20, 20, 20)
            _lah_rl_desc = QLabel(
                "LAH 경로계획 (Hex Grid)\n\n"
                "육각형 맵 기반으로 유인기 경로를 계획합니다.\n"
                "기존 A* 경로계획과 RL(PPO) 경로계획을 전환할 수 있습니다.\n\n"
                "아래 버튼을 눌러 별도 창을 엽니다."
            )
            _lah_rl_desc.setWordWrap(True)
            _lah_rl_layout.addWidget(_lah_rl_desc)
            _lah_rl_btn = QPushButton("LAH Hex 경로계획 열기")
            _lah_rl_btn.setFixedHeight(44)
            _lah_rl_btn.setStyleSheet("font-size:14px; font-weight:bold;")
            _lah_rl_btn.clicked.connect(self._open_lah_rl_planner)
            _lah_rl_layout.addWidget(_lah_rl_btn)
            _lah_rl_layout.addStretch(1)
            tabs.addTab(_lah_rl_tab, "LAH 경로계획 (Hex)")
        except Exception:
            pass
        self._log_tab = None
        self._visual_tab = None
        self._pipeline_logger = PipelineLogManager(
            emit_callback=self.pipeline_log_sig.emit,
            log_tab_provider=lambda: getattr(self, "_log_tab", None),
            sanitize_reason=_sanitize_reason,
        )
        self._mission_plan_logger = MissionPlanFileLogger()
        self._active_plan_log_run = None
        self._deferred_id_tab_update_payload: Optional[dict] = None
        self._log_file_path: Optional[Path] = None
        self._log_file_db_root: Optional[str] = None
        self._db_root: Optional[str] = None
        self._init_gui_log_file_sink()
        self.id_tab_update_sig.connect(self._apply_id_tab_update)
        self.tab_mark_sent_sig.connect(self._mark_tab_sent)
        self._on_algo_settings_applied()
        self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
        QTimer.singleShot(0, lambda: self._schedule_planner_warmup("startup"))

        # ── 상단 모드 슬라이더
        top = QWidget()
        top.setObjectName("TopBar")
        top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(4, 2, 4, 2)
        top_layout.setSpacing(12)
        self._latest_input_label = QLabel("0201/0203 \uc218\uc2e0 \ud604\ud669")
        self._latest_input_label.setObjectName("InfoBadge")
        top_layout.addWidget(self._latest_input_label)
        self._planning_elapsed_label = QLabel("최근 계획 시간: -")
        self._planning_elapsed_label.setObjectName("InfoBadge")
        top_layout.addWidget(self._planning_elapsed_label)
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
        top_layout.addWidget(lbl); top_layout.addWidget(slider_wrap); top_layout.addWidget(self.mode_now)
        self._refresh_input_banner()

        center = QWidget()
        v = QVBoxLayout(center)
        v.setContentsMargins(12, 12, 12, 12)
        v.setSpacing(10)
        v.addWidget(top); v.addWidget(tabs)
        self.setCentralWidget(center)

        # 초기 기동 시 즉시 초기화 모드로 전환
        self._set_mode_slider_by_text("초기화 모드")
        self._apply_power_state()

        # 신호 연결
        self.ctrl_payload.connect(self._handle_ctrl_payload)
        self.log_sig.connect(self._append_log_line)
        self.pipeline_log_sig.connect(self._pipeline_logger.handle_event)
        self.planning_metric_sig.connect(self._apply_planning_metric)
        self.start_push_seq.connect(self._start_push_sequence)
        self.resume_deferred_replan_sig.connect(self._resume_deferred_replan_request)
        if self._log_file_path:
            self._append_log_line(f"[LOG] Mission planning log started: {self._log_file_path}")
        self._init_db_root_sync()

        self._ctrl_thread = None
        try:
            port = env_ctrl_port(45981)
            self._ctrl_thread = start_ctrl_listener(port, lambda payload: self.ctrl_payload.emit(payload))
            self._append_log_line(f"[CTRL] listener started @ 127.0.0.1:{port}")
            self._emit_lifecycle(
                "listener_start",
                component="ctrl_listener",
                outcome="ok",
                extra={"host": "127.0.0.1", "port": int(port)},
            )
        except Exception as exc:
            self._append_log_line(f"[CTRL] listener start failed: {exc}")
            self._emit_lifecycle(
                "listener_fail",
                component="ctrl_listener",
                outcome="failure",
                reason=str(exc),
            )

        # nFusion RX 초기화 + 테스트 단축키
        self._rx_setup_thread = threading.Thread(
            target=self._rx_setup,
            name="MMR-RX-Setup",
            daemon=True,
        )
        self._rx_setup_thread.start()
        self._install_test_shortcuts()

        # GUI 표시 후 상태 OK(=1) 한 번 송신.
        QTimer.singleShot(800, self._send_startup_0102_once)

        # run.py 등의 self_check ON 신호 수신 시에도 내부에서만 0102=1 송신
        # ★★★ 0101 수신 → 모드 반영 리스너 + 폴백 폴링 설치
        self._install_0101_mode_listener()
        self._start_0101_rx_poller()
        self._emit_lifecycle("window_init_done", component="gui", outcome="ok")

    def _refresh_visual_tab(self) -> None:
        tab = getattr(self, "_visual_tab", None)
        if tab is None:
            return
        try:
            tab.refresh()
        except Exception:
            pass

    @staticmethod
    def _format_planning_elapsed(elapsed_ms: Optional[float]) -> str:
        if elapsed_ms is None:
            return "-"
        try:
            seconds = float(elapsed_ms) / 1000.0
        except Exception:
            return "-"
        return f"{seconds:.2f}초"

    def _apply_planning_metric(self, payload: Dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            return
        state = str(payload.get("state") or "idle")
        reason = _sanitize_reason(payload.get("reason"), self._planning_timer_reason or "-")
        elapsed_ms_raw = payload.get("elapsed_ms")
        try:
            elapsed_ms = float(elapsed_ms_raw) if elapsed_ms_raw is not None else None
        except Exception:
            elapsed_ms = None

        if state == "running":
            self._planning_timer_reason = reason
            self._planning_timer_started_at = time.perf_counter()
            self._last_planning_status = "running"
            self._planning_elapsed_label.setText(f"최근 계획 시간: 측정 중 ({reason})")
            return

        if elapsed_ms is not None:
            self._last_planning_elapsed_ms = elapsed_ms
        self._planning_timer_started_at = None
        self._planning_timer_reason = reason
        self._last_planning_status = state

        elapsed_text = self._format_planning_elapsed(self._last_planning_elapsed_ms)
        if state == "success":
            self._planning_elapsed_label.setText(f"최근 계획 시간: {elapsed_text} ({reason})")
        elif state == "failed":
            self._planning_elapsed_label.setText(f"최근 계획 시간: 실패 {elapsed_text} ({reason})")
        else:
            self._planning_elapsed_label.setText(f"최근 계획 시간: {elapsed_text}")

    def _mark_planning_metric_start(self, reason: str) -> None:
        self.planning_metric_sig.emit({"state": "running", "reason": reason})

    def _mark_planning_metric_finish(self, reason: str, *, success: bool) -> Optional[float]:
        start_ts = self._planning_timer_started_at
        elapsed_ms = None
        if start_ts is not None:
            elapsed_ms = max(0.0, (time.perf_counter() - start_ts) * 1000.0)
        self.planning_metric_sig.emit(
            {
                "state": "success" if success else "failed",
                "reason": reason,
                "elapsed_ms": elapsed_ms,
            }
        )
        return elapsed_ms

    def _emit_lifecycle(
        self,
        lifecycle: str,
        *,
        component: str = "mission_planning_gui",
        outcome: str = "ok",
        reason: Optional[str] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        try:
            emit_process_lifecycle_event(
                "mission_planning",
                lifecycle,
                component=component,
                outcome=outcome,
                reason=reason,
                extra=extra,
            )
        except Exception:
            pass

    def _start_replan_timing(
        self,
        ctx: Dict[str, Any],
        payload: Dict[str, Any] | None = None,
        *,
        received_perf: Optional[float] = None,
        received_wall_ms: Optional[int] = None,
    ) -> None:
        if not isinstance(ctx, dict):
            return
        wall_ms = int(received_wall_ms) if received_wall_ms is not None else int(time.time() * 1000)
        base_perf = float(received_perf) if received_perf is not None else time.perf_counter()
        source_plan_ids = ctx.get("plan_ids")
        if not isinstance(source_plan_ids, (list, tuple)):
            source_plan_ids = []
        timing: Dict[str, Any] = {
            "base_perf": base_perf,
            "base_wall_ms": wall_ms,
            "replanTimingId": (
                f"replan-{wall_ms}-{time.perf_counter_ns()}-{threading.get_ident()}"
            ),
            # ctx["plan_ids"] is replaced with generated plan IDs later.  Keep
            # the IDs received in this 0902 so concurrent/deferred requests can
            # always be correlated with their own measurement.
            "source_plan_ids": list(source_plan_ids),
        }
        if isinstance(payload, dict):
            payload_ts = payload.get("timestamp")
            request_time = payload.get("replanRequestTime")
            if isinstance(request_time, dict):
                payload_ts = request_time.get("replanRequestTimestamp", payload_ts)
            try:
                timing["payload_timestamp_ms"] = int(payload_ts)
            except Exception:
                pass
        timing.update(_classify_replan_timing_context(ctx, payload))
        ctx["_replan_timing"] = timing
        self._record_replan_timing_snapshot(
            "0902_received",
            event_perf=base_perf,
            wall_ms=wall_ms,
            ctx=ctx,
            extra=_classify_replan_timing_context(ctx, payload),
        )

    def _record_replan_timing_event(
        self,
        event_name: str,
        *,
        ctx: Optional[Dict[str, Any]] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        try:
            event = str(event_name or "").strip()
            if not event:
                return
            target_ctx = ctx if isinstance(ctx, dict) else getattr(self, "_active_plan_context", None)
            if not isinstance(target_ctx, dict):
                return
            timing = target_ctx.get("_replan_timing")
            if not isinstance(timing, dict):
                return
            now_perf = time.perf_counter()
            now_wall_ms = int(time.time() * 1000)
            try:
                base_perf = float(timing.get("base_perf"))
            except Exception:
                return
            elapsed_ms = max(0.0, (now_perf - base_perf) * 1000.0)
            timing[f"{event}_ms"] = round(elapsed_ms, 3)
            timing[f"{event}_wall_ms"] = now_wall_ms
            detail = dict(extra or {})
            transaction_id = (
                detail.get("replanTransactionId")
                or target_ctx.get("replanTransactionId")
                or target_ctx.get("replan_transaction_id")
                or timing.get("replanTransactionId")
            )
            if transaction_id:
                transaction_id = str(transaction_id)
                detail.setdefault("replanTransactionId", transaction_id)
                timing["replanTransactionId"] = transaction_id
            for key, value in detail.items():
                timing[f"{event}_{key}"] = value
            variant_value = detail.get("variant")
            if variant_value is not None:
                try:
                    variant_key = str(int(variant_value))
                except Exception:
                    variant_key = str(variant_value)
                phase_key = event
                if phase_key.startswith("variant_"):
                    phase_key = phase_key[len("variant_") :]
                variants = timing.setdefault("variants", {})
                if isinstance(variants, dict):
                    variant_entry = variants.setdefault(variant_key, {})
                    if isinstance(variant_entry, dict):
                        variant_entry[phase_key] = {
                            "elapsed_ms": round(elapsed_ms, 3),
                            "wall_ms": now_wall_ms,
                            **{k: v for k, v in detail.items() if k != "variant"},
                        }
                timing[f"variant_{variant_key}_{phase_key}_ms"] = round(elapsed_ms, 3)
            detail_text = ""
            if detail:
                detail_text = " " + " ".join(f"{key}={value}" for key, value in detail.items())
            self.log_sig.emit(
                f"[REPLAN][TIME] {event}_ms={elapsed_ms:.1f} wall_ms={now_wall_ms}{detail_text}"
            )
            replan_detail = target_ctx.get("replan_detail")
            replan_detail = replan_detail if isinstance(replan_detail, dict) else {}
            try:
                mission_plan_id = int((target_ctx.get("plan_ids") or [None])[0])
            except Exception:
                mission_plan_id = None
            emit_replan_checkpoint(
                name=event,
                replan_transaction_id=transaction_id,
                trigger=str(timing.get("trigger") or detail.get("trigger") or ""),
                trigger_type=str(replan_detail.get("triggerType") or detail.get("triggerType") or ""),
                pipeline=str(detail.get("pipeline") or timing.get("pipeline") or ""),
                mission_plan_id=mission_plan_id,
                elapsed_ms=round(elapsed_ms, 3),
                extra=detail,
            )
        except Exception:
            return

    def _record_replan_timing_snapshot(
        self,
        event_name: str,
        *,
        event_perf: float,
        wall_ms: Optional[int] = None,
        ctx: Optional[Dict[str, Any]] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        try:
            event = str(event_name or "").strip()
            if not event:
                return
            target_ctx = ctx if isinstance(ctx, dict) else getattr(self, "_active_plan_context", None)
            if not isinstance(target_ctx, dict):
                return
            timing = target_ctx.get("_replan_timing")
            if not isinstance(timing, dict):
                return
            try:
                base_perf = float(timing.get("base_perf"))
            except Exception:
                return
            now_wall_ms = int(wall_ms) if wall_ms is not None else int(time.time() * 1000)
            elapsed_ms = max(0.0, (float(event_perf) - base_perf) * 1000.0)
            timing[f"{event}_ms"] = round(elapsed_ms, 3)
            timing[f"{event}_wall_ms"] = now_wall_ms
            detail = dict(extra or {})
            for key, value in detail.items():
                timing[f"{event}_{key}"] = value
            variant_value = detail.get("variant")
            if variant_value is not None:
                try:
                    variant_key = str(int(variant_value))
                except Exception:
                    variant_key = str(variant_value)
                phase_key = event
                if phase_key.startswith("variant_"):
                    phase_key = phase_key[len("variant_") :]
                variants = timing.setdefault("variants", {})
                if isinstance(variants, dict):
                    variant_entry = variants.setdefault(variant_key, {})
                    if isinstance(variant_entry, dict):
                        variant_entry[phase_key] = {
                            "elapsed_ms": round(elapsed_ms, 3),
                            "wall_ms": now_wall_ms,
                            **{k: v for k, v in detail.items() if k != "variant"},
                        }
                timing[f"variant_{variant_key}_{phase_key}_ms"] = round(elapsed_ms, 3)
            detail_text = ""
            if detail:
                detail_text = " " + " ".join(f"{key}={value}" for key, value in detail.items())
            self.log_sig.emit(
                f"[REPLAN][TIME] {event}_ms={elapsed_ms:.1f} wall_ms={now_wall_ms}{detail_text}"
            )
        except Exception:
            return

    def _persist_completed_replan_timing(
        self,
        *,
        reason: str,
        planning_success: bool,
        planning_elapsed_ms: Optional[float],
        ctx: Optional[Dict[str, Any]] = None,
    ) -> Optional[Dict[str, Any]]:
        """Persist one 0902-to-0305 timing record and its cumulative average."""

        target_ctx = ctx if isinstance(ctx, dict) else getattr(self, "_active_plan_context", None)
        if not isinstance(target_ctx, dict):
            return None
        timing = target_ctx.get("_replan_timing")
        if (
            not isinstance(timing, dict)
            or timing.get("history_recorded")
            or timing.get("history_recording")
        ):
            return None

        try:
            elapsed_ms = float(timing.get("0305_status_2_ms"))
        except (TypeError, ValueError):
            # Completion history must represent a successfully sent status=2
            # 0305, never the time at which this helper happened to run.
            return None

        try:
            running_sent_ms = float(timing.get("0305_status_1_ms"))
        except (TypeError, ValueError):
            running_sent_ms = None
        queue_elapsed_ms = max(0.0, running_sent_ms) if running_sent_ms is not None else None
        request_planning_elapsed_ms = (
            max(0.0, elapsed_ms - running_sent_ms)
            if running_sent_ms is not None
            else None
        )

        replan_detail = target_ctx.get("replan_detail")
        replan_detail = replan_detail if isinstance(replan_detail, dict) else {}
        source_plan_ids = timing.get("source_plan_ids")
        if not isinstance(source_plan_ids, (list, tuple)):
            source_plan_ids = []
        result_plan_ids = timing.get("delivered_plan_ids")
        if not isinstance(result_plan_ids, (list, tuple)):
            # Only IDs confirmed by the successful 0301 delivery path belong
            # in this request's result mapping.  A no-op/failure context can
            # still contain source or previously active plan IDs.
            result_plan_ids = []

        timing["history_recording"] = True
        try:
            result = _record_replan_timing_history(
                elapsed_ms=elapsed_ms,
                # Use request-local 0305 checkpoints.  The GUI planning timer
                # is retained only for display and may be overwritten by a
                # queued request before this completion is persisted.
                planning_elapsed_ms=request_planning_elapsed_ms,
                queue_elapsed_ms=queue_elapsed_ms,
                status="success" if planning_success else "failed",
                reason=reason,
                trigger=str(timing.get("trigger") or "unknown"),
                trigger_type=str(replan_detail.get("triggerType") or ""),
                replan_level=timing.get("replanLevel", target_ctx.get("replan_level")),
                option_count=timing.get("optionCount"),
                timing_id=timing.get("replanTimingId"),
                transaction_id=(
                    timing.get("replanTransactionId")
                    or target_ctx.get("replanTransactionId")
                    or target_ctx.get("replan_transaction_id")
                ),
                source_plan_ids=source_plan_ids,
                result_plan_ids=result_plan_ids,
                started_at_ms=timing.get("base_wall_ms"),
                running_at_ms=timing.get("0305_status_1_wall_ms"),
                completed_at_ms=timing.get("0305_status_2_wall_ms"),
                metadata={
                    "forceDirect": bool(timing.get("forceDirect")),
                    "noOp": "재계획 불필요" in str(reason or ""),
                    "payloadTimestampMs": timing.get("payload_timestamp_ms"),
                    "uiPlanningElapsedMs": planning_elapsed_ms,
                },
            )
        except Exception:
            timing.pop("history_recording", None)
            raise
        timing["history_recorded"] = True
        timing.pop("history_recording", None)

        summary = result.get("summary") if isinstance(result, dict) else {}
        summary = summary if isinstance(summary, dict) else {}
        count = int(summary.get("completedCount") or 0)
        average_ms = float(summary.get("averageElapsedMs") or 0.0)
        planning_average_ms = float(summary.get("averagePlanningElapsedMs") or 0.0)
        self.log_sig.emit(
            "[REPLAN][HISTORY] "
            f"count={count} elapsed={elapsed_ms / 1000.0:.2f}s "
            f"average={average_ms / 1000.0:.2f}s "
            f"planning_average={planning_average_ms / 1000.0:.2f}s "
            f"status={'success' if planning_success else 'failed'} "
            f"file={result.get('path', '-') if isinstance(result, dict) else '-'}"
        )
        return result

    def _build_planner_runtime(self) -> Dict[str, Any]:
        runtime_timing: Dict[str, float] = {}
        step_started = time.perf_counter()
        source_signature = _planner_runtime_source_signature()
        runtime_timing["source_signature_ms"] = (time.perf_counter() - step_started) * 1000.0
        step_started = time.perf_counter()
        with self._planner_runtime_lock:
            previous_signature = self._planner_runtime_source_signature_seen
        runtime_timing["lock_read_ms"] = (time.perf_counter() - step_started) * 1000.0
        force_reload = previous_signature is not None and previous_signature != source_signature
        step_started = time.perf_counter()
        _ensure_mission_planner_import_paths()
        runtime_timing["import_path_ms"] = (time.perf_counter() - step_started) * 1000.0
        if force_reload:
            step_started = time.perf_counter()
            _refresh_live_planning_helpers()
            runtime_timing["refresh_helpers_ms"] = (time.perf_counter() - step_started) * 1000.0
        else:
            runtime_timing["refresh_helpers_ms"] = 0.0
        step_started = time.perf_counter()
        import AnS as mp_ans
        from data_def import d0302, d0303, d0304

        try:
            import config as mp_config
        except Exception:
            mp_config = None
        try:
            from data_def import search_speed as mp_search_speed
        except Exception:
            mp_search_speed = None
        runtime_timing["module_import_ms"] = (time.perf_counter() - step_started) * 1000.0
        if force_reload:
            step_started = time.perf_counter()
            mp_ans = importlib.reload(mp_ans)
            d0302 = importlib.reload(d0302)
            d0303 = importlib.reload(d0303)
            d0304 = importlib.reload(d0304)
            if mp_config is not None:
                try:
                    mp_config = importlib.reload(mp_config)
                except Exception:
                    pass
            if mp_search_speed is not None:
                try:
                    mp_search_speed = importlib.reload(mp_search_speed)
                except Exception:
                    pass
            runtime_timing["module_reload_ms"] = (time.perf_counter() - step_started) * 1000.0
        else:
            runtime_timing["module_reload_ms"] = 0.0
        run_divide_and_pattern = mp_ans.run_divide_and_pattern
        build_mission_plan_0301 = mp_ans.build_mission_plan_0301

        uav_cruise_speed = 40.0
        uav_turn_step = 15.0
        applied = False
        param_error: Optional[str] = None
        step_started = time.perf_counter()
        try:
            uav_cruise_speed, uav_turn_step, applied = self._apply_uav_params_from_store(
                d0303,
                d0304_module=d0304,
                mp_config_module=mp_config,
                search_speed_module=mp_search_speed,
                default_cruise=uav_cruise_speed,
                default_turn_step=uav_turn_step,
            )
        except Exception as exc:
            param_error = str(exc)
        runtime_timing["uav_params_ms"] = (time.perf_counter() - step_started) * 1000.0

        step_started = time.perf_counter()
        with self._planner_runtime_lock:
            self._planner_runtime_source_signature_seen = source_signature
        runtime_timing["lock_write_ms"] = (time.perf_counter() - step_started) * 1000.0
        return {
            "run_divide_and_pattern": run_divide_and_pattern,
            "build_mission_plan_0301": build_mission_plan_0301,
            "d0302": d0302,
            "d0303": d0303,
            "d0304": d0304,
            "uav_cruise_speed": float(uav_cruise_speed),
            "uav_turn_step": float(uav_turn_step),
            "uav_params_applied": bool(applied),
            "uav_params_error": param_error,
            "warm_status": {},
            "warm_errors": {},
            "source_signature": source_signature,
            "runtime_timing": {key: round(float(value), 3) for key, value in runtime_timing.items()},
            "runtime_cache_status": "built",
            "runtime_cache_wait_ms": 0.0,
            "runtime_force_reload": bool(force_reload),
        }

    def _warm_planner_auxiliary_pipelines(self) -> Dict[str, Dict[str, Any]]:
        warm_status: Dict[str, Any] = {}
        warm_errors: Dict[str, str] = {}
        for key, warm_fn in (
            ("prior_pipeline", warm_prior_mission_pipeline),
            ("prior_post_rejoin_pipeline", warm_prior_post_rejoin_pipeline),
            ("imaging_schedule_pipeline", warm_imaging_schedule_replan_pipeline),
            ("path_deviation_pipeline", warm_path_deviation_replan_pipeline),
            ("next_collab_pipeline", warm_next_collab_replan_pipeline),
            ("attack_pipeline", warm_attack_plan_pipeline),
            ("post_attack_rejoin_pipeline", warm_post_attack_rejoin_pipeline),
        ):
            try:
                warm_status[key] = warm_fn()
            except BaseException as exc:
                warm_errors[key] = str(exc)
        return {"warm_status": warm_status, "warm_errors": warm_errors}

    def _emit_planner_runtime_warm_ready(
        self,
        runtime: Dict[str, Any],
        *,
        reason: str,
        duration_ms: float,
        cache_status: str,
    ) -> None:
        timing = dict(runtime.get("runtime_timing") or {})
        extra: Dict[str, Any] = {
            "cacheStatus": str(cache_status or runtime.get("runtime_cache_status") or ""),
            "forceReload": int(bool(runtime.get("runtime_force_reload"))),
            "sourceFileCount": len(runtime.get("source_signature") or ()),
            "durationMs": round(float(duration_ms), 3),
        }
        for key, value in timing.items():
            try:
                extra[f"build_{key}"] = round(float(value), 3)
            except Exception:
                continue
        self._emit_lifecycle(
            "planner_runtime_warm_ready",
            component="planner_runtime",
            outcome="ok",
            reason=reason,
            extra=extra,
        )

    def _emit_static_resource_warm_ready(
        self,
        auxiliary_result: Dict[str, Any],
        *,
        reason: str,
        duration_ms: float,
    ) -> None:
        warm_status = dict((auxiliary_result or {}).get("warm_status") or {})
        warm_errors = dict((auxiliary_result or {}).get("warm_errors") or {})
        self._emit_lifecycle(
            "static_resource_warm_ready",
            component="static_resource",
            outcome="ok" if not warm_errors else "degraded",
            reason=reason,
            extra={
                "readyCount": len(warm_status),
                "errorCount": len(warm_errors),
                "pipelines": ",".join(sorted(str(key) for key in warm_status.keys())),
                "errorPipelines": ",".join(sorted(str(key) for key in warm_errors.keys())),
                "durationMs": round(float(duration_ms), 3),
            },
        )

    def _get_planner_runtime(self) -> Dict[str, Any]:
        signature = _planner_runtime_source_signature()
        with self._planner_runtime_lock:
            runtime = self._planner_runtime_cache
            warmup_running = bool(self._planner_runtime_warmup_running)
        if runtime is not None and runtime.get("source_signature") == signature:
            runtime["runtime_cache_status"] = "cache_hit"
            runtime["runtime_cache_wait_ms"] = 0.0
            return runtime

        if warmup_running:
            wait_started = time.perf_counter()
            try:
                self._planner_runtime_ready_event.wait(timeout=3.0)
            except Exception:
                pass
            wait_ms = (time.perf_counter() - wait_started) * 1000.0
            with self._planner_runtime_lock:
                runtime = self._planner_runtime_cache
            if runtime is not None and runtime.get("source_signature") == signature:
                runtime["runtime_cache_status"] = "warmup_hit"
                runtime["runtime_cache_wait_ms"] = round(float(wait_ms), 3)
                return runtime

        build_started = time.perf_counter()
        runtime = self._build_planner_runtime()
        runtime["runtime_cache_status"] = "built"
        runtime["runtime_cache_wait_ms"] = 0.0
        with self._planner_runtime_lock:
            self._planner_runtime_cache = runtime
            self._planner_runtime_ready_event.set()
        self._emit_planner_runtime_warm_ready(
            runtime,
            reason="on_demand",
            duration_ms=(time.perf_counter() - build_started) * 1000.0,
            cache_status="built",
        )
        return runtime

    def _prime_latest_input_file(self, msg_id: str) -> None:
        latest_id = get_latest_package_id(msg_id)
        if latest_id is None:
            return
        snapshot = get_latest_snapshot(msg_id)
        payload = getattr(snapshot, "payload", None)
        if not isinstance(payload, dict):
            return

        prepared = prepare_cached_payload_for_file(msg_id, latest_id, payload)
        if prepared is None:
            return
        directory_name, package_id, payload_copy = prepared

        try:
            db_root = db_paths.get_active_db_root()
        except Exception:
            db_root = db_paths.LEGACY_DB_ROOT

        directory = db_root / directory_name
        directory.mkdir(parents=True, exist_ok=True)
        write_json(
            directory / f"{package_id}.json",
            payload_copy,
            pretty=True,
            ensure_ascii=False,
            skip_if_unchanged=True,
        )

    def _collect_latest_dem_warmup_points(self, *, limit: int = 512) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        seen: set[tuple[float, float]] = set()
        max_points = max(0, int(limit))

        def add_point(lat: Any, lon: Any) -> None:
            if len(points) >= max_points:
                return
            try:
                pair = (round(float(lat), 7), round(float(lon), 7))
            except Exception:
                return
            if pair not in seen:
                seen.add(pair)
                points.append(pair)

        def coord_pair(value: Any) -> tuple[float, float] | None:
            if not isinstance(value, dict):
                return None
            lat = value.get("latitude", value.get("lat"))
            lon = value.get("longitude", value.get("lon"))
            if lat is None or lon is None:
                return None
            try:
                return (float(lat), float(lon))
            except Exception:
                return None

        def add_coordinate_samples(coords: Any) -> None:
            if len(points) >= max_points or not isinstance(coords, (list, tuple)):
                return
            pairs = [pair for pair in (coord_pair(item) for item in coords) if pair is not None]
            if not pairs:
                return
            for lat, lon in pairs:
                add_point(lat, lon)
                if len(points) >= max_points:
                    return

            for (lat_a, lon_a), (lat_b, lon_b) in zip(pairs, pairs[1:]):
                for fraction in (0.25, 0.5, 0.75):
                    add_point(
                        lat_a + (lat_b - lat_a) * fraction,
                        lon_a + (lon_b - lon_a) * fraction,
                    )
                    if len(points) >= max_points:
                        return

            if len(pairs) >= 3:
                lats = [lat for lat, _lon in pairs]
                lons = [lon for _lat, lon in pairs]
                lat_min, lat_max = min(lats), max(lats)
                lon_min, lon_max = min(lons), max(lons)
                for fy in (0.25, 0.5, 0.75):
                    for fx in (0.25, 0.5, 0.75):
                        add_point(
                            lat_min + (lat_max - lat_min) * fy,
                            lon_min + (lon_max - lon_min) * fx,
                        )
                        if len(points) >= max_points:
                            return

        def visit(value: Any) -> None:
            if len(points) >= max_points:
                return
            if isinstance(value, dict):
                if isinstance(value.get("coordinateList"), (list, tuple)):
                    add_coordinate_samples(value.get("coordinateList"))
                    if len(points) >= max_points:
                        return
                pair = coord_pair(value)
                if pair is not None:
                    add_point(pair[0], pair[1])
                for nested in value.values():
                    visit(nested)
                    if len(points) >= max_points:
                        return
            elif isinstance(value, (list, tuple)):
                for nested in value:
                    visit(nested)
                    if len(points) >= max_points:
                        return

        for msg_id in ("0201", "0203"):
            try:
                snapshot = get_latest_snapshot(msg_id)
                payload = getattr(snapshot, "payload", None)
                if isinstance(payload, dict):
                    visit(payload)
            except Exception:
                continue
        if len(points) >= max_points:
            return points

        try:
            db_root = db_paths.get_active_db_root()
        except Exception:
            db_root = db_paths.LEGACY_DB_ROOT
        for directory_name in ("InputMissionPlan", "MissionReferenceInfo"):
            if len(points) >= max_points:
                break
            directory = db_root / directory_name
            try:
                candidates = sorted(
                    (path for path in directory.glob("*.json") if path.is_file()),
                    key=lambda path: (
                        path.stat().st_mtime,
                        int(path.stem) if str(path.stem).isdigit() else -1,
                    ),
                    reverse=True,
                )
            except Exception:
                candidates = []
            for path in candidates[:3]:
                if len(points) >= max_points:
                    break
                try:
                    payload = json.loads(path.read_text(encoding="utf-8"))
                except Exception:
                    continue
                visit(payload)

        return points

    def _warm_terrain_runtime_cache(
        self,
        reason: str,
        *,
        points: Optional[list[tuple[float, float]]] = None,
    ) -> None:
        with self._terrain_runtime_warmup_lock:
            if self._terrain_runtime_warmup_running:
                self.log_sig.emit(f"[WARM] DEM cache warm-up skipped ({reason}, already_running)")
                self._emit_lifecycle(
                    "terrain_warm_ready",
                    component="terrain_warmup",
                    outcome="skipped",
                    reason=reason,
                    extra={"skipReason": "already_running"},
                )
                return
            self._terrain_runtime_warmup_running = True
        started = time.perf_counter()
        if points is None:
            point_limit = 1024 if "initial" in str(reason).lower() else 512
            points = self._collect_latest_dem_warmup_points(limit=point_limit)
        namespace = "data_def"
        d0303_alt_points = 0
        d0303_alt_ms = 0.0
        coverage_error = False
        try:
            helper_module = None
            for module_name in (
                "data_def.mission_helpers",
                "modules.mission_planning.MissionPlanner.data_def.mission_helpers",
            ):
                candidate = sys.modules.get(module_name)
                if candidate is not None and callable(getattr(candidate, "warm_terrain_cache", None)):
                    helper_module = candidate
                    namespace = module_name
                    break
            if helper_module is None:
                try:
                    _ensure_mission_planner_import_paths()
                    from data_def import mission_helpers as helper_module  # type: ignore
                except Exception:
                    try:
                        from modules.mission_planning.MissionPlanner.data_def import mission_helpers as helper_module
                        namespace = "modules"
                    except Exception as exc:
                        self.log_sig.emit(f"[WARM WARN] DEM cache warm-up import failed ({reason}): {exc}")
                        self._emit_lifecycle(
                            "terrain_warm_ready",
                            component="terrain_warmup",
                            outcome="error",
                            reason=reason,
                            extra={"error": str(exc)[:500]},
                        )
                        return
            try:
                info = helper_module.warm_terrain_cache(points)
            except Exception as exc:
                self.log_sig.emit(f"[WARM WARN] DEM cache warm-up failed ({reason}): {exc}")
                self._emit_lifecycle(
                    "terrain_warm_ready",
                    component="terrain_warmup",
                    outcome="error",
                    reason=reason,
                    extra={"error": str(exc)[:500]},
                )
                return

            elapsed_ms = (time.perf_counter() - started) * 1000.0
            terrain_pixel = info.get("terrain_pixel") if isinstance(info, dict) else {}
            pixel_currsize = (
                int(terrain_pixel.get("currsize") or 0)
                if isinstance(terrain_pixel, dict)
                else 0
            )
            warmup_info = info.get("warmup") if isinstance(info, dict) else {}
            loaded_tiles = (
                warmup_info.get("loadedTiles")
                if isinstance(warmup_info, dict) and isinstance(warmup_info.get("loadedTiles"), list)
                else []
            )
            bbox_tiles = (
                warmup_info.get("bboxLoadedTiles")
                if isinstance(warmup_info, dict) and isinstance(warmup_info.get("bboxLoadedTiles"), list)
                else []
            )
            missing_dem_names = (
                list(warmup_info.get("missingDemNames") or [])
                if isinstance(warmup_info, dict)
                else []
            )
            available_dem_names = (
                list(warmup_info.get("availableDemNames") or [])
                if isinstance(warmup_info, dict)
                else []
            )
            detected_tif_names = (
                list(warmup_info.get("detectedTifNames") or [])
                if isinstance(warmup_info, dict)
                else []
            )
            unregistered_tif_names = (
                list(warmup_info.get("unregisteredTifNames") or [])
                if isinstance(warmup_info, dict)
                else []
            )
            requested_dem_names = (
                list(warmup_info.get("requestedDemNames") or [])
                if isinstance(warmup_info, dict)
                else []
            )
            missing_required_names = (
                list(warmup_info.get("missingRequiredDemNames") or [])
                if isinstance(warmup_info, dict)
                else []
            )
            unresolved_count = (
                int(warmup_info.get("unresolvedRequirementPointCount") or 0)
                if isinstance(warmup_info, dict)
                else 0
            )
            unresolved_samples = (
                list(warmup_info.get("unresolvedRequirementSamples") or [])
                if isinstance(warmup_info, dict)
                else []
            )
            load_errors = (
                list(warmup_info.get("loadErrors") or [])
                if isinstance(warmup_info, dict)
                else []
            )
            inventory_warning = bool(missing_dem_names or unregistered_tif_names or load_errors)
            coverage_error = bool(points) and bool(unresolved_count or load_errors)

            if inventory_warning:
                self.log_sig.emit(
                    "[DEM][WARN] Operational DEM inventory incomplete "
                    f"(missing={missing_dem_names or 'none'}, "
                    f"available={available_dem_names or 'none'}, "
                    f"detected={detected_tif_names or 'none'}, "
                    f"unregistered={unregistered_tif_names or 'none'}, "
                    f"loadErrors={len(load_errors)})"
                )
            if coverage_error:
                first_sample = unresolved_samples[0] if unresolved_samples else {}
                expected_path = (
                    first_sample.get("expectedDemPath")
                    if isinstance(first_sample, dict)
                    else None
                )
                self.log_sig.emit(
                    "[DEM][ERROR] Required DEM unavailable; warm-up not ready "
                    f"(reason={reason}, requested={requested_dem_names or 'none'}, "
                    f"missingRequired={missing_required_names or 'none'}, "
                    f"expectedPath={expected_path or 'n/a'}, unresolvedPoints={unresolved_count}, "
                    f"loadErrors={load_errors or 'none'}, "
                    f"unregistered={unregistered_tif_names or 'none'}, elapsed={elapsed_ms:.1f}ms)"
                )
            else:
                self.log_sig.emit(
                    f"[WARM] DEM cache ready ({reason}, points={len(points)}, "
                    f"pixels={pixel_currsize}, tiles={len(loaded_tiles)}, bboxTiles={len(bbox_tiles)}, "
                    f"namespace={namespace}, elapsed={elapsed_ms:.1f}ms)"
                )
            lifecycle_outcome = "error" if coverage_error else ("warning" if inventory_warning else "ok")
            self._emit_lifecycle(
                "terrain_warm_ready",
                component="terrain_warmup",
                outcome=lifecycle_outcome,
                reason=reason,
                extra={
                    "points": len(points),
                    "pixels": pixel_currsize,
                    "tiles": len(loaded_tiles),
                    "bboxTiles": len(bbox_tiles),
                    "namespace": namespace,
                    "durationMs": round(float(elapsed_ms), 3),
                    "missingDemNames": missing_dem_names,
                    "availableDemNames": available_dem_names,
                    "detectedTifNames": detected_tif_names,
                    "unregisteredTifNames": unregistered_tif_names,
                    "requestedDemNames": requested_dem_names,
                    "missingRequiredDemNames": missing_required_names,
                    "unresolvedRequirementPointCount": unresolved_count,
                    "loadErrors": load_errors,
                },
            )
            try:
                dem_alt_many = None
                for module_name in (
                    "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0303",
                    "modules.mission_planning.MissionPlanner.data_def.d0303",
                    "data_def.d0303",
                ):
                    candidate = sys.modules.get(module_name)
                    if candidate is not None and callable(getattr(candidate, "_dem_alt_many", None)):
                        dem_alt_many = getattr(candidate, "_dem_alt_many")
                        break
                if dem_alt_many is not None and points and not coverage_error:
                    d0303_alt_started = time.perf_counter()
                    dem_alt_many(list(points))
                    d0303_alt_ms = (time.perf_counter() - d0303_alt_started) * 1000.0
                    d0303_alt_points = len(points)
            except Exception:
                d0303_alt_points = 0
                d0303_alt_ms = 0.0
        finally:
            if d0303_alt_points:
                try:
                    self.log_sig.emit(
                        f"[WARM] 0303 DEM altitude cache ready ({reason}, "
                        f"points={d0303_alt_points}, elapsed={d0303_alt_ms:.1f}ms)"
                    )
                    self._emit_lifecycle(
                        "terrain_altitude_warm_ready",
                        component="terrain_warmup",
                        outcome="ok",
                        reason=reason,
                        extra={
                            "points": int(d0303_alt_points),
                            "durationMs": round(float(d0303_alt_ms), 3),
                        },
                    )
                except Exception:
                    pass
            with self._terrain_runtime_warmup_lock:
                self._terrain_runtime_warmup_running = False

    def _schedule_replan_terrain_warmup(self, ctx: Dict[str, Any], payload: Dict[str, Any]) -> None:
        try:
            timing_kind = _classify_replan_timing_context(ctx, payload).get("trigger")
        except Exception:
            timing_kind = ""
        reason_text = ""
        try:
            if isinstance(ctx, dict):
                reason_text = str(ctx.get("reason") or "")
            if not reason_text and isinstance(payload, dict):
                reason_text = str(payload.get("reason") or "")
        except Exception:
            reason_text = ""
        is_general_option = str(timing_kind) == "general_3_option"
        is_initial_replan = reason_text.strip() == "초기임무재계획"
        if not (is_general_option or is_initial_replan):
            return
        warmup_reason = "0902_initial_preplan" if is_initial_replan else "0902_preplan"
        with self._replan_terrain_warmup_lock:
            if self._replan_terrain_warmup_running:
                return
            self._replan_terrain_warmup_running = True

        ctx_snapshot = ctx if isinstance(ctx, dict) else {}
        try:
            self._record_replan_timing_event(
                "terrain_warmup_started",
                ctx=ctx_snapshot,
                extra={"reason": warmup_reason},
            )
        except Exception:
            pass

        def worker() -> None:
            started = time.perf_counter()
            point_count = 0
            try:
                point_limit = 1024 if warmup_reason == "0902_initial_preplan" else 512
                warmup_points = self._collect_latest_dem_warmup_points(limit=point_limit)
                point_count = len(warmup_points)
                self._warm_terrain_runtime_cache(warmup_reason, points=warmup_points)
            finally:
                with self._replan_terrain_warmup_lock:
                    self._replan_terrain_warmup_running = False
                try:
                    self._record_replan_timing_event(
                        "terrain_warmup_finished",
                        ctx=ctx_snapshot,
                        extra={
                            "reason": warmup_reason,
                            "points": int(point_count),
                            "duration_ms": round((time.perf_counter() - started) * 1000.0, 3),
                        },
                    )
                except Exception:
                    pass

        threading.Thread(
            target=worker,
            name="Replan-Terrain-Warmup",
            daemon=True,
        ).start()

    def _schedule_planner_warmup(self, reason: str = "background") -> None:
        with self._planner_runtime_lock:
            if self._planner_runtime_warmup_running:
                self._planner_runtime_warmup_pending = reason
                return
            self._planner_runtime_warmup_running = True

        threading.Thread(
            target=self._planner_warmup_worker,
            args=(reason,),
            name="Planner-Warmup",
            daemon=True,
        ).start()

    def _schedule_0303_process_pool_warmup(self, reason: str) -> None:
        def _bool_value(raw: Any) -> bool:
            if isinstance(raw, str):
                return raw.strip().lower() in {"1", "true", "yes", "on"}
            return bool(raw)

        try:
            values = (load_runtime_settings().get("values") or {})
            general_enabled = _bool_value(
                values.get("replan_0303_aircraft_process_parallel_enabled", False)
            )
            input_refresh_enabled = _bool_value(
                values.get("input_refresh_0303_process_parallel_enabled", False)
            )
            enabled = bool(general_enabled or input_refresh_enabled)
            general_workers = max(
                1,
                int(values.get("replan_0303_aircraft_process_workers", 3) or 3),
            )
            input_refresh_workers = max(
                1,
                int(values.get("input_refresh_0303_process_workers", 3) or 3),
            )
            workers = max(
                general_workers if general_enabled else 1,
                input_refresh_workers if input_refresh_enabled else 1,
            )
        except Exception:
            enabled = False
            workers = 1
        if not enabled or workers < 2 or getattr(self, "_shutdown_started", False):
            return
        with self._0303_process_warmup_lock:
            if self._0303_process_warmup_running:
                return
            self._0303_process_warmup_running = True

        def worker() -> None:
            try:
                if getattr(self, "_shutdown_started", False):
                    return
                # Resolve after planner hot-reload so the helper and pool
                # generation always match the next build function.
                from modules.mission_planning.runtime.aircraft_parallel_0303 import (
                    warm_persistent_0303_process_pool,
                )

                terrain_points = self._collect_latest_dem_warmup_points(limit=512)
                runtime_payload = load_runtime_settings()
                runtime_values = dict(runtime_payload.get("values") or {})
                runtime_values["dem_alt_cache_round_decimals"] = runtime_values.get(
                    "input_refresh_dem_alt_cache_round_decimals",
                    runtime_values.get("dem_alt_cache_round_decimals", 7),
                )
                runtime_payload = dict(runtime_payload)
                runtime_payload["values"] = runtime_values
                result = warm_persistent_0303_process_pool(
                    max_workers=workers,
                    terrain_points=terrain_points,
                    runtime_payload=runtime_payload,
                )
                if bool(result.get("warmed")):
                    self.log_sig.emit(
                        f"[WARM] 0303 process pool ready ({reason}, workers={workers}, "
                        f"processes={len(result.get('workerPIDs') or [])}, "
                        f"terrainPoints={int(result.get('terrainPointCount') or 0)}, "
                        f"elapsed={float(result.get('elapsed_ms') or 0.0):.1f}ms)"
                    )
                elif str(result.get("reason") or "") not in {
                    "persistent_pool_disabled",
                    "worker_count_lt_2",
                }:
                    self.log_sig.emit(
                        f"[WARM WARN] 0303 process pool warm-up skipped/failed "
                        f"({reason}, detail={result.get('reason') or 'unknown'})"
                    )
            except Exception as exc:
                self.log_sig.emit(f"[WARM WARN] 0303 process pool warm-up failed ({reason}): {exc}")
            finally:
                with self._0303_process_warmup_lock:
                    self._0303_process_warmup_running = False

        threading.Thread(
            target=worker,
            name="Planner-0303-Process-Warmup",
            daemon=True,
        ).start()

    def _planner_warmup_worker(self, reason: str) -> None:
        announce_ready = False
        runtime: Optional[Dict[str, Any]] = None
        started = time.perf_counter()
        try:
            runtime = self._build_planner_runtime()
            with self._planner_runtime_lock:
                self._planner_runtime_cache = runtime
                self._planner_runtime_ready_event.set()
                if not self._planner_runtime_ready_logged:
                    self._planner_runtime_ready_logged = True
                    announce_ready = True
            self._schedule_0303_process_pool_warmup(reason)
            self._emit_planner_runtime_warm_ready(
                runtime,
                reason=reason,
                duration_ms=(time.perf_counter() - started) * 1000.0,
                cache_status="warmup_built",
            )
        except Exception as exc:
            self.log_sig.emit(f"[WARM WARN] Planner warm-up failed ({reason}): {exc}")
            self._emit_lifecycle(
                "planner_runtime_warm_ready",
                component="planner_runtime",
                outcome="error",
                reason=reason,
                extra={
                    "error": str(exc)[:500],
                    "durationMs": round((time.perf_counter() - started) * 1000.0, 3),
                },
            )
        else:
            auxiliary_result: Dict[str, Any] = {}
            auxiliary_started = time.perf_counter()
            try:
                auxiliary_result = self._warm_planner_auxiliary_pipelines()
            except Exception as exc:
                self.log_sig.emit(f"[WARM WARN] Planner auxiliary warm-up failed ({reason}): {exc}")
                auxiliary_result = {"warm_status": {}, "warm_errors": {"auxiliary": str(exc)}}
            self._emit_static_resource_warm_ready(
                auxiliary_result,
                reason=reason,
                duration_ms=(time.perf_counter() - auxiliary_started) * 1000.0,
            )
            warm_errors = dict((auxiliary_result or {}).get("warm_errors") or {})
            for pipeline_name, error_text in warm_errors.items():
                self.log_sig.emit(f"[WARM WARN] {pipeline_name} warm-up failed ({reason}): {error_text}")
            if announce_ready:
                self.log_sig.emit(f"[WARM] Mission planner runtime ready ({reason})")
            try:
                self._prime_latest_input_file("0201")
                self._prime_latest_input_file("0203")
                self._warm_terrain_runtime_cache(reason)
            except Exception as exc:
                self.log_sig.emit(f"[WARM WARN] Planner post-runtime warm-up failed ({reason}): {exc}")
        finally:
            pending_reason = None
            with self._planner_runtime_lock:
                self._planner_runtime_warmup_running = False
                pending_reason = self._planner_runtime_warmup_pending
                self._planner_runtime_warmup_pending = None
            if pending_reason:
                self._schedule_planner_warmup(pending_reason)

    def _invalidate_planner_runtime(self, *, warm_reason: Optional[str] = None) -> None:
        with self._planner_runtime_lock:
            self._planner_runtime_cache = None
            self._planner_runtime_ready_event.clear()
        if warm_reason:
            self._schedule_planner_warmup(warm_reason)

    # ───────── 0101 모드 수신 리스너 ─────────
    def _install_0101_mode_listener(self):
        """
        receive_center.notify("0101", raw)를 직접 수신해
        systemMode 숫자코드를 슬라이더로 바로 반영.
        """
        class _Rx0101:
            def __init__(self, host): self.host = host
            def mark_received(self, msg_id: str, raw: bytes | None = None):
                try:
                    self.host._on_rx_0101(raw)
                except Exception:
                    pass

        try:
            self._rx0101 = _Rx0101(self)
            register_listener("0101", self._rx0101)
            self._append_log_line("[0101] 모드 수신 리스너 등록 완료")
            self._emit_lifecycle("listener_start", component="0101_listener", outcome="ok")
        except Exception as e:
            self._append_log_line(f"[0101] 리스너 등록 실패: {e}")
            self._emit_lifecycle(
                "listener_fail",
                component="0101_listener",
                outcome="failure",
                reason=str(e),
            )

    def _on_rx_0101(self, raw: bytes | None):
        body = _parse_system_mode_payload_body(raw)
        code = extract_system_mode_code(raw, body)

        if code is None:
            # 조용히 무시(불필요한 실패 로그 없음)
            return

        if self._apply_system_mode_code(code):
            self._append_log_line(f"[0101] 시스템 운용 모드 수신 → code={code}")
        else:
            self._append_log_line(f"[MODE] 미지원 코드({code})")

    def _extract_mode_code(self, body: dict) -> int | None:
        """
        dict의 다양한 키에서 모드코드를 견고하게 추출.
        - 키 대/소문자 무시
        - 값이 str/bool/float 모두 허용
        """
        return _extract_mode_code_from_body(body)

    def _apply_system_mode_code(self, code: int) -> bool:
        """
        외부 0101 systemMode 매핑
          0 : 초기화 모드
          1 : 대기 모드
          2 : 초기임무계획 모드
          3 : 임무수행 모드
        내부 슬라이더(0~3): [0=초기화 모드, 1=대기모드, 2=초기 임무 계획, 3=임무 수행]
        → 매핑: 동일(0→0, 1→1, 2→2, 3→3)
        """
        if code not in (0, 1, 2, 3):
            return None
        val = code
        try:
            self.mode_slider.blockSignals(True)
            self.mode_slider.setValue(val)
            self.mode_slider.blockSignals(False)
            # 기존 부수효과(전원/주기TX/모니터링 통지) 실행
            self._on_mode_slider_changed(val)
            if code in (2, 3):
                self._mark_post_0301_ready(trigger=f"0101 code={code}")
        except Exception:
            return None
        return True

    # ───────── RX 테이블 폴링 기반 0101 모드 반영(리시버 경로 폴백) ─────────
    def _start_0101_rx_poller(self):
        self._last_0101_raw = None
        self._poll_0101_timer = QTimer(self)
        self._poll_0101_timer.setInterval(250)  # 4Hz
        self._poll_0101_timer.timeout.connect(self._poll_0101_in_rx_table)
        self._poll_0101_timer.start()

    def _poll_0101_in_rx_table(self):
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
            payload = item.data(Qt.UserRole) if item else None
            if tab and hasattr(tab, "_latest_payload_bytes"):
                raw_latest = tab._latest_payload_bytes(payload)
            else:
                raw_latest = payload if isinstance(payload, (bytes, bytearray)) else b""
            if not raw_latest:
                return
            if self._last_0101_raw is not None and raw_latest == self._last_0101_raw:
                return

            body = _parse_system_mode_payload_body(raw_latest)
            code = extract_system_mode_code(raw_latest, body)

            if code is not None:
                if self._apply_system_mode_code(code):
                    self._append_log_line(f"[0101/POLL] 모드 동기화 code={code}")
                self._last_0101_raw = raw_latest
        except Exception:
            pass


    # ───────── 모니터링(대시보드) 전송 훅 ─────────
    def _find_tx_row(self, code: str) -> int:
        tab = getattr(self, "_tab", None)
        tbl = getattr(tab, "tbl_tx", None) if tab else None
        if tbl is None:
            return -1
        try:
            needle = str(code).strip()
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == needle:
                    return r
        except Exception:
            return -1
        return -1

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

    def _normalize_0102_body_template(self, body: Optional[dict], *, status: Optional[int] = None) -> dict[str, Any]:
        return normalize_0102_body_template(body, status=status)

    def _set_0102_body_template(self, body: Optional[dict], *, status: Optional[int] = None) -> None:
        self._hb_0102_body_template = self._normalize_0102_body_template(body, status=status)

    def _build_0102_body(self, *, status: Optional[int] = None) -> dict[str, Any]:
        return _build_0102_body_payload(
            getattr(self, "_hb_0102_body_template", None),
            now_ms=_now_ms_since_2000,
            status=status,
        )

    def _0102_push_enabled(self) -> bool:
        raw = str(os.getenv("KU_MMR_0102_PUSH_ENABLED", "1") or "1").strip().lower()
        return raw in {"1", "true", "yes", "on"}

    def _send_startup_0102_once(self) -> None:
        if not self._0102_push_enabled():
            if not getattr(self, "_startup_0102_skip_logged", False):
                self._startup_0102_skip_logged = True
                self._append_log_line("[0102] startup status push disabled (set KU_MMR_0102_PUSH_ENABLED=1 to enable)")
                self._emit_lifecycle(
                    "startup_status_skip",
                    component="0102",
                    outcome="skipped",
                    reason="push_disabled",
                )
            return
        send_status_ok("MMR")

    def _start_0102_heartbeat_worker_if_needed(self) -> None:
        if not self._0102_push_enabled():
            if not getattr(self, "_heartbeat_0102_skip_logged", False):
                self._heartbeat_0102_skip_logged = True
                self._append_log_line("[0102] heartbeat disabled (set KU_MMR_0102_PUSH_ENABLED=1 to enable)")
                self._emit_lifecycle(
                    "heartbeat_skip",
                    component="0102_heartbeat",
                    outcome="skipped",
                    reason="push_disabled",
                )
            return
        timer = getattr(self, "_hb_0102_timer", None)
        if timer is None:
            timer = QTimer(self)
            timer.setInterval(max(1, int(float(getattr(self, "_hb_0102_interval_sec", 0.2) or 0.2) * 1000)))
            timer.timeout.connect(self._send_0102_heartbeat_tick)
            self._hb_0102_timer = timer
        if timer.isActive():
            return
        timer.start()
        self._emit_lifecycle(
            "heartbeat_timer_start",
            component="0102_heartbeat",
            outcome="ok",
            extra={"intervalSec": float(getattr(self, "_hb_0102_interval_sec", 0.2) or 0.2)},
        )

    def _send_0102_heartbeat_tick(self) -> None:
        if not self._hb_0102_enabled:
            self._hb_0102_last_tick_monotonic = 0.0
            return
        if not self._power_on:
            self._hb_0102_last_tick_monotonic = 0.0
            return
        if not getattr(self, "_bus_ready", False):
            self._hb_0102_last_tick_monotonic = 0.0
            return
        tick_started = time.monotonic()
        interval_sec = float(getattr(self, "_hb_0102_interval_sec", 0.2) or 0.2)
        previous_tick = float(getattr(self, "_hb_0102_last_tick_monotonic", 0.0) or 0.0)
        self._hb_0102_last_tick_monotonic = tick_started
        tick_lag_ms = (
            max(0.0, (tick_started - previous_tick - interval_sec) * 1000.0)
            if previous_tick > 0.0
            else 0.0
        )

        def _record_0102_timing(*, success: bool, send_ms: float) -> None:
            now_mono = time.monotonic()
            if success:
                self._hb_0102_last_success_monotonic = now_mono
            last_success = float(getattr(self, "_hb_0102_last_success_monotonic", 0.0) or 0.0)
            last_success_age_ms = (
                max(0.0, (now_mono - last_success) * 1000.0)
                if last_success > 0.0
                else None
            )
            if success and tick_lag_ms < 400.0 and send_ms < 100.0:
                return
            try:
                from modules.common.process_console import request_runtime_diagnostic_snapshot

                request_runtime_diagnostic_snapshot(
                    "mission_planning",
                    "0102_heartbeat_threshold",
                    context={
                        "event": "0102_heartbeat_tick",
                        "tickLagMs": round(tick_lag_ms, 3),
                        "sendMs": round(max(0.0, send_ms), 3),
                        "lastSuccessAgeMs": (
                            None if last_success_age_ms is None else round(last_success_age_ms, 3)
                        ),
                        "success": bool(success),
                    },
                )
            except Exception:
                pass
        try:
            from push_center import push_message
        except Exception as exc:
            _record_0102_timing(success=False, send_ms=0.0)
            now = time.monotonic()
            if (now - float(getattr(self, "_hb_0102_last_warn", 0.0) or 0.0)) >= 5.0:
                self._hb_0102_last_warn = now
                self._append_log_line(f"[ERR] 0102 heartbeat import failed: {exc}")
                self._emit_lifecycle(
                    "heartbeat_timer_fail",
                    component="0102_heartbeat",
                    outcome="failure",
                    reason=str(exc),
                )
            return

        try:
            send_started = time.monotonic()
            body = self._build_0102_body()
            ok = push_message("0102", NodeMessenger, body_dict=body)
            send_ms = max(0.0, (time.monotonic() - send_started) * 1000.0)
            if ok:
                self._self_check_sent = True
                _record_0102_timing(success=True, send_ms=send_ms)
                return
            raise RuntimeError("push_message returned False")
        except Exception as exc:
            send_ms = max(
                0.0,
                (time.monotonic() - float(locals().get("send_started", tick_started))) * 1000.0,
            )
            _record_0102_timing(success=False, send_ms=send_ms)
            now = time.monotonic()
            if (now - float(getattr(self, "_hb_0102_last_warn", 0.0) or 0.0)) >= 5.0:
                self._hb_0102_last_warn = now
                self._append_log_line(f"[WARN] 0102 heartbeat send failed: {exc}")
                self._emit_lifecycle(
                    "heartbeat_timer_fail",
                    component="0102_heartbeat",
                    outcome="failure",
                    reason=str(exc),
                )

    def _run_0102_heartbeat_worker(self) -> None:
        try:
            from push_center import push_message
        except Exception as exc:
            try:
                self.log_sig.emit(f"[ERR] 0102 heartbeat import failed: {exc}")
            except Exception:
                pass
            self._emit_lifecycle(
                "heartbeat_worker_fail",
                component="0102_heartbeat",
                outcome="failure",
                reason=str(exc),
            )
            return

        interval = float(getattr(self, "_hb_0102_interval_sec", 0.2) or 0.2)
        next_due = time.monotonic() + interval
        last_warn = 0.0

        while not self._hb_0102_stop.is_set():
            if not self._hb_0102_enabled:
                self._hb_0102_stop.wait(0.1)
                next_due = time.monotonic() + interval
                continue
            if not self._power_on:
                self._hb_0102_stop.wait(0.1)
                next_due = time.monotonic() + interval
                continue
            if not getattr(self, "_bus_ready", False):
                self._hb_0102_stop.wait(0.1)
                next_due = time.monotonic() + interval
                continue

            now = time.monotonic()
            if now < next_due:
                self._hb_0102_stop.wait(min(0.05, next_due - now))
                continue

            # 큰 지연이 발생한 경우 burst 전송 대신 다음 주기로 재정렬한다.
            if now - next_due > interval:
                next_due = now + interval
            else:
                next_due += interval

            try:
                body = self._build_0102_body()
                push_message("0102", NodeMessenger, body_dict=body)
                self._self_check_sent = True
            except Exception as exc:
                # heartbeat 실패 로그는 저주기로만 남겨서 GUI 이벤트 큐 과부하를 막는다.
                if (now - last_warn) >= 5.0:
                    try:
                        self.log_sig.emit(f"[WARN] 0102 heartbeat send failed: {exc}")
                    except Exception:
                        pass
                    self._emit_lifecycle(
                        "heartbeat_worker_fail",
                        component="0102_heartbeat",
                        outcome="failure",
                        reason=str(exc),
                    )
                    last_warn = now
                next_due = time.monotonic() + max(interval, 0.5)
        self._emit_lifecycle("heartbeat_worker_stop", component="0102_heartbeat", outcome="ok")

    def _set_0102_heartbeat_enabled(self, enabled: bool) -> None:
        self._hb_0102_enabled = bool(enabled)
        if self._hb_0102_enabled:
            self._start_0102_heartbeat_worker_if_needed()
            self._set_0102_tx_state(True)
            self._emit_lifecycle("heartbeat_enable", component="0102_heartbeat", outcome="ok")
        else:
            timer = getattr(self, "_hb_0102_timer", None)
            if timer is not None and timer.isActive():
                timer.stop()
            self._set_0102_tx_state(False)
            self._emit_lifecycle("heartbeat_disable", component="0102_heartbeat", outcome="ok")

    def _stop_tab_periodic_0102_if_running(self) -> None:
        try:
            tab = self._tab
            timers = getattr(tab, "periodic_timers", {})
            timer = timers.get("0102")
            if timer is not None:
                timer.stop()
                timer.deleteLater()
                del timers["0102"]
        except Exception:
            pass

    def _install_0301_override(self):
        tab = getattr(self, "_tab", None)
        if not tab or not hasattr(tab, "_on_tx_button_clicked") or hasattr(self, "_tx_override_installed"):
            return

        original_handler = tab._on_tx_button_clicked

        def _wrapped(row: int):
            code = ""
            try:
                item = tab.tbl_tx.item(row, 0)
                if item is not None:
                    code = item.text().strip()
            except Exception:
                code = ""

            if code == "0102":
                if bool(getattr(self, "_hb_0102_enabled", False)):
                    self._ensure_0102(False)
                    return

                freq = None
                try:
                    freq = getattr(tab, "periodic_config", {}).get("0102")
                except Exception:
                    freq = None

                body = None
                edit_payload = getattr(tab, "edit_tx_payload", None)
                build_payload = getattr(tab, "default_tx_payload", None)
                if callable(build_payload):
                    try:
                        body = build_payload("0102")
                    except Exception:
                        body = None
                if not isinstance(body, dict):
                    body = self._build_0102_body()

                if callable(edit_payload):
                    try:
                        body = edit_payload("0102", body, periodic_rate_hz=freq)
                    except Exception as exc:
                        self._append_log_line(f"[ERR] 0102 편집창 실패: {exc}")
                        return
                if body is None:
                    self._append_log_line("[0102] 송신 취소")
                    return

                self._set_0102_body_template(body)
                self._ensure_0102(True)
                return

            if code == "0301":
                plan_ids: list[int] = []
                for pid in self._scheduled_0301_plan_ids or []:
                    try:
                        plan_ids.append(int(pid))
                    except Exception:
                        continue
                plan_ids = list(dict.fromkeys(plan_ids))
                if not plan_ids:
                    return original_handler(row)
                payloads = self._edit_0301_delivery_payloads(plan_ids)
                if not payloads:
                    self._append_log_line("[0301] 송신 취소")
                    return
                self._send_0301_payloads(payloads)
                return

            return original_handler(row)

        tab._on_tx_button_clicked = _wrapped  # type: ignore
        self._tx_override_installed = True

    def _normalize_0301_payload(self, body: Optional[dict], *, fallback_plan_id: Optional[int] = None) -> dict[str, Any]:
        payload = dict(body or {})
        payload["timestamp"] = int(payload.get("timestamp") or payload.get("Timestamp") or _now_ms_since_2000())
        source = payload.get("source") or payload.get("Source") or "MMR"
        payload["source"] = str(source)
        payload.pop("Source", None)

        plan_id = payload.get("missionPlanID", fallback_plan_id)
        try:
            payload["missionPlanID"] = int(plan_id)
        except Exception:
            payload["missionPlanID"] = 0
        return payload

    def _push_0301_payload(self, body: dict) -> bool:
        try:
            from push_center import push_message
        except Exception as exc:
            self._append_log_line(f"[ERR] 0301 push unavailable: {exc}")
            return False

        body = self._normalize_0301_payload(body)
        mpid = body.get("missionPlanID", 0)
        if int(mpid) <= 0:
            self._append_log_line(f"[WARN] 0301 skipped: invalid missionPlanID={mpid}")
            return False

        try:
            sent = bool(push_message("0301", NodeMessenger, body_dict=body))
            if not sent:
                self._append_log_line(f"[ERR] 0301 push returned false: missionPlanID={mpid}")
                return False
            raw = json.dumps(body, ensure_ascii=False).encode("utf-8", "ignore")
        except Exception as exc:
            self._append_log_line(f"[ERR] 0301 push failed: {exc}")
            return False

        try:
            self.log_sig.emit(f"[0301] missionPlanID={mpid} 전송")
        except Exception:
            pass

        try:
            self.tab_mark_sent_sig.emit(_z4("0301"), raw)
        except Exception:
            pass
        return True

    # NOTE: MMR no longer publishes 0204. The 협업기저임무계획(0204) is pre-sent by
    # MSM exclusively, and only for the type-1 0201 review flow; other package
    # types go straight to 0902 with no 0204. MMR keeps only the 0204 receiver
    # (_on_output_plan_0204) as a trace of MSM's reviewed-package announcements.

    def _push_single_0301(self, mission_plan_id: int) -> bool:
        body = {
            "timestamp": _now_ms_since_2000(),
            "source": "MMR",
            "missionPlanID": mission_plan_id,
        }
        return bool(self._push_0301_payload(body))

    def _send_0301_batch(self, plan_ids: list[int]) -> bool:
        results = [bool(self._push_single_0301(pid)) for pid in plan_ids]
        self._scheduled_0301_plan_ids = []
        return bool(results) and all(results)

    def _send_0301_payloads(self, payloads: list[dict]) -> bool:
        results = [bool(self._push_0301_payload(payload)) for payload in payloads]
        self._scheduled_0301_plan_ids = []
        return bool(results) and all(results)

    def _edit_0301_delivery_payloads(self, plan_ids: list[int]) -> list[dict] | None:
        unique_ids: list[int] = []
        for plan_id in plan_ids:
            try:
                normalized = int(plan_id)
            except Exception:
                continue
            if normalized > 0:
                unique_ids.append(normalized)
        unique_ids = list(dict.fromkeys(unique_ids))
        if not unique_ids:
            return None

        tab = getattr(self, "_tab", None)
        base_payloads = [
            self._normalize_0301_payload(
                {
                    "timestamp": _now_ms_since_2000(),
                    "source": "MMR",
                    "missionPlanID": plan_id,
                },
                fallback_plan_id=plan_id,
            )
            for plan_id in unique_ids
        ]

        if len(base_payloads) == 1:
            editor = getattr(tab, "edit_tx_payload", None)
            if callable(editor):
                payload = editor("0301", base_payloads[0], periodic_rate_hz=None)
                if payload is None:
                    return None
                return [self._normalize_0301_payload(payload, fallback_plan_id=unique_ids[0])]
            return base_payloads

        dialog = JsonPayloadBatchDialog("0301", "임무 계획", base_payloads, parent=self)
        if dialog.exec_() != QDialog.Accepted:
            return None

        edited_payloads = dialog.payloads or []
        normalized_payloads: list[dict] = []
        for index, payload in enumerate(edited_payloads):
            fallback_plan_id = unique_ids[index] if index < len(unique_ids) else None
            normalized = self._normalize_0301_payload(payload, fallback_plan_id=fallback_plan_id)
            if int(normalized.get("missionPlanID", 0)) <= 0:
                continue
            normalized_payloads.append(normalized)
        return normalized_payloads or None

    def _start_0102_stream(self, _retry: int = 0):
        """초기화 모드 직후 0.5s 뒤 0102를 5Hz로 자동 시작."""
        if not self._0102_push_enabled():
            if not getattr(self, "_heartbeat_0102_skip_logged", False):
                self._heartbeat_0102_skip_logged = True
                self._append_log_line("[0102] auto heartbeat skipped (set KU_MMR_0102_PUSH_ENABLED=1 to enable)")
                self._emit_lifecycle(
                    "heartbeat_skip",
                    component="0102_heartbeat",
                    outcome="skipped",
                    reason="push_disabled",
                )
            return
        if not self._power_on:
            return
        if not getattr(self, "_bus_ready", False):
            if _retry == 0:
                self._append_log_line("[0102] NodeMessenger 초기화 대기 중 ? 자동 송신 보류")
            if _retry < 30:
                QTimer.singleShot(300, lambda r=_retry + 1: self._start_0102_stream(r))
            else:
                self._append_log_line("[WARN] NodeMessenger가 준비되지 않아 0102 자동 송신을 건너뜁니다.")
            return
        try:
            self._tab.periodic_config['0102'] = 5
        except Exception:
            pass
        self._ensure_0102(True)

    # ───────── 최신 0201/0203 및 0204 ID 트래킹 ─────────
    def _install_input_listeners(self):
        """0201/0203 수신 캐시와 0204 중복 발신 방지 상태를 유지한다."""
        if getattr(self, "_input_listener_refs", None):
            for msg_id, handler in self._input_listener_refs:
                try:
                    unregister_listener(msg_id, handler)
                except Exception:
                    pass
            self._input_listener_refs.clear()
        listeners = (
            ("0201", self._on_input_payload_0201),
            ("0203", self._on_input_payload_0203),
            ("0204", self._on_output_plan_0204),
        )
        for msg_id, handler in listeners:
            try:
                register_listener(msg_id, handler)
                self._input_listener_refs.append((msg_id, handler))
                self._emit_lifecycle(
                    "listener_start",
                    component=f"{msg_id}_listener",
                    outcome="ok",
                )
            except Exception:
                self._append_log_line(f"[WARN] Listener registration failed for {msg_id}")
                self._emit_lifecycle(
                    "listener_fail",
                    component=f"{msg_id}_listener",
                    outcome="failure",
                )

    # ───────── 0201/0203 최신 상태 배너 ─────────
    def _build_input_banner_info(self) -> tuple[str, str]:
        """GUI 상단 배너에 보여줄 0201/0203 ID·파일 정보를 만든다."""
        try:
            db_root = db_paths.get_active_db_root()
        except Exception:
            db_root = db_paths.LEGACY_DB_ROOT

        return _build_latest_input_banner_info(
            db_root,
            get_latest_package_id=get_latest_package_id,
            resolve_path_from_cache=resolve_path_from_cache,
        )

    def _refresh_input_banner(self) -> None:
        label = getattr(self, "_latest_input_label", None)
        if label is None:
            return
        try:
            text, tip = self._build_input_banner_info()
        except Exception as exc:
            text = f"0201/0203 상태 표시 실패: {exc}"
            tip = text

        def _apply() -> None:
            lbl = getattr(self, "_latest_input_label", None)
            if lbl is None:
                return
            try:
                lbl.setText(text)
                lbl.setToolTip(tip)
            except Exception:
                pass

        QTimer.singleShot(0, _apply)

    def _load_attack_context(self, cmpk_path: Path) -> Optional[Dict[str, Any]]:
        return load_attack_context(cmpk_path, getattr(self.log_sig, "emit", None))

    def _build_attack_context_from_replan_detail(self, detail: Any) -> Optional[Dict[str, Any]]:
        return build_attack_context_from_replan_detail(detail)

    @staticmethod
    def _to_optional_int(value: Any) -> Optional[int]:
        try:
            if value is None:
                return None
            iv = int(value)
        except Exception:
            return None
        return iv if iv > 0 else None

    @staticmethod
    def _to_optional_float(value: Any) -> Optional[float]:
        try:
            if value is None:
                return None
            return float(value)
        except Exception:
            return None

    def _evaluate_no_available_uav_replan_guard(
        self,
        ctx: Dict[str, Any] | None,
    ) -> Optional[Dict[str, Any]]:
        context = dict(ctx or {})
        try:
            replan_level = int(context.get("replan_level", context.get("replanLevel", 0)))
        except Exception:
            replan_level = 0
        if replan_level <= 1:
            return None

        status_known, available_aircraft_ids, available_uav_ids, status_path = _load_vehicle_status_available_ids()
        if not status_known or available_uav_ids:
            return None

        detail = context.get("replan_detail")
        if not isinstance(detail, dict):
            raw_detail = context.get("replanDetail")
            detail = raw_detail if isinstance(raw_detail, dict) else {}

        return {
            "notice": _NO_AVAILABLE_UAV_NOTICE,
            "replan_level": int(replan_level),
            "reason": str(context.get("reason") or "").strip(),
            "trigger": str(detail.get("trigger") or "").strip() if isinstance(detail, dict) else "",
            "trigger_type": str(detail.get("triggerType") or "").strip() if isinstance(detail, dict) else "",
            "available_aircraft_ids": sorted(int(aid) for aid in available_aircraft_ids),
            "available_uav_ids": sorted(int(aid) for aid in available_uav_ids),
            "status_path": str(status_path) if status_path is not None else "",
        }

    @staticmethod
    def _format_no_available_uav_replan_guard_log(detail: Dict[str, Any] | None) -> str:
        payload = dict(detail or {})
        parts: list[str] = [
            "replan pipeline skipped: no available UAVs",
            f"replanLevel={payload.get('replan_level')}",
        ]
        reason = str(payload.get("reason") or "").strip()
        if reason:
            parts.append(f"reason={reason}")
        trigger = str(payload.get("trigger") or "").strip()
        trigger_type = str(payload.get("trigger_type") or "").strip()
        if trigger or trigger_type:
            parts.append(f"trigger={trigger or '-'}:{trigger_type or '-'}")
        available_aircraft_ids = payload.get("available_aircraft_ids")
        if isinstance(available_aircraft_ids, list):
            parts.append(f"availableAircraft={available_aircraft_ids}")
        status_path = str(payload.get("status_path") or "").strip()
        if status_path:
            parts.append(f"status={status_path}")
        return "[BLOCK] " + ", ".join(parts)

    def _build_follow_up_attack_target_bundle(
        self,
        detail: Any,
        *,
        limit: int = 3,
    ) -> list[dict[str, Any]]:
        if not isinstance(detail, dict):
            return []

        def _target_map_entry_sort_key(entry: dict[str, Any]) -> tuple[int, int, int, float, int]:
            settings = get_target_detection_settings() or {}
            priority_raw = settings.get("target_type_priority") or [1, 2, 3, 4, 5, 6]
            priority_order: list[int] = []
            seen: set[int] = set()
            for item in priority_raw:
                iv = self._to_optional_int(item)
                if iv is None or iv in seen:
                    continue
                seen.add(iv)
                priority_order.append(iv)
            for fallback in (1, 2, 3, 4, 5, 6):
                if fallback not in seen:
                    priority_order.append(fallback)
            priority_rank = {value: idx for idx, value in enumerate(priority_order)}

            target_type = self._to_optional_int(entry.get("targetType"))
            target_id = self._to_optional_int(entry.get("targetID")) or 0
            threat = self._to_optional_float(entry.get("threat")) or 0.0
            is_used = 1 if self._to_optional_int(entry.get("isUsed")) == 1 else 0
            in_frame = 1 if bool(entry.get("targetInFrame")) else 0
            updated = self._to_optional_int(entry.get("lastUpdated")) or self._to_optional_int(entry.get("firstDetected")) or 0
            return (
                priority_rank.get(target_type or 0, len(priority_rank)),
                is_used,
                0 if in_frame else 1,
                -float(threat),
                -int(updated),
            )

        bundle_raw = detail.get("targetBundle")
        normalized: list[dict[str, Any]] = []
        if isinstance(bundle_raw, list) and bundle_raw:
            for item in bundle_raw:
                if not isinstance(item, dict):
                    continue
                target_id = self._to_optional_int(item.get("targetID") or item.get("targetId"))
                if target_id is None:
                    continue
                normalized.append(dict(item))
        else:
            try:
                target_path = db_paths.get_db_subpath("DSS_Internal") / "targetInfo.json"
                raw_data = json.loads(target_path.read_text(encoding="utf-8"))
            except Exception:
                return []
            target_map = raw_data.get("targetList") if isinstance(raw_data, dict) else None
            if not isinstance(target_map, dict):
                return []

            for key, entry in target_map.items():
                if not isinstance(entry, dict):
                    continue
                target_id = self._to_optional_int(entry.get("targetID"))
                if target_id is None:
                    continue
                if bool(entry.get("isDestroyed")):
                    continue
                if self._to_optional_int(entry.get("isIgnored")) not in (None, 0):
                    continue
                coordinate = entry.get("coordinate")
                if not isinstance(coordinate, dict):
                    continue
                lat = self._to_optional_float(coordinate.get("latitude"))
                lon = self._to_optional_float(coordinate.get("longitude"))
                if lat is None or lon is None:
                    continue
                if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                    continue
                watcher_id = self._to_optional_int(entry.get("watcherID"))
                if watcher_id is None:
                    watcher_field = entry.get("watcher")
                    if isinstance(watcher_field, dict):
                        watcher_id = self._to_optional_int(
                            watcher_field.get("aircraftID")
                            or watcher_field.get("watcherID")
                            or watcher_field.get("id")
                        )
                    else:
                        watcher_id = self._to_optional_int(watcher_field)
                normalized.append(
                    {
                        "key": str(key),
                        "targetID": target_id,
                        "watcherID": watcher_id,
                        "targetType": self._to_optional_int(entry.get("targetType")),
                        "coordinate": {
                            "latitude": lat,
                            "longitude": lon,
                            "altitude": self._to_optional_float(coordinate.get("altitude")) or 0.0,
                        },
                        "isDestroyed": bool(entry.get("isDestroyed")),
                        "isUsed": self._to_optional_int(entry.get("isUsed")) or 0,
                        "isIgnored": self._to_optional_int(entry.get("isIgnored")) or 0,
                        "targetInFrame": bool(entry.get("targetInFrame")),
                        "threat": self._to_optional_float(entry.get("threat")) or 0.0,
                        "firstDetected": self._to_optional_int(entry.get("firstDetected")),
                        "lastUpdated": self._to_optional_int(entry.get("lastUpdated")),
                    }
                )

        if not normalized:
            return []

        best_by_target: dict[int, tuple[tuple[int, int, int, float, int], dict[str, Any]]] = {}
        for entry in normalized:
            target_id = self._to_optional_int(entry.get("targetID"))
            if target_id is None:
                continue
            score = _target_map_entry_sort_key(entry)
            current = best_by_target.get(target_id)
            if current is None or score < current[0]:
                best_by_target[target_id] = (score, dict(entry))

        ordered = [item[1] for item in sorted(best_by_target.values(), key=lambda item: item[0])]
        if limit > 0:
            ordered = ordered[: int(limit)]

        watcher_cfg = get_target_detection_settings() or {}
        watcher_pool_raw = watcher_cfg.get("watcher_uav_ids") or [4, 5, 6]
        watcher_pool: list[int] = []
        for item in watcher_pool_raw:
            watcher_id = self._to_optional_int(item)
            if watcher_id is None or watcher_id in watcher_pool:
                continue
            watcher_pool.append(watcher_id)
        if not watcher_pool:
            watcher_pool = [4, 5, 6]

        used_watchers: set[int] = set()
        bundle: list[dict[str, Any]] = []
        for index, entry in enumerate(ordered):
            target_id = self._to_optional_int(entry.get("targetID"))
            if target_id is None:
                continue
            source_watcher_id = self._to_optional_int(entry.get("watcherID"))
            assigned_watcher_id = source_watcher_id
            if assigned_watcher_id is None or assigned_watcher_id in used_watchers:
                assigned_watcher_id = next((wid for wid in watcher_pool if wid not in used_watchers), assigned_watcher_id)
            if assigned_watcher_id is not None:
                used_watchers.add(int(assigned_watcher_id))
            item = dict(entry)
            item["targetID"] = int(target_id)
            item["sourceWatcherID"] = source_watcher_id
            item["watcherID"] = assigned_watcher_id
            item["bundleIndex"] = int(index)
            bundle.append(item)
        return bundle[: int(limit)] if limit > 0 else bundle

    def _get_current_attack_source_plan_id(self) -> Optional[int]:
        last_plan_id = self._to_optional_int(getattr(self, "_last_mission_plan_id", None))
        if last_plan_id is None:
            return None
        active_ctx = getattr(self, "_active_plan_context", {})
        if not isinstance(active_ctx, dict):
            return None
        option_meta = active_ctx.get("_option_meta")
        if not isinstance(option_meta, dict):
            return None
        meta = option_meta.get(int(last_plan_id))
        if not isinstance(meta, dict):
            return None
        if not bool(meta.get("attack")):
            post_ctx = meta.get("postAttackRejoinContext")
            if not isinstance(post_ctx, dict):
                return None
            for key in ("source_plan_id", "sourcePlanID", "sourceMissionPlanID", "currentMissionPlanID"):
                source_plan_id = self._to_optional_int(post_ctx.get(key))
                if source_plan_id is not None and source_plan_id > 0:
                    return int(source_plan_id)
            return None
        return int(last_plan_id)

    @staticmethod
    def _is_post_attack_rejoin_detail(detail: Any) -> bool:
        return is_post_attack_rejoin_detail(detail)

    @staticmethod
    def _is_prior_post_rejoin_detail(detail: Any) -> bool:
        return is_prior_post_rejoin_detail(detail)

    def _prepare_follow_up_attack_detail(self, detail: Dict[str, Any]) -> bool:
        if not isinstance(detail, dict):
            return False
        trigger = str(detail.get("trigger") or "").strip()
        if trigger != "0402":
            return False

        previous_attack_plan_id = self._get_current_attack_source_plan_id()
        if previous_attack_plan_id is None:
            return False
        current_plan_id = (
            self._to_optional_int(detail.get("currentMissionPlanID"))
            or self._to_optional_int(getattr(self, "_last_mission_plan_id", None))
            or self._to_optional_int(detail.get("sourceMissionPlanID"))
            or previous_attack_plan_id
        )

        bundle = self._build_follow_up_attack_target_bundle(detail, limit=3)
        if not bundle:
            return False

        detail["targetBundle"] = bundle
        detail["attackTargetList"] = list(bundle)
        detail["targets"] = list(bundle)
        detail["targetBundleMode"] = "follow_up"
        detail["followUpAttackMode"] = True
        # Keep the previous attack plan only as lineage.  Reusing it as the
        # executable source revives its old LINE remaining-domain snapshot
        # after a post-attack rejoin has already advanced the mission.
        detail["previousAttackMissionPlanID"] = int(previous_attack_plan_id)
        detail["sourceMissionPlanID"] = int(current_plan_id)
        detail["currentMissionPlanID"] = int(current_plan_id)
        return True

    def _load_uav_params_from_store(self) -> Optional[Dict[str, Any]]:
        payload = load_runtime_settings()
        if not isinstance(payload, dict) or not payload:
            return None
        return payload

    def _apply_uav_params_from_store(
        self,
        d0303_module,
        *,
        d0304_module=None,
        mp_config_module=None,
        search_speed_module=None,
        default_cruise: float = 40.0,
        default_turn_step: float = 15.0,
    ) -> tuple[float, float, bool]:
        payload = self._load_uav_params_from_store()
        if not payload:
            return default_cruise, default_turn_step, False

        values = payload.get("values")
        if not isinstance(values, dict):
            values = {}

        def _get_float(key: str, default: float) -> float:
            if key not in values:
                return default
            try:
                return float(values.get(key))
            except Exception:
                return default

        def _get_int(key: str, default: int) -> int:
            if key not in values:
                return default
            try:
                return int(float(values.get(key)))
            except Exception:
                return default

        cruise_speed = _get_float("cruise_speed_mps", default_cruise)
        turn_step = _get_float("turn_step_deg", default_turn_step)
        sweep_sep = _get_float(
            "default_sweep_separation_m",
            float(getattr(mp_config_module, "DEFAULT_SWEEP_SEPARATION_M", d0303_module.SWEEP_GEOMETRY.separation_m)),
        )
        search_weight = _get_float(
            "search_speed_weight",
            float(getattr(mp_config_module, "SEARCH_SPEED_WEIGHT", 1.0)),
        )
        area_search_weight = _get_float(
            "area_search_speed_weight",
            float(getattr(d0303_module, "AREA_SEARCH_SPEED_WEIGHT", 1.2)),
        )
        db_fov_weight = _get_float(
            "db_fov_weight",
            float(getattr(mp_config_module, "DB_FOV_WEIGHT", 1.0)),
        )
        if db_fov_weight <= 0.0:
            db_fov_weight = 1.0
        line_fov_deg_raw = _get_float(
            "line_custom_fov_deg",
            float(getattr(d0303_module, "FOV_DEG", 0.0)),
        )
        area_custom_fov_deg_raw = _get_float(
            "area_custom_fov_deg",
            float(line_fov_deg_raw),
        )
        area_output_fov_scale = _get_float(
            "area_output_fov_scale",
            float(getattr(d0303_module, "AREA_OUTPUT_FOV_SCALE", 3.0)),
        )
        line_density_scale = _get_float(
            "line_density_scale",
            float(getattr(d0303_module, "LINE_SWEEP_DENSITY_SCALE", 1.18)),
        )
        fov_db_sep_safety_factor = _get_float(
            "fov_db_sep_safety_factor",
            float(getattr(d0303_module, "FOV_DB_SEP_SAFETY_FACTOR", 1.7)),
        )
        if fov_db_sep_safety_factor <= 0.0:
            fov_db_sep_safety_factor = 1.7
        area_density_scale = _get_float(
            "area_density_scale",
            float(getattr(d0303_module, "AREA_SWEEP_DENSITY_SCALE", 1.5)),
        )
        line_route_offset_scale = _get_float(
            "line_route_offset_scale",
            float(getattr(d0303_module, "LINE_ROUTE_OFFSET_SCALE", 1.0)),
        )
        area_route_offset_scale = _get_float(
            "area_route_offset_scale",
            float(getattr(d0303_module, "AREA_ROUTE_OFFSET_SCALE", 0.5)),
        )
        uav_wp_interval_m = _get_float(
            "uav_wp_interval_m",
            float(getattr(d0303_module, "SWEEP_ROUTE_WP_SPACING_M", 2000.0)),
        )
        area_wp_interval_m = _get_float(
            "area_wp_interval_m",
            float(getattr(d0303_module, "AREA_SWEEP_ROUTE_WP_SPACING_M", 1000.0)),
        )
        lah_wp_interval_m = _get_float(
            "lah_wp_interval_m",
            float(getattr(d0304_module, "WP_INTERVAL_M", 3000.0)) if d0304_module is not None else 3000.0,
        )
        dubins_turn_radius_m = _get_float(
            "dubins_turn_radius_m",
            float(getattr(d0303_module, "DUBINS_TURN_RADIUS_M", 450.0)),
        )
        altitude = _get_int("altitude_m", int(getattr(d0303_module, "Altitude", 1000)))
        altitude_layer_step_m = _get_float("altitude_layer_step_m", 10.0)
        if altitude_layer_step_m < 0.0:
            altitude_layer_step_m = 10.0
        altitude_layers_m = tuple(float(altitude) + (float(altitude_layer_step_m) * idx) for idx in range(3))
        line_fov_deg = float(
            apply_runtime_camera_adjusted_fov_deg(
                line_fov_deg_raw,
                payload,
                context="MISSION_PLAN LINE",
            )
        )
        area_custom_fov_deg = float(
            apply_runtime_camera_adjusted_fov_deg(
                area_custom_fov_deg_raw,
                payload,
                context="MISSION_PLAN AREA",
            )
        )
        d0303_module.FOV_DEG = line_fov_deg
        d0303_module.AREA_CUSTOM_FOV_DEG = area_custom_fov_deg
        d0303_module.AREA_OUTPUT_FOV_SCALE = area_output_fov_scale
        d0303_module.LINE_SWEEP_DENSITY_SCALE = line_density_scale
        d0303_module.FOV_DB_SEP_SAFETY_FACTOR = fov_db_sep_safety_factor
        d0303_module.AREA_SWEEP_DENSITY_SCALE = area_density_scale
        d0303_module.LINE_ROUTE_OFFSET_SCALE = line_route_offset_scale
        d0303_module.AREA_ROUTE_OFFSET_SCALE = area_route_offset_scale
        d0303_module.LINE_SEARCH_SPEED_WEIGHT = search_weight
        d0303_module.AREA_SEARCH_SPEED_WEIGHT = area_search_weight
        d0303_module.AREA_FIRST_PACKET_SEARCH_SPEED_SCALE = _get_float(
            "area_first_packet_search_speed_scale",
            float(getattr(d0303_module, "AREA_FIRST_PACKET_SEARCH_SPEED_SCALE", 1.2)),
        )
        d0303_module.AREA_FIRST_PACKET_SWEEP_GROUP_SCALE = _get_float(
            "area_first_packet_sweep_group_scale",
            float(getattr(d0303_module, "AREA_FIRST_PACKET_SWEEP_GROUP_SCALE", 1.0)),
        )
        d0303_module.SWEEP_ROUTE_WP_SPACING_M = uav_wp_interval_m
        d0303_module.AREA_SWEEP_ROUTE_WP_SPACING_M = area_wp_interval_m
        d0303_module.DUBINS_TURN_RADIUS_M = dubins_turn_radius_m
        d0303_module.DB_FOV_WEIGHT = float(db_fov_weight)
        d0303_module.Altitude = int(round(altitude))
        d0303_module.ALTITUDE_LAYERS_M = altitude_layers_m
        d0303_module.SWEEP_MERGE_HEADING_DEG = _get_float(
            "sweep_merge_heading_deg", float(getattr(d0303_module, "SWEEP_MERGE_HEADING_DEG", 5.0))
        )
        d0303_module.SWEEP_LINE_INTERP_POINTS = _get_int(
            "sweep_line_interp_points", int(getattr(d0303_module, "SWEEP_LINE_INTERP_POINTS", 2))
        )
        d0303_module.MAX_LINESEARCH_COORDS_PER_WAYPOINT = _get_int(
            "max_linesearch_coords_per_waypoint",
            int(getattr(d0303_module, "MAX_LINESEARCH_COORDS_PER_WAYPOINT", 2000)),
        )
        d0303_module.LINESEARCH_INNER_PARALLEL_MIN_STRIPS = _get_int(
            "linesearch_inner_parallel_min_strips",
            int(getattr(d0303_module, "LINESEARCH_INNER_PARALLEL_MIN_STRIPS", 256)),
        )
        d0303_module.LINESEARCH_INNER_PARALLEL_MIN_COORDS = _get_int(
            "linesearch_inner_parallel_min_coords",
            int(getattr(d0303_module, "LINESEARCH_INNER_PARALLEL_MIN_COORDS", 512)),
        )
        d0303_module.LINESEARCH_INNER_PARALLEL_WORKERS = _get_int(
            "linesearch_inner_parallel_workers",
            int(getattr(d0303_module, "LINESEARCH_INNER_PARALLEL_WORKERS", 2)),
        )
        d0303_module.FORMATION_FOLLOWER_POSTPROCESS_PARALLEL_MIN_FOLLOWERS = _get_int(
            "formation_follower_postprocess_parallel_min_followers",
            int(getattr(d0303_module, "FORMATION_FOLLOWER_POSTPROCESS_PARALLEL_MIN_FOLLOWERS", 2)),
        )
        d0303_module.FORMATION_FOLLOWER_POSTPROCESS_WORKERS = _get_int(
            "formation_follower_postprocess_workers",
            int(getattr(d0303_module, "FORMATION_FOLLOWER_POSTPROCESS_WORKERS", 2)),
        )
        d0303_module.MIN_SWEEP_LEN_M = _get_float(
            "min_sweep_len_m", float(getattr(d0303_module, "MIN_SWEEP_LEN_M", 3.0))
        )
        d0303_module.MIN_ROUTE_SPACING_M = _get_float(
            "min_route_spacing_m", float(getattr(d0303_module, "MIN_ROUTE_SPACING_M", 200.0))
        )
        d0303_module.AREA_DUBINS_ENTRY_LINKS_ENABLED = bool(
            values.get(
                "area_dubins_entry_links_enabled",
                bool(getattr(d0303_module, "AREA_DUBINS_ENTRY_LINKS_ENABLED", True)),
            )
        )
        d0303_module.DEFAULT_SEARCH_SPEED_MULTIPLIER = _get_float(
            "default_search_speed_multiplier",
            float(getattr(d0303_module, "DEFAULT_SEARCH_SPEED_MULTIPLIER", 16.0)),
        )
        d0303_module.POINT_FOV_DEG = apply_runtime_camera_adjusted_fov_deg(
            _get_float("point_fov_deg", float(getattr(d0303_module, "POINT_FOV_DEG", 31.2))),
            payload,
            context="MISSION_PLAN POINT",
        )
        d0303_module.AREA_NADIR_FOV_DEG = apply_runtime_camera_adjusted_fov_deg(
            _get_float("area_nadir_fov_deg", float(getattr(d0303_module, "AREA_NADIR_FOV_DEG", 31.2))),
            payload,
            context="MISSION_PLAN AREA_NADIR",
        )
        d0303_module.ENTRY_HOLD_FOV_DEG = apply_runtime_camera_adjusted_fov_deg(
            _get_float("entry_hold_fov_deg", float(getattr(d0303_module, "ENTRY_HOLD_FOV_DEG", 10.0))),
            payload,
            context="MISSION_PLAN ENTRY_HOLD",
        )
        d0303_module.ENTRY_HOLD_GIMBAL_PITCH = _get_float(
            "entry_hold_gimbal_pitch",
            float(getattr(d0303_module, "ENTRY_HOLD_GIMBAL_PITCH", -90.0)),
        )
        d0303_module.ENTRY_HOLD_GIMBAL_YAW = _get_float(
            "entry_hold_gimbal_yaw",
            float(getattr(d0303_module, "ENTRY_HOLD_GIMBAL_YAW", 0.0)),
        )
        d0303_module.LOITER_RADIUS_M = _get_float(
            "loiter_radius_m", float(getattr(d0303_module, "LOITER_RADIUS_M", 800.0))
        )
        d0303_module.LOITER_DIRECTION = _get_int(
            "loiter_direction", int(getattr(d0303_module, "LOITER_DIRECTION", 1))
        )
        d0303_module.LOITER_TIME_S = _get_float(
            "loiter_time_s", float(getattr(d0303_module, "LOITER_TIME_S", 30.0))
        )
        d0303_module.LOITER_SPEED_MPS = _get_float(
            "loiter_speed_mps", float(getattr(d0303_module, "LOITER_SPEED_MPS", 30.0))
        )
        d0303_module.SWEEP_GEOMETRY = d0303_module.SweepConfig(
            separation_m=sweep_sep,
            fov_deg=line_fov_deg,
        )
        if d0304_module is not None:
            d0304_module.WP_INTERVAL_M = float(lah_wp_interval_m)
            d0304_module.ALTITUDE_LAYERS_M = altitude_layers_m

        # Keep one authoritative settings file by backfilling missing keys
        # into modules/resource/uav_params.json.
        resolved_values = {
            "camera_adjust_enabled": bool(values.get("camera_adjust_enabled", False)),
            "camera_adjust_percent": _get_float("camera_adjust_percent", 10.0),
            "manual_fov_global_sync_enabled": bool(values.get("manual_fov_global_sync_enabled", True)),
            "global_manual_fov_deg": _get_float("global_manual_fov_deg", float(line_fov_deg_raw)),
            "global_density_scale": _get_float("global_density_scale", float(line_density_scale)),
            "global_search_speed_weight": _get_float("global_search_speed_weight", float(search_weight)),
            "global_route_offset_scale": _get_float("global_route_offset_scale", float(line_route_offset_scale)),
            "line_override_enabled": bool(values.get("line_override_enabled", False)),
            "area_override_enabled": bool(values.get("area_override_enabled", False)),
            "area_nadir_override_enabled": bool(values.get("area_nadir_override_enabled", False)),
            "recon_override_enabled": bool(values.get("recon_override_enabled", False)),
            "line_override_fov_deg": _get_float("line_override_fov_deg", float(line_fov_deg_raw)),
            "line_override_density_scale": _get_float("line_override_density_scale", float(line_density_scale)),
            "line_override_search_speed_weight": _get_float("line_override_search_speed_weight", float(search_weight)),
            "line_override_route_offset_scale": _get_float("line_override_route_offset_scale", float(line_route_offset_scale)),
            "area_override_fov_deg": _get_float("area_override_fov_deg", float(area_custom_fov_deg_raw)),
            "area_override_density_scale": _get_float("area_override_density_scale", float(area_density_scale)),
            "area_override_search_speed_weight": _get_float("area_override_search_speed_weight", float(area_search_weight)),
            "area_override_route_offset_scale": _get_float("area_override_route_offset_scale", float(area_route_offset_scale)),
            "area_nadir_override_fov_deg": _get_float(
                "area_nadir_override_fov_deg",
                _get_float("area_nadir_fov_deg", float(getattr(d0303_module, "AREA_NADIR_FOV_DEG", 31.2))),
            ),
            "search_speed_weight": float(search_weight),
            "area_search_speed_weight": float(area_search_weight),
            "cruise_speed_mps": float(cruise_speed),
            "turn_step_deg": float(turn_step),
            "altitude_m": int(round(altitude)),
            "altitude_layer_step_m": float(altitude_layer_step_m),
            "line_custom_fov_deg": float(line_fov_deg_raw),
            "area_custom_fov_deg": float(area_custom_fov_deg_raw),
            "area_nadir_fov_deg": _get_float("area_nadir_fov_deg", float(getattr(d0303_module, "AREA_NADIR_FOV_DEG", 31.2))),
            "point_fov_deg": _get_float("point_fov_deg", float(getattr(d0303_module, "POINT_FOV_DEG", 31.2))),
            "entry_hold_fov_deg": _get_float("entry_hold_fov_deg", float(getattr(d0303_module, "ENTRY_HOLD_FOV_DEG", 10.0))),
            "area_output_fov_scale": float(area_output_fov_scale),
            "default_sweep_separation_m": float(sweep_sep),
            "fov_db_sep_safety_factor": float(fov_db_sep_safety_factor),
            "db_fov_weight": float(db_fov_weight),
            "fov_db_smaller_fov_steps": _get_int("fov_db_smaller_fov_steps", 4),
            "area_fov_db_smaller_fov_steps": _get_int("area_fov_db_smaller_fov_steps", 3),
            "line_density_scale": float(line_density_scale),
            "area_density_scale": float(area_density_scale),
            "line_route_offset_scale": float(line_route_offset_scale),
            "area_route_offset_scale": float(area_route_offset_scale),
            "area_first_packet_search_speed_scale": _get_float("area_first_packet_search_speed_scale", 1.2),
            "area_first_packet_sweep_group_scale": _get_float("area_first_packet_sweep_group_scale", 1.0),
            "next_collab_area_density_scale": _get_float("next_collab_area_density_scale", 2.4),
            "uav_wp_interval_m": float(uav_wp_interval_m),
            "area_wp_interval_m": float(area_wp_interval_m),
            "lah_wp_interval_m": float(lah_wp_interval_m),
            "dubins_turn_radius_m": float(dubins_turn_radius_m),
            "default_search_speed_multiplier": _get_float(
                "default_search_speed_multiplier",
                float(getattr(d0303_module, "DEFAULT_SEARCH_SPEED_MULTIPLIER", 16.0)),
            ),
            "sweep_merge_heading_deg": _get_float(
                "sweep_merge_heading_deg",
                float(getattr(d0303_module, "SWEEP_MERGE_HEADING_DEG", 5.0)),
            ),
            "sweep_line_interp_points": int(d0303_module.SWEEP_LINE_INTERP_POINTS),
            "min_sweep_len_m": _get_float("min_sweep_len_m", float(getattr(d0303_module, "MIN_SWEEP_LEN_M", 3.0))),
            "min_route_spacing_m": _get_float(
                "min_route_spacing_m",
                float(getattr(d0303_module, "MIN_ROUTE_SPACING_M", 200.0)),
            ),
            "enhanced_area_review_max_segment_m": _get_float("enhanced_area_review_max_segment_m", 1500.0),
            "recon_area_review_max_split_count": _get_int("recon_area_review_max_split_count", 0),
            "recon_area_review_min_segment_m": _get_float("recon_area_review_min_segment_m", 0.0),
            "enhanced_auto_fov_from_db": bool(values.get("enhanced_auto_fov_from_db", True)),
            "area_dubins_entry_links_enabled": bool(
                values.get(
                    "area_dubins_entry_links_enabled",
                    bool(getattr(d0303_module, "AREA_DUBINS_ENTRY_LINKS_ENABLED", True)),
                )
            ),
            "recon_override_split_width_m": _get_float("recon_override_split_width_m", 600.0),
            "recon_override_fixed_fov_deg": _get_float("recon_override_fixed_fov_deg", 15.0),
            "recon_override_sweep_separation_scale": _get_float("recon_override_sweep_separation_scale", 0.50),
            "recon_area_split_width_m": _get_float("recon_area_split_width_m", 600.0),
            "recon_area_fixed_fov_deg": _get_float("recon_area_fixed_fov_deg", 15.0),
            "recon_sweep_separation_scale": _get_float("recon_sweep_separation_scale", 0.50),
            "entry_hold_gimbal_pitch": _get_float(
                "entry_hold_gimbal_pitch",
                float(getattr(d0303_module, "ENTRY_HOLD_GIMBAL_PITCH", -90.0)),
            ),
            "entry_hold_gimbal_yaw": _get_float(
                "entry_hold_gimbal_yaw",
                float(getattr(d0303_module, "ENTRY_HOLD_GIMBAL_YAW", 0.0)),
            ),
            "loiter_radius_m": _get_float(
                "loiter_radius_m",
                float(getattr(d0303_module, "LOITER_RADIUS_M", 800.0)),
            ),
            "loiter_direction": _get_int(
                "loiter_direction",
                int(getattr(d0303_module, "LOITER_DIRECTION", 1)),
            ),
            "loiter_time_s": _get_float(
                "loiter_time_s",
                float(getattr(d0303_module, "LOITER_TIME_S", 30.0)),
            ),
            "loiter_speed_mps": _get_float(
                "loiter_speed_mps",
                float(getattr(d0303_module, "LOITER_SPEED_MPS", 30.0)),
            ),
            "next_collab_default_entry_strategy": str(values.get("next_collab_default_entry_strategy", "turn_projection")),
            "next_collab_sweep_step_ratio": _get_float("next_collab_sweep_step_ratio", 0.60),
            "next_collab_entry_tprime_target_sep_ratio": _get_float(
                "next_collab_entry_tprime_target_sep_ratio",
                0.30,
            ),
            "next_collab_entry_tprime_ratio_scale": _get_float("next_collab_entry_tprime_ratio_scale", 0.50),
            "next_collab_area_path0_trigger_sep_m": _get_float("next_collab_area_path0_trigger_sep_m", 1500.0),
            "next_collab_area_path0_target_sep_ratio": _get_float("next_collab_area_path0_target_sep_ratio", 0.20),
            "next_collab_turn_radius_scale": _get_float("next_collab_turn_radius_scale", 1.20),
            "next_collab_takeover_first_step_ratio": _get_float("next_collab_takeover_first_step_ratio", 0.40),
            "next_collab_area_fov_scale": _get_float("next_collab_area_fov_scale", 1.00),
            "next_collab_area_search_speed_scale": _get_float("next_collab_area_search_speed_scale", 1.30),
            "next_collab_area_gsd_margin_ratio": _get_float("next_collab_area_gsd_margin_ratio", 0.90),
            "next_collab_line_db_width_weight": _get_float("next_collab_line_db_width_weight", 0.30),
            "next_collab_line_db_sep_weight": _get_float("next_collab_line_db_sep_weight", 0.25),
            "next_collab_line_db_fov_weight": _get_float("next_collab_line_db_fov_weight", 0.45),
            "next_collab_sweep_points_per_leg": _get_int("next_collab_sweep_points_per_leg", 3),
            "next_collab_first_line_fov_scale": _get_float("next_collab_first_line_fov_scale", 1.35),
            "next_collab_first_line_fov_max_deg": _get_float("next_collab_first_line_fov_max_deg", 15.4),
            "lah_path_mode": str(values.get("lah_path_mode", "linear")),
            "lah_rl_hex_step": _get_int("lah_rl_hex_step", 50),
            "lah_rl_area_km": _get_float("lah_rl_area_km", 10.0),
        }
        for key in PERSISTED_RUNTIME_VALUE_KEYS:
            if key not in resolved_values and key in values:
                resolved_values[key] = copy.deepcopy(values.get(key))
        manual_fov_rollback = values.get(MANUAL_FOV_ROLLBACK_KEY)
        if isinstance(manual_fov_rollback, dict):
            resolved_values[MANUAL_FOV_ROLLBACK_KEY] = copy.deepcopy(manual_fov_rollback)
        flyover = payload.get("flyover")
        if not isinstance(flyover, dict):
            flyover = {}
        normalized_input = copy.deepcopy(payload) if isinstance(payload, dict) else {}
        normalized_input["values"] = resolved_values
        normalized_input["flyover"] = flyover
        normalized_payload = canonicalize_runtime_payload(normalized_input)
        try:
            path = runtime_settings_path()
            path.write_text(json.dumps(normalized_payload, ensure_ascii=True, indent=2), encoding="utf-8")
        except Exception:
            pass

        if mp_config_module is not None:
            mp_config_module.DEFAULT_SWEEP_SEPARATION_M = float(sweep_sep)
            mp_config_module.SEARCH_SPEED_WEIGHT = float(search_weight)
            mp_config_module.DB_FOV_WEIGHT = float(db_fov_weight)
        if search_speed_module is not None:
            search_speed_module._CFG_WEIGHT = float(search_weight)

        flyover = payload.get("flyover")
        if isinstance(flyover, dict):
            d0303_module.set_flyover_options(
                entry_offset=bool(flyover.get("entry_offset", False)),
                dubins_prefix=bool(flyover.get("dubins_prefix", False)),
                last_point=bool(flyover.get("last_point", False)),
                all_wps=bool(flyover.get("all_wps", False)),
            )

        return cruise_speed, turn_step, True

    def _open_lah_rl_planner(self) -> None:
        """LAH Hex 경로계획 별도 창 열기."""
        try:
            from modules.mission_planning.manual.lah_rl_planner_gui import LAHPlannerWindow
        except ImportError:
            try:
                from lah_rl_planner_gui import LAHPlannerWindow
            except ImportError as exc:
                QMessageBox.critical(self, "모듈 로드 실패", f"lah_rl_planner_gui를 불러올 수 없습니다.\n{exc}")
                return
        if not hasattr(self, "_lah_rl_win") or self._lah_rl_win is None:
            self._lah_rl_win = LAHPlannerWindow(self)
        self._lah_rl_win.show()
        self._lah_rl_win.raise_()

    def _on_algo_settings_applied(self, payload: Optional[Dict[str, Any]] = None) -> None:
        try:
            _ensure_mission_planner_import_paths()
            from data_def import d0303, d0304, search_speed
            try:
                import config as mp_config
            except Exception:
                mp_config = None
            cruise, turn_step, loaded = self._apply_uav_params_from_store(
                d0303,
                d0304_module=d0304,
                mp_config_module=mp_config,
                search_speed_module=search_speed,
            )
            if loaded:
                self.log_sig.emit(
                    "[CONFIG] 알고리즘 설정 적용 "
                    f"(cruise={cruise:.1f}m/s, turn_step={turn_step:.1f}°, "
                    f"areaMode={DEFAULT_AREA_SWEEP_MODE}, "
                    f"areaSplit={DEFAULT_AREA_SPLIT_MODE}, "
                    f"uavPlan={DEFAULT_UAV_PLAN_MODE}, "
                    f"autoFovDb={bool((payload or {}).get('values', {}).get('enhanced_auto_fov_from_db', True))})"
                )
        except Exception as exc:
            self.log_sig.emit(f"[WARN] 알고리즘 설정 즉시 적용 실패: {exc}")
        self._invalidate_planner_runtime(warm_reason="algo_settings")

    def _compute_attack_waypoint(
        self, friendly: Dict[str, Any], target: Dict[str, Any], variant_no: int
    ) -> Dict[str, float]:
        return compute_attack_waypoint(
            PROJECT_ROOT, friendly, target, variant_no, getattr(self.log_sig, "emit", None)
        )

    def _apply_attack_customizations(
        self,
        missions: List[Dict[str, Any]],
        flight_plans_0304: List[Dict[str, Any]],
        attack_context: Dict[str, Any],
        variant_no: int,
        *,
        replan_detail: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        return apply_attack_customizations(
            missions,
            flight_plans_0304,
            attack_context,
            variant_no,
            replan_detail=replan_detail,
            project_root=PROJECT_ROOT,
            log_cb=getattr(self.log_sig, "emit", None),
        )

    def _on_input_payload_0201(self, msg_id, payload):
        self._handle_latest_input_payload(msg_id, payload)

    def _on_input_payload_0203(self, msg_id, payload):
        self._handle_latest_input_payload(msg_id, payload)

    def _parse_message_payload_dict(self, payload: Any) -> dict[str, Any]:
        if isinstance(payload, dict):
            return dict(payload)
        parsed = parse_replan_payload(payload)
        return dict(parsed) if isinstance(parsed, dict) else {}

    @staticmethod
    def _payload_input_package_id(payload: dict[str, Any] | None) -> Optional[int]:
        if not isinstance(payload, dict):
            return None
        for key in ("inputMissionPackageID", "InputMissionPackageID", "inputMissionPackageId"):
            value = _safe_int_value(payload.get(key))
            if value is not None and int(value) > 0:
                return int(value)
        return None

    def _on_output_plan_0204(self, msg_id, payload):
        body = self._parse_message_payload_dict(payload)
        package_id = self._payload_input_package_id(body)
        if package_id is None:
            self.log_sig.emit("[WARN] 0204 received without inputMissionPackageID")
            return
        try:
            self._review_0204_sent_package_ids.add(int(package_id))
        except Exception:
            self._review_0204_sent_package_ids = {int(package_id)}
        self.log_sig.emit(
            f"[0204] received inputMissionPackageID={int(package_id)} "
            "(duplicate-send guard only)"
        )

    def _create_empty_scope(self) -> Dict[str, Set[int]]:
        return {
            "packages": set(),
            "plans": set(),
            "individual_packages": set(),
            "paths": set(),
        }

    def _reset_session_scope(self) -> None:
        self._session_scope = self._create_empty_scope()

    def _submit_id_tab_update(
        self,
        *,
        scope: Optional[Dict[str, Set[int]]] = None,
        cmpk_id: Optional[int] = None,
        mrpk_id: Optional[int] = None,
        plan_state: Optional[str] = None,
        defer_until_post_delivery: bool = False,
    ) -> None:
        scope_payload = None
        if scope is not None:
            scope_payload = {
                "packages": set(scope.get("packages", set())),
                "plans": set(scope.get("plans", set())),
                "individual_packages": set(scope.get("individual_packages", set())),
                "paths": set(scope.get("paths", set())),
            }
        payload = {
            "scope": scope_payload,
            "cmpk_id": cmpk_id,
            "mrpk_id": mrpk_id,
            "plan_state": plan_state,
        }
        if defer_until_post_delivery:
            self._deferred_id_tab_update_payload = payload
            self._record_replan_timing_event("id_tab_update_deferred")
            return
        self.id_tab_update_sig.emit(payload)

    def _flush_deferred_id_tab_update(self) -> None:
        payload = getattr(self, "_deferred_id_tab_update_payload", None)
        if not isinstance(payload, dict):
            return
        self._deferred_id_tab_update_payload = None
        self._record_replan_timing_event("id_tab_update_flushed")
        self.id_tab_update_sig.emit(payload)

    def _apply_id_tab_update(self, payload: object) -> None:
        tab = getattr(self, "_id_tab", None)
        if tab is None:
            return
        data = payload if isinstance(payload, dict) else {}
        scope_payload = data.get("scope")
        cmpk_id = data.get("cmpk_id")
        mrpk_id = data.get("mrpk_id")
        plan_state = data.get("plan_state")
        should_refresh = bool(
            isinstance(scope_payload, dict)
            and any(scope_payload.get(key) for key in ("plans", "individual_packages", "paths"))
        )
        should_reveal = bool(plan_state and "완료" in str(plan_state))
        if should_refresh:
            tab.refresh()
        if isinstance(scope_payload, dict):
            tab.update_session_scope(scope_payload)
        tab.update_input_status(
            cmpk_id=cmpk_id,
            mrpk_id=mrpk_id,
            plan_state=plan_state,
        )
        if should_reveal:
            tabs = getattr(self, "_tabs", None)
            if tabs is not None:
                try:
                    tabs.setCurrentWidget(tab)
                except Exception:
                    idx = getattr(self, "_id_tab_index", -1)
                    if isinstance(idx, int) and idx >= 0:
                        try:
                            tabs.setCurrentIndex(idx)
                        except Exception:
                            pass

    def _set_plan_status(self, status: str) -> None:
        self._plan_status = status
        self._submit_id_tab_update(plan_state=status)

    def _handle_latest_input_payload(self, msg_id: str, payload):
        try:
            prev = self._last_logged_input_ids.get(msg_id)
        except Exception:
            prev = None
        cache_update_from_payload(msg_id, payload)
        self._refresh_input_banner()
        current = get_latest_package_id(msg_id)
        if current is None or current == prev:
            self._submit_id_tab_update(plan_state=self._plan_status)
            return
        self._last_logged_input_ids[msg_id] = current
        src = _extract_input_payload_source(payload if isinstance(payload, dict) else None)
        note = f"[INFO] Latest {msg_id} ID updated → {current}"
        if src:
            note += f" (source={src})"
        self.log_sig.emit(note)
        self._schedule_planner_warmup(f"{msg_id}_updated")

        cmpk_update = current if msg_id == "0201" else None
        mrpk_update = current if msg_id == "0203" else None
        scope_update: Optional[Dict[str, Set[int]]] = None
        if msg_id == "0201":
            if current is not None:
                self._reset_session_scope()
                self._session_scope["packages"].add(int(current))
            self._plan_status = "임무계획 전"
            scope_update = self._session_scope
        self._submit_id_tab_update(
            scope=scope_update,
            cmpk_id=cmpk_update,
            mrpk_id=mrpk_update,
            plan_state=self._plan_status,
        )

    # ───────── Power OFF 가드(발신/수신/카운트/우회 클릭 차단) ─────────
    def _install_power_gate_hooks(self):
        try:
            tab = self._tab
            tbl = getattr(tab, "tbl_tx", None)

            # TX만 차단
            if tbl is not None:
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
                self._pg_filter = _PG(self)
                tbl.installEventFilter(self._pg_filter)

            # TX 버튼 우회만 차단
            if hasattr(tab, "_on_tx_button_clicked"):
                self._orig_tx_click = tab._on_tx_button_clicked
                def _wrapped_tx_click(row):
                    if not self._power_on:
                        self._append_log_line("[BLOCK] Power OFF → TX 버튼 무시")
                        return
                    try:
                        it = getattr(tab, "tbl_tx", None).item(row, 0) if getattr(tab, "tbl_tx", None) else None
                    except Exception: pass
                    return self._orig_tx_click(row)
                tab._on_tx_button_clicked = _wrapped_tx_click

        except Exception:
            pass

    def _apply_power_state(self):
        on = bool(self._power_on)
        previous = getattr(self, "_last_lifecycle_power_on", None)
        if previous is None or bool(previous) != on:
            self._last_lifecycle_power_on = on
            self._emit_lifecycle(
                "power_on" if on else "power_off",
                component="power",
                outcome="ok",
            )
        try:
            self._update_tx_table_enabled(on)
            self._update_rx_table_enabled(True)  # ? RX는 항상 보이게
            if not on:
                self._stop_all_periodic()
        except Exception:
            pass

    def _update_tx_table_enabled(self, enabled: bool):
        """TX 테이블 및 전송 버튼 활성/비활성."""
        try:
            tab = self._tab
            tbl = getattr(tab, "tbl_tx", None)
            if tbl is None:
                return
            tbl.setEnabled(enabled)
            for r in range(tbl.rowCount()):
                w = tbl.cellWidget(r, 3)  # 전송 버튼 컬럼
                if w is not None and hasattr(w, "setEnabled"):
                    w.setEnabled(enabled)
        except Exception:
            pass

    def _update_rx_table_enabled(self, enabled: bool):
        """RX 테이블 및 셀 위젯(버튼 등) 활성/비활성."""
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

    def _stop_all_periodic(self):
        """주기 전송(0102 등) 일괄 중지."""
        try:
            tab = self._tab
            timers = getattr(tab, "periodic_timers", {})
            for code, t in list(timers.items()):
                try: t.stop()
                except Exception: pass
            try: timers.clear()
            except Exception: pass
            self._set_0102_heartbeat_enabled(False)
            self._append_log_line("[POWER] periodic TX 정지")
        except Exception:
            pass

    # ───────── 순차 푸시(0301 송신 → 0305 완료 → 0903 요청) ─────────
    def _queue_post_0301_delivery(
        self,
        *,
        plan_ids: list[int],
        option_names: list[str],
        plan_meta: dict,
        is_execution_mode: bool,
        force_direct: bool,
        suppress_0702_fallback: bool,
    ) -> None:
        valid_plan_ids: list[int] = []
        for plan_id in plan_ids:
            try:
                valid_plan_ids.append(int(plan_id))
            except Exception:
                self._append_log_line(f"[WARN] post-0301 skip: invalid missionPlanID={plan_id}")

        if not valid_plan_ids and not (is_execution_mode and not force_direct):
            self._append_log_line("[WARN] No valid missionPlanID for post-0301 apply")
            return

        plan_count = max(len(valid_plan_ids), 1)
        grace_ms, fallback_ms, delivery_mode = _post_0301_delivery_delays(
            plan_count=plan_count,
            force_direct=bool(force_direct),
        )
        self._post_0301_delivery = {
            "plan_ids": valid_plan_ids,
            "option_names": list(option_names or []),
            "plan_meta": dict(plan_meta or {}),
            "execution_mode": bool(is_execution_mode),
            "force_direct": bool(force_direct),
            "suppress_0702_fallback": bool(suppress_0702_fallback),
            "replanTransactionId": str(
                (getattr(self, "_active_plan_context", {}) or {}).get("replanTransactionId") or ""
            ),
            "delivery_mode": delivery_mode,
            "ready_at": time.monotonic() + (grace_ms / 1000.0),
            "mode_ready": False,
            "requires_mode_ready": not bool(force_direct),
            "completion_ready": False,
        }
        self._record_replan_timing_event(
            "post_0301_delivery_queued",
            extra={
                "grace_ms": int(grace_ms),
                "timeout_ms": int(fallback_ms),
                "force_direct": int(bool(force_direct)),
                "mode": delivery_mode,
            },
        )
        try:
            if self._post_0301_timer.isActive():
                self._post_0301_timer.stop()
            self._post_0301_timer.start(int(fallback_ms))
        except Exception:
            pass
        self._append_log_line(
            "[INFO] Post-0301 apply queued "
            f"(mode={delivery_mode}, grace={grace_ms}ms, timeout={fallback_ms}ms, plans={len(valid_plan_ids)})"
        )

    def _mark_post_0301_ready(self, *, trigger: str) -> bool:
        pending = getattr(self, "_post_0301_delivery", None)
        if not pending:
            return False
        if not pending.get("mode_ready"):
            pending["mode_ready"] = True
            self._append_log_line(f"[INFO] Post-0301 ready signal received ({trigger})")
        return self._try_flush_post_0301_delivery(trigger=trigger, force=False)

    def _try_flush_post_0301_delivery(self, *, trigger: str, force: bool) -> bool:
        pending = getattr(self, "_post_0301_delivery", None)
        if not pending:
            return False
        if not force and pending.get("requires_mode_ready") and not pending.get("mode_ready"):
            return False
        if not pending.get("completion_ready"):
            if force:
                self._record_replan_timing_event(
                    "post_0301_waiting_completion",
                    extra={"trigger": trigger, "mode": str(pending.get("delivery_mode") or "")},
                )
            return False

        remaining_ms = max(
            0,
            int(round((float(pending.get("ready_at", 0.0)) - time.monotonic()) * 1000.0)),
        )
        if remaining_ms > 0 and not force:
            try:
                timer = getattr(self, "_post_0301_timer", None)
                if timer is not None:
                    timer.start(remaining_ms)
            except Exception:
                pass
            return False

        self._post_0301_delivery = None
        try:
            timer = getattr(self, "_post_0301_timer", None)
            if timer is not None and timer.isActive():
                timer.stop()
        except Exception:
            pass

        valid_plan_ids = list(pending.get("plan_ids") or [])
        option_names = list(pending.get("option_names") or [])
        plan_meta = dict(pending.get("plan_meta") or {})
        is_execution_mode = bool(pending.get("execution_mode"))
        force_direct = bool(pending.get("force_direct"))
        suppress_0702_fallback = bool(pending.get("suppress_0702_fallback"))
        if _plan_meta_has_quality_speed(plan_meta):
            force_direct = True
            suppress_0702_fallback = True
            option_names = []

        if self._consume_attack_delivery_suppress_flag(phase=f"post-0301 ({trigger})"):
            self._flush_deferred_id_tab_update()
            return True

        if is_execution_mode and not force_direct:
            self._append_log_line(f"[INFO] Post-0301 ready ({trigger}) -> sending 0901")
            self._record_replan_timing_event(
                "post_0301_flushed",
                extra={"delivery": "0901", "trigger": trigger},
            )
            self._push_0901_options(valid_plan_ids, option_names, plan_meta)
            self._flush_deferred_id_tab_update()
            return True

        if not valid_plan_ids:
            self._append_log_line("[WARN] Post-0301 ready but no valid missionPlanID for 0903")
            self._flush_deferred_id_tab_update()
            return False

        self._append_log_line(f"[INFO] Post-0301 ready ({trigger}) -> sending 0903")
        self._record_replan_timing_event(
            "post_0301_flushed",
            extra={
                "delivery": "0903",
                "trigger": trigger,
                "plans": len(valid_plan_ids),
                "suppress_0702": int(bool(suppress_0702_fallback)),
            },
        )
        for idx, mpid in enumerate(valid_plan_ids):
            delay = idx * 200
            QTimer.singleShot(delay, lambda pid=mpid: self._push_0903(pid))
            if force_direct and not suppress_0702_fallback:
                QTimer.singleShot(delay + 250, lambda pid=mpid: self._push_0702_auto_apply(pid))
            elif force_direct and suppress_0702_fallback:
                self._record_replan_timing_event(
                    "0702_suppressed",
                    extra={"missionPlanID": int(mpid), "reason": "suppress_0702_fallback"},
                )
        self._flush_deferred_id_tab_update()
        return True

    def _push_post_0301_completion(self, *, reason: str) -> bool:
        pending = getattr(self, "_post_0301_delivery", None)
        completed_plan_ids = (
            _normalize_mission_plan_ids(pending.get("plan_ids"))
            if isinstance(pending, dict)
            else []
        )
        sent = bool(
            self._push_0305(
                status=2,
                reason=reason,
                completed_plan_ids=completed_plan_ids,
            )
        )
        if not isinstance(pending, dict):
            return sent
        if sent:
            pending["completion_ready"] = True
            pending["completion_ready_at"] = time.monotonic()
            self._record_replan_timing_event(
                "post_0301_completion_ready",
                extra={"mode": str(pending.get("delivery_mode") or "")},
            )
            self._try_flush_post_0301_delivery(trigger="0305_status_2", force=False)
        else:
            self._post_0301_delivery = None
            try:
                timer = getattr(self, "_post_0301_timer", None)
                if timer is not None and timer.isActive():
                    timer.stop()
            except Exception:
                pass
            self._record_replan_timing_event(
                "post_0301_completion_failed",
                extra={"mode": str(pending.get("delivery_mode") or "")},
            )
            self._append_log_line("[WARN] 0305 completion failed/suppressed -> post-0301 delivery dropped")
            self._flush_deferred_id_tab_update()
        return sent

    @staticmethod
    def _normalize_post_delivery_waypoint_mark(payload: Any) -> Dict[str, Any] | None:
        return normalize_post_delivery_waypoint_mark(payload)

    @classmethod
    def _merge_post_delivery_waypoint_mark(cls, existing: Any, incoming: Any) -> Dict[str, Any] | None:
        return merge_post_delivery_waypoint_mark(existing, incoming)

    def _schedule_post_delivery_waypoint_mark(self, payload: Any) -> None:
        mark_payload = self._normalize_post_delivery_waypoint_mark(payload)
        if not mark_payload:
            return

        def worker() -> None:
            started = time.perf_counter()
            max_waypoint_id = int(mark_payload.get("max_waypoint_id") or 0)
            try:
                _ensure_mission_planner_import_paths()
                from modules.mission_planning.engine.mission_generation.id_allocation.allocator import (
                    mark_waypoint_files_written,
                )
            except Exception:
                try:
                    from data_def.id_allocator import mark_waypoint_files_written  # type: ignore
                except Exception as exc:
                    self.log_sig.emit(
                        f"[WARN] Post-delivery waypoint mark import failed: {exc}"
                    )
                    return
            try:
                mark_waypoint_files_written(max_waypoint_id if max_waypoint_id > 0 else None)
                outcome = "ok"
            except Exception as exc:
                outcome = "error"
                self.log_sig.emit(f"[WARN] Post-delivery waypoint mark failed: {exc}")
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            try:
                self._record_replan_timing_event(
                    "post_delivery_waypoint_mark",
                    extra={
                        "max_waypoint_id": int(max_waypoint_id),
                        "variants": int(mark_payload.get("variants") or 0),
                        "elapsed_ms": round(float(elapsed_ms), 3),
                        "outcome": outcome,
                    },
                )
            except Exception:
                pass
            self.log_sig.emit(
                "[REPLAN][METRIC] general_variant_store_post_delivery_waypoint_mark "
                f"variants={int(mark_payload.get('variants') or 0)} "
                f"max_waypoint_id={int(max_waypoint_id)} "
                f"elapsed_ms={float(elapsed_ms):.3f} "
                f"outcome={outcome}"
            )

        threading.Thread(
            target=worker,
            name="PostDelivery-WaypointMark",
            daemon=True,
        ).start()

    @staticmethod
    def _normalize_post_delivery_snapshot_carry_forward(payload: Any) -> Dict[str, Any] | None:
        return normalize_post_delivery_snapshot_carry_forward(payload)

    @classmethod
    def _merge_post_delivery_snapshot_carry_forward(cls, existing: Any, incoming: Any) -> Dict[str, Any] | None:
        return merge_post_delivery_snapshot_carry_forward(existing, incoming)

    def _schedule_post_delivery_snapshot_carry_forward(self, payload: Any) -> None:
        carry_payload = self._normalize_post_delivery_snapshot_carry_forward(payload)
        if not carry_payload:
            return

        def worker() -> None:
            started = time.perf_counter()
            carried = 0
            skipped = 0
            errors = 0
            items = list(carry_payload.get("items") or [])
            for item in items:
                try:
                    path = mission_area_replan_store.carry_forward_snapshot(
                        int(item.get("sourceMissionPlanID") or 0),
                        int(item.get("targetMissionPlanID") or 0),
                        reason=str(item.get("reason") or ""),
                    )
                    if path is not None:
                        carried += 1
                    else:
                        skipped += 1
                except Exception as exc:
                    errors += 1
                    self.log_sig.emit(
                        "[WARN] Post-delivery mission area snapshot carry-forward failed "
                        f"(sourcePlan={item.get('sourceMissionPlanID')}, "
                        f"targetPlan={item.get('targetMissionPlanID')}): {exc}"
                    )
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            outcome = "ok" if errors == 0 else "error"
            try:
                self._record_replan_timing_event(
                    "post_delivery_snapshot_carry_forward",
                    extra={
                        "items": len(items),
                        "carried": int(carried),
                        "skipped": int(skipped),
                        "errors": int(errors),
                        "elapsed_ms": round(float(elapsed_ms), 3),
                        "outcome": outcome,
                    },
                )
            except Exception:
                pass
            self.log_sig.emit(
                "[REPLAN][METRIC] general_variant_store_post_delivery_snapshot_carry_forward "
                f"items={len(items)} carried={int(carried)} skipped={int(skipped)} "
                f"errors={int(errors)} elapsed_ms={float(elapsed_ms):.3f} outcome={outcome}"
            )

        threading.Thread(
            target=worker,
            name="PostDelivery-SnapshotCarry",
            daemon=True,
        ).start()

    def _start_push_sequence(self):
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF -> push sequence blocked")
            return
        payload = self._pending_plan_push or {}
        force_direct = bool(payload.get("force_direct_update"))
        suppress_0702_fallback = bool(payload.get("suppress_0702_fallback"))
        plan_ids = list(payload.get("plan_ids") or [])
        option_names = list(payload.get("option_names") or [])
        reason = _sanitize_reason(payload.get("reason"), "init-plan")
        plan_meta = payload.get("option_meta") or {}
        post_delivery_waypoint_mark = payload.get("post_delivery_waypoint_mark")
        post_delivery_snapshot_carry_forward = payload.get("post_delivery_snapshot_carry_forward")
        if _is_quality_speed_reason_text(reason) or _plan_meta_has_quality_speed(plan_meta):
            force_direct = True
            suppress_0702_fallback = True
            option_names = []

        if self._consume_attack_delivery_suppress_flag(phase="0301"):
            self._pending_plan_push = None
            return

        is_execution_mode = False
        if not force_direct:
            try:
                mode_slider = getattr(self, "mode_slider", None)
                if mode_slider is not None:
                    is_execution_mode = int(mode_slider.value()) == 3
            except Exception:
                is_execution_mode = False
        else:
            if suppress_0702_fallback:
                self._append_log_line("[INFO] Direct delivery -> skip 0901/0701, send 0301+0903 only")
            else:
                self._append_log_line("[INFO] Direct delivery -> skip 0901/0701, send 0301+0903(+0702 fallback)")

        if not plan_ids:
            self._append_log_line("[WARN] No missionPlanID to push (0301)")
            return

        # 0301 serializes MissionPlan files immediately.  Replace any value
        # inherited from the source plan before that read.  The exact
        # 0902->0305 value is backfilled after the successful status=2 send.
        provisional_ctx = getattr(self, "_replan_timing_context_inflight", None)
        if not isinstance(provisional_ctx, dict):
            provisional_ctx = getattr(self, "_active_plan_context", None)
        provisional_ms = _replan_elapsed_ms(provisional_ctx)
        if provisional_ms is not None:
            sync_result = _sync_mission_plan_planning_time(plan_ids, provisional_ms)
            timing = provisional_ctx.get("_replan_timing") if isinstance(provisional_ctx, dict) else None
            if isinstance(timing, dict):
                timing["mission_plan_pre_0301_ms"] = float(provisional_ms)
            synced_count = len(sync_result.get("updatedPlanIDs") or []) + len(
                sync_result.get("unchangedPlanIDs") or []
            )
            self.log_sig.emit(
                "[REPLAN][TIME] MissionPlan planningTime provisional "
                f"0902_to_pre0301_ms={provisional_ms:.3f} "
                f"plans={synced_count}/{len(_normalize_mission_plan_ids(plan_ids))}"
            )
            if sync_result.get("missingPlanIDs") or sync_result.get("errors"):
                self.log_sig.emit(
                    "[WARN] MissionPlan planningTime provisional sync incomplete: "
                    f"missing={sync_result.get('missingPlanIDs') or []}, "
                    f"errors={sync_result.get('errors') or []}"
                )

        # Send 0301 first, then queue 0305 completion on the next event-loop turn.
        sent_0301 = self._click_tx_button_for("0301")
        if sent_0301:
            self._record_replan_timing_event("0301_sent", extra={"plans": len(plan_ids)})
            # 0204(협업기저임무계획) is owned by MSM: it is pre-sent only in the
            # type-1 0201 review flow, before the 0902. MMR must not publish its
            # own 0204 - non-type-1 packages have no 0204 at all. (The legacy
            # post-0301 auto-send that lived here leaked 0204 for every package.)
        else:
            self._record_replan_timing_event("0301_failed", extra={"plans": len(plan_ids)})
            self._append_log_line("[WARN] 0301 push failed -> 0305 completion not scheduled")
            self._flush_deferred_id_tab_update()

        if sent_0301:
            self._queue_post_0301_delivery(
                plan_ids=plan_ids,
                option_names=option_names,
                plan_meta=plan_meta,
                is_execution_mode=is_execution_mode,
                force_direct=force_direct,
                suppress_0702_fallback=suppress_0702_fallback,
            )
            if self._push_post_0301_completion(reason=reason):
                self._schedule_post_delivery_waypoint_mark(post_delivery_waypoint_mark)
                self._schedule_post_delivery_snapshot_carry_forward(post_delivery_snapshot_carry_forward)

        self._pending_plan_push = None

    def _is_target_detection_attack_delivery_context(self, ctx: Any) -> bool:
        if not isinstance(ctx, dict):
            return False
        detail = ctx.get("replan_detail")
        if not isinstance(detail, dict):
            return False
        trigger = str(detail.get("trigger") or detail.get("triggerType") or "").strip()
        return trigger == "0402"

    @staticmethod
    def _coerce_int_set(values: Any) -> set[int]:
        out: set[int] = set()
        if values is None:
            return out
        if isinstance(values, (str, bytes)) or not isinstance(values, (list, tuple, set)):
            values = [values]
        for value in values:
            try:
                out.add(int(value))
            except Exception:
                continue
        return out

    def _suppress_flag_matches_active_context(
        self,
        suppress_flag: Any,
        active_ctx: Any,
        *,
        phase: str,
    ) -> bool:
        if not isinstance(suppress_flag, dict):
            return False
        if not isinstance(active_ctx, dict):
            return False

        detail = active_ctx.get("replan_detail")
        if not isinstance(detail, dict):
            detail = {}

        flag_plan_ids = self._coerce_int_set(suppress_flag.get("plan_ids"))
        active_plan_ids = self._coerce_int_set(active_ctx.get("plan_ids"))
        if flag_plan_ids and active_plan_ids and flag_plan_ids.isdisjoint(active_plan_ids):
            self._append_log_line(
                "[0402] stale suppress flag ignored before "
                f"{phase}: flagPlans={sorted(flag_plan_ids)}, activePlans={sorted(active_plan_ids)}"
            )
            return False

        try:
            flag_target_id = int(suppress_flag.get("target_id"))
        except Exception:
            flag_target_id = None
        try:
            active_target_id = int(detail.get("targetID") or detail.get("targetId"))
        except Exception:
            active_target_id = None
        if flag_target_id is not None and active_target_id is not None and flag_target_id != active_target_id:
            self._append_log_line(
                "[0402] stale suppress flag ignored before "
                f"{phase}: flagTarget={flag_target_id}, activeTarget={active_target_id}"
            )
            return False

        flag_target_key = str(suppress_flag.get("target_key") or "").strip()
        active_target_key = str(detail.get("targetKey") or "").strip()
        if flag_target_key and active_target_key and flag_target_key != active_target_key:
            self._append_log_line(
                "[0402] stale suppress flag ignored before "
                f"{phase}: flagKey={flag_target_key}, activeKey={active_target_key}"
            )
            return False

        return True

    def _consume_attack_delivery_suppress_flag(self, *, phase: str) -> bool:
        active_ctx = getattr(self, "_active_plan_context", {}) or {}
        if not self._is_target_detection_attack_delivery_context(active_ctx):
            return False
        try:
            from modules.monitoring.logic.replan_queue_manager import read_and_clear_suppress_option_flag

            suppress_flag = read_and_clear_suppress_option_flag()
        except Exception:
            suppress_flag = None
        if suppress_flag is None:
            return False
        if not self._suppress_flag_matches_active_context(
            suppress_flag,
            active_ctx,
            phase=phase,
        ):
            return False

        reason = str(suppress_flag.get("reason") or "Target 정보 변경으로 인한 재계획 중단").strip()
        if not reason or (reason.lower().startswith("target") and "?" in reason):
            reason = "Target 정보 변경으로 인한 재계획 중단"
        if "target_option_suppressed" not in reason.lower():
            reason = f"target_option_suppressed: {reason}"

        self._pending_plan_push = None
        self._scheduled_0301_plan_ids = []
        self._post_0301_delivery = None
        try:
            timer = getattr(self, "_post_0301_timer", None)
            if timer is not None and timer.isActive():
                timer.stop()
        except Exception:
            pass

        self._append_log_line(f"[0402] delivery suppressed before {phase}: {reason}")
        sent = self._push_0305(
            status=2,
            reason=reason,
            planning_success=True,
            check_delivery_suppress=False,
        )
        if sent:
            self._append_log_line(f"[0402] suppress completion sent via 0305 before {phase}")
        else:
            self._append_log_line(f"[0402] suppress completion 0305 send failed before {phase}")
        return True

    def _click_tx_button_for(self, code: str) -> bool:
        if not self._power_on:
            self._append_log_line(f"[BLOCK] Power OFF -> TX '{code}' blocked")
            return False
        try:
            tab = getattr(self, "_tab", None)
            if tab is None or not hasattr(tab, "tbl_tx"):
                self._append_log_line(f"[WARN] TX table missing for code={code}")
                return False

            tbl = tab.tbl_tx
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == str(code):
                    target_row = r
                    break

            if target_row < 0:
                self._append_log_line(f"[WARN] TX table has no entry for {code}")
                return False

            if str(code).strip() == "0301":
                plan_ids: list[int] = []
                for pid in self._scheduled_0301_plan_ids or []:
                    try:
                        plan_ids.append(int(pid))
                    except Exception:
                        continue
                plan_ids = list(dict.fromkeys(plan_ids))
                if plan_ids:
                    sent = bool(self._send_0301_batch(plan_ids))
                    if sent:
                        self._append_log_line(f"[PUSH] {code} direct batch send")
                    else:
                        self._append_log_line(f"[ERR] {code} direct batch send failed")
                    return sent

            try:
                if hasattr(tab, "send_tx_row"):
                    ok = bool(tab.send_tx_row(target_row, interactive=False))
                    if ok:
                        self._append_log_line(f"[PUSH] {code} direct handler invoked")
                    else:
                        self._append_log_line(f"[WARN] {code} direct handler returned false")
                    return ok
            except Exception:
                pass

            try:
                btn = tbl.cellWidget(target_row, 3)
                if btn is not None and hasattr(btn, "click"):
                    btn.click()
                    self._append_log_line(f"[PUSH] {code} button click()")
                    return True
            except Exception:
                pass

            try:
                if hasattr(tab, "_on_tx_button_clicked"):
                    tab._on_tx_button_clicked(target_row)
                    self._append_log_line(f"[PUSH] {code} handler invoked")
                    return True
            except Exception:
                pass

            self._append_log_line(f"[ERR] {code} push failed: no button/handler")
            return False
        except Exception as e:
            self._append_log_line(f"[ERR] {code} push failed: {e}")
            return False

    def _init_gui_log_file_sink(self, *, force_new: bool = False, db_root: Optional[Path] = None) -> None:
        try:
            if db_root is None:
                db_root = db_paths.peek_active_db_root(existing_only=True)
            if db_root is None:
                if force_new:
                    self._log_file_path = None
                    self._log_file_db_root = None
                return
            db_root_str = str(db_root)
            log_dir = db_root / "DSS_Internal"
            log_dir.mkdir(parents=True, exist_ok=True)
            current_path = getattr(self, "_log_file_path", None)
            current_root = str(getattr(self, "_log_file_db_root", "") or "")
            if (
                not force_new
                and current_path is not None
                and current_root == db_root_str
                and current_path.parent == log_dir
            ):
                return
            token = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            path = log_dir / f"mission_planning_gui_{token}.log"
            path.write_text("", encoding="utf-8")
            self._log_file_path = path
            self._log_file_db_root = db_root_str
        except Exception:
            self._log_file_path = None
            self._log_file_db_root = None

    def _init_db_root_sync(self) -> None:
        self._refresh_db_root(log_first=True)
        self._db_root_timer = QTimer(self)
        self._db_root_timer.setInterval(1000)
        self._db_root_timer.timeout.connect(self._refresh_db_root)
        self._db_root_timer.start()

    def _refresh_db_root(self, log_first: bool = False) -> None:
        try:
            root = db_paths.peek_active_db_root()
        except Exception as exc:
            self._append_log_line(f"[PATH] DB root check failed: {exc}")
            return
        if root is None:
            return
        root_str = str(root)
        root_exists = root.exists()
        prev_root = getattr(self, "_db_root", None)
        current_sink_root = str(getattr(self, "_log_file_db_root", "") or "")
        if not root_exists:
            self._db_root = root_str
            return
        need_log_rebound = bool(root_exists and current_sink_root != root_str)
        if not log_first and root_str == prev_root and not need_log_rebound:
            return
        self._db_root = root_str
        try:
            if need_log_rebound:
                self._init_gui_log_file_sink(force_new=True, db_root=root)
        except Exception:
            pass
        if prev_root and prev_root != root_str:
            self._append_log_line(f"[PATH] DB root changed: {prev_root} -> {root_str}")
            self._emit_lifecycle(
                "db_root_rebind",
                component="db_root",
                outcome="ok",
                extra={"previousRoot": str(prev_root), "dbRoot": root_str},
            )
        else:
            self._append_log_line(f"[PATH] DB root -> {root_str}")
            self._emit_lifecycle(
                "db_root_bind",
                component="db_root",
                outcome="ok",
                extra={"dbRoot": root_str},
            )
        if need_log_rebound and getattr(self, "_log_file_path", None):
            self._append_log_line(f"[LOG] Mission planning log rebound: {self._log_file_path}")
        try:
            if prev_root and prev_root != root_str:
                self._schedule_planner_warmup("db_root_rebind")
            elif log_first or need_log_rebound:
                self._schedule_planner_warmup("db_root_bind")
        except Exception:
            pass

    def _persist_gui_log(self, text: str) -> None:
        path = getattr(self, "_log_file_path", None)
        if not path:
            return
        write_started = time.perf_counter()
        write_retry = False
        try:
            if not path.parent.exists():
                self._init_gui_log_file_sink()
                path = getattr(self, "_log_file_path", None)
                if not path:
                    return
            stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
            with path.open("a", encoding="utf-8") as fh:
                fh.write(f"{stamp} {text}\n")
        except Exception:
            write_retry = True
            try:
                self._init_gui_log_file_sink()
                path = getattr(self, "_log_file_path", None)
                if not path:
                    return
                stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
                with path.open("a", encoding="utf-8") as fh:
                    fh.write(f"{stamp} {text}\n")
            except Exception:
                pass
        finally:
            write_elapsed_ms = max(0.0, (time.perf_counter() - write_started) * 1000.0)
            if write_retry or write_elapsed_ms >= 100.0:
                try:
                    from modules.common.process_console import request_runtime_diagnostic_snapshot

                    request_runtime_diagnostic_snapshot(
                        "mission_planning",
                        "gui_log_write_slow",
                        context={
                            "event": "gui_log_write",
                            "writeElapsedMs": round(write_elapsed_ms, 3),
                            "writeRetry": bool(write_retry),
                        },
                    )
                except Exception:
                    pass

    def _append_log_line(self, text: str):
        if QThread.currentThread() != self.thread():
            try:
                self.log_sig.emit(str(text))
            except Exception:
                pass
            return
        try:
            emit_process_log("mission_planning", str(text))
        except Exception:
            pass
        try:
            run_log = getattr(self, "_active_plan_log_run", None)
            if run_log:
                run_log.add_message(text)
        except Exception:
            pass
        self._persist_gui_log(text)
        try:
            if getattr(self, "_tab", None) and hasattr(self._tab, "append_log"):
                self._tab.append_log(text); return
        except Exception:
            pass
        try: print(text)
        except Exception: pass

    def _mark_tab_sent(self, msg_id: str, raw: object = None) -> None:
        try:
            if getattr(self, "_tab", None) is not None:
                self._tab.mark_sent(str(msg_id), raw if isinstance(raw, bytes) else None)
        except Exception:
            pass

    def _flush_runtime_fov_adjustment_logs(self) -> None:
        try:
            messages = pop_runtime_camera_fov_adjustment_logs()
        except Exception:
            messages = []
        for message in messages:
            if message:
                try:
                    self.log_sig.emit(str(message))
                except Exception:
                    self._append_log_line(str(message))

    def _mark_attack_target_used(self, attack_result: dict | None) -> None:
        try:
            result = (attack_result or {}).get("result") or {}
            targets_payload = result.get("attack_targets") or result.get("attackTargets")
            payload_items: list[dict[str, Any]] = []

            if isinstance(targets_payload, list):
                for item in targets_payload:
                    if not isinstance(item, dict):
                        continue
                    payload: dict[str, Any] = {}
                    key = item.get("key") or item.get("targetKey")
                    target_id = item.get("target_id") or item.get("targetID") or item.get("targetId")
                    watcher_id = item.get("watcher_id") or item.get("watcherID") or item.get("watcherId")
                    if key is not None:
                        payload["key"] = key
                    if target_id is not None:
                        payload["targetID"] = target_id
                    if watcher_id is not None:
                        payload["watcherID"] = watcher_id
                    if payload:
                        payload_items.append(payload)

            primary = result.get("primary_target")
            if not payload_items and not isinstance(primary, dict):
                return

            if not payload_items and isinstance(primary, dict):
                key = primary.get("key") or primary.get("targetKey")
                target_id = primary.get("target_id") or primary.get("targetID") or primary.get("targetId")
                watcher_id = primary.get("watcher_id") or primary.get("watcherID") or primary.get("watcherId")
                raw = primary.get("raw")
                if isinstance(raw, dict):
                    key = key or raw.get("key") or raw.get("targetKey")
                    target_id = target_id or raw.get("targetID") or raw.get("targetId")
                    watcher_id = watcher_id or raw.get("watcherID") or raw.get("watcherId")
                payload = {}
                if key is not None:
                    payload["key"] = key
                if target_id is not None:
                    payload["targetID"] = target_id
                if watcher_id is not None:
                    payload["watcherID"] = watcher_id
                if payload:
                    payload_items.append(payload)
            if not payload_items:
                return

            try:
                from modules.monitoring.logic.target_info import mark_targets_as_used
            except Exception as exc:
                self._append_log_line(f"[ATTACK] mark target used import failed: {exc}")
                return

            mark_targets_as_used(payload_items)
            target_ids = [
                str(item.get("targetID"))
                for item in payload_items
                if item.get("targetID") is not None
            ]
            self._append_log_line(f"[ATTACK] mark target used: targetIDs={', '.join(target_ids) or '-'}")
        except Exception as exc:
            self._append_log_line(f"[ATTACK] mark target used failed: {exc}")

    # ───────── 모드/슬라이더 ─────────
    def _on_mode_slider_changed(self, val: int):
        try: self.mode_now.setText(MODE_LABELS[int(val)])
        except Exception: pass
        self._power_on = True
        self._append_log_line(f"[MODE] 슬라이더 변경 → {MODE_LABELS[int(val)] if 0 <= val < len(MODE_LABELS) else val}")
        self._apply_power_state()
        if self._power_on:
            QTimer.singleShot(500, self._start_0102_stream)

    def _resolve_mode_code_from_text(self, text: str) -> int:
        return resolve_mode_code_from_text(text)

    def _current_mode_code(self) -> int | None:
        try:
            slider = getattr(self, "mode_slider", None)
            if slider is None:
                return None
            return int(slider.value())
        except Exception:
            return None

    def _should_ignore_ctrl_mode_change(self, requested_code: int) -> bool:
        current_code = self._current_mode_code()
        if current_code == 3 and int(requested_code) != 3:
            self._append_log_line(
                f"[MODE] CTRL mode change ignored during execution: requested={requested_code}"
            )
            return True
        return False

    def _set_mode_slider_by_text(self, text: str):
        val = self._resolve_mode_code_from_text(text)
        try:
            if getattr(self, "mode_slider", None):
                if self.mode_slider.value() != val:
                    self.mode_slider.blockSignals(True)
                    self.mode_slider.setValue(val)
                    self.mode_slider.blockSignals(False)
            if getattr(self, "mode_now", None):
                self.mode_now.setText(MODE_LABELS[val])
            # ★ 텍스트 기반 모드 변경도 통지
        except Exception:
            pass
        self._power_on = True
        self._apply_power_state()
        if self._power_on:
            QTimer.singleShot(500, self._start_0102_stream)

    # ───────── nFusion RX 초기화 ─────────
    def _rx_setup(self):
        try:
            with fusion_runtime_working_dir(project_root=PROJECT_ROOT):
                FusionNodeIoc.Configure()
                NodeMessenger.Initialize("MMR_ReceiveNode")
                NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
                NodeMessenger.InitAllSubscriberFromAssembly()
                NodeMessenger.RegistAllProviderFromFusionNodeIoc()
            self._bus_ready = True
            try:
                self.log_sig.emit("[BUS] NodeMessenger 초기화 완료")
            except Exception:
                pass
        except Exception as exc:
            self._bus_ready = False
            try:
                self.log_sig.emit(f"[BUS ERR] NodeMessenger 초기화 실패: {exc}")
            except Exception:
                pass

    # ───────── 0305 / 0903 요청 ─────────
    def _should_prefix_0305_reason_for_next_collab(self) -> bool:
        for container in (
            getattr(self, "_active_plan_context", None),
            getattr(self, "_staged_plan_context", None),
        ):
            if not isinstance(container, dict):
                continue
            detail = container.get("replan_detail")
            if not isinstance(detail, dict):
                continue
            if str(detail.get("triggerType") or "").strip() == "nextCollaborativeMission":
                return True
        return False

    def _push_0305(
        self,
        status: int,
        reason: str = "초기임무재계획",
        *,
        planning_success: bool = True,
        check_delivery_suppress: bool = True,
        completed_plan_ids: Optional[List[int]] = None,
    ) -> bool:
        delivered_plan_ids = _normalize_mission_plan_ids(completed_plan_ids)
        active_ctx = getattr(self, "_active_plan_context", None)
        inflight_ctx = getattr(self, "_replan_timing_context_inflight", None)
        if int(status) == 2 and isinstance(inflight_ctx, dict):
            target_ctx = inflight_ctx
        else:
            target_ctx = active_ctx
        target_ctx = target_ctx if isinstance(target_ctx, dict) else None
        if int(status) == 1 and target_ctx is not None:
            # Pin the context that sent the running 0305.  A newly received or
            # deferred 0902 must not redirect the completion checkpoint to a
            # different request.
            self._replan_timing_context_inflight = target_ctx
        clean_reason = limit_utf8_bytes(_sanitize_reason(reason, "초기임무재계획"))
        if self._should_prefix_0305_reason_for_next_collab() and not clean_reason.startswith("_"):
            clean_reason = f"_{clean_reason}"
        elapsed_ms: Optional[float] = None
        if int(status) == 1:
            self._mark_planning_metric_start(clean_reason)
        elif int(status) == 2:
            elapsed_ms = self._mark_planning_metric_finish(clean_reason, success=bool(planning_success))
            if (
                check_delivery_suppress
                and planning_success
                and self._consume_attack_delivery_suppress_flag(phase="0305(status=2)")
            ):
                self.log_sig.emit(f"[0305] status=2 suppressed: {clean_reason}")
                return False
        try:
            from push_center import push_message
            body = {
                "timestamp": _now_ms_since_2000(),
                "source": "MMR",
                "missionPlanningStatus": int(status),  # 1: 재계획 수행 중, 2: 재계획 완료
                "replanReason": clean_reason,
            }
            orig_mark_fn = getattr(self, "_orig_mark_sent", None)
            mon_sent_via_wrapper = {"done": False}
            send_checkpoint = {"recorded": False}

            def _after_push(mid, raw):
                # push_center invokes this callback only after make_and_push
                # returns successfully.  Capture that boundary before GUI/log
                # work so elapsed time means actual 0305 transmission.
                event_perf = time.perf_counter()
                event_wall_ms = int(time.time() * 1000)
                if int(status) == 1:
                    self._record_replan_timing_snapshot(
                        "0305_status_1",
                        event_perf=event_perf,
                        wall_ms=event_wall_ms,
                        ctx=target_ctx,
                        extra={"reason": clean_reason},
                    )
                    send_checkpoint["recorded"] = True
                elif int(status) == 2:
                    self._record_replan_timing_snapshot(
                        "0305_status_2",
                        event_perf=event_perf,
                        wall_ms=event_wall_ms,
                        ctx=target_ctx,
                        extra={"reason": clean_reason},
                    )
                    send_checkpoint["recorded"] = True
                mid_norm = _z4(str(mid))
                if callable(orig_mark_fn):
                    try:
                        orig_mark_fn(mid_norm, raw)
                        return
                    except Exception:
                        pass
                tab = getattr(self, "_tab", None)
                mark_method = getattr(tab, "mark_sent", None) if tab else None
                if callable(mark_method):
                    try:
                        mark_method(mid_norm, raw)
                        if callable(orig_mark_fn):
                            mon_sent_via_wrapper["done"] = True
                    except Exception:
                        pass

            sent = bool(
                push_message(
                    "0305",
                    NodeMessenger,
                    on_done=_after_push,
                    body_dict=body,
                )
            )
            if not sent:
                if int(status) == 1 and getattr(self, "_replan_timing_context_inflight", None) is target_ctx:
                    self._replan_timing_context_inflight = None
                self.log_sig.emit(f"[ERR] 0305 전송 실패: status={status}, reason={clean_reason}")
                return False
            if int(status) in (1, 2) and target_ctx is not None and not send_checkpoint["recorded"]:
                self.log_sig.emit(
                    f"[ERR] 0305 전송 시각 누락: status={status}, reason={clean_reason}"
                )
                return False
            if int(status) == 2:
                timing = target_ctx.get("_replan_timing") if isinstance(target_ctx, dict) else None
                if planning_success and delivered_plan_ids and isinstance(timing, dict):
                    timing["delivered_plan_ids"] = list(delivered_plan_ids)
                    try:
                        exact_planning_time_ms = float(timing.get("0305_status_2_ms"))
                    except (TypeError, ValueError):
                        exact_planning_time_ms = -1.0
                    if math.isfinite(exact_planning_time_ms) and exact_planning_time_ms >= 0.0:
                        sync_result = _sync_mission_plan_planning_time(
                            delivered_plan_ids,
                            exact_planning_time_ms,
                        )
                        timing["mission_plan_planning_time_ms"] = round(
                            exact_planning_time_ms,
                            3,
                        )
                        timing["mission_plan_planning_time_synced_ids"] = list(
                            sync_result.get("updatedPlanIDs") or []
                        ) + list(sync_result.get("unchangedPlanIDs") or [])
                        synced_count = len(timing["mission_plan_planning_time_synced_ids"])
                        self.log_sig.emit(
                            "[REPLAN][TIME] MissionPlan planningTime finalized "
                            f"0902_to_0305_status_2_ms={exact_planning_time_ms:.3f} "
                            f"plans={synced_count}/{len(delivered_plan_ids)}"
                        )
                        if sync_result.get("missingPlanIDs") or sync_result.get("errors"):
                            self.log_sig.emit(
                                "[WARN] MissionPlan planningTime final sync incomplete: "
                                f"missing={sync_result.get('missingPlanIDs') or []}, "
                                f"errors={sync_result.get('errors') or []}"
                            )
                try:
                    self._persist_completed_replan_timing(
                        reason=clean_reason,
                        planning_success=bool(planning_success),
                        planning_elapsed_ms=elapsed_ms,
                        ctx=target_ctx,
                    )
                except Exception as exc:
                    self.log_sig.emit(f"[WARN] 재계획 시간 누적 기록 실패: {exc}")
                try:
                    timing = target_ctx.get("_replan_timing") if isinstance(target_ctx, dict) else {}
                    if isinstance(timing, dict) and str(timing.get("trigger") or "") == "general_3_option":
                        summary = timing.get("general_3_option_summary")
                        if isinstance(summary, dict):
                            status2_ms = float(timing.get("0305_status_2_ms") or 0.0)
                            pipeline_done_ms = float(timing.get("pipeline_done_ms") or 0.0)
                            delivery_tail_ms = max(0.0, status2_ms - pipeline_done_ms)
                            try:
                                sent_0301_ms = float(timing.get("0301_sent_ms") or 0.0)
                            except Exception:
                                sent_0301_ms = 0.0
                            pipeline_to_0301_sent_ms = (
                                max(0.0, sent_0301_ms - pipeline_done_ms)
                                if sent_0301_ms > 0.0 and pipeline_done_ms > 0.0
                                else 0.0
                            )
                            sent_0301_to_status2_ms = (
                                max(0.0, status2_ms - sent_0301_ms)
                                if sent_0301_ms > 0.0 and status2_ms > 0.0
                                else 0.0
                            )
                            summary["delivery_tail_ms"] = round(float(delivery_tail_ms), 3)
                            summary["pipeline_to_0301_sent_ms"] = round(float(pipeline_to_0301_sent_ms), 3)
                            summary["0301_sent_to_status2_ms"] = round(float(sent_0301_to_status2_ms), 3)
                            self.log_sig.emit(
                                "[REPLAN][METRIC] general_3_option_summary "
                                f"total_ms={float(summary.get('total_ms') or 0.0):.3f} "
                                f"parallel_gate_ms={float(summary.get('parallel_gate_ms') or 0.0):.3f} "
                                f"max_core_ms={float(summary.get('max_core_ms') or 0.0):.3f} "
                                f"max_total_ms={float(summary.get('max_total_ms') or 0.0):.3f} "
                                f"store_total_ms={float(summary.get('store_total_ms') or 0.0):.3f} "
                                f"store_wall_tail_ms={float(summary.get('store_wall_tail_ms') or 0.0):.3f} "
                                f"store_prepare_order_wait_ms={float(summary.get('store_prepare_order_wait_ms') or 0.0):.3f} "
                                f"store_prepare_order_wait_total_ms={float(summary.get('store_prepare_order_wait_total_ms') or 0.0):.3f} "
                                f"store_prepare_queue_wait_ms={float(summary.get('store_prepare_queue_wait_ms') or 0.0):.3f} "
                                f"store_prepare_queue_wait_total_ms={float(summary.get('store_prepare_queue_wait_total_ms') or 0.0):.3f} "
                                f"store_prepare_path_id_reservation_ms={float(summary.get('store_prepare_path_id_reservation_ms') or 0.0):.3f} "
                                f"store_prepare_path_id_reservation_total_ms={float(summary.get('store_prepare_path_id_reservation_total_ms') or 0.0):.3f} "
                                f"store_prepare_cross_path_id_reservation_ms={float(summary.get('store_prepare_cross_path_id_reservation_ms') or 0.0):.3f} "
                                f"store_prepare_enqueue_total_wait_ms={float(summary.get('store_prepare_enqueue_total_wait_ms') or 0.0):.3f} "
                                f"delivery_tail_ms={float(summary.get('delivery_tail_ms') or 0.0):.3f} "
                                f"pipeline_to_0301_sent_ms={float(summary.get('pipeline_to_0301_sent_ms') or 0.0):.3f} "
                                f"0301_sent_to_status2_ms={float(summary.get('0301_sent_to_status2_ms') or 0.0):.3f} "
                                f"critical_variant={int(summary.get('critical_variant') or 0)} "
                                f"critical_option={int(summary.get('critical_option') or 0)} "
                                f"max_core_variant={int(summary.get('max_core_variant') or 0)} "
                                f"max_core_option={int(summary.get('max_core_option') or 0)} "
                                f"variant_count={int(summary.get('variant_count') or 0)} "
                                f"pipeline_done_ms={pipeline_done_ms:.3f} "
                                f"0305_status_2_ms={status2_ms:.3f}"
                            )
                except Exception:
                    pass
                if getattr(self, "_replan_timing_context_inflight", None) is target_ctx:
                    self._replan_timing_context_inflight = None
            timing = target_ctx.get("_replan_timing") if isinstance(target_ctx, dict) else {}
            total_elapsed_ms = timing.get("0305_status_2_ms") if isinstance(timing, dict) else None
            if int(status) == 2 and total_elapsed_ms is not None:
                self.log_sig.emit(
                    f"[0305] status={status}, reason={clean_reason}, "
                    f"0902_to_0305={float(total_elapsed_ms) / 1000.0:.3f}s 전송"
                )
            else:
                self.log_sig.emit(f"[0305] status={status}, reason={clean_reason} 전송")
            return True
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0305 전송 실패: {e}")
            return False

    def _push_replan_failure_completion(self, contents: str) -> bool:
        if not contents:
            return False
        reason = str(contents).strip()
        if "실패" not in reason:
            reason = f"재계획 실패: {reason}"
        return bool(self._push_0305(status=2, reason=reason, planning_success=False))

    def _push_replan_noop_completion(self, reason: str, detail: str = "재계획 불필요") -> bool:
        base_reason = _sanitize_reason(reason, "초기임무재계획")
        detail_text = str(detail or "재계획 불필요").strip() or "재계획 불필요"
        if detail_text in base_reason:
            completion_reason = base_reason
        else:
            completion_reason = f"{base_reason} / {detail_text}"
        return bool(self._push_0305(status=2, reason=completion_reason))

    def _push_0903(self, mission_plan_id):
        try:
            from push_center import push_message
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0903 push unavailable: {e}")
            return

        if mission_plan_id is None:
            self.log_sig.emit("[WARN] 0903 skipped: missionPlanID missing")
            return

        try:
            mpid = int(mission_plan_id)
        except Exception:
            self.log_sig.emit(f"[WARN] 0903 skipped: invalid missionPlanID={mission_plan_id}")
            return

        body = {
            "timestamp": _now_ms_since_2000(),
            "source": "MMR",
            "missionPlanID": mpid,
        }
        try:
            push_message("0903", NodeMessenger, body_dict=body)
            self._record_replan_timing_event("0903_sent", extra={"missionPlanID": mpid})
            self.log_sig.emit(f"[0903] request sent (missionPlanID={mpid})")
            try:
                raw = json.dumps(body, ensure_ascii=False).encode("utf-8", "ignore")
            except Exception:
                raw = None
            try:
                self.tab_mark_sent_sig.emit(_z4("0903"), raw)
            except Exception:
                pass
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0903 push failed: {e}")

    def _push_0702_auto_apply(self, mission_plan_id):
        try:
            from push_center import push_message
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0702 push unavailable: {e}")
            return

        try:
            mpid = int(mission_plan_id)
        except Exception:
            self.log_sig.emit(f"[WARN] 0702 auto-apply skipped: invalid missionPlanID={mission_plan_id}")
            return
        if mpid <= 0:
            self.log_sig.emit(f"[WARN] 0702 auto-apply skipped: invalid missionPlanID={mission_plan_id}")
            return

        body = {
            "timestamp": _now_ms_since_2000(),
            "source": "MMR",
            "ignore": 2,
            "missionPlanID": mpid,
        }
        try:
            push_message("0702", NodeMessenger, body_dict=body)
            self._record_replan_timing_event("0702_sent", extra={"missionPlanID": mpid})
            self.log_sig.emit(f"[0702][AUTO] ignore=2 sent (missionPlanID={mpid})")
            try:
                raw = json.dumps(body, ensure_ascii=False).encode("utf-8", "ignore")
            except Exception:
                raw = None
            try:
                self.tab_mark_sent_sig.emit(_z4("0702"), raw)
            except Exception:
                pass
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0702 auto-apply push failed: {e}")


    def _allocate_option_ids(self, count: int) -> list[int]:
        ids: list[int] = []
        try:
            total = int(count)
        except Exception:
            total = 0
        for _ in range(max(total, 0)):
            self._option_id_counter += 1
            ids.append(self._option_id_counter)
        return ids

    def _push_0901_options(self, plan_ids, option_names, plan_meta=None):
        """Push option info request using supplied plan IDs."""
        try:
            active_ctx = dict(getattr(self, "_active_plan_context", {}) or {})
        except Exception:
            active_ctx = {}
        try:
            from modules.monitoring.logic.replan_queue_manager import read_and_clear_suppress_option_flag
            suppress_flag = read_and_clear_suppress_option_flag()
        except Exception:
            suppress_flag = None
        if suppress_flag is not None:
            if self._suppress_flag_matches_active_context(
                suppress_flag,
                active_ctx,
                phase="0901",
            ):
                reason = str(suppress_flag.get("reason") or "Target 정보 변경으로 인한 재계획 중단").strip()
                if not reason or (reason.lower().startswith("target") and "?" in reason):
                    reason = "Target 정보 변경으로 인한 재계획 중단"
                if "target_option_suppressed" not in reason.lower():
                    reason = f"target_option_suppressed: {reason}"
                self.log_sig.emit(f"[0901] option suppressed: {reason}")
                self._push_0305(
                    status=2,
                    reason=reason,
                    planning_success=True,
                    check_delivery_suppress=False,
                )
                return
            self.log_sig.emit("[0901] stale option suppress flag ignored")

        active_detail = active_ctx.get("replan_detail") if isinstance(active_ctx, dict) else None
        if (
            _plan_meta_has_quality_speed(plan_meta)
            or _is_quality_speed_reason_text(active_ctx.get("reason") if isinstance(active_ctx, dict) else None)
            or (
                isinstance(active_detail, dict)
                and _is_quality_speed_trigger_type(active_detail.get("triggerType"))
            )
        ):
            self.log_sig.emit("[QUALITY] 0901 blocked: quality improvement replan must not create options")
            return
        try:
            from push_center import push_message
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0901 push unavailable: {e}")
            return
        try:
            ts = _now_ms_since_2000()
            plan_list, name_list = _sort_plan_delivery_entries(plan_ids, option_names)
            meta_map = dict(plan_meta or {})
            valid_entries: list[tuple[int, int]] = []
            defaults = list(DEFAULT_OPTION_CODE_SEQUENCE) or [1]
            for idx, plan_id in enumerate(plan_list, 1):
                try:
                    pid = int(plan_id)
                except Exception:
                    continue
                raw_name = name_list[idx - 1] if idx - 1 < len(name_list) else None
                fallback_code = defaults[idx - 1] if idx - 1 < len(defaults) else defaults[-1]
                code = normalize_option_code(raw_name)
                if code is None:
                    code = fallback_code
                    if raw_name not in (None, ""):
                        self.log_sig.emit(
                            f"[WARN] Unknown option label for 0901 #{idx}: "
                            f"{raw_name!r}; fallback optionCode={code}({option_code_to_label(code)})"
                        )
                valid_entries.append((pid, code))
            if not valid_entries:
                self.log_sig.emit("[WARN] 0901 skipped: no entries")
                return
            option_ids = self._allocate_option_ids(len(valid_entries))
            entries = []
            for oid, (pid, code) in zip(option_ids, valid_entries):
                entry = {"optionID": oid, "optionName": code, "missionPlanID": pid}
                meta = meta_map.get(pid)
                if meta:
                    entry["optionMeta"] = meta
                entries.append(entry)
            body = {
                "timestamp": ts,
                "source": "MMR",
                "requestTime": ts,
                "pendingOptionList": entries,
            }
            push_message("0901", NodeMessenger, body_dict=body)
            self._record_replan_timing_event("0901_sent", extra={"options": len(entries)})
            labels = ", ".join(
                f"{entry['optionName']}({option_code_to_label(entry['optionName'])})"
                for entry in entries
            )
            self.log_sig.emit(f"[0901] option request sent (count={len(entries)}, codes={labels})")
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0901 push failed: {e}")

    def _push_0001_notice(self, contents: str) -> None:
        if not contents:
            return
        contents = limit_utf8_bytes(contents)
        try:
            from push_center import push_message
        except Exception as exc:
            self.log_sig.emit(f"[ERR] 0001 push unavailable: {exc}")
            return
        body = {
            "timestamp": _now_ms_since_2000(),
            "source": "MMR",
            "contents": str(contents),
        }
        try:
            push_message("0001", NodeMessenger, body_dict=body)
            self.log_sig.emit(f"[0001] notice sent: {contents}")
            try:
                raw = json.dumps(body, ensure_ascii=False).encode("utf-8", "ignore")
            except Exception:
                raw = None
            try:
                self.tab_mark_sent_sig.emit(_z4("0001"), raw)
            except Exception:
                pass
        except Exception as exc:
            self.log_sig.emit(f"[ERR] 0001 push failed: {exc}")

    def _plan_file_notice(self, label: str, exc: Optional[BaseException]) -> str:
        if isinstance(exc, FileNotFoundError):
            return limit_utf8_bytes(f"임무계획 실패: {label} 없음")
        if isinstance(exc, PermissionError):
            return limit_utf8_bytes(f"임무계획 실패: {label} 접근 실패")
        if isinstance(exc, (json.JSONDecodeError, UnicodeDecodeError)):
            return limit_utf8_bytes(f"임무계획 실패: {label} 형식 오류")
        return limit_utf8_bytes(f"임무계획 실패: {label} 읽기 실패")

    def _select_next_collab_failure_message(self, messages: List[str]) -> str:
        meaningful = [
            str(msg).strip()
            for msg in messages
            if isinstance(msg, str)
            and str(msg).strip()
            and "Using dedicated pipeline" not in str(msg)
            and "pipeline complete" not in str(msg)
        ]
        if not meaningful:
            return ""
        priority_tokens = (
            "planner failed:",
            "produced no path rows",
            "returned no valid",
            "missing",
            "not found",
            "requires",
            "failed to load",
            "failed to reserve",
            "unavailable",
            "unresolved",
            "empty",
        )
        for message in reversed(meaningful):
            lowered = message.lower()
            if any(token in lowered for token in priority_tokens):
                return message
        return meaningful[-1]

    def _build_next_collab_failure_notice(
        self,
        *,
        reason_text: str,
        detail: Optional[Dict[str, Any]] = None,
        ctx: Optional[Dict[str, Any]] = None,
        log_messages: Optional[List[str]] = None,
    ) -> str:
        detail = detail or {}
        ctx = ctx or {}
        raw_message = self._select_next_collab_failure_message(list(log_messages or []))
        message = str(raw_message or ctx.get("_next_collab_failure_reason") or "").strip()
        lowered = message.lower()

        def _starts_with(token: str) -> bool:
            return lowered.startswith(token.lower())

        if "sourcemissionplanid missing" in lowered:
            return "임무계획 실패: 원본 MP ID 없음"
        if "currentinputmissionid missing" in lowered:
            return "임무계획 실패: 현재 임무 ID 없음"
        if "targetinputmissionid missing" in lowered:
            return "임무계획 실패: 대상 임무 ID 없음"
        if "entryaircraftlist missing/empty" in lowered:
            return "임무계획 실패: 진입 기체 없음"
        if "planner aircraft entries unresolved" in lowered or "no planner aircraft entries resolved" in lowered:
            return "임무계획 실패: 진입 좌표/방향 실패"
        if "target input mission" in lowered and "not found in inputmissionplan" in lowered:
            target_input_id = (
                detail.get("targetInputMissionID")
                if isinstance(detail, dict)
                else None
            )
            if target_input_id:
                return f"임무계획 실패: 대상 임무 없음(ID={int(target_input_id)})"
            return "임무계획 실패: 대상 임무 없음"
        if "no target individual missions found" in lowered or "no source templates found" in lowered:
            return "임무계획 실패: 재구성 템플릿 없음"
        if "source missionplan has no aircraftlist" in lowered:
            return "임무계획 실패: 원본 MP 기체목록 없음"
        if "source missionplan missing inputmissionpackageid" in lowered:
            return "임무계획 실패: 원본 MP 입력ID 없음"
        if "representative entry coordinate unavailable" in lowered:
            return "임무계획 실패: 시작 좌표 없음"
        if "failed to load source missionplan" in lowered:
            return "임무계획 실패: 원본 MP 읽기 실패"
        if "failed to load inputmissionplan" in lowered:
            return "임무계획 실패: 0201 읽기 실패"
        if "failed to load individualmissionplan" in lowered:
            return "임무계획 실패: 0302 읽기 실패"
        if "failed to reserve" in lowered:
            return "임무계획 실패: ID 예약 실패"

        if "requires valid line geometry" in lowered:
            return "임무계획 실패: line 형상 오류"
        if "requires at least one aircraft entry" in lowered:
            return "임무계획 실패: 진입 기체 정보 없음"
        if "failed to produce split pieces" in lowered:
            return "임무계획 실패: line 분할 실패"
        if "produced no path rows" in lowered or "planner returned no valid path rows" in lowered:
            return "임무계획 실패: line 접근경로 없음"
        if "area replacement requires a valid mission polygon" in lowered:
            return "임무계획 실패: area 경계 오류"
        if "division planner returned no valid area path rows" in lowered or "division planner returned no final path rows" in lowered:
            return "임무계획 실패: area 경로 없음"
        if "division planner failed" in lowered:
            return "임무계획 실패: area planner 오류"

        if message:
            if ":" in message:
                _, tail = message.split(":", 1)
                tail = tail.strip()
            else:
                tail = message
            if tail:
                return limit_utf8_bytes(f"임무계획 실패: 다음협업 오류: {tail}")

        target_input_id = detail.get("targetInputMissionID") if isinstance(detail, dict) else None
        if target_input_id:
            return f"임무계획 실패: 다음협업 오류(ID={int(target_input_id)})"
        return "임무계획 실패: 다음협업 재계획 오류"

    def _build_plan_failure_notice(
        self,
        failure_code: str,
        *,
        exc: Optional[BaseException] = None,
        detail: Optional[Dict[str, Any]] = None,
        ctx: Optional[Dict[str, Any]] = None,
    ) -> str:
        detail = detail or {}
        ctx = ctx or {}

        if failure_code == "0201_missing":
            return "임무계획 실패: 0201 파일 없음"
        if failure_code == "0203_missing":
            return "임무계획 실패: 0203 파일 없음"
        if failure_code == "input_missing":
            missing_labels: list[str] = []
            if not detail.get("cmpk_path"):
                missing_labels.append("0201 입력 임무")
            if not detail.get("mrpk_path"):
                missing_labels.append("0203 비행참조정보")
            if missing_labels:
                return limit_utf8_bytes("임무계획 실패: " + ", ".join(missing_labels) + " 없음")
            return "임무계획 실패: 0201/0203 없음"
        if failure_code == "0201_load_failed":
            return self._plan_file_notice("0201 입력 임무", exc)
        if failure_code == "0203_load_failed":
            return self._plan_file_notice("0203 비행참조정보", exc)
        if failure_code == "variant_0201_load_failed":
            return self._plan_file_notice("옵션용 0201 입력 임무", exc)
        if failure_code == "input_validation_failed":
            errors = detail.get("errors") if isinstance(detail.get("errors"), list) else []
            has_0201 = False
            has_0203 = False
            for item in errors:
                key = str((item or {}).get("key") or "")
                if key.startswith("inputMission") or key == "availableAircraftList" or key == "mainSensor":
                    has_0201 = True
                if key.startswith("takeOver") or key.startswith("handOver") or key.startswith("flightArea"):
                    has_0203 = True
            if has_0201 and has_0203:
                return "임무계획 실패: 0201/0203 데이터 오류"
            if has_0201:
                return "임무계획 실패: 0201 데이터 오류"
            if has_0203:
                return "임무계획 실패: 0203 데이터 오류"
            return "임무계획 실패: 입력 데이터가 부족하거나 형식이 올바르지 않습니다."
        if failure_code == "no_available_uav":
            return _NO_AVAILABLE_UAV_NOTICE
        if failure_code in {"attack_pipeline_failed", "attack_finalize_failed", "attack_pipeline_empty"}:
            return "임무계획 실패: 공격 재계획 생성 중 오류가 발생했습니다."
        if failure_code == "next_collab_pipeline_failed":
            next_collab_notice = str(ctx.get("_next_collab_failure_notice") or "").strip()
            if next_collab_notice:
                return next_collab_notice
            return self._build_next_collab_failure_notice(detail=detail, ctx=ctx, reason_text="")
        if failure_code == "imp_generation_failed":
            return "임무계획 실패: 개별 임무 계획 생성 중 오류가 발생했습니다."
        if failure_code == "flightpath_generation_failed":
            return "임무계획 실패: 비행경로 생성에 실패했습니다."
        if failure_code == "flightpath_missing_ids":
            return "임무계획 실패: 비행경로 누락"

        exc_text = str(exc or "")
        if "No UAV available for mission planning." in exc_text or "UAV 없음 → IMP 생성 불가" in exc_text:
            return _NO_AVAILABLE_UAV_NOTICE

        if isinstance(exc, (ModuleNotFoundError, ImportError)):
            return "임무계획 실패: 라이브러리 로드 실패"
        if isinstance(exc, FileNotFoundError):
            return "임무계획 실패: 필요한 입력 파일을 찾을 수 없습니다."
        if isinstance(exc, PermissionError):
            return "임무계획 실패: 파일 접근 실패"
        if isinstance(exc, (json.JSONDecodeError, UnicodeDecodeError)):
            return "임무계획 실패: 입력 파일 형식이 올바르지 않습니다."

        try:
            replan_level = int(ctx.get("replan_level", ctx.get("replanLevel", 0)))
        except Exception:
            replan_level = 0
        if replan_level == 4:
            return "임무계획 실패: 선행임무 재계획 오류"
        return "임무계획 실패: 임무계획 처리 중 오류가 발생했습니다."

    # ───────── 0102 폴백(일반적으론 send_status_ok 사용) ─────────
    def _send_self_check_0102(self, status: int = 1, _retry: int = 0):
        if not self._0102_push_enabled():
            self._append_log_line("[0102] self-check push skipped (set KU_MMR_0102_PUSH_ENABLED=1 to enable)")
            self._emit_lifecycle(
                "self_check_skip",
                component="0102",
                outcome="skipped",
                reason="push_disabled",
            )
            return
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF → 0102 폴백 차단")
            return
        if not getattr(self, "_bus_ready", False):
            if _retry == 0:
                self._append_log_line("[0102] NodeMessenger 초기화 대기 중 ? 송신을 재시도합니다.")
            if _retry < 10:
                QTimer.singleShot(500, lambda r=_retry + 1: self._send_self_check_0102(status=status, _retry=r))
                return
            self._append_log_line("[WARN] NodeMessenger가 준비되지 않아 0102 강제 송신을 시도합니다.")
        try:
            from push_center import push_message
        except Exception as e:
            self._append_log_line(f"0102 push import 실패: {e}")
            return

        # 탭이 바디를 오버라이드해 줄 수 있으면 사용, 아니면 폴백
        try:
            body = self._tab._build_overridden_body("0102") or {}
        except Exception:
            body = {}
        if not body:
            body = self._build_0102_body(status=int(status))
        else:
            body = self._normalize_0102_body_template(body, status=int(status))
            body["timestamp"] = _now_ms_since_2000()

        try:
            push_message("0102", NodeMessenger, body_dict=body)
            self._append_log_line("자체점검(0102) 발신")
            self._self_check_sent = True
        except Exception as e:
            if _retry < 5:
                QTimer.singleShot(500, lambda: self._send_self_check_0102(status=status, _retry=_retry+1))
            else:
                self._append_log_line(f"자체점검(0102) 발신 실패: {e}")

    # ───────── 테스트 단축키 ─────────
    def _install_test_shortcuts(self):
        # 테스트: 1 → 0102 ON 토글, 0 → 0102 OFF 토글
        QShortcut(QKeySequence("1"), self, activated=lambda: self._ensure_0102(True))
        QShortcut(QKeySequence("0"), self, activated=lambda: self._ensure_0102(False))

    def _ensure_0102(self, on: bool) -> bool:
        if on and not self._0102_push_enabled():
            self._append_log_line("[0102] periodic push skipped (set KU_MMR_0102_PUSH_ENABLED=1 to enable)")
            self._emit_lifecycle(
                "heartbeat_skip",
                component="0102_heartbeat",
                outcome="skipped",
                reason="push_disabled",
            )
            return False
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF → 0102 차단")
            return False
        if on and not getattr(self, "_bus_ready", False):
            self._append_log_line("[WAIT] NodeMessenger 초기화 전 ? 0102 ON 요청을 지연합니다.")
            QTimer.singleShot(300, self._start_0102_stream)
            return False
        try:
            if self._find_tx_row("0102") < 0:
                self._append_log_line("[CTRL] TX 테이블에 0102 행이 없음"); return False
            # 0102는 GUI QTimer 대신 별도 heartbeat 스레드로 유지한다.
            # 재계획 중 UI 이벤트 큐가 밀려도 0102 공백이 생기지 않도록 한다.
            self._stop_tab_periodic_0102_if_running()
            self._set_0102_heartbeat_enabled(bool(on))
            return True
        except Exception as e:
            self._append_log_line(f"[CTRL] 0102 토글 처리 실패: {e}"); return False

    # ───────── CTRL/수신 핸들러 ─────────
    def _handle_ctrl_payload(self, payload: dict):
        import time
        if handle_window_control(self, payload, role="mission", log=self._append_log_line):
            return
        try: cmd = str(payload.get("cmd") or "")
        except Exception: return

        key = f"{cmd}:{payload.get('text') or payload.get('status')}"
        now = time.monotonic(); last = self._last_ctrl_ts.get(key, 0.0)
        if (now - last) < 1.0: return
        self._last_ctrl_ts[key] = now

        # Power OFF 상태에서는 mode 외에는 무시
        if not self._power_on and cmd not in ("mode", "db_root", "debug_db_root", "log_db_root"):
            self._append_log_line(f"[BLOCK] Power OFF → CTRL '{cmd}' 무시")
            return

        if cmd == "self_check":
            try: status = int(payload.get("status", 1))
            except Exception: status = 1
            ok = self._ensure_0102(on=(status == 1))
            if not ok:
                self._send_self_check_0102(status=status)

        elif cmd == "mode":
            text = str(payload.get("text") or "").strip()
            if self._should_ignore_ctrl_mode_change(self._resolve_mode_code_from_text(text)):
                return
            self._append_log_line(f"[CTRL] MODE change request: {text}")
            self._set_mode_slider_by_text(text)

        elif cmd in ("db_root", "debug_db_root", "log_db_root"):
            self._refresh_db_root(log_first=True)

        elif cmd == "init_plan_context":
            # 외부에서 초기 컨텍스트를 제공하는 경우(파일 경로/ID 등)
            self._stage_plan_context(payload.get("context") or {}, payload.get("trigger") or "")
            return

    # ───────── 0902(재계획 요청) 처리 ─────────
    def _parse_replan_payload(self, raw: bytes | None):
        return parse_replan_payload(raw)

    def _capture_replan_payload_for_replay(self, payload: Dict[str, Any], ctx: Dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            return
        flag = str(os.environ.get("REPLAN_CAPTURE_0902", "1") or "1").strip().lower()
        if flag in {"0", "false", "no", "off"}:
            return

        captured_path: Optional[Path] = None
        try:
            from modules.common import replan_request_transport_store

            if not replan_request_transport_store.sidecar_enabled():
                return
            path_for_payload = getattr(replan_request_transport_store, "payload_path_for_payload", None)
            if callable(path_for_payload):
                existing_path = path_for_payload(dict(payload))
                if existing_path is not None and existing_path.exists():
                    captured_path = existing_path
            if captured_path is None:
                captured_path = replan_request_transport_store.save_payload(dict(payload))
        except Exception:
            captured_path = None

        if captured_path is None:
            request_time = payload.get("replanRequestTime")
            timestamp_value = payload.get("timestamp")
            if isinstance(request_time, dict):
                timestamp_value = request_time.get("replanRequestTimestamp", timestamp_value)
            try:
                timestamp_ms = int(timestamp_value)
            except Exception:
                timestamp_ms = int(time.time() * 1000)
            plan_ids = []
            for value in ctx.get("plan_ids") or []:
                try:
                    plan_ids.append(str(int(value)))
                except Exception:
                    continue
            plan_token = "-".join(plan_ids) if plan_ids else "none"
            try:
                archive_dir = db_paths.get_db_subpath("DSS_Internal", "replan_request_archive")
                captured_path = archive_dir / f"0902_{timestamp_ms}_plans_{plan_token}.json"
                write_json(
                    captured_path,
                    {
                        "messageID": "0902",
                        "archivedAt": datetime.now(timezone.utc).isoformat(),
                        "captureSource": "mission_planning_gui",
                        "context": {
                            "reason": ctx.get("reason"),
                            "replan_level": ctx.get("replan_level"),
                            "plan_ids": list(ctx.get("plan_ids") or []),
                            "mission_ids": list(ctx.get("mission_ids") or []),
                            "option_names": list(ctx.get("option_names") or []),
                        },
                        "payload": payload,
                    },
                    pretty=False,
                    ensure_ascii=False,
                    skip_if_unchanged=False,
                )
            except Exception as exc:
                self._append_log_line(f"[WARN] 0902 replay capture failed: {exc}")
                return

        if captured_path is not None:
            ctx["_0902_capture_path"] = str(captured_path)
            self._record_replan_timing_event(
                "0902_archived",
                ctx=ctx,
                extra={"path": captured_path.name},
            )

    def _stage_plan_context(self, raw_context: dict, trigger: str = ""):
        """외부에서 사전 컨텍스트를 주입(파일 경로/ID/옵션명 등)."""
        if not isinstance(raw_context, dict):
            self._append_log_line("[CTRL] init_plan_context ignored: invalid payload")
            return

        ctx: dict = {}
        # plan_ids
        plan_ids: list[int] = []
        for v in raw_context.get("plan_ids", []):
            try: plan_ids.append(int(v))
            except Exception: pass

        # mission_ids(필요시)
        mission_ids: list[int] = []
        for v in raw_context.get("mission_ids", []):
            try: mission_ids.append(int(v))
            except Exception: pass

        # option_names
        option_names: list[str] = []
        for name in raw_context.get("option_names", []):
            if name is not None:
                option_names.append(str(name))
        while len(option_names) < len(plan_ids):
            option_names.append(f"option{len(option_names) + 1}")

        if plan_ids:     ctx["plan_ids"] = plan_ids
        if mission_ids:  ctx["mission_ids"] = mission_ids
        if option_names: ctx["option_names"] = option_names

        # 입력 파일 경로(선택)
        for key in ("cmpk_path", "mrpk_path"):
            value = raw_context.get(key)
            if isinstance(value, str) and value.strip():
                ctx[key] = value.strip()

        ctx["reason"] = _sanitize_reason(raw_context.get("reason"), "init-plan")
        try: ctx["replan_level"] = int(raw_context.get("replan_level", 1))
        except Exception: ctx["replan_level"] = 1

        if raw_context.get("fallback_plan_id") is not None:
            try: ctx["fallback_plan_id"] = int(raw_context.get("fallback_plan_id"))
            except Exception: pass

        self._staged_plan_context = ctx
        summary = ", ".join(str(pid) for pid in ctx.get("plan_ids", [])) or "-"
        note = f"[CTRL] init_plan_context received (planIds={summary})"
        if trigger: note += f" trigger={trigger}"
        self._append_log_line(note)

    def _handle_replan_received(self, msg_id, raw):
        received_perf = time.perf_counter()
        received_wall_ms = int(time.time() * 1000)
        try:
            return self._handle_replan_received_impl(
                msg_id,
                raw,
                received_perf=received_perf,
                received_wall_ms=received_wall_ms,
            )
        except Exception as exc:
            self._append_log_line(f"[ERR] 0902 handling failed before pipeline scheduling: {exc}")
            try:
                trace_text = traceback.format_exc().strip()
                if trace_text:
                    self._append_log_line("[TRACE] " + trace_text)
            except Exception:
                pass
            return

    def _handle_replan_received_impl(
        self,
        msg_id,
        raw,
        *,
        received_perf: Optional[float] = None,
        received_wall_ms: Optional[int] = None,
    ):
        """탭에서 0902 수신 시 호출해주는 콜백."""
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF → 0902 수신 무시")
            return

        payload = self._parse_replan_payload(raw)
        if not payload:
            self._append_log_line("[ERR] 0902 payload parse failed")
            return
        staged = self._staged_plan_context if isinstance(getattr(self, '_staged_plan_context', {}), dict) else {}

        selection = extract_replan_request_selection(payload)
        detail_payload = selection.detail
        detail_trigger = selection.detail_trigger_type
        plan_ids = list(selection.plan_ids)
        option_names = list(selection.option_names)
        mission_ids = list(selection.mission_ids)

        staged_reason = _sanitize_reason(staged.get("reason"), "init-plan")
        reason = _sanitize_reason(payload.get("replanRequest") or payload.get("replanReason"), staged_reason)

        ctx = dict(staged)
        if plan_ids:
            ctx["plan_ids"] = plan_ids
        if option_names:
            ctx["option_names"] = option_names
        if mission_ids:
            ctx["mission_ids"] = mission_ids
        ctx["reason"] = reason
        if detail_payload is not None:
            ctx["replan_detail"] = detail_payload
            if isinstance(detail_payload, dict):
                for key_name in ("sourceMissionPlanID", "currentMissionPlanID"):
                    value = self._to_optional_int(detail_payload.get(key_name))
                    if value is not None and value > 0:
                        ctx[key_name] = int(value)
                if (not self._is_post_attack_rejoin_detail(detail_payload)) and self._prepare_follow_up_attack_detail(detail_payload):
                    for key_name in ("sourceMissionPlanID", "currentMissionPlanID"):
                        value = self._to_optional_int(detail_payload.get(key_name))
                        if value is not None and value > 0:
                            ctx[key_name] = int(value)
                if self._is_post_attack_rejoin_detail(detail_payload):
                    ctx["force_direct_update"] = True
                    ctx["suppress_0702_fallback"] = True
        try:
            ctx["replan_level"] = int(payload.get("replanLevel", ctx.get("replan_level", 1)))
        except Exception:
            ctx["replan_level"] = ctx.get("replan_level", 1)
        if _is_quality_speed_trigger_type(detail_trigger) or _is_quality_speed_reason_text(reason):
            ctx["option_names"] = []
            ctx["force_direct_update"] = True
            ctx["suppress_0702_fallback"] = True
        for field_name in ("inputMissionPackageID", "missionReferencePackageID"):
            field_value = payload.get(field_name)
            if field_value is None and isinstance(detail_payload, dict):
                field_value = detail_payload.get(field_name)
            if field_value is None:
                continue
            try:
                ctx[field_name] = int(field_value)
            except Exception:
                pass
        review_0204_presend_value = payload.get(_INPUT_0201_REVIEW_0204_SENT_FLAG)
        if review_0204_presend_value is None and isinstance(detail_payload, dict):
            review_0204_presend_value = detail_payload.get(_INPUT_0201_REVIEW_0204_SENT_FLAG)
        review_0204_presend = (
            review_0204_presend_value is True
            or str(review_0204_presend_value).strip().lower() in {"1", "true", "yes", "on"}
        )
        if review_0204_presend:
            ctx[_INPUT_0201_REVIEW_0204_SENT_FLAG] = True
            reviewed_pkg_id = self._to_optional_int(ctx.get("inputMissionPackageID"))
            if reviewed_pkg_id is not None and reviewed_pkg_id > 0:
                try:
                    self._review_0204_sent_package_ids.add(int(reviewed_pkg_id))
                except Exception:
                    self._review_0204_sent_package_ids = {int(reviewed_pkg_id)}
                self._append_log_line(
                    f"[0204] 0902 marked reviewed package pre-sent: "
                    f"inputMissionPackageID={int(reviewed_pkg_id)}"
                )
        if payload.get("fallbackPlanId") is not None:
            try: ctx["fallback_plan_id"] = int(payload.get("fallbackPlanId"))
            except Exception: pass
        try:
            replan_reason_text = str(ctx.get("reason") or "")
        except Exception:
            replan_reason_text = ""
        if _is_path_deviation_reason_text(replan_reason_text):
            self._log_path_deviation_event(
                "mission_0902_received",
                {
                    "payloadKeys": sorted(payload.keys()),
                    "reason": replan_reason_text,
                    "planIDs": list(ctx.get("plan_ids") or []),
                    "optionNames": list(ctx.get("option_names") or []),
                    "hasReplanDetail": isinstance(detail_payload, dict),
                    "detailKeys": sorted(detail_payload.keys()) if isinstance(detail_payload, dict) else [],
                },
            )
        if _is_imaging_schedule_reason_text(replan_reason_text):
            self._log_imaging_schedule_event(
                "mission_0902_received",
                {
                    "payloadKeys": sorted(payload.keys()),
                    "reason": replan_reason_text,
                    "planIDs": list(ctx.get("plan_ids") or []),
                    "optionNames": list(ctx.get("option_names") or []),
                    "hasReplanDetail": isinstance(detail_payload, dict),
                    "detailKeys": sorted(detail_payload.keys()) if isinstance(detail_payload, dict) else [],
                },
            )

        try:
            self._start_replan_timing(
                ctx,
                payload,
                received_perf=received_perf,
                received_wall_ms=received_wall_ms,
            )
        except Exception as exc:
            self._append_log_line(f"[WARN] 0902 timing start skipped: {exc}")
        try:
            self._schedule_replan_terrain_warmup(ctx, payload)
        except Exception as exc:
            self._append_log_line(f"[WARN] 0902 DEM warm-up schedule skipped: {exc}")
        summary = ", ".join(str(pid) for pid in ctx.get("plan_ids", [])) or "-"
        self._append_log_line(f"[AUTO] 0902 received (planIds={summary})")
        delay_ms = self._replan_delay_ms_for_payload(payload)
        if getattr(self, "_initplan_running", False):
            self._queue_deferred_replan_request(ctx, delay_ms=delay_ms)
            try:
                self._capture_replan_payload_for_replay(payload, ctx)
            except Exception as exc:
                self._append_log_line(f"[WARN] 0902 replay capture skipped: {exc}")
            return

        # 새 0902 요청 수신 시 이전 전송 버퍼 초기화
        self._pending_plan_push = None
        try:
            self._attack_delivery_buffer.clear()
        except Exception:
            self._attack_delivery_buffer = []

        self._active_plan_context = ctx
        self._schedule_replan_pipeline(delay_ms=delay_ms)
        try:
            self._capture_replan_payload_for_replay(payload, ctx)
        except Exception as exc:
            self._append_log_line(f"[WARN] 0902 replay capture skipped: {exc}")

    def _replan_delay_ms_for_payload(self, payload: Dict[str, Any]) -> int:
        policy = replan_delay_policy(payload)
        if policy.runtime_setting_key:
            return self._runtime_replan_delay_ms(policy.runtime_setting_key, policy.default_delay_ms)
        return int(policy.default_delay_ms)

    def _runtime_replan_delay_ms(self, key: str, default: int) -> int:
        try:
            values = (load_runtime_settings().get("values") or {})
            raw = values.get(key, default) if isinstance(values, dict) else default
            return max(0, int(float(raw)))
        except Exception:
            return max(0, int(default))

    def _queue_deferred_replan_request(self, ctx: Dict[str, Any], *, delay_ms: int) -> None:
        try:
            normalized_delay_ms = max(0, int(delay_ms))
        except Exception:
            normalized_delay_ms = 0
        queue = getattr(self, "_deferred_replan_requests", None)
        if not isinstance(queue, list):
            queue = []
        queue.append(
            {
                "ctx": copy.deepcopy(dict(ctx or {})),
                "due_at": time.monotonic() + (normalized_delay_ms / 1000.0),
            }
        )
        try:
            queue.sort(key=lambda item: float(item.get("due_at") or 0.0))
        except Exception:
            pass
        self._deferred_replan_requests = queue
        summary = ", ".join(str(pid) for pid in (ctx.get("plan_ids") or [])) or "-"
        self._record_replan_timing_event(
            "deferred_queued",
            ctx=ctx,
            extra={"delay_ms": normalized_delay_ms, "queued": len(queue)},
        )
        self._append_log_line(
            f"[AUTO] 0902 deferred while replan pipeline running (planIds={summary}, queued={len(queue)})"
        )

    def _resume_deferred_replan_request(self) -> None:
        if self._initplan_running:
            return
        queue = getattr(self, "_deferred_replan_requests", None)
        if not isinstance(queue, list) or not queue:
            return
        item = queue.pop(0)
        self._deferred_replan_requests = queue
        ctx = item.get("ctx") if isinstance(item, dict) else {}
        if not isinstance(ctx, dict):
            ctx = {}
        self._active_plan_context = dict(ctx)
        due_at = item.get("due_at") if isinstance(item, dict) else None
        try:
            due_at_value = float(due_at)
        except Exception:
            due_at_value = 0.0
        remaining_delay_ms = max(0, int(round((due_at_value - time.monotonic()) * 1000.0)))
        summary = ", ".join(str(pid) for pid in (ctx.get("plan_ids") or [])) or "-"
        self._record_replan_timing_event(
            "deferred_resumed",
            extra={"delay_ms": remaining_delay_ms, "queued": len(queue)},
        )
        self._append_log_line(
            "[AUTO] deferred 0902 resumed "
            f"(planIds={summary}, queued={len(queue)}, delay={remaining_delay_ms}ms)"
        )
        self._schedule_replan_pipeline(delay_ms=remaining_delay_ms)

    def _should_use_attack_pipeline(self, ctx: Dict[str, Any]) -> bool:
        return should_use_attack_pipeline(ctx)

    def _should_use_post_attack_rejoin_pipeline(self, ctx: Dict[str, Any]) -> bool:
        return should_use_post_attack_rejoin_pipeline(ctx)

    def _should_use_prior_post_rejoin_pipeline(self, ctx: Dict[str, Any]) -> bool:
        return should_use_prior_post_rejoin_pipeline(ctx)

    def _ensure_ctx_package_ids(self, ctx: Dict[str, Any], staged: Dict[str, Any]) -> None:
        """replan 파이프라인 진입 전에 필수 패키지 ID를 미리 채워둔다."""

        def _coerce_positive_int(value: Any) -> Optional[int]:
            if value is None:
                return None
            try:
                ivalue = int(value)
            except Exception:
                return None
            return ivalue if ivalue > 0 else None

        source_package_ids: Dict[str, int] = {}
        source_plan_candidates: List[Any] = []
        for container in (
            ctx.get("replan_detail"),
            ctx,
            staged.get("replan_detail"),
            staged,
        ):
            if not isinstance(container, dict):
                continue
            for source_key in ("sourceMissionPlanID", "currentMissionPlanID", "missionPlanID"):
                source_plan_candidates.append(container.get(source_key))
        for source_plan_candidate in source_plan_candidates:
            source_package_ids = _load_source_plan_package_ids(source_plan_candidate)
            if source_package_ids:
                break

        def _ensure_id(key: str, snapshot_id: str) -> None:
            existing = _coerce_positive_int(ctx.get(key))
            if existing is None:
                existing = _coerce_positive_int(staged.get(key))
                if existing is not None:
                    ctx[key] = existing
                    return
            if existing is not None:
                ctx[key] = existing
                return
            source_value = _coerce_positive_int(source_package_ids.get(key))
            if source_value is not None:
                ctx[key] = source_value
                self.log_sig.emit(
                    f"[INFO] Using source MissionPlan {source_package_ids.get('sourceMissionPlanID')} "
                    f"package ID {source_value} for {key}"
                )
                return
            try:
                latest = get_latest_package_id(snapshot_id)
            except Exception as exc:
                self.log_sig.emit(f"[WARN] Failed to query latest {snapshot_id} ID for {key}: {exc}")
                return
            if latest is not None:
                ctx[key] = latest
                self.log_sig.emit(f"[INFO] Using latest {snapshot_id} ID {latest} for {key} (fallback)")

        _ensure_id("inputMissionPackageID", "0201")
        _ensure_id("missionReferencePackageID", "0203")

    # ───────── 재계획 파이프라인(파일 생성/저장 후 0301만 송신) ─────────
    def _schedule_replan_pipeline(self, delay_ms: int = 1000) -> None:
        """Delay the replan pipeline start to avoid race conditions with rapid 0902 dispatch."""
        try:
            delay_log_ms = int(delay_ms)
        except Exception:
            delay_log_ms = 0
        self._record_replan_timing_event("pipeline_scheduled", extra={"delay_ms": delay_log_ms})
        timer = getattr(self, "_replan_delay_timer", None)
        if timer is not None:
            timer.stop()
            timer.deleteLater()
        if delay_ms <= 0:
            self._replan_delay_timer = None
            self._run_replan_pipeline_async()
            return
        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.setInterval(int(delay_ms))
        timer.timeout.connect(self._run_replan_pipeline_async)
        self._replan_delay_timer = timer
        timer.start()
        self._append_log_line(f"[AUTO] replan pipeline scheduled after {delay_ms/1000:.1f}s delay")

    def _run_replan_pipeline_async(self):
        timer = getattr(self, "_replan_delay_timer", None)
        if timer is not None:
            timer.stop()
            timer.deleteLater()
            self._replan_delay_timer = None
        ctx = getattr(self, "_active_plan_context", {}) or {}
        reason = _sanitize_reason(ctx.get("reason"), "초기임무재계획")
        plan_logger = getattr(self, "_mission_plan_logger", None)
        if not self._power_on:
            msg = "[BLOCK] Power OFF 상태에서 replan pipeline 차단"
            self._append_log_line(msg)
            try:
                if plan_logger:
                    plan_logger.log_blocked(ctx, reason, msg)
            except Exception:
                pass
            return
        if self._initplan_running:
            msg = "[INFO] replan pipeline already running"
            self._append_log_line(msg)
            try:
                if plan_logger:
                    plan_logger.log_blocked(ctx, reason, msg)
            except Exception:
                pass
            return
        no_uav_guard = self._evaluate_no_available_uav_replan_guard(ctx)
        if no_uav_guard is not None:
            msg = self._format_no_available_uav_replan_guard_log(no_uav_guard)
            self._append_log_line(msg)
            try:
                if plan_logger:
                    plan_logger.log_blocked(ctx, reason, msg)
            except Exception:
                pass
            self._push_replan_failure_completion(_NO_AVAILABLE_UAV_NOTICE)
            try:
                self._plan_status = "임무계획 실패"
                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
            except Exception:
                pass
            return
        self._initplan_running = True
        self._pending_plan_push = None
        self._record_replan_timing_event("scheduled_start", ctx=ctx)
        self._push_0305(status=1, reason=reason)
        session_id = self._pipeline_logger.open_session(ctx, reason)
        plan_run_log = None
        try:
            if plan_logger:
                plan_run_log = plan_logger.start_run(ctx, reason, session_id=session_id)
        except Exception:
            plan_run_log = None
        threading.Thread(
            target=self._run_replan_pipeline_thread_entry,
            args=(session_id, plan_run_log),
            name="Replan-GUI",
            daemon=True,
        ).start()

    def _run_replan_pipeline_thread_entry(self, session_id: Optional[str], plan_run_log=None):
        # Emit from inside the worker so a very fast stop/exception can never
        # overtake the start event and leave diagnostics in the fast window.
        self._emit_lifecycle(
            "worker_thread_start",
            component="replan_pipeline_thread",
            outcome="ok",
            extra={"sessionId": session_id},
        )
        try:
            self._run_replan_pipeline_do(session_id, plan_run_log)
        except Exception as exc:
            try:
                self.log_sig.emit(f"[ERR] Replan pipeline thread crashed: {exc}")
                trace_text = traceback.format_exc().strip()
                if trace_text:
                    self.log_sig.emit("[TRACE] " + trace_text)
            except Exception:
                pass
            try:
                self._pipeline_logger.log_event(session_id, "error", f"Replan pipeline thread crashed: {exc}")
            except Exception:
                pass
            self._emit_lifecycle(
                "worker_thread_exception",
                component="replan_pipeline_thread",
                outcome="failure",
                reason=str(exc),
                extra={"sessionId": session_id},
            )
            try:
                self._push_replan_failure_completion(f"재계획 실패: {exc}")
            except Exception:
                pass
            try:
                self._plan_status = "임무계획 실패"
                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
            except Exception:
                pass
            self._initplan_running = False
            try:
                self._mission_plan_logger.clear_active()
            except Exception:
                pass

    def _run_replan_pipeline_do(self, session_id: Optional[str], plan_run_log=None):
        success = False
        plan_log_status = "error"
        plan_log_stop_reason: Optional[str] = None
        plan_log_summary: Dict[str, Any] = {}
        summary_info: Optional[Dict[str, Any]] = None
        plan_log = plan_run_log
        ctx: Dict[str, Any] = {}
        staged: Dict[str, Any] = {}
        failure_notice_sent = False
        reason = "init-plan"
        source_cache = SourceArtifactCache()
        source_cache_scope = None
        attack_exclusion_executor: concurrent.futures.ThreadPoolExecutor | None = None
        attack_exclusion_future: concurrent.futures.Future | None = None
        attack_exclusion_parallel_info: Dict[str, Any] = {}
        try:
            import os, json
            from pathlib import Path

            source_cache_scope = use_source_artifact_cache(source_cache)
            source_cache_scope.__enter__()
            try:
                clear_runtime_camera_fov_adjustment_logs()
            except Exception:
                pass
            ctx = getattr(self, '_active_plan_context', {}) or {}
            staged = self._staged_plan_context if isinstance(getattr(self, '_staged_plan_context', {}), dict) else {}
            self._pending_plan_push = None
            try:
                self._attack_delivery_buffer.clear()
            except Exception:
                self._attack_delivery_buffer = []
            self._ensure_ctx_package_ids(ctx, staged)
            staged_reason = _sanitize_reason(staged.get('reason'), 'init-plan')
            reason = _sanitize_reason(ctx.get('reason'), staged_reason)

            plan_log = plan_run_log
            if plan_log is None:
                try:
                    plan_log = self._mission_plan_logger.start_run(ctx, reason, session_id=session_id)
                except Exception:
                    plan_log = None
            if plan_log:
                self._active_plan_log_run = plan_log
                plan_log.add_step(
                    "start",
                    "info",
                    detail={
                        "reason": reason,
                        "plan_ids": list(ctx.get("plan_ids") or []),
                        "replan_level": ctx.get("replan_level", ctx.get("replanLevel")),
                    },
                )
            def _record_step(name: str, status: str, detail: Optional[Dict[str, Any]] = None, message: str = "") -> None:
                if plan_log:
                    plan_log.add_step(name, status, detail=detail, message=message)

            def _record_issue(code: str, message: str = "", detail: Optional[Dict[str, Any]] = None, *, status: str = "error") -> None:
                nonlocal plan_log_status, plan_log_stop_reason
                plan_log_stop_reason = plan_log_stop_reason or code
                plan_log_status = status or plan_log_status
                if plan_log:
                    if message or detail:
                        plan_log.add_issue(code, message=message or None, detail=detail)
                    plan_log.add_step(code, status or "error", detail=detail, message=message)

            def _notify_failure_once(
                code: str,
                *,
                exc: Optional[BaseException] = None,
                detail: Optional[Dict[str, Any]] = None,
            ) -> None:
                nonlocal failure_notice_sent
                if failure_notice_sent:
                    return
                notice = self._build_plan_failure_notice(code, exc=exc, detail=detail, ctx=ctx)
                if not notice:
                    return
                self._push_replan_failure_completion(notice)
                failure_notice_sent = True

            no_uav_guard = self._evaluate_no_available_uav_replan_guard(ctx)
            if no_uav_guard is not None:
                self.log_sig.emit(self._format_no_available_uav_replan_guard_log(no_uav_guard))
                _record_issue(
                    "no_available_uav",
                    "No UAV available for replanning.",
                    detail=no_uav_guard,
                    status="error",
                )
                plan_log_summary.update({"stop_reason": "no_available_uav", **no_uav_guard})
                _notify_failure_once("no_available_uav", detail=no_uav_guard)
                self._plan_status = "임무계획 실패"
                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                return

            self.log_sig.emit(f"[STEP 0] Replan pipeline start (reason={reason})")
            self._pipeline_logger.log_event(
                session_id,
                "info",
                "Replan pipeline start",
                detail={
                    "reason": reason,
                    "plan_ids": list(ctx.get("plan_ids") or []),
                    "replan_level": ctx.get("replan_level"),
                },
            )

            def _collect_attack_option_indices(context: Dict[str, Any]) -> list[int]:
                labels = list(context.get("option_names") or [])
                return [
                    idx for idx, label in enumerate(labels)
                    if is_option_code_value(label, 2)
                ]

            def _collect_attack_exclusion_option_indices(context: Dict[str, Any]) -> list[int]:
                labels = list(context.get("option_names") or [])
                return [
                    idx for idx, label in enumerate(labels)
                    if is_option_code_value(label, 3)
                ]

            def _select_by_indices(values: list[Any], indices: list[int]) -> list[Any]:
                seq = list(values or [])
                return [seq[idx] if idx < len(seq) else None for idx in indices]

            def _filter_context_by_indices(context: Dict[str, Any], keep_indices: list[int]) -> Dict[str, Any]:
                filtered = dict(context)
                plan_ids = list(context.get("plan_ids") or [])
                option_names = list(context.get("option_names") or [])
                if keep_indices:
                    filtered["plan_ids"] = _select_by_indices(plan_ids, keep_indices)
                    filtered["option_names"] = _select_by_indices(option_names, keep_indices)
                else:
                    filtered.pop("plan_ids", None)
                    filtered.pop("option_names", None)
                return filtered

            def _load_imp_package(imp_path: str | Path) -> tuple[int, dict]:
                pkg = source_cache.read_json(imp_path, kind="IndividualMissionPlan")
                aid = int(pkg.get("aircraftID", 0))
                return aid, pkg

            def _load_imp_packages(imp_paths: list[str]) -> list[tuple[int, dict]]:
                if len(imp_paths) <= 1:
                    return [_load_imp_package(path) for path in imp_paths]
                loaded: list[tuple[int, dict] | None] = [None] * len(imp_paths)
                with concurrent.futures.ThreadPoolExecutor(
                    max_workers=min(6, len(imp_paths)),
                    thread_name_prefix="LoadIMP",
                ) as executor:
                    futures = {
                        executor.submit(_load_imp_package, path): idx
                        for idx, path in enumerate(imp_paths)
                    }
                    for future in concurrent.futures.as_completed(futures):
                        loaded[futures[future]] = future.result()
                return [row for row in loaded if row is not None]

            def _write_json_batch(
                rows: list[tuple[Path, Any]],
                *,
                pretty: bool = True,
                max_workers: Optional[int] = None,
            ) -> int:
                if not rows:
                    return 0

                def _write_one(item: tuple[Path, Any]) -> bool:
                    path, payload = item
                    return bool(
                        write_json(
                            path,
                            payload,
                            pretty=pretty,
                            ensure_ascii=False,
                            skip_if_unchanged=True,
                        )
                    )

                if len(rows) == 1:
                    return 1 if _write_one(rows[0]) else 0

                written = 0
                try:
                    worker_cap = max(1, int(max_workers)) if max_workers is not None else 8
                except Exception:
                    worker_cap = 8
                with concurrent.futures.ThreadPoolExecutor(
                    max_workers=min(worker_cap, len(rows)),
                    thread_name_prefix="WriteJSON",
                ) as executor:
                    futures = [executor.submit(_write_one, row) for row in rows]
                    for future in concurrent.futures.as_completed(futures):
                        if future.result():
                            written += 1
                return written

            def _serialize_json_batch(
                rows: list[tuple[Path, Any]],
                *,
                pretty: bool = True,
                max_workers: Optional[int] = None,
            ) -> list[tuple[Path, bytes]]:
                if not rows:
                    return []

                def _serialize_one(item: tuple[Path, Any]) -> tuple[Path, bytes]:
                    path, payload = item
                    return (
                        Path(path),
                        serialize_json_payload(
                            Path(path),
                            payload,
                            pretty=pretty,
                            ensure_ascii=False,
                        ),
                    )

                if len(rows) == 1:
                    return [_serialize_one(rows[0])]

                try:
                    worker_cap = max(1, int(max_workers)) if max_workers is not None else 8
                except Exception:
                    worker_cap = 8
                if worker_cap <= 1:
                    return [_serialize_one(row) for row in rows]
                serialized: list[tuple[Path, bytes] | None] = [None] * len(rows)
                with concurrent.futures.ThreadPoolExecutor(
                    max_workers=min(worker_cap, len(rows)),
                    thread_name_prefix="SerializeJSON",
                ) as executor:
                    futures = {
                        executor.submit(_serialize_one, row): idx
                        for idx, row in enumerate(rows)
                    }
                    for future in concurrent.futures.as_completed(futures):
                        serialized[futures[future]] = future.result()
                return [row for row in serialized if row is not None]

            def _write_json_bytes_batch_results(
                rows: list[tuple[Path, bytes]],
                *,
                max_workers: Optional[int] = None,
                skip_if_unchanged: bool = True,
            ) -> list[bool]:
                if not rows:
                    return []

                def _write_one(item: tuple[Path, bytes]) -> bool:
                    path, payload = item
                    return bool(
                        write_json_bytes(
                            Path(path),
                            payload,
                            skip_if_unchanged=skip_if_unchanged,
                        )
                    )

                if len(rows) == 1:
                    return [_write_one(rows[0])]

                try:
                    worker_cap = max(1, int(max_workers)) if max_workers is not None else 8
                except Exception:
                    worker_cap = 8
                results: list[bool | None] = [None] * len(rows)
                with concurrent.futures.ThreadPoolExecutor(
                    max_workers=min(worker_cap, len(rows)),
                    thread_name_prefix="WriteJSONBytes",
                ) as executor:
                    futures = {
                        executor.submit(_write_one, row): idx
                        for idx, row in enumerate(rows)
                    }
                    for future in concurrent.futures.as_completed(futures):
                        results[futures[future]] = bool(future.result())
                return [bool(row) for row in results]

            def _write_json_bytes_batch(
                rows: list[tuple[Path, bytes]],
                *,
                max_workers: Optional[int] = None,
            ) -> int:
                return sum(
                    1
                    for written in _write_json_bytes_batch_results(
                        rows,
                        max_workers=max_workers,
                    )
                    if written
                )

            def _repair_missing_flight_path_files(
                *,
                variant_no: int,
                dir_fp: Path,
                imp_pkgs: list[dict],
                flight_plans_0303: list[dict],
                flight_plans_0304: list[dict],
            ) -> tuple[int, list[int]]:
                repair_plan = collect_missing_flight_path_repairs(
                    flight_path_dir=dir_fp,
                    individual_mission_plans=imp_pkgs,
                    flight_plans_0303=flight_plans_0303,
                    flight_plans_0304=flight_plans_0304,
                    scope=f"variant={variant_no}",
                )
                referenced_path_ids = set(repair_plan.get("referencedPathIDs") or [])
                missing_ids = list(repair_plan.get("missingPathIDs") or [])
                repaired = 0
                if missing_ids:
                    repair_rows = repair_plan.get("repairRows") or []
                    if repair_rows:
                        repaired = _write_json_batch(repair_rows, pretty=True)
                        if repaired:
                            self.log_sig.emit(
                                f"[WARN] FlightPath repair attempted (variant={variant_no}): rewrote={repaired}"
                            )
                    missing_ids = sorted(
                        pid for pid in referenced_path_ids if not (dir_fp / f"{int(pid)}.json").exists()
                    )
                return repaired, missing_ids

            def _sync_flight_plan_individual_mission_ids(
                *,
                variant_no: int,
                imp_pkgs: list[dict],
                flight_plans_0303: list[dict],
                flight_plans_0304: list[dict],
            ) -> int:
                summary = sync_flight_plan_individual_mission_ids(
                    individual_mission_plans=imp_pkgs,
                    flight_plans_0303=flight_plans_0303,
                    flight_plans_0304=flight_plans_0304,
                    scope=f"variant={variant_no}",
                )
                fixed = int(summary.get("fixed") or 0)
                if fixed:
                    self.log_sig.emit(
                        f"[INFO] FlightPath individualMissionID synced (variant={variant_no}): fixed={fixed}"
                    )
                return fixed

            def _validate_unique_flightpath_ids(
                *,
                variant_no: int,
                flight_plans_0303: list[dict],
                flight_plans_0304: list[dict],
            ) -> None:
                try:
                    validate_unique_flightpath_ids(
                        flight_plans_0303=flight_plans_0303,
                        flight_plans_0304=flight_plans_0304,
                        scope=f"variant={variant_no}",
                    )
                except ReplanValidationError as exc:
                    message = "; ".join(exc.errors)
                    self.log_sig.emit(f"[ERR] {message}")
                    raise RuntimeError(message)

            def _validate_mission_flightpath_links(
                *,
                variant_no: int,
                missions: list[dict],
                flight_plans_0303: list[dict],
                flight_plans_0304: list[dict],
            ) -> None:
                try:
                    validate_mission_flightpath_links(
                        missions=missions,
                        flight_plans_0303=flight_plans_0303,
                        flight_plans_0304=flight_plans_0304,
                        scope=f"variant={variant_no}",
                    )
                except ReplanValidationError as exc:
                    message = "; ".join(exc.errors)
                    self.log_sig.emit(f"[ERR] {message}")
                    raise RuntimeError(message)

            def _initial_plan_template_allowed(
                *,
                reason_text: str,
                plan_count_value: int,
                option_code_value: int,
                attack_selected: bool,
                attack_exclusion_selected_value: bool,
                prior_context: Optional[Dict[str, Any]],
                current_request: Optional[CurrentRemainingHybridRequest],
            ) -> bool:
                if str(os.environ.get("DSS_INITIAL_PLAN_TEMPLATE_CACHE", "1")).strip().lower() in {"0", "false", "off", "no"}:
                    return False
                if str(reason_text or "").strip() != "초기임무재계획":
                    return False
                if int(plan_count_value or 0) != 1:
                    return False
                if int(option_code_value or 0) != 1:
                    return False
                if attack_selected or attack_exclusion_selected_value or prior_context is not None:
                    return False
                if current_request is not None:
                    return False
                return True

            def _type1_initial_lah_hold_by_aircraft(
                *,
                cmpk_source: Path,
                current_request: Optional[CurrentRemainingHybridRequest],
                log_emit: Callable[[str], None],
            ) -> Dict[int, Dict[str, float]]:
                """Resolve TO-centred LAH route starts only for a genuine initial plan."""
                if str(reason or "").strip() != "초기임무재계획":
                    return {}
                if input_refresh_context or current_request is not None:
                    return {}
                try:
                    source_payload = source_cache.read_json(
                        Path(cmpk_source),
                        kind="InputMissionPlan",
                    )
                    package_type = int(
                        source_payload.get("inputMissionPackageType", 0) or 0
                    )
                    if package_type not in d0304.lah_initial_to_start_package_types():
                        return {}
                    holds = d0304.lah_initial_hold_by_aircraft_from_mrpk(mrpk_data)
                except Exception as exc:
                    log_emit(f"[WARN] [LAH][INITIAL-TRANSIT] TO 기준점 해석 실패; 기존 시작점 사용: {exc}")
                    return {}
                if holds:
                    formatted = ", ".join(
                        f"AC{int(aid)}=({float(coord['latitude']):.7f},{float(coord['longitude']):.7f})"
                        for aid, coord in sorted(holds.items())
                    )
                    log_emit(
                        f"[INFO] [LAH][INITIAL-TRANSIT] Type{package_type} 초기계획 TO 편대 출발점 적용: "
                        + formatted
                    )
                else:
                    log_emit(
                        f"[WARN] [LAH][INITIAL-TRANSIT] Type{package_type} 초기계획이지만 유효한 TakeOver 기준점이 없어 "
                        "기존 임무 시작점을 사용합니다."
                    )
                return holds

            def _reassign_cached_flightpath_waypoints(
                flight_plans: list[dict],
                wp_alloc: Any,
                *,
                waypoint_keys: tuple[str, ...],
            ) -> int:
                if wp_alloc is None or not hasattr(wp_alloc, "alloc"):
                    raise RuntimeError("Waypoint allocator unavailable for initial plan template materialization.")
                refs: list[dict] = []
                for fp in flight_plans or []:
                    if not isinstance(fp, dict):
                        continue
                    waypoints = None
                    for key in waypoint_keys:
                        candidate = fp.get(key)
                        if isinstance(candidate, list):
                            waypoints = candidate
                            break
                    if not isinstance(waypoints, list) or not waypoints:
                        continue
                    for wp in waypoints:
                        if isinstance(wp, dict):
                            refs.append(wp)
                for wp in refs:
                    wp["waypointID"] = int(wp_alloc.alloc())
                for fp in flight_plans or []:
                    if not isinstance(fp, dict):
                        continue
                    waypoints = None
                    for key in waypoint_keys:
                        candidate = fp.get(key)
                        if isinstance(candidate, list):
                            waypoints = candidate
                            break
                    if not isinstance(waypoints, list) or not waypoints:
                        continue
                    for idx_wp in range(len(waypoints) - 1):
                        if isinstance(waypoints[idx_wp], dict) and isinstance(waypoints[idx_wp + 1], dict):
                            waypoints[idx_wp]["nextWaypointID"] = int(
                                waypoints[idx_wp + 1].get("waypointID", 0) or 0
                            )
                    if isinstance(waypoints[-1], dict):
                        waypoints[-1]["nextWaypointID"] = 0
                return len(refs)

            def _materialize_initial_plan_flightpath_template(
                *,
                template: Dict[str, Any],
                pid_map: Dict[tuple[int, int], int],
                wp_alloc_0303_obj: Any,
                wp_alloc_0304_obj: Any,
            ) -> tuple[list[dict], list[dict], Dict[str, Any]]:
                started = time.perf_counter()
                flight_plans_0303_cached = copy.deepcopy(template.get("flight_plans_0303") or [])
                flight_plans_0304_cached = copy.deepcopy(template.get("flight_plans_0304") or [])
                path_key_map_raw = template.get("path_id_to_mission_key")
                path_key_map = path_key_map_raw if isinstance(path_key_map_raw, dict) else {}
                current_ts = _now_ms_since_2000()
                remapped = 0
                missing = 0

                def _remap_plan_path_id(fp: dict) -> None:
                    nonlocal remapped, missing
                    try:
                        old_pid = int(fp.get("pathID", 0) or 0)
                    except Exception:
                        old_pid = 0
                    raw_key = path_key_map.get(str(old_pid))
                    if raw_key is None:
                        raw_key = path_key_map.get(old_pid)
                    mission_key: tuple[int, int] | None = None
                    if isinstance(raw_key, (list, tuple)) and len(raw_key) >= 2:
                        try:
                            mission_key = (int(raw_key[0]), int(raw_key[1]))
                        except Exception:
                            mission_key = None
                    if mission_key is None:
                        missing += 1
                        return
                    new_pid = pid_map.get(mission_key)
                    if new_pid is None:
                        missing += 1
                        return
                    fp["pathID"] = int(new_pid)
                    fp["timestamp"] = current_ts
                    remapped += 1

                for fp in flight_plans_0303_cached:
                    if isinstance(fp, dict):
                        _remap_plan_path_id(fp)
                for fp in flight_plans_0304_cached:
                    if isinstance(fp, dict):
                        _remap_plan_path_id(fp)

                if missing:
                    raise RuntimeError(f"initial plan template path remap missing entries: {missing}")

                wp0303_count = _reassign_cached_flightpath_waypoints(
                    flight_plans_0303_cached,
                    wp_alloc_0303_obj,
                    waypoint_keys=("waypointList", "uavWaypointList"),
                )
                wp0304_count = _reassign_cached_flightpath_waypoints(
                    flight_plans_0304_cached,
                    wp_alloc_0304_obj,
                    waypoint_keys=("lahWaypointList", "waypointList"),
                )
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                meta = {
                    "elapsed_ms": round(float(elapsed_ms), 3),
                    "path_remapped": int(remapped),
                    "waypoint_0303": int(wp0303_count),
                    "waypoint_0304": int(wp0304_count),
                }
                return flight_plans_0303_cached, flight_plans_0304_cached, meta

            def _store_initial_plan_template(
                *,
                cache_key: str | None,
                mp_json_template: Dict[str, Any] | None,
                missions_before_path_ids: list[dict] | None,
                missions_after_path_ids: list[dict],
                flight_plans_0303_template: list[dict],
                flight_plans_0304_template: list[dict],
                variant_no_value: int,
            ) -> None:
                if not cache_key or mp_json_template is None or missions_before_path_ids is None:
                    return
                path_id_to_mission_key: Dict[str, list[int]] = {}
                for mission in missions_after_path_ids or []:
                    if not isinstance(mission, dict):
                        continue
                    try:
                        aid = int(mission.get("aircraftID", 0) or 0)
                        mid = int(mission.get("individualMissionID", 0) or 0)
                    except Exception:
                        continue
                    path_id = _imp_path_id(mission)
                    if aid <= 0 or mid < 0 or path_id is None or int(path_id) <= 0:
                        continue
                    path_id_to_mission_key[str(int(path_id))] = [int(aid), int(mid)]
                if not path_id_to_mission_key:
                    return
                store_result = put_initial_plan_template(
                    cache_key,
                    {
                        "mp_json": mp_json_template,
                        "missions": missions_before_path_ids,
                        "flight_plans_0303": flight_plans_0303_template,
                        "flight_plans_0304": flight_plans_0304_template,
                        "path_id_to_mission_key": path_id_to_mission_key,
                    },
                )
                if bool(store_result.get("stored")):
                    self.log_sig.emit(
                        "[REPLAN][CACHE] initial_plan_template_store "
                        f"variant={int(variant_no_value)} "
                        f"entries={int(store_result.get('entries') or 0)} "
                        f"missions={len(missions_before_path_ids or [])} "
                        f"fp0303={len(flight_plans_0303_template or [])} "
                        f"fp0304={len(flight_plans_0304_template or [])} "
                        f"diskStored={int(bool(store_result.get('diskStored')))} "
                        f"diskBytes={int(store_result.get('diskBytes') or 0)} "
                        f"disk_ms={float(store_result.get('diskElapsedMs') or 0.0):.3f} "
                        f"elapsed_ms={float(store_result.get('elapsedMs') or 0.0):.3f}"
                    )

            attack_option_indices = _collect_attack_option_indices(ctx)
            attack_exclusion_ctx_indices = _collect_attack_exclusion_option_indices(ctx)
            attack_ctx = _filter_context_by_indices(ctx, attack_option_indices) if attack_option_indices else ctx
            attack_summary_info: Optional[Dict[str, Any]] = None
            suppress_attack_exclusion_fallback = False
            attack_exclusion_source_plan_id = self._to_optional_int(
                ctx.get("currentMissionPlanID")
                or ctx.get("sourceMissionPlanID")
                or getattr(self, "_last_mission_plan_id", None)
            )
            attack_detail = ctx.get("replan_detail") if isinstance(ctx.get("replan_detail"), dict) else {}
            attack_trigger = str(
                (attack_detail or {}).get("trigger") or (attack_detail or {}).get("triggerType") or ""
            ).strip()

            def _run_attack_exclusion_parallel(exclusion_ctx: Dict[str, Any]) -> Dict[str, Any]:
                started_at = time.perf_counter()
                result = call_with_source_artifact_cache(
                    source_cache,
                    run_attack_exclusion_pipeline,
                    exclusion_ctx,
                )
                return {
                    "result": result,
                    "duration_ms": round((time.perf_counter() - started_at) * 1000.0, 3),
                }

            def _validate_attack_exclusion_parallel_result(
                parallel_payload: Dict[str, Any] | None,
            ) -> tuple[bool, str]:
                if not isinstance(parallel_payload, dict):
                    return False, "parallel payload missing"
                result_wrapper = parallel_payload.get("result")
                if not isinstance(result_wrapper, dict):
                    return False, "pipeline wrapper missing"
                result_payload = result_wrapper.get("result")
                if not isinstance(result_payload, dict):
                    return False, "pipeline result missing"

                expected_source_id = self._to_optional_int(attack_exclusion_parallel_info.get("sourcePlanID"))
                result_source_id = self._to_optional_int(result_payload.get("sourcePlanID"))
                if (
                    expected_source_id is not None
                    and result_source_id is not None
                    and int(result_source_id) != int(expected_source_id)
                ):
                    return False, f"sourcePlanID mismatch ({result_source_id} != {expected_source_id})"

                expected_plan_id = self._to_optional_int(attack_exclusion_parallel_info.get("expectedPlanID"))
                result_plan_id = self._to_optional_int(result_payload.get("missionPlanID"))
                if (
                    expected_plan_id is not None
                    and result_plan_id is not None
                    and int(result_plan_id) != int(expected_plan_id)
                ):
                    return False, f"missionPlanID mismatch ({result_plan_id} != {expected_plan_id})"

                attack_plan_ids = {
                    int(pid)
                    for pid in (attack_ctx.get("plan_ids") or [])
                    if self._to_optional_int(pid) is not None
                }
                if result_plan_id is not None and int(result_plan_id) in attack_plan_ids:
                    return False, f"missionPlanID collides with attack option ({result_plan_id})"

                updates = result_payload.get("missionUpdates")
                if not isinstance(updates, dict):
                    return False, "missionUpdates missing"
                mode = str(updates.get("mode") or "").strip()
                if mode != "attack_exclusion":
                    return False, f"unexpected mode {mode!r}"

                return True, ""

            def _attack_exclusion_parallel_max_wait_s() -> Optional[float]:
                raw_value = os.environ.get("REPLAN_ATTACK_EXCLUSION_MAX_WAIT_MS")
                if raw_value is None or str(raw_value).strip() == "":
                    return None
                try:
                    wait_ms = float(raw_value)
                except Exception:
                    return None
                if wait_ms < 0.0:
                    return None
                return float(wait_ms) / 1000.0

            use_attack_pipeline = self._should_use_attack_pipeline(ctx)
            attack_exclusion_parallel_enabled = (
                str(os.environ.get("REPLAN_ATTACK_EXCLUSION_PARALLEL", "1") or "").strip().lower()
                not in {"0", "false", "no", "off"}
            )
            if (
                use_attack_pipeline
                and attack_exclusion_parallel_enabled
                and len(attack_option_indices) == 1
                and len(attack_exclusion_ctx_indices) == 1
                and attack_exclusion_source_plan_id is not None
            ):
                try:
                    exclusion_idx = int(attack_exclusion_ctx_indices[0])
                    parallel_ctx = copy.deepcopy(_filter_context_by_indices(ctx, [exclusion_idx]))
                    parallel_ctx["sourceMissionPlanID"] = int(attack_exclusion_source_plan_id)
                    expected_plan_id = None
                    parallel_plan_ids = list(parallel_ctx.get("plan_ids") or [])
                    if parallel_plan_ids:
                        expected_plan_id = self._to_optional_int(parallel_plan_ids[0])
                    attack_exclusion_executor = concurrent.futures.ThreadPoolExecutor(
                        max_workers=1,
                        thread_name_prefix="AttackExclude",
                    )
                    attack_exclusion_future = attack_exclusion_executor.submit(
                        _run_attack_exclusion_parallel,
                        parallel_ctx,
                    )
                    attack_exclusion_parallel_info = {
                        "originalIndex": int(exclusion_idx),
                        "sourcePlanID": int(attack_exclusion_source_plan_id),
                        "expectedPlanID": int(expected_plan_id) if expected_plan_id is not None else None,
                        "trigger": attack_trigger,
                        "startedAt": time.perf_counter(),
                    }
                    self._record_replan_timing_event(
                        "attack_exclusion_parallel_started",
                        extra={
                            "option_index": int(exclusion_idx) + 1,
                            "sourcePlanID": int(attack_exclusion_source_plan_id),
                            "expectedPlanID": int(expected_plan_id) if expected_plan_id is not None else None,
                            "trigger": attack_trigger,
                        },
                    )
                    self.log_sig.emit(
                        "[ATTACK-EXCLUDE] parallel generation started "
                        f"(sourcePlanID={attack_exclusion_source_plan_id}, optionIndex={exclusion_idx + 1})"
                    )
                except Exception as exc:
                    attack_exclusion_future = None
                    if attack_exclusion_executor is not None:
                        try:
                            attack_exclusion_executor.shutdown(wait=False, cancel_futures=True)
                        except Exception:
                            pass
                        attack_exclusion_executor = None
                    attack_exclusion_parallel_info = {}
                    self.log_sig.emit(f"[WARN] 공격 배제 병렬 시작 실패 -> 순차 처리로 전환: {exc}")

            if use_attack_pipeline:
                _record_step("attack_pipeline", "start", detail={"reason": reason})
                self.log_sig.emit("[ATTACK] 공격 특화 재계획 요청 감지 → 전용 파이프라인 실행")
                try:
                    attack_result = run_attack_plan_pipeline(attack_ctx, log_callback=self._append_log_line)
                    attack_ctx["_attack_pipeline"] = attack_result
                    log_path = (attack_result or {}).get("log_path")
                    if log_path:
                        self.log_sig.emit(f"[ATTACK] 분석 로그 저장: {log_path}")
                    self._pipeline_logger.log_event(
                        session_id,
                        "info",
                        "Attack pipeline evaluated",
                        detail={"log_path": log_path},
                    )
                    _record_step("attack_pipeline", "evaluated", detail={"log_path": str(log_path) if log_path else None})
                    attack_result_body = (attack_result or {}).get("result") or {}
                    attack_updates = ((attack_result or {}).get("result") or {}).get("missionUpdates")
                    attack_failure_notice = str(attack_result_body.get("failure_notice") or "").strip()
                    if not attack_updates and attack_failure_notice and not failure_notice_sent:
                        self.log_sig.emit(f"[ATTACK] 공격 임무 생성 실패 -> 0305 재계획 완료(실패 사유) 발송: {attack_failure_notice}")
                        self._push_replan_failure_completion(attack_failure_notice)
                        failure_notice_sent = True
                    if not attack_updates and attack_trigger == "0402":
                        suppress_attack_exclusion_fallback = True
                        if attack_exclusion_future is not None:
                            attack_exclusion_parallel_info["discardReason"] = "0402_attack_pipeline_empty"
                            cancelled = False
                            try:
                                cancelled = bool(attack_exclusion_future.cancel())
                            except Exception:
                                cancelled = False
                            self._record_replan_timing_event(
                                "attack_exclusion_parallel_discarded",
                                extra={
                                    "reason": "0402_attack_pipeline_empty",
                                    "cancelled": bool(cancelled),
                                },
                            )
                        self.log_sig.emit(
                            "[ATTACK] 0402 공격 특화안 생성 실패 -> 공격 배제 fallback을 생략하고 현재 계획을 유지합니다."
                        )
                    if attack_updates:
                        try:
                            self._finalize_attack_pipeline(
                                attack_ctx, attack_result, attack_updates, reason, session_id, schedule_delivery=False
                            )
                            self._mark_attack_target_used(attack_result)

                            attack_summary_info = {
                                "mode": "attack",
                                "plan_ids": list(attack_ctx.get("plan_ids") or []),
                                "attack_log": log_path,
                            }

                            self._attack_delivery_buffer.append(
                                {
                                    "plan_ids": list(attack_ctx.get("plan_ids") or []),
                                    "option_names": list(attack_ctx.get("option_names") or []),
                                    "option_meta": dict(attack_ctx.get("_option_meta") or {}),
                                }
                            )
                            _record_step("attack_pipeline", "complete", detail={"plan_ids": list(attack_ctx.get("plan_ids") or []), "log_path": str(log_path) if log_path else None})
                        except Exception as exc:
                            self._append_log_line(f"[ATTACK][ERR] finalize failed: {exc}")
                            self._pipeline_logger.log_event(
                                session_id, "error", f"Attack finalize failed: {exc}"
                            )
                            _record_issue("attack_finalize_failed", f"Attack finalize failed: {exc}")
                            _notify_failure_once("attack_finalize_failed", exc=exc)
                except Exception as exc:
                    self._append_log_line(f"[ATTACK][ERR] pipeline failed: {exc}")
                    self._pipeline_logger.log_event(
                        session_id, "error", f"Attack pipeline failed: {exc}"
                    )
                    _record_issue("attack_pipeline_failed", f"Attack pipeline failed: {exc}")
                    _notify_failure_once("attack_pipeline_failed", exc=exc)

            # 공격 옵션은 공격 파이프라인 결과만 사용 (일반 파이프라인에서 제외)
            if attack_option_indices:
                excluded_indices = set(int(idx) for idx in attack_option_indices)
                if suppress_attack_exclusion_fallback:
                    excluded_indices.update(int(idx) for idx in attack_exclusion_ctx_indices)
                keep_indices = [
                    idx for idx in range(max(len(ctx.get("plan_ids") or []), len(ctx.get("option_names") or [])))
                    if idx not in excluded_indices
                ]
                if not keep_indices:
                    if attack_summary_info:
                        try:
                            self._schedule_plan_delivery(
                                list(attack_ctx.get("plan_ids") or []),
                                list(attack_ctx.get("option_names") or []),
                                reason,
                                dict(attack_ctx.get("_option_meta") or {}),
                                force_direct_update=False,
                            )
                            _record_step(
                                "attack_delivery",
                                "queued",
                                detail={"plan_ids": list(attack_ctx.get("plan_ids") or [])},
                            )
                        except Exception as exc:
                            self._append_log_line(f"[ATTACK][ERR] delivery queue failed: {exc}")
                            self._pipeline_logger.log_event(
                                session_id, "error", f"Attack delivery queue failed: {exc}"
                            )
                            _record_issue(
                                "attack_delivery_queue_failed",
                                f"Attack delivery queue failed: {exc}",
                            )
                            _notify_failure_once("attack_delivery_queue_failed", exc=exc)
                        summary_info = attack_summary_info
                        plan_log_status = "success"
                        plan_log_summary.update(summary_info or {})
                        success = True
                    else:
                        _record_issue(
                            "attack_pipeline_empty",
                            "Attack option requested but no attack plan was generated.",
                            status="error",
                        )
                        _notify_failure_once("attack_pipeline_empty")
                    return
                ctx = _filter_context_by_indices(ctx, keep_indices)

            post_attack_handled, post_attack_summary = self._try_run_post_attack_rejoin_pipeline(
                ctx,
                reason,
                session_id=session_id,
            )
            if post_attack_handled:
                if post_attack_summary:
                    summary_info = {"mode": "postAttackRejoin", **post_attack_summary}
                    post_attack_status = str(post_attack_summary.get("status") or "").strip().lower()
                    plan_log_status = "success"
                    plan_log_summary.update(summary_info or {})
                    if plan_log:
                        plan_log.set_plan_ids(
                            post_attack_summary.get("plan_ids")
                            or post_attack_summary.get("planIds")
                            or ctx.get("plan_ids")
                            or []
                        )
                        plan_log.update_summary(summary_info)
                        _record_step(
                            "post_attack_rejoin_pipeline",
                            "info" if post_attack_status == "skipped" else "success",
                            detail=summary_info,
                        )
                    success = True
                else:
                    _record_issue(
                        "post_attack_rejoin_pipeline_failed",
                        "Post-attack rejoin request could not be materialized.",
                    )
                    _notify_failure_once("post_attack_rejoin_pipeline_failed")
                return

            next_collab_handled, next_collab_summary = self._try_run_next_collab_replan_pipeline(
                ctx,
                reason,
                session_id=session_id,
            )
            if next_collab_handled:
                if next_collab_summary:
                    summary_info = {"mode": "nextCollaborativeMission", **next_collab_summary}
                    next_collab_status = str(next_collab_summary.get("status") or "").strip().lower()
                    plan_log_status = "success"
                    plan_log_summary.update(summary_info or {})
                    if plan_log:
                        plan_log.set_plan_ids(
                            next_collab_summary.get("plan_ids")
                            or next_collab_summary.get("planIds")
                            or ctx.get("plan_ids")
                            or []
                        )
                        plan_log.update_summary(summary_info)
                        _record_step(
                            "next_collab_pipeline",
                            "info" if next_collab_status == "skipped" else "success",
                            detail=summary_info,
                        )
                    success = True
                else:
                    _record_issue(
                        "next_collab_pipeline_failed",
                        "Next collaborative mission replan request could not be materialized.",
                    )
                    _notify_failure_once("next_collab_pipeline_failed")
                return

            imaging_schedule_handled, imaging_schedule_summary = self._try_run_imaging_schedule_replan_pipeline(
                ctx,
                reason,
                session_id=session_id,
            )
            if imaging_schedule_handled:
                if imaging_schedule_summary:
                    trigger_type = str(imaging_schedule_summary.get("trigger_type") or "")
                    mode_name = "qualityMonitorSep" if trigger_type == "qualityMonitorSep" else "imagingSchedule"
                    summary_info = {"mode": mode_name, **imaging_schedule_summary}
                    plan_log_status = "success"
                    plan_log_summary.update(summary_info or {})
                    if plan_log:
                        plan_log.set_plan_ids(
                            imaging_schedule_summary.get("plan_ids")
                            or imaging_schedule_summary.get("planIds")
                            or ctx.get("plan_ids")
                            or []
                        )
                        plan_log.update_summary(summary_info)
                        _record_step("imaging_schedule_pipeline", "success", detail=summary_info)
                    success = True
                else:
                    _record_issue(
                        "imaging_schedule_pipeline_failed",
                        "Imaging-schedule replan request could not be materialized.",
                    )
                    _notify_failure_once("imaging_schedule_pipeline_failed")
                return

            path_deviation_handled, path_deviation_summary = self._try_run_path_deviation_replan_pipeline(
                ctx,
                reason,
                session_id=session_id,
            )
            if path_deviation_handled:
                if path_deviation_summary:
                    summary_info = {"mode": "pathDeviation", **path_deviation_summary}
                    plan_log_status = "success"
                    plan_log_summary.update(summary_info or {})
                    if plan_log:
                        plan_log.set_plan_ids(
                            path_deviation_summary.get("plan_ids")
                            or path_deviation_summary.get("planIds")
                            or ctx.get("plan_ids")
                            or []
                        )
                        plan_log.update_summary(summary_info)
                        _record_step("path_deviation_pipeline", "success", detail=summary_info)
                    success = True
                else:
                    _record_issue(
                        "path_deviation_pipeline_failed",
                        "Path-deviation replan request could not be materialized.",
                    )
                    _notify_failure_once("path_deviation_pipeline_failed")
                return

            prior_summary = self._try_run_prior_mission_pipeline(ctx, reason, session_id=session_id)
            if prior_summary:
                summary_info = {"mode": "prior", **prior_summary}
                prior_status = str(prior_summary.get("status") or "").strip().lower()
                if prior_status in {"failed", "error"}:
                    failure_detail = dict(summary_info)
                    _record_issue(
                        "prior_pipeline_failed",
                        "Prior mission replan could not be materialized; legacy fallback was blocked.",
                        detail=failure_detail,
                    )
                    plan_log_summary.update({"stop_reason": "prior_pipeline_failed", **failure_detail})
                    _notify_failure_once("prior_pipeline_failed", detail=failure_detail)
                    self._plan_status = "임무계획 실패"
                    self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                    return
                plan_log_status = "success"
                plan_log_summary.update(summary_info or {})
                if plan_log:
                    plan_log.set_plan_ids(
                        prior_summary.get("plan_ids") or prior_summary.get("planIds") or ctx.get("plan_ids") or []
                    )
                    plan_log.update_summary(summary_info)
                    _record_step("prior_pipeline", "info" if prior_status == "skipped" else "success", detail=summary_info)
                success = True
                return

            generated_imp_ids: Set[int] = set()
            generated_path_ids: Set[int] = set()
            stored_path_ids: Set[int] = set()

            planner_runtime_started = time.perf_counter()
            planner_runtime = self._get_planner_runtime()
            planner_runtime_timing = dict(planner_runtime.get("runtime_timing") or {})
            self._record_replan_timing_event(
                "planner_runtime_ready",
                extra={
                    "duration_ms": round((time.perf_counter() - planner_runtime_started) * 1000.0, 3),
                    "cache_status": str(planner_runtime.get("runtime_cache_status") or ""),
                    "cache_wait_ms": float(planner_runtime.get("runtime_cache_wait_ms") or 0.0),
                    "force_reload": int(bool(planner_runtime.get("runtime_force_reload"))),
                    **{f"build_{key}": value for key, value in planner_runtime_timing.items()},
                    "checkpoint": "planner_runtime_ready",
                },
            )
            run_divide_and_pattern = planner_runtime["run_divide_and_pattern"]
            build_mission_plan_0301 = planner_runtime["build_mission_plan_0301"]
            d0302 = planner_runtime["d0302"]
            d0303 = planner_runtime["d0303"]
            d0304 = planner_runtime["d0304"]
            _ensure_mission_planner_import_paths()
            from modules.mission_planning.engine.mission_generation.id_allocation.allocator import (
                reserve_imp_ids,
                reserve_individual_mission_ids,
                reserve_mission_plan_ids,
                mark_waypoint_files_written,
                reserve_path_id_blocks,
                reserve_path_ids,
                reserve_replan_id_bundle,
                reserve_waypoint_block,
                reserve_waypoint_blocks,
            )

            uav_cruise_speed = float(planner_runtime.get("uav_cruise_speed", 40.0))
            uav_turn_step = float(planner_runtime.get("uav_turn_step", 15.0))
            if planner_runtime.get("uav_params_applied"):
                self.log_sig.emit(
                    f"[INFO] UAV params loaded (cruise={uav_cruise_speed:.2f}, turn_step={uav_turn_step:.1f})"
                )
            elif planner_runtime.get("uav_params_error"):
                self.log_sig.emit(f"[WARN] UAV params load failed: {planner_runtime['uav_params_error']}")

            def _max_waypoint_id_from_flight_plans(*flight_plan_groups) -> int | None:
                max_waypoint_id = None
                for group in flight_plan_groups:
                    for fp in group or []:
                        if not isinstance(fp, dict):
                            continue
                        for list_key in ("waypointList", "lahWaypointList"):
                            waypoint_list = fp.get(list_key)
                            if not isinstance(waypoint_list, list):
                                continue
                            for waypoint in waypoint_list:
                                if not isinstance(waypoint, dict):
                                    continue
                                try:
                                    waypoint_id = int(waypoint.get("waypointID"))
                                except Exception:
                                    continue
                                if waypoint_id <= 0:
                                    continue
                                if max_waypoint_id is None or waypoint_id > max_waypoint_id:
                                    max_waypoint_id = waypoint_id
                return max_waypoint_id

            def _imp_path_id(im):
                for key in ('pathID', 'pathId', 'individualMissionPathID', 'missionPathID'):
                    value = im.get(key)
                    try:
                        if value is not None:
                            return int(value)
                    except Exception:
                        continue
                mission_info = im.get('missionInfo')
                if isinstance(mission_info, dict):
                    for key in ('pathID', 'pathId'):
                        value = mission_info.get(key)
                        try:
                            if value is not None:
                                return int(value)
                        except Exception:
                            continue
                return None

            def _fp_mission_id(fp):
                for key in ("individualMissionID", "individualMissionId", "missionID", "missionId"):
                    value = fp.get(key)
                    try:
                        if value is not None:
                            return int(value)
                    except Exception:
                        continue
                return None

            def _snapshot_mission_path_ids(missions):
                snapshot: Dict[tuple[int, int], list[int]] = {}
                for mission in missions or []:
                    if not isinstance(mission, dict):
                        continue
                    try:
                        aircraft_id = int(mission.get("aircraftID", 0))
                        mission_id = int(mission.get("individualMissionID", 0))
                    except Exception:
                        continue
                    # Staged missions use ID 0 until IMP allocation; keep them for pathID remap.
                    if aircraft_id <= 0 or mission_id < 0:
                        continue
                    path_id = _imp_path_id(mission)
                    if path_id is None or int(path_id) <= 0:
                        continue
                    snapshot.setdefault((aircraft_id, mission_id), []).append(int(path_id))
                return snapshot

            def _build_path_id_remap(old_snapshot, missions):
                remap_candidates: Dict[int, Set[int]] = {}
                cursors: Dict[tuple[int, int], int] = {}
                for mission in missions or []:
                    if not isinstance(mission, dict):
                        continue
                    try:
                        aircraft_id = int(mission.get("aircraftID", 0))
                        mission_id = int(mission.get("individualMissionID", 0))
                    except Exception:
                        continue
                    # Staged missions use ID 0 until IMP allocation; keep them for pathID remap.
                    if aircraft_id <= 0 or mission_id < 0:
                        continue
                    key = (aircraft_id, mission_id)
                    old_values = old_snapshot.get(key)
                    old_path_id = None
                    if isinstance(old_values, (list, tuple)):
                        cursor = int(cursors.get(key, 0))
                        if cursor < len(old_values):
                            old_path_id = old_values[cursor]
                        elif old_values:
                            old_path_id = old_values[-1]
                        cursors[key] = cursor + 1
                    else:
                        old_path_id = old_values
                    new_path_id = _imp_path_id(mission)
                    if old_path_id is None or new_path_id is None:
                        continue
                    old_pid = int(old_path_id)
                    new_pid = int(new_path_id)
                    if old_pid > 0 and new_pid > 0 and old_pid != new_pid:
                        remap_candidates.setdefault(old_pid, set()).add(new_pid)
                return {
                    int(old_pid): int(next(iter(new_values)))
                    for old_pid, new_values in remap_candidates.items()
                    if len(new_values) == 1
                }

            def _enforce_fp_path_ids(fps, pid_map, *, path_remap_by_old=None):
                fixed = 0
                remap = {
                    int(old_pid): int(new_pid)
                    for old_pid, new_pid in dict(path_remap_by_old or {}).items()
                    if old_pid is not None and new_pid is not None
                }
                for fp in fps or []:
                    try:
                        aid = int(fp.get('aircraftID', 0))
                        desired = None
                        mid = _fp_mission_id(fp)
                        old_path_id = fp.get("pathID")
                        try:
                            old_path_id = int(old_path_id) if old_path_id is not None else None
                        except Exception:
                            old_path_id = None
                        if mid is not None:
                            desired = pid_map.get((aid, int(mid)))
                        if desired is None and old_path_id is not None:
                            desired = remap.get(int(old_path_id))
                        if desired is not None and fp.get('pathID') != desired:
                            fp['pathID'] = desired
                            fixed += 1
                    except Exception:
                        continue
                return fixed

            def _expected_mission_path_ids(missions) -> Set[int]:
                expected: Set[int] = set()
                for mission in missions or []:
                    if not isinstance(mission, dict):
                        continue
                    path_id = _imp_path_id(mission)
                    if path_id is None:
                        continue
                    try:
                        path_id_int = int(path_id)
                    except Exception:
                        continue
                    if path_id_int > 0:
                        expected.add(path_id_int)
                return expected

            def _repair_duplicate_flightpath_path_ids(
                *,
                missions,
                flight_plans_0303,
                flight_plans_0304,
                generated_path_ids: Set[int],
                pid_map: Dict[tuple[int, int], int],
            ) -> int:
                rows: list[tuple[str, dict]] = []
                for channel, plans in (("0303", flight_plans_0303 or []), ("0304", flight_plans_0304 or [])):
                    for fp in plans:
                        if isinstance(fp, dict):
                            rows.append((channel, fp))
                used_path_ids: Set[int] = set()
                for _channel, fp in rows:
                    try:
                        path_id = int(fp.get("pathID", 0))
                    except Exception:
                        continue
                    if path_id > 0:
                        used_path_ids.add(path_id)
                generated_path_ids.update(int(pid) for pid in used_path_ids if int(pid) > 0)

                def _reserve_unused_path_id(aircraft_id: int) -> int:
                    aid = int(aircraft_id)
                    if aid < 1 or aid > 6:
                        raise RuntimeError(f"cannot repair duplicate FlightPath pathID for aircraftID={aid}")
                    for _attempt in range(16):
                        for candidate in reserve_path_ids(aid, 1):
                            candidate_int = int(candidate)
                            if candidate_int <= 0:
                                continue
                            if candidate_int in used_path_ids or candidate_int in generated_path_ids:
                                continue
                            used_path_ids.add(candidate_int)
                            generated_path_ids.add(candidate_int)
                            return candidate_int
                    raise RuntimeError(f"duplicate FlightPath pathID repair exhausted aircraftID={aid}")

                def _update_matching_mission_path_id(
                    *,
                    aircraft_id: int,
                    mission_id: int | None,
                    old_path_id: int,
                    new_path_id: int,
                ) -> None:
                    if mission_id is None:
                        return
                    for mission in missions or []:
                        if not isinstance(mission, dict):
                            continue
                        try:
                            mission_aircraft_id = int(mission.get("aircraftID", 0))
                            mission_individual_id = int(mission.get("individualMissionID", 0))
                        except Exception:
                            continue
                        if mission_aircraft_id != int(aircraft_id) or mission_individual_id != int(mission_id):
                            continue
                        current_path_id = _imp_path_id(mission)
                        if current_path_id is None or int(current_path_id) != int(old_path_id):
                            continue
                        mission["pathID"] = int(new_path_id)
                        pid_map[(int(aircraft_id), int(mission_id))] = int(new_path_id)
                        return

                seen: Dict[int, tuple[str, dict]] = {}
                repaired = 0
                for channel, fp in rows:
                    try:
                        path_id = int(fp.get("pathID", 0))
                    except Exception:
                        continue
                    if path_id <= 0:
                        continue
                    if path_id not in seen:
                        seen[path_id] = (channel, fp)
                        continue
                    try:
                        aircraft_id = int(fp.get("aircraftID", 0))
                    except Exception:
                        aircraft_id = int(path_id) // 100_000_000
                    if aircraft_id < 1 or aircraft_id > 6:
                        aircraft_id = int(path_id) // 100_000_000
                    new_path_id = _reserve_unused_path_id(int(aircraft_id))
                    mission_id = _fp_mission_id(fp)
                    fp["pathID"] = int(new_path_id)
                    _update_matching_mission_path_id(
                        aircraft_id=int(aircraft_id),
                        mission_id=mission_id,
                        old_path_id=int(path_id),
                        new_path_id=int(new_path_id),
                    )
                    repaired += 1
                return int(repaired)

            db_root = db_paths.get_active_db_root()

            def _locate_prior_mission_plan():
                dss_dir = db_root / 'DSS_Internal'
                if not dss_dir.exists():
                    return None
                candidates = sorted(
                    (p for p in dss_dir.glob("0201_*.json") if p.is_file()),
                    key=lambda p: p.stat().st_mtime,
                    reverse=True,
                )
                for candidate in candidates:
                    try:
                        data = json.loads(candidate.read_text(encoding='utf-8'))
                    except Exception:
                        continue
                    ctx = data.get("_priorMissionContext")
                    if isinstance(ctx, dict):
                        return candidate, ctx
                return None

            def _apply_prior_mission_customizations(
                missions,
                flight_plans_0303,
                context,
                variant_no,
                pid_map,
                generated_path_ids,
            ):
                if not isinstance(context, dict):
                    return

                def _to_int(value, default=None):
                    try:
                        iv = int(value)
                        return iv
                    except Exception:
                        return default

                def _to_float(value, default=None):
                    try:
                        fv = float(value)
                        return fv
                    except Exception:
                        return default

                input_mission_id = _to_int(context.get("inputMissionID")) or _to_int(context.get("mission_id"))
                if input_mission_id is None:
                    self.log_sig.emit(f"[WARN] Prior mission customization skipped (variant={variant_no}): missing inputMissionID")
                    return

                coord = context.get("coordinate") or {}
                lat = _to_float(coord.get("latitude"))
                lon = _to_float(coord.get("longitude"))
                alt = _to_float(coord.get("altitude"), 800.0)
                if lat is None or lon is None:
                    self.log_sig.emit(f"[WARN] Prior mission customization skipped (variant={variant_no}): coordinate missing")
                    return

                prior_mission_id = _to_int(context.get("priorMissionID"), 0) or 0
                mission_type = _to_int(context.get("missionType"), 1) or 1
                target_id = _to_int(context.get("targetID"))
                preferred_aircraft_ids = (4, 5, 6)

                mission_entry = None
                fallback_entry = None
                for im in missions:
                    rel = im.get("relatedMission") or {}
                    if _to_int(rel.get("inputMissionID")) != input_mission_id:
                        continue
                    fallback_entry = im if fallback_entry is None else fallback_entry
                    if _to_int(im.get("aircraftID")) in preferred_aircraft_ids:
                        mission_entry = im
                        break
                if mission_entry is None:
                    mission_entry = fallback_entry
                if mission_entry is None:
                    self.log_sig.emit(f"[WARN] Prior mission customization skipped (variant={variant_no}): matching mission not found")
                    return

                aircraft_id = _to_int(mission_entry.get("aircraftID"))
                if aircraft_id not in preferred_aircraft_ids:
                    aircraft_id = preferred_aircraft_ids[0]
                    mission_entry["aircraftID"] = aircraft_id

                path_id = _to_int(mission_entry.get("pathID"))
                if not path_id or path_id <= 0:
                    path_id = int(reserve_path_ids(aircraft_id, 1)[0])
                    mission_entry["pathID"] = path_id
                generated_path_ids.add(path_id)
                pid_map[(aircraft_id, _to_int(mission_entry.get("individualMissionID")))] = path_id

                rel_block = dict(mission_entry.get("relatedMission") or {})
                rel_block["relatedMissionType"] = 2
                rel_block["inputMissionID"] = input_mission_id
                rel_block["priorMissionID"] = prior_mission_id
                mission_entry["relatedMission"] = rel_block
                mission_entry["isDone"] = False

                mission_info = dict(mission_entry.get("individualMissionInfo") or {})
                mission_info["individualMissionType"] = 1 if mission_type == 2 else 5
                mission_info["patternType"] = 1
                mission_info["autoZoomIn"] = True
                mission_info["coordinateList"] = [
                    {
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": int(round(alt)),
                    }
                ]
                mission_info["lineList"] = []
                mission_info["areaList"] = []
                mission_info["targetID"] = target_id if (mission_type == 2 and target_id is not None) else 0
                mission_entry["individualMissionInfo"] = mission_info

                flight_entry = None
                for fp in flight_plans_0303 or []:
                    if _to_int(fp.get("aircraftID")) == aircraft_id:
                        flight_entry = fp
                        break
                if flight_entry is None:
                    flight_entry = {
                        "timestamp": _now_ms_since_2000(),
                        "Source": "MMR",
                        "pathID": path_id,
                        "aircraftID": aircraft_id,
                        "isFormationFlight": False,
                        "waypointList": [],
                    }
                    flight_plans_0303.append(flight_entry)
                else:
                    flight_entry["pathID"] = path_id
                    flight_entry["aircraftID"] = aircraft_id

                prior_profile = get_runtime_prior_mission_profile(
                    default_turn_radius_m=400.0,
                    default_fov_deg=5.0,
                )
                target_speed = float(get_runtime_prior_float("target_speed_mps", 30.0))
                loiter_seconds = (
                    int(get_runtime_prior_int("tracking_loiter_seconds", 300))
                    if mission_type == 2
                    else int(get_runtime_prior_int("default_loiter_seconds", 50))
                )
                filming_property = {
                    "fieldOfView": float(prior_profile.get("fov_deg", 5.0) or 5.0),
                    "sensorType": 1,
                    "operationMode": 3 if mission_type == 2 else 1,
                }
                if mission_type == 2 and target_id is not None:
                    filming_property["autoTracking"] = {"targetID": target_id}
                else:
                    filming_property["coordinateOrientation"] = {
                        "coordinate": {
                            "latitude": lat,
                            "longitude": lon,
                            "altitude": 0,
                        }
                    }

                waypoint_id = 1
                try:
                    if flight_entry.get("waypointList"):
                        waypoint_id = _to_int(flight_entry["waypointList"][0].get("waypointID")) or 1
                except Exception:
                    waypoint_id = 1

                waypoint = {
                    "waypointID": waypoint_id,
                    "coordinate": {
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": int(round(alt)),
                    },
                    "speed": target_speed,
                    "eta": 300,
                    "ecf": 0.0,
                    "nextWaypointID": 0,
                    "waypointPassType": 2,
                    "filmingProperty": filming_property,
                    "loiterProperty": {
                        "radius": float(prior_profile.get("turn_radius_m", 400.0) or 400.0),
                        "direction": 1,
                        "time": loiter_seconds,
                        "speed": target_speed,
                    },
                }

                flight_entry["timestamp"] = _now_ms_since_2000()
                flight_entry["Source"] = flight_entry.get("Source") or "MMR"
                flight_entry["isFormationFlight"] = False
                flight_entry["waypointList"] = [waypoint]

                self.log_sig.emit(
                    f"[variant {variant_no}] Prior mission customization applied "
                    f"(aircraft={aircraft_id}, inputMissionID={input_mission_id})"
                )

            def _apply_manual_runtime_fov_overrides(
                *,
                missions: list[dict],
                flight_plans_0303: list[dict],
                variant_no: int,
            ) -> int:
                def _to_float(value, default: float = 0.0) -> float:
                    try:
                        return float(value)
                    except Exception:
                        return float(default)

                try:
                    runtime_values = dict((load_runtime_settings().get("values") or {}))
                except Exception as exc:
                    self.log_sig.emit(
                        f"[WARN] Manual FOV overwrite skipped (variant={variant_no}): settings load failed: {exc}"
                    )
                    return 0

                line_auto_from_db = bool(runtime_values.get("enhanced_auto_fov_from_db", True))
                area_auto_from_db = bool(line_auto_from_db)
                line_manual_fov_deg = _to_float(
                    runtime_values.get("line_custom_fov_deg", 0.0)
                )
                area_manual_fov_deg = _to_float(
                    runtime_values.get("area_custom_fov_deg", line_manual_fov_deg)
                )
                nadir_manual_fov_deg = _to_float(
                    runtime_values.get("area_nadir_fov_deg", area_manual_fov_deg)
                )
                if line_manual_fov_deg > 0.0:
                    line_manual_fov_deg = float(
                        apply_runtime_camera_adjusted_fov_deg(
                            line_manual_fov_deg,
                            context=f"MISSION_PLAN VARIANT{variant_no} MANUAL_LINE",
                        )
                    )
                if area_manual_fov_deg > 0.0:
                    area_manual_fov_deg = float(
                        apply_runtime_camera_adjusted_fov_deg(
                            area_manual_fov_deg,
                            context=f"MISSION_PLAN VARIANT{variant_no} MANUAL_AREA",
                        )
                    )
                if nadir_manual_fov_deg > 0.0:
                    nadir_manual_fov_deg = float(
                        apply_runtime_camera_adjusted_fov_deg(
                            nadir_manual_fov_deg,
                            context=f"MISSION_PLAN VARIANT{variant_no} MANUAL_NADIR",
                        )
                    )

                if line_auto_from_db and area_auto_from_db:
                    return 0

                mission_by_path: Dict[int, Dict[str, Any]] = {}
                mission_by_aircraft_path: Dict[tuple[int, int], Dict[str, Any]] = {}
                for mission in missions or []:
                    if not isinstance(mission, dict):
                        continue
                    aircraft_id = _safe_int_value(mission.get("aircraftID"))
                    path_id = _safe_int_value(mission.get("pathID"))
                    if path_id is None or int(path_id) <= 0:
                        continue
                    mission_by_path.setdefault(int(path_id), mission)
                    if aircraft_id is not None and int(aircraft_id) > 0:
                        mission_by_aircraft_path.setdefault((int(aircraft_id), int(path_id)), mission)

                updated_waypoints = 0
                touched_paths = 0
                for fp in flight_plans_0303 or []:
                    if not isinstance(fp, dict):
                        continue
                    aircraft_id = _safe_int_value(fp.get("aircraftID"))
                    path_id = _safe_int_value(fp.get("pathID"))
                    if path_id is None or int(path_id) <= 0:
                        continue

                    mission = None
                    if aircraft_id is not None and int(aircraft_id) > 0:
                        mission = mission_by_aircraft_path.get((int(aircraft_id), int(path_id)))
                    if mission is None:
                        mission = mission_by_path.get(int(path_id))
                    if not isinstance(mission, dict):
                        continue

                    mission_info = mission.get("individualMissionInfo")
                    if not isinstance(mission_info, dict):
                        continue

                    pattern_type = _safe_int_value(mission_info.get("patternType")) or 0
                    is_area_mission = bool(mission_info.get("areaList"))
                    is_line_mission = bool(mission_info.get("lineList"))
                    manual_fov_deg: float | None = None
                    if pattern_type == 3:
                        if not area_auto_from_db and nadir_manual_fov_deg > 0.0:
                            manual_fov_deg = float(nadir_manual_fov_deg)
                    elif is_area_mission:
                        if not area_auto_from_db and area_manual_fov_deg > 0.0:
                            manual_fov_deg = float(area_manual_fov_deg)
                    elif is_line_mission and not line_auto_from_db and line_manual_fov_deg > 0.0:
                        manual_fov_deg = float(line_manual_fov_deg)

                    if manual_fov_deg is None:
                        continue

                    waypoints = fp.get("waypointList")
                    if not isinstance(waypoints, list):
                        continue

                    touched_this_path = False
                    for waypoint in waypoints:
                        if not isinstance(waypoint, dict):
                            continue
                        filming = waypoint.get("filmingProperty")
                        if not isinstance(filming, dict):
                            continue
                        current_fov_deg = _to_float(filming.get("fieldOfView"), -1.0)
                        if math.isclose(current_fov_deg, manual_fov_deg, rel_tol=0.0, abs_tol=1e-6):
                            continue
                        filming["fieldOfView"] = float(manual_fov_deg)
                        updated_waypoints += 1
                        touched_this_path = True
                    if touched_this_path:
                        touched_paths += 1

                if updated_waypoints > 0:
                    self.log_sig.emit(
                        f"[INFO] Manual FOV overwrite applied (variant={variant_no}): "
                        f"paths={touched_paths}, waypoints={updated_waypoints}"
                    )
                return int(updated_waypoints)

            dir_0201 = db_root / 'InputMissionPlan'
            dir_0203 = db_root / 'MissionReferenceInfo'
            out_root_base = db_root / 'mission_output'
            out_root_base.mkdir(parents=True, exist_ok=True)

            def _pick_json(directory: Path):
                return _pick_latest_package_json(directory)

            def _resolve_path(value, directory: Path):
                if value:
                    try:
                        candidate = Path(value)
                        if candidate.exists():
                            return candidate
                    except Exception:
                        pass
                return _pick_json(directory)

            self._record_replan_timing_event(
                "input_resolve_start",
                extra={"checkpoint": "input_resolve_start"},
            )
            self.log_sig.emit(f"[INFO] Latest input snapshot → {describe_latest_ids()}")

            cached_latest_cmpk_id = get_latest_package_id("0201")
            cached_latest_mrpk_id = get_latest_package_id("0203")
            requested_cmpk_id = _safe_int_value(
                ctx.get("inputMissionPackageID") or staged.get("inputMissionPackageID")
            )
            requested_mrpk_id = _safe_int_value(
                ctx.get("missionReferencePackageID") or staged.get("missionReferencePackageID")
            )
            latest_cmpk_id = requested_cmpk_id if requested_cmpk_id is not None else cached_latest_cmpk_id
            latest_mrpk_id = requested_mrpk_id if requested_mrpk_id is not None else cached_latest_mrpk_id
            self._record_replan_timing_event(
                "latest_input_ids_loaded",
                extra={
                    "has_0201": int(latest_cmpk_id is not None),
                    "has_0203": int(latest_mrpk_id is not None),
                    "cached_0201": int(cached_latest_cmpk_id or 0),
                    "requested_0201": int(requested_cmpk_id or 0),
                    "checkpoint": "latest_input_ids_loaded",
                },
            )

            cmpk_path = None
            cmpk_missing = False
            if latest_cmpk_id is not None:
                candidate = dir_0201 / f"{latest_cmpk_id}.json"
                if candidate.exists():
                    cmpk_path = candidate
                    ctx['inputMissionPackageID'] = latest_cmpk_id
                    source_label = "requested" if requested_cmpk_id is not None else "latest"
                    self.log_sig.emit(
                        f"[STEP 0] Using {source_label} 0201 ID {latest_cmpk_id} ({candidate.name})"
                    )
                else:
                    payload_0201 = None
                    if cached_latest_cmpk_id is not None and int(cached_latest_cmpk_id) == int(latest_cmpk_id):
                        snap_0201 = get_latest_snapshot("0201")
                        payload_0201 = getattr(snap_0201, "payload", None)
                    if isinstance(payload_0201, dict) and (
                        payload_0201.get("inputMissionList") or payload_0201.get("availableAircraftList")
                    ):
                        payload_copy = dict(payload_0201)
                        payload_copy["inputMissionPackageID"] = latest_cmpk_id
                        try:
                            write_json(candidate, payload_copy, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
                            cmpk_path = candidate
                            ctx['inputMissionPackageID'] = latest_cmpk_id
                            self.log_sig.emit(f"[STEP 0] Materialized latest 0201 ID {latest_cmpk_id} from cache payload ({candidate.name})")
                        except Exception as exc:
                            self.log_sig.emit(f"[ERR] Failed to materialize latest 0201 ID {latest_cmpk_id}: {exc}")
                            cmpk_missing = True
                    else:
                        self.log_sig.emit(f"[ERR] Latest 0201 ID {latest_cmpk_id} missing and cache payload unavailable")
                        cmpk_missing = True
            if cmpk_missing:
                failure_detail = {"latest_0201": latest_cmpk_id}
                _record_issue("0201_missing", "latest 0201 missing", detail=failure_detail)
                plan_log_summary.update({"stop_reason": "0201_missing", **failure_detail})
                _notify_failure_once("0201_missing", detail=failure_detail)
                self._plan_status = "임무계획 실패"
                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                return
            if cmpk_path is None:
                fallback_cmpk = ctx.get('cmpk_path') or staged.get('cmpk_path')
                cmpk_path = _resolve_path(fallback_cmpk, dir_0201)
                if cmpk_path:
                    try:
                        ctx.setdefault('inputMissionPackageID', int(Path(cmpk_path).stem))
                    except Exception:
                        pass
                    self.log_sig.emit(f"[INFO] Fallback 0201 file selected: {cmpk_path.name}")

            if cmpk_path:
                _record_step("0201_resolved", "ok", detail={"path": str(cmpk_path), "latest_id": latest_cmpk_id})
                self._record_replan_timing_event(
                    "input_0201_resolved",
                    extra={"latest_id": latest_cmpk_id or 0, "checkpoint": "input_0201_resolved"},
                )
            mrpk_path = None
            mrpk_missing = False
            if latest_mrpk_id is not None:
                candidate = dir_0203 / f"{latest_mrpk_id}.json"
                if candidate.exists():
                    mrpk_path = candidate
                    ctx['missionReferencePackageID'] = latest_mrpk_id
                    source_label = "requested" if requested_mrpk_id is not None else "latest"
                    self.log_sig.emit(
                        f"[STEP 0] Using {source_label} 0203 ID {latest_mrpk_id} ({candidate.name})"
                    )
                else:
                    payload_0203 = None
                    if cached_latest_mrpk_id is not None and int(cached_latest_mrpk_id) == int(latest_mrpk_id):
                        snap_0203 = get_latest_snapshot("0203")
                        payload_0203 = getattr(snap_0203, "payload", None)
                    if isinstance(payload_0203, dict) and (
                        payload_0203.get("takeOverInfoList") or payload_0203.get("flightAreaList") or payload_0203.get("handOverInfoList")
                    ):
                        payload_copy = dict(payload_0203)
                        payload_copy["missionReferencePackageID"] = latest_mrpk_id
                        try:
                            write_json(candidate, payload_copy, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
                            mrpk_path = candidate
                            ctx['missionReferencePackageID'] = latest_mrpk_id
                            self.log_sig.emit(f"[STEP 0] Materialized latest 0203 ID {latest_mrpk_id} from cache payload ({candidate.name})")
                        except Exception as exc:
                            self.log_sig.emit(f"[ERR] Failed to materialize latest 0203 ID {latest_mrpk_id}: {exc}")
                            mrpk_missing = True
                    else:
                        self.log_sig.emit(f"[ERR] Latest 0203 ID {latest_mrpk_id} missing and cache payload unavailable")
                        mrpk_missing = True
            if mrpk_missing:
                failure_detail = {"latest_0203": latest_mrpk_id}
                _record_issue("0203_missing", "latest 0203 missing", detail=failure_detail)
                plan_log_summary.update({"stop_reason": "0203_missing", **failure_detail})
                _notify_failure_once("0203_missing", detail=failure_detail)
                self._plan_status = "임무계획 실패"
                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                return
            if mrpk_path is None:
                fallback_mrpk = ctx.get('mrpk_path') or staged.get('mrpk_path')
                mrpk_path = _resolve_path(fallback_mrpk, dir_0203)
                if mrpk_path:
                    try:
                        ctx.setdefault('missionReferencePackageID', int(Path(mrpk_path).stem))
                    except Exception:
                        pass
                    self.log_sig.emit(f"[INFO] Fallback 0203 file selected: {mrpk_path.name}")

            if mrpk_path:
                _record_step("0203_resolved", "ok", detail={"path": str(mrpk_path), "latest_id": latest_mrpk_id})
                self._record_replan_timing_event(
                    "input_0203_resolved",
                    extra={"latest_id": latest_mrpk_id or 0, "checkpoint": "input_0203_resolved"},
                )
            if not cmpk_path or not mrpk_path:
                self.log_sig.emit('[ERR] Replan pipeline aborted: missing 0201/0203 input')
                missing_detail = {"cmpk_path": str(cmpk_path) if cmpk_path else None, "mrpk_path": str(mrpk_path) if mrpk_path else None}
                _record_issue("input_missing", "missing 0201/0203 input", detail=missing_detail)
                plan_log_summary.update({"stop_reason": "input_missing", **missing_detail})
                _notify_failure_once("input_missing", detail=missing_detail)
                self._plan_status = "임무계획 실패"
                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                return

            self._record_replan_timing_event(
                "input_paths_resolved",
                extra={
                    "has_0201": int(bool(cmpk_path)),
                    "has_0203": int(bool(mrpk_path)),
                    "checkpoint": "input_paths_resolved",
                },
            )
            cmpk_data = None
            mrpk_data = None
            try:
                cmpk_data = source_cache.read_json(cmpk_path, kind="InputMissionPlan")
            except Exception as exc:
                self.log_sig.emit(f"[ERR] 0201 로드 실패: {exc}")
                _record_issue("0201_load_failed", "failed to load 0201", detail={"path": str(cmpk_path), "error": str(exc)})
                plan_log_summary.update({"stop_reason": "0201_load_failed", "cmpk_path": str(cmpk_path)})
                _notify_failure_once("0201_load_failed", exc=exc, detail={"path": str(cmpk_path)})
                self._plan_status = "임무계획 실패"
                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                return
            try:
                mrpk_data = source_cache.read_json(mrpk_path, kind="MissionReferenceInfo")
            except Exception as exc:
                self.log_sig.emit(f"[ERR] 0203 로드 실패: {exc}")
                _record_issue("0203_load_failed", "failed to load 0203", detail={"path": str(mrpk_path), "error": str(exc)})
                plan_log_summary.update({"stop_reason": "0203_load_failed", "mrpk_path": str(mrpk_path)})
                _notify_failure_once("0203_load_failed", exc=exc, detail={"path": str(mrpk_path)})
                self._plan_status = "임무계획 실패"
                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                return

            def _coerce_package_id(value: Any) -> Optional[int]:
                try:
                    ivalue = int(value)
                except Exception:
                    return None
                return ivalue if ivalue >= 0 else None

            def _normalize_loaded_package_id(
                payload: Any,
                path: Path,
                *,
                package_key: str,
                ctx_key: str,
                label: str,
            ) -> None:
                if not isinstance(payload, dict):
                    return
                expected = _coerce_package_id(ctx.get(ctx_key))
                path_id = _coerce_package_id(Path(path).stem)
                if expected is None:
                    expected = path_id
                if expected is None:
                    return
                current = _coerce_package_id(payload.get(package_key))
                ctx[ctx_key] = int(expected)
                if current == expected:
                    return
                payload[package_key] = int(expected)
                current_text = str(current) if current is not None else "missing"
                self.log_sig.emit(
                    f"[WARN] {label} package ID mismatch: {Path(path).name} has "
                    f"{package_key}={current_text}; using {expected}"
                )
                try:
                    write_json(
                        path,
                        payload,
                        pretty=True,
                        ensure_ascii=False,
                        skip_if_unchanged=True,
                    )
                except Exception as exc:
                    self.log_sig.emit(
                        f"[WARN] {label} package ID normalization write failed ({Path(path).name}): {exc}"
                    )

            _normalize_loaded_package_id(
                cmpk_data,
                cmpk_path,
                package_key="inputMissionPackageID",
                ctx_key="inputMissionPackageID",
                label="0201",
            )
            _normalize_loaded_package_id(
                mrpk_data,
                mrpk_path,
                package_key="missionReferencePackageID",
                ctx_key="missionReferencePackageID",
                label="0203",
            )

            self._record_replan_timing_event(
                "input_json_loaded",
                extra={"checkpoint": "input_json_loaded"},
            )
            validation_errors: list[dict[str, Any]] = []

            def _require_list(
                payload: dict,
                key: str,
                label: str,
                *,
                allow_empty: bool = False,
                allow_missing: bool = False,
            ) -> list:
                value = payload.get(key)
                if not isinstance(value, list):
                    if value is None and allow_missing:
                        self.log_sig.emit(f"[WARN] {label} 없음 (선택 항목)")
                        return []
                    self.log_sig.emit(f"[ERR] {label} 필요하지만 없음/형식오류 → 중단")
                    validation_errors.append({"key": key, "reason": "missing_or_invalid"})
                    return []
                if not value and not allow_empty:
                    self.log_sig.emit(f"[ERR] {label} 필요하지만 비어있음 → 중단")
                    validation_errors.append({"key": key, "reason": "empty"})
                else:
                    self.log_sig.emit(f"[OK] {label} 확인됨 (count={len(value)}) → 진행")
                return value

            def _summarize_ids(values: list, limit: int = 5) -> str:
                if not values:
                    return "-"
                return ", ".join(str(v) for v in values[:limit])

            def _is_single_point_coordinate_only_mission(mission: Any) -> bool:
                if not isinstance(mission, dict):
                    return False
                detail = mission.get("missionDetail")
                if not isinstance(detail, dict):
                    return False
                line_list = detail.get("lineList")
                area_list = detail.get("areaList")
                if isinstance(line_list, list) and line_list:
                    return False
                if isinstance(area_list, list) and area_list:
                    return False
                coord_list = detail.get("coordinateList")
                return isinstance(coord_list, list) and len(coord_list) == 1

            def _infer_input_mission_type(detail: Any) -> int | None:
                if not isinstance(detail, dict):
                    return None
                if detail.get("lineList"):
                    return 1
                if detail.get("areaList"):
                    return 2
                return None

            def _inferred_type_label(mtype: int | None) -> str:
                if int(mtype or 0) == 1:
                    return "line"
                if int(mtype or 0) == 2:
                    return "area"
                return "unknown"

            self.log_sig.emit("[CHECK] 0201/0203 필수 데이터 확인 시작")
            mission_list = _require_list(cmpk_data or {}, "inputMissionList", "0201 inputMissionList")
            aircraft_list = _require_list(cmpk_data or {}, "availableAircraftList", "0201 availableAircraftList")
            self._record_replan_timing_event(
                "input_required_lists_checked",
                extra={
                    "missions": len(mission_list),
                    "aircrafts": len(aircraft_list),
                    "checkpoint": "input_required_lists_checked",
                },
            )

            pkg_type_raw = (cmpk_data or {}).get("inputMissionPackageType")
            try:
                pkg_type = int(pkg_type_raw)
            except Exception:
                pkg_type = None
            if not pkg_type or pkg_type == 0:
                self.log_sig.emit("[ERR] 0201 inputMissionPackageType=0/없음 → 중단")
                validation_errors.append({"key": "inputMissionPackageType", "reason": "invalid"})
            else:
                self.log_sig.emit(f"[OK] 0201 inputMissionPackageType={pkg_type} 확인")

            main_sensor_raw = (cmpk_data or {}).get("mainSensor")
            try:
                main_sensor = int(main_sensor_raw)
            except Exception:
                main_sensor = None
            if not main_sensor or main_sensor == 0:
                self.log_sig.emit("[ERR] 0201 mainSensor=0/없음 → 중단")
                validation_errors.append({"key": "mainSensor", "reason": "invalid"})
            else:
                self.log_sig.emit(f"[OK] 0201 mainSensor={main_sensor} 확인")

            uav_count = 0
            lah_count = 0
            zero_aircraft: list[Any] = []
            invalid_aircraft: list[Any] = []
            for idx, entry in enumerate(aircraft_list):
                aircraft_id = None
                if isinstance(entry, dict):
                    aircraft_id = entry.get("aircraftID")
                elif isinstance(entry, int):
                    aircraft_id = entry
                elif isinstance(entry, str) and entry.isdigit():
                    aircraft_id = int(entry)
                if not isinstance(aircraft_id, int):
                    invalid_aircraft.append(idx)
                    continue
                if aircraft_id == 0:
                    zero_aircraft.append(idx)
                    continue
                if 1 <= aircraft_id <= 3:
                    lah_count += 1
                elif 4 <= aircraft_id <= 6:
                    uav_count += 1
                else:
                    invalid_aircraft.append(aircraft_id)
            if aircraft_list:
                self.log_sig.emit(f"[CHECK] 0201 기체 수={len(aircraft_list)} (UAV={uav_count}, LAH={lah_count})")
                if zero_aircraft:
                    self.log_sig.emit(
                        f"[ERR] 0201 availableAircraftList에 0 포함 (idx={_summarize_ids(zero_aircraft)}) → 중단"
                    )
                    validation_errors.append({"key": "availableAircraftList", "reason": "contains_zero"})
                if invalid_aircraft:
                    self.log_sig.emit(
                        f"[ERR] 0201 availableAircraftList 형식/범위 오류 (idx={_summarize_ids(invalid_aircraft)}) → 중단"
                    )
                    validation_errors.append({"key": "availableAircraftList", "reason": "invalid_entries"})
                if uav_count == 0:
                    self.log_sig.emit("[ERR] 0201 UAV 정보 없음 → 중단")
                    validation_errors.append({"key": "availableAircraftList", "reason": "no_uav"})
                if lah_count == 0:
                    self.log_sig.emit("[ERR] 0201 LAH 정보 없음 → 중단")
                    validation_errors.append({"key": "availableAircraftList", "reason": "no_lah"})

            missing_id: list[Any] = []
            missing_detail: list[Any] = []
            missing_shape: list[Any] = []
            unknown_type: list[Any] = []
            type_zero_invalid: list[Any] = []
            type_zero_autofix: list[str] = []
            line_only_violation: list[Any] = []
            skipped_single_point_coordinate_only: list[Any] = []
            plannable_mission_count = 0
            not_done_mission_count = 0
            # Lenient typing: the shape in missionDetail decides how a mission is
            # planned; inputMissionType only feeds counters/warnings. A mission is
            # unplannable only when it has no usable shape at all.
            for mission in mission_list:
                if not isinstance(mission, dict):
                    missing_detail.append("?")
                    continue
                mid = mission.get("inputMissionID")
                if mid is None:
                    missing_id.append("?")
                if bool(mission.get("isDone")):
                    continue
                not_done_mission_count += 1
                detail = mission.get("missionDetail")
                if not isinstance(detail, dict):
                    missing_detail.append(mid)
                    continue
                if _is_single_point_coordinate_only_mission(mission):
                    skipped_single_point_coordinate_only.append(mid)
                    continue
                mtype_raw = mission.get("inputMissionType")
                try:
                    mtype = int(mtype_raw)
                except Exception:
                    mtype = None
                inferred_type = _infer_input_mission_type(detail)
                has_line = bool(detail.get("lineList"))
                has_area = bool(detail.get("areaList"))
                coord_list = detail.get("coordinateList")
                has_coords = isinstance(coord_list, list) and len(coord_list) >= 2
                if not has_line and not has_area and not has_coords:
                    missing_shape.append(mid)
                    continue
                plannable_mission_count += 1
                if mtype == 0 or mtype is None:
                    type_zero_autofix.append(
                        f"{mid}({_inferred_type_label(inferred_type)})"
                    )
                elif mtype not in (1, 2, 3, 4, 5, 6, 7):
                    unknown_type.append(mid)
                elif mtype in (1, 7) and has_area:
                    line_only_violation.append(mid)

            if mission_list:
                self.log_sig.emit(
                    "[CHECK] 0201 임무: total={total} missingID={mid} missingDetail={md} "
                    "missingShape={ms} unknownType={ut} typeZeroAuto={tza} typeZeroInvalid={tzi} "
                    "lineOnlyViolation={lv} skippedSinglePoint={sp}".format(
                        total=len(mission_list),
                        mid=len(missing_id),
                        md=len(missing_detail),
                        ms=len(missing_shape),
                        ut=len(unknown_type),
                        tza=len(type_zero_autofix),
                        tzi=len(type_zero_invalid),
                        lv=len(line_only_violation),
                        sp=len(skipped_single_point_coordinate_only),
                    )
                )
                if skipped_single_point_coordinate_only:
                    self.log_sig.emit(
                        "[INFO] 0201 single-point coordinate-only 임무는 무시합니다: "
                        f"{_summarize_ids(skipped_single_point_coordinate_only)}"
                    )
                if (
                    missing_id
                    or missing_detail
                    or missing_shape
                    or unknown_type
                    or type_zero_invalid
                    or line_only_violation
                ):
                    self.log_sig.emit(
                        "[DETAIL] 0201 문제 샘플: missingID={mid} missingDetail={md} "
                        "missingShape={ms} unknownType={ut} typeZeroAuto={tza} typeZeroInvalid={tzi} lineOnlyViolation={lv}".format(
                            mid=_summarize_ids(missing_id),
                            md=_summarize_ids(missing_detail),
                            ms=_summarize_ids(missing_shape),
                            ut=_summarize_ids(unknown_type),
                            tza=_summarize_ids(type_zero_autofix),
                            tzi=_summarize_ids(type_zero_invalid),
                            lv=_summarize_ids(line_only_violation),
                        )
                    )
                    # Lenient typing: shape-less/odd-typed entries are skipped or
                    # planned by shape downstream; abort only when nothing is left.
                    self.log_sig.emit(
                        "[WARN] 0201 임무 일부 문제 → 도형 기준으로 계획 진행 "
                        f"(plannable={plannable_mission_count})"
                    )
                if not_done_mission_count > 0 and plannable_mission_count == 0:
                    self.log_sig.emit("[ERR] 0201 계획 가능한 임무 없음 → 중단")
                    validation_errors.append({"key": "inputMissionList", "reason": "invalid_entries"})

            take_over_list = _require_list(mrpk_data or {}, "takeOverInfoList", "0203 takeOverInfoList")
            hand_over_list = _require_list(mrpk_data or {}, "handOverInfoList", "0203 handOverInfoList")
            flight_area_list = _require_list(
                mrpk_data or {},
                "flightAreaList",
                "0203 flightAreaList",
                allow_empty=True,
                allow_missing=True,
            )
            bad_take: list[int] = []
            bad_hand: list[int] = []

            def _count_bad_coords(entries: list) -> list[int]:
                bad = []
                for idx, item in enumerate(entries):
                    coord = item.get("coordinate") if isinstance(item, dict) else None
                    if not isinstance(coord, dict) or "latitude" not in coord or "longitude" not in coord:
                        bad.append(idx)
                return bad

            if take_over_list:
                bad_take = _count_bad_coords(take_over_list)
                if bad_take:
                    self.log_sig.emit(f"[WARN] 0203 takeOver 좌표 누락: {len(bad_take)}건 (idx={_summarize_ids(bad_take)})")
            if hand_over_list:
                bad_hand = _count_bad_coords(hand_over_list)
                if bad_hand:
                    self.log_sig.emit(f"[WARN] 0203 handOver 좌표 누락: {len(bad_hand)}건 (idx={_summarize_ids(bad_hand)})")
            if flight_area_list:
                self.log_sig.emit(f"[CHECK] 0203 flightAreaList count={len(flight_area_list)}")
            else:
                self.log_sig.emit("[WARN] 0203 flightAreaList 비어있음 (선택 항목)")

            if type_zero_autofix:
                notice = (
                    "0201 임무 type 이상 경고: "
                    + ", ".join(type_zero_autofix[:5])
                    + " -> 도형 기준 자동보정"
                )
                self.log_sig.emit("[WARN] " + notice)
                self._push_0001_notice(notice)

            if validation_errors:
                notice_parts: list[str] = []
                reason_labels = {
                    "missing_or_invalid": "누락/형식오류",
                    "empty": "빈목록",
                    "invalid": "값오류",
                    "contains_zero": "0포함",
                    "invalid_entries": "형식오류",
                    "no_uav": "UAV없음",
                    "no_lah": "LAH없음",
                }
                key_labels = {
                    "inputMissionPackageType": "0201 패키지타입",
                    "mainSensor": "0201 메인센서",
                    "availableAircraftList": "0201 기체목록",
                    "inputMissionList": "0201 임무목록",
                    "takeOverInfoList": "0203 인계목록",
                    "handOverInfoList": "0203 반환목록",
                    "flightAreaList": "0203 비행구역목록",
                }
                err_map: dict[str, set[str]] = {}
                for err in validation_errors:
                    key = err.get("key") if isinstance(err, dict) else None
                    reason = err.get("reason") if isinstance(err, dict) else None
                    key_text = key_labels.get(str(key), str(key or "알수없음"))
                    reason_text = reason_labels.get(str(reason), str(reason or "오류"))
                    err_map.setdefault(key_text, set()).add(reason_text)
                if err_map:
                    chunks = []
                    for key_text, reasons in err_map.items():
                        reason_text = ",".join(sorted(reasons))
                        chunks.append(f"{key_text}({reason_text})")
                    notice_parts.append("항목: " + " / ".join(chunks))
                if (
                    missing_id
                    or missing_detail
                    or missing_shape
                    or unknown_type
                    or type_zero_invalid
                    or line_only_violation
                ):
                    notice_parts.append(
                        "0201 임무오류(ID누락={mid},상세누락={md},도형누락={ms},타입미정={ut},타입0미보정={tz},lineOnly위반={lv})".format(
                            mid=len(missing_id),
                            md=len(missing_detail),
                            ms=len(missing_shape),
                            ut=len(unknown_type),
                            tz=len(type_zero_invalid),
                            lv=len(line_only_violation),
                        )
                    )
                if bad_take or bad_hand:
                    notice_parts.append(
                        f"0203 좌표누락(인계={len(bad_take)},반환={len(bad_hand)})"
                    )
                if notice_parts:
                    self.log_sig.emit("[ERR] 0201/0203 검증 실패 요약: " + " | ".join(notice_parts))
                self.log_sig.emit("[ERR] 0201/0203 필수 데이터 부족 → 임무계획 중단")
                _record_issue("input_validation_failed", "0201/0203 validation failed", detail={"errors": validation_errors})
                plan_log_summary.update({"stop_reason": "input_validation_failed", "errors": validation_errors})
                _notify_failure_once("input_validation_failed", detail={"errors": validation_errors})
                self._plan_status = "임무계획 실패"
                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                return
            _record_step(
                "input_validation",
                "ok",
                detail={
                    "missions": len(mission_list),
                    "aircrafts": len(aircraft_list),
                    "uavs": uav_count,
                    "lahs": lah_count,
                    "skippedSinglePointCoordinateOnly": len(skipped_single_point_coordinate_only),
                    "takeOver": len(take_over_list),
                    "handOver": len(hand_over_list),
                    "flightArea": len(flight_area_list),
                },
            )
            self._record_replan_timing_event(
                "input_validation_done",
                extra={
                    "missions": len(mission_list),
                    "aircrafts": len(aircraft_list),
                    "checkpoint": "input_validation_done",
                },
            )

            # Exclude completed input missions (isDone=True) before generating new plans.
            mission_whitelist: Set[int] = set()
            for source in (ctx.get("mission_ids"), staged.get("mission_ids")):
                for value in source or []:
                    try:
                        mission_whitelist.add(int(value))
                    except Exception:
                        continue

            snapshot_source_plan_id = None
            for source_value in (
                ctx.get("sourceMissionPlanID"),
                staged.get("sourceMissionPlanID"),
                ((ctx.get("replan_detail") or {}) if isinstance(ctx.get("replan_detail"), dict) else {}).get("sourceMissionPlanID"),
                ((staged.get("replan_detail") or {}) if isinstance(staged.get("replan_detail"), dict) else {}).get("sourceMissionPlanID"),
                ctx.get("currentMissionPlanID"),
            ):
                snapshot_source_plan_id = _safe_int_value(source_value)
                if snapshot_source_plan_id is not None and snapshot_source_plan_id > 0:
                    break

            current_remaining_source_detail = None
            for container in (ctx, staged):
                detail_payload = container.get("replan_detail") if isinstance(container, dict) else None
                if isinstance(detail_payload, dict) and bool(detail_payload.get("currentRemainingCollaborativeReplan")):
                    current_remaining_source_detail = detail_payload
                    break
            if snapshot_source_plan_id is None and current_remaining_source_detail is not None:
                for source_value in (
                    current_remaining_source_detail.get("sourceMissionPlanID"),
                    current_remaining_source_detail.get("currentMissionPlanID"),
                    ctx.get("sourceMissionPlanID"),
                    ctx.get("currentMissionPlanID"),
                    staged.get("sourceMissionPlanID"),
                    staged.get("currentMissionPlanID"),
                    getattr(self, "_last_mission_plan_id", None),
                ):
                    snapshot_source_plan_id = _safe_int_value(source_value)
                    if snapshot_source_plan_id is not None and snapshot_source_plan_id > 0:
                        ctx["sourceMissionPlanID"] = int(snapshot_source_plan_id)
                        ctx["currentMissionPlanID"] = int(snapshot_source_plan_id)
                        current_remaining_source_detail["sourceMissionPlanID"] = int(snapshot_source_plan_id)
                        current_remaining_source_detail["currentMissionPlanID"] = int(snapshot_source_plan_id)
                        self.log_sig.emit(
                            "[INFO] 현재위치 첫 임무 hybrid source plan 보강: "
                            f"sourcePlan={int(snapshot_source_plan_id)}"
                        )
                        break

            # A normal 0201 input refresh used to rebuild the active LINE from
            # its complete source geometry.  Prefer the current ID attached by
            # Monitoring; for older senders, recover it only when the exact
            # source snapshot has one unambiguous started mission.
            if (
                isinstance(cmpk_data, dict)
                and snapshot_source_plan_id is not None
                and snapshot_source_plan_id > 0
                and is_input_refresh_context(ctx, staged)
                and input_refresh_current_input_id(ctx, staged) is None
            ):
                refresh_snapshot = mission_area_replan_store.load_snapshot(
                    int(snapshot_source_plan_id)
                )
                inferred_current_input_id = infer_started_input_mission_id(
                    refresh_snapshot if isinstance(refresh_snapshot, dict) else None,
                    cmpk_data,
                )
                if inferred_current_input_id is not None:
                    attach_input_refresh_current_input_id(
                        int(inferred_current_input_id),
                        ctx,
                        staged,
                    )
                    self.log_sig.emit(
                        "[INFO] inputRefresh 현재 임무 복구: "
                        f"sourcePlan={int(snapshot_source_plan_id)}, "
                        f"currentInputMissionID={int(inferred_current_input_id)}, "
                        "source=remaining snapshot"
                    )

            snapshot_should_apply, snapshot_reason = _should_apply_remaining_snapshot(
                ctx=ctx,
                staged=staged,
                source_plan_id=snapshot_source_plan_id,
            )
            snapshot_audit_context = _remaining_snapshot_audit_context(ctx=ctx, staged=staged)

            snapshot_apply_result: Dict[str, Any] = {
                "applied": 0,
                "marked_done": 0,
                "snapshotMissionCount": 0,
                "snapshotPlanID": snapshot_source_plan_id,
            }
            collapse_apply_result: Dict[str, Any] = {
                "mutated": False,
                "groupCount": 0,
                "removedInputMissionIDs": [],
                "normalizedInputMissionIDs": [],
            }
            snapshot_mission_whitelist = _snapshot_apply_whitelist_for_current_remaining_hybrid(
                ctx=ctx,
                staged=staged,
                cmpk_data=cmpk_data if isinstance(cmpk_data, dict) else {},
                mission_whitelist=mission_whitelist,
            )
            if snapshot_should_apply and isinstance(cmpk_data, dict):
                snapshot_apply_result = _override_input_missions_with_remaining_snapshot(
                    cmpk_data,
                    source_plan_id=snapshot_source_plan_id,
                    mission_whitelist=snapshot_mission_whitelist,
                    audit_context=snapshot_audit_context,
                )
                if int(snapshot_apply_result.get("applied") or 0) > 0 or int(snapshot_apply_result.get("marked_done") or 0) > 0:
                    self.log_sig.emit(
                        "[INFO] 임무 진행영역 스냅샷 적용: "
                        f"sourcePlan={snapshot_apply_result.get('snapshotPlanID') or snapshot_source_plan_id}, "
                        f"updated={int(snapshot_apply_result.get('applied') or 0)}, "
                        f"done={int(snapshot_apply_result.get('marked_done') or 0)}"
                    )
                collapse_apply_result = _collapse_input_missions_for_replan(
                    cmpk_data,
                    mission_whitelist=snapshot_mission_whitelist,
                )
                if bool(collapse_apply_result.get("mutated")):
                    normalized_summary = ", ".join(
                        str(value) for value in (collapse_apply_result.get("normalizedInputMissionIDs") or [])
                    ) or "-"
                    self.log_sig.emit(
                        "[INFO] 임무 진행영역 재계획 collapse 적용: "
                        f"groups={int(collapse_apply_result.get('groupCount') or 0)}, "
                        f"normalized={normalized_summary}"
                    )
            elif isinstance(cmpk_data, dict) and snapshot_source_plan_id is not None and snapshot_source_plan_id > 0:
                self.log_sig.emit(
                    "[INFO] 임무 진행영역 스냅샷 미적용: "
                    f"sourcePlan={snapshot_source_plan_id}, reason={snapshot_reason}"
                )

            current_remaining_hybrid_request = _build_current_remaining_hybrid_request(
                ctx=ctx,
                staged=staged,
                cmpk_data=cmpk_data if isinstance(cmpk_data, dict) else {},
                source_plan_id=snapshot_source_plan_id,
                mission_whitelist=mission_whitelist,
            )
            if current_remaining_hybrid_request is not None:
                current_remaining_hybrid_request_validation = validate_current_remaining_hybrid_request(
                    current_remaining_hybrid_request,
                    trigger_detail=current_remaining_source_detail if isinstance(current_remaining_source_detail, dict) else {},
                )
                if not bool(current_remaining_hybrid_request_validation.get("valid")):
                    self.log_sig.emit(
                        "[WARN] 현재 임무 collaborative hybrid 비활성: "
                        f"requestValidation={current_remaining_hybrid_request_validation}"
                    )
                    current_remaining_hybrid_request = None
                else:
                    self.log_sig.emit(
                        "[INFO] 현재 임무 collaborative hybrid request validation: "
                        f"{current_remaining_hybrid_request_validation}"
                    )
            if current_remaining_hybrid_request is not None:
                hybrid_scope_ordinals = sorted(
                    int(value)
                    for value in (getattr(current_remaining_hybrid_request, "apply_option_ordinals", None) or [])
                )
                hybrid_scope_text = (
                    ", ".join(str(value) for value in hybrid_scope_ordinals)
                    if hybrid_scope_ordinals
                    else "all"
                )
                self.log_sig.emit(
                    "[INFO] 현재 임무 collaborative hybrid armed: "
                    f"inputMissionID={current_remaining_hybrid_request.current_input_id}, "
                    f"sourceTemplateInputID={getattr(current_remaining_hybrid_request, 'source_template_input_id', None)}, "
                    f"sourcePlan={current_remaining_hybrid_request.source_plan_id}, "
                    f"mode={getattr(current_remaining_hybrid_request, 'planner_mode', 'current_remaining')}, "
                    f"aircraft={sorted(current_remaining_hybrid_request.entry_coord_map.keys())}, "
                    f"representativeEntry={current_remaining_hybrid_request.representative_entry}, "
                    f"options={hybrid_scope_text}"
                )
            elif current_remaining_source_detail is not None:
                entry_rows = current_remaining_source_detail.get("entryAircraftList")
                self.log_sig.emit(
                    "[WARN] 현재위치 첫 임무 hybrid 비활성: "
                    f"sourcePlan={snapshot_source_plan_id}, "
                    f"currentInputMissionID={current_remaining_source_detail.get('currentInputMissionID')}, "
                    f"entryAircraft={len(entry_rows) if isinstance(entry_rows, list) else 0}"
                )
            self._record_replan_timing_event(
                "snapshot_hybrid_prep_done",
                extra={
                    "snapshot_apply": int(snapshot_apply_result.get("applied") or 0),
                    "current_hybrid": int(current_remaining_hybrid_request is not None),
                    "checkpoint": "snapshot_hybrid_prep_done",
                },
            )

            def _current_remaining_request_for_variant(
                request: CurrentRemainingHybridRequest | None,
                variant_no: int,
            ) -> CurrentRemainingHybridRequest | None:
                if request is None:
                    return None
                apply_ordinals = getattr(request, "apply_option_ordinals", None)
                if apply_ordinals:
                    try:
                        variant_ordinal = int(variant_no)
                    except Exception:
                        return None
                    if int(variant_ordinal) not in {int(value) for value in apply_ordinals}:
                        return None
                return copy.deepcopy(request)

            def _persist_internal_replan_input_snapshot(payload: Dict[str, Any], suffix: str) -> None:
                if not isinstance(payload, dict):
                    return
                mission_items = payload.get("inputMissionList")
                if not isinstance(mission_items, list):
                    return
                try:
                    dss_dir = db_root / "DSS_Internal" / "replan_inputs"
                    dss_dir.mkdir(parents=True, exist_ok=True)
                    source_tag = f"source{int(snapshot_source_plan_id)}" if snapshot_source_plan_id else "source0"
                    path = dss_dir / f"0201_override_{source_tag}_{suffix}.json"
                    write_debug_json(path, payload, pretty=True, ensure_ascii=False, skip_if_unchanged=False)
                except Exception as exc:
                    self.log_sig.emit(f"[WARN] 내부 재계획 0201 로그 저장 실패 ({suffix}): {exc}")

            def _filter_input_missions_payload(payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
                mission_list_local = payload.get("inputMissionList") if isinstance(payload, dict) else None
                if not isinstance(mission_list_local, list):
                    return None
                original_count = len(mission_list_local)

                snapshot_apply_result: Dict[str, Any] = {
                    "applied": 0,
                    "marked_done": 0,
                    "snapshotMissionCount": 0,
                }
                snapshot_mutated = False
                collapse_apply_result: Dict[str, Any] = {
                    "mutated": False,
                    "groupCount": 0,
                    "removedInputMissionIDs": [],
                    "normalizedInputMissionIDs": [],
                }
                if snapshot_should_apply and isinstance(payload, dict):
                    snapshot_apply_result = _override_input_missions_with_remaining_snapshot(
                        payload,
                        source_plan_id=snapshot_source_plan_id,
                        mission_whitelist=snapshot_mission_whitelist,
                        audit_context=snapshot_audit_context,
                    )
                    snapshot_mutated = bool(
                        int(snapshot_apply_result.get("applied") or 0) > 0
                        or int(snapshot_apply_result.get("marked_done") or 0) > 0
                    )
                    collapse_apply_result = _collapse_input_missions_for_replan(
                        payload,
                        mission_whitelist=snapshot_mission_whitelist,
                    )
                    mission_list_local = payload.get("inputMissionList") if isinstance(payload, dict) else None
                    if not isinstance(mission_list_local, list):
                        return None

                filtered_list: List[Dict[str, Any]] = []
                removed_ids: list[str] = []
                converted_ids: list[str] = []
                width_adjusted_ids: list[str] = []
                skipped_single_point_ids: list[str] = []
                active_ids: list[int] = []

                for mission in mission_list_local:
                    if not isinstance(mission, dict):
                        continue
                    mid_raw = mission.get("inputMissionID")
                    try:
                        mid_int = int(mid_raw)
                    except Exception:
                        mid_int = None
                    if _is_single_point_coordinate_only_mission(mission):
                        skipped_single_point_ids.append(str(mid_raw))
                        removed_ids.append(str(mid_raw))
                        continue
                    if mission_whitelist and (mid_int is None or mid_int not in mission_whitelist):
                        removed_ids.append(str(mid_raw))
                        continue
                    if bool(mission.get("isDone")):
                        removed_ids.append(str(mid_raw))
                        continue

                    mtype = mission.get("inputMissionType")
                    if not isinstance(mtype, int) or mtype == 0:
                        detail = mission.get("missionDetail") or {}
                        inferred_type = _infer_input_mission_type(detail)
                        if inferred_type is None:
                            removed_ids.append(str(mid_raw))
                            continue
                        mission["inputMissionType"] = inferred_type
                        mtype = inferred_type
                        converted_ids.append(f"{mid_raw}->{inferred_type}")

                    if mtype in (1, 7):
                        detail = mission.get("missionDetail") or {}
                        for entry in detail.get("lineList") or []:
                            try:
                                width_val = float(entry.get("width", 0))
                            except Exception:
                                width_val = 0.0
                            if width_val <= 0:
                                entry["width"] = 1 if mtype == 7 else 1000
                                width_adjusted_ids.append(str(mid_raw))

                    filtered_list.append(mission)
                    if mid_int is not None:
                        active_ids.append(mid_int)

                return {
                    "filtered_list": filtered_list,
                    "removed_ids": removed_ids,
                    "converted_ids": converted_ids,
                    "width_adjusted_ids": width_adjusted_ids,
                    "skipped_single_point_ids": skipped_single_point_ids,
                    "active_ids": active_ids,
                    "original_count": int(original_count),
                    "snapshot_apply_result": snapshot_apply_result,
                    "snapshot_mutated": snapshot_mutated,
                    "collapse_apply_result": collapse_apply_result,
                    "collapse_mutated": bool(collapse_apply_result.get("mutated")),
                }

            filtered_cmpk_path = cmpk_path
            base_filtered_payload_materialized = True
            mission_list = cmpk_data.get("inputMissionList") if isinstance(cmpk_data, dict) else None
            self._record_replan_timing_event(
                "input_filter_start",
                extra={"checkpoint": "input_filter_start"},
            )
            if isinstance(mission_list, list):
                filtered_list = []
                removed_ids: list[str] = []
                converted_ids: list[str] = []
                width_adjusted_ids: list[str] = []
                skipped_single_point_ids: list[str] = []
                active_ids: list[int] = []
                for mission in mission_list:
                    mid_raw = mission.get("inputMissionID")
                    try:
                        mid_int = int(mid_raw)
                    except Exception:
                        mid_int = None
                    if _is_single_point_coordinate_only_mission(mission):
                        skipped_single_point_ids.append(str(mid_raw))
                        removed_ids.append(str(mid_raw))
                        continue
                    if mission_whitelist and (mid_int is None or mid_int not in mission_whitelist):
                        removed_ids.append(str(mid_raw))
                        continue
                    if bool(mission.get("isDone")):
                        removed_ids.append(str(mid_raw))
                        continue
                    mtype = mission.get("inputMissionType")
                    if not isinstance(mtype, int) or mtype == 0:
                        detail = mission.get("missionDetail") or {}
                        inferred_type = _infer_input_mission_type(detail)
                        if inferred_type is None:
                            removed_ids.append(str(mid_raw))
                            continue
                        mission["inputMissionType"] = inferred_type
                        mtype = inferred_type
                        converted_ids.append(f"{mid_raw}->{inferred_type}")
                    if mtype in (1, 7):
                        detail = mission.get("missionDetail") or {}
                        for entry in detail.get("lineList") or []:
                            try:
                                width_val = float(entry.get("width", 0))
                            except Exception:
                                width_val = 0.0
                            if width_val <= 0:
                                entry["width"] = 1 if mtype == 7 else 1000
                                width_adjusted_ids.append(str(mid_raw))
                    filtered_list.append(mission)
                    if mid_int is not None:
                        active_ids.append(mid_int)
                if not filtered_list:
                    plan_log_status = "skipped"
                    _record_issue(
                        "0201_filter_empty",
                        "filtered 0201 has no missions",
                        detail={"removed": removed_ids, "mission_whitelist": sorted(mission_whitelist) if mission_whitelist else []},
                        status="skipped",
                    )
                    plan_log_summary.update({
                        "stop_reason": "0201_filter_empty",
                        "removed_input_ids": removed_ids,
                        "mission_whitelist": sorted(mission_whitelist) if mission_whitelist else [],
                    })
                    self.log_sig.emit("[WARN] No pending missions remain after filtering; skipping replan pipeline.")
                    self._push_replan_noop_completion(reason, "재계획 불필요")
                    self._plan_status = "replan_skipped"
                    self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                    return
                if active_ids:
                    ctx["mission_ids"] = active_ids

                base_snapshot_mutated = bool(
                    int(snapshot_apply_result.get("applied") or 0) > 0
                    or int(snapshot_apply_result.get("marked_done") or 0) > 0
                )
                base_collapse_mutated = bool(collapse_apply_result.get("mutated"))
                needs_base_filtered_payload = (
                    base_snapshot_mutated
                    or base_collapse_mutated
                    or len(filtered_list) != len(mission_list)
                    or bool(converted_ids)
                    or bool(width_adjusted_ids)
                    or (mission_whitelist and set(active_ids) != mission_whitelist)
                )
                if needs_base_filtered_payload:
                    base_filtered_payload_materialized = False
                    cmpk_data["inputMissionList"] = filtered_list
                    filtered_dir = out_root_base / "_filtered"
                    filtered_dir.mkdir(parents=True, exist_ok=True)
                    filtered_cmpk_path = filtered_dir / cmpk_path.name
                    try:
                        filtered_cmpk_path.write_text(
                            json.dumps(cmpk_data, ensure_ascii=False, indent=2),
                            encoding="utf-8",
                        )
                        cmpk_path = filtered_cmpk_path
                        base_filtered_payload_materialized = True
                    except Exception as exc:
                        self.log_sig.emit(f"[WARN] Failed to persist filtered 0201 snapshot: {exc}")
                    else:
                        _persist_internal_replan_input_snapshot(cmpk_data, "base")
                        removed_summary = ", ".join(removed_ids) if removed_ids else "-"
                        converted_summary = ", ".join(converted_ids) if converted_ids else "-"
                        width_summary = ", ".join(width_adjusted_ids) if width_adjusted_ids else "-"
                        skipped_single_point_summary = ", ".join(skipped_single_point_ids) if skipped_single_point_ids else "-"
                        snapshot_summary = (
                            f"updated={int(snapshot_apply_result.get('applied') or 0)}, "
                            f"done={int(snapshot_apply_result.get('marked_done') or 0)}"
                            if base_snapshot_mutated
                            else "-"
                        )
                        collapse_summary = (
                            f"groups={int(collapse_apply_result.get('groupCount') or 0)}, "
                            f"normalized={','.join(str(v) for v in (collapse_apply_result.get('normalizedInputMissionIDs') or [])) or '-'}"
                            if base_collapse_mutated
                            else "-"
                        )
                        self.log_sig.emit(
                            "[INFO] Filtered completed input missions "
                            f"(removed={removed_summary or '-'}, converted={converted_summary or '-'}, "
                            f"widthAdjusted={width_summary or '-'}, skippedSinglePoint={skipped_single_point_summary or '-'}, "
                            f"snapshot={snapshot_summary}, collapse={collapse_summary})"
                        )
            else:
                self.log_sig.emit("[WARN] 0201 payload missing valid inputMissionList; continuing without filtering")
            self._record_replan_timing_event(
                "input_filter_done",
                extra={"checkpoint": "input_filter_done"},
            )

            filtered_cmpk_cache: Dict[str, Path] = {}
            try:
                filtered_cmpk_cache[str(Path(cmpk_path).resolve())] = Path(cmpk_path)
            except Exception:
                filtered_cmpk_cache[str(cmpk_path)] = Path(cmpk_path)

            ctx['cmpk_path'] = str(cmpk_path)
            ctx['mrpk_path'] = str(mrpk_path)

            self._record_replan_timing_event(
                "general_options_prepare_start",
                extra={"checkpoint": "general_options_prepare_start"},
            )
            plan_ids_source = ctx.get('plan_ids') or staged.get('plan_ids') or []
            plan_ids: list[int | None] = []
            for val in plan_ids_source:
                try:
                    plan_ids.append(int(val))
                except Exception:
                    plan_ids.append(None)

            raw_option_values = list(ctx.get('option_names') or staged.get('option_names') or [])
            plan_count = max(len(plan_ids), len(raw_option_values), 1)
            while len(plan_ids) < plan_count:
                plan_ids.append(None)

            def _warn_unknown_option(idx: int, raw: object, fallback_code: int) -> None:
                self.log_sig.emit(
                    f"[WARN] Unknown option label #{idx + 1}: "
                    f"{raw!r}; fallback optionCode={fallback_code}({option_code_to_label(fallback_code)})"
                )

            option_codes = ensure_option_code_sequence(
                raw_option_values,
                plan_count,
                on_unknown=_warn_unknown_option,
            )
            option_labels: List[str] = []
            for idx in range(plan_count):
                label = ""
                if idx < len(raw_option_values):
                    value = raw_option_values[idx]
                    label = str(value).strip() if value is not None else ""
                option_labels.append(label)

            base_runtime_payload = load_runtime_settings()
            base_runtime_values = (
                dict((base_runtime_payload.get("values") or {}))
                if isinstance(base_runtime_payload, dict)
                else {}
            )
            self._record_replan_timing_event(
                "runtime_settings_loaded",
                extra={"checkpoint": "runtime_settings_loaded"},
            )

            def _ctx_trigger_type_is_input_refresh() -> bool:
                for source in (ctx, staged):
                    if not isinstance(source, dict):
                        continue
                    detail = source.get("replan_detail") or source.get("replanDetail")
                    if not isinstance(detail, dict):
                        continue
                    trigger = str(detail.get("trigger") or "").strip()
                    trigger_type = str(detail.get("triggerType") or "").strip()
                    trigger_type_key = trigger_type.lower()
                    if "inputrefresh" in trigger_type_key:
                        return True
                    if trigger == "0201" and not trigger_type:
                        return True
                return False

            def _base_bool_value(key: str, default: bool) -> bool:
                raw = base_runtime_values.get(key, default)
                if isinstance(raw, str):
                    text = raw.strip().lower()
                    if text in {"0", "false", "no", "off"}:
                        return False
                    if text in {"1", "true", "yes", "on"}:
                        return True
                return bool(raw)

            def _base_int_value(key: str, default: int) -> int:
                try:
                    return int(float(base_runtime_values.get(key, default)))
                except Exception:
                    return int(default)

            def _base_float_value(key: str, default: float) -> float:
                try:
                    return float(base_runtime_values.get(key, default))
                except Exception:
                    return float(default)

            input_refresh_context = bool(_ctx_trigger_type_is_input_refresh())
            input_refresh_fast_dem_context = bool(
                input_refresh_context
                and _base_bool_value("input_refresh_fast_dem_enabled", True)
            )

            def _apply_input_refresh_runtime_profile(payload: Dict[str, Any]) -> Dict[str, Any]:
                if not input_refresh_context:
                    return payload
                values = payload.get("values")
                if not isinstance(values, dict):
                    values = {}
                    payload["values"] = values
                input_refresh_uav_wp_interval_m = max(
                    1.0,
                    _base_float_value("uav_wp_interval_m", 2000.0),
                )
                values.update(
                    {
                        # inputRefresh must reuse the initial planner's route
                        # geometry.  Partial carry-forward removes intervening
                        # Line missions from the 0303 input, so Area packets can
                        # otherwise look falsely adjacent and gain Dubins links.
                        "uav_wp_interval_m": float(input_refresh_uav_wp_interval_m),
                        "input_refresh_preserve_initial_geometry_enabled": True,
                        "area_dubins_entry_links_enabled": False,
                    }
                )
                if not input_refresh_fast_dem_context:
                    return payload
                values.update(
                    {
                        "dem_alt_cache_round_decimals": max(
                            0,
                            min(
                                7,
                                _base_int_value(
                                    "input_refresh_dem_alt_cache_round_decimals",
                                    0,
                                ),
                            ),
                        ),
                        "ground_required_sample_step_m": max(
                            1.0,
                            _base_float_value("input_refresh_ground_required_sample_step_m", 4000.0),
                        ),
                        "dense_linesearch_ground_sample_step_m": max(
                            1.0,
                            _base_float_value(
                                "input_refresh_dense_linesearch_ground_sample_step_m",
                                4000.0,
                            ),
                        ),
                        "dense_linesearch_ground_sample_min_interp_points": max(
                            1,
                            _base_int_value(
                                "input_refresh_dense_linesearch_ground_sample_min_interp_points",
                                2,
                            ),
                        ),
                        "ground_required_line_coord_fast_path_min_count": max(
                            2,
                            _base_int_value(
                                "input_refresh_ground_required_line_coord_fast_path_min_count",
                                2,
                            ),
                        ),
                        "ground_required_line_coord_fast_path_step_tolerance": max(
                            1.0,
                            _base_float_value(
                                "input_refresh_ground_required_line_coord_fast_path_step_tolerance",
                                5000.0,
                            ),
                        ),
                        "replan_0303_aircraft_process_parallel_enabled": _base_bool_value(
                            "input_refresh_0303_process_parallel_enabled",
                            False,
                        ),
                        "replan_0303_aircraft_process_workers": max(
                            1,
                            _base_int_value("input_refresh_0303_process_workers", 3),
                        ),
                        "ground_required_coord_altitude_fast_path_enabled": True,
                        "ground_required_trusted_coord_altitude_fast_path_enabled": True,
                    }
                )
                return payload

            if input_refresh_context:
                self.log_sig.emit(
                    "[REPLAN][INPUT_REFRESH] initial geometry profile enabled "
                    f"(routeWp={_base_float_value('uav_wp_interval_m', 2000.0):.1f}m, "
                    f"sweepInterp={_base_int_value('sweep_line_interp_points', 3)}, "
                    f"lineDensity={_base_float_value('line_density_scale', 1.4):.2f}, "
                    f"areaDensity={_base_float_value('area_density_scale', 1.4):.2f}, "
                    "areaReview=initial, areaDubins=0, "
                    f"fastDem={int(input_refresh_fast_dem_context)}, "
                    f"process0303={int(_base_bool_value('input_refresh_0303_process_parallel_enabled', False))})"
                )

            def _variant_runtime_override_payload(option_code: int, option_label: str) -> Optional[Dict[str, Any]]:
                payload = copy.deepcopy(base_runtime_payload) if isinstance(base_runtime_payload, dict) else {}
                if is_recon_specialized_option(option_code, option_label) and not input_refresh_context:
                    payload = build_recon_specialized_runtime_payload(payload)
                if input_refresh_context:
                    payload = _apply_input_refresh_runtime_profile(payload)
                return payload or None

            def _current_remaining_request_share_key(request: CurrentRemainingHybridRequest | None) -> Optional[str]:
                if request is None:
                    return None
                entry_rows = [
                    (
                        int(aid),
                        round(float(coord.get("latitude", 0.0) or 0.0), 7),
                        round(float(coord.get("longitude", 0.0) or 0.0), 7),
                        round(float(coord.get("altitude", 0.0) or 0.0), 2),
                    )
                    for aid, coord in sorted((request.entry_coord_map or {}).items())
                    if isinstance(coord, dict)
                ]
                heading_rows = [
                    (int(aid), round(float(value), 3))
                    for aid, value in sorted((request.heading_map or {}).items())
                ]
                context_rows = [
                    (
                        int(aid),
                        row.get("currentCoordinate"),
                        row.get("speedMps"),
                        row.get("turnSign"),
                        row.get("turnRadiusM"),
                    )
                    for aid, row in sorted((request.entry_aircraft_context_map or {}).items())
                    if isinstance(row, dict)
                ]
                key_payload = {
                    "sourcePlanID": int(request.source_plan_id),
                    "currentInputMissionID": int(request.current_input_id),
                    "sourceTemplateInputID": _safe_int_value(request.source_template_input_id),
                    "plannerMode": str(getattr(request, "planner_mode", "") or "current_remaining"),
                    "entryAircraftList": entry_rows,
                    "headingList": heading_rows,
                    "entryAircraftContextList": context_rows,
                    "representativeEntry": request.representative_entry,
                }
                return json.dumps(key_payload, sort_keys=True, ensure_ascii=False)

            def _current_remaining_runtime_share_key(option_code: int, option_label: str) -> str:
                payload = _variant_runtime_override_payload(int(option_code), option_label)
                if not isinstance(payload, dict):
                    return "{}"
                return json.dumps(payload, sort_keys=True, ensure_ascii=False)

            def _evaluate_current_remaining_hybrid_share_policy() -> Dict[str, Any]:
                active_ordinals: List[int] = []
                request_keys: Set[str] = set()
                runtime_keys: Set[str] = set()
                raw_runtime_keys: Set[str] = set()
                non_recon_runtime_groups: Dict[str, List[int]] = {}
                recon_ordinals: List[int] = []
                non_recon_ordinals: List[int] = []
                recon_neutralized_ordinals: List[int] = []
                if current_remaining_hybrid_request is None:
                    return {
                        "candidate": False,
                        "shareAllowed": False,
                        "shareAllowedNonRecon": False,
                        "reason": "request_unavailable",
                        "shareStrategy": "none",
                        "activeOptionOrdinals": active_ordinals,
                        "reconOptionOrdinals": recon_ordinals,
                        "sharedVariantOrdinals": [],
                        "isolatedVariantOrdinals": [],
                        "isolationReason": "request_unavailable",
                        "runtimeKeyCount": 0,
                        "sharedRuntimeGroupSize": 0,
                        "plannerMode": "current_remaining",
                        "sourceCurrentDifferent": False,
                        "reexecuteGenericSkipInputMissionID": None,
                        "reconRuntimeOverrideAffectsHybridSharing": False,
                        "currentMissionGeometry": "",
                        "rawRuntimeKeyCount": 0,
                        "reexecuteLineReconHybridShareEnabled": False,
                        "reexecuteLineReconRuntimeNeutralized": False,
                        "reexecuteLineReconNeutralizedOrdinals": [],
                    }
                planner_mode_text = str(
                    getattr(current_remaining_hybrid_request, "planner_mode", "")
                    or "current_remaining"
                )
                source_template_id = _safe_int_value(
                    getattr(current_remaining_hybrid_request, "source_template_input_id", None)
                )
                current_input_id_for_share = _safe_int_value(
                    getattr(current_remaining_hybrid_request, "current_input_id", None)
                )
                source_current_different = (
                    planner_mode_text == "reexecute_first_mission"
                    and source_template_id is not None
                    and current_input_id_for_share is not None
                    and int(source_template_id) != int(current_input_id_for_share)
                )
                current_mission_geometry = _mission_geometry_bucket(
                    getattr(current_remaining_hybrid_request, "current_input_mission", {}) or {}
                )

                def _boolish(value: object, default: bool) -> bool:
                    if isinstance(value, str):
                        text = value.strip().lower()
                        if text in {"0", "false", "no", "off"}:
                            return False
                        if text in {"1", "true", "yes", "on"}:
                            return True
                    if value is None:
                        return bool(default)
                    return bool(value)

                line_recon_hybrid_share_enabled = _boolish(
                    os.environ.get("REPLAN_REEXECUTE_LINE_RECON_HYBRID_SHARE"),
                    _boolish(
                        base_runtime_values.get(
                            "replan_reexecute_line_recon_hybrid_share_enabled",
                            True,
                        ),
                        True,
                    ),
                )
                reexecute_line_current_share = (
                    planner_mode_text == "reexecute_first_mission"
                    and str(current_mission_geometry) == "line"
                    and bool(line_recon_hybrid_share_enabled)
                )
                for local_idx in range(plan_count):
                    variant_request = _current_remaining_request_for_variant(
                        current_remaining_hybrid_request,
                        local_idx + 1,
                    )
                    if variant_request is None:
                        continue
                    active_ordinals.append(local_idx + 1)
                    key = _current_remaining_request_share_key(variant_request)
                    if key is not None:
                        request_keys.add(str(key))
                    option_label = option_labels[local_idx] if local_idx < len(option_labels) else ""
                    runtime_key = _current_remaining_runtime_share_key(
                        int(option_codes[local_idx]),
                        option_label,
                    )
                    raw_runtime_keys.add(str(runtime_key))
                    hybrid_runtime_key = (
                        "reexecute_line_current_hybrid_runtime_neutral"
                        if reexecute_line_current_share
                        else str(runtime_key)
                    )
                    runtime_keys.add(str(hybrid_runtime_key))
                    is_recon_option = is_recon_specialized_option(option_codes[local_idx], option_label)
                    if is_recon_option:
                        recon_ordinals.append(local_idx + 1)
                        if not reexecute_line_current_share:
                            continue
                        recon_neutralized_ordinals.append(local_idx + 1)
                    else:
                        non_recon_ordinals.append(local_idx + 1)
                    if (not is_recon_option) or reexecute_line_current_share:
                        group_key = json.dumps(
                            {
                                "request": str(key or ""),
                                "runtime": str(hybrid_runtime_key),
                            },
                            sort_keys=True,
                            ensure_ascii=False,
                        )
                        non_recon_runtime_groups.setdefault(group_key, []).append(local_idx + 1)
                candidate = len(active_ordinals) > 1 and len(request_keys) == 1
                largest_non_recon_runtime_group = (
                    max(non_recon_runtime_groups.values(), key=len)
                    if non_recon_runtime_groups
                    else []
                )
                identical_input_refresh_runtime = bool(
                    input_refresh_context
                    and len(runtime_keys) == 1
                    and len(raw_runtime_keys) == 1
                )
                share_allowed = bool(
                    candidate
                    and len(runtime_keys) == 1
                    and (not recon_ordinals or identical_input_refresh_runtime)
                )
                share_allowed_non_recon = bool(candidate and len(largest_non_recon_runtime_group) > 1)
                if not active_ordinals:
                    reason_text = "no_active_variant"
                elif len(active_ordinals) <= 1:
                    reason_text = "single_variant"
                elif len(request_keys) != 1:
                    reason_text = "request_key_mismatch"
                elif len(runtime_keys) != 1 and not share_allowed_non_recon:
                    reason_text = "runtime_key_mismatch"
                elif recon_ordinals and reexecute_line_current_share and recon_neutralized_ordinals:
                    reason_text = "reexecute_line_recon_runtime_neutralized"
                elif recon_ordinals and identical_input_refresh_runtime:
                    reason_text = "input_refresh_runtime_identical"
                elif recon_ordinals:
                    reason_text = "recon_runtime_override"
                else:
                    reason_text = "same_entry_current_source"
                runtime_override_affects_hybrid = (
                    planner_mode_text == "reexecute_first_mission"
                    and reason_text == "recon_runtime_override"
                )
                if share_allowed:
                    shared_ordinals = list(active_ordinals)
                    isolated_ordinals: List[int] = []
                    isolation_reason = "-"
                    share_strategy = "all_shared"
                elif share_allowed_non_recon:
                    shared_ordinals = list(largest_non_recon_runtime_group)
                    isolated_ordinals = [
                        value
                        for value in active_ordinals
                        if value not in set(shared_ordinals)
                    ]
                    if reexecute_line_current_share and recon_neutralized_ordinals:
                        isolation_reason = (
                            "reexecute_line_recon_runtime_neutralized"
                            if not isolated_ordinals
                            else "reexecute_line_recon_runtime_partially_neutralized"
                        )
                        share_strategy = (
                            "all_shared_reexecute_line_recon_neutralized"
                            if not isolated_ordinals
                            else "line_recon_neutralized_shared"
                        )
                    else:
                        isolation_reason = "recon_runtime_override"
                        share_strategy = "non_recon_shared_recon_isolated"
                else:
                    shared_ordinals = []
                    isolated_ordinals = list(active_ordinals)
                    isolation_reason = reason_text
                    share_strategy = f"all_isolated_{reason_text}"
                return {
                    "candidate": bool(candidate),
                    "shareAllowed": bool(share_allowed),
                    "shareAllowedNonRecon": bool(share_allowed_non_recon),
                    "reason": reason_text,
                    "shareStrategy": share_strategy,
                    "activeOptionOrdinals": active_ordinals,
                    "reconOptionOrdinals": recon_ordinals,
                    "nonReconOptionOrdinals": non_recon_ordinals,
                    "sharedVariantOrdinals": shared_ordinals,
                    "isolatedVariantOrdinals": isolated_ordinals,
                    "isolationReason": isolation_reason,
                    "requestKeyCount": len(request_keys),
                    "runtimeKeyCount": len(runtime_keys),
                    "rawRuntimeKeyCount": len(raw_runtime_keys),
                    "identicalInputRefreshRuntime": bool(identical_input_refresh_runtime),
                    "sharedRuntimeGroupSize": len(shared_ordinals),
                    "plannerMode": planner_mode_text,
                    "currentMissionGeometry": str(current_mission_geometry or ""),
                    "sourceCurrentDifferent": bool(source_current_different),
                    "reexecuteGenericSkipInputMissionID": current_input_id_for_share
                    if planner_mode_text == "reexecute_first_mission"
                    else None,
                    "reconRuntimeOverrideAffectsHybridSharing": bool(runtime_override_affects_hybrid),
                    "reexecuteLineReconHybridShareEnabled": bool(line_recon_hybrid_share_enabled),
                    "reexecuteLineReconRuntimeNeutralized": bool(
                        reexecute_line_current_share and recon_neutralized_ordinals
                    ),
                    "reexecuteLineReconNeutralizedOrdinals": list(recon_neutralized_ordinals),
                }

            current_remaining_hybrid_share_policy = _evaluate_current_remaining_hybrid_share_policy()
            self._record_replan_timing_event(
                "hybrid_share_policy_evaluated",
                extra={
                    "candidate": int(bool(current_remaining_hybrid_share_policy.get("candidate"))),
                    "active_options": len(current_remaining_hybrid_share_policy.get("activeOptionOrdinals") or []),
                    "reason": str(current_remaining_hybrid_share_policy.get("reason") or ""),
                    "checkpoint": "hybrid_share_policy_evaluated",
                },
            )
            if current_remaining_hybrid_request is not None:
                self.log_sig.emit(
                    "[INFO] current remaining hybrid share policy: "
                    f"{current_remaining_hybrid_share_policy}"
                )
                self._record_replan_timing_event(
                    "current_remaining_hybrid_share_policy",
                    extra={
                        "candidate": int(bool(current_remaining_hybrid_share_policy.get("candidate"))),
                        "share_allowed": int(bool(current_remaining_hybrid_share_policy.get("shareAllowed"))),
                        "reason": str(current_remaining_hybrid_share_policy.get("reason") or ""),
                        "share_strategy": str(current_remaining_hybrid_share_policy.get("shareStrategy") or ""),
                        "active_options": len(current_remaining_hybrid_share_policy.get("activeOptionOrdinals") or []),
                        "recon_options": len(current_remaining_hybrid_share_policy.get("reconOptionOrdinals") or []),
                        "non_recon_options": len(current_remaining_hybrid_share_policy.get("nonReconOptionOrdinals") or []),
                        "share_allowed_non_recon": int(
                            bool(current_remaining_hybrid_share_policy.get("shareAllowedNonRecon"))
                        ),
                        "shared_variants": "|".join(
                            str(value)
                            for value in (current_remaining_hybrid_share_policy.get("sharedVariantOrdinals") or [])
                        )
                        or "-",
                        "isolated_variants": "|".join(
                            str(value)
                            for value in (current_remaining_hybrid_share_policy.get("isolatedVariantOrdinals") or [])
                        )
                        or "-",
                        "isolation_reason": str(current_remaining_hybrid_share_policy.get("isolationReason") or ""),
                        "runtime_key_count": int(current_remaining_hybrid_share_policy.get("runtimeKeyCount") or 0),
                        "raw_runtime_key_count": int(current_remaining_hybrid_share_policy.get("rawRuntimeKeyCount") or 0),
                        "shared_runtime_group_size": int(
                            current_remaining_hybrid_share_policy.get("sharedRuntimeGroupSize") or 0
                        ),
                        "planner_mode": str(current_remaining_hybrid_share_policy.get("plannerMode") or ""),
                        "current_mission_geometry": str(
                            current_remaining_hybrid_share_policy.get("currentMissionGeometry") or ""
                        ),
                        "source_current_different": int(
                            bool(current_remaining_hybrid_share_policy.get("sourceCurrentDifferent"))
                        ),
                        "recon_runtime_override_affects_hybrid": int(
                            bool(
                                current_remaining_hybrid_share_policy.get(
                                    "reconRuntimeOverrideAffectsHybridSharing"
                                )
                            )
                        ),
                        "reexecute_line_recon_runtime_neutralized": int(
                            bool(
                                current_remaining_hybrid_share_policy.get(
                                    "reexecuteLineReconRuntimeNeutralized"
                                )
                            )
                        ),
                        "reexecute_line_recon_neutralized_ordinals": "|".join(
                            str(value)
                            for value in (
                                current_remaining_hybrid_share_policy.get(
                                    "reexecuteLineReconNeutralizedOrdinals"
                                )
                                or []
                            )
                        )
                        or "-",
                    },
                )
            attack_option_indices: Set[int] = {
                idx for idx, code in enumerate(option_codes) if int(code) == 2
            }
            attack_exclusion_option_indices: Set[int] = {
                idx for idx, code in enumerate(option_codes) if int(code) == 3
            }
            shared_attack_detail = ctx.get("replan_detail") if isinstance(ctx.get("replan_detail"), dict) else None
            shared_attack_context = (
                self._build_attack_context_from_replan_detail(shared_attack_detail) if shared_attack_detail else None
            )
            attack_cmpk_path: Optional[Path] = None
            if attack_option_indices:
                dss_dir = db_root / 'DSS_Internal'
                attack_candidates: List[Path] = []
                if dss_dir.exists():
                    attack_candidates.extend(sorted(dss_dir.glob("0201_*.json")))
                    attack_candidates.extend(sorted(dss_dir.glob("0201_attack*.json")))
                if attack_candidates:
                    attack_cmpk_path = max(attack_candidates, key=lambda p: p.stat().st_mtime)
                else:
                    legacy_attack = dss_dir / '0201_attack.json'
                    if legacy_attack.exists():
                        attack_cmpk_path = legacy_attack
                if attack_cmpk_path:
                    self.log_sig.emit(
                        f"[INFO] 공격 옵션에 {attack_cmpk_path.name} 적용: {sorted(idx + 1 for idx in attack_option_indices)}"
                    )
                elif shared_attack_context is None:
                    self.log_sig.emit("[WARN] 공격 옵션이 있으나 활용 가능한 대상 정보가 없어 기본 임무를 유지합니다.")
                    attack_option_indices.clear()

            prior_option_indices: Set[int] = {idx for idx, label in enumerate(option_labels) if label == "선행임무 재계획"}
            prior_variant_contexts: Dict[int, Dict[str, Any]] = {}
            if prior_option_indices:
                prior_plan = _locate_prior_mission_plan()
                if prior_plan is None:
                    self.log_sig.emit("[WARN] 선행임무 재계획 옵션이 있으나 DSS_Internal/0201_prior 데이터를 찾지 못했습니다.")
                    prior_option_indices.clear()
                else:
                    prior_path, prior_ctx = prior_plan
                    for idx in prior_option_indices:
                        prior_variant_contexts[idx] = {"path": prior_path, "context": prior_ctx}
                    self.log_sig.emit(
                        f"[INFO] 선행임무 재계획 옵션에 {prior_path.name} 적용: "
                        f"{sorted(i + 1 for i in prior_option_indices)}"
                    )

            self._record_replan_timing_event(
                "option_contexts_ready",
                extra={
                    "attack_options": len(attack_option_indices),
                    "prior_options": len(prior_option_indices),
                    "checkpoint": "option_contexts_ready",
                },
            )
            try:
                cmpk_id = int(Path(cmpk_path).stem)
            except Exception:
                cmpk_id = 0

            dir_mp = db_root / 'MissionPlan'
            dir_imp = db_root / 'IndividualMissionPlan'
            dir_fp = db_root / 'FlightPath'
            for directory in (dir_mp, dir_imp, dir_fp):
                directory.mkdir(parents=True, exist_ok=True)

            def _scan_existing_ids(target_dir: Path) -> set[int]:
                results: set[int] = set()
                try:
                    for item in target_dir.glob("*.json"):
                        stem = item.stem
                        if stem.isdigit():
                            try:
                                results.add(int(stem))
                            except Exception:
                                continue
                except Exception:
                    pass
                return results

            used_plan_ids: set[int] = _scan_existing_ids(dir_mp)

            def _allocate_plan_id(preferred: int | None) -> int:
                if preferred is not None:
                    try:
                        preferred_id = int(preferred)
                    except Exception:
                        preferred_id = None
                    if preferred_id is not None and preferred_id > 0 and preferred_id not in used_plan_ids:
                        used_plan_ids.add(preferred_id)
                        return preferred_id
                while True:
                    reserved_plan_ids = reserve_mission_plan_ids(1)
                    if not reserved_plan_ids:
                        raise RuntimeError("missionPlanID allocation failed")
                    try:
                        assigned = int(reserved_plan_ids[0])
                    except Exception as exc:
                        raise RuntimeError("missionPlanID allocation returned invalid value") from exc
                    if assigned in used_plan_ids:
                        continue
                    used_plan_ids.add(assigned)
                    return assigned

            def _positive_int_or_none(value: Any) -> Optional[int]:
                try:
                    if value is None:
                        return None
                    parsed = int(value)
                except Exception:
                    return None
                return parsed if parsed > 0 else None

            def _option_dependent_isolation_contract(
                *,
                variant_no: int,
                option_code: int,
                mode: str,
            ) -> Dict[str, Any]:
                return {
                    "sharedBeforeValidation": False,
                    "validationBeforeStore": True,
                    "variant": int(variant_no),
                    "optionCode": int(option_code),
                    "mode": str(mode or ""),
                    "isolatedArtifacts": [
                        "splitResult",
                        "typeDeciderResult",
                        "expectedPaths",
                        "reconAreaReview",
                    ],
                }

            def _allocate_general_variant_plan_id(
                *,
                variant_no: int,
                option_code: int,
                requested_plan_id: Any,
                generated_plan_json: Dict[str, Any],
            ) -> tuple[int, Dict[str, Any]]:
                expected_plan_id = _positive_int_or_none(requested_plan_id)
                if expected_plan_id is not None:
                    if expected_plan_id in used_plan_ids:
                        raise RuntimeError(
                            "requested pending MissionPlanID already exists "
                            f"(variant={variant_no}, optionCode={option_code}, missionPlanID={expected_plan_id})"
                        )
                    plan_id = _allocate_plan_id(expected_plan_id)
                else:
                    plan_id = _allocate_plan_id(_positive_int_or_none(generated_plan_json.get("missionPlanID")))
                contract = {
                    "variant": int(variant_no),
                    "optionCode": int(option_code),
                    "requestedMissionPlanID": expected_plan_id,
                    "missionPlanID": int(plan_id),
                    "missionPlanIDMatchesRequest": expected_plan_id is None or int(plan_id) == int(expected_plan_id),
                }
                if not bool(contract["missionPlanIDMatchesRequest"]):
                    raise RuntimeError(
                        "MissionPlanID does not match requested pending plan ID "
                        f"(variant={variant_no}, optionCode={option_code}, "
                        f"requested={expected_plan_id}, actual={plan_id})"
                    )
                return int(plan_id), contract

            def _imp_id_reservation_count(plan_json: Dict[str, Any]) -> int:
                aircraft_list = plan_json.get("aircraftList")
                if not isinstance(aircraft_list, list):
                    return 0
                seen: Set[int] = set()
                count = 0
                for aircraft in aircraft_list:
                    if not isinstance(aircraft, dict):
                        continue
                    try:
                        aid = int(aircraft.get("aircraftID", 0))
                    except Exception:
                        continue
                    if aid < 1 or aid > 6 or aid in seen:
                        continue
                    seen.add(aid)
                    count += 1
                return int(count)

            def _individual_mission_reservation_count(missions: list[dict]) -> int:
                count = 0
                for mission in missions or []:
                    if not isinstance(mission, dict):
                        continue
                    try:
                        aid = int(mission.get("aircraftID", 0))
                    except Exception:
                        continue
                    if 1 <= aid <= 6:
                        count += 1
                return int(count)

            def _allocate_imp_id_map(
                plan_json: Dict[str, Any],
                *,
                reserved_imp_ids: list[int] | tuple[int, ...] | None = None,
            ) -> Dict[int, int]:
                allocated: Dict[int, int] = {}
                aircraft_list = plan_json.get("aircraftList")
                if not isinstance(aircraft_list, list):
                    return allocated
                pending_rows: list[tuple[dict, int]] = []
                for aircraft in aircraft_list:
                    if not isinstance(aircraft, dict):
                        continue
                    try:
                        aid = int(aircraft.get("aircraftID", 0))
                    except Exception:
                        continue
                    if aid < 1 or aid > 6 or aid in allocated:
                        continue
                    pending_rows.append((aircraft, aid))
                if not pending_rows:
                    return allocated
                reserved_pkg_ids = [
                    int(value)
                    for value in (reserved_imp_ids or [])
                    if value is not None
                ]
                if len(reserved_pkg_ids) < len(pending_rows):
                    reserved_pkg_ids = reserve_imp_ids(len(pending_rows))
                for (aircraft, aid), pkg_id in zip(pending_rows, reserved_pkg_ids):
                    pkg_id_int = int(pkg_id)
                    aircraft["individualMissionPackageID"] = pkg_id_int
                    allocated[aid] = pkg_id_int
                return allocated

            def _assign_fresh_path_ids(
                missions: list[dict],
                generated_ids: Set[int],
                *,
                preserve_mission_ids: Set[int] | None = None,
                reserved_path_ids_by_aircraft: Optional[Dict[int, list[int]]] = None,
            ) -> Dict[tuple[int, int], int]:
                preserve_set = {int(value) for value in (preserve_mission_ids or set())}
                pid_map: Dict[tuple[int, int], int] = {}
                grouped: Dict[int, list[dict]] = {}
                for im in missions:
                    try:
                        aid = int(im.get("aircraftID", 0))
                        mid = int(im.get("individualMissionID", 0))
                    except Exception:
                        continue
                    if aid < 1 or aid > 6:
                        continue
                    grouped.setdefault(aid, []).append(im)
                reserved_by_aircraft: Dict[int, list[int]] = {}
                for raw_aid, raw_values in (reserved_path_ids_by_aircraft or {}).items():
                    try:
                        aid_int = int(raw_aid)
                    except Exception:
                        continue
                    reserved_by_aircraft[aid_int] = [int(value) for value in (raw_values or [])]
                for aid, items in grouped.items():
                    reserved_path_ids_for_aid = reserved_by_aircraft.get(aid, [])
                    reserve_cursor = 0
                    for im in items:
                        try:
                            mid = int(im.get("individualMissionID", 0))
                        except Exception:
                            continue
                        if mid in preserve_set:
                            try:
                                pid_int = int(im.get("pathID", 0))
                            except Exception:
                                pid_int = 0
                            if pid_int > 0:
                                pid_map[(aid, mid)] = pid_int
                                generated_ids.add(pid_int)
                                continue
                        if reserve_cursor >= len(reserved_path_ids_for_aid):
                            raise RuntimeError(
                                "fresh pathID reservation missing "
                                f"(aircraftID={aid}, requiredIndex={reserve_cursor}, reserved={len(reserved_path_ids_for_aid)})"
                            )
                        pid = reserved_path_ids_for_aid[reserve_cursor]
                        reserve_cursor += 1
                        pid_int = int(pid)
                        im["pathID"] = pid_int
                        pid_map[(aid, mid)] = pid_int
                        generated_ids.add(pid_int)
                return pid_map

            def _fresh_path_id_reservation_plan(
                missions: list[dict],
                *,
                preserve_mission_ids: Set[int] | None = None,
            ) -> Dict[int, int]:
                preserve_set = {int(value) for value in (preserve_mission_ids or set())}
                counts: Dict[int, int] = {}
                for im in missions or []:
                    try:
                        aid = int(im.get("aircraftID", 0))
                        mid = int(im.get("individualMissionID", 0))
                    except Exception:
                        continue
                    if aid < 1 or aid > 6:
                        continue
                    if mid in preserve_set:
                        try:
                            existing_pid = int(im.get("pathID", 0))
                        except Exception:
                            existing_pid = 0
                        if existing_pid > 0:
                            continue
                    counts[aid] = counts.get(aid, 0) + 1
                return counts

            def _reserve_fresh_path_ids_for_missions(
                missions: list[dict],
                *,
                preserve_mission_ids: Set[int] | None = None,
            ) -> Dict[int, list[int]]:
                reservation_plan = _fresh_path_id_reservation_plan(
                    missions,
                    preserve_mission_ids=preserve_mission_ids,
                )
                reserved: Dict[int, list[int]] = {}
                if reservation_plan:
                    try:
                        bulk_reserved = reserve_path_id_blocks(
                            {int(aid): int(count) for aid, count in reservation_plan.items() if int(count) > 0}
                        )
                    except Exception:
                        bulk_reserved = {}
                    for aid, count in reservation_plan.items():
                        if count <= 0:
                            continue
                        values = [int(value) for value in (bulk_reserved.get(int(aid)) or [])]
                        if not values:
                            values = [int(value) for value in reserve_path_ids(int(aid), int(count))]
                        if len(values) < count:
                            raise RuntimeError(
                                "fresh pathID reservation short "
                                f"(aircraftID={aid}, required={count}, reserved={len(values)})"
                            )
                        reserved[int(aid)] = values
                return reserved

            def _collect_valid_path_ids(fps):
                collected: Set[int] = set()
                for fp in fps or []:
                    path_id = fp.get("pathID")
                    if path_id is None:
                        continue
                    if fp.get("isFormationFlight"):
                        try:
                            collected.add(int(path_id))
                        except Exception:
                            pass
                        continue
                    waypoints = fp.get("waypointList")
                    if not waypoints:
                        waypoints = fp.get("lahWaypointList")
                    if not waypoints:
                        continue
                    try:
                        pid_int = int(path_id)
                    except Exception:
                        continue
                    if pid_int > 0:
                        collected.add(pid_int)
                return collected

            def _flight_plan_waypoints(fp: Any) -> list:
                if not isinstance(fp, dict):
                    return []
                waypoints = fp.get("waypointList")
                if isinstance(waypoints, list):
                    return waypoints
                waypoints = fp.get("lahWaypointList")
                if isinstance(waypoints, list):
                    return waypoints
                return []

            def _summarize_flight_plan_metrics(fps: Any) -> Dict[str, int]:
                summary = {
                    "paths": 0,
                    "waypoints": 0,
                    "max_waypoints": 0,
                    "empty_paths": 0,
                    "formation_paths": 0,
                }
                if not isinstance(fps, list):
                    return summary
                summary["paths"] = len(fps)
                for fp in fps:
                    if not isinstance(fp, dict):
                        continue
                    waypoints = _flight_plan_waypoints(fp)
                    wp_count = len(waypoints)
                    summary["waypoints"] += wp_count
                    summary["max_waypoints"] = max(summary["max_waypoints"], wp_count)
                    if wp_count <= 0:
                        summary["empty_paths"] += 1
                    if bool(fp.get("isFormationFlight")):
                        summary["formation_paths"] += 1
                return summary

            def _emit_flightpath_metric(
                emit: Callable[[str], None],
                *,
                variant_no: int,
                option_code: int,
                mode: str,
                flight_plans_0303: list[dict],
                flight_plans_0304: list[dict],
            ) -> None:
                summary3 = _summarize_flight_plan_metrics(flight_plans_0303)
                summary4 = _summarize_flight_plan_metrics(flight_plans_0304)
                total_paths = int(summary3["paths"]) + int(summary4["paths"])
                total_waypoints = int(summary3["waypoints"]) + int(summary4["waypoints"])
                max_waypoints = max(int(summary3["max_waypoints"]), int(summary4["max_waypoints"]))
                empty_paths = int(summary3["empty_paths"]) + int(summary4["empty_paths"])
                formation_paths = int(summary3["formation_paths"]) + int(summary4["formation_paths"])
                recon_flag = 1 if is_recon_specialized_option(option_code, None) else 0
                emit(
                    "[REPLAN][METRIC] flightpath_counts "
                    f"variant={int(variant_no)} option={int(option_code)} recon={recon_flag} mode={mode} "
                    f"paths0303={int(summary3['paths'])} paths0304={int(summary4['paths'])} pathsTotal={total_paths} "
                    f"waypoints0303={int(summary3['waypoints'])} waypoints0304={int(summary4['waypoints'])} "
                    f"waypointsTotal={total_waypoints} maxWaypointsPerPath={max_waypoints} "
                    f"emptyPaths={empty_paths} formationPaths={formation_paths}"
                )

            def _emit_flightpath_write_metric(
                emit: Callable[[str], None],
                *,
                variant_no: int,
                option_code: int,
                mode: str,
                files_0303: int,
                files_0304: int,
                write_ms: float,
            ) -> None:
                recon_flag = 1 if is_recon_specialized_option(option_code, None) else 0
                emit(
                    "[REPLAN][METRIC] flightpath_write "
                    f"variant={int(variant_no)} option={int(option_code)} recon={recon_flag} mode={mode} "
                    f"files0303={int(files_0303)} files0304={int(files_0304)} "
                    f"filesTotal={int(files_0303) + int(files_0304)} write_ms={float(write_ms):.3f}"
                )

            def _emit_0303_build_metric(
                emit: Callable[[str], None],
                *,
                variant_no: int,
                option_code: int,
                mode: str,
                build_result: Dict[str, Any],
            ) -> None:
                worker_ms = build_result.get("worker_ms_by_aircraft")
                worker_text = "-"
                if isinstance(worker_ms, dict) and worker_ms:
                    worker_text = "|".join(
                        f"{int(aid)}:{float(ms):.3f}"
                        for aid, ms in sorted(worker_ms.items(), key=lambda item: int(item[0]))
                    )
                group_ms = build_result.get("group_worker_ms")
                group_text = "-"
                if isinstance(group_ms, dict) and group_ms:
                    group_text = "|".join(
                        f"{str(group_id)}:{float(ms):.3f}"
                        for group_id, ms in sorted(group_ms.items(), key=lambda item: str(item[0]))
                    )
                fallback_reasons = build_result.get("fallback_reasons")
                fallback_text = "-"
                if isinstance(fallback_reasons, list) and fallback_reasons:
                    fallback_text = "|".join(str(item) for item in fallback_reasons)
                recon_flag = 1 if is_recon_specialized_option(option_code, None) else 0
                worker_policy = build_result.get("worker_policy")
                if not isinstance(worker_policy, dict):
                    worker_policy = {}
                line_counts = build_result.get("line_search_counts")
                if not isinstance(line_counts, dict):
                    line_counts = {}
                dense_metrics = build_result.get("dense_linesearch_metrics")
                if not isinstance(dense_metrics, dict):
                    dense_metrics = {}
                elapsed_0303_ms = float(build_result.get("elapsed_ms") or 0.0)
                build_workers = max(1, int(build_result.get("workers") or 1))
                phase_wall_divisor = float(build_workers)

                def _dense_ms(key: str) -> float:
                    try:
                        return float(dense_metrics.get(key) or 0.0)
                    except Exception:
                        return 0.0

                def _phase_ms(key: str) -> float:
                    phase_map = build_result.get("phase_ms")
                    if not isinstance(phase_map, dict):
                        return 0.0
                    try:
                        return float(phase_map.get(key) or 0.0)
                    except Exception:
                        return 0.0

                primary_phase_budget_ms = sum(
                    _dense_ms(key) / phase_wall_divisor
                    for key in (
                        "generateLineSearchMs",
                        "formationGroupingMs",
                        "missionPacketBuildMs",
                        "lineSearchMergeMs",
                        "groundRequiredComputeMs",
                        "filmingNormalizeMs",
                        "formationPostProcessMs",
                        "etaEcfMs",
                        "altitudeGuardMs",
                        "areaRepositionMs",
                        "areaPackMs",
                        "outputNormalizeMs",
                        "jsonReadyMs",
                    )
                )
                primary_phase_budget_ms += sum(
                    _phase_ms(key)
                    for key in (
                        "sort_by_input_order",
                        "normalize_timestamps",
                        "reassign_waypoint_ids",
                        "dependency_sort_by_input_order",
                        "dependency_normalize_timestamps",
                        "dependency_reassign_waypoint_ids",
                    )
                )
                unaccounted_0303_ms = max(0.0, elapsed_0303_ms - primary_phase_budget_ms)
                line_search_coordinate_cap = int(dense_metrics.get("lineSearchCoordinateCap") or 0)
                line_search_cap_exceeded_count = int(
                    dense_metrics.get("lineSearchCoordinateCapExceededCount") or 0
                )
                line_search_density_guard_passed = (
                    line_search_coordinate_cap <= 0
                    or (
                        int(line_counts.get("maxLineSearchCoords") or 0) <= int(line_search_coordinate_cap)
                        and line_search_cap_exceeded_count <= 0
                    )
                )
                emit(
                    "[REPLAN][METRIC] flightpath_build_0303 "
                    f"variant={int(variant_no)} option={int(option_code)} recon={recon_flag} mode={mode} "
                    f"buildMode={str(build_result.get('mode') or 'sequential')} "
                    f"workers={int(build_result.get('workers') or 1)} "
                    f"aircraft={int(build_result.get('aircraft') or 0)} "
                    f"dependencyGroups={int(build_result.get('dependency_groups') or 0)} "
                    f"formationGroups={int(build_result.get('formation_groups') or 0)} "
                    f"independentGroups={int(build_result.get('independent_groups') or 0)} "
                    f"dagGroups={int(build_result.get('dag_groups') or 0)} "
                    f"dagFallbackReason={str(build_result.get('dag_fallback_reason') or '-')} "
                    f"paths={len(build_result.get('plans') or [])} "
                    f"reassignedWaypoints={int(build_result.get('reassigned_waypoints') or 0)} "
                    f"waypointCountPrepass={int(build_result.get('waypoint_count_prepass') or 0)} "
                    f"legacyDefaultTwoWorkerBottleneck={int(bool(worker_policy.get('legacyDefaultTwoWorkerBottleneck')))} "
                    f"effectiveWorkerBottleneck={int(bool(worker_policy.get('effectiveWorkerBottleneck')))} "
                    f"lineSearchCount={int(line_counts.get('lineSearchCount') or 0)} "
                    f"lineSearchCoordCount={int(line_counts.get('lineSearchCoordCount') or 0)} "
                    f"lineSweepInterpolationEnabled={int(dense_metrics.get('lineSweepInterpolationEnabled') or 0)} "
                    f"lineSweepInterpolationRemovedCoords={int(dense_metrics.get('lineSweepInterpolationRemovedCoords') or 0)} "
                    f"maxLineSearchCoords={int(line_counts.get('maxLineSearchCoords') or 0)} "
                    f"lineSearchCoordinateCap={line_search_coordinate_cap} "
                    f"lineSearchCoordinateCapExceededCount={line_search_cap_exceeded_count} "
                    f"lineSearchDensityGuardPassed={int(bool(line_search_density_guard_passed))} "
                    f"lineSearchJsonBytes={int(line_counts.get('lineSearchJsonBytes') or 0)} "
                    f"demLookupCount={int(dense_metrics.get('demLookupCount') or 0)} "
                    f"demCacheHitCount={int(dense_metrics.get('demCacheHitCount') or 0)} "
                    f"demCacheMissCount={int(dense_metrics.get('demCacheMissCount') or 0)} "
                    f"demUniquePointCount={int(dense_metrics.get('demUniquePointCount') or 0)} "
                    f"demTileCount={int(dense_metrics.get('demTileCount') or 0)} "
                    f"demResolvedByTile={int(dense_metrics.get('demResolvedByTile') or 0)} "
                    f"demPixelCacheHitCount={int(dense_metrics.get('demPixelCacheHitCount') or 0)} "
                    f"demPixelCacheMissCount={int(dense_metrics.get('demPixelCacheMissCount') or 0)} "
                    f"demPixelCacheUniqueMissCount={int(dense_metrics.get('demPixelCacheUniqueMissCount') or 0)} "
                    f"demPixelCacheClearCount={int(dense_metrics.get('demPixelCacheClearCount') or 0)} "
                    f"demAltCacheClearCount={int(dense_metrics.get('demAltCacheClearCount') or 0)} "
                    f"demRunMapHitCount={int(dense_metrics.get('demRunMapHitCount') or 0)} "
                    f"demRunMapStoreCount={int(dense_metrics.get('demRunMapStoreCount') or 0)} "
                    f"demRunMapSize={int(dense_metrics.get('demRunMapSize') or 0)} "
                    f"demBatchFallbackCount={int(dense_metrics.get('demBatchFallbackCount') or 0)} "
                    f"demBatchFallbackReason={str(dense_metrics.get('demBatchFallbackReason') or '-')} "
                    f"demTileCandidateCount={int(dense_metrics.get('demTileCandidateCount') or 0)} "
                    f"demTileFallbackCandidateCount={int(dense_metrics.get('demTileFallbackCandidateCount') or 0)} "
                    f"demTileApplyCallCount={int(dense_metrics.get('demTileApplyCallCount') or 0)} "
                    f"demTileLoadLeaderCount={int(dense_metrics.get('demTileLoadLeaderCount') or 0)} "
                    f"demTileLoadWaiterCount={int(dense_metrics.get('demTileLoadWaiterCount') or 0)} "
                    f"demTileLoadTimeoutCount={int(dense_metrics.get('demTileLoadTimeoutCount') or 0)} "
                    f"groundProfileSampleCount={int(dense_metrics.get('groundProfileSampleCount') or 0)} "
                    f"groundPrepassMs={float(dense_metrics.get('groundPrepassMs') or 0.0):.3f} "
                    f"groundRequiredPrecomputeCount={int(dense_metrics.get('groundRequiredPrecomputeCount') or 0)} "
                    f"groundRequiredPrecomputeHitCount={int(dense_metrics.get('groundRequiredPrecomputeHitCount') or 0)} "
                    f"groundRequiredLineCoordReuseCount={int(dense_metrics.get('groundRequiredLineCoordReuseCount') or 0)} "
                    f"groundRequiredSignatureCacheHitCount={int(dense_metrics.get('groundRequiredSignatureCacheHitCount') or 0)} "
                    f"groundRequiredSignatureCacheStoreCount={int(dense_metrics.get('groundRequiredSignatureCacheStoreCount') or 0)} "
                    f"groundRequiredPreOwnerMs={float(dense_metrics.get('groundRequiredPreOwnerMs') or 0.0):.3f} "
                    f"groundRequiredPreOwnerSignatureCount={int(dense_metrics.get('groundRequiredPreOwnerSignatureCount') or 0)} "
                    f"groundRequiredPreOwnerPublishedCount={int(dense_metrics.get('groundRequiredPreOwnerPublishedCount') or 0)} "
                    f"groundRequiredPreOwnerHitCount={int(dense_metrics.get('groundRequiredPreOwnerHitCount') or 0)} "
                    f"groundRequiredPreOwnerSkippedCount={int(dense_metrics.get('groundRequiredPreOwnerSkippedCount') or 0)} "
                    f"groundRequiredMaterializationCacheHitCount={int(dense_metrics.get('groundRequiredMaterializationCacheHitCount') or 0)} "
                    f"groundRequiredMaterializationWaitMs={float(dense_metrics.get('groundRequiredMaterializationWaitMs') or 0.0):.3f} "
                    f"groundRequiredFinalPrepassFreshSkipCount={int(dense_metrics.get('groundRequiredFinalPrepassFreshSkipCount') or 0)} "
                    f"groundRequiredFinalPrepassFreshSkipMs={float(dense_metrics.get('groundRequiredFinalPrepassFreshSkipMs') or 0.0):.3f} "
                    f"groundRequiredProcessCacheHitCount={int(dense_metrics.get('groundRequiredProcessCacheHitCount') or 0)} "
                    f"groundRequiredProcessCacheStoreCount={int(dense_metrics.get('groundRequiredProcessCacheStoreCount') or 0)} "
                    f"groundRequiredProcessCacheInflightLeaderCount={int(dense_metrics.get('groundRequiredProcessCacheInflightLeaderCount') or 0)} "
                    f"groundRequiredProcessCacheInflightWaiterCount={int(dense_metrics.get('groundRequiredProcessCacheInflightWaiterCount') or 0)} "
                    f"groundRequiredProcessCacheInflightLocalFastPathCount={int(dense_metrics.get('groundRequiredProcessCacheInflightLocalFastPathCount') or 0)} "
                    f"groundRequiredProcessCacheInflightTimeoutCount={int(dense_metrics.get('groundRequiredProcessCacheInflightTimeoutCount') or 0)} "
                    f"groundRequiredProcessCacheInflightWaitMs={float(dense_metrics.get('groundRequiredProcessCacheInflightWaitMs') or 0.0):.3f} "
                    f"groundRequiredLazyFallbackCount={int(dense_metrics.get('groundRequiredLazyFallbackCount') or 0)} "
                    f"groundRequiredDenseFastPathCount={int(dense_metrics.get('groundRequiredDenseFastPathCount') or 0)} "
                    f"groundRequiredDenseFastPathCoordCount={int(dense_metrics.get('groundRequiredDenseFastPathCoordCount') or 0)} "
                    f"groundRequiredFastPathRejectCount={int(dense_metrics.get('groundRequiredFastPathRejectCount') or 0)} "
                    f"groundRequiredFastPathRejectReason={str(dense_metrics.get('groundRequiredFastPathRejectReason') or '-')} "
                    f"groundRequiredCoordAltitudeFastPathCount={int(dense_metrics.get('groundRequiredCoordAltitudeFastPathCount') or 0)} "
                    f"groundRequiredCoordAltitudeFastPathCoordCount={int(dense_metrics.get('groundRequiredCoordAltitudeFastPathCoordCount') or 0)} "
                    f"groundRequiredCoordAltitudeFastPathRejectCount={int(dense_metrics.get('groundRequiredCoordAltitudeFastPathRejectCount') or 0)} "
                    f"groundRequiredCoordAltitudeFastPathRejectReason={str(dense_metrics.get('groundRequiredCoordAltitudeFastPathRejectReason') or '-')} "
                    f"groundRequiredLineCoordStampCount={int(dense_metrics.get('groundRequiredLineCoordStampCount') or 0)} "
                    f"groundRequiredLineCoordStampChangedCount={int(dense_metrics.get('groundRequiredLineCoordStampChangedCount') or 0)} "
                    f"groundRequiredLineCoordCacheSeedCount={int(dense_metrics.get('groundRequiredLineCoordCacheSeedCount') or 0)} "
                    f"groundRequiredLineCoordCacheSeedSkippedCount={int(dense_metrics.get('groundRequiredLineCoordCacheSeedSkippedCount') or 0)} "
                    f"groundRequiredLineCoordCacheWarmCount={int(dense_metrics.get('groundRequiredLineCoordCacheWarmCount') or 0)} "
                    f"groundRequiredLineCoordCacheWarmSkippedCount={int(dense_metrics.get('groundRequiredLineCoordCacheWarmSkippedCount') or 0)} "
                    f"groundRequiredComputeMs={float(dense_metrics.get('groundRequiredComputeMs') or 0.0):.3f} "
                    f"groundRequiredScanMs={float(dense_metrics.get('groundRequiredScanMs') or 0.0):.3f} "
                    f"groundRequiredHitAssignMs={float(dense_metrics.get('groundRequiredHitAssignMs') or 0.0):.3f} "
                    f"groundRequiredWaiterAssignMs={float(dense_metrics.get('groundRequiredWaiterAssignMs') or 0.0):.3f} "
                    f"groundRequiredOwnerComputeMs={float(dense_metrics.get('groundRequiredOwnerComputeMs') or 0.0):.3f} "
                    f"groundRequiredCacheReadMs={float(dense_metrics.get('groundRequiredCacheReadMs') or 0.0):.3f} "
                    f"groundRequiredCacheLockWaitMs={float(dense_metrics.get('groundRequiredCacheLockWaitMs') or 0.0):.3f} "
                    f"groundRequiredLineCoordStampMs={float(dense_metrics.get('groundRequiredLineCoordStampMs') or 0.0):.3f} "
                    f"groundRequiredLineCoordCacheSeedMs={float(dense_metrics.get('groundRequiredLineCoordCacheSeedMs') or 0.0):.3f} "
                    f"groundRequiredLineCoordCacheWarmMs={float(dense_metrics.get('groundRequiredLineCoordCacheWarmMs') or 0.0):.3f} "
                    f"groundRequiredSampleBuildMs={float(dense_metrics.get('groundRequiredSampleBuildMs') or 0.0):.3f} "
                    f"groundRequiredDemLookupMs={float(dense_metrics.get('groundRequiredDemLookupMs') or 0.0):.3f} "
                    f"groundRequiredResultAssignMs={float(dense_metrics.get('groundRequiredResultAssignMs') or 0.0):.3f} "
                    f"groundRequiredProcessCacheMs={float(dense_metrics.get('groundRequiredProcessCacheMs') or 0.0):.3f} "
                    f"demBulkLookupMs={float(dense_metrics.get('demBulkLookupMs') or 0.0):.3f} "
                    f"demTileResolveMs={float(dense_metrics.get('demTileResolveMs') or 0.0):.3f} "
                    f"demTileCandidateIndexMs={float(dense_metrics.get('demTileCandidateIndexMs') or 0.0):.3f} "
                    f"demTileCandidateAssignMs={float(dense_metrics.get('demTileCandidateAssignMs') or 0.0):.3f} "
                    f"demTileApplyMs={float(dense_metrics.get('demTileApplyMs') or 0.0):.3f} "
                    f"demTileFallbackScanMs={float(dense_metrics.get('demTileFallbackScanMs') or 0.0):.3f} "
                    f"demTileLoadMs={float(dense_metrics.get('demTileLoadMs') or 0.0):.3f} "
                    f"demTileLoadWaitMs={float(dense_metrics.get('demTileLoadWaitMs') or 0.0):.3f} "
                    f"demNativeTransformMs={float(dense_metrics.get('demNativeTransformMs') or 0.0):.3f} "
                    f"demRowColTransformMs={float(dense_metrics.get('demRowColTransformMs') or 0.0):.3f} "
                    f"demPixelReadMs={float(dense_metrics.get('demPixelReadMs') or 0.0):.3f} "
                    f"demCacheReadMs={float(dense_metrics.get('demCacheReadMs') or 0.0):.3f} "
                    f"demCacheWriteMs={float(dense_metrics.get('demCacheWriteMs') or 0.0):.3f} "
                    f"demAltCacheReadMs={float(dense_metrics.get('demAltCacheReadMs') or 0.0):.3f} "
                    f"demAltCacheWriteMs={float(dense_metrics.get('demAltCacheWriteMs') or 0.0):.3f} "
                    f"innerParallelWorkers={int(dense_metrics.get('innerParallelWorkers') or 0)} "
                    f"innerParallelBatches={int(dense_metrics.get('innerParallelBatches') or 0)} "
                    f"innerParallelSuppressedCount={int(dense_metrics.get('innerParallelSuppressedCount') or 0)} "
                    f"generateLineSearchMs={float(dense_metrics.get('generateLineSearchMs') or 0.0):.3f} "
                    f"lineSearchSpeedMs={float(dense_metrics.get('lineSearchSpeedMs') or 0.0):.3f} "
                    f"lineSearchMergeMs={float(dense_metrics.get('lineSearchMergeMs') or 0.0):.3f} "
                    f"areaPostProcessMs={float(dense_metrics.get('areaPostProcessMs') or 0.0):.3f} "
                    f"areaRepositionMs={float(dense_metrics.get('areaRepositionMs') or 0.0):.3f} "
                    f"areaPrePackAltitudeMs={float(dense_metrics.get('areaPrePackAltitudeMs') or 0.0):.3f} "
                    f"areaPackMs={float(dense_metrics.get('areaPackMs') or 0.0):.3f} "
                    f"jsonReadyMs={float(dense_metrics.get('jsonReadyMs') or 0.0):.3f} "
                    f"outputNormalizeMs={float(dense_metrics.get('outputNormalizeMs') or 0.0):.3f} "
                    f"unaccounted0303Ms={unaccounted_0303_ms:.3f} "
                    f"altitudeGuardMs={float(dense_metrics.get('altitudeGuardMs') or 0.0):.3f} "
                    f"altitudeApplyMs={float(dense_metrics.get('altitudeApplyMs') or 0.0):.3f} "
                    f"altitudeFloorMs={float(dense_metrics.get('altitudeFloorMs') or 0.0):.3f} "
                    f"formationLeaderBuildMs={float(dense_metrics.get('formationLeaderBuildMs') or 0.0):.3f} "
                    f"formationFollowerBuildMs={float(dense_metrics.get('formationFollowerBuildMs') or 0.0):.3f} "
                    f"formationGroupingMs={float(dense_metrics.get('formationGroupingMs') or 0.0):.3f} "
                    f"missionPacketBuildMs={float(dense_metrics.get('missionPacketBuildMs') or 0.0):.3f} "
                    f"formationDemMs={float(dense_metrics.get('formationDemMs') or 0.0):.3f} "
                    f"formationPostProcessMs={float(dense_metrics.get('formationPostProcessMs') or 0.0):.3f} "
                    f"formationPostProcessWorkers={int(dense_metrics.get('formationPostProcessWorkers') or 0)} "
                    f"formationPostProcessTasks={int(dense_metrics.get('formationPostProcessTasks') or 0)} "
                    f"etaEcfMs={float(dense_metrics.get('etaEcfMs') or 0.0):.3f} "
                    f"searchSpeedRecalcMs={float(dense_metrics.get('searchSpeedRecalcMs') or 0.0):.3f} "
                    f"searchSpeedRecalcCount={int(dense_metrics.get('searchSpeedRecalcCount') or 0)} "
                    f"searchSpeedRecalcCoordCount={int(dense_metrics.get('searchSpeedRecalcCoordCount') or 0)} "
                    f"searchSpeedRecalcSkippedCount={int(dense_metrics.get('searchSpeedRecalcSkippedCount') or 0)} "
                    f"filmingNormalizeMs={float(dense_metrics.get('filmingNormalizeMs') or 0.0):.3f} "
                    f"filmingTerrainBatchMs={float(dense_metrics.get('filmingTerrainBatchMs') or 0.0):.3f} "
                    f"filmingCandidateScanMs={float(dense_metrics.get('filmingCandidateScanMs') or 0.0):.3f} "
                    f"filmingCandidateWaypointCount={int(dense_metrics.get('filmingCandidateWaypointCount') or 0)} "
                    f"filmingTargetCoordCount={int(dense_metrics.get('filmingTargetCoordCount') or 0)} "
                    f"filmingUniqueCoordCount={int(dense_metrics.get('filmingUniqueCoordCount') or 0)} "
                    f"filmingNormalizeCallCount={int(dense_metrics.get('filmingNormalizeCallCount') or 0)} "
                    f"filmingNormalizeChangedCount={int(dense_metrics.get('filmingNormalizeChangedCount') or 0)} "
                    f"filmingNormalizeSkippedCount={int(dense_metrics.get('filmingNormalizeSkippedCount') or 0)} "
                    f"filmingTerrainBatchFallbackCount={int(dense_metrics.get('filmingTerrainBatchFallbackCount') or 0)} "
                    f"elapsed_ms={elapsed_0303_ms:.3f} "
                    f"fallback={fallback_text} "
                    f"dependencyParallelFallback={str(build_result.get('dependency_parallel_fallback') or '-')} "
                    f"worker_ms={worker_text} groupWorkerMs={group_text}"
                )

            def _get_lah_mission_plan_timings() -> list[dict]:
                getter = getattr(d0304, "get_last_lah_mission_plan_timings", None)
                if not callable(getter):
                    return []
                try:
                    rows = getter(reset=True) or []
                except Exception:
                    return []
                if not isinstance(rows, list):
                    return []
                return [dict(row) for row in rows if isinstance(row, dict)]

            def _emit_mission_plan_timing_metrics(
                emit: Callable[[str], None],
                *,
                variant_no: int,
                option_code: int,
                mode: str,
                build_result_0303: Optional[Dict[str, Any]],
                mission_timings_0304: list[dict],
                elapsed_0303_ms: float,
                elapsed_0304_ms: float,
                flightpath_concurrent: bool,
            ) -> None:
                rows: list[dict] = []
                if isinstance(build_result_0303, dict):
                    rows.extend(
                        dict(row)
                        for row in (build_result_0303.get("mission_timings") or [])
                        if isinstance(row, dict)
                    )
                rows.extend(dict(row) for row in (mission_timings_0304 or []) if isinstance(row, dict))

                def _row_int(row: dict, key: str) -> int:
                    try:
                        return int(row.get(key) or 0)
                    except Exception:
                        return 0

                def _row_float(row: dict, key: str) -> float:
                    try:
                        return float(row.get(key) or 0.0)
                    except Exception:
                        return 0.0

                rows.sort(
                    key=lambda row: (
                        _row_int(row, "aircraftID"),
                        _row_int(row, "individualMissionID"),
                        str(row.get("artifact") or ""),
                    )
                )
                for row in rows:
                    emit(
                        "[REPLAN][MISSION_TIME] individual "
                        f"variant={int(variant_no)} option={int(option_code)} mode={mode} "
                        f"artifact={str(row.get('artifact') or '-')} "
                        f"phase={str(row.get('phase') or '-')} "
                        f"aircraftID={_row_int(row, 'aircraftID')} "
                        f"individualMissionID={_row_int(row, 'individualMissionID')} "
                        f"individualMissionType={_row_int(row, 'individualMissionType')} "
                        f"inputMissionID={_row_int(row, 'inputMissionID')} "
                        f"baseInputMissionID={_row_int(row, 'baseInputMissionID')} "
                        f"pathID={_row_int(row, 'pathID')} "
                        f"missionKind={str(row.get('missionKind') or '-')} "
                        f"waypointCount={_row_int(row, 'waypointCount')} "
                        f"skipped={int(bool(row.get('skipped')))} "
                        f"elapsed_ms={_row_float(row, 'elapsedMs'):.3f}"
                    )

                grouped: dict[int, list[dict]] = {}
                for row in rows:
                    base_id = _row_int(row, "baseInputMissionID")
                    if base_id <= 0:
                        base_id = _row_int(row, "inputMissionID")
                    if base_id <= 0:
                        continue
                    grouped.setdefault(base_id, []).append(row)
                for base_id, base_rows in sorted(grouped.items()):
                    elapsed_sum = sum(_row_float(row, "elapsedMs") for row in base_rows)
                    max_row = max(base_rows, key=lambda row: _row_float(row, "elapsedMs"))
                    artifacts: dict[str, int] = {}
                    aircraft_ids: set[int] = set()
                    path_ids: set[int] = set()
                    for row in base_rows:
                        artifact = str(row.get("artifact") or "-")
                        artifacts[artifact] = int(artifacts.get(artifact, 0)) + 1
                        aid = _row_int(row, "aircraftID")
                        pid = _row_int(row, "pathID")
                        if aid:
                            aircraft_ids.add(aid)
                        if pid:
                            path_ids.add(pid)
                    artifact_text = "|".join(f"{key}:{value}" for key, value in sorted(artifacts.items())) or "-"
                    emit(
                        "[REPLAN][MISSION_TIME] collab_base "
                        f"variant={int(variant_no)} option={int(option_code)} mode={mode} "
                        f"baseInputMissionID={int(base_id)} "
                        f"missionCount={len(base_rows)} "
                        f"aircraftIDs={','.join(str(v) for v in sorted(aircraft_ids)) or '-'} "
                        f"pathIDs={','.join(str(v) for v in sorted(path_ids)) or '-'} "
                        f"artifacts={artifact_text} "
                        f"elapsed_sum_ms={float(elapsed_sum):.3f} "
                        f"max_individual_ms={_row_float(max_row, 'elapsedMs'):.3f} "
                        f"maxIndividualMissionID={_row_int(max_row, 'individualMissionID')}"
                    )

                individual_sum_ms = sum(_row_float(row, "elapsedMs") for row in rows)
                if flightpath_concurrent and elapsed_0303_ms > 0.0 and elapsed_0304_ms > 0.0:
                    flightpath_wall_ms = max(float(elapsed_0303_ms), float(elapsed_0304_ms))
                else:
                    flightpath_wall_ms = float(elapsed_0303_ms) + float(elapsed_0304_ms)
                emit(
                    "[REPLAN][MISSION_TIME] flightpath_total "
                    f"variant={int(variant_no)} option={int(option_code)} mode={mode} "
                    f"missionRows={len(rows)} collabBaseCount={len(grouped)} "
                    f"individual_elapsed_sum_ms={float(individual_sum_ms):.3f} "
                    f"flightpath_0303_ms={float(elapsed_0303_ms):.3f} "
                    f"flightpath_0304_ms={float(elapsed_0304_ms):.3f} "
                    f"flightpath_concurrent={int(bool(flightpath_concurrent))} "
                    f"flightpath_wall_ms={float(flightpath_wall_ms):.3f} "
                    f"shared_or_unassigned_ms={max(0.0, float(flightpath_wall_ms) - float(individual_sum_ms)):.3f} "
                    f"parallel_overlap_or_double_count_ms={max(0.0, float(individual_sum_ms) - float(flightpath_wall_ms)):.3f}"
                )

            def _emit_option_total_timing_metric(
                emit: Callable[[str], None],
                *,
                variant_no: int,
                option_code: int,
                mode: str,
                core_phase_ms: Optional[Dict[str, Any]],
                store_phase_ms: Optional[Dict[str, Any]],
                variant_total_ms: float,
            ) -> None:
                core_map = core_phase_ms if isinstance(core_phase_ms, dict) else {}
                store_map = store_phase_ms if isinstance(store_phase_ms, dict) else {}

                def _sum_values(values: Dict[str, Any]) -> float:
                    total = 0.0
                    for value in values.values():
                        try:
                            total += float(value or 0.0)
                        except Exception:
                            continue
                    return total

                core_sum_ms = _sum_values(core_map)
                store_sum_ms = _sum_values(store_map)
                tracked_sum_ms = core_sum_ms + store_sum_ms
                emit(
                    "[REPLAN][MISSION_TIME] option_total "
                    f"variant={int(variant_no)} option={int(option_code)} mode={mode} "
                    f"core_phase_sum_ms={float(core_sum_ms):.3f} "
                    f"store_phase_sum_ms={float(store_sum_ms):.3f} "
                    f"tracked_phase_sum_ms={float(tracked_sum_ms):.3f} "
                    f"variant_total_ms={float(variant_total_ms):.3f} "
                    f"untracked_or_overlap_ms={float(variant_total_ms) - float(tracked_sum_ms):.3f}"
                )

            generated_plan_ids: list[int] = []
            option_codes_out: list[int] = []
            plan_meta_map: Dict[int, Dict[str, Any]] = {}
            total_imp_files = 0
            total_fp_files = 0
            post_delivery_waypoint_mark: Dict[str, Any] | None = None
            post_delivery_snapshot_carry_forward: Dict[str, Any] | None = None
            def _trust_input_aircraft_for_replan() -> bool:
                # This 0902 path is always a replan flow. Even when MSM marks it as
                # replanLevel=1 (for example RTB/health-unavailable), the planner must
                # still respect the latest VehicleStatus snapshot instead of the stale
                # 0201 availableAircraftList.
                return False

            general_parallel_candidate = (
                plan_count > 1
                and not attack_option_indices
                and not attack_exclusion_option_indices
                and not prior_option_indices
            )
            general_parallel_replan = False
            general_parallel_workers = 1
            general_parallel_gate_reasons: list[str] = []

            def _env_flag(name: str, default: bool = True) -> bool:
                raw = str(os.environ.get(name, "1" if default else "0") or "").strip().lower()
                if raw in {"0", "false", "no", "off"}:
                    return False
                if raw in {"1", "true", "yes", "on"}:
                    return True
                return bool(default)

            def _env_int(name: str, default: int) -> int:
                try:
                    return int(os.environ.get(name, default))
                except Exception:
                    return int(default)

            def _runtime_bool_setting(key: str, default: bool) -> bool:
                raw = base_runtime_values.get(key, default)
                if isinstance(raw, str):
                    text = raw.strip().lower()
                    if text in {"0", "false", "no", "off"}:
                        return False
                    if text in {"1", "true", "yes", "on"}:
                        return True
                return bool(raw)

            def _runtime_int_setting(key: str, default: int) -> int:
                try:
                    return int(float(base_runtime_values.get(key, default)))
                except Exception:
                    return int(default)

            def _runtime_env_flag(setting_key: str, env_name: str, default: bool) -> bool:
                return _env_flag(env_name, _runtime_bool_setting(setting_key, default))

            def _runtime_env_int(setting_key: str, env_name: str, default: int) -> int:
                return _env_int(env_name, _runtime_int_setting(setting_key, default))

            def _general_parallel_runtime_config() -> Dict[str, Any]:
                return {
                    "replan_variant_parallel_enabled": _runtime_env_flag(
                        "replan_variant_parallel_enabled",
                        "REPLAN_VARIANT_PARALLEL",
                        True,
                    ),
                    "replan_current_remaining_variant_parallel_enabled": _runtime_env_flag(
                        "replan_current_remaining_variant_parallel_enabled",
                        "REPLAN_CURRENT_REMAINING_VARIANT_PARALLEL",
                        True,
                    ),
                    "replan_variant_workers": max(
                        1,
                        _runtime_env_int(
                            "replan_variant_workers",
                            "REPLAN_VARIANT_WORKERS",
                            3,
                        ),
                    ),
                    "replan_variant_waypoint_block_size": max(
                        0,
                        _runtime_env_int(
                            "replan_variant_waypoint_block_size",
                            "REPLAN_VARIANT_WAYPOINT_BLOCK_SIZE",
                            5000,
                        ),
                    ),
                    "replan_recon_worker_cap": max(
                        0,
                        _runtime_env_int(
                            "replan_recon_worker_cap",
                            "REPLAN_RECON_WORKER_CAP",
                            0,
                        ),
                    ),
                    "replan_current_remaining_precompute_workers": max(
                        1,
                        _runtime_env_int(
                            "replan_current_remaining_precompute_workers",
                            "REPLAN_CURRENT_REMAINING_PRECOMPUTE_WORKERS",
                            2,
                        ),
                    ),
                    "replan_store_prepare_workers": max(
                        1,
                        _runtime_env_int(
                            "replan_store_prepare_workers",
                            "REPLAN_STORE_PREPARE_WORKERS",
                            2,
                        ),
                    ),
                    "replan_store_prepare_out_of_order": _runtime_env_flag(
                        "replan_store_prepare_out_of_order",
                        "REPLAN_STORE_PREPARE_OUT_OF_ORDER",
                        True,
                    ),
                    "replan_store_commit_workers": max(
                        1,
                        _runtime_env_int(
                            "replan_store_commit_workers",
                            "REPLAN_STORE_COMMIT_WORKERS",
                            2,
                        ),
                    ),
                    "replan_store_json_write_workers": max(
                        1,
                        _runtime_env_int(
                            "replan_store_json_write_workers",
                            "REPLAN_STORE_JSON_WRITE_WORKERS",
                            2,
                        ),
                    ),
                    "replan_store_path_id_cross_variant_bulk": _runtime_env_flag(
                        "replan_store_path_id_cross_variant_bulk",
                        "REPLAN_STORE_PATH_ID_CROSS_VARIANT_BULK",
                        True,
                    ),
                    "replan_store_snapshot_post_delivery": _runtime_env_flag(
                        "replan_store_snapshot_post_delivery",
                        "REPLAN_STORE_SNAPSHOT_POST_DELIVERY",
                        True,
                    ),
                }

            def _evaluate_general_parallel_safety() -> tuple[bool, int, list[str]]:
                reasons: list[str] = []
                runtime_parallel_config = _general_parallel_runtime_config()
                if not general_parallel_candidate:
                    reasons.append("not_general_multi_variant")
                if not bool(runtime_parallel_config["replan_variant_parallel_enabled"]):
                    reasons.append("runtime_replan_variant_parallel_disabled")
                if current_remaining_hybrid_request is not None and not bool(
                    runtime_parallel_config["replan_current_remaining_variant_parallel_enabled"]
                ):
                    reasons.append("current_remaining_hybrid_active")
                reasons.extend(
                    parallel_snapshot_safety_reasons(
                        snapshot_apply_result=snapshot_apply_result,
                        collapse_apply_result=collapse_apply_result,
                        filtered_payload_materialized=base_filtered_payload_materialized,
                    )
                )
                if (
                    not callable(reserve_imp_ids)
                    or not callable(reserve_path_ids)
                    or not callable(reserve_mission_plan_ids)
                    or not callable(reserve_waypoint_block)
                ):
                    reasons.append("id_allocator_reserve_unavailable")
                if not Path(cmpk_path).exists() or not Path(mrpk_path).exists():
                    reasons.append("source_artifact_missing")
                requested_positive_ids = [
                    int(value) for value in plan_ids
                    if value is not None and int(value) > 0
                ]
                if (
                    len(requested_positive_ids) != int(plan_count)
                    or len(set(requested_positive_ids)) != int(plan_count)
                ):
                    reasons.append("requested_plan_ids_not_positive_distinct")

                requested_workers = max(1, int(runtime_parallel_config["replan_variant_workers"]))
                workers = min(plan_count, requested_workers)
                if workers < 2:
                    reasons.append("worker_count_lt_2")
                waypoint_block_size = max(0, int(runtime_parallel_config["replan_variant_waypoint_block_size"]))
                if waypoint_block_size < 1000:
                    reasons.append("waypoint_block_too_small")

                if reasons:
                    return False, 1, reasons
                return True, workers, []

            general_parallel_replan, general_parallel_workers, general_parallel_gate_reasons = _evaluate_general_parallel_safety()
            if general_parallel_candidate:
                self._record_replan_timing_event(
                    "parallel_safety_gate",
                    extra={
                        "enabled": bool(general_parallel_replan),
                        "workers": int(general_parallel_workers),
                        "waypoint_block_size": max(
                            0,
                            int(_general_parallel_runtime_config()["replan_variant_waypoint_block_size"]),
                        ),
                        "reasons": "|".join(general_parallel_gate_reasons) if general_parallel_gate_reasons else "-",
                        "runtime_settings": "general_variant_parallel",
                    },
                )
                if not general_parallel_replan:
                    self.log_sig.emit(
                        "[INFO] 일반 재계획 옵션 병렬 생성 비활성화(safety gate): "
                        + ", ".join(general_parallel_gate_reasons)
                    )
                elif current_remaining_hybrid_request is not None:
                    self.log_sig.emit(
                        "[INFO] 현재임무 재수행 옵션 병렬 생성 활성화: variant별 hybrid 요청 격리 적용"
                    )

            class _VariantCoreError(RuntimeError):
                def __init__(self, code: str, message: str, *, variant_no: int, detail: Optional[Dict[str, Any]] = None):
                    super().__init__(message)
                    self.code = str(code or "variant_core_error")
                    self.message = str(message or code or "variant_core_error")
                    self.variant_no = int(variant_no)
                    self.detail = dict(detail or {})

            def _build_current_remaining_hybrid_locked(
                request: CurrentRemainingHybridRequest,
                *,
                variant_no: int,
                log_emit: Callable[[str], None],
                timing_sink: Optional[Dict[str, float]] = None,
            ):
                global_lock_enabled = _current_remaining_hybrid_global_lock_enabled()
                lock_wait_started = time.perf_counter()
                lock_acquired = False
                if global_lock_enabled:
                    _CURRENT_REMAINING_HYBRID_BUILD_LOCK.acquire()
                    lock_acquired = True
                lock_wait_ms = (time.perf_counter() - lock_wait_started) * 1000.0
                build_started = time.perf_counter()
                hybrid_result = None
                try:
                    hybrid_result = build_current_remaining_hybrid(
                        request,
                        log=lambda msg, n=variant_no: log_emit(f"[variant {n}] {msg}"),
                    )
                    return hybrid_result
                finally:
                    build_ms = (time.perf_counter() - build_started) * 1000.0
                    if lock_acquired:
                        _CURRENT_REMAINING_HYBRID_BUILD_LOCK.release()
                    if isinstance(timing_sink, dict):
                        timing_sink["current_hybrid_wait_ms"] = float(lock_wait_ms)
                        timing_sink["current_hybrid_build_ms"] = float(build_ms)
                        timing_sink["current_hybrid_global_lock"] = float(int(global_lock_enabled))
                    log_emit(
                        f"[TIME] current_remaining_hybrid_build "
                        f"(variant={variant_no}): wait={lock_wait_ms:.1f} ms, build={build_ms:.1f} ms, "
                        f"globalLock={int(global_lock_enabled)}"
                    )
                    log_emit(
                        "[REPLAN][METRIC] current_remaining_hybrid_lock_policy "
                        f"variant={int(variant_no)} "
                        f"global_lock={int(global_lock_enabled)} "
                        f"lock_wait_ms={float(lock_wait_ms):.3f} "
                        f"build_total_ms={float(build_ms):.3f}"
                    )
                    prepare_timing = getattr(hybrid_result, "prepare_timing_ms", None)
                    if isinstance(prepare_timing, dict) and prepare_timing:
                        def _timing_value(name: str) -> float:
                            try:
                                return float(prepare_timing.get(name) or 0.0)
                            except Exception:
                                return 0.0

                        log_emit(
                            "[REPLAN][METRIC] current_remaining_hybrid_prepare "
                            f"variant={int(variant_no)} "
                            f"source_load_ms={_timing_value('source_load'):.3f} "
                            f"target_input_resolve_ms={_timing_value('target_input_resolve'):.3f} "
                            f"entry_coordinate_resolve_ms={_timing_value('entry_coordinate_resolve'):.3f} "
                            f"area_planner_run_ms={_timing_value('area_planner_run'):.3f} "
                            f"id_reservation_ms={_timing_value('id_reservation'):.3f} "
                            f"replacement_mission_build_ms={_timing_value('replacement_mission_build'):.3f} "
                            f"flight_path_build_ms={_timing_value('flight_path_build'):.3f} "
                            f"lock_wait_ms={float(lock_wait_ms):.3f} "
                            f"global_lock={int(global_lock_enabled)} "
                            f"build_total_ms={float(build_ms):.3f}"
                        )

            def _apply_current_remaining_hybrid_to_variant(
                *,
                variant_no: int,
                missions: list[dict],
                flight_plans_0303: list[dict],
                flight_plans_0304: list[dict],
                request: CurrentRemainingHybridRequest | None,
                hybrid_result: Any = None,
                log_emit: Callable[[str], None] | None = None,
            ) -> tuple[list[dict], list[dict], list[dict], Set[int]]:
                emit = log_emit or self.log_sig.emit
                if request is None:
                    return missions, flight_plans_0303, flight_plans_0304, set()
                try:
                    hybrid = hybrid_result
                    if hybrid is None:
                        hybrid = _build_current_remaining_hybrid_locked(
                            request,
                            variant_no=variant_no,
                            log_emit=emit,
                        )
                except Exception as exc:
                    emit(
                        f"[WARN] [variant {variant_no}] current remaining collaborative hybrid failed: {exc}"
                    )
                    return missions, flight_plans_0303, flight_plans_0304, set()
                if hybrid is None:
                    emit(
                        f"[WARN] [variant {variant_no}] current remaining collaborative hybrid unavailable; keep generic output"
                    )
                    return missions, flight_plans_0303, flight_plans_0304, set()
                merged = merge_current_remaining_hybrid(
                    missions=missions,
                    flight_plans_0303=flight_plans_0303,
                    flight_plans_0304=flight_plans_0304,
                    hybrid=hybrid,
                )
                path_validation = dict(merged.get("pathValidation") or {})
                if path_validation and not bool(path_validation.get("valid", True)):
                    overlap_summary = ",".join(
                        str(pid) for pid in (path_validation.get("overlapPathIDs") or [])
                    )
                    raise RuntimeError(
                        "current remaining hybrid path validation failed "
                        f"(variant={variant_no}, overlapPathIDs={overlap_summary or '-'})"
                    )
                temporary_path_id_remap = dict(merged.get("temporaryPathIdRemap") or {})
                if temporary_path_id_remap:
                    remap_summary = ",".join(
                        f"{old_pid}->{new_pid}"
                        for old_pid, new_pid in sorted(temporary_path_id_remap.items())
                    )
                    emit(
                        f"[variant {variant_no}] current remaining hybrid temporary pathID remapped: "
                        f"{remap_summary}"
                    )
                emit(
                    f"[variant {variant_no}] current remaining hybrid path validation: "
                    f"{path_validation or validate_current_remaining_hybrid_paths(generic_path_ids=[], hybrid_path_ids=[])}"
                )
                emit(
                    f"[variant {variant_no}] current remaining collaborative hybrid applied: "
                    f"inputMissionID={hybrid.current_input_id}, "
                    f"aircraft={sorted(merged.get('replace_aircraft_ids') or [])}, "
                    f"removedPaths={len(merged.get('removed_path_ids') or [])}, "
                    f"workflow={merged.get('planner_workflow') or '-'}"
                )
                return (
                    list(merged.get("missions") or []),
                    list(merged.get("flight_plans_0303") or []),
                    list(merged.get("flight_plans_0304") or []),
                    set(int(pid) for pid in (merged.get("generated_path_ids") or set()) if pid is not None),
                )

            def _mission_related_input_id(mission: Dict[str, Any]) -> Optional[int]:
                if not isinstance(mission, dict):
                    return None
                related = mission.get("relatedMission") if isinstance(mission.get("relatedMission"), dict) else {}
                for value in (related.get("inputMissionID"), mission.get("inputMissionID")):
                    parsed = _safe_int_value(value)
                    if parsed is not None and parsed > 0:
                        return int(parsed)
                return None

            def _is_reexecute_line_hold_request(request: CurrentRemainingHybridRequest | None) -> bool:
                if request is None:
                    return False
                if str(getattr(request, "planner_mode", "") or "") != "reexecute_first_mission":
                    return False
                return True

            def _lah_mission_has_line_route(mission: Dict[str, Any]) -> bool:
                if not isinstance(mission, dict):
                    return False
                info = mission.get("individualMissionInfo") if isinstance(mission.get("individualMissionInfo"), dict) else {}
                line_list = info.get("lineList") if isinstance(info.get("lineList"), list) else []
                for line in line_list:
                    if not isinstance(line, dict):
                        continue
                    if len(_normalize_coord_list(line.get("coordinateList"), min_len=2)) >= 2:
                        return True
                try:
                    mission_type = int(info.get("individualMissionType", 0) or 0)
                except Exception:
                    mission_type = 0
                if mission_type in (6, 7):
                    return len(_normalize_coord_list(info.get("coordinateList"), min_len=2)) >= 2
                return False

            def _mark_lah_reexecute_line_hold_for_0304(
                mission: Dict[str, Any],
                *,
                hold_seconds: int = 300,
            ) -> Dict[str, Any]:
                marked = copy.deepcopy(mission)
                marked["_lahHoldAtLineEnd"] = True
                marked["_lahLineHoldSeconds"] = int(hold_seconds)
                return marked

            def _manned_missions_for_reexecute_line_hold_0304(
                manned_missions: List[Dict[str, Any]],
                request: CurrentRemainingHybridRequest | None,
            ) -> List[Dict[str, Any]]:
                if not _is_reexecute_line_hold_request(request):
                    return list(manned_missions or [])
                current_input_id = _safe_int_value(getattr(request, "current_input_id", None))
                if current_input_id is None or current_input_id <= 0:
                    return list(manned_missions or [])
                out: List[Dict[str, Any]] = []
                marked_count = 0
                for mission in manned_missions or []:
                    if (
                        isinstance(mission, dict)
                        and _mission_related_input_id(mission) == int(current_input_id)
                        and _lah_mission_has_line_route(mission)
                    ):
                        out.append(_mark_lah_reexecute_line_hold_for_0304(mission, hold_seconds=300))
                        marked_count += 1
                    else:
                        out.append(mission)
                if marked_count:
                    self.log_sig.emit(
                        "[REEXEC-FIRST] LAH current LINE hold armed for 0304: "
                        f"inputMissionID={int(current_input_id)}, missions={int(marked_count)}, hover=300s"
                    )
                return out

            def _lah_line_hold_info_from_input_mission(
                input_mission: Dict[str, Any],
            ) -> Dict[str, Any] | None:
                if _mission_geometry_bucket(input_mission) != "line":
                    return None
                detail = input_mission.get("missionDetail") if isinstance(input_mission.get("missionDetail"), dict) else {}
                if not isinstance(detail, dict):
                    return None
                info: Dict[str, Any] = {
                    "individualMissionType": 6,
                    "patternType": 8,
                    "autoZoomIn": False,
                    "targetID": None,
                }
                line_list = detail.get("lineList")
                if isinstance(line_list, list) and line_list:
                    info["lineList"] = copy.deepcopy(line_list)
                    return info
                coord_list = detail.get("coordinateList")
                coords = _normalize_coord_list(coord_list, min_len=2)
                if coords:
                    info["coordinateList"] = copy.deepcopy(coords)
                    return info
                return None

            def _source_lah_current_mission_templates(
                *,
                source_aircraft_rows: List[Dict[str, Any]],
                template_input_id: int,
                log_emit: Callable[[str], None],
            ) -> Dict[int, Dict[str, Any]]:
                templates: Dict[int, Dict[str, Any]] = {}
                for source_aircraft in source_aircraft_rows or []:
                    if not isinstance(source_aircraft, dict):
                        continue
                    aircraft_id = _safe_int_value(source_aircraft.get("aircraftID"))
                    if aircraft_id is None or not (1 <= int(aircraft_id) <= 3):
                        continue
                    package_id = _safe_int_value(
                        source_aircraft.get("individualMissionPackageID")
                        or source_aircraft.get("individualMissionPlanPackageID")
                        or source_aircraft.get("individualMissionPackageId")
                    )
                    if package_id is None or package_id <= 0:
                        continue
                    try:
                        source_imp = source_cache.load_individual_mission_plan(int(package_id))
                    except Exception as exc:
                        log_emit(
                            f"[WARN] [REEXEC-FIRST] source LAH IMP load failed "
                            f"(aircraftID={aircraft_id}, packageID={package_id}): {exc}"
                        )
                        continue
                    for source_mission in source_imp.get("individualMissionList") or []:
                        if not isinstance(source_mission, dict):
                            continue
                        if _mission_related_input_id(source_mission) != int(template_input_id):
                            continue
                        mission_copy = copy.deepcopy(source_mission)
                        mission_copy["aircraftID"] = int(aircraft_id)
                        if has_reusable_lah_role_geometry(mission_copy):
                            templates[int(aircraft_id)] = mission_copy
                        break
                return templates

            def _append_reexecute_lah_line_hold_current_artifacts(
                *,
                request: CurrentRemainingHybridRequest,
                source_aircraft_ids: Set[int],
                source_lah_templates: Dict[int, Dict[str, Any]] | None = None,
                missions: List[Dict[str, Any]],
                flight_plans_0304: List[Dict[str, Any]],
                log_emit: Callable[[str], None],
            ) -> Dict[str, Any]:
                summary: Dict[str, Any] = {
                    "applied": False,
                    "aircraftIDs": [],
                    "roleTemplateAircraftIDs": [],
                    "fallbackLineAircraftIDs": [],
                    "missionCount": 0,
                    "flightPathCount": 0,
                    "pathIDs": [],
                    "mode": "",
                    "reason": "",
                }
                if not _is_reexecute_line_hold_request(request):
                    summary["reason"] = "not_reexecute_line"
                    return summary
                current_input_id = _safe_int_value(getattr(request, "current_input_id", None))
                if current_input_id is None or current_input_id <= 0:
                    summary["reason"] = "current_input_id_unavailable"
                    return summary
                current_input_mission = getattr(request, "current_input_mission", None)
                if not isinstance(current_input_mission, dict):
                    summary["reason"] = "current_input_mission_unavailable"
                    return summary
                info_template = _lah_line_hold_info_from_input_mission(current_input_mission)
                role_templates = {
                    int(aid): copy.deepcopy(template)
                    for aid, template in dict(source_lah_templates or {}).items()
                    if isinstance(template, dict) and has_reusable_lah_role_geometry(template)
                }
                if not isinstance(info_template, dict) and not role_templates:
                    summary["reason"] = "lah_role_geometry_unavailable"
                    return summary
                lah_ids = sorted(
                    int(aid)
                    for aid in (source_aircraft_ids or set())
                    if 1 <= int(aid) <= 3
                )
                if not lah_ids:
                    summary["reason"] = "source_plan_lah_unavailable"
                    return summary

                hold_missions_for_imp: List[Dict[str, Any]] = []
                hold_missions_for_fp: List[Dict[str, Any]] = []
                role_template_aircraft_ids: List[int] = []
                fallback_line_aircraft_ids: List[int] = []
                for aid in lah_ids:
                    try:
                        path_id = int(reserve_path_ids(int(aid), 1)[0])
                    except Exception as exc:
                        log_emit(f"[WARN] [REEXEC-FIRST] LAH hold pathID reservation failed (aircraftID={aid}): {exc}")
                        continue
                    if int(aid) in role_templates:
                        mission_entry = rebind_reexecute_lah_role_mission(
                            role_templates[int(aid)],
                            aircraft_id=int(aid),
                            current_input_id=int(current_input_id),
                            path_id=int(path_id),
                        )
                        role_template_aircraft_ids.append(int(aid))
                    elif isinstance(info_template, dict):
                        mission_entry = {
                            "aircraftID": int(aid),
                            "individualMissionID": 0,
                            "isDone": False,
                            "relatedMission": {
                                "relatedMissionType": 1,
                                "inputMissionID": int(current_input_id),
                                "priorMissionID": 0,
                            },
                            "individualMissionInfo": copy.deepcopy(info_template),
                            "pathID": int(path_id),
                        }
                        fallback_line_aircraft_ids.append(int(aid))
                    else:
                        continue
                    hold_missions_for_imp.append(copy.deepcopy(mission_entry))
                    if _lah_mission_has_line_route(mission_entry):
                        hold_missions_for_fp.append(
                            _mark_lah_reexecute_line_hold_for_0304(mission_entry, hold_seconds=300)
                        )
                    else:
                        # Point/area role missions must retain their original geometry. Marking
                        # them as a LINE hold would move the LAH onto the UAV scan centerline.
                        hold_missions_for_fp.append(copy.deepcopy(mission_entry))

                if not hold_missions_for_fp:
                    summary["reason"] = "no_hold_missions_built"
                    return summary

                wp_alloc = None
                if all(_lah_mission_has_line_route(mission) for mission in hold_missions_for_fp):
                    try:
                        wp_count = max(1, len(hold_missions_for_fp))
                        wp_start = int(reserve_waypoint_block(int(wp_count)))
                        wp_alloc = d0304._WPAllocator(start=wp_start, end=wp_start + wp_count - 1)
                    except Exception as exc:
                        log_emit(f"[WARN] [REEXEC-FIRST] LAH hold waypoint block reservation failed: {exc}")

                hold_fps = d0304.build_lah_flight_plans_fixed(
                    hold_missions_for_fp,
                    cruise_speed=40.0,
                    manned_plan_mode="normal",
                    lah_path_mode="linear",
                    wp_alloc=wp_alloc,
                )
                if not hold_fps:
                    summary["reason"] = "d0304_hold_flightpath_empty"
                    return summary

                missions.extend(hold_missions_for_imp)
                flight_plans_0304.extend(hold_fps)
                summary.update(
                    {
                        "applied": True,
                        "aircraftIDs": [int(mission.get("aircraftID")) for mission in hold_missions_for_imp],
                        "roleTemplateAircraftIDs": list(role_template_aircraft_ids),
                        "fallbackLineAircraftIDs": list(fallback_line_aircraft_ids),
                        "missionCount": len(hold_missions_for_imp),
                        "flightPathCount": len(hold_fps),
                        "pathIDs": [
                            int(fp.get("pathID"))
                            for fp in hold_fps
                            if _safe_int_value(fp.get("pathID")) is not None
                        ],
                        "mode": "source_role" if role_template_aircraft_ids else "line_fallback",
                        "reason": "ok",
                    }
                )
                log_emit(
                    "[REEXEC-FIRST] LAH current role position generated: "
                    f"inputMissionID={int(current_input_id)}, "
                    f"aircraft={summary['aircraftIDs']}, "
                    f"roleTemplates={summary['roleTemplateAircraftIDs']}, "
                    f"fallbackLine={summary['fallbackLineAircraftIDs']}, "
                    f"pathIDs={summary['pathIDs']}"
                )
                return summary

            def _current_cmpk_pending_input_ids() -> Set[int]:
                result: Set[int] = set()
                mission_rows = cmpk_data.get("inputMissionList") if isinstance(cmpk_data, dict) else None
                if not isinstance(mission_rows, list):
                    return result
                for mission in mission_rows:
                    if not isinstance(mission, dict) or bool(mission.get("isDone")):
                        continue
                    input_id = _safe_int_value(mission.get("inputMissionID"))
                    if input_id is not None and input_id > 0:
                        result.add(int(input_id))
                return result

            def _waypoint_lists_in_flight_path(flight_path: Dict[str, Any]) -> list[list[dict]]:
                lists: list[list[dict]] = []
                if not isinstance(flight_path, dict):
                    return lists
                for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
                    rows = flight_path.get(key)
                    if isinstance(rows, list):
                        lists.append([row for row in rows if isinstance(row, dict)])
                return lists

            def _remap_flight_path_waypoints_from_start(
                flight_path: Dict[str, Any],
                *,
                start_id: int,
            ) -> int:
                waypoint_lists = _waypoint_lists_in_flight_path(flight_path)
                waypoint_count = sum(len(rows) for rows in waypoint_lists)
                if waypoint_count <= 0:
                    return 0
                next_id = int(start_id)
                id_map: Dict[int, int] = {}
                for rows in waypoint_lists:
                    for waypoint in rows:
                        old_id = _safe_int_value(waypoint.get("waypointID"))
                        new_id = int(next_id)
                        next_id += 1
                        if old_id is not None and old_id > 0:
                            id_map[int(old_id)] = int(new_id)
                        waypoint["waypointID"] = int(new_id)
                for rows in waypoint_lists:
                    for waypoint in rows:
                        old_next = _safe_int_value(waypoint.get("nextWaypointID"))
                        if old_next is not None and int(old_next) in id_map:
                            waypoint["nextWaypointID"] = int(id_map[int(old_next)])
                return int(waypoint_count)

            def _reserve_and_remap_flight_path_waypoints(flight_path: Dict[str, Any]) -> int:
                waypoint_count = sum(len(rows) for rows in _waypoint_lists_in_flight_path(flight_path))
                if waypoint_count <= 0:
                    return 0
                start_id = int(reserve_waypoint_block(int(waypoint_count)))
                return _remap_flight_path_waypoints_from_start(flight_path, start_id=int(start_id))

            def _clone_carry_waypoint_template(waypoint: Dict[str, Any]) -> Dict[str, Any]:
                cloned = dict(waypoint)
                for key, value in list(cloned.items()):
                    if not isinstance(value, dict):
                        continue
                    nested = dict(value)
                    for nested_key, nested_value in list(nested.items()):
                        if isinstance(nested_value, dict):
                            nested[nested_key] = dict(nested_value)
                    cloned[key] = nested
                return cloned

            def _clone_carry_flight_path_template(
                flight_path: Dict[str, Any],
                *,
                shallow: bool,
            ) -> Dict[str, Any]:
                if not shallow:
                    return copy.deepcopy(flight_path)
                cloned = dict(flight_path)
                for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
                    rows = flight_path.get(key)
                    if isinstance(rows, list):
                        cloned[key] = [
                            _clone_carry_waypoint_template(row) if isinstance(row, dict) else row
                            for row in rows
                        ]
                return cloned

            input_refresh_carry_template_cache: Dict[int, Dict[str, Any]] = {}
            input_refresh_carry_template_lock = threading.RLock()

            def _input_refresh_flightpath_carry_enabled() -> bool:
                # A carried single-mission path can match in isolation while
                # changing the neighbouring missions rebuilt from a shortened
                # list.  Keep this optimisation disabled until the builders can
                # reuse paths without removing missions from their full order.
                return False

            def _input_refresh_carry_signature(mission: Dict[str, Any]) -> str:
                volatile_keys = {
                    "individualMissionID",
                    "individualMissionId",
                    "individualMissionPlanPackageID",
                    "individualMissionPackageID",
                    "individualMissionPackageId",
                    "missionPlanID",
                    "pathID",
                    "pathId",
                    "individualMissionPathID",
                    "missionPathID",
                    "aircraftID",
                    "timestamp",
                    "createdAt",
                    "updatedAt",
                    "Source",
                    # Enhanced split/scheduling derives these transient hints from
                    # neighbouring missions. They are intentionally not exported in
                    # 0302, so a stored source IMP cannot contain them. The stable
                    # current+predecessor mission signatures below are the contract.
                    "bearingIn_deg",
                    "bearingOut_deg",
                    "bearing_deg",
                    "boundaryAxisBearing_deg",
                    "nextPoint",
                    "prevPoint",
                    "phaseMoveBearing_deg",
                    "phaseSplitBearing_deg",
                    "splitBearing_deg",
                }

                def _normalize(value: Any) -> Any:
                    if isinstance(value, dict):
                        normalized: Dict[str, Any] = {}
                        for key, val in sorted(value.items(), key=lambda item: str(item[0])):
                            key_text = str(key)
                            if key_text in volatile_keys or key_text.startswith("_"):
                                continue
                            # 0302 canonicalization omits an empty coordinateList;
                            # treat omitted and [] as the same non-geometry value.
                            if key_text == "coordinateList" and (val is None or val == []):
                                continue
                            normalized[key_text] = _normalize(val)
                        return normalized
                    if isinstance(value, list):
                        return [_normalize(item) for item in value]
                    return value

                try:
                    return json.dumps(
                        _normalize(mission),
                        sort_keys=True,
                        ensure_ascii=False,
                        separators=(",", ":"),
                    )
                except Exception:
                    return repr(_normalize(mission))

            def _input_refresh_variant_mission_signature(mission: Dict[str, Any]) -> str:
                volatile_keys = {
                    "individualMissionID",
                    "individualMissionId",
                    "individualMissionPlanPackageID",
                    "individualMissionPackageID",
                    "individualMissionPackageId",
                    "missionPlanID",
                    "pathID",
                    "pathId",
                    "individualMissionPathID",
                    "missionPathID",
                    "timestamp",
                    "createdAt",
                    "updatedAt",
                    "Source",
                }

                def _normalize(value: Any) -> Any:
                    if isinstance(value, dict):
                        return {
                            str(key): _normalize(val)
                            for key, val in sorted(value.items(), key=lambda item: str(item[0]))
                            if str(key) not in volatile_keys and not str(key).startswith("_")
                        }
                    if isinstance(value, list):
                        return [_normalize(item) for item in value]
                    return value

                try:
                    return json.dumps(
                        _normalize(mission),
                        sort_keys=True,
                        ensure_ascii=False,
                        separators=(",", ":"),
                    )
                except Exception:
                    return repr(_normalize(mission))

            def _input_refresh_flightpath_channel(aircraft_id: int) -> str | None:
                aid = int(aircraft_id)
                if aid in (1, 2, 3):
                    return "0304"
                if aid in (4, 5, 6):
                    return "0303"
                return None

            def _input_refresh_source_carry_template_cache_key(source_plan_id: int) -> int:
                return int(source_plan_id)

            def _input_refresh_source_plan_for_option(
                *,
                option_code: int,
                fallback_source_plan_id: Any,
            ) -> int | None:
                target_code = normalize_option_code(option_code, fallback=None)

                def _existing_plan_id(value: Any) -> int | None:
                    plan_id = _safe_int_value(value)
                    if plan_id is None or plan_id <= 0:
                        return None
                    try:
                        if (dir_mp / f"{int(plan_id)}.json").exists():
                            return int(plan_id)
                    except Exception:
                        return int(plan_id)
                    return None

                def _meta_option_code(row: Any) -> int | None:
                    if not isinstance(row, dict):
                        return None
                    for key in ("optionCode", "option_code", "optionName", "option_name", "label"):
                        code = normalize_option_code(row.get(key), fallback=None)
                        if code is not None:
                            return int(code)
                    return None

                # The source recorded in the snapshot/replan request is the
                # authoritative predecessor. Requested output plan IDs may already
                # exist during a retry/replay and must never become their own carry
                # source merely because their option code matches.
                fallback = _existing_plan_id(fallback_source_plan_id)
                if fallback is not None:
                    return int(fallback)

                containers = [
                    getattr(self, "_active_plan_context", None),
                    ctx,
                    staged,
                ]
                for container in containers:
                    if not isinstance(container, dict):
                        continue
                    meta = container.get("_option_meta")
                    if isinstance(meta, dict):
                        for raw_plan_id, meta_row in meta.items():
                            plan_id = None
                            if isinstance(meta_row, dict):
                                plan_id = _existing_plan_id(
                                    meta_row.get("missionPlanID")
                                    or meta_row.get("planID")
                                    or meta_row.get("requestedMissionPlanID")
                                    or raw_plan_id
                                )
                            else:
                                plan_id = _existing_plan_id(raw_plan_id)
                            if plan_id is None:
                                continue
                            if target_code is not None and _meta_option_code(meta_row) == int(target_code):
                                return int(plan_id)
                    plan_ids_raw = list(container.get("plan_ids") or container.get("planIDs") or [])
                    option_names_raw = list(container.get("option_names") or container.get("optionNames") or [])
                    for idx, raw_plan_id in enumerate(plan_ids_raw):
                        plan_id = _existing_plan_id(raw_plan_id)
                        if plan_id is None:
                            continue
                        raw_option = option_names_raw[idx] if idx < len(option_names_raw) else None
                        if target_code is not None and normalize_option_code(raw_option, fallback=None) == int(target_code):
                            return int(plan_id)
                return None

            def _build_input_refresh_source_carry_template(source_plan_id: int) -> Dict[str, Any]:
                phase_ms: Dict[str, float] = {}
                source_imp_scan_count = 0
                source_mission_scan_count = 0
                source_missing_imp_count = 0
                source_missing_path_count = 0
                warnings: list[Dict[str, Any]] = []
                source_rows_by_aircraft: Dict[int, list[Dict[str, Any]]] = {}

                phase_t0 = time.perf_counter()
                source_plan = source_cache.load_mission_plan(int(source_plan_id), copy_result=False)
                phase_ms["source_plan_load_ms"] = (time.perf_counter() - phase_t0) * 1000.0
                source_aircraft_rows = source_plan.get("aircraftList") if isinstance(source_plan, dict) else None
                if not isinstance(source_aircraft_rows, list) or not source_aircraft_rows:
                    raise RuntimeError(f"inputRefresh carry source MissionPlan invalid: {source_plan_id}")

                phase_t0 = time.perf_counter()
                for source_aircraft in source_aircraft_rows:
                    if not isinstance(source_aircraft, dict):
                        continue
                    aircraft_id = _safe_int_value(source_aircraft.get("aircraftID"))
                    if aircraft_id is None or not (1 <= int(aircraft_id) <= 6):
                        continue
                    package_id = _safe_int_value(
                        source_aircraft.get("individualMissionPackageID")
                        or source_aircraft.get("individualMissionPlanPackageID")
                        or source_aircraft.get("individualMissionPackageId")
                    )
                    if package_id is None or package_id <= 0:
                        continue
                    try:
                        source_imp = source_cache.load_individual_mission_plan(
                            int(package_id),
                            copy_result=False,
                        )
                    except Exception as exc:
                        source_missing_imp_count += 1
                        warnings.append(
                            {
                                "kind": "source_imp_load_failed",
                                "aircraftID": int(aircraft_id),
                                "packageID": int(package_id),
                                "error": str(exc),
                            }
                        )
                        continue
                    source_imp_scan_count += 1
                    channel = _input_refresh_flightpath_channel(int(aircraft_id))
                    if channel is None:
                        continue
                    rows = source_rows_by_aircraft.setdefault(int(aircraft_id), [])
                    ordinal = 0
                    previous_input_id: int | None = None
                    previous_signature: str | None = None
                    for source_mission in source_imp.get("individualMissionList") or []:
                        if not isinstance(source_mission, dict):
                            continue
                        source_mission_scan_count += 1
                        input_id = _mission_related_input_id(source_mission)
                        path_id = _safe_int_value(source_mission.get("pathID"))
                        mission_signature = _input_refresh_carry_signature(source_mission)
                        if input_id is None or input_id <= 0:
                            ordinal += 1
                            continue
                        if bool(source_mission.get("isDone")):
                            previous_input_id = int(input_id)
                            previous_signature = str(mission_signature)
                            ordinal += 1
                            continue
                        if path_id is None or path_id <= 0:
                            source_missing_path_count += 1
                            rows.append(
                                {
                                    "aircraftID": int(aircraft_id),
                                    "inputMissionID": int(input_id),
                                    "previousInputMissionID": (
                                        int(previous_input_id)
                                        if previous_input_id is not None and previous_input_id > 0
                                        else None
                                    ),
                                    "previousMissionSignature": previous_signature,
                                    "pathID": 0,
                                    "channel": str(channel),
                                    "ordinal": int(ordinal),
                                    "signature": str(mission_signature),
                                    "usable": False,
                                }
                            )
                            previous_input_id = int(input_id)
                            previous_signature = str(mission_signature)
                            ordinal += 1
                            continue
                        rows.append(
                            {
                                "aircraftID": int(aircraft_id),
                                "inputMissionID": int(input_id),
                                "previousInputMissionID": (
                                    int(previous_input_id)
                                    if previous_input_id is not None and previous_input_id > 0
                                    else None
                                ),
                                "previousMissionSignature": previous_signature,
                                "pathID": int(path_id),
                                "channel": str(channel),
                                "ordinal": int(ordinal),
                                "signature": str(mission_signature),
                                "usable": True,
                            }
                        )
                        previous_input_id = int(input_id)
                        previous_signature = str(mission_signature)
                        ordinal += 1
                phase_ms["source_template_scan_ms"] = (time.perf_counter() - phase_t0) * 1000.0
                return {
                    "sourcePlanID": int(source_plan_id),
                    "phaseMs": phase_ms,
                    "rowsByAircraft": source_rows_by_aircraft,
                    "warnings": warnings,
                    "sourceImpScanCount": int(source_imp_scan_count),
                    "sourceMissionScanCount": int(source_mission_scan_count),
                    "sourceMissingImpCount": int(source_missing_imp_count),
                    "sourceMissingPathCount": int(source_missing_path_count),
                }

            def _load_input_refresh_source_carry_template(source_plan_id: int) -> tuple[Dict[str, Any], bool, float]:
                cache_key = _input_refresh_source_carry_template_cache_key(int(source_plan_id))
                with input_refresh_carry_template_lock:
                    cached = input_refresh_carry_template_cache.get(cache_key)
                    if isinstance(cached, dict):
                        return cached, True, 0.0
                    started = time.perf_counter()
                    template = _build_input_refresh_source_carry_template(int(source_plan_id))
                    elapsed_ms = (time.perf_counter() - started) * 1000.0
                    template["sourceTemplateBuildMs"] = float(elapsed_ms)
                    input_refresh_carry_template_cache[cache_key] = template
                    return template, False, float(elapsed_ms)

            def _remap_input_refresh_carry_waypoints(
                flight_path: Dict[str, Any],
                *,
                channel: str,
                waypoint_cursor_by_channel: Dict[str, Dict[str, int | None]],
                metrics: Dict[str, int],
            ) -> int:
                waypoint_count = sum(len(rows) for rows in _waypoint_lists_in_flight_path(flight_path))
                if waypoint_count <= 0:
                    return 0
                cursor = waypoint_cursor_by_channel.get(str(channel))
                cursor_next = _safe_int_value(cursor.get("next")) if isinstance(cursor, dict) else None
                cursor_end = _safe_int_value(cursor.get("end")) if isinstance(cursor, dict) else None
                if (
                    cursor_next is not None
                    and cursor_next > 0
                    and (cursor_end is None or int(cursor_next) + int(waypoint_count) - 1 <= int(cursor_end))
                ):
                    if isinstance(cursor, dict):
                        cursor["next"] = int(cursor_next) + int(waypoint_count)
                    if str(channel) == "0303":
                        metrics["waypointBlockUsed0303"] = int(metrics.get("waypointBlockUsed0303", 0)) + int(waypoint_count)
                    else:
                        metrics["waypointBlockUsed0304"] = int(metrics.get("waypointBlockUsed0304", 0)) + int(waypoint_count)
                    return _remap_flight_path_waypoints_from_start(
                        flight_path,
                        start_id=int(cursor_next),
                    )
                metrics["waypointFallbackCount"] = int(metrics.get("waypointFallbackCount", 0)) + 1
                metrics["waypointFallbackWaypoints"] = int(metrics.get("waypointFallbackWaypoints", 0)) + int(waypoint_count)
                return _reserve_and_remap_flight_path_waypoints(flight_path)

            def _emit_input_refresh_carry_template_warnings(
                *,
                template: Dict[str, Any],
                variant_no: int,
                log_emit: Callable[[str], None],
            ) -> None:
                for warning in template.get("warnings") or []:
                    if isinstance(warning, dict) and warning.get("kind") == "source_imp_load_failed":
                        log_emit(
                            f"[WARN] [variant {variant_no}] inputRefresh carry source IMP load failed "
                            f"(aircraftID={warning.get('aircraftID')}, "
                            f"packageID={warning.get('packageID')}): {warning.get('error')}"
                        )

            def _apply_input_refresh_flightpath_carry_forward(
                *,
                variant_no: int,
                option_code: int,
                source_plan_id: Any,
                manned_missions: list[dict],
                unmanned_missions: list[dict],
                waypoint_block_0303_start: Any = None,
                waypoint_block_0303_end: Any = None,
                waypoint_block_0304_start: Any = None,
                waypoint_block_0304_end: Any = None,
                log_emit: Callable[[str], None],
            ) -> Dict[str, Any]:
                summary: Dict[str, Any] = {
                    "applied": False,
                    "reason": "disabled",
                    "sourcePlanID": None,
                    "templateCacheHit": False,
                    "sourceTemplateBuildMs": 0.0,
                    "sourceTemplateScanMs": 0.0,
                    "sourceImpScanCount": 0,
                    "sourceMissionScanCount": 0,
                    "candidates0303": 0,
                    "candidates0304": 0,
                    "carried0303": 0,
                    "carried0304": 0,
                    "build0303": len(unmanned_missions or []),
                    "build0304": len(manned_missions or []),
                    "stoppedAircraft": [],
                    "unmatchedAircraft": [],
                    "carriedWaypoints": 0,
                    "waypointBlockUsed0303": 0,
                    "waypointBlockUsed0304": 0,
                    "waypointFallbackCount": 0,
                    "waypointFallbackWaypoints": 0,
                    "flightPathLoadCount": 0,
                    "flightPathMissingCount": 0,
                    "matchMs": 0.0,
                    "copyMs": 0.0,
                    "flight_plans_0303": [],
                    "flight_plans_0304": [],
                    "manned_to_build": list(manned_missions or []),
                    "unmanned_to_build": list(unmanned_missions or []),
                    "waypoint_block_0303_next": waypoint_block_0303_start,
                    "waypoint_block_0304_next": waypoint_block_0304_start,
                }
                if not _input_refresh_flightpath_carry_enabled():
                    return summary
                source_id = _safe_int_value(source_plan_id)
                if source_id is None or source_id <= 0:
                    summary["reason"] = "source_plan_unavailable"
                    return summary
                summary["sourcePlanID"] = int(source_id)
                if not manned_missions and not unmanned_missions:
                    summary["reason"] = "no_current_missions"
                    return summary

                template_t0 = time.perf_counter()
                try:
                    source_template, template_hit, template_build_ms = _load_input_refresh_source_carry_template(int(source_id))
                except Exception as exc:
                    summary["reason"] = "source_template_failed"
                    summary["error"] = str(exc)
                    log_emit(
                        f"[WARN] [variant {variant_no}] inputRefresh FlightPath carry skipped: {exc}"
                    )
                    return summary
                summary["templateCacheHit"] = bool(template_hit)
                summary["sourceTemplateBuildMs"] = float(template_build_ms)
                phase_ms = source_template.get("phaseMs") if isinstance(source_template.get("phaseMs"), dict) else {}
                summary["sourceTemplateScanMs"] = float(phase_ms.get("source_template_scan_ms") or 0.0)
                summary["sourceImpScanCount"] = int(source_template.get("sourceImpScanCount") or 0)
                summary["sourceMissionScanCount"] = int(source_template.get("sourceMissionScanCount") or 0)
                summary["sourceTemplateLookupMs"] = (time.perf_counter() - template_t0) * 1000.0
                _emit_input_refresh_carry_template_warnings(
                    template=source_template,
                    variant_no=int(variant_no),
                    log_emit=log_emit,
                )

                rows_by_aircraft = (
                    source_template.get("rowsByAircraft")
                    if isinstance(source_template.get("rowsByAircraft"), dict)
                    else {}
                )
                source_match_by_aircraft: Dict[int, Dict[tuple[Any, ...], list[Dict[str, Any]]]] = {}
                for aid, rows in rows_by_aircraft.items():
                    aid_int = _safe_int_value(aid)
                    if aid_int is None or not isinstance(rows, list):
                        continue
                    index: Dict[tuple[Any, ...], list[Dict[str, Any]]] = {}
                    for source_row in rows:
                        if not isinstance(source_row, dict):
                            continue
                        source_input_id = _safe_int_value(source_row.get("inputMissionID"))
                        if source_input_id is None or source_input_id <= 0:
                            continue
                        previous_input_id = _safe_int_value(source_row.get("previousInputMissionID"))
                        key = (
                            int(source_input_id),
                            int(previous_input_id) if previous_input_id is not None and previous_input_id > 0 else None,
                            str(source_row.get("previousMissionSignature") or ""),
                            str(source_row.get("channel") or ""),
                            str(source_row.get("signature") or ""),
                        )
                        index.setdefault(key, []).append(source_row)
                    source_match_by_aircraft[int(aid_int)] = index
                waypoint_cursor_by_channel: Dict[str, Dict[str, int | None]] = {
                    "0303": {
                        "next": _safe_int_value(waypoint_block_0303_start),
                        "end": _safe_int_value(waypoint_block_0303_end),
                    },
                    "0304": {
                        "next": _safe_int_value(waypoint_block_0304_start),
                        "end": _safe_int_value(waypoint_block_0304_end),
                    },
                }
                # SourceArtifactCache returns shared objects. Carry templates must be
                # fully isolated because later hybrid/ETA/FOV post-processing mutates
                # nested waypoint/filming structures in place.
                shallow_carry_clone = False
                carry_object_ids: Set[int] = set()
                carried_fps_0303: list[dict] = []
                carried_fps_0304: list[dict] = []
                unmatched_aircraft: Set[int] = set()
                current_previous_by_aircraft: Dict[int, int | None] = {}
                current_previous_signature_by_aircraft: Dict[int, str | None] = {}
                metrics: Dict[str, int] = {
                    "waypointBlockUsed0303": 0,
                    "waypointBlockUsed0304": 0,
                    "waypointFallbackCount": 0,
                    "waypointFallbackWaypoints": 0,
                }

                match_t0 = time.perf_counter()
                candidate_rows: list[tuple[str, Dict[str, Any], Dict[str, Any]]] = []
                for channel, mission_rows in (
                    ("0304", list(manned_missions or [])),
                    ("0303", list(unmanned_missions or [])),
                ):
                    for mission in mission_rows:
                        if not isinstance(mission, dict):
                            continue
                        aircraft_id = _safe_int_value(mission.get("aircraftID"))
                        if aircraft_id is None or not (1 <= int(aircraft_id) <= 6):
                            continue
                        if str(channel) == "0303":
                            summary["candidates0303"] = int(summary["candidates0303"]) + 1
                        else:
                            summary["candidates0304"] = int(summary["candidates0304"]) + 1
                        current_input_id = _mission_related_input_id(mission)
                        current_path_id = _safe_int_value(mission.get("pathID"))
                        if (
                            current_input_id is None
                            or current_input_id <= 0
                            or current_path_id is None
                            or current_path_id <= 0
                            or bool(mission.get("isDone"))
                        ):
                            unmatched_aircraft.add(int(aircraft_id))
                            continue
                        previous_input_id = current_previous_by_aircraft.get(int(aircraft_id))
                        previous_signature = current_previous_signature_by_aircraft.get(int(aircraft_id))
                        current_signature = _input_refresh_carry_signature(mission)
                        current_previous_by_aircraft[int(aircraft_id)] = int(current_input_id)
                        current_previous_signature_by_aircraft[int(aircraft_id)] = str(current_signature)
                        # The first path starts from the aircraft's live/current position.
                        # Mission equality alone cannot prove that this ingress leg is
                        # unchanged, so it must always be regenerated.
                        if previous_input_id is None:
                            unmatched_aircraft.add(int(aircraft_id))
                            continue
                        key = (
                            int(current_input_id),
                            int(previous_input_id) if previous_input_id is not None and previous_input_id > 0 else None,
                            str(previous_signature or ""),
                            str(channel),
                            str(current_signature),
                        )
                        source_candidates = source_match_by_aircraft.get(int(aircraft_id), {}).get(key) or []
                        if not source_candidates:
                            unmatched_aircraft.add(int(aircraft_id))
                            continue
                        source_row = source_candidates.pop(0)
                        candidate_rows.append((str(channel), mission, source_row))
                summary["matchMs"] = (time.perf_counter() - match_t0) * 1000.0

                copy_t0 = time.perf_counter()
                for channel, mission, source_row in candidate_rows:
                    source_path_id = _safe_int_value(source_row.get("pathID"))
                    current_path_id = _safe_int_value(mission.get("pathID"))
                    aircraft_id = _safe_int_value(mission.get("aircraftID"))
                    if (
                        source_path_id is None
                        or source_path_id <= 0
                        or current_path_id is None
                        or current_path_id <= 0
                        or aircraft_id is None
                    ):
                        if aircraft_id is not None:
                            unmatched_aircraft.add(int(aircraft_id))
                        continue
                    try:
                        source_fp = source_cache.load_flight_path(int(source_path_id), copy_result=False)
                    except Exception:
                        summary["flightPathMissingCount"] = int(summary["flightPathMissingCount"]) + 1
                        unmatched_aircraft.add(int(aircraft_id))
                        continue
                    if not isinstance(source_fp, dict):
                        summary["flightPathMissingCount"] = int(summary["flightPathMissingCount"]) + 1
                        unmatched_aircraft.add(int(aircraft_id))
                        continue
                    summary["flightPathLoadCount"] = int(summary["flightPathLoadCount"]) + 1
                    fp_copy = _clone_carry_flight_path_template(
                        source_fp,
                        shallow=bool(shallow_carry_clone),
                    )
                    fp_copy["aircraftID"] = int(aircraft_id)
                    fp_copy["pathID"] = int(current_path_id)
                    fp_copy["timestamp"] = int(_now_ms_since_2000())
                    # 소스 계획의 individualMissionID 가 남아 있으면, 이후
                    # _enforce_fp_path_ids 의 (aircraftID, missionID) 우선 매칭이
                    # 현재 variant 의 '다른' 임무와 우연히 충돌해 그 임무의
                    # pathID 를 가로채고(중복 -> 수리 -> 임무-경로 매핑 이탈)
                    # 무결성 검사에서 missing pathID 로 재계획 전체가 실패한다.
                    # carry 된 경로는 매칭된 '현재' 임무의 identity 로 재바인딩한다.
                    current_mission_id = _safe_int_value(mission.get("individualMissionID"))
                    for stale_mission_key in ("individualMissionId", "missionID", "missionId"):
                        fp_copy.pop(stale_mission_key, None)
                    if current_mission_id is not None and int(current_mission_id) > 0:
                        fp_copy["individualMissionID"] = int(current_mission_id)
                    else:
                        fp_copy.pop("individualMissionID", None)
                    copied_waypoints = _remap_input_refresh_carry_waypoints(
                        fp_copy,
                        channel=str(channel),
                        waypoint_cursor_by_channel=waypoint_cursor_by_channel,
                        metrics=metrics,
                    )
                    summary["carriedWaypoints"] = int(summary["carriedWaypoints"]) + int(copied_waypoints)
                    carry_object_ids.add(id(mission))
                    if str(channel) == "0303":
                        carried_fps_0303.append(fp_copy)
                    else:
                        carried_fps_0304.append(fp_copy)
                summary["copyMs"] = (time.perf_counter() - copy_t0) * 1000.0

                manned_to_build = [mission for mission in (manned_missions or []) if id(mission) not in carry_object_ids]
                unmanned_to_build = [mission for mission in (unmanned_missions or []) if id(mission) not in carry_object_ids]
                summary.update(
                    {
                        "applied": bool(carried_fps_0303 or carried_fps_0304),
                        "reason": "ok" if (carried_fps_0303 or carried_fps_0304) else "no_exact_prev_match",
                        "carried0303": int(len(carried_fps_0303)),
                        "carried0304": int(len(carried_fps_0304)),
                        "build0303": int(len(unmanned_to_build)),
                        "build0304": int(len(manned_to_build)),
                        "stoppedAircraft": sorted(int(aid) for aid in unmatched_aircraft),
                        "unmatchedAircraft": sorted(int(aid) for aid in unmatched_aircraft),
                        "waypointBlockUsed0303": int(metrics.get("waypointBlockUsed0303", 0)),
                        "waypointBlockUsed0304": int(metrics.get("waypointBlockUsed0304", 0)),
                        "waypointFallbackCount": int(metrics.get("waypointFallbackCount", 0)),
                        "waypointFallbackWaypoints": int(metrics.get("waypointFallbackWaypoints", 0)),
                        "flight_plans_0303": carried_fps_0303,
                        "flight_plans_0304": carried_fps_0304,
                        "manned_to_build": manned_to_build,
                        "unmanned_to_build": unmanned_to_build,
                        "waypoint_block_0303_next": (
                            waypoint_cursor_by_channel.get("0303", {}).get("next")
                            if isinstance(waypoint_cursor_by_channel.get("0303"), dict)
                            else waypoint_block_0303_start
                        ),
                        "waypoint_block_0304_next": (
                            waypoint_cursor_by_channel.get("0304", {}).get("next")
                            if isinstance(waypoint_cursor_by_channel.get("0304"), dict)
                            else waypoint_block_0304_start
                        ),
                    }
                )
                log_emit(
                    "[REPLAN][METRIC] input_refresh_flightpath_carry_forward "
                    f"variant={int(variant_no)} option={int(option_code)} "
                    f"sourcePlan={int(source_id)} "
                    f"applied={int(bool(summary['applied']))} "
                    f"reason={summary['reason']} "
                    f"templateCacheHit={int(bool(summary.get('templateCacheHit')))} "
                    f"sourceTemplateBuildMs={float(summary.get('sourceTemplateBuildMs') or 0.0):.3f} "
                    f"sourceTemplateLookupMs={float(summary.get('sourceTemplateLookupMs') or 0.0):.3f} "
                    f"sourceTemplateScanMs={float(summary.get('sourceTemplateScanMs') or 0.0):.3f} "
                    f"sourceImpScanCount={int(summary.get('sourceImpScanCount') or 0)} "
                    f"sourceMissionScanCount={int(summary.get('sourceMissionScanCount') or 0)} "
                    f"candidates0303={int(summary['candidates0303'])} "
                    f"candidates0304={int(summary['candidates0304'])} "
                    f"carried0303={int(summary['carried0303'])} "
                    f"carried0304={int(summary['carried0304'])} "
                    f"build0303={int(summary['build0303'])} "
                    f"build0304={int(summary['build0304'])} "
                    f"unmatchedAircraft={'|'.join(str(v) for v in summary['unmatchedAircraft']) or '-'} "
                    f"flightPathLoadCount={int(summary['flightPathLoadCount'])} "
                    f"flightPathMissingCount={int(summary['flightPathMissingCount'])} "
                    f"matchMs={float(summary['matchMs']):.3f} "
                    f"copyMs={float(summary['copyMs']):.3f} "
                    f"carriedWaypoints={int(summary['carriedWaypoints'])} "
                    f"waypointBlockUsed0303={int(summary['waypointBlockUsed0303'])} "
                    f"waypointBlockUsed0304={int(summary['waypointBlockUsed0304'])} "
                    f"waypointFallbackCount={int(summary['waypointFallbackCount'])} "
                    f"waypointFallbackWaypoints={int(summary['waypointFallbackWaypoints'])} "
                    f"shallowCarryClone={int(bool(shallow_carry_clone))}"
                )
                return summary

            input_refresh_variant_fp_template_lock = threading.RLock()
            input_refresh_variant_fp_template_cache: Dict[str, concurrent.futures.Future] = {}
            input_refresh_shared_split_state: Dict[str, Any] | None = None
            if input_refresh_fast_dem_context and _runtime_env_flag(
                "input_refresh_shared_split_enabled",
                "REPLAN_INPUT_REFRESH_SHARED_SPLIT",
                True,
            ):
                # Per-replan only: the enhanced planner additionally keys by the
                # fully filtered 0201/0202 content before sharing a prototype.
                input_refresh_shared_split_state = {
                    "lock": threading.RLock(),
                    "futures": {},
                }

            def _input_refresh_variant_fp_template_enabled() -> bool:
                return bool(
                    input_refresh_fast_dem_context
                    and _runtime_env_flag(
                        "input_refresh_variant_flightpath_template_enabled",
                        "REPLAN_INPUT_REFRESH_VARIANT_FP_TEMPLATE",
                        True,
                    )
                )

            def _input_refresh_variant_fp_template_key(
                missions: list[dict],
                *,
                source_plan_id: Any,
            ) -> str:
                rows: list[Any] = []
                for mission in missions or []:
                    if not isinstance(mission, dict):
                        continue
                    aircraft_id = _safe_int_value(mission.get("aircraftID"))
                    rows.append(
                        [
                            int(aircraft_id or 0),
                            str(_input_refresh_flightpath_channel(int(aircraft_id or 0)) or ""),
                            _input_refresh_variant_mission_signature(mission),
                        ]
                    )
                return json.dumps(
                    {
                        "sourcePlanID": int(_safe_int_value(source_plan_id) or 0),
                        "missionRows": rows,
                    },
                    sort_keys=True,
                    ensure_ascii=False,
                    separators=(",", ":"),
                )

            def _sort_flightpaths_for_mission_order(
                flight_plans: list[dict],
                missions: list[dict],
                *,
                formation_leader_first: bool,
            ) -> list[dict]:
                order_by_path: Dict[int, int] = {}
                for idx, mission in enumerate(missions or []):
                    if not isinstance(mission, dict):
                        continue
                    path_id = _safe_int_value(mission.get("pathID"))
                    if path_id is not None and path_id > 0 and int(path_id) not in order_by_path:
                        order_by_path[int(path_id)] = int(idx)
                indexed = list(enumerate(fp for fp in (flight_plans or []) if isinstance(fp, dict)))
                indexed.sort(
                    key=lambda item: (
                        order_by_path.get(
                            int(_safe_int_value(item[1].get("pathID")) or -1),
                            len(order_by_path) + int(item[0]),
                        ),
                        int(item[0]),
                    )
                )
                ordered = [fp for _idx, fp in indexed]
                if not formation_leader_first or len(ordered) < 2:
                    return ordered
                idx = 0
                while idx < len(ordered):
                    plan = ordered[idx]
                    formation = plan.get("formationInfo") if isinstance(plan.get("formationInfo"), dict) else {}
                    leader_id = _safe_int_value(formation.get("leaderAircraftID"))
                    aircraft_id = _safe_int_value(plan.get("aircraftID"))
                    if leader_id is None or aircraft_id is None or int(leader_id) == int(aircraft_id):
                        idx += 1
                        continue
                    leader_idx = None
                    for candidate_idx, candidate in enumerate(ordered):
                        if candidate_idx == idx:
                            continue
                        if _safe_int_value(candidate.get("aircraftID")) == int(leader_id):
                            leader_idx = int(candidate_idx)
                            break
                    if leader_idx is not None and leader_idx > idx:
                        leader_plan = ordered.pop(leader_idx)
                        ordered.insert(idx, leader_plan)
                        idx += 2
                        continue
                    idx += 1
                return ordered

            def _renumber_flightpath_waypoints(
                flight_plans: list[dict],
                *,
                list_key: str,
                block_start: Any,
                block_end: Any,
            ) -> int:
                next_id = _safe_int_value(block_start)
                end_id = _safe_int_value(block_end)
                if next_id is None or next_id <= 0:
                    raise RuntimeError(f"Waypoint block start unavailable for {list_key}")
                count = 0
                for flight_path in flight_plans or []:
                    rows = flight_path.get(list_key) if isinstance(flight_path, dict) else None
                    if not isinstance(rows, list):
                        continue
                    waypoint_rows = [row for row in rows if isinstance(row, dict)]
                    if end_id is not None and int(next_id) + len(waypoint_rows) - 1 > int(end_id):
                        raise RuntimeError(f"Waypoint reserved block exhausted for {list_key}")
                    for waypoint in waypoint_rows:
                        waypoint["waypointID"] = int(next_id)
                        next_id += 1
                        count += 1
                    for idx, waypoint in enumerate(waypoint_rows):
                        waypoint["nextWaypointID"] = (
                            int(waypoint_rows[idx + 1].get("waypointID") or 0)
                            if idx + 1 < len(waypoint_rows)
                            else 0
                        )
                return int(count)

            def _normalize_input_refresh_variant_flightpaths(
                *,
                flight_plans_0303: list[dict],
                flight_plans_0304: list[dict],
                missions: list[dict],
                waypoint_block_0303_start: Any,
                waypoint_block_0303_end: Any,
                waypoint_block_0304_start: Any,
                waypoint_block_0304_end: Any,
            ) -> tuple[list[dict], list[dict], Dict[str, Any]]:
                manned_missions = [
                    mission for mission in (missions or [])
                    if isinstance(mission, dict) and int(mission.get("aircraftID", 0) or 0) in (1, 2, 3)
                ]
                unmanned_missions = [
                    mission for mission in (missions or [])
                    if isinstance(mission, dict) and int(mission.get("aircraftID", 0) or 0) in (4, 5, 6)
                ]
                ordered_0303 = _sort_flightpaths_for_mission_order(
                    list(flight_plans_0303 or []),
                    unmanned_missions,
                    formation_leader_first=True,
                )
                ordered_0304 = _sort_flightpaths_for_mission_order(
                    list(flight_plans_0304 or []),
                    manned_missions,
                    formation_leader_first=False,
                )
                count_0303 = _renumber_flightpath_waypoints(
                    ordered_0303,
                    list_key="waypointList",
                    block_start=waypoint_block_0303_start,
                    block_end=waypoint_block_0303_end,
                )
                count_0304 = _renumber_flightpath_waypoints(
                    ordered_0304,
                    list_key="lahWaypointList",
                    block_start=waypoint_block_0304_start,
                    block_end=waypoint_block_0304_end,
                )
                return ordered_0303, ordered_0304, {
                    "waypoints0303": int(count_0303),
                    "waypoints0304": int(count_0304),
                }

            def _clone_input_refresh_variant_fp_template(
                template: Dict[str, Any],
                *,
                missions: list[dict],
                waypoint_block_0303_start: Any,
                waypoint_block_0303_end: Any,
                waypoint_block_0304_start: Any,
                waypoint_block_0304_end: Any,
            ) -> tuple[list[dict], list[dict], Dict[str, Any]]:
                source_missions = list(template.get("missions") or [])
                current_missions = list(missions or [])
                if len(source_missions) != len(current_missions):
                    raise RuntimeError("inputRefresh variant template mission count mismatch")
                path_id_map: Dict[int, int] = {}
                mission_id_by_path: Dict[int, int] = {}
                for source_mission, current_mission in zip(source_missions, current_missions):
                    if not isinstance(source_mission, dict) or not isinstance(current_mission, dict):
                        raise RuntimeError("inputRefresh variant template mission row mismatch")
                    if _input_refresh_variant_mission_signature(
                        source_mission
                    ) != _input_refresh_variant_mission_signature(current_mission):
                        raise RuntimeError("inputRefresh variant template mission signature mismatch")
                    source_path_id = _safe_int_value(source_mission.get("pathID"))
                    current_path_id = _safe_int_value(current_mission.get("pathID"))
                    if source_path_id is None or current_path_id is None:
                        raise RuntimeError("inputRefresh variant template pathID unavailable")
                    path_id_map[int(source_path_id)] = int(current_path_id)
                    current_mission_id = _safe_int_value(current_mission.get("individualMissionID"))
                    if current_mission_id is not None and current_mission_id > 0:
                        mission_id_by_path[int(current_path_id)] = int(current_mission_id)

                def _clone_group(rows: Any) -> list[dict]:
                    cloned_rows = copy.deepcopy(list(rows or []))
                    for flight_path in cloned_rows:
                        if not isinstance(flight_path, dict):
                            continue
                        source_path_id = _safe_int_value(flight_path.get("pathID"))
                        if source_path_id is None or int(source_path_id) not in path_id_map:
                            raise RuntimeError("inputRefresh variant template FlightPath mapping failed")
                        current_path_id = int(path_id_map[int(source_path_id)])
                        flight_path["pathID"] = int(current_path_id)
                        current_mission_id = mission_id_by_path.get(int(current_path_id))
                        if current_mission_id is not None:
                            flight_path["individualMissionID"] = int(current_mission_id)
                    return cloned_rows

                cloned_0303 = _clone_group(template.get("flight_plans_0303"))
                cloned_0304 = _clone_group(template.get("flight_plans_0304"))
                normalized_0303, normalized_0304, counts = _normalize_input_refresh_variant_flightpaths(
                    flight_plans_0303=cloned_0303,
                    flight_plans_0304=cloned_0304,
                    missions=current_missions,
                    waypoint_block_0303_start=waypoint_block_0303_start,
                    waypoint_block_0303_end=waypoint_block_0303_end,
                    waypoint_block_0304_start=waypoint_block_0304_start,
                    waypoint_block_0304_end=waypoint_block_0304_end,
                )
                counts["immutable0304PathIDs"] = [
                    int(path_id_map[int(source_path_id)])
                    for source_path_id in (template.get("immutable_0304_path_ids") or [])
                    if _safe_int_value(source_path_id) is not None
                    and int(source_path_id) in path_id_map
                ]
                return normalized_0303, normalized_0304, counts

            reexecute_future_carry_template_cache: Dict[Any, Dict[str, Any]] = {}
            reexecute_future_carry_template_lock = threading.RLock()

            def _reexecute_future_carry_template_cache_key(
                *,
                source_plan_id: int,
                current_input_id: int,
                source_template_input_id: int | None,
                future_input_ids: Set[int],
            ) -> tuple[Any, ...]:
                return (
                    int(source_plan_id),
                    int(current_input_id),
                    int(source_template_input_id or 0),
                    tuple(sorted(int(value) for value in future_input_ids)),
                )

            def _build_reexecute_future_carry_template(
                *,
                source_plan_id: int,
                current_input_id: int,
                source_template_input_id: int | None,
                future_input_ids: Set[int],
            ) -> Dict[str, Any]:
                phase_ms: Dict[str, float] = {}
                template_warnings: list[Any] = []

                def _collect_template_warning(message: str) -> None:
                    template_warnings.append({"kind": "message", "message": str(message)})

                phase_t0 = time.perf_counter()
                source_plan = source_cache.load_mission_plan(int(source_plan_id))
                phase_ms["source_plan_load_ms"] = (time.perf_counter() - phase_t0) * 1000.0
                source_aircraft_rows = source_plan.get("aircraftList") if isinstance(source_plan, dict) else None
                if not isinstance(source_aircraft_rows, list) or not source_aircraft_rows:
                    raise RuntimeError(f"reexecute fast-path source MissionPlan invalid: {source_plan_id}")

                phase_t0 = time.perf_counter()
                aircraft_rows_by_id: Dict[int, Dict[str, Any]] = {}
                for aircraft in source_aircraft_rows:
                    if not isinstance(aircraft, dict):
                        continue
                    aircraft_id = _safe_int_value(aircraft.get("aircraftID"))
                    if aircraft_id is None or aircraft_id <= 0:
                        continue
                    aircraft_rows_by_id.setdefault(
                        int(aircraft_id),
                        {"aircraftID": int(aircraft_id), "individualMissionPackageID": 0},
                    )
                phase_ms["source_aircraft_index_ms"] = (time.perf_counter() - phase_t0) * 1000.0

                phase_t0 = time.perf_counter()
                lah_template_input_id = resolve_reexecute_lah_template_input_id(
                    current_input_id,
                    source_template_input_id,
                )
                source_lah_current_templates = _source_lah_current_mission_templates(
                    source_aircraft_rows=source_aircraft_rows,
                    template_input_id=int(lah_template_input_id or current_input_id),
                    log_emit=_collect_template_warning,
                )
                if (
                    not source_lah_current_templates
                    and lah_template_input_id is not None
                    and int(lah_template_input_id) != int(current_input_id)
                ):
                    source_lah_current_templates = _source_lah_current_mission_templates(
                        source_aircraft_rows=source_aircraft_rows,
                        template_input_id=int(current_input_id),
                        log_emit=_collect_template_warning,
                    )
                    if source_lah_current_templates:
                        lah_template_input_id = int(current_input_id)
                phase_ms["lah_template_resolve_ms"] = (time.perf_counter() - phase_t0) * 1000.0

                carry_rows: list[Dict[str, Any]] = []
                missing_fp_count = 0
                source_imp_scan_count = 0
                source_mission_scan_count = 0
                source_flight_path_load_count = 0

                phase_t0 = time.perf_counter()
                for source_aircraft in source_aircraft_rows:
                    if not isinstance(source_aircraft, dict):
                        continue
                    aircraft_id = _safe_int_value(source_aircraft.get("aircraftID"))
                    if aircraft_id is None or not (1 <= int(aircraft_id) <= 6):
                        continue
                    package_id = _safe_int_value(
                        source_aircraft.get("individualMissionPackageID")
                        or source_aircraft.get("individualMissionPlanPackageID")
                        or source_aircraft.get("individualMissionPackageId")
                    )
                    if package_id is None or package_id <= 0:
                        continue
                    try:
                        source_imp = source_cache.load_individual_mission_plan(int(package_id))
                    except Exception as exc:
                        template_warnings.append(
                            {
                                "kind": "source_imp_load_failed",
                                "aircraftID": int(aircraft_id),
                                "packageID": int(package_id),
                                "error": str(exc),
                            }
                        )
                        continue
                    source_imp_scan_count += 1
                    for source_mission in source_imp.get("individualMissionList") or []:
                        if not isinstance(source_mission, dict):
                            continue
                        source_mission_scan_count += 1
                        input_id = _mission_related_input_id(source_mission)
                        if input_id is None or int(input_id) not in future_input_ids:
                            continue
                        if bool(source_mission.get("isDone")):
                            continue
                        path_id = _safe_int_value(source_mission.get("pathID"))
                        if path_id is None or path_id <= 0:
                            continue
                        try:
                            flight_path = source_cache.load_flight_path(int(path_id))
                        except Exception as exc:
                            missing_fp_count += 1
                            template_warnings.append(
                                {
                                    "kind": "source_flight_path_load_failed",
                                    "pathID": int(path_id),
                                    "inputMissionID": int(input_id),
                                    "error": str(exc),
                                }
                            )
                            continue
                        source_flight_path_load_count += 1
                        carry_rows.append(
                            {
                                "aircraftID": int(aircraft_id),
                                "inputMissionID": int(input_id),
                                "sourceMission": source_mission,
                                "flightPath": flight_path,
                            }
                        )
                phase_ms["source_future_template_scan_ms"] = (time.perf_counter() - phase_t0) * 1000.0
                return {
                    "phaseMs": phase_ms,
                    "aircraftRowsById": aircraft_rows_by_id,
                    "sourceLahCurrentTemplates": source_lah_current_templates,
                    "sourceLahTemplateInputMissionID": int(lah_template_input_id or current_input_id),
                    "carryRows": carry_rows,
                    "warnings": template_warnings,
                    "sourceImpScanCount": int(source_imp_scan_count),
                    "sourceMissionScanCount": int(source_mission_scan_count),
                    "sourceFlightPathLoadCount": int(source_flight_path_load_count),
                    "missingFlightPathCount": int(missing_fp_count),
                }

            def _load_reexecute_future_carry_template(
                *,
                source_plan_id: int,
                current_input_id: int,
                source_template_input_id: int | None,
                future_input_ids: Set[int],
            ) -> tuple[Dict[str, Any], bool, float]:
                cache_key = _reexecute_future_carry_template_cache_key(
                    source_plan_id=int(source_plan_id),
                    current_input_id=int(current_input_id),
                    source_template_input_id=source_template_input_id,
                    future_input_ids=future_input_ids,
                )
                with reexecute_future_carry_template_lock:
                    existing = reexecute_future_carry_template_cache.get(cache_key)
                    if isinstance(existing, dict):
                        return existing, True, 0.0
                    build_t0 = time.perf_counter()
                    template = _build_reexecute_future_carry_template(
                        source_plan_id=int(source_plan_id),
                        current_input_id=int(current_input_id),
                        source_template_input_id=source_template_input_id,
                        future_input_ids=future_input_ids,
                    )
                    build_ms = (time.perf_counter() - build_t0) * 1000.0
                    template["sourceTemplateBuildMs"] = float(build_ms)
                    reexecute_future_carry_template_cache[cache_key] = template
                    return template, False, float(build_ms)

            def _emit_reexecute_future_carry_template_warnings(
                *,
                template: Dict[str, Any],
                variant_no: int,
                log_emit: Callable[[str], None],
            ) -> None:
                for warning in template.get("warnings") or []:
                    if isinstance(warning, dict) and warning.get("kind") == "message":
                        message = warning.get("message")
                        if message:
                            log_emit(str(message))
                    elif isinstance(warning, dict) and warning.get("kind") == "source_imp_load_failed":
                        log_emit(
                            f"[WARN] [variant {variant_no}] reexecute fast-path source IMP load failed "
                            f"(aircraftID={warning.get('aircraftID')}, "
                            f"packageID={warning.get('packageID')}): {warning.get('error')}"
                        )
                    elif isinstance(warning, dict) and warning.get("kind") == "source_flight_path_load_failed":
                        log_emit(
                            f"[WARN] [variant {variant_no}] reexecute fast-path source FlightPath load failed "
                            f"(pathID={warning.get('pathID')}, "
                            f"inputMissionID={warning.get('inputMissionID')}): {warning.get('error')}"
                        )

            def _append_reexecute_current_fast_path_future_artifacts(
                *,
                variant_no: int,
                request: CurrentRemainingHybridRequest,
                hybrid_result: Any,
                mp_json: Dict[str, Any],
                missions: list[dict],
                flight_plans_0303: list[dict],
                flight_plans_0304: list[dict],
                log_emit: Callable[[str], None],
                waypoint_block_0303_start: Any = None,
                waypoint_block_0303_end: Any = None,
                waypoint_block_0304_start: Any = None,
                waypoint_block_0304_end: Any = None,
            ) -> Dict[str, Any]:
                source_plan_id = _safe_int_value(getattr(request, "source_plan_id", None))
                current_input_id = _safe_int_value(getattr(request, "current_input_id", None))
                source_template_input_id = _safe_int_value(
                    getattr(request, "source_template_input_id", None)
                )
                if source_plan_id is None or source_plan_id <= 0:
                    raise RuntimeError("reexecute fast-path source plan unavailable")
                if current_input_id is None or current_input_id <= 0:
                    raise RuntimeError("reexecute fast-path current input unavailable")
                pending_input_ids = _current_cmpk_pending_input_ids()
                if not pending_input_ids:
                    raise RuntimeError("reexecute fast-path pending input list unavailable")
                future_input_ids = set(int(value) for value in pending_input_ids)
                future_input_ids.discard(int(current_input_id))
                if source_template_input_id is not None and source_template_input_id > 0:
                    future_input_ids.discard(int(source_template_input_id))

                carry_phase_ms: Dict[str, float] = {}
                phase_t0 = time.perf_counter()
                source_template, source_template_cache_hit, source_template_build_ms = _load_reexecute_future_carry_template(
                    source_plan_id=int(source_plan_id),
                    current_input_id=int(current_input_id),
                    source_template_input_id=(
                        int(source_template_input_id)
                        if source_template_input_id is not None and source_template_input_id > 0
                        else None
                    ),
                    future_input_ids=future_input_ids,
                )
                carry_phase_ms["source_template_lookup_ms"] = (time.perf_counter() - phase_t0) * 1000.0
                carry_phase_ms["source_template_build_ms"] = float(source_template_build_ms)
                source_template_phase_ms = (
                    source_template.get("phaseMs")
                    if isinstance(source_template.get("phaseMs"), dict)
                    else {}
                )
                if source_template_cache_hit:
                    carry_phase_ms["source_plan_load_ms"] = 0.0
                    carry_phase_ms["source_aircraft_index_ms"] = 0.0
                    carry_phase_ms["lah_template_resolve_ms"] = 0.0
                    carry_phase_ms["source_future_template_scan_ms"] = 0.0
                else:
                    carry_phase_ms["source_plan_load_ms"] = float(source_template_phase_ms.get("source_plan_load_ms") or 0.0)
                    carry_phase_ms["source_aircraft_index_ms"] = float(source_template_phase_ms.get("source_aircraft_index_ms") or 0.0)
                    carry_phase_ms["lah_template_resolve_ms"] = float(source_template_phase_ms.get("lah_template_resolve_ms") or 0.0)
                    carry_phase_ms["source_future_template_scan_ms"] = float(
                        source_template_phase_ms.get("source_future_template_scan_ms") or 0.0
                    )
                _emit_reexecute_future_carry_template_warnings(
                    template=source_template,
                    variant_no=int(variant_no),
                    log_emit=log_emit,
                )
                aircraft_rows_by_id = (
                    source_template.get("aircraftRowsById")
                    if isinstance(source_template.get("aircraftRowsById"), dict)
                    else {}
                )
                source_lah_current_templates = (
                    source_template.get("sourceLahCurrentTemplates")
                    if isinstance(source_template.get("sourceLahCurrentTemplates"), dict)
                    else {}
                )
                phase_t0 = time.perf_counter()
                lah_hold_summary = _append_reexecute_lah_line_hold_current_artifacts(
                    request=request,
                    source_aircraft_ids=set(int(aid) for aid in aircraft_rows_by_id.keys()),
                    source_lah_templates=source_lah_current_templates,
                    missions=missions,
                    flight_plans_0304=flight_plans_0304,
                    log_emit=log_emit,
                )
                carry_phase_ms["lah_hold_append_ms"] = (time.perf_counter() - phase_t0) * 1000.0
                carried_missions: list[dict] = []
                carried_fps_0303: list[dict] = []
                carried_fps_0304: list[dict] = []
                carried_input_ids: Set[int] = set()
                copied_waypoints = 0
                missing_fp_count = int(source_template.get("missingFlightPathCount") or 0)
                source_imp_scan_count = (
                    0 if source_template_cache_hit else int(source_template.get("sourceImpScanCount") or 0)
                )
                source_mission_scan_count = (
                    0 if source_template_cache_hit else int(source_template.get("sourceMissionScanCount") or 0)
                )
                source_flight_path_load_count = (
                    0 if source_template_cache_hit else int(source_template.get("sourceFlightPathLoadCount") or 0)
                )
                source_template_imp_scan_count = int(source_template.get("sourceImpScanCount") or 0)
                source_template_mission_scan_count = int(source_template.get("sourceMissionScanCount") or 0)
                source_template_flight_path_load_count = int(source_template.get("sourceFlightPathLoadCount") or 0)
                source_template_missing_flight_path_count = int(source_template.get("missingFlightPathCount") or 0)
                source_template_carry_rows = (
                    source_template.get("carryRows")
                    if isinstance(source_template.get("carryRows"), list)
                    else []
                )
                source_flight_path_copy_count = 0
                source_flight_path_shallow_clone_count = 0
                source_flight_path_deepcopy_count = 0
                waypoint_block_used_0303 = 0
                waypoint_block_used_0304 = 0
                waypoint_fallback_count = 0
                waypoint_fallback_waypoints = 0
                shallow_carry_clone = _runtime_env_flag(
                    "replan_reexecute_current_fast_path_shallow_carry_clone",
                    "REPLAN_REEXECUTE_CURRENT_FAST_PATH_SHALLOW_CARRY",
                    True,
                )
                waypoint_cursor_by_channel: Dict[str, Dict[str, int | None]] = {
                    "0303": {
                        "next": _safe_int_value(waypoint_block_0303_start),
                        "end": _safe_int_value(waypoint_block_0303_end),
                    },
                    "0304": {
                        "next": _safe_int_value(waypoint_block_0304_start),
                        "end": _safe_int_value(waypoint_block_0304_end),
                    },
                }

                def _remap_carry_waypoints(flight_path: Dict[str, Any], *, channel: str) -> int:
                    nonlocal waypoint_block_used_0303
                    nonlocal waypoint_block_used_0304
                    nonlocal waypoint_fallback_count
                    nonlocal waypoint_fallback_waypoints
                    waypoint_count = sum(len(rows) for rows in _waypoint_lists_in_flight_path(flight_path))
                    if waypoint_count <= 0:
                        return 0
                    cursor = waypoint_cursor_by_channel.get(str(channel))
                    cursor_next = _safe_int_value(cursor.get("next")) if isinstance(cursor, dict) else None
                    cursor_end = _safe_int_value(cursor.get("end")) if isinstance(cursor, dict) else None
                    if (
                        cursor_next is not None
                        and cursor_next > 0
                        and (cursor_end is None or int(cursor_next) + int(waypoint_count) - 1 <= int(cursor_end))
                    ):
                        if isinstance(cursor, dict):
                            cursor["next"] = int(cursor_next) + int(waypoint_count)
                        if str(channel) == "0303":
                            waypoint_block_used_0303 += int(waypoint_count)
                        else:
                            waypoint_block_used_0304 += int(waypoint_count)
                        return _remap_flight_path_waypoints_from_start(
                            flight_path,
                            start_id=int(cursor_next),
                        )
                    waypoint_fallback_count += 1
                    waypoint_fallback_waypoints += int(waypoint_count)
                    return _reserve_and_remap_flight_path_waypoints(flight_path)

                if not future_input_ids:
                    log_emit(
                        f"[INFO] [variant {variant_no}] reexecute fast-path has no future input missions; "
                        "current mission only."
                    )

                phase_t0 = time.perf_counter()
                for carry_row in source_template_carry_rows:
                    if not isinstance(carry_row, dict):
                        continue
                    aircraft_id = _safe_int_value(carry_row.get("aircraftID"))
                    if aircraft_id is None or not (1 <= int(aircraft_id) <= 6):
                        continue
                    input_id = _safe_int_value(carry_row.get("inputMissionID"))
                    if input_id is None or int(input_id) not in future_input_ids:
                        continue
                    source_mission = carry_row.get("sourceMission")
                    flight_path = carry_row.get("flightPath")
                    if not isinstance(source_mission, dict) or not isinstance(flight_path, dict):
                        continue
                    mission_copy = copy.deepcopy(source_mission)
                    mission_copy["aircraftID"] = int(aircraft_id)
                    mission_copy["individualMissionPlanPackageID"] = 0
                    flight_path_copy = _clone_carry_flight_path_template(
                        flight_path,
                        shallow=bool(shallow_carry_clone),
                    )
                    flight_path_copy["aircraftID"] = int(aircraft_id)
                    source_flight_path_copy_count += 1
                    if shallow_carry_clone:
                        source_flight_path_shallow_clone_count += 1
                    else:
                        source_flight_path_deepcopy_count += 1
                    carried_missions.append(mission_copy)
                    if int(aircraft_id) in (1, 2, 3):
                        copied_waypoints += _remap_carry_waypoints(flight_path_copy, channel="0304")
                        carried_fps_0304.append(flight_path_copy)
                    else:
                        copied_waypoints += _remap_carry_waypoints(flight_path_copy, channel="0303")
                        carried_fps_0303.append(flight_path_copy)
                    carried_input_ids.add(int(input_id))
                carry_phase_ms["future_scan_copy_ms"] = (time.perf_counter() - phase_t0) * 1000.0

                if future_input_ids and not carried_missions:
                    raise RuntimeError(
                        "reexecute fast-path found no carry-forward missions "
                        f"(sourcePlan={source_plan_id}, futureInputs={sorted(future_input_ids)})"
                    )
                missing_future_input_ids = sorted(
                    int(value)
                    for value in set(int(v) for v in future_input_ids).difference(carried_input_ids)
                )
                if missing_future_input_ids:
                    raise RuntimeError(
                        "reexecute fast-path missing source carry-forward missions "
                        f"(sourcePlan={source_plan_id}, missingFutureInputs={missing_future_input_ids})"
                    )

                phase_t0 = time.perf_counter()
                active_aircraft_ids = {
                    int(mission.get("aircraftID", 0))
                    for mission in list(missions or []) + carried_missions
                }
                for aircraft_id in sorted(aircraft_rows_by_id):
                    if int(aircraft_id) not in active_aircraft_ids:
                        continue
                    if not any(
                        _safe_int_value(row.get("aircraftID")) == int(aircraft_id)
                        for row in mp_json.get("aircraftList", [])
                        if isinstance(row, dict)
                    ):
                        mp_json.setdefault("aircraftList", []).append(copy.deepcopy(aircraft_rows_by_id[aircraft_id]))
                mp_json["aircraftList"] = sorted(
                    [
                        row
                        for row in (mp_json.get("aircraftList") or [])
                        if isinstance(row, dict)
                        and _safe_int_value(row.get("aircraftID")) is not None
                        and int(_safe_int_value(row.get("aircraftID")) or 0) in active_aircraft_ids
                    ],
                    key=lambda row: int(row.get("aircraftID", 0)),
                )
                carry_phase_ms["aircraft_list_merge_ms"] = (time.perf_counter() - phase_t0) * 1000.0

                phase_t0 = time.perf_counter()
                missions.extend(carried_missions)
                flight_plans_0303.extend(carried_fps_0303)
                flight_plans_0304.extend(carried_fps_0304)
                carry_phase_ms["append_lists_ms"] = (time.perf_counter() - phase_t0) * 1000.0
                log_emit(
                    "[REPLAN][METRIC] reexecute_current_fast_path_detail "
                    f"variant={int(variant_no)} "
                    f"source_template_cache_hit={int(bool(source_template_cache_hit))} "
                    f"source_template_lookup_ms={carry_phase_ms.get('source_template_lookup_ms', 0.0):.3f} "
                    f"source_template_build_ms={carry_phase_ms.get('source_template_build_ms', 0.0):.3f} "
                    f"source_future_template_scan_ms={carry_phase_ms.get('source_future_template_scan_ms', 0.0):.3f} "
                    f"source_template_carry_rows={int(len(source_template_carry_rows))} "
                    f"source_template_imp_scan_count={int(source_template_imp_scan_count)} "
                    f"source_template_mission_scan_count={int(source_template_mission_scan_count)} "
                    f"source_template_flight_path_load_count={int(source_template_flight_path_load_count)} "
                    f"source_template_missing_flight_path_count={int(source_template_missing_flight_path_count)} "
                    f"source_plan_load_ms={carry_phase_ms.get('source_plan_load_ms', 0.0):.3f} "
                    f"source_aircraft_index_ms={carry_phase_ms.get('source_aircraft_index_ms', 0.0):.3f} "
                    f"lah_template_resolve_ms={carry_phase_ms.get('lah_template_resolve_ms', 0.0):.3f} "
                    f"lah_hold_append_ms={carry_phase_ms.get('lah_hold_append_ms', 0.0):.3f} "
                    f"future_scan_copy_ms={carry_phase_ms.get('future_scan_copy_ms', 0.0):.3f} "
                    f"aircraft_list_merge_ms={carry_phase_ms.get('aircraft_list_merge_ms', 0.0):.3f} "
                    f"append_lists_ms={carry_phase_ms.get('append_lists_ms', 0.0):.3f} "
                    f"source_imp_scan_count={int(source_imp_scan_count)} "
                    f"source_mission_scan_count={int(source_mission_scan_count)} "
                    f"source_flight_path_load_count={int(source_flight_path_load_count)} "
                    f"source_flight_path_copy_count={int(source_flight_path_copy_count)} "
                    f"source_flight_path_shallow_clone_count={int(source_flight_path_shallow_clone_count)} "
                    f"source_flight_path_deepcopy_count={int(source_flight_path_deepcopy_count)} "
                    f"waypoint_block_used_0303={int(waypoint_block_used_0303)} "
                    f"waypoint_block_used_0304={int(waypoint_block_used_0304)} "
                    f"waypoint_fallback_count={int(waypoint_fallback_count)} "
                    f"waypoint_fallback_waypoints={int(waypoint_fallback_waypoints)} "
                    f"shallow_carry_clone={int(bool(shallow_carry_clone))} "
                    f"carriedMissions={int(len(carried_missions))} "
                    f"carried0303={int(len(carried_fps_0303))} "
                    f"carried0304={int(len(carried_fps_0304))} "
                    f"carriedWaypoints={int(copied_waypoints)}"
                )
                return {
                    "sourcePlanID": int(source_plan_id),
                    "currentInputMissionID": int(current_input_id),
                    "sourceTemplateInputMissionID": (
                        int(source_template_input_id)
                        if source_template_input_id is not None and source_template_input_id > 0
                        else None
                    ),
                    "futureInputMissionIDs": sorted(int(value) for value in future_input_ids),
                    "carriedInputMissionIDs": sorted(int(value) for value in carried_input_ids),
                    "carriedMissionCount": int(len(carried_missions)),
                    "carriedFlightPath0303Count": int(len(carried_fps_0303)),
                    "carriedFlightPath0304Count": int(len(carried_fps_0304)),
                    "carriedWaypointCount": int(copied_waypoints),
                    "missingFlightPathCount": int(missing_fp_count),
                    "sourceTemplateCacheHit": bool(source_template_cache_hit),
                    "sourceTemplateBuildMs": float(source_template_build_ms),
                    "sourceTemplateCarryRows": int(len(source_template_carry_rows)),
                    "hybridMissionCount": int(len(getattr(hybrid_result, "missions", []) or [])),
                    "hybridFlightPath0303Count": int(len(getattr(hybrid_result, "flight_plans_0303", []) or [])),
                    "hybridFlightPath0304Count": int(len(getattr(hybrid_result, "flight_plans_0304", []) or [])),
                    "lahHoldApplied": bool(lah_hold_summary.get("applied")),
                    "lahHoldAircraftIDs": [
                        int(value)
                        for value in (lah_hold_summary.get("aircraftIDs") or [])
                        if _safe_int_value(value) is not None
                    ],
                    "lahHoldMissionCount": int(lah_hold_summary.get("missionCount") or 0),
                    "lahHoldFlightPath0304Count": int(lah_hold_summary.get("flightPathCount") or 0),
                    "lahHoldPathIDs": [
                        int(value)
                        for value in (lah_hold_summary.get("pathIDs") or [])
                        if _safe_int_value(value) is not None
                    ],
                    "lahHoldReason": str(lah_hold_summary.get("reason") or ""),
                }

            def _build_reexecute_current_fast_path_variant_core(
                *,
                variant_no: int,
                option_code: int,
                requested_plan_id: Any,
                cmpk_source_path: Path,
                iter_out_root: Path,
                current_remaining_request: CurrentRemainingHybridRequest,
                shared_current_remaining_future: Any,
                shared_current_remaining_role: str,
                core_phase_ms: Dict[str, float],
                variant_generated_path_ids: Set[int],
                log_emit: Callable[[str], None],
                waypoint_block_0303_start: Any = None,
                waypoint_block_0303_end: Any = None,
                waypoint_block_0304_start: Any = None,
                waypoint_block_0304_end: Any = None,
            ) -> Optional[Dict[str, Any]]:
                if current_remaining_request is None:
                    return None
                if str(getattr(current_remaining_request, "planner_mode", "") or "") != "reexecute_first_mission":
                    return None
                if not _runtime_env_flag(
                    "replan_reexecute_current_fast_path_enabled",
                    "REPLAN_REEXECUTE_CURRENT_FAST_PATH",
                    True,
                ):
                    return None

                current_remaining_hybrid_result = None
                try:
                    if isinstance(shared_current_remaining_future, concurrent.futures.Future):
                        join_started = time.perf_counter()
                        current_remaining_hybrid_result = shared_current_remaining_future.result()
                        join_ms = (time.perf_counter() - join_started) * 1000.0
                        core_phase_ms["current_hybrid_wait_ms"] = float(join_ms)
                        core_phase_ms["current_hybrid_build_ms"] = 0.0
                        log_emit(
                            "[REPLAN][METRIC] current_remaining_hybrid_shared_join "
                            f"variant={int(variant_no)} option={int(option_code)} "
                            f"role={shared_current_remaining_role or 'shared'} "
                            f"join_wait_ms={join_ms:.3f} "
                            f"result_available={int(current_remaining_hybrid_result is not None)}"
                        )
                    else:
                        current_remaining_hybrid_result = _build_current_remaining_hybrid_locked(
                            current_remaining_request,
                            variant_no=variant_no,
                            log_emit=log_emit,
                            timing_sink=core_phase_ms,
                        )
                except Exception as exc:
                    log_emit(
                        f"[WARN] [variant {variant_no}] reexecute current fast-path hybrid build failed: {exc}"
                    )
                    return None
                if current_remaining_hybrid_result is None:
                    log_emit(f"[WARN] [variant {variant_no}] reexecute current fast-path hybrid unavailable")
                    return None

                mp_json: Dict[str, Any] = {
                    "missionPlanID": 0,
                    "timestamp": _now_ms_since_2000(),
                    "missionPlanTimestamp": _now_ms_since_2000(),
                    "planningTime": 0.0,
                    "plannerID": 1,
                    "inputMissionPackageID": cmpk_id,
                    "missionReferencePackageID": int(Path(mrpk_path).stem) if mrpk_path else 0,
                    "aircraftList": [],
                    "Source": "MMR",
                }
                missions: list[dict] = []
                flight_plans_0303: list[dict] = []
                flight_plans_0304: list[dict] = []
                step_t0 = time.perf_counter()
                try:
                    missions, flight_plans_0303, flight_plans_0304, hybrid_path_ids = _apply_current_remaining_hybrid_to_variant(
                        variant_no=variant_no,
                        missions=missions,
                        flight_plans_0303=flight_plans_0303,
                        flight_plans_0304=flight_plans_0304,
                        request=current_remaining_request,
                        hybrid_result=current_remaining_hybrid_result,
                        log_emit=log_emit,
                    )
                    core_phase_ms["hybrid_merge_ms"] = (time.perf_counter() - step_t0) * 1000.0
                    variant_generated_path_ids.update(int(pid) for pid in hybrid_path_ids if pid is not None)

                    step_t0 = time.perf_counter()
                    carry_summary = _append_reexecute_current_fast_path_future_artifacts(
                        variant_no=variant_no,
                        request=current_remaining_request,
                        hybrid_result=current_remaining_hybrid_result,
                        mp_json=mp_json,
                        missions=missions,
                        flight_plans_0303=flight_plans_0303,
                        flight_plans_0304=flight_plans_0304,
                        log_emit=log_emit,
                        waypoint_block_0303_start=waypoint_block_0303_start,
                        waypoint_block_0303_end=waypoint_block_0303_end,
                        waypoint_block_0304_start=waypoint_block_0304_start,
                        waypoint_block_0304_end=waypoint_block_0304_end,
                    )
                except Exception as exc:
                    log_emit(
                        f"[WARN] [variant {variant_no}] reexecute current fast-path fallback to full generation: {exc}"
                    )
                    return None
                carry_ms = (time.perf_counter() - step_t0) * 1000.0
                core_phase_ms["collect_missions_ms"] = float(carry_ms)
                log_emit(
                    "[REPLAN][METRIC] reexecute_current_fast_path "
                    f"variant={int(variant_no)} option={int(option_code)} "
                    f"sourcePlan={carry_summary['sourcePlanID']} "
                    f"currentInputMissionID={carry_summary['currentInputMissionID']} "
                    f"sourceTemplateInputMissionID={carry_summary.get('sourceTemplateInputMissionID') or 0} "
                    f"futureInputs={'|'.join(str(v) for v in carry_summary['futureInputMissionIDs']) or '-'} "
                    f"carriedInputs={'|'.join(str(v) for v in carry_summary['carriedInputMissionIDs']) or '-'} "
                    f"carriedMissions={carry_summary['carriedMissionCount']} "
                    f"carried0303={carry_summary['carriedFlightPath0303Count']} "
                    f"carried0304={carry_summary['carriedFlightPath0304Count']} "
                    f"carriedWaypoints={carry_summary['carriedWaypointCount']} "
                    f"sourceTemplateCacheHit={int(bool(carry_summary.get('sourceTemplateCacheHit')))} "
                    f"sourceTemplateCarryRows={int(carry_summary.get('sourceTemplateCarryRows') or 0)} "
                    f"lahHoldApplied={int(bool(carry_summary.get('lahHoldApplied')))} "
                    f"lahHoldAircraft={'|'.join(str(v) for v in (carry_summary.get('lahHoldAircraftIDs') or [])) or '-'} "
                    f"lahHoldPaths={'|'.join(str(v) for v in (carry_summary.get('lahHoldPathIDs') or [])) or '-'} "
                    f"missingFlightPaths={carry_summary['missingFlightPathCount']} "
                    f"carry_ms={carry_ms:.3f}"
                )
                log_emit(
                    f"[INFO] [variant {variant_no}] 현재임무 재수행 fast-path 적용: "
                    f"current hybrid만 생성, 미래 FlightPath는 source plan에서 carry-forward "
                    f"(missions={len(missions)}, fp0303={len(flight_plans_0303)}, fp0304={len(flight_plans_0304)})"
                )
                return {
                    "variant_no": variant_no,
                    "requested_plan_id": requested_plan_id,
                    "option_code": option_code,
                    "cmpk_source_path": cmpk_source_path,
                    "iter_out_root": iter_out_root,
                    "mp_json": mp_json,
                    "imp_id_map": {},
                    "missions": missions,
                    "flight_plans_0303": flight_plans_0303,
                    "flight_plans_0304": flight_plans_0304,
                    "generated_path_ids": variant_generated_path_ids,
                    "option_dependent_isolation": _option_dependent_isolation_contract(
                        variant_no=variant_no,
                        option_code=option_code,
                        mode="parallel_core_fast_path",
                    ),
                    "current_remaining_hybrid_active": True,
                    "current_remaining_hybrid_applied": True,
                    "remaining_hybrid_result": None,
                    "reexecute_current_fast_path": carry_summary,
                }

            def _apply_remaining_hybrid_customization(
                *,
                variant_no: int,
                cmpk_source_path: Path,
                missions: List[Dict[str, Any]],
                flight_plans_0303: List[Dict[str, Any]],
                snapshot_mutated: bool,
            ):
                hybrid_started = time.perf_counter()
                if not snapshot_mutated:
                    self.log_sig.emit(
                        f"[REMAINING][variant {variant_no}] hybrid customization skipped: "
                        "snapshot_not_mutated (elapsed=0.0 ms)"
                    )
                    return None
                active_detail = None
                for container in (ctx, staged):
                    detail_payload = container.get("replan_detail") if isinstance(container, dict) else None
                    if isinstance(detail_payload, dict) and detail_payload:
                        active_detail = detail_payload
                        break
                try:
                    from .replanning.triggers.remaining_hybrid.general import (
                        apply_remaining_hybrid_replan,
                    )
                except Exception as exc:
                    elapsed_ms = (time.perf_counter() - hybrid_started) * 1000.0
                    self.log_sig.emit(
                        f"[REMAINING][variant {variant_no}] hybrid helper import skipped: {exc} "
                        f"(elapsed={elapsed_ms:.1f} ms)"
                    )
                    return None
                try:
                    result = apply_remaining_hybrid_replan(
                        cmpk_source_path=Path(cmpk_source_path),
                        replan_detail=active_detail if isinstance(active_detail, dict) else None,
                        missions=missions,
                        flight_plans_0303=flight_plans_0303,
                        timestamp_ms=int(time.time() * 1000),
                        log=lambda msg, n=variant_no: self.log_sig.emit(f"[variant {n}] {msg}"),
                    )
                except Exception as exc:
                    elapsed_ms = (time.perf_counter() - hybrid_started) * 1000.0
                    self.log_sig.emit(
                        f"[REMAINING][variant {variant_no}] hybrid customization failed: {exc} "
                        f"(elapsed={elapsed_ms:.1f} ms)"
                    )
                    return None
                elapsed_ms = (time.perf_counter() - hybrid_started) * 1000.0
                path_count = len([fp for fp in (flight_plans_0303 or []) if isinstance(fp, dict)])
                if getattr(result, "applied", False):
                    mode = str(getattr(result, "mode", "") or "-")
                    input_mid = getattr(result, "input_mission_id", None)
                    aircraft_ids = list(getattr(result, "aircraft_ids", []) or [])
                    workflow = str(getattr(result, "planner_workflow", "") or "")
                    self.log_sig.emit(
                        "[REMAINING] hybrid customization applied "
                        f"(variant={variant_no}, mode={mode}, inputMissionID={input_mid}, "
                        f"aircraft={aircraft_ids}, paths0303={path_count}, "
                        f"workflow={workflow or '-'}, elapsed={elapsed_ms:.1f} ms)"
                    )
                else:
                    reason_text = str(getattr(result, "reason", "") or "").strip()
                    if reason_text:
                        self.log_sig.emit(
                            f"[REMAINING][variant {variant_no}] hybrid customization skipped: {reason_text} "
                            f"(paths0303={path_count}, elapsed={elapsed_ms:.1f} ms)"
                        )
                return result

            def _carry_forward_mission_area_snapshot(
                *,
                variant_no: int,
                plan_id: int,
                reason: str,
            ) -> Dict[str, Any]:
                source_id = _positive_int_or_none(snapshot_source_plan_id)
                target_id = _positive_int_or_none(plan_id)
                summary = {
                    "sourceMissionPlanID": source_id,
                    "targetMissionPlanID": target_id,
                    "carried": False,
                    "path": None,
                    "reason": str(reason or ""),
                }
                if source_id is None or target_id is None:
                    return summary
                try:
                    carried_path = mission_area_replan_store.carry_forward_snapshot(
                        int(source_id),
                        int(target_id),
                        reason=str(reason or ""),
                    )
                except Exception as exc:
                    summary["error"] = str(exc)
                    self.log_sig.emit(
                        f"[WARN] mission area snapshot carry-forward failed "
                        f"(variant={variant_no}, sourcePlan={source_id}, targetPlan={target_id}): {exc}"
                    )
                    return summary
                if carried_path is not None:
                    summary["carried"] = True
                    summary["path"] = str(carried_path)
                    self.log_sig.emit(
                        f"[INFO] mission area snapshot carried forward "
                        f"(variant={variant_no}, sourcePlan={source_id}, targetPlan={target_id})"
                    )
                else:
                    self.log_sig.emit(
                        f"[INFO] mission area snapshot carry-forward skipped "
                        f"(variant={variant_no}, sourcePlan={source_id}, targetPlan={target_id})"
                    )
                return summary

            def _run_general_variant_core(spec: Dict[str, Any]) -> Dict[str, Any]:
                variant_no = int(spec["variant_no"])
                option_code = int(spec["option_code"])
                requested_plan_id = spec.get("requested_plan_id")
                cmpk_source_path = Path(spec["cmpk_source_path"])
                runtime_payload = spec.get("runtime_payload")
                current_remaining_request = spec.get("current_remaining_hybrid_request")
                shared_current_remaining_future = spec.get("shared_current_remaining_hybrid_future")
                shared_current_remaining_role = str(spec.get("shared_current_remaining_hybrid_role") or "")
                variant_logs: list[str] = []
                variant_timing_events: list[Dict[str, Any]] = []

                def _variant_log(message: Any) -> None:
                    variant_logs.append(str(message))

                def _variant_timing(event: str, extra: Dict[str, Any]) -> None:
                    variant_timing_events.append(
                        {
                            "event": str(event),
                            "perf": time.perf_counter(),
                            "wall_ms": int(time.time() * 1000),
                            "extra": dict(extra or {}),
                        }
                    )

                if not isinstance(current_remaining_request, CurrentRemainingHybridRequest):
                    current_remaining_request = None
                waypoint_block_0303_start = spec.get("waypoint_block_0303_start", spec.get("waypoint_block_start"))
                waypoint_block_0303_end = spec.get("waypoint_block_0303_end", spec.get("waypoint_block_end"))
                waypoint_block_0304_start = spec.get("waypoint_block_0304_start")
                waypoint_block_0304_end = spec.get("waypoint_block_0304_end")
                reserved_waypoint_block_0303_start = waypoint_block_0303_start
                reserved_waypoint_block_0303_end = waypoint_block_0303_end
                reserved_waypoint_block_0304_start = waypoint_block_0304_start
                reserved_waypoint_block_0304_end = waypoint_block_0304_end
                core_phase_ms: Dict[str, float] = {
                    "divide_and_pattern_ms": 0.0,
                    "build_0301_load_ms": 0.0,
                    "collect_missions_ms": 0.0,
                    "current_hybrid_wait_ms": 0.0,
                    "current_hybrid_build_ms": 0.0,
                    "flightpath_0303_ms": 0.0,
                    "flightpath_0304_ms": 0.0,
                    "hybrid_merge_ms": 0.0,
                    "eta_follow_ms": 0.0,
                }
                with runtime_settings_override(runtime_payload):
                    iter_out_root = out_root_base / f"variant_{variant_no:02d}"
                    if iter_out_root.exists():
                        shutil.rmtree(iter_out_root)
                    iter_out_root.mkdir(parents=True, exist_ok=True)

                    variant_start = time.perf_counter()
                    _variant_timing(
                        "variant_started",
                        {
                            "variant": variant_no,
                            "option": option_code,
                            "mode": "parallel_core",
                        },
                    )
                    variant_generated_path_ids: Set[int] = set()
                    fast_path_result = _build_reexecute_current_fast_path_variant_core(
                        variant_no=variant_no,
                        option_code=option_code,
                        requested_plan_id=requested_plan_id,
                        cmpk_source_path=cmpk_source_path,
                        iter_out_root=iter_out_root,
                        current_remaining_request=current_remaining_request,
                        shared_current_remaining_future=shared_current_remaining_future,
                        shared_current_remaining_role=shared_current_remaining_role,
                        core_phase_ms=core_phase_ms,
                        variant_generated_path_ids=variant_generated_path_ids,
                        log_emit=_variant_log,
                        waypoint_block_0303_start=waypoint_block_0303_start,
                        waypoint_block_0303_end=waypoint_block_0303_end,
                        waypoint_block_0304_start=waypoint_block_0304_start,
                        waypoint_block_0304_end=waypoint_block_0304_end,
                    )
                    if isinstance(fast_path_result, dict):
                        flight_plans_0303 = list(fast_path_result.get("flight_plans_0303") or [])
                        flight_plans_0304 = list(fast_path_result.get("flight_plans_0304") or [])
                        if flight_plans_0303 and flight_plans_0304:
                            step_t0 = time.perf_counter()
                            try:
                                fast_path_result["flight_plans_0304"] = d0304.apply_uav_eta_follow_speed_plan(
                                    list(flight_plans_0304),
                                    list(flight_plans_0303),
                                    lah_missions=list(fast_path_result.get("missions") or []),
                                )
                                _variant_log(
                                    f"[INFO] Applied LAH-UAV ETA follow speed plan (variant={variant_no}, fast-path)"
                                )
                            except Exception as exc:
                                _variant_log(
                                    f"[WARN] Failed to apply LAH-UAV ETA follow speed plan "
                                    f"(variant={variant_no}, fast-path): {exc}"
                                )
                            finally:
                                core_phase_ms["eta_follow_ms"] = (time.perf_counter() - step_t0) * 1000.0
                        if not fast_path_result.get("flight_plans_0303") and not fast_path_result.get("flight_plans_0304"):
                            _variant_log(f"[ERR] FlightPath generation failed (variant={variant_no}, fast-path)")
                            raise _VariantCoreError(
                                "flightpath_generation_failed",
                                f"FlightPath generation failed (variant={variant_no}, fast-path)",
                                variant_no=variant_no,
                            )
                        _variant_log(
                            f"[OK] FlightPath counts (variant={variant_no}, fast-path): "
                            f"0303={len(fast_path_result.get('flight_plans_0303') or [])} / "
                            f"0304={len(fast_path_result.get('flight_plans_0304') or [])}"
                        )
                        _emit_flightpath_metric(
                            _variant_log,
                            variant_no=variant_no,
                            option_code=option_code,
                            mode="parallel_core_fast_path",
                            flight_plans_0303=fast_path_result.get("flight_plans_0303") or [],
                            flight_plans_0304=fast_path_result.get("flight_plans_0304") or [],
                        )
                        core_total_ms = (time.perf_counter() - variant_start) * 1000.0
                        if isinstance(fast_path_result.get("mp_json"), dict):
                            fast_path_result["mp_json"]["planningTime"] = float(core_total_ms)
                        _variant_log(
                            "[REPLAN][METRIC] general_variant_core_phase "
                            f"variant={int(variant_no)} option={int(option_code)} mode=parallel_core_fast_path "
                            f"divide_and_pattern_ms={core_phase_ms['divide_and_pattern_ms']:.3f} "
                            f"build_0301_load_ms={core_phase_ms['build_0301_load_ms']:.3f} "
                            f"collect_missions_ms={core_phase_ms['collect_missions_ms']:.3f} "
                            f"current_hybrid_wait_ms={core_phase_ms['current_hybrid_wait_ms']:.3f} "
                            f"current_hybrid_build_ms={core_phase_ms['current_hybrid_build_ms']:.3f} "
                            f"flightpath_0303_ms={core_phase_ms['flightpath_0303_ms']:.3f} "
                            f"flightpath_0304_ms={core_phase_ms['flightpath_0304_ms']:.3f} "
                            f"hybrid_merge_ms={core_phase_ms['hybrid_merge_ms']:.3f} "
                            f"eta_follow_ms={core_phase_ms['eta_follow_ms']:.3f} "
                            f"core_total_ms={float(core_total_ms):.3f}"
                        )
                        _variant_timing(
                            "variant_core_finished",
                            {
                                "variant": variant_no,
                                "option": option_code,
                                "mode": "parallel_core_fast_path",
                                "status": "success",
                                "duration_ms": round(core_total_ms, 3),
                            },
                        )
                        fast_path_result.update(
                            {
                                "core_total_ms": core_total_ms,
                                "core_phase_ms": dict(core_phase_ms),
                                "log_messages": variant_logs,
                                "timing_events": variant_timing_events,
                            }
                        )
                        return fast_path_result

                    step_t0 = time.perf_counter()
                    _variant_log(f"[STEP 1.{variant_no}] Divide & Pattern start")
                    imp_paths = run_divide_and_pattern(
                        str(cmpk_source_path),
                        str(mrpk_path),
                        str(iter_out_root),
                        log=lambda msg, n=variant_no: _variant_log(f"[variant {n}] {msg}"),
                        option_code=option_code,
                        trust_input_aircraft=_trust_input_aircraft_for_replan(),
                        shared_split_state=(
                            input_refresh_shared_split_state
                            if current_remaining_request is None
                            else None
                        ),
                    )
                    if not imp_paths:
                        _variant_log(f"[ERR] IMP generation failed (variant={variant_no})")
                        raise _VariantCoreError(
                            "imp_generation_failed",
                            f"IMP generation failed (variant={variant_no})",
                            variant_no=variant_no,
                        )
                    step_ms = (time.perf_counter() - step_t0) * 1000.0
                    core_phase_ms["divide_and_pattern_ms"] = float(step_ms)
                    _variant_log(f"[TIME] divide_and_pattern (variant={variant_no}): {step_ms:.1f} ms")
                    _variant_log(f"[OK] IMP generated: {len(imp_paths)} file(s) (variant={variant_no})")

                    step_t0 = time.perf_counter()
                    mp_tmp = iter_out_root / f"MissionPlan_{int(time.time()*1000)}.json"
                    mp_json = build_mission_plan_0301(
                        str(cmpk_source_path),
                        str(mrpk_path),
                        imp_paths,
                        str(mp_tmp),
                        mission_plan_id=0,
                    )
                    if not isinstance(mp_json, dict):
                        with mp_tmp.open(encoding="utf-8") as f:
                            mp_json = json.load(f)
                    imp_id_map = {
                        a.get("aircraftID"): a.get("individualMissionPackageID")
                        for a in mp_json.get("aircraftList", [])
                    }
                    step_ms = (time.perf_counter() - step_t0) * 1000.0
                    core_phase_ms["build_0301_load_ms"] = float(step_ms)
                    _variant_log(f"[TIME] build_0301+load (variant={variant_no}): {step_ms:.1f} ms")
                    _variant_log(f"[OK] MissionPlan built: {mp_tmp.name} (variant={variant_no})")

                    step_t0 = time.perf_counter()
                    missions = []
                    loaded_imp_packages = _load_imp_packages(list(imp_paths))
                    for aid, pkg in loaded_imp_packages:
                        for im in pkg.get("individualMissionList", []):
                            im_copy = dict(im)
                            im_copy["aircraftID"] = aid
                            if "individualMissionPlanPackageID" not in im_copy and imp_id_map:
                                im_copy["individualMissionPlanPackageID"] = imp_id_map.get(aid)
                            missions.append(im_copy)
                    handover_marked = _mark_handover_terminal_missions_from_path(
                        missions,
                        cmpk_source_path,
                    )
                    if handover_marked:
                        _variant_log(
                            f"[INFO] UAV control-transfer direct transit enabled for {handover_marked} mission row(s) "
                            f"(variant={variant_no})"
                        )
                    step_ms = (time.perf_counter() - step_t0) * 1000.0
                    core_phase_ms["collect_missions_ms"] = float(step_ms)
                    _variant_log(
                        f"[TIME] collect_missions (variant={variant_no}): {step_ms:.1f} ms, count={len(missions)}"
                    )

                    manned = [im for im in missions if int(im.get("aircraftID", 0)) in (1, 2, 3)]
                    unmanned = [im for im in missions if int(im.get("aircraftID", 0)) in (4, 5, 6)]
                    generic_unmanned = list(unmanned)
                    carried_flight_plans_0303: list[dict] = []
                    carried_flight_plans_0304: list[dict] = []
                    input_refresh_immutable_0304_path_ids: Set[int] = set()
                    input_refresh_carry_summary: Dict[str, Any] | None = None
                    input_refresh_source_plan_id: int | None = None
                    input_refresh_variant_template_future: concurrent.futures.Future | None = None
                    input_refresh_variant_template_owner = False
                    input_refresh_variant_template_reused = False
                    input_refresh_variant_template_wait_ms = 0.0
                    current_remaining_hybrid_result = None
                    if current_remaining_request is not None:
                        try:
                            if isinstance(shared_current_remaining_future, concurrent.futures.Future):
                                join_started = time.perf_counter()
                                current_remaining_hybrid_result = shared_current_remaining_future.result()
                                join_ms = (time.perf_counter() - join_started) * 1000.0
                                core_phase_ms["current_hybrid_wait_ms"] = float(join_ms)
                                core_phase_ms["current_hybrid_build_ms"] = 0.0
                                _variant_log(
                                    "[REPLAN][METRIC] current_remaining_hybrid_shared_join "
                                    f"variant={int(variant_no)} option={int(option_code)} "
                                    f"role={shared_current_remaining_role or 'shared'} "
                                    f"join_wait_ms={join_ms:.3f} "
                                    f"result_available={int(current_remaining_hybrid_result is not None)}"
                                )
                            else:
                                current_remaining_hybrid_result = _build_current_remaining_hybrid_locked(
                                    current_remaining_request,
                                    variant_no=variant_no,
                                    log_emit=_variant_log,
                                    timing_sink=core_phase_ms,
                                )
                        except Exception as exc:
                            _variant_log(
                                f"[WARN] [variant {variant_no}] current remaining collaborative hybrid failed before generic skip: {exc}"
                            )
                            if isinstance(shared_current_remaining_future, concurrent.futures.Future):
                                try:
                                    _variant_log(
                                        f"[variant {variant_no}] shared current remaining hybrid fallback -> per-variant build"
                                    )
                                    current_remaining_hybrid_result = _build_current_remaining_hybrid_locked(
                                        current_remaining_request,
                                        variant_no=variant_no,
                                        log_emit=_variant_log,
                                        timing_sink=core_phase_ms,
                                    )
                                except Exception as fallback_exc:
                                    _variant_log(
                                        f"[WARN] [variant {variant_no}] current remaining fallback build failed: {fallback_exc}"
                                    )
                                    current_remaining_hybrid_result = None
                            else:
                                current_remaining_hybrid_result = None
                        if current_remaining_hybrid_result is not None:
                            skip_result = filter_generic_flightpath_missions_for_hybrid(
                                generic_unmanned,
                                request=current_remaining_request,
                                hybrid=current_remaining_hybrid_result,
                            )
                            generic_unmanned = list(skip_result.missions)
                            skip_policy = getattr(skip_result, "skip_policy", {}) or {}
                            if (
                                str(getattr(current_remaining_request, "planner_mode", "") or "")
                                == "reexecute_first_mission"
                            ):
                                _variant_log(
                                    f"[REEXEC-FIRST] generic 0303 skip result: "
                                    f"policy={skip_policy}, skipped={int(skip_result.skipped_count)}, "
                                    f"aircraft={sorted(skip_result.skipped_aircraft_ids)}, "
                                    f"pathIDs={sorted(skip_result.skipped_path_ids)}"
                                )
                            if int(skip_result.skipped_count) > 0:
                                _variant_log(
                                    f"[variant {variant_no}] current remaining generic 0303 skipped: "
                                    f"inputMissionID={current_remaining_hybrid_result.current_input_id}, "
                                    f"missions={int(skip_result.skipped_count)}, "
                                    f"aircraft={sorted(skip_result.skipped_aircraft_ids)}, "
                                    f"pathIDs={sorted(skip_result.skipped_path_ids)}"
                                )

                    if current_remaining_request is None and input_refresh_fast_dem_context:
                        input_refresh_source_plan_id = _input_refresh_source_plan_for_option(
                            option_code=int(option_code),
                            fallback_source_plan_id=snapshot_source_plan_id,
                        )
                        if _input_refresh_variant_fp_template_enabled():
                            template_key = _input_refresh_variant_fp_template_key(
                                missions,
                                source_plan_id=input_refresh_source_plan_id,
                            )
                            with input_refresh_variant_fp_template_lock:
                                input_refresh_variant_template_future = input_refresh_variant_fp_template_cache.get(
                                    template_key
                                )
                                if input_refresh_variant_template_future is None:
                                    input_refresh_variant_template_future = concurrent.futures.Future()
                                    input_refresh_variant_fp_template_cache[template_key] = (
                                        input_refresh_variant_template_future
                                    )
                                    input_refresh_variant_template_owner = True
                            if not input_refresh_variant_template_owner:
                                wait_started = time.perf_counter()
                                try:
                                    template_payload = input_refresh_variant_template_future.result(timeout=15.0)
                                    (
                                        carried_flight_plans_0303,
                                        carried_flight_plans_0304,
                                        template_waypoint_counts,
                                    ) = _clone_input_refresh_variant_fp_template(
                                        template_payload,
                                        missions=missions,
                                        waypoint_block_0303_start=reserved_waypoint_block_0303_start,
                                        waypoint_block_0303_end=reserved_waypoint_block_0303_end,
                                        waypoint_block_0304_start=reserved_waypoint_block_0304_start,
                                        waypoint_block_0304_end=reserved_waypoint_block_0304_end,
                                    )
                                    generic_unmanned = []
                                    manned = []
                                    input_refresh_immutable_0304_path_ids = {
                                        int(value)
                                        for value in (
                                            template_waypoint_counts.get("immutable0304PathIDs") or []
                                        )
                                        if _safe_int_value(value) is not None
                                    }
                                    input_refresh_variant_template_reused = True
                                    _variant_log(
                                        "[REPLAN][METRIC] input_refresh_variant_fp_template "
                                        f"variant={int(variant_no)} option={int(option_code)} role=follower "
                                        f"sourceVariant={int(template_payload.get('variant_no') or 0)} "
                                        f"paths0303={len(carried_flight_plans_0303)} "
                                        f"paths0304={len(carried_flight_plans_0304)} "
                                        f"waypoints0303={int(template_waypoint_counts.get('waypoints0303') or 0)} "
                                        f"waypoints0304={int(template_waypoint_counts.get('waypoints0304') or 0)}"
                                    )
                                except Exception as exc:
                                    _variant_log(
                                        f"[WARN] inputRefresh variant FlightPath template unavailable "
                                        f"(variant={variant_no}) -> independent build: {exc}"
                                    )
                                finally:
                                    input_refresh_variant_template_wait_ms = (
                                        time.perf_counter() - wait_started
                                    ) * 1000.0

                    if (
                        current_remaining_request is None
                        and input_refresh_fast_dem_context
                        and not input_refresh_variant_template_reused
                    ):
                        input_refresh_carry_summary = _apply_input_refresh_flightpath_carry_forward(
                            variant_no=variant_no,
                            option_code=option_code,
                            source_plan_id=input_refresh_source_plan_id,
                            manned_missions=manned,
                            unmanned_missions=generic_unmanned,
                            waypoint_block_0303_start=waypoint_block_0303_start,
                            waypoint_block_0303_end=waypoint_block_0303_end,
                            waypoint_block_0304_start=waypoint_block_0304_start,
                            waypoint_block_0304_end=waypoint_block_0304_end,
                            log_emit=_variant_log,
                        )
                        if bool(input_refresh_carry_summary.get("applied")):
                            carried_flight_plans_0303 = list(
                                input_refresh_carry_summary.get("flight_plans_0303") or []
                            )
                            carried_flight_plans_0304 = list(
                                input_refresh_carry_summary.get("flight_plans_0304") or []
                            )
                            input_refresh_immutable_0304_path_ids = {
                                int(path_id)
                                for path_id in (
                                    _safe_int_value(fp.get("pathID"))
                                    for fp in carried_flight_plans_0304
                                    if isinstance(fp, dict)
                                )
                                if path_id is not None and int(path_id) > 0
                            }
                            generic_unmanned = list(
                                input_refresh_carry_summary.get("unmanned_to_build") or []
                            )
                            manned = list(input_refresh_carry_summary.get("manned_to_build") or [])
                            waypoint_block_0303_start = input_refresh_carry_summary.get(
                                "waypoint_block_0303_next",
                                waypoint_block_0303_start,
                            )
                            waypoint_block_0304_start = input_refresh_carry_summary.get(
                                "waypoint_block_0304_next",
                                waypoint_block_0304_start,
                            )
                            _variant_log(
                                f"[INFO] inputRefresh FlightPath carry-forward applied "
                                f"(variant={variant_no}, carried0303={len(carried_flight_plans_0303)}, "
                                f"carried0304={len(carried_flight_plans_0304)}, "
                                f"build0303={len(generic_unmanned)}, build0304={len(manned)})"
                            )
                        elif str(input_refresh_carry_summary.get("reason") or "") in {
                            "source_plan_unavailable",
                            "no_current_missions",
                            "source_template_failed",
                        }:
                            _variant_log(
                                "[REPLAN][METRIC] input_refresh_flightpath_carry_forward "
                                f"variant={int(variant_no)} option={int(option_code)} "
                                f"sourcePlan={int(input_refresh_carry_summary.get('sourcePlanID') or 0)} "
                                "applied=0 "
                                f"reason={input_refresh_carry_summary.get('reason') or 'skipped'} "
                                f"build0303={len(generic_unmanned)} build0304={len(manned)}"
                            )

                    def _make_wp_allocator(start_value, end_value):
                        if start_value is None:
                            return d0303._WPAllocator()
                        return d0303._WPAllocator(
                            start=int(start_value),
                            end=int(end_value) if end_value is not None else None,
                            overflow_block_size=int(waypoint_block_size),
                        )

                    wp_alloc_0303 = _make_wp_allocator(waypoint_block_0303_start, waypoint_block_0303_end)
                    wp_alloc_0304 = _make_wp_allocator(waypoint_block_0304_start, waypoint_block_0304_end)
                    try:
                        manned_plan_mode = str(
                            ((load_runtime_settings().get("values") or {}).get("manned_plan_mode")) or "normal"
                        ).strip().lower()
                    except Exception:
                        manned_plan_mode = "normal"

                    def _build_0303(unmanned_missions: list[dict]) -> Dict[str, Any]:
                        return build_0303_flight_plans_aircraft_parallel(
                            d0303,
                            unmanned_missions,
                            runtime_payload=runtime_payload,
                            wp_alloc=wp_alloc_0303,
                            cruise_speed=float(uav_cruise_speed),
                            turn_step_deg=float(uav_turn_step),
                            ref0203=mrpk_data,
                        )

                    initial_hold_by_aircraft = _type1_initial_lah_hold_by_aircraft(
                        cmpk_source=cmpk_source_path,
                        current_request=current_remaining_request,
                        log_emit=_variant_log,
                    )

                    def _build_0304():
                        start = time.perf_counter()
                        try:
                            _rv = (runtime_payload.get("values") or {}) if isinstance(runtime_payload, dict) else {}
                        except Exception:
                            _rv = {}
                        plans = d0304.build_lah_flight_plans_fixed(
                            manned,
                            cruise_speed=40.0,
                            manned_plan_mode=manned_plan_mode,
                            lah_path_mode=str(_rv.get("lah_path_mode", "linear")),
                            lah_rl_hex_step=int(_rv.get("lah_rl_hex_step", 50)),
                            lah_rl_area_km=float(_rv.get("lah_rl_area_km", 10.0)),
                            wp_alloc=wp_alloc_0304,
                            initial_hold_by_aircraft=initial_hold_by_aircraft,
                        )
                        return plans, (time.perf_counter() - start) * 1000.0, _get_lah_mission_plan_timings()

                    flight_plans_0303: list[dict] = list(carried_flight_plans_0303)
                    flight_plans_0304: list[dict] = list(carried_flight_plans_0304)
                    elapsed_0303_ms = 0.0
                    elapsed_0304_ms = 0.0
                    build_result_0303: Dict[str, Any] | None = None
                    mission_timings_0304: list[dict] = []
                    flightpath_builds_concurrent = False
                    if generic_unmanned and manned:
                        flightpath_builds_concurrent = True
                        with concurrent.futures.ThreadPoolExecutor(
                            max_workers=2,
                            thread_name_prefix="Build0303_0304",
                        ) as fp_executor:
                            future_0303 = fp_executor.submit(_build_0303, generic_unmanned)
                            future_0304 = fp_executor.submit(_build_0304)
                            build_result_0303 = future_0303.result()
                            flight_plans_0304, elapsed_0304_ms, mission_timings_0304 = future_0304.result()
                        flight_plans_0303.extend(list(build_result_0303.get("plans") or []))
                        flight_plans_0304 = list(carried_flight_plans_0304) + list(flight_plans_0304 or [])
                        elapsed_0303_ms = float(build_result_0303.get("elapsed_ms") or 0.0)
                        core_phase_ms["flightpath_0303_ms"] = float(elapsed_0303_ms)
                        core_phase_ms["flightpath_0304_ms"] = float(elapsed_0304_ms)
                    elif generic_unmanned:
                        build_result_0303 = _build_0303(generic_unmanned)
                        flight_plans_0303.extend(list(build_result_0303.get("plans") or []))
                        elapsed_0303_ms = float(build_result_0303.get("elapsed_ms") or 0.0)
                        core_phase_ms["flightpath_0303_ms"] = float(elapsed_0303_ms)
                    if manned and not flightpath_builds_concurrent:
                        built_0304, elapsed_0304_ms, mission_timings_0304 = _build_0304()
                        flight_plans_0304.extend(list(built_0304 or []))
                        core_phase_ms["flightpath_0304_ms"] = float(elapsed_0304_ms)
                    if generic_unmanned or manned:
                        mode_parts = []
                        if build_result_0303 is not None:
                            mode_parts.append(
                                "0303="
                                f"{str(build_result_0303.get('mode') or 'sequential')} "
                                f"workers={int(build_result_0303.get('workers') or 1)} "
                                f"aircraft={int(build_result_0303.get('aircraft') or 0)}"
                            )
                        elif unmanned and current_remaining_hybrid_result is not None:
                            mode_parts.append("0303=skipped_by_current_remaining_hybrid")
                        if manned:
                            mode_parts.append("0304=sequential_concurrent" if flightpath_builds_concurrent else "0304=sequential")
                        _variant_log(
                            f"[INFO] FlightPath build mode (variant={variant_no}): "
                            + ", ".join(mode_parts)
                        )
                        if build_result_0303 is not None:
                            _emit_0303_build_metric(
                                _variant_log,
                                variant_no=variant_no,
                                option_code=option_code,
                                mode="parallel_core",
                                build_result=build_result_0303,
                            )
                        parts = []
                        if generic_unmanned:
                            parts.append(f"0303={elapsed_0303_ms:.1f} ms")
                        elif unmanned and current_remaining_hybrid_result is not None:
                            parts.append("0303=skipped_by_current_remaining_hybrid")
                        if manned:
                            parts.append(f"0304={elapsed_0304_ms:.1f} ms")
                        _variant_log("[INFO] FlightPath build time: " + ", ".join(parts))
                        _emit_mission_plan_timing_metrics(
                            _variant_log,
                            variant_no=variant_no,
                            option_code=option_code,
                            mode="parallel_core",
                            build_result_0303=build_result_0303,
                            mission_timings_0304=mission_timings_0304,
                            elapsed_0303_ms=elapsed_0303_ms,
                            elapsed_0304_ms=elapsed_0304_ms,
                            flightpath_concurrent=flightpath_builds_concurrent,
                        )
                    elif carried_flight_plans_0303 or carried_flight_plans_0304:
                        _variant_log(
                            f"[INFO] FlightPath build skipped by inputRefresh reuse "
                            f"(variant={variant_no}, carried0303={len(carried_flight_plans_0303)}, "
                            f"carried0304={len(carried_flight_plans_0304)})"
                        )

                    if (
                        bool((input_refresh_carry_summary or {}).get("applied"))
                        or input_refresh_variant_template_reused
                    ):
                        (
                            flight_plans_0303,
                            flight_plans_0304,
                            normalized_waypoint_counts,
                        ) = _normalize_input_refresh_variant_flightpaths(
                            flight_plans_0303=flight_plans_0303,
                            flight_plans_0304=flight_plans_0304,
                            missions=missions,
                            waypoint_block_0303_start=reserved_waypoint_block_0303_start,
                            waypoint_block_0303_end=reserved_waypoint_block_0303_end,
                            waypoint_block_0304_start=reserved_waypoint_block_0304_start,
                            waypoint_block_0304_end=reserved_waypoint_block_0304_end,
                        )
                        _variant_log(
                            "[REPLAN][METRIC] input_refresh_waypoint_order_restore "
                            f"variant={int(variant_no)} option={int(option_code)} "
                            f"waypoints0303={int(normalized_waypoint_counts.get('waypoints0303') or 0)} "
                            f"waypoints0304={int(normalized_waypoint_counts.get('waypoints0304') or 0)}"
                        )

                    if (
                        input_refresh_variant_template_owner
                        and isinstance(input_refresh_variant_template_future, concurrent.futures.Future)
                        and not input_refresh_variant_template_future.done()
                    ):
                        input_refresh_variant_template_future.set_result(
                            {
                                "variant_no": int(variant_no),
                                "missions": copy.deepcopy(missions),
                                "flight_plans_0303": copy.deepcopy(flight_plans_0303),
                                "flight_plans_0304": copy.deepcopy(flight_plans_0304),
                                "immutable_0304_path_ids": sorted(
                                    int(value) for value in input_refresh_immutable_0304_path_ids
                                ),
                            }
                        )
                        _variant_log(
                            "[REPLAN][METRIC] input_refresh_variant_fp_template "
                            f"variant={int(variant_no)} option={int(option_code)} role=owner "
                            f"paths0303={len(flight_plans_0303)} paths0304={len(flight_plans_0304)}"
                        )

                    remaining_hybrid_result = None
                    if current_remaining_request is not None:
                        step_t0 = time.perf_counter()
                        missions, flight_plans_0303, flight_plans_0304, hybrid_path_ids = _apply_current_remaining_hybrid_to_variant(
                            variant_no=variant_no,
                            missions=missions,
                            flight_plans_0303=flight_plans_0303,
                            flight_plans_0304=flight_plans_0304,
                            request=current_remaining_request,
                            hybrid_result=current_remaining_hybrid_result,
                            log_emit=_variant_log,
                        )
                        core_phase_ms["hybrid_merge_ms"] = (time.perf_counter() - step_t0) * 1000.0
                        variant_generated_path_ids.update(int(pid) for pid in hybrid_path_ids if pid is not None)
                    else:
                        step_t0 = time.perf_counter()
                        remaining_hybrid_result = _apply_remaining_hybrid_customization(
                            variant_no=variant_no,
                            cmpk_source_path=cmpk_source_path,
                            missions=missions,
                            flight_plans_0303=flight_plans_0303,
                            snapshot_mutated=bool(
                                int(snapshot_apply_result.get("applied") or 0) > 0
                                or int(snapshot_apply_result.get("marked_done") or 0) > 0
                            ),
                        )
                        core_phase_ms["hybrid_merge_ms"] = (time.perf_counter() - step_t0) * 1000.0

                    if flight_plans_0303 and flight_plans_0304:
                        step_t0 = time.perf_counter()
                        try:
                            flight_plans_0304 = d0304.apply_uav_eta_follow_speed_plan(
                                list(flight_plans_0304),
                                list(flight_plans_0303),
                                immutable_path_ids=set(input_refresh_immutable_0304_path_ids),
                                lah_missions=list(missions or []),
                            )
                            _variant_log(
                                f"[INFO] Applied LAH-UAV ETA follow speed plan (variant={variant_no})"
                            )
                        except Exception as exc:
                            _variant_log(
                                f"[WARN] Failed to apply LAH-UAV ETA follow speed plan (variant={variant_no}): {exc}"
                            )
                        finally:
                            core_phase_ms["eta_follow_ms"] = (time.perf_counter() - step_t0) * 1000.0

                    if not flight_plans_0303 and not flight_plans_0304:
                        _variant_log(f"[ERR] FlightPath generation failed (variant={variant_no})")
                        raise _VariantCoreError(
                            "flightpath_generation_failed",
                            f"FlightPath generation failed (variant={variant_no})",
                            variant_no=variant_no,
                        )
                    _variant_log(
                        f"[OK] FlightPath counts (variant={variant_no}): 0303={len(flight_plans_0303)} / 0304={len(flight_plans_0304)}"
                    )
                    _emit_flightpath_metric(
                        _variant_log,
                        variant_no=variant_no,
                        option_code=option_code,
                        mode="parallel_core",
                        flight_plans_0303=flight_plans_0303,
                        flight_plans_0304=flight_plans_0304,
                    )

                    core_total_ms = (time.perf_counter() - variant_start) * 1000.0
                    _variant_log(
                        "[REPLAN][METRIC] general_variant_core_phase "
                        f"variant={int(variant_no)} option={int(option_code)} mode=parallel_core "
                        f"divide_and_pattern_ms={core_phase_ms['divide_and_pattern_ms']:.3f} "
                        f"build_0301_load_ms={core_phase_ms['build_0301_load_ms']:.3f} "
                        f"collect_missions_ms={core_phase_ms['collect_missions_ms']:.3f} "
                        f"current_hybrid_wait_ms={core_phase_ms['current_hybrid_wait_ms']:.3f} "
                        f"current_hybrid_build_ms={core_phase_ms['current_hybrid_build_ms']:.3f} "
                        f"flightpath_0303_ms={core_phase_ms['flightpath_0303_ms']:.3f} "
                        f"flightpath_0304_ms={core_phase_ms['flightpath_0304_ms']:.3f} "
                        f"hybrid_merge_ms={core_phase_ms['hybrid_merge_ms']:.3f} "
                        f"eta_follow_ms={core_phase_ms['eta_follow_ms']:.3f} "
                        f"core_total_ms={float(core_total_ms):.3f}"
                    )
                    _variant_timing(
                        "variant_core_finished",
                        {
                            "variant": variant_no,
                            "option": option_code,
                            "mode": "parallel_core",
                            "status": "success",
                            "duration_ms": round(core_total_ms, 3),
                        },
                    )
                    return {
                        "variant_no": variant_no,
                        "requested_plan_id": requested_plan_id,
                        "option_code": option_code,
                        "cmpk_source_path": cmpk_source_path,
                        "iter_out_root": iter_out_root,
                        "mp_json": mp_json,
                        "imp_id_map": imp_id_map,
                        "missions": missions,
                        "flight_plans_0303": flight_plans_0303,
                        "flight_plans_0304": flight_plans_0304,
                        "generated_path_ids": variant_generated_path_ids,
                        "option_dependent_isolation": _option_dependent_isolation_contract(
                            variant_no=variant_no,
                            option_code=option_code,
                            mode="parallel_core",
                        ),
                        "current_remaining_hybrid_active": current_remaining_request is not None,
                        "current_remaining_hybrid_applied": current_remaining_hybrid_result is not None,
                        "remaining_hybrid_result": remaining_hybrid_result,
                        "core_total_ms": core_total_ms,
                        "core_phase_ms": dict(core_phase_ms),
                        "mission_timings_0303": list((build_result_0303 or {}).get("mission_timings") or []),
                        "mission_timings_0304": list(mission_timings_0304 or []),
                        "flightpath_builds_concurrent": bool(flightpath_builds_concurrent),
                        "log_messages": variant_logs,
                        "timing_events": variant_timing_events,
                    }

            def _prepare_general_variant_store(result: Dict[str, Any]) -> Dict[str, Any]:
                variant_no = int(result["variant_no"])
                variant_store_prepare_start = time.perf_counter()
                requested_plan_id = result.get("requested_plan_id")
                option_code = int(result["option_code"])
                iter_out_root = Path(result["iter_out_root"])
                mp_json = result["mp_json"]
                imp_id_map = result["imp_id_map"]
                missions = result["missions"]
                flight_plans_0303 = result["flight_plans_0303"]
                flight_plans_0304 = result["flight_plans_0304"]
                remaining_hybrid_result = result.get("remaining_hybrid_result")
                current_remaining_parallel = bool(result.get("current_remaining_hybrid_active"))
                option_dependent_isolation = dict(result.get("option_dependent_isolation") or {})
                try:
                    store_json_write_workers = max(
                        1,
                        int(
                            result.get("store_json_write_workers")
                            or _general_parallel_runtime_config().get("replan_store_json_write_workers")
                            or 2
                        ),
                    )
                except Exception:
                    store_json_write_workers = 2
                variant_generated_path_ids: Set[int] = {
                    int(val) for val in (result.get("generated_path_ids") or set()) if val is not None
                }
                store_phase_ms: Dict[str, float] = {
                    "pathID_mapping_ms": 0.0,
                    "build_0302_ms": 0.0,
                    "validate_ms": 0.0,
                    "serialize_0302_ms": 0.0,
                    "serialize_flightpath_ms": 0.0,
                    "write_0302_ms": 0.0,
                    "write_flightpath_ms": 0.0,
                    "write_artifacts_ms": 0.0,
                    "repair_flightpath_ms": 0.0,
                    "write_0301_ms": 0.0,
                    "waypoint_mark_ms": 0.0,
                    "carry_forward_snapshot_ms": 0.0,
                    "store_prepare_ms": 0.0,
                    "store_commit_ms": 0.0,
                    "store_total_ms": 0.0,
                }

                pre_path_snapshot = _snapshot_mission_path_ids(missions)
                step_t0 = time.perf_counter()
                pid_map = _assign_fresh_path_ids(
                    missions,
                    variant_generated_path_ids,
                    reserved_path_ids_by_aircraft=result.get("reserved_path_ids_by_aircraft") or {},
                )
                step_ms = (time.perf_counter() - step_t0) * 1000.0
                store_phase_ms["pathID_mapping_ms"] = float(step_ms)
                if current_remaining_parallel:
                    self.log_sig.emit(
                        f"[TIME] pathID_mapping (variant={variant_no}, current_remaining_parallel_store): {step_ms:.1f} ms"
                    )
                    self.log_sig.emit(
                        f"[INFO] pathID mapping done after current remaining hybrid merge (variant={variant_no}, parallel_store)"
                    )
                else:
                    self.log_sig.emit(f"[TIME] pathID_mapping (variant={variant_no}): {step_ms:.1f} ms")
                    self.log_sig.emit(f"[INFO] pathID mapping done for 0302/0303/0304 (variant={variant_no})")
                path_remap_by_old = _build_path_id_remap(pre_path_snapshot, missions)

                fixed3 = _enforce_fp_path_ids(
                    flight_plans_0303,
                    pid_map,
                    path_remap_by_old=path_remap_by_old,
                )
                fixed4 = _enforce_fp_path_ids(
                    flight_plans_0304,
                    pid_map,
                    path_remap_by_old=path_remap_by_old,
                )
                if fixed3 or fixed4:
                    self.log_sig.emit(
                        f"[INFO] FlightPath pathID enforced (variant={variant_no}): 0303={fixed3}, 0304={fixed4}"
                    )
                duplicate_repairs = _repair_duplicate_flightpath_path_ids(
                    missions=missions,
                    flight_plans_0303=flight_plans_0303,
                    flight_plans_0304=flight_plans_0304,
                    generated_path_ids=variant_generated_path_ids,
                    pid_map=pid_map,
                )
                if duplicate_repairs:
                    self.log_sig.emit(
                        f"[WARN] FlightPath duplicate pathID repaired before write "
                        f"(variant={variant_no}): fixed={duplicate_repairs}"
                    )
                _validate_unique_flightpath_ids(
                    variant_no=variant_no,
                    flight_plans_0303=flight_plans_0303,
                    flight_plans_0304=flight_plans_0304,
                )
                _apply_manual_runtime_fov_overrides(
                    missions=missions,
                    flight_plans_0303=flight_plans_0303,
                    variant_no=variant_no,
                )
                expected_path_ids = _expected_mission_path_ids(missions)
                available_path_ids = _collect_valid_path_ids(flight_plans_0303)
                available_path_ids.update(_collect_valid_path_ids(flight_plans_0304))
                missing_path_ids = sorted(pid for pid in expected_path_ids if pid not in available_path_ids)
                if missing_path_ids:
                    missing_summary = ", ".join(str(pid) for pid in missing_path_ids)
                    self.log_sig.emit(
                        f"[ERR] FlightPath generation incomplete (variant={variant_no}): missing pathID(s) {missing_summary}"
                    )
                    raise RuntimeError(
                        f"FlightPath generation incomplete (variant={variant_no}): missing pathID(s) {missing_summary}"
                    )

                plan_id, plan_id_contract = _allocate_general_variant_plan_id(
                    variant_no=variant_no,
                    option_code=option_code,
                    requested_plan_id=requested_plan_id,
                    generated_plan_json=mp_json,
                )
                mp_json["missionPlanID"] = plan_id
                imp_id_map = _allocate_imp_id_map(
                    mp_json,
                    reserved_imp_ids=result.get("reserved_imp_ids") or [],
                )

                step_t0 = time.perf_counter()
                imp_pkgs = d0302.build_mission_packages(
                    missions,
                    cmpk_id=cmpk_id,
                    plan_pkg_map=imp_id_map,
                    reserved_individual_mission_ids=result.get("reserved_individual_mission_ids") or [],
                )
                step_ms = (time.perf_counter() - step_t0) * 1000.0
                store_phase_ms["build_0302_ms"] = float(step_ms)
                self.log_sig.emit(
                    f"[TIME] build_0302 (variant={variant_no}): {step_ms:.1f} ms, packages={len(imp_pkgs)}"
                )
                _sync_flight_plan_individual_mission_ids(
                    variant_no=variant_no,
                    imp_pkgs=imp_pkgs,
                    flight_plans_0303=flight_plans_0303,
                    flight_plans_0304=flight_plans_0304,
                )
                _validate_mission_flightpath_links(
                    variant_no=variant_no,
                    missions=missions,
                    flight_plans_0303=flight_plans_0303,
                    flight_plans_0304=flight_plans_0304,
                )
                step_t0 = time.perf_counter()
                validation_summary = validate_replan_payloads(
                    mission_plan=mp_json,
                    individual_mission_plans=imp_pkgs,
                    flight_paths=list(flight_plans_0303 or []) + list(flight_plans_0304 or []),
                    scope=f"generalFallback:{plan_id}",
                    allow_existing_db_artifacts=True,
                    log=self.log_sig.emit,
                )
                step_ms = (time.perf_counter() - step_t0) * 1000.0
                store_phase_ms["validate_ms"] = float(step_ms)
                self.log_sig.emit(
                    f"[TIME] validate_general_variant (variant={variant_no}): {step_ms:.1f} ms"
                )

                imp_write_rows: list[tuple[Path, Any]] = []
                prepared_imp_ids: Set[int] = set()
                for pkg in imp_pkgs:
                    imp_id = pkg.get("individualMissionPackageID") or pkg.get("individualMissionPlanPackageID")
                    if imp_id is None:
                        continue
                    try:
                        prepared_imp_ids.add(int(imp_id))
                    except Exception:
                        pass
                    imp_write_rows.append((dir_imp / f"{int(imp_id)}.json", pkg))

                def _build_fp_rows(target_dir, fps):
                    rows: list[tuple[Path, Any]] = []
                    ids: Set[int] = set()
                    for fp in fps:
                        pid = fp.get("pathID")
                        if pid is None:
                            continue
                        rows.append((target_dir / f"{int(pid)}.json", fp))
                        try:
                            ids.add(int(pid))
                        except Exception:
                            pass
                    return rows, ids

                fp_write_rows_0303, fp_ids_0303 = _build_fp_rows(dir_fp, flight_plans_0303)
                fp_write_rows_0304, fp_ids_0304 = _build_fp_rows(dir_fp, flight_plans_0304)
                fp_count_0303 = len(fp_write_rows_0303)
                fp_count_0304 = len(fp_write_rows_0304)
                step_t0 = time.perf_counter()
                imp_write_bytes_rows = _serialize_json_batch(
                    imp_write_rows,
                    pretty=True,
                    max_workers=1,
                )
                store_phase_ms["serialize_0302_ms"] = (time.perf_counter() - step_t0) * 1000.0
                step_t0 = time.perf_counter()
                fp_write_rows_all = list(fp_write_rows_0303) + list(fp_write_rows_0304)
                fp_write_bytes_rows_all = _serialize_json_batch(
                    fp_write_rows_all,
                    pretty=False,
                    max_workers=store_json_write_workers,
                )
                fp_split_idx = len(fp_write_rows_0303)
                fp_write_bytes_rows_0303 = fp_write_bytes_rows_all[:fp_split_idx]
                fp_write_bytes_rows_0304 = fp_write_bytes_rows_all[fp_split_idx:]
                store_phase_ms["serialize_flightpath_ms"] = (time.perf_counter() - step_t0) * 1000.0
                prepare_total_ms = (time.perf_counter() - variant_store_prepare_start) * 1000.0
                store_phase_ms["store_prepare_ms"] = float(prepare_total_ms)
                result["store_phase_ms"] = dict(store_phase_ms)
                result["store_prepare_total_ms"] = float(prepare_total_ms)
                result["store_prepare_wait_for_order_ms"] = float(
                    result.get("store_prepare_wait_for_order_ms") or 0.0
                )
                result["store_prepare_queue_wait_ms"] = float(
                    result.get("store_prepare_queue_wait_ms") or result.get("store_prepare_wait_for_order_ms") or 0.0
                )
                result["store_prepare_enqueue_total_wait_ms"] = float(
                    result.get("store_prepare_enqueue_total_wait_ms") or 0.0
                )
                self.log_sig.emit(
                    "[REPLAN][METRIC] general_variant_store_prepare_phase "
                    f"variant={int(variant_no)} option={int(option_code)} mode=parallel_store_prepare "
                    f"wait_for_order_ms={float(result.get('store_prepare_wait_for_order_ms') or 0.0):.3f} "
                    f"queue_wait_ms={float(result.get('store_prepare_queue_wait_ms') or 0.0):.3f} "
                    f"enqueue_total_wait_ms={float(result.get('store_prepare_enqueue_total_wait_ms') or 0.0):.3f} "
                    f"path_id_reservation_ms={float(result.get('path_id_reservation_ms') or 0.0):.3f} "
                    f"pathID_mapping_ms={store_phase_ms['pathID_mapping_ms']:.3f} "
                    f"build_0302_ms={store_phase_ms['build_0302_ms']:.3f} "
                    f"validate_ms={store_phase_ms['validate_ms']:.3f} "
                    f"serialize_0302_ms={store_phase_ms['serialize_0302_ms']:.3f} "
                    f"serialize_flightpath_ms={store_phase_ms['serialize_flightpath_ms']:.3f} "
                    "flightpath_json_pretty=0 "
                    f"imp_files={len(imp_pkgs)} fp_files={fp_count_0303 + fp_count_0304} "
                    f"store_json_write_workers={int(store_json_write_workers)} "
                    f"store_prepare_ms={float(prepare_total_ms):.3f}"
                )
                return {
                    "result": result,
                    "variant_no": variant_no,
                    "option_code": option_code,
                    "iter_out_root": iter_out_root,
                    "plan_id": plan_id,
                    "plan_id_contract": plan_id_contract,
                    "mp_json": mp_json,
                    "imp_pkgs": imp_pkgs,
                    "flight_plans_0303": flight_plans_0303,
                    "flight_plans_0304": flight_plans_0304,
                    "imp_write_rows": imp_write_rows,
                    "imp_write_bytes_rows": imp_write_bytes_rows,
                    "fp_write_rows_0303": fp_write_rows_0303,
                    "fp_write_rows_0304": fp_write_rows_0304,
                    "fp_write_bytes_rows_0303": fp_write_bytes_rows_0303,
                    "fp_write_bytes_rows_0304": fp_write_bytes_rows_0304,
                    "fp_count_0303": fp_count_0303,
                    "fp_count_0304": fp_count_0304,
                    "prepared_imp_ids": prepared_imp_ids,
                    "prepared_path_ids": set(fp_ids_0303).union(fp_ids_0304),
                    "variant_generated_path_ids": variant_generated_path_ids,
                    "remaining_hybrid_result": remaining_hybrid_result,
                    "option_dependent_isolation": option_dependent_isolation,
                    "validation_summary": validation_summary,
                    "store_phase_ms": store_phase_ms,
                    "prepare_total_ms": float(prepare_total_ms),
                    "store_json_write_workers": int(store_json_write_workers),
                    "defer_waypoint_files_written_mark": bool(
                        result.get("defer_waypoint_files_written_mark")
                    ),
                    "defer_snapshot_carry_forward": bool(result.get("defer_snapshot_carry_forward")),
                }

            def _write_general_variant_store_files(staged: Dict[str, Any]) -> Dict[str, Any]:
                result = staged["result"]
                variant_no = int(staged["variant_no"])
                option_code = int(staged["option_code"])
                plan_id = int(staged["plan_id"])
                mp_json = staged["mp_json"]
                imp_pkgs = staged["imp_pkgs"]
                flight_plans_0303 = staged["flight_plans_0303"]
                flight_plans_0304 = staged["flight_plans_0304"]
                store_phase_ms: Dict[str, float] = dict(staged.get("store_phase_ms") or {})
                try:
                    store_json_write_workers = max(1, int(staged.get("store_json_write_workers") or 2))
                except Exception:
                    store_json_write_workers = 2
                file_commit_start = time.perf_counter()
                created_final_paths: list[Path] = []

                def _split_write_results(results: list[bool], first_count: int) -> tuple[int, int]:
                    split_idx = max(0, min(int(first_count), len(results)))
                    first_written = sum(1 for written in results[:split_idx] if written)
                    second_written = sum(1 for written in results[split_idx:] if written)
                    return int(first_written), int(second_written)

                def _track_new_paths(rows: list[tuple[Path, Any]]) -> None:
                    for path, _payload in rows or []:
                        try:
                            if not Path(path).exists():
                                created_final_paths.append(Path(path))
                        except Exception:
                            continue

                try:
                    imp_write_rows = list(staged.get("imp_write_bytes_rows") or staged.get("imp_write_rows") or [])
                    fp_write_rows_0303 = list(
                        staged.get("fp_write_bytes_rows_0303") or staged.get("fp_write_rows_0303") or []
                    )
                    fp_write_rows_0304 = list(
                        staged.get("fp_write_bytes_rows_0304") or staged.get("fp_write_rows_0304") or []
                    )
                    fp_write_rows_all = fp_write_rows_0303 + fp_write_rows_0304
                    artifact_write_rows = imp_write_rows + fp_write_rows_all
                    _track_new_paths(artifact_write_rows)
                    all_artifact_paths_new = len(created_final_paths) == len(artifact_write_rows)
                    step_t0 = time.perf_counter()
                    artifact_write_results = _write_json_bytes_batch_results(
                        artifact_write_rows,
                        max_workers=store_json_write_workers,
                        skip_if_unchanged=not all_artifact_paths_new,
                    )
                    artifact_write_ms = (time.perf_counter() - step_t0) * 1000.0
                    imp_written_count, fp_written_count = _split_write_results(
                        artifact_write_results,
                        len(imp_write_rows),
                    )
                    fp_results = artifact_write_results[len(imp_write_rows):]
                    fp_count_0303, fp_count_0304 = _split_write_results(
                        fp_results,
                        len(fp_write_rows_0303),
                    )
                    store_phase_ms["write_0302_ms"] = float(artifact_write_ms if imp_write_rows and not fp_write_rows_all else 0.0)
                    store_phase_ms["write_flightpath_ms"] = float(artifact_write_ms if fp_write_rows_all else 0.0)
                    store_phase_ms["write_artifacts_ms"] = float(artifact_write_ms)
                    self.log_sig.emit(
                        f"[TIME] write_0302+FlightPath (variant={variant_no}): {artifact_write_ms:.1f} ms, "
                        f"impFiles={len(imp_pkgs)}, fpFiles={fp_count_0303 + fp_count_0304}, "
                        f"written={imp_written_count + fp_written_count}"
                    )
                    step_ms = (time.perf_counter() - step_t0) * 1000.0
                    self.log_sig.emit(
                        f"[TIME] write_FlightPath (variant={variant_no}, combined): {step_ms:.1f} ms, files={fp_count_0303 + fp_count_0304}"
                    )
                    _emit_flightpath_write_metric(
                        self.log_sig.emit,
                        variant_no=variant_no,
                        option_code=option_code,
                        mode="parallel_store",
                        files_0303=fp_count_0303,
                        files_0304=fp_count_0304,
                        write_ms=step_ms,
                    )
                    repair_t0 = time.perf_counter()
                    repaired_fp_count, missing_fp_ids = _repair_missing_flight_path_files(
                        variant_no=variant_no,
                        dir_fp=dir_fp,
                        imp_pkgs=imp_pkgs,
                        flight_plans_0303=flight_plans_0303,
                        flight_plans_0304=flight_plans_0304,
                    )
                    store_phase_ms["repair_flightpath_ms"] = (time.perf_counter() - repair_t0) * 1000.0
                    if missing_fp_ids:
                        missing_summary = ", ".join(str(pid) for pid in missing_fp_ids)
                        self.log_sig.emit(
                            f"[ERR] FlightPath write incomplete (variant={variant_no}): missing pathID(s) {missing_summary}"
                        )
                        raise RuntimeError(
                            f"FlightPath write incomplete (variant={variant_no}): missing pathID(s) {missing_summary}"
                        )
                    if repaired_fp_count:
                        step_ms = (time.perf_counter() - step_t0) * 1000.0
                        store_phase_ms["write_flightpath_ms"] = float(step_ms)
                        self.log_sig.emit(
                            f"[TIME] write_FlightPath+repair (variant={variant_no}): {step_ms:.1f} ms"
                        )
                    max_waypoint_id = _max_waypoint_id_from_flight_plans(
                        flight_plans_0303,
                        flight_plans_0304,
                    )
                    try:
                        if not bool(staged.get("defer_waypoint_files_written_mark")):
                            waypoint_mark_t0 = time.perf_counter()
                            mark_waypoint_files_written(max_waypoint_id)
                            store_phase_ms["waypoint_mark_ms"] = (
                                time.perf_counter() - waypoint_mark_t0
                            ) * 1000.0
                    except Exception:
                        pass

                    variant_total_ms = (
                        float(result.get("core_total_ms") or 0.0)
                        + float(staged.get("prepare_total_ms") or 0.0)
                        + (time.perf_counter() - file_commit_start) * 1000.0
                    )
                    step_t0 = time.perf_counter()
                    mp_json["planningTime"] = variant_total_ms
                    mp_path = dir_mp / f"{plan_id}.json"
                    if not mp_path.exists():
                        created_final_paths.append(mp_path)
                    write_json(mp_path, mp_json, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
                    step_ms = (time.perf_counter() - step_t0) * 1000.0
                    store_phase_ms["write_0301_ms"] = float(step_ms)
                    self.log_sig.emit(f"[TIME] write_0301 (variant={variant_no}): {step_ms:.1f} ms")
                    store_phase_ms["store_file_commit_ms"] = (time.perf_counter() - file_commit_start) * 1000.0
                    return {
                        "staged": staged,
                        "result": result,
                        "variant_no": variant_no,
                        "option_code": option_code,
                        "plan_id": plan_id,
                        "mp_json": mp_json,
                        "imp_pkgs": imp_pkgs,
                        "flight_plans_0303": flight_plans_0303,
                        "flight_plans_0304": flight_plans_0304,
                        "fp_count_0303": int(fp_count_0303),
                        "fp_count_0304": int(fp_count_0304),
                        "store_phase_ms": dict(store_phase_ms),
                        "variant_total_ms": float(variant_total_ms),
                        "created_final_paths": list(created_final_paths),
                        "store_json_write_workers": int(store_json_write_workers),
                        "max_waypoint_id": max_waypoint_id,
                    }
                except Exception:
                    for path in reversed(created_final_paths):
                        try:
                            if path.exists():
                                path.unlink()
                        except Exception:
                            pass
                    raise

            def _finalize_general_variant_store(committed: Dict[str, Any]) -> None:
                nonlocal total_imp_files, total_fp_files
                staged = committed["staged"]
                result = committed["result"]
                variant_no = int(committed["variant_no"])
                option_code = int(committed["option_code"])
                iter_out_root = Path(staged["iter_out_root"])
                plan_id = int(committed["plan_id"])
                plan_id_contract = dict(staged.get("plan_id_contract") or {})
                imp_pkgs = committed["imp_pkgs"]
                remaining_hybrid_result = staged.get("remaining_hybrid_result")
                option_dependent_isolation = dict(staged.get("option_dependent_isolation") or {})
                variant_generated_path_ids: Set[int] = {
                    int(val) for val in (staged.get("variant_generated_path_ids") or set()) if val is not None
                }
                validation_summary = staged.get("validation_summary")
                store_phase_ms: Dict[str, float] = dict(committed.get("store_phase_ms") or {})
                try:
                    store_json_write_workers = max(1, int(committed.get("store_json_write_workers") or 2))
                except Exception:
                    store_json_write_workers = 2
                finalize_start = time.perf_counter()
                snapshot_reason = (
                    "general_remaining_hybrid"
                    if getattr(remaining_hybrid_result, "applied", False)
                    else "general_fallback"
                )
                defer_snapshot_carry = bool(staged.get("defer_snapshot_carry_forward"))
                if defer_snapshot_carry:
                    source_id = _positive_int_or_none(snapshot_source_plan_id)
                    target_id = _positive_int_or_none(plan_id)
                    snapshot_carry_forward = {
                        "sourceMissionPlanID": source_id,
                        "targetMissionPlanID": target_id,
                        "carried": False,
                        "path": None,
                        "reason": str(snapshot_reason or ""),
                        "deferred": bool(source_id is not None and target_id is not None),
                    }
                    if source_id is not None and target_id is not None:
                        result["post_delivery_snapshot_carry_forward_item"] = {
                            "sourceMissionPlanID": int(source_id),
                            "targetMissionPlanID": int(target_id),
                            "variant": int(variant_no),
                            "reason": str(snapshot_reason or ""),
                        }
                        snapshot_carry_forward["scheduledPostDelivery"] = True
                    store_phase_ms["carry_forward_snapshot_ms"] = 0.0
                else:
                    step_t0 = time.perf_counter()
                    snapshot_carry_forward = _carry_forward_mission_area_snapshot(
                        variant_no=variant_no,
                        plan_id=plan_id,
                        reason=snapshot_reason,
                    )
                    store_phase_ms["carry_forward_snapshot_ms"] = (time.perf_counter() - step_t0) * 1000.0
                store_phase_ms["store_finalize_ms"] = (time.perf_counter() - finalize_start) * 1000.0
                store_phase_ms["store_commit_ms"] = float(store_phase_ms.get("store_file_commit_ms") or 0.0) + float(
                    store_phase_ms["store_finalize_ms"]
                )
                store_phase_ms["store_total_ms"] = float(staged.get("prepare_total_ms") or 0.0) + float(
                    store_phase_ms["store_commit_ms"]
                )
                result["store_phase_ms"] = dict(store_phase_ms)
                result["store_total_ms"] = float(store_phase_ms["store_total_ms"])
                result["variant_wall_total_ms"] = float(result.get("core_total_ms") or 0.0) + float(
                    store_phase_ms["store_total_ms"]
                )
                variant_total_ms = float(committed.get("variant_total_ms") or 0.0)
                self.log_sig.emit(
                    "[REPLAN][METRIC] general_variant_store_phase "
                    f"variant={int(variant_no)} option={int(option_code)} mode=parallel_store "
                    f"pathID_mapping_ms={store_phase_ms['pathID_mapping_ms']:.3f} "
                    f"build_0302_ms={store_phase_ms['build_0302_ms']:.3f} "
                    f"validate_ms={store_phase_ms['validate_ms']:.3f} "
                    f"serialize_0302_ms={float(store_phase_ms.get('serialize_0302_ms') or 0.0):.3f} "
                    f"serialize_flightpath_ms={float(store_phase_ms.get('serialize_flightpath_ms') or 0.0):.3f} "
                    f"write_0302_ms={store_phase_ms['write_0302_ms']:.3f} "
                    f"write_flightpath_ms={store_phase_ms['write_flightpath_ms']:.3f} "
                    f"write_artifacts_ms={float(store_phase_ms.get('write_artifacts_ms') or 0.0):.3f} "
                    f"repair_flightpath_ms={float(store_phase_ms.get('repair_flightpath_ms') or 0.0):.3f} "
                    f"write_0301_ms={store_phase_ms['write_0301_ms']:.3f} "
                    f"waypoint_mark_ms={float(store_phase_ms.get('waypoint_mark_ms') or 0.0):.3f} "
                    f"defer_waypoint_mark={int(bool(staged.get('defer_waypoint_files_written_mark')))} "
                    f"carry_forward_snapshot_ms={store_phase_ms['carry_forward_snapshot_ms']:.3f} "
                    f"store_prepare_ms={store_phase_ms['store_prepare_ms']:.3f} "
                    f"store_file_commit_ms={float(store_phase_ms.get('store_file_commit_ms') or 0.0):.3f} "
                    f"store_finalize_ms={store_phase_ms['store_finalize_ms']:.3f} "
                    f"store_json_write_workers={int(store_json_write_workers)} "
                    f"store_commit_ms={store_phase_ms['store_commit_ms']:.3f} "
                    f"store_total_ms={store_phase_ms['store_total_ms']:.3f}"
                )
                self.log_sig.emit(
                    f"[OK] Stored variant {variant_no}: MissionPlanID={plan_id}, IMP={len(imp_pkgs)}, FlightPath={int(committed['fp_count_0303']) + int(committed['fp_count_0304'])}"
                )
                self._record_replan_timing_event(
                    "variant_finished",
                    extra={
                        "variant": variant_no,
                        "option": option_code,
                        "mode": "parallel_store",
                        "status": "success",
                        "duration_ms": round(variant_total_ms, 3),
                    },
                )
                self.log_sig.emit(f"[TIME] variant_total (variant={variant_no}): {variant_total_ms:.1f} ms")
                _emit_option_total_timing_metric(
                    self.log_sig.emit,
                    variant_no=variant_no,
                    option_code=option_code,
                    mode="parallel_store",
                    core_phase_ms=result.get("core_phase_ms") if isinstance(result, dict) else {},
                    store_phase_ms=store_phase_ms,
                    variant_total_ms=variant_total_ms,
                )

                generated_imp_ids.update(int(value) for value in (staged.get("prepared_imp_ids") or set()))
                stored_path_ids.update(int(value) for value in (staged.get("prepared_path_ids") or set()))
                total_imp_files += len(imp_pkgs)
                total_fp_files += int(committed["fp_count_0303"]) + int(committed["fp_count_0304"])
                generated_plan_ids.append(plan_id)
                option_codes_out.append(int(option_code))
                generated_path_ids.update(variant_generated_path_ids)
                plan_meta_map.setdefault(int(plan_id), {}).update(
                    {
                        "variant": int(variant_no),
                        "optionCode": int(option_code),
                        "requestedMissionPlanID": plan_id_contract["requestedMissionPlanID"],
                        "missionPlanIDMatchesRequest": plan_id_contract["missionPlanIDMatchesRequest"],
                        "planIDContract": plan_id_contract,
                        "optionDependentIsolation": option_dependent_isolation,
                        "currentRemainingHybridSharePolicy": copy.deepcopy(current_remaining_hybrid_share_policy),
                        "missionAreaSnapshotCarryForward": snapshot_carry_forward,
                        "validation": validation_summary,
                    }
                )
                if getattr(remaining_hybrid_result, "applied", False):
                    plan_meta_map.setdefault(int(plan_id), {}).update(
                        {
                            "remainingHybridApplied": True,
                            "remainingHybridMode": str(getattr(remaining_hybrid_result, "mode", "") or ""),
                            "remainingHybridInputMissionID": _safe_int_value(
                                getattr(remaining_hybrid_result, "input_mission_id", None)
                            ),
                            "remainingHybridAircraftIDs": [
                                int(value)
                                for value in (getattr(remaining_hybrid_result, "aircraft_ids", []) or [])
                                if _safe_int_value(value) is not None
                            ],
                            "remainingHybridWorkflow": str(
                                getattr(remaining_hybrid_result, "planner_workflow", "") or ""
                            ),
                            "remainingHybridValidation": copy.deepcopy(
                                getattr(remaining_hybrid_result, "validation", None) or {}
                            ),
                        }
                    )
                self.log_sig.emit(
                    f"[INFO] Option mapping #{variant_no}: planID={plan_id}, optionCode={option_code}({option_code_to_label(option_code)})"
                )
                try:
                    shutil.rmtree(iter_out_root)
                except Exception:
                    pass

            def _cleanup_committed_store_files(committed_rows: list[Dict[str, Any]]) -> None:
                for committed in reversed(committed_rows or []):
                    for path in reversed(committed.get("created_final_paths") or []):
                        try:
                            path = Path(path)
                            if path.exists():
                                path.unlink()
                        except Exception:
                            pass

            def _commit_general_variant_store(staged: Dict[str, Any]) -> None:
                committed = _write_general_variant_store_files(staged)
                _finalize_general_variant_store(committed)

            def _store_general_variant(result: Dict[str, Any]) -> None:
                if "reserved_path_ids_by_aircraft" not in result:
                    result["reserved_path_ids_by_aircraft"] = _reserve_fresh_path_ids_for_missions(
                        result.get("missions") or []
                    )
                result.setdefault(
                    "store_json_write_workers",
                    int(_general_parallel_runtime_config().get("replan_store_json_write_workers") or 2),
                )
                staged = _prepare_general_variant_store(result)
                _commit_general_variant_store(staged)

            def _flush_parallel_variant_diagnostics(result: Dict[str, Any]) -> None:
                for event_payload in list(result.get("timing_events") or []):
                    if not isinstance(event_payload, dict):
                        continue
                    self._record_replan_timing_snapshot(
                        str(event_payload.get("event") or ""),
                        event_perf=float(event_payload.get("perf") or time.perf_counter()),
                        wall_ms=int(event_payload.get("wall_ms") or int(time.time() * 1000)),
                        extra=dict(event_payload.get("extra") or {}),
                    )
                for line in list(result.get("log_messages") or []):
                    self.log_sig.emit(str(line))
                result["timing_events"] = []
                result["log_messages"] = []

            def _record_general_3_option_summary(results_by_idx: Dict[int, Dict[str, Any]]) -> None:
                if not results_by_idx:
                    return
                ordered_results = [
                    results_by_idx[idx]
                    for idx in sorted(results_by_idx)
                    if isinstance(results_by_idx.get(idx), dict)
                ]
                if not ordered_results:
                    return
                timing = ctx.get("_replan_timing") if isinstance(ctx, dict) else {}
                timing = timing if isinstance(timing, dict) else {}
                try:
                    base_perf = float(timing.get("base_perf"))
                    total_ms = max(0.0, (time.perf_counter() - base_perf) * 1000.0)
                except Exception:
                    total_ms = 0.0
                core_rows = [
                    (
                        float(row.get("core_total_ms") or 0.0),
                        int(row.get("variant_no") or 0),
                        int(row.get("option_code") or 0),
                    )
                    for row in ordered_results
                ]
                total_rows = [
                    (
                        float(row.get("variant_wall_total_ms") or row.get("core_total_ms") or 0.0),
                        int(row.get("variant_no") or 0),
                        int(row.get("option_code") or 0),
                    )
                    for row in ordered_results
                ]
                max_core_ms, max_core_variant, max_core_option = max(core_rows, default=(0.0, 0, 0))
                max_total_ms, critical_variant, critical_option = max(total_rows, default=(0.0, 0, 0))
                store_total_ms = sum(float(row.get("store_total_ms") or 0.0) for row in ordered_results)
                summary_perf = time.perf_counter()
                core_ready_perfs = [
                    float(row.get("store_core_result_ready_perf") or 0.0)
                    for row in ordered_results
                    if float(row.get("store_core_result_ready_perf") or 0.0) > 0.0
                ]
                if core_ready_perfs:
                    store_wall_tail_ms = max(0.0, (summary_perf - max(core_ready_perfs)) * 1000.0)
                else:
                    store_wall_tail_ms = max(0.0, float(total_ms) - float(max_core_ms))
                store_prepare_order_wait_values = [
                    float(row.get("store_prepare_wait_for_order_ms") or 0.0)
                    for row in ordered_results
                ]
                store_prepare_queue_wait_values = [
                    float(row.get("store_prepare_queue_wait_ms") or row.get("store_prepare_wait_for_order_ms") or 0.0)
                    for row in ordered_results
                ]
                store_prepare_path_reservation_values = [
                    float(row.get("path_id_reservation_ms") or 0.0)
                    for row in ordered_results
                ]
                store_prepare_cross_path_reservation_values = [
                    float(row.get("cross_path_id_reservation_ms") or 0.0)
                    for row in ordered_results
                ]
                store_prepare_enqueue_total_wait_values = [
                    float(row.get("store_prepare_enqueue_total_wait_ms") or 0.0)
                    for row in ordered_results
                ]
                store_prepare_order_wait_ms = max(store_prepare_order_wait_values, default=0.0)
                store_prepare_order_wait_total_ms = sum(store_prepare_order_wait_values)
                store_prepare_queue_wait_ms = max(store_prepare_queue_wait_values, default=0.0)
                store_prepare_queue_wait_total_ms = sum(store_prepare_queue_wait_values)
                store_prepare_path_id_reservation_ms = max(store_prepare_path_reservation_values, default=0.0)
                store_prepare_path_id_reservation_total_ms = sum(store_prepare_path_reservation_values)
                store_prepare_cross_path_id_reservation_ms = max(
                    store_prepare_cross_path_reservation_values,
                    default=0.0,
                )
                store_prepare_enqueue_total_wait_ms = max(store_prepare_enqueue_total_wait_values, default=0.0)
                summary = {
                    "total_ms": round(float(total_ms), 3),
                    "parallel_gate_ms": round(float(timing.get("parallel_safety_gate_ms") or 0.0), 3),
                    "max_core_ms": round(float(max_core_ms), 3),
                    "max_total_ms": round(float(max_total_ms), 3),
                    "store_total_ms": round(float(store_total_ms), 3),
                    "store_wall_tail_ms": round(float(store_wall_tail_ms), 3),
                    "store_prepare_order_wait_ms": round(float(store_prepare_order_wait_ms), 3),
                    "store_prepare_order_wait_total_ms": round(float(store_prepare_order_wait_total_ms), 3),
                    "store_prepare_queue_wait_ms": round(float(store_prepare_queue_wait_ms), 3),
                    "store_prepare_queue_wait_total_ms": round(float(store_prepare_queue_wait_total_ms), 3),
                    "store_prepare_path_id_reservation_ms": round(float(store_prepare_path_id_reservation_ms), 3),
                    "store_prepare_path_id_reservation_total_ms": round(float(store_prepare_path_id_reservation_total_ms), 3),
                    "store_prepare_cross_path_id_reservation_ms": round(float(store_prepare_cross_path_id_reservation_ms), 3),
                    "store_prepare_enqueue_total_wait_ms": round(float(store_prepare_enqueue_total_wait_ms), 3),
                    "critical_variant": int(critical_variant or max_core_variant),
                    "critical_option": int(critical_option or max_core_option),
                    "max_core_variant": int(max_core_variant),
                    "max_core_option": int(max_core_option),
                    "variant_count": len(ordered_results),
                }
                if isinstance(timing, dict):
                    timing["general_3_option_summary"] = dict(summary)
                self.log_sig.emit(
                    "[REPLAN][METRIC] general_3_option_summary "
                    f"total_ms={summary['total_ms']:.3f} "
                    f"parallel_gate_ms={summary['parallel_gate_ms']:.3f} "
                    f"max_core_ms={summary['max_core_ms']:.3f} "
                    f"max_total_ms={summary['max_total_ms']:.3f} "
                    f"store_total_ms={summary['store_total_ms']:.3f} "
                    f"store_wall_tail_ms={summary['store_wall_tail_ms']:.3f} "
                    f"store_prepare_order_wait_ms={summary['store_prepare_order_wait_ms']:.3f} "
                    f"store_prepare_order_wait_total_ms={summary['store_prepare_order_wait_total_ms']:.3f} "
                    f"store_prepare_queue_wait_ms={summary['store_prepare_queue_wait_ms']:.3f} "
                    f"store_prepare_queue_wait_total_ms={summary['store_prepare_queue_wait_total_ms']:.3f} "
                    f"store_prepare_path_id_reservation_ms={summary['store_prepare_path_id_reservation_ms']:.3f} "
                    f"store_prepare_path_id_reservation_total_ms={summary['store_prepare_path_id_reservation_total_ms']:.3f} "
                    f"store_prepare_cross_path_id_reservation_ms={summary['store_prepare_cross_path_id_reservation_ms']:.3f} "
                    f"store_prepare_enqueue_total_wait_ms={summary['store_prepare_enqueue_total_wait_ms']:.3f} "
                    f"critical_variant={summary['critical_variant']} "
                    f"critical_option={summary['critical_option']} "
                    f"max_core_variant={summary['max_core_variant']} "
                    f"max_core_option={summary['max_core_option']} "
                    f"variant_count={summary['variant_count']} "
                    "0305_status_2_ms=pending"
                )

            def _build_shared_current_remaining_hybrid_for_parallel(
                *,
                request: CurrentRemainingHybridRequest,
                runtime_payload: Optional[Dict[str, Any]],
                representative_variant: int,
                shared_variants: List[int],
            ):
                build_started = time.perf_counter()
                self.log_sig.emit(
                    "[REPLAN][METRIC] current_remaining_hybrid_shared_precompute_start "
                    f"representative_variant={int(representative_variant)} "
                    f"shared_variants={'|'.join(str(int(v)) for v in shared_variants) or '-'}"
                )
                with runtime_settings_override(runtime_payload):
                    result = _build_current_remaining_hybrid_locked(
                        request,
                        variant_no=int(representative_variant),
                        log_emit=self.log_sig.emit,
                    )
                elapsed_ms = (time.perf_counter() - build_started) * 1000.0
                self.log_sig.emit(
                    "[REPLAN][METRIC] current_remaining_hybrid_shared_precompute_finish "
                    f"representative_variant={int(representative_variant)} "
                    f"shared_variants={'|'.join(str(int(v)) for v in shared_variants) or '-'} "
                    f"elapsed_ms={elapsed_ms:.3f} "
                    f"result_available={int(result is not None)}"
                )
                return result

            variant_loop_indices = range(plan_count)
            if general_parallel_replan:
                runtime_parallel_config = _general_parallel_runtime_config()
                max_workers = max(1, int(general_parallel_workers))
                recon_worker_cap = max(0, int(runtime_parallel_config["replan_recon_worker_cap"]))
                if recon_worker_cap > 0 and max_workers > recon_worker_cap:
                    requested_workers_before_recon_cap = int(max_workers)
                    has_recon_specialized_variant = any(
                        is_recon_specialized_option(
                            option_codes[idx],
                            option_labels[idx] if idx < len(option_labels) else "",
                        )
                        for idx in range(plan_count)
                    )
                    if has_recon_specialized_variant:
                        max_workers = recon_worker_cap
                        self.log_sig.emit(
                            f"[INFO] 정찰특화 옵션 포함 -> REPLAN_RECON_WORKER_CAP={recon_worker_cap} 적용 "
                            f"(requestedWorkers={requested_workers_before_recon_cap}, cappedWorkers={max_workers})"
                        )
                        self.log_sig.emit(
                            "[REPLAN][METRIC] recon_worker_cap "
                            f"requestedWorkers={requested_workers_before_recon_cap} "
                            f"cappedWorkers={int(max_workers)} variants={plan_count}"
                        )
                self.log_sig.emit(
                    f"[INFO] 일반 재계획 옵션 병렬 생성 활성화: variants={plan_count}, workers={max_workers}"
                )
                waypoint_block_size = max(
                    1000,
                    int(runtime_parallel_config["replan_variant_waypoint_block_size"]),
                )
                shared_current_remaining_hybrid_future: Optional[concurrent.futures.Future] = None
                shared_current_remaining_hybrid_executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
                isolated_current_remaining_hybrid_executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
                isolated_current_remaining_hybrid_futures: Dict[int, concurrent.futures.Future] = {}
                shared_current_remaining_ordinals = [
                    int(value)
                    for value in (current_remaining_hybrid_share_policy.get("sharedVariantOrdinals") or [])
                    if _safe_int_value(value) is not None
                ]
                isolated_current_remaining_ordinals = [
                    int(value)
                    for value in (current_remaining_hybrid_share_policy.get("isolatedVariantOrdinals") or [])
                    if _safe_int_value(value) is not None
                ]
                share_current_remaining_hybrid = (
                    current_remaining_hybrid_request is not None
                    and len(shared_current_remaining_ordinals) > 1
                    and bool(
                        current_remaining_hybrid_share_policy.get("shareAllowed")
                        or current_remaining_hybrid_share_policy.get("shareAllowedNonRecon")
                    )
                )
                if share_current_remaining_hybrid:
                    representative_variant = int(shared_current_remaining_ordinals[0])
                    if bool(current_remaining_hybrid_share_policy.get("reexecuteLineReconRuntimeNeutralized")):
                        non_recon_shared_variants = []
                        for shared_variant in shared_current_remaining_ordinals:
                            shared_idx = max(0, int(shared_variant) - 1)
                            shared_label = option_labels[shared_idx] if shared_idx < len(option_labels) else ""
                            if not is_recon_specialized_option(option_codes[shared_idx], shared_label):
                                non_recon_shared_variants.append(int(shared_variant))
                        if non_recon_shared_variants:
                            representative_variant = int(non_recon_shared_variants[0])
                    representative_request = _current_remaining_request_for_variant(
                        current_remaining_hybrid_request,
                        representative_variant,
                    )
                    if representative_request is not None:
                        representative_idx = max(0, representative_variant - 1)
                        representative_runtime_payload = _variant_runtime_override_payload(
                            int(option_codes[representative_idx]),
                            option_labels[representative_idx] if representative_idx < len(option_labels) else "",
                        )
                        shared_current_remaining_hybrid_executor = concurrent.futures.ThreadPoolExecutor(
                            max_workers=1,
                            thread_name_prefix="HybridShare",
                        )
                        shared_current_remaining_hybrid_future = shared_current_remaining_hybrid_executor.submit(
                            _build_shared_current_remaining_hybrid_for_parallel,
                            request=representative_request,
                            runtime_payload=representative_runtime_payload,
                            representative_variant=representative_variant,
                            shared_variants=list(shared_current_remaining_ordinals),
                        )
                        self.log_sig.emit(
                            "[REPLAN][METRIC] current_remaining_hybrid_share_enabled "
                            f"shared_variants={'|'.join(str(int(v)) for v in shared_current_remaining_ordinals)} "
                            f"isolated_variants={'|'.join(str(int(v)) for v in (current_remaining_hybrid_share_policy.get('isolatedVariantOrdinals') or [])) or '-'} "
                            f"runtime_key_count={int(current_remaining_hybrid_share_policy.get('runtimeKeyCount') or 0)} "
                            f"raw_runtime_key_count={int(current_remaining_hybrid_share_policy.get('rawRuntimeKeyCount') or 0)} "
                            f"shared_runtime_group_size={int(current_remaining_hybrid_share_policy.get('sharedRuntimeGroupSize') or 0)} "
                            f"current_geometry={current_remaining_hybrid_share_policy.get('currentMissionGeometry') or '-'} "
                            f"line_recon_neutralized={int(bool(current_remaining_hybrid_share_policy.get('reexecuteLineReconRuntimeNeutralized')))} "
                            f"line_recon_ordinals={'|'.join(str(int(v)) for v in (current_remaining_hybrid_share_policy.get('reexecuteLineReconNeutralizedOrdinals') or [])) or '-'} "
                            f"reason={current_remaining_hybrid_share_policy.get('reason') or '-'} "
                            f"share_strategy={current_remaining_hybrid_share_policy.get('shareStrategy') or '-'}"
                        )
                    else:
                        shared_current_remaining_ordinals = []
                isolated_precompute_ordinals = [
                    int(variant)
                    for variant in isolated_current_remaining_ordinals
                    if int(variant) not in set(shared_current_remaining_ordinals)
                ]
                if current_remaining_hybrid_request is not None and isolated_precompute_ordinals:
                    isolated_workers = max(
                        1,
                        min(
                            len(isolated_precompute_ordinals),
                            int(runtime_parallel_config.get("replan_current_remaining_precompute_workers") or 2),
                        ),
                    )
                    isolated_current_remaining_hybrid_executor = concurrent.futures.ThreadPoolExecutor(
                        max_workers=isolated_workers,
                        thread_name_prefix="HybridIso",
                    )
                    for isolated_variant in isolated_precompute_ordinals:
                        isolated_request = _current_remaining_request_for_variant(
                            current_remaining_hybrid_request,
                            isolated_variant,
                        )
                        if isolated_request is None:
                            continue
                        isolated_idx = max(0, int(isolated_variant) - 1)
                        isolated_runtime_payload = _variant_runtime_override_payload(
                            int(option_codes[isolated_idx]),
                            option_labels[isolated_idx] if isolated_idx < len(option_labels) else "",
                        )
                        isolated_current_remaining_hybrid_futures[int(isolated_variant)] = (
                            isolated_current_remaining_hybrid_executor.submit(
                                _build_shared_current_remaining_hybrid_for_parallel,
                                request=isolated_request,
                                runtime_payload=isolated_runtime_payload,
                                representative_variant=int(isolated_variant),
                                shared_variants=[int(isolated_variant)],
                            )
                        )
                    if isolated_current_remaining_hybrid_futures:
                        self.log_sig.emit(
                            "[REPLAN][METRIC] current_remaining_hybrid_isolated_precompute_enabled "
                            f"isolated_variants={'|'.join(str(int(v)) for v in sorted(isolated_current_remaining_hybrid_futures))} "
                            f"workers={int(isolated_workers)} "
                            f"reason={current_remaining_hybrid_share_policy.get('reason') or '-'} "
                            f"share_strategy={current_remaining_hybrid_share_policy.get('shareStrategy') or '-'}"
                        )
                store_commit_workers_budget = max(
                    1,
                    min(
                        int(plan_count),
                        int(runtime_parallel_config.get("replan_store_commit_workers") or 1),
                    ),
                )
                store_prepare_workers_budget = max(
                    1,
                    min(
                        int(plan_count),
                        int(runtime_parallel_config.get("replan_store_prepare_workers") or 1),
                    ),
                )
                store_json_write_workers_budget = max(
                    1,
                    int(runtime_parallel_config.get("replan_store_json_write_workers") or 2),
                )
                store_commit_peak_threads = int(store_commit_workers_budget) * int(
                    store_json_write_workers_budget
                )
                dependency_parallel_enabled = _runtime_bool_setting(
                    "replan_0303_dependency_parallel_enabled",
                    True,
                )
                dependency_workers_budget = max(
                    1,
                    _runtime_int_setting("replan_0303_dependency_workers", 3),
                ) if dependency_parallel_enabled else 1
                linesearch_inner_workers_budget = max(
                    1,
                    _runtime_int_setting("linesearch_inner_parallel_workers", 2),
                )
                formation_post_workers_budget = max(
                    1,
                    _runtime_int_setting("formation_follower_postprocess_workers", 2),
                )
                cpu_count = int(os.cpu_count() or 1)
                shared_hybrid_worker_count = 1 if share_current_remaining_hybrid else 0
                isolated_hybrid_worker_count = (
                    int(getattr(isolated_current_remaining_hybrid_executor, "_max_workers", 0) or 0)
                    if isolated_current_remaining_hybrid_futures
                    else 0
                )
                core_phase_peak_threads = (
                    int(max_workers)
                    + int(store_prepare_workers_budget)
                    + int(shared_hybrid_worker_count)
                    + int(isolated_hybrid_worker_count)
                )
                self.log_sig.emit(
                    "[REPLAN][METRIC] parallel_thread_budget "
                    f"variants={int(plan_count)} core_workers={int(max_workers)} "
                    f"store_prepare_workers={int(store_prepare_workers_budget)} "
                    f"store_prepare_out_of_order={int(bool(runtime_parallel_config.get('replan_store_prepare_out_of_order')))} "
                    f"store_commit_workers={int(store_commit_workers_budget)} "
                    f"store_json_write_workers={int(store_json_write_workers_budget)} "
                    f"shared_hybrid_workers={int(shared_hybrid_worker_count)} "
                    f"isolated_hybrid_workers={int(isolated_hybrid_worker_count)} "
                    f"load_imp_fixed_cap=6 mp0301_fixed_cap=6 save0302_fixed_cap=8 "
                    f"dependency_parallel_enabled={int(bool(dependency_parallel_enabled))} "
                    f"dependency_workers={int(dependency_workers_budget)} "
                    f"linesearch_inner_workers={int(linesearch_inner_workers_budget)} "
                    f"formation_post_workers={int(formation_post_workers_budget)} "
                    f"cpu_count={int(cpu_count)} "
                    f"core_phase_peak_threads={int(core_phase_peak_threads)} "
                    f"store_commit_peak_threads={int(store_commit_peak_threads)} "
                    f"peak_configured_threads={max(int(core_phase_peak_threads), int(store_commit_peak_threads))}"
                )
                parallel_specs = []
                waypoint_blocks: list[tuple[int, int]] = []
                waypoint_reserve_t0 = time.perf_counter()
                waypoint_bulk_used = False
                waypoint_fallback_count = 0
                try:
                    if callable(reserve_waypoint_blocks):
                        waypoint_blocks = list(
                            reserve_waypoint_blocks([waypoint_block_size] * (int(plan_count) * 2))
                            or []
                        )
                        waypoint_bulk_used = len(waypoint_blocks) >= int(plan_count) * 2
                except Exception as exc:
                    waypoint_blocks = []
                    self.log_sig.emit(f"[WARN] bulk waypointID block reservation failed; fallback to legacy: {exc}")
                for idx in range(plan_count):
                    variant_current_remaining_request = _current_remaining_request_for_variant(
                        current_remaining_hybrid_request,
                        idx + 1,
                    )
                    block_idx = int(idx) * 2
                    if len(waypoint_blocks) >= block_idx + 2:
                        waypoint_block_0303_start, waypoint_block_0303_end = waypoint_blocks[block_idx]
                        waypoint_block_0304_start, waypoint_block_0304_end = waypoint_blocks[block_idx + 1]
                    else:
                        waypoint_block_0303_start = int(reserve_waypoint_block(waypoint_block_size))
                        waypoint_block_0303_end = waypoint_block_0303_start + waypoint_block_size - 1
                        waypoint_block_0304_start = int(reserve_waypoint_block(waypoint_block_size))
                        waypoint_block_0304_end = waypoint_block_0304_start + waypoint_block_size - 1
                        waypoint_fallback_count += 2
                    variant_ordinal = idx + 1
                    current_hybrid_future = (
                        shared_current_remaining_hybrid_future
                        if variant_ordinal in set(shared_current_remaining_ordinals)
                        else isolated_current_remaining_hybrid_futures.get(variant_ordinal)
                    )
                    current_hybrid_role = (
                        "non_recon_shared"
                        if variant_ordinal in set(shared_current_remaining_ordinals)
                        else (
                            "isolated_precompute"
                            if variant_ordinal in isolated_current_remaining_hybrid_futures
                            else "isolated"
                        )
                    )
                    parallel_specs.append(
                        {
                            "idx": idx,
                            "variant_no": variant_ordinal,
                            "requested_plan_id": plan_ids[idx],
                            "option_code": int(option_codes[idx]),
                            "cmpk_source_path": Path(cmpk_path),
                            "runtime_payload": _variant_runtime_override_payload(
                                int(option_codes[idx]),
                                option_labels[idx] if idx < len(option_labels) else "",
                            ),
                            "current_remaining_hybrid_request": variant_current_remaining_request,
                            "shared_current_remaining_hybrid_future": current_hybrid_future,
                            "shared_current_remaining_hybrid_role": current_hybrid_role,
                            "waypoint_block_0303_start": waypoint_block_0303_start,
                            "waypoint_block_0303_end": waypoint_block_0303_end,
                            "waypoint_block_0304_start": waypoint_block_0304_start,
                            "waypoint_block_0304_end": waypoint_block_0304_end,
                        }
                    )
                    self.log_sig.emit(
                        f"[INFO] variant {idx + 1} waypointID blocks reserved: "
                        f"0303={waypoint_block_0303_start}-{waypoint_block_0303_end}, "
                        f"0304={waypoint_block_0304_start}-{waypoint_block_0304_end}"
                    )
                waypoint_reserve_ms = (time.perf_counter() - waypoint_reserve_t0) * 1000.0
                self._record_replan_timing_event(
                    "waypoint_blocks_reserved",
                    extra={
                        "blocks": len(parallel_specs) * 2,
                        "bulk": int(bool(waypoint_bulk_used)),
                        "fallback_count": int(waypoint_fallback_count),
                        "block_size": int(waypoint_block_size),
                        "reserve_wall_ms": round(float(waypoint_reserve_ms), 3),
                    },
                )
                self._record_replan_timing_event(
                    "parallel_specs_ready",
                    extra={
                        "specs": len(parallel_specs),
                        "workers": int(max_workers),
                    },
                )
                parallel_results: Dict[int, Dict[str, Any]] = {}
                prepared_store_results: Dict[int, Dict[str, Any]] = {}
                committed_store_results: Dict[int, Dict[str, Any]] = {}
                store_prepare_future_map: Dict[concurrent.futures.Future, int] = {}
                store_commit_future_map: Dict[concurrent.futures.Future, int] = {}
                store_prepare_submitted_indices: Set[int] = set()
                store_commit_submitted_indices: Set[int] = set()
                next_store_prepare_idx = 0
                store_prepare_executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
                store_commit_executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
                try:
                    store_prepare_workers = max(
                        1,
                        min(
                            int(plan_count),
                            int(runtime_parallel_config.get("replan_store_prepare_workers") or 1),
                        ),
                    )
                    store_prepare_out_of_order = bool(
                        runtime_parallel_config.get("replan_store_prepare_out_of_order")
                    )
                    store_prepare_executor = concurrent.futures.ThreadPoolExecutor(
                        max_workers=store_prepare_workers,
                        thread_name_prefix="StorePrepare",
                    )
                    store_commit_workers = max(
                        1,
                        min(
                            int(plan_count),
                            int(runtime_parallel_config.get("replan_store_commit_workers") or 1),
                        ),
                    )
                    store_commit_executor = concurrent.futures.ThreadPoolExecutor(
                        max_workers=store_commit_workers,
                        thread_name_prefix="StoreCommit",
                    )
                    store_path_id_cross_variant_bulk = bool(
                        runtime_parallel_config.get("replan_store_path_id_cross_variant_bulk")
                    )
                    cross_path_id_reservation_done = not store_path_id_cross_variant_bulk
                    cross_path_id_reservation_ms = 0.0

                    def _ensure_cross_variant_path_id_reservations() -> bool:
                        nonlocal cross_path_id_reservation_done, cross_path_id_reservation_ms
                        if cross_path_id_reservation_done:
                            return True
                        if len(parallel_results) < len(parallel_specs):
                            return False

                        reservation_started = time.perf_counter()
                        plans_by_idx: Dict[int, Dict[int, int]] = {}
                        aggregate_counts: Dict[int, int] = {}
                        imp_counts_by_idx: Dict[int, int] = {}
                        individual_counts_by_idx: Dict[int, int] = {}
                        for idx_to_reserve in sorted(parallel_results):
                            result_to_reserve = parallel_results[idx_to_reserve]
                            if "reserved_path_ids_by_aircraft" not in result_to_reserve:
                                reservation_plan = _fresh_path_id_reservation_plan(
                                    result_to_reserve.get("missions") or []
                                )
                                plans_by_idx[int(idx_to_reserve)] = dict(reservation_plan)
                                for aid, count in reservation_plan.items():
                                    aggregate_counts[int(aid)] = aggregate_counts.get(int(aid), 0) + int(count)
                            if "reserved_imp_ids" not in result_to_reserve:
                                imp_counts_by_idx[int(idx_to_reserve)] = _imp_id_reservation_count(
                                    result_to_reserve.get("mp_json") or {}
                                )
                            if "reserved_individual_mission_ids" not in result_to_reserve:
                                individual_counts_by_idx[int(idx_to_reserve)] = _individual_mission_reservation_count(
                                    result_to_reserve.get("missions") or []
                                )

                        reserve_started = time.perf_counter()
                        total_imp_count = sum(int(count) for count in imp_counts_by_idx.values())
                        total_individual_count = sum(int(count) for count in individual_counts_by_idx.values())
                        reserved_bundle = reserve_replan_id_bundle(
                            path_count_by_aircraft={
                                int(aid): int(count)
                                for aid, count in aggregate_counts.items()
                                if int(count) > 0
                            },
                            imp_count=int(total_imp_count),
                            individual_mission_count=int(total_individual_count),
                        )
                        bulk_reserved: Dict[int, list[int]] = {
                            int(aid): [int(value) for value in (values or [])]
                            for aid, values in (reserved_bundle.get("pathID") or {}).items()
                        }
                        reserved_imp_ids_all = [
                            int(value)
                            for value in (reserved_bundle.get("individualMissionPackage") or [])
                        ]
                        reserved_individual_ids_all = [
                            int(value)
                            for value in (reserved_bundle.get("individualMission") or [])
                        ]
                        if len(reserved_imp_ids_all) < int(total_imp_count):
                            raise RuntimeError(
                                "cross-variant IMP ID bundle reservation short "
                                f"(required={total_imp_count}, reserved={len(reserved_imp_ids_all)})"
                            )
                        if len(reserved_individual_ids_all) < int(total_individual_count):
                            raise RuntimeError(
                                "cross-variant individualMission ID bundle reservation short "
                                f"(required={total_individual_count}, reserved={len(reserved_individual_ids_all)})"
                            )
                        reserve_ms = (time.perf_counter() - reserve_started) * 1000.0

                        cursors: Dict[int, int] = {int(aid): 0 for aid in aggregate_counts}
                        for idx_to_reserve in sorted(plans_by_idx):
                            assigned: Dict[int, list[int]] = {}
                            for aid, count in sorted(plans_by_idx[idx_to_reserve].items()):
                                aid_int = int(aid)
                                count_int = int(count)
                                if count_int <= 0:
                                    continue
                                values = [int(value) for value in (bulk_reserved.get(aid_int) or [])]
                                cursor = int(cursors.get(aid_int, 0))
                                segment = values[cursor : cursor + count_int]
                                if len(segment) < count_int:
                                    raise RuntimeError(
                                        "cross-variant pathID reservation short "
                                        f"(aircraftID={aid_int}, required={count_int}, "
                                        f"reserved={len(values)}, cursor={cursor})"
                                    )
                                assigned[aid_int] = segment
                                cursors[aid_int] = cursor + count_int
                            parallel_results[idx_to_reserve]["reserved_path_ids_by_aircraft"] = assigned
                            parallel_results[idx_to_reserve]["path_id_reservation_ms"] = 0.0
                            parallel_results[idx_to_reserve]["cross_path_id_reservation_ms"] = float(reserve_ms)
                            parallel_results[idx_to_reserve]["cross_path_id_reservation_shared"] = True
                        imp_cursor = 0
                        for idx_to_reserve in sorted(imp_counts_by_idx):
                            count = int(imp_counts_by_idx[idx_to_reserve])
                            segment = reserved_imp_ids_all[imp_cursor : imp_cursor + count]
                            if len(segment) < count:
                                raise RuntimeError(
                                    "cross-variant IMP ID reservation short "
                                    f"(variantIndex={idx_to_reserve}, required={count}, reserved={len(reserved_imp_ids_all)})"
                                )
                            parallel_results[idx_to_reserve]["reserved_imp_ids"] = segment
                            imp_cursor += count
                        individual_cursor = 0
                        for idx_to_reserve in sorted(individual_counts_by_idx):
                            count = int(individual_counts_by_idx[idx_to_reserve])
                            segment = reserved_individual_ids_all[individual_cursor : individual_cursor + count]
                            if len(segment) < count:
                                raise RuntimeError(
                                    "cross-variant individualMission ID reservation short "
                                    f"(variantIndex={idx_to_reserve}, required={count}, reserved={len(reserved_individual_ids_all)})"
                                )
                            parallel_results[idx_to_reserve]["reserved_individual_mission_ids"] = segment
                            individual_cursor += count

                        total_ms = (time.perf_counter() - reservation_started) * 1000.0
                        cross_path_id_reservation_ms = float(total_ms)
                        cross_path_id_reservation_done = True
                        self.log_sig.emit(
                            "[REPLAN][METRIC] general_variant_cross_path_id_reservation "
                            f"variants={len(plans_by_idx)} "
                            f"aircraft={len(aggregate_counts)} "
                            f"totalCount={sum(int(value) for value in aggregate_counts.values())} "
                            f"impCount={total_imp_count} "
                            f"individualMissionCount={total_individual_count} "
                            f"reserve_ms={reserve_ms:.3f} "
                            f"total_ms={total_ms:.3f} "
                            f"counts="
                            + "|".join(
                                f"{int(aid)}:{int(count)}"
                                for aid, count in sorted(aggregate_counts.items())
                            )
                        )
                        return True

                    def _queue_store_prepare(idx_to_submit: int) -> None:
                        if store_prepare_executor is None or idx_to_submit in store_prepare_submitted_indices:
                            return
                        result_to_prepare = parallel_results[idx_to_submit]
                        ready_perf = float(
                            result_to_prepare.get("store_core_result_ready_perf")
                            or time.perf_counter()
                        )
                        queue_ready_perf = time.perf_counter()
                        queue_wait_ms = max(0.0, (queue_ready_perf - ready_perf) * 1000.0)
                        reserve_t0 = time.perf_counter()
                        if "reserved_path_ids_by_aircraft" not in result_to_prepare:
                            if store_path_id_cross_variant_bulk:
                                result_to_prepare["reserved_path_ids_by_aircraft"] = {}
                            else:
                                result_to_prepare["reserved_path_ids_by_aircraft"] = _reserve_fresh_path_ids_for_missions(
                                    result_to_prepare.get("missions") or []
                                )
                        reserve_ms = (time.perf_counter() - reserve_t0) * 1000.0
                        queued_perf = time.perf_counter()
                        enqueue_total_wait_ms = max(0.0, (queued_perf - ready_perf) * 1000.0)
                        result_to_prepare["store_prepare_wait_for_order_ms"] = float(queue_wait_ms)
                        result_to_prepare["store_prepare_queue_wait_ms"] = float(queue_wait_ms)
                        result_to_prepare["store_prepare_enqueue_total_wait_ms"] = float(enqueue_total_wait_ms)
                        result_to_prepare["store_prepare_queued_perf"] = float(queued_perf)
                        if not bool(result_to_prepare.get("cross_path_id_reservation_shared")):
                            result_to_prepare["path_id_reservation_ms"] = float(reserve_ms)
                        result_to_prepare["store_json_write_workers"] = int(
                            runtime_parallel_config.get("replan_store_json_write_workers") or 2
                        )
                        result_to_prepare["defer_waypoint_files_written_mark"] = True
                        result_to_prepare["defer_snapshot_carry_forward"] = bool(
                            runtime_parallel_config.get("replan_store_snapshot_post_delivery")
                        )
                        store_prepare_future_map[
                            store_prepare_executor.submit(
                                _prepare_general_variant_store,
                                result_to_prepare,
                            )
                        ] = idx_to_submit
                        store_prepare_submitted_indices.add(idx_to_submit)
                        self.log_sig.emit(
                            "[REPLAN][METRIC] general_variant_store_prepare_queued "
                            f"variant={int(result_to_prepare.get('variant_no') or (idx_to_submit + 1))} "
                            f"option={int(result_to_prepare.get('option_code') or 0)} "
                            f"idx={idx_to_submit} wait_for_order_ms={queue_wait_ms:.3f} "
                            f"queue_wait_ms={queue_wait_ms:.3f} "
                            f"enqueue_total_wait_ms={enqueue_total_wait_ms:.3f} "
                            f"path_id_reservation_ms={float(result_to_prepare.get('path_id_reservation_ms') or reserve_ms):.3f} "
                            f"cross_path_id_reservation_ms={float(result_to_prepare.get('cross_path_id_reservation_ms') or 0.0):.3f} "
                            f"cross_path_id_reservation_shared={int(bool(result_to_prepare.get('cross_path_id_reservation_shared')))} "
                            f"workers={int(store_prepare_workers)} "
                            f"out_of_order={int(store_prepare_out_of_order)}"
                        )

                    def _submit_ready_store_prepare() -> None:
                        nonlocal next_store_prepare_idx
                        if store_prepare_executor is None:
                            return
                        if store_path_id_cross_variant_bulk and not _ensure_cross_variant_path_id_reservations():
                            return
                        if store_prepare_out_of_order:
                            for idx_to_submit in sorted(parallel_results):
                                _queue_store_prepare(int(idx_to_submit))
                            return
                        while next_store_prepare_idx in parallel_results:
                            idx_to_submit = int(next_store_prepare_idx)
                            _queue_store_prepare(idx_to_submit)
                            next_store_prepare_idx += 1

                    def _queue_store_commit(idx_to_submit: int, staged: Dict[str, Any]) -> None:
                        if store_commit_executor is None or idx_to_submit in store_commit_submitted_indices:
                            return
                        store_commit_future_map[
                            store_commit_executor.submit(
                                _write_general_variant_store_files,
                                staged,
                            )
                        ] = int(idx_to_submit)
                        store_commit_submitted_indices.add(int(idx_to_submit))
                        self.log_sig.emit(
                            "[REPLAN][METRIC] general_variant_store_commit_queued "
                            f"variant={int(staged.get('variant_no') or (idx_to_submit + 1))} "
                            f"option={int(staged.get('option_code') or 0)} "
                            f"idx={idx_to_submit} workers={store_commit_workers} "
                            f"json_write_workers={int(staged.get('store_json_write_workers') or 2)} "
                            "streaming=1"
                        )

                    def _record_store_prepare_failure(idx: int, exc: BaseException) -> None:
                        result = parallel_results.get(idx) or {}
                        self._record_replan_timing_event(
                            "variant_finished",
                            extra={
                                "variant": int(result.get("variant_no") or (idx + 1)),
                                "option": int(result.get("option_code") or 0),
                                "mode": "parallel_store_prepare",
                                "status": "failed",
                                "code": type(exc).__name__,
                            },
                        )

                    def _record_store_commit_failure(idx: int, exc: BaseException) -> None:
                        result = parallel_results.get(idx) or {}
                        self._record_replan_timing_event(
                            "variant_finished",
                            extra={
                                "variant": int(result.get("variant_no") or (idx + 1)),
                                "option": int(result.get("option_code") or 0),
                                "mode": "parallel_store_commit",
                                "status": "failed",
                                "code": type(exc).__name__,
                            },
                        )

                    def _drain_store_commit_futures(
                        *,
                        wait_all: bool,
                        failures: list[BaseException] | None = None,
                    ) -> None:
                        futures = list(store_commit_future_map)
                        if not futures:
                            return
                        iterable = concurrent.futures.as_completed(futures) if wait_all else [f for f in futures if f.done()]
                        for commit_future in iterable:
                            if commit_future not in store_commit_future_map:
                                continue
                            idx = int(store_commit_future_map.pop(commit_future))
                            try:
                                committed_store_results[idx] = commit_future.result()
                            except concurrent.futures.CancelledError:
                                continue
                            except Exception as exc:
                                _record_store_commit_failure(idx, exc)
                                if failures is not None:
                                    failures.append(exc)

                    def _drain_store_prepare_futures(
                        *,
                        wait_all: bool,
                        prepare_failures: list[BaseException],
                        commit_failures: list[BaseException],
                    ) -> None:
                        futures = list(store_prepare_future_map)
                        if not futures:
                            return
                        iterable = concurrent.futures.as_completed(futures) if wait_all else [f for f in futures if f.done()]
                        for prepare_future in iterable:
                            if prepare_future not in store_prepare_future_map:
                                continue
                            idx = int(store_prepare_future_map.pop(prepare_future))
                            try:
                                staged = prepare_future.result()
                                prepared_store_results[idx] = staged
                                if not prepare_failures and not commit_failures:
                                    _queue_store_commit(idx, staged)
                            except concurrent.futures.CancelledError:
                                continue
                            except Exception as exc:
                                _record_store_prepare_failure(idx, exc)
                                prepare_failures.append(exc)

                    def _cancel_streaming_store_work_and_cleanup() -> None:
                        for pending_prepare in list(store_prepare_future_map):
                            try:
                                pending_prepare.cancel()
                            except Exception:
                                pass
                        for pending_commit in list(store_commit_future_map):
                            try:
                                pending_commit.cancel()
                            except Exception:
                                pass
                        if store_commit_executor is not None:
                            try:
                                store_commit_executor.shutdown(wait=True, cancel_futures=True)
                            except Exception:
                                pass
                        _drain_store_commit_futures(wait_all=True, failures=None)
                        if committed_store_results:
                            _cleanup_committed_store_files(list(committed_store_results.values()))

                    with concurrent.futures.ThreadPoolExecutor(
                        max_workers=max_workers,
                        thread_name_prefix="PlanVariant",
                    ) as executor:
                        store_prepare_failures: list[BaseException] = []
                        store_commit_failures: list[BaseException] = []
                        future_map = {
                            executor.submit(
                                call_with_source_artifact_cache,
                                source_cache,
                                _run_general_variant_core,
                                spec,
                            ): spec
                            for spec in parallel_specs
                        }
                        self._record_replan_timing_event(
                            "parallel_executor_submitted",
                            extra={
                                "futures": len(future_map),
                                "workers": int(max_workers),
                            },
                        )
                        for future in concurrent.futures.as_completed(future_map):
                            spec = future_map[future]
                            try:
                                result = future.result()
                                _flush_parallel_variant_diagnostics(result)
                                result["store_core_result_ready_perf"] = time.perf_counter()
                                parallel_results[int(spec["idx"])] = result
                                _submit_ready_store_prepare()
                                _drain_store_prepare_futures(
                                    wait_all=False,
                                    prepare_failures=store_prepare_failures,
                                    commit_failures=store_commit_failures,
                                )
                                _drain_store_commit_futures(
                                    wait_all=False,
                                    failures=store_commit_failures,
                                )
                            except _VariantCoreError as exc:
                                self._record_replan_timing_event(
                                    "variant_finished",
                                    extra={
                                        "variant": exc.variant_no,
                                        "option": int(spec.get("option_code") or 0),
                                        "mode": "parallel_core",
                                        "status": "failed",
                                        "code": exc.code,
                                    },
                                )
                                for pending in future_map:
                                    if pending is not future:
                                        pending.cancel()
                                _record_issue(exc.code, exc.message, detail={"variant": exc.variant_no, **exc.detail}, status="error")
                                plan_log_summary.update({"stop_reason": exc.code, "variant": exc.variant_no, **exc.detail})
                                _notify_failure_once(exc.code, detail={"variant": exc.variant_no, **exc.detail})
                                self._plan_status = "임무계획 실패"
                                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                                _cancel_streaming_store_work_and_cleanup()
                                return
                            except Exception as exc:
                                self._record_replan_timing_event(
                                    "variant_finished",
                                    extra={
                                        "variant": int(spec.get("variant_no") or 0),
                                        "option": int(spec.get("option_code") or 0),
                                        "mode": "parallel_core",
                                        "status": "failed",
                                        "code": type(exc).__name__,
                                    },
                                )
                                raise
                            if store_prepare_failures:
                                for pending in future_map:
                                    if pending is not future:
                                        pending.cancel()
                                _cancel_streaming_store_work_and_cleanup()
                                raise store_prepare_failures[0]
                            if store_commit_failures:
                                for pending in future_map:
                                    if pending is not future:
                                        pending.cancel()
                                _cancel_streaming_store_work_and_cleanup()
                                raise store_commit_failures[0]
                    _drain_store_prepare_futures(
                        wait_all=True,
                        prepare_failures=store_prepare_failures,
                        commit_failures=store_commit_failures,
                    )
                    if store_prepare_failures:
                        _cancel_streaming_store_work_and_cleanup()
                        raise store_prepare_failures[0]
                    _drain_store_commit_futures(
                        wait_all=True,
                        failures=store_commit_failures,
                    )
                    if store_commit_failures:
                        _cancel_streaming_store_work_and_cleanup()
                        raise store_commit_failures[0]
                    deferred_max_waypoint_id = max(
                        (
                            int(row.get("max_waypoint_id") or 0)
                            for row in committed_store_results.values()
                            if isinstance(row, dict)
                        ),
                        default=0,
                    )
                    post_delivery_waypoint_mark = {
                        "max_waypoint_id": int(deferred_max_waypoint_id),
                        "variants": int(plan_count),
                        "reason": "general_3_option_parallel_store",
                    }
                    self.log_sig.emit(
                        "[REPLAN][METRIC] general_variant_store_deferred_waypoint_mark "
                        f"variants={int(plan_count)} "
                        f"max_waypoint_id={int(deferred_max_waypoint_id)} "
                        "mode=post_delivery"
                    )
                    for idx in range(plan_count):
                        try:
                            _finalize_general_variant_store(committed_store_results[idx])
                        except Exception as exc:
                            result = parallel_results.get(idx) or {}
                            self._record_replan_timing_event(
                                "variant_finished",
                                extra={
                                    "variant": int(result.get("variant_no") or (idx + 1)),
                                    "option": int(result.get("option_code") or 0),
                                    "mode": "parallel_store_finalize",
                                    "status": "failed",
                                    "code": type(exc).__name__,
                                },
                            )
                            raise
                    snapshot_carry_items = [
                        dict(item)
                        for idx in range(plan_count)
                        for item in [
                            (parallel_results.get(idx) or {}).get(
                                "post_delivery_snapshot_carry_forward_item"
                            )
                        ]
                        if isinstance(item, dict)
                    ]
                    if snapshot_carry_items:
                        post_delivery_snapshot_carry_forward = {
                            "items": snapshot_carry_items,
                            "reason": "general_3_option_parallel_store",
                        }
                        self.log_sig.emit(
                            "[REPLAN][METRIC] general_variant_store_deferred_snapshot_carry_forward "
                            f"variants={int(plan_count)} items={len(snapshot_carry_items)} mode=post_delivery"
                        )
                    if (
                        int(plan_count) >= 3
                        and str(((ctx.get("_replan_timing") or {}).get("trigger") if isinstance(ctx, dict) else "") or "")
                        == "general_3_option"
                    ):
                        _record_general_3_option_summary(parallel_results)
                    variant_loop_indices = range(0)
                except Exception as exc:
                    self.log_sig.emit(f"[ERR] 일반 재계획 병렬 생성 실패: {exc}")
                    raise
                finally:
                    if store_prepare_executor is not None:
                        try:
                            store_prepare_executor.shutdown(wait=True, cancel_futures=True)
                        except Exception:
                            pass
                    if store_commit_executor is not None:
                        try:
                            store_commit_executor.shutdown(wait=True, cancel_futures=True)
                        except Exception:
                            pass
                    if shared_current_remaining_hybrid_executor is not None:
                        try:
                            shared_current_remaining_hybrid_executor.shutdown(wait=True, cancel_futures=True)
                        except Exception:
                            pass
                    if isolated_current_remaining_hybrid_executor is not None:
                        try:
                            isolated_current_remaining_hybrid_executor.shutdown(wait=True, cancel_futures=True)
                        except Exception:
                            pass

            for idx in variant_loop_indices:
                variant_no = idx + 1
                requested_plan_id = plan_ids[idx]
                option_code = option_codes[idx]
                option_label = option_labels[idx] if idx < len(option_labels) else ""
                variant_generated_path_ids: Set[int] = set()
                option_dependent_isolation = _option_dependent_isolation_contract(
                    variant_no=variant_no,
                    option_code=int(option_code),
                    mode="sequential",
                )
                variant_current_remaining_request = _current_remaining_request_for_variant(
                    current_remaining_hybrid_request,
                    variant_no,
                )
                attack_exclusion_selected = idx in attack_exclusion_option_indices
                cmpk_source_path = cmpk_path
                variant_attack_context: Optional[Dict[str, Any]] = None
                variant_prior_context: Optional[Dict[str, Any]] = None
                attack_option_selected = idx in attack_option_indices
                if attack_exclusion_selected:
                    parallel_exclusion_used = (
                        attack_exclusion_future is not None
                        and not bool(attack_exclusion_parallel_info.get("consumed"))
                    )
                    self._record_replan_timing_event(
                        "variant_started",
                        extra={
                            "variant": variant_no,
                            "option": int(option_code),
                            "mode": "attack_exclusion",
                        },
                    )
                    if parallel_exclusion_used:
                        self.log_sig.emit(f"[variant {variant_no}] 공격 배제 병렬 결과 병합")
                        wait_t0 = time.perf_counter()
                        try:
                            max_wait_s = _attack_exclusion_parallel_max_wait_s()
                            if max_wait_s is None:
                                parallel_payload = attack_exclusion_future.result()
                            else:
                                parallel_payload = attack_exclusion_future.result(timeout=max_wait_s)
                        except concurrent.futures.TimeoutError:
                            wait_ms = (time.perf_counter() - wait_t0) * 1000.0
                            attack_exclusion_parallel_info["consumed"] = True
                            attack_exclusion_parallel_info["deferred"] = True
                            try:
                                cancelled = bool(attack_exclusion_future.cancel())
                            except Exception:
                                cancelled = False
                            if attack_exclusion_executor is not None:
                                try:
                                    attack_exclusion_executor.shutdown(wait=False, cancel_futures=True)
                                except Exception:
                                    pass
                                attack_exclusion_executor = None
                            self.log_sig.emit(
                                "[ATTACK-EXCLUDE] parallel result timed out; "
                                f"skip attack-exclusion option for immediate attack delivery "
                                f"(variant={variant_no}, wait_ms={wait_ms:.1f}, cancelled={int(bool(cancelled))})."
                            )
                            self._record_replan_timing_event(
                                "attack_exclusion_parallel_timeout_skipped",
                                extra={
                                    "variant": variant_no,
                                    "option": int(option_code),
                                    "wait_ms": round(wait_ms, 3),
                                    "max_wait_ms": round(float(max_wait_s or 0.0) * 1000.0, 3),
                                    "cancelled": bool(cancelled),
                                },
                            )
                            plan_log_summary.setdefault("skipped_options", []).append(
                                {
                                    "variant": variant_no,
                                    "option": int(option_code),
                                    "mode": "attack_exclusion",
                                    "reason": "parallel_timeout",
                                }
                            )
                            continue
                        except Exception as exc:
                            wait_ms = (time.perf_counter() - wait_t0) * 1000.0
                            self.log_sig.emit(f"[ERR] 공격 배제 병렬 생성 실패: {exc}")
                            self._record_replan_timing_event(
                                "variant_finished",
                                extra={
                                    "variant": variant_no,
                                    "option": int(option_code),
                                    "mode": "attack_exclusion",
                                    "status": "failed",
                                    "code": type(exc).__name__,
                                    "wait_ms": round(wait_ms, 3),
                                },
                            )
                            _record_issue(
                                "attack_exclusion_failed",
                                f"공격 배제 병렬 생성 실패: {exc}",
                                detail={"variant": variant_no},
                            )
                            plan_log_summary.update({"stop_reason": "attack_exclusion_failed", "variant": variant_no})
                            _notify_failure_once("attack_exclusion_failed", detail={"variant": variant_no})
                            return
                        wait_ms = (time.perf_counter() - wait_t0) * 1000.0
                        attack_exclusion_parallel_info["consumed"] = True
                        parallel_valid, parallel_reject_reason = _validate_attack_exclusion_parallel_result(
                            parallel_payload
                        )
                        if not parallel_valid:
                            parallel_exclusion_used = False
                            self.log_sig.emit(
                                "[WARN] 공격 배제 병렬 결과 폐기 -> 순차 재생성: "
                                f"{parallel_reject_reason}"
                            )
                            self._record_replan_timing_event(
                                "attack_exclusion_parallel_discarded",
                                extra={
                                    "variant": variant_no,
                                    "reason": str(parallel_reject_reason),
                                    "wait_ms": round(wait_ms, 3),
                                },
                            )
                        else:
                            exclusion_result = (parallel_payload or {}).get("result") or {}
                            exclusion_ms = float((parallel_payload or {}).get("duration_ms") or 0.0)
                            for message in (exclusion_result.get("logMessages") or []):
                                self._append_log_line(f"[ATTACK-EXCLUDE] {message}")
                            self._record_replan_timing_event(
                                "attack_exclusion_parallel_joined",
                                extra={
                                    "variant": variant_no,
                                    "duration_ms": round(exclusion_ms, 3),
                                    "wait_ms": round(wait_ms, 3),
                                },
                            )
                        if attack_exclusion_executor is not None:
                            try:
                                attack_exclusion_executor.shutdown(wait=False, cancel_futures=False)
                            except Exception:
                                pass
                            attack_exclusion_executor = None
                    if not parallel_exclusion_used:
                        self.log_sig.emit(f"[variant {variant_no}] 공격 배제 전용 재개 파이프라인 적용")
                        exclusion_ctx = _filter_context_by_indices(ctx, [idx])
                        if attack_exclusion_source_plan_id is not None:
                            exclusion_ctx["sourceMissionPlanID"] = attack_exclusion_source_plan_id
                        exclusion_t0 = time.perf_counter()
                        exclusion_result = run_attack_exclusion_pipeline(
                            exclusion_ctx,
                            log_callback=self._append_log_line,
                        )
                        exclusion_ms = (time.perf_counter() - exclusion_t0) * 1000.0
                    exclusion_payload = (exclusion_result or {}).get("result") or {}
                    exclusion_plan_id = exclusion_payload.get("missionPlanID")
                    if exclusion_plan_id is None:
                        message = "공격 배제 재개 임무 생성에 실패했습니다."
                        if exclusion_payload.get("error") == "no_updates":
                            message = "공격 배제 재개 임무를 만들 수 있는 UAV가 없습니다."
                        self.log_sig.emit(f"[ERR] {message} (variant={variant_no})")
                        _record_issue(
                            "attack_exclusion_failed",
                            message,
                            detail={"variant": variant_no, "payload": exclusion_payload},
                        )
                        plan_log_summary.update({"stop_reason": "attack_exclusion_failed", "variant": variant_no})
                        _notify_failure_once("attack_exclusion_failed", detail={"variant": variant_no})
                        self._record_replan_timing_event(
                            "variant_finished",
                            extra={
                                "variant": variant_no,
                                "option": int(option_code),
                                "mode": "attack_exclusion",
                                "status": "failed",
                                "code": "attack_exclusion_failed",
                                "duration_ms": round(exclusion_ms, 3),
                            },
                        )
                        return
                    try:
                        generated_plan_ids.append(int(exclusion_plan_id))
                    except Exception:
                        generated_plan_ids.append(exclusion_plan_id)
                    option_codes_out.append(int(option_code))
                    self.log_sig.emit(
                        f"[OK] 공격 배제 variant 저장: MissionPlanID={exclusion_plan_id} "
                        f"(elapsed={exclusion_ms:.1f} ms)"
                    )
                    self.log_sig.emit(
                        f"[INFO] Option mapping #{variant_no}: planID={exclusion_plan_id}, "
                        f"optionCode={option_code}({option_code_to_label(option_code)})"
                    )
                    self._record_replan_timing_event(
                        "variant_finished",
                        extra={
                            "variant": variant_no,
                            "option": int(option_code),
                            "mode": "attack_exclusion",
                            "status": "success",
                            "duration_ms": round(exclusion_ms, 3),
                        },
                    )
                    continue
                if attack_option_selected and attack_cmpk_path is not None:
                    cmpk_source_path = attack_cmpk_path
                    self.log_sig.emit(
                        f"[variant {variant_no}] 공격 전용 0201 적용: {cmpk_source_path.name}"
                    )
                if attack_option_selected:
                    if shared_attack_context:
                        variant_attack_context = copy.deepcopy(shared_attack_context)
                    elif attack_cmpk_path is not None:
                        variant_attack_context = self._load_attack_context(cmpk_source_path)
                if idx in prior_variant_contexts:
                    prior_info = prior_variant_contexts[idx]
                    cmpk_source_path = prior_info["path"]
                    variant_prior_context = prior_info.get("context")
                    self.log_sig.emit(
                        f"[variant {variant_no}] 선행임무 0201 적용: {cmpk_source_path.name}"
                    )

                try:
                    source_key = str(Path(cmpk_source_path).resolve())
                except Exception:
                    source_key = str(cmpk_source_path)
                cached_filtered = filtered_cmpk_cache.get(source_key)
                if cached_filtered is not None:
                    cmpk_source_path = cached_filtered
                else:
                    try:
                        cmpk_variant_data = source_cache.read_json(cmpk_source_path, kind="InputMissionPlan")
                    except Exception as exc:
                        self.log_sig.emit(
                            f"[ERR] [variant {variant_no}] 0201 소스 로드 실패: {cmpk_source_path} ({exc})"
                        )
                        detail_payload = {"path": str(cmpk_source_path), "error": str(exc), "variant": variant_no}
                        _record_issue(
                            "variant_0201_load_failed",
                            f"variant {variant_no} failed to load source 0201",
                            detail=detail_payload,
                        )
                        _notify_failure_once("variant_0201_load_failed", exc=exc, detail=detail_payload)
                        self._plan_status = "임무계획 실패"
                        self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                        return

                    filtered_result = _filter_input_missions_payload(cmpk_variant_data)
                    if filtered_result is not None:
                        v_filtered = filtered_result.get("filtered_list") or []
                        v_removed = filtered_result.get("removed_ids") or []
                        v_converted = filtered_result.get("converted_ids") or []
                        v_width = filtered_result.get("width_adjusted_ids") or []
                        v_skipped_single_point = filtered_result.get("skipped_single_point_ids") or []
                        v_active = filtered_result.get("active_ids") or []
                        v_original_count = int(filtered_result.get("original_count") or 0)
                        v_snapshot_apply_result = filtered_result.get("snapshot_apply_result") or {}
                        v_snapshot_mutated = bool(filtered_result.get("snapshot_mutated"))
                        v_collapse_apply_result = filtered_result.get("collapse_apply_result") or {}
                        v_collapse_mutated = bool(filtered_result.get("collapse_mutated"))

                        if not v_filtered:
                            self.log_sig.emit(
                                f"[WARN] [variant {variant_no}] 필터 후 유효 임무 없음 (source={Path(cmpk_source_path).name})"
                            )
                            plan_log_status = "skipped"
                            _record_issue(
                                "variant_0201_filter_empty",
                                f"variant {variant_no} filtered 0201 has no missions",
                                detail={
                                    "variant": variant_no,
                                    "path": str(cmpk_source_path),
                                    "removed": v_removed,
                                    "mission_whitelist": sorted(mission_whitelist) if mission_whitelist else [],
                                },
                                status="skipped",
                            )
                            plan_log_summary.update(
                                {
                                    "stop_reason": "variant_0201_filter_empty",
                                    "variant": variant_no,
                                    "source_path": str(cmpk_source_path),
                                }
                            )
                            self._push_replan_noop_completion(reason, "재계획 불필요")
                            self._plan_status = "replan_skipped"
                            self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                            return

                        needs_variant_filter = (
                            v_snapshot_mutated
                            or
                            v_collapse_mutated
                            or
                            len(v_filtered) != v_original_count
                            or bool(v_converted)
                            or bool(v_width)
                            or (mission_whitelist and set(v_active) != mission_whitelist)
                        )
                        if needs_variant_filter:
                            cmpk_variant_data["inputMissionList"] = v_filtered
                            filtered_dir = out_root_base / "_filtered"
                            filtered_dir.mkdir(parents=True, exist_ok=True)
                            variant_filtered_path = filtered_dir / f"{Path(cmpk_source_path).stem}_v{variant_no:02d}.json"
                            try:
                                variant_filtered_path.write_text(
                                    json.dumps(cmpk_variant_data, ensure_ascii=False, indent=2),
                                    encoding="utf-8",
                                )
                                cmpk_source_path = variant_filtered_path
                                removed_summary = ", ".join(v_removed) if v_removed else "-"
                                converted_summary = ", ".join(v_converted) if v_converted else "-"
                                width_summary = ", ".join(v_width) if v_width else "-"
                                skipped_single_point_summary = ", ".join(v_skipped_single_point) if v_skipped_single_point else "-"
                                snapshot_summary = (
                                    f"updated={int(v_snapshot_apply_result.get('applied') or 0)}, "
                                    f"done={int(v_snapshot_apply_result.get('marked_done') or 0)}"
                                    if v_snapshot_mutated
                                    else "-"
                                )
                                collapse_summary = (
                                    f"groups={int(v_collapse_apply_result.get('groupCount') or 0)}, "
                                    f"normalized={','.join(str(v) for v in (v_collapse_apply_result.get('normalizedInputMissionIDs') or [])) or '-'}"
                                    if v_collapse_mutated
                                    else "-"
                                )
                                self.log_sig.emit(
                                    f"[variant {variant_no}] 0201 필터 적용 "
                                    f"(removed={removed_summary}, converted={converted_summary}, "
                                    f"widthAdjusted={width_summary}, skippedSinglePoint={skipped_single_point_summary}, "
                                    f"snapshot={snapshot_summary}, collapse={collapse_summary})"
                                )
                            except Exception as exc:
                                self.log_sig.emit(
                                    f"[WARN] [variant {variant_no}] 필터된 0201 저장 실패: {exc}"
                                )
                            else:
                                _persist_internal_replan_input_snapshot(
                                    cmpk_variant_data,
                                    f"variant_{variant_no:02d}",
                                )
                    filtered_cmpk_cache[source_key] = Path(cmpk_source_path)

                iter_out_root = out_root_base / f'variant_{variant_no:02d}'
                if iter_out_root.exists():
                    shutil.rmtree(iter_out_root)
                iter_out_root.mkdir(parents=True, exist_ok=True)

                runtime_payload = _variant_runtime_override_payload(int(option_code), option_label)
                if runtime_payload:
                    if is_recon_specialized_option(option_code, option_label) and not input_refresh_context:
                        values = runtime_payload.get("values") if isinstance(runtime_payload.get("values"), dict) else {}
                        self.log_sig.emit(
                            f"[variant {variant_no}] 정찰특화 전용 runtime 적용 "
                            f"(areaWidth={float(values.get('enhanced_area_review_max_segment_m', 600.0)):.1f}m, "
                            f"areaFov={float(values.get('area_custom_fov_deg', 15.0)):.1f}deg, "
                            f"areaSep={float(values.get('default_sweep_separation_m', 1000.0)):.1f}m, "
                            f"turnR={float(values.get('dubins_turn_radius_m', 0.0)):.1f}m, "
                            f"splitCap={int(values.get('recon_area_review_max_split_count', 0) or 0)}, "
                            f"minSeg={float(values.get('recon_area_review_min_segment_m', 0.0)):.1f}m)"
                        )
                    elif input_refresh_context:
                        self.log_sig.emit(
                            f"[variant {variant_no}] inputRefresh 초기계획 경로 형상 적용"
                        )
                    else:
                        self.log_sig.emit(f"[variant {variant_no}] 일반 재계획 모드 적용")

                trust_input_aircraft_for_variant = _trust_input_aircraft_for_replan()
                initial_template_key: str | None = None
                initial_template_payload: Dict[str, Any] | None = None
                initial_template_meta: Dict[str, Any] = {}
                initial_template_enabled_for_variant = _initial_plan_template_allowed(
                    reason_text=reason,
                    plan_count_value=plan_count,
                    option_code_value=int(option_code),
                    attack_selected=bool(attack_option_selected),
                    attack_exclusion_selected_value=bool(attack_exclusion_selected),
                    prior_context=variant_prior_context,
                    current_request=variant_current_remaining_request,
                )
                if initial_template_enabled_for_variant:
                    try:
                        initial_template_key = make_initial_plan_template_key(
                            cmpk_path=cmpk_source_path,
                            mrpk_path=mrpk_path,
                            runtime_payload=runtime_payload,
                            option_code=int(option_code),
                            trust_input_aircraft=bool(trust_input_aircraft_for_variant),
                        )
                        initial_template_payload, initial_template_meta = get_initial_plan_template(initial_template_key)
                        if initial_template_payload is not None:
                            self.log_sig.emit(
                                "[REPLAN][CACHE] initial_plan_template_hit "
                                f"variant={int(variant_no)} "
                                f"source={str(initial_template_meta.get('source') or 'memory')} "
                                f"elapsed_ms={float(initial_template_meta.get('elapsedMs') or 0.0):.3f}"
                            )
                    except Exception as exc:
                        initial_template_key = None
                        initial_template_payload = None
                        initial_template_meta = {"enabled": False, "hit": False, "error": str(exc)}
                        self.log_sig.emit(f"[WARN] initial plan template cache lookup failed: {exc}")

                with runtime_settings_override(runtime_payload):
                    variant_start = time.perf_counter()
                    variant_phase_ms: Dict[str, float] = {}
                    self._record_replan_timing_event(
                        "variant_started",
                        extra={
                            "variant": variant_no,
                            "option": int(option_code),
                            "mode": "sequential",
                        },
                    )
                    missions_before_path_ids_for_template: list[dict] | None = None
                    mp_json_template_for_cache: Dict[str, Any] | None = None
                    if initial_template_payload is not None:
                        step_t0 = time.perf_counter()
                        mp_json = copy.deepcopy(initial_template_payload.get("mp_json") or {})
                        missions = copy.deepcopy(initial_template_payload.get("missions") or [])
                        if not isinstance(mp_json, dict) or not missions:
                            self.log_sig.emit("[WARN] initial plan template invalid; falling back to full build")
                            initial_template_payload = None
                        else:
                            imp_id_map = {
                                a.get("aircraftID"): a.get("individualMissionPackageID")
                                for a in mp_json.get("aircraftList", [])
                                if isinstance(a, dict)
                            }
                            step_ms = (time.perf_counter() - step_t0) * 1000.0
                            variant_phase_ms["divide_and_pattern_ms"] = 0.0
                            variant_phase_ms["build_0301_load_ms"] = 0.0
                            variant_phase_ms["collect_missions_ms"] = float(step_ms)
                            self.log_sig.emit(
                                f"[TIME] divide_and_pattern (variant={variant_no}, template_cache): 0.0 ms"
                            )
                            self.log_sig.emit(
                                f"[TIME] build_0301+load (variant={variant_no}, template_cache): 0.0 ms"
                            )
                            self.log_sig.emit(
                                f"[TIME] collect_missions (variant={variant_no}, template_cache): "
                                f"{step_ms:.1f} ms, count={len(missions)}"
                            )

                    if initial_template_payload is None:
                        step_t0 = time.perf_counter()
                        self.log_sig.emit(f"[STEP 1.{variant_no}] Divide & Pattern start")
                        try:
                            imp_paths = run_divide_and_pattern(
                                str(cmpk_source_path),
                                str(mrpk_path),
                                str(iter_out_root),
                                log=lambda msg, n=variant_no: self.log_sig.emit(f"[variant {n}] {msg}"),
                                option_code=int(option_code),
                                trust_input_aircraft=trust_input_aircraft_for_variant,
                                shared_split_state=(
                                    input_refresh_shared_split_state
                                    if variant_current_remaining_request is None
                                    else None
                                ),
                            )
                        except Exception as exc:
                            self.log_sig.emit(f"[ERR] Divide & Pattern failed (variant={variant_no}): {exc}")
                            detail_payload = {"variant": variant_no, "error": str(exc)}
                            _record_issue(
                                "divide_and_pattern_exception",
                                f"Divide & Pattern failed (variant={variant_no})",
                                detail=detail_payload,
                            )
                            plan_log_summary.update(
                                {
                                    "stop_reason": "divide_and_pattern_exception",
                                    "variant": variant_no,
                                    "exception": str(exc),
                                }
                            )
                            _notify_failure_once(
                                "divide_and_pattern_exception",
                                exc=exc,
                                detail=detail_payload,
                            )
                            self._plan_status = "임무계획 실패"
                            self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                            return
                        if not imp_paths:
                            self.log_sig.emit(f"[ERR] IMP generation failed (variant={variant_no})")
                            _record_issue("imp_generation_failed", f"IMP generation failed (variant={variant_no})", status="error")
                            plan_log_summary.update({"stop_reason": "imp_generation_failed", "variant": variant_no})
                            _notify_failure_once("imp_generation_failed", detail={"variant": variant_no})
                            self._plan_status = "임무계획 실패"
                            self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                            self._record_replan_timing_event(
                                "variant_finished",
                                extra={
                                    "variant": variant_no,
                                    "option": int(option_code),
                                    "mode": "sequential",
                                    "status": "failed",
                                    "code": "imp_generation_failed",
                                    "duration_ms": round((time.perf_counter() - variant_start) * 1000.0, 3),
                                },
                            )
                            return
                        step_ms = (time.perf_counter() - step_t0) * 1000.0
                        variant_phase_ms["divide_and_pattern_ms"] = float(step_ms)
                        self.log_sig.emit(
                            f"[TIME] divide_and_pattern (variant={variant_no}): {step_ms:.1f} ms"
                        )
                        self.log_sig.emit(f"[OK] IMP generated: {len(imp_paths)} file(s) (variant={variant_no})")

                        step_t0 = time.perf_counter()
                        mp_tmp = iter_out_root / f"MissionPlan_{int(time.time()*1000)}.json"
                        mp_json = build_mission_plan_0301(
                            str(cmpk_source_path),
                            str(mrpk_path),
                            imp_paths,
                            str(mp_tmp),
                            mission_plan_id=0,
                        )
                        if not isinstance(mp_json, dict):
                            with mp_tmp.open(encoding='utf-8') as f:
                                mp_json = json.load(f)
                        imp_id_map = {a.get('aircraftID'): a.get('individualMissionPackageID') for a in mp_json.get('aircraftList', [])}
                        step_ms = (time.perf_counter() - step_t0) * 1000.0
                        variant_phase_ms["build_0301_load_ms"] = float(step_ms)
                        self.log_sig.emit(
                            f"[TIME] build_0301+load (variant={variant_no}): {step_ms:.1f} ms"
                        )
                        self.log_sig.emit(f"[OK] MissionPlan built: {mp_tmp.name} (variant={variant_no})")

                        step_t0 = time.perf_counter()
                        missions = []
                        loaded_imp_packages = _load_imp_packages(list(imp_paths))
                        for aid, pkg in loaded_imp_packages:
                            for im in pkg.get('individualMissionList', []):
                                im_copy = dict(im)
                                im_copy['aircraftID'] = aid
                                if 'individualMissionPlanPackageID' not in im_copy and imp_id_map:
                                    im_copy['individualMissionPlanPackageID'] = imp_id_map.get(aid)
                                missions.append(im_copy)
                        missions_before_path_ids_for_template = copy.deepcopy(missions)
                        mp_json_template_for_cache = copy.deepcopy(mp_json)
                        step_ms = (time.perf_counter() - step_t0) * 1000.0
                        variant_phase_ms["collect_missions_ms"] = float(step_ms)
                        self.log_sig.emit(
                            f"[TIME] collect_missions (variant={variant_no}): {step_ms:.1f} ms, count={len(missions)}"
                        )

                    handover_marked = _mark_handover_terminal_missions_from_path(
                        missions,
                        cmpk_source_path,
                    )
                    if handover_marked:
                        self.log_sig.emit(
                            f"[INFO] UAV control-transfer direct transit enabled for {handover_marked} mission row(s) "
                            f"(variant={variant_no})"
                        )

                    pre_path_snapshot = _snapshot_mission_path_ids(missions)
                    pid_map: Dict[tuple[int, int], int] = {}
                    if variant_current_remaining_request is None:
                        step_t0 = time.perf_counter()
                        reserved_path_ids_by_aircraft = _reserve_fresh_path_ids_for_missions(missions)
                        pid_map = _assign_fresh_path_ids(
                            missions,
                            variant_generated_path_ids,
                            reserved_path_ids_by_aircraft=reserved_path_ids_by_aircraft,
                        )
                        step_ms = (time.perf_counter() - step_t0) * 1000.0
                        variant_phase_ms["pathID_mapping_ms"] = float(step_ms)
                        self.log_sig.emit(
                            f"[TIME] pathID_mapping (variant={variant_no}): {step_ms:.1f} ms"
                        )
                        self.log_sig.emit(f"[INFO] pathID mapping done for 0302/0303/0304 (variant={variant_no})")
                    else:
                        self.log_sig.emit(
                            f"[INFO] pathID mapping delayed until current remaining hybrid merge (variant={variant_no})"
                        )

                    manned = [im for im in missions if int(im.get('aircraftID', 0)) in (1, 2, 3)]
                    unmanned = [im for im in missions if int(im.get('aircraftID', 0)) in (4, 5, 6)]
                    generic_unmanned = list(unmanned)
                    current_remaining_hybrid_result = None
                    if variant_current_remaining_request is not None:
                        try:
                            current_remaining_hybrid_result = _build_current_remaining_hybrid_locked(
                                variant_current_remaining_request,
                                variant_no=variant_no,
                                log_emit=self.log_sig.emit,
                            )
                        except Exception as exc:
                            self.log_sig.emit(
                                f"[WARN] [variant {variant_no}] current remaining collaborative hybrid failed before generic skip: {exc}"
                            )
                            current_remaining_hybrid_result = None
                        if current_remaining_hybrid_result is not None:
                            skip_result = filter_generic_flightpath_missions_for_hybrid(
                                generic_unmanned,
                                request=variant_current_remaining_request,
                                hybrid=current_remaining_hybrid_result,
                            )
                            generic_unmanned = list(skip_result.missions)
                            skip_policy = getattr(skip_result, "skip_policy", {}) or {}
                            if (
                                str(getattr(variant_current_remaining_request, "planner_mode", "") or "")
                                == "reexecute_first_mission"
                            ):
                                self.log_sig.emit(
                                    f"[REEXEC-FIRST] generic 0303 skip result: "
                                    f"policy={skip_policy}, skipped={int(skip_result.skipped_count)}, "
                                    f"aircraft={sorted(skip_result.skipped_aircraft_ids)}, "
                                    f"pathIDs={sorted(skip_result.skipped_path_ids)}"
                                )
                            if int(skip_result.skipped_count) > 0:
                                self.log_sig.emit(
                                    f"[variant {variant_no}] current remaining generic 0303 skipped: "
                                    f"inputMissionID={current_remaining_hybrid_result.current_input_id}, "
                                    f"missions={int(skip_result.skipped_count)}, "
                                    f"aircraft={sorted(skip_result.skipped_aircraft_ids)}, "
                                    f"pathIDs={sorted(skip_result.skipped_path_ids)}"
                                )
                    wp_alloc = d0303._WPAllocator()
                    wp_alloc_0303 = wp_alloc
                    wp_alloc_0304 = wp_alloc
                    flightpath_wp_blocks_reserved = False
                    try:
                        block_size = max(
                            1000,
                            int(_general_parallel_runtime_config()["replan_variant_waypoint_block_size"]),
                        )
                        wp0303_start = int(reserve_waypoint_block(block_size))
                        wp0304_start = int(reserve_waypoint_block(block_size))
                        wp_alloc_0303 = d0303._WPAllocator(
                            start=wp0303_start,
                            end=wp0303_start + block_size - 1,
                            overflow_block_size=block_size,
                        )
                        wp_alloc_0304 = d0304._WPAllocator(
                            start=wp0304_start,
                            end=wp0304_start + block_size - 1,
                        )
                        flightpath_wp_blocks_reserved = True
                    except Exception as exc:
                        self.log_sig.emit(
                            f"[WARN] FlightPath waypointID block reservation failed; using shared allocator: {exc}"
                        )
                    try:
                        payload_values = (runtime_payload.get("values") or {}) if isinstance(runtime_payload, dict) else {}
                        manned_plan_mode = str(payload_values.get("manned_plan_mode") or "normal").strip().lower()
                    except Exception:
                        manned_plan_mode = "normal"

                    def _build_0303(unmanned_missions: list[dict]) -> Dict[str, Any]:
                        return build_0303_flight_plans_aircraft_parallel(
                            d0303,
                            unmanned_missions,
                            runtime_payload=runtime_payload,
                            wp_alloc=wp_alloc_0303,
                            cruise_speed=float(uav_cruise_speed),
                            turn_step_deg=float(uav_turn_step),
                            ref0203=mrpk_data,
                        )

                    initial_hold_by_aircraft = _type1_initial_lah_hold_by_aircraft(
                        cmpk_source=cmpk_source_path,
                        current_request=variant_current_remaining_request,
                        log_emit=self.log_sig.emit,
                    )

                    def _build_0304():
                        start = time.perf_counter()
                        try:
                            _rv = (runtime_payload.get("values") or {}) if isinstance(runtime_payload, dict) else {}
                        except Exception:
                            _rv = {}
                        manned_for_0304 = _manned_missions_for_reexecute_line_hold_0304(
                            manned,
                            variant_current_remaining_request,
                        )
                        plans = d0304.build_lah_flight_plans_fixed(
                            manned_for_0304,
                            cruise_speed=40.0,
                            manned_plan_mode=manned_plan_mode,
                            lah_path_mode=str(_rv.get("lah_path_mode", "linear")),
                            lah_rl_hex_step=int(_rv.get("lah_rl_hex_step", 50)),
                            lah_rl_area_km=float(_rv.get("lah_rl_area_km", 10.0)),
                            wp_alloc=wp_alloc_0304,
                            initial_hold_by_aircraft=initial_hold_by_aircraft,
                        )
                        return plans, (time.perf_counter() - start) * 1000.0, _get_lah_mission_plan_timings()

                    flight_plans_0303: list[dict] = []
                    flight_plans_0304: list[dict] = []
                    elapsed_0303_ms = 0.0
                    elapsed_0304_ms = 0.0
                    build_result_0303: Dict[str, Any] | None = None
                    mission_timings_0304: list[dict] = []
                    flightpath_builds_concurrent = False
                    flightpath_template_cache_hit = False

                    if initial_template_payload is not None:
                        try:
                            flight_plans_0303, flight_plans_0304, template_fp_meta = _materialize_initial_plan_flightpath_template(
                                template=initial_template_payload,
                                pid_map=pid_map,
                                wp_alloc_0303_obj=wp_alloc_0303,
                                wp_alloc_0304_obj=wp_alloc_0304,
                            )
                            flightpath_template_cache_hit = True
                            elapsed_0303_ms = float(template_fp_meta.get("elapsed_ms") or 0.0)
                            elapsed_0304_ms = 0.0
                            if flight_plans_0303:
                                build_result_0303 = {
                                    "plans": flight_plans_0303,
                                    "elapsed_ms": elapsed_0303_ms,
                                    "mode": "template_cache",
                                    "workers": 1,
                                    "aircraft": len({int(fp.get("aircraftID", 0) or 0) for fp in flight_plans_0303 if isinstance(fp, dict)}),
                                    "fallback_reasons": [],
                                    "reassigned_waypoints": int(template_fp_meta.get("waypoint_0303") or 0),
                                    "waypoint_count_prepass": int(template_fp_meta.get("waypoint_0303") or 0),
                                    "dense_linesearch_metrics": {},
                                    "line_search_counts": {},
                                    "phase_ms": {"template_materialize": elapsed_0303_ms},
                                }
                            self.log_sig.emit(
                                "[REPLAN][CACHE] initial_plan_flightpath_template_hit "
                                f"variant={int(variant_no)} "
                                f"fp0303={len(flight_plans_0303)} fp0304={len(flight_plans_0304)} "
                                f"pathRemapped={int(template_fp_meta.get('path_remapped') or 0)} "
                                f"wp0303={int(template_fp_meta.get('waypoint_0303') or 0)} "
                                f"wp0304={int(template_fp_meta.get('waypoint_0304') or 0)} "
                                f"elapsed_ms={float(template_fp_meta.get('elapsed_ms') or 0.0):.3f}"
                            )
                        except Exception as exc:
                            self.log_sig.emit(
                                f"[WARN] initial plan flightpath template materialize failed; fallback to full build: {exc}"
                            )
                            initial_template_payload = None
                            flight_plans_0303 = []
                            flight_plans_0304 = []
                            build_result_0303 = None
                            elapsed_0303_ms = 0.0
                            elapsed_0304_ms = 0.0

                    if initial_template_payload is None and generic_unmanned and manned and flightpath_wp_blocks_reserved:
                        flightpath_builds_concurrent = True
                        with concurrent.futures.ThreadPoolExecutor(
                            max_workers=2,
                            thread_name_prefix="Build0303_0304",
                        ) as fp_executor:
                            future_0303 = fp_executor.submit(_build_0303, generic_unmanned)
                            future_0304 = fp_executor.submit(_build_0304)
                            build_result_0303 = future_0303.result()
                            flight_plans_0304, elapsed_0304_ms, mission_timings_0304 = future_0304.result()
                        flight_plans_0303 = list(build_result_0303.get("plans") or [])
                        elapsed_0303_ms = float(build_result_0303.get("elapsed_ms") or 0.0)
                    elif initial_template_payload is None and generic_unmanned:
                        build_result_0303 = _build_0303(generic_unmanned)
                        flight_plans_0303 = list(build_result_0303.get("plans") or [])
                        elapsed_0303_ms = float(build_result_0303.get("elapsed_ms") or 0.0)
                    if initial_template_payload is None and manned and not flightpath_builds_concurrent:
                        flight_plans_0304, elapsed_0304_ms, mission_timings_0304 = _build_0304()
                    if generic_unmanned or manned:
                        variant_phase_ms["flightpath_0303_ms"] = float(elapsed_0303_ms)
                        variant_phase_ms["flightpath_0304_ms"] = float(elapsed_0304_ms)
                        mode_parts = []
                        if build_result_0303 is not None:
                            mode_parts.append(
                                "0303="
                                f"{str(build_result_0303.get('mode') or 'sequential')} "
                                f"workers={int(build_result_0303.get('workers') or 1)} "
                                f"aircraft={int(build_result_0303.get('aircraft') or 0)}"
                            )
                        elif unmanned and current_remaining_hybrid_result is not None:
                            mode_parts.append("0303=skipped_by_current_remaining_hybrid")
                        if manned:
                            if flightpath_template_cache_hit:
                                mode_parts.append("0304=template_cache")
                            else:
                                mode_parts.append("0304=sequential_concurrent" if flightpath_builds_concurrent else "0304=sequential")
                        self.log_sig.emit(
                            f"[INFO] FlightPath build mode (variant={variant_no}): "
                            + ", ".join(mode_parts)
                        )
                        if build_result_0303 is not None:
                            _emit_0303_build_metric(
                                self.log_sig.emit,
                                variant_no=variant_no,
                                option_code=int(option_code),
                                mode="sequential",
                                build_result=build_result_0303,
                            )
                        parts = []
                        if generic_unmanned:
                            parts.append(f"0303={elapsed_0303_ms:.1f} ms")
                        elif unmanned and current_remaining_hybrid_result is not None:
                            parts.append("0303=skipped_by_current_remaining_hybrid")
                        if manned:
                            if flightpath_template_cache_hit:
                                parts.append("0304=template_cache")
                            else:
                                parts.append(f"0304={elapsed_0304_ms:.1f} ms")
                        self.log_sig.emit(
                            "[INFO] FlightPath build time: " + ", ".join(parts)
                        )
                        _emit_mission_plan_timing_metrics(
                            self.log_sig.emit,
                            variant_no=variant_no,
                            option_code=int(option_code),
                            mode="sequential",
                            build_result_0303=build_result_0303,
                            mission_timings_0304=mission_timings_0304,
                            elapsed_0303_ms=elapsed_0303_ms,
                            elapsed_0304_ms=elapsed_0304_ms,
                            flightpath_concurrent=flightpath_builds_concurrent,
                        )

                    remaining_hybrid_result = None
                    hybrid_path_ids: Set[int] = set()
                    if variant_current_remaining_request is not None:
                        missions, flight_plans_0303, flight_plans_0304, hybrid_path_ids = _apply_current_remaining_hybrid_to_variant(
                            variant_no=variant_no,
                            missions=missions,
                            flight_plans_0303=flight_plans_0303,
                            flight_plans_0304=flight_plans_0304,
                            request=variant_current_remaining_request,
                            hybrid_result=current_remaining_hybrid_result,
                        )
                        post_hybrid_path_snapshot = _snapshot_mission_path_ids(missions)
                        step_t0 = time.perf_counter()
                        reserved_path_ids_by_aircraft = _reserve_fresh_path_ids_for_missions(missions)
                        pid_map = _assign_fresh_path_ids(
                            missions,
                            variant_generated_path_ids,
                            reserved_path_ids_by_aircraft=reserved_path_ids_by_aircraft,
                        )
                        step_ms = (time.perf_counter() - step_t0) * 1000.0
                        variant_phase_ms["pathID_mapping_ms"] = float(
                            variant_phase_ms.get("pathID_mapping_ms", 0.0)
                        ) + float(step_ms)
                        self.log_sig.emit(
                            f"[TIME] pathID_mapping (variant={variant_no}, current_remaining): {step_ms:.1f} ms"
                        )
                        self.log_sig.emit(
                            f"[INFO] pathID mapping done after current remaining hybrid merge (variant={variant_no})"
                        )
                        path_remap_by_old = _build_path_id_remap(post_hybrid_path_snapshot, missions)
                    else:
                        remaining_hybrid_result = _apply_remaining_hybrid_customization(
                            variant_no=variant_no,
                            cmpk_source_path=cmpk_source_path,
                            missions=missions,
                            flight_plans_0303=flight_plans_0303,
                            snapshot_mutated=bool(
                                not variant_attack_context
                                and not variant_prior_context
                                and (
                                    int(snapshot_apply_result.get("applied") or 0) > 0
                                    or int(snapshot_apply_result.get("marked_done") or 0) > 0
                                )
                            ),
                        )
                        path_remap_by_old = _build_path_id_remap(pre_path_snapshot, missions)

                    if flight_plans_0303 and flight_plans_0304:
                        try:
                            flight_plans_0304 = d0304.apply_uav_eta_follow_speed_plan(
                                list(flight_plans_0304),
                                list(flight_plans_0303),
                                lah_missions=list(manned or []),
                            )
                            self.log_sig.emit(
                                f"[INFO] Applied LAH-UAV ETA follow speed plan (variant={variant_no})"
                            )
                        except Exception as exc:
                            self.log_sig.emit(
                                f"[WARN] Failed to apply LAH-UAV ETA follow speed plan (variant={variant_no}): {exc}"
                            )

                    custom_t0 = None
                    if variant_attack_context or variant_prior_context:
                        custom_t0 = time.perf_counter()

                    if variant_attack_context:
                        attack_custom_result = self._apply_attack_customizations(
                            missions,
                            flight_plans_0304 or [],
                            variant_attack_context,
                            variant_no,
                            replan_detail=shared_attack_detail,
                        )
                        attack_custom_notice = str((attack_custom_result or {}).get("failure_notice") or "").strip()
                        if attack_custom_notice and not failure_notice_sent:
                            self.log_sig.emit(f"[ATTACK] 공격 옵션 적용 실패 -> 0305 재계획 완료(실패 사유) 발송: {attack_custom_notice}")
                            self._push_replan_failure_completion(attack_custom_notice)
                            failure_notice_sent = True
                    if variant_prior_context:
                        _apply_prior_mission_customizations(
                            missions,
                            flight_plans_0303,
                            variant_prior_context,
                            variant_no,
                            pid_map,
                            variant_generated_path_ids,
                        )
                    if custom_t0 is not None:
                        custom_ms = (time.perf_counter() - custom_t0) * 1000.0
                        self.log_sig.emit(
                            f"[TIME] customizations (variant={variant_no}): {custom_ms:.1f} ms"
                        )

                    for fp in (flight_plans_0303 or []) + (flight_plans_0304 or []):
                        pid_val = fp.get('pathID')
                        if pid_val is not None:
                            try:
                                variant_generated_path_ids.add(int(pid_val))
                            except Exception:
                                pass

                    fixed3 = _enforce_fp_path_ids(
                        flight_plans_0303,
                        pid_map,
                        path_remap_by_old=path_remap_by_old,
                    )
                    fixed4 = _enforce_fp_path_ids(
                        flight_plans_0304,
                        pid_map,
                        path_remap_by_old=path_remap_by_old,
                    )
                    if fixed3 or fixed4:
                        self.log_sig.emit(f"[INFO] FlightPath pathID enforced (variant={variant_no}): 0303={fixed3}, 0304={fixed4}")
                    duplicate_repairs = _repair_duplicate_flightpath_path_ids(
                        missions=missions,
                        flight_plans_0303=flight_plans_0303,
                        flight_plans_0304=flight_plans_0304,
                        generated_path_ids=variant_generated_path_ids,
                        pid_map=pid_map,
                    )
                    if duplicate_repairs:
                        self.log_sig.emit(
                            f"[WARN] FlightPath duplicate pathID repaired before write "
                            f"(variant={variant_no}): fixed={duplicate_repairs}"
                        )
                    _validate_unique_flightpath_ids(
                        variant_no=variant_no,
                        flight_plans_0303=flight_plans_0303,
                        flight_plans_0304=flight_plans_0304,
                    )
                    _apply_manual_runtime_fov_overrides(
                        missions=missions,
                        flight_plans_0303=flight_plans_0303,
                        variant_no=variant_no,
                    )
                    if (
                        initial_template_enabled_for_variant
                        and initial_template_payload is None
                        and initial_template_key
                        and missions_before_path_ids_for_template is not None
                        and mp_json_template_for_cache is not None
                        and (flight_plans_0303 or flight_plans_0304)
                    ):
                        try:
                            _store_initial_plan_template(
                                cache_key=initial_template_key,
                                mp_json_template=mp_json_template_for_cache,
                                missions_before_path_ids=missions_before_path_ids_for_template,
                                missions_after_path_ids=missions,
                                flight_plans_0303_template=flight_plans_0303,
                                flight_plans_0304_template=flight_plans_0304,
                                variant_no_value=variant_no,
                            )
                        except Exception as exc:
                            self.log_sig.emit(f"[WARN] initial plan template store failed: {exc}")
                    if not flight_plans_0303 and not flight_plans_0304:
                        self.log_sig.emit(f"[ERR] FlightPath generation failed (variant={variant_no})")
                        _record_issue("flightpath_generation_failed", f"FlightPath generation failed (variant={variant_no})")
                        plan_log_summary.update({"stop_reason": "flightpath_generation_failed", "variant": variant_no})
                        _notify_failure_once("flightpath_generation_failed", detail={"variant": variant_no})
                        self._record_replan_timing_event(
                            "variant_finished",
                            extra={
                                "variant": variant_no,
                                "option": int(option_code),
                                "mode": "sequential",
                                "status": "failed",
                                "code": "flightpath_generation_failed",
                                "duration_ms": round((time.perf_counter() - variant_start) * 1000.0, 3),
                            },
                        )
                        return
                    self.log_sig.emit(f"[OK] FlightPath counts (variant={variant_no}): 0303={len(flight_plans_0303)} / 0304={len(flight_plans_0304)}")
                    _emit_flightpath_metric(
                        self.log_sig.emit,
                        variant_no=variant_no,
                        option_code=int(option_code),
                        mode="sequential",
                        flight_plans_0303=flight_plans_0303,
                        flight_plans_0304=flight_plans_0304,
                    )

                    expected_path_ids = _expected_mission_path_ids(missions)

                    def _collect_valid_path_ids(fps):
                        collected: Set[int] = set()
                        for fp in fps or []:
                            path_id = fp.get("pathID")
                            if path_id is None:
                                continue
                            # Formation followers may not have explicit waypoints.
                            if fp.get("isFormationFlight"):
                                try:
                                    collected.add(int(path_id))
                                except Exception:
                                    pass
                                continue
                            waypoints = fp.get("waypointList")
                            if not waypoints:
                                waypoints = fp.get("lahWaypointList")
                            if not waypoints:
                                continue
                            try:
                                collected.add(int(path_id))
                            except Exception:
                                continue
                        return collected

                    available_path_ids = _collect_valid_path_ids(flight_plans_0303)
                    available_path_ids.update(_collect_valid_path_ids(flight_plans_0304))
                    missing_path_ids = sorted(pid for pid in expected_path_ids if pid not in available_path_ids)
                    if missing_path_ids:
                        missing_summary = ", ".join(str(pid) for pid in missing_path_ids)
                        self.log_sig.emit(
                            f"[ERR] FlightPath generation incomplete (variant={variant_no}): missing pathID(s) {missing_summary}"
                        )
                        _record_issue("flightpath_missing_ids", f"missing pathID(s) {missing_summary}")
                        plan_log_summary.update({"stop_reason": "flightpath_missing_ids", "missing_paths": missing_summary})
                        _notify_failure_once("flightpath_missing_ids", detail={"missing_paths": missing_summary, "variant": variant_no})
                        self._plan_status = "임무계획 실패"
                        self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                        self._record_replan_timing_event(
                            "variant_finished",
                            extra={
                                "variant": variant_no,
                                "option": int(option_code),
                                "mode": "sequential",
                                "status": "failed",
                                "code": "flightpath_missing_ids",
                                "duration_ms": round((time.perf_counter() - variant_start) * 1000.0, 3),
                            },
                        )
                        return

                    plan_id, plan_id_contract = _allocate_general_variant_plan_id(
                        variant_no=variant_no,
                        option_code=int(option_code),
                        requested_plan_id=requested_plan_id,
                        generated_plan_json=mp_json,
                    )
                    mp_json['missionPlanID'] = plan_id
                    imp_id_map = _allocate_imp_id_map(mp_json)
                    plan_meta_entry = plan_meta_map.setdefault(plan_id, {})
                    plan_meta_entry.update(
                        {
                            "variant": int(variant_no),
                            "optionCode": int(option_code),
                            "requestedMissionPlanID": plan_id_contract["requestedMissionPlanID"],
                            "missionPlanIDMatchesRequest": plan_id_contract["missionPlanIDMatchesRequest"],
                            "planIDContract": plan_id_contract,
                            "optionDependentIsolation": option_dependent_isolation,
                            "currentRemainingHybridSharePolicy": copy.deepcopy(current_remaining_hybrid_share_policy),
                        }
                    )
                    if variant_attack_context:
                        attack_meta = {
                            "attack": True,
                            "targetCount": int(variant_attack_context.get("targetCount") or 1),
                            "targetID": variant_attack_context.get("targetID"),
                        }
                        if shared_attack_detail:
                            attack_meta["replanDetail"] = shared_attack_detail
                        plan_meta_entry.update(attack_meta)
                    if variant_prior_context:
                        try:
                            prior_mid = int(variant_prior_context.get("priorMissionID") or 0)
                        except Exception:
                            prior_mid = 0
                        try:
                            prior_input_id = int(variant_prior_context.get("inputMissionID") or 0)
                        except Exception:
                            prior_input_id = 0
                        plan_meta_entry.update(
                            {
                                "priorMission": True,
                                "priorMissionID": prior_mid,
                                "inputMissionID": prior_input_id,
                            }
                        )
                    if getattr(remaining_hybrid_result, "applied", False):
                        plan_meta_entry.update(
                            {
                                "remainingHybridApplied": True,
                                "remainingHybridMode": str(getattr(remaining_hybrid_result, "mode", "") or ""),
                                "remainingHybridInputMissionID": _safe_int_value(
                                    getattr(remaining_hybrid_result, "input_mission_id", None)
                                ),
                                "remainingHybridAircraftIDs": [
                                    int(value)
                                    for value in (getattr(remaining_hybrid_result, "aircraft_ids", []) or [])
                                    if _safe_int_value(value) is not None
                                ],
                                "remainingHybridWorkflow": str(
                                    getattr(remaining_hybrid_result, "planner_workflow", "") or ""
                                ),
                                "remainingHybridValidation": copy.deepcopy(
                                    getattr(remaining_hybrid_result, "validation", None) or {}
                                ),
                            }
                        )

                    step_t0 = time.perf_counter()
                    imp_pkgs = d0302.build_mission_packages(missions, cmpk_id=cmpk_id, plan_pkg_map=imp_id_map)
                    step_ms = (time.perf_counter() - step_t0) * 1000.0
                    variant_phase_ms["build_0302_ms"] = float(step_ms)
                    self.log_sig.emit(
                        f"[TIME] build_0302 (variant={variant_no}): {step_ms:.1f} ms, packages={len(imp_pkgs)}"
                    )
                    _sync_flight_plan_individual_mission_ids(
                        variant_no=variant_no,
                        imp_pkgs=imp_pkgs,
                        flight_plans_0303=flight_plans_0303,
                        flight_plans_0304=flight_plans_0304,
                    )
                    _validate_mission_flightpath_links(
                        variant_no=variant_no,
                        missions=missions,
                        flight_plans_0303=flight_plans_0303,
                        flight_plans_0304=flight_plans_0304,
                    )
                    step_t0 = time.perf_counter()
                    validation_summary = validate_replan_payloads(
                        mission_plan=mp_json,
                        individual_mission_plans=imp_pkgs,
                        flight_paths=list(flight_plans_0303 or []) + list(flight_plans_0304 or []),
                        scope=f"generalFallback:{plan_id}",
                        allow_existing_db_artifacts=True,
                        log=self.log_sig.emit,
                    )
                    step_ms = (time.perf_counter() - step_t0) * 1000.0
                    plan_meta_entry["validation"] = validation_summary
                    variant_phase_ms["validate_ms"] = float(step_ms)
                    self.log_sig.emit(
                        f"[TIME] validate_general_variant (variant={variant_no}, sequential): {step_ms:.1f} ms"
                    )
                    step_t0 = time.perf_counter()
                    imp_write_rows: list[tuple[Path, Any]] = []
                    for pkg in imp_pkgs:
                        imp_id = pkg.get('individualMissionPackageID') or pkg.get('individualMissionPlanPackageID')
                        if imp_id is None:
                            continue
                        try:
                            generated_imp_ids.add(int(imp_id))
                        except Exception:
                            pass
                        imp_write_rows.append((dir_imp / f"{int(imp_id)}.json", pkg))
                    _write_json_batch(imp_write_rows, pretty=True)
                    total_imp_files += len(imp_pkgs)
                    step_ms = (time.perf_counter() - step_t0) * 1000.0
                    variant_phase_ms["write_0302_ms"] = float(step_ms)
                    self.log_sig.emit(
                        f"[TIME] write_0302 (variant={variant_no}): {step_ms:.1f} ms, files={len(imp_pkgs)}"
                    )

                    def _dump_fp(target_dir, fps):
                        rows: list[tuple[Path, Any]] = []
                        for fp in fps:
                            pid = fp.get('pathID')
                            if pid is None:
                                continue
                            rows.append((target_dir / f"{int(pid)}.json", fp))
                            try:
                                stored_path_ids.add(int(pid))
                            except Exception:
                                pass
                        _write_json_batch(rows, pretty=True)
                        return len(rows)

                    step_t0 = time.perf_counter()
                    fp_count_0303 = _dump_fp(dir_fp, flight_plans_0303)
                    fp_count_0304 = _dump_fp(dir_fp, flight_plans_0304)
                    total_fp_files += fp_count_0303 + fp_count_0304
                    step_ms = (time.perf_counter() - step_t0) * 1000.0
                    variant_phase_ms["write_flightpath_ms"] = float(step_ms)
                    self.log_sig.emit(
                        f"[TIME] write_FlightPath (variant={variant_no}): {step_ms:.1f} ms, files={fp_count_0303 + fp_count_0304}"
                    )
                    _emit_flightpath_write_metric(
                        self.log_sig.emit,
                        variant_no=variant_no,
                        option_code=int(option_code),
                        mode="sequential",
                        files_0303=fp_count_0303,
                        files_0304=fp_count_0304,
                        write_ms=step_ms,
                    )
                    repaired_fp_count, missing_fp_ids = _repair_missing_flight_path_files(
                        variant_no=variant_no,
                        dir_fp=dir_fp,
                        imp_pkgs=imp_pkgs,
                        flight_plans_0303=flight_plans_0303,
                        flight_plans_0304=flight_plans_0304,
                    )
                    if missing_fp_ids:
                        missing_summary = ", ".join(str(pid) for pid in missing_fp_ids)
                        self.log_sig.emit(
                            f"[ERR] FlightPath write incomplete (variant={variant_no}): missing pathID(s) {missing_summary}"
                        )
                        _record_issue(
                            "flightpath_write_missing_ids",
                            f"FlightPath write incomplete (variant={variant_no})",
                            detail={"variant": variant_no, "missing_paths": missing_summary},
                        )
                        plan_log_summary.update(
                            {"stop_reason": "flightpath_write_missing_ids", "variant": variant_no, "missing_paths": missing_summary}
                        )
                        _notify_failure_once(
                            "flightpath_write_missing_ids",
                            detail={"variant": variant_no, "missing_paths": missing_summary},
                        )
                        self._plan_status = "임무계획 실패"
                        self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                        self._record_replan_timing_event(
                            "variant_finished",
                            extra={
                                "variant": variant_no,
                                "option": int(option_code),
                                "mode": "sequential",
                                "status": "failed",
                                "code": "flightpath_write_missing_ids",
                                "duration_ms": round((time.perf_counter() - variant_start) * 1000.0, 3),
                            },
                        )
                        return
                    if repaired_fp_count:
                        step_ms = (time.perf_counter() - step_t0) * 1000.0
                        self.log_sig.emit(
                            f"[TIME] write_FlightPath+repair (variant={variant_no}): {step_ms:.1f} ms"
                        )
                    try:
                        mark_waypoint_files_written(
                            _max_waypoint_id_from_flight_plans(flight_plans_0303, flight_plans_0304)
                        )
                    except Exception:
                        pass

                    step_t0 = time.perf_counter()
                    mp_json["planningTime"] = float((time.perf_counter() - variant_start) * 1000.0)
                    write_json(dir_mp / f"{plan_id}.json", mp_json, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
                    step_ms = (time.perf_counter() - step_t0) * 1000.0
                    variant_phase_ms["write_0301_ms"] = float(step_ms)
                    self.log_sig.emit(
                        f"[TIME] write_0301 (variant={variant_no}): {step_ms:.1f} ms"
                    )
                    snapshot_carry_forward = _carry_forward_mission_area_snapshot(
                        variant_no=variant_no,
                        plan_id=plan_id,
                        reason=(
                            "general_remaining_hybrid"
                            if getattr(remaining_hybrid_result, "applied", False)
                            else "general_fallback"
                        ),
                    )
                    plan_meta_entry["missionAreaSnapshotCarryForward"] = snapshot_carry_forward

                    self.log_sig.emit(f"[OK] Stored variant {variant_no}: MissionPlanID={plan_id}, IMP={len(imp_pkgs)}, FlightPath={fp_count_0303 + fp_count_0304}")
                    total_ms = (time.perf_counter() - variant_start) * 1000.0
                    self._record_replan_timing_event(
                        "variant_finished",
                        extra={
                            "variant": variant_no,
                            "option": int(option_code),
                            "mode": "sequential",
                            "status": "success",
                            "duration_ms": round(total_ms, 3),
                        },
                    )
                    self.log_sig.emit(
                        f"[TIME] variant_total (variant={variant_no}): {total_ms:.1f} ms"
                    )
                    _emit_option_total_timing_metric(
                        self.log_sig.emit,
                        variant_no=variant_no,
                        option_code=int(option_code),
                        mode="sequential",
                        core_phase_ms=variant_phase_ms,
                        store_phase_ms={},
                        variant_total_ms=total_ms,
                    )

                    generated_plan_ids.append(plan_id)
                    option_codes_out.append(int(option_code))
                    generated_path_ids.update(variant_generated_path_ids)
                    self.log_sig.emit(
                        f"[INFO] Option mapping #{variant_no}: "
                        f"planID={plan_id}, optionCode={option_code}({option_code_to_label(option_code)})"
                    )

                    try:
                        shutil.rmtree(iter_out_root)
                    except Exception:
                        pass

            try:
                if out_root_base.exists():
                    shutil.rmtree(out_root_base)
            except Exception:
                pass

            self.log_sig.emit(f"[OK] Stored mission data: MissionPlan={len(generated_plan_ids)}, IndividualMission={total_imp_files}, FlightPath={total_fp_files}")
            _record_step("store_output", "ok", detail={"plan_count": len(generated_plan_ids), "imp_files": total_imp_files, "flightpath_files": total_fp_files})

            self._last_mission_plan_ids = generated_plan_ids
            self._last_mission_plan_id = generated_plan_ids[0] if generated_plan_ids else None
            self.visual_refresh.emit()
            quality_speed_delivery = (
                _is_quality_speed_reason_text(reason)
                or _plan_meta_has_quality_speed(plan_meta_map)
                or _is_quality_speed_trigger_type(
                    ((ctx.get("replan_detail") or {}) if isinstance(ctx.get("replan_detail"), dict) else {}).get("triggerType")
                )
            )
            if quality_speed_delivery:
                option_codes_out = []
            ctx['plan_ids'] = generated_plan_ids
            ctx['option_names'] = option_codes_out
            ctx["_option_meta"] = dict(plan_meta_map)
            self._active_plan_context = ctx

            try:
                input_pkg_id_int = int(ctx.get('inputMissionPackageID'))
            except Exception:
                input_pkg_id_int = None
            try:
                ref_pkg_id_int = int(ctx.get('missionReferencePackageID'))
            except Exception:
                ref_pkg_id_int = None

            plan_id_set = {int(pid) for pid in generated_plan_ids if pid is not None}
            imp_id_set = {int(val) for val in generated_imp_ids if val is not None}
            path_id_set = {int(val) for val in stored_path_ids if val is not None}

            if input_pkg_id_int is not None:
                self._session_scope['packages'].add(input_pkg_id_int)
            self._session_scope['plans'].update(plan_id_set)
            self._session_scope['individual_packages'].update(imp_id_set)
            self._session_scope['paths'].update(path_id_set)
            self._plan_status = "임무계획 완료"
            self._submit_id_tab_update(
                scope=self._session_scope,
                cmpk_id=input_pkg_id_int,
                mrpk_id=ref_pkg_id_int,
                plan_state=self._plan_status,
                defer_until_post_delivery=bool(generated_plan_ids),
            )

            # 강제 전송 여부는 옵션/재계획 레벨(4)에 따라 결정
            force_direct_update = bool(ctx.get("force_direct_update"))
            suppress_0702_fallback = bool(ctx.get("suppress_0702_fallback"))
            try:
                replan_level_val = int(ctx.get("replan_level", 0))
            except Exception:
                replan_level_val = 0
            if replan_level_val == 4:
                force_direct_update = True
            if quality_speed_delivery:
                force_direct_update = True
                suppress_0702_fallback = True


            self._schedule_plan_delivery(
                generated_plan_ids,
                option_codes_out,
                reason,
                plan_meta_map,
                force_direct_update=force_direct_update,
                suppress_0702_fallback=suppress_0702_fallback,
                post_delivery_waypoint_mark=post_delivery_waypoint_mark,
                post_delivery_snapshot_carry_forward=post_delivery_snapshot_carry_forward,
            )
            summary_info = {
                "mode": "legacy",
                "plan_ids": list(generated_plan_ids),
                "option_codes": list(option_codes_out),
            }
            plan_log_status = "success"
            plan_log_summary.update(summary_info)
            if plan_log:
                try:
                    plan_log.set_plan_ids(generated_plan_ids)
                    plan_log.update_summary(summary_info)
                    _record_step(
                        "replan_pipeline",
                        "success",
                        detail={"plan_ids": list(generated_plan_ids), "option_codes": list(option_codes_out)},
                    )
                except Exception:
                    pass
            success = True

        except Exception as exc:
            self.log_sig.emit(f"[ERR] Replan pipeline failed: {exc}")
            try:
                trace_text = traceback.format_exc().strip()
                if trace_text:
                    self.log_sig.emit("[TRACE] " + trace_text)
            except Exception:
                pass
            self._pipeline_logger.log_event(session_id, "error", f"Replan pipeline failed: {exc}")
            plan_log_status = "error"
            plan_log_stop_reason = plan_log_stop_reason or "exception"
            try:
                _record_issue("exception", f"{exc}", detail={"reason": reason})
                plan_log_summary.setdefault("stop_reason", plan_log_stop_reason)
                plan_log_summary.setdefault("exception", str(exc))
            except Exception:
                pass
            self._emit_lifecycle(
                "uncaught_exception",
                component="replan_pipeline",
                outcome="failure",
                reason=str(exc),
                extra={"sessionId": session_id},
            )
            _notify_failure_once(plan_log_stop_reason or "exception", exc=exc, detail=dict(plan_log_summary))
        finally:
            try:
                if attack_exclusion_executor is not None:
                    self._emit_lifecycle(
                        "future_cancel",
                        component="attack_exclusion_parallel",
                        outcome="cancelled",
                        extra={"pending": bool(attack_exclusion_future is not None and not attack_exclusion_future.done())},
                    )
                    attack_exclusion_executor.shutdown(wait=True, cancel_futures=True)
                    attack_exclusion_executor = None
            except Exception:
                pass
            try:
                cache_stats = source_cache.stats()
                self.log_sig.emit(
                    "[REPLAN][METRIC] source_artifact_cache "
                    f"entries={int(cache_stats.get('entries') or 0)} "
                    f"hits={int(cache_stats.get('hits') or 0)} "
                    f"misses={int(cache_stats.get('misses') or 0)}"
                )
                summary_payload_cache = dict(cache_stats)
                plan_log_summary.setdefault("sourceArtifactCache", summary_payload_cache)
            except Exception:
                pass
            try:
                if source_cache_scope is not None:
                    source_cache_scope.__exit__(None, None, None)
            except Exception:
                pass
            if not success and self._planning_timer_started_at is not None:
                try:
                    self._mark_planning_metric_finish(reason, success=False)
                except Exception:
                    pass
            try:
                self._flush_runtime_fov_adjustment_logs()
            except Exception:
                pass
            self._initplan_running = False
            final_status = "success" if success else plan_log_status
            summary_payload: Dict[str, Any] = {}
            if plan_log_summary:
                summary_payload.update(plan_log_summary)
            if summary_info:
                try:
                    summary_payload.update(summary_info)
                except Exception:
                    pass
            if plan_log:
                try:
                    if plan_log_stop_reason and final_status != "success":
                        plan_log.set_stop_reason(plan_log_stop_reason)
                    plan_log.set_plan_ids(
                        getattr(self, "_last_mission_plan_ids", None) or ctx.get("plan_ids") or []
                    )
                    plan_log.finalize(final_status, summary=summary_payload or summary_info)
                except Exception:
                    pass
            try:
                self._mission_plan_logger.clear_active()
            except Exception:
                pass
                self._active_plan_log_run = None
            status_text = (
                "success"
                if success
                else (final_status if final_status in ("success", "error") else "error")
            )
            self._pipeline_logger.close_session(session_id, status_text, summary=summary_info or summary_payload)
            self._emit_lifecycle(
                "worker_thread_stop",
                component="replan_pipeline_thread",
                outcome=status_text,
                reason=plan_log_stop_reason,
                extra={"sessionId": session_id},
            )
            try:
                self.resume_deferred_replan_sig.emit()
            except Exception:
                pass

    def _finalize_attack_pipeline(
        self,
        ctx: Dict[str, Any],
        attack_result: Dict[str, Any],
        attack_updates: Dict[str, Any],
        reason: str,
        session_id: Optional[str],
        *,
        schedule_delivery: bool = True,
        plan_log=None,
    ) -> None:
        try:
            attack_plan_id = int(attack_updates.get("mission_plan_id"))
        except Exception:
            attack_plan_id = None

        provided_plan_ids = list(ctx.get("plan_ids") or [])
        plan_ids = []
        if attack_plan_id is None:
            if provided_plan_ids:
                try:
                    attack_plan_id = int(provided_plan_ids[0])
                except Exception:
                    attack_plan_id = None
        if attack_plan_id is not None:
            plan_ids = [attack_plan_id]
        else:
            plan_ids = list(provided_plan_ids)
        if not plan_ids:
            raise RuntimeError("Attack pipeline produced no mission plan IDs.")

        option_names = list(ctx.get("option_names") or [])
        while len(option_names) < len(plan_ids):
            option_names.append(option_names[-1] if option_names else f"option{len(option_names) + 1}")

        plan_meta_map = dict(ctx.get("_option_meta") or {})
        if attack_plan_id:
            attack_meta = plan_meta_map.setdefault(attack_plan_id, {})
            attack_meta.update(
                {
                    "attack": True,
                    "attackContext": {
                        "logPath": attack_result.get("log_path"),
                        "missionUpdates": attack_updates,
                    },
                }
            )

        if plan_log:
            try:
                plan_log.set_plan_ids(plan_ids)
                plan_log.update_summary(
                    {"attack_plan_ids": list(plan_ids), "attack_log": attack_result.get("log_path")}
                )
                plan_log.add_step(
                    "attack_pipeline",
                    "success",
                    detail={"plan_ids": list(plan_ids), "log_path": attack_result.get("log_path")},
                )
            except Exception:
                pass

        ctx["plan_ids"] = plan_ids
        ctx["option_names"] = option_names
        ctx["_option_meta"] = plan_meta_map
        self._active_plan_context = ctx
        self._last_mission_plan_ids = plan_ids
        self._last_mission_plan_id = plan_ids[0] if plan_ids else None
        self.visual_refresh.emit()

        def _safe_int(value):
            try:
                return int(value)
            except (TypeError, ValueError):
                return None

        input_pkg_id_int = _safe_int(ctx.get("inputMissionPackageID"))
        ref_pkg_id_int = _safe_int(ctx.get("missionReferencePackageID"))

        plan_id_set = {int(pid) for pid in plan_ids if pid is not None}
        generated_imp_ids, generated_path_ids = self._collect_attack_generated_ids(attack_updates)
        self._session_scope["plans"].update(plan_id_set)
        self._session_scope["individual_packages"].update(generated_imp_ids)
        self._session_scope["paths"].update(generated_path_ids)
        self._plan_status = "임무재계획 완료"
        self._submit_id_tab_update(
            scope=self._session_scope,
            cmpk_id=input_pkg_id_int,
            mrpk_id=ref_pkg_id_int,
            plan_state=self._plan_status,
            defer_until_post_delivery=bool(schedule_delivery and plan_ids),
        )

        summary = ", ".join(str(pid) for pid in plan_ids)
        self.log_sig.emit(f"[ATTACK] 공격 임무 계획 완료 (planIds={summary})")
        self._pipeline_logger.log_event(
            session_id,
            "info",
            "Attack pipeline complete",
            detail={
                "plan_ids": plan_ids,
                "log_path": attack_result.get("log_path"),
            },
        )

        if schedule_delivery:
            self._schedule_plan_delivery(
                plan_ids,
                option_names,
                reason,
                plan_meta_map,
                force_direct_update=False,
            )

    def _collect_attack_generated_ids(self, attack_updates: Dict[str, Any]) -> tuple[Set[int], Set[int]]:
        imp_ids: Set[int] = set()
        path_ids: Set[int] = set()
        aircraft_entries = attack_updates.get("aircraft") if isinstance(attack_updates, dict) else None
        if not isinstance(aircraft_entries, list):
            return imp_ids, path_ids
        for entry in aircraft_entries:
            if not isinstance(entry, dict):
                continue
            pkg = entry.get("individualMissionPackageID")
            if pkg is not None:
                try:
                    imp_ids.add(int(pkg))
                except Exception:
                    pass
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                pid = mission.get("pathID")
                if pid is not None:
                    try:
                        path_ids.add(int(pid))
                    except Exception:
                        pass
            for block in (entry.get("tracking") or {}, entry.get("resume") or {}):
                if not isinstance(block, dict):
                    continue
                pid = block.get("pathID")
                if pid is not None:
                    try:
                        path_ids.add(int(pid))
                    except Exception:
                        pass
            flight_paths = entry.get("flightPaths")
            if isinstance(flight_paths, dict):
                for pid in flight_paths.values():
                    self._try_add_path_from_value(path_ids, pid)
            extra_path = entry.get("pathID")
            if extra_path is not None:
                try:
                    path_ids.add(int(extra_path))
                except Exception:
                    pass
        return imp_ids, path_ids

    @staticmethod
    def _try_add_path_from_value(target_set: Set[int], value: Any) -> None:
        if value is None:
            return
        str_value = str(value).strip()
        if not str_value:
            return
        if str_value.isdigit():
            try:
                target_set.add(int(str_value))
                return
            except Exception:
                pass
        candidate = Path(str_value)
        if candidate.stem and candidate.stem.isdigit():
            try:
                target_set.add(int(candidate.stem))
                return
            except Exception:
                pass
        try:
            cleaned = "".join(ch for ch in str_value if ch.isdigit())
            if cleaned:
                target_set.add(int(cleaned))
        except Exception:
            pass

    def _run_trigger_pipeline_with_source_cache(
        self,
        label: str,
        func: Callable[..., Any],
        *args: Any,
        **kwargs: Any,
    ) -> Any:
        if str(os.environ.get("REPLAN_TRIGGER_SOURCE_CACHE", "1") or "").strip().lower() in {
            "0",
            "false",
            "no",
            "off",
        }:
            return func(*args, **kwargs)
        cache = SourceArtifactCache()
        started = time.perf_counter()
        try:
            with use_source_artifact_cache(cache):
                return func(*args, **kwargs)
        finally:
            try:
                stats = cache.stats()
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                self.log_sig.emit(
                    "[REPLAN][CACHE] "
                    f"{label} sourceArtifactCache entries={stats.get('entries')} "
                    f"hits={stats.get('hits')} misses={stats.get('misses')} "
                    f"elapsedMs={elapsed_ms:.3f}"
                )
            except Exception:
                pass

    def _try_run_post_attack_rejoin_pipeline(
        self,
        ctx: Dict[str, Any],
        reason: str,
        *,
        session_id: Optional[str] = None,
    ) -> tuple[bool, Optional[Dict[str, Any]]]:
        if not self._should_use_post_attack_rejoin_pipeline(ctx):
            return False, None

        detail = ctx.get("replan_detail")
        if not isinstance(detail, dict):
            self.log_sig.emit("[POSTATTACK] closure detail missing; keeping current plan.")
            self._push_replan_noop_completion(reason, "재계획 불필요")
            return True, {"status": "skipped", "reason": "missing_replan_detail"}

        self.log_sig.emit("[POSTATTACK] 공격 종료 재계획 요청 감지 → 복귀 협업 재계획 평가 실행")
        try:
            result = self._run_trigger_pipeline_with_source_cache(
                "post_attack_rejoin",
                run_post_attack_rejoin_pipeline,
                ctx,
                detail,
                reason,
                log=lambda msg: self.log_sig.emit(msg),
            )
        except Exception as exc:
            self.log_sig.emit(f"[POSTATTACK][ERR] pipeline failed: {exc}")
            if session_id:
                self._pipeline_logger.log_event(
                    session_id,
                    "error",
                    f"Post-attack rejoin pipeline failed: {exc}",
                )
            return True, None

        if not result:
            self.log_sig.emit("[POSTATTACK] pipeline returned no result; keeping current plan.")
            self._push_replan_noop_completion(reason, "재계획 불필요")
            return True, {"status": "skipped", "reason": "pipeline_returned_none"}

        summary = dict(result.summary or {})
        summary.setdefault("status", str(getattr(result, "status", "skipped") or "skipped"))
        summary.setdefault("log_path", str(getattr(result, "log_path", "")))
        transaction_id = summary.get("replanTransactionId")
        if transaction_id:
            ctx["replanTransactionId"] = str(transaction_id)
            try:
                timing = ctx.get("_replan_timing")
                if isinstance(timing, dict):
                    timing["replanTransactionId"] = str(transaction_id)
            except Exception:
                pass
            self._active_plan_context = ctx
        if str(summary.get("status") or "").strip().lower() == "skipped" and not result.plan_ids:
            skip_reason = str(summary.get("reason") or "rejoin_not_needed").strip() or "rejoin_not_needed"
            group_skip_reasons: List[str] = []
            group_evaluations = summary.get("group_evaluations")
            if isinstance(group_evaluations, list):
                for evaluation in group_evaluations:
                    if not isinstance(evaluation, dict):
                        continue
                    group_skip_reason = str(evaluation.get("skip_reason") or "").strip()
                    if group_skip_reason:
                        group_skip_reasons.append(group_skip_reason)
            if skip_reason == "rejoin_not_needed" or "remaining_work_too_small" in group_skip_reasons:
                if "remaining_work_too_small" in group_skip_reasons:
                    detail_text = "잔여 임무 30% 미만"
                else:
                    detail_text = "협업 복귀 재계획 불필요"
                notice = "공격후 복귀 불필요: " + detail_text
                self._push_0305(status=2, reason=f"{reason} / 재계획 불필요")
                self._push_0001_notice(notice)

        if result.plan_ids:
            ctx["plan_ids"] = list(result.plan_ids)
            ctx["option_names"] = list(result.option_names or [])
            ctx["_option_meta"] = dict(result.plan_meta_map or {})
            ctx["force_direct_update"] = True
            ctx["suppress_0702_fallback"] = True

            self._active_plan_context = ctx
            self._last_mission_plan_ids = list(result.plan_ids)
            self._last_mission_plan_id = result.plan_ids[0] if result.plan_ids else None
            self.visual_refresh.emit()

            self._schedule_plan_delivery(
                list(result.plan_ids),
                list(result.option_names or []),
                reason,
                dict(result.plan_meta_map or {}),
                force_direct_update=True,
                suppress_0702_fallback=True,
            )

            input_pkg_id_int = self._to_optional_int(ctx.get("inputMissionPackageID"))
            ref_pkg_id_int = self._to_optional_int(ctx.get("missionReferencePackageID"))
            self._session_scope["plans"].update(int(pid) for pid in result.plan_ids if pid is not None)
            self._session_scope["individual_packages"].update(
                int(val) for val in getattr(result, "generated_imp_ids", set()) if val is not None
            )
            self._session_scope["paths"].update(
                int(val) for val in getattr(result, "generated_path_ids", set()) if val is not None
            )
            self._plan_status = "임무재계획 완료"
            self._submit_id_tab_update(
                scope=self._session_scope,
                cmpk_id=input_pkg_id_int,
                mrpk_id=ref_pkg_id_int,
                plan_state=self._plan_status,
                defer_until_post_delivery=bool(result.plan_ids),
            )

        if session_id:
            self._pipeline_logger.log_event(
                session_id,
                "info",
                "Post-attack rejoin pipeline handled",
                detail=summary,
            )
        return True, summary

    def _try_run_prior_post_rejoin_pipeline(
        self,
        ctx: Dict[str, Any],
        reason: str,
        *,
        session_id: Optional[str] = None,
    ) -> tuple[bool, Optional[Dict[str, Any]]]:
        if not self._should_use_prior_post_rejoin_pipeline(ctx):
            return False, None

        detail = ctx.get("replan_detail")
        if not isinstance(detail, dict):
            self.log_sig.emit("[PRIOR-REJOIN] closure detail missing; keeping current plan.")
            self._push_replan_noop_completion(reason, "재계획 불필요")
            return True, {"status": "skipped", "reason": "missing_replan_detail", "mode": "priorPostRejoin"}

        self.log_sig.emit("[PRIOR-REJOIN] 선행임무 종료 재계획 요청 감지 -> 복귀 협업 재계획 평가 실행")
        try:
            result = self._run_trigger_pipeline_with_source_cache(
                "prior_post_rejoin",
                run_prior_post_rejoin_pipeline,
                ctx,
                detail,
                reason,
                log=lambda msg: self.log_sig.emit(msg),
            )
        except Exception as exc:
            self.log_sig.emit(f"[PRIOR-REJOIN][ERR] pipeline failed: {exc}")
            if session_id:
                self._pipeline_logger.log_event(
                    session_id,
                    "error",
                    f"Prior post-rejoin pipeline failed: {exc}",
                )
            return True, None

        if not result:
            self.log_sig.emit("[PRIOR-REJOIN] pipeline returned no result; keeping current plan.")
            self._push_replan_noop_completion(reason, "재계획 불필요")
            return True, {"status": "skipped", "reason": "pipeline_returned_none", "mode": "priorPostRejoin"}

        summary = dict(result.summary or {})
        summary.setdefault("status", str(getattr(result, "status", "skipped") or "skipped"))
        summary.setdefault("log_path", str(getattr(result, "log_path", "")))
        summary.setdefault("mode", "priorPostRejoin")
        if str(summary.get("status") or "").strip().lower() == "skipped" and not result.plan_ids:
            skip_reason = str(summary.get("reason") or "rejoin_not_needed").strip() or "rejoin_not_needed"
            group_skip_reasons: List[str] = []
            group_evaluations = summary.get("group_evaluations")
            if isinstance(group_evaluations, list):
                for evaluation in group_evaluations:
                    if not isinstance(evaluation, dict):
                        continue
                    group_skip_reason = str(evaluation.get("skip_reason") or "").strip()
                    if group_skip_reason:
                        group_skip_reasons.append(group_skip_reason)
            if skip_reason == "rejoin_not_needed" or "remaining_work_too_small" in group_skip_reasons:
                if "remaining_work_too_small" in group_skip_reasons:
                    detail_text = "잔여 임무 30% 미만"
                else:
                    detail_text = "협업 복귀 재계획 불필요"
                notice = "선행후 복귀 불필요: " + detail_text
                self._push_0305(status=2, reason=f"{reason} / 재계획 불필요")
                self._push_0001_notice(notice)

        if result.plan_ids:
            ctx["plan_ids"] = list(result.plan_ids)
            ctx["option_names"] = list(result.option_names or [])
            ctx["_option_meta"] = dict(result.plan_meta_map or {})
            ctx["force_direct_update"] = True
            ctx["suppress_0702_fallback"] = True

            self._active_plan_context = ctx
            self._last_mission_plan_ids = list(result.plan_ids)
            self._last_mission_plan_id = result.plan_ids[0] if result.plan_ids else None
            self.visual_refresh.emit()

            self._schedule_plan_delivery(
                list(result.plan_ids),
                list(result.option_names or []),
                reason,
                dict(result.plan_meta_map or {}),
                force_direct_update=True,
                suppress_0702_fallback=True,
            )

            input_pkg_id_int = self._to_optional_int(ctx.get("inputMissionPackageID"))
            ref_pkg_id_int = self._to_optional_int(ctx.get("missionReferencePackageID"))
            self._session_scope["plans"].update(int(pid) for pid in result.plan_ids if pid is not None)
            self._session_scope["individual_packages"].update(
                int(val) for val in getattr(result, "generated_imp_ids", set()) if val is not None
            )
            self._session_scope["paths"].update(
                int(val) for val in getattr(result, "generated_path_ids", set()) if val is not None
            )
            self._plan_status = "임무재계획 완료"
            self._submit_id_tab_update(
                scope=self._session_scope,
                cmpk_id=input_pkg_id_int,
                mrpk_id=ref_pkg_id_int,
                plan_state=self._plan_status,
                defer_until_post_delivery=bool(result.plan_ids),
            )

        if session_id:
            self._pipeline_logger.log_event(
                session_id,
                "info",
                "Prior post-rejoin pipeline handled",
                detail=summary,
            )
        return True, summary

    def _try_run_prior_mission_pipeline(
        self,
        ctx: Dict[str, Any],
        reason: str,
        *,
        session_id: Optional[str] = None,
    ) -> Optional[Dict[str, Any]]:
        try:
            replan_level = int(ctx.get("replan_level", ctx.get("replanLevel", 0)))
        except Exception:
            replan_level = 0
        detail = ctx.get("replan_detail")
        if replan_level != 4:
            return None

        prior_post_rejoin_handled, prior_post_rejoin_summary = self._try_run_prior_post_rejoin_pipeline(
            ctx,
            reason,
            session_id=session_id,
        )
        if prior_post_rejoin_handled:
            return prior_post_rejoin_summary or {
                "status": "failed",
                "reason": "prior_post_rejoin_pipeline_failed",
                "mode": "priorPostRejoin",
            }

        if isinstance(detail, dict):
            self.log_sig.emit("[PRIOR] Level-4 prior mission request detected. Using dedicated pipeline.")
        else:
            self.log_sig.emit("[PRIOR] Level-4 prior mission request but replanDetail missing/invalid → running prior pipeline with empty detail.")
            detail = {} if detail is None else {"_raw": detail}

        result = self._run_trigger_pipeline_with_source_cache(
            "prior_mission",
            run_prior_mission_pipeline,
            ctx,
            detail,
            reason,
            log=lambda msg: self.log_sig.emit(msg),
        )
        if not result:
            self.log_sig.emit(
                "[PRIOR][ERR] Prior mission pipeline returned no result; "
                "legacy replan fallback blocked."
            )
            if session_id:
                self._pipeline_logger.log_event(
                    session_id,
                    "error",
                    "Prior mission pipeline failed; legacy fallback blocked",
                )
            return {
                "status": "failed",
                "reason": "prior_pipeline_returned_none",
                "mode": "prior",
            }

        # The prior pipeline may rebase a queued request from an older source
        # plan to the latest applied descendant. Keep the plan-run lineage log
        # aligned with that effective context for later close/rejoin checks.
        try:
            active_plan_log = getattr(self, "_active_plan_log_run", None)
            plan_log_context = getattr(active_plan_log, "context", None)
            if isinstance(plan_log_context, dict):
                effective_detail = ctx.get("replan_detail")
                if isinstance(effective_detail, dict):
                    plan_log_context["replan_detail"] = copy.deepcopy(effective_detail)
                for key_name in ("sourceMissionPlanID", "currentMissionPlanID"):
                    if ctx.get(key_name) is not None:
                        plan_log_context[key_name] = ctx.get(key_name)
        except Exception:
            pass

        generated_plan_ids = result.plan_ids
        option_names = result.option_names
        plan_meta_map = result.plan_meta_map

        ctx["plan_ids"] = generated_plan_ids
        ctx["option_names"] = option_names
        ctx["_option_meta"] = dict(plan_meta_map)

        self._active_plan_context = ctx
        self._last_mission_plan_ids = generated_plan_ids
        self._last_mission_plan_id = generated_plan_ids[0] if generated_plan_ids else None
        self.visual_refresh.emit()

        # delivery를 먼저 큐잉한다. (이후 메타/GUI 갱신 중 예외가 나도 0903 누락 방지)
        self._deliver_prior_direct_now(
            generated_plan_ids,
            reason,
            option_names=option_names,
            option_meta=plan_meta_map,
        )

        def _to_optional_int(value: Any) -> Optional[int]:
            if value is None:
                return None
            try:
                iv = int(value)
            except Exception:
                return None
            return iv if iv > 0 else None

        input_pkg_id_int = _to_optional_int(ctx.get("inputMissionPackageID"))
        ref_pkg_id_int = _to_optional_int(ctx.get("missionReferencePackageID"))

        plan_id_set = {int(pid) for pid in generated_plan_ids if pid is not None}
        imp_id_set = {int(val) for val in result.generated_imp_ids if val is not None}
        path_id_set = {int(val) for val in result.generated_path_ids if val is not None}

        if input_pkg_id_int is not None:
            self._session_scope["packages"].add(input_pkg_id_int)
        self._session_scope["plans"].update(plan_id_set)
        self._session_scope["individual_packages"].update(imp_id_set)
        self._session_scope["paths"].update(path_id_set)

        self._plan_status = "임무계획 완료"
        self._submit_id_tab_update(
            scope=self._session_scope,
            cmpk_id=input_pkg_id_int,
            mrpk_id=ref_pkg_id_int,
            plan_state=self._plan_status,
            defer_until_post_delivery=bool(generated_plan_ids),
        )

        self.log_sig.emit(
            f"[PRIOR] Prior mission pipeline complete (planIds={generated_plan_ids}, log={result.log_path})"
        )
        summary = {
            "plan_ids": list(generated_plan_ids),
            "option_names": list(option_names),
            "generated_individual_mission_package_ids": sorted(imp_id_set),
            "generated_path_ids": sorted(path_id_set),
            "log_path": str(result.log_path),
        }
        if session_id:
            self._pipeline_logger.log_event(
                session_id, "info", "Prior mission pipeline complete", detail=summary
            )
        return summary

    def _try_run_next_collab_replan_pipeline(
        self,
        ctx: Dict[str, Any],
        reason: str,
        *,
        session_id: Optional[str] = None,
    ) -> tuple[bool, Optional[Dict[str, Any]]]:
        return self._try_run_next_collab_replan_pipeline_impl(
            ctx,
            reason,
            session_id=session_id,
        )

    def _load_next_collab_target_input_mission(self, detail: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        def _as_int(value: Any) -> Optional[int]:
            try:
                if value is None:
                    return None
                return int(value)
            except Exception:
                return None

        source_plan_id = _as_int(detail.get("sourceMissionPlanID"))
        target_input_id = _as_int(detail.get("targetInputMissionID"))
        if source_plan_id is None or source_plan_id <= 0 or target_input_id is None or target_input_id <= 0:
            return None

        try:
            plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
            plan_payload = json.loads(plan_path.read_text(encoding="utf-8"))
        except Exception:
            return None

        input_package_id = _as_int(
            plan_payload.get("inputMissionPackageID")
            or plan_payload.get("InputMissionPackageID")
            or plan_payload.get("inputMissionPackageId")
        )
        if input_package_id is None or input_package_id <= 0:
            return None

        try:
            input_path = db_paths.get_db_subpath("InputMissionPlan", f"{int(input_package_id)}.json")
            input_payload = json.loads(input_path.read_text(encoding="utf-8"))
        except Exception:
            return None

        for mission in input_payload.get("inputMissionList") or []:
            if not isinstance(mission, dict):
                continue
            if _as_int(mission.get("inputMissionID")) == int(target_input_id):
                return mission
        return None

    @staticmethod
    def _extract_next_collab_line_coords_and_width(
        mission: Dict[str, Any],
    ) -> tuple[List[Dict[str, Any]], Optional[float]]:
        detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
        if not isinstance(detail, dict):
            detail = {}

        line_rows = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
        if line_rows:
            for row in line_rows:
                if not isinstance(row, dict):
                    continue
                coords = _normalize_coord_list(row.get("coordinateList"), min_len=2)
                if not coords:
                    continue
                width_value = row.get("width")
                if width_value in (None, ""):
                    width_value = row.get("lineWidth")
                if width_value in (None, ""):
                    width_value = row.get("lineWidthM")
                try:
                    width_m = float(width_value) if width_value not in (None, "") else None
                except Exception:
                    width_m = None
                return coords, width_m

        coords = _normalize_coord_list(detail.get("coordinateList"), min_len=2)
        return coords, None

    @staticmethod
    def _next_collab_polyline_length_m(coords: List[Dict[str, Any]]) -> float:
        if len(coords) < 2:
            return 0.0
        total_m = 0.0
        radius_m = 6_371_000.0
        for idx in range(len(coords) - 1):
            start = coords[idx]
            end = coords[idx + 1]
            try:
                lat1 = math.radians(float(start["latitude"]))
                lon1 = math.radians(float(start["longitude"]))
                lat2 = math.radians(float(end["latitude"]))
                lon2 = math.radians(float(end["longitude"]))
            except Exception:
                continue
            d_lat = lat2 - lat1
            d_lon = lon2 - lon1
            sin_dlat = math.sin(d_lat * 0.5)
            sin_dlon = math.sin(d_lon * 0.5)
            a = sin_dlat * sin_dlat + math.cos(lat1) * math.cos(lat2) * sin_dlon * sin_dlon
            total_m += 2.0 * radius_m * math.asin(min(1.0, math.sqrt(max(0.0, a))))
        return float(total_m)

    def _build_next_collab_short_line_skip_summary(
        self,
        *,
        detail: Dict[str, Any],
        failure_reason: str,
    ) -> Optional[Dict[str, Any]]:
        started_at = time.perf_counter()
        lowered = str(failure_reason or "").lower()
        if "produced no path rows" not in lowered and "planner returned no valid path rows" not in lowered:
            return None

        target_input_mission = self._load_next_collab_target_input_mission(detail)
        if not isinstance(target_input_mission, dict) or not self._is_next_collab_line_target_mission(target_input_mission):
            return None

        coords, width_m = self._extract_next_collab_line_coords_and_width(target_input_mission)
        if len(coords) < 2:
            return None

        total_length_m = self._next_collab_polyline_length_m(coords)
        short_line_threshold_m = 150.0
        if not (total_length_m > 0.0 and total_length_m <= short_line_threshold_m):
            return None

        return {
            "status": "skipped",
            "reason": "short_line_segment",
            "target_input_mission_id": int(target_input_mission.get("inputMissionID") or 0),
            "line_length_m": round(float(total_length_m), 1),
            "threshold_m": round(float(short_line_threshold_m), 1),
            "line_width_m": round(float(width_m), 1) if isinstance(width_m, (int, float)) and width_m > 0.0 else None,
            "skip_decision_elapsed_ms": round(max(0.0, (time.perf_counter() - started_at) * 1000.0), 3),
        }

    @staticmethod
    def _is_next_collab_line_target_mission(mission: Dict[str, Any]) -> bool:
        try:
            mission_type = int(mission.get("inputMissionType"))
        except Exception:
            mission_type = None

        if mission_type in (1, 7):
            return True
        if mission_type in (2, 3, 4, 5, 6):
            return False

        detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
        line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
        if line_list:
            return True
        return False

    def _try_run_next_collab_replan_pipeline_impl(
        self,
        ctx: Dict[str, Any],
        reason: str,
        *,
        session_id: Optional[str] = None,
    ) -> tuple[bool, Optional[Dict[str, Any]]]:
        try:
            replan_level = int(ctx.get("replan_level", ctx.get("replanLevel", 0)))
        except Exception:
            replan_level = 0
        detail = ctx.get("replan_detail")
        plan_ids = list(ctx.get("plan_ids") or [])
        reason_text = str(reason or ctx.get("reason") or "").strip()
        store_detail = load_next_collab_detail(plan_ids)
        is_reason_match = _is_next_collab_reason_text(reason_text)
        detail_trigger = ""
        if isinstance(detail, dict):
            detail_trigger = str(detail.get("triggerType") or "").strip()
        if replan_level != 3:
            return False, None
        if not (detail_trigger == "nextCollaborativeMission" or is_reason_match or store_detail):
            return False, None
        if not isinstance(detail, dict) or not detail:
            detail = dict(store_detail or {})
            ctx["replan_detail"] = detail
        elif isinstance(store_detail, dict) and store_detail:
            merged_detail = dict(store_detail)
            merged_detail.update(detail)
            detail = merged_detail
            ctx["replan_detail"] = detail
        if isinstance(detail, dict):
            detail_trigger = str(detail.get("triggerType") or detail_trigger or "").strip()

        target_input_mission = (
            self._load_next_collab_target_input_mission(detail) if isinstance(detail, dict) else None
        )
        target_input_mission_type = None
        if isinstance(target_input_mission, dict):
            try:
                target_input_mission_type = int(target_input_mission.get("inputMissionType"))
            except Exception:
                target_input_mission_type = None
        if int(target_input_mission_type or 0) == 7:
            self.log_sig.emit(
                "[NEXTCOLLAB] formation-flight target detected. "
                "Using reference-route formation replan."
            )

        record_next_collab_event(
            "mission_receive",
            {
                "reason": reason_text,
                "replanLevel": replan_level,
                "planIDs": plan_ids,
                "detailTriggerType": detail_trigger or None,
                "storeDetailLoaded": bool(store_detail),
                "detailKeys": sorted(detail.keys()) if isinstance(detail, dict) else [],
            },
        )

        self.log_sig.emit("[NEXTCOLLAB] Level-3 next collaborative mission request detected. Using dedicated pipeline.")
        pipeline_messages: list[str] = []

        def _emit_next_collab_log(message: str) -> None:
            msg = str(message or "").strip()
            if msg:
                pipeline_messages.append(msg)
            self.log_sig.emit(message)

        ctx.pop("_next_collab_failure_reason", None)
        ctx.pop("_next_collab_failure_notice", None)
        result = run_next_collab_replan_pipeline(
            ctx,
            detail,
            reason,
            log=_emit_next_collab_log,
        )
        if not result:
            failure_reason = self._select_next_collab_failure_message(pipeline_messages)
            short_line_skip_summary = self._build_next_collab_short_line_skip_summary(
                detail=detail,
                failure_reason=failure_reason,
            )
            if short_line_skip_summary is not None:
                notice = "기존 임무 활용: 구간 짧음"
                self.log_sig.emit(
                    "[NEXTCOLLAB] skipped: predictive entry could not be materialized on a short line segment; "
                    "keeping current mission chain."
                )
                self._push_replan_noop_completion(reason, "재계획 불필요")
                self._push_0001_notice(notice)
                self._record_replan_timing_event(
                    "next_collab_short_line_skip",
                    extra=dict(short_line_skip_summary),
                )
                record_next_collab_event(
                    "mission_skipped",
                    {
                        "reason": reason_text,
                        "planIDs": plan_ids,
                        "detailKeys": sorted(detail.keys()) if isinstance(detail, dict) else [],
                        "failureReason": failure_reason or None,
                        **short_line_skip_summary,
                    },
                )
                if session_id:
                    self._pipeline_logger.log_event(
                        session_id,
                        "info",
                        "Next collaborative mission pipeline skipped for short line segment",
                        detail=short_line_skip_summary,
                    )
                return True, short_line_skip_summary
            failure_notice = self._build_next_collab_failure_notice(
                reason_text=reason_text,
                detail=detail,
                ctx=ctx,
                log_messages=pipeline_messages,
            )
            if failure_reason:
                ctx["_next_collab_failure_reason"] = failure_reason
            if failure_notice:
                ctx["_next_collab_failure_notice"] = failure_notice
            self.log_sig.emit("[NEXTCOLLAB] Dedicated next collaborative mission pipeline failed.")
            if session_id:
                self._pipeline_logger.log_event(
                    session_id,
                    "error",
                    "Next collaborative mission pipeline failed",
                    detail={
                        "failure_reason": failure_reason or None,
                        "failure_notice": failure_notice or None,
                    },
                )
            record_next_collab_event(
                "mission_pipeline_failed",
                {
                    "reason": reason_text,
                    "planIDs": plan_ids,
                    "detailKeys": sorted(detail.keys()) if isinstance(detail, dict) else [],
                    "failureReason": failure_reason or None,
                    "failureNotice": failure_notice or None,
                },
            )
            return True, None

        generated_plan_ids = result.plan_ids
        option_names = result.option_names
        plan_meta_map = result.plan_meta_map

        ctx["plan_ids"] = generated_plan_ids
        ctx["option_names"] = option_names
        ctx["_option_meta"] = dict(plan_meta_map)
        ctx["inputMissionPackageID"] = int(result.new_input_package_id)
        ctx["force_direct_update"] = True
        ctx["suppress_0702_fallback"] = True

        self._active_plan_context = ctx
        self._last_mission_plan_ids = generated_plan_ids
        self._last_mission_plan_id = generated_plan_ids[0] if generated_plan_ids else None
        self.visual_refresh.emit()

        self._deliver_next_collab_direct_now(
            generated_plan_ids,
            reason,
            option_names=option_names,
            option_meta=plan_meta_map,
        )

        def _to_optional_int(value: Any) -> Optional[int]:
            if value is None:
                return None
            try:
                iv = int(value)
            except Exception:
                return None
            return iv if iv > 0 else None

        input_pkg_id_int = _to_optional_int(result.new_input_package_id)
        ref_pkg_id_int = _to_optional_int(ctx.get("missionReferencePackageID"))

        if input_pkg_id_int is not None:
            self._session_scope["packages"].add(input_pkg_id_int)
        self._session_scope["plans"].update(int(pid) for pid in generated_plan_ids if pid is not None)
        self._session_scope["individual_packages"].update(
            int(val) for val in result.generated_imp_ids if val is not None
        )
        self._session_scope["paths"].update(
            int(val) for val in result.generated_path_ids if val is not None
        )

        self._plan_status = "임무계획 완료"
        self._submit_id_tab_update(
            scope=self._session_scope,
            cmpk_id=input_pkg_id_int,
            mrpk_id=ref_pkg_id_int,
            plan_state=self._plan_status,
            defer_until_post_delivery=bool(generated_plan_ids),
        )

        summary = {
            "plan_ids": list(generated_plan_ids),
            "option_names": list(option_names),
            "inputMissionPackageID": int(result.new_input_package_id),
            "generated_individual_mission_package_ids": sorted(
                int(val) for val in result.generated_imp_ids if val is not None
            ),
            "generated_path_ids": sorted(
                int(val) for val in result.generated_path_ids if val is not None
            ),
            "force_direct_update": True,
            "suppress_0702_fallback": True,
            "log_path": str(result.log_path),
        }
        self.log_sig.emit(
            f"[NEXTCOLLAB] Next collaborative mission pipeline complete (planIds={generated_plan_ids}, log={result.log_path})"
        )
        if session_id:
            self._pipeline_logger.log_event(
                session_id,
                "info",
                "Next collaborative mission pipeline complete",
                detail=summary,
            )
        return True, summary

    def _try_run_path_deviation_replan_pipeline(
        self,
        ctx: Dict[str, Any],
        reason: str,
        *,
        session_id: Optional[str] = None,
    ) -> tuple[bool, Optional[Dict[str, Any]]]:
        return self._try_run_path_deviation_replan_pipeline_impl(
            ctx,
            reason,
            session_id=session_id,
        )

    def _try_run_imaging_schedule_replan_pipeline(
        self,
        ctx: Dict[str, Any],
        reason: str,
        *,
        session_id: Optional[str] = None,
    ) -> tuple[bool, Optional[Dict[str, Any]]]:
        try:
            replan_level = int(ctx.get("replan_level", ctx.get("replanLevel", 0)))
        except Exception:
            replan_level = 0
        detail = ctx.get("replan_detail")
        plan_ids = list(ctx.get("plan_ids") or [])
        reason_text = str(reason or ctx.get("reason") or "").strip()
        store_detail = self._load_imaging_schedule_detail_from_store(plan_ids)
        is_reason_match = _is_imaging_schedule_reason_text(reason_text)
        is_quality_reason_match = _is_quality_speed_reason_text(reason_text)
        detail_trigger = ""
        if isinstance(detail, dict):
            detail_trigger = str(detail.get("triggerType") or "").strip()
        if replan_level != 3:
            return False, None
        if not (
            detail_trigger in {"imagingScheduleDeviation", "qualityMonitorSep"}
            or is_reason_match
            or is_quality_reason_match
            or store_detail
        ):
            return False, None
        if not isinstance(detail, dict) or not detail:
            detail = dict(store_detail or {})
            ctx["replan_detail"] = detail
        elif isinstance(store_detail, dict) and store_detail:
            merged_detail = dict(store_detail)
            merged_detail.update(detail)
            detail = merged_detail
            ctx["replan_detail"] = detail

        self._log_imaging_schedule_event(
            "mission_receive",
            {
                "reason": reason_text,
                "replanLevel": replan_level,
                "planIDs": plan_ids,
                "detailTriggerType": detail_trigger or None,
                "storeDetailLoaded": bool(store_detail),
                "detailKeys": sorted(detail.keys()) if isinstance(detail, dict) else [],
            },
        )

        is_quality_mode = _is_quality_speed_trigger_type(detail_trigger) or is_quality_reason_match
        trigger_label = "QUALITY" if is_quality_mode else "IMGSCH"
        mode_label = "quality speed" if is_quality_mode else "imaging schedule"
        self.log_sig.emit(f"[{trigger_label}] Level-3 {mode_label} request detected. Using dedicated pipeline.")
        result = run_imaging_schedule_replan_pipeline(
            ctx,
            detail,
            reason,
            log=lambda msg: self.log_sig.emit(msg),
        )
        if not result:
            self.log_sig.emit(f"[{trigger_label}] Dedicated {mode_label} pipeline failed.")
            if session_id:
                self._pipeline_logger.log_event(
                    session_id,
                    "error",
                    f"{mode_label.title()} pipeline failed",
                )
            self._log_imaging_schedule_event(
                "mission_pipeline_failed",
                {
                    "reason": reason_text,
                    "planIDs": plan_ids,
                    "detailKeys": sorted(detail.keys()) if isinstance(detail, dict) else [],
                },
            )
            return True, None

        generated_plan_ids = result.plan_ids
        option_names = result.option_names
        plan_meta_map = result.plan_meta_map

        ctx["plan_ids"] = generated_plan_ids
        ctx["option_names"] = option_names
        ctx["_option_meta"] = dict(plan_meta_map)

        self._active_plan_context = ctx
        self._last_mission_plan_ids = generated_plan_ids
        self._last_mission_plan_id = generated_plan_ids[0] if generated_plan_ids else None
        self.visual_refresh.emit()

        self._deliver_imaging_schedule_direct_now(
            generated_plan_ids,
            reason,
            option_names=option_names,
            option_meta=plan_meta_map,
            suppress_0702_fallback=bool(is_quality_mode),
        )

        def _to_optional_int(value: Any) -> Optional[int]:
            if value is None:
                return None
            try:
                iv = int(value)
            except Exception:
                return None
            return iv if iv > 0 else None

        input_pkg_id_int = _to_optional_int(ctx.get("inputMissionPackageID"))
        ref_pkg_id_int = _to_optional_int(ctx.get("missionReferencePackageID"))

        if input_pkg_id_int is not None:
            self._session_scope["packages"].add(input_pkg_id_int)
        self._session_scope["plans"].update(int(pid) for pid in generated_plan_ids if pid is not None)
        self._session_scope["individual_packages"].update(
            int(val) for val in result.generated_imp_ids if val is not None
        )
        self._session_scope["paths"].update(
            int(val) for val in result.generated_path_ids if val is not None
        )

        self._plan_status = "임무계획 완료"
        self._submit_id_tab_update(
            scope=self._session_scope,
            cmpk_id=input_pkg_id_int,
            mrpk_id=ref_pkg_id_int,
            plan_state=self._plan_status,
            defer_until_post_delivery=bool(generated_plan_ids),
        )

        summary = {
            "plan_ids": list(generated_plan_ids),
            "option_names": list(option_names),
            "log_path": str(result.log_path),
            "replaced_waypoint_id": int(result.replaced_waypoint_id),
            "new_waypoint_id": int(result.new_waypoint_id),
            "trigger_type": str(getattr(result, "trigger_type", detail_trigger or "")),
            "removed_waypoint_id": getattr(result, "removed_waypoint_id", None),
            "anchor_waypoint_id": getattr(result, "anchor_waypoint_id", None),
            "search_speed_scale": getattr(result, "search_speed_scale", None),
            "speed_adjustment_direction": getattr(result, "speed_adjustment_direction", None),
            "trimmed_sweep_points": int(getattr(result, "trimmed_sweep_points", 0) or 0),
        }
        self.log_sig.emit(
            f"[{trigger_label}] {mode_label.title()} pipeline complete (planIds={generated_plan_ids}, log={result.log_path})"
        )
        if session_id:
            self._pipeline_logger.log_event(
                session_id,
                "info",
                f"{mode_label.title()} pipeline complete",
                detail=summary,
            )
        return True, summary

    def _try_run_path_deviation_replan_pipeline_impl(
        self,
        ctx: Dict[str, Any],
        reason: str,
        *,
        session_id: Optional[str] = None,
    ) -> tuple[bool, Optional[Dict[str, Any]]]:
        try:
            replan_level = int(ctx.get("replan_level", ctx.get("replanLevel", 0)))
        except Exception:
            replan_level = 0
        detail = ctx.get("replan_detail")
        plan_ids = list(ctx.get("plan_ids") or [])
        reason_text = str(reason or ctx.get("reason") or "").strip()
        store_detail = self._load_path_deviation_detail_from_store(plan_ids)
        is_reason_match = _is_path_deviation_reason_text(reason_text)
        detail_trigger = ""
        if isinstance(detail, dict):
            detail_trigger = str(detail.get("triggerType") or "").strip()
        if replan_level != 3:
            return False, None
        if not (detail_trigger == "pathDeviation" or is_reason_match or store_detail):
            return False, None
        if not isinstance(detail, dict) or not detail:
            detail = dict(store_detail or {})
            ctx["replan_detail"] = detail
        elif isinstance(store_detail, dict) and store_detail:
            merged_detail = dict(store_detail)
            merged_detail.update(detail)
            detail = merged_detail
            ctx["replan_detail"] = detail
        if isinstance(detail, dict):
            def _ctx_positive_int(*keys: str) -> Optional[int]:
                for key in keys:
                    try:
                        value = int(ctx.get(key))
                    except Exception:
                        continue
                    if value > 0:
                        return value
                return None

            latest_source_plan_id = _ctx_positive_int(
                "currentMissionPlanID",
                "sourceMissionPlanID",
                "currentPlanID",
                "sourcePlanID",
                "current_mission_plan_id",
                "source_mission_plan_id",
                "source_plan_id",
            )
            if latest_source_plan_id is not None:
                detail = dict(detail)
                detail["currentMissionPlanID"] = int(latest_source_plan_id)
                detail["sourceMissionPlanID"] = int(latest_source_plan_id)
                ctx["replan_detail"] = detail

        self._log_path_deviation_event(
            "mission_receive",
            {
                "reason": reason_text,
                "replanLevel": replan_level,
                "planIDs": plan_ids,
                "detailTriggerType": detail_trigger or None,
                "storeDetailLoaded": bool(store_detail),
                "ctxCurrentMissionPlanID": ctx.get("currentMissionPlanID"),
                "ctxSourceMissionPlanID": ctx.get("sourceMissionPlanID"),
                "detailCurrentMissionPlanID": detail.get("currentMissionPlanID") if isinstance(detail, dict) else None,
                "detailSourceMissionPlanID": detail.get("sourceMissionPlanID") if isinstance(detail, dict) else None,
                "detailKeys": sorted(detail.keys()) if isinstance(detail, dict) else [],
            },
        )

        self.log_sig.emit("[PATHDEV] Level-3 path deviation request detected. Using dedicated pipeline.")
        result = self._run_trigger_pipeline_with_source_cache(
            "path_deviation",
            run_path_deviation_replan_pipeline,
            ctx,
            detail,
            reason,
            log=lambda msg: self.log_sig.emit(msg),
        )
        if not result:
            self.log_sig.emit("[PATHDEV] Dedicated path-deviation pipeline failed.")
            if session_id:
                self._pipeline_logger.log_event(
                    session_id,
                    "error",
                    "Path-deviation pipeline failed",
                )
            self._log_path_deviation_event(
                "mission_pipeline_failed",
                {
                    "reason": reason_text,
                    "planIDs": plan_ids,
                    "detailKeys": sorted(detail.keys()) if isinstance(detail, dict) else [],
                },
            )
            return True, None

        generated_plan_ids = result.plan_ids
        option_names = result.option_names
        plan_meta_map = result.plan_meta_map

        ctx["plan_ids"] = generated_plan_ids
        ctx["option_names"] = option_names
        ctx["_option_meta"] = dict(plan_meta_map)

        self._active_plan_context = ctx
        self._last_mission_plan_ids = generated_plan_ids
        self._last_mission_plan_id = generated_plan_ids[0] if generated_plan_ids else None
        self.visual_refresh.emit()

        self._deliver_path_deviation_direct_now(
            generated_plan_ids,
            reason,
            option_names=option_names,
            option_meta=plan_meta_map,
        )

        def _to_optional_int(value: Any) -> Optional[int]:
            if value is None:
                return None
            try:
                iv = int(value)
            except Exception:
                return None
            return iv if iv > 0 else None

        input_pkg_id_int = _to_optional_int(ctx.get("inputMissionPackageID"))
        ref_pkg_id_int = _to_optional_int(ctx.get("missionReferencePackageID"))

        if input_pkg_id_int is not None:
            self._session_scope["packages"].add(input_pkg_id_int)
        self._session_scope["plans"].update(int(pid) for pid in generated_plan_ids if pid is not None)
        self._session_scope["individual_packages"].update(
            int(val) for val in result.generated_imp_ids if val is not None
        )
        self._session_scope["individual_packages"].update(
            int(val) for val in getattr(result, "preserved_manned_imp_ids", set()) if val is not None
        )
        self._session_scope["paths"].update(
            int(val) for val in result.generated_path_ids if val is not None
        )
        self._session_scope["paths"].update(
            int(val) for val in getattr(result, "preserved_manned_path_ids", set()) if val is not None
        )

        self._plan_status = "임무계획 완료"
        self._submit_id_tab_update(
            scope=self._session_scope,
            cmpk_id=input_pkg_id_int,
            mrpk_id=ref_pkg_id_int,
            plan_state=self._plan_status,
            defer_until_post_delivery=bool(generated_plan_ids),
        )

        summary = {
            "plan_ids": list(generated_plan_ids),
            "option_names": list(option_names),
            "generated_individual_mission_package_ids": sorted(
                int(val) for val in result.generated_imp_ids if val is not None
            ),
            "generated_path_ids": sorted(
                int(val) for val in result.generated_path_ids if val is not None
            ),
            "preserved_manned_individual_mission_package_ids": sorted(
                int(val) for val in getattr(result, "preserved_manned_imp_ids", set()) if val is not None
            ),
            "preserved_manned_path_ids": sorted(
                int(val) for val in getattr(result, "preserved_manned_path_ids", set()) if val is not None
            ),
            "log_path": str(result.log_path),
            "removed_waypoint_id": int(result.removed_waypoint_id),
            "inserted_waypoint_id": int(result.inserted_waypoint_id),
        }
        self.log_sig.emit(
            f"[PATHDEV] Path-deviation pipeline complete (planIds={generated_plan_ids}, log={result.log_path})"
        )
        if session_id:
            self._pipeline_logger.log_event(
                session_id,
                "info",
                "Path-deviation pipeline complete",
                detail=summary,
            )
        return True, summary

    @staticmethod
    def _load_imaging_schedule_detail_from_store(plan_ids: List[int]) -> Optional[Dict[str, Any]]:
        for value in plan_ids or []:
            try:
                plan_id = int(value)
            except Exception:
                continue
            payload = imaging_schedule_replan_store.load_detail(plan_id)
            if payload:
                return payload
        return None

    def _log_imaging_schedule_event(self, stage: str, payload: Dict[str, Any]) -> None:
        try:
            imaging_schedule_replan_store.save_event(stage, payload)
        except Exception:
            pass

    def _deliver_imaging_schedule_direct_now(
        self,
        plan_ids: List[int],
        reason: str,
        *,
        option_names: Optional[List[str]] = None,
        option_meta: Optional[Dict[int, Dict[str, Any]]] = None,
        suppress_0702_fallback: bool = False,
    ) -> None:
        valid_ids: List[int] = []
        for value in plan_ids or []:
            try:
                pid = int(value)
            except Exception:
                continue
            if pid > 0 and pid not in valid_ids:
                valid_ids.append(pid)
        if not valid_ids:
            self.log_sig.emit("[IMGSCH][DELIVERY] skipped: no valid missionPlanID")
            return

        self.log_sig.emit(
            f"[IMGSCH][DELIVERY] direct push start (planIds={', '.join(str(v) for v in valid_ids)})"
        )
        self._schedule_plan_delivery(
            valid_ids,
            list(option_names or []),
            reason,
            dict(option_meta or {}),
            force_direct_update=True,
            suppress_0702_fallback=bool(suppress_0702_fallback),
        )

    def _deliver_next_collab_direct_now(
        self,
        plan_ids: List[int],
        reason: str,
        *,
        option_names: Optional[List[str]] = None,
        option_meta: Optional[Dict[int, Dict[str, Any]]] = None,
    ) -> None:
        valid_ids: List[int] = []
        for value in plan_ids or []:
            try:
                pid = int(value)
            except Exception:
                continue
            if pid > 0 and pid not in valid_ids:
                valid_ids.append(pid)
        if not valid_ids:
            self.log_sig.emit("[NEXTCOLLAB][DELIVERY] skipped: no valid missionPlanID")
            return

        self.log_sig.emit(
            f"[NEXTCOLLAB][DELIVERY] direct push start (planIds={', '.join(str(v) for v in valid_ids)})"
        )
        self._schedule_plan_delivery(
            valid_ids,
            list(option_names or []),
            reason,
            dict(option_meta or {}),
            force_direct_update=True,
            suppress_0702_fallback=True,
        )

    @staticmethod
    def _load_path_deviation_detail_from_store(plan_ids: List[int]) -> Optional[Dict[str, Any]]:
        for value in plan_ids or []:
            try:
                plan_id = int(value)
            except Exception:
                continue
            payload = path_deviation_replan_store.load_detail(plan_id)
            if payload:
                return payload
        return None

    def _log_path_deviation_event(self, stage: str, payload: Dict[str, Any]) -> None:
        try:
            path_deviation_replan_store.save_event(stage, payload)
        except Exception:
            pass

    def _deliver_path_deviation_direct_now(
        self,
        plan_ids: List[int],
        reason: str,
        *,
        option_names: Optional[List[str]] = None,
        option_meta: Optional[Dict[int, Dict[str, Any]]] = None,
    ) -> None:
        valid_ids: List[int] = []
        for value in plan_ids or []:
            try:
                pid = int(value)
            except Exception:
                continue
            if pid > 0 and pid not in valid_ids:
                valid_ids.append(pid)
        if not valid_ids:
            self.log_sig.emit("[PATHDEV][DELIVERY] skipped: no valid missionPlanID")
            return

        self.log_sig.emit(
            f"[PATHDEV][DELIVERY] direct push start (planIds={', '.join(str(v) for v in valid_ids)})"
        )
        self._schedule_plan_delivery(
            valid_ids,
            list(option_names or []),
            reason,
            dict(option_meta or {}),
            force_direct_update=True,
            suppress_0702_fallback=True,
        )

    def _deliver_prior_direct_now(
        self,
        plan_ids: List[int],
        reason: str,
        *,
        option_names: Optional[List[str]] = None,
        option_meta: Optional[Dict[int, Dict[str, Any]]] = None,
    ) -> None:
        valid_ids: List[int] = []
        for value in plan_ids or []:
            try:
                pid = int(value)
            except Exception:
                continue
            if pid > 0 and pid not in valid_ids:
                valid_ids.append(pid)
        if not valid_ids:
            self.log_sig.emit("[PRIOR][DELIVERY] skipped: no valid missionPlanID")
            return

        self.log_sig.emit(
            f"[PRIOR][DELIVERY] direct push start (planIds={', '.join(str(v) for v in valid_ids)})"
        )
        # GUI 전송 경로(0301 버튼 경유 + 0903 순차 푸시)를 그대로 사용한다.
        # suppress_0702_fallback=False 이므로 기본적으로 0903 뒤에 0702 fallback도 함께 송신한다.
        self._schedule_plan_delivery(
            valid_ids,
            list(option_names or []),
            reason,
            dict(option_meta or {}),
            force_direct_update=True,
        )

    def _schedule_plan_delivery(
        self,
        plan_ids,
        option_names,
        reason,
        option_meta=None,
        *,
        force_direct_update: bool = False,
        suppress_0702_fallback: bool = False,
        post_delivery_waypoint_mark: Optional[Dict[str, Any]] = None,
        post_delivery_snapshot_carry_forward: Optional[Dict[str, Any]] = None,
    ):
        safe_reason = _sanitize_reason(reason, "init-plan")
        is_quality_speed_delivery = _is_quality_speed_reason_text(safe_reason) or _plan_meta_has_quality_speed(option_meta)
        if is_quality_speed_delivery:
            force_direct_update = True
            suppress_0702_fallback = True
        new_plan_ids = [pid for pid in (plan_ids or []) if pid is not None]
        new_option_names = [] if is_quality_speed_delivery else list(option_names or [])
        if not is_quality_speed_delivery:
            while len(new_option_names) < len(new_plan_ids):
                new_option_names.append(f"option{len(new_option_names) + 1}")

        # Prior/force-direct 흐름은 옵션 생성(0901) 병합 없이 최신 플랜만 전달
        if force_direct_update:
            try:
                self._attack_delivery_buffer.clear()
            except Exception:
                self._attack_delivery_buffer = []
            self._pending_plan_push = None

        if getattr(self, "_attack_delivery_buffer", None):
            for buf in self._attack_delivery_buffer:
                buf_ids = list(buf.get("plan_ids") or [])
                buf_names = list(buf.get("option_names") or [])
                for idx, pid in enumerate(buf_ids):
                    try:
                        pid_int = int(pid)
                    except Exception:
                        continue
                    if pid_int in new_plan_ids:
                        continue
                    new_plan_ids.append(pid_int)
                    if idx < len(buf_names):
                        new_option_names.append(str(buf_names[idx]))
                    else:
                        new_option_names.append(f"option{len(new_option_names) + 1}")
                try:
                    buf_meta = dict(buf.get("option_meta") or {})
                    if buf_meta:
                        option_meta = dict(option_meta or {})
                        option_meta.update(buf_meta)
                except Exception:
                    pass
            self._attack_delivery_buffer.clear()

        new_plan_ids, new_option_names = _sort_plan_delivery_entries(new_plan_ids, new_option_names)
        self._record_replan_timing_event(
            "pipeline_done",
            extra={
                "plans": len(new_plan_ids),
                "force_direct": bool(force_direct_update),
                "suppress_0702": bool(suppress_0702_fallback),
                "quality_speed": bool(is_quality_speed_delivery),
                "option_names": len(new_option_names),
            },
        )

        if self._pending_plan_push:
            pending = self._pending_plan_push
            merged_ids = list(pending.get("plan_ids") or [])
            merged_names = list(pending.get("option_names") or [])
            merged_meta = dict(pending.get("option_meta") or {})

            for idx, pid in enumerate(new_plan_ids):
                try:
                    pid_int = int(pid)
                except Exception:
                    continue
                if pid_int in merged_ids:
                    continue
                merged_ids.append(pid_int)
                if idx < len(new_option_names):
                    merged_names.append(new_option_names[idx])
                else:
                    merged_names.append(f"option{len(merged_names) + 1}")

            merged_meta.update(dict(option_meta or {}))
            merged_ids, merged_names = _sort_plan_delivery_entries(merged_ids, merged_names)
            pending["plan_ids"] = merged_ids
            pending["option_names"] = merged_names
            pending["option_meta"] = merged_meta
            pending["force_direct_update"] = pending.get("force_direct_update") or bool(force_direct_update)
            pending["suppress_0702_fallback"] = pending.get("suppress_0702_fallback") or bool(suppress_0702_fallback)
            if not pending.get("inputMissionPackageID"):
                input_pkg_id = _safe_int_value((getattr(self, "_active_plan_context", None) or {}).get("inputMissionPackageID"))
                if input_pkg_id is not None and int(input_pkg_id) > 0:
                    pending["inputMissionPackageID"] = int(input_pkg_id)
            merged_mark = self._merge_post_delivery_waypoint_mark(
                pending.get("post_delivery_waypoint_mark"),
                post_delivery_waypoint_mark,
            )
            if merged_mark:
                pending["post_delivery_waypoint_mark"] = merged_mark
            merged_snapshot_carry = self._merge_post_delivery_snapshot_carry_forward(
                pending.get("post_delivery_snapshot_carry_forward"),
                post_delivery_snapshot_carry_forward,
            )
            if merged_snapshot_carry:
                pending["post_delivery_snapshot_carry_forward"] = merged_snapshot_carry
            self._pending_plan_push = pending
            summary = ", ".join(str(pid) for pid in merged_ids) or "-"
            self._record_replan_timing_event("0301_merged", extra={"plans": len(merged_ids)})
            self.log_sig.emit(f"[STEP 4] 0301 push merged (planIds={summary})")
            return

        pending_payload = {
            "plan_ids": new_plan_ids,
            "option_names": new_option_names,
            "reason": safe_reason,
            "option_meta": dict(option_meta or {}),
            "force_direct_update": bool(force_direct_update),
            "suppress_0702_fallback": bool(suppress_0702_fallback),
        }
        input_pkg_id = _safe_int_value((getattr(self, "_active_plan_context", None) or {}).get("inputMissionPackageID"))
        if input_pkg_id is not None and int(input_pkg_id) > 0:
            pending_payload["inputMissionPackageID"] = int(input_pkg_id)
        normalized_mark = self._normalize_post_delivery_waypoint_mark(post_delivery_waypoint_mark)
        if normalized_mark:
            pending_payload["post_delivery_waypoint_mark"] = normalized_mark
        normalized_snapshot_carry = self._normalize_post_delivery_snapshot_carry_forward(
            post_delivery_snapshot_carry_forward
        )
        if normalized_snapshot_carry:
            pending_payload["post_delivery_snapshot_carry_forward"] = normalized_snapshot_carry
        self._pending_plan_push = pending_payload
        try:
            self._scheduled_0301_plan_ids = [int(pid) for pid in new_plan_ids if pid is not None]
        except Exception:
            self._scheduled_0301_plan_ids = list(new_plan_ids)
        summary = ", ".join(str(pid) for pid in new_plan_ids) or "-"
        self._record_replan_timing_event("0301_queued", extra={"plans": len(new_plan_ids)})
        self.log_sig.emit(f"[STEP 4] 0301 push queued (planIds={summary})")
        self.start_push_seq.emit()

    def _shutdown_for_process_exit(self, reason: str = "close_event") -> None:
        if getattr(self, "_shutdown_started", False):
            return
        self._shutdown_started = True
        safe_reason = str(reason or "process_exit")
        self._emit_lifecycle(
            "graceful_shutdown",
            component="gui",
            outcome="start",
            reason=safe_reason,
        )
        try:
            self._stop_all_periodic()
        except Exception:
            pass
        try:
            # 0303 process workers are intentionally persistent between replans;
            # release them explicitly before the interpreter's executor-exit hook.
            from modules.mission_planning.runtime.persistent_process_pool import (
                shutdown_process_pools,
            )

            shutdown_process_pools(wait=False, cancel_futures=True)
        except Exception:
            pass
        try:
            for attr in (
                "_poll_0101_timer",
                "_db_root_timer",
                "_post_0301_timer",
                "_replan_delay_timer",
            ):
                timer = getattr(self, attr, None)
                if timer is not None:
                    try:
                        timer.stop()
                    except Exception:
                        pass
        except Exception:
            pass
        try:
            self._hb_0102_enabled = False
            self._hb_0102_stop.set()
            hb_thread = getattr(self, "_hb_0102_thread", None)
            if hb_thread is not None and hb_thread.is_alive() and hb_thread is not threading.current_thread():
                hb_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            rx_thread = getattr(self, "_rx_setup_thread", None)
            if rx_thread is not None and rx_thread.is_alive() and rx_thread is not threading.current_thread():
                self._emit_lifecycle("worker_thread_wait", component="rx_setup", outcome="start")
                rx_thread.join(timeout=3.0)
                self._emit_lifecycle(
                    "worker_thread_wait",
                    component="rx_setup",
                    outcome="ok" if not rx_thread.is_alive() else "timeout",
                )
        except Exception:
            pass
        try:
            for msg_id, handler in list(getattr(self, "_input_listener_refs", [])):
                unregister_listener(msg_id, handler)
                self._emit_lifecycle(
                    "listener_stop",
                    component=f"{msg_id}_listener",
                    outcome="ok",
                )
            self._input_listener_refs = []
            rx0101 = getattr(self, "_rx0101", None)
            if rx0101 is not None:
                try:
                    unregister_listener("0101", rx0101)
                    self._emit_lifecycle("listener_stop", component="0101_listener", outcome="ok")
                except Exception:
                    pass
                self._rx0101 = None
        except Exception:
            self._emit_lifecycle(
                "listener_stop_fail",
                component="input_listener",
                outcome="failure",
                reason=safe_reason,
            )
            pass
        self._emit_lifecycle(
            "graceful_shutdown",
            component="gui",
            outcome="ok",
            reason=safe_reason,
        )

    def closeEvent(self, event):
        if not getattr(self, "_force_process_exit_close", False) and hide_instead_of_close(
            self,
            event,
            log=self._append_log_line,
        ):
            return
        self._shutdown_for_process_exit("close_event")
        super().closeEvent(event)


def _smoke_launch_main() -> int:
    from modules.mission_planning.app.gui_entrypoint import smoke_launch_main

    return smoke_launch_main(sys.modules[__name__])


# ───────── 엔트리 ─────────
if __name__ == "__main__":
    from modules.mission_planning.app.gui_entrypoint import run_public_gui_entrypoint

    _exit_code = 1
    try:
        _exit_code = int(run_public_gui_entrypoint(sys.modules[__name__]))
    except SystemExit as exc:
        try:
            _exit_code = int(exc.code or 0)
        except Exception:
            _exit_code = 1
    except BaseException:
        try:
            sys.excepthook(*sys.exc_info())
        except Exception:
            pass
        _exit_code = 1
    try:
        from modules.common.process_console import close_process_file_logging

        close_process_file_logging("mission_planning")
    except Exception:
        pass
    try:
        sys.stdout.flush()
    except Exception:
        pass
    try:
        sys.stderr.flush()
    except Exception:
        pass
    os._exit(_exit_code)
