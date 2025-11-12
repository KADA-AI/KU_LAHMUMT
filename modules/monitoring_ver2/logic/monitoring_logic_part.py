
from datetime import datetime, timezone
from dataclasses import asdict
import threading
import time

from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple, Union

from data.message_models import (
    ModuleStatusModelModel,
    MissionProgressBodyModel,
    MissionEndRequestBodyModel,
    ReplanRequestBodyModel,
    ReplanRequestTimeStampModel,
    InputMissionIDModel,
    IndividualMissionIDListModel,
    PriorMissionListModel,
    OptionListModel,
    DecisionResultModel,
)
from push.push_center import push_message
from .monitoring_actual_logic import run_monitoring_procedure
from .replan_actual_logic import (
    run_replan_procedure,
    REPLAN_FIELD_DETAIL,
    REPLAN_FIELD_REASON,
    REPLAN_FIELD_SITUATION,
)
from .replan_utils import (
    ensure_replan_level_details_file,
    load_target_info,
    update_target_info_from_0402,
    mark_targets_as_used,
)
import udp_reporter
import socket
import json
import os
import math
from modules.common import db_paths
from modules.monitoring_ver2.utils.vehicle_status import (
    MANNED_AIRCRAFT_IDS,
    UNMANNED_AIRCRAFT_IDS,
    write_vehicle_status,
)
from modules.monitoring_ver2.logic.prior_mission_replan import PriorMissionReplanCoordinator
from modules.monitoring_ver2.utils.mission_progress_logger import MissionProgressExporter


def _resolve_fuel_capacity() -> float:
    raw = os.getenv("KU_MON_FUEL_CAPACITY_L", "15")
    try:
        value = float(raw) if raw is not None else 15.0
        if value > 0:
            return value
    except (TypeError, ValueError):
        pass
    return 15.0


FUEL_CAPACITY_LITERS = _resolve_fuel_capacity()

COLLAB_REPLAN_REASON_REEXECUTE = "협업기저임무 재수행에 대한 재계획"
COLLAB_REPLAN_REASON_REINPUT = "협업기저임무 재입력에 대한 재계획"
COLLAB_REPLAN_DEFAULT_SOURCE = "MSM"
FORCED_HOLD_DELAY_SECONDS = 10.0
FORCED_HOLD_DELAY_REASON = "강제대기 후 10초 경과"
FORCED_HOLD_DEADLINE_ATTR = "_hold_defer_deadline"
FORCED_HOLD_REASON_ATTR = "_hold_defer_reason"
FORCED_HOLD_STATE_KEY = "forced_hold_state_map"
TARGET_REPLAN_SITUATION_LABEL = "0402 표적 탐지 재계획"
ROI_REPLAN_SITUATION_LABEL = "0402 ROI 지속 재계획"
ROI_CAUTION_TIMEOUT_SECONDS = 10.0
TARGET_TYPE_LABELS = {
    None: "표적",
    0: "표적",
    1: "전차",
    2: "장갑차",
    3: "방사포",
    4: "곡사포",
    5: "고정고사포",
    6: "군인",
}
WATCHER_CALLSIGN_MAP = {
    4: "무인기 1번",
    5: "무인기 2번",
    6: "무인기 3번",
}


def _ms_since_2000() -> int:
    """Return current UTC timestamp in milliseconds since 2000-01-01."""
    return int(
        (
            datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)
        ).total_seconds()
        * 1000
    )


def _inform_info_module(msg_id: str, body: dict):
    try:
        port = int(os.getenv("KU_INFO_CTRL_PORT", "45984"))
    except Exception:
        port = 45984
    payload = {
        "cmd": "inject_msg",
        "msg_id": msg_id,
        "body": body,
    }
    try:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(data, ("127.0.0.1", port))
    except Exception:
        pass


PushBodyType = Union[
    ModuleStatusModelModel,
    MissionProgressBodyModel,
    MissionEndRequestBodyModel,
    ReplanRequestBodyModel,
]


class MonitoringLogic:
    def __init__(self, manager):
        self.manager = manager
        self._current_mission_plan_id: Optional[int] = None
        self._plan_context: Optional[Dict[str, Any]] = None
        self._input_mission_tracker: Dict[int, Dict[str, Set[Tuple[int, int, Optional[int], int]]]] = {}
        self._mission_to_input: Dict[
            Tuple[int, int, Optional[int], int], int
        ] = {}
        self._completed_input_ids: Set[int] = set()
        self._collab_completion_sent: bool = False
        self._pending_input_completion_queue: List[int] = []
        self._monitoring_suspended: bool = False
        self._active_aircraft_ids: Set[int] = set()
        self._mission_progress_max: Dict[Tuple[int, int, Optional[int], int], int] = {}
        self._mission_file_map: Dict[
            Tuple[int, int, Optional[int], int], Tuple[int, int, int]
        ] = {}
        self._input_completion_notified: Set[int] = set()
        self._collab_pause_active: bool = False
        self._collab_pause_prev_suspended: bool = False
        self._collab_reexecute_mode: bool = False
        self._collab_reexecute_trigger_ts: Optional[int] = None
        self._collab_replan_pending: bool = False
        self._collab_replan_inflight: bool = False
        self._collab_replan_trigger: Optional[Dict[str, Any]] = None
        self._collab_last_replan_key: Optional[Tuple[Optional[int], Optional[int]]] = None
        self._latest_input_plan_key: Optional[Tuple[Optional[int], Optional[int]]] = None
        self._collab_replan_required_input_key: Optional[
            Tuple[Optional[int], Optional[int]]
        ] = None
        self._collab_replan_waiting_for_new_input_logged: bool = False
        self._used_option_ids: Set[int] = set()
        self._target_trigger_history: Dict[str, int] = {}
        self._allocated_plan_ids: Set[int] = set()
        self._existing_mission_plan_ids: Set[int] = set()
        self._pending_mission_plan_id: Optional[int] = None
        self._pending_decision_command: Optional[Tuple[Optional[int], Optional[int]]] = None
        self._current_input_mission_id: Optional[int] = None
        self._input_mission_package_id: Optional[int] = None
        self._input_mission_plan_path: Optional[Path] = None
        self._input_mission_index_map: Dict[int, int] = {}
        self._input_mission_order: List[int] = []
        self._input_plan_lookup_failed: bool = False
        try:
            self.manager.logic_store.set_data("collab_pause_active", False)
        except Exception:
            pass
        self._availability_base: Set[int] = set()
        self._availability_health_block: Set[int] = set()
        self._availability_mandatory_override: Dict[int, bool] = {}
        self._mission_progress_exporter = MissionProgressExporter(
            log_callback=self.manager._log
        )
        self._prior_mission_replan = PriorMissionReplanCoordinator(self)
        self._roi_caution_active: bool = False
        self._roi_caution_started_ms: Optional[int] = None
        self._roi_caution_timer: Optional[threading.Timer] = None
        self._roi_caution_snapshot: List[Dict[str, Any]] = []
        self._roi_caution_triggered: bool = False
        self._pending_plan_update_meta: Optional[Dict[str, Any]] = None
        self._last_processed_0903_signature: Optional[Tuple[Optional[int], Optional[int]]] = None

    def trigger_prior_mission_replan(self) -> None:
        """Expose prior mission replan processing for immediate 0202 handling."""
        try:
            self._prior_mission_replan.process(self._plan_context)
        except Exception as exc:
            self.manager._log(
                "PRIOR_MISSION",
                "WARN",
                f"0202 replan handler failed: {exc}",
            )

    def _recompute_availability(self) -> None:
        available: Set[int] = set(self._availability_base)
        if self._availability_health_block:
            available.difference_update(self._availability_health_block)
        for aid, forced in self._availability_mandatory_override.items():
            if forced:
                available.add(aid)
            else:
                available.discard(aid)
        ordered = sorted(available)
        try:
            self.manager.logic_store.set_data("input_plan_available_aircraft", ordered)
        except Exception:
            pass
        write_vehicle_status(ordered)

    def _set_baseline_availability(self, aircraft_ids: Iterable[int]) -> None:
        baseline: Set[int] = set()
        for value in aircraft_ids or []:
            try:
                baseline.add(int(value))
            except (TypeError, ValueError):
                continue
        if baseline != self._availability_base:
            self._availability_base = baseline
            if self._availability_health_block:
                self._availability_health_block = {
                    aid for aid in self._availability_health_block if aid in baseline
                }
            if self._availability_mandatory_override:
                self._availability_mandatory_override = {
                    aid: flag
                    for aid, flag in self._availability_mandatory_override.items()
                    if aid in baseline
                }
            try:
                self.manager.logic_store.set_data(
                    "availability_baseline", sorted(self._availability_base)
                )
            except Exception:
                pass
            self._recompute_availability()

    def _update_health_based_availability(self, agent_states: Iterable[Any]) -> None:
        if agent_states is None:
            return

        unhealthy: Set[int] = set()
        observed: Set[int] = set()

        for agent_state in agent_states:
            aircraft_id = self._to_int(self._safe_get(agent_state, "aircraftID", "AircraftID"))
            if aircraft_id is None:
                continue

            observed.add(aircraft_id)

            health_code = self._to_int(self._safe_get(agent_state, "health", "Health"))
            if health_code is None or health_code != 1:
                unhealthy.add(aircraft_id)
                continue

            coord = self._safe_get(agent_state, "coordinate", "Coordinate")
            if coord is None:
                unhealthy.add(aircraft_id)
                continue
            latitude = self._safe_get(coord, "latitude", "Latitude")
            longitude = self._safe_get(coord, "longitude", "Longitude")
            altitude = self._safe_get(coord, "altitude", "Altitude")
            if latitude is None or longitude is None or altitude is None:
                unhealthy.add(aircraft_id)
                continue

            fuel_value = self._safe_get(agent_state, "fuel", "Fuel")
            if fuel_value is None:
                unhealthy.add(aircraft_id)
                continue

        expected_ids: Set[int] = set(self._availability_base)
        if self._availability_mandatory_override:
            for aid, forced in self._availability_mandatory_override.items():
                if forced:
                    expected_ids.add(aid)
                else:
                    expected_ids.discard(aid)

        missing_ids = {aid for aid in expected_ids if aid not in observed}
        combined_unhealthy = unhealthy.union(missing_ids)

        if combined_unhealthy != self._availability_health_block:
            self._availability_health_block = combined_unhealthy
            self._recompute_availability()

    # ------------------------------------------------------------------ #
    # Message Hooks
    # ------------------------------------------------------------------ #

    def handle_message(self, msg_id: str, data: Any) -> None:
        if msg_id == "0803":
            self._handle_collab_command(data)
        elif msg_id == "0802":
            self._handle_mandatory_command(data)
        elif msg_id == "0402":
            self._handle_situation_awareness(data)
        elif msg_id == "0201":
            self._handle_new_input_plan(data)
        elif msg_id == "0305":
            self._handle_replan_status(data)
        elif msg_id == "0702":
            self._handle_decision_result(data)
        elif msg_id == "0903":
            self._handle_mission_update_request(data)

    def _handle_decision_result(self, data: Any) -> None:
        ignore_val = getattr(data, "ignore", None)
        plan_val = getattr(data, "missionPlanID", None)
        if isinstance(data, DecisionResultModel):
            ignore_val = data.ignore
            plan_val = data.missionPlanID
        if isinstance(data, dict):
            ignore_val = data.get("ignore", ignore_val)
            plan_val = data.get("missionPlanID", plan_val)
        try:
            ignore_int = int(ignore_val) if ignore_val is not None else None
        except (TypeError, ValueError):
            ignore_int = None
        try:
            plan_int = int(plan_val) if plan_val is not None else None
        except (TypeError, ValueError):
            plan_int = None

        self._pending_decision_command = (ignore_int, plan_int)

        if ignore_int == 2 and plan_int is not None:
            self.manager._log(
                "MON_LOGIC", "INFO", f"0702 decision received: apply missionPlanID={plan_int}"
            )
        elif ignore_int == 1:
            self.manager._log(
                "MON_LOGIC", "INFO", "0702 decision received: keep existing mission plan"
            )
            self._pending_mission_plan_id = None
        else:
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                f"0702 decision received: ignore={ignore_int}, missionPlanID={plan_int}",
            )

        self._process_mission_plan_update()

    def _handle_mission_update_request(self, data: Any) -> None:
        """React immediately when a performance mission update (0903) arrives."""
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            "[0903] Performance mission update request received. Refreshing mission plan.",
        )
        self._process_mission_plan_update()

    def _track_plan_update_request(
        self,
        plan_id: Optional[int],
        request_timestamp: Optional[int],
        raw_value: Any = None,
    ) -> None:
        """Store metadata about the latest 0903 request for GUI purposes."""
        if (
            self._pending_plan_update_meta
            and self._pending_plan_update_meta.get("source") == "0903"
            and self._pending_plan_update_meta.get("requestedPlanID") == plan_id
            and self._pending_plan_update_meta.get("requestTimestamp") == request_timestamp
        ):
            return
        self._pending_plan_update_meta = {
            "source": "0903",
            "requestedPlanID": plan_id,
            "requestTimestamp": request_timestamp,
            "rawMissionPlanID": raw_value,
            "receivedAt": _ms_since_2000(),
        }

    def _emit_plan_update_status(
        self,
        status: str,
        *,
        plan_id: Optional[int] = None,
        extra: Optional[Dict[str, Any]] = None,
        finalize: bool = False,
    ) -> None:
        meta = self._pending_plan_update_meta
        if not meta or meta.get("source") != "0903":
            return
        payload = {
            "status": status,
            "planID": plan_id if plan_id is not None else meta.get("requestedPlanID"),
            "requestedPlanID": meta.get("requestedPlanID"),
            "source": "0903",
            "stateTimestamp": _ms_since_2000(),
        }
        if meta.get("requestTimestamp") is not None:
            payload["requestTimestamp"] = meta["requestTimestamp"]
        if meta.get("receivedAt") is not None:
            payload["receivedAt"] = meta["receivedAt"]
        if meta.get("rawMissionPlanID") is not None:
            payload["rawMissionPlanID"] = meta["rawMissionPlanID"]
        if extra:
            payload.update(extra)
        try:
            self.manager.logic_store.set_data("mission_update_status", payload)
        except Exception:
            pass
        if self.manager.gui_update_callback:
            try:
                self.manager.gui_update_callback("logic", "mission_update_status", payload)
            except Exception:
                pass
        if finalize:
            self._last_processed_0903_signature = (
                meta.get("requestedPlanID"),
                meta.get("requestTimestamp"),
            )
            self._pending_plan_update_meta = None

    def on_system_mode_changed(self, mode: int) -> None:
        if mode not in (3, 4) and self._collab_pause_active:
            self._deactivate_collab_pause(reason=f"system mode changed to {mode}")

    def _process_mission_plan_update(self) -> None:
        candidate_plan_id: Optional[int] = None
        candidate_source: Optional[str] = None
        candidate_request_ts: Optional[int] = None
        raw_candidate_value: Any = None

        try:
            data_0903 = self.manager.receive_store.get_data("0903")
        except Exception:
            data_0903 = None
        if data_0903:
            candidate_source = "0903"
            raw_candidate_value = getattr(data_0903, "missionPlanID", None)
            candidate_plan_id = raw_candidate_value
            if candidate_plan_id is None and isinstance(data_0903, dict):
                candidate_plan_id = data_0903.get("missionPlanID")
            candidate_request_ts = getattr(data_0903, "timestamp", None)
            if candidate_request_ts is None and isinstance(data_0903, dict):
                candidate_request_ts = data_0903.get("timestamp")

        if candidate_plan_id is None:
            try:
                data_0902 = self.manager.receive_store.get_data("0902")
            except Exception:
                data_0902 = None
            if data_0902:
                candidate_plan_id = getattr(data_0902, "missionPlanID", None)
                if candidate_plan_id is None and isinstance(data_0902, dict):
                    candidate_plan_id = data_0902.get("missionPlanID")

        invalid_request = False
        if candidate_plan_id is not None:
            try:
                candidate_plan_id = int(candidate_plan_id)
            except (TypeError, ValueError):
                invalid_request = candidate_source == "0903"
                candidate_plan_id = None
        elif candidate_source == "0903":
            invalid_request = True

        if invalid_request:
            already_handled_invalid = (
                self._pending_plan_update_meta is None
                and self._last_processed_0903_signature == (None, candidate_request_ts)
            )
            if not already_handled_invalid:
                self._track_plan_update_request(None, candidate_request_ts, raw_candidate_value)
                self._emit_plan_update_status(
                    "failed",
                    plan_id=None,
                    extra={"reason": "유효한 missionPlanID가 포함되지 않았습니다."},
                    finalize=True,
                )
            candidate_source = None

        candidate_signature = None
        if candidate_source == "0903" and candidate_plan_id is not None:
            candidate_signature = (candidate_plan_id, candidate_request_ts)
            already_processed = (
                self._pending_plan_update_meta is None
                and self._last_processed_0903_signature == candidate_signature
            )
            if not already_processed:
                meta = self._pending_plan_update_meta
                is_new_request = not (
                    meta
                    and meta.get("source") == "0903"
                    and meta.get("requestedPlanID") == candidate_plan_id
                    and meta.get("requestTimestamp") == candidate_request_ts
                )
                self._track_plan_update_request(
                    candidate_plan_id, candidate_request_ts, raw_candidate_value
                )
                if is_new_request:
                    self._emit_plan_update_status("requested", plan_id=candidate_plan_id)
            else:
                candidate_source = None

        if (
            candidate_plan_id is None
            and self._pending_mission_plan_id is None
            and self._current_mission_plan_id is None
        ):
            candidate_plan_id = self._scan_latest_mission_plan_id()

        if candidate_plan_id is not None:
            self._pending_mission_plan_id = candidate_plan_id

        decision_ignore = None
        decision_plan = None
        if self._pending_decision_command is not None:
            decision_ignore, decision_plan = self._pending_decision_command

        if decision_plan is not None:
            try:
                decision_plan = int(decision_plan)
            except (TypeError, ValueError):
                decision_plan = None

        mission_plan_id = None
        command_consumed = False

        if decision_ignore == 2 and decision_plan is not None:
            mission_plan_id = decision_plan
            command_consumed = True
            self._pending_mission_plan_id = None
            self._pending_plan_update_meta = None
        elif decision_ignore == 1:
            mission_plan_id = self._current_mission_plan_id
            command_consumed = True
            self._pending_mission_plan_id = None
        elif self._pending_mission_plan_id is not None:
            mission_plan_id = self._pending_mission_plan_id
        else:
            mission_plan_id = self._current_mission_plan_id

        if mission_plan_id is None:
            if command_consumed:
                self._pending_decision_command = None
            if self._pending_plan_update_meta and self._pending_plan_update_meta.get("source") == "0903":
                self._emit_plan_update_status(
                    "failed",
                    plan_id=None,
                    extra={"reason": "적용할 MissionPlanID를 결정하지 못했습니다."},
                    finalize=True,
                )
            return

        try:
            mission_plan_id = int(mission_plan_id)
        except (TypeError, ValueError):
            if command_consumed:
                self._pending_decision_command = None
            if self._pending_plan_update_meta and self._pending_plan_update_meta.get("source") == "0903":
                self._emit_plan_update_status(
                    "failed",
                    plan_id=None,
                    extra={"reason": "MissionPlanID 형식이 올바르지 않습니다."},
                    finalize=True,
                )
            return

        active_0903_request = bool(
            self._pending_plan_update_meta
            and self._pending_plan_update_meta.get("source") == "0903"
        )

        if mission_plan_id == self._current_mission_plan_id and not active_0903_request:
            if (
                self._pending_plan_update_meta
                and self._pending_plan_update_meta.get("source") == "0903"
            ):
                self._emit_plan_update_status(
                    "applied",
                    plan_id=mission_plan_id,
                    extra={"detail": "이미 적용 중인 계획입니다."},
                    finalize=True,
                )
            if command_consumed:
                self._pending_decision_command = None
            return

        mission_plan_path = db_paths.get_db_subpath("MissionPlan", f"{mission_plan_id}.json")
        fallback_applied: Optional[int] = None
        if not mission_plan_path.exists():
            fallback_id = self._scan_latest_mission_plan_id()
            if fallback_id and fallback_id != mission_plan_id:
                alt_path = db_paths.get_db_subpath("MissionPlan", f"{fallback_id}.json")
                if alt_path.exists():
                    self.manager._log(
                        "MON_LOGIC",
                        "WARN",
                        f"MissionPlan {mission_plan_id} not found, fallback to {fallback_id}.",
                    )
                    fallback_applied = fallback_id
                    mission_plan_id = fallback_id
                    mission_plan_path = alt_path
                else:
                    if self._pending_plan_update_meta and self._pending_plan_update_meta.get("source") == "0903":
                        self._emit_plan_update_status(
                            "failed",
                            plan_id=fallback_id,
                            extra={"reason": "미션 계획 파일을 찾을 수 없습니다."},
                            finalize=True,
                        )
                    if command_consumed:
                        self._pending_decision_command = None
                    return
            else:
                if self._pending_plan_update_meta and self._pending_plan_update_meta.get("source") == "0903":
                    self._emit_plan_update_status(
                        "failed",
                        plan_id=mission_plan_id,
                        extra={"reason": "미션 계획 파일을 찾을 수 없습니다."},
                        finalize=True,
                    )
                if command_consumed:
                    self._pending_decision_command = None
                return
        try:
            context = self._load_mission_plan_context(mission_plan_id)
        except FileNotFoundError as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"MissionPlan {mission_plan_id} file missing: {exc}",
            )
            if self._pending_plan_update_meta and self._pending_plan_update_meta.get("source") == "0903":
                self._emit_plan_update_status(
                    "failed",
                    plan_id=mission_plan_id,
                    extra={"reason": "MissionPlan 파일이 존재하지 않습니다."},
                    finalize=True,
                )
            if command_consumed:
                self._pending_decision_command = None
            return
        except Exception as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"MissionPlan {mission_plan_id} load failed: {exc}",
            )
            if self._pending_plan_update_meta and self._pending_plan_update_meta.get("source") == "0903":
                self._emit_plan_update_status(
                    "failed",
                    plan_id=mission_plan_id,
                    extra={"reason": f"MissionPlan 로드 실패: {exc}"},
                    finalize=True,
                )
            if command_consumed:
                self._pending_decision_command = None
            return
        previous_plan_id = self._current_mission_plan_id
        tracker_initialized = bool(self._input_mission_tracker)
        self._plan_context = context
        self._mission_progress_exporter.reset()
        self._current_mission_plan_id = mission_plan_id
        reinitialized = (
            active_0903_request
            or previous_plan_id != mission_plan_id
            or not tracker_initialized
        )
        if reinitialized:
            self._initialize_input_tracker(context)
            self._current_input_mission_id = self._find_next_input_mission_id(
                initial=True
            )
            self._update_plan_context_active_input()
        try:
            self.manager.logic_store.set_data("current_mission_plan", context)
        except Exception:
            pass
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            f"MissionPlan {mission_plan_id} loaded for monitoring",
        )
        extra_status: Dict[str, Any] = {
            "appliedPlanID": mission_plan_id,
            "previousPlanID": previous_plan_id,
        }
        same_plan_reload = active_0903_request and previous_plan_id == mission_plan_id
        if fallback_applied is not None:
            extra_status["fallbackPlanID"] = fallback_applied
        if same_plan_reload:
            extra_status["detail"] = "동일 ID 재적용 요청으로 갱신했습니다."
        if (
            self._pending_plan_update_meta
            and self._pending_plan_update_meta.get("source") == "0903"
        ):
            self._emit_plan_update_status(
                "applied",
                plan_id=mission_plan_id,
                extra=extra_status,
                finalize=True,
            )
        self._pending_decision_command = None
        if self._pending_mission_plan_id == mission_plan_id:
            self._pending_mission_plan_id = None

    def _scan_latest_mission_plan_id(self) -> Optional[int]:
        """MissionPlan ?붾젆?곕━?먯꽌 媛??理쒖떊??plan ID瑜?異붾줎?쒕떎."""
        try:
            mission_plan_dir = db_paths.get_db_subpath("MissionPlan")
        except Exception:
            return None
        try:
            entries = list(mission_plan_dir.glob("*.json"))
        except Exception:
            return None
        if not entries:
            return None
        try:
            latest = max(entries, key=lambda p: p.stat().st_mtime)
        except Exception:
            return None
        try:
            return int(latest.stem)
        except (TypeError, ValueError):
            return None

    def _load_mission_plan_context(self, mission_plan_id: int) -> Dict[str, Any]:
        mission_plan_path = db_paths.get_db_subpath("MissionPlan", f"{mission_plan_id}.json")
        with mission_plan_path.open("r", encoding="utf-8") as fh:
            plan_data = json.load(fh)

        aircraft_map: Dict[int, Dict[str, Any]] = {}
        input_ids: List[int] = []
        input_id_seen: Set[int] = set()

        for entry in plan_data.get("aircraftList", []):
            try:
                aircraft_id = int(entry.get("aircraftID"))
                package_id = int(entry.get("individualMissionPackageID"))
            except (TypeError, ValueError):
                continue

            imp_path = db_paths.get_db_subpath(
                "IndividualMissionPlan", f"{package_id}.json"
            )
            try:
                with imp_path.open("r", encoding="utf-8") as fh:
                    imp_data = json.load(fh)
            except FileNotFoundError:
                self.manager._log(
                    "MON_LOGIC",
                    "WARN",
                    f"IndividualMissionPlan {package_id} file missing",
                )
                continue
            except Exception as exc:
                self.manager._log(
                    "MON_LOGIC",
                    "WARN",
                    f"IndividualMissionPlan {package_id} load failed: {exc}",
                )
                continue

            missions = []
            for mission_entry in imp_data.get("individualMissionList", []):
                individual_mission_id = mission_entry.get("individualMissionID")
                path_id = mission_entry.get("pathID")
                related = mission_entry.get("relatedMission") or {}
                input_mission_id = related.get("inputMissionID")
                if input_mission_id is not None:
                    try:
                        normalized_id = int(input_mission_id)
                    except (TypeError, ValueError):
                        normalized_id = None
                    if (
                        normalized_id is not None
                        and normalized_id not in input_id_seen
                    ):
                        input_id_seen.add(normalized_id)
                        input_ids.append(normalized_id)

                waypoint_ids = []
                if path_id is not None:
                    try:
                        fp_path = db_paths.get_db_subpath(
                            "FlightPath", f"{int(path_id)}.json"
                        )
                        with fp_path.open("r", encoding="utf-8") as fh:
                            fp_data = json.load(fh)
                        waypoint_ids = [
                            int(wp.get("waypointID"))
                            for wp in fp_data.get("waypointList", [])
                            if wp.get("waypointID") is not None
                        ]
                    except FileNotFoundError:
                        self.manager._log(
                            "MON_LOGIC",
                            "WARN",
                            f"FlightPath {path_id} file missing",
                        )
                    except Exception as exc:
                        self.manager._log(
                            "MON_LOGIC",
                            "WARN",
                            f"FlightPath {path_id} load failed: {exc}",
                        )

                missions.append(
                    {
                        "individualMissionID": int(individual_mission_id)
                        if individual_mission_id is not None
                        else 0,
                        "pathID": int(path_id) if path_id is not None else None,
                        "waypoints": waypoint_ids,
                        "inputMissionID": int(input_mission_id)
                        if input_mission_id is not None
                        else None,
                        "isDone": bool(mission_entry.get("isDone")),
                    }
                )

            waypoint_map: Dict[int, Any] = {}
            for idx, mission in enumerate(missions):
                for pos, waypoint_id in enumerate(mission.get("waypoints") or []):
                    waypoint_map[waypoint_id] = (idx, pos)

            aircraft_map[aircraft_id] = {
                "missions": missions,
                "waypoint_map": waypoint_map,
                "individualMissionPackageID": package_id,
            }

        return {
            "missionPlanID": mission_plan_id,
            "inputMissionPackageID": plan_data.get("inputMissionPackageID"),
            "aircraft": aircraft_map,
            "inputMissionIDs": list(input_ids),
        }

    def _initialize_input_tracker(self, context: Dict[str, Any]) -> None:
        tracker: Dict[int, Dict[str, Set[Tuple[int, int, Optional[int], int]]]] = {}
        reverse_map: Dict[Tuple[int, int, Optional[int], int], int] = {}
        file_map: Dict[Tuple[int, int, Optional[int], int], Tuple[int, int, int]] = {}
        self._input_mission_package_id = None
        self._input_mission_plan_path = None
        self._input_mission_index_map = {}
        self._input_plan_lookup_failed = False
        package_value = context.get("inputMissionPackageID")
        if package_value is not None:
            try:
                self._input_mission_package_id = int(package_value)
            except (TypeError, ValueError):
                self._input_mission_package_id = None
        if self._input_mission_package_id is not None:
            try:
                self._input_mission_plan_path = db_paths.get_db_subpath(
                    "InputMissionPlan", f"{self._input_mission_package_id}.json"
                )
            except Exception:
                self._input_mission_plan_path = None
            else:
                self._refresh_input_mission_index_map(force=True)
        for aircraft_id, payload in (context.get("aircraft") or {}).items():
            missions = payload.get("missions") or []
            try:
                aircraft_id_int = int(aircraft_id)
            except (TypeError, ValueError):
                continue
            for idx, mission in enumerate(missions):
                input_id = mission.get("inputMissionID")
                if input_id is None:
                    continue
                try:
                    input_id_int = int(input_id)
                except (TypeError, ValueError):
                    continue
                try:
                    mission_id = int(mission.get("individualMissionID") or 0)
                except (TypeError, ValueError):
                    mission_id = 0
                path_id = mission.get("pathID")
                if path_id is not None:
                    try:
                        path_id = int(path_id)
                    except (TypeError, ValueError):
                        path_id = None
                key = (aircraft_id_int, mission_id, path_id, idx)
                entry = tracker.setdefault(
                    input_id_int, {"total": set(), "completed": set(), "inactive": set()}
                )
                entry["total"].add(key)
                reverse_map[key] = input_id_int
                package_id = payload.get("individualMissionPackageID")
                try:
                    package_id_int = int(package_id)
                except (TypeError, ValueError):
                    continue
                file_map[key] = (package_id_int, idx, aircraft_id_int)
        self._input_mission_tracker = tracker
        self._mission_to_input = reverse_map
        self._completed_input_ids = set()
        self._collab_completion_sent = False
        self._monitoring_suspended = False
        self._active_aircraft_ids = set()
        self._mission_progress_max = {}
        self._mission_file_map = file_map
        self._current_input_mission_id = None
        self._input_completion_notified = set()

    def _extract_available_ids_from_payload(self, payload: Any) -> List[int]:
        raw_list = self._safe_get(payload, "availableAircraftList", "AvailableAircraftList")
        extracted: List[int] = []
        if raw_list is None:
            return extracted
        for item in raw_list:
            candidate = self._safe_get(item, "aircraftID", "AircraftID")
            if candidate is None:
                continue
            try:
                extracted.append(int(candidate))
            except (TypeError, ValueError):
                continue
        return extracted

    def _load_available_ids_from_package(self, package_id: Optional[int]) -> List[int]:
        if package_id is None:
            return []
        try:
            plan_path = db_paths.get_db_subpath("InputMissionPlan", f"{int(package_id)}.json")
        except Exception:
            return []
        if not plan_path.exists():
            return []
        try:
            data = json.loads(plan_path.read_text(encoding="utf-8"))
        except Exception:
            return []
        return self._extract_available_ids_from_payload(data)

    def _refresh_input_mission_index_map(self, force: bool = False) -> Dict[int, int]:
        if not force and self._input_mission_index_map:
            return self._input_mission_index_map
        mapping: Dict[int, int] = {}
        path = self._input_mission_plan_path
        if not path:
            self._input_plan_lookup_failed = True
            self._input_mission_index_map = mapping
            return mapping
        try:
            with path.open("r", encoding="utf-8") as fh:
                data = json.load(fh)
        except FileNotFoundError:
            if not self._input_plan_lookup_failed:
                self.manager._log(
                    "MON_LOGIC",
                    "WARN",
                    f"InputMissionPlan file missing (package={self._input_mission_package_id})",
                )
            self._input_plan_lookup_failed = True
            self._input_mission_index_map = mapping
            return mapping
        except Exception as exc:
            if not self._input_plan_lookup_failed:
                self.manager._log(
                    "MON_LOGIC",
                    "WARN",
                    f"InputMissionPlan load failed (package={self._input_mission_package_id}): {exc}",
                )
            self._input_plan_lookup_failed = True
            self._input_mission_index_map = mapping
            return mapping
        mission_list = data.get("inputMissionList")
        available_ids: Set[int] = set()
        order: List[int] = []
        order_seen: Set[int] = set()
        if isinstance(mission_list, list):
            for idx, mission in enumerate(mission_list):
                mission_id = mission.get("inputMissionID")
                try:
                    mission_id_int = int(mission_id)
                except (TypeError, ValueError):
                    continue
                mapping[mission_id_int] = idx
                if mission_id_int not in order_seen:
                    order_seen.add(mission_id_int)
                    order.append(mission_id_int)
        self._input_mission_order = order
        available_list = data.get("availableAircraftList")
        if isinstance(available_list, list):
            for item in available_list:
                if isinstance(item, dict):
                    candidate = item.get("aircraftID")
                else:
                    candidate = getattr(item, "aircraftID", None)
                try:
                    if candidate is None:
                        continue
                    available_ids.add(int(candidate))
                except (TypeError, ValueError):
                    continue
        self._input_plan_lookup_failed = False
        self._input_mission_index_map = mapping
        stored_ids = sorted(available_ids)
        self._set_baseline_availability(stored_ids)
        return mapping

    def _resolve_mission_tracker_key(
        self,
        keys: Set[Tuple[int, int, Optional[int], int]],
        target: Tuple[int, int, Optional[int], int],
    ) -> Optional[Tuple[int, int, Optional[int], int]]:
        if target in keys:
            return target
        for candidate in keys:
            if candidate[:3] == target[:3]:
                return candidate
        for candidate in keys:
            if candidate[0] == target[0] and candidate[1] == target[1]:
                return candidate
        return None

    def _update_plan_context_active_input(self) -> None:
        if self._plan_context is not None:
            self._plan_context["activeInputMissionID"] = self._current_input_mission_id
        try:
            self.manager.logic_store.set_data(
                "active_input_mission_id", self._current_input_mission_id
            )
        except Exception:
            pass

    def _handle_situation_awareness(self, data: Any) -> None:
        message_ts = self._to_int(self._safe_get(data, "timestamp", "Timestamp"))
        try:
            previous = load_target_info()
        except Exception:
            previous = {"targetList": {}}
        try:
            info, new_detections = update_target_info_from_0402(data)
        except Exception as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"0402 targetInfo 업데이트 실패: {exc}",
            )
            return

        replan_triggered = False
        prev_targets: Dict[str, Dict[str, Any]] = (previous or {}).get("targetList") or {}
        new_targets: Dict[str, Dict[str, Any]] = info.get("targetList") or {}

        if new_targets:
            target_replan_triggered = self._process_target_replan_candidates(new_targets)
            if target_replan_triggered:
                replan_triggered = True
                try:
                    self._trigger_replan_if_active()
                except Exception as exc:
                    self.manager._log(
                        "MON_LOGIC",
                        "WARN",
                        f"0402 표적 기반 재계획 트리거 실행 실패: {exc}",
                    )

        try:
            self.manager.logic_store.set_data("targetInfo", info)
            self.manager.logic_store.set_data("targetInfoNewDetections", new_detections)
        except Exception:
            pass
        else:
            if new_detections:
                self.manager._log(
                    "MON_LOGIC",
                    "INFO",
                    f"0402 신규 표적 감지 {len(new_detections)}건",
                )
                if not replan_triggered:
                    try:
                        self._trigger_replan_if_active()
                        replan_triggered = True
                    except Exception as exc:
                        self.manager._log(
                            "MON_LOGIC",
                            "WARN",
                            f"새 표적 감지 후 재계획 트리거 시도 실패: {exc}",
                        )

        prev_count = len(prev_targets)
        new_count = len(new_targets)
        if new_count != prev_count:
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                f"0402 targetInfo 갱신: 대상 수 {prev_count} → {new_count}",
            )

        if new_targets:
            sample_key, sample_value = next(iter(new_targets.items()))
            coord = sample_value.get("coordinate") or {}
            watcher_id = sample_value.get("watcherID")
            if sample_key.startswith("unknown-"):
                self.manager._log(
                    "MON_LOGIC",
                    "INFO",
                    f"0402 ROI 업데이트: key={sample_key}, watcher={watcher_id}, "
                    f"lat={coord.get('latitude')}, lon={coord.get('longitude')}",
                )
            else:
                self.manager._log(
                    "MON_LOGIC",
                    "INFO",
                    f"0402 타겟 업데이트: key={sample_key}, watcher={watcher_id}, "
                    f"threat={sample_value.get('threat')}, "
                    f"lat={coord.get('latitude')}, lon={coord.get('longitude')}",
                )

        roi_entries = self._collect_active_roi_entries(new_targets)
        self._update_roi_caution_state(roi_entries, message_ts)

        pending_targets: List[str] = []
        for key, entry in new_targets.items():
            if not isinstance(entry, dict):
                continue
            if self._to_int(entry.get("isUsed")) == 1:
                continue
            if self._to_int(entry.get("isIgnored")) == 1:
                continue
            if bool(entry.get("isDestroyed")):
                continue
            pending_targets.append(str(key))

        if pending_targets and not replan_triggered:
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                f"isUsed=0 상태 표적 재계획 확인 요청 (count={len(pending_targets)}, sample={pending_targets[0]})",
            )
            try:
                self._trigger_replan_if_active()
            except Exception as exc:
                self.manager._log(
                    "MON_LOGIC",
                    "WARN",
                    f"기존 표적 재확인 재계획 트리거 실패: {exc}",
                )

    def _process_target_replan_candidates(self, target_map: Dict[str, Any]) -> bool:
        """
        Scan DSS targetInfo data for actionable targets (isUsed=0, known target) and
        enqueue replan situations with descriptive reasons. Returns True if at least one
        new trigger was added.
        """
        if not isinstance(target_map, dict):
            self._target_trigger_history.clear()
            return False

        triggered = False
        actionable_keys: Set[str] = set()

        for key, entry in target_map.items():
            key_str = str(key)
            if not self._is_actionable_target_entry(key_str, entry):
                continue
            keep_history = True

            previous = self._target_trigger_history.get(key_str)
            last_updated = self._extract_target_timestamp(entry)
            if last_updated is None and previous is not None:
                last_updated = previous
            if previous is not None and last_updated is not None and last_updated <= previous:
                actionable_keys.add(key_str)
                continue

            payload = self._build_target_replan_payload(key_str, entry)
            if not payload:
                actionable_keys.add(key_str)
                continue

            self._append_replan_situation(payload)
            if last_updated is not None:
                self._target_trigger_history[key_str] = last_updated
            else:
                self._target_trigger_history[key_str] = self._current_time_ms()

            reason = payload.get(REPLAN_FIELD_REASON, "")
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                f"0402 표적 기반 재계획 트리거 적재: key={key_str}, reason={reason}",
            )
            triggered = True
            keep_history = False

            self._mark_target_as_used_for_trigger(key_str, entry)

            if keep_history:
                actionable_keys.add(key_str)

        stale_keys = [key for key in self._target_trigger_history.keys() if key not in actionable_keys]
        for key in stale_keys:
            self._target_trigger_history.pop(key, None)

        return triggered

    def _is_actionable_target_entry(self, key: str, entry: Any) -> bool:
        if not isinstance(entry, dict):
            return False
        if key.startswith("unknown-"):
            return False
        target_id = self._to_int(entry.get("targetID"))
        if target_id is None:
            return False
        if self._to_int(entry.get("isUsed")) == 1:
            return False
        if self._to_int(entry.get("isIgnored")) == 1:
            return False
        if bool(entry.get("isDestroyed")):
            return False
        return True

    def _extract_target_timestamp(self, entry: Dict[str, Any]) -> Optional[int]:
        if not isinstance(entry, dict):
            return None
        for field in ("lastUpdated", "LastUpdated", "timestamp", "Timestamp", "lastObserved"):
            value = entry.get(field)
            ts = self._to_int(value)
            if ts is not None:
                return ts
        return self._to_int(entry.get("firstDetected"))

    def _build_target_replan_payload(self, key: str, entry: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        target_id = self._to_int(entry.get("targetID"))
        if target_id is None:
            return None
        watcher_id = self._to_int(entry.get("watcherID"))
        if watcher_id is None:
            watcher_id = self._extract_watcher_from_key(key)
        target_type = self._to_int(entry.get("targetType"))
        reason = self._format_target_reason(watcher_id, target_type, target_id)
        detail_payload = {
            "trigger": "0402",
            "targetKey": key,
            "targetID": target_id,
            "watcherID": watcher_id,
            "targetType": target_type,
            "threat": entry.get("threat"),
            "targetInFrame": entry.get("targetInFrame"),
            "coordinate": entry.get("coordinate"),
            "preferredOptionCount": 3,
            "snapshot": entry,
        }
        return {
            REPLAN_FIELD_SITUATION: TARGET_REPLAN_SITUATION_LABEL,
            REPLAN_FIELD_REASON: reason,
            REPLAN_FIELD_DETAIL: detail_payload,
            "original_message_id": "0402",
        }

    def _append_replan_situation(self, payload: Dict[str, Any]) -> None:
        try:
            existing = self.manager.logic_store.get_data("ReplanSituations")
        except Exception:
            existing = None
        if not isinstance(existing, list):
            existing = []
        existing.append(payload)
        self.manager.logic_store.set_data("ReplanSituations", existing)
        # expose for UI immediately
        try:
            self.manager.logic_store.set_data(
                "replan_triggers",
                [payload],
            )
        except Exception:
            pass

    def _mark_target_as_used_for_trigger(self, key: str, entry: Dict[str, Any]) -> None:
        target_id = self._to_int(entry.get("targetID"))
        watcher_id = self._to_int(entry.get("watcherID"))
        payload = {"key": key, "targetID": target_id, "watcherID": watcher_id}
        try:
            mark_targets_as_used([payload])
        except Exception as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"0402 표적 isUsed 업데이트 실패(key={key}): {exc}",
            )
            return
        if isinstance(entry, dict):
            entry["isUsed"] = 1

    def _extract_watcher_from_key(self, key: str) -> Optional[int]:
        parts = str(key).split("-")
        if len(parts) != 2:
            return None
        return self._to_int(parts[1])

    def _format_target_reason(
        self,
        watcher_id: Optional[int],
        target_type: Optional[int],
        target_id: int,
    ) -> str:
        watcher_label = self._format_watcher_label(watcher_id)
        target_label = self._resolve_target_type_label(target_type)
        return f"{watcher_label} - {target_label}(ID-{target_id}) 발견으로 인한 재계획"

    def _format_watcher_label(self, watcher_id: Optional[int]) -> str:
        if watcher_id is None:
            return "무인기"
        return WATCHER_CALLSIGN_MAP.get(watcher_id, f"무인기 {watcher_id}")

    def _resolve_target_type_label(self, target_type: Optional[int]) -> str:
        if target_type in TARGET_TYPE_LABELS:
            return TARGET_TYPE_LABELS[target_type]
        if target_type is None:
            return TARGET_TYPE_LABELS[None]
        return TARGET_TYPE_LABELS.get(None, "표적")

    def _collect_active_roi_entries(self, target_map: Dict[str, Any]) -> List[Dict[str, Any]]:
        entries: List[Dict[str, Any]] = []
        if not isinstance(target_map, dict):
            return entries
        for key, entry in target_map.items():
            key_str = str(key)
            if not key_str.startswith("unknown-"):
                continue
            if not isinstance(entry, dict):
                continue
            if bool(entry.get("isDestroyed")):
                continue
            watcher_id = self._to_int(entry.get("watcherID"))
            if watcher_id is None:
                watcher_id = self._extract_watcher_from_key(key_str)
            entries.append(
                {
                    "key": key_str,
                    "watcherID": watcher_id,
                    "coordinate": entry.get("coordinate"),
                    "threat": entry.get("threat"),
                    "targetInFrame": entry.get("targetInFrame"),
                    "lastUpdated": self._extract_target_timestamp(entry),
                }
            )
        return entries

    def _update_roi_caution_state(
        self, roi_entries: List[Dict[str, Any]], timestamp_ms: Optional[int]
    ) -> None:
        if roi_entries:
            self._set_roi_caution_state(True, timestamp_ms, roi_entries)
        else:
            self._set_roi_caution_state(False, None, [])

    def _set_roi_caution_state(
        self,
        active: bool,
        timestamp_ms: Optional[int],
        roi_entries: List[Dict[str, Any]],
    ) -> None:
        if active:
            self._roi_caution_snapshot = list(roi_entries)
            if self._roi_caution_active:
                return
            self._roi_caution_active = True
            self._roi_caution_triggered = False
            self._roi_caution_started_ms = timestamp_ms or self._current_time_ms()
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                f"0402 ROI caution 모드 ON (count={len(roi_entries)}, timeout={ROI_CAUTION_TIMEOUT_SECONDS}s)",
            )
            self._schedule_roi_caution_check()
        else:
            if not self._roi_caution_active:
                return
            self._roi_caution_active = False
            self._roi_caution_triggered = False
            self._roi_caution_started_ms = None
            self._roi_caution_snapshot = []
            self._cancel_roi_caution_timer()
            self.manager._log("MON_LOGIC", "INFO", "0402 ROI caution 모드 OFF")

    def _schedule_roi_caution_check(self) -> None:
        self._cancel_roi_caution_timer()
        timer = threading.Timer(ROI_CAUTION_TIMEOUT_SECONDS, self._roi_caution_timeout_fired)
        timer.daemon = True
        self._roi_caution_timer = timer
        timer.start()

    def _cancel_roi_caution_timer(self) -> None:
        timer = self._roi_caution_timer
        if timer is not None:
            try:
                timer.cancel()
            except Exception:
                pass
        self._roi_caution_timer = None

    def _roi_caution_timeout_fired(self) -> None:
        self._roi_caution_timer = None
        if not self._roi_caution_active or self._roi_caution_triggered:
            return
        roi_entries = list(self._roi_caution_snapshot)
        if not roi_entries:
            try:
                target_info = self.manager.logic_store.get_data("targetInfo") or {}
            except Exception:
                target_info = {}
            roi_entries = self._collect_active_roi_entries(target_info.get("targetList") or {})
            self._roi_caution_snapshot = list(roi_entries)
        if not roi_entries:
            self._set_roi_caution_state(False, None, [])
            return
        self._roi_caution_triggered = True
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            "0402 ROI caution 10초 지속 → 재계획 트리거",
        )
        self._trigger_roi_timeout_replan(roi_entries)

    def _trigger_roi_timeout_replan(self, roi_entries: List[Dict[str, Any]]) -> None:
        payload = self._build_roi_replan_payload(roi_entries)
        if not payload:
            return
        self._append_replan_situation(payload)
        try:
            self._trigger_replan_if_active()
        except Exception as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"0402 ROI 기반 재계획 트리거 실행 실패: {exc}",
            )

    def _build_roi_replan_payload(
        self, roi_entries: List[Dict[str, Any]]
    ) -> Optional[Dict[str, Any]]:
        if not roi_entries:
            return None
        sample = roi_entries[0]
        watcher_label = self._format_watcher_label(self._to_int(sample.get("watcherID")))
        reason = f"{watcher_label} ROI 지속 감시 (count={len(roi_entries)})"
        detail_payload = {
            "trigger": "0402_ROI",
            "roiEntries": roi_entries,
            "cautionStartedMs": self._roi_caution_started_ms,
            "timeoutSeconds": ROI_CAUTION_TIMEOUT_SECONDS,
        }
        return {
            REPLAN_FIELD_SITUATION: ROI_REPLAN_SITUATION_LABEL,
            REPLAN_FIELD_REASON: reason,
            REPLAN_FIELD_DETAIL: detail_payload,
            "original_message_id": "0402",
        }

    def _handle_mandatory_command(self, data: Any) -> None:
        aircraft_id = self._to_int(self._safe_get(data, "aircraftID", "AircraftID"))
        mandatory_type = self._to_int(
            self._safe_get(data, "mandatoryType", "MandatoryType")
        )
        if aircraft_id is None or mandatory_type is None:
            return

        hold_state_obj = self.manager.logic_store.get_data(FORCED_HOLD_STATE_KEY)
        hold_state = dict(hold_state_obj) if isinstance(hold_state_obj, dict) else {}

        reason_map = {
            1: "강제대기로 인한 재계획",
            2: "강제귀환으로 인한 재계획",
            3: "강제임무복귀로 인한 재계획",
        }
        replan_reason = reason_map.get(mandatory_type)
        if replan_reason:
            try:
                setattr(data, "replan_reason", replan_reason)
            except Exception:
                pass

        availability_updated = False
        if mandatory_type in (1, 2):
            availability_updated = self._set_aircraft_availability(aircraft_id, False)
        elif mandatory_type == 3:
            availability_updated = self._set_aircraft_availability(aircraft_id, True)

        if availability_updated:
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                f"0802 mandatoryType={mandatory_type} applied; aircraft {aircraft_id} "
                f"{'available' if mandatory_type == 3 else 'unavailable'}.",
            )

        if mandatory_type == 1:
            defer_deadline = time.monotonic() + FORCED_HOLD_DELAY_SECONDS
            try:
                setattr(data, FORCED_HOLD_DEADLINE_ATTR, defer_deadline)
                setattr(data, FORCED_HOLD_REASON_ATTR, FORCED_HOLD_DELAY_REASON)
            except Exception:
                pass
            hold_state[aircraft_id] = {
                "deadline": defer_deadline,
                "timestamp": getattr(data, "timestamp", None),
            }
            self.manager.logic_store.set_data(FORCED_HOLD_STATE_KEY, hold_state)
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                "0802 강제대기 명령 수신 – 10초 경과 후 재계획을 시도합니다.",
            )
            return

        hold_info = hold_state.get(aircraft_id)
        if mandatory_type == 3 and hold_info:
            deadline_value = hold_info.get("deadline")
            if isinstance(deadline_value, (int, float)) and time.monotonic() < deadline_value:
                enriched_info = dict(hold_info)
                enriched_info.setdefault("resume_requested_timestamp", getattr(data, "timestamp", None))
                hold_state[aircraft_id] = enriched_info
                self.manager.logic_store.set_data(FORCED_HOLD_STATE_KEY, hold_state)
                self.manager._log(
                    "MON_LOGIC",
                    "INFO",
                    f"0802 강제임무복귀 명령이 강제대기 유예({FORCED_HOLD_DELAY_SECONDS:.0f}s) 내에 수신되어 재계획을 생략합니다. 대상 기체={aircraft_id}",
                )
                try:
                    if hasattr(data, FORCED_HOLD_DEADLINE_ATTR):
                        delattr(data, FORCED_HOLD_DEADLINE_ATTR)
                except Exception:
                    pass
                try:
                    if hasattr(data, FORCED_HOLD_REASON_ATTR):
                        delattr(data, FORCED_HOLD_REASON_ATTR)
                except Exception:
                    pass
                return

        if hold_info and mandatory_type in (2, 3):
            hold_state.pop(aircraft_id, None)
            self.manager.logic_store.set_data(FORCED_HOLD_STATE_KEY, hold_state)

        try:
            if hasattr(data, FORCED_HOLD_DEADLINE_ATTR):
                delattr(data, FORCED_HOLD_DEADLINE_ATTR)
        except Exception:
            pass
        try:
            if hasattr(data, FORCED_HOLD_REASON_ATTR):
                delattr(data, FORCED_HOLD_REASON_ATTR)
        except Exception:
            pass

        should_trigger_replan = False
        if mandatory_type == 2:
            should_trigger_replan = True
        elif mandatory_type == 3:
            should_trigger_replan = True

        if should_trigger_replan:
            self._trigger_replan_if_active()

    def _trigger_replan_if_active(self) -> None:
        try:
            system_mode = int(self.manager.logic_store.get_data("SystemMode") or 0)
        except (TypeError, ValueError):
            system_mode = 0
        if system_mode in (3, 4):
            try:
                run_replan_procedure(self.manager)
            except Exception as exc:
                self.manager._log(
                    "MON_LOGIC",
                    "ERROR",
                    f"Failed to run replan after forced command: {exc}",
                )

    def _set_aircraft_availability(self, aircraft_id: int, available: bool) -> bool:
        if aircraft_id is None:
            return False
        previous = self._availability_mandatory_override.get(aircraft_id)
        if previous == available:
            # still recompute to ensure ordering consistent when no baseline yet
            self._recompute_availability()
            return False
        self._availability_mandatory_override[aircraft_id] = available
        self._recompute_availability()
        return True

    def _find_next_input_mission_id(self, initial: bool = False) -> Optional[int]:
        if self._input_mission_order:
            source_iterable: Iterable[Any] = list(self._input_mission_order)
        elif self._plan_context:
            source_iterable = list(self._plan_context.get("inputMissionIDs") or [])
        else:
            source_iterable = list(self._input_mission_tracker.keys())
        normalized_ids: List[int] = []
        seen: Set[int] = set()
        for value in source_iterable:
            try:
                candidate = int(value)
            except (TypeError, ValueError):
                continue
            if candidate in seen:
                continue
            seen.add(candidate)
            normalized_ids.append(candidate)
        if not normalized_ids:
            for value in self._input_mission_tracker.keys():
                try:
                    candidate = int(value)
                except (TypeError, ValueError):
                    continue
                if candidate in seen:
                    continue
                seen.add(candidate)
                normalized_ids.append(candidate)
        if not normalized_ids:
            return None
        current = None if initial else self._current_input_mission_id
        start_index = 0
        if current is not None and current in normalized_ids and not initial:
            start_index = normalized_ids.index(current) + 1
        for idx in range(start_index, len(normalized_ids)):
            candidate = normalized_ids[idx]
            if candidate not in self._completed_input_ids:
                return candidate
        return None

    def _advance_to_next_input_mission(self, force: bool = False) -> None:
        if (
            not force
            and self._current_input_mission_id is not None
            and self._current_input_mission_id not in self._completed_input_ids
        ):
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                "[COLLAB] execute=1 received but current input mission is not completed yet; ignoring advance request.",
            )
            return
        if (
            force
            and self._current_input_mission_id is not None
            and self._current_input_mission_id not in self._completed_input_ids
        ):
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                "[COLLAB] execute=1 forcing advance to next input mission despite incomplete status.",
            )
        next_id = self._find_next_input_mission_id()
        if next_id == self._current_input_mission_id:
            return
        self._current_input_mission_id = next_id
        self._update_plan_context_active_input()
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            f"[COLLAB] active input mission set to {next_id}.",
        )

    def execute(self, mode_override=None):
        """?쒖뒪??紐⑤뱶瑜??뺤씤?섍퀬, 'monitoring'??寃쎌슦?먮쭔 濡쒖쭅???ㅽ뻾?⑸땲??"""
        system_mode = (
            mode_override
            if mode_override is not None
            else self.manager.logic_store.get_data("SystemMode")
        )

        if system_mode == 3:
            self.manager._log("MON_LOGIC", "EXEC", "Monitoring loop executing")
            self._process_mission_plan_update()
            plan_context = self._plan_context
            current_plan_id = self._current_mission_plan_id
            # 401 ?곗씠??媛?몄삤湲?
            data_401 = self.manager.receive_store.get_data("0401")
            if data_401:
                self.manager._log(
                    "MON_LOGIC", "INFO", "0401 agent status received. Running monitoring cycle."
                )
                try:
                    agent_states = getattr(data_401, "agentStateList", None)
                except Exception:
                    agent_states = None
                self._update_health_based_availability(agent_states)
                # 紐⑤땲?곕쭅 ?덉감 ?ㅽ뻾?섏뿬 0501 硫붿떆吏 蹂몃Ц ?앹꽦
                if self._current_input_mission_id is None:
                    self._current_input_mission_id = self._find_next_input_mission_id(
                        initial=True
                    )
                self._update_plan_context_active_input()
                body_0501, mission_status = run_monitoring_procedure(
                    data_401, plan_context, current_plan_id
                )

                self._update_input_mission_progress(mission_status)
                if plan_context:
                    snapshot_ts = body_0501.get("timestamp") if isinstance(body_0501, dict) else None
                    try:
                        self._mission_progress_exporter.write_snapshot(
                            plan_context,
                            mission_status,
                            timestamp_ms=snapshot_ts,
                            mission_plan_id=current_plan_id,
                        )
                    except Exception as exc:
                        self.manager._log(
                            "MISSION_PROGRESS",
                            "WARN",
                            f"Failed to export mission snapshot: {exc}",
                        )
                    try:
                        self._prior_mission_replan.process(plan_context)
                    except Exception as exc:
                        self.manager._log(
                            "PRIOR_MISSION",
                            "WARN",
                            f"0202 replan handler failed: {exc}",
                        )

                # ?곕즺 寃쎄퀬 濡쒖쭅
                feul_data = []
                prev_warnings_raw = (
                    self.manager.logic_store.get_data("fuel_warning_prev") or {}
                )
                prev_warnings: Dict[int, str] = {}
                for key, value in dict(prev_warnings_raw).items():
                    try:
                        norm_key = int(key)
                    except (TypeError, ValueError):
                        try:
                            norm_key = int(str(key))
                        except (TypeError, ValueError):
                            norm_key = key
                    prev_warnings[norm_key] = value

                for agent_state in data_401.agentStateList:
                    if agent_state.isUnmanned == 1:
                        try:
                            aircraft_id = int(getattr(agent_state, "aircraftID", 0))
                        except (TypeError, ValueError):
                            aircraft_id = getattr(agent_state, "aircraftID", 0)

                        fuel_value = getattr(agent_state, "fuel", None)
                        fuel_liters: Optional[float] = None
                        if fuel_value is not None:
                            try:
                                candidate = float(fuel_value)
                            except (TypeError, ValueError):
                                candidate = None
                            if candidate is not None and math.isfinite(candidate):
                                if candidate < 0:
                                    candidate = 0.0
                                fuel_liters = candidate

                        if fuel_liters is None:
                            feul_data.append(
                                {
                                    "id": aircraft_id,
                                    "warning": "unknown",
                                }
                            )
                            prev_warnings[aircraft_id] = "unknown"
                            continue

                        capacity = FUEL_CAPACITY_LITERS if FUEL_CAPACITY_LITERS > 0 else 15.0
                        red_threshold = capacity * 0.1
                        yellow_threshold = capacity * 0.2
                        text = "green"
                        fuel_level = 0
                        if fuel_liters <= red_threshold:
                            text = "red"
                            fuel_level = 2
                        elif fuel_liters <= yellow_threshold:
                            text = "yellow"
                            fuel_level = 1

                        if capacity > 0:
                            fuel_percent = max(
                                0.0, min(100.0, (fuel_liters / capacity) * 100.0)
                            )
                        else:
                            fuel_percent = 0.0

                        feul_data.append(
                            {
                                "id": aircraft_id,
                                "warning": text,
                            }
                        )

                        if fuel_level in (1, 2):
                            last_state = prev_warnings.get(aircraft_id)
                            if last_state != text:
                                warning_body = {
                                    "timestamp": int(
                                        (
                                            datetime.now(timezone.utc)
                                            - datetime(2000, 1, 1, tzinfo=timezone.utc)
                                        ).total_seconds()
                                        * 1000
                                    ),
                                    "source": "MSM",
                                    "aircraftID": aircraft_id,
                                    "fuelLevel": fuel_level,
                                }
                                push_message(
                                    "0504",
                                    self.manager.node_messenger,
                                    body_dict=warning_body,
                                )
                                self.manager._log(
                                    "MON_LOGIC",
                                    "INFO",
                                    f"0504 ?곕즺 寃쎄퀬 ?꾩넚 (UAV={agent_state.aircraftID}, level={text})",
                                )
                                try:
                                    self.manager.push_store.add_data("0504", warning_body)
                                except Exception:
                                    pass
                                try:
                                    udp_reporter.notify_tx("0504")
                                except Exception:
                                    pass
                                _inform_info_module("0504", warning_body)
                                if self.manager.gui_update_callback:
                                    try:
                                        self.manager.gui_update_callback(
                                            "logic", "0504", warning_body
                                        )
                                    except Exception:
                                        pass
                        prev_warnings[aircraft_id] = text

                self.manager.logic_store.set_data(
                    "fuel_warning_prev", prev_warnings
                )

                if body_0501:
                    push_message(
                        "0501", self.manager.node_messenger, body_dict=body_0501
                    )
                    self.manager._log(
                        "MON_LOGIC", "INFO", "0501 mission progress message sent."
                    )

                    self.manager.push_store.add_data("0501", body_0501)
                    self.manager.logic_store.set_data("0501_data", body_0501)
                    udp_reporter.notify_tx("0501")

                    if self.manager.gui_update_callback:
                        self.manager.gui_update_callback("logic", "0501", body_0501)


                if feul_data:  
                    self.manager.logic_store.set_data("fuel_data", feul_data)
                    if self.manager.gui_update_callback:
                        self.manager.gui_update_callback(
                            "logic", "fuel_data", feul_data
                        )
            self._maybe_stage_replan(reason="logic_loop")
        else:
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                "0401 agent status not available; skipping monitoring cycle.",
            )

    def _handle_collab_command(self, data: Any) -> None:
        execute = self._safe_get(data, "execute", "Execute")
        if execute is None:
            return
        try:
            execute_val = int(execute)
        except (TypeError, ValueError):
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"[COLLAB] invalid execute value received: {execute}",
            )
            return

        if execute_val == 2:
            self._activate_collab_pause(data)
            self._register_collab_replan_trigger(data)
            self._maybe_stage_replan(reason="rx_0803")
        else:
            self._deactivate_collab_pause(reason=f"execute={execute_val} command received")
            self._cancel_pending_replan(reason=f"execute={execute_val}")
            if execute_val == 1:
                committed_id = self._commit_next_input_completion()
                if committed_id is None:
                    self.manager._log(
                        "MON_LOGIC",
                        "INFO",
                        "[COLLAB] execute=1 received but no pending input mission completions were queued.",
                    )
                self._advance_to_next_input_mission(force=True)

    def _handle_new_input_plan(self, data: Any) -> None:
        package_id = self._to_int(self._safe_get(data, "inputMissionPackageID"))
        timestamp = self._to_int(self._safe_get(data, "timestamp", "Timestamp"))
        previous_key = self._latest_input_plan_key
        current_key = (package_id, timestamp)
        self._latest_input_plan_key = current_key
        available_ids = self._extract_available_ids_from_payload(data)
        if (not available_ids) and package_id is not None:
            available_ids = self._load_available_ids_from_package(package_id)
        if available_ids:
            self._set_baseline_availability(available_ids)

        if self._collab_replan_pending:
            self._maybe_stage_replan(reason="rx_0201", input_plan=data)
            return
        if package_id is None and timestamp is None:
            return
        system_mode = self.manager.logic_store.get_data("SystemMode")
        if system_mode not in (3, 4):
            return
        self._register_input_plan_refresh_trigger(
            package_id=package_id,
            timestamp=timestamp,
            trigger_source=self._safe_get(data, "source", "Source"),
        )
        self._maybe_stage_replan(reason="rx_0201_input_refresh", input_plan=data)

    def _register_input_plan_refresh_trigger(
        self,
        package_id: Optional[int],
        timestamp: Optional[int],
        trigger_source: Optional[str] = None,
    ) -> None:
        if self._collab_replan_inflight or self._collab_replan_pending:
            self._cancel_pending_replan(reason="override:new_input_0201")
        source = trigger_source or COLLAB_REPLAN_DEFAULT_SOURCE
        command_ts = timestamp if timestamp is not None else self._current_time_ms()
        reason_text = COLLAB_REPLAN_REASON_REINPUT
        self._collab_replan_trigger = {
            "command_timestamp": command_ts,
            "source": source,
            "replanLevel": 3,
            "reason": reason_text,
            "replanRequestText": reason_text,
            "triggerType": "input_plan_update",
        }
        self._collab_replan_required_input_key = None
        self._collab_replan_waiting_for_new_input_logged = False
        self._collab_replan_pending = True
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            f"[COLLAB] new InputMissionPlan detected (package={package_id}, timestamp={timestamp}); requesting collaborative replan.",
        )
        try:
            self.manager.logic_store.set_data(
                "collab_replan_trigger", dict(self._collab_replan_trigger)
            )
        except Exception:
            pass

    def _load_input_plan_from_storage(self, package_id: int) -> Optional[Dict[str, Any]]:
        candidates: List[Path] = []
        try:
            candidates.append(
                db_paths.get_db_subpath("InputMissionPlan", f"{package_id}.json")
            )
        except Exception as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"[COLLAB] failed to resolve active InputMissionPlan path for package={package_id}: {exc}",
            )
        try:
            info = db_paths.get_info()
        except Exception:
            info = {}
        scenario_dir_raw = info.get("scenario_dir")
        agency_code = info.get("agency") or os.environ.get("KU_AGENCY_CODE") or "SBC3"
        if scenario_dir_raw:
            scenario_path = Path(str(scenario_dir_raw))
            candidates.append(
                scenario_path / agency_code / "InputMissionPlan" / f"{package_id}.json"
            )
        candidates.append(
            db_paths.LEGACY_DB_ROOT / "InputMissionPlan" / f"{package_id}.json"
        )
        for path in candidates:
            try:
                if path is None:
                    continue
                with path.open("r", encoding="utf-8") as fh:
                    data = json.load(fh)
                return data
            except FileNotFoundError:
                continue
            except Exception as exc:
                self.manager._log(
                    "MON_LOGIC",
                    "WARN",
                    f"[COLLAB] InputMissionPlan load failed for package={package_id} ({path}): {exc}",
                )
        self.manager._log(
            "MON_LOGIC",
            "WARN",
            f"[COLLAB] InputMissionPlan not found for package={package_id}; using fallback IDs.",
        )
        return None

    def _register_collab_replan_trigger(self, data: Any) -> None:
        if self._collab_replan_inflight:
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                "[COLLAB] replan trigger ignored because a replan is already in progress.",
            )
            return
        trigger_ts = self._to_int(self._safe_get(data, "timestamp", "Timestamp"))
        source = self._safe_get(data, "source", "Source") or "IDM"
        replan_level = self._to_int(self._safe_get(data, "replanLevel", "ReplanLevel"))
        if replan_level is None:
            replan_level = 3
        reason_text = f"execute=2 command from {source}"
        request_text = COLLAB_REPLAN_REASON_REEXECUTE
        self._collab_replan_trigger = {
            "command_timestamp": trigger_ts,
            "source": source,
            "replanLevel": replan_level,
            "reason": reason_text,
            "replanRequestText": request_text,
            "triggerType": "execute_2",
        }
        self._collab_replan_required_input_key = (
            None if self._latest_input_plan_key is None else tuple(self._latest_input_plan_key)
        )
        self._collab_replan_waiting_for_new_input_logged = False
        self._collab_replan_pending = True
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            f"[COLLAB] replan trigger registered (source={source}, ts={trigger_ts}).",
        )
        try:
            self.manager.logic_store.set_data(
                "collab_replan_trigger", dict(self._collab_replan_trigger)
            )
        except Exception:
            pass

    def _cancel_pending_replan(self, reason: str = "") -> None:
        if not (self._collab_replan_pending or self._collab_replan_inflight):
            return
        self._collab_replan_pending = False
        if self._collab_replan_inflight:
            self._collab_replan_inflight = False
            self._exit_collab_reexecute_mode(reason=f"cancel({reason})")
        self._collab_replan_trigger = None
        self._collab_replan_required_input_key = None
        self._collab_replan_waiting_for_new_input_logged = False
        try:
            self.manager.logic_store.set_data(
                "collab_replan_state",
                {
                    "status": "cancelled",
                    "reason": reason,
                    "timestamp": self._current_time_ms(),
                },
            )
        except Exception:
            pass

    def _maybe_stage_replan(self, reason: str, input_plan: Optional[Any] = None) -> None:
        if not self._collab_replan_pending or self._collab_replan_inflight:
            return
        system_mode = self.manager.logic_store.get_data("SystemMode")
        if system_mode not in (3, 4):
            return
        plan_payload = input_plan or self.manager.receive_store.get_data("0201")
        if not plan_payload:
            return
        package_id = self._to_int(self._safe_get(plan_payload, "inputMissionPackageID"))
        timestamp = self._to_int(self._safe_get(plan_payload, "timestamp", "Timestamp"))
        current_key = (package_id, timestamp)
        reason_str = str(reason or "")
        is_input_refresh = reason_str.startswith("rx_0201")
        if (
            not is_input_refresh
            and self._collab_replan_required_input_key is not None
            and current_key == self._collab_replan_required_input_key
        ):
            if not self._collab_replan_waiting_for_new_input_logged:
                self.manager._log(
                    "MON_LOGIC",
                    "INFO",
                    "[COLLAB] waiting for new 0201 input plan before dispatching replan.",
                )
                self._collab_replan_waiting_for_new_input_logged = True
            return
        self._collab_replan_waiting_for_new_input_logged = False
        self._latest_input_plan_key = current_key
        if not is_input_refresh and current_key == self._collab_last_replan_key:
            return
        payload = self._build_replan_body(plan_payload)
        if payload is None:
            return
        replan_body, context = payload
        context["triggerReason"] = reason
        success = self._dispatch_collab_replan(replan_body, context)
        if success:
            self._collab_last_replan_key = current_key
            self._collab_replan_required_input_key = current_key
            self._collab_replan_pending = False
            self._collab_replan_inflight = True
            self._enter_collab_reexecute_mode(context.get("timestamp"))
        else:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                "[COLLAB] 0902 dispatch failed; will retry when prerequisites are met.",
            )

    def _build_replan_body(
        self, input_plan: Any
    ) -> Optional[Tuple[ReplanRequestBodyModel, Dict[str, Any]]]:
        trigger = self._collab_replan_trigger or {}
        timestamp = self._current_time_ms()
        package_id = self._to_int(self._safe_get(input_plan, "inputMissionPackageID"))
        input_ids = self._extract_input_mission_ids(input_plan)
        message_input_ids = list(input_ids)
        file_plan_payload: Optional[Dict[str, Any]] = None
        used_file_payload = False
        if package_id is not None:
            file_plan_payload = self._load_input_plan_from_storage(package_id)
            if file_plan_payload:
                file_input_ids = self._extract_input_mission_ids(file_plan_payload)
                if file_input_ids and (not input_ids or file_input_ids != message_input_ids):
                    input_ids = file_input_ids
                    used_file_payload = True
        if not input_ids:
            plan_ids = (self._plan_context or {}).get("inputMissionIDs") or []
            input_ids = list(plan_ids)
        if not input_ids and package_id is not None:
            input_ids = [package_id]
        input_models = [
            InputMissionIDModel(inputMissionID=i)
            for i in input_ids
            if i is not None
        ]
        if not input_models:
            input_models.append(InputMissionIDModel(inputMissionID=0))
        individual_ids: List[int] = []
        individual_models: List[IndividualMissionIDListModel] = []
        prior_ids: List[int] = self._collect_prior_mission_ids()
        prior_models: List[PriorMissionListModel] = [
            PriorMissionListModel(priorMissionID=pid)
            for pid in prior_ids
            if pid is not None
        ]
        mission_plan_id = self._resolve_mission_plan_id(input_plan, timestamp)
        option_models, new_plan_ids = self._build_collab_option_list()
        replan_level = trigger.get("replanLevel", 3)
        try:
            replan_level = int(replan_level)
        except (TypeError, ValueError):
            replan_level = 3
        source = trigger.get("source") or COLLAB_REPLAN_DEFAULT_SOURCE
        reason_text = trigger.get("replanRequestText") or "협업 재계획 요청"
        replan_body = ReplanRequestBodyModel(
            source=source,
            timestamp=timestamp,
            replanRequestTime=ReplanRequestTimeStampModel(
                replanRequestTimestamp=timestamp
            ),
            replanLevel=replan_level,
            inputMissionIDList=input_models,
            IndividualMissionIDList=individual_models,
            priorMissionList=prior_models,
            replanRequest=reason_text,
            optionList=option_models,
        )
        context = {
            "timestamp": timestamp,
            "source": source,
            "replanLevel": replan_level,
            "reason": reason_text,
            "replanRequestText": reason_text,
            "missionPlanID": mission_plan_id,
            "newMissionPlanIDs": new_plan_ids,
            "inputMissionPackageID": package_id,
            "inputMissionIDs": input_ids,
            "individualMissionIDs": individual_ids,
            "priorMissionIDs": prior_ids,
            "commandTimestamp": trigger.get("command_timestamp"),
            "triggerType": trigger.get("triggerType"),
            "options": [asdict(opt) for opt in option_models],
            "inputMissionPlanSource": "file" if used_file_payload else "message",
            "inputMissionPlanPackageID": package_id,
        }
        return replan_body, context

    def _collect_individual_mission_ids(self) -> List[int]:
        context = self._plan_context or {}
        aircraft_map = context.get("aircraft") or {}
        ids: Set[int] = set()
        for payload in aircraft_map.values():
            missions = payload.get("missions") or []
            for mission in missions:
                value = self._to_int(mission.get("individualMissionID"))
                if value is not None:
                    ids.add(value)
        return sorted(ids)

    def _collect_prior_mission_ids(self) -> List[int]:
        try:
            prior = self.manager.receive_store.get_data("0202")
        except Exception:
            prior = None
        mission_list = self._safe_get(prior, "priorMissionList", "PriorMissionList")
        ids: Set[int] = set()
        for item in mission_list or []:
            value = self._to_int(self._safe_get(item, "priorMissionID", "PriorMissionID"))
            if value is not None:
                ids.add(value)
        return sorted(ids)

    def _dispatch_collab_replan(
        self, replan_body: ReplanRequestBodyModel, context: Dict[str, Any]
    ) -> bool:
        try:
            ensure_replan_level_details_file()
        except Exception as exc:
            try:
                self.manager._log("MON_LOGIC", "WARN", f"[COLLAB] replanInfo 준비 실패: {exc}")
            except Exception:
                pass
        body_dict = asdict(replan_body)
        try:
            push_message("0902", self.manager.node_messenger, body_dict=body_dict)
        except Exception as exc:
            self.manager._log(
                "MON_LOGIC", "ERROR", f"[COLLAB] failed to dispatch 0902: {exc}"
            )
            return False
        try:
            self.manager.push_store.add_data("0902", replan_body)
        except Exception:
            pass
        try:
            udp_reporter.notify_tx("0902")
        except Exception:
            pass
        state_payload = dict(context)
        state_payload["status"] = "requested"
        try:
            self.manager.logic_store.set_data("collab_replan_state", state_payload)
        except Exception:
            pass
        try:
            _inform_info_module("0902", body_dict)
        except Exception:
            pass
        if self.manager.gui_update_callback:
            try:
                self.manager.gui_update_callback("logic", "0902", state_payload)
            except Exception:
                pass
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            "[COLLAB] 0902 replan request dispatched to mission planning.",
        )
        return True

    def _handle_replan_status(self, data: Any) -> None:
        status = self._to_int(
            self._safe_get(data, "missionPlanningStatus", "MissionPlanningStatus")
        )
        if status is None:
            return
        status_entry = {
            "status_code": status,
            "timestamp": self._to_int(self._safe_get(data, "timestamp", "Timestamp")),
            "reason": self._safe_get(data, "replanReason", "ReplanReason"),
        }
        status_entry["status"] = {0: "queued", 1: "in_progress", 2: "completed"}.get(
            status, "unknown"
        )
        try:
            current = self.manager.logic_store.get_data("collab_replan_state") or {}
            current.update(status_entry)
            self.manager.logic_store.set_data("collab_replan_state", current)
        except Exception:
            pass
        if self.manager.gui_update_callback:
            try:
                self.manager.gui_update_callback("logic", "0305", status_entry)
            except Exception:
                pass
        if status == 2:
            self._finalize_collab_replan(status_entry.get("reason"))

    def _finalize_collab_replan(self, reason: Optional[str]) -> None:
        self._collab_replan_inflight = False
        self._collab_replan_pending = False
        self._collab_replan_trigger = None
        self._exit_collab_reexecute_mode(reason="0305 status=2")
        if self._collab_pause_active:
            self._deactivate_collab_pause(reason="replan completed")
        try:
            current = self.manager.logic_store.get_data("collab_replan_state") or {}
            current.update(
                {
                    "status": "completed",
                    "status_code": 2,
                    "completedReason": reason,
                    "completedAt": self._current_time_ms(),
                }
            )
            self.manager.logic_store.set_data("collab_replan_state", current)
        except Exception:
            pass
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            "[COLLAB] collaborative replan marked as completed.",
        )

    def _activate_collab_pause(self, data: Any) -> None:
        if self._collab_pause_active:
            return
        system_mode = self.manager.logic_store.get_data("SystemMode")
        if system_mode not in (3, 4):
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                "[COLLAB] execute=2 received outside mission execution; ignored.",
            )
            return

        self._collab_pause_prev_suspended = self._monitoring_suspended
        self._monitoring_suspended = True
        self._collab_pause_active = True

        timestamp = self._safe_get(data, "timestamp", "Timestamp")
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            f"[COLLAB] ?묒뾽湲곗??꾨Т ?ъ닔???湲??곹깭濡??꾪솚 (timestamp={timestamp})",
        )
        try:
            self.manager.logic_store.set_data(
                "collab_pause_info",
                {
                    "timestamp": timestamp,
                    "source": self._safe_get(data, "source", "Source") or "CSP",
                },
            )
            self.manager.logic_store.set_data("collab_pause_active", True)
        except Exception:
            pass

        if self.manager.gui_update_callback:
            try:
                self.manager.gui_update_callback("logic", "collab_pause", None)
            except Exception:
                pass

    def _deactivate_collab_pause(self, reason: str = "") -> None:
        if not self._collab_pause_active:
            return
        self._collab_pause_active = False
        self._monitoring_suspended = self._collab_pause_prev_suspended
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            f"[COLLAB] ?묒뾽湲곗??꾨Т ?湲??곹깭 ?댁젣 ({reason})",
        )
        try:
            self.manager.logic_store.set_data("collab_pause_active", False)
        except Exception:
            pass

        if self.manager.gui_update_callback:
            try:
                self.manager.gui_update_callback("logic", "collab_pause", None)
            except Exception:
                pass
    def _update_input_mission_progress(self, mission_status: List[Dict[str, Any]]) -> None:
        if not mission_status or not self._input_mission_tracker:
            return
        changed = False
        active_ids: Set[int] = set()
        for entry in mission_status:
            try:
                aircraft_id = int(entry.get("aircraftID", 0))
                mission_id = int(entry.get("individualMissionID", 0))
            except (TypeError, ValueError):
                continue
            active_ids.add(aircraft_id)
            path_id = entry.get("pathID")
            if path_id is not None:
                try:
                    path_id = int(path_id)
                except (TypeError, ValueError):
                    path_id = None
            mission_index = entry.get("missionIndex")
            try:
                mission_index = int(mission_index) if mission_index is not None else 0
            except (TypeError, ValueError):
                mission_index = 0
            raw_key = (aircraft_id, mission_id, path_id, mission_index)
            input_id = entry.get("inputMissionID")
            if input_id is None:
                input_id = self._mission_to_input.get(raw_key)
            if input_id is None:
                continue
            try:
                input_id = int(input_id)
            except (TypeError, ValueError):
                continue
            tracker = self._input_mission_tracker.get(input_id)
            if not tracker:
                continue
            total_keys = tracker.get("total") or set()
            key = raw_key
            if key not in total_keys:
                resolved = self._resolve_mission_tracker_key(total_keys, raw_key)
                if resolved is None:
                    continue
                key = resolved
                self._mission_to_input[raw_key] = input_id
                if raw_key not in self._mission_file_map and key in self._mission_file_map:
                    self._mission_file_map[raw_key] = self._mission_file_map[key]
            inactive_set = tracker.get("inactive")
            if inactive_set and key in inactive_set:
                inactive_set.discard(key)
            try:
                progress = int(entry.get("progress", 0))
            except (TypeError, ValueError):
                progress = 0
            prev = self._mission_progress_max.get(key, 0)
            if progress > prev:
                self._mission_progress_max[key] = progress
            if progress >= 100 and prev < 100 and key not in tracker["completed"]:
                tracker["completed"].add(key)
                changed = True
                self._mark_individual_mission_done(key)
        if not changed:
            return
        if active_ids:
            if not self._active_aircraft_ids:
                self._active_aircraft_ids = set(active_ids)
            else:
                self._active_aircraft_ids.update(active_ids)
            for data in self._input_mission_tracker.values():
                total = data.get("total") or set()
                completed = data.get("completed") or set()
                inactive = data.get("inactive")
                if inactive is None:
                    inactive = data["inactive"] = set()
                pruned = {
                    key for key in list(total) if key[0] not in self._active_aircraft_ids
                }
                if pruned:
                    inactive.update(pruned)
                    completed.difference_update(pruned)
                    for key in pruned:
                        self._mission_progress_max.pop(key, None)
                        self._mission_file_map.pop(key, None)
        for input_id, data in self._input_mission_tracker.items():
            total = data.get("total") or set()
            inactive = data.get("inactive") or set()
            completed = data.get("completed") or set()
            effective_completed = { (cid, mid, pid, order) for (cid, mid, pid, order) in completed | inactive }
            if not total:
                self._completed_input_ids.add(input_id)
                self._notify_input_mission_completed(input_id)
                continue
            if total <= effective_completed:
                self._completed_input_ids.add(input_id)
                for key in total - inactive:
                    self._mark_individual_mission_done(key)
                self._notify_input_mission_completed(input_id)
        if self._completed_input_ids and all(
            (info.get("total") or set())
            <= ((info.get("completed") or set()) | (info.get("inactive") or set()))
            for info in self._input_mission_tracker.values()
        ):
            self._handle_all_input_missions_completed()
        try:
            self.manager.logic_store.set_data(
                "completed_input_ids", sorted(self._completed_input_ids)
            )
        except Exception:
            pass
        if self.manager.gui_update_callback:
            try:
                self.manager.gui_update_callback("logic", "mission_overview", None)
            except Exception:
                pass

    def _notify_input_mission_completed(self, input_id: int) -> None:
        self._queue_input_completion_write(input_id)
        if input_id in self._input_completion_notified:
            return
        self._input_completion_notified.add(input_id)
        self._send_0503_notification(f"Input mission completed (ID={input_id})")

    def _send_0503_notification(self, log_context: str) -> None:
        timestamp = int(
            (
                datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)
            ).total_seconds()
            * 1000
        )
        body_0503 = {
            "timestamp": timestamp,
            "source": "MSM",
            "systemRecommend": 1,
        }
        try:
            push_message("0503", self.manager.node_messenger, body_dict=body_0503)
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                f"0503 collaborative completion notice sent ({log_context})",
            )
        except Exception as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"0503 completion notice failed ({log_context}): {exc}",
            )
            return
        try:
            self.manager.push_store.add_data("0503", body_0503)
        except Exception:
            pass
        try:
            self.manager.logic_store.set_data("0503_data", body_0503)
        except Exception:
            pass
        try:
            udp_reporter.notify_tx("0503")
        except Exception:
            pass
        _inform_info_module("0503", body_0503)
        if self.manager.gui_update_callback:
            try:
                self.manager.gui_update_callback("logic", "0503", body_0503)
            except Exception:
                pass

    def _queue_input_completion_write(self, input_id: Optional[int]) -> None:
        queued_id = self._to_int(input_id)
        if queued_id is None:
            return
        if queued_id in self._pending_input_completion_queue:
            return
        self._pending_input_completion_queue.append(queued_id)
        try:
            self.manager.logic_store.set_data(
                "pending_input_completion_ids",
                list(self._pending_input_completion_queue),
            )
        except Exception:
            pass
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            f"[COLLAB] queued input mission {queued_id} for deferred isDone update.",
        )

    def _commit_next_input_completion(self) -> Optional[int]:
        if not self._pending_input_completion_queue:
            return None
        committed_id = self._pending_input_completion_queue.pop(0)
        self._mark_input_mission_done(committed_id)
        try:
            self.manager.logic_store.set_data(
                "pending_input_completion_ids",
                list(self._pending_input_completion_queue),
            )
        except Exception:
            pass
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            f"[COLLAB] applied deferred completion write for input mission {committed_id}.",
        )
        return committed_id

    def _enter_collab_reexecute_mode(self, timestamp: Optional[int]) -> None:
        if self._collab_reexecute_mode:
            return
        self._collab_reexecute_mode = True
        ts = (
            int(timestamp)
            if isinstance(timestamp, (int, float))
            else self._current_time_ms()
        )
        self._collab_reexecute_trigger_ts = ts
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            f"[COLLAB] ?묒뾽湲곗??꾨Т ?ъ닔??紐⑤뱶 吏꾩엯 (timestamp={ts})",
        )
        try:
            self.manager.logic_store.set_data("collab_reexecute_mode", True)
        except Exception:
            pass

    def _exit_collab_reexecute_mode(self, reason: str = "") -> None:
        if not self._collab_reexecute_mode:
            return
        self._collab_reexecute_mode = False
        self._collab_reexecute_trigger_ts = None
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            f"[COLLAB] ?묒뾽湲곗??꾨Т ?ъ닔??紐⑤뱶 ?댁젣 ({reason})",
        )
        try:
            self.manager.logic_store.set_data("collab_reexecute_mode", False)
        except Exception:
            pass

    def _extract_input_mission_ids(self, data: Any) -> List[int]:
        mission_list = getattr(data, "inputMissionList", None)
        if mission_list is None and isinstance(data, dict):
            mission_list = data.get("inputMissionList")
        ids: List[int] = []
        seen: Set[int] = set()
        for item in mission_list or []:
            is_done_val = self._safe_get(item, "isDone", "IsDone")
            try:
                if isinstance(is_done_val, str):
                    is_done_flag = is_done_val.strip().lower() in ("1", "true", "yes")
                else:
                    is_done_flag = bool(is_done_val)
            except Exception:
                is_done_flag = False
            if is_done_flag:
                continue
            value = self._safe_get(item, "inputMissionID", "InputMissionID")
            if value is None:
                continue
            try:
                candidate = int(value)
            except (TypeError, ValueError):
                continue
            if candidate in seen:
                continue
            seen.add(candidate)
            ids.append(candidate)
        return ids

    def _resolve_mission_plan_id(self, data: Any, timestamp: int) -> int:
        if self._current_mission_plan_id is not None:
            try:
                return int(self._current_mission_plan_id)
            except (TypeError, ValueError):
                pass
        candidate = self._safe_get(data, "inputMissionPackageID")
        if candidate is not None:
            try:
                return int(candidate)
            except (TypeError, ValueError):
                pass
        return 700_000_000 + (timestamp % 1_000)

    def _build_collab_option_list(self) -> Tuple[List[OptionListModel], List[int]]:
        option_names = ["시스템추천", "촬영효율우선", "비행효율우선"]
        option_ids = self._allocate_option_ids(len(option_names))
        mission_plan_ids = self._allocate_mission_plan_ids(len(option_names))
        options: List[OptionListModel] = []
        for name, oid, mid in zip(option_names, option_ids, mission_plan_ids):
            options.append(
                OptionListModel(
                    optionID=oid,
                    optionName=name,
                    missionPlanID=mid,
                )
            )
        return options, mission_plan_ids

    def _allocate_option_ids(self, count: int) -> List[int]:
        allocated: List[int] = []
        candidate = max(self._used_option_ids or {0}) + 1
        while len(allocated) < count:
            if candidate not in self._used_option_ids:
                self._used_option_ids.add(candidate)
                allocated.append(candidate)
            candidate += 1
        return allocated

    def _allocate_mission_plan_ids(self, count: int) -> List[int]:
        existing = self._scan_existing_mission_plan_ids()
        combined = set(existing) | set(self._allocated_plan_ids)
        base = max(combined) if combined else 700000000
        next_candidate = base
        allocated: List[int] = []
        while len(allocated) < count:
            next_candidate += 1
            if next_candidate not in combined:
                allocated.append(next_candidate)
                self._allocated_plan_ids.add(next_candidate)
                combined.add(next_candidate)
        self._existing_mission_plan_ids.update(allocated)
        return allocated

    def _scan_existing_mission_plan_ids(self) -> Set[int]:
        if self._existing_mission_plan_ids:
            return set(self._existing_mission_plan_ids)
        ids: Set[int] = set()
        try:
            mission_plan_dir = db_paths.get_db_subpath("MissionPlan")
        except Exception:
            return ids
        try:
            for entry in mission_plan_dir.glob("*.json"):
                stem = entry.stem
                if stem.isdigit():
                    ids.add(int(stem))
        except Exception:
            pass
        self._existing_mission_plan_ids = ids
        return set(ids)

    def _current_time_ms(self) -> int:
        return int(
            (
                datetime.now(timezone.utc)
                - datetime(2000, 1, 1, tzinfo=timezone.utc)
            ).total_seconds()
            * 1000
        )

    def _safe_get(self, obj: Any, *names: str) -> Any:
        for name in names:
            if hasattr(obj, name):
                return getattr(obj, name)
        if isinstance(obj, dict):
            for name in names:
                if name in obj:
                    return obj[name]
        return None

    def _to_int(self, value: Any) -> Optional[int]:
        if value is None:
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _handle_all_input_missions_completed(self) -> None:
        if self._collab_completion_sent:
            return
        self._collab_completion_sent = True
        self._monitoring_suspended = True
        self._current_input_mission_id = None
        self._update_plan_context_active_input()
        self._send_0503_notification("All input missions completed")

    def _mark_input_mission_done(self, input_id: Optional[int]) -> None:
        if input_id is None:
            return
        try:
            input_id_int = int(input_id)
        except (TypeError, ValueError):
            return
        path = self._input_mission_plan_path
        if not path:
            return
        index_map = self._refresh_input_mission_index_map()
        index = index_map.get(input_id_int)
        if index is None:
            index_map = self._refresh_input_mission_index_map(force=True)
            index = index_map.get(input_id_int)
            if index is None:
                return
        try:
            with path.open("r", encoding="utf-8") as fh:
                data = json.load(fh)
        except Exception as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"InputMissionPlan reload failed (package={self._input_mission_package_id}, input={input_id_int}): {exc}",
            )
            return
        mission_list = data.get("inputMissionList")
        if not isinstance(mission_list, list):
            return
        if index < 0 or index >= len(mission_list):
            return
        mission_entry = mission_list[index]
        if mission_entry.get("isDone") is True:
            return
        mission_entry["isDone"] = True
        try:
            path.write_text(
                json.dumps(data, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except Exception as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"InputMissionPlan write failed (package={self._input_mission_package_id}, input={input_id_int}): {exc}",
            )

    def _mark_individual_mission_done(
        self, key: Tuple[int, int, Optional[int], int]
    ) -> None:
        record = self._mission_file_map.get(key)
        if not record or not self._plan_context:
            return
        package_id, mission_index, aircraft_id = record
        aircraft_payload = (
            (self._plan_context.get("aircraft") or {}).get(aircraft_id) or {}
        )
        missions = aircraft_payload.get("missions") or []
        if 0 <= mission_index < len(missions):
            if missions[mission_index].get("isDone") is True:
                return
            missions[mission_index]["isDone"] = True
        imp_path = db_paths.get_db_subpath(
            "IndividualMissionPlan", f"{package_id}.json"
        )
        try:
            with imp_path.open("r", encoding="utf-8") as fh:
                imp_data = json.load(fh)
        except Exception:
            return
        mission_list = imp_data.get("individualMissionList")
        if (
            isinstance(mission_list, list)
            and 0 <= mission_index < len(mission_list)
            and mission_list[mission_index].get("isDone") is not True
        ):
            mission_list[mission_index]["isDone"] = True
            try:
                imp_path.write_text(
                    json.dumps(imp_data, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
            except Exception:
                pass

    def generate_body_for(self, msg_id: str) -> PushBodyType:
        timestamp = int(
            (
                datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)
            ).total_seconds()
            * 1000
        )
        source_module = "MSM"

        if msg_id == "0102":
            return ModuleStatusModelModel(
                timestamp=timestamp, source=source_module, status=1
            )
        elif msg_id == "0501":
            return MissionProgressBodyModel(
                timestamp=timestamp, source=source_module
            )
        elif msg_id == "0502":
            return MissionEndRequestBodyModel(
                timestamp=timestamp, source=source_module, reason=0
            )
        elif msg_id == "0902":
            return ReplanRequestBodyModel(
                timestamp=timestamp,
                source=source_module,
                replanRequest="ManualTrigger",
            )

        raise ValueError(f"Body generation not implemented for msg_id: {msg_id}")

