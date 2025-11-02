# logic/monitoring_logic_part.py: '모니터링' 도메인에 대한 세부 비즈니스 로직을 구현합니다.

from datetime import datetime, timezone
from dataclasses import asdict

from typing import Any, Dict, List, Optional, Set, Tuple, Union

# --- 데이터 모델 import ---
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
import udp_reporter
import socket
import json
import os
import math
from modules.common import db_paths


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


# --- 반환 가능한 모든 Push 메시지 본문 타입을 정의 ---

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
        self._input_mission_tracker: Dict[
            int, Dict[str, Set[Tuple[int, int, Optional[int]]]]
        ] = {}
        self._mission_to_input: Dict[Tuple[int, int, Optional[int]], int] = {}
        self._completed_input_ids: Set[int] = set()
        self._collab_completion_sent: bool = False
        self._monitoring_suspended: bool = False
        self._active_aircraft_ids: Set[int] = set()
        self._mission_progress_max: Dict[Tuple[int, int, Optional[int]], int] = {}
        self._mission_file_map: Dict[
            Tuple[int, int, Optional[int]], Tuple[int, int, int]
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
        self._used_option_ids: Set[int] = set()
        self._allocated_plan_ids: Set[int] = set()
        self._existing_mission_plan_ids: Set[int] = set()
        self._pending_mission_plan_id: Optional[int] = None
        self._pending_decision_command: Optional[Tuple[Optional[int], Optional[int]]] = None
        self._current_input_mission_id: Optional[int] = None
        try:
            self.manager.logic_store.set_data("collab_pause_active", False)
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # Message Hooks
    # ------------------------------------------------------------------ #

    def handle_message(self, msg_id: str, data: Any) -> None:
        if msg_id == "0803":
            self._handle_collab_command(data)
        elif msg_id == "0201":
            self._handle_new_input_plan(data)
        elif msg_id == "0305":
            self._handle_replan_status(data)
        elif msg_id == "0702":
            self._handle_decision_result(data)

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

    def on_system_mode_changed(self, mode: int) -> None:
        if mode not in (3, 4) and self._collab_pause_active:
            self._deactivate_collab_pause(reason=f"system mode changed to {mode}")

    def _process_mission_plan_update(self) -> None:
        candidate_plan_id = None
        try:
            data_0903 = self.manager.receive_store.get_data("0903")
        except Exception:
            data_0903 = None
        if data_0903:
            candidate_plan_id = getattr(data_0903, "missionPlanID", None)

        if candidate_plan_id is None:
            try:
                data_0902 = self.manager.receive_store.get_data("0902")
            except Exception:
                data_0902 = None
            if data_0902:
                candidate_plan_id = getattr(data_0902, "missionPlanID", None)
                if candidate_plan_id is None and isinstance(data_0902, dict):
                    candidate_plan_id = data_0902.get("missionPlanID")

        if candidate_plan_id is not None:
            try:
                candidate_plan_id = int(candidate_plan_id)
            except (TypeError, ValueError):
                candidate_plan_id = None

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
        elif decision_ignore == 1:
            mission_plan_id = self._current_mission_plan_id
            command_consumed = True
            self._pending_mission_plan_id = None
        else:
            if self._current_mission_plan_id is not None:
                mission_plan_id = self._current_mission_plan_id
            else:
                mission_plan_id = self._pending_mission_plan_id

        if mission_plan_id is None:
            if command_consumed:
                self._pending_decision_command = None
            return

        try:
            mission_plan_id = int(mission_plan_id)
        except (TypeError, ValueError):
            if command_consumed:
                self._pending_decision_command = None
            return

        if mission_plan_id == self._current_mission_plan_id:
            if command_consumed:
                self._pending_decision_command = None
            return

        mission_plan_path = db_paths.get_db_subpath("MissionPlan", f"{mission_plan_id}.json")
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
                    mission_plan_id = fallback_id
                    mission_plan_path = alt_path
                else:
                    if command_consumed:
                        self._pending_decision_command = None
                    return
            else:
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
            if command_consumed:
                self._pending_decision_command = None
            return
        except Exception as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"MissionPlan {mission_plan_id} load failed: {exc}",
            )
            if command_consumed:
                self._pending_decision_command = None
            return
        previous_plan_id = self._current_mission_plan_id
        tracker_initialized = bool(self._input_mission_tracker)
        self._plan_context = context
        self._current_mission_plan_id = mission_plan_id
        reinitialized = (
            previous_plan_id != mission_plan_id or not tracker_initialized
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
        self._pending_decision_command = None
        if self._pending_mission_plan_id == mission_plan_id:
            self._pending_mission_plan_id = None

    def _scan_latest_mission_plan_id(self) -> Optional[int]:
        """MissionPlan 디렉터리에서 가장 최신의 plan ID를 추론한다."""
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
        input_ids = set()

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
                        input_ids.add(int(input_mission_id))
                    except (TypeError, ValueError):
                        pass

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
            "inputMissionIDs": sorted(input_ids),
        }

    def _initialize_input_tracker(self, context: Dict[str, Any]) -> None:
        tracker: Dict[int, Dict[str, Set[Tuple[int, int, Optional[int]]]]] = {}
        reverse_map: Dict[Tuple[int, int, Optional[int]], int] = {}
        file_map: Dict[Tuple[int, int, Optional[int]], Tuple[int, int, int]] = {}
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
                key = (aircraft_id_int, mission_id, path_id)
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

    def _update_plan_context_active_input(self) -> None:
        if self._plan_context is not None:
            self._plan_context["activeInputMissionID"] = self._current_input_mission_id
        try:
            self.manager.logic_store.set_data(
                "active_input_mission_id", self._current_input_mission_id
            )
        except Exception:
            pass

    def _find_next_input_mission_id(self, initial: bool = False) -> Optional[int]:
        raw_ids: List[Any] = []
        if self._plan_context:
            raw_ids = list(self._plan_context.get("inputMissionIDs") or [])
        if not raw_ids:
            raw_ids = list(self._input_mission_tracker.keys())
        normalized_ids: List[int] = []
        for value in raw_ids:
            try:
                normalized_ids.append(int(value))
            except (TypeError, ValueError):
                continue
        normalized_ids = sorted(set(normalized_ids))
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

    def _advance_to_next_input_mission(self) -> None:
        if (
            self._current_input_mission_id is not None
            and self._current_input_mission_id not in self._completed_input_ids
        ):
            self.manager._log(
                "MON_LOGIC",
                "INFO",
                "[COLLAB] execute=1 received but current input mission is not completed yet; ignoring advance request.",
            )
            return
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
        """시스템 모드를 확인하고, 'monitoring'일 경우에만 로직을 실행합니다."""
        system_mode = (
            mode_override
            if mode_override is not None
            else self.manager.logic_store.get_data("SystemMode")
        )

        if system_mode == 3:
            self.manager._log("MON_LOGIC", "EXEC", "모니터링 로직 실행됨.")
            self._process_mission_plan_update()
            plan_context = self._plan_context
            current_plan_id = self._current_mission_plan_id
            # 401 데이터 가져오기
            data_401 = self.manager.receive_store.get_data("0401")
            if data_401:
                self.manager._log(
                    "MON_LOGIC", "INFO", "401 데이터 확인. 모니터링 절차 실행."
                )
                # 모니터링 절차 실행하여 0501 메시지 본문 생성
                if self._current_input_mission_id is None:
                    self._current_input_mission_id = self._find_next_input_mission_id(
                        initial=True
                    )
                self._update_plan_context_active_input()
                body_0501, mission_status = run_monitoring_procedure(
                    data_401, plan_context, current_plan_id
                )

                self._update_input_mission_progress(mission_status)

                # 연료 경고 로직
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
                                    "fuelLiters": None,
                                    "fuelPercent": None,
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
                                "fuelLiters": round(fuel_liters, 2),
                                "fuelPercent": round(fuel_percent, 1),
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
                                    "fuelLiters": round(fuel_liters, 2),
                                    "fuelPercent": round(fuel_percent, 1),
                                }
                                push_message(
                                    "0504",
                                    self.manager.node_messenger,
                                    body_dict=warning_body,
                                )
                                self.manager._log(
                                    "MON_LOGIC",
                                    "INFO",
                                    f"0504 연료 경고 전송 (UAV={agent_state.aircraftID}, level={text})",
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
                    # 0501 메시지 발신
                    # print(f"body_0501: {body_0501}")
                    push_message(
                        "0501", self.manager.node_messenger, body_dict=body_0501
                    )
                    self.manager._log(
                        "MON_LOGIC", "INFO", "0501 메시지를 발신했습니다."
                    )
                    # PushStorage에 저장
                    self.manager.push_store.add_data("0501", body_0501)
                    # LogicStorage에도 저장
                    self.manager.logic_store.set_data("0501_data", body_0501)
                    # UDP 통지 추가
                    udp_reporter.notify_tx("0501")

                    # GUI 업데이트 콜백 호출 (0501은 로직에서 생성된 데이터이므로 'logic' 타입으로 전달)
                    if self.manager.gui_update_callback:
                        self.manager.gui_update_callback("logic", "0501", body_0501)

                # fuel_data�� LogicStorage�� �����ϰ� GUI ������Ʈ
                if feul_data:  # feul_data�� ������� ���� ��쿡�� ó��
                    self.manager.logic_store.set_data("fuel_data", feul_data)
                    if self.manager.gui_update_callback:
                        self.manager.gui_update_callback(
                            "logic", "fuel_data", feul_data
                        )
            self._maybe_stage_replan(reason="logic_loop")
        else:
            self.manager._log(
                "MON_LOGIC", "INFO", "401 데이터가 없어 모니터링을 건너뜁니다."
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
                self._advance_to_next_input_mission()

    def _handle_new_input_plan(self, data: Any) -> None:
        package_id = self._to_int(self._safe_get(data, "inputMissionPackageID"))
        timestamp = self._to_int(self._safe_get(data, "timestamp", "Timestamp"))
        self._latest_input_plan_key = (package_id, timestamp)
        if self._collab_replan_pending:
            self._maybe_stage_replan(reason="rx_0201")

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
        self._collab_replan_trigger = {
            "command_timestamp": trigger_ts,
            "source": source,
            "replanLevel": replan_level,
            "reason": reason_text,
        }
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

    def _maybe_stage_replan(self, reason: str) -> None:
        if not self._collab_replan_pending or self._collab_replan_inflight:
            return
        system_mode = self.manager.logic_store.get_data("SystemMode")
        if system_mode not in (3, 4):
            return
        input_plan = self.manager.receive_store.get_data("0201")
        if not input_plan:
            return
        package_id = self._to_int(self._safe_get(input_plan, "inputMissionPackageID"))
        timestamp = self._to_int(self._safe_get(input_plan, "timestamp", "Timestamp"))
        current_key = (package_id, timestamp)
        self._latest_input_plan_key = current_key
        if current_key == self._collab_last_replan_key:
            return
        payload = self._build_replan_body(input_plan)
        if payload is None:
            return
        replan_body, context = payload
        context["triggerReason"] = reason
        success = self._dispatch_collab_replan(replan_body, context)
        if success:
            self._collab_last_replan_key = current_key
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
        prior_ids = self._collect_prior_mission_ids()
        prior_models = [PriorMissionListModel(priorMissionID=i) for i in prior_ids]
        mission_plan_id = self._resolve_mission_plan_id(input_plan, timestamp)
        option_models, new_plan_ids = self._build_collab_option_list()
        replan_level = trigger.get("replanLevel", 3)
        try:
            replan_level = int(replan_level)
        except (TypeError, ValueError):
            replan_level = 3
        source = trigger.get("source") or "MonitoringModule"
        reason_text = "협업기저임무 재수행"
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
            "missionPlanID": mission_plan_id,
             "newMissionPlanIDs": new_plan_ids,
            "inputMissionPackageID": package_id,
            "inputMissionIDs": input_ids,
            "individualMissionIDs": individual_ids,
            "priorMissionIDs": prior_ids,
            "commandTimestamp": trigger.get("command_timestamp"),
            "options": [asdict(opt) for opt in option_models],
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
        mission_list = getattr(prior, "priorMissionList", None)
        if mission_list is None and isinstance(prior, dict):
            mission_list = prior.get("priorMissionList")
        ids: Set[int] = set()
        for item in mission_list or []:
            value = self._to_int(self._safe_get(item, "priorMissionID", "PriorMissionID"))
            if value is not None:
                ids.add(value)
        return sorted(ids)

    def _dispatch_collab_replan(
        self, replan_body: ReplanRequestBodyModel, context: Dict[str, Any]
    ) -> bool:
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
        elif status == 1 and not self._collab_replan_inflight:
            self._collab_replan_inflight = True
            self._enter_collab_reexecute_mode(status_entry.get("timestamp"))

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
            f"[COLLAB] 협업기저임무 재수행 대기 상태로 전환 (timestamp={timestamp})",
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
            f"[COLLAB] 협업기저임무 대기 상태 해제 ({reason})",
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
            key = (aircraft_id, mission_id, path_id)
            input_id = entry.get("inputMissionID")
            if input_id is None:
                input_id = self._mission_to_input.get(key)
            if input_id is None:
                continue
            try:
                input_id = int(input_id)
            except (TypeError, ValueError):
                continue
            tracker = self._input_mission_tracker.get(input_id)
            if not tracker or key not in tracker.get("total", set()):
                continue
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
                self._notify_input_mission_completed(input_id)
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
            effective_completed = completed | inactive
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

    def _notify_input_mission_completed(self, input_id: int) -> None:
        if input_id in self._input_completion_notified:
            return
        self._input_completion_notified.add(input_id)
        self._send_0503_notification(f"협업기저임무 ID={input_id}")

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
                f"0503 협업기저임무 완료 알림을 발신했습니다. ({log_context})",
            )
        except Exception as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"0503 메시지 발신 실패({log_context}): {exc}",
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
            f"[COLLAB] 협업기저임무 재수행 모드 진입 (timestamp={ts})",
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
            f"[COLLAB] 협업기저임무 재수행 모드 해제 ({reason})",
        )
        try:
            self.manager.logic_store.set_data("collab_reexecute_mode", False)
        except Exception:
            pass

    def _extract_input_mission_ids(self, data: Any) -> List[int]:
        mission_list = getattr(data, "inputMissionList", None)
        if mission_list is None and isinstance(data, dict):
            mission_list = data.get("inputMissionList")
        ids: Set[int] = set()
        for item in mission_list or []:
            value = self._safe_get(item, "inputMissionID", "InputMissionID")
            if value is None:
                continue
            try:
                ids.add(int(value))
            except (TypeError, ValueError):
                continue
        return sorted(ids)

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
        option_names = ["시스템추천", "촬영 효과 우선", "비행 효과 우선"]
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
        self._send_0503_notification("전체 협업기저임무 완료")


    def _mark_individual_mission_done(
        self, key: Tuple[int, int, Optional[int]]
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
        """메시지 ID에 따라 데이터 클래스 인스턴스를 생성하여 반환합니다."""
        timestamp = int(
            (
                datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)
            ).total_seconds()
            * 1000
        )
        source_module = "MonitoringModule"

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
