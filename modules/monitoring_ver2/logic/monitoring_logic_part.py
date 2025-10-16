# logic/monitoring_logic_part.py: '모니터링' 도메인에 대한 세부 비즈니스 로직을 구현합니다.

from datetime import datetime, timezone

from typing import Any, Dict, Optional, Union

# --- 데이터 모델 import ---
from data.message_models import (
    ModuleStatusModelModel,
    MissionProgressBodyModel,
    MissionEndRequestBodyModel,
    ReplanRequestBodyModel,
)
from push.push_center import push_message
from .monitoring_actual_logic import run_monitoring_procedure
import udp_reporter
import socket
import json
import os
from modules.common import db_paths


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

    def _process_mission_plan_update(self) -> None:
        data_0903 = None
        try:
            data_0903 = self.manager.receive_store.get_data("0903")
        except Exception:
            data_0903 = None

        mission_plan_id = None
        if data_0903:
            mission_plan_id = getattr(data_0903, "missionPlanID", None)

        if mission_plan_id is None:
            try:
                data_0902 = self.manager.receive_store.get_data("0902")
            except Exception:
                data_0902 = None
            if data_0902:
                # 메시지 구조가 dict 또는 객체일 수 있으므로 getattr/키 조회 병행
                mission_plan_id = getattr(data_0902, "missionPlanID", None)
                if mission_plan_id is None and isinstance(data_0902, dict):
                    mission_plan_id = data_0902.get("missionPlanID")

        if mission_plan_id is None:
            mission_plan_id = self._scan_latest_mission_plan_id()

        try:
            mission_plan_id = int(mission_plan_id) if mission_plan_id is not None else None
        except (TypeError, ValueError):
            mission_plan_id = None

        if mission_plan_id is None or mission_plan_id == self._current_mission_plan_id:
            return

        mission_plan_path = db_paths.get_db_subpath("MissionPlan", f"{mission_plan_id}.json")
        if not mission_plan_path.exists():
            # MissionPlan 파일이 아직 생성되지 않았다면 다음 사이클까지 대기
            return
        try:
            context = self._load_mission_plan_context(mission_plan_id)
        except FileNotFoundError as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"MissionPlan {mission_plan_id} file missing: {exc}",
            )
            return
        except Exception as exc:
            self.manager._log(
                "MON_LOGIC",
                "WARN",
                f"MissionPlan {mission_plan_id} load failed: {exc}",
            )
            return
        self._plan_context = context
        self._current_mission_plan_id = mission_plan_id
        try:
            self.manager.logic_store.set_data("current_mission_plan", context)
        except Exception:
            pass
        self.manager._log(
            "MON_LOGIC",
            "INFO",
            f"MissionPlan {mission_plan_id} loaded for monitoring",
        )

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
                body_0501 = run_monitoring_procedure(data_401, plan_context, current_plan_id)

                # 연료 경고 로직
                feul_data = []
                prev_warnings = dict(
                    self.manager.logic_store.get_data("fuel_warning_prev") or {}
                )

                for agent_state in data_401.agentStateList:
                    if agent_state.isUnmanned == 1:
                        text = ""
                        fuel_level = 0
                        if agent_state.fuel * 100 // 100 <= 10:
                            text = "red"
                            fuel_level = 2
                        elif agent_state.fuel * 100 // 100 <= 20:
                            text = "yellow"
                            fuel_level = 1
                        else:
                            text = "green"

                        feul_data.append(
                            {"id": agent_state.aircraftID, "warning": text}
                        )

                        if fuel_level in (1, 2):
                            last_state = prev_warnings.get(agent_state.aircraftID)
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
                                    "aircraftID": agent_state.aircraftID,
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
                        prev_warnings[agent_state.aircraftID] = text

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

                # fuel_data를 LogicStorage에 저장하고 GUI 업데이트
                if feul_data:  # feul_data가 비어있지 않은 경우에만 처리
                    self.manager.logic_store.set_data("fuel_data", feul_data)
                    if self.manager.gui_update_callback:
                        self.manager.gui_update_callback(
                            "logic", "fuel_data", feul_data
                        )
            else:
                self.manager._log(
                    "MON_LOGIC", "INFO", "401 데이터가 없어 모니터링을 건너뜁니다."
                )

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
