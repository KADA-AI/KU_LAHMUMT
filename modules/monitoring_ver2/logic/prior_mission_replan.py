from __future__ import annotations

from dataclasses import asdict
from typing import Any, Dict, List, Optional, TYPE_CHECKING

import udp_reporter
from modules.monitoring_ver2.data.message_models import (
    InputMissionIDModel,
    OptionListModel,
    PriorMissionListModel,
    ReplanRequestBodyModel,
    ReplanRequestTimeStampModel,
)
from modules.monitoring_ver2.logic.replan_utils import ensure_replan_level_details_file
from modules.monitoring_ver2.push.message0902_push import make_and_push as push_message_0902
from modules.common import prior_replan_store

if TYPE_CHECKING:
    from modules.monitoring_ver2.logic.monitoring_logic_part import MonitoringLogic


MISSION_TYPE_LABELS = {
    1: "좌표지향",
    2: "표적추적",
}


class PriorMissionReplanCoordinator:
    """Handle 0202 PriorMissionInfo inputs and emit dedicated 0902 replan requests."""

    def __init__(self, logic: "MonitoringLogic") -> None:
        self.logic = logic
        self.manager = logic.manager
        self._handled_prior_missions: Dict[int, int] = {}

    def process(self, plan_context: Optional[Dict[str, Any]]) -> None:
        prior_info = self._get_prior_info()
        if not prior_info:
            return
        message_ts = self._to_int(self._safe_get(prior_info, "timestamp", "Timestamp"))
        if message_ts is None:
            return
        entries = self._extract_prior_entries(prior_info)
        if not entries:
            return
        source = self._safe_get(prior_info, "source", "Source") or "MMR"

        dispatched: List[int] = []
        for entry in entries:
            mission_id = entry.get("priorMissionID")
            mission_type = entry.get("missionType")
            if mission_id is None or mission_type is None:
                continue
            last_ts = self._handled_prior_missions.get(mission_id)
            if last_ts is not None and message_ts <= last_ts:
                continue
            payload = self._build_replan_payload(entry, plan_context, source)
            if not payload:
                continue
            body, context = payload
            if self._dispatch_replan(body, context):
                dispatched.append(mission_id)
                self._handled_prior_missions[mission_id] = message_ts

        if dispatched:
            labels = ", ".join(str(mid) for mid in dispatched)
            self.manager._log(
                "PRIOR_MISSION",
                "INFO",
                f"0202 prior missions processed → 0902 dispatched (priorMissionID={labels})",
            )

    # ------------------------------------------------------------------ #
    # Payload builders
    # ------------------------------------------------------------------ #

    def _build_replan_payload(
        self,
        entry: Dict[str, Any],
        plan_context: Optional[Dict[str, Any]],
        source: str,
    ) -> Optional[tuple[ReplanRequestBodyModel, Dict[str, Any]]]:
        mission_id = entry.get("priorMissionID")
        mission_type = entry.get("missionType")
        if mission_id is None or mission_type is None:
            return None

        timestamp = self.logic._current_time_ms()
        reason = f"선행임무 : {MISSION_TYPE_LABELS.get(mission_type, f'타입 {mission_type}')}"
        input_models = self._build_input_mission_models(plan_context)
        option_id = self.logic._allocate_option_ids(1)[0]
        mission_plan_id = self.logic._allocate_mission_plan_ids(1)[0]
        option_model = OptionListModel(
            optionID=option_id,
            optionName="선행임무 반영",
            missionPlanID=mission_plan_id,
        )
        prior_model = PriorMissionListModel(
            priorMissionID=mission_id,
            missionType=mission_type,
        )
        body = ReplanRequestBodyModel(
            source=str(source),
            timestamp=timestamp,
            replanRequestTime=ReplanRequestTimeStampModel(
                replanRequestTimestamp=timestamp
            ),
            replanLevel=4,
            inputMissionIDList=input_models,
            IndividualMissionIDList=[],
            priorMissionList=[prior_model],
            replanRequest=reason,
            optionList=[option_model],
        )
        context = {
            "timestamp": timestamp,
            "source": source,
            "reason": reason,
            "missionPlanID": mission_plan_id,
            "priorMissionID": mission_id,
            "missionType": mission_type,
            "optionID": option_id,
            "options": [asdict(option_model)],
            "inputMissionIDs": [model.inputMissionID for model in input_models],
        }
        orientation_note = self._summarize_orientation(entry)
        if orientation_note:
            context["orientation"] = orientation_note
        self._persist_detail_bundle(mission_plan_id, entry, timestamp)
        return body, context

    def _build_input_mission_models(
        self, plan_context: Optional[Dict[str, Any]]
    ) -> List[InputMissionIDModel]:
        ids: List[int] = []
        if isinstance(plan_context, dict):
            for value in plan_context.get("inputMissionIDs") or []:
                converted = self._to_int(value)
                if converted is not None:
                    ids.append(converted)
        if not ids:
            current = getattr(self.logic, "_current_input_mission_id", None)
            converted = self._to_int(current)
            if converted is not None:
                ids.append(converted)
        if not ids:
            ids.append(0)
        return [InputMissionIDModel(inputMissionID=i) for i in ids]

    def _summarize_orientation(self, entry: Dict[str, Any]) -> Optional[str]:
        mission_type = entry.get("missionType")
        if mission_type == 1:
            coord_block = self._safe_get(entry, "coordinateOrientation", "CoordinateOrientation")
            coordinate = self._safe_get(coord_block, "coordinate", "Coordinate") or {}
            lat = self._to_float(self._safe_get(coordinate, "latitude", "Latitude"))
            lon = self._to_float(self._safe_get(coordinate, "longitude", "Longitude"))
            alt = self._to_float(self._safe_get(coordinate, "altitude", "Altitude"))
            if lat is None or lon is None:
                return None
            if alt is None:
                return f"좌표({lat:.6f}, {lon:.6f})"
            return f"좌표({lat:.6f}, {lon:.6f}, alt={alt:.1f})"
        if mission_type == 2:
            target_block = self._safe_get(entry, "targetOrientation", "TargetOrientation")
            target_id = self._to_int(self._safe_get(target_block or {}, "targetID", "TargetID"))
            if target_id is None:
                return None
            return f"표적ID={target_id}"
        return None

    # ------------------------------------------------------------------ #
    # Dispatch helpers
    # ------------------------------------------------------------------ #

    def _dispatch_replan(
        self,
        body: ReplanRequestBodyModel,
        context: Dict[str, Any],
    ) -> bool:
        try:
            ensure_replan_level_details_file()
        except Exception as exc:
            self.manager._log(
                "PRIOR_MISSION",
                "WARN",
                f"replanInfo 준비 실패: {exc}",
            )
        try:
            push_message_0902(body, self.manager.node_messenger)
        except Exception as exc:
            self.manager._log(
                "PRIOR_MISSION",
                "ERROR",
                f"0902 push 실패: {exc}",
            )
            return False

        try:
            self.manager.push_store.add_data("0902", body)
        except Exception:
            pass
        try:
            udp_reporter.notify_tx("0902")
        except Exception:
            pass

        payload = dict(context)
        payload["status"] = "requested"
        try:
            self.manager.logic_store.set_data("prior_mission_replan_state", payload)
        except Exception:
            pass
        if self.manager.gui_update_callback:
            try:
                self.manager.gui_update_callback("logic", "0902", payload)
            except Exception:
                pass
        self.manager._log(
            "PRIOR_MISSION",
            "INFO",
            f"선행임무 재계획 요청 발신 (priorMissionID={context.get('priorMissionID')}, missionPlanID={context.get('missionPlanID')})",
        )
        return True
    def _persist_detail_bundle(self, mission_plan_id: int, entry: Dict[str, Any], timestamp: int) -> None:
        detail_payload = {
            "sourceMissionPlanID": getattr(self.logic, "_current_mission_plan_id", None),
            "priorMissionID": entry.get("priorMissionID"),
            "missionType": entry.get("missionType"),
            "targetCoordinate": self._extract_coordinate(entry),
            "rawEntry": entry,
            "timestamp": timestamp,
        }
        try:
            detail_path = prior_replan_store.save_detail(mission_plan_id, detail_payload)
            self.manager._log(
                "PRIOR_MISSION",
                "INFO",
                f"prior detail 저장 완료 (missionPlanID={mission_plan_id}, path={detail_path})",
            )
        except Exception as exc:
            self.manager._log(
                "PRIOR_MISSION",
                "WARN",
                f"prior detail 저장 실패 (missionPlanID={mission_plan_id}): {exc}",
            )

    # ------------------------------------------------------------------ #
    # Extraction helpers
    # ------------------------------------------------------------------ #

    def _get_prior_info(self) -> Optional[Any]:
        try:
            return self.manager.receive_store.get_data("0202")
        except Exception:
            return None

    def _extract_prior_entries(self, prior_info: Any) -> List[Dict[str, Any]]:
        raw_list = self._safe_get(prior_info, "priorMissionList", "PriorMissionList")
        entries: List[Dict[str, Any]] = []
        for item in raw_list or []:
            mission_id = self._to_int(self._safe_get(item, "priorMissionID", "PriorMissionID"))
            mission_type = self._to_int(self._safe_get(item, "missionType", "MissionType"))
            if mission_id is None or mission_type is None:
                continue
            entries.append(
                {
                    "priorMissionID": mission_id,
                    "missionType": mission_type,
                    "coordinateOrientation": self._safe_get(
                        item, "coordinateOrientation", "CoordinateOrientation"
                    ),
                    "targetOrientation": self._safe_get(
                        item, "targetOrientation", "TargetOrientation"
                    ),
                }
            )
        return entries

    # ------------------------------------------------------------------ #
    # Utility helpers
    # ------------------------------------------------------------------ #

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

    def _to_float(self, value: Any) -> Optional[float]:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _extract_coordinate(self, entry: Dict[str, Any]) -> Optional[Dict[str, float]]:
        coord_block = self._safe_get(entry, "coordinateOrientation", "CoordinateOrientation")
        coordinate = self._safe_get(coord_block, "coordinate", "Coordinate") or {}
        lat = self._to_float(self._safe_get(coordinate, "latitude", "Latitude"))
        lon = self._to_float(self._safe_get(coordinate, "longitude", "Longitude"))
        alt = self._to_float(self._safe_get(coordinate, "altitude", "Altitude"))
        if lat is None or lon is None:
            return None
        payload = {"latitude": lat, "longitude": lon}
        if alt is not None:
            payload["altitude"] = alt
        return payload
