# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Callable

from modules.common import prior_replan_store
from modules.monitoring.logic.init_replan import (
    allocate_mission_plan_ids,
    collect_input_mission_ids,
)
from modules.monitoring.logic.mission_update import parse_payload


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _coerce_float(value: object) -> float | None:
    try:
        return float(value)
    except Exception:
        return None


def _extract_timestamp(plan: dict[str, Any]) -> int | None:
    for key in ("timestamp", "Timestamp", "timeStamp", "TimeStamp"):
        if key in plan:
            return _coerce_int(plan.get(key))
    return None


def _extract_source(plan: dict[str, Any]) -> str:
    for key in ("source", "Source", "sourceModuleName", "SourceModuleName"):
        if key in plan:
            try:
                text = str(plan.get(key))
            except Exception:
                text = ""
            if text:
                return text
    return "MMR"


def _extract_prior_entries(plan: dict[str, Any]) -> list[dict[str, Any]]:
    raw_list = plan.get("priorMissionList") or plan.get("PriorMissionList") or []
    entries: list[dict[str, Any]] = []
    for item in raw_list:
        if not isinstance(item, dict):
            continue
        prior_id = _coerce_int(item.get("priorMissionID") or item.get("PriorMissionID"))
        mission_type = _coerce_int(item.get("missionType") or item.get("MissionType"))
        if prior_id is None or mission_type is None:
            continue
        entries.append(
            {
                "priorMissionID": prior_id,
                "missionType": mission_type,
                "coordinateOrientation": item.get("coordinateOrientation")
                or item.get("CoordinateOrientation"),
                "targetOrientation": item.get("targetOrientation")
                or item.get("TargetOrientation"),
                "rawEntry": item,
            }
        )
    return entries


def _extract_coordinate(entry: dict[str, Any]) -> dict[str, Any]:
    coord_block = entry.get("coordinateOrientation") or {}
    coordinate = {}
    if isinstance(coord_block, dict):
        coordinate = coord_block.get("coordinate") or coord_block.get("Coordinate") or {}
    if not isinstance(coordinate, dict):
        return {}

    def _pick(keys: Iterable[str]) -> object | None:
        for key in keys:
            if key in coordinate:
                return coordinate.get(key)
        return None

    # NOTE: do not use "or" here; 0.0 is a valid value but falsy.
    lat = _coerce_float(_pick(("latitude", "Latitude")))
    lon = _coerce_float(_pick(("longitude", "Longitude")))
    alt = _coerce_float(_pick(("altitude", "Altitude")))
    out: dict[str, Any] = {}
    if lat is not None:
        out["latitude"] = lat
    if lon is not None:
        out["longitude"] = lon
    if alt is not None:
        out["altitude"] = alt
    return out


MISSION_TYPE_LABELS: dict[int, str] = {
    1: "좌표지정",
    2: "표적추적",
}


@dataclass
class PriorMissionState:
    handled_prior_ts: dict[int, int] = field(default_factory=dict)


class PriorMissionReplanCoordinator:
    """Handle 0202 PriorMissionInfo inputs and emit level-4 0902 requests."""

    OPTION_LABEL = "선행임무 반영"
    REPLAN_LEVEL = 4
    DL_RISK_REPLAN_LEVEL = 5

    def __init__(
        self,
        *,
        now_fn: Callable[[], int],
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._now_ms = now_fn
        self._log = logger
        self._state = PriorMissionState()

    def on_prior_mission(
        self,
        payload: object | None,
        *,
        system_mode: int | None,
        current_mission_plan_id: int | None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        logs: list[str] = []
        if system_mode not in (3, 4):
            return [], logs

        plan = parse_payload(payload)
        if not plan:
            return [], logs

        source = _extract_source(plan)
        message_ts = _extract_timestamp(plan)
        if message_ts is None:
            message_ts = int(self._now_ms())

        entries = _extract_prior_entries(plan)
        if not entries:
            return [], logs

        mission_ids = collect_input_mission_ids()
        if not mission_ids:
            mission_ids = [0]
        input_models = [{"inputMissionID": int(mid)} for mid in mission_ids]

        dispatch_payloads: list[dict[str, Any]] = []
        for entry in entries:
            prior_id = int(entry["priorMissionID"])
            mission_type = int(entry["missionType"])

            last_ts = self._state.handled_prior_ts.get(prior_id)
            if last_ts is not None and int(message_ts) <= int(last_ts):
                continue

            now_ts = int(self._now_ms())
            mission_plan_id = self._allocate_plan_id(now_ts)
            reason = self._build_reason(mission_type)
            coordinate = _extract_coordinate(entry)

            detail_payload = {
                "sourceMissionPlanID": current_mission_plan_id,
                "priorMissionID": prior_id,
                "missionType": mission_type,
                "targetCoordinate": coordinate,
                "targetOrientation": entry.get("targetOrientation") or {},
                "timestamp": now_ts,
                "rawEntry": entry.get("rawEntry") or {},
            }
            # Persist detail for the prior-mission pipeline without adding
            # non-ICD fields into the outgoing 0902 payload.
            self._persist_detail(mission_plan_id, detail_payload)

            # NOTE: the current mission-planning prior pipeline expects exactly
            # one plan option to carry the missionPlanID, even though this is
            # not a user-facing multi-option flow.
            option_block = [
                {
                    "optionID": 1,
                    "optionName": self.OPTION_LABEL,
                    "missionPlanID": int(mission_plan_id),
                }
            ]
            prior_block = [
                {
                    "priorMissionID": prior_id,
                    "missionType": mission_type,
                }
            ]

            payload_0902: dict[str, Any] = {
                "timestamp": now_ts,
                "source": "MSM",
                "replanRequestTime": {"replanRequestTimestamp": now_ts},
                "replanLevel": int(self.REPLAN_LEVEL),
                "replanRequest": reason,
                "inputMissionIDList": input_models,
                "priorMissionList": prior_block,
                "pendingOptionList": option_block,
                "replanDetail": detail_payload,
            }
            dispatch_payloads.append(payload_0902)
            self._state.handled_prior_ts[prior_id] = int(message_ts)
            logs.append(
                "[PRIOR] 0202 processed -> dispatching 0902"
                f" (priorMissionID={prior_id}, missionPlanID={mission_plan_id})"
            )

        return dispatch_payloads, logs

    def on_risk_update(
        self,
        risk_score: float,
        *,
        system_mode: int | None,
        current_mission_plan_id: int | None,
        risky_aircraft_ids: list[int] | None = None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        """Handle DL-based risk updates and emit level-5 0902 requests."""
        logs: list[str] = []
        if system_mode not in (3, 4):
            return [], logs
        if risk_score <= 0.5:
            return [], logs

        now_ts = int(self._now_ms())
        mission_plan_id = self._allocate_plan_id(now_ts)

        risky_ids_str = ",".join(map(str, sorted(risky_aircraft_ids))) if risky_aircraft_ids else "Unknown"
        reason = f"Risk analysis: high risk detected (Score: {risk_score:.2f}, AC: {risky_ids_str})"

        mission_ids = collect_input_mission_ids()
        if not mission_ids:
            mission_ids = [0]
        input_models = [{"inputMissionID": int(mid)} for mid in mission_ids]

        option_block = [
            {
                "optionID": 1,
                "optionName": "Risk Avoidance",
                "missionPlanID": int(mission_plan_id),
            }
        ]
        payload_0902: dict[str, Any] = {
            "timestamp": now_ts,
            "source": "MSM",
            "replanRequestTime": {"replanRequestTimestamp": now_ts},
            "replanLevel": int(self.DL_RISK_REPLAN_LEVEL),
            "replanRequest": reason,
            "inputMissionIDList": input_models,
            "priorMissionList": [],
            "pendingOptionList": option_block,
            "replanDetail": {
                "sourceMissionPlanID": current_mission_plan_id,
                "riskScore": float(risk_score),
                "riskyAircraftIDList": [int(aid) for aid in (risky_aircraft_ids or [])],
                "timestamp": now_ts,
            },
        }
        logs.append(f"[DL] High Risk ({risk_score:.2f}, AC:{risky_ids_str}) -> dispatching 0902")
        return [payload_0902], logs

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _allocate_plan_id(self, now_ts: int) -> int:
        allocated = allocate_mission_plan_ids(1)
        if allocated:
            try:
                return int(allocated[0])
            except Exception:
                pass
        # Fallback seed to avoid blocking the flow on allocator failure.
        return int(700_000_000 + (int(now_ts) % 1_000))

    def _build_reason(self, mission_type: int) -> str:
        label = MISSION_TYPE_LABELS.get(int(mission_type))
        if label:
            return f"선행임무 : {label}"
        return f"선행임무 : 타입 {int(mission_type)}"

    def _persist_detail(self, mission_plan_id: int, detail_payload: dict[str, Any]) -> Any | None:
        try:
            return prior_replan_store.save_detail(int(mission_plan_id), detail_payload)
        except Exception:
            return None
