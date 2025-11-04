"""재계획 판단 로직 (monitoring_backup 구현 통합)."""

from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Set, Tuple

import udp_reporter
from modules.common import db_paths
from data.logic_model import FinalReplanOutput
from data.message_models import (
    IndividualMissionIDListModel,
    InputMissionIDModel,
    OptionListModel,
    PriorMissionListModel,
    ReplanRequestBodyModel,
    ReplanRequestTimeStampModel,
)
from push.message0902_push import make_and_push as push_message_0902


FORCED_HOLD_DELAY_SECONDS = 10.0
FORCED_HOLD_DELAY_REASON = "강제대기 후 10초 경과"
FORCED_HOLD_DEADLINE_ATTR = "_hold_defer_deadline"
FORCED_HOLD_REASON_ATTR = "_hold_defer_reason"


def _now_timestamp_ms() -> int:
    """Return the current UTC timestamp in milliseconds (epoch: 2000-01-01)."""
    return int(
        (datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)).total_seconds()
        * 1000
    )


def _json_default(obj: Any) -> Any:
    """Default serializer for JSON dumping of complex objects."""
    if hasattr(obj, "__dict__"):
        try:
            return obj.__dict__
        except Exception:
            return str(obj)
    if isinstance(obj, set):
        return list(obj)
    return str(obj)


def _serialize_reason(data: Any) -> str:
    """Serialize confirmed replan information into JSON for the 0902 payload."""
    try:
        return json.dumps(data, ensure_ascii=False, default=_json_default)
    except Exception:
        return json.dumps(str(data), ensure_ascii=False)


def _extract_first_mission_type(detail: Any) -> Optional[int]:
    """Return the first missionType from a PriorMissionInfo payload."""
    try:
        if hasattr(detail, "priorMissionList"):
            prior_list = detail.priorMissionList
        elif isinstance(detail, dict):
            prior_list = detail.get("priorMissionList")
        else:
            prior_list = None
        if not prior_list:
            return None
        first = prior_list[0]
        if isinstance(first, dict):
            return int(first.get("missionType")) if first.get("missionType") is not None else None
        if hasattr(first, "missionType"):
            return int(first.missionType)
    except Exception:
        return None
    return None


def _extract_mandatory_type(detail: Any) -> Optional[int]:
    """Return the mandatoryType from a ForcedCommand payload."""
    try:
        if isinstance(detail, dict):
            value = detail.get("mandatoryType")
        elif hasattr(detail, "mandatoryType"):
            value = detail.mandatoryType
        else:
            value = None
        return int(value) if value is not None else None
    except Exception:
        return None


def _format_replan_reason(replan_info: Dict[str, Any]) -> str:
    """Generate human readable replanReason text for selected triggers."""
    msg_id = str(replan_info.get("original_message_id") or "").zfill(4)
    detail = _find_detail_payload(replan_info)

    if msg_id == "0202":
        mission_type = _extract_first_mission_type(detail)
        mission_desc = {1: "좌표지정 요청", 2: "추적임무 요청"}.get(mission_type)
        return f"선행임무 입력({mission_desc})" if mission_desc else "선행임무 입력"

    if msg_id == "0801":
        return "운용자 명령으로 인한 재계획"

    if msg_id == "0802":
        mandatory_desc = {
            1: "강제대기로 인한 재계획",
            2: "강제귀환으로 인한 재계획",
            3: "강제임무복귀로 인한 재계획",
        }.get(_extract_mandatory_type(detail))
        return mandatory_desc or "강제명령으로 인한 재계획"

    return _serialize_reason(replan_info)

def _convert_trigger_for_ui(replan_info: Dict[str, Any]) -> Dict[str, Any]:
    """Convert monitoring_backup-style trigger info into the structure ReplanTab expects."""
    detail = replan_info.get("재계획 상세 사유")
    if isinstance(detail, (dict, list)):
        detail_text = json.dumps(detail, ensure_ascii=False, default=_json_default)
    elif detail is None:
        detail_text = None
    else:
        detail_text = str(detail)

    return {
        "ReplanReason": str(replan_info.get("재계획 사유", "")),
        "ReplanSituation": str(replan_info.get("재계획 상황", "")),
        "ReplanDetail": detail_text,
        "MessageID": replan_info.get("original_message_id"),
    }


def judge_replan_situation(manager) -> List[Dict[str, Any]]:
    """monitoring_backup 버전의 재계획 상황 판단 로직."""
    replan_situations: List[Dict[str, Any]] = []

    msg_0202 = manager.receive_store.get_data("0202")
    msg_0402 = manager.receive_store.get_data("0402")
    msg_0801 = manager.receive_store.get_data("0801")
    msg_0802 = manager.receive_store.get_data("0802")

    manager._log(
        "REPLAN_JUDGE",
        "INFO",
        f"재계획 판단 입력 상태 -> 0202:{msg_0202 is not None}, "
        f"0801:{msg_0801 is not None}, 0802:{msg_0802 is not None}, 0402:{msg_0402 is not None}",
    )

    operator_inputs = {"0202": msg_0202, "0801": msg_0801, "0802": msg_0802}
    for msg_id, reason_data in operator_inputs.items():
        if not reason_data:
            continue
        if msg_id == "0802":
            mandatory_type_raw = getattr(reason_data, "mandatoryType", None)
            try:
                mandatory_type = int(mandatory_type_raw)
            except (TypeError, ValueError):
                mandatory_type = None
            if mandatory_type == 1:
                now = time.monotonic()
                deadline = getattr(reason_data, FORCED_HOLD_DEADLINE_ATTR, None)
                if deadline is None:
                    try:
                        setattr(
                            reason_data,
                            FORCED_HOLD_DEADLINE_ATTR,
                            now + FORCED_HOLD_DELAY_SECONDS,
                        )
                    except Exception:
                        pass
                    continue
                try:
                    deadline_value = float(deadline)
                except (TypeError, ValueError):
                    try:
                        setattr(
                            reason_data,
                            FORCED_HOLD_DEADLINE_ATTR,
                            now + FORCED_HOLD_DELAY_SECONDS,
                        )
                    except Exception:
                        pass
                    continue
                if now < deadline_value:
                    continue
                try:
                    hold_reason = getattr(
                        reason_data,
                        FORCED_HOLD_REASON_ATTR,
                        FORCED_HOLD_DELAY_REASON,
                    )
                    setattr(reason_data, "replan_reason", hold_reason)
                except Exception:
                    pass
                try:
                    if hasattr(reason_data, FORCED_HOLD_DEADLINE_ATTR):
                        delattr(reason_data, FORCED_HOLD_DEADLINE_ATTR)
                except Exception:
                    pass
                try:
                    if hasattr(reason_data, FORCED_HOLD_REASON_ATTR):
                        delattr(reason_data, FORCED_HOLD_REASON_ATTR)
                except Exception:
                    pass

        current_ts = getattr(reason_data, "timestamp", None)
        if current_ts is None:
            manager._log(
                "REPLAN_JUDGE", "WARN", f"메시지 {msg_id} 에 timestamp 가 없어 건너뜁니다."
            )
            continue

        last_ts_key = f"replan_last_ts_{msg_id}"
        last_ts = manager.logic_store.get_data(last_ts_key)
        if current_ts == last_ts:
            continue

        manager._log(
            "REPLAN_JUDGE",
            "INFO",
            f"{msg_id} 에 대한 신규 재계획 이벤트 감지 (ts={current_ts}).",
        )

        replan_reason = getattr(reason_data, "replan_reason", f"운용자 입력 ({msg_id})")
        replan_info = {
            "재계획 상황": "운용자 입력에 의한 재계획",
            "재계획 사유": replan_reason,
            "재계획 상세 사유": reason_data,
            "original_message_id": msg_id,
        }
        replan_situations.append(replan_info)
        manager.logic_store.set_data(last_ts_key, current_ts)

    if msg_0402:
        current_ts = getattr(msg_0402, "timestamp", None)
        if current_ts is None:
            manager._log(
                "REPLAN_JUDGE", "WARN", "메시지 0402 에 timestamp 가 없어 건너뜁니다."
            )
        else:
            last_ts_key = "replan_last_ts_0402"
            last_ts = manager.logic_store.get_data(last_ts_key)
            if current_ts != last_ts:
                manager._log(
                    "REPLAN_JUDGE",
                    "INFO",
                    f"0402 상황 인지 정보 기반 재계획 이벤트 감지 (ts={current_ts}).",
                )
                replan_info = {
                    "재계획 상황": "상황 재계획",
                    "재계획 조건": "협업 임무 중 위협 탐지",
                    "재계획 사유": "신규 ROI 탐지",
                    "재계획 상세 사유": getattr(msg_0402, "ROIInfo", None),
                    "original_message_id": "0402",
                }
                replan_situations.append(replan_info)
                manager.logic_store.set_data(last_ts_key, current_ts)

    return replan_situations


def manage_replan_triggers(manager) -> Optional[Dict[str, Any]]:
    """감지된 재계획 트리거 중 우선순위가 높은 항목을 확정."""
    situations: List[Dict[str, Any]] = manager.logic_store.get_data("ReplanSituations") or []
    if not situations:
        manager.logic_store.set_data("replan_triggers", [])
        return None

    # 운용자 입력 기반 요청을 최우선으로 처리.
    situations.sort(
        key=lambda item: 0 if "운용자" in str(item.get("재계획 상황", "")) else 1
    )
    chosen = situations[0]

    manager.logic_store.set_data("ConfirmedReplanRequest", chosen)
    manager.logic_store.set_data("ReplanSituations", situations[1:])
    manager.logic_store.set_data("replan_triggers", [_convert_trigger_for_ui(chosen)])
    manager._log(
        "REPLAN_TRIGGER",
        "INFO",
        f"재계획 트리거 확정: {chosen.get('original_message_id')} ({chosen.get('재계획 사유')})",
    )
    return chosen


def _safe_int(value: Any) -> Optional[int]:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _safe_get(source: Any, *names: str) -> Any:
    for name in names:
        if isinstance(source, dict) and name in source:
            return source[name]
        if hasattr(source, name):
            return getattr(source, name)
    return None


def _find_detail_payload(replan_info: Dict[str, Any]) -> Any:
    candidates = ["재계획 상세 사유", "?ш퀎???곸꽭 ?ъ쑀", "상세", "detail"]
    for key in replan_info.keys():
        key_str = str(key)
        if any(token in key_str for token in candidates):
            try:
                return replan_info[key]
            except Exception:
                continue
    return None


def _extract_forced_aircraft_ids(replan_info: Dict[str, Any]) -> Set[int]:
    msg_id = str(replan_info.get("original_message_id") or "").zfill(4)
    if msg_id != "0802":
        return set()
    detail = _find_detail_payload(replan_info)

    aircraft_ids: Set[int] = set()

    def _handle(entry: Any) -> None:
        if entry is None:
            return
        if isinstance(entry, (list, tuple, set)):
            for item in entry:
                _handle(item)
            return
        aircraft_value = _safe_get(entry, "aircraftID", "aircraftId", "aircraft_id")
        mandatory_value = _safe_get(entry, "mandatoryType", "MandatoryType", "mandatory_type")
        mandatory_int = _safe_int(mandatory_value)
        if mandatory_int not in (1, 2):
            return
        aircraft_int = _safe_int(aircraft_value)
        if aircraft_int is not None:
            aircraft_ids.add(aircraft_int)

    _handle(detail)
    return aircraft_ids


def _gather_plan_context(manager) -> Tuple[Dict[str, Any], Dict[int, bool], Any]:
    monitoring_logic = getattr(getattr(manager, "logic_handler", None), "monitoring_logic", None)
    if monitoring_logic is not None:
        plan_context = getattr(monitoring_logic, "_plan_context", None) or {}
        status_map = dict(getattr(monitoring_logic, "_input_mission_status", {}) or {})
    else:
        plan_context = manager.logic_store.get_data("current_mission_plan") or {}
        status_map = manager.logic_store.get_data("input_mission_status") or {}

    normalized_status: Dict[int, bool] = {}
    for key, value in status_map.items():
        key_int = _safe_int(key)
        if key_int is not None:
            normalized_status[key_int] = bool(value)

    return plan_context, normalized_status, monitoring_logic


def _load_latest_input_plan_ids(
    excluded_input_ids: Set[int],
) -> List[int]:
    try:
        plan_root = db_paths.get_db_subpath("InputMissionPlan")
    except Exception:
        return []

    try:
        latest_path = max(
            (p for p in plan_root.glob("*.json") if p.is_file()),
            key=lambda p: p.stat().st_mtime,
        )
    except ValueError:
        return []
    except Exception:
        return []

    try:
        with latest_path.open("r", encoding="utf-8") as fh:
            plan_data = json.load(fh)
    except Exception:
        return []

    mission_list = plan_data.get("inputMissionList") or []
    latest_ids: List[int] = []
    for mission in mission_list:
        if _safe_get(mission, "isDone", "IsDone"):
            continue
        input_id = _safe_int(_safe_get(mission, "inputMissionID", "InputMissionID"))
        if (
            input_id is None
            or input_id <= 0
            or input_id in excluded_input_ids
        ):
            continue
        latest_ids.append(input_id)
    return latest_ids


def _resolve_input_plan_payload(manager, plan_context: Dict[str, Any]) -> Tuple[Any, List[Any]]:
    """
    Retrieve the latest input mission plan payload along with its mission list.
    Falls back to disk when no in-memory copy is available.
    """
    input_plan = manager.receive_store.get_data("0201")
    if input_plan is None:
        package_id = _safe_int(
            _safe_get(plan_context, "inputMissionPackageID", "InputMissionPackageID")
        )
        if package_id is not None:
            try:
                plan_path = db_paths.get_db_subpath("InputMissionPlan", f"{package_id}.json")
                with plan_path.open("r", encoding="utf-8") as fh:
                    input_plan = json.load(fh)
            except Exception:
                input_plan = None

    mission_list = (
        _safe_get(input_plan, "inputMissionList", "InputMissionList") if input_plan else None
    )
    if mission_list is None:
        normalized: List[Any] = []
    elif isinstance(mission_list, list):
        normalized = mission_list
    elif isinstance(mission_list, tuple):
        normalized = list(mission_list)
    else:
        try:
            normalized = list(mission_list)  # type: ignore[arg-type]
        except Exception:
            normalized = [mission_list]
    return input_plan, normalized


def _collect_replan_inputs(
    manager,
    excluded_aircraft: Set[int],
) -> Tuple[List[int], Dict[str, Any], Any, Dict[int, bool], Set[int], Set[int]]:
    plan_context, status_map, monitoring_logic = _gather_plan_context(manager)
    aircraft_map = plan_context.get("aircraft") or {}
    _, input_mission_list = _resolve_input_plan_payload(manager, plan_context)

    completed_input_ids: Set[int] = set()
    for payload in aircraft_map.values():
        missions = (payload or {}).get("missions") or []
        for mission in missions:
            if _safe_get(mission, "isDone", "IsDone"):
                completed_id = _safe_int(
                    _safe_get(mission, "inputMissionID", "InputMissionID", "inputMissionId")
                )
                if completed_id is not None and completed_id > 0:
                    completed_input_ids.add(completed_id)

    for mission in input_mission_list:
        if not _safe_get(mission, "isDone", "IsDone"):
            continue
        completed_id = _safe_int(
            _safe_get(mission, "inputMissionID", "InputMissionID", "inputMissionId")
        )
        if completed_id is not None and completed_id > 0:
            completed_input_ids.add(completed_id)

    excluded_input_ids: Set[int] = set()
    for aircraft_key, payload in aircraft_map.items():
        aircraft_id = _safe_int(aircraft_key)
        if aircraft_id is None:
            aircraft_id = _safe_int(_safe_get(payload or {}, "aircraftID", "aircraftId"))
        if aircraft_id is None or aircraft_id not in excluded_aircraft:
            continue
        missions = (payload or {}).get("missions") or []
        for mission in missions:
            input_id = _safe_int(_safe_get(mission, "inputMissionID", "inputMissionId"))
            if input_id is not None and input_id > 0:
                excluded_input_ids.add(input_id)

    candidate_ids: List[int] = []
    raw_ids = plan_context.get("inputMissionIDs") or []
    for value in raw_ids:
        value_int = _safe_int(value)
        if (
            value_int is not None
            and value_int > 0
            and value_int not in completed_input_ids
        ):
            candidate_ids.append(value_int)

    if not candidate_ids:
        for key in status_map.keys():
            key_int = _safe_int(key)
            if (
                key_int is not None
                and key_int > 0
                and key_int not in completed_input_ids
            ):
                candidate_ids.append(key_int)

    if not candidate_ids:
        for payload in aircraft_map.values():
            missions = (payload or {}).get("missions") or []
            for mission in missions:
                input_id = _safe_int(_safe_get(mission, "inputMissionID", "inputMissionId"))
                if (
                    input_id is not None
                    and input_id > 0
                    and input_id not in completed_input_ids
                ):
                    candidate_ids.append(input_id)

    if not candidate_ids:
        for mission in input_mission_list:
            if _safe_get(mission, "isDone", "IsDone"):
                continue
            input_id = _safe_int(
                _safe_get(mission, "inputMissionID", "InputMissionID", "inputMissionId")
            )
            if (
                input_id is not None
                and input_id > 0
                and input_id not in completed_input_ids
            ):
                candidate_ids.append(input_id)

    if not candidate_ids:
        candidate_ids.extend(_load_latest_input_plan_ids(excluded_input_ids))

    filtered_ids: List[int] = []
    seen: Set[int] = set()
    for candidate in candidate_ids:
        if candidate in seen:
            continue
        seen.add(candidate)
        if candidate in excluded_input_ids:
            continue
        if candidate in completed_input_ids:
            continue
        if status_map.get(candidate):
            continue
        filtered_ids.append(candidate)

    return (
        filtered_ids,
        plan_context,
        monitoring_logic,
        status_map,
        excluded_input_ids,
        completed_input_ids,
    )



def _allocate_unique_option_ids(manager, count: int) -> List[int]:
    monitoring_logic = getattr(getattr(manager, "logic_handler", None), "monitoring_logic", None)
    if monitoring_logic and hasattr(monitoring_logic, "_allocate_option_ids"):
        try:
            return monitoring_logic._allocate_option_ids(count)
        except Exception:
            pass
    try:
        counter = int(manager.logic_store.get_data("_option_id_counter") or 0)
    except Exception:
        counter = 0
    allocated: List[int] = []
    for _ in range(count):
        counter += 1
        allocated.append(counter)
    try:
        manager.logic_store.set_data("_option_id_counter", counter)
    except Exception:
        pass
    if monitoring_logic and hasattr(monitoring_logic, "_used_option_ids"):
        try:
            monitoring_logic._used_option_ids.update(allocated)
        except Exception:
            pass
    return allocated


def _fallback_option_list(manager, timestamp_ms: int) -> Tuple[List[OptionListModel], List[int]]:
    option_names = ["?????", "?? ?? ??", "?? ?? ??"]
    base_plan_id = 700_000_000 + (timestamp_ms % 10_000)
    options: List[OptionListModel] = []
    mission_plan_ids: List[int] = []
    option_ids = _allocate_unique_option_ids(manager, len(option_names))
    for idx, (name, option_id) in enumerate(zip(option_names, option_ids), start=1):
        plan_id = base_plan_id + idx
        mission_plan_ids.append(plan_id)
        options.append(
            OptionListModel(
                optionID=option_id,
                optionName=name,
                missionPlanID=plan_id,
            )
        )
    return options, mission_plan_ids

def determine_level_and_send_request(manager, confirmed_request: Optional[Dict[str, Any]]) -> None:
    """확정된 재계획 요청을 0902 메시지로 전송."""
    if not confirmed_request:
        manager.logic_store.set_data(
            "final_replan_output",
            FinalReplanOutput(
                new_plan={"status": "no_change"},
                replan_status="COMPLETED",
                final_replan_type=None,
            ),
        )
        return

    msg_id = str(confirmed_request.get("original_message_id") or "").zfill(4)
    raw_situation = (
        confirmed_request.get("재계획 상황")
        or confirmed_request.get("?ш퀎???곹솴")
        or confirmed_request.get("situation")
        or ""
    )
    replan_situation = str(raw_situation).strip()
    if not replan_situation and msg_id == "0801":
        replan_situation = "운용자 입력에 의한 재계획"
    if not replan_situation and msg_id == "0802":
        replan_situation = "강제명령 기반 재계획"
    if "운용자" in replan_situation:
        replan_level = 1
    elif msg_id in ("0801", "0802"):
        replan_level = 1
    else:
        replan_level = 2
    timestamp_ms = _now_timestamp_ms()
    reason_text = _format_replan_reason(confirmed_request)

    excluded_aircraft_ids = _extract_forced_aircraft_ids(confirmed_request)
    (
        input_ids,
        plan_context,
        monitoring_logic,
        status_map,
        excluded_input_ids,
        completed_input_ids,
    ) = _collect_replan_inputs(manager, excluded_aircraft_ids)

    if not input_ids:
        fallback_ids: List[int] = []
        for key, done in status_map.items():
            key_int = _safe_int(key)
            if key_int is None or key_int <= 0:
                continue
            if done:
                continue
            if key_int in excluded_input_ids:
                continue
            if key_int in completed_input_ids:
                continue
            fallback_ids.append(key_int)
        if fallback_ids:
            input_ids = fallback_ids
    if not input_ids:
        _, mission_list = _resolve_input_plan_payload(manager, plan_context)
        derived = []
        for mission in mission_list:
            if _safe_get(mission, "isDone", "IsDone"):
                continue
            input_id = _safe_int(
                _safe_get(mission, "inputMissionID", "InputMissionID", "inputMissionId")
            )
            if input_id is None or input_id <= 0:
                continue
            if input_id in excluded_input_ids:
                continue
            if input_id in completed_input_ids:
                continue
            derived.append(input_id)
        if derived:
            existing: Set[int] = set(input_ids)
            for value in derived:
                if value in existing or value <= 0:
                    continue
                if value in completed_input_ids:
                    continue
                existing.add(value)
                input_ids.append(value)
    if not input_ids:
        latest_ids = _load_latest_input_plan_ids(excluded_input_ids)
        if latest_ids:
            existing: Set[int] = set(input_ids)
            for value in latest_ids:
                if value <= 0:
                    continue
                if value in completed_input_ids:
                    continue
                if value in existing:
                    continue
                existing.add(value)
                input_ids.append(value)
    latest_ids_override = _load_latest_input_plan_ids(set())
    if latest_ids_override:
        override_ids: List[int] = []
        seen_override: Set[int] = set()
        for value in latest_ids_override:
            if value is None or value <= 0:
                continue
            if value in seen_override:
                continue
            seen_override.add(value)
            override_ids.append(value)
        input_ids = override_ids
    filtered_input_ids: List[int] = []
    seen_inputs: Set[int] = set()
    for value in input_ids:
        if value is None or value <= 0:
            continue
        if value in seen_inputs:
            continue
        seen_inputs.add(value)
        filtered_input_ids.append(value)

    input_ids = filtered_input_ids
    input_models = [InputMissionIDModel(inputMissionID=i) for i in input_ids]
    if not input_models:
        input_ids = [0]
        input_models = [InputMissionIDModel(inputMissionID=0)]

    option_models: List[OptionListModel] = []
    mission_plan_ids: List[int] = []
    if monitoring_logic is not None and hasattr(monitoring_logic, "_build_collab_option_list"):
        try:
            option_models, mission_plan_ids = monitoring_logic._build_collab_option_list()
        except Exception:
            option_models, mission_plan_ids = _fallback_option_list(manager, timestamp_ms)
    else:
        option_models, mission_plan_ids = _fallback_option_list(manager, timestamp_ms)
    if not mission_plan_ids:
        mission_plan_ids = [opt.missionPlanID for opt in option_models]

    replan_body = ReplanRequestBodyModel(
        source="MSM",
        timestamp=timestamp_ms,
        replanRequestTime=ReplanRequestTimeStampModel(replanRequestTimestamp=timestamp_ms),
        replanLevel=replan_level,
        inputMissionIDList=input_models,
        IndividualMissionIDList=[],
        priorMissionList=[],
        replanRequest=reason_text,
        optionList=option_models,
    )

    push_message_0902(replan_body, manager.node_messenger)
    manager.push_store.add_data("0902", replan_body)
    udp_reporter.notify_tx("0902")

    manager._log(
        "REPLAN_PUSH",
        "INFO",
        f"재계획 요청(0902) 전송 완료. level={replan_level}, situation={replan_situation}, "
        f"inputIDs={input_ids}, excludedAircraft={sorted(excluded_aircraft_ids)}, optionPlanIDs={mission_plan_ids}",
    )

    new_plan_summary = {
        "status": "replan_request_sent",
        "replanLevel": replan_level,
        "situation": replan_situation,
        "inputMissionIDs": input_ids,
        "optionPlanIDs": mission_plan_ids,
        "excludedAircraft": sorted(excluded_aircraft_ids),
        "excludedInputMissionIDs": sorted(excluded_input_ids),
    }

    manager.logic_store.set_data(
        "final_replan_output",
        FinalReplanOutput(
            new_plan=new_plan_summary,
            replan_status="TRIGGERED",
            final_replan_type=replan_situation or None,
        ),
    )
    manager.logic_store.set_data("ConfirmedReplanRequest", None)



def run_replan_procedure(manager):
    """재계획 판단, 트리거 관리, 메시지 전송까지 전체 절차를 실행."""
    manager._log("REPLAN_PROCEDURE", "INFO", "--- 재계획 절차 START ---")

    new_situations = judge_replan_situation(manager)
    if new_situations:
        existing = manager.logic_store.get_data("ReplanSituations") or []
        existing.extend(new_situations)
        manager.logic_store.set_data("ReplanSituations", existing)
        manager._log(
            "REPLAN_PROCEDURE",
            "INFO",
            f"재계획 상황 {len(new_situations)}건 누적.",
        )

    pending = manager.logic_store.get_data("ReplanSituations") or []
    if pending:
        manager.logic_store.set_data(
            "replan_triggers",
            [_convert_trigger_for_ui(item) for item in pending],
        )
    else:
        manager.logic_store.set_data("replan_triggers", [])

    confirmed = manage_replan_triggers(manager)
    determine_level_and_send_request(manager, confirmed)

    manager._log("REPLAN_PROCEDURE", "INFO", "--- 재계획 절차 END ---")
