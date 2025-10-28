"""재계획 판단 로직 (monitoring_backup 구현 통합)."""

from __future__ import annotations

import json
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
    detail = replan_info.get("재계획 상세 사유")

    if msg_id == "0202":
        mission_type = _extract_first_mission_type(detail)
        mission_desc = {
            1: "좌표지향 요청",
            2: "표적추적 요청",
        }.get(mission_type)
        return f"선행임무 입력({mission_desc})" if mission_desc else "선행임무 입력"

    if msg_id == "0801":
        return "운용자 요청으로 인한 임무재계획"

    if msg_id == "0802":
        mandatory_desc = {
            1: "강제 대기",
            2: "강제 귀환",
            3: "강제 임무복귀",
        }.get(_extract_mandatory_type(detail))
        return f"강제명령({mandatory_desc})" if mandatory_desc else "강제명령"

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


def _collect_replan_inputs(
    manager,
    excluded_aircraft: Set[int],
) -> Tuple[List[int], Dict[str, Any], Any, Dict[int, bool], Set[int]]:
    plan_context, status_map, monitoring_logic = _gather_plan_context(manager)
    aircraft_map = plan_context.get("aircraft") or {}

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
            if input_id is not None:
                excluded_input_ids.add(input_id)

    candidate_ids: List[int] = []
    raw_ids = plan_context.get("inputMissionIDs") or []
    for value in raw_ids:
        value_int = _safe_int(value)
        if value_int is not None:
            candidate_ids.append(value_int)

    if not candidate_ids:
        candidate_ids.extend(status_map.keys())

    if not candidate_ids:
        for payload in aircraft_map.values():
            missions = (payload or {}).get("missions") or []
            for mission in missions:
                input_id = _safe_int(_safe_get(mission, "inputMissionID", "inputMissionId"))
                if input_id is not None:
                    candidate_ids.append(input_id)

    if not candidate_ids:
        input_plan = manager.receive_store.get_data("0201")
        if input_plan is None:
            package_id = _safe_int(_safe_get(plan_context, "inputMissionPackageID", "InputMissionPackageID"))
            if package_id is not None:
                try:
                    plan_path = db_paths.get_db_subpath("InputMissionPlan", f"{package_id}.json")
                    with plan_path.open('r', encoding='utf-8') as fh:
                        input_plan = json.load(fh)
                except Exception:
                    input_plan = None
        mission_list = _safe_get(input_plan, "inputMissionList", "InputMissionList") if input_plan else []
        for mission in mission_list or []:
            if _safe_get(mission, "isDone", "IsDone"):
                continue
            input_id = _safe_int(_safe_get(mission, "inputMissionID", "InputMissionID"))
            if input_id is not None:
                candidate_ids.append(input_id)

    filtered_ids: List[int] = []
    seen: Set[int] = set()
    for candidate in sorted(candidate_ids):
        if candidate in seen:
            continue
        seen.add(candidate)
        if candidate in excluded_input_ids:
            continue
        if status_map.get(candidate):
            continue
        filtered_ids.append(candidate)

    return filtered_ids, plan_context, monitoring_logic, status_map, excluded_input_ids


def _fallback_option_list(timestamp_ms: int) -> Tuple[List[OptionListModel], List[int]]:
    option_names = ["시스템추천", "촬영 효과 우선", "비행 효과 우선"]
    base_plan_id = 700_000_000 + (timestamp_ms % 10_000)
    options: List[OptionListModel] = []
    mission_plan_ids: List[int] = []
    for idx, name in enumerate(option_names, start=1):
        plan_id = base_plan_id + idx
        mission_plan_ids.append(plan_id)
        options.append(
            OptionListModel(
                optionID=idx,
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
    ) = _collect_replan_inputs(manager, excluded_aircraft_ids)

    if not input_ids:
        fallback_ids = sorted(
            i for i, done in status_map.items() if not done and i not in excluded_input_ids
        )
        if fallback_ids:
            input_ids = fallback_ids
    if not input_ids:
        input_plan = manager.receive_store.get_data("0201")
        if input_plan is None:
            package_id = _safe_int(_safe_get(plan_context, "inputMissionPackageID", "InputMissionPackageID"))
            if package_id is not None:
                try:
                    plan_path = db_paths.get_db_subpath("InputMissionPlan", f"{package_id}.json")
                    with plan_path.open('r', encoding='utf-8') as fh:
                        input_plan = json.load(fh)
                except Exception:
                    input_plan = None
        mission_list = _safe_get(input_plan, "inputMissionList", "InputMissionList") if input_plan else []
        derived = []
        for mission in mission_list or []:
            if _safe_get(mission, "isDone", "IsDone"):
                continue
            input_id = _safe_int(_safe_get(mission, "inputMissionID", "InputMissionID"))
            if input_id is None:
                continue
            if input_id in excluded_input_ids:
                continue
            derived.append(input_id)
        if derived:
            input_ids = sorted(set(input_ids) | set(derived))
    input_models = [InputMissionIDModel(inputMissionID=i) for i in input_ids]
    if not input_models:
        input_models = [InputMissionIDModel(inputMissionID=0)]

    option_models: List[OptionListModel] = []
    mission_plan_ids: List[int] = []
    if monitoring_logic is not None and hasattr(monitoring_logic, "_build_collab_option_list"):
        try:
            option_models, mission_plan_ids = monitoring_logic._build_collab_option_list()
        except Exception:
            option_models, mission_plan_ids = _fallback_option_list(timestamp_ms)
    else:
        option_models, mission_plan_ids = _fallback_option_list(timestamp_ms)
    if not mission_plan_ids:
        mission_plan_ids = [opt.missionPlanID for opt in option_models]

    replan_body = ReplanRequestBodyModel(
        source="MonitoringModule",
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
