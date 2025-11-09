"""재계획 판단 로직 (monitoring_backup 통합 버전)."""

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
from modules.monitoring_ver2.utils.create_0201_attack import create_attack_plan_from_target
from .replan_utils import ensure_replan_level_details_file, mark_targets_as_used


FORCED_HOLD_DELAY_SECONDS = 10.0
FORCED_HOLD_DELAY_REASON = "강제대기 후 10초 경과"
FORCED_HOLD_DEADLINE_ATTR = "_hold_defer_deadline"
FORCED_HOLD_REASON_ATTR = "_hold_defer_reason"

TARGET_TYPE_LABELS = {
    0: "None",
    1: "전차",
    2: "장갑차",
    3: "방사포",
    4: "곡사포",
    5: "고정고사포",
    6: "군인",
}
REPLAN_FIELD_SITUATION = "재계획상황"
REPLAN_FIELD_CONDITION = "재계획조건"
REPLAN_FIELD_REASON = "재계획사유"
REPLAN_FIELD_DETAIL = "재계획상세 사유"


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
        mission_desc = {1: "의무투입 재계획", 2: "재배치 재계획"}.get(mission_type)
        return f"임무 재계획 요청({mission_desc})" if mission_desc else "임무 재계획 요청"

    if msg_id == "0801":
        return "운용자 명령으로 인한 재계획"

    if msg_id == "0802":
        mandatory_desc = {
            1: "강제대기로 인한 재계획",
            2: "강제귀환으로 인한 재계획",
            3: "강제임무복귀로 인한 재계획",
        }.get(_extract_mandatory_type(detail))
        return mandatory_desc or "강제명령으로 인한 재계획"

    if msg_id == "0402":
        detail_obj = None
        if isinstance(detail, str):
            try:
                detail_obj = json.loads(detail)
            except Exception:
                detail_obj = None
        elif isinstance(detail, dict):
            detail_obj = detail

        targets = []
        if isinstance(detail_obj, dict):
            maybe_targets = detail_obj.get("targets")
            if isinstance(maybe_targets, list):
                targets = maybe_targets
        elif isinstance(detail, list):
            targets = detail

        summary_parts: List[str] = []
        for target in targets:
            if not isinstance(target, dict):
                continue
            watcher_raw = target.get("watcherID")
            target_raw = target.get("targetID")
            type_raw = target.get("targetType")

            try:
                watcher_id = int(watcher_raw) if watcher_raw is not None else None
            except (TypeError, ValueError):
                watcher_id = None
            try:
                target_id = int(target_raw) if target_raw is not None else None
            except (TypeError, ValueError):
                target_id = None
            try:
                type_id = int(type_raw) if type_raw is not None else None
            except (TypeError, ValueError):
                type_id = None

            watcher_label = f"{watcher_id}번 무인기" if watcher_id is not None else "무인기"
            if target_id is not None:
                target_label = f"target ID {target_id}번"
            else:
                target_label = f"target ID {target_raw}"
            type_name = TARGET_TYPE_LABELS.get(type_id, "알 수 없음")
            summary_parts.append(f"{watcher_label}의 {target_label}({type_name}) 최초 발견")

        if summary_parts:
            return "0402 신규 표적 감지: " + "; ".join(summary_parts)
        return "0402 신규 표적 감지"

    return _serialize_reason(replan_info)

def _convert_trigger_for_ui(replan_info: Dict[str, Any]) -> Dict[str, Any]:
    """Convert trigger information into the structure expected by the UI tab."""
    detail = _find_detail_payload(replan_info)
    if isinstance(detail, (dict, list)):
        detail_text = json.dumps(detail, ensure_ascii=False, default=_json_default)
    elif detail is None:
        detail_text = None
    else:
        detail_text = str(detail)

    return {
        "ReplanReason": str(replan_info.get(REPLAN_FIELD_REASON, "")),
        "ReplanSituation": str(replan_info.get(REPLAN_FIELD_SITUATION, "")),
        "ReplanDetail": detail_text,
        "MessageID": replan_info.get("original_message_id"),
    }


def _find_detail_payload(replan_info: Dict[str, Any]) -> Any:
    candidates = [
        REPLAN_FIELD_DETAIL,
        "detail",
        "Detail",
    ]
    for key in replan_info.keys():
        key_str = str(key)
        if any(token in key_str for token in candidates):
            try:
                return replan_info[key]
            except Exception:
                continue
    return None


def _resolve_target_list(detail_payload: Any) -> Optional[List[Any]]:
    if detail_payload is None:
        return None
    candidate_keys = ("targets", "targetList", "newTargets")
    if isinstance(detail_payload, dict):
        for key in candidate_keys:
            value = detail_payload.get(key)
            if value:
                return value
    for key in candidate_keys:
        if hasattr(detail_payload, key):
            value = getattr(detail_payload, key)
            if value:
                return value
    return None


def _extract_primary_target(detail_payload: Any) -> Any:
    targets = _resolve_target_list(detail_payload)
    if not targets:
        return None
    for entry in targets:
        if entry is None:
            continue
        if isinstance(entry, dict):
            coord = entry.get("coordinate") or entry.get("Coordinate")
        else:
            coord = getattr(entry, "coordinate", None) or getattr(entry, "Coordinate", None)
        if coord:
            return entry
    return None


def _create_attack_plan_file(detail_payload: Any, manager) -> Optional[str]:
    target_entry = _extract_primary_target(detail_payload)
    if not target_entry:
        return None
    try:
        path, meta = create_attack_plan_from_target(target_entry=target_entry)
    except Exception as exc:
        try:
            manager._log("REPLAN_PUSH", "WARN", f"Attack 0201 preparation failed: {exc}")
        except Exception:
            pass
        return None
    try:
        manager._log(
            "REPLAN_PUSH",
            "INFO",
            f"Attack 0201 stub ready ({meta.get('output_path')}); missionID={meta.get('mission_id')}",
        )
    except Exception:
        pass
    return meta.get("output_path")

def judge_replan_situation(manager) -> List[Dict[str, Any]]:
    """Evaluate incoming messages and determine whether a replan should be triggered."""
    replan_situations: List[Dict[str, Any]] = []

    msg_0202 = manager.receive_store.get_data("0202")
    msg_0402 = manager.receive_store.get_data("0402")
    msg_0801 = manager.receive_store.get_data("0801")
    msg_0802 = manager.receive_store.get_data("0802")

    manager._log(
        "REPLAN_JUDGE",
        "INFO",
        f"Replan judge inputs -> 0202:{msg_0202 is not None}, "
        f"0801:{msg_0801 is not None}, 0802:{msg_0802 is not None}, 0402:{msg_0402 is not None}",
    )

    operator_inputs = {"0202": msg_0202, "0801": msg_0801, "0802": msg_0802}
    operator_situation_labels = {
        "0202": "운용자 요청 재계획",
        "0801": "운용자 명령 재계획",
        "0802": "강제 명령 재계획",
    }

    for msg_id, reason_data in operator_inputs.items():
        if not reason_data:
            continue

        current_ts = getattr(reason_data, "timestamp", None)
        if current_ts is None:
            manager._log(
                "REPLAN_JUDGE",
                "WARN",
                f"{msg_id} message missing timestamp; skipping operator-triggered evaluation.",
            )
            continue

        last_ts_key = f"replan_last_ts_{msg_id}"
        last_ts = manager.logic_store.get_data(last_ts_key)
        if current_ts == last_ts:
            continue

        manager._log(
            "REPLAN_JUDGE",
            "INFO",
            f"{msg_id} operator-triggered replan candidate detected (ts={current_ts}).",
        )

        fallback_reason = {
            "0202": "운용자 임무 재계획 요청",
            "0801": "운용자 명령",
            "0802": "강제 명령",
        }.get(msg_id, f"운용자 요청 ({msg_id})")

        replan_reason = getattr(reason_data, "replan_reason", fallback_reason)
        replan_info = {
            REPLAN_FIELD_SITUATION: operator_situation_labels.get(msg_id, "운용자 요청 재계획"),
            REPLAN_FIELD_REASON: replan_reason,
            REPLAN_FIELD_DETAIL: reason_data,
            "original_message_id": msg_id,
        }
        replan_situations.append(replan_info)
        manager.logic_store.set_data(last_ts_key, current_ts)

    if msg_0402:
        current_ts = getattr(msg_0402, "timestamp", None)
        if current_ts is None:
            manager._log(
                "REPLAN_JUDGE",
                "WARN",
                "0402 message missing timestamp; cannot evaluate new target trigger.",
            )
        else:
            last_ts_key = "replan_last_ts_0402"
            last_ts = manager.logic_store.get_data(last_ts_key)
            if current_ts == last_ts:
                manager.logic_store.set_data("targetInfoNewDetections", [])
            else:
                new_targets = manager.logic_store.get_data("targetInfoNewDetections") or []
                filtered_targets = [dict(target) for target in new_targets if isinstance(target, dict)]

                if filtered_targets:
                    manager._log(
                        "REPLAN_JUDGE",
                        "INFO",
                        f"0402 new target trigger registered (count={len(filtered_targets)}, ts={current_ts}).",
                    )
                    replan_info = {
                        REPLAN_FIELD_SITUATION: "표적 재계획",
                        REPLAN_FIELD_CONDITION: "new_target_detected",
                        REPLAN_FIELD_REASON: "0402 new target detected",
                        REPLAN_FIELD_DETAIL: {
                            "timestamp": current_ts,
                            "targets": filtered_targets,
                        },
                        "original_message_id": "0402",
                    }
                    replan_situations.append(replan_info)
                else:
                    manager._log(
                        "REPLAN_JUDGE",
                        "INFO",
                        f"0402 message received without actionable targets (ts={current_ts}); skipping trigger.",
                    )

                manager.logic_store.set_data("targetInfoNewDetections", [])
                manager.logic_store.set_data(last_ts_key, current_ts)

    return replan_situations

def manage_replan_triggers(manager) -> Optional[Dict[str, Any]]:
    """Select the next replan trigger and expose it to the UI."""
    situations: List[Dict[str, Any]] = manager.logic_store.get_data("ReplanSituations") or []
    if not situations:
        manager.logic_store.set_data("replan_triggers", [])
        return None

    def _priority(item: Dict[str, Any]) -> int:
        situation_text = str(item.get(REPLAN_FIELD_SITUATION, ""))
        return 0 if "운용자" in situation_text else 1

    situations.sort(key=_priority)
    chosen = situations[0]

    manager.logic_store.set_data("ConfirmedReplanRequest", chosen)
    manager.logic_store.set_data("ReplanSituations", situations[1:])
    manager.logic_store.set_data("replan_triggers", [_convert_trigger_for_ui(chosen)])
    manager._log(
        "REPLAN_TRIGGER",
        "INFO",
        f"Replan trigger selected: {chosen.get('original_message_id')} ({chosen.get(REPLAN_FIELD_REASON)})",
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
    option_names = ["옵션 A", "옵션 B", "옵션 C"]
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


def _derive_replan_situation(msg_id: str, confirmed_request: Dict[str, Any]) -> str:
    raw_situation = (
        confirmed_request.get(REPLAN_FIELD_SITUATION)
        or confirmed_request.get("situation")
        or ""
    )
    replan_situation = str(raw_situation).strip()
    if replan_situation:
        return replan_situation

    if msg_id == "0801":
        return "운용자 명령 재계획"
    if msg_id == "0802":
        return "강제 명령 재계획"
    return "운용자 요청 재계획"


def _determine_replan_level(msg_id: str, replan_situation: str) -> int:
    if "운용자" in replan_situation or msg_id in ("0801", "0802"):
        return 1
    return 2


def _prepare_common_replan_payload(
    manager,
    confirmed_request: Dict[str, Any],
    *,
    msg_id: str,
) -> Dict[str, Any]:
    timestamp_ms = _now_timestamp_ms()
    replan_situation = _derive_replan_situation(msg_id, confirmed_request)
    replan_level = _determine_replan_level(msg_id, replan_situation)
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
            if done or key_int in excluded_input_ids or key_int in completed_input_ids:
                continue
            fallback_ids.append(key_int)
        if fallback_ids:
            input_ids = fallback_ids

    if not input_ids:
        _, mission_list = _resolve_input_plan_payload(manager, plan_context)
        derived: List[int] = []
        for mission in mission_list:
            if _safe_get(mission, "isDone", "IsDone"):
                continue
            input_id = _safe_int(
                _safe_get(mission, "inputMissionID", "InputMissionID", "inputMissionId")
            )
            if input_id is None or input_id <= 0:
                continue
            if input_id in excluded_input_ids or input_id in completed_input_ids:
                continue
            derived.append(input_id)
        if derived:
            existing: Set[int] = set(input_ids)
            for value in derived:
                if value in existing or value <= 0 or value in completed_input_ids:
                    continue
                existing.add(value)
                input_ids.append(value)

    if not input_ids:
        latest_ids = _load_latest_input_plan_ids(excluded_input_ids)
        existing: Set[int] = set(input_ids)
        for value in latest_ids:
            if value <= 0 or value in completed_input_ids or value in existing:
                continue
            existing.add(value)
            input_ids.append(value)

    latest_ids_override = _load_latest_input_plan_ids(set())
    if latest_ids_override:
        override_ids: List[int] = []
        seen_override: Set[int] = set()
        for value in latest_ids_override:
            if value is None or value <= 0 or value in seen_override:
                continue
            seen_override.add(value)
            override_ids.append(value)
        if override_ids:
            input_ids = override_ids

    filtered_input_ids: List[int] = []
    seen_inputs: Set[int] = set()
    for value in input_ids:
        if value is None or value <= 0 or value in seen_inputs:
            continue
        seen_inputs.add(value)
        filtered_input_ids.append(value)

    input_ids = filtered_input_ids
    input_models = [InputMissionIDModel(inputMissionID=i) for i in input_ids]
    if not input_models:
        input_ids = [0]
        input_models = [InputMissionIDModel(inputMissionID=0)]

    if monitoring_logic is not None and hasattr(monitoring_logic, "_build_collab_option_list"):
        try:
            option_models, mission_plan_ids = monitoring_logic._build_collab_option_list()
        except Exception:
            option_models, mission_plan_ids = _fallback_option_list(manager, timestamp_ms)
    else:
        option_models, mission_plan_ids = _fallback_option_list(manager, timestamp_ms)
    if not mission_plan_ids:
        mission_plan_ids = [opt.missionPlanID for opt in option_models]

    return {
        "timestamp_ms": timestamp_ms,
        "replan_situation": replan_situation,
        "replan_level": replan_level,
        "reason_text": reason_text,
        "input_ids": input_ids,
        "input_models": input_models,
        "option_models": option_models,
        "mission_plan_ids": mission_plan_ids,
        "excluded_aircraft_ids": excluded_aircraft_ids,
        "excluded_input_ids": excluded_input_ids,
    }


def _prepare_replan_payload_for_0402(manager, confirmed_request: Dict[str, Any]) -> Dict[str, Any]:
    """0402 기반 재계획 준비 구간 (현재는 기본 로직과 동일)."""
    return _prepare_common_replan_payload(manager, confirmed_request, msg_id="0402")


def _prepare_replan_payload_for_0801(manager, confirmed_request: Dict[str, Any]) -> Dict[str, Any]:
    """0801 기반 재계획 준비 구간 (현재는 기본 로직과 동일)."""
    return _prepare_common_replan_payload(manager, confirmed_request, msg_id="0801")


def _prepare_replan_payload_for_0802(manager, confirmed_request: Dict[str, Any]) -> Dict[str, Any]:
    """0802 기반 재계획 준비 구간 (현재는 기본 로직과 동일)."""
    return _prepare_common_replan_payload(manager, confirmed_request, msg_id="0802")


def _prepare_replan_payload_default(
    manager, confirmed_request: Dict[str, Any], msg_id: str
) -> Dict[str, Any]:
    """그 외 메시지에 대한 기본 재계획 준비 구간."""
    return _prepare_common_replan_payload(manager, confirmed_request, msg_id=msg_id)


def determine_level_and_send_request(manager, confirmed_request: Optional[Dict[str, Any]]) -> None:
    """Build and send a 0902 replan request based on the confirmed trigger."""
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
    detail_payload = _find_detail_payload(confirmed_request)
    attack_plan_path = None

    if msg_id == "0402":
        attack_plan_path = _create_attack_plan_file(detail_payload, manager)
        preparation = _prepare_replan_payload_for_0402(manager, confirmed_request)
    elif msg_id == "0801":
        preparation = _prepare_replan_payload_for_0801(manager, confirmed_request)
    elif msg_id == "0802":
        preparation = _prepare_replan_payload_for_0802(manager, confirmed_request)
    else:
        preparation = _prepare_replan_payload_default(manager, confirmed_request, msg_id)

    timestamp_ms = preparation["timestamp_ms"]
    replan_situation = preparation["replan_situation"]
    replan_level = preparation["replan_level"]
    reason_text = preparation["reason_text"]
    input_ids = preparation["input_ids"]
    input_models = preparation["input_models"]
    option_models = preparation["option_models"]
    mission_plan_ids = preparation["mission_plan_ids"]
    excluded_aircraft_ids = preparation["excluded_aircraft_ids"]
    excluded_input_ids = preparation["excluded_input_ids"]

    if msg_id == "0402" and option_models:
        try:
            option_models[0].optionName = "공격추천"
        except Exception:
            pass

    try:
        ensure_replan_level_details_file()
    except Exception as exc:
        try:
            manager._log("REPLAN_PUSH", "WARN", f"Failed to prepare replanLevelDetails file: {exc}")
        except Exception:
            pass

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

    if msg_id == "0402":
        targets_payload = None
        if isinstance(detail_payload, dict):
            targets_payload = detail_payload.get("targets")
        elif hasattr(detail_payload, "targets"):
            targets_payload = getattr(detail_payload, "targets")

        if targets_payload:
            if isinstance(targets_payload, (list, tuple, set)):
                targets_iterable = list(targets_payload)
            else:
                targets_iterable = [targets_payload]
            try:
                updated_info = mark_targets_as_used(targets_iterable)
            except Exception as exc:
                try:
                    manager._log(
                        "REPLAN_PUSH",
                        "WARN",
                        f"Failed to mark targets as used after 0902: {exc}",
                    )
                except Exception:
                    pass
            else:
                try:
                    manager.logic_store.set_data("targetInfo", updated_info)
                except Exception:
                    pass

    manager._log(
        "REPLAN_PUSH",
        "INFO",
        f"Sent 0902 replan request. level={replan_level}, situation={replan_situation}, "
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
    """Execute the full replan pipeline: judge, manage triggers, and send 0902."""
    manager._log("REPLAN_PROCEDURE", "INFO", "--- Replan procedure START ---")

    new_situations = judge_replan_situation(manager)
    if new_situations:
        existing = manager.logic_store.get_data("ReplanSituations") or []
        existing.extend(new_situations)
        manager.logic_store.set_data("ReplanSituations", existing)
        manager._log(
            "REPLAN_PROCEDURE",
            "INFO",
            f"Replan situations accumulated: {len(new_situations)} entries.",
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

    manager._log("REPLAN_PROCEDURE", "INFO", "--- Replan procedure END ---")
