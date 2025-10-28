"""재계획 판단 로직 (monitoring_backup 구현 통합)."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

import udp_reporter
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

    replan_situation = str(confirmed_request.get("재계획 상황", "")).strip()
    replan_level = 1 if replan_situation == "운용자 입력에 의한 재계획" else 2
    timestamp_ms = _now_timestamp_ms()

    replan_body = ReplanRequestBodyModel(
        source="MonitoringModule",
        timestamp=timestamp_ms,
        replanRequestTime=ReplanRequestTimeStampModel(replanRequestTimestamp=timestamp_ms),
        replanLevel=replan_level,
        inputMissionIDList=[],
        IndividualMissionIDList=[],
        priorMissionList=[],
        replanRequest=_serialize_reason(confirmed_request),
        optionList=[],
    )

    push_message_0902(replan_body, manager.node_messenger)
    manager.push_store.add_data("0902", replan_body)
    udp_reporter.notify_tx("0902")

    manager._log(
        "REPLAN_PUSH",
        "INFO",
        f"재계획 요청(0902) 전송 완료. level={replan_level}, situation={replan_situation}",
    )

    manager.logic_store.set_data(
        "final_replan_output",
        FinalReplanOutput(
            new_plan={
                "status": "replan_request_sent",
                "replanLevel": replan_level,
                "situation": replan_situation,
            },
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

