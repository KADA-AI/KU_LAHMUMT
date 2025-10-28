import time
import json
import importlib
from dataclasses import is_dataclass, asdict
from datetime import datetime, timezone

# C# Imports needed for the new push function
try:
    from System.Collections.Generic import List
    from nFusion.Model.msg_0902 import ReplanRequest
    from nFusion.Model.CommonType import ReplanRequestTime, InputMissionID, IndividualMissionID, PriorMission, PendingOption
except ImportError:
    print("[WARNING] Could not import C# types. Push functionality will be limited.")
    List = list

from .logic_utils import log_to_file

# --- Start of self-contained helpers for 0902 push ---

def _to_dict_recursive(obj):
    """Recursively converts nested dataclasses into dictionaries."""
    if isinstance(obj, list):
        return [_to_dict_recursive(item) for item in obj]
    if isinstance(obj, dict):
        return {key: _to_dict_recursive(value) for key, value in obj.items()}
    if is_dataclass(obj):
        return _to_dict_recursive(asdict(obj))
    return obj

def _h_try_set(obj, name: str, value) -> bool:
    """Helper to set attribute with PascalCase fallback."""
    for k in (name, name[:1].upper() + name[1:] if name else name):
        try:
            if hasattr(obj, k):
                setattr(obj, k, value)
                return True
        except Exception:
            pass
    return False

def _h_new(name: str):
    """Helper to create new C# object instances."""
    try:
        return globals()[name]()
    except KeyError:
        print(f"[ERROR] C# type not found: {name}. Using a dummy object.")
        return type(f"Dummy_{name}", (object,), {})()

def _h_dict_to_ReplanRequest(data: dict) -> 'ReplanRequest':
    """Converts a dictionary to a C# ReplanRequest object."""
    obj = _h_new('ReplanRequest')
    if "timestamp" in data: _h_try_set(obj, "timestamp", int(data["timestamp"]))
    if "source" in data: _h_try_set(obj, "source", str(data["source"]))
    if "replanLevel" in data: _h_try_set(obj, "replanLevel", int(data["replanLevel"]))
    if "replanReason" in data: _h_try_set(obj, "replanReason", str(data["replanReason"]))
    return obj

def _send_0902_request(manager, body_dict: dict):
    """
    Self-contained function to create and push a 0902 ReplanRequest message.
    """
    reason_dict = body_dict.get("ReplanReason", {})
    # Recursively convert dataclasses to pure dicts for clean JSON.
    clean_dict = _to_dict_recursive(reason_dict)
    reason_str = json.dumps(clean_dict, ensure_ascii=False)

    formatted_body = {
        "timestamp": body_dict.get("TimeStamp", int(time.time() * 1000)),
        "source": "MSM",
        "replanLevel": body_dict.get("ReplanLevel"),
        "replanReason": reason_str
    }

    msg = _h_dict_to_ReplanRequest(formatted_body)
    manager.node_messenger.Push(msg)
    
    log_msg = f"### 재계획 요청 메시지 전송 (0902) ###: {formatted_body}"
    log_to_file(log_msg)
    print(log_msg)

# --- End of self-contained helpers ---

def judge_replan_situation(manager) -> list:
    """Manager로부터 데이터를 받아 재계획 상황을 판단합니다."""
    replan_situations = []
    
    msg_0202 = manager.receive_store.get_data("0202")
    msg_0402 = manager.receive_store.get_data("0402")
    msg_0801 = manager.receive_store.get_data("0801")
    msg_0802 = manager.receive_store.get_data("0802")

    log_msg = f"[judge_replan_situation] Received data -> 0202: {msg_0202 is not None}, 0801: {msg_0801 is not None}, 0802: {msg_0802 is not None}, 0402: {msg_0402 is not None}"
    log_to_file(log_msg)
    print(log_msg)

    operator_inputs = {"0202": msg_0202, "0801": msg_0801, "0802": msg_0802}
    for msg_id, reason_data in operator_inputs.items():
        if reason_data:
            current_ts = getattr(reason_data, 'timestamp', None)
            
            if current_ts is None:
                log_msg = f"[WARNING] Message {msg_id} has a null timestamp. Skipping."
                log_to_file(log_msg)
                print(log_msg)
                continue

            last_ts_key = f"replan_last_ts_{msg_id}"
            last_ts = manager.logic_store.get_data(last_ts_key)

            if current_ts != last_ts:
                log_msg = f"[judge_replan_situation] New event for {msg_id} detected. TS: {current_ts}"
                log_to_file(log_msg)
                print(log_msg)
                
                replan_reason = getattr(reason_data, "replan_reason", f"운용자 입력 ({msg_id})")
                replan_details = reason_data

                replan_info = {
                    "재계획 상황": "운용자 입력에 의한 재계획",
                    "재계획 근거": replan_reason,
                    "재계획 세부 근거": replan_details,
                    "original_message_id": msg_id
                }
                replan_situations.append(replan_info)
                manager.logic_store.set_data(last_ts_key, current_ts)

    if msg_0402:
        roi_info = getattr(msg_0402, "ROIInfo", None)
        if roi_info:
            current_ts = getattr(msg_0402, 'timestamp', None)
            if current_ts is None:
                log_msg = f"[WARNING] Message 0402 has a null timestamp. Skipping."
                log_to_file(log_msg)
                print(log_msg)
            else:
                last_ts_key = "replan_last_ts_0402"
                last_ts = manager.logic_store.get_data(last_ts_key)
                if current_ts != last_ts:
                    log_msg = f"[judge_replan_situation] New event for 0402 detected. TS: {current_ts}"
                    log_to_file(log_msg)
                    print(log_msg)
                    replan_info = {
                        "재계획 상황": "추적 재계획",
                        "재계획 유형": "임무 일시 제외 임무 재계획",
                        "재계획 근거": "신규 ROI 탐지",
                        "재계획 세부 근거": roi_info,
                        "original_message_id": "0402"
                    }
                    replan_situations.append(replan_info)
                    manager.logic_store.set_data(last_ts_key, current_ts)
    
    return replan_situations

def manage_replan_triggers(manager):
    situations = manager.logic_store.get_data('ReplanSituations')
    if not situations:
        return

    situations.sort(key=lambda s: 0 if '운용자' in s.get('재계획 상황', '') else 1)
    chosen_situation = situations[0]

    manager.logic_store.set_data('ConfirmedReplanRequest', chosen_situation)
    log_msg = f"### 재계획 트리거 확정 ###: {chosen_situation}"
    log_to_file(log_msg)
    print(log_msg)

    manager.logic_store.set_data('ReplanSituations', [])

def determine_level_and_send_request(manager):
    confirmed_request = manager.logic_store.get_data('ConfirmedReplanRequest')
    if not confirmed_request:
        return

    replan_level = 1 if confirmed_request['재계획 상황'] == '운용자 입력에 의한 재계획' else 2
    
    replan_request_message = {
        "TimeStamp": int(time.time()),
        "ReplanLevel": replan_level,
        "ReplanReason": confirmed_request
    }

    try:
        _send_0902_request(manager, replan_request_message)
    except Exception as e:
        log_msg = f"Error sending replan request message: {e}"
        log_to_file(log_msg)
        print(log_msg)
        import traceback
        traceback.print_exc()

    manager.logic_store.set_data('ConfirmedReplanRequest', None)

def run_replan_procedure(manager):
    """재계획 판단, 트리거 관리, 요청 전송까지의 전체 절차를 실행합니다."""
    log_msg = "--- [run_replan_procedure] START ---"
    log_to_file(log_msg)
    print(log_msg)
    
    judged_situations = judge_replan_situation(manager)
    if judged_situations:
        existing_situations = manager.logic_store.get_data('ReplanSituations') or []
        existing_situations.extend(judged_situations)
        manager.logic_store.set_data('ReplanSituations', existing_situations)
        log_msg = f"### 재계획 상황 판단 ###: {judged_situations}"
        log_to_file(log_msg)
        print(log_msg)

    manage_replan_triggers(manager)
    determine_level_and_send_request(manager)
    
    log_msg = "--- [run_replan_procedure] END ---"
    log_to_file(log_msg)
    print(log_msg)