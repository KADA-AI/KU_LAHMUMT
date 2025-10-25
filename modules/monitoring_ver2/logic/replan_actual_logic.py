import time

def judge_replan_situation(manager) -> list:
    """
    Manager로부터 데이터를 받아 재계획 상황을 판단합니다.
    `get_data`는 데이터를 store에서 가져온 후 비운다고 가정합니다.
    """
    replan_situations = []
    # ... (이전 단계에서 작성한 내용과 동일) ...
    msg_0202 = manager.receive_store.get_data("0202")
    msg_0402 = manager.receive_store.get_data("0402")
    msg_0801 = manager.receive_store.get_data("0801")
    msg_0802 = manager.receive_store.get_data("0802")

    operator_inputs = {
        "0202": msg_0202, "0801": msg_0801, "0802": msg_0802
    }

    for msg_id, reason_data in operator_inputs.items():
        if reason_data:
            replan_info = {
                "재계획 상황": "운용자 입력에 의한 재계획",
                "재계획 근거": reason_data.get("replan_reason", f"운용자 입력 ({msg_id})"),
                "재계획 세부 근거": reason_data.get("replan_details", reason_data),
                "original_message_id": msg_id
            }
            replan_situations.append(replan_info)

    if msg_0402 and msg_0402.get("ROIInfo"):
        replan_info = {
            "재계획 상황": "추적 재계획",
            "재계획 유형": "임무 일시 제외 임무 재계획",
            "재계획 근거": "신규 ROI 탐지",
            "재계획 세부 근거": msg_0402["ROIInfo"],
            "original_message_id": "0402"
        }
        replan_situations.append(replan_info)
        msg_0402["ROIInfo"] = []
    
    return replan_situations

def manage_replan_triggers(manager):
    """
    재계획 상황을 관리하고, 우선순위에 따라 실제 재계획을 트리거할지 결정합니다.
    (쿨다운 로직 제외됨)
    """
    # 1. 저장된 재계획 상황 목록 가져오기
    situations = manager.logic_store.get_data('ReplanSituations')
    if not situations:
        return

    # 2. 우선순위 기반으로 가장 중요한 상황 하나를 선택 (운용자 입력 > 추적 재계획)
    situations.sort(key=lambda s: 0 if '운용자' in s.get('재계획 상황', '') else 1)
    chosen_situation = situations[0]

    # 3. 최종 재계획 요청 확정 및 저장
    manager.logic_store.set_data('ConfirmedReplanRequest', chosen_situation)
    print(f"### 재계획 트리거 확정 ###: {chosen_situation}")

    # 4. 처리된 상황 목록 비우기
    manager.logic_store.set_data('ReplanSituations', [])

def determine_level_and_send_request(manager):
    """
    확정된 재계획 요청을 바탕으로 재계획 수준을 결정하고,
    실제 재계획 수행 모듈에 요청 메시지를 전송합니다.
    """
    confirmed_request = manager.logic_store.get_data('ConfirmedReplanRequest')
    if not confirmed_request:
        return

    # 재계획 수준 결정 (1: 전체 재계획, 2: 부분 재계획)
    replan_level = 1 if confirmed_request['재계획 상황'] == '운용자 입력에 의한 재계획' else 2
    
    # 재계획 요청 메시지 생성 (0902 메시지)
    replan_request_message = {
        "MsgID": "0902",
        "TimeStamp": int(time.time()),
        "ReplanLevel": replan_level,
        "ReplanReason": confirmed_request
    }

    # 메시지 전송 (manager의 push_center를 사용한다고 가정)
    try:
        # manager.push_center.push_message("0902", replan_request_message)
        print(f"### 재계획 요청 메시지 전송 (0902) ###: {replan_request_message}")
    except Exception as e:
        print(f"Error sending replan request message: {e}")

    # 처리된 요청 비우기
    manager.logic_store.set_data('ConfirmedReplanRequest', None)

def run_replan_procedure(manager):
    """
    재계획 판단, 트리거 관리, 요청 전송까지의 전체 절차를 실행합니다.
    """
    # 1. 재계획 상황 판단
    judged_situations = judge_replan_situation(manager)
    if judged_situations:
        existing_situations = manager.logic_store.get_data('ReplanSituations') or []
        existing_situations.extend(judged_situations)
        manager.logic_store.set_data('ReplanSituations', existing_situations)
        print(f"### 재계획 상황 판단 ###: {judged_situations}")

    # 2. 재계획 트리거 관리
    manage_replan_triggers(manager)

    # 3. 재계획 수준 결정 및 요청 전송
    determine_level_and_send_request(manager)