# c:\Users\HJW\Documents\Dev\MUMT\KU_LAHMUMT\TODO\monitoring_ver2\logic\replan_actual_logic.py

def judge_replan_situation(manager) -> list:
    """
    Manager로부터 데이터를 받아 재계획 상황을 판단합니다.
    `get_data`는 데이터를 store에서 가져온 후 비운다고 가정합니다.

    :param manager: receive_store를 포함하는 manager 객체
    :return: 재계획 상황 정보가 담긴 딕셔너리의 리스트
    """
    replan_situations = []

    # 1. Manager의 receive_store에서 실제 수신 데이터 가져오기
    msg_0202 = manager.receive_store.get_data("0202")
    msg_0402 = manager.receive_store.get_data("0402")
    msg_0801 = manager.receive_store.get_data("0801")
    msg_0802 = manager.receive_store.get_data("0802")

    # 2. 운용자 입력에 의한 재계획 판단 (0202, 0801, 0802)
    operator_inputs = {
        "0202": msg_0202,
        "0801": msg_0801,
        "0802": msg_0802
    }

    for msg_id, reason_data in operator_inputs.items():
        if reason_data:
            replan_reason = reason_data.get("replan_reason", f"운용자 입력 ({msg_id})")
            replan_details = reason_data.get("replan_details", reason_data)

            replan_info = {
                "재계획 상황": "운용자 입력에 의한 재계획",
                "재계획 근거": replan_reason,
                "재계획 세부 근거": replan_details,
                "original_message_id": msg_id
            }
            replan_situations.append(replan_info)

    # 3. 추적 재계획 상황 판단 (0402)
    if msg_0402 and msg_0402.get("ROIInfo"):
        replan_info = {
            "재계획 상황": "추적 재계획",
            "재계획 근거": "신규 ROI 탐지",
            "재계획 세부 근거": msg_0402["ROIInfo"],
            "original_message_id": "0402"
        }
        replan_situations.append(replan_info)
        # 사용한 ROIInfo 데이터 비우기 (원본 데이터가 변경됨)
        msg_0402["ROIInfo"] = []
    
    return replan_situations

def run_replan_procedure(manager):
    """
    재계획 판단 절차를 실행하고, 결과를 manager에 저장합니다.
    """
    # 1. 재계획 상황 판단
    situations = judge_replan_situation(manager)

    # 2. 상황 발생 시 데이터 저장 (또는 다른 처리)
    if situations:
        # logic_store에 'ReplanSituations'라는 키로 저장한다고 가정
        # (실제 저장 방식은 manager의 구현에 따라 달라질 수 있음)
        existing_situations = manager.logic_store.get_data('ReplanSituations') or []
        existing_situations.extend(situations)
        manager.logic_store.set_data('ReplanSituations', existing_situations)
        
        # 콘솔에 로그 출력
        print(f"### 재계획 상황 발생 ###: {situations}")