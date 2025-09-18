# logic/replan_actual_logic.py: 실제 재계획 판단 로직을 수행하는 함수를 정의합니다.

from logic.Replan.replan_management import ReplanManager
import udp_reporter
from data.message_models import ReplanRequestBodyModel
from datetime import datetime, timezone


def run_replan_procedure(manager):
    """
    실제 재계획 판단 로직의 시작점입니다.
    ReplanManager를 사용하여 재계획 프로세스를 관리합니다.
    """
    manager._log("REPLAN_PROCEDURE", "INFO", "실제 재계획 판단 로직 실행 시작.")

    # 1. Manager의 receive_store에서 실제 수신 데이터 가져오기
    agent_state = manager.receive_store.get_data("0401")
    situationAwarenessInfo = manager.receive_store.get_data("0402")
    mandatory_command = manager.receive_store.get_data("0802")
    prior_mission_info = manager.receive_store.get_data("0202")

    # 2. 필수 데이터 존재 여부 확인
    if not agent_state:
        manager._log(
            "REPLAN_PROCEDURE",
            "INFO",
            "필수 데이터(0401)가 없어 재계획 판단을 건너뜁니다.",
        )
        return

    # ReplanManager 인스턴스 생성
    replan_manager = ReplanManager()

    # 3. 실제 데이터를 인자로 전달하여 재계획 프로세스 실행
    final_replan_output, trigger = replan_manager.manage_replan(
        agent_state=agent_state,
        mandatory_command=mandatory_command,
        prior_mission_info=prior_mission_info,
    )

    # 4. 최종 결과를 logic_store에 저장
    manager.logic_store.set_data(
        "final_replan_output",
        final_replan_output,
    )
    # 5. 트리거 결과를 logic_store에 저장 (GUI 표시용)
    manager.logic_store.set_data(
        "replan_triggers",
        trigger,
    )

    # 0902 ReplanRequest 메시지 본문 생성
    ## 0918 적 발견으로 인한 유인기 경로 재계획 명령 확인용 더미 데이터
    final_replan_output = "적 탐지로 인한 유인기 공격 판단 필요"

    timestamp = int(
        (
            datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)
        ).total_seconds()
        * 1000
    )
    replan_body = ReplanRequestBodyModel(
        timestamp=timestamp,
        sourceModuleName="MonitoringModule",
        replanLevel=3,  # 유인기 공격 모델 호출 / 경로 및 촬영 재계획
        replanRequest=final_replan_output,  # final_replan_output을 replanRequest 필드에 사용
    )

    # PushStorage에 저장
    manager.push_store.add_data("0902", replan_body)

    # 재계획 결과가 저장되었음을 UDP로 통지
    udp_reporter.notify_tx("0902")

    manager._log("REPLAN_PROCEDURE", "INFO", f"최종 재계획 결과: {final_replan_output}")
    manager._log("REPLAN_PROCEDURE", "INFO", "실제 재계획 판단 로직 실행 종료.")
