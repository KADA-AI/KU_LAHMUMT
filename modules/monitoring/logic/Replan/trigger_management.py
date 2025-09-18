from datetime import datetime, timezone


class TriggerManagement:
    """
    재계획 트리거 관리 모듈
    """

    def __init__(self):
        self.is_replanning = False
        self.prev_trigger = None

    def manage_triggers(self, trigger):

        if trigger is None:
            return
        # 현재 시간 기록
        timestamp = datetime.now().isoformat()

        # 입력된 트리거와 이전 트리거 비교
        if trigger != self.prev_trigger:
            if self.prev_trigger is not None:
                # 기존 임무 계획 취소
                print("Cancelling previous mission planning")

            # 새로운 임무 계획 호출
            print("Calling new mission planning")
            self.is_replanning = True
            self.prev_trigger = trigger

        # 임무 계획 완료 후 상태 초기화
        self.is_replanning = False
        self.prev_trigger = None

        result = {
            "Timestamp": timestamp,
            "MissionPlanningStatus": trigger.get("MissionPlanningStatus", "Unknown"),
            "ReplanReason": trigger.get("ReplanReason", "No reason provided"),
        }

        print(result)

        # 반환 값 구성
        return result
