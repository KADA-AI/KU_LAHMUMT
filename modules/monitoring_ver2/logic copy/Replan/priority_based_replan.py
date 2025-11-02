class PriorityBasedReplan:
    """
    우선순위 기반 재계획 모듈
    """

    PRIORITY_MAPPING = {
        "운용자 입력": 1,
        "무인기 선행 임무로 전환": 2,
        "무인기 소실": 3,
        "충돌 예측": 4,
        "임무 수행 불균형": 5,
    }

    def determine_priority(self, rule_trigger, stats_trigger):
        print("Priority-based replanning")

        # 트리거 병합 및 필터링 (딕셔너리만 남김)
        all_triggers = [
            trigger
            for trigger in (rule_trigger + stats_trigger)
            if isinstance(trigger, dict)
        ]

        # 우선순위 확인
        if all_triggers:
            prioritized_task = min(
                all_triggers,
                key=lambda x: self.PRIORITY_MAPPING.get(
                    x.get("ReplanReason", "Unknown"), float("inf")
                ),
            )
        else:
            prioritized_task = None

        return prioritized_task
