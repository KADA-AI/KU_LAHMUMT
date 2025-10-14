class StatisticalReplan:
    """
    통계적 기반 재계획 판단 모듈
    """

    def find_high_risk_values(self, data):
        high_risk_values = {key: value for key, value in data.items() if value > 0.3}
        return high_risk_values

    def analyze_statistics(self, risk_data: dict):
        """
        DBNN의 위험도 예측 결과를 분석하여 재계획 트리거를 생성합니다.
        :param risk_data: 단일 예측 결과에 대한 위험도 딕셔너리.
        """
        # risk_data가 딕셔너리이므로 [0] 인덱싱을 제거합니다.
        schedule_adherence_risk = risk_data.get("Schedule_Adherence_Risk", 0)
        collision_risk = risk_data.get("Collision_Risk", 0)
        enemy_risk = risk_data.get("Enemy_Risk", 0)
        # sustainability_risk, operational_risk 등은 현재 로직에서 사용되지 않아 주석 처리 가능

        triggers = []
        # 참고: 위험도(risk) 값이 백분율(0-100)이라고 가정하고 임계값을 50으로 설정했습니다.
        # 실제 값의 범위에 맞게 임계값 조정이 필요합니다.

        # 중복 키 버그를 수정하고 로직을 명확하게 변경합니다.
        if collision_risk > 50:
            triggers.append(
                {
                    "MissionPlanningStatus": "개별 임무 재할당",
                    "ReplanReason": f"충돌 예측률 {collision_risk}%",
                }
            )
        if enemy_risk > 50:
            triggers.append(
                {
                    "MissionPlanningStatus": "개별 임무 재할당 및 비행 경로 재계획",
                    "ReplanReason": f"무인기 격추 확률 {enemy_risk}%",
                }
            )
        if schedule_adherence_risk > 50:
            triggers.append(
                {
                    "MissionPlanningStatus": " 스케줄링",
                    "ReplanReason": f"임무 수행 불균형 발생 확률 {schedule_adherence_risk}%",
                }
            )

        return triggers
