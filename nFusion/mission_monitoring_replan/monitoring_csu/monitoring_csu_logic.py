# c:\Users\HJW\Documents\Dev\MUMT\nFusion\mission_monitoring_replan\monitoring_csu\monitoring_csu_logic.py
"""
모니터링 CSU 로직: 모니터링 및 초기 판단
CSC로부터 전달받은 데이터를 기반으로 모니터링 관련 로직을 수행합니다.
UI나 NodeMessenger에 직접 의존하지 않습니다.
"""

import time  # 예시용


def _perform_monitoring_analysis(data: dict) -> dict:
    """
    모니터링 데이터 분석 로직 예시
    """
    print(f"[MonitoringCSULogic] 모니터링 분석 시작 (데이터 키: {list(data.keys())})")
    # 실제 분석 로직 구현
    time.sleep(0.1)  # 작업 시뮬레이션
    status_info_data = (
        data.get("status_info") if data.get("status_info") is not None else {}
    )

    analysis_result = {
        "analysis_status": "completed",
        "anomalies_detected": status_info_data.get("anomaly", False),
        "summary": "모니터링 분석 결과 요약",
    }
    print(f"[MonitoringCSULogic] 모니터링 분석 완료: {analysis_result}")
    return analysis_result


class MonitoringCSUHandler:
    def __init__(self):
        self._monitoring_state = {}  # 모니터링 CSU 내부 상태
        print("[MonitoringCSUHandler] 모니터링 CSU 로직 핸들러 초기화됨.")

    def run_monitoring(self, csc_provided_data: dict) -> dict:
        """
        CSC로부터 데이터를 받아 모니터링 로직을 실행합니다.
        """
        self._monitoring_state["last_input"] = csc_provided_data
        return _perform_monitoring_analysis(csc_provided_data)
