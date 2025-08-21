# c:\Users\HJW\Documents\Dev\MUMT\nFusion\mission_monitoring_replan\mission_monitoring_replan_csc_config.py
"""
Mission Monitoring & Replan CSC 탭에서 사용될 Push 및 Receive 메시지 목록을 정의합니다.
이 메시지들은 CSC 레벨에서 처리됩니다.
"""

# 기존 MissionMonitoringTab의 메시지 목록을 가져와서 필요에 따라 추가/수정
PUSH_MESSAGES = (
    ("0102", "운용모드 설정"),
    ("0501", "임무수행상태정보"),
    ("0502", "임무지표정보"),
    ("0503", "협업임무종료통보"),
    ("0902", "재계획 요청"),  # 재계획 관련 메시지 (CSC가 CSU2 결과를 받아 발신)
)

RECEIVE_MESSAGES = (
    ("0101", "시스템 운용모드"),
    ("0201", "협업기저임무 계획"),
    ("0202", "선행임무정보"),
    ("0203", "비행참조정보"),
    ("0401", "유무인기 상태정보"),
    ("0402", "전장상황인지정보"),
    ("0501", "임무수행상태정보"),
    ("0806", "SW종료 명령"),
    ("0902", "재계획 요청"),  # 재계획 트리거 (외부 -> CSC 수신)
)
