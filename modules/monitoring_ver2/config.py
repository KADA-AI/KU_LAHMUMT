"""Configuration for the Monitoring CSC (ver2)."""

POWER_OFF_MODE = 5
INITIAL_MODE = 0

SYSTEM_MODE_LABELS = {
    POWER_OFF_MODE: "전원 OFF 모드",
    INITIAL_MODE: "초기화 모드",
    1: "대기 모드",
    2: "초기 임무 재계획 모드",
    3: "임무 수행 모드",
}

SYSTEM_MODE_ORDER = [POWER_OFF_MODE, INITIAL_MODE, 1, 2, 3]
SYSTEM_MODE_OPTIONS = [(code, SYSTEM_MODE_LABELS[code]) for code in SYSTEM_MODE_ORDER]

PUSH_MESSAGES = (
    ("0102", "모듈 상태 정보"),
    ("0501", "임무수행상태정보"),
    ("0502", "임무종료 요청"),
    ("0503", "협업기저임무 완료 알림"),
    ("0504", "연료량 경고"),
    ("0902", "재계획 요청"),  # Moved to RECEIVE_MESSAGES
)

RECEIVE_MESSAGES = (
    ("0101", "시스템 운용 모드"),
    ("0201", "협업기저임무 계획"),
    ("0202", "선행임무정보"),
    ("0203", "비행참조정보"),
    ("0301", "임무 계획"),
    ("0302", "개별 임무 계획"),
    ("0303", "무인기 비행 계획"),
    ("0304", "LAH 비행 계획"),
    ("0401", "유무인기 상태정보"),
    ("0402", "전장상황인지정보"),
    ("0504", "연료량 경고"),
    ("0601", "기저행위"),
    ("0702", "의사결정 결과"),
    ("0801", "운용자 임무재계획 명령"),
    ("0802", "강제명령"),
    ("0803", "다음 협업기저임무 수행 명령"),
    ("0805", "운용 이벤트"),
    ("0806", "시스템 부팅 명령"),
    # ("0902", "재계획 요청"),  # Moved from PUSH_MESSAGES
    ("0903", "수행임무갱신요청"),
)

