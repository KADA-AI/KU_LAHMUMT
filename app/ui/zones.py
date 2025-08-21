# -*- coding: utf-8 -*-

GRID_ROWS = 35
GRID_COLS = 50

def Z(r1, c1, r2, c2):
    """1-based → dict(0-based, span)"""
    return {
        "r0": r1 - 1,
        "c0": c1 - 1,
        "rs": (r2 - r1 + 1),
        "cs": (c2 - c1 + 1),
    }

ZONES = {
    # (1) 상단 타이틀: 카드 없이 큰/굵은 라벨만
    "TITLE": Z(2, 2, 3, 25),

    # (2) 경로 지정 버튼(= '찾아보기' 버튼)
    "ROUTE_BUTTON": Z(3, 47, 3, 48),

    # (3) DB 경로 텍스트 창(버튼 없음)
    "DB_PATH": Z(3, 28, 3, 45),

    # (4~6 + 7~9 통합) 모듈+로그 결합 영역
    "MODULE_MISSION_COMBO":   Z(5, 14, 27, 24),   # 임무 할당 및 계획 + 로그
    "MODULE_MONITOR_COMBO":   Z(5, 26, 27, 36),   # 모니터링 및 판단 + 로그
    "MODULE_DECISION_COMBO":  Z(5, 38, 27, 48),   # 의사결정 지원 + 로그

    # (10) 데이터 흐름 시각화 (제목 없음)
    "FLOW_VIS": Z(5, 6, 27, 12),

    # (11) 모드 할당 버튼부 (제목 없음)
    "MODE_BUTTONS": Z(5, 2, 27, 4),

    # (12) 운용흐름 모니터링 (제목 없음)
    "OPS_FLOW": Z(29, 2, 33, 48),

    # (13) 하단 검정 바(흰 글자) — 빈 곳 없이 전체 폭
    "FOOTER": Z(35, 1, 35, 50),
}
