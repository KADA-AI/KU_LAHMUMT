# 파일: /mnt/data/decision_support_tab.py
# -*- coding: utf-8 -*-
from Tabs.csc_tab_base import CSCTabBase
import time

_EPOCH2000_MS = 946684800000
def _now_ms_since_2000():
    return int(time.time() * 1000) - _EPOCH2000_MS

class DecisionSupportTab(CSCTabBase):
    TITLE = "의사결정 지원 CSC"
    
    # **FB → Push**
    PUSH_MESSAGES = [
        ("0102", "모듈 상태 정보"),
        ("0701", "의사결정 옵션정보"),
    ]
    
    # **BF → Receive**
    RECEIVE_MESSAGES = [
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
        ("0501", "임무수행상태정보"),
        ("0701", "의사결정 옵션정보"),
        ("0806", "시스템 부팅 명령"),
        ("0901", "옵션 정보 생성 요청"),
    ]

    # ★ 클릭 경로에서 0701 바디를 주입 (0102는 기존 동작 유지)
    def _build_overridden_body(self, msg_id: str):
        mid = str(msg_id).strip()
        if mid == "0102":
            return {}  # 기본 생성 규칙 사용
        if mid == "0701":
            ts = _now_ms_since_2000()
            mpid = int(getattr(self, "_last_mission_plan_id", 0) or 0)
            # 필요 시 optionName 등 필드 스펙 맞춰 조정
            return {
                "timestamp": ts,
                "source": "MOB",
                "autoExecution": False,
                "optionList": [
                    {
                        "optionID": 1,
                        "optionName": 1,
                        "missionPlanID": mpid,
                        "survivalRate": 1,
                        "timeContraction": -1,
                        "recogEffectiveness": 1,
                        "distance": 30000,
                        "target": 0,
                    }
                ],
            }
        return None  # 그 외는 기본(제네레이터) 사용
