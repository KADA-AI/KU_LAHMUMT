# 파일: /mnt/data/decision_support_tab.py
# -*- coding: utf-8 -*-
from Tabs.csc_tab_base import CSCTabBase
import time

_EPOCH2000_MS = 946684800000
def _now_ms_since_2000():
    return int(time.time() * 1000) - _EPOCH2000_MS

class DecisionSupportTab(CSCTabBase):
    TITLE = "의사결정 지원 CSC"
    def __init__(self, *, messenger, parent=None, owner=None):
        super().__init__(messenger=messenger, parent=parent)
        self._owner = owner

    
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
            stored_entries = getattr(self, "_last_option_entries", None) or []
            patterns = [(0, 0, 0), (-1, -1, 1), (1, 1, -1)]
            option_list = []
            for idx, entry in enumerate(stored_entries):
                try:
                    plan_id = int(entry.get("missionPlanID"))
                except Exception:
                    continue
                try:
                    option_id = int(entry.get("optionID", idx + 1))
                except Exception:
                    option_id = idx + 1
                sr, tc, reff = patterns[idx] if idx < len(patterns) else (0, 0, 0)
                option_list.append({
                    "optionID": option_id,
                    "missionPlanID": plan_id,
                    "survivalRate": sr,
                    "timeContraction": tc,
                    "recogEffectiveness": reff,
                    "distance": 0,
                    "target": 0,
                })

            if not option_list:
                fallback_plan = int(getattr(self, "_last_mission_plan_id", 0) or 0)
                option_list.append({
                    "optionID": 1,
                    "missionPlanID": fallback_plan,
                    "survivalRate": 0,
                    "timeContraction": 0,
                    "recogEffectiveness": 0,
                    "distance": 0,
                    "target": 0,
                })

            return {
                "timestamp": ts,
                "source": "MOB",
                "autoExecution": False,
                "optionList": option_list,
            }

        return None  # 그 외는 기본(제네레이터) 사용

    def mark_received(self, msg_id: str, raw: bytes | None = None):
        super().mark_received(msg_id, raw)
        if str(msg_id).zfill(4) == "0901":
            owner = getattr(self, '_owner', None)
            if owner is not None:
                try:
                    owner.mark_received('0901', raw)
                except Exception as exc:
                    try:
                        owner._append_log_line(f"[ERR] 0901 처리 실패: {exc}")
                    except Exception:
                        pass
