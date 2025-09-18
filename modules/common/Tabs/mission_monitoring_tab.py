# -*- coding: utf-8 -*-
from typing import Any, Dict, List, Optional

from Tabs.csc_tab_base import CSCTabBase, _now_ms_since_2000


class MissionMonitoringTab(CSCTabBase):
    TITLE = "정보 관리 CSC"
    PUSH_MESSAGES = (
        ("0102", "모듈 상태 정보"),
        ("0501", "임무 수행상황"),
        ("0502", "임무종료 요청"),
        ("0503", "협업기초임무 종료 알림"),
        ("0902", "재계획 요청"),
    )

    RECEIVE_MESSAGES = (
        ("0101", "시스템운용 모드"),
        ("0201", "입력임무 계획"),
        ("0202", "비행임무정보"),
        ("0203", "비행참조정보"),
        ("0301", "임무 계획"),
        ("0302", "개별 임무 계획"),
        ("0303", "무인기 비행 계획"),
        ("0304", "LAH 비행 계획"),
        ("0401", "임무대기 상태정보"),
        ("0402", "현장상황정보"),
        ("0601", "기동위치"),
        ("0702", "의사결정 결과"),
        ("0801", "자율임무계획명령"),
        ("0802", "강제명령"),
        ("0803", "처음 협업기초임무 수행 명령"),
        ("0805", "자율 이벤트"),
        ("0806", "시스템부분명령"),
        ("0903", "비행임무갱신요청"),
    )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._replan_context: Optional[Dict[str, Any]] = None

    def set_replan_context(self, context: Optional[Dict[str, Any]]) -> None:
        self._replan_context = context

    def _find_tx_row(self, msg_id: str) -> int:
        if not hasattr(self, "tbl_tx"):
            return -1
        for row in range(self.tbl_tx.rowCount()):
            item = self.tbl_tx.item(row, 0)
            if item and item.text().strip() == str(msg_id):
                return row
        return -1

    def send_replan_request(self, context: Dict[str, Any], reason: str) -> bool:
        if not context:
            return False
        context = dict(context)
        context["reason"] = reason
        self.set_replan_context(context)
        row = self._find_tx_row("0902")
        if row < 0:
            return False
        try:
            self.tbl_tx.selectRow(row)
        except Exception:
            pass
        try:
            self._on_tx_double_clicked(row, 0)
            return True
        finally:
            self.set_replan_context(None)

    def _build_overridden_body(self, msg_id: str):
        if str(msg_id).strip() == "0902" and isinstance(self._replan_context, dict):
            ctx = dict(self._replan_context)
            ts = _now_ms_since_2000()
            mission_ids = [
                {"inputMissionID": int(mid)}
                for mid in ctx.get("mission_ids", [])
                if mid is not None
            ]
            plan_ids: List[int] = [int(pid) for pid in ctx.get("plan_ids", [])]
            option_names: List[str] = list(ctx.get("option_names", []))
            while len(option_names) < len(plan_ids):
                option_names.append(f"옵션{len(option_names) + 1}")
            options = []
            for idx, pid in enumerate(plan_ids, start=1):
                name = option_names[idx - 1] if idx - 1 < len(option_names) else f"옵션{idx}"
                options.append(
                    {
                        "optionID": idx,
                        "optionName": name,
                        "missionPlanID": pid,
                    }
                )
            if not options and ctx.get("fallback_plan_id") is not None:
                pid = int(ctx["fallback_plan_id"])
                options.append(
                    {
                        "optionID": 1,
                        "optionName": "시스템추천",
                        "missionPlanID": pid,
                    }
                )
            reason = str(ctx.get("reason") or "초기임무재계획")
            return {
                "timestamp": ts,
                "source": "MSM",
                "replanRequestTime": {"replanRequestTimestamp": ts},
                "replanLevel": int(ctx.get("replan_level", 1)),
                "inputMissionIDList": mission_ids,
                "replanReason": reason,
                "pendingOptionList": options,
            }
        return super()._build_overridden_body(msg_id)
