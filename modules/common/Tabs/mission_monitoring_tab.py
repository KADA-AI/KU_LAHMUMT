# -*- coding: utf-8 -*-  # /mnt/data/mission_monitoring_tab.py
from typing import Any, Dict, List, Optional
import json, re

from modules.common.option_codes import (
    DEFAULT_OPTION_CODE_SEQUENCE,
    ensure_option_code_sequence,
)

from .csc_tab_base import CSCTabBase, _now_ms_since_2000


class MissionMonitoringTab(CSCTabBase):
    TITLE = "임무 모니터링 CSC (MSM)"
    HISTORY_LIMIT = 10
    LOG_ENTRY_LIMIT = 10
    LOG_PAYLOAD_MAX_CHARS = 2000
    LOG_FLUSH_INTERVAL_MS = 200
    PUSH_MESSAGES = (
        ("0001", "공지"),
        ("0102", "모듈 상태 정보"),
        ("0501", "임무수행상태정보"),
        ("0502", "임무종료 요청"),
        ("0503", "협업기저임무 완료 알림"),
        ("0504", "연료량 경고"),
        ("0902", "재계획 요청"),
    )

    RECEIVE_MESSAGES = (
        ("0001", "공지"),
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
        ("0903", "수행임무갱신요청"),
    )

    # ─────────────────────────────────────────────────────────────
    # 0101 모드 정규화 도우미
    _MODE_NAME_MAP = {
        "S100": "초기화",
        "S101": "일괄기동",
        "S102": "자가점검ON",
        "S110": "초기준비",
        "S200": "대기모드",
        "S300": "임무모드",
    }
    _MODE_NUM_TO_S = {
        100: "S100",
        101: "S101",
        102: "S102",
        110: "S110",
        200: "S200",
        300: "S300",
    }
    _MODE_CODE_NAME_MAP = {
        0: "초기화 모드",
        1: "대기 모드",
        2: "초기 임무 계획",
        3: "임무 수행 모드",
    }
    _MODE_KEY_CANDIDATES = (
        "systemMode", "SystemMode", "system_mode",
        "systemOperationMode", "SystemOperationMode",
        "operationMode", "OperationMode",
        "opMode", "OpMode",
        "mode", "Mode",
        "modeCode", "ModeCode",
        "modeID", "ModeID",
        "state", "State",
    )
    _SUBKEYS = ("mode","Mode","code","Code","value","Value","id","Id","modeCode","ModeCode")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._current_mode_s: Optional[str] = None  # e.g., "S200"
        self._replan_context: Optional[Dict[str, Any]] = None

    # ── 0101 수신 가로채기 → 모드 반영 ─────────────────────────────
    def mark_received(self, msg_id: str, raw: bytes | None = None):
        super().mark_received(msg_id, raw)
        if str(msg_id).strip() != "0101" or not raw:
            return

        body = self._extract_json(raw)
        mode_s, mode_name = self._extract_mode(body)
        if not mode_s:
            # 못 알아들었으면 로그만 남김
            self._append_mode_log(f"수신은 했으나 모드 파싱 실패: {body!r}")
            return

        self._apply_system_mode(mode_s, mode_name, body)

    # ── 내부: JSON 추출(원문에서 첫 중괄호 블록) ─────────────────────
    def _extract_json(self, raw: bytes) -> Dict[str, Any]:
        try:
            txt = raw.decode(errors="ignore")
            m = re.search(r"\{.*\}", txt, flags=re.S)
            if m:
                return json.loads(m.group(0))
            return json.loads(txt)
        except Exception:
            return {}

    # ── 내부: 모드 코드/이름 정규화 (S100/S200 등으로 통일) ───────────
    def _extract_mode(self, obj: Dict[str, Any]) -> (Optional[str], str):
        val = None
        # 1) 후보 키에서 값 찾기
        for k in self._MODE_KEY_CANDIDATES:
            if k in obj:
                val = obj[k]
                break
        # 2) 중첩 dict면 하위 키에서 다시 추출
        if isinstance(val, dict):
            for sk in self._SUBKEYS:
                if sk in val:
                    val = val[sk]
                    break

        # 3) 문자열이면 바로 처리
        if isinstance(val, str):
            s_raw = val.strip()
            if s_raw.isdigit():
                code = int(s_raw)
                name = self._MODE_CODE_NAME_MAP.get(code, "")
                return s_raw, name
            s = s_raw.upper()
            # "S200", "STANDBY", "대기" 등 처리
            if s.startswith("S") and s[1:].isdigit():
                mode_s = "S" + str(int(s[1:]))  # 정규화
                name = self._MODE_NAME_MAP.get(mode_s or "", "")
                return mode_s, name
            # 한글/영문 명칭에서 추정
            if "대기" in s or "STANDBY" in s:
                mode_s = "S200"
            elif "임무" in s or "MISSION" in s:
                mode_s = "S300"
            elif "초기준비" in s or "INIT" in s and "PREP" in s:
                mode_s = "S110"
            elif "자가점검" in s or "SELF" in s:
                mode_s = "S102"
            elif "일괄" in s or "RUN_ALL" in s or "RUNALL" in s:
                mode_s = "S101"
            elif "초기" in s or s == "INIT":
                mode_s = "S100"
            else:
                mode_s = None
            name = self._MODE_NAME_MAP.get(mode_s or "", "")
            return mode_s, name

        # 4) 숫자면 매핑
        if isinstance(val, int):
            mode_s = self._MODE_NUM_TO_S.get(val)
            if mode_s is not None:
                name = self._MODE_NAME_MAP.get(mode_s or "", "")
                return mode_s, name
            name = self._MODE_CODE_NAME_MAP.get(val, "")
            return str(val), name

        # 5) 못 찾음
        return None, ""

    # ── 내부: 모드 적용(타이틀/로그/자동동작) ───────────────────────
    def _apply_system_mode(self, mode_s: str, mode_name: str, payload: Dict[str, Any]):
        self._current_mode_s = mode_s
        tag = f"{mode_s} {mode_name}".strip()

        # 제목은 고정 표기
        try:
            if hasattr(self, "_title_label") and self._title_label:
                self._title_label.setText(self.TITLE)
        except Exception:
            pass

        # 로그 한 줄 남김
        self._append_mode_log(f"→ 시스템 운용 모드 변경: {tag}")

        # MonitoringCSCTab에서는 manager에 모드 반영 (상단 슬라이더 동기화용)
        manager = getattr(self, "manager", None)
        if manager is not None and hasattr(manager, "set_system_mode"):
            mode_code = None
            if mode_s and mode_s.isdigit():
                mode_code = int(mode_s)
            elif mode_s.startswith("S") and mode_s[1:].isdigit():
                s_code = int(mode_s[1:])
                s_to_code = {100: 0, 101: 0, 102: 0, 110: 2, 200: 1, 300: 3}
                mode_code = s_to_code.get(s_code)
            if mode_code is not None:
                try:
                    manager.set_system_mode(int(mode_code))
                except Exception:
                    pass

        # (선택) 모드에 따라 0102 주기 송신 자동 토글 예시
        try:
            row_0102 = self._find_tx_row("0102")
            if row_0102 >= 0:
                freq = self.periodic_config.get("0102")
                mode_num = int(mode_s) if mode_s and mode_s.isdigit() else None
                start_modes = {"S102", "S200", "S300"}
                stop_modes = {"S100"}
                if mode_num is not None:
                    if mode_num in (1, 2, 3):
                        start_modes.add(str(mode_num))
                    if mode_num == 0:
                        stop_modes.add(str(mode_num))
                if mode_s in start_modes and freq:
                    if "0102" not in self.periodic_timers:
                        self._start_periodic_send("0102", row_0102, freq)
                if mode_s in stop_modes and "0102" in self.periodic_timers:
                    self._stop_periodic_send("0102", row_0102)
        except Exception:
            pass

    def _append_mode_log(self, msg: str):
        try:
            self.append_log(f"[MODE] {msg}")
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────
    # 이하 기존 구현 (재계획 요청 관련) ------------------------------
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
        ctx_data = getattr(self, "_replan_context", None)
        if str(msg_id).strip() == "0902" and isinstance(ctx_data, dict):
            ctx = dict(ctx_data)
            ts = _now_ms_since_2000()
            mission_ids = [
                {"inputMissionID": int(mid)}
                for mid in ctx.get("mission_ids", [])
                if mid is not None
            ]
            plan_ids: List[int] = [int(pid) for pid in ctx.get("plan_ids", [])]
            raw_option_values: List[Any] = list(ctx.get("option_names", []))
            target_count = max(len(plan_ids), len(raw_option_values), 1) if plan_ids else len(raw_option_values) or 1
            option_codes = ensure_option_code_sequence(raw_option_values, target_count)
            while len(option_codes) < len(plan_ids):
                fallback_idx = len(option_codes)
                fallback_code = (
                    DEFAULT_OPTION_CODE_SEQUENCE[fallback_idx]
                    if fallback_idx < len(DEFAULT_OPTION_CODE_SEQUENCE)
                    else DEFAULT_OPTION_CODE_SEQUENCE[-1]
                )
                option_codes.append(fallback_code)
            options = []
            for idx, pid in enumerate(plan_ids, start=1):
                code = (
                    option_codes[idx - 1]
                    if idx - 1 < len(option_codes)
                    else DEFAULT_OPTION_CODE_SEQUENCE[min(idx - 1, len(DEFAULT_OPTION_CODE_SEQUENCE) - 1)]
                )
                options.append(
                    {
                        "optionID": idx,
                        "optionName": code,
                        "missionPlanID": pid,
                    }
                )
            if not options and ctx.get("fallback_plan_id") is not None:
                pid = int(ctx["fallback_plan_id"])
                options.append(
                    {
                        "optionID": 1,
                        "optionName": DEFAULT_OPTION_CODE_SEQUENCE[0],
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
                "replanRequest": reason,
                "optionList": options,
            }
        return super()._build_overridden_body(msg_id)
