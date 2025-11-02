# gui/monitoring_gui.py: 애플리케이션의 메인 윈도우(QMainWindow)를 생성하고, 여러 탭들을 관리합니다.

# -*- coding: utf-8 -*-
# MonitoringTab.py

import json
import sys
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from PyQt5.QtCore import pyqtSignal, Qt, pyqtSlot
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QTabWidget,
    QTextEdit,
    QDockWidget,
)

# os.environ["KU_ROLE"] = "monitoring"

# 분리된 탭들을 임포트
from modules.monitoring_ver2.gui.tabs.MonitoringTab import MonitoringTab
from modules.monitoring_ver2.gui.tabs.ReplanTab import ReplanTab
from modules.monitoring_ver2.gui.tabs.MonitoringCSCTab import MonitoringCSCTab
from modules.monitoring_ver2.gui.tabs.SystemModeControlTab import SystemModeControlTab
from modules.monitoring_ver2.data.message_models import (
    InputMissionIDModel,
    OptionListModel,
    ReplanRequestBodyModel,
    ReplanRequestTimeStampModel,
)
from modules.common import db_paths


def _now_ms_since_2000() -> int:
    """2000년 1월 1일(UTC)을 기준으로 한 경과 밀리초를 반환한다."""
    epoch_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
    return int((datetime.now(timezone.utc) - epoch_2000).total_seconds() * 1000)


# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    ctrl_payload = pyqtSignal(dict)
    log_received = pyqtSignal(str)  # 스레드 안전 로깅을 위한 시그널
    update_gui_signal = pyqtSignal(str, str, object)  # GUI 업데이트를 위한 새로운 시그널

    def __init__(self, manager):
        super().__init__()
        self.setWindowTitle("임무 모니터링·판단 (MSM)")
        self.resize(1100, 700)
        self.manager = manager

        # 탭 위젯 생성 및 설정
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # 각 탭 인스턴스 생성
        self.monitoring_tab = MonitoringTab(manager=self.manager)
        self.replan_tab = ReplanTab(manager=self.manager)
        self.csc_tab = MonitoringCSCTab(manager=self.manager)
        self._auto_initplan_triggered = False

        # 탭 위젯에 탭 추가
        self.tabs.addTab(self.csc_tab, "모니터링 CSC")
        self.tabs.addTab(self.monitoring_tab, "모니터링 요약")
        self.tabs.addTab(self.replan_tab, "재계획 판단")

        # --- 로그 창 추가 ---
        self.log_dock = QDockWidget("로그", self)
        self.log_widget = QTextEdit()
        self.log_widget.setReadOnly(True)
        self.log_dock.setWidget(self.log_widget)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_dock)

        # 업데이트 유형에 따라 처리할 탭들을 리스트로 매핑한다.
        self.update_handlers = {
            "receive": [self.monitoring_tab, self.replan_tab, self.csc_tab],
            "logic": [self.monitoring_tab, self.replan_tab, self.csc_tab],
            "send": [self.csc_tab],
        }

        # 시그널-슬롯 연결
        self.log_received.connect(self._append_log_to_widget)
        self.update_gui_signal.connect(self._perform_gui_update)

    @pyqtSlot(str)
    def _append_log_to_widget(self, message: str) -> None:
        """GUI 스레드에서 로그 위젯에 메시지를 추가한다."""
        self.log_widget.append(message)

    def add_log_message(
        self, tag: str, log_type: str, message: str, raw_data: bytes | None
    ) -> None:
        """다른 스레드에서 호출 가능한 메서드. 시그널을 발생시켜 GUI 스레드에서 처리되도록 한다."""
        log_entry = f"[{tag}] [{log_type}] {message}"
        self.log_received.emit(log_entry)

    def _append_log_line(self, message: str) -> None:
        """로그 독에 직접 한 줄을 추가한다."""
        self.log_received.emit(str(message))

    @pyqtSlot(str, str, object)
    def _perform_gui_update(
        self, update_type: str, key: str, data_object: object = None
    ) -> None:
        """GUI 업데이트를 메인 스레드에서 수행하는 슬롯."""
        update_info = (update_type, key)
        handlers = self.update_handlers.get(update_type, [])
        for handler_tab in handlers:
            if hasattr(handler_tab, "refresh_display"):
                handler_tab.refresh_display(update_info, data_object)

    def update_view(self, update_type: str, key: str, data_object: object = None) -> None:
        """Manager가 데이터 변경을 알리기 위해 호출하는 콜백 메소드. 시그널을 통해 GUI 스레드로 전달한다."""
        if (update_type, key) in {("logic", "SystemMode"), ("receive", "0101")}:
            self._handle_system_mode_update(data_object)
        self.update_gui_signal.emit(update_type, key, data_object)

    # ------------------------------------------------------------------ 자동 0902 지원
    def _handle_system_mode_update(self, payload: object | None) -> None:
        """시스템 모드 변화를 감지해 자동 0902 컨텍스트를 준비한다."""
        if payload is not None and hasattr(payload, "systemMode"):
            mode_value = getattr(payload, "systemMode", None)
        else:
            mode_value = self.manager.get_logic_result("SystemMode")
        try:
            mode_int = int(mode_value)
        except (TypeError, ValueError):
            return

        if mode_int == 2:
            if not self._auto_initplan_triggered:
                self._auto_initplan_triggered = True
                self._auto_prepare_replan()
        else:
            self._auto_initplan_triggered = False

    def _auto_prepare_replan(self) -> None:
        """초기 임무 계획 모드 진입 시 0902 요청을 생성·전송한다."""
        try:
            body = self._build_0902_body()
        except Exception as exc:
            self._append_log_line(f"[AUTO] 0902 컨텍스트 생성 실패: {exc}")
            return

        context = self._make_replan_context(body)
        self._stage_replan_context(context, trigger="auto", body=body)

        if self._dispatch_replan_request(body, context):
            self._append_log_line("[AUTO] 0902 재계획 요청 자동 송신 실행")
        else:
            self._append_log_line("[AUTO] 0902 자동 송신 실패: 전송 경로를 찾지 못함")

    def _dispatch_replan_request(
        self, body: ReplanRequestBodyModel | Dict[str, Any], context: Dict[str, Any] | None = None
    ) -> bool:
        """CSC 탭에 준비된 0902 요청을 위임해 송신한다."""
        tab = getattr(self, "csc_tab", None)
        if tab and hasattr(tab, "dispatch_replan_request"):
            try:
                return bool(tab.dispatch_replan_request(body, context=context))
            except Exception as exc:
                self._append_log_line(f"[AUTO] 0902 재계획 요청 전송 실패: {exc}")
                return False
        return False

    def _collect_input_mission_ids(self) -> List[int]:
        """database/InputMissionPlan 경로에서 inputMissionID를 모두 수집한다."""
        ids: List[int] = []
        try:
            base = db_paths.get_db_subpath("InputMissionPlan")
            candidates: List[Path] = []
            if base.exists() and base.is_dir():
                candidates.extend(p for p in base.glob("*.json") if p.is_file())
            single = db_paths.get_db_subpath("InputMissionPlan.json")
            if single.exists():
                candidates.append(single)

            for fp in candidates:
                try:
                    data = json.loads(fp.read_text(encoding="utf-8"))
                except Exception:
                    continue
                for item in data.get("inputMissionList") or []:
                    try:
                        ids.append(int(item.get("inputMissionID")))
                    except Exception:
                        continue
        except Exception:
            ids = []
        unique_ids = sorted({value for value in ids if value is not None})
        if not unique_ids:
            unique_ids = [107, 108]
        return unique_ids

    def _next_mission_plan_ids(self, count: int) -> List[int]:
        """mission_plan_seq.txt를 이용해 연속된 missionPlanID를 생성한다."""
        seq_file = db_paths.get_db_subpath("mission_plan_seq.txt")
        start = 700000001
        try:
            if seq_file.exists():
                raw = seq_file.read_text(encoding="utf-8").strip()
                if raw:
                    start = max(start, int(raw))
        except Exception:
            start = 700000001

        out = list(range(start, start + int(count)))
        try:
            seq_file.parent.mkdir(parents=True, exist_ok=True)
            seq_file.write_text(str(start + int(count)), encoding="utf-8")
        except Exception:
            pass
        return out

    def _build_0902_body(self) -> ReplanRequestBodyModel:
        """Build a 0902 replan request payload following the agreed defaults."""
        now = _now_ms_since_2000()
        input_ids = self._collect_input_mission_ids()

        reason = "초기임무재계획"
        option_specs = [
            {"optionID": 1, "optionName": "시스템추천"},
            {"optionID": 2, "optionName": "임무시간최소화"},
            {"optionID": 3, "optionName": "촬영효과최대"},
        ]
        # 초기 임무 재계획 시에는 추천 옵션 하나만 사용한다.
        if reason == "초기임무재계획":
            option_specs = option_specs[:1]

        mission_plan_ids = self._next_mission_plan_ids(len(option_specs))

        body = ReplanRequestBodyModel(
            source="MMR",
            timestamp=now,
            replanRequestTime=ReplanRequestTimeStampModel(replanRequestTimestamp=now),
            replanLevel=1,
            inputMissionIDList=[
                InputMissionIDModel(inputMissionID=int(mid)) for mid in input_ids
            ],
            IndividualMissionIDList=[],
            priorMissionList=[],
            replanRequest=reason,
            optionList=[
                OptionListModel(
                    optionID=spec["optionID"],
                    optionName=spec["optionName"],
                    missionPlanID=mpid,
                )
                for spec, mpid in zip(option_specs, mission_plan_ids)
            ],
        )

        if mission_plan_ids:
            self._append_log_line(
                f"[0902] 재계획 요청 생성 완료 (inputMissionIDs={len(input_ids)}, optionCount={len(option_specs)}, mpid@{mission_plan_ids[0]})"
            )
        else:
            self._append_log_line(
                f"[0902] 재계획 요청 생성 완료 (inputMissionIDs={len(input_ids)}, optionCount=0)"
            )
        return body

    def _make_replan_context(
        self, body: ReplanRequestBodyModel | Dict[str, Any]
    ) -> Dict[str, Any]:
        """Create a concise context dict for GUI display and reuse."""
        if is_dataclass(body):
            payload = asdict(body)
        elif isinstance(body, dict):
            payload = dict(body)
        else:
            return {}

        option_key = "optionList" if "optionList" in payload else "pendingOptionList"
        options = payload.get(option_key) or []
        mission_list = payload.get("inputMissionIDList") or []

        def _extract(values: Iterable[Dict[str, Any]], key: str) -> List[int]:
            out: List[int] = []
            for item in values:
                if not isinstance(item, dict):
                    continue
                value = item.get(key)
                try:
                    out.append(int(value))
                except Exception:
                    continue
            return out

        mission_ids = _extract(mission_list, "inputMissionID")
        plan_ids = _extract(options, "missionPlanID")
        option_names = [
            item.get("optionName")
            for item in options
            if isinstance(item, dict) and item.get("optionName")
        ]
        if not option_names and options:
            option_names = [f"옵션{i + 1}" for i in range(len(options))]

        return {
            "plan_ids": plan_ids,
            "mission_ids": mission_ids,
            "option_names": option_names,
            "replan_level": payload.get("replanLevel", 1),
            "reason": payload.get("replanReason") or payload.get("replanRequest") or "",
        }

    def _stage_replan_context(
        self,
        context: Dict[str, Any],
        *,
        trigger: str,
        body: ReplanRequestBodyModel | Dict[str, Any] | None = None,
    ) -> None:
        """Hand the prepared context to the CSC tab and record helper logs."""
        if context is None:
            return

        if self.csc_tab and hasattr(self.csc_tab, "set_replan_context"):
            try:
                self.csc_tab.set_replan_context(context, body=body)
            except Exception:
                pass

        summary = ", ".join(str(pid) for pid in context.get("plan_ids", []) if pid) or "-"
        prefix = "[AUTO]" if trigger == "auto" else "[CTRL]"
        self._append_log_line(f"{prefix} 0902 재계획 요청 준비 완료 (planIds: {summary})")
        if trigger != "auto":
            self._append_log_line("[GUIDE] 모니터링 탭에서 0902 버튼을 눌러 재계획 요청을 송신하세요.")
