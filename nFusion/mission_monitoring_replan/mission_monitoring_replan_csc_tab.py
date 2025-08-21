# c:\Users\HJW\Documents\Dev\MUMT\nFusion\mission_monitoring_replan\mission_monitoring_replan_csc_tab.py
from PyQt5.QtWidgets import QPushButton, QWidget, QLabel, QVBoxLayout, QGroupBox
from PyQt5.QtCore import Qt
from datetime import datetime  # CSC 데이터 저장소에 타임스탬프 사용 예시

from csc_tab_base import CSCTabBase
from .mission_monitoring_replan_csc_config import PUSH_MESSAGES, RECEIVE_MESSAGES

# CSCManager를 임포트합니다.
from .mission_monitoring_replan_csc_manager import (
    MissionMonitoringReplanCSCManager,
    MockNodeMessenger,  # 테스트용
)


class MissionMonitoringReplanCSCTab(CSCTabBase):
    TITLE = "임무 모니터링·판단 CSC"
    PUSH_MESSAGES = PUSH_MESSAGES
    RECEIVE_MESSAGES = RECEIVE_MESSAGES

    def __init__(self, *, messenger, parent: QWidget | None = None):
        # PUSH_MESSAGES와 RECEIVE_MESSAGES는 mission_monitoring_replan_csc_config.py에서 직접 가져오므로
        # CSCTabBase 생성자 호출 전에 이미 클래스 변수로 설정되어 있습니다.
        # super().__init__()가 이 값들을 사용하여 테이블을 생성합니다.

        # CSCTabBase의 UI 요소들이 먼저 초기화되도록 super().__init__()를 먼저 호출합니다.
        # CSCTabBase는 messenger를 내부적으로 push_center와 함께 사용합니다.
        # CSCManager가 모든 통신을 담당하게 되면, CSCTabBase의 messenger 의존성을 줄이거나
        # CSCTabBase의 메시지 발신 로직도 Manager를 통하도록 변경하는 것을 고려할 수 있습니다.
        # 현재는 CSCTabBase의 더블클릭 발신 기능을 유지하기 위해 messenger를 전달합니다.
        super().__init__(messenger=messenger, parent=parent)

        # CSCManager 인스턴스 생성.
        # 실제 환경에서는 main_window.py에서 생성된 NodeMessenger 인스턴스를 messenger로 전달받아 사용합니다.
        # 여기서는 테스트를 위해 MockNodeMessenger를 사용하고 있습니다.
        # 실제 NodeMessenger를 사용하려면 아래 주석을 해제하고 mock_messenger_instance 부분을 실제 messenger로 변경하세요.
        # self.csc_manager = MissionMonitoringReplanCSCManager(node_messenger=messenger, log_callback=self._manager_log_handler)
        mock_messenger_instance = MockNodeMessenger(
            "MMR_CSC_NM_FOR_TAB"
        )  # Manager 전용 Mock 메신저
        self.csc_manager = MissionMonitoringReplanCSCManager(
            node_messenger=mock_messenger_instance,  # 또는 실제 messenger
            log_callback=self._handle_manager_log,
        )
        # 실제 nFusion 환경에서는 Manager가 메시지를 수신하도록 설정해야 합니다.
        # 예: messenger.set_receive_callback(self.csc_manager._handle_message_reception)
        # 또는 main_window.py에서 Consumer를 통해 Manager의 수신 핸들러를 호출하도록 설정

        self._add_custom_csc_ui()

    def _handle_manager_log(
        self, tag: str, log_type: str, message: str, raw_data: bytes | None
    ):
        """CSCManager로부터 로그를 받아 GUI 로그창에 기록하고, UI 상태를 업데이트합니다."""
        log_target = self.log_tx
        if log_type in [
            "RECV",
            "STORE",
            "MON_RES",
            "RPLN_RES",
            "SEND_FAIL",
        ]:  # 수신 관련 또는 결과 로그는 RX 창에
            log_target = self.log_rx

        self._write_log(log_target, f"{tag}-{log_type}", message, raw_data)

        # UI 테이블 상태 업데이트
        if log_type == "SEND_OK":
            # 메시지 ID를 message 문자열에서 파싱하거나, 콜백에서 직접 받아야 함
            # 여기서는 message에 "메시지 발신 완료: MSG_ID" 형태라고 가정
            try:
                msg_id_from_log = message.split("메시지 발신 완료: ")[1].split(",")[0]
                self._update_state(self.tbl_tx, msg_id_from_log, "발신 완료 (매니저)")
            except IndexError:
                self._write_log(
                    self.log_tx, "GUI_WARN", "SEND_OK 로그에서 MSG_ID 파싱 실패", None
                )
        elif log_type == "SEND_FAIL":
            try:
                msg_id_from_log = message.split("메시지 발신 실패: ")[1].split(",")[0]
                self._update_state(self.tbl_tx, msg_id_from_log, "발신 실패 (매니저)")
            except IndexError:
                self._write_log(
                    self.log_tx, "GUI_WARN", "SEND_FAIL 로그에서 MSG_ID 파싱 실패", None
                )
        elif log_type == "RECV":
            try:
                msg_id_from_log = message.split("메시지 수신: ")[1].split(",")[0]
                self._update_state(self.tbl_rx, msg_id_from_log, "수신 완료 (매니저)")
            except IndexError:
                self._write_log(
                    self.log_rx, "GUI_WARN", "RECV 로그에서 MSG_ID 파싱 실패", None
                )

    def _add_custom_csc_ui(self):
        """
        Mission Monitoring & Replan CSC 탭에 특정적인 UI 요소를 추가합니다.
        CSCTabBase의 기본 UI 위에 추가됩니다.
        """
        # CSU1 Controls (Monitoring)
        monitoring_csu_group = QGroupBox("모니터링 CSU")  # UI 텍스트 변경
        monitoring_csu_layout = QVBoxLayout()
        self.btn_trigger_monitoring_csu = QPushButton(
            "모니터링 로직 실행"
        )  # UI 텍스트 변경
        self.btn_trigger_monitoring_csu.clicked.connect(
            self.csc_manager.trigger_monitoring_csu_logic  # Manager의 메소드 직접 호출
        )  # 메소드명 변경
        monitoring_csu_layout.addWidget(self.btn_trigger_monitoring_csu)
        monitoring_csu_group.setLayout(monitoring_csu_layout)

        # CSU2 Controls (Replan)
        replan_csu_group = QGroupBox("재계획 CSU")  # UI 텍스트 변경
        replan_csu_layout = QVBoxLayout()
        self.btn_trigger_replan_csu = QPushButton("재계획 로직 실행")  # UI 텍스트 변경
        self.btn_trigger_replan_csu.clicked.connect(
            self.csc_manager.trigger_replan_csu_logic  # Manager의 메소드 직접 호출
        )  # 메소드명 변경
        replan_csu_layout.addWidget(self.btn_trigger_replan_csu)
        replan_csu_group.setLayout(replan_csu_layout)

        # Add CSU groups to the main layout (e.g., below the TX log)
        # CSCTabBase의 레이아웃 구조를 활용하여 위젯을 추가합니다.
        # self.layout()은 CSCTabBase의 root QVBoxLayout 입니다.
        # itemAt(1)은 body QHBoxLayout, itemAt(0)은 left QWidget (발신 영역)
        if self.layout() and self.layout().count() > 1:
            body_layout = (
                self.layout().itemAt(1).layout()
            )  # Main QHBoxLayout (left_side, right_side)
            if body_layout and body_layout.count() > 0:
                left_side_widget = body_layout.itemAt(
                    0
                ).widget()  # Left QWidget (TX area)
                if left_side_widget and left_side_widget.layout():
                    left_v_layout = (
                        left_side_widget.layout()
                    )  # QVBoxLayout of the TX side
                    left_v_layout.addWidget(monitoring_csu_group)
                    left_v_layout.addWidget(replan_csu_group)

    def mark_received(self, msg_id: str, raw: bytes | None = None):
        """
        CSCTabBase의 이 메서드는 nFusion Consumer에 의해 직접 호출될 수 있습니다.
        CSCManager가 모든 통신을 담당하는 경우, 이 메서드는 Manager에게 수신 정보를 전달하거나,
        Manager가 직접 Consumer 역할을 하도록 설정합니다.

        현재 구조에서는 Manager가 자체 NodeMessenger(Mock 또는 실제)를 가지고 있고,
        GUI는 Manager의 로그 콜백을 통해 수신 상태를 간접적으로 알 수 있습니다.
        따라서 이 GUI의 mark_received는 CSCTabBase의 기본 동작을 수행하되,
        Manager를 통한 수신 처리와 중복될 수 있음을 인지해야 합니다.

        만약 main_window.py의 Consumer가 이 GUI 탭의 mark_received를 호출하고,
        이 정보를 Manager에게 전달하고 싶다면 다음과 같이 할 수 있습니다:
        if hasattr(self, 'csc_manager') and self.csc_manager:
             self.csc_manager._handle_message_reception(msg_id, raw if raw else b'') # Manager에게 전달
        """
        super().mark_received(msg_id, raw)  # 기본 수신 처리 (UI 업데이트, 로그)
        self._write_log(
            self.log_rx,
            "GUI_RECV",  # 태그를 변경하여 Manager의 RECV 로그와 구분
            f"GUI Tab이 직접 {msg_id} 수신 (CSCTabBase)",
            raw,
        )
