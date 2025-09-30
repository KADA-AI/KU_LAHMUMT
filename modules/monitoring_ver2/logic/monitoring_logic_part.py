# logic/monitoring_logic_part.py: '모니터링' 도메인에 대한 세부 비즈니스 로직을 구현합니다.

from datetime import datetime, timezone

from typing import Union

# --- 데이터 모델 import ---
from data.message_models import (
    ModuleStatusModelModel,
    MissionProgressBodyModel,
    MissionEndRequestBodyModel,
    ReplanRequestBodyModel,
)
from modules.common.push_center import push_message
from .monitoring_actual_logic import run_monitoring_procedure
import udp_reporter


# --- 반환 가능한 모든 Push 메시지 본문 타입을 정의 ---
PushBodyType = Union[
    ModuleStatusModelModel,
    MissionProgressBodyModel,
    MissionEndRequestBodyModel,
    ReplanRequestBodyModel,
]


class MonitoringLogic:
    def __init__(self, manager):
        self.manager = manager

    def execute(self, mode_override=None):
        """시스템 모드를 확인하고, 'monitoring'일 경우에만 로직을 실행합니다."""
        system_mode = (
            mode_override
            if mode_override is not None
            else self.manager.logic_store.get_data("SystemMode")
        )

        if system_mode == 3:
            self.manager._log("MON_LOGIC", "EXEC", "모니터링 로직 실행됨.")
            # 401 데이터 가져오기
            data_401 = self.manager.receive_store.get_data("0401")
            if data_401:
                self.manager._log(
                    "MON_LOGIC", "INFO", "401 데이터 확인. 모니터링 절차 실행."
                )
                # 모니터링 절차 실행하여 0501 메시지 본문 생성
                body_0501 = run_monitoring_procedure(data_401)

                # test feul Logic
                feul_data = []
                for agent_state in data_401.agentStateList:
                    if agent_state.isUnmanned == 1:
                        text = ""
                        print(f"feul : {agent_state.fuel}")
                        if agent_state.fuel * 100 // 100 <= 20:
                            text = "yellow"
                        elif agent_state.fuel * 100 // 100 <= 10:
                            text = "red"
                        else:
                            text = "green"

                        feul_data.append(
                            {"id": agent_state.aircraftID, "warning": text}
                        )

                if body_0501:
                    # 0501 메시지 발신
                    # print(f"body_0501: {body_0501}")
                    push_message(
                        "0501", self.manager.node_messenger, body_dict=body_0501
                    )
                    self.manager._log(
                        "MON_LOGIC", "INFO", "0501 메시지를 발신했습니다."
                    )
                    # PushStorage에 저장
                    self.manager.push_store.add_data("0501", body_0501)
                    # LogicStorage에도 저장
                    self.manager.logic_store.set_data("0501_data", body_0501)
                    # UDP 통지 추가
                    udp_reporter.notify_tx("0501")

                    # GUI 업데이트 콜백 호출 (0501은 로직에서 생성된 데이터이므로 'logic' 타입으로 전달)
                    if self.manager.gui_update_callback:
                        self.manager.gui_update_callback("logic", "0501", body_0501)

                # fuel_data를 LogicStorage에 저장하고 GUI 업데이트
                if feul_data:  # feul_data가 비어있지 않은 경우에만 처리
                    self.manager.logic_store.set_data("fuel_data", feul_data)
                    if self.manager.gui_update_callback:
                        self.manager.gui_update_callback(
                            "logic", "fuel_data", feul_data
                        )
            else:
                self.manager._log(
                    "MON_LOGIC", "INFO", "401 데이터가 없어 모니터링을 건너뜁니다."
                )

    def generate_body_for(self, msg_id: str) -> PushBodyType:
        """메시지 ID에 따라 데이터 클래스 인스턴스를 생성하여 반환합니다."""
        timestamp = int(
            (
                datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)
            ).total_seconds()
            * 1000
        )
        source_module = "MonitoringModule"

        if msg_id == "0102":
            return ModuleStatusModelModel(
                timestamp=timestamp, source=source_module, status=1
            )
        elif msg_id == "0501":
            return MissionProgressBodyModel(
                timestamp=timestamp, source=source_module
            )
        elif msg_id == "0502":
            return MissionEndRequestBodyModel(
                timestamp=timestamp, source=source_module, reason=0
            )
        elif msg_id == "0902":
            return ReplanRequestBodyModel(
                timestamp=timestamp,
                source=source_module,
                replanRequest="ManualTrigger",
            )

        raise ValueError(f"Body generation not implemented for msg_id: {msg_id}")
