# modules/common/Tabs/integration_tab.py
from __future__ import annotations
from typing import Optional
from PyQt5.QtWidgets import QWidget
from Tabs.csc_tab_base import CSCTabBase, _now_ms_since_2000

class IntegrationTab(CSCTabBase):
    TITLE = "Integration CSC"

    PUSH_MESSAGES = [
        ("0201", "협업기저임무 계획"),
        ("0202", "선행임무정보"),
        ("0203", "비행참조정보"),
        ("0401", "유무인기 상태정보"),
        ("0402", "전장상황인지정보"),
        ("0702", "의사결정 결과"),
        ("0802", "강제명령"),
        ("0803", "다음 협업기저임무 수행 명령"),
    ]

    RECEIVE_MESSAGES = [
        # (생략) 전체 수신 등록 그대로 둠
        ("0000","응답(Response)"),("0101","시스템 운용 모드"),("0102","모듈 상태 정보"),("0103","SW 상태정보"),
        ("0201","협업기저임무 계획"),("0202","선행임무정보"),("0203","비행참조정보"),
        ("0301","임무 계획"),("0302","개별 임무 계획"),("0303","무인기 비행 계획"),("0304","LAH 비행 계획"),
        ("0305","재계획 수행 상태 정보"),("0401","유무인기 상태정보"),("0402","전장상황인지정보"),
        ("0501","임무수행상태정보"),("0502","임무종료 요청"),("0503","협업기저임무 완료 알림"),
        ("0601","기저행위"),("0602","무인기 통제 명령"),
        ("0701","의사결정 옵션정보"),("0702","의사결정 결과"),
        ("0801","운용자 임무재계획 명령"),("0802","강제명령"),("0803","다음 협업기저임무 수행 명령"),
        ("0805","운용 이벤트"),("0806","시스템 부팅 명령"),
        ("0901","옵션 정보 생성 요청"),("0902","재계획 요청"),("0903","수행임무갱신요청"),("0904","행동트리 서비스 제공 요청"),
    ]

    def __init__(self, *, messenger, parent: Optional[QWidget] = None):
        super().__init__(messenger=messenger, parent=parent)

    def _build_overridden_body(self, msg_id: str):
        """
        ❌ 예전처럼 모든 메시지에 최소 바디(timestamp, source) 넣지 말 것.
        ✅ None을 반환하면 push_center가 제너레이터/DB 규칙으로 ‘풍부한 바디’를 구성한다.
        단, 0201/0203은 공용 push가 화이트리스트로 최소 필드만 보내도록 설계되어 있어
        아래처럼 제너레이터를 직접 호출해도 실제 전송 시 최소 필드로 축약된다.
        """
        mid = str(msg_id).zfill(4)

        if mid in ("0201", "0203"):
            # 제너레이터로 풍부한 바디를 만들어서 넘기되,
            # 공용 push의 TX_FIELD_WHITELIST 때문에 실제 전송은 최소 필드로 줄어듦(2단계에서 우회 가능)
            try:
                mod = __import__(f"modules.common.generator.message{mid}_generator",
                                 fromlist=[f"make_msg{mid}_body"])
                body = getattr(mod, f"make_msg{mid}_body")("INT")
                return body
            except Exception:
                return {"timestamp": _now_ms_since_2000(), "source": "INT"}

        # 그 외 메시지는 None → 공용 push가 generator.make_msgXXXX_body() 또는 make_random_and_push() 사용
        return None
