# /modules/common/states/S110_initial_prep.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from . import register_state

@register_state("S110")
def run(orch):
    """
    새 S110 플로우:
      1) Info 모듈이 0101(SystemMode=2) 전송 → 각 모듈 초기임무계획 모드로.
      ※ run.py는 '요청'만, 실제 0101 전송은 Info 모듈이 수행.
    """
    orch._safe_log("[OPS] S110: Info 모듈에 SystemMode=2 전송 요청")
    # CTRL 채널로 Info 모듈에 system_mode 지시 (Info가 0101을 '전송'함)
    ok = orch._send_ctrl_single("info", {"cmd": "system_mode", "mode": 2, "reason": "S110"})
    if not ok:
        orch._safe_log("[WARN] Info 모듈에 SystemMode=2 지시 실패")
    # 이후 흐름:
    # - run.py는 버스의 0101 수신을 모니터링해 _enter_initial_plan() (모드 전파/표시) 수행
    # - 0902/0305/0301/0901/0701/0702 등은 각 모듈 버튼/로직이 전담 (run.py 미전송)
