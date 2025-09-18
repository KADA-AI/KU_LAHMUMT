# 파일: /modules/common/states/S110_initial_prep.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from . import register_state
try:
    # S110 체크리스트가 있다면 함께 띄움(없어도 동작)
    from ..ops_checklist import ensure_s110_checklist
except Exception:
    ensure_s110_checklist = lambda _orch: None

@register_state("S110")
def run(orch):
    orch._safe_log("[OPS] S110: 초기임무계획 진입 요청")

    # 1) 체크리스트 시작(있으면)
    try:
        ensure_s110_checklist(orch)
    except Exception:
        pass

    # 2) UI/모듈 모드 즉시 싱크(다른 프로그램 켜져 있어도 무관)
    #    → 각 모듈에 CTRL 'mode=초기임무계획' 브로드캐스트
    try:
        orch._enter_initial_plan()
    except Exception:
        pass

    # 3) Info 모듈에 실제 0101(SystemMode=2) 생성 지시
    #    (아래 패치된 info_manage.py가 이를 받아 0101을 버스로 push)
    ok = False
    try:
        ok = orch._send_ctrl_single("info", {"cmd": "system_mode", "mode": 2, "reason": "S110"})
    except Exception as e:
        orch._safe_log(f"[WARN] Info system_mode 지시 실패: {e}")
    if not ok:
        orch._safe_log("[WARN] Info 모듈 0101 전송 지시 실패")
    # 4) Kick off the initial mission-planning pipeline
    try:
        orch.trigger_initial_plan_pipeline(reason="초기임무재계획")
    except AttributeError:
        orch._safe_log('[WARN] trigger_initial_plan_pipeline not available on orchestrator')
    except Exception as exc:
        orch._safe_log(f'[WARN] initial mission planning pipeline failed: {exc}')

