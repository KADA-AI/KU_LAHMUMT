# /modules/common/button_wiring.py
# -*- coding: utf-8 -*-
from __future__ import annotations

def _find_button(win, keywords):
    try:
        from PyQt5.QtWidgets import QPushButton
        for btn in win.findChildren(QPushButton):
            text = (btn.text() or "").lower()
            objn = (btn.objectName() or "").lower()
            if any(k in text for k in keywords) or any(k in objn for k in keywords):
                return btn
    except Exception:
        pass
    return None

def wire_dashboard_buttons(orch):
    """
    대시보드 상단 버튼을 '상태 코드'로만 연결.
    실제 데이터 송/수신은 각 모듈 버튼/로직에서 수행.
    """
    win = getattr(orch, "win", None)

    def _safe_dispatch(code: str):
        try:
            if hasattr(orch, "_handle_operation_state"):
                orch._handle_operation_state(code)
            else:
                if hasattr(orch, "_safe_log"):
                    orch._safe_log(f"[OPS] 상태 진입 실패(핸들러 없음): {code}")
        except Exception as e:
            try:
                orch._safe_log(f"[ERR] 상태 호출 실패({code}): {e}")
            except Exception:
                try: print(f"[ERR] 상태 호출 실패({code}): {e}")
                except Exception: pass

    # 상단/공용 버튼 → 상태 코드
    mapping = [
        (("sw 실행", "전체 실행", "all run", "launch all", "run all", "전체 구동"), "S101"),
        (("초기임무계획모드", "초기임무계획", "initial", "init plan"), "S110"),
        (("임무수행모드", "임무 수행", "운용모드", "mission mode", "operation mode"), "S300"),
    ]
    for keywords, code in mapping:
        btn = _find_button(win, tuple(k.lower() for k in keywords))
        if btn is not None:
            try: btn.clicked.disconnect()
            except Exception: pass
            btn.clicked.connect(lambda _=False, c=code: _safe_dispatch(c))

    # 각 카드의 'GUI 실행' 버튼
    module_script = {
        "assignment": "mission_planning_gui.py",
        "monitoring": "monitoring_gui.py",
        "decision":   "decision_support_gui.py",
    }
    for key, script in module_script.items():
        card = getattr(orch, "widgets", {}).get(key)
        if not card:
            continue
        btn = getattr(card, "btn_run", None)
        if btn and hasattr(btn, "clicked"):
            try: btn.clicked.disconnect()
            except Exception: pass
            btn.clicked.connect(lambda _=False, sn=script: orch._launch_gui(sn))
