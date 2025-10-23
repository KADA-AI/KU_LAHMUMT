# 파일: modules/common/status_reporter.py
# -*- coding: utf-8 -*-
"""
각 GUI에서 '정상 기동' 시 0102 상태 메시지(status=1) 전송 헬퍼.
대문자 키( Timestamp / Status / Source )로 고정.
"""
from __future__ import annotations
import sys, time

def send_status_ok(source_module_name: str) -> bool:
    """
    0102 (SelfCheck / ModuleStatus) 메시지 전송: Status=1(정상)
    source_module_name 예: "MOB"(의사결정), "MSM"(모니터링), "MMR"(임무계획), "INF"(정보관리)
    """
    try:
        from dll_files.nFusionImports import NodeMessenger  # type: ignore
        from push_center import push_message               # type: ignore
    except Exception as e:
        try: sys.stderr.write(f"[WARN] send_status_ok: nFusion not available: {e}\n")
        except Exception: pass
        return False

    body = {
        "timestamp": int(time.time() * 1000),     # ← 대문자 키로 통일
        "status": 1,                              # 0: Unknown, 1: 정상, 2: 비정상
        "source": str(source_module_name or "UNKNOWN"),
    }
    try:
        ok = push_message("0102", NodeMessenger, body_dict=body)
        if not ok:
            sys.stderr.write("[WARN] send_status_ok: push_message returned False\n")
        return bool(ok)
    except Exception as e:
        try: sys.stderr.write(f"[ERR] send_status_ok failed: {e}\n")
        except Exception: pass
        return False
