# modules/common/status_reporter.py
from __future__ import annotations

import sys
import time

EPOCH_2000_MS = 946684800000


def now_ms_since_2000() -> int:
    return int(time.time() * 1000) - EPOCH_2000_MS


def send_status_ok(source_module_name: str) -> bool:
    """Send 0102 ModuleStatus with Status=1 using the ICD 2000-epoch timestamp."""
    try:
        from dll_files.nFusionImports import NodeMessenger  # type: ignore
        from push_center import push_message  # type: ignore
    except Exception as exc:
        try:
            sys.stderr.write(f"[WARN] send_status_ok: nFusion not available: {exc}\n")
        except Exception:
            pass
        return False

    body = {
        "timestamp": now_ms_since_2000(),
        "status": 1,
        "source": str(source_module_name or "UNKNOWN"),
    }
    try:
        ok = push_message("0102", NodeMessenger, body_dict=body)
        if not ok:
            sys.stderr.write("[WARN] send_status_ok: push_message returned False\n")
        return bool(ok)
    except Exception as exc:
        try:
            sys.stderr.write(f"[ERR] send_status_ok failed: {exc}\n")
        except Exception:
            pass
        return False
