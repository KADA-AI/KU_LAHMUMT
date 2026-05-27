# modules/common/ctrl_listener.py
# -*- coding: utf-8 -*-
"""
각 GUI에서 로컬 CTRL(UDP) 수신을 간단히 후킹하기 위한 리스너.
run.py가 보내는 {"cmd":"self_check","status":1} 등을 받아 콜백으로 전달.
"""
from __future__ import annotations

import os, json, socket, threading

def start_ctrl_listener(port: int, on_payload):
    """
    port: KU_CTRL_PORT (env) 또는 모듈별 기본 포트
    on_payload: def on_payload(dict)
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", int(port)))

    def loop():
        while True:
            try:
                data, _ = sock.recvfrom(8192)
                try:
                    payload = json.loads(data.decode("utf-8", "ignore"))
                except Exception:
                    continue
                try:
                    on_payload(payload)
                except Exception:
                    pass
            except Exception:
                pass

    th = threading.Thread(target=loop, name=f"CTRL@{port}", daemon=True)
    th.start()
    return th

def env_ctrl_port(default_port: int) -> int:
    try:
        v = int(os.environ.get("KU_CTRL_PORT", "").strip() or "0")
        return v if v > 0 else default_port
    except Exception:
        return default_port
