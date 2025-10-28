# udp_reporter.py
import socket
import json
import time
import os

# --- Constants ---
PORT = int(os.getenv("KU_MON_MONITORING_PORT", "46982"))
ADDR = ("127.0.0.1", PORT)
ROLE = "MSM" # Mission State Monitor

# --- Internal Send Function ---
def _send(obj: dict):
    """Constructs the final JSON and sends it via UDP socket."""
    try:
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(data, ADDR)
    except Exception as e:
        print(f"[ERROR][UDP_REPORTER] Failed to send UDP notification: {e}")

# --- Public Notification Functions ---

def notify_mode(mode_text: str):
    """Notifies a change in the system operation mode."""
    payload = {
        "kind": "mode",
        "text": mode_text,
        "role": ROLE,
        "ts": int(time.time() * 1000)
    }
    _send(payload)

def notify_tx(msg_id: str):
    """Notifies that a specific message ID has been transmitted (pushed)."""
    payload = {
        "kind": "tx",
        "msg_id": msg_id,
        "role": ROLE,
        "ts": int(time.time() * 1000)
    }
    _send(payload)
