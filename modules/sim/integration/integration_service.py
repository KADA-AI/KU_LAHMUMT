from __future__ import annotations

import json
import os
import re
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

from modules.common.fusion_files import copy_file_with_retry
from modules.common.push_center import push_message
from modules.common.receive_center import register_listener
from modules.sim.config import SIM_0401_IDLE_HZ


_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)


def _now_ms_2000() -> int:
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)


def _safe_json(obj: Any) -> Any:
    if obj is None:
        return None
    if isinstance(obj, (dict, list)):
        return obj
    if isinstance(obj, (bytes, bytearray)):
        text = None
        try:
            text = bytes(obj).decode("utf-8", "ignore")
            try:
                return json.loads(text)
            except Exception:
                m = re.search(r"\{.*\}", text, flags=re.S)
                if m:
                    try:
                        return json.loads(m.group(0))
                    except Exception:
                        return text
            return text
        except Exception:
            return text or ""
    if isinstance(obj, str):
        try:
            return json.loads(obj)
        except Exception:
            m = re.search(r"\{.*\}", obj, flags=re.S)
            if m:
                try:
                    return json.loads(m.group(0))
                except Exception:
                    return obj
            return obj
    return obj


def _fmt_ts(ts: float) -> str:
    try:
        base = datetime.fromtimestamp(ts)
        centis = int((float(ts) * 100.0) % 100.0)
        return f"{base.strftime('%H:%M:%S')}:{centis:02d}"
    except Exception:
        return "--:--:--:--"


def _fmt_since(seconds: float) -> str:
    try:
        seconds = max(0.0, float(seconds))
    except Exception:
        return "--"
    if seconds < 1:
        return "0s"
    if seconds < 60:
        return f"{int(seconds)}s"
    mins = int(seconds // 60)
    return f"{mins}m"


def _load_message_ids() -> tuple[list[str], list[str]]:
    default_tx = ["0001", "0201", "0202", "0203", "0401", "0702", "0802", "0803"]
    default_rx = [
        "0000",
        "0001",
        "0101",
        "0102",
        "0103",
        "0104",
        "0201",
        "0202",
        "0203",
        "0301",
        "0302",
        "0303",
        "0304",
        "0305",
        "0401",
        "0402",
        "0501",
        "0502",
        "0503",
        "0504",
        "0601",
        "0602",
        "0701",
        "0702",
        "0801",
        "0802",
        "0803",
        "0804",
        "0805",
        "0806",
        "0901",
        "0902",
        "0903",
        "0904",
    ]
    try:
        tab_path = Path(__file__).resolve().parents[3] / "modules" / "common" / "Tabs" / "integration_tab.py"
        if not tab_path.exists():
            return default_tx, default_rx
        text = tab_path.read_text(encoding="utf-8", errors="ignore").splitlines()
        tx: list[str] = []
        rx: list[str] = []
        mode = None
        for line in text:
            if "PUSH_MESSAGES" in line:
                mode = "tx"
                continue
            if "RECEIVE_MESSAGES" in line:
                mode = "rx"
                continue
            if mode and "]" in line:
                mode = None
                continue
            if not mode:
                continue
            for match in re.findall(r"['\"](\d{4})['\"]", line):
                if mode == "tx":
                    tx.append(match)
                else:
                    rx.append(match)
        return (tx or default_tx), (rx or default_rx)
    except Exception:
        return default_tx, default_rx


def _resolve_message_name(msg_id: str) -> str:
    msg_id = str(msg_id).zfill(4)
    try:
        mod = __import__(f"modules.common.generator.message{msg_id}_generator", fromlist=[f"make_msg{msg_id}_body"])
        fn = getattr(mod, f"make_msg{msg_id}_body", None)
        doc = getattr(fn, "__doc__", "") or ""
        m = re.search(r"RootType:\s*([A-Za-z0-9_]+)", doc)
        if m:
            return m.group(1)
    except Exception:
        pass
    return f"Message {msg_id}"


class _PeriodicSender:
    def __init__(self, interval_sec: float, send_func):
        self.interval = max(1.0 / 60.0, float(interval_sec))
        self._send = send_func
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        if not self._thread.is_alive():
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        next_ts = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_ts:
                try:
                    self._send()
                except Exception:
                    pass
                next_ts = now + self.interval
            else:
                self._stop.wait(max(0.01, next_ts - now))


class IntegrationService:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.enabled = False
        self.error: Optional[str] = None

        self.tx_ids, self.rx_ids = _load_message_ids()
        self.tx_names = {mid: _resolve_message_name(mid) for mid in self.tx_ids}
        self.rx_names = {mid: _resolve_message_name(mid) for mid in self.rx_ids}

        self.tx_log = deque(maxlen=60)
        self.rx_log = deque(maxlen=60)
        self.tx_payload: Dict[str, Any] = {}
        self.rx_payload: Dict[str, Any] = {}
        self.tx_times: Dict[str, deque] = {}
        self.rx_times: Dict[str, deque] = {}
        self.tx_counts: Dict[str, int] = {}
        self.rx_counts: Dict[str, int] = {}

        self.periodic_config = {
            "0102": 5,
            "0103": 5,
            "0401": float(SIM_0401_IDLE_HZ),
            "0501": 5,
        }
        self._periodic_senders: Dict[str, _PeriodicSender] = {}

        self._messenger = None

        self._init_bus()

    def _init_bus(self) -> None:
        try:
            os.environ.setdefault("KU_ROLE", "integration")
            root = Path(__file__).resolve().parents[3]
            self._ensure_fusion_configs(root)
            self._load_msglib_and_deps(root)
            from modules.common.dll_files.nFusionImports import FusionNodeIoc, NodeMessenger  # pylint: disable=import-error

            # Register receivers
            __import__("modules.common.receive", fromlist=["*"])

            FusionNodeIoc.Configure()
            NodeMessenger.Initialize("SIM_WEB_INT")
            NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
            NodeMessenger.InitAllSubscriberFromAssembly()
            NodeMessenger.RegistAllProviderFromFusionNodeIoc()

            self._messenger = NodeMessenger
            self.enabled = True
            self.error = None
        except Exception as exc:
            self.enabled = False
            self.error = str(exc)
            return

        for mid in self.rx_ids:
            register_listener(mid, self._on_receive)

    def _ensure_fusion_configs(self, root: Path) -> None:
        candidates = [
            root / "nFusionSettings.json",
            root / "modules" / "common" / "nFusionSettings.json",
            root / "FusionSettings.json",
            root / "modules" / "common" / "FusionSettings.json",
            root / "nFusion" / "FusionSettings.json",
        ]
        src = next((p for p in candidates if p.exists()), None)
        if src is None:
            raise FileNotFoundError("nFusionSettings.json missing")
        dst = root / "nFusionSettings.json"
        if src != dst:
            copy_file_with_retry(src, dst)

        lcands = [
            root / "nFusionLicense.lic",
            root / "modules" / "common" / "nFusionLicense.lic",
            root / "nFusion" / "nFusionLicense.lic",
        ]
        lsrc = next((p for p in lcands if p.exists()), None)
        if lsrc:
            ldst = root / "nFusionLicense.lic"
            if lsrc != ldst:
                copy_file_with_retry(lsrc, ldst)

    def _load_msglib_and_deps(self, root: Path) -> None:
        try:
            from modules.common.dll_files.nFusionImports import clr  # pylint: disable=import-error
        except Exception:
            import clr  # type: ignore

        msg_dir = root / "modules" / "common" / "msg_files"
        stem = msg_dir / "MessageLibrary"
        try:
            clr.AddReference(str(stem))
        except Exception:
            try:
                clr.AddReference(str(stem.with_suffix(".dll")))
            except Exception:
                pass
        for s in ("K4586Model", "K4586Model.Assist", "MiscUtil"):
            dll = msg_dir / f"{s}.dll"
            if dll.exists():
                try:
                    clr.AddReference(str(dll.with_suffix("")))
                except Exception:
                    try:
                        clr.AddReference(str(dll))
                    except Exception:
                        pass

    def _append_log(self, which: str, msg_id: str, payload: Any) -> None:
        ts = _fmt_ts(time.time())
        line = f"[{ts}] {which:<4} : {msg_id}"
        if which == "SEND":
            self.tx_log.appendleft(line)
        else:
            self.rx_log.appendleft(line)

    def _record_time(self, store: Dict[str, deque], msg_id: str, counts: Dict[str, int]) -> None:
        if msg_id not in store:
            store[msg_id] = deque(maxlen=120)
        store[msg_id].append(time.time())
        counts[msg_id] = counts.get(msg_id, 0) + 1

    def _calc_rate(self, times: deque, window_sec: float = 10.0) -> Optional[float]:
        if not times or len(times) < 2:
            return None
        latest = times[-1]
        cutoff = latest - window_sec
        recent = [t for t in times if t >= cutoff]
        if len(recent) < 2:
            return None
        span = recent[-1] - recent[0]
        if span <= 0:
            return None
        return (len(recent) - 1) / span

    def _send_once(self, msg_id: str, body: Optional[dict] = None) -> dict:
        if not self.enabled or self._messenger is None:
            return {"ok": False, "error": self.error or "Messenger not ready"}
        msg_id = str(msg_id).zfill(4)
        try:
            captured: dict[str, Any] = {}

            def _on_done(mid: str, raw: Any) -> None:
                captured["raw"] = raw

            push_message(msg_id, self._messenger, on_done=_on_done, body_dict=body)
            raw = captured.get("raw")
            with self._lock:
                self.tx_payload[msg_id] = _safe_json(raw)
                self._append_log("SEND", msg_id, raw)
                self._record_time(self.tx_times, msg_id, self.tx_counts)
            return {"ok": True}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def toggle_send(self, msg_id: str) -> dict:
        if not msg_id:
            return {"ok": False, "error": "msgId required"}
        msg_id = str(msg_id).zfill(4)
        if not self.enabled or self._messenger is None:
            return {"ok": False, "error": self.error or "Messenger not ready"}
        freq = self.periodic_config.get(msg_id)
        if not freq:
            return self._send_once(msg_id)

        with self._lock:
            if msg_id in self._periodic_senders:
                self._periodic_senders[msg_id].stop()
                del self._periodic_senders[msg_id]
                return {"ok": True, "running": False}

            sender = _PeriodicSender(1.0 / float(freq), lambda: self._send_once(msg_id))
            self._periodic_senders[msg_id] = sender
            sender.start()
            return {"ok": True, "running": True}

    def generate(self, msg_id: str) -> dict:
        if not msg_id:
            return {"ok": False, "error": "msgId required"}
        msg_id = str(msg_id).zfill(4)
        try:
            mod = __import__(f"modules.common.generator.message{msg_id}_generator", fromlist=[f"make_msg{msg_id}_body"])
            fn = getattr(mod, f"make_msg{msg_id}_body", None)
            if fn is None:
                return {"ok": False, "error": "Generator not found"}
            body = fn("SIM")
            return {"ok": True, "payload": body}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def send_custom(self, msg_id: str, body: Optional[dict]) -> dict:
        if not msg_id:
            return {"ok": False, "error": "msgId required"}
        if not isinstance(body, dict):
            return {"ok": False, "error": "body required"}
        return self._send_once(str(msg_id).zfill(4), body)

    def get_payload(self, msg_id: str, payload_type: str) -> dict:
        msg_id = str(msg_id).zfill(4)
        if payload_type == "rx":
            payload = self.rx_payload.get(msg_id)
        else:
            payload = self.tx_payload.get(msg_id)
        if payload is None:
            return {"ok": False, "error": "No payload"}
        return {"ok": True, "payload": payload}

    def _on_receive(self, msg_id: str, payload: object) -> None:
        msg_id = str(msg_id).zfill(4)
        with self._lock:
            self.rx_payload[msg_id] = _safe_json(payload)
            self._append_log("RECV", msg_id, payload)
            self._record_time(self.rx_times, msg_id, self.rx_counts)

    def _build_state_row(self, msg_id: str, name: str, is_tx: bool) -> dict:
        msg_id = str(msg_id).zfill(4)
        level = "muted"
        state = "Idle"
        running = False

        if is_tx:
            running = msg_id in self._periodic_senders
            freq = self.periodic_config.get(msg_id)
            count = self.tx_counts.get(msg_id, 0)
            if count > 0:
                state = f"SENT({count})"
                level = "ok"
            elif running and freq:
                state = "SENT(0)"
                level = "warn"
        else:
            freq = self.periodic_config.get(msg_id)
            count = self.rx_counts.get(msg_id, 0)
            if count > 0:
                state = f"RECV({count})"
                level = "ok"
            elif freq:
                state = "RECV(0)"
                level = "muted"
        return {
            "id": msg_id,
            "name": name,
            "state": state,
            "level": level,
            "running": running,
            "periodic": msg_id in self.periodic_config,
        }

    def get_state(self) -> dict:
        with self._lock:
            tx_rows = [self._build_state_row(mid, self.tx_names.get(mid, mid), True) for mid in self.tx_ids]
            rx_rows = [self._build_state_row(mid, self.rx_names.get(mid, mid), False) for mid in self.rx_ids]
            return {
                "ok": True,
                "enabled": self.enabled,
                "error": self.error,
                "tx": tx_rows,
                "rx": rx_rows,
                "logs": {"tx": list(self.tx_log), "rx": list(self.rx_log)},
                "timestamp": _now_ms_2000(),
            }

    def reset_state(self) -> dict:
        with self._lock:
            for sender in list(self._periodic_senders.values()):
                try:
                    sender.stop()
                except Exception:
                    pass
            self._periodic_senders = {}
            self.tx_log.clear()
            self.rx_log.clear()
            self.tx_payload = {}
            self.rx_payload = {}
            self.tx_times = {}
            self.rx_times = {}
            self.tx_counts = {}
            self.rx_counts = {}
        return {"ok": True}

    def shutdown(self) -> None:
        with self._lock:
            for sender in list(self._periodic_senders.values()):
                try:
                    sender.stop()
                except Exception:
                    pass
            self._periodic_senders.clear()
