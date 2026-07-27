from __future__ import annotations

import json
import os
import re
import socket
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

from modules.common.settings_paths import (
    ensure_fusion_license_file,
    ensure_fusion_settings_file,
    fusion_settings_candidates,
    fusion_settings_runtime_targets,
    fusion_runtime_working_dir,
)
from modules.common.push_center import push_message
from modules.common.receive_center import register_listener
from modules.sim.config import SIM_0401_IDLE_HZ


_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_DEFAULT_MIDDLEWARE_SETTINGS = {
    "Name": "AVS1",
    "NetworkAddress": "192.168.20.105",
    "LocalDomain": 10,
    "ExternalDomain": 100,
}
_PAYLOAD_OBSERVATION_IDS = {"0902", "0305"}
_PAYLOAD_OBSERVATION_MAX_RECORDS = 240
_NEXT_COLLAB_PLAN_APPLY_RETRY_DELAYS = (
    (0.0, "0903-now"),
    (0.15, "0903-artifact-settle"),
    (0.5, "0903-artifact-retry-1"),
    (1.0, "0903-artifact-retry-2"),
    (2.0, "0903-artifact-retry-3"),
    (4.0, "0903-artifact-retry-4"),
    (8.0, "0903-artifact-retry-5"),
    (15.0, "0903-artifact-retry-6"),
)


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


def _pick_ci(obj: Any, *keys: str) -> Any:
    if not isinstance(obj, dict):
        return None
    for key in keys:
        if key in obj:
            return obj.get(key)
    lowered = {str(key).lower(): value for key, value in obj.items()}
    for key in keys:
        value = lowered.get(str(key).lower())
        if value is not None:
            return value
    return None


def _to_int(value: Any, default: int | None = None) -> int | None:
    try:
        if value is None or value == "":
            return default
        return int(float(value))
    except Exception:
        return default


def _to_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return bool(int(value))
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ("1", "true", "yes", "y", "on"):
            return True
        if lowered in ("0", "false", "no", "n", "off"):
            return False
    return default


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
        "0204",
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


def _coerce_middleware_settings(payload: Any) -> tuple[Optional[dict[str, Any]], Optional[str]]:
    source = payload
    if isinstance(payload, dict) and isinstance(payload.get("Middleware"), dict):
        source = payload.get("Middleware")
    if not isinstance(source, dict):
        source = {}

    name = str(source.get("Name") or _DEFAULT_MIDDLEWARE_SETTINGS["Name"]).strip()
    network = str(source.get("NetworkAddress") or _DEFAULT_MIDDLEWARE_SETTINGS["NetworkAddress"]).strip()
    try:
        local_domain = int(source.get("LocalDomain", _DEFAULT_MIDDLEWARE_SETTINGS["LocalDomain"]))
    except Exception:
        return None, "LocalDomain must be an integer"
    try:
        external_domain = int(source.get("ExternalDomain", _DEFAULT_MIDDLEWARE_SETTINGS["ExternalDomain"]))
    except Exception:
        return None, "ExternalDomain must be an integer"

    if not name:
        return None, "Name is required"
    if not network:
        return None, "NetworkAddress is required"

    return {
        "Name": name,
        "NetworkAddress": network,
        "LocalDomain": local_domain,
        "ExternalDomain": external_domain,
    }, None


def _collect_local_ipv4_addresses() -> list[str]:
    found: set[str] = {"127.0.0.1"}
    try:
        hostname = socket.gethostname()
        _, _, addrs = socket.gethostbyname_ex(hostname)
        for addr in addrs:
            if addr:
                found.add(str(addr))
    except Exception:
        pass

    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        local_ip = sock.getsockname()[0]
        if local_ip:
            found.add(str(local_ip))
    except Exception:
        pass
    finally:
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass

    return sorted(found)


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
    def __init__(self, node_name: str = "SIM_WEB_INT") -> None:
        self._lock = threading.RLock()
        self._node_name = str(node_name or "SIM_WEB_INT").strip() or "SIM_WEB_INT"
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
        self._payload_observation_lock = threading.Lock()

        self.periodic_config = {
            "0102": 5,
            "0103": 5,
            "0401": float(SIM_0401_IDLE_HZ),
            "0501": 5,
        }
        self._periodic_senders: Dict[str, _PeriodicSender] = {}

        self._sim_service = None
        self._handled_0803_keys = deque(maxlen=32)
        self._next_collab_transition_seq = 0
        self._pending_next_collab_transition: dict[str, Any] | None = None
        self._last_next_collab_transition: dict[str, Any] | None = None
        self._direct_plan_apply_seq = 0
        self._pending_direct_plan_apply: dict[str, Any] | None = None
        self._last_direct_plan_apply: dict[str, Any] | None = None
        self._messenger = None
        self._init_bus()

    def set_sim_service(self, sim) -> None:
        with self._lock:
            if self._sim_service is not sim:
                self._next_collab_transition_seq += 1
                self._pending_next_collab_transition = None
                self._direct_plan_apply_seq += 1
                self._pending_direct_plan_apply = None
                self.rx_payload.pop("0903", None)
            self._sim_service = sim

    def _project_root(self) -> Path:
        return Path(__file__).resolve().parents[3]

    def _settings_candidates(self, root: Path) -> list[Path]:
        return list(
            fusion_settings_candidates(
                project_root=root,
                common_dir=root / "modules" / "common",
                ds_dir=root / "modules" / "app" / "ui",
            )
        )

    def _settings_path(self, root: Path) -> Path:
        for candidate in self._settings_candidates(root):
            if candidate.exists():
                return candidate
        return root / "settings" / "nFusionSettings.json"

    def _read_middleware_settings(self) -> dict[str, Any]:
        root = self._project_root()
        cfg_path = self._settings_path(root)
        try:
            raw = json.loads(cfg_path.read_text(encoding="utf-8"))
        except Exception:
            raw = {}
        settings, _error = _coerce_middleware_settings(raw)
        if settings is None:
            return dict(_DEFAULT_MIDDLEWARE_SETTINGS)
        return settings

    def get_settings(self) -> dict:
        with self._lock:
            active = bool(self.enabled and self._messenger is not None)
            return {
                "ok": True,
                "settings": self._read_middleware_settings(),
                "localAddresses": _collect_local_ipv4_addresses(),
                "active": active,
                "restartRequired": active,
                "note": (
                    "nFusion is already active. Network changes are saved, but they fully apply after restarting sim_main."
                    if active
                    else "Saved values will be used when nFusion activates on Play or Monitoring."
                ),
            }

    def update_settings(self, payload: Any) -> dict:
        settings, error = _coerce_middleware_settings(payload)
        if settings is None:
            return {"ok": False, "error": error or "Invalid middleware settings"}

        root = self._project_root()
        cfg_json = json.dumps({"Middleware": settings}, ensure_ascii=False, separators=(",", ":"))
        targets = list(
            fusion_settings_runtime_targets(
                project_root=root,
                common_dir=root / "modules" / "common",
                ds_dir=root / "modules" / "app" / "ui",
            )
        )

        updated_paths: list[str] = []
        for cfg_path in targets:
            try:
                cfg_path.parent.mkdir(parents=True, exist_ok=True)
                cfg_path.write_text(cfg_json, encoding="utf-8")
                updated_paths.append(str(cfg_path))
            except Exception as exc:
                return {"ok": False, "error": f"Failed to update {cfg_path.name}: {exc}"}

        active = bool(self.enabled and self._messenger is not None)
        return {
            "ok": True,
            "settings": settings,
            "updatedPaths": updated_paths,
            "localAddresses": _collect_local_ipv4_addresses(),
            "active": active,
            "restartRequired": active,
            "note": (
                "nFusion is already active. Network changes are saved, but they fully apply after restarting sim_main."
                if active
                else "Saved. These values will be applied when nFusion activates."
            ),
        }

    def ensure_ready(self) -> bool:
        if self.enabled and self._messenger is not None:
            return True
        with self._lock:
            if self.enabled and self._messenger is not None:
                return True
            self._init_bus()
            return bool(self.enabled and self._messenger is not None)

    def _init_bus(self) -> None:
        if self.enabled and self._messenger is not None:
            return
        try:
            os.environ.setdefault("KU_ROLE", "integration")
            root = self._project_root()
            self._ensure_fusion_configs(root)
            self._load_msglib_and_deps(root)
            from modules.common.dll_files.nFusionImports import FusionNodeIoc, NodeMessenger  # pylint: disable=import-error

            # Register receivers
            __import__("modules.common.receive", fromlist=["*"])

            with fusion_runtime_working_dir(project_root=root):
                FusionNodeIoc.Configure()
                NodeMessenger.Initialize(self._node_name)
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
        dst = ensure_fusion_settings_file(
            project_root=root,
            common_dir=root / "modules" / "common",
            ds_dir=root / "modules" / "app" / "ui",
        )
        if dst is None:
            raise FileNotFoundError("nFusionSettings.json missing")
        ensure_fusion_license_file(project_root=root, common_dir=root / "modules" / "common")

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

    def _record_time(self, store: Dict[str, deque], msg_id: str, counts: Dict[str, int]) -> float:
        observed_at = time.time()
        if msg_id not in store:
            store[msg_id] = deque(maxlen=120)
        store[msg_id].append(observed_at)
        counts[msg_id] = counts.get(msg_id, 0) + 1
        return observed_at

    def _payload_observation_path(self, msg_id: str) -> Path | None:
        if msg_id not in _PAYLOAD_OBSERVATION_IDS:
            return None
        try:
            from modules.common import db_paths

            return Path(db_paths.get_active_db_root()) / "DSS_Internal" / "sim_payload_observations" / f"{msg_id}.json"
        except Exception:
            return None

    def _payload_observation_record(
        self,
        msg_id: str,
        payload_type: str,
        payload: Any,
        observed_at: float,
        extra: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        observed_ms = int(float(observed_at) * 1000.0)
        record: dict[str, Any] = {
            "ok": True,
            "msgId": msg_id,
            "type": payload_type,
            "observedAtSec": float(observed_at),
            "observedAtMs": observed_ms,
            "payload": _safe_json(payload),
        }
        if payload_type == "rx":
            record["receivedAtSec"] = float(observed_at)
            record["receivedAtMs"] = observed_ms
        else:
            record["sentAtSec"] = float(observed_at)
            record["sentAtMs"] = observed_ms
        if extra:
            for key, value in extra.items():
                if value is not None:
                    record[key] = value
        return record

    def _append_payload_observation(self, record: dict[str, Any]) -> None:
        msg_id = str(record.get("msgId") or "").zfill(4)
        path = self._payload_observation_path(msg_id)
        if path is None:
            return
        try:
            with self._payload_observation_lock:
                path.parent.mkdir(parents=True, exist_ok=True)
                records: list[Any] = []
                if path.exists():
                    try:
                        loaded = json.loads(path.read_text(encoding="utf-8"))
                        records = loaded if isinstance(loaded, list) else [loaded]
                    except Exception:
                        records = []
                records.append(record)
                records = records[-_PAYLOAD_OBSERVATION_MAX_RECORDS:]
                tmp = path.with_suffix(path.suffix + ".tmp")
                tmp.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
                tmp.replace(path)
        except Exception:
            pass

    def _record_payload_observation(
        self,
        msg_id: str,
        payload_type: str,
        payload: Any,
        observed_at: float,
    ) -> None:
        msg_id = str(msg_id).zfill(4)
        if msg_id not in _PAYLOAD_OBSERVATION_IDS:
            return
        record = self._payload_observation_record(msg_id, payload_type, payload, observed_at)
        self._append_payload_observation(record)

    def record_payload_observation(self, body: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(body, dict):
            return {"ok": False, "error": "body required"}
        msg_id = str(body.get("msgId") or body.get("msgID") or "").zfill(4)
        if msg_id not in _PAYLOAD_OBSERVATION_IDS:
            return {"ok": False, "error": "unsupported msgId"}
        payload_type = str(body.get("type") or body.get("payloadType") or "rx")
        if payload_type not in ("rx", "tx"):
            payload_type = "rx"
        try:
            observed_at = float(body.get("observedAtSec") or body.get("receivedAtSec") or body.get("sentAtSec"))
        except Exception:
            observed_at = time.time()
        extra: dict[str, Any] = {}
        displayed_value = body.get("displayedAtSec") or body.get("popupDisplayedAtSec") or body.get("openedAtSec")
        if displayed_value is not None:
            try:
                displayed_at = float(displayed_value)
                extra["displayedAtSec"] = displayed_at
                extra["popupDisplayedAtSec"] = displayed_at
                extra["displayedAtMs"] = int(displayed_at * 1000.0)
                extra["popupDisplayedAtMs"] = int(displayed_at * 1000.0)
            except Exception:
                pass
        payload = body.get("payload")
        record = self._payload_observation_record(msg_id, payload_type, payload, observed_at, extra)
        self._merge_payload_observation(record)
        return {"ok": True}

    def _merge_payload_observation(self, record: dict[str, Any]) -> None:
        msg_id = str(record.get("msgId") or "").zfill(4)
        path = self._payload_observation_path(msg_id)
        if path is None:
            return
        try:
            with self._payload_observation_lock:
                path.parent.mkdir(parents=True, exist_ok=True)
                records: list[Any] = []
                if path.exists():
                    try:
                        loaded = json.loads(path.read_text(encoding="utf-8"))
                        records = loaded if isinstance(loaded, list) else [loaded]
                    except Exception:
                        records = []
                target_ms = record.get("observedAtMs")
                target_type = record.get("type")
                updated = False
                for existing in reversed(records):
                    if not isinstance(existing, dict):
                        continue
                    if str(existing.get("msgId") or "").zfill(4) != msg_id:
                        continue
                    if existing.get("type") != target_type:
                        continue
                    if existing.get("observedAtMs") != target_ms:
                        continue
                    for key, value in record.items():
                        if value is not None:
                            existing[key] = value
                    updated = True
                    break
                if not updated:
                    records.append(record)
                records = records[-_PAYLOAD_OBSERVATION_MAX_RECORDS:]
                tmp = path.with_suffix(path.suffix + ".tmp")
                tmp.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
                tmp.replace(path)
        except Exception:
            pass

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
        if (not self.enabled or self._messenger is None) and not self.ensure_ready():
            return {"ok": False, "error": self.error or "Messenger not ready"}
        msg_id = str(msg_id).zfill(4)
        try:
            captured: dict[str, Any] = {}

            def _on_done(mid: str, raw: Any) -> None:
                captured["raw"] = raw

            sent = push_message(msg_id, self._messenger, on_done=_on_done, body_dict=body)
            if not sent:
                return {"ok": False, "error": f"Message {msg_id} push failed"}
            raw = captured.get("raw")
            with self._lock:
                self.tx_payload[msg_id] = _safe_json(raw)
                self._append_log("SEND", msg_id, raw)
                observed_at = self._record_time(self.tx_times, msg_id, self.tx_counts)
            self._record_payload_observation(msg_id, "tx", raw, observed_at)
            if msg_id == "0803":
                self._handle_0803(body if isinstance(body, dict) else raw)
            return {"ok": True}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def toggle_send(self, msg_id: str) -> dict:
        if not msg_id:
            return {"ok": False, "error": "msgId required"}
        msg_id = str(msg_id).zfill(4)
        if (not self.enabled or self._messenger is None) and not self.ensure_ready():
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
        with self._lock:
            if payload_type == "rx":
                payload = self.rx_payload.get(msg_id)
                times = self.rx_times.get(msg_id)
                count = self.rx_counts.get(msg_id, 0)
                observed_key = "receivedAtSec"
            else:
                payload = self.tx_payload.get(msg_id)
                times = self.tx_times.get(msg_id)
                count = self.tx_counts.get(msg_id, 0)
                observed_key = "sentAtSec"
            observed_at = float(times[-1]) if times else None
        if payload is None:
            return {"ok": False, "error": "No payload"}
        result = {"ok": True, "payload": payload, "count": count}
        if observed_at is not None:
            result[observed_key] = observed_at
            result["observedAtSec"] = observed_at
            result["observedAtMs"] = int(observed_at * 1000.0)
        return result

    def _extract_mission_plan_id(self, payload: Any) -> int | None:
        body = _safe_json(payload)
        if not isinstance(body, dict):
            return None
        return _to_int(
            _pick_ci(
                body,
                "missionPlanID",
                "MissionPlanID",
                "missionPlanId",
                "mission_plan_id",
            ),
            None,
        )

    def _extract_timestamp(self, payload: Any) -> int | None:
        body = _safe_json(payload)
        if not isinstance(body, dict):
            return None
        return _to_int(_pick_ci(body, "timestamp", "Timestamp", "timeStamp", "TimeStamp"), None)

    def _latest_mission_plan_id_from_db(self) -> tuple[int | None, int | None]:
        try:
            from modules.common import db_paths

            folder = Path(db_paths.get_active_db_root()) / "MissionPlan"
        except Exception:
            return None, None
        if not folder.exists():
            return None, None
        best_id = None
        best_ts = None
        for path in folder.glob("*.json"):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except Exception:
                data = {}
            plan_id = self._extract_mission_plan_id(data)
            if plan_id is None:
                plan_id = _to_int(path.stem, None)
            ts = self._extract_timestamp(data)
            if ts is None:
                ts = _to_int(_pick_ci(data, "missionPlanTimestamp", "MissionPlanTimestamp"), None)
            if ts is None:
                try:
                    ts = int(path.stat().st_mtime_ns)
                except Exception:
                    ts = None
            if plan_id is None or ts is None:
                continue
            if best_id is None or int(ts) >= int(best_ts or -1):
                best_id = int(plan_id)
                best_ts = int(ts)
        return best_id, best_ts

    def _resolve_current_mission_plan_id(self, *, prefer_loaded: bool = True) -> int | None:
        sim = self._sim_service
        if prefer_loaded and sim is not None:
            loaded_plan_id = _to_int(getattr(sim, "_loaded_mission_plan_id", None), None)
            if loaded_plan_id is not None and loaded_plan_id > 0:
                return int(loaded_plan_id)
        candidates: list[tuple[int, int]] = []
        with self._lock:
            payload_0903 = self.rx_payload.get("0903")
            payload_0702 = self.rx_payload.get("0702")
            payload_0701 = self.rx_payload.get("0701")
        plan_id = self._extract_mission_plan_id(payload_0903)
        if plan_id is not None:
            candidates.append((self._extract_timestamp(payload_0903) or 0, int(plan_id)))
        body_0702 = _safe_json(payload_0702)
        if isinstance(body_0702, dict):
            ignore = _to_int(_pick_ci(body_0702, "ignore", "Ignore"), None)
            plan_id = self._extract_mission_plan_id(body_0702) if ignore == 2 else None
            if plan_id is not None:
                candidates.append((self._extract_timestamp(body_0702) or 0, int(plan_id)))
        body_0701 = _safe_json(payload_0701)
        if isinstance(body_0701, dict):
            raw_options = (
                _pick_ci(body_0701, "optionList", "OptionList", "optionInfoList")
                or []
            )
            if isinstance(raw_options, list):
                selected = None
                for option in raw_options:
                    if not isinstance(option, dict):
                        continue
                    plan_id = self._extract_mission_plan_id(option)
                    if plan_id is None:
                        continue
                    recommended = bool(_pick_ci(option, "recommend", "Recommend", "recommended"))
                    if selected is None or recommended:
                        selected = int(plan_id)
                    if recommended:
                        break
                if selected is not None:
                    candidates.append((self._extract_timestamp(body_0701) or 0, selected))
        db_plan_id, db_ts = self._latest_mission_plan_id_from_db()
        if db_plan_id is not None:
            candidates.append((db_ts or 0, int(db_plan_id)))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        return int(candidates[-1][1])

    def _capture_0803_transition_inputs_and_hold(
        self,
    ) -> tuple[dict[int, int], dict[int, int]]:
        sim = self._sim_service
        if sim is None:
            return {}, {}
        current: dict[int, int] = {}
        expected: dict[int, int] = {}
        lock = getattr(sim, "_lock", None)
        context = lock if lock is not None else threading.RLock()
        with context:
            aircraft_ids = set(getattr(sim, "input_mission_order_by_aircraft", {}) or {})
            for simv in (getattr(sim, "vehicles", {}) or {}).values():
                try:
                    aircraft_ids.add(int(getattr(simv, "aircraft_id", 0) or 0))
                except Exception:
                    continue
            for aid in aircraft_ids:
                if int(aid) <= 0:
                    continue
                try:
                    next_id = sim._next_input_mission_id_for(int(aid))
                    current_id = sim._current_input_mission_id_for(int(aid))
                except Exception:
                    next_id = None
                    current_id = None
                target_id = next_id if next_id is not None else current_id
                if current_id is not None:
                    current[int(aid)] = int(current_id)
                if target_id is not None:
                    expected[int(aid)] = int(target_id)
            # The SIM step loop uses the same lock, so no queued advance can
            # slip between the boundary snapshot and the hold operation.
            sim.clear_pending_input_advances()
        return current, expected

    def _begin_next_collab_transition(
        self,
        *,
        expected: dict[int, int],
        current: dict[int, int] | None = None,
        source_plan_id: int | None,
        request_body: dict[str, Any],
    ) -> int:
        normalized_expected = {
            int(aid): int(input_id)
            for aid, input_id in (expected or {}).items()
            if int(aid) > 0 and int(input_id) > 0
        }
        request_timestamp = self._extract_timestamp(request_body)
        normalized_current = {
            int(aid): int(input_id)
            for aid, input_id in (current or {}).items()
            if int(aid) > 0 and int(input_id) > 0
        }
        with self._lock:
            # Do not let the browser poller mistake the previous notification
            # for the result of this newly requested transition.
            self.rx_payload.pop("0903", None)
            self._direct_plan_apply_seq += 1
            self._pending_direct_plan_apply = None
            self._next_collab_transition_seq += 1
            token = int(self._next_collab_transition_seq)
            self._pending_next_collab_transition = {
                "token": token,
                "status": "armed",
                "sourcePlanID": int(source_plan_id) if source_plan_id is not None else None,
                "currentInputs": normalized_current,
                "expectedInputs": normalized_expected,
                "requestTimestamp": int(request_timestamp) if request_timestamp is not None else None,
                "requestedAtMonotonic": float(time.monotonic()),
                "scheduledPlanIDs": set(),
                "applyInProgress": False,
            }
            self._last_next_collab_transition = None
            return token

    def _cancel_next_collab_transition(self, *, wait_for_apply: bool = False) -> None:
        with self._lock:
            self._next_collab_transition_seq += 1
            self._pending_next_collab_transition = None
            self.rx_payload.pop("0903", None)
        if wait_for_apply:
            sim = self._sim_service
            mission_load_lock = getattr(sim, "_mission_load_lock", None)
            if mission_load_lock is not None:
                with mission_load_lock:
                    pass

    def cancel_pending_next_collab_transition(self) -> None:
        self._cancel_next_collab_transition(wait_for_apply=True)

    def has_pending_next_collab_transition(self) -> bool:
        with self._lock:
            return isinstance(self._pending_next_collab_transition, dict)

    @staticmethod
    def _next_collab_plan_result_ready(
        plan_result: dict[str, Any],
        *,
        target_input_id: int | None = None,
        expected_aircraft_ids: set[int] | None = None,
    ) -> bool:
        if not isinstance(plan_result, dict) or not plan_result.get("ok"):
            return False
        if list(plan_result.get("missingPathIds") or []):
            return False
        if not list(plan_result.get("inputMissionPlans") or []):
            return False
        if not list(plan_result.get("flightPaths") or []):
            return False
        mission_plan = plan_result.get("missionPlan")
        aircraft_rows = (
            mission_plan.get("aircraftList")
            if isinstance(mission_plan, dict) and isinstance(mission_plan.get("aircraftList"), list)
            else []
        )
        expected_packages = {
            int(package_id)
            for row in aircraft_rows
            if isinstance(row, dict)
            for package_id in [
                _to_int(
                    _pick_ci(
                        row,
                        "individualMissionPackageID",
                        "IndividualMissionPackageID",
                        "individualMissionPackageId",
                    ),
                    None,
                )
            ]
            if package_id is not None and int(package_id) > 0
        }
        individual_plans = list(plan_result.get("individualMissionPlans") or [])
        actual_packages: set[int] = set()
        first_active_input_by_aircraft: dict[int, int] = {}
        for row in individual_plans:
            if not isinstance(row, dict):
                continue
            data = row.get("data") if isinstance(row.get("data"), dict) else row
            package_id = _to_int(
                _pick_ci(
                    data,
                    "individualMissionPackageID",
                    "IndividualMissionPackageID",
                    "individualMissionPackageId",
                ),
                None,
            )
            if package_id is not None and package_id > 0:
                actual_packages.add(int(package_id))
            aircraft_id = _to_int(_pick_ci(data, "aircraftID", "AircraftID"), None)
            if aircraft_id is None or aircraft_id <= 0:
                continue
            for mission in list(data.get("individualMissionList") or []):
                if not isinstance(mission, dict):
                    continue
                related = _pick_ci(mission, "relatedMission", "RelatedMission") or {}
                if not isinstance(related, dict):
                    related = {}
                input_id = _to_int(
                    _pick_ci(related, "inputMissionID", "InputMissionID")
                    or _pick_ci(mission, "inputMissionID", "InputMissionID"),
                    None,
                )
                is_explicit_target = (
                    target_input_id is not None
                    and input_id is not None
                    and int(input_id) == int(target_input_id)
                )
                # startInputMissionID is an explicit SIM execution command.
                # Monitoring can briefly write stale completion bits while the
                # new plan is settling; do not let those bits make the exact
                # requested transition permanently unloadable.
                if (
                    not is_explicit_target
                    and _to_bool(_pick_ci(mission, "isDone", "IsDone"), False)
                ):
                    continue
                if (
                    not is_explicit_target
                    and _to_bool(
                        _pick_ci(
                            mission,
                            "executionBlockedUntilNextCollab",
                            "ExecutionBlockedUntilNextCollab",
                        ),
                        False,
                    )
                ):
                    continue
                if input_id is not None and input_id > 0:
                    first_active_input_by_aircraft[int(aircraft_id)] = int(input_id)
                    break
        if expected_packages and not expected_packages.issubset(actual_packages):
            return False
        if target_input_id is not None:
            expected_aids = {
                int(aid)
                for aid in (expected_aircraft_ids or set(first_active_input_by_aircraft))
                if int(aid) > 0
            }
            if not expected_aids or not expected_aids.issubset(first_active_input_by_aircraft):
                return False
            if any(
                int(first_active_input_by_aircraft[aid]) != int(target_input_id)
                for aid in expected_aids
            ):
                return False
        return True

    @staticmethod
    def _next_collab_detail_matches_pending(
        *,
        mission_plan_id: int,
        pending: dict[str, Any],
        detail: dict[str, Any] | None = None,
    ) -> bool:
        if detail is None:
            try:
                from modules.mission_planning.runtime import next_collab_replan_store

                detail = next_collab_replan_store.load_detail(int(mission_plan_id))
            except Exception:
                return False
        if not isinstance(detail, dict):
            return False
        trigger = str(detail.get("trigger") or "").strip().lower()
        trigger_type = str(detail.get("triggerType") or "").strip().lower()
        if trigger != "0803" or trigger_type != "nextcollaborativemission":
            return False

        source_plan_id = _to_int(pending.get("sourcePlanID"), None)
        detail_source_plan_id = _to_int(detail.get("sourceMissionPlanID"), None)
        if source_plan_id is None or source_plan_id <= 0 or detail_source_plan_id is None:
            return False
        if int(mission_plan_id) <= int(source_plan_id):
            return False
        if int(detail_source_plan_id) != int(source_plan_id):
            return False
        detail_plan_id = _to_int(detail.get("missionPlanID"), None)
        if detail_plan_id is not None and int(detail_plan_id) != int(mission_plan_id):
            return False

        expected_map = {
            int(aid): int(value)
            for aid, value in dict(pending.get("expectedInputs") or {}).items()
            if _to_int(aid, None) is not None and _to_int(value, None) is not None
        }
        current_map = {
            int(aid): int(value)
            for aid, value in dict(pending.get("currentInputs") or {}).items()
            if _to_int(aid, None) is not None and _to_int(value, None) is not None
        }
        if not expected_map and not current_map:
            return False
        target_input_id = _to_int(detail.get("targetInputMissionID"), None)
        if target_input_id is None or target_input_id <= 0:
            return False

        entry_aircraft_ids = {
            int(aid)
            for row in list(detail.get("entryAircraftList") or [])
            if isinstance(row, dict)
            for aid in [_to_int(row.get("aircraftID"), None)]
            if aid is not None and int(aid) > 0
        }
        known_aircraft_ids = set(expected_map) | set(current_map)
        if not entry_aircraft_ids or not entry_aircraft_ids.issubset(known_aircraft_ids):
            return False
        for aid in entry_aircraft_ids:
            boundary_ids = {
                int(value)
                for value in (current_map.get(aid), expected_map.get(aid))
                if value is not None
            }
            detail_current_input_id = _to_int(detail.get("currentInputMissionID"), None)
            if (
                int(target_input_id) not in boundary_ids
                and (
                    detail_current_input_id is None
                    or int(detail_current_input_id) not in boundary_ids
                )
            ):
                return False

        request_timestamp = _to_int(pending.get("requestTimestamp"), None)
        detail_timestamp = _to_int(detail.get("timestamp"), None)
        if request_timestamp is None or detail_timestamp is None:
            return False
        if int(detail_timestamp) < int(request_timestamp):
            return False
        return True

    def _schedule_next_collab_plan_apply(self, mission_plan_id: int) -> None:
        plan_id = int(mission_plan_id)
        with self._lock:
            pending = self._pending_next_collab_transition
            if not isinstance(pending, dict):
                return
            source_plan_id = _to_int(pending.get("sourcePlanID"), None)
            if source_plan_id is not None and plan_id <= int(source_plan_id):
                return
            scheduled = pending.get("scheduledPlanIDs")
            if not isinstance(scheduled, set):
                scheduled = set()
                pending["scheduledPlanIDs"] = scheduled
            if plan_id in scheduled:
                return
            scheduled.add(plan_id)
            token = int(pending.get("token") or 0)
            pending["status"] = "plan-notified"
            pending["notifiedPlanID"] = int(plan_id)
        self._schedule_next_collab_apply_attempt(token, plan_id, 0)

    def _schedule_direct_plan_apply(self, mission_plan_id: int) -> None:
        """Apply a 0903 plan that is not an 0803 next-collab transition.

        Attack/rejoin and other replans publish 0903 directly.  They must not
        depend on the special 0803 transition latch in order to reach SIM.
        """

        plan_id = int(mission_plan_id)
        sim = self._sim_service
        if sim is None:
            return
        loaded_plan_id = _to_int(getattr(sim, "_loaded_mission_plan_id", None), None)
        with self._lock:
            if isinstance(self._pending_next_collab_transition, dict):
                return
            if loaded_plan_id is not None and int(loaded_plan_id) >= plan_id:
                self._last_direct_plan_apply = {
                    "status": "already-loaded" if int(loaded_plan_id) == plan_id else "stale",
                    "missionPlanID": plan_id,
                }
                return
            pending = self._pending_direct_plan_apply
            if isinstance(pending, dict) and _to_int(pending.get("missionPlanID"), None) == plan_id:
                return
            self._direct_plan_apply_seq += 1
            token = int(self._direct_plan_apply_seq)
            self._pending_direct_plan_apply = {
                "token": token,
                "status": "plan-notified",
                "missionPlanID": plan_id,
                "applyInProgress": False,
            }
            self._last_direct_plan_apply = None
        self._schedule_direct_plan_apply_attempt(token, plan_id, 0)

    def _schedule_direct_plan_apply_attempt(
        self,
        token: int,
        mission_plan_id: int,
        attempt_index: int,
        *,
        delay_override: float | None = None,
    ) -> None:
        if attempt_index < 0 or attempt_index >= len(_NEXT_COLLAB_PLAN_APPLY_RETRY_DELAYS):
            return
        delay, _reason = _NEXT_COLLAB_PLAN_APPLY_RETRY_DELAYS[int(attempt_index)]
        timer = threading.Timer(
            float(delay_override if delay_override is not None else delay),
            self._apply_pending_direct_plan,
            args=(int(token), int(mission_plan_id), int(attempt_index)),
        )
        timer.daemon = True
        timer.start()

    def _apply_pending_direct_plan(
        self,
        token: int,
        mission_plan_id: int,
        attempt_index: int,
    ) -> None:
        sim = self._sim_service
        if sim is None:
            return
        with self._lock:
            pending = self._pending_direct_plan_apply
            if (
                not isinstance(pending, dict)
                or int(pending.get("token") or 0) != int(token)
                or isinstance(self._pending_next_collab_transition, dict)
            ):
                return
            if bool(pending.get("applyInProgress")):
                self._schedule_direct_plan_apply_attempt(
                    token,
                    mission_plan_id,
                    attempt_index,
                    delay_override=0.25,
                )
                return
            pending["applyInProgress"] = True
            pending["lastAttempt"] = int(attempt_index)

        applied = False
        failure_reason = "0903 plan artifacts are not ready"
        try:
            from modules.sim.mission.mission_plan_loader import build_mission_plan_payload

            plan_result = build_mission_plan_payload(int(mission_plan_id))
            if not self._next_collab_plan_result_ready(plan_result):
                return
            payload = dict(plan_result.get("payload") or {})
            payload["missionPlanID"] = int(mission_plan_id)
            payload["preserveState"] = True
            payload["skipIfMissionPlanAlreadyLoaded"] = True

            mission_load_lock = getattr(sim, "_mission_load_lock", None)
            load_context = mission_load_lock if mission_load_lock is not None else threading.RLock()
            with load_context:
                with self._lock:
                    current = self._pending_direct_plan_apply
                    if (
                        not isinstance(current, dict)
                        or int(current.get("token") or 0) != int(token)
                        or isinstance(self._pending_next_collab_transition, dict)
                    ):
                        return
                result = sim.load_mission(payload)
            if not result.get("ok"):
                failure_reason = str(result.get("error") or "SIM mission load failed")
                return
            loaded_plan_id = _to_int(getattr(sim, "_loaded_mission_plan_id", None), None)
            result_plan_id = _to_int(result.get("missionPlanID"), None)
            applied = bool(
                loaded_plan_id == int(mission_plan_id)
                or result_plan_id == int(mission_plan_id)
            )
            if not applied:
                failure_reason = "SIM accepted the payload but did not bind the 0903 plan"
        except Exception as exc:
            failure_reason = f"{type(exc).__name__}: {exc}"
        finally:
            retry_attempt: int | None = None
            with self._lock:
                current = self._pending_direct_plan_apply
                if isinstance(current, dict) and int(current.get("token") or 0) == int(token):
                    if applied:
                        self._last_direct_plan_apply = {
                            "status": "applied",
                            "missionPlanID": int(mission_plan_id),
                            "attempt": int(attempt_index),
                        }
                        self._pending_direct_plan_apply = None
                    else:
                        current["applyInProgress"] = False
                        current["status"] = "waiting-retry"
                        current["lastError"] = str(failure_reason)
                        next_attempt = int(attempt_index) + 1
                        if next_attempt < len(_NEXT_COLLAB_PLAN_APPLY_RETRY_DELAYS):
                            retry_attempt = next_attempt
                        else:
                            retry_attempt = len(_NEXT_COLLAB_PLAN_APPLY_RETRY_DELAYS) - 1
            if retry_attempt is not None:
                self._schedule_direct_plan_apply_attempt(
                    token,
                    mission_plan_id,
                    retry_attempt,
                )

    def _schedule_next_collab_apply_attempt(
        self,
        token: int,
        mission_plan_id: int,
        attempt_index: int,
        *,
        delay_override: float | None = None,
    ) -> None:
        if attempt_index < 0 or attempt_index >= len(_NEXT_COLLAB_PLAN_APPLY_RETRY_DELAYS):
            return
        delay, _reason = _NEXT_COLLAB_PLAN_APPLY_RETRY_DELAYS[int(attempt_index)]
        timer = threading.Timer(
            float(delay_override if delay_override is not None else delay),
            self._apply_pending_next_collab_plan,
            args=(int(token), int(mission_plan_id), int(attempt_index)),
        )
        timer.daemon = True
        timer.start()

    def _sim_inputs_match_expected(self, expected: dict[int, int]) -> bool:
        sim = self._sim_service
        if sim is None or not expected:
            return False
        lock = getattr(sim, "_lock", None)
        context = lock if lock is not None else threading.RLock()
        with context:
            for aid, expected_id in expected.items():
                try:
                    current_id = sim._current_input_mission_id_for(int(aid))
                except Exception:
                    return False
                if current_id is None or int(current_id) != int(expected_id):
                    return False
        return True

    def _apply_pending_next_collab_plan(
        self,
        token: int,
        mission_plan_id: int,
        attempt_index: int,
    ) -> None:
        sim = self._sim_service
        if sim is None:
            return
        with self._lock:
            pending = self._pending_next_collab_transition
            if not isinstance(pending, dict) or int(pending.get("token") or 0) != int(token):
                return
            if bool(pending.get("applyInProgress")):
                self._schedule_next_collab_apply_attempt(
                    token,
                    mission_plan_id,
                    attempt_index,
                    delay_override=0.25,
                )
                return
            pending["applyInProgress"] = True
            pending["lastAttempt"] = int(attempt_index)
            pending_snapshot = dict(pending)

        applied = False
        failure_reason = "transition plan is not ready"
        try:
            from modules.mission_planning.runtime import next_collab_replan_store

            detail = next_collab_replan_store.load_detail(int(mission_plan_id))
            if not self._next_collab_detail_matches_pending(
                mission_plan_id=int(mission_plan_id),
                pending=pending_snapshot,
                detail=detail,
            ):
                failure_reason = "0903 plan detail does not match the pending 0803 transition"
                return
            from modules.sim.mission.mission_plan_loader import build_mission_plan_payload

            target_input_id = _to_int(
                detail.get("targetInputMissionID") if isinstance(detail, dict) else None,
                None,
            )
            if target_input_id is None or target_input_id <= 0:
                failure_reason = "next-collab detail has no valid targetInputMissionID"
                return
            plan_result = build_mission_plan_payload(int(mission_plan_id))
            known_aircraft_ids = {
                int(aid)
                for source_key in ("currentInputs", "expectedInputs")
                for aid in dict(pending_snapshot.get(source_key) or {})
                if _to_int(aid, None) is not None and int(aid) > 0
            }
            entry_aircraft_ids = {
                int(aid)
                for row in list(detail.get("entryAircraftList") or [])
                if isinstance(row, dict)
                for aid in [_to_int(row.get("aircraftID"), None)]
                if aid is not None and int(aid) > 0
            }
            # Only aircraft participating in this collaborative entry are
            # authoritative for the target-start check.  Requiring unrelated
            # LAHs to expose the same first active input can reject a valid UAV
            # transition.
            expected_aircraft_ids = entry_aircraft_ids & known_aircraft_ids
            if not expected_aircraft_ids:
                expected_aircraft_ids = known_aircraft_ids
            if not expected_aircraft_ids:
                failure_reason = "pending transition has no known aircraft"
                return
            if not self._next_collab_plan_result_ready(
                plan_result,
                target_input_id=target_input_id,
                expected_aircraft_ids=expected_aircraft_ids,
            ):
                failure_reason = "0903 artifacts do not expose the target as the first executable mission"
                return
            expected_after_load = {
                int(aid): int(target_input_id) for aid in expected_aircraft_ids
            }
            payload = dict(plan_result.get("payload") or {})
            payload["missionPlanID"] = int(mission_plan_id)
            payload["preserveState"] = True
            payload["startInputMissionID"] = int(target_input_id)

            mission_load_lock = getattr(sim, "_mission_load_lock", None)
            load_context = mission_load_lock if mission_load_lock is not None else threading.RLock()
            with load_context:
                with self._lock:
                    current = self._pending_next_collab_transition
                    if (
                        not isinstance(current, dict)
                        or int(current.get("token") or 0) != int(token)
                    ):
                        return
                result = sim.load_mission(payload)
            if not result.get("ok"):
                failure_reason = str(result.get("error") or "SIM mission load failed")
                return
            applied = self._sim_inputs_match_expected(expected_after_load)
            if not applied:
                failure_reason = "SIM loaded the plan but did not enter targetInputMissionID"
        except Exception as exc:
            failure_reason = f"{type(exc).__name__}: {exc}"
            return
        finally:
            retry_attempt: int | None = None
            with self._lock:
                current = self._pending_next_collab_transition
                if isinstance(current, dict) and int(current.get("token") or 0) == int(token):
                    if applied:
                        self._last_next_collab_transition = {
                            "status": "applied",
                            "sourcePlanID": current.get("sourcePlanID"),
                            "missionPlanID": int(mission_plan_id),
                            "targetInputMissionID": (
                                int(target_input_id)
                                if target_input_id is not None
                                else None
                            ),
                            "attempt": int(attempt_index),
                        }
                        self._pending_next_collab_transition = None
                    else:
                        current["applyInProgress"] = False
                        current["status"] = "waiting-retry"
                        current["lastError"] = str(failure_reason)
                        current["lastPlanID"] = int(mission_plan_id)
                        next_attempt = int(attempt_index) + 1
                        if next_attempt < len(_NEXT_COLLAB_PLAN_APPLY_RETRY_DELAYS):
                            retry_attempt = next_attempt
                        else:
                            # A 0903 notification is one-shot.  Keep polling
                            # its exact plan at the slowest interval until the
                            # artifacts settle or the transition is canceled.
                            retry_attempt = len(_NEXT_COLLAB_PLAN_APPLY_RETRY_DELAYS) - 1
            if retry_attempt is not None:
                self._schedule_next_collab_apply_attempt(
                    token,
                    mission_plan_id,
                    retry_attempt,
                )

    def _capture_aircraft_ids(self) -> list[int]:
        sim = self._sim_service
        if sim is None:
            return []
        aircraft_ids: set[int] = set()
        lock = getattr(sim, "_lock", None)
        context = lock if lock is not None else threading.RLock()
        with context:
            aircraft_ids.update(int(aid) for aid in (getattr(sim, "input_mission_order_by_aircraft", {}) or {}).keys())
            for simv in (getattr(sim, "vehicles", {}) or {}).values():
                try:
                    aid = int(getattr(simv, "aircraft_id", 0) or 0)
                except Exception:
                    continue
                if aid > 0:
                    aircraft_ids.add(aid)
        return sorted(aid for aid in aircraft_ids if aid > 0)

    def _current_reexecute_input_id(self) -> int | None:
        sim = self._sim_service
        if sim is None:
            return None
        candidates: list[tuple[int, int]] = []
        lock = getattr(sim, "_lock", None)
        context = lock if lock is not None else threading.RLock()
        with context:
            aircraft_ids = self._capture_aircraft_ids()
            preferred = [4, 5, 6, 1, 2, 3]
            ordered_ids = preferred + [aid for aid in aircraft_ids if aid not in preferred]
            for order, aid in enumerate(ordered_ids):
                try:
                    current_id = sim._current_input_mission_id_for(int(aid))
                except Exception:
                    current_id = None
                if current_id is not None:
                    candidates.append((order, int(current_id)))
        if not candidates:
            return None
        counts: dict[int, int] = {}
        first_order: dict[int, int] = {}
        for order, input_id in candidates:
            counts[input_id] = counts.get(input_id, 0) + 1
            first_order.setdefault(input_id, order)
        return sorted(counts.keys(), key=lambda mid: (-counts[mid], first_order.get(mid, 999)))[0]

    def _latest_input_package_id_from_db(self) -> int | None:
        try:
            from modules.common import db_paths

            input_dir = db_paths.get_active_db_root() / "InputMissionPlan"
            ids: list[int] = []
            for path in input_dir.glob("*.json"):
                try:
                    ids.append(int(path.stem))
                except Exception:
                    continue
            return int(max(ids)) if ids else None
        except Exception:
            return None

    def _extract_input_package_id(self, payload: Any) -> int | None:
        body = _safe_json(payload)
        if not isinstance(body, dict):
            return None
        return _to_int(_pick_ci(body, "inputMissionPackageID", "InputMissionPackageID"), None)

    def _resolve_reexecute_source_package_id(self) -> int | None:
        candidates: list[int] = []
        with self._lock:
            payloads = [
                self.tx_payload.get("0201"),
                self.rx_payload.get("0201"),
            ]
        for payload in payloads:
            package_id = self._extract_input_package_id(payload)
            if package_id is not None and package_id > 0:
                candidates.append(int(package_id))
        db_package_id = self._latest_input_package_id_from_db()
        if db_package_id is not None and db_package_id > 0:
            candidates.append(int(db_package_id))
        return int(max(candidates)) if candidates else None

    def _send_reexecute_0201(self, package_id: int | None) -> None:
        if package_id is None or int(package_id) <= 0:
            return
        body = {
            "timestamp": _now_ms_2000(),
            "inputMissionPackageID": int(package_id),
        }
        self._send_once("0201", body)

    def _handle_0803_reexecute_local(self) -> dict[int, int]:
        current_input_id = self._current_reexecute_input_id()
        source_package_id = self._resolve_reexecute_source_package_id()
        try:
            from modules.common.reexecute_input_clone import clone_current_input_for_reexecute

            result = clone_current_input_for_reexecute(
                current_input_id=current_input_id,
                source_package_id=source_package_id,
                now_ms=_now_ms_2000,
            )
        except Exception:
            return {}
        if not result.ok:
            return {}
        if result.db_root is not None:
            try:
                os.environ["KU_MISSION_DB_ROOT"] = str(result.db_root)
            except Exception:
                pass
        timer = threading.Timer(0.15, self._send_reexecute_0201, args=(result.new_package_id,))
        timer.daemon = True
        timer.start()
        if result.new_input_id is None:
            return {}
        return {aid: int(result.new_input_id) for aid in self._capture_aircraft_ids()}

    def _align_sim_to_expected_inputs(self, expected: dict[int, int]) -> int:
        sim = self._sim_service
        if sim is None or not expected:
            return 0
        advanced = 0
        lock = getattr(sim, "_lock", None)
        context = lock if lock is not None else threading.RLock()
        with context:
            advance_one = getattr(sim, "_advance_input_mission_for_aircraft_locked", None)
            if not callable(advance_one):
                return 0
            steps_by_aircraft: dict[int, int] = {}
            for aid, expected_id in expected.items():
                try:
                    order = [
                        int(item)
                        for item in list(
                            (sim.input_mission_order_by_aircraft or {}).get(int(aid)) or []
                        )
                    ]
                    current_id = sim._current_input_mission_id_for(int(aid))
                except Exception:
                    return 0
                if current_id is None or int(current_id) not in order or int(expected_id) not in order:
                    return 0
                current_idx = order.index(int(current_id))
                expected_idx = order.index(int(expected_id))
                if current_idx > expected_idx or expected_idx - current_idx > 4:
                    return 0
                steps_by_aircraft[int(aid)] = int(expected_idx - current_idx)

            for aid, step_count in steps_by_aircraft.items():
                for _ in range(step_count):
                    try:
                        advanced += int(
                            bool(advance_one(int(aid), allow_queue=True))
                        )
                    except Exception:
                        return int(advanced)
        return int(advanced)

    def _refresh_sim_mission_after_0803(
        self,
        expected: dict[int, int],
        reason: str,
        min_plan_id: int | None = None,
    ) -> None:
        sim = self._sim_service
        if sim is None:
            return
        plan_id = self._resolve_current_mission_plan_id(prefer_loaded=False)
        if plan_id is None:
            return
        if min_plan_id is not None and int(plan_id) <= int(min_plan_id):
            return
        try:
            from modules.sim.mission.mission_plan_loader import build_mission_plan_payload

            plan_result = build_mission_plan_payload(int(plan_id))
            if not plan_result.get("ok"):
                return
            payload = dict(plan_result.get("payload") or {})
            payload["missionPlanID"] = int(plan_id)
            payload["preserveState"] = True
            payload["skipIfMissionPlanAlreadyLoaded"] = True
            result = sim.load_mission(payload)
            if result.get("ok"):
                self._align_sim_to_expected_inputs(expected)
        except Exception:
            return

    def _schedule_0803_mission_refresh(
        self,
        expected: dict[int, int],
        delays: tuple[tuple[float, str], ...] | None = None,
        min_plan_id: int | None = None,
    ) -> None:
        if delays is None:
            delays = ((0.25, "0803-fast"), (1.5, "0803-settle"))
        for delay, reason in delays:
            timer = threading.Timer(
                delay,
                self._refresh_sim_mission_after_0803,
                args=(dict(expected), reason, min_plan_id),
            )
            timer.daemon = True
            timer.start()

    def _0803_key(self, body: dict[str, Any]) -> tuple[int, int | None, str]:
        timestamp = self._extract_timestamp(body)
        if timestamp is None:
            timestamp = id(body)
        return (
            int(timestamp),
            _to_int(_pick_ci(body, "execute", "Execute"), None),
            str(_pick_ci(body, "source", "Source") or ""),
        )

    def _mark_0803_handled(self, body: dict[str, Any]) -> bool:
        key = self._0803_key(body)
        with self._lock:
            if key in self._handled_0803_keys:
                return False
            self._handled_0803_keys.append(key)
        return True

    def _on_receive(self, msg_id: str, payload: object) -> None:
        msg_id = str(msg_id).zfill(4)
        with self._lock:
            self.rx_payload[msg_id] = _safe_json(payload)
            self._append_log("RECV", msg_id, payload)
            observed_at = self._record_time(self.rx_times, msg_id, self.rx_counts)
        self._record_payload_observation(msg_id, "rx", payload, observed_at)
        if msg_id == "0903":
            mission_plan_id = self._extract_mission_plan_id(payload)
            if mission_plan_id is not None:
                if self.has_pending_next_collab_transition():
                    self._schedule_next_collab_plan_apply(int(mission_plan_id))
                else:
                    self._schedule_direct_plan_apply(int(mission_plan_id))
        if msg_id == "0803":
            self._handle_0803(payload)

    def _handle_0803(self, payload: object) -> None:
        sim = self._sim_service
        if sim is None:
            return
        try:
            body = _safe_json(payload)
            if not isinstance(body, dict):
                return
            execute = None
            for key in ("execute", "Execute"):
                if key in body:
                    try:
                        execute = int(body[key])
                    except Exception:
                        pass
                    break
            if execute not in (1, 2):
                return
            if not self._mark_0803_handled(body):
                return
            if execute == 1:
                current, expected = self._capture_0803_transition_inputs_and_hold()
                source_plan_id = self._resolve_current_mission_plan_id()
                # 0803 is a planning request, not permission to release the
                # controller into the old plan's next input mission.  Keep the
                # current terminal turn/hold until the exact new 0903 plan is
                # validated and loaded.
                self._begin_next_collab_transition(
                    expected=expected,
                    current=current,
                    source_plan_id=source_plan_id,
                    request_body=body,
                )
            elif execute == 2:
                self._cancel_next_collab_transition(wait_for_apply=True)
                sim.clear_pending_input_advances()
                before_plan_id = self._resolve_current_mission_plan_id()
                expected = self._handle_0803_reexecute_local()
                self._schedule_0803_mission_refresh(
                    expected,
                    delays=(
                        (1.0, "0803-repeat-fast"),
                        (3.0, "0803-repeat-settle"),
                        (6.0, "0803-repeat-late"),
                    ),
                    min_plan_id=before_plan_id,
                )
        except Exception:
            pass

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
            transition = None
            source_transition = (
                self._pending_next_collab_transition
                if isinstance(self._pending_next_collab_transition, dict)
                else getattr(self, "_last_next_collab_transition", None)
            )
            if isinstance(source_transition, dict):
                transition = {
                    key: (sorted(value) if isinstance(value, set) else value)
                    for key, value in source_transition.items()
                    if key != "requestedAtMonotonic"
                }
            return {
                "ok": True,
                "enabled": self.enabled,
                "error": self.error,
                "tx": tx_rows,
                "rx": rx_rows,
                "logs": {"tx": list(self.tx_log), "rx": list(self.rx_log)},
                "nextCollabTransition": transition,
                "timestamp": _now_ms_2000(),
            }

    def reset_state(self) -> dict:
        self._cancel_next_collab_transition(wait_for_apply=True)
        with self._lock:
            self._direct_plan_apply_seq += 1
            self._pending_direct_plan_apply = None
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
            self._next_collab_transition_seq += 1
            self._pending_next_collab_transition = None
            self._direct_plan_apply_seq += 1
            self._pending_direct_plan_apply = None
            for sender in list(self._periodic_senders.values()):
                try:
                    sender.stop()
                except Exception:
                    pass
            self._periodic_senders.clear()
