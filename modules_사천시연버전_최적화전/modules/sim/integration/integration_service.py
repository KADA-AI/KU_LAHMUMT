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
    fusion_runtime_working_dir,
)
from modules.common.push_center import push_message
from modules.common.receive_center import register_listener
from modules.sim.config import SIM_0401_IDLE_HZ


_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_DEFAULT_MIDDLEWARE_SETTINGS = {
    "Name": "AVS1",
    "NetworkAddress": "203",
    "LocalDomain": 10,
    "ExternalDomain": 100,
}
_PAYLOAD_OBSERVATION_IDS = {"0902", "0305"}
_PAYLOAD_OBSERVATION_MAX_RECORDS = 240


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
        self._messenger = None
        self._init_bus()

    def set_sim_service(self, sim) -> None:
        self._sim_service = sim

    def _project_root(self) -> Path:
        return Path(__file__).resolve().parents[3]

    def _settings_candidates(self, root: Path) -> list[Path]:
        return list(
            fusion_settings_candidates(
                project_root=root,
                common_dir=root / "modules" / "common",
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
        targets = [root / "settings" / "nFusionSettings.json"]

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
        dst = ensure_fusion_settings_file(project_root=root, common_dir=root / "modules" / "common")
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

            push_message(msg_id, self._messenger, on_done=_on_done, body_dict=body)
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

    def _resolve_current_mission_plan_id(self) -> int | None:
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

    def _capture_0803_expected_inputs(self) -> dict[int, int]:
        sim = self._sim_service
        if sim is None:
            return {}
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
                if target_id is not None:
                    expected[int(aid)] = int(target_id)
        return expected

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
        for aid, expected_id in expected.items():
            for _ in range(4):
                try:
                    order = list((sim.input_mission_order_by_aircraft or {}).get(int(aid)) or [])
                    current_id = sim._current_input_mission_id_for(int(aid))
                except Exception:
                    break
                if current_id is None or int(current_id) == int(expected_id):
                    break
                if int(expected_id) not in [int(item) for item in order]:
                    break
                try:
                    current_idx = [int(item) for item in order].index(int(current_id))
                    expected_idx = [int(item) for item in order].index(int(expected_id))
                except Exception:
                    break
                if current_idx >= expected_idx:
                    break
                try:
                    advanced += int(sim.advance_input_mission(int(aid)) or 0)
                except Exception:
                    break
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
        plan_id = self._resolve_current_mission_plan_id()
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
            payload["preserveState"] = True
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
                expected = self._capture_0803_expected_inputs()
                sim.advance_input_mission()
                self._schedule_0803_mission_refresh(expected)
            elif execute == 2:
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
