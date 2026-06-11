from __future__ import annotations

import json
import socket
import subprocess
from functools import lru_cache
from pathlib import Path
from typing import Any

from modules.common.settings_paths import fusion_settings_candidates


PROJECT_ROOT = Path(__file__).resolve().parents[2]
COMMON_DIR = Path(__file__).resolve().parent


def _candidate_config_paths() -> tuple[Path, ...]:
    return fusion_settings_candidates(project_root=PROJECT_ROOT, common_dir=COMMON_DIR)


def load_fusion_settings() -> dict:
    for path in _candidate_config_paths():
        try:
            if path.exists():
                return json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
    return {}


def get_fusion_network_address() -> str:
    settings = load_fusion_settings()
    middleware = settings.get("Middleware") if isinstance(settings, dict) else {}
    if not isinstance(middleware, dict):
        return ""
    return str(middleware.get("NetworkAddress") or "").strip()


def _normalize_ipv4(value: str) -> str:
    text = str(value or "").strip()
    parts = text.split(".")
    if len(parts) != 4:
        return ""
    normalized: list[str] = []
    for part in parts:
        if not part.isdigit():
            return ""
        number = int(part)
        if number < 0 or number > 255:
            return ""
        normalized.append(str(number))
    return ".".join(normalized)


def _normalize_ipv4_prefix(value: str) -> str:
    text = str(value or "").strip()
    while text.endswith("."):
        text = text[:-1]
    if not text:
        return ""
    parts = text.split(".")
    if len(parts) < 1 or len(parts) > 3:
        return ""
    normalized: list[str] = []
    for part in parts:
        if not part.isdigit():
            return ""
        number = int(part)
        if number < 0 or number > 255:
            return ""
        normalized.append(str(number))
    return ".".join(normalized) + "."


def _append_unique(target: list[str], value: str) -> None:
    normalized = _normalize_ipv4(value)
    if not normalized or normalized in target:
        return
    target.append(normalized)


def _powershell_ipv4_addresses() -> list[str]:
    try:
        result = subprocess.run(
            [
                "powershell",
                "-NoProfile",
                "-Command",
                "Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | "
                "Select-Object -ExpandProperty IPAddress",
            ],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="ignore",
            timeout=3,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            check=False,
        )
    except Exception:
        return []
    found: list[str] = []
    for line in (result.stdout or "").splitlines():
        _append_unique(found, line.strip())
    return found


def _powershell_json(command: str) -> Any:
    try:
        result = subprocess.run(
            [
                "powershell",
                "-NoProfile",
                "-Command",
                command,
            ],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="ignore",
            timeout=5,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            check=False,
        )
    except Exception:
        return None
    text = (result.stdout or "").strip()
    if not text:
        return None
    try:
        return json.loads(text)
    except Exception:
        return None


def _as_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    return [value]


def _to_int(value: Any) -> int | None:
    try:
        return int(str(value).strip())
    except Exception:
        return None


def _powershell_ipv4_details() -> list[dict[str, Any]]:
    data = _powershell_json(
        "Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | "
        "Where-Object { $_.IPAddress -notlike '127.*' } | "
        "Select-Object InterfaceIndex,InterfaceAlias,IPAddress,PrefixLength,AddressState,SkipAsSource | "
        "ConvertTo-Json -Compress"
    )
    rows: list[dict[str, Any]] = []
    for item in _as_list(data):
        if not isinstance(item, dict):
            continue
        ip = _normalize_ipv4(item.get("IPAddress"))
        if not ip:
            continue
        row = dict(item)
        row["IPAddress"] = ip
        rows.append(row)
    return rows


def _powershell_neighbors() -> list[dict[str, Any]]:
    data = _powershell_json(
        "Get-NetNeighbor -AddressFamily IPv4 -ErrorAction SilentlyContinue | "
        "Select-Object ifIndex,IPAddress,State | "
        "ConvertTo-Json -Compress"
    )
    rows: list[dict[str, Any]] = []
    for item in _as_list(data):
        if not isinstance(item, dict):
            continue
        ip = _normalize_ipv4(item.get("IPAddress"))
        if not ip:
            continue
        row = dict(item)
        row["IPAddress"] = ip
        rows.append(row)
    return rows


def _neighbor_state_score(state: str) -> int:
    scores = {
        "Reachable": 8,
        "Stale": 5,
        "Delay": 4,
        "Probe": 3,
        "Permanent": 0,
        "Unreachable": -2,
        "Incomplete": -1,
    }
    return scores.get(str(state or "").strip(), 0)


def resolve_runtime_fusion_network_address(raw: str, *, default: str = "") -> str:
    exact = _normalize_ipv4(raw)
    if exact:
        return exact

    prefix = _normalize_ipv4_prefix(raw)
    if not prefix:
        return str(default or "")

    local_matches = [
        addr for addr in local_ipv4_addresses(include_loopback=False)
        if addr.startswith(prefix)
    ]
    if len(local_matches) <= 1:
        return local_matches[0] if local_matches else str(default or "")

    details = [
        item for item in _powershell_ipv4_details()
        if str(item.get("IPAddress") or "").startswith(prefix)
    ]
    if not details:
        return local_matches[0]

    by_ip = {
        str(item.get("IPAddress")): item
        for item in details
        if str(item.get("IPAddress") or "")
    }
    neighbors = _powershell_neighbors()
    scores_by_ifindex: dict[int, int] = {}
    positive_by_ifindex: dict[int, int] = {}
    negative_by_ifindex: dict[int, int] = {}

    for item in details:
        ifindex = _to_int(item.get("InterfaceIndex"))
        if ifindex is None:
            continue
        scores_by_ifindex.setdefault(ifindex, 0)
        positive_by_ifindex.setdefault(ifindex, 0)
        negative_by_ifindex.setdefault(ifindex, 0)

    for neighbor in neighbors:
        ip = str(neighbor.get("IPAddress") or "")
        if not ip.startswith(prefix) or ip.endswith(".255"):
            continue
        if ip in by_ip:
            continue
        ifindex = _to_int(neighbor.get("ifIndex"))
        if ifindex is None or ifindex not in scores_by_ifindex:
            continue
        score = _neighbor_state_score(str(neighbor.get("State") or ""))
        scores_by_ifindex[ifindex] += score
        if score > 0:
            positive_by_ifindex[ifindex] += 1
        elif score < 0:
            negative_by_ifindex[ifindex] += 1

    def _candidate_sort_key(item: dict[str, Any]) -> tuple[int, int, int, int, int]:
        ifindex = _to_int(item.get("InterfaceIndex")) or 0
        score = scores_by_ifindex.get(ifindex, 0)
        positives = positive_by_ifindex.get(ifindex, 0)
        negatives = negative_by_ifindex.get(ifindex, 0)
        skip_as_source = 1 if bool(item.get("SkipAsSource")) else 0
        return (score, positives, -negatives, -skip_as_source, -ifindex)

    best = max(details, key=_candidate_sort_key)
    best_ip = _normalize_ipv4(best.get("IPAddress"))
    if best_ip:
        return best_ip
    return local_matches[0]


def materialize_fusion_settings_file(src: Path, dst: Path) -> str:
    if not src.exists():
        raise FileNotFoundError(f"source not found: {src}")

    try:
        settings = json.loads(src.read_text(encoding="utf-8"))
    except Exception:
        data = src.read_bytes()
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(data)
        return ""

    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(
        json.dumps(settings, ensure_ascii=False, indent=4),
        encoding="utf-8",
    )
    try:
        middleware = settings.get("Middleware") if isinstance(settings, dict) else {}
        if isinstance(middleware, dict):
            return str(middleware.get("NetworkAddress") or "").strip()
    except Exception:
        pass
    return ""


@lru_cache(maxsize=1)
def _local_ipv4_addresses_cached() -> tuple[str, ...]:
    found: list[str] = []
    _append_unique(found, "127.0.0.1")
    try:
        hostname = socket.gethostname()
        _, _, addrs = socket.gethostbyname_ex(hostname)
        for addr in addrs:
            _append_unique(found, addr)
    except Exception:
        pass
    try:
        hostname = socket.gethostname()
        infos = socket.getaddrinfo(hostname, None, socket.AF_INET, socket.SOCK_DGRAM)
        for info in infos:
            sockaddr = info[4] if len(info) > 4 else None
            if sockaddr:
                _append_unique(found, sockaddr[0])
    except Exception:
        pass
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        _append_unique(found, sock.getsockname()[0])
    except Exception:
        pass
    finally:
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass
    for addr in _powershell_ipv4_addresses():
        _append_unique(found, addr)
    return tuple(found)


def local_ipv4_addresses(*, include_loopback: bool = True) -> list[str]:
    found = list(_local_ipv4_addresses_cached())
    if include_loopback:
        return found
    return [addr for addr in found if not addr.startswith("127.")]


def _resolve_network_address(default: str) -> str:
    raw = get_fusion_network_address()
    resolved = resolve_runtime_fusion_network_address(raw, default="")
    if resolved:
        return resolved
    exact = _normalize_ipv4(raw)
    if exact:
        return exact
    return str(default)


def resolve_fusion_bind_host(default: str = "127.0.0.1") -> str:
    return _resolve_network_address(default)


def resolve_fusion_target_host(default: str = "127.0.0.1") -> str:
    return _resolve_network_address(default)
