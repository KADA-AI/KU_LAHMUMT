from __future__ import annotations

import os
import socket
import sys
import time
import traceback
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_PROJECT_ROOT_TEXT = str(_PROJECT_ROOT)
if _PROJECT_ROOT_TEXT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT_TEXT)

from modules.common.process_console import ensure_console, install_process_file_logging

ensure_console(os.getenv("KU_CONSOLE_TITLE", "KU Simulation Console"))
# 로그 파일은 대기모드 진입 후 DB 경로가 확정된 뒤에 쌓도록 지연 설치

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


def main() -> None:
    print(f"[sim] bootstrap start: python={sys.executable}")
    from modules.common import db_paths
    from modules.sim.config import SERVER_HOST, SERVER_PORT, resolve_server_binding
    from modules.sim.server.http_server import MapServer

    db_root = db_paths.bootstrap_db_root()
    db_paths.ensure_env_watch()
    host, port = resolve_server_binding(SERVER_HOST, SERVER_PORT)
    server = MapServer(host=host, port=port)
    server.start()
    install_process_file_logging("simulation")
    print(f"[sim] active db root: {db_root}")
    print(f"[sim] current_scenario.json: {db_paths.INFO_PATH}")
    print(f"[sim] web map running at http://127.0.0.1:{port}/")
    if host in ("0.0.0.0", "::"):
        for ip in _collect_local_ipv4_addresses():
            if ip.startswith("127."):
                continue
            print(f"[sim] LAN access: http://{ip}:{port}/")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    try:
        main()
    except ModuleNotFoundError as exc:
        missing_name = str(getattr(exc, "name", "") or "").strip() or "unknown"
        print(f"[sim ERR] startup failed: missing dependency '{missing_name}'", file=sys.stderr)
        print(
            f"[sim HINT] install with: \"{sys.executable}\" -m pip install {missing_name}",
            file=sys.stderr,
        )
        traceback.print_exc()
        raise
    except Exception as exc:
        print(f"[sim ERR] startup failed: {exc}", file=sys.stderr)
        traceback.print_exc()
        raise
