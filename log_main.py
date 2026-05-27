from __future__ import annotations

import os
import socket
import sys
import time
import traceback

from modules.common.process_console import ensure_console, install_process_file_logging

ensure_console(os.getenv("KU_CONSOLE_TITLE", "KU Log Analyzer Console"))
install_process_file_logging("log_analyzer")


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
    print(f"[log-analyzer] bootstrap start: python={sys.executable}")
    from modules.logAnalyzer.config import SERVER_HOST, SERVER_PORT, resolve_logs_dir, resolve_server_binding
    from modules.logAnalyzer.server import LogAnalyzerServer

    host, port = resolve_server_binding(SERVER_HOST, SERVER_PORT)
    server = LogAnalyzerServer(host=host, port=port)
    server.start()
    print(f"[log-analyzer] logs directory: {resolve_logs_dir()}")
    print(f"[log-analyzer] web UI running at http://127.0.0.1:{port}/")
    if host in ("0.0.0.0", "::"):
        for ip in _collect_local_ipv4_addresses():
            if ip.startswith("127."):
                continue
            print(f"[log-analyzer] LAN access: http://{ip}:{port}/")
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
        print(f"[log-analyzer ERR] startup failed: missing dependency '{missing_name}'", file=sys.stderr)
        print(
            f"[log-analyzer HINT] install with: \"{sys.executable}\" -m pip install {missing_name}",
            file=sys.stderr,
        )
        traceback.print_exc()
        raise
    except Exception as exc:
        print(f"[log-analyzer ERR] startup failed: {exc}", file=sys.stderr)
        traceback.print_exc()
        raise
