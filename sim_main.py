from __future__ import annotations

import os
import time

from modules.common.process_console import ensure_console, install_process_file_logging
from modules.sim.config import SERVER_HOST, SERVER_PORT
from modules.sim.server.http_server import MapServer

ensure_console(os.getenv("KU_CONSOLE_TITLE", "KU Simulation Console"))
install_process_file_logging("simulation")


def main() -> None:
    server = MapServer(host=SERVER_HOST, port=SERVER_PORT)
    server.start()
    print(f"[sim] web map running at {server.url}")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
