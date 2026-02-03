from __future__ import annotations

import time

from modules.sim.config import SERVER_HOST, SERVER_PORT
from modules.sim.server.http_server import MapServer


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
