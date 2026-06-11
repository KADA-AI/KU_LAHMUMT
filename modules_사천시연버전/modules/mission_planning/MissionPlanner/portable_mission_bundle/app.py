from __future__ import annotations

from pathlib import Path
import os
import sys

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from portable_mission import create_app

app = create_app(ROOT)


if __name__ == "__main__":
    host = os.environ.get("MISSION_APP_HOST", "127.0.0.1")
    port = int(os.environ.get("MISSION_APP_PORT", "8877"))
    app.run(host=host, port=port, debug=False)
