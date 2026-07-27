from __future__ import annotations

import os
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

from modules.common import db_paths


class ScenarioDbRootGuardTests(unittest.TestCase):
    def test_transient_older_scenario_does_not_rebind_running_process(self) -> None:
        cache_before = deepcopy(db_paths._cache)
        env_before = {
            key: os.environ.get(key)
            for key in (db_paths.ENV_DB_ROOT, db_paths.ENV_SCENARIO_ROOT)
        }
        try:
            with tempfile.TemporaryDirectory() as temp_dir:
                base = Path(temp_dir)
                old_root = base / "Scenario_old" / "SBC3"
                current_root = base / "Scenario_current" / "SBC3"
                old_root.mkdir(parents=True)
                current_root.mkdir(parents=True)
                db_paths._cache.update(
                    {
                        "source": "scenario",
                        "timestamp_ms": 200,
                        "db_root": current_root,
                    }
                )
                os.environ[db_paths.ENV_DB_ROOT] = str(current_root)
                os.environ[db_paths.ENV_SCENARIO_ROOT] = str(current_root.parent)
                stale_info = {
                    "source": "scenario",
                    "timestamp_ms": 100,
                    "db_root": str(old_root),
                }

                with patch.object(db_paths, "_read_info_snapshot", return_value=stale_info):
                    selected = db_paths.peek_active_db_root(existing_only=True)

                self.assertIsNotNone(selected)
                self.assertEqual(selected.resolve(), current_root.resolve())
        finally:
            db_paths._cache.clear()
            db_paths._cache.update(cache_before)
            for key, value in env_before.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value


if __name__ == "__main__":
    unittest.main()
