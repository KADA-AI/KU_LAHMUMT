from __future__ import annotations

import json
import os
from pathlib import Path

from modules.common import db_paths
from modules.mission_planning.MissionPlanner.data_def import mission_helpers


def test_dem_usage_log_path_is_resolved_once_per_scenario(monkeypatch, tmp_path: Path) -> None:
    first_root = tmp_path / "Scenario_first" / "SBC3"
    second_root = tmp_path / "Scenario_second" / "SBC3"
    calls: list[Path] = []

    def resolved_subpath(*parts: str) -> Path:
        root = Path(os.environ["KU_MISSION_DB_ROOT"])
        calls.append(root)
        return root.joinpath(*parts)

    monkeypatch.setattr(db_paths, "get_db_subpath", resolved_subpath)
    mission_helpers._dem_usage_log_path_cached.cache_clear()

    monkeypatch.setenv("KU_MISSION_DB_ROOT", str(first_root))
    first = mission_helpers._dem_usage_log_path()
    repeated = mission_helpers._dem_usage_log_path()

    monkeypatch.setenv("KU_MISSION_DB_ROOT", str(second_root))
    second = mission_helpers._dem_usage_log_path()

    assert first == first_root / "DSS_Internal" / "dem_usage.jsonl"
    assert repeated == first
    assert second == second_root / "DSS_Internal" / "dem_usage.jsonl"
    assert calls == [first_root, second_root]

    mission_helpers._dem_usage_log_path_cached.cache_clear()


def test_dem_miss_audit_names_required_file_and_unregistered_tiff(
    monkeypatch,
    tmp_path: Path,
) -> None:
    resource_dir = tmp_path / "resource"
    resource_dir.mkdir()
    for name in ("Hongik_48km.tif", "Jipo_48km.tif", "Inje_48km.tif"):
        (resource_dir / name).touch()
    log_path = tmp_path / "dem_usage.jsonl"

    monkeypatch.setattr(mission_helpers, "_DEM_DIR", resource_dir)
    monkeypatch.setattr(mission_helpers, "_dem_usage_log_path", lambda: log_path)
    mission_helpers._DEM_MISS_LOGGED_KEYS.clear()

    # An unrelated out-of-coverage miss must not suppress the later Inje miss.
    mission_helpers._log_dem_miss_once(37.5, 127.8)
    mission_helpers._log_dem_miss_once(37.721709, 128.1074691)
    mission_helpers._log_dem_miss_once(37.721709, 128.1074691)

    events = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
    assert len(events) == 2
    outside, inje = events
    assert outside["reason"] == "coordinate_outside_operational_coverage"
    assert outside["expectedDemName"] is None
    assert inje["event"] == "dem_source_missing"
    assert inje["reason"] == "required_dem_file_missing"
    assert inje["expectedDemName"] == "Inje_10m.tif"
    assert inje["expectedDemPath"] == str(resource_dir / "Inje_10m.tif")
    assert inje["availableDemNames"] == ["Hongik_48km.tif", "Jipo_48km.tif"]
    assert "Inje_48km.tif" in inje["detectedTifNames"]
    assert inje["unregisteredTifNames"] == ["Inje_48km.tif"]

    mission_helpers._DEM_MISS_LOGGED_KEYS.clear()


def test_dem_warmup_reports_missing_dem_required_by_input_points(
    monkeypatch,
    tmp_path: Path,
) -> None:
    resource_dir = tmp_path / "resource"
    resource_dir.mkdir()
    for name in ("Hongik_48km.tif", "Jipo_48km.tif", "Inje_48km.tif"):
        (resource_dir / name).touch()
    log_path = tmp_path / "dem_usage.jsonl"

    monkeypatch.setattr(mission_helpers, "_DEM_DIR", resource_dir)
    monkeypatch.setattr(mission_helpers, "_dem_usage_log_path", lambda: log_path)
    monkeypatch.setattr(mission_helpers, "_available_dem_tiles", lambda: ())
    monkeypatch.setattr(mission_helpers, "terrain_elev_many", lambda coords: [0.0 for _ in coords])
    monkeypatch.setattr(mission_helpers, "terrain_cache_info", lambda: {"terrain_pixel": {"currsize": 0}})
    mission_helpers._DEM_MISS_LOGGED_KEYS.clear()

    info = mission_helpers.warm_terrain_cache([(37.721709, 128.1074691)])
    warmup = info["warmup"]

    assert warmup["requestedDemNames"] == ["Inje_10m.tif"]
    assert warmup["missingRequiredDemNames"] == ["Inje_10m.tif"]
    assert warmup["missingRequiredPointCount"] == 1
    assert warmup["unresolvedRequirementPointCount"] == 1
    assert warmup["requestedCoverageReady"] is False
    assert warmup["unregisteredTifNames"] == ("Inje_48km.tif",)
    events = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
    assert [event["event"] for event in events] == [
        "dem_inventory_incomplete",
        "dem_source_missing",
    ]

    mission_helpers._DEM_MISS_LOGGED_KEYS.clear()
