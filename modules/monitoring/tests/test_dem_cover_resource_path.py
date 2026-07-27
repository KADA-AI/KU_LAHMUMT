from pathlib import Path

import pytest

from modules.monitoring.logic import anti_armor_air_strike_review as review
from modules.monitoring.logic.dem_cover.config import CoverConfig, resolve_repo_root


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def test_dem_cover_uses_project_resource_not_modules_resource() -> None:
    assert resolve_repo_root() == PROJECT_ROOT
    assert CoverConfig().dem_full_path == PROJECT_ROOT / "resource" / "Inje_10m.tif"


def test_type1_review_reports_expected_inje_file_instead_of_zero_coverage_fallback(
    monkeypatch,
    tmp_path: Path,
) -> None:
    resource_dir = tmp_path / "resource"
    resource_dir.mkdir()
    for name in ("Hongik_48km.tif", "Jipo_48km.tif", "Inje_48km.tif"):
        (resource_dir / name).touch()
    config = CoverConfig(dem_path=str(resource_dir / "Inje_10m.tif"))
    monkeypatch.setattr(review, "CoverConfig", lambda: config)

    with pytest.raises(review.AntiArmorReviewError) as raised:
        review._select_dem_config(
            [{"latitude": 37.721709, "longitude": 128.1074691, "altitude": 0}]
        )

    message = str(raised.value)
    assert "required_dem_file_missing" in message
    assert "expectedDemName=Inje_10m.tif" in message
    assert f"expectedDemPath={resource_dir / 'Inje_10m.tif'}" in message
    assert "Inje_48km.tif" in message
    assert "Hongik_48km.tif" in message


def test_type1_review_reports_unreadable_required_dem(monkeypatch, tmp_path: Path) -> None:
    resource_dir = tmp_path / "resource"
    resource_dir.mkdir()
    for name in ("Inje_10m.tif", "Hongik_48km.tif", "Jipo_48km.tif"):
        (resource_dir / name).touch()
    config = CoverConfig(dem_path=str(resource_dir / "Inje_10m.tif"))
    monkeypatch.setattr(review, "CoverConfig", lambda: config)

    with pytest.raises(review.AntiArmorReviewError) as raised:
        review._select_dem_config(
            [{"latitude": 37.721709, "longitude": 128.1074691, "altitude": 0}]
        )

    message = str(raised.value)
    assert "required_dem_read_error" in message
    assert "Inje_10m.tif" in message
    assert "errorType" in message
