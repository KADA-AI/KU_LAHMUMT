from __future__ import annotations

from pathlib import Path

import pytest
import rasterio
from rasterio.warp import transform, transform_bounds

from modules.common.regional_dem import (
    REGIONAL_DEM_FILENAMES,
    REGIONAL_DEM_SPECS,
    regional_dem_inventory,
    regional_dem_path_for_coordinate,
    regional_dem_paths,
    select_regional_dem,
)
from modules.mission_planning.MissionPlanner.data_def import mission_helpers
from modules.sim.map.dem_tiles import DemTileProvider


PROJECT_ROOT = Path(__file__).resolve().parents[3]
RESOURCE_DIR = PROJECT_ROOT / "resource"


@pytest.mark.parametrize("spec", REGIONAL_DEM_SPECS)
def test_registered_bounds_match_geotiff_metadata(spec) -> None:
    path = RESOURCE_DIR / spec.filename
    with rasterio.open(path) as dataset:
        west, south, east, north = transform_bounds(
            dataset.crs,
            "EPSG:4326",
            *dataset.bounds,
            densify_pts=21,
        )

    assert spec.epsg == 32652
    assert (spec.west, spec.south, spec.east, spec.north) == pytest.approx(
        (west, south, east, north),
        abs=1e-9,
    )


@pytest.mark.parametrize("spec", REGIONAL_DEM_SPECS)
def test_shared_sampler_selects_and_reads_the_registered_dem(spec) -> None:
    latitude = (spec.south + spec.north) * 0.5
    longitude = (spec.west + spec.east) * 0.5
    path = RESOURCE_DIR / spec.filename

    selected = select_regional_dem(latitude, longitude)
    assert selected is not None
    assert selected.filename == spec.filename
    assert regional_dem_path_for_coordinate(RESOURCE_DIR, latitude, longitude) == path
    assert mission_helpers.terrain_source_name(latitude, longitude) == spec.filename

    with rasterio.open(path) as dataset:
        xs, ys = transform("EPSG:4326", dataset.crs, [longitude], [latitude])
        expected = float(next(dataset.sample([(xs[0], ys[0])]))[0])
    assert mission_helpers.terrain_elev(latitude, longitude) == pytest.approx(
        expected,
        abs=1.0,
    )


def test_runtime_dem_discovery_excludes_legacy_degree_tiles() -> None:
    expected = tuple(REGIONAL_DEM_FILENAMES)
    assert tuple(path.name for path in regional_dem_paths(RESOURCE_DIR)) == expected
    assert tuple(path.name for path, _bounds in mission_helpers._available_dem_tiles()) == expected

    provider = DemTileProvider(RESOURCE_DIR)
    assert provider.source_names == expected


def test_outside_operational_coverage_is_not_silently_mapped_to_another_dem() -> None:
    latitude = 37.5
    longitude = 127.8
    assert select_regional_dem(latitude, longitude) is None
    assert mission_helpers.terrain_source_name(latitude, longitude) is None
    assert mission_helpers.terrain_elev(latitude, longitude) == 0.0


def test_inventory_exposes_legacy_inje_filename_as_unregistered(tmp_path: Path) -> None:
    for name in ("Hongik_48km.tif", "Jipo_48km.tif", "Inje_48km.tif"):
        (tmp_path / name).touch()

    inventory = regional_dem_inventory(tmp_path)

    assert inventory["availableDemNames"] == ("Hongik_48km.tif", "Jipo_48km.tif")
    assert inventory["missingDemNames"] == ("Inje_10m.tif",)
    assert "Inje_48km.tif" in inventory["detectedTifNames"]
    assert inventory["unregisteredTifNames"] == ("Inje_48km.tif",)
