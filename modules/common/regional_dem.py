"""Canonical regional DEM registry shared by planning, monitoring, and SIM.

The operational scenarios use three projected GeoTIFFs under ``resource/``.
Keeping their WGS84 footprints in one place prevents a subsystem from silently
falling back to a lower-resolution degree tile or choosing a raster by filename
ordering alone.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class RegionalDemSpec:
    filename: str
    epsg: int
    west: float
    south: float
    east: float
    north: float
    resolution_m: float

    def contains(self, latitude: float, longitude: float) -> bool:
        lat = float(latitude)
        lon = float(longitude)
        return self.west <= lon <= self.east and self.south <= lat <= self.north

    @property
    def latlon_bounds(self) -> tuple[float, float, float, float]:
        """Return ``(min_lat, max_lat, min_lon, max_lon)``."""

        return (self.south, self.north, self.west, self.east)


# Bounds were transformed from each GeoTIFF's native EPSG:32652 outer bounds
# to EPSG:4326 with densified edges. The 10 m Inje raster is first so that a
# future overlap always resolves to the highest-resolution operational source.
REGIONAL_DEM_SPECS: tuple[RegionalDemSpec, ...] = (
    RegionalDemSpec(
        filename="Inje_10m.tif",
        epsg=32652,
        west=127.99106913570418,
        south=37.62045612866306,
        east=128.55421143095074,
        north=38.065236393917395,
        resolution_m=10.0,
    ),
    RegionalDemSpec(
        filename="Hongik_48km.tif",
        epsg=32652,
        west=126.79612248582086,
        south=36.925432261442026,
        east=127.35105864035538,
        north=37.36832595536616,
        resolution_m=30.0,
    ),
    RegionalDemSpec(
        filename="Jipo_48km.tif",
        epsg=32652,
        west=127.13058826610084,
        south=37.836638564862106,
        east=127.6906709761248,
        north=38.27814749239764,
        resolution_m=30.0,
    ),
)

REGIONAL_DEM_FILENAMES: tuple[str, ...] = tuple(
    spec.filename for spec in REGIONAL_DEM_SPECS
)
REGIONAL_DEM_EPSG_BY_NAME: dict[str, int] = {
    spec.filename.lower(): int(spec.epsg) for spec in REGIONAL_DEM_SPECS
}


def select_regional_dem(latitude: float, longitude: float) -> RegionalDemSpec | None:
    """Select the highest-priority operational DEM containing a WGS84 point."""

    for spec in REGIONAL_DEM_SPECS:
        if spec.contains(latitude, longitude):
            return spec
    return None


def regional_dem_paths(resource_dir: str | Path) -> tuple[Path, ...]:
    """Return existing operational DEM paths in canonical priority order."""

    root = Path(resource_dir)
    return tuple(
        root / spec.filename
        for spec in REGIONAL_DEM_SPECS
        if (root / spec.filename).is_file()
    )


def regional_dem_inventory(resource_dir: str | Path) -> dict[str, str | tuple[str, ...]]:
    """Describe the expected and actually present operational DEM files.

    Only the three registered filenames are usable by production terrain
    selection.  ``detectedTifNames`` deliberately includes other TIFFs so a
    deployment typo such as ``Inje_48km.tif`` is visible in diagnostics rather
    than being mistaken for a missing Python/GDAL dependency.
    """

    root = Path(resource_dir)
    expected_paths = tuple(root / name for name in REGIONAL_DEM_FILENAMES)
    available_paths = tuple(path for path in expected_paths if path.is_file())
    missing_paths = tuple(path for path in expected_paths if not path.is_file())
    try:
        detected_paths = tuple(
            sorted(
                (path for path in root.glob("*.tif") if path.is_file()),
                key=lambda path: path.name.lower(),
            )
        )
    except Exception:
        detected_paths = ()
    expected_names_lower = {name.lower() for name in REGIONAL_DEM_FILENAMES}
    unregistered_paths = tuple(
        path for path in detected_paths if path.name.lower() not in expected_names_lower
    )

    return {
        "resourceDir": str(root),
        "expectedDemNames": tuple(path.name for path in expected_paths),
        "expectedDemPaths": tuple(str(path) for path in expected_paths),
        "availableDemNames": tuple(path.name for path in available_paths),
        "availableDemPaths": tuple(str(path) for path in available_paths),
        "missingDemNames": tuple(path.name for path in missing_paths),
        "missingDemPaths": tuple(str(path) for path in missing_paths),
        "detectedTifNames": tuple(path.name for path in detected_paths),
        "detectedTifPaths": tuple(str(path) for path in detected_paths),
        "unregisteredTifNames": tuple(path.name for path in unregistered_paths),
        "unregisteredTifPaths": tuple(str(path) for path in unregistered_paths),
    }


def regional_dem_path_for_coordinate(
    resource_dir: str | Path,
    latitude: float,
    longitude: float,
) -> Path | None:
    spec = select_regional_dem(latitude, longitude)
    if spec is None:
        return None
    path = Path(resource_dir) / spec.filename
    return path if path.is_file() else None


def regional_dem_specs_for_paths(paths: Iterable[str | Path]) -> tuple[RegionalDemSpec, ...]:
    names = {Path(path).name.lower() for path in paths}
    return tuple(
        spec for spec in REGIONAL_DEM_SPECS if spec.filename.lower() in names
    )


__all__ = [
    "REGIONAL_DEM_EPSG_BY_NAME",
    "REGIONAL_DEM_FILENAMES",
    "REGIONAL_DEM_SPECS",
    "RegionalDemSpec",
    "regional_dem_path_for_coordinate",
    "regional_dem_inventory",
    "regional_dem_paths",
    "regional_dem_specs_for_paths",
    "select_regional_dem",
]
