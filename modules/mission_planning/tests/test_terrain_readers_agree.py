"""Every terrain reader must resolve a coordinate to the same DEM cell.

The inverse of a raster transform yields cell-CORNER indices: an integer sits on
a boundary and a cell centre lands on .5. Rounding those to an index picks the
neighbouring cell half the time. On a slope that is tens of metres of elevation,
and it made the attack solver and the cover analyser disagree about the very
same ground - certifying hide points the enemy could see and firing points the
shot could not clear. Measured on the reported geometry: 20 m apart before,
exact after.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from modules.mission_planning.MissionPlanner.data_def.mission_helpers import terrain_elev

# Points from the reported hide/attack geometry, on sloping ground where a
# one-cell offset shows up clearly.
POINTS = [
    (37.9860072, 127.3023118),
    (37.98763, 127.30227),
    (37.98601, 127.30573),
    (37.98791, 127.30295),
    (37.98550, 127.30220),
    (38.02516, 127.31109),
]
TILE = Path("resource/Jipo_48km.tif")


pytestmark = pytest.mark.skipif(
    not TILE.exists(), reason="operational DEM tile not available"
)


def _dem_grid():
    from modules.monitoring.logic.dem_cover.dem import DemGrid

    # Native resolution: the comparison is about which cell is chosen, not about
    # what down-sampling does to it.
    return DemGrid(TILE, max_dim=4000)


def test_the_cover_analyser_reads_the_same_cell_as_the_planner() -> None:
    grid = _dem_grid()

    for latitude, longitude in POINTS:
        x, y = grid.latlon_to_native(latitude, longitude)
        row, col = grid.native_to_rowcol(x, y)
        row_i = max(0, min(grid.height - 1, math.floor(row)))
        col_i = max(0, min(grid.width - 1, math.floor(col)))

        assert float(grid.elev[row_i, col_i]) == pytest.approx(
            terrain_elev(latitude, longitude), abs=1e-3
        )


def test_the_planner_selects_the_containing_cell_not_the_nearest_corner() -> None:
    """A centre lands on .5; rounding there jumps to the next cell."""

    import rasterio

    from modules.mission_planning.MissionPlanner.data_def import mission_helpers as mh

    tile = TILE.resolve()
    band, transform, _bounds, _nodata = mh._load_dem_data(tile)
    straddling = 0
    for latitude, longitude in POINTS:
        native = mh._lonlat_to_native(tile, longitude, latitude)
        assert native is not None
        col_f, row_f = (~transform) * native
        if abs(row_f - math.floor(row_f) - 0.5) < 1e-3:
            straddling += 1
        row = int(math.floor(row_f))
        col = int(math.floor(col_f))
        assert float(band[row, col]) == pytest.approx(
            terrain_elev(latitude, longitude), abs=1e-3
        )
    # At least one sample sits exactly on the boundary that used to flip.
    assert straddling >= 1


# The simulator's own footprint sampler is held to the same rule by
# modules/sim/tests/test_footprint_dem_sampler.py, which compares it against
# this shared lookup tile by tile.
