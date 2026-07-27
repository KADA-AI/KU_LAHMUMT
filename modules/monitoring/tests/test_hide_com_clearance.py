from __future__ import annotations

import numpy as np

from modules.monitoring.logic.dem_cover.config import CoverConfig
from modules.monitoring.logic.dem_cover.hide_com import _required_target_altitude_batch


def test_required_altitude_matches_shared_masking_clearance_policy() -> None:
    # terrain_los treats clearance as masking tolerance: terrain blocks only
    # above ray + clearance.  At halfway this therefore requires 180 m:
    # 100 + ((150 - 10) - 100) / 0.5 = 180.
    terrain = np.array([[100.0, 150.0, 100.0]], dtype=np.float64)
    result = _required_target_altitude_batch(
        terrain,
        observer_row=0.0,
        observer_col=0.0,
        observer_altitude_m=100.0,
        target_rows=np.array([0.0], dtype=np.float64),
        target_cols=np.array([2.0], dtype=np.float64),
        horizontal_distances_m=np.array([20.0], dtype=np.float64),
        config=CoverConfig(
            los_clearance_m=10.0,
            earth_curvature=False,
            los_samples_per_cell=1.0,
            los_max_steps=8,
        ),
        steps_override=3,
    )

    assert result.tolist() == [180.0]
