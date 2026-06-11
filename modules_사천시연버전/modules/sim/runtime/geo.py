from __future__ import annotations

import math


class GeoConverter:
    """Lightweight lon/lat <-> local ENU conversion (meters)."""

    def __init__(self, ref_lon: float, ref_lat: float) -> None:
        self.ref_lon = float(ref_lon)
        self.ref_lat = float(ref_lat)
        self.m_per_deg_lat = 111320.0
        self.m_per_deg_lon = math.cos(math.radians(self.ref_lat)) * 111320.0
        if abs(self.m_per_deg_lon) < 1e-6:
            self.m_per_deg_lon = 1e-6

    def lonlat_to_xy(self, lon: float, lat: float) -> tuple[float, float]:
        x = (float(lon) - self.ref_lon) * self.m_per_deg_lon
        y = (float(lat) - self.ref_lat) * self.m_per_deg_lat
        return x, y

    def xy_to_lonlat(self, x: float, y: float) -> tuple[float, float]:
        lon = self.ref_lon + float(x) / self.m_per_deg_lon
        lat = self.ref_lat + float(y) / self.m_per_deg_lat
        return lon, lat
