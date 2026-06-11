# mission_pipeline.py (상단 util 영역에 추가)

import math

EARTH_RADIUS = 6378137.0       # WGS‑84 적도반경 [m]

def llh_to_xy(lat, lon, lat0, lon0):
    """
    단순 equirectangular (ENU 근사) 변환.
    lat0/lon0: 로컬 원점 [deg]
    반환: (x[ m ], y[ m ])
    """
    d_lat = math.radians(lat - lat0)
    d_lon = math.radians(lon - lon0)
    avg_lat = math.radians((lat + lat0) / 2.0)

    x = EARTH_RADIUS * d_lon * math.cos(avg_lat)
    y = EARTH_RADIUS * d_lat
    return x, y

def xy_to_llh(x, y, lat0, lon0):
    d_lat = y / EARTH_RADIUS
    d_lon = x / (EARTH_RADIUS * math.cos(math.radians(lat0)))
    lat = math.degrees(d_lat) + lat0
    lon = math.degrees(d_lon) + lon0
    return lat, lon
