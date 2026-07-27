"""DEM cover-position analysis — reusable logic package.

Public API (import from here to use the algorithm without any GUI)::

    from modules.monitoring.logic.dem_cover import CoverConfig, DemGrid, CoverAnalyzer, Polygon2D

    dem = DemGrid(CoverConfig().dem_full_path, max_dim=340)
    analyzer = CoverAnalyzer(dem, CoverConfig())
    poly = Polygon2D([(x0, y0), (x1, y1), (x2, y2)])
    enemies = analyzer.sample_enemies(poly)
    ref = analyzer.make_ref(x_ref, y_ref)
    result = analyzer.analyze(poly, enemies, ref)
    print(result.summary())
"""

from __future__ import annotations

from .config import CoverConfig
from .dem import DemGrid, DemMeta
from .geometry import Polygon2D
from .sampling import EnemyPoint, sample_enemy_positions
from .visibility import accumulate_engagement
from .dem_native import NativeDemRaster
from .hide import HideResult, NearestHideAnalyzer, enemy_from_native
from .hide_com import (
    CommunicationHideAnalyzer,
    CommunicationHideResult,
    HideEndpointCandidate,
    UavPoint,
    uav_from_native,
)
from .hide_com_refine import refine_communication_hide
from .hide_com_route import (
    HideRouteResult,
    RouteCandidate,
    RouteDynamics,
    RouteWaypoint,
    plan_hide_communication_routes,
    validate_route_waypoints,
)


def __getattr__(name: str):
    # scipy is needed only by the broad AREA-cover analyser.  Enemy-contact
    # replanning uses the narrow NumPy/rasterio path below and must not pay the
    # scipy import cost on the attack critical path.
    if name in {"AnalysisResult", "CoverAnalyzer", "RefPoint"}:
        from .analysis import AnalysisResult, CoverAnalyzer, RefPoint

        return {
            "AnalysisResult": AnalysisResult,
            "CoverAnalyzer": CoverAnalyzer,
            "RefPoint": RefPoint,
        }[name]
    raise AttributeError(name)

__all__ = [
    "CoverConfig",
    "DemGrid",
    "DemMeta",
    "NativeDemRaster",
    "Polygon2D",
    "EnemyPoint",
    "sample_enemy_positions",
    "accumulate_engagement",
    "AnalysisResult",
    "CoverAnalyzer",
    "RefPoint",
    "HideResult",
    "NearestHideAnalyzer",
    "enemy_from_native",
    "CommunicationHideAnalyzer",
    "CommunicationHideResult",
    "HideEndpointCandidate",
    "UavPoint",
    "uav_from_native",
    "refine_communication_hide",
    "RouteDynamics",
    "RouteWaypoint",
    "RouteCandidate",
    "HideRouteResult",
    "plan_hide_communication_routes",
    "validate_route_waypoints",
]
