"""
Shared configuration constants for MissionPlanner modules.
Adjust values here to propagate consistently across area/corridor sweep logic.
"""

import sys

_CANONICAL_NAME = "modules.mission_planning.MissionPlanner.config"
if __name__ == "config":
    sys.modules.setdefault(_CANONICAL_NAME, sys.modules[__name__])
elif __name__ == _CANONICAL_NAME:
    sys.modules.setdefault("config", sys.modules[__name__])

DEFAULT_SWEEP_SEPARATION_M = 1000

# Apply extra margin to sweep spacing (e.g., 1.1 = +10% wider).
SWEEP_SPACING_MARGIN = 1.1

# Scaling factor applied to spacing-based search-speed calculations.
SEARCH_SPEED_WEIGHT = 1.1

# Multiplier applied only to FOV values selected from the DB.
DB_FOV_WEIGHT = 1.0

# Enable corridor search-speed DB lookup by default.
USE_DB_FOR_CORRIDOR = False
