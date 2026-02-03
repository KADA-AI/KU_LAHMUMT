"""
Shared configuration constants for MissionPlanner modules.
Adjust values here to propagate consistently across area/corridor sweep logic.
"""

DEFAULT_SWEEP_SEPARATION_M = 1500

# Scaling factor applied to spacing-based search-speed calculations.
SEARCH_SPEED_WEIGHT = 3.0

# Enable corridor search-speed DB lookup by default.
USE_DB_FOR_CORRIDOR = False
