# /mnt/data/zones.py
# -*- coding: utf-8 -*-

GRID_ROWS = 35
GRID_COLS = 50


def Z(r1, c1, r2, c2):
    """Convert 1-based inclusive coordinates to a grid layout descriptor."""
    return {
        "r0": r1 - 1,
        "c0": c1 - 1,
        "rs": (r2 - r1 + 1),
        "cs": (c2 - c1 + 1),
    }


ZONES = {
    # (1) Title area
    "TITLE": Z(2, 2, 3, 25),

    # (2) Path browse button
    "ROUTE_BUTTON": Z(3, 47, 3, 48),

    # (3) Database path summary
    "DB_PATH": Z(3, 28, 3, 45),

    # (3-1) Middleware settings row
    "MIDDLEWARE": Z(4, 28, 4, 48),

    # (4~9) Central module area (currently empty placeholder)
    "MODULE_CENTER": Z(5, 14, 27, 48),

    # (10) Flow visualizer
    "FLOW_VIS": Z(5, 6, 27, 12),

    # (11) Mode buttons
    "MODE_BUTTONS": Z(5, 2, 27, 4),

    # (12) Operations flow panel
    "OPS_FLOW": Z(29, 2, 33, 48),

    # (13) Footer row
    "FOOTER": Z(35, 1, 35, 50),
}
