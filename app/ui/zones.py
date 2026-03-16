# /mnt/data/zones.py
# -*- coding: utf-8 -*-

GRID_ROWS = 9
GRID_COLS = 28


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
    "TITLE": Z(1, 1, 1, 28),

    # (2) Path browse button
    "ROUTE_BUTTON": Z(3, 24, 3, 28),

    # (3) Database path summary
    "DB_PATH": Z(2, 1, 3, 23),

    # (3-1) Middleware settings row
    "MIDDLEWARE": Z(4, 1, 4, 28),

    # (4~9) Central module area (currently empty placeholder)
    "MODULE_CENTER": Z(5, 1, 8, 28),

    # (10) Flow visualizer
    "FLOW_VIS": Z(5, 1, 8, 28),

    # (11) Mode buttons
    "MODE_BUTTONS": Z(5, 1, 8, 28),

    # (12) Operations flow panel
    "OPS_FLOW": Z(8, 1, 8, 28),

    # (13) Footer row
    "FOOTER": Z(9, 1, 9, 28),
}
