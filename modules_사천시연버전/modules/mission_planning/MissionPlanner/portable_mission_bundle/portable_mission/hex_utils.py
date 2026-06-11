from __future__ import annotations

import math
from typing import Dict, List, Tuple

import numpy as np

ALTITUDE_BUFFER_M = 25.0

DIRECTION_SEQUENCE = [1, 2, 3, 4, 5, 6]
DIRECTION_TO_INDEX = {direction: idx for idx, direction in enumerate(DIRECTION_SEQUENCE)}
RELATIVE_ACTION_DELTAS = {0: -1, 1: 0, 2: 1}

EVEN_ROW_DELTAS = {
    1: (-1, -1),
    2: (-1, 0),
    3: (0, 1),
    4: (1, 0),
    5: (1, -1),
    6: (0, -1),
}

ODD_ROW_DELTAS = {
    1: (-1, 0),
    2: (-1, 1),
    3: (0, 1),
    4: (1, 1),
    5: (1, 0),
    6: (0, -1),
}


def offset_to_axial(row: int, col: int) -> Tuple[int, int]:
    q = col - ((row - (row & 1)) // 2)
    r = row
    return q, r


def axial_to_offset(q: int, r: int) -> Tuple[int, int]:
    row = r
    col = q + ((r - (r & 1)) // 2)
    return row, col


def compute_obstacle_mask(
    dem: np.ma.MaskedArray | np.ndarray,
    threshold: float,
) -> np.ndarray:
    if np.ma.isMaskedArray(dem):
        valid = ~np.ma.getmaskarray(dem)
        data = dem.data
    else:
        valid = np.ones_like(dem, dtype=bool)
        data = dem
    return (data >= threshold) & valid


def aggregate_to_hex(mask: np.ndarray, step: int) -> np.ndarray:
    step = max(int(step), 1)
    height, width = mask.shape
    nrows = max(1, (height + step - 1) // step)
    ncols = max(1, (width + step - 1) // step)

    occupancy = np.zeros((nrows, ncols), dtype=bool)
    for row in range(nrows):
        row_slice = slice(row * step, min((row + 1) * step, height))
        for col in range(ncols):
            col_slice = slice(col * step, min((col + 1) * step, width))
            occupancy[row, col] = bool(np.any(mask[row_slice, col_slice]))
    return occupancy


def step_neighbor(row: int, col: int, direction: int) -> Tuple[int, int]:
    deltas = EVEN_ROW_DELTAS if row % 2 == 0 else ODD_ROW_DELTAS
    dr, dc = deltas[direction]
    return row + dr, col + dc


def compute_ring_offsets(
    radius: int,
) -> Tuple[Dict[int, List[Tuple[int, int]]], Dict[int, int]]:
    if radius < 1:
        raise ValueError("radius must be >= 1")

    q0, r0 = offset_to_axial(0, 0)
    axial_dirs = {}
    for direction in DIRECTION_SEQUENCE:
        nr, nc = step_neighbor(0, 0, direction)
        q1, r1 = offset_to_axial(nr, nc)
        axial_dirs[direction] = (q1 - q0, r1 - r0)

    start_q = axial_dirs[5][0] * radius
    start_r = axial_dirs[5][1] * radius
    axial_ring: List[Tuple[int, int]] = []
    q, r = start_q, start_r
    for direction in DIRECTION_SEQUENCE:
        dq, dr = axial_dirs[direction]
        for _ in range(radius):
            axial_ring.append((q, r))
            q += dq
            r += dr

    offsets: Dict[int, List[Tuple[int, int]]] = {}
    for parity in (0, 1):
        center_row, center_col = parity, 0
        center_q, center_r = offset_to_axial(center_row, center_col)
        offsets[parity] = []
        for aq, ar in axial_ring:
            row, col = axial_to_offset(center_q + aq, center_r + ar)
            offsets[parity].append((row - center_row, col - center_col))

    forward_indices: Dict[int, int] = {}
    for direction, (dq, dr) in axial_dirs.items():
        forward_indices[direction] = axial_ring.index((dq * radius, dr * radius))
    return offsets, forward_indices


def relative_action_to_direction(heading_idx: int, action: int) -> Tuple[int, int]:
    if action not in RELATIVE_ACTION_DELTAS:
        raise ValueError("action must be in {0, 1, 2}")
    new_heading_idx = (heading_idx + RELATIVE_ACTION_DELTAS[action]) % len(DIRECTION_SEQUENCE)
    return new_heading_idx, DIRECTION_SEQUENCE[new_heading_idx]


def generate_hex_geometry(bounds, shape: Tuple[int, int]):
    nrows, ncols = shape
    sqrt3 = math.sqrt(3.0)
    template = np.array(
        [
            (0.0, 1.0),
            (sqrt3 / 2.0, 0.5),
            (sqrt3 / 2.0, -0.5),
            (0.0, -1.0),
            (-sqrt3 / 2.0, -0.5),
            (-sqrt3 / 2.0, 0.5),
        ],
        dtype=float,
    )

    polygons_norm = np.empty((nrows * ncols, 6, 2), dtype=float)
    centers_norm = np.empty((nrows * ncols, 2), dtype=float)

    idx = 0
    for row in range(nrows):
        center_y = 1.5 * row
        for col in range(ncols):
            center_x = sqrt3 * (col + 0.5 * (row & 1))
            centers_norm[idx] = (center_x, center_y)
            polygons_norm[idx] = template + np.array([center_x, center_y])
            idx += 1

    x_min = float(polygons_norm[:, :, 0].min())
    x_max = float(polygons_norm[:, :, 0].max())
    y_min = float(polygons_norm[:, :, 1].min())
    y_max = float(polygons_norm[:, :, 1].max())

    span_x = x_max - x_min if x_max != x_min else 1.0
    span_y = y_max - y_min if y_max != y_min else 1.0
    scale_x = (bounds.right - bounds.left) / span_x
    scale_y = (bounds.top - bounds.bottom) / span_y

    polygons = np.empty_like(polygons_norm)
    polygons[:, :, 0] = bounds.left + (polygons_norm[:, :, 0] - x_min) * scale_x
    polygons[:, :, 1] = bounds.top - (polygons_norm[:, :, 1] - y_min) * scale_y

    centers = np.empty_like(centers_norm)
    centers[:, 0] = bounds.left + (centers_norm[:, 0] - x_min) * scale_x
    centers[:, 1] = bounds.top - (centers_norm[:, 1] - y_min) * scale_y

    transform = {
        "x_min": x_min,
        "y_min": y_min,
        "scale_x": scale_x,
        "scale_y": scale_y,
        "sqrt3": sqrt3,
        "nrows": nrows,
        "ncols": ncols,
    }
    return polygons, centers.reshape((nrows, ncols, 2)), transform
