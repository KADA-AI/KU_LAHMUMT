"""Polygon helpers in native (projected-metre) coordinates.

Pure NumPy (no matplotlib) so the logic package stays importable head-lessly.
Provides a vectorised ray-casting point-in-polygon test (mask a whole DEM grid
at once), a shoelace area/centroid, and a rejection sampler for placing random
enemy points inside the target area.
"""

from __future__ import annotations

import numpy as np

XY = tuple[float, float]


def _tri_area(a: XY, b: XY, c: XY) -> float:
    return 0.5 * abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]))


def _poly_area(pts: list[XY]) -> float:
    x = np.array([p[0] for p in pts])
    y = np.array([p[1] for p in pts])
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def simplify_region_to_polygon(
    xs: np.ndarray,
    ys: np.ndarray,
    *,
    max_vertices: int = 6,
    min_vertices: int = 4,
    area_tol: float = 0.03,
) -> list[XY]:
    """Reduce a cloud of cell centres (a region) to a simple, usable convex
    polygon of ``min_vertices``..``max_vertices`` corners.

    Uses the convex hull (so the polygon encloses the region, possibly with a
    little extra non-safe area) and greedily drops the "cheapest" corners. The
    final vertex count adapts to shape complexity: near-rectangular regions
    collapse to 4 corners, irregular ones keep up to ``max_vertices``.
    """
    pts_arr = np.unique(np.column_stack([np.asarray(xs, float), np.asarray(ys, float)]), axis=0)
    if pts_arr.shape[0] < 3:
        cx = float(pts_arr[:, 0].mean()); cy = float(pts_arr[:, 1].mean())
        d = 75.0
        return [(cx - d, cy - d), (cx + d, cy - d), (cx + d, cy + d), (cx - d, cy + d)]

    try:
        from scipy.spatial import ConvexHull
        hull = ConvexHull(pts_arr)
        poly: list[XY] = [(float(x), float(y)) for x, y in pts_arr[hull.vertices]]
    except Exception:  # collinear / qhull failure -> axis-aligned box
        min_x, min_y = pts_arr.min(axis=0)
        max_x, max_y = pts_arr.max(axis=0)
        return [(float(min_x), float(min_y)), (float(max_x), float(min_y)),
                (float(max_x), float(max_y)), (float(min_x), float(max_y))]

    def cheapest_corner(p: list[XY]) -> tuple[int, float]:
        n = len(p)
        best_i, best_e = 0, float("inf")
        for i in range(n):
            e = _tri_area(p[(i - 1) % n], p[i], p[(i + 1) % n])
            if e < best_e:
                best_i, best_e = i, e
        return best_i, best_e

    while len(poly) > max_vertices:
        i, _ = cheapest_corner(poly)
        poly.pop(i)

    base_area = max(_poly_area(poly), 1e-9)
    while len(poly) > min_vertices:
        i, err = cheapest_corner(poly)
        if err / base_area > area_tol:
            break
        poly.pop(i)
    return poly


def _points_in_polygon(px: np.ndarray, py: np.ndarray, vx: np.ndarray, vy: np.ndarray) -> np.ndarray:
    """Vectorised even-odd ray-casting test. px/py are 1-D query arrays."""
    inside = np.zeros(px.shape, dtype=bool)
    n = vx.size
    j = n - 1
    for i in range(n):
        yi, yj = vy[i], vy[j]
        if yi == yj:  # horizontal edge contributes no crossing
            j = i
            continue
        xi, xj = vx[i], vx[j]
        cond = (yi > py) != (yj > py)
        x_int = xi + (py - yi) * (xj - xi) / (yj - yi)
        inside ^= cond & (px < x_int)
        j = i
    return inside


class Polygon2D:
    """A simple closed polygon defined by native (x, y) vertices."""

    def __init__(self, vertices: list[XY]) -> None:
        pts = [(float(x), float(y)) for x, y in vertices]
        if len(pts) < 3:
            raise ValueError("A polygon needs at least 3 vertices.")
        self.vertices: list[XY] = pts
        self._vx = np.array([p[0] for p in pts], dtype=np.float64)
        self._vy = np.array([p[1] for p in pts], dtype=np.float64)

    # -- basic properties --------------------------------------------------
    @property
    def xs(self) -> np.ndarray:
        return np.array([v[0] for v in self.vertices], dtype=np.float64)

    @property
    def ys(self) -> np.ndarray:
        return np.array([v[1] for v in self.vertices], dtype=np.float64)

    def bounds(self) -> tuple[float, float, float, float]:
        xs, ys = self.xs, self.ys
        return float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())

    def area_m2(self) -> float:
        xs, ys = self.xs, self.ys
        return 0.5 * abs(float(np.dot(xs, np.roll(ys, -1)) - np.dot(ys, np.roll(xs, -1))))

    def centroid(self) -> XY:
        xs, ys = self.xs, self.ys
        cross = xs * np.roll(ys, -1) - np.roll(xs, -1) * ys
        a = float(np.sum(cross)) / 2.0
        if abs(a) < 1e-9:
            return float(xs.mean()), float(ys.mean())
        cx = float(np.sum((xs + np.roll(xs, -1)) * cross)) / (6.0 * a)
        cy = float(np.sum((ys + np.roll(ys, -1)) * cross)) / (6.0 * a)
        return cx, cy

    # -- membership --------------------------------------------------------
    def contains_points(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Boolean mask (same shape as ``x``) of points inside the polygon."""
        xa = np.asarray(x, dtype=np.float64)
        ya = np.asarray(y, dtype=np.float64)
        inside = _points_in_polygon(xa.ravel(), ya.ravel(), self._vx, self._vy)
        return inside.reshape(xa.shape)

    def contains_point(self, x: float, y: float) -> bool:
        m = _points_in_polygon(
            np.array([float(x)]), np.array([float(y)]), self._vx, self._vy
        )
        return bool(m[0])

    def sample_uniform(self, count: int, rng: np.random.Generator) -> list[XY]:
        """Rejection-sample ``count`` points uniformly inside the polygon."""
        min_x, min_y, max_x, max_y = self.bounds()
        out: list[XY] = []
        max_batches = 400  # generous; thin/diagonal polygons converge slowly
        for _ in range(max_batches):
            need = count - len(out)
            if need <= 0:
                break
            batch = max(need * 8, 128)
            xs = rng.uniform(min_x, max_x, batch)
            ys = rng.uniform(min_y, max_y, batch)
            hit = _points_in_polygon(xs, ys, self._vx, self._vy)
            for x, y in zip(xs[hit], ys[hit]):
                out.append((float(x), float(y)))
                if len(out) >= count:
                    break
        return out[:count]
