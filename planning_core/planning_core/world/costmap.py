"""Rolling local costmap from a 2D laser scan (Member A). Pure numpy, unit-tested.

Grid layout matches nav_msgs/OccupancyGrid: row = y index, column = x index, origin = the
map-frame position of cell (0, 0)'s corner. 0 = free, 100 = occupied.
"""

import math
from typing import Tuple

import numpy as np


def scan_to_points(ranges, angle_min: float, angle_increment: float, range_min: float, range_max: float):
    """Laser ranges -> (N, 2) endpoints in the scan frame, dropping invalid readings."""
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= range_min) & (r < range_max)
    return np.column_stack([r[ok] * np.cos(angles[ok]), r[ok] * np.sin(angles[ok])])


def transform_points(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([x + c * points[:, 0] - s * points[:, 1], y + s * points[:, 0] + c * points[:, 1]])


def rolling_grid(
    points_xy: np.ndarray, center_xy: Tuple[float, float], size: float, resolution: float
) -> Tuple[np.ndarray, Tuple[float, float]]:
    """Square grid of side `size` centred on center_xy, aligned to the map axes.

    The origin is snapped to the resolution so cells don't shimmer as the car moves.
    Returns (grid int8 [n, n], origin (x, y)).
    """
    n = int(round(size / resolution))
    ox = math.floor((center_xy[0] - size / 2) / resolution) * resolution
    oy = math.floor((center_xy[1] - size / 2) / resolution) * resolution
    grid = np.zeros((n, n), dtype=np.int8)
    if len(points_xy):
        cols = np.floor((points_xy[:, 0] - ox) / resolution).astype(int)
        rows = np.floor((points_xy[:, 1] - oy) / resolution).astype(int)
        ok = (rows >= 0) & (rows < n) & (cols >= 0) & (cols < n)
        grid[rows[ok], cols[ok]] = 100
    return grid, (ox, oy)


def inflate(grid: np.ndarray, radius_cells: int) -> np.ndarray:
    """Mark every cell within radius of an occupied cell as occupied (100)."""
    occupied = grid >= 50
    out = grid.copy()
    h, w = grid.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            shifted = np.zeros_like(occupied)
            ys, yd = slice(max(dy, 0), h + min(dy, 0)), slice(max(-dy, 0), h + min(-dy, 0))
            xs, xd = slice(max(dx, 0), w + min(dx, 0)), slice(max(-dx, 0), w + min(-dx, 0))
            shifted[ys, xs] = occupied[yd, xd]
            out[shifted] = 100
    return out
