"""Collision checks against an occupancy grid and predicted obstacles (Member B)."""

from typing import List

import numpy as np

from planning_core.types import Obstacle


def path_hits_grid(
    xy: np.ndarray, grid: np.ndarray, resolution: float, origin_xy, occupied_threshold: int = 50
) -> bool:
    """True if any path point falls on an occupied (or unknown, -1) cell.

    grid: (H, W) int array, row = y index, col = x index (nav_msgs/OccupancyGrid layout).
    The grid is expected to be inflated by the vehicle radius already (world model does it).
    """
    cols = np.floor((xy[:, 0] - origin_xy[0]) / resolution).astype(int)
    rows = np.floor((xy[:, 1] - origin_xy[1]) / resolution).astype(int)
    h, w = grid.shape
    inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
    if not np.all(inside):
        return True
    values = grid[rows, cols]
    return bool(np.any((values >= occupied_threshold) | (values < 0)))


def path_hits_obstacles(xy: np.ndarray, times: np.ndarray, obstacles: List[Obstacle], radius: float) -> bool:
    """Time-indexed check against constant-velocity obstacle predictions (task P2.4)."""
    for ob in obstacles:
        ox = ob.x + ob.vx * times
        oy = ob.y + ob.vy * times
        clearance = radius + 0.5 * max(ob.length, ob.width)
        if np.any(np.hypot(xy[:, 0] - ox, xy[:, 1] - oy) < clearance):
            return True
    return False
