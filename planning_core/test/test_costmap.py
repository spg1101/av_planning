import math

import numpy as np

from planning_core.world.costmap import inflate, rolling_grid, scan_to_points, transform_points


def test_scan_points_drop_invalid():
    pts = scan_to_points([1.0, float("inf"), 0.01, 2.0], -math.pi / 2, math.pi / 2, 0.1, 30.0)
    assert len(pts) == 2
    assert np.allclose(pts[0], [0.0, -1.0], atol=1e-9)  # straight right
    assert np.allclose(pts[1], [-2.0, 0.0], atol=1e-9)  # straight back


def test_transform_and_grid():
    pts = transform_points(np.array([[5.0, 0.0]]), 10.0, 20.0, math.pi / 2)  # car facing +y
    assert np.allclose(pts, [[10.0, 25.0]])
    grid, (ox, oy) = rolling_grid(pts, (10.0, 20.0), 20.0, 0.5)
    assert grid.shape == (40, 40)
    row, col = int((25.0 - oy) / 0.5), int((10.0 - ox) / 0.5)
    assert grid[row, col] == 100 and grid.sum() == 100


def test_inflate_radius():
    g = np.zeros((11, 11), dtype=np.int8)
    g[5, 5] = 100
    out = inflate(g, 2)
    assert out[5, 7] == 100 and out[5, 8] == 0 and out[7, 7] == 0
