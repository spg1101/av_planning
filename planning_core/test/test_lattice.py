import numpy as np

from planning_core.geometry.frenet import ReferencePath
from planning_core.local_planner.collision import path_hits_grid
from planning_core.local_planner.lattice import generate_candidates, quintic_lateral


def test_quintic_endpoints():
    s = np.linspace(0, 5, 51)
    d = quintic_lateral(0.0, 1.0, 5.0, s)
    assert d[0] == 0.0 and abs(d[-1] - 1.0) < 1e-9


def test_candidates_and_grid_check():
    ref = ReferencePath(np.array([[0.0, 0.0], [20.0, 0.0]]))
    cands = generate_candidates(ref, 0.0, 0.0, 10.0, [-0.5, 0.0, 0.5], [5.0])
    assert len(cands) == 3
    grid = np.zeros((40, 400), dtype=int)  # 0.05 m cells, origin (0, -1)
    grid[17:24, 96:105] = 100  # small obstacle around x=5.0, y=0.0
    assert path_hits_grid(cands[1], grid, 0.05, (0.0, -1.0))  # straight ahead: blocked
    assert not path_hits_grid(cands[0], np.zeros((40, 400), dtype=int), 0.05, (0.0, -1.0))
