import numpy as np

from planning_eval.metrics import match_by_time, position_rmse


def test_rmse_constant_offset():
    gt = np.zeros((10, 2))
    est = gt + np.array([0.3, 0.4])
    assert abs(position_rmse(est, gt) - 0.5) < 1e-9


def test_match_by_time_rejects_far_samples():
    idx = match_by_time(np.array([0.0, 1.02, 5.0]), np.array([0.0, 1.0, 2.0]), max_dt=0.05)
    assert list(idx) == [0, 1, -1]
