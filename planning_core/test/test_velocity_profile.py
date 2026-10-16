import numpy as np

from planning_core.trajectory.velocity_profile import time_stamps, velocity_profile


def test_respects_all_limits():
    s = np.linspace(0, 50, 251)
    k = np.where((s > 20) & (s < 30), 0.5, 0.0)  # a tight curve in the middle
    v = velocity_profile(s, k, v_max=5.0, a_lat_max=2.0, a_accel=1.5, a_decel=3.0, v_start=1.0, v_end=0.0)
    ds = np.diff(s)
    assert np.all(v <= 5.0 + 1e-9)
    assert np.all(v[k > 0] <= np.sqrt(2.0 / 0.5) + 1e-9)
    assert np.all(np.diff(v**2) <= 2 * 1.5 * ds + 1e-6)  # acceleration limit
    assert np.all(-np.diff(v**2) <= 2 * 3.0 * ds + 1e-6)  # braking limit
    assert v[0] <= 1.0 + 1e-9 and v[-1] == 0.0


def test_time_stamps_increase():
    s = np.linspace(0, 10, 11)
    t = time_stamps(s, np.full(11, 2.0))
    assert np.all(np.diff(t) > 0) and abs(t[-1] - 5.0) < 1e-9
