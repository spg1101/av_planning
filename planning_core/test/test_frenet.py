import numpy as np
import pytest

from planning_core.geometry.frenet import ReferencePath
from planning_core.geometry.path import resample


def circle(radius=10.0, n=400):
    t = np.linspace(0, np.pi, n)
    return np.column_stack([radius * np.cos(t), radius * np.sin(t)])


@pytest.mark.parametrize("d", [-0.5, -0.1, 0.0, 0.2, 0.5])
def test_round_trip_on_circle(d):
    ref = ReferencePath(resample(circle(), 0.1))
    for s in np.linspace(1.0, ref.length - 1.0, 25):
        x, y, _ = ref.to_cartesian(s, d)
        s2, d2 = ref.to_frenet(x, y)
        x2, y2, _ = ref.to_cartesian(s2, d2)
        assert np.hypot(x2 - x, y2 - y) < 0.01


def test_left_is_positive():
    ref = ReferencePath(np.array([[0.0, 0.0], [10.0, 0.0]]))
    _, d = ref.to_frenet(5.0, 1.0)
    assert d == pytest.approx(1.0)
