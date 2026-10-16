from planning_core.trajectory.stitching import stitch_start
from planning_core.types import EgoState, Trajectory, TrajectoryPoint


def make_prev():
    pts = [
        TrajectoryPoint(x=float(i), y=0.0, yaw=0.0, velocity=1.0, time_from_start=float(i)) for i in range(5)
    ]
    return Trajectory(points=pts, stamp=100.0)


def test_uses_previous_plan_when_close():
    point, stitched = stitch_start(make_prev(), now=102.0, ego=EgoState(2.1, 0.1, 0.0, 1.0))
    assert stitched and point.x == 2.0


def test_falls_back_when_drifted():
    point, stitched = stitch_start(make_prev(), now=102.0, ego=EgoState(2.0, 1.0, 0.0, 1.0))
    assert not stitched and point.y == 1.0
