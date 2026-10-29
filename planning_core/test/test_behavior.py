from planning_core.behavior.state_machine import (
    BehaviorInputs,
    BehaviorParams,
    BehaviorState,
    BehaviorStateMachine,
    follow_speed_limit,
)


def test_stop_then_resume():
    sm = BehaviorStateMachine()
    assert sm.step(BehaviorInputs()) == BehaviorState.FOLLOW_LANE
    assert sm.step(BehaviorInputs(obstacle_ahead_dist=1.0)) == BehaviorState.STOP
    assert sm.step(BehaviorInputs(obstacle_ahead_dist=3.0)) == BehaviorState.STOP  # hysteresis
    assert sm.step(BehaviorInputs(obstacle_ahead_dist=10.0)) == BehaviorState.FOLLOW_LANE


def test_emergency_wins():
    sm = BehaviorStateMachine()
    assert sm.step(BehaviorInputs(emergency=True)) == BehaviorState.E_STOP


def test_follow_speed():
    p = BehaviorParams(follow_range=30.0, min_gap=5.0, time_gap=1.0, follow_gain=0.5)
    assert follow_speed_limit(100.0, 3.0, 8.0, p) == 8.0  # nobody close: free driving
    assert follow_speed_limit(13.0, 8.0, 8.0, p) == 8.0  # at the desired gap: match the leader
    assert follow_speed_limit(9.0, 8.0, 8.0, p) < 8.0  # too close: slower than the leader
    assert follow_speed_limit(2.0, 0.0, 8.0, p) == 0.0  # stopped leader right ahead: stop
