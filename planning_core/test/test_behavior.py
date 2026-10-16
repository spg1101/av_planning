from planning_core.behavior.state_machine import BehaviorInputs, BehaviorState, BehaviorStateMachine


def test_stop_then_resume():
    sm = BehaviorStateMachine()
    assert sm.step(BehaviorInputs()) == BehaviorState.FOLLOW_LANE
    assert sm.step(BehaviorInputs(obstacle_ahead_dist=1.0)) == BehaviorState.STOP
    assert sm.step(BehaviorInputs(obstacle_ahead_dist=3.0)) == BehaviorState.STOP  # hysteresis
    assert sm.step(BehaviorInputs(obstacle_ahead_dist=10.0)) == BehaviorState.FOLLOW_LANE


def test_emergency_wins():
    sm = BehaviorStateMachine()
    assert sm.step(BehaviorInputs(emergency=True)) == BehaviorState.E_STOP
