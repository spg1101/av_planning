"""Behavior state machine. Values match av_msgs/BehaviorCommand constants."""

from dataclasses import dataclass
from enum import IntEnum


class BehaviorState(IntEnum):
    FOLLOW_LANE = 0
    STOP = 1
    YIELD = 2
    OVERTAKE = 3
    E_STOP = 4


@dataclass
class BehaviorInputs:
    emergency: bool = False
    obstacle_ahead_dist: float = float("inf")  # m along the route, in our lane
    obstacle_ahead_speed: float = 0.0  # m/s
    adjacent_lane_free: bool = False
    ego_speed: float = 0.0


@dataclass
class BehaviorParams:
    stop_distance: float = 2.0  # m: stop when a stopped obstacle is this close
    clear_distance: float = 4.0  # m: resume once the obstacle is beyond this
    stopped_speed: float = 0.2  # m/s: obstacle counts as stopped below this
    allow_overtake: bool = False  # TODO(lead): enable once the lattice handles lane offsets
    # Following a moving lead vehicle (adaptive cruise): keep min_gap + time_gap * lead speed.
    follow_range: float = 15.0  # m: start adapting speed to a lead vehicle inside this gap
    min_gap: float = 1.0  # m
    time_gap: float = 1.0  # s
    follow_gain: float = 0.5  # 1/s: how strongly the gap error changes the speed


def follow_speed_limit(gap: float, lead_speed: float, speed_limit: float, p: BehaviorParams) -> float:
    """Speed that settles at the desired gap behind a lead vehicle (never above speed_limit)."""
    if gap >= p.follow_range:
        return speed_limit
    desired_gap = p.min_gap + p.time_gap * max(lead_speed, 0.0)
    return max(0.0, min(speed_limit, lead_speed + p.follow_gain * (gap - desired_gap)))


class BehaviorStateMachine:
    def __init__(self, params: BehaviorParams = BehaviorParams()):
        self.params = params
        self.state = BehaviorState.FOLLOW_LANE

    def step(self, inp: BehaviorInputs) -> BehaviorState:
        p = self.params
        if inp.emergency:
            self.state = BehaviorState.E_STOP
            return self.state
        blocked = inp.obstacle_ahead_dist < p.stop_distance and inp.obstacle_ahead_speed < p.stopped_speed
        if self.state in (BehaviorState.FOLLOW_LANE, BehaviorState.E_STOP):
            if blocked:
                self.state = (
                    BehaviorState.OVERTAKE
                    if p.allow_overtake and inp.adjacent_lane_free
                    else BehaviorState.STOP
                )
            else:
                self.state = BehaviorState.FOLLOW_LANE
        elif self.state == BehaviorState.STOP:
            if inp.obstacle_ahead_dist > p.clear_distance or inp.obstacle_ahead_speed >= p.stopped_speed:
                self.state = BehaviorState.FOLLOW_LANE
            elif p.allow_overtake and inp.adjacent_lane_free:
                self.state = BehaviorState.OVERTAKE
        elif self.state == BehaviorState.OVERTAKE:
            # TODO(lead): finish overtake when the obstacle is behind us.
            if inp.obstacle_ahead_dist > p.clear_distance:
                self.state = BehaviorState.FOLLOW_LANE
        # TODO(lead): YIELD for junctions / crossing pedestrians (Phase 3).
        return self.state
