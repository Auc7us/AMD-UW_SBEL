import math

import pytest

from amd_uw_ros2.builder_orbit_controller import BuilderState
from amd_uw_ros2.builder_orbit_controller import (
    circle_lookahead_target,
)
from amd_uw_ros2.builder_orbit_controller import (
    circle_pure_pursuit_steering,
)


def test_ccw_lookahead_and_steering_on_circle():
    state = BuilderState(x=40.0, y=0.0, yaw=math.pi / 2, speed=1.0)
    target = circle_lookahead_target(
        state, 0.0, 0.0, 40.0, 8.0, True
    )
    assert math.hypot(*target) == pytest.approx(40.0)
    assert target[0] < 40.0
    assert target[1] > 0.0

    steering = circle_pure_pursuit_steering(
        state, 0.0, 0.0, 40.0, 8.0, 4.0, True
    )
    assert steering == pytest.approx(0.1, rel=0.02)


def test_clockwise_steering_has_opposite_sign():
    state = BuilderState(
        x=40.0, y=0.0, yaw=-math.pi / 2, speed=1.0
    )
    steering = circle_pure_pursuit_steering(
        state, 0.0, 0.0, 40.0, 8.0, 4.0, False
    )
    assert steering == pytest.approx(-0.1, rel=0.02)


def test_outside_circle_commands_stronger_inward_turn():
    on_circle = BuilderState(
        x=40.0, y=0.0, yaw=math.pi / 2, speed=1.0
    )
    outside = BuilderState(
        x=44.0, y=0.0, yaw=math.pi / 2, speed=1.0
    )
    nominal = circle_pure_pursuit_steering(
        on_circle, 0.0, 0.0, 40.0, 8.0, 4.0, True
    )
    correction = circle_pure_pursuit_steering(
        outside, 0.0, 0.0, 40.0, 8.0, 4.0, True
    )
    assert correction > nominal
