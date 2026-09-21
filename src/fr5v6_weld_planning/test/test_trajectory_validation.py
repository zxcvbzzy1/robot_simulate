import numpy as np
import pytest
from action_msgs.msg import GoalStatus

from fr5v6_weld_planning.moveit_executor import (
    TrajectoryValidationError,
    build_robot_trajectory,
    densify_joint_path,
    pose_errors,
    terminal_status_name,
    terminal_allows_next,
    time_parameterize,
    validate_joint_path,
)


BOUNDS = [(-3.0, 3.0)] * 6


def test_densification_limits_each_interpolated_joint_step():
    dense = densify_joint_path([[0] * 6, [0.11] * 6], 0.02)
    steps = np.abs(np.diff(np.asarray(dense), axis=0))
    assert steps.max() <= 0.02 + 1e-12
    np.testing.assert_allclose(dense[-1], [0.11] * 6)


@pytest.mark.parametrize('states,message', [
    ([[0] * 6, [float('nan')] * 6], 'finite'),
    ([[0] * 6, [3.1] * 6], 'limit'),
    ([[0] * 6, [0.8] * 6], 'jump'),
])
def test_joint_validation_rejects_unsafe_states(states, message):
    with pytest.raises(TrajectoryValidationError, match=message):
        validate_joint_path(states, BOUNDS, max_jump=0.5)


def test_time_parameterization_is_strictly_increasing_and_bounded():
    states = [[0] * 6, [0.1] * 6, [0.2] * 6]
    times = time_parameterize(states, max_velocities=[1.0] * 6,
                              max_accelerations=[1.0] * 6, minimum_dt=0.01)
    assert times[0] == 0.0
    assert all(second > first for first, second in zip(times, times[1:]))
    # Zero endpoint velocities with cubic interpolation require dt >= sqrt(6*d/a).
    assert times[1] >= np.sqrt(0.6)


def test_time_parameterization_honours_cartesian_process_time():
    states = [[0] * 6, [0.001] * 6]
    times = time_parameterize(states, [1.0] * 6, [1.0] * 6,
                              minimum_dt=0.01, segment_minimum_dts=[2.0])
    assert times == [0.0, 2.0]


@pytest.mark.parametrize('status', [
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
])
def test_only_documented_terminal_states_allow_followup(status):
    assert terminal_allows_next(status)


def test_unknown_execution_state_suppresses_followup():
    assert not terminal_allows_next(GoalStatus.STATUS_UNKNOWN)


def test_terminal_status_has_a_readable_name():
    assert terminal_status_name(GoalStatus.STATUS_SUCCEEDED) == 'SUCCEEDED'
    assert terminal_status_name(99) == 'UNKNOWN_99'


def test_pose_errors_accept_quaternion_sign_equivalence():
    position_error, orientation_error = pose_errors(
        ([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, -1.0]),
        ([1.0, 2.0, 3.01], [0.0, 0.0, 0.0, 1.0]),
    )
    assert position_error == pytest.approx(0.01)
    assert orientation_error == pytest.approx(0.0)


def test_robot_trajectory_preserves_joint_order_and_strict_times():
    trajectory = build_robot_trajectory(
        ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'],
        [[0] * 6, [0.1] * 6], [0.0, 1.25],
    )
    assert list(trajectory.joint_trajectory.joint_names) == [
        'j1', 'j2', 'j3', 'j4', 'j5', 'j6']
    assert trajectory.joint_trajectory.points[1].time_from_start.sec == 1
    assert trajectory.joint_trajectory.points[1].time_from_start.nanosec == 250_000_000
    assert list(trajectory.joint_trajectory.points[1].velocities) == [0.0] * 6
