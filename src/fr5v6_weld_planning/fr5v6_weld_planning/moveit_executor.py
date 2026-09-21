"""MoveIt execution helpers and independently testable safety invariants."""

from dataclasses import dataclass
import math
import time

import numpy as np
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints, JointConstraint, MoveItErrorCodes, RobotState, RobotTrajectory,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryValidationError(ValueError):
    pass


@dataclass(frozen=True)
class ValidationResult:
    point_count: int
    maximum_jump: float


def _duration(seconds):
    seconds = float(seconds)
    whole = int(math.floor(seconds))
    nanoseconds = int(round((seconds - whole) * 1e9))
    if nanoseconds == 1_000_000_000:
        whole += 1
        nanoseconds = 0
    return Duration(sec=whole, nanosec=nanoseconds)


def build_robot_trajectory(joint_names, states, times):
    if len(states) != len(times) or not states:
        raise TrajectoryValidationError('trajectory states and times do not match')
    if float(times[0]) != 0.0 or any(second <= first for first, second in zip(times, times[1:])):
        raise TrajectoryValidationError('trajectory time must start at zero and strictly increase')
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = list(joint_names)
    for values, stamp in zip(states, times):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in values]
        point.velocities = [0.0] * len(joint_names)
        point.accelerations = [0.0] * len(joint_names)
        point.time_from_start = _duration(stamp)
        result.joint_trajectory.points.append(point)
    return result


def terminal_allows_next(status):
    return status in {
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_CANCELED,
        GoalStatus.STATUS_ABORTED,
    }


def terminal_status_name(status):
    names = {
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
    }
    return names.get(status, f'UNKNOWN_{status}')


def pose_errors(actual, expected):
    actual_position = np.asarray(actual[0], dtype=float)
    actual_quaternion = np.asarray(actual[1], dtype=float)
    expected_position = np.asarray(expected[0], dtype=float)
    expected_quaternion = np.asarray(expected[1], dtype=float)
    position_error = float(np.linalg.norm(actual_position - expected_position))
    dot = float(np.clip(abs(np.dot(actual_quaternion, expected_quaternion)), 0.0, 1.0))
    return position_error, 2.0 * math.acos(dot)


def validate_joint_path(states, bounds, max_jump):
    values = np.asarray(states, dtype=float)
    if values.ndim != 2 or values.shape[0] < 2 or values.shape[1] != len(bounds):
        raise TrajectoryValidationError('joint path has an invalid shape')
    if not np.isfinite(values).all():
        raise TrajectoryValidationError('joint path must contain finite values')
    for index, (lower, upper) in enumerate(bounds):
        if np.any(values[:, index] < lower) or np.any(values[:, index] > upper):
            raise TrajectoryValidationError(f'joint {index + 1} limit violation')
    jumps = np.max(np.abs(np.diff(values, axis=0)), axis=1)
    maximum = float(jumps.max(initial=0.0))
    if maximum > float(max_jump) + 1e-12:
        point = int(np.argmax(jumps)) + 1
        raise TrajectoryValidationError(f'joint jump at point {point}: {maximum:.6f} rad')
    return ValidationResult(len(values), maximum)


def densify_joint_path(states, maximum_step):
    maximum_step = float(maximum_step)
    if maximum_step <= 0:
        raise TrajectoryValidationError('densification step must be positive')
    values = [np.asarray(state, dtype=float) for state in states]
    result = [values[0].copy()]
    for start, end in zip(values, values[1:]):
        count = max(1, int(math.ceil(float(np.max(np.abs(end - start))) / maximum_step)))
        result.extend(start + (end - start) * (index / count) for index in range(1, count + 1))
    return [value.tolist() for value in result]


def time_parameterize(states, max_velocities, max_accelerations, minimum_dt=0.01,
                      segment_minimum_dts=None):
    values = np.asarray(states, dtype=float)
    velocities = np.asarray(max_velocities, dtype=float)
    accelerations = np.asarray(max_accelerations, dtype=float)
    if np.any(velocities <= 0) or np.any(accelerations <= 0):
        raise TrajectoryValidationError('dynamic limits must be positive')
    times = [0.0]
    minimums = list(segment_minimum_dts or [minimum_dt] * (len(values) - 1))
    if len(minimums) != len(values) - 1:
        raise TrajectoryValidationError('segment timing count does not match joint path')
    for delta, required_dt in zip(np.abs(np.diff(values, axis=0)), minimums):
        # Cubic interpolation between zero-velocity endpoints has maxima
        # 1.5*d/dt and 6*d/dt^2.
        velocity_dt = float(np.max(1.5 * delta / velocities))
        acceleration_dt = float(np.max(np.sqrt(6.0 * delta / accelerations)))
        dt = max(float(minimum_dt), float(required_dt), velocity_dt, acceleration_dt)
        times.append(times[-1] + dt)
    return times


class WeldMoveItExecutor:
    """Sequential IK, validation, preview planning and terminal-safe execution."""

    def __init__(self, node, config, bounds):
        self.node = node
        self.config = config
        self.joints = list(config['robot']['joints'])
        self.group = config['robot']['group']
        self.tcp = config['tcp']['child_frame']
        self.bounds = list(bounds)
        self.positions = None
        self.received_wall = None
        node.create_subscription(JointState, '/joint_states', self._joint_state,
                                 qos_profile_sensor_data)
        self.ik_client = node.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = node.create_client(GetPositionFK, '/compute_fk')
        self.validity_client = node.create_client(GetStateValidity, '/check_state_validity')
        self.plan_client = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.execute_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')

    def _joint_state(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values and math.isfinite(values[name]) for name in self.joints):
            self.positions = [float(values[name]) for name in self.joints]
            self.received_wall = time.monotonic()

    def _wait_future(self, future, timeout, label):
        deadline = time.monotonic() + float(timeout)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError(f'{label} timed out')
        if future.exception() is not None:
            raise RuntimeError(f'{label} failed: {future.exception()}')
        return future.result()

    def ready(self):
        timeout = float(self.config['timeouts']['startup'])
        deadline = time.monotonic() + timeout
        clients = [self.ik_client, self.fk_client, self.validity_client, self.plan_client]
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            fresh = self.received_wall is not None and (
                time.monotonic() - self.received_wall <= self.config['safety']['feedback_max_age'])
            if (fresh and all(client.service_is_ready() for client in clients)
                    and self.execute_client.server_is_ready()):
                return
        raise TimeoutError('MoveIt services, execution action, or fresh joint feedback unavailable')

    def robot_state(self, values):
        result = RobotState()
        result.joint_state = JointState(name=self.joints, position=list(map(float, values)))
        result.is_diff = True
        return result

    def solve_ik(self, world_poses, segments):
        seed = list(self.positions)
        states = []
        timeout = float(self.config['timeouts']['service'])
        for index, ((position, quaternion), segment) in enumerate(zip(world_poses, segments)):
            request = GetPositionIK.Request()
            ik = request.ik_request
            ik.group_name = self.group
            ik.robot_state = self.robot_state(seed)
            ik.avoid_collisions = True
            ik.ik_link_name = self.tcp
            ik.timeout = _duration(self.config['ik']['timeout'])
            ik.pose_stamped = PoseStamped()
            ik.pose_stamped.header.frame_id = 'world'
            ik.pose_stamped.pose.position.x, ik.pose_stamped.pose.position.y, \
                ik.pose_stamped.pose.position.z = map(float, position)
            ik.pose_stamped.pose.orientation.x, ik.pose_stamped.pose.orientation.y, \
                ik.pose_stamped.pose.orientation.z, ik.pose_stamped.pose.orientation.w = map(float, quaternion)
            response = self._wait_future(self.ik_client.call_async(request), timeout,
                                         f'IK {segment}[{index}]')
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                raise TrajectoryValidationError(
                    f'IK failed at {segment}[{index}], MoveIt code {response.error_code.val}')
            by_name = dict(zip(response.solution.joint_state.name,
                               response.solution.joint_state.position))
            if not all(name in by_name for name in self.joints):
                raise TrajectoryValidationError(f'IK response missing joints at {segment}[{index}]')
            solution = [float(by_name[name]) for name in self.joints]
            validate_joint_path([seed, solution], self.bounds,
                                self.config['safety']['max_joint_jump'] if states else math.inf)
            states.append(solution)
            seed = solution
        validate_joint_path(states, self.bounds, self.config['safety']['max_joint_jump'])
        return states

    def state_valid(self, values):
        request = GetStateValidity.Request(
            robot_state=self.robot_state(values), group_name=self.group)
        response = self._wait_future(
            self.validity_client.call_async(request), self.config['timeouts']['service'],
            'state validity')
        return bool(response.valid)

    def validate_collisions(self, states):
        dense = densify_joint_path(states, self.config['safety']['densify_max_joint_step'])
        for index, values in enumerate(dense):
            if not self.state_valid(values):
                raise TrajectoryValidationError(f'collision at densified state {index}')
        return len(dense)

    def plan_to(self, target):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = self.group
        motion.pipeline_id = 'ompl'
        motion.planner_id = 'RRTConnectkConfigDefault'
        motion.num_planning_attempts = 3
        motion.allowed_planning_time = float(self.config['planning']['allowed_time'])
        motion.max_velocity_scaling_factor = float(self.config['planning']['velocity_scaling'])
        motion.max_acceleration_scaling_factor = float(self.config['planning']['acceleration_scaling'])
        motion.start_state = self.robot_state(self.positions)
        constraints = Constraints()
        constraints.joint_constraints = [
            JointConstraint(joint_name=name, position=float(value), tolerance_above=0.001,
                            tolerance_below=0.001, weight=1.0)
            for name, value in zip(self.joints, target)
        ]
        motion.goal_constraints = [constraints]
        retries = int(self.config['planning']['service_retries'])
        if retries <= 0:
            raise TrajectoryValidationError('planning service_retries must be positive')
        last_detail = 'no planning response'
        for attempt in range(1, retries + 1):
            response = self._wait_future(
                self.plan_client.call_async(request), self.config['timeouts']['planning'],
                f'approach positioning plan attempt {attempt}')
            result = response.motion_plan_response
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                last_detail = f'MoveIt code {result.error_code.val}'
            else:
                try:
                    self.review_moveit_trajectory(result.trajectory, self.positions, target)
                    return result.trajectory
                except TrajectoryValidationError as error:
                    last_detail = str(error)
            self.node.get_logger().warning(
                f'approach positioning attempt {attempt}/{retries} rejected: {last_detail}')
        raise TrajectoryValidationError(
            f'approach positioning plan failed after {retries} attempts: {last_detail}')

    def review_moveit_trajectory(self, trajectory, start, target):
        names = list(trajectory.joint_trajectory.joint_names)
        if any(name not in names for name in self.joints):
            raise TrajectoryValidationError('MoveIt trajectory is missing controlled joints')
        order = [names.index(name) for name in self.joints]
        states = [[point.positions[index] for index in order]
                  for point in trajectory.joint_trajectory.points]
        if not states:
            raise TrajectoryValidationError('MoveIt returned an empty trajectory')
        validate_joint_path(states, self.bounds, math.inf)
        if max(abs(a - b) for a, b in zip(states[0], start)) > 0.02:
            raise TrajectoryValidationError('MoveIt trajectory start mismatch')
        if max(abs(a - b) for a, b in zip(states[-1], target)) > 0.02:
            raise TrajectoryValidationError('MoveIt trajectory goal mismatch')
        stamps = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                  for point in trajectory.joint_trajectory.points]
        if any(second <= first for first, second in zip(stamps, stamps[1:])):
            raise TrajectoryValidationError('MoveIt trajectory time is not increasing')
        self.validate_collisions([list(start)] + states)

    def verify_fk(self, states, expected_poses, segments):
        maximum_position = 0.0
        maximum_orientation = 0.0
        for index, (state, expected, segment) in enumerate(zip(states, expected_poses, segments)):
            request = GetPositionFK.Request()
            request.header.frame_id = 'world'
            request.fk_link_names = [self.tcp]
            request.robot_state = self.robot_state(state)
            response = self._wait_future(self.fk_client.call_async(request),
                                         self.config['timeouts']['service'],
                                         f'FK {segment}[{index}]')
            if response.error_code.val != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
                raise TrajectoryValidationError(f'FK failed at {segment}[{index}]')
            actual = response.pose_stamped[0].pose
            actual_pose = (
                [actual.position.x, actual.position.y, actual.position.z],
                [actual.orientation.x, actual.orientation.y,
                 actual.orientation.z, actual.orientation.w],
            )
            position_error, orientation_error = pose_errors(actual_pose, expected)
            maximum_position = max(maximum_position, position_error)
            maximum_orientation = max(maximum_orientation, orientation_error)
            if segment == 'seam':
                if position_error > self.config['safety']['tcp_position_tolerance']:
                    raise TrajectoryValidationError(
                        f'TCP position error at seam[{index}]: {position_error:.6f} m')
                if orientation_error > self.config['safety']['tcp_orientation_tolerance']:
                    raise TrajectoryValidationError(
                        f'TCP orientation error at seam[{index}]: {orientation_error:.6f} rad')
        return {'planned_tcp_max_error_m': maximum_position,
                'planned_orientation_max_error_rad': maximum_orientation}

    def final_tcp_errors(self, expected):
        metrics = self.verify_fk([self.positions], [expected], ['final'])
        position_error = metrics['planned_tcp_max_error_m']
        orientation_error = metrics['planned_orientation_max_error_rad']
        if position_error > self.config['safety']['tcp_position_tolerance']:
            raise TrajectoryValidationError(
                f'final TCP position error: {position_error:.6f} m')
        if orientation_error > self.config['safety']['tcp_orientation_tolerance']:
            raise TrajectoryValidationError(
                f'final TCP orientation error: {orientation_error:.6f} rad')
        return {
            'final_tcp_error_m': position_error,
            'final_tcp_orientation_error_rad': orientation_error,
        }

    def execute(self, trajectory, target, label):
        duration = trajectory.joint_trajectory.points[-1].time_from_start
        expected_duration = duration.sec + duration.nanosec * 1e-9
        future = self.execute_client.send_goal_async(ExecuteTrajectory.Goal(
            trajectory=trajectory, controller_names=['joint_trajectory_controller']))
        handle = self._wait_future(future, 10.0, f'{label} goal')
        if not handle.accepted:
            raise RuntimeError(f'{label} execution goal rejected')
        result_future = handle.get_result_async()
        deadline = time.monotonic() + expected_duration + self.config['timeouts']['execution_slack']
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if not result_future.done():
            cancel = self._wait_future(handle.cancel_goal_async(), 5.0, f'{label} cancellation')
            if not cancel.goals_canceling and not result_future.done():
                raise RuntimeError(f'{label} cancellation rejected; follow-up motion suppressed')
        terminal = self._wait_future(result_future, 5.0, f'{label} terminal result')
        if not terminal_allows_next(terminal.status):
            raise RuntimeError(
                f'{label} returned non-terminal status {terminal.status}; follow-up motion suppressed')
        if terminal.result is None or terminal.result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = None if terminal.result is None else terminal.result.error_code.val
            raise RuntimeError(f'{label} execution failed with code {code}')
        stable_since = None
        deadline = time.monotonic() + 10.0
        error = math.inf
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.positions is None:
                continue
            error = max(abs(a - b) for a, b in zip(self.positions, target))
            if error <= self.config['safety']['final_joint_tolerance']:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 0.5:
                    return {
                        'status': terminal_status_name(terminal.status),
                        'status_code': int(terminal.status),
                        'final_max_joint_error_rad': error,
                    }
            else:
                stable_since = None
        feedback_age = (math.inf if self.received_wall is None
                        else time.monotonic() - self.received_wall)
        raise TimeoutError(
            f'{label} final joint state did not settle; last max error '
            f'{error:.6f} rad, feedback age {feedback_age:.3f} s')
