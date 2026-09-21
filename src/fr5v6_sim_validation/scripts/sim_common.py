"""Wall-clock bounded ROS checks; simulation time may stop or run slowly."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
from control_msgs.action import FollowJointTrajectory

JOINTS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']

class SimNode(Node):
    def __init__(self, name):
        super().__init__(name)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.positions = {}
        self.received = 0.0
        self.stamp = 0.0
        self.sequence = 0
        self.create_subscription(JointState, '/joint_states', self.on_state, qos_profile_sensor_data)
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.trajectory = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    def on_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions and math.isfinite(positions[j]) for j in JOINTS):
            self.positions = positions
            self.received = time.monotonic()
            self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.sequence += 1

    def spin(self):
        rclpy.spin_once(self, timeout_sec=0.05)

    def wait(self, future, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            self.spin()
        if not future.done():
            raise TimeoutError('ROS response timed out')
        return future.result()

    def fresh(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        return (bool(self.positions) and time.monotonic() - self.received < 1.0
                and now > 0 and abs(now - self.stamp) < 1.0)

    def ready(self, timeout):
        end = time.monotonic() + timeout
        initial = self.get_clock().now().nanoseconds
        pending = None
        active = False
        while rclpy.ok() and time.monotonic() < end:
            self.spin()
            if pending is None and self.controllers.service_is_ready():
                pending = self.controllers.call_async(ListControllers.Request())
            if pending is not None and pending.done():
                states = {c.name: c.state for c in pending.result().controller}
                active = all(states.get(c) == 'active' for c in ['joint_state_broadcaster', 'joint_trajectory_controller'])
                pending = None
            if active and self.trajectory.server_is_ready() and self.fresh() and self.get_clock().now().nanoseconds > initial:
                return
        raise TimeoutError('Controllers, trajectory action, fresh joint states or advancing /clock not ready')
