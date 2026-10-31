"""This is a fixed planning seed, never simulated execution feedback."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = Node("weld_planning_state")
    publisher = node.create_publisher(JointState, "/joint_states", 10)

    def publish():
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = [f"j{i}" for i in range(1, 7)]
        import math
        message.position = [math.radians(v) for v in [0, -120, 120, -90, -90, 0]]
        publisher.publish(message)

    node.create_timer(.1, publish)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
