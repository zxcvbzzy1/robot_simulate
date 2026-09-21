"""RViz MarkerArray and DisplayTrajectory publishing."""

from geometry_msgs.msg import Point
from moveit_msgs.msg import DisplayTrajectory
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .scene_manager import pose_message


def _color(red, green, blue, alpha=1.0):
    return ColorRGBA(r=float(red), g=float(green), b=float(blue), a=float(alpha))


def _point(values):
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def _marker(namespace, marker_id, marker_type, frame='workpiece_frame'):
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def build_markers(candidates, selected, samples, path, mesh_uri, workpiece_pose):
    markers = []
    workpiece = _marker('workpiece', 0, Marker.MESH_RESOURCE, 'world')
    workpiece.mesh_resource = mesh_uri
    workpiece.mesh_use_embedded_materials = False
    workpiece.color = _color(0.55, 0.55, 0.62, 0.65)
    workpiece.pose = pose_message(workpiece_pose)
    workpiece.scale.x = workpiece.scale.y = workpiece.scale.z = 1.0
    markers.append(workpiece)
    for index, candidate in enumerate(candidates):
        marker = _marker('candidate_seams', index, Marker.LINE_STRIP)
        marker.scale.x = 0.0015
        marker.color = _color(0.3, 0.65, 1.0, 0.7)
        marker.points = [_point(value) for value in candidate.points]
        markers.append(marker)
    chosen = _marker('selected_seam', 0, Marker.LINE_STRIP)
    chosen.scale.x = 0.004
    chosen.color = _color(1.0, 0.75, 0.0)
    chosen.points = [_point(value) for value in selected.points]
    markers.append(chosen)
    sampled = _marker('sampled_seam', 0, Marker.SPHERE_LIST)
    sampled.scale.x = sampled.scale.y = sampled.scale.z = 0.006
    sampled.color = _color(1.0, 0.1, 0.8)
    sampled.points = [_point(value.position) for value in samples]
    markers.append(sampled)
    for namespace, axis, color in (
            ('tcp_x', 'torch_x', _color(1, 0, 0)),
            ('tcp_y', 'torch_y', _color(0, 1, 0)),
            ('tcp_z', 'torch_z', _color(0, 0.35, 1))):
        marker = _marker(namespace, 0, Marker.LINE_LIST)
        marker.scale.x = 0.0015
        marker.color = color
        for item in path:
            if item.segment != 'seam':
                continue
            marker.points.extend([_point(item.position),
                                  _point(item.position + getattr(item, axis) * 0.015)])
        markers.append(marker)
    for segment, color in (('approach', _color(0, 1, 1)), ('retreat', _color(1, 0.35, 0))):
        marker = _marker(segment, 0, Marker.SPHERE_LIST)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.006
        marker.color = color
        marker.points = [_point(item.position) for item in path if item.segment == segment]
        markers.append(marker)
    return MarkerArray(markers=markers)


class WeldVisualization:
    def __init__(self, node):
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = node.create_publisher(MarkerArray, '/weld_markers', qos)
        self.trajectory_publisher = node.create_publisher(
            DisplayTrajectory, '/display_planned_path', qos)

    def publish_markers(self, markers):
        self.marker_publisher.publish(markers)

    def publish_trajectories(self, trajectories, start_state):
        message = DisplayTrajectory()
        message.model_id = 'fr5v6'
        message.trajectory_start = start_state
        message.trajectory = list(trajectories)
        self.trajectory_publisher.publish(message)
