"""Synchronize the exact same STL and pose to MoveIt and Gazebo."""

from pathlib import Path
import subprocess

from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from shape_msgs.msg import Mesh, MeshTriangle

from .transforms import quaternion_from_matrix, rotation_from_rpy


class SceneSyncError(RuntimeError):
    pass


def pose_message(pose):
    result = Pose()
    result.position.x, result.position.y, result.position.z = map(float, pose['xyz'])
    quaternion = quaternion_from_matrix(rotation_from_rpy(pose['rpy']))
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = map(float, quaternion)
    return result


def mesh_message(mesh):
    result = Mesh()
    result.vertices = [Point(x=float(value[0]), y=float(value[1]), z=float(value[2]))
                       for value in mesh.vertices]
    result.triangles = [MeshTriangle(vertex_indices=[int(index) for index in triangle])
                        for triangle in mesh.triangles]
    return result


def collision_object(object_id, mesh, pose):
    result = CollisionObject()
    result.header.frame_id = 'world'
    result.id = object_id
    result.operation = CollisionObject.ADD
    result.meshes = [mesh_message(mesh)]
    result.mesh_poses = [pose_message(pose)]
    return result


def workpiece_sdf(mesh_path, scale):
    uri = Path(mesh_path).resolve().as_uri()
    scale_text = ' '.join(str(float(value)) for value in scale)
    return (
        "<?xml version='1.0'?><sdf version='1.9'>"
        "<model name='workpiece'><static>true</static><link name='workpiece_link'>"
        f"<collision name='collision'><geometry><mesh><uri>{uri}</uri><scale>{scale_text}</scale>"
        "</mesh></geometry></collision>"
        f"<visual name='visual'><geometry><mesh><uri>{uri}</uri><scale>{scale_text}</scale>"
        "</mesh></geometry><material><ambient>0.45 0.45 0.5 1</ambient>"
        "<diffuse>0.55 0.55 0.62 1</diffuse></material></visual>"
        "</link></model></sdf>"
    )


class WorkpieceSynchronizer:
    def __init__(self, node, timeout=30.0, world='empty'):
        self.node = node
        self.timeout = float(timeout)
        self.world = world
        self.apply_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_client = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.spawn_client = node.create_client(SpawnEntity, f'/world/{world}/create')
        self.delete_client = node.create_client(DeleteEntity, f'/world/{world}/remove')

    def _call(self, client, request, label):
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise SceneSyncError(f'{label} service unavailable')
        future = client.call_async(request)
        import rclpy
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        if not future.done() or future.exception() is not None:
            raise SceneSyncError(f'{label} failed or timed out')
        return future.result()

    def _gazebo_names(self):
        result = subprocess.run(['gz', 'model', '--list', '--force-version', '8'],
                                text=True, capture_output=True, timeout=self.timeout, check=False)
        if result.returncode != 0:
            raise SceneSyncError(f'Gazebo model list failed: {result.stderr.strip()}')
        return {line.strip()[2:] for line in result.stdout.splitlines()
                if line.strip().startswith('- ')}

    def apply(self, mesh, mesh_path, pose, scale, object_id='workpiece'):
        if object_id in self._gazebo_names():
            request = DeleteEntity.Request()
            request.entity = Entity(name=object_id, type=Entity.MODEL)
            response = self._call(self.delete_client, request, 'Gazebo workpiece delete')
            if not response.success:
                raise SceneSyncError('Gazebo rejected workpiece deletion')
        scene_request = GetPlanningScene.Request(
            components=PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_NAMES))
        before = self._call(self.get_client, scene_request, 'MoveIt scene read')
        existing = {item.id for item in before.scene.world.collision_objects}
        if object_id in existing:
            remove = CollisionObject()
            remove.header.frame_id = 'world'
            remove.id = object_id
            remove.operation = CollisionObject.REMOVE
            response = self._call(
                self.apply_client, ApplyPlanningScene.Request(scene=_scene_diff([remove])),
                'MoveIt workpiece removal')
            if not response.success:
                raise SceneSyncError('MoveIt rejected workpiece removal')
        item = collision_object(object_id, mesh, pose)
        response = self._call(
            self.apply_client, ApplyPlanningScene.Request(scene=_scene_diff([item])),
            'MoveIt workpiece apply')
        if not response.success:
            raise SceneSyncError('MoveIt rejected workpiece mesh')
        factory = EntityFactory(name=object_id, allow_renaming=False,
                                sdf=workpiece_sdf(mesh_path, scale),
                                pose=pose_message(pose), relative_to='world')
        response = self._call(self.spawn_client, SpawnEntity.Request(entity_factory=factory),
                              'Gazebo workpiece spawn')
        if not response.success:
            raise SceneSyncError('Gazebo rejected workpiece mesh')
        after = self._call(self.get_client, scene_request, 'MoveIt scene verification')
        if object_id not in {value.id for value in after.scene.world.collision_objects}:
            raise SceneSyncError('workpiece missing from MoveIt PlanningScene after apply')
        if object_id not in self._gazebo_names():
            raise SceneSyncError('workpiece missing from Gazebo after spawn')


def _scene_diff(objects):
    from moveit_msgs.msg import PlanningScene
    result = PlanningScene(is_diff=True)
    result.world.collision_objects = list(objects)
    return result
