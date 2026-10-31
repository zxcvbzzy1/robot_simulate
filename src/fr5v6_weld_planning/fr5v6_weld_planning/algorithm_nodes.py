"""Independent rclpy servers: detection and path generation, never execution."""
import rclpy
from rclpy.node import Node
from fr5v6_weld_interfaces.srv import DetectSeams, GeneratePaths

from .pipeline import detect, generate


class SeamDetector(Node):
    def __init__(self):
        super().__init__('weld_seam_detector')
        self.service = self.create_service(DetectSeams, '/weld/detect_seams', self.detect_callback)
        self.get_logger().info('Ready: /weld/detect_seams')

    def detect_callback(self, request, response):
        try:
            result = detect(request.config_file, request.job_id)
            self.get_logger().info(f'{request.job_id[:12]}: detected {len(result.candidates)} candidates')
            return result
        except Exception as error:
            response.job_id = request.job_id
            response.success = False
            response.error = f'{type(error).__name__}: {error}'
            self.get_logger().error(response.error)
            return response


class PathGenerator(Node):
    def __init__(self):
        super().__init__('weld_path_generator')
        self.service = self.create_service(GeneratePaths, '/weld/generate_paths', self.generate_callback)
        self.get_logger().info('Ready: /weld/generate_paths')

    def generate_callback(self, request, response):
        try:
            result = generate(request.config_file, request.job_id, request.mesh_sha256, request.candidates)
            self.get_logger().info(f'{request.job_id[:12]}: generated {len(result.paths)} weld paths')
            return result
        except Exception as error:
            response.job_id = request.job_id
            response.success = False
            response.error = f'{type(error).__name__}: {error}'
            self.get_logger().error(response.error)
            return response


def serve(node_type):
    rclpy.init()
    node = node_type()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def detector_main():
    serve(SeamDetector)


def generator_main():
    serve(PathGenerator)
