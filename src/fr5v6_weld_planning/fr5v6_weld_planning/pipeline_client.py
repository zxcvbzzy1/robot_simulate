"""Task-manager client of the two independent Python algorithm services."""
import rclpy
from fr5v6_weld_interfaces.srv import DetectSeams, GeneratePaths

from .pipeline import job_identity, unpack


class PipelineError(RuntimeError):
    pass


def call(node, service_type, name, request, startup, timeout):
    client = node.create_client(service_type, name)
    try:
        if not client.wait_for_service(timeout_sec=float(startup)):
            raise PipelineError(f'{name}: service unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=float(timeout))
        if not future.done():
            future.cancel()
            raise PipelineError(f'{name}: timeout; no motion will be sent')
        result = future.result()
        if result is None or not result.success:
            raise PipelineError(f'{name}: {result.error if result else "empty response"}')
        return result
    finally:
        node.destroy_client(client)


def prepare_via_nodes(node, config_path, config):
    identity = job_identity(config_path)
    timeouts = config['timeouts']
    timeout = float(timeouts.get('algorithm', timeouts['startup']))
    detected = call(node, DetectSeams, '/weld/detect_seams',
                    DetectSeams.Request(job_id=identity, config_file=str(config_path.resolve())),
                    timeouts['startup'], timeout)
    if detected.job_id != identity:
        raise PipelineError('detection job identity mismatch')
    generated = call(node, GeneratePaths, '/weld/generate_paths',
                     GeneratePaths.Request(job_id=identity, config_file=str(config_path.resolve()),
                                           mesh_sha256=detected.mesh_sha256,
                                           candidates=detected.candidates),
                     timeouts['startup'], timeout)
    try:
        prepared = unpack(config_path.resolve(), identity, detected, generated)
    except ValueError as error:
        raise PipelineError(f'algorithm response validation: {error}') from error
    node.get_logger().info(f'Python node pipeline: {len(prepared)} seams, job {identity[:12]}')
    return prepared
