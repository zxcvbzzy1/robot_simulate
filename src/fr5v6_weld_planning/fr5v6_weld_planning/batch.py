"""Plan the entire job before moving, then execute each transfer and weld."""
from .moveit_executor import build_robot_trajectory, time_parameterize, validate_ready_state
from .workflow import world_path_poses, process_segment_dts


class JobFailure(RuntimeError):
    def __init__(self, result, cause):
        super().__init__(str(cause))
        self.result = result
        self.cause = cause


def run_batch(prepared, executor, config, execute, persist, display):
    result = {'schema_version': 2, 'status': 'planning', 'execute_requested': bool(execute),
              'completed_seams': 0, 'total_seams': len(prepared), 'seams': [],
              'initial_joint_positions': dict(zip(executor.joints, executor.positions))}
    plans = []
    start = list(executor.positions)
    phase = 'preflight'
    active = None
    try:
        for item in prepared:
            active = item.selected.name
            record = {'seam_id': active, 'status': 'planning', 'length_m': item.selected.length,
                      'path_point_count': len(item.path)}
            result['seams'].append(record)
            persist(result)
            poses = world_path_poses(item.path, config['workpiece']['pose_in_world'])
            segments = [point.segment for point in item.path]
            phase = 'ik'
            states = executor.solve_ik(poses, segments, seed=start)
            phase = 'collision_and_fk'
            record['dense_collision_state_count'] = executor.validate_collisions(states)
            record.update(executor.verify_fk(states, poses, segments))
            times = time_parameterize(states, config['motion']['max_joint_velocity'],
                                     config['motion']['max_joint_acceleration'],
                                     config['motion']['minimum_dt'],
                                     process_segment_dts(item.path, config['motion']['speed_mps']))
            weld = build_robot_trajectory(executor.joints, states, times)
            phase = 'transfer_planning'
            transfer = executor.plan_to(states[0], start=start)
            plans.append((transfer, weld, states, poses))
            start = list(states[-1])
            record['status'] = 'planned'
            persist(result)
        display([trajectory for transfer, weld, _, _ in plans for trajectory in (transfer, weld)])
        if not execute:
            result['status'] = 'planned_only'
        else:
            result['status'] = 'executing'
            for record, (transfer, weld, states, poses) in zip(result['seams'], plans):
                active = record['seam_id']
                phase = 'transfer_execution'
                record['status'] = 'executing'
                persist(result)
                executor.ensure_fresh_feedback()
                names = transfer.joint_trajectory.joint_names
                expected = [transfer.joint_trajectory.points[0].positions[names.index(name)]
                            for name in executor.joints]
                validate_ready_state(executor.positions, expected, 0.01)
                record['transfer_execution'] = executor.execute(transfer, states[0], f'{active}/transfer')
                persist(result)
                phase = 'weld_execution'
                record['weld_execution'] = executor.execute(weld, states[-1], f'{active}/weld')
                phase = 'final_tcp_check'
                record['weld_execution'].update(executor.final_tcp_errors(poses[-1]))
                record['status'] = 'succeeded'
                result['completed_seams'] += 1
                persist(result)
            result['status'] = 'succeeded'
        persist(result)
        return result
    except Exception as error:
        result.update(status='failed', failed_seam=active, failure_stage=phase,
                      detail=str(error), followup_motion_suppressed=True)
        if result['seams']:
            next(r for r in result['seams'] if r['seam_id'] == active)['status'] = 'failed'
        persist(result)
        raise JobFailure(result, error) from error
