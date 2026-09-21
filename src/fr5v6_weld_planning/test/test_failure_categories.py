import pytest

from fr5v6_weld_planning.config import ConfigError
from fr5v6_weld_planning.mesh_loader import MeshError
from fr5v6_weld_planning.moveit_executor import TrajectoryValidationError
from fr5v6_weld_planning.run_weld_demo import failure_category
from fr5v6_weld_planning.scene_manager import SceneSyncError
from fr5v6_weld_planning.seam_extractor import SeamExtractionError


@pytest.mark.parametrize(('error', 'category'), [
    (ConfigError('bad config'), 'configuration'),
    (MeshError('bad mesh'), 'stl'),
    (SeamExtractionError('no valid seam candidates'), 'candidate'),
    (SeamExtractionError('zero or cancelling surface normal at point 4'), 'pose'),
    (SceneSyncError('spawn failed'), 'scene_sync'),
    (TrajectoryValidationError('IK failed at seam[4]'), 'ik'),
    (TrajectoryValidationError('joint 2 limit violation'), 'joint_limit'),
    (TrajectoryValidationError('joint jump at point 3'), 'joint_jump'),
    (TrajectoryValidationError('collision at densified state 7'), 'collision'),
    (RuntimeError('weld execution goal rejected'), 'execution'),
    (RuntimeError('non-terminal status 0; follow-up motion suppressed'), 'terminal_unknown'),
])
def test_failure_category_is_stable(error, category):
    assert failure_category(error) == category
