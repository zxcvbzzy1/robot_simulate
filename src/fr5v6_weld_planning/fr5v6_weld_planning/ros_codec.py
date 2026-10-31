"""Lossless dataclass <-> ROS messages. No ROS node or algorithm lives here."""
import numpy as np
from geometry_msgs.msg import Point, Quaternion, Vector3
from fr5v6_weld_interfaces import msg

from .seam_extractor import SeamCandidate, SeamEdge
from .seam_processor import SampledPoint
from .pose_generator import WeldPathPoint


def vector(value, cls=Vector3):
    return cls(**dict(zip(('x', 'y', 'z', 'w'), map(float, value))))


def array(value, quaternion=False):
    result = np.array([getattr(value, key) for key in ('xyzw' if quaternion else 'xyz')])
    if not np.all(np.isfinite(result)):
        raise ValueError('ROS geometry must be finite')
    return result


def candidate_to_msg(value):
    return msg.SeamCandidate(
        name=value.name, vertex_indices=list(value.vertex_indices),
        points=[vector(p, Point) for p in value.points],
        normals_1=[vector(p) for p in value.normals_1],
        normals_2=[vector(p) for p in value.normals_2],
        edges=[msg.SeamEdge(vertex_indices=list(e.vertices), normal_1=vector(e.normal_1),
                            normal_2=vector(e.normal_2), dihedral_deg=float(e.dihedral_deg),
                            concave=bool(e.concave)) for e in value.edges],
        length=float(value.length), concave=bool(value.concave))


def candidate_from_msg(value):
    count = len(value.points)
    if (not value.name or count < 2 or len(value.vertex_indices) != count
            or len(value.normals_1) != count or len(value.normals_2) != count
            or len(value.edges) not in (count - 1, count)
            or not np.isfinite(value.length) or value.length <= 0):
        raise ValueError('invalid candidate geometry or topology')
    edges = []
    for e in value.edges:
        if not np.isfinite(e.dihedral_deg):
            raise ValueError('dihedral must be finite')
        edges.append(SeamEdge(tuple(e.vertex_indices), array(e.normal_1), array(e.normal_2),
                              e.dihedral_deg, e.concave))
    return SeamCandidate(value.name, tuple(value.vertex_indices),
                         np.array([array(p) for p in value.points]),
                         np.array([array(p) for p in value.normals_1]),
                         np.array([array(p) for p in value.normals_2]),
                         tuple(edges), value.length, value.concave)


def path_to_msg(prepared):
    return msg.WeldPath(candidate_name=prepared.selected.name,
        samples=[msg.SampledPoint(position=vector(p.position, Point), tangent=vector(p.tangent),
                                 normal_1=vector(p.normal_1), normal_2=vector(p.normal_2))
                 for p in prepared.samples],
        points=[msg.WeldPathPoint(segment=p.segment, position=vector(p.position, Point),
                                 quaternion=vector(p.quaternion, Quaternion),
                                 **{key: vector(getattr(p, key)) for key in
                                    ('tangent', 'normal_1', 'normal_2', 'torch_x', 'torch_y', 'torch_z')})
                for p in prepared.path])


def path_from_msg(value):
    if len(value.samples) < 2 or not value.points:
        raise ValueError('empty or incomplete weld path')
    samples = [SampledPoint(array(p.position), array(p.tangent), array(p.normal_1), array(p.normal_2))
               for p in value.samples]
    points = []
    for index, p in enumerate(value.points):
        if p.segment not in ('approach', 'seam', 'retreat'):
            raise ValueError(f'point {index}: invalid segment')
        q = array(p.quaternion, True)
        if not np.isclose(np.linalg.norm(q), 1.0, atol=1e-6):
            raise ValueError(f'point {index}: non-unit quaternion')
        points.append(WeldPathPoint(p.segment, array(p.position), q, *[
            array(getattr(p, key)) for key in
            ('tangent', 'normal_1', 'normal_2', 'torch_x', 'torch_y', 'torch_z')]))
    seam = [p for p in points if p.segment == 'seam']
    if len(seam) != len(samples):
        raise ValueError('incomplete seam samples')
    order = {'approach': 0, 'seam': 1, 'retreat': 2}
    if any(order[b.segment] < order[a.segment] for a, b in zip(points, points[1:])):
        raise ValueError('invalid path segment order')
    for p, s in zip(seam, samples):
        if not np.allclose(p.position, s.position, atol=1e-9, rtol=0):
            raise ValueError('seam sample order or position changed')
    return samples, points
