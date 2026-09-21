"""Generate a deterministic, metre-scale T coupon for simulation acceptance."""

from pathlib import Path

import numpy as np


def _triangle_text(points):
    normal = np.cross(points[1] - points[0], points[2] - points[0])
    normal = normal / np.linalg.norm(normal)
    rows = [f'  facet normal {normal[0]:.9g} {normal[1]:.9g} {normal[2]:.9g}',
            '    outer loop']
    rows.extend(f'      vertex {point[0]:.9g} {point[1]:.9g} {point[2]:.9g}' for point in points)
    rows.extend(['    endloop', '  endfacet'])
    return '\n'.join(rows)


def create_t_coupon_stl(path):
    """Write a closed extruded T profile with two 200 mm concave seam edges."""
    polygon = [
        (-0.050, -0.010), (0.050, -0.010), (0.050, 0.000),
        (0.005, 0.000), (0.005, 0.050), (-0.005, 0.050),
        (-0.005, 0.000), (-0.050, 0.000),
    ]
    ys = (-0.100, 0.100)
    vertices = np.asarray([[x, y, z] for y in ys for x, z in polygon], dtype=float)
    faces = []
    count = len(polygon)
    for index in range(count):
        following = (index + 1) % count
        low_a, low_b = index, following
        high_a, high_b = count + index, count + following
        faces.extend([(low_a, high_b, low_b), (low_a, high_a, high_b)])
    # The two rectangles exactly tile the concave T cross-section.
    cap_quads = [(0, 1, 2, 7), (6, 3, 4, 5)]
    for a, b, c, d in cap_quads:
        faces.extend([(a, b, c), (a, c, d)])  # y=-0.1, outward -Y
        faces.extend([(count + a, count + c, count + b),
                      (count + a, count + d, count + c)])  # outward +Y
    lines = ['solid fr5_t_coupon']
    lines.extend(_triangle_text(vertices[list(face)]) for face in faces)
    lines.append('endsolid fr5_t_coupon')
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text('\n'.join(lines) + '\n', encoding='ascii')
    return target
