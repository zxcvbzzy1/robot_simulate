import struct

import numpy as np
import pytest

from fr5v6_weld_planning.mesh_loader import MeshError, load_stl


def test_ascii_stl_is_deduplicated_and_scaled_to_metres(tmp_path):
    path = tmp_path / 'triangle.stl'
    path.write_text(
        'solid t\nfacet normal 0 0 1\nouter loop\n'
        'vertex 0 0 0\nvertex 1000 0 0\nvertex 0 1000 0\n'
        'endloop\nendfacet\nendsolid t\n', encoding='ascii'
    )

    mesh = load_stl(path, scale=0.001)

    np.testing.assert_allclose(mesh.vertices[1], [1.0, 0.0, 0.0])
    assert mesh.triangles.tolist() == [[0, 1, 2]]
    np.testing.assert_allclose(mesh.face_normals[0], [0.0, 0.0, 1.0])


def test_binary_stl_header_starting_with_solid_is_detected_by_size(tmp_path):
    path = tmp_path / 'binary.stl'
    header = b'solid misleading binary' + bytes(80 - len('solid misleading binary'))
    facet = struct.pack('<12fH', 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0)
    path.write_bytes(header + struct.pack('<I', 1) + facet)

    mesh = load_stl(path)

    assert mesh.vertices.shape == (3, 3)
    assert mesh.triangles.shape == (1, 3)


def test_degenerate_triangle_is_rejected(tmp_path):
    path = tmp_path / 'bad.stl'
    path.write_text(
        'solid bad\nfacet normal 0 0 0\nouter loop\n'
        'vertex 0 0 0\nvertex 1 0 0\nvertex 2 0 0\n'
        'endloop\nendfacet\nendsolid bad\n', encoding='ascii'
    )

    with pytest.raises(MeshError, match='degenerate'):
        load_stl(path)
