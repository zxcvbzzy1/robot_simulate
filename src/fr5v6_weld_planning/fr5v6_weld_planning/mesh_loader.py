"""Small, dependency-free (apart from NumPy) ASCII/binary STL reader."""

from dataclasses import dataclass
from pathlib import Path
import re
import struct

import numpy as np


class MeshError(ValueError):
    """Raised when a mesh cannot be used safely."""


@dataclass(frozen=True)
class MeshData:
    vertices: np.ndarray
    triangles: np.ndarray
    face_normals: np.ndarray


_VERTEX = re.compile(
    rb'^\s*vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*$',
    re.MULTILINE,
)


def _raw_triangles(data: bytes) -> np.ndarray:
    if len(data) >= 84:
        count = struct.unpack_from('<I', data, 80)[0]
        if len(data) == 84 + count * 50:
            result = np.empty((count, 3, 3), dtype=float)
            for index in range(count):
                values = struct.unpack_from('<12fH', data, 84 + index * 50)
                result[index] = np.asarray(values[3:12]).reshape(3, 3)
            return result
    matches = _VERTEX.findall(data)
    if not matches or len(matches) % 3:
        raise MeshError('invalid ASCII or binary STL')
    return np.asarray([[float(value) for value in row] for row in matches]).reshape(-1, 3, 3)


def load_stl(path, scale=1.0) -> MeshData:
    """Load STL and return indexed triangles in metres."""
    scale = float(scale)
    if not np.isfinite(scale) or scale <= 0:
        raise MeshError('scale must be a positive finite number')
    source = Path(path)
    try:
        facets = _raw_triangles(source.read_bytes()) * scale
    except OSError as error:
        raise MeshError(f'unable to read STL: {source}') from error
    if not len(facets) or not np.isfinite(facets).all():
        raise MeshError('STL contains no finite triangles')

    vertices = []
    lookup = {}
    triangles = []
    for facet in facets:
        indices = []
        for vertex in facet:
            key = tuple(float(value) for value in vertex)
            if key not in lookup:
                lookup[key] = len(vertices)
                vertices.append(vertex.copy())
            indices.append(lookup[key])
        triangles.append(indices)
    vertices = np.asarray(vertices, dtype=float)
    triangles = np.asarray(triangles, dtype=np.int64)
    cross = np.cross(
        vertices[triangles[:, 1]] - vertices[triangles[:, 0]],
        vertices[triangles[:, 2]] - vertices[triangles[:, 0]],
    )
    magnitudes = np.linalg.norm(cross, axis=1)
    if np.any(magnitudes <= 1e-12):
        raise MeshError('STL contains a degenerate triangle')
    normals = cross / magnitudes[:, None]

    counts = {}
    for triangle in triangles:
        for first, second in zip(triangle, np.roll(triangle, -1)):
            edge = tuple(sorted((int(first), int(second))))
            counts[edge] = counts.get(edge, 0) + 1
    bad = [edge for edge, count in counts.items() if count > 2]
    if bad:
        raise MeshError(f'STL contains non-manifold edges: {len(bad)}')
    return MeshData(vertices, triangles, normals)
