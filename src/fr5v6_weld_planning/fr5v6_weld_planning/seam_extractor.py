"""Detect, cluster and order sharp shared edges in a triangle mesh."""

from dataclasses import dataclass
import math

import numpy as np

from .mesh_loader import MeshData


class SeamExtractionError(ValueError):
    pass


@dataclass(frozen=True)
class SeamEdge:
    vertices: tuple[int, int]
    normal_1: np.ndarray
    normal_2: np.ndarray
    dihedral_deg: float
    concave: bool


@dataclass(frozen=True)
class SeamCandidate:
    name: str
    vertex_indices: tuple[int, ...]
    points: np.ndarray
    normals_1: np.ndarray
    normals_2: np.ndarray
    edges: tuple[SeamEdge, ...]
    length: float
    concave: bool


def _shared_edges(mesh: MeshData, config) -> list[SeamEdge]:
    minimum = float(config['min_dihedral_deg'])
    maximum = float(config['max_dihedral_deg'])
    if not 0 <= minimum <= maximum <= 180:
        raise SeamExtractionError('invalid dihedral angle range')
    owners = {}
    for face_index, triangle in enumerate(mesh.triangles):
        for first, second in zip(triangle, np.roll(triangle, -1)):
            key = tuple(sorted((int(first), int(second))))
            owners.setdefault(key, []).append(face_index)
    if any(len(faces) > 2 for faces in owners.values()):
        raise SeamExtractionError('non-manifold shared edge')
    centroids = mesh.vertices[mesh.triangles].mean(axis=1)
    result = []
    for vertices, faces in owners.items():
        if len(faces) != 2:
            continue
        first, second = faces
        n1, n2 = mesh.face_normals[first], mesh.face_normals[second]
        angle = math.degrees(math.acos(float(np.clip(np.dot(n1, n2), -1.0, 1.0))))
        if angle < minimum or angle > maximum:
            continue
        midpoint = mesh.vertices[list(vertices)].mean(axis=0)
        score = float(np.dot(centroids[second] - midpoint, n1)
                      + np.dot(centroids[first] - midpoint, n2))
        result.append(SeamEdge(vertices, n1.copy(), n2.copy(), angle, score > 1e-10))
    return result


def _components(edges, mesh, max_turn_deg):
    vertex_edges = {}
    for index, edge in enumerate(edges):
        for vertex in edge.vertices:
            vertex_edges.setdefault(vertex, set()).add(index)
    pending = set(range(len(edges)))
    while pending:
        seed = pending.pop()
        component = {seed}
        frontier = [seed]
        while frontier:
            index = frontier.pop()
            neighbours = set()
            for vertex in edges[index].vertices:
                base = mesh.vertices[edges[index].vertices[1]] - mesh.vertices[edges[index].vertices[0]]
                base /= np.linalg.norm(base)
                for candidate in vertex_edges[vertex]:
                    direction = (mesh.vertices[edges[candidate].vertices[1]]
                                 - mesh.vertices[edges[candidate].vertices[0]])
                    direction /= np.linalg.norm(direction)
                    turn = math.degrees(math.acos(float(np.clip(abs(np.dot(base, direction)), 0, 1))))
                    if turn <= max_turn_deg:
                        neighbours.add(candidate)
            fresh = neighbours.intersection(pending)
            pending.difference_update(fresh)
            component.update(fresh)
            frontier.extend(fresh)
        yield [edges[index] for index in component]


def _ordered_vertices(component, mesh):
    adjacency = {}
    edge_by_pair = {}
    for edge in component:
        first, second = edge.vertices
        adjacency.setdefault(first, []).append(second)
        adjacency.setdefault(second, []).append(first)
        edge_by_pair[frozenset((first, second))] = edge
    if any(len(values) > 2 for values in adjacency.values()):
        raise SeamExtractionError('candidate edge chain branches')
    endpoints = [vertex for vertex, neighbours in adjacency.items() if len(neighbours) == 1]
    if len(endpoints) not in (0, 2):
        raise SeamExtractionError('candidate edge chain is disconnected')
    pool = endpoints or list(adjacency)
    start = min(pool, key=lambda vertex: tuple(mesh.vertices[vertex]))
    ordered = [start]
    previous = None
    current = start
    ordered_edges = []
    while True:
        choices = [value for value in adjacency[current] if value != previous]
        if not choices:
            break
        following = min(choices, key=lambda vertex: tuple(mesh.vertices[vertex]))
        if following == start:
            ordered_edges.append(edge_by_pair[frozenset((current, following))])
            break
        if following in ordered:
            raise SeamExtractionError('candidate edge chain repeats a vertex')
        ordered_edges.append(edge_by_pair[frozenset((current, following))])
        ordered.append(following)
        previous, current = current, following
    return ordered, ordered_edges


def extract_candidates(mesh: MeshData, config) -> list[SeamCandidate]:
    edges = _shared_edges(mesh, config)
    minimum_length = float(config['min_seam_length'])
    max_turn_deg = float(config.get('max_chain_turn_deg', 20.0))
    candidates = []
    for component in _components(edges, mesh, max_turn_deg):
        ordered, ordered_edges = _ordered_vertices(component, mesh)
        points = mesh.vertices[ordered]
        length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        if len(ordered_edges) == len(ordered):  # closed loop closing edge
            length += float(np.linalg.norm(points[-1] - points[0]))
        if length + 1e-12 < minimum_length:
            continue
        normals_1, normals_2 = [], []
        for position in range(len(ordered)):
            touching = [edge for edge in ordered_edges if ordered[position] in edge.vertices]
            n1 = np.mean([edge.normal_1 for edge in touching], axis=0)
            n2 = np.mean([edge.normal_2 for edge in touching], axis=0)
            normals_1.append(n1 / np.linalg.norm(n1))
            normals_2.append(n2 / np.linalg.norm(n2))
        candidates.append(SeamCandidate(
            '', tuple(ordered), points.copy(), np.asarray(normals_1), np.asarray(normals_2),
            tuple(ordered_edges), length, sum(edge.concave for edge in component) * 2 >= len(component),
        ))
    prefer_concave = bool(config.get('prefer_concave', True))
    candidates.sort(key=lambda item: (
        -(1 if prefer_concave and item.concave else 0), -item.length,
        tuple(item.points[0]),
    ))
    return [SeamCandidate(
        f'candidate_{index}', item.vertex_indices, item.points, item.normals_1,
        item.normals_2, item.edges, item.length, item.concave,
    ) for index, item in enumerate(candidates)]


def select_candidate(candidates, candidate_index=None) -> SeamCandidate:
    if not candidates:
        raise SeamExtractionError('no valid seam candidates')
    if candidate_index is None:
        return candidates[0]
    index = int(candidate_index)
    if index < 0 or index >= len(candidates):
        raise SeamExtractionError(f'candidate_index {index} is out of range')
    return candidates[index]
