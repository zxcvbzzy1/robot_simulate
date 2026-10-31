"""Known-geometry fixtures for offline verification, never robot commands.

Coordinates and expected seams are specified independently of the extractor.
"""

import numpy as np


def fixture_cloud(model_type="three_plane", *, noise_mm=0.0, seed=18,
                  nonuniform=False, outliers=False, yaw_deg=0.0, translation=(0, 0, 0)):
    rng = np.random.default_rng(seed)

    def surface(axis, value, first, second):
        other = [i for i in range(3) if i != axis]
        u, v = np.meshgrid(np.linspace(*first, round(first[1] - first[0]) + 1),
                           np.linspace(*second, round(second[1] - second[0]) + 1))
        cloud = np.empty((u.size, 3))
        cloud[:, axis] = value
        cloud[:, other[0]], cloud[:, other[1]] = u.ravel(), v.ravel()
        return cloud

    if model_type == "three_plane":
        faces = [surface(2, 0, (0, 60), (0, 60)), surface(0, 0, (0, 60), (0, 60)),
                 surface(1, 0, (0, 60), (0, 60))]
        truth = [([0, 0, 60], [0, 0, 0]), ([0, 0, 0], [0, 60, 0]), ([0, 0, 0], [60, 0, 0])]
        dimensions = {}
    elif model_type == "h_section":
        faces = [surface(2, 0, (0, 60), (-20, 60)), surface(0, 0, (0, 60), (0, 60)),
                 surface(1, 0, (0, 60), (0, 60)), surface(2, 60, (0, 60), (-20, 60))]
        truth = [([0, 0, 50], [0, 0, 10]), ([20, -20, 60], [20, 20, 60]),
                 ([0, 0, 0], [20, 0, 0]), ([20, 0, 0], [20, 20, 0])]
        dimensions = {"h_width_mm": 40.0}
    elif model_type == "square_tube":
        faces = [surface(2, 0, (0, 60), (-30, 60)), surface(0, 0, (-30, 60), (0, 70)),
                 surface(1, 0, (0, 60), (0, 30)), surface(2, 30, (0, 60), (-30, 0)),
                 surface(2, 70, (0, 5), (-30, 60))]
        truth = [([0, 0, 30], [0, 0, 0]), ([0, 0, 0], [0, 60, 0]),
                 ([0, 0, 0], [60, 0, 0]), ([0, 0, 30], [0, -30, 30])]
        dimensions = {"tube_width_mm": 30.0}
    else:
        raise ValueError(model_type)
    points = np.unique(np.concatenate(faces), axis=0)
    if nonuniform:
        keep = (points[:, 0] <= 30) | (rng.random(len(points)) < 0.45)
        points = points[keep]
    if noise_mm:
        points = points + rng.normal(0, noise_mm, points.shape)
    if outliers:
        points = np.vstack((points, rng.uniform([-5, -35, -5], [65, 65, 75], (80, 3))))
    angle = np.deg2rad(yaw_deg)
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    translation = np.asarray(translation)
    points = points @ rotation.T + translation
    truth = np.asarray(truth) @ rotation.T + translation
    viewpoint = np.array([180, 180, 180]) @ rotation.T + translation
    rng.shuffle(points)
    return points, truth, viewpoint, dimensions
