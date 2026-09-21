"""Rigid transform helpers using ROS quaternion ordering (x, y, z, w)."""

import math

import numpy as np


def rotation_from_rpy(rpy):
    roll, pitch, yaw = map(float, rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_from_quaternion(quaternion):
    x, y, z, w = map(float, quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError('zero quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quaternion_from_matrix(matrix):
    matrix = np.asarray(matrix, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        values = [(matrix[2, 1] - matrix[1, 2]) / s,
                  (matrix[0, 2] - matrix[2, 0]) / s,
                  (matrix[1, 0] - matrix[0, 1]) / s, 0.25 * s]
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            s = math.sqrt(1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            values = [0.25 * s, (matrix[0, 1] + matrix[1, 0]) / s,
                      (matrix[0, 2] + matrix[2, 0]) / s, (matrix[2, 1] - matrix[1, 2]) / s]
        elif index == 1:
            s = math.sqrt(1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            values = [(matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s,
                      (matrix[1, 2] + matrix[2, 1]) / s, (matrix[0, 2] - matrix[2, 0]) / s]
        else:
            s = math.sqrt(1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            values = [(matrix[0, 2] + matrix[2, 0]) / s,
                      (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s,
                      (matrix[1, 0] - matrix[0, 1]) / s]
    values = np.asarray(values)
    return values / np.linalg.norm(values)


def pose_matrix(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = rotation_from_rpy(rpy)
    result[:3, 3] = np.asarray(xyz, dtype=float)
    return result


def transform_pose_values(position, quaternion, transform):
    transform = np.asarray(transform, dtype=float)
    point = transform @ np.r_[np.asarray(position, dtype=float), 1.0]
    rotation = transform[:3, :3] @ matrix_from_quaternion(quaternion)
    return point[:3], quaternion_from_matrix(rotation)
