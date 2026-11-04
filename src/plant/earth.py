"""Geometry for the plant's fixed-origin Earth-fixed Cartesian reference axes.

In rotating mode the axes coincide with NED at the spherical surface origin;
they do not follow the vehicle's local vertical. Fixed mode retains flat geometry.
"""

import numpy as np


def geocentric(position, radius):
    return np.asarray(position, dtype=float) + np.array([0.0, 0.0, -float(radius)])


def altitude(position, params):
    r = np.asarray(position, dtype=float)
    if not bool(params.earth.enabled):
        return -r[..., 2]
    radius = float(params.gravity.radius)
    distance = np.linalg.norm(geocentric(r, radius), axis=-1)
    return (np.sum(r * r, axis=-1) - 2 * radius * r[..., 2]) / (distance + radius)


def down_direction(position, params):
    r = np.asarray(position, dtype=float)
    if not bool(params.earth.enabled):
        return np.broadcast_to([0.0, 0.0, 1.0], r.shape)
    center = geocentric(r, params.gravity.radius)
    return -center / np.linalg.norm(center, axis=-1, keepdims=True)
