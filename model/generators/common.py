"""Shared helpers: MJCF attribute parsing and rotation conversions.

Quaternions are (w, x, y, z), MuJoCo's order; scipy 1.11 (Ubuntu 24.04) takes and returns
(x, y, z, w). Euler angles are URDF rpy: extrinsic x, then y, then z (scipy's lowercase "xyz").
"""
import warnings

import numpy as np
from scipy.spatial.transform import Rotation

def vec(s, n, default):
    if s is None:
        return list(default)
    v = [float(x) for x in s.split()]
    if len(v) != n:
        raise ValueError(f"expected {n} floats, got {s!r}")
    return v

def geom_size(s):
    if s is None:
        raise ValueError("geom has no size and no resolved default size")
    v = [float(x) for x in s.split()]
    if not 1 <= len(v) <= 3:
        raise ValueError(f"geom size must have 1-3 values, got {s!r}")
    if any(x <= 0.0 for x in v):
        raise ValueError(f"geom size values must be positive, got {s!r}")
    return v

def _rotation(q_wxyz):
    """The rotation of a (w, x, y, z) quaternion. scipy normalises it, as MuJoCo does."""
    return Rotation.from_quat(np.roll(q_wxyz, -1))

def quat_to_rpy(q):
    # At pitch +-pi/2 roll and yaw are not unique; scipy warns and returns one valid pair (yaw 0),
    # which is all URDF needs.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", "Gimbal lock detected", UserWarning)
        return _rotation(q).as_euler("xyz").tolist()

def quat_to_matrix(q):
    return _rotation(q).as_matrix()

def rpy_to_quat(rpy):
    """(w, x, y, z), with w >= 0."""
    return np.roll(Rotation.from_euler("xyz", rpy).as_quat(canonical=True), 1).tolist()
