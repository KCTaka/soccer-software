"""Shared helpers. Stdlib only."""
import math

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

def quat_to_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]

def quat_to_matrix(q):
    """Rotation matrix of a (w, x, y, z) quaternion, normalised first as MuJoCo does."""
    n = math.sqrt(sum(c * c for c in q))
    w, x, y, z = (c / n for c in q)
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]

def rpy_to_quat(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
