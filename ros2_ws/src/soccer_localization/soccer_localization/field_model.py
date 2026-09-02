"""Field model + likelihood-field map (localization report §3.1, §6).

Holds the exact field geometry (lines + centre circle + goalposts) and bakes a
**distance-transform map**: a grid whose value at each cell is the metric
distance to the nearest field line. Weighting an observed line point is then an
O(1) lookup — the efficient form of Chamfer matching the top RoboCup teams use
(Bit-Bots ``lines.png``). ``SoccerbotField`` is the single source of field truth,
reused by both the MCL and any visualization.

The grid stores **distance in metres**, not a pre-baked Gaussian. That is a
deliberate change: the ground-projection uncertainty of an observed line point
grows with the square of its range (see
:class:`soccer_localization.sensor_model.GroundProjectionNoise`), so the Gaussian
falloff has to be evaluated per observation instead of being frozen into the map
at one fixed ``line_sigma``.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt

    _HAVE_SCIPY = True
except Exception:  # pragma: no cover
    _HAVE_SCIPY = False


@dataclass
class SoccerbotField:
    """A small symmetric soccer field, centred at the origin (metres)."""

    length: float = 6.0          # x extent (touchline to touchline)
    width: float = 4.0           # y extent
    centre_circle_r: float = 0.75
    resolution: float = 0.05     # distance-field grid cell size (m)
    line_sigma: float = 0.15     # default falloff when no per-point sigma is given
    off_grid_distance: float = 2.0   # distance reported for points off the map (m)
    goalposts: list = field(default_factory=list)

    def __post_init__(self) -> None:
        hl, hw = self.length / 2.0, self.width / 2.0
        # Goalposts at both goals (the asymmetry-breaking landmarks).
        self.goalposts = [
            (-hl, -1.0), (-hl, 1.0),  # own goal
            (hl, -1.0), (hl, 1.0),    # opponent goal
        ]
        self._segments = self._build_segments()
        self._dist, self._origin = self._bake_distance_field()

    # ── Geometry ──
    def _build_segments(self) -> list[tuple[float, float, float, float]]:
        hl, hw = self.length / 2.0, self.width / 2.0
        segs = [
            (-hl, -hw, hl, -hw), (-hl, hw, hl, hw),    # touchlines
            (-hl, -hw, -hl, hw), (hl, -hw, hl, hw),    # goal lines
            (0.0, -hw, 0.0, hw),                        # halfway line
        ]
        # Approximate the centre circle with chords.
        n = 24
        for i in range(n):
            a0 = 2 * np.pi * i / n
            a1 = 2 * np.pi * (i + 1) / n
            segs.append((
                self.centre_circle_r * np.cos(a0), self.centre_circle_r * np.sin(a0),
                self.centre_circle_r * np.cos(a1), self.centre_circle_r * np.sin(a1),
            ))
        return segs

    # ── Distance field ──
    def _bake_distance_field(self):
        margin = 0.5
        hl, hw = self.length / 2.0 + margin, self.width / 2.0 + margin
        nx = int(2 * hl / self.resolution)
        ny = int(2 * hw / self.resolution)
        occ = np.ones((ny, nx), dtype=np.uint8)  # 1 = free, 0 = on a line
        origin = (-hl, -hw)

        def to_cell(x, y):
            return (int((x - origin[0]) / self.resolution),
                    int((y - origin[1]) / self.resolution))

        for (x0, y0, x1, y1) in self._segments:
            steps = int(max(abs(x1 - x0), abs(y1 - y0)) / self.resolution) + 1
            for t in np.linspace(0.0, 1.0, steps):
                cx, cy = to_cell(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
                if 0 <= cx < nx and 0 <= cy < ny:
                    occ[cy, cx] = 0

        if _HAVE_SCIPY:
            dist = distance_transform_edt(occ) * self.resolution
        else:  # coarse fallback: 0 on lines, "far" elsewhere
            dist = np.where(occ == 0, 0.0, self.off_grid_distance)
        return dist.astype(np.float32), origin

    # ── Lookups ──
    def line_distance(self, pts_xy: np.ndarray) -> np.ndarray:
        """Metres to the nearest field line, for world-frame points.

        Accepts any array shaped ``(..., 2)`` and returns the matching ``(...)``
        distances, so an entire particle set can be scored in a single call.
        Points outside the baked map report :attr:`off_grid_distance`.
        """
        pts = np.asarray(pts_xy, dtype=np.float64)
        ny, nx = self._dist.shape
        cx = ((pts[..., 0] - self._origin[0]) / self.resolution).astype(np.intp)
        cy = ((pts[..., 1] - self._origin[1]) / self.resolution).astype(np.intp)
        inside = (cx >= 0) & (cx < nx) & (cy >= 0) & (cy < ny)
        np.clip(cx, 0, nx - 1, out=cx)
        np.clip(cy, 0, ny - 1, out=cy)
        return np.where(inside, self._dist[cy, cx], self.off_grid_distance)

    def line_likelihood(self, pts_xy: np.ndarray, sigma=None) -> np.ndarray:
        """Gaussian likelihood in (0,1] for world-frame points shaped ``(..., 2)``.

        ``sigma`` may be a scalar or an array broadcastable against the point
        axis — that is how the MCL applies a per-observation, range-dependent
        uncertainty to a single shared map.
        """
        s = self.line_sigma if sigma is None else np.asarray(sigma, dtype=np.float64)
        return np.exp(-0.5 * (self.line_distance(pts_xy) / s) ** 2)

    # ── Sampling ──
    def random_poses(self, rng: np.random.Generator, n: int) -> np.ndarray:
        """``n`` uniformly random on-field poses ``[x, y, theta]``, shape (n, 3)."""
        hl, hw = self.length / 2.0, self.width / 2.0
        out = np.empty((n, 3))
        out[:, 0] = rng.uniform(-hl, hl, n)
        out[:, 1] = rng.uniform(-hw, hw, n)
        out[:, 2] = rng.uniform(-np.pi, np.pi, n)
        return out

    def random_pose(self, rng: np.random.Generator) -> np.ndarray:
        """A uniformly random on-field pose [x, y, theta] (for explorer particles)."""
        return self.random_poses(rng, 1)[0]
