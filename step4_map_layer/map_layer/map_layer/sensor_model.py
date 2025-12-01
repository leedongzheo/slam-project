from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np

from .occupancy_grid import OccupancyGrid


@dataclass
class LikelihoodFieldSensorModel:
    """Likelihood field model using the occupancy grid as distance field."""

    z_hit: float = 0.95
    sigma_hit: float = 0.2
    z_rand: float = 0.05
    max_range: float = 3.5

    def compute_weight(
        self,
        grid: OccupancyGrid,
        pose: Tuple[float, float, float],
        ranges: Iterable[float],
        angles: Iterable[float],
    ) -> float:
        x, y, theta = pose
        prob = 1.0
        occ = grid.occupancy_probability()
        for r, a in zip(ranges, angles):
            if not np.isfinite(r):
                continue
            r = min(r, self.max_range)
            wx = x + r * math.cos(theta + a)
            wy = y + r * math.sin(theta + a)
            gx, gy = grid.world_to_grid(wx, wy)
            if 0 <= gx < grid.size_x and 0 <= gy < grid.size_y:
                p = occ[gy, gx]
                hit_prob = self.z_hit * self._gaussian_prob(0.0, self.sigma_hit)
                mix = hit_prob * p + self.z_rand * (1.0 / self.max_range)
            else:
                mix = self.z_rand * (1.0 / self.max_range)
            prob *= max(mix, 1e-6)
        return prob

    def _gaussian_prob(self, mu: float, sigma: float) -> float:
        return 1.0 / math.sqrt(2 * math.pi * sigma**2) * math.exp(-mu**2 / (2 * sigma**2))
