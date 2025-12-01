from __future__ import annotations

import numpy as np

from .particle import Particle


class LowVarianceResampler:
    """Systematic low-variance resampling for particle filters."""

    def resample(self, particles: list[Particle]) -> list[Particle]:
        if not particles:
            return []
        weights = np.array([p.weight for p in particles], dtype=np.float64)
        weights /= np.sum(weights)
        cumulative = np.cumsum(weights)
        m = len(particles)
        r = np.random.uniform(0, 1.0 / m)
        indexes = []
        i = 0
        for j in range(m):
            u = r + j / m
            while u > cumulative[i]:
                i += 1
            indexes.append(i)
        return [particles[i].copy() for i in indexes]
