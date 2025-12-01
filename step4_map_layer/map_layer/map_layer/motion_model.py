from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class DifferentialDriveMotionModel:
    """Odometry-based motion model for differential drive robots."""

    alpha1: float = 0.1
    alpha2: float = 0.1
    alpha3: float = 0.2
    alpha4: float = 0.2

    def sample(self, prev_pose: Tuple[float, float, float], odom_delta: Tuple[float, float, float]) -> Tuple[float, float, float]:
        x, y, theta = prev_pose
        rot1, trans, rot2 = odom_delta

        rot1_hat = rot1 + np.random.normal(0, self.alpha1 * abs(rot1) + self.alpha2 * trans)
        trans_hat = trans + np.random.normal(0, self.alpha3 * trans + self.alpha4 * (abs(rot1) + abs(rot2)))
        rot2_hat = rot2 + np.random.normal(0, self.alpha1 * abs(rot2) + self.alpha2 * trans)

        new_x = x + trans_hat * math.cos(theta + rot1_hat)
        new_y = y + trans_hat * math.sin(theta + rot1_hat)
        new_theta = self._normalize_angle(theta + rot1_hat + rot2_hat)
        return new_x, new_y, new_theta

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi
