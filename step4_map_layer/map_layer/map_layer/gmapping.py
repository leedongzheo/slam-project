from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Tuple

import numpy as np

from .motion_model import DifferentialDriveMotionModel
from .occupancy_grid import OccupancyGrid
from .particle import Particle
from .resampler import LowVarianceResampler
from .sensor_model import LikelihoodFieldSensorModel


@dataclass
class GMapping:
    """Lightweight particle-filter SLAM inspired by GMapping."""

    num_particles: int = 30
    map_size: Tuple[int, int] = (400, 400)
    map_resolution: float = 0.05
    map_origin: Tuple[float, float] = (-10.0, -10.0)
    resample_threshold: float = 0.5
    motion_model: DifferentialDriveMotionModel = field(default_factory=DifferentialDriveMotionModel)
    sensor_model: LikelihoodFieldSensorModel = field(default_factory=LikelihoodFieldSensorModel)
    resampler: LowVarianceResampler = field(default_factory=LowVarianceResampler)
    max_range: float = 3.5

    particles: List[Particle] = field(init=False)
    map_angles: np.ndarray = field(init=False)
    last_odom: Tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        grid_template = OccupancyGrid(
            resolution=self.map_resolution,
            size_x=self.map_size[0],
            size_y=self.map_size[1],
            origin=self.map_origin,
        )
        self.sensor_model.max_range = self.max_range
        self.particles = [Particle(pose=(0.0, 0.0, 0.0), weight=1.0, grid=grid_template.copy()) for _ in range(self.num_particles)]
        self.map_angles = None

    def initialize_angles(self, scan_angle_min: float, scan_angle_increment: float, count: int) -> None:
        self.map_angles = np.array([scan_angle_min + i * scan_angle_increment for i in range(count)], dtype=np.float32)

    def predict(self, odom: Tuple[float, float, float]) -> None:
        if self.last_odom is None:
            self.last_odom = odom
            return
        delta = self._odom_delta(self.last_odom, odom)
        self.last_odom = odom
        for p in self.particles:
            p.pose = self.motion_model.sample(p.pose, delta)

    def update(self, ranges: Iterable[float]) -> None:
        if self.map_angles is None:
            raise RuntimeError("Scan angles not initialized")
        ranges = np.array(ranges, dtype=np.float32)
        total_weight = 0.0
        for p in self.particles:
            weight = self.sensor_model.compute_weight(p.grid, p.pose, ranges, self.map_angles)
            p.weight = weight
            total_weight += weight
            p.grid.update_with_scan(p.pose, ranges, self.map_angles, self.max_range)
            p.append_trajectory()
        for p in self.particles:
            p.normalize_weight(total_weight)
        if self._neff() < self.resample_threshold * len(self.particles):
            self.particles = self.resampler.resample(self.particles)

    def best_particle(self) -> Particle:
        return max(self.particles, key=lambda p: p.weight)

    def _neff(self) -> float:
        weights = np.array([p.weight for p in self.particles])
        return 1.0 / np.sum(np.square(weights) + 1e-9)

    def _odom_delta(self, prev: Tuple[float, float, float], curr: Tuple[float, float, float]) -> Tuple[float, float, float]:
        x0, y0, t0 = prev
        x1, y1, t1 = curr
        dx = x1 - x0
        dy = y1 - y0
        trans = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        rot1 = self._normalize_angle(heading - t0)
        rot2 = self._normalize_angle(t1 - t0 - rot1)
        return rot1, trans, rot2

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi
