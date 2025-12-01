from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Tuple

import numpy as np


@dataclass
class OccupancyGrid:
    """Simple log-odds occupancy grid for 2D mapping."""

    resolution: float
    size_x: int
    size_y: int
    origin: Tuple[float, float] = (0.0, 0.0)
    log_odds_hit: float = math.log(4.0)
    log_odds_miss: float = math.log(1.0 / 4.0)
    clamp_min: float = -10.0
    clamp_max: float = 10.0
    grid: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.grid = np.zeros((self.size_y, self.size_x), dtype=np.float32)

    @property
    def width(self) -> float:
        return self.size_x * self.resolution

    @property
    def height(self) -> float:
        return self.size_y * self.resolution

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        wx = gx * self.resolution + self.origin[0]
        wy = gy * self.resolution + self.origin[1]
        return wx, wy

    def update_with_scan(
        self, pose: Tuple[float, float, float],
        ranges: Iterable[float],
        angles: Iterable[float],
        max_range: float,
    ) -> None:
        x, y, theta = pose
        for r, a in zip(ranges, angles):
            if not np.isfinite(r):
                continue
            clamped_r = min(r, max_range)
            end_x = x + clamped_r * math.cos(theta + a)
            end_y = y + clamped_r * math.sin(theta + a)
            self._bresenham_update((x, y), (end_x, end_y), r < max_range)

    def _bresenham_update(self, start: Tuple[float, float], end: Tuple[float, float], hit: bool) -> None:
        start_g = self.world_to_grid(*start)
        end_g = self.world_to_grid(*end)
        points = self._bresenham_line(start_g, end_g)
        for gx, gy in points[:-1]:
            if self._in_bounds(gx, gy):
                self.grid[gy, gx] = self._clamp(self.grid[gy, gx] + self.log_odds_miss)
        end_gx, end_gy = points[-1]
        if hit and self._in_bounds(end_gx, end_gy):
            self.grid[end_gy, end_gx] = self._clamp(self.grid[end_gy, end_gx] + self.log_odds_hit)

    def occupancy_probability(self) -> np.ndarray:
        odds = np.exp(self.grid)
        return odds / (1.0 + odds)

    def copy(self) -> "OccupancyGrid":
        new_grid = OccupancyGrid(
            resolution=self.resolution,
            size_x=self.size_x,
            size_y=self.size_y,
            origin=self.origin,
            log_odds_hit=self.log_odds_hit,
            log_odds_miss=self.log_odds_miss,
            clamp_min=self.clamp_min,
            clamp_max=self.clamp_max,
        )
        new_grid.grid = self.grid.copy()
        return new_grid

    def _clamp(self, val: float) -> float:
        return float(np.clip(val, self.clamp_min, self.clamp_max))

    def _in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.size_x and 0 <= gy < self.size_y

    @staticmethod
    def _bresenham_line(start: Tuple[int, int], end: Tuple[int, int]) -> Iterable[Tuple[int, int]]:
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        points = []
        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return points
