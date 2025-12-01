from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

from .occupancy_grid import OccupancyGrid


@dataclass
class Particle:
    """Particle holding pose hypothesis and map."""

    pose: Tuple[float, float, float]
    weight: float = 1.0
    grid: OccupancyGrid | None = None
    trajectory: List[Tuple[float, float, float]] = field(default_factory=list)

    def copy(self) -> "Particle":
        new_grid = None
        if self.grid is not None:
            new_grid = OccupancyGrid(
                resolution=self.grid.resolution,
                size_x=self.grid.size_x,
                size_y=self.grid.size_y,
                origin=self.grid.origin,
                log_odds_hit=self.grid.log_odds_hit,
                log_odds_miss=self.grid.log_odds_miss,
                clamp_min=self.grid.clamp_min,
                clamp_max=self.grid.clamp_max,
            )
            new_grid.grid = self.grid.grid.copy()
        return Particle(pose=self.pose, weight=self.weight, grid=new_grid, trajectory=list(self.trajectory))

    def normalize_weight(self, total_weight: float) -> None:
        if total_weight > 0.0:
            self.weight /= total_weight

    def append_trajectory(self) -> None:
        self.trajectory.append(self.pose)
