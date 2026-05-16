"""
slam.py - Occupancy-grid SLAM with Bresenham ray casting.

Conventions (must match firmware):
    World frame: x forward, y left, theta CCW positive.
    Grid:        100 x 100 cells, 0.05 m per cell, origin at cell (50, 50).
    Indexing:    grid[gx, gy], where
                 gx = origin + round(x / resolution)
                 gy = origin + round(y / resolution)

Cell states (occupancy()):
    -1 = unknown
     0 = free
     1 = occupied

The internal representation uses log-odds for numerical stability; the
ternary occupancy view is derived on demand for visualization and planning.
"""

from __future__ import annotations

import math
from typing import Iterable, List, Tuple

import numpy as np


GRID_SIZE = 500
GRID_RES = 0.1      # meters / cell
MAX_RANGE = 2.5      # meters


class OccupancyGrid:
    # Log-odds increments per ray hit
    L_FREE = -0.40
    L_OCC  =  0.85
    L_MIN  = -4.0
    L_MAX  =  4.0

    # Decision thresholds for the ternary view
    OCC_THRESHOLD  =  0.6
    FREE_THRESHOLD = -0.4

    INFLATE_CELLS = 1   # spec: 1-cell inflation radius

    def __init__(self, size: int = GRID_SIZE, resolution: float = GRID_RES) -> None:
        self.size = size
        self.resolution = resolution
        self.origin = size // 2
        self.log_odds = np.zeros((size, size), dtype=np.float32)

    # ----------------- coordinate transforms -----------------
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(round(self.origin + x / self.resolution))
        gy = int(round(self.origin + y / self.resolution))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = (gx - self.origin) * self.resolution
        y = (gy - self.origin) * self.resolution
        return x, y

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.size and 0 <= gy < self.size

    # ----------------- views -----------------
    def occupancy(self) -> np.ndarray:
        """Return -1/0/1 ternary grid for visualization and planning."""
        g = np.full((self.size, self.size), -1, dtype=np.int8)
        g[self.log_odds >  self.OCC_THRESHOLD]  =  1
        g[self.log_odds <  self.FREE_THRESHOLD] =  0
        return g

    def inflated(self) -> np.ndarray:
        """Occupancy grid with INFLATE_CELLS-radius dilation of occupied cells."""
        occ = self.occupancy()
        if self.INFLATE_CELLS <= 0:
            return occ
        mask = (occ == 1)
        dilated = mask.copy()
        for _ in range(self.INFLATE_CELLS):
            nxt = dilated.copy()
            nxt[1:,  :] |= dilated[:-1, :]
            nxt[:-1, :] |= dilated[1:,  :]
            nxt[:, 1: ] |= dilated[:, :-1]
            nxt[:, :-1] |= dilated[:, 1: ]
            dilated = nxt
        out = occ.copy()
        out[dilated & (out != 1)] = 1
        return out

    # ----------------- Bresenham ray casting -----------------
    @staticmethod
    def _bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        cells: List[Tuple[int, int]] = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy
        x, y = x0, y0
        # Safety cap; at 0.05 m / cell and 2.5 m max range the chain is < 60 cells.
        for _ in range(4096):
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return cells

    def _update_ray(self, x0: float, y0: float, x1: float, y1: float,
                    hit: bool) -> None:
        gx0, gy0 = self.world_to_grid(x0, y0)
        gx1, gy1 = self.world_to_grid(x1, y1)
        cells = self._bresenham(gx0, gy0, gx1, gy1)
        if not cells:
            return
        last = len(cells) - 1
        for i, (gx, gy) in enumerate(cells):
            if not self.in_bounds(gx, gy):
                continue
            if i == last and hit:
                self.log_odds[gx, gy] = min(self.L_MAX,
                                            self.log_odds[gx, gy] + self.L_OCC)
            else:
                self.log_odds[gx, gy] = max(self.L_MIN,
                                            self.log_odds[gx, gy] + self.L_FREE)

    # ----------------- SLAM update -----------------
    def update_from_scan(self,
                         pose: Tuple[float, float, float],
                         distances: Iterable[float],
                         sensor_angles: Iterable[float]) -> None:
        """
        pose           : (x, y, theta) in world frame
        distances      : per-sensor range readings (m); negative = invalid
        sensor_angles  : per-sensor body-frame mounting angles (rad), same order
        """
        x, y, theta = pose
        for ang, d in zip(sensor_angles, distances):
            if d is None:
                continue
            # invalid reading - skip entirely so it doesn't poison the map
            if d < 0 or not math.isfinite(d):
                continue
            valid_hit = (d <= MAX_RANGE)
            ray_d = d if valid_hit else MAX_RANGE
            ray_th = theta + ang
            x1 = x + ray_d * math.cos(ray_th)
            y1 = y + ray_d * math.sin(ray_th)
            self._update_ray(x, y, x1, y1, hit=valid_hit)
