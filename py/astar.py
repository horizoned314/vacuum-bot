"""
astar.py - A* path planner on an 8-connected occupancy grid.

Inputs:
    grid  : 2D int array (numpy or list-of-lists)
            Cells with value 1 are obstacles. -1 (unknown) and 0 (free) are
            both treated as traversable; the caller may pre-mark unknown
            cells as blocked if a stricter policy is desired.
    start : (gx, gy)
    goal  : (gx, gy)

Returns:
    List of (gx, gy) cells from start to goal inclusive, or None if no path.

Notes:
    - Euclidean heuristic, so paths are admissible / shortest in Euclidean cost.
    - Uses heapq priority queue (open set).
    - Closed set + lazy duplicate skip handles re-insertions correctly.
"""

from __future__ import annotations

import heapq
import math
from typing import List, Optional, Tuple

import numpy as np


_DIAG = math.sqrt(2.0)

# 8-neighbourhood with movement cost
_NEIGHBOURS: Tuple[Tuple[int, int, float], ...] = (
    (-1, -1, _DIAG), (-1, 0, 1.0), (-1, 1, _DIAG),
    ( 0, -1, 1.0),                  ( 0, 1, 1.0),
    ( 1, -1, _DIAG), ( 1, 0, 1.0), ( 1, 1, _DIAG),
)


def astar(grid: np.ndarray,
          start: Tuple[int, int],
          goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    H, W = grid.shape
    sx, sy = start
    gx, gy = goal

    if not (0 <= sx < H and 0 <= sy < W):
        return None
    if not (0 <= gx < H and 0 <= gy < W):
        return None
    if grid[gx, gy] == 1:
        return None
    if start == goal:
        return [start]

    def heuristic(p: Tuple[int, int]) -> float:
        return math.hypot(p[0] - gx, p[1] - gy)

    open_heap: List[Tuple[float, float, Tuple[int, int]]] = []
    heapq.heappush(open_heap, (heuristic(start), 0.0, start))
    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        f, g_curr, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            # reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        closed.add(current)

        cx, cy = current
        for dx, dy, cost in _NEIGHBOURS:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < H and 0 <= ny < W):
                continue
            if grid[nx, ny] == 1:
                continue
            # Prevent diagonal cutting through corners between two obstacles
            if dx != 0 and dy != 0:
                if grid[cx + dx, cy] == 1 and grid[cx, cy + dy] == 1:
                    continue
            n = (nx, ny)
            tentative = g_curr + cost
            if tentative < g_score.get(n, math.inf):
                came_from[n] = current
                g_score[n] = tentative
                heapq.heappush(open_heap, (tentative + heuristic(n), tentative, n))
    return None
