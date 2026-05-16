"""
main.py - VacBot SLAM Navigator PC orchestrator.

Pipeline per tick:

    1. handle UI events
    2. read latest telemetry  (UDP)
    3. update occupancy grid  (SLAM)
    4. mark robot footprint as visited  (coverage tracking)
    5. if no goal and not manual, cascade goal pick:
         a. frontier   (if auto-explore is on, and unknowns remain)
         b. sweep      (if sweep is on, and unvisited free cells remain)
    6. plan / replan path     (A*)
    7. compute differential PWM command
    8. send command           (UDP)
    9. render frame           (Pygame)

Modes (priority high -> low):
    Manual (WASD held)   keyboard drives directly, overrides everything.
    Click goal           manually set destination, A* drives there.
    Auto-explore ('T')   nearest reachable frontier (free cell adjacent to
                         unknown). Continues until the map is fully mapped.
    Sweep        ('Y')   nearest reachable unvisited free cell. Continues
                         until the floor is fully covered.
"""

from __future__ import annotations

import math
import os
import time
from typing import List, Optional, Tuple

import numpy as np
import pygame

from astar import astar
from network import UDPLink
from slam import OccupancyGrid
from visualizer import Visualizer

# ==================== USER CONFIGURATION ====================
ROBOT_IP    = "192.168.1.29"
ROBOT_PORT  = 5006
PC_LISTEN_PORT = 5005

SENSOR_ANGLES: Tuple[float, ...] = (0.0, 1.5707963, -1.5707963)

# ============ Control gains — TUNED FOR CHEAP HARDWARE ============
# These values are deliberately gentle.  Single-channel encoders + HC-SR04 +
# MPU6050 give you a heading estimate that jitters a few degrees at rest,
# so aggressive gains turn that noise into visible motor twitch.
KP_HEADING        = 0.8     # was 1.5  ↓ — high gain amplified heading noise
PWM_PER_RAD       = 70
FORWARD_MIN       = 110     # stay above DEADBAND (28) with margin
FORWARD_BASE      = 130
FORWARD_MAX       = 160
ROTATE_ONLY_ERR   = 0.5     # was 0.7  ↓ — commit to in-place turn earlier
GOAL_TOLERANCE_M  = 0.20    # was 0.10 ↑ — cheap odom can't do 10 cm reliably
LOOKAHEAD_CELLS   = 8       # was 4    ↑ — averages out heading jitter
HEADING_DEADBAND  = 0.08    # NEW (~4.5°) — kill motor wiggle from gyro noise

REPLAN_PERIOD_S   = 1.5     # was 0.5  ↑ — fewer replans = less path jitter

# ============ Auto-explore (frontier-based mapping) ============
# FRONTIER_MIN_DIST_CELLS = 4 was way too close — robot would "reach" a goal
# in two ticks and immediately pick another 20 cm away.  12 cells at 5 cm/cell
# is 60 cm, far enough that the robot actually has to drive somewhere new.
FRONTIER_MIN_DIST_CELLS = 12     # was 4   ↑↑
FRONTIER_CANDIDATE_CAP  = 12     # was 8   ↑
AUTO_RESCAN_S           = 3.0    # was 1.0 ↑ — commit longer between picks

# Stuck detection: if we haven't made progress toward our goal in N seconds,
# drop it so the cascade can try something else.
STUCK_TIMEOUT_S       = 10.0
STUCK_MIN_PROGRESS_M  = 0.15

# ============ Sweep / coverage ============
SWEEP_BRUSH_CELLS = 2

MAPS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps")
MAP_FILE = os.path.join(MAPS_DIR, "map.npy")


# ==================== helpers ====================
def wrap_pi(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def compute_control(pose: Tuple[float, float, float],
                    path: Optional[List[Tuple[int, int]]],
                    grid: OccupancyGrid,
                    goal_world: Optional[Tuple[float, float]]
                    ) -> Tuple[int, int, bool]:
    """Returns (left_pwm, right_pwm, goal_reached)."""
    if goal_world is None or not path or len(path) < 2:
        return 0, 0, False

    x, y, theta = pose
    gx_world, gy_world = goal_world
    if math.hypot(gx_world - x, gy_world - y) < GOAL_TOLERANCE_M:
        return 0, 0, True

    idx = min(LOOKAHEAD_CELLS, len(path) - 1)
    tx, ty = grid.grid_to_world(*path[idx])

    desired_th = math.atan2(ty - y, tx - x)
    err = wrap_pi(desired_th - theta)

    # Heading deadband — don't fight tiny errors, they're mostly sensor noise.
    if abs(err) < HEADING_DEADBAND:
        turn = 0
    else:
        turn = int(KP_HEADING * err * PWM_PER_RAD)

    if abs(err) > ROTATE_ONLY_ERR:
        fwd = 0
    else:
        scale = 1.0 - (abs(err) / ROTATE_ONLY_ERR)
        fwd = int(FORWARD_MIN + (FORWARD_BASE - FORWARD_MIN) * scale)
        fwd = clamp(fwd, FORWARD_MIN, FORWARD_MAX)

    left  = clamp(fwd - turn, -255, 255)
    right = clamp(fwd + turn, -255, 255)
    return left, right, False


def manual_to_pwm(manual: Tuple[int, int]) -> Tuple[int, int]:
    fwd, turn = manual
    return clamp(fwd - turn, -255, 255), clamp(fwd + turn, -255, 255)


# ==================== coverage tracking ====================
def mark_visited_disk(visited: np.ndarray,
                      grid: OccupancyGrid,
                      pose: Tuple[float, float, float],
                      radius: int) -> None:
    sgx, sgy = grid.world_to_grid(pose[0], pose[1])
    if not grid.in_bounds(sgx, sgy):
        return
    r = max(0, int(radius))
    gx0, gx1 = max(0, sgx - r), min(grid.size, sgx + r + 1)
    gy0, gy1 = max(0, sgy - r), min(grid.size, sgy + r + 1)
    xs = np.arange(gx0, gx1).reshape(-1, 1)
    ys = np.arange(gy0, gy1).reshape( 1, -1)
    disk = (xs - sgx) ** 2 + (ys - sgy) ** 2 <= r * r
    visited[gx0:gx1, gy0:gy1] |= disk


# ==================== auto goal pickers ====================
def _nearest_reachable(grid: OccupancyGrid,
                       pose: Tuple[float, float, float],
                       candidate_mask: np.ndarray
                       ) -> Optional[Tuple[int, int]]:
    if not candidate_mask.any():
        return None
    sgx, sgy = grid.world_to_grid(pose[0], pose[1])
    fxs, fys = np.where(candidate_mask)
    dists = (fxs - sgx) ** 2 + (fys - sgy) ** 2

    far_enough = dists >= (FRONTIER_MIN_DIST_CELLS ** 2)
    if far_enough.any():
        fxs, fys, dists = fxs[far_enough], fys[far_enough], dists[far_enough]
    if len(dists) == 0:
        return None

    order = np.argsort(dists)
    inflated = grid.inflated()
    if grid.in_bounds(sgx, sgy):
        inflated = inflated.copy()
        inflated[sgx, sgy] = 0

    tried = 0
    for k in order:
        if tried >= FRONTIER_CANDIDATE_CAP:
            break
        cand = (int(fxs[k]), int(fys[k]))
        if inflated[cand] == 1:
            continue
        path = astar(inflated, (sgx, sgy), cand)
        if path is not None and len(path) > 1:
            return cand
        tried += 1
    return None


def find_frontier_goal(grid: OccupancyGrid,
                       pose: Tuple[float, float, float]
                       ) -> Optional[Tuple[int, int]]:
    occ = grid.occupancy()
    unknown_mask = (occ == -1)
    nbr_unknown = np.zeros_like(unknown_mask)
    nbr_unknown[1:,  :] |= unknown_mask[:-1, :]
    nbr_unknown[:-1, :] |= unknown_mask[1:,  :]
    nbr_unknown[:,  1:] |= unknown_mask[:, :-1]
    nbr_unknown[:, :-1] |= unknown_mask[:, 1: ]
    frontier_mask = (occ == 0) & nbr_unknown
    return _nearest_reachable(grid, pose, frontier_mask)


def find_sweep_goal(grid: OccupancyGrid,
                    visited: np.ndarray,
                    pose: Tuple[float, float, float]
                    ) -> Optional[Tuple[int, int]]:
    occ = grid.occupancy()
    unvisited_free = (occ == 0) & ~visited
    return _nearest_reachable(grid, pose, unvisited_free)


# ==================== main ====================
def main() -> None:
    os.makedirs(MAPS_DIR, exist_ok=True)

    link = UDPLink(ROBOT_IP, ROBOT_PORT, PC_LISTEN_PORT, stale_after=0.5)
    grid = OccupancyGrid()
    visited = np.zeros((grid.size, grid.size), dtype=bool)
    vis  = Visualizer(grid)

    goal_world: Optional[Tuple[float, float]] = None
    goal_kind: Optional[str] = None
    path: Optional[List[Tuple[int, int]]] = None
    last_plan_t = 0.0

    auto_explore_on = False
    sweep_on = False
    last_auto_scan_t = 0.0

    # Stuck detection state
    goal_set_t      = 0.0
    goal_best_dist  = float("inf")
    goal_best_t     = 0.0

    clock = pygame.time.Clock()
    running = True

    print(f"[main] PC listening on UDP port {PC_LISTEN_PORT}")
    print(f"[main] Will send commands to {ROBOT_IP}:{ROBOT_PORT}")
    print(f"[main] T = auto-explore (mapping)   Y = sweep (coverage)")

    while running:
        # ----- 1. UI events -----
        ui = vis.handle_events(pygame.event.get())
        if ui["quit"]:
            running = False
            break
        if ui["reset"]:
            grid.log_odds[:] = 0.0
            visited[:] = False
            goal_world = None
            goal_kind = None
            path = None
            print("[main] grid & coverage reset")
        if ui["save_map"]:
            try:
                np.save(MAP_FILE, grid.log_odds)
                print(f"[main] map saved -> {MAP_FILE}")
            except OSError as e:
                print(f"[main] save failed: {e}")
        if ui["load_map"]:
            try:
                loaded = np.load(MAP_FILE)
                if loaded.shape == grid.log_odds.shape:
                    grid.log_odds[:] = loaded.astype(np.float32)
                    visited[:] = False
                    print(f"[main] map loaded <- {MAP_FILE}")
                else:
                    print(f"[main] map shape mismatch {loaded.shape}")
            except FileNotFoundError:
                print(f"[main] no saved map at {MAP_FILE}")
            except OSError as e:
                print(f"[main] load failed: {e}")
        if ui["click_world"] is not None:
            goal_world = ui["click_world"]
            goal_kind = "user"
            path = None
            last_plan_t = 0.0
            goal_set_t = time.time()
            goal_best_dist = float("inf")
            goal_best_t = goal_set_t
            print(f"[main] new goal: ({goal_world[0]:+.2f}, {goal_world[1]:+.2f}) m")
        if ui["auto_toggle"]:
            auto_explore_on = not auto_explore_on
            if goal_kind == "frontier":
                goal_world = None; goal_kind = None; path = None
            print(f"[main] auto-explore: {'ON' if auto_explore_on else 'off'}")
        if ui["sweep_toggle"]:
            sweep_on = not sweep_on
            if goal_kind == "sweep":
                goal_world = None; goal_kind = None; path = None
            print(f"[main] sweep:        {'ON' if sweep_on else 'off'}")
        vis.set_auto_explore(auto_explore_on)
        vis.set_sweep(sweep_on)
        manual = ui["manual"]

        # ----- 2. Latest telemetry -----
        msg = link.latest()

        status_lines: List[str] = []

        if msg is None:
            link.send_cmd(0, 0)
            status_lines.append("link: NO TELEMETRY")
            vis.set_status_lines(status_lines)
            vis.draw(None, None, SENSOR_ANGLES, path, goal_world,
                     goal_kind=goal_kind, visited=visited)
            clock.tick(30)
            continue

        pose = (float(msg["x"]), float(msg["y"]), float(msg["theta"]))
        distances = [float(d) for d in msg["distances"]]

        # ----- 3. SLAM update -----
        try:
            grid.update_from_scan(pose, distances, SENSOR_ANGLES)
        except Exception as e:  # noqa: BLE001
            print(f"[main] SLAM frame skipped: {e}")

        # ----- 4. Coverage -----
        mark_visited_disk(visited, grid, pose, SWEEP_BRUSH_CELLS)

        now = time.time()

        # ----- Stuck check: if we have a goal but aren't making progress, drop it.
        if goal_world is not None:
            d_to_goal = math.hypot(goal_world[0] - pose[0], goal_world[1] - pose[1])
            if d_to_goal + STUCK_MIN_PROGRESS_M < goal_best_dist:
                goal_best_dist = d_to_goal
                goal_best_t = now
            elif goal_kind in ("frontier", "sweep") \
                 and (now - goal_best_t) > STUCK_TIMEOUT_S:
                print(f"[main] stuck on {goal_kind} goal ({d_to_goal:.2f} m) — dropping")
                goal_world = None
                goal_kind = None
                path = None
                status_lines.append("stuck — picking new goal")

        # ----- 5. Cascade auto goal pick -----
        if (goal_world is None
                and manual == (0, 0)
                and (auto_explore_on or sweep_on)
                and (now - last_auto_scan_t) > AUTO_RESCAN_S):
            last_auto_scan_t = now

            cell: Optional[Tuple[int, int]] = None
            kind: Optional[str] = None

            if auto_explore_on:
                cell = find_frontier_goal(grid, pose)
                if cell is not None:
                    kind = "frontier"

            if cell is None and sweep_on:
                cell = find_sweep_goal(grid, visited, pose)
                if cell is not None:
                    kind = "sweep"

            if cell is not None:
                wx, wy = grid.grid_to_world(*cell)
                goal_world = (wx, wy)
                goal_kind  = kind
                path = None
                last_plan_t = 0.0
                goal_set_t = now
                goal_best_dist = math.hypot(wx - pose[0], wy - pose[1])
                goal_best_t = now
                print(f"[main] {kind} -> cell {cell} world "
                      f"({wx:+.2f},{wy:+.2f})  dist={goal_best_dist:.2f}m")
                status_lines.append(f"{kind}: -> ({wx:+.2f},{wy:+.2f})")
            else:
                if auto_explore_on and sweep_on:
                    status_lines.append("auto: map + sweep complete")
                elif auto_explore_on:
                    status_lines.append("auto: map complete")
                else:
                    status_lines.append("sweep: complete")

        # ----- 6. Plan / replan -----
        need_replan = (
            goal_world is not None
            and (path is None or now - last_plan_t > REPLAN_PERIOD_S)
        )
        if need_replan:
            last_plan_t = now
            inflated = grid.inflated()
            start_cell = grid.world_to_grid(pose[0], pose[1])
            goal_cell  = grid.world_to_grid(goal_world[0], goal_world[1])
            if grid.in_bounds(*start_cell):
                inflated_for_plan = inflated.copy()
                inflated_for_plan[start_cell] = 0
                path = astar(inflated_for_plan, start_cell, goal_cell)
                if path is None:
                    status_lines.append("plan: no path")
                    if goal_kind in ("frontier", "sweep"):
                        goal_world = None
                        goal_kind = None
                else:
                    status_lines.append(f"plan: {len(path)} cells")

        # ----- 7. Control -----
        if manual != (0, 0):
            left, right = manual_to_pwm(manual)
            status_lines.append(f"mode: manual ({left:+},{right:+})")
        else:
            left, right, reached = compute_control(pose, path, grid, goal_world)
            if reached:
                print(f"[main] goal reached ({goal_kind})")
                goal_world = None
                goal_kind = None
                path = None
                left, right = 0, 0
                status_lines.append("mode: idle (arrived)")
            elif goal_world is None:
                if auto_explore_on or sweep_on:
                    status_lines.append("mode: auto (scanning)")
                else:
                    status_lines.append("mode: idle")
            else:
                status_lines.append(f"mode: {goal_kind} ({left:+},{right:+})")

        # ----- 8. Send command -----
        link.send_cmd(left, right)

        # ----- 9. Render -----
        vis.set_status_lines(status_lines)
        vis.draw(pose, distances, SENSOR_ANGLES, path, goal_world,
                 goal_kind=goal_kind, visited=visited)

        clock.tick(30)

    link.send_cmd(0, 0)
    time.sleep(0.05)
    link.send_cmd(0, 0)
    link.close()
    vis.close()
    print("[main] shut down cleanly")


if __name__ == "__main__":
    main()