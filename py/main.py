"""
main.py - VacBot SLAM Navigator PC orchestrator.

Pipeline per tick:

    1. handle UI events  (visualizer.handle_events)
    2. read latest telemetry  (UDP)
    3. update occupancy grid  (SLAM)
    4. plan / replan path     (A*)
    5. compute differential PWM command
    6. send command           (UDP)
    7. render frame           (Pygame)

The loop is single-threaded; UDP receive runs in a background thread inside
the UDPLink object and the latest packet is read with .latest().
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
# IP/port pairing must match what was flashed into main.ino.
ROBOT_IP    = "192.168.1.50"   # ESP32 IP (printed on the serial monitor at boot)
ROBOT_PORT  = 5006             # ESP32 listen port
PC_LISTEN_PORT = 5005          # this PC listens here

# Must match firmware order
SENSOR_ANGLES: Tuple[float, ...] = (0.0, 1.5707963, -1.5707963)

# Control gains
KP_HEADING  = 1.5
PWM_PER_RAD = 80                # converts (Kp * err [rad]) into PWM units
FORWARD_MIN = 100
FORWARD_BASE = 120
FORWARD_MAX = 150
ROTATE_ONLY_ERR = 0.7           # rad; if |err| > this, stop forward, just rotate
GOAL_TOLERANCE_M = 0.10
LOOKAHEAD_CELLS = 4             # ~0.2 m ahead along the planned path

REPLAN_PERIOD_S = 0.5

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
    """
    Returns (left_pwm, right_pwm, goal_reached).
    """
    if goal_world is None or not path or len(path) < 2:
        return 0, 0, False

    x, y, theta = pose
    gx_world, gy_world = goal_world
    d_goal = math.hypot(gx_world - x, gy_world - y)
    if d_goal < GOAL_TOLERANCE_M:
        return 0, 0, True

    # Lookahead point a few cells ahead along the path
    idx = min(LOOKAHEAD_CELLS, len(path) - 1)
    tx, ty = grid.grid_to_world(*path[idx])

    desired_th = math.atan2(ty - y, tx - x)
    err = wrap_pi(desired_th - theta)

    turn = int(KP_HEADING * err * PWM_PER_RAD)

    if abs(err) > ROTATE_ONLY_ERR:
        fwd = 0
    else:
        # scale forward smoothly toward target the closer we are to heading
        scale = 1.0 - (abs(err) / ROTATE_ONLY_ERR)
        fwd = int(FORWARD_MIN + (FORWARD_BASE - FORWARD_MIN) * scale)
        fwd = clamp(fwd, FORWARD_MIN, FORWARD_MAX)

    left  = clamp(fwd - turn, -255, 255)
    right = clamp(fwd + turn, -255, 255)
    return left, right, False


def manual_to_pwm(manual: Tuple[int, int]) -> Tuple[int, int]:
    fwd, turn = manual
    # +turn = CCW (left): left wheel slower, right wheel faster.
    return clamp(fwd - turn, -255, 255), clamp(fwd + turn, -255, 255)


# ==================== main ====================
def main() -> None:
    os.makedirs(MAPS_DIR, exist_ok=True)

    link = UDPLink(ROBOT_IP, ROBOT_PORT, PC_LISTEN_PORT, stale_after=0.5)
    grid = OccupancyGrid()
    vis  = Visualizer(grid)

    goal_world: Optional[Tuple[float, float]] = None
    path: Optional[List[Tuple[int, int]]] = None
    last_plan_t = 0.0
    last_pose_for_replan: Optional[Tuple[float, float]] = None

    clock = pygame.time.Clock()
    running = True

    print(f"[main] PC listening on UDP port {PC_LISTEN_PORT}")
    print(f"[main] Will send commands to {ROBOT_IP}:{ROBOT_PORT}")

    while running:
        # ----- 1. UI events -----
        ui = vis.handle_events(pygame.event.get())
        if ui["quit"]:
            running = False
            break
        if ui["reset"]:
            grid.log_odds[:] = 0.0
            goal_world = None
            path = None
            print("[main] grid reset")
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
                    print(f"[main] map loaded <- {MAP_FILE}")
                else:
                    print(f"[main] map shape mismatch {loaded.shape}")
            except FileNotFoundError:
                print(f"[main] no saved map at {MAP_FILE}")
            except OSError as e:
                print(f"[main] load failed: {e}")
        if ui["click_world"] is not None:
            goal_world = ui["click_world"]
            path = None
            last_plan_t = 0.0
            print(f"[main] new goal: ({goal_world[0]:+.2f}, {goal_world[1]:+.2f}) m")
        manual = ui["manual"]

        # ----- 2. Latest telemetry -----
        msg = link.latest()

        status_lines: List[str] = []

        if msg is None:
            # link down or stale: stop motors, draw "no data" frame
            link.send_cmd(0, 0)
            status_lines.append("link: NO TELEMETRY")
            vis.set_status_lines(status_lines)
            vis.draw(None, None, SENSOR_ANGLES, path, goal_world)
            clock.tick(30)
            continue

        pose = (float(msg["x"]), float(msg["y"]), float(msg["theta"]))
        distances = [float(d) for d in msg["distances"]]

        # ----- 3. SLAM update -----
        try:
            grid.update_from_scan(pose, distances, SENSOR_ANGLES)
        except Exception as e:  # noqa: BLE001 - skip-frame policy per spec
            print(f"[main] SLAM frame skipped: {e}")

        # ----- 4. Plan / replan -----
        now = time.time()
        need_replan = (
            goal_world is not None
            and (path is None or now - last_plan_t > REPLAN_PERIOD_S)
        )
        if need_replan:
            last_plan_t = now
            inflated = grid.inflated()
            start_cell = grid.world_to_grid(pose[0], pose[1])
            goal_cell  = grid.world_to_grid(goal_world[0], goal_world[1])
            # The robot may straddle an inflated cell; force-clear the start
            # so the planner can leave the current pose.
            if grid.in_bounds(*start_cell):
                inflated_for_plan = inflated.copy()
                inflated_for_plan[start_cell] = 0
                path = astar(inflated_for_plan, start_cell, goal_cell)
                if path is None:
                    status_lines.append("plan: no path")
                else:
                    status_lines.append(f"plan: {len(path)} cells")

        # ----- 5. Control -----
        if manual != (0, 0):
            left, right = manual_to_pwm(manual)
            status_lines.append(f"mode: manual ({left:+},{right:+})")
        else:
            left, right, reached = compute_control(pose, path, grid, goal_world)
            if reached:
                print("[main] goal reached")
                goal_world = None
                path = None
                left, right = 0, 0
                status_lines.append("mode: idle (arrived)")
            elif goal_world is None:
                status_lines.append("mode: idle")
            else:
                status_lines.append(f"mode: auto ({left:+},{right:+})")

        # ----- 6. Send command -----
        link.send_cmd(left, right)

        # ----- 7. Render -----
        vis.set_status_lines(status_lines)
        vis.draw(pose, distances, SENSOR_ANGLES, path, goal_world)

        clock.tick(30)  # cap UI / control loop at 30 Hz

    link.send_cmd(0, 0)
    time.sleep(0.05)
    link.send_cmd(0, 0)  # extra stop in case of packet loss
    link.close()
    vis.close()
    print("[main] shut down cleanly")


if __name__ == "__main__":
    main()
