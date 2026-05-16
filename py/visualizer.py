"""
visualizer.py - Pygame real-time visualization for the VacBot SLAM Navigator.

Display conventions (chosen for natural reading of the map):
    +x world (robot forward)  ->  screen UP
    +y world (robot left)     ->  screen LEFT

So when the robot is at the origin facing +x, the heading arrow points up
and its left-side ultrasonic ray points to the left of the screen.

Controls:
    Mouse left-click on map : set navigation goal
    W / A / S / D           : manual drive override
    T                       : toggle autonomous frontier exploration
    R                       : reset map
    O                       : save map
    L                       : load map
    ESC                     : quit
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
import pygame


CELL_PX = 6        # screen pixels per grid cell
PANEL_W = 240      # right-hand info panel width
BG_COLOR        = ( 30,  30,  30)
COLOR_UNKNOWN   = ( 60,  60,  60)
COLOR_FREE      = (220, 220, 220)
COLOR_OCCUPIED  = ( 20,  20,  20)
COLOR_ROBOT     = ( 60, 200, 255)
COLOR_HEADING   = (255, 230,   0)
COLOR_RAY_HIT   = (255,  80,  80)
COLOR_RAY_MISS  = (110, 110, 110)
COLOR_PATH      = ( 40, 240, 120)
COLOR_GOAL      = (255,  60, 200)
COLOR_FRONTIER  = ( 90, 180, 255)
COLOR_TEXT      = (235, 235, 235)
COLOR_WARN      = (255, 200,   0)
COLOR_AUTO_ON   = ( 80, 250, 130)
COLOR_AUTO_OFF  = (180, 180, 180)


class Visualizer:
    def __init__(self, grid) -> None:
        pygame.init()
        self.grid = grid
        self.map_px = grid.size * CELL_PX
        self.W = self.map_px + PANEL_W
        self.H = self.map_px
        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption("VacBot SLAM Navigator")
        self.font_s = pygame.font.SysFont("Consolas", 14)
        self.font_m = pygame.font.SysFont("Consolas", 17)
        self._panel_lines_extra: List[str] = []
        self._auto_explore_on = False

    # ===================== coordinate transforms =====================
    # Cell (gx, gy) -> screen pixel (center of cell).
    # We flip both axes so that +x is up and +y is left.
    def _cell_to_px(self, gx: int, gy: int) -> Tuple[int, int]:
        sx = (self.grid.size - 1 - gy) * CELL_PX + CELL_PX // 2
        sy = (self.grid.size - 1 - gx) * CELL_PX + CELL_PX // 2
        return sx, sy

    def _world_to_px(self, x: float, y: float) -> Tuple[int, int]:
        gx, gy = self.grid.world_to_grid(x, y)
        return self._cell_to_px(gx, gy)

    def _px_to_world(self, sx: int, sy: int) -> Tuple[float, float]:
        # Inverse of _cell_to_px (treating cell-center pixels).
        gy = self.grid.size - 1 - (sx - CELL_PX // 2) / CELL_PX
        gx = self.grid.size - 1 - (sy - CELL_PX // 2) / CELL_PX
        x = (gx - self.grid.origin) * self.grid.resolution
        y = (gy - self.grid.origin) * self.grid.resolution
        return float(x), float(y)

    # ===================== event handling =====================
    def handle_events(self, events) -> dict:
        state = {
            "quit": False,
            "reset": False,
            "save_map": False,
            "load_map": False,
            "click_world": None,
            "manual": (0, 0),
            "auto_toggle": False,
        }
        for ev in events:
            if ev.type == pygame.QUIT:
                state["quit"] = True
            elif ev.type == pygame.KEYDOWN:
                if   ev.key == pygame.K_ESCAPE: state["quit"] = True
                elif ev.key == pygame.K_r:      state["reset"] = True
                elif ev.key == pygame.K_o:      state["save_map"] = True   # 'o' for sa(o)ve to avoid conflict with WASD
                elif ev.key == pygame.K_l:      state["load_map"] = True
                elif ev.key == pygame.K_t:      state["auto_toggle"] = True
                elif ev.key == pygame.K_F5:     state["save_map"] = True   # backup binding
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                if ev.pos[0] < self.map_px:
                    state["click_world"] = self._px_to_world(*ev.pos)

        # Held-key manual drive (WASD).
        # +turn = CCW (left) so A pressed -> +turn, D pressed -> -turn.
        keys = pygame.key.get_pressed()
        fwd, turn = 0, 0
        if keys[pygame.K_w]: fwd  += 140
        if keys[pygame.K_s]: fwd  -= 140
        if keys[pygame.K_a]: turn +=  80
        if keys[pygame.K_d]: turn -=  80
        if fwd != 0 or turn != 0:
            state["manual"] = (fwd, turn)
        return state

    def set_status_lines(self, lines: List[str]) -> None:
        self._panel_lines_extra = list(lines)

    def set_auto_explore(self, on: bool) -> None:
        self._auto_explore_on = bool(on)

    # ===================== drawing =====================
    def _draw_grid(self) -> None:
        occ = self.grid.occupancy()
        size = self.grid.size
        # base[gx, gy] -> RGB
        base = np.full((size, size, 3), COLOR_UNKNOWN, dtype=np.uint8)
        base[occ == 0] = COLOR_FREE
        base[occ == 1] = COLOR_OCCUPIED
        # Reorient so that screen pixel (sx, sy) maps to cell
        # (gx, gy) = (size-1-sy, size-1-sx). See _cell_to_px.
        screen_img = base[::-1, ::-1].transpose(1, 0, 2)
        surf = pygame.surfarray.make_surface(screen_img)
        surf = pygame.transform.scale(surf, (self.map_px, self.map_px))
        self.screen.blit(surf, (0, 0))

    def _draw_robot(self, pose: Tuple[float, float, float]) -> None:
        x, y, th = pose
        sx, sy = self._world_to_px(x, y)
        pygame.draw.circle(self.screen, COLOR_ROBOT, (sx, sy), 8)
        hx = x + 0.18 * math.cos(th)
        hy = y + 0.18 * math.sin(th)
        hsx, hsy = self._world_to_px(hx, hy)
        pygame.draw.line(self.screen, COLOR_HEADING, (sx, sy), (hsx, hsy), 3)

    def _draw_rays(self,
                   pose: Tuple[float, float, float],
                   distances: Sequence[float],
                   sensor_angles: Sequence[float]) -> None:
        x, y, th = pose
        sx, sy = self._world_to_px(x, y)
        for ang, d in zip(sensor_angles, distances):
            if d is None or d < 0:
                # missing reading - draw faint full-range ray to indicate sensor liveness
                d_use = 2.5
                color = (50, 50, 50)
            elif d > 2.5:
                d_use = 2.5
                color = COLOR_RAY_MISS
            else:
                d_use = d
                color = COLOR_RAY_HIT
            ex = x + d_use * math.cos(th + ang)
            ey = y + d_use * math.sin(th + ang)
            esx, esy = self._world_to_px(ex, ey)
            pygame.draw.line(self.screen, color, (sx, sy), (esx, esy), 1)

    def _draw_path(self, path: Optional[List[Tuple[int, int]]]) -> None:
        if not path or len(path) < 2:
            return
        pts = [self._cell_to_px(gx, gy) for gx, gy in path]
        pygame.draw.lines(self.screen, COLOR_PATH, False, pts, 2)

    def _draw_goal(self,
                   goal_world: Optional[Tuple[float, float]],
                   auto_pick: bool = False) -> None:
        if goal_world is None:
            return
        sx, sy = self._world_to_px(*goal_world)
        color = COLOR_FRONTIER if auto_pick else COLOR_GOAL
        pygame.draw.circle(self.screen, color, (sx, sy), 7, 2)
        pygame.draw.line(self.screen, color, (sx - 9, sy), (sx + 9, sy), 1)
        pygame.draw.line(self.screen, color, (sx, sy - 9), (sx, sy + 9), 1)

    def _draw_panel(self,
                    pose: Optional[Tuple[float, float, float]],
                    distances: Optional[Sequence[float]]) -> None:
        x0 = self.map_px + 12
        y  = 14
        pygame.draw.rect(self.screen, (45, 45, 45),
                         pygame.Rect(self.map_px, 0, PANEL_W, self.H))
        title = self.font_m.render("VacBot SLAM", True, COLOR_TEXT)
        self.screen.blit(title, (x0, y)); y += 28

        # Auto-explore badge
        auto_color = COLOR_AUTO_ON if self._auto_explore_on else COLOR_AUTO_OFF
        auto_label = "AUTO  ON " if self._auto_explore_on else "AUTO  off"
        self.screen.blit(self.font_m.render(auto_label, True, auto_color),
                         (x0, y)); y += 26

        if pose is None:
            self.screen.blit(self.font_s.render("No telemetry", True, COLOR_WARN),
                             (x0, y)); y += 22
        else:
            self.screen.blit(self.font_s.render(
                f"x:     {pose[0]:+6.2f} m", True, COLOR_TEXT), (x0, y)); y += 18
            self.screen.blit(self.font_s.render(
                f"y:     {pose[1]:+6.2f} m", True, COLOR_TEXT), (x0, y)); y += 18
            self.screen.blit(self.font_s.render(
                f"theta: {math.degrees(pose[2]):+6.1f} deg", True, COLOR_TEXT),
                (x0, y)); y += 22

        if distances is not None:
            for name, d in zip(("front", "left ", "right"), distances):
                txt = f"{name}: {d:5.2f} m" if (d is not None and d >= 0) \
                      else f"{name}:   --"
                self.screen.blit(self.font_s.render(txt, True, COLOR_TEXT),
                                 (x0, y)); y += 18
            y += 6

        for ln in self._panel_lines_extra:
            self.screen.blit(self.font_s.render(ln, True, COLOR_WARN),
                             (x0, y)); y += 18

        y += 8
        help_lines = [
            "----- controls -----",
            " mouse: set goal",
            " WASD : manual drive",
            " T    : toggle AUTO",
            " R    : reset map",
            " O    : save map",
            " L    : load map",
            " ESC  : quit",
        ]
        for ln in help_lines:
            self.screen.blit(self.font_s.render(ln, True, COLOR_TEXT),
                             (x0, y)); y += 18

    # ===================== top-level draw =====================
    def draw(self,
             pose: Optional[Tuple[float, float, float]],
             distances: Optional[Sequence[float]],
             sensor_angles: Sequence[float],
             path: Optional[List[Tuple[int, int]]],
             goal_world: Optional[Tuple[float, float]],
             goal_is_frontier: bool = False) -> None:
        self.screen.fill(BG_COLOR)
        self._draw_grid()
        self._draw_path(path)
        self._draw_goal(goal_world, auto_pick=goal_is_frontier)
        if pose is not None and distances is not None:
            self._draw_rays(pose, distances, sensor_angles)
            self._draw_robot(pose)
        self._draw_panel(pose, distances)
        pygame.display.flip()

    def close(self) -> None:
        pygame.quit()
