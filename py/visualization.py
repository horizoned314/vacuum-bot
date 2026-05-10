"""
visualization.py
================
Pygame-based real-time visualization for VacBot V2.

Displays:
  - Robot as filled triangle (pointing in heading direction)
  - Path trail (blue line)
  - Obstacle cloud (red dots from ultrasonic readings)
  - HUD with sensor readings, pose, and mode status
  - Grid overlay for spatial reference
"""

import pygame
import math

# ─────────────────────────────────────────────
# Display Configuration
# ─────────────────────────────────────────────
SCREEN_W = 900
SCREEN_H = 700
HUD_W    = 220          # Right-side panel width
MAP_W    = SCREEN_W - HUD_W

FPS      = 30
TITLE    = "VacBot V2 — Real-Time Control"

# Pixels per meter (scale factor for world → screen)
PIXELS_PER_METER = 80   # 80 px = 1 m

# Colors (RGB)
C_BG         = (20,  20,  30)
C_GRID       = (40,  40,  55)
C_PATH       = (50, 120, 220)
C_ROBOT      = (60, 220, 100)
C_ROBOT_DIR  = (255, 255, 80)
C_OBSTACLE   = (220,  60,  60)
C_HUD_BG     = (28,  28,  42)
C_HUD_BORDER = (60,  60,  90)
C_TEXT       = (200, 200, 220)
C_TEXT_GOOD  = (80, 220, 100)
C_TEXT_WARN  = (240, 180,  40)
C_TEXT_BAD   = (220,  70,  70)
C_ORIGIN     = (200, 200, 200)

ROBOT_RADIUS  = 12          # px
OBSTACLE_R    = 3           # px
PATH_WIDTH    = 2           # px


class Visualizer:
    """
    Manages the pygame window and renders the robot world.
    """

    def __init__(self):
        pygame.init()
        pygame.display.set_caption(TITLE)
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        self.clock  = pygame.time.Clock()

        # Fonts
        self.font_sm  = pygame.font.SysFont("monospace", 13)
        self.font_md  = pygame.font.SysFont("monospace", 15, bold=True)
        self.font_lg  = pygame.font.SysFont("monospace", 18, bold=True)

        # Map view offset — camera follows robot
        # Origin starts at center of map area
        self._view_x = MAP_W  // 2
        self._view_y = SCREEN_H // 2

    # ─────────────────────────────────────────
    # Public Update
    # ─────────────────────────────────────────

    def render(self, pose, path_history, obstacles, sensor_data, mode, packet_ok, sweep_grid=None):
        """
        Render one frame.

        pose          : RobotPose (x, y, theta)
        path_history  : list of (x, y) in meters
        obstacles     : iterable of (x, y) in meters
        sensor_data   : latest dict dari UDPReceiver
        mode          : 'manual', 'auto', atau 'sweep'
        packet_ok     : bool
        sweep_grid    : dict dari SweeperController.get_grid_map() (opsional)
        """
        self.screen.fill(C_BG)

        self._draw_grid(MAP_W // 2, SCREEN_H // 2, pose)

        # Tampilkan grid BFS saat mode sweep
        if sweep_grid:
            self._draw_sweep_grid(sweep_grid, pose)

        self._draw_obstacles(obstacles, pose)
        self._draw_path(path_history, pose)
        self._draw_robot(MAP_W // 2, SCREEN_H // 2, pose.theta)
        self._draw_hud(sensor_data, pose, mode, packet_ok)
        self._draw_map_border()

        pygame.display.flip()
        self.clock.tick(FPS)

    # ─────────────────────────────────────────
    # Coordinate Transform
    # ─────────────────────────────────────────

    def _world_to_screen(self, wx, wy, robot_x, robot_y):
        """
        Convert world coordinates (meters) to screen pixels.
        Screen is centered on the robot.
        Y axis is flipped (screen Y grows downward, world Y grows upward).
        """
        sx = MAP_W  // 2 + int((wx - robot_x) * PIXELS_PER_METER)
        sy = SCREEN_H // 2 - int((wy - robot_y) * PIXELS_PER_METER)
        return sx, sy

    def _world_to_px_x(self, w): return int(w * PIXELS_PER_METER)
    def _world_to_px_y(self, w): return int(w * PIXELS_PER_METER)

    # ─────────────────────────────────────────
    # Drawing Routines
    # ─────────────────────────────────────────

    def _draw_grid(self, cam_x, cam_y, pose):
        """Draw a metric grid centered on origin."""
        GRID_SPACING = PIXELS_PER_METER   # 1 meter per cell

        # Number of cells visible
        cols = MAP_W  // GRID_SPACING + 3
        rows = SCREEN_H // GRID_SPACING + 3

        # Offset based on robot position
        offset_x = int(pose.x * PIXELS_PER_METER) % GRID_SPACING
        offset_y = int(pose.y * PIXELS_PER_METER) % GRID_SPACING

        for i in range(-1, cols):
            x = i * GRID_SPACING - offset_x
            pygame.draw.line(self.screen, C_GRID, (x, 0), (x, SCREEN_H), 1)

        for j in range(-1, rows):
            y = j * GRID_SPACING + offset_y
            pygame.draw.line(self.screen, C_GRID, (0, y), (MAP_W, y), 1)

        # Draw world origin marker
        ox, oy = self._world_to_screen(0, 0, pose.x, pose.y)
        if 0 < ox < MAP_W and 0 < oy < SCREEN_H:
            pygame.draw.line(self.screen, C_ORIGIN, (ox - 8, oy), (ox + 8, oy), 2)
            pygame.draw.line(self.screen, C_ORIGIN, (ox, oy - 8), (ox, oy + 8), 2)

    def _draw_path(self, path_history, pose):
        """Draw robot's traveled path as connected line segments."""
        if len(path_history) < 2:
            return

        # Convert all path points to screen coords
        pts = [self._world_to_screen(x, y, pose.x, pose.y) for x, y in path_history]

        # Only draw points inside the map area
        visible = [(x, y) for x, y in pts if 0 <= x <= MAP_W and 0 <= y <= SCREEN_H]
        if len(visible) >= 2:
            pygame.draw.lines(self.screen, C_PATH, False, visible, PATH_WIDTH)

    def _draw_obstacles(self, obstacles, pose):
        """Draw obstacle cloud as small red dots."""
        for wx, wy in obstacles:
            sx, sy = self._world_to_screen(wx, wy, pose.x, pose.y)
            if 0 <= sx <= MAP_W and 0 <= sy <= SCREEN_H:
                pygame.draw.circle(self.screen, C_OBSTACLE, (sx, sy), OBSTACLE_R)

    def _draw_robot(self, cx, cy, theta):
        """
        Draw robot as a filled triangle pointing in the heading direction.
        Triangle: front tip + two rear corners rotated by theta.
        """
        R = ROBOT_RADIUS

        # Triangle vertices in local frame
        # Front tip, rear-left, rear-right
        local_pts = [
            (R * 1.8, 0),        # front tip
            (-R, -R * 0.9),      # rear left
            (-R, +R * 0.9),      # rear right
        ]

        # Rotate and translate to screen
        def rot(px, py):
            rx = px * math.cos(theta) - py * math.sin(theta)
            ry = px * math.sin(theta) + py * math.cos(theta)
            # Flip Y for screen coordinates
            return (int(cx + rx), int(cy - ry))

        pts = [rot(px, py) for px, py in local_pts]
        pygame.draw.polygon(self.screen, C_ROBOT, pts)
        pygame.draw.polygon(self.screen, C_ROBOT_DIR, pts, 2)  # outline

        # Draw a direction indicator dot at front tip
        front_x = cx + int(R * 2.0 * math.cos(theta))
        front_y = cy - int(R * 2.0 * math.sin(theta))
        pygame.draw.circle(self.screen, C_ROBOT_DIR, (front_x, front_y), 3)

    def _draw_hud(self, sensor_data, pose, mode, packet_ok):
        """Draw the right-side information panel."""
        # Panel background
        hud_rect = pygame.Rect(MAP_W, 0, HUD_W, SCREEN_H)
        pygame.draw.rect(self.screen, C_HUD_BG, hud_rect)
        pygame.draw.line(self.screen, C_HUD_BORDER, (MAP_W, 0), (MAP_W, SCREEN_H), 2)

        x = MAP_W + 12
        y = 15

        def text(label, value="", color=C_TEXT, font=None):
            nonlocal y
            f = font or self.font_sm
            surf = f.render(f"{label}{value}", True, color)
            self.screen.blit(surf, (x, y))
            y += surf.get_height() + 4

        def sep():
            nonlocal y
            pygame.draw.line(self.screen, C_HUD_BORDER, (MAP_W + 8, y), (MAP_W + HUD_W - 8, y), 1)
            y += 8

        # Title
        text("VacBot V2", font=self.font_lg, color=C_TEXT_GOOD)
        sep()

        # Connection status
        conn_color = C_TEXT_GOOD if packet_ok else C_TEXT_BAD
        conn_label = "● LIVE" if packet_ok else "● NO SIGNAL"
        text(conn_label, color=conn_color, font=self.font_md)
        sep()

        # Mode
        mode_color = C_TEXT_WARN if mode in ("auto","sweep") else C_TEXT
        text(f"Mode: {mode.upper()}", color=mode_color, font=self.font_md)
        sep()

        # Pose
        text("── POSE ──", font=self.font_md)
        text(f"  X:     {pose.x:+.3f} m")
        text(f"  Y:     {pose.y:+.3f} m")
        text(f"  Yaw: {math.degrees(pose.theta):+.1f} °")
        sep()

        # Ultrasonic
        front = sensor_data.get("ultra_front", 999)
        left  = sensor_data.get("ultra_left",  999)
        right = sensor_data.get("ultra_right", 999)

        def sensor_color(v):
            if v < 20:  return C_TEXT_BAD
            if v < 50:  return C_TEXT_WARN
            return C_TEXT_GOOD

        text("── ULTRASONIC ──", font=self.font_md)
        text(f"  Front: {front:5.1f} cm", color=sensor_color(front))
        text(f"  Left:  {left:5.1f} cm",  color=sensor_color(left))
        text(f"  Right: {right:5.1f} cm", color=sensor_color(right))
        sep()

        # Encoders
        text("── ENCODERS ──", font=self.font_md)
        text(f"  Left:  {sensor_data.get('enc_left',  0)}")
        text(f"  Right: {sensor_data.get('enc_right', 0)}")
        sep()

        # Controls help
        text("── CONTROLS ──", font=self.font_md)
        text("  W/S/A/D  move")
        text("  SPACE    stop")
        text("  TAB      ganti mode")
        text("  G        toggle sweep")
        text("  R        reset pose")
        text("  ESC      quit")

    def _draw_map_border(self):
        """Draw border around the map area."""
        pygame.draw.rect(self.screen, C_HUD_BORDER, (0, 0, MAP_W, SCREEN_H), 2)

    def _draw_sweep_grid(self, sweep_grid: dict, pose):
        """
        Visualisasikan grid BFS Sweeper.
          Hijau = sel sudah dikunjungi (nilai 1)
          Merah = obstacle terdeteksi (nilai -1)
        """
        from navigation import CELL_SIZE_M

        CELL_PX = int(CELL_SIZE_M * PIXELS_PER_METER)
        surf    = pygame.Surface((MAP_W, SCREEN_H), pygame.SRCALPHA)

        for (gx, gy), val in sweep_grid.items():
            wx = gx * CELL_SIZE_M
            wy = -gy * CELL_SIZE_M   # balik Y (grid ke bawah, world ke atas)
            sx, sy = self._world_to_screen(wx, wy, pose.x, pose.y)

            if not (0 <= sx <= MAP_W and 0 <= sy <= SCREEN_H):
                continue

            if val == 1:
                pygame.draw.rect(surf, (50, 180, 80, 55),
                                 (sx - CELL_PX//2, sy - CELL_PX//2, CELL_PX, CELL_PX))
                pygame.draw.rect(surf, (50, 180, 80, 120),
                                 (sx - CELL_PX//2, sy - CELL_PX//2, CELL_PX, CELL_PX), 1)
            elif val == -1:
                pygame.draw.rect(surf, (220, 60, 60, 80),
                                 (sx - CELL_PX//2, sy - CELL_PX//2, CELL_PX, CELL_PX))

        self.screen.blit(surf, (0, 0))
