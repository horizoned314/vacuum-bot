"""
navigation.py — VacBot Final
=============================
Isi:
  1. RobotPose            — dataclass posisi robot
  2. BFS utilities        — algoritma dari dungba88/cleaner_robot (tidak perlu install)
  3. VacBotRobot          — adapter robot ke API BFS
  4. SweeperController    — state machine eksekutor BFS ke robot fisik
  5. Navigator            — kelas utama yang dipakai main.py

Tidak perlu install library apapun selain stdlib + pygame.
Algoritma BFS sudah ditulis ulang di sini langsung.
"""

import math
from collections import deque
from dataclasses import dataclass

# ══════════════════════════════════════════════
# Konstanta Fisik Robot
# ══════════════════════════════════════════════
WHEEL_DIAMETER_M    = 0.065
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER_M
TICKS_PER_REV       = 20
DIST_PER_TICK       = WHEEL_CIRCUMFERENCE / TICKS_PER_REV  # meter/tick

SENSOR_FRONT_OFFSET = 0.12   # offset sensor depan dari pusat robot (m)
SENSOR_SIDE_OFFSET  = 0.08   # offset sensor kiri/kanan dari pusat robot (m)
MAX_OBSTACLE_POINTS = 800

# ══════════════════════════════════════════════
# Konstanta BFS Sweeper
# ══════════════════════════════════════════════
CELL_SIZE_M        = 0.20    # 1 sel grid = 20 cm
OBSTACLE_THRESH_CM = 25.0    # jarak < ini = obstacle/dinding
TURN_DONE_DEG      = 12.0    # toleransi selesai putar 90°


# ══════════════════════════════════════════════
# RobotPose
# ══════════════════════════════════════════════
@dataclass
class RobotPose:
    """Posisi dan orientasi robot di world frame (meter, radian)."""
    x:     float = 0.0
    y:     float = 0.0
    theta: float = 0.0   # radian, dari MPU6050


# ══════════════════════════════════════════════
# BFS Utilities
# Diadaptasi dari: github.com/dungba88/cleaner_robot
# Arah: 0=kanan, 1=atas, 2=kiri, 3=bawah
# ══════════════════════════════════════════════

def _dx(direction: int) -> int:
    return [1, 0, -1, 0][direction]

def _dy(direction: int) -> int:
    """dy di koordinat grid (y bertambah ke bawah)."""
    return [0, -1, 0, 1][direction]

def _bfs_nearest_unvisited(pos: dict, facing: int, grid: dict, spiral: bool = True):
    """
    BFS dari pos saat ini, cari sel (gx, gy) terdekat yang belum dikunjungi.
    Kembalikan list of direction (path) atau None jika semua sudah dikunjungi.

    grid: dict { (gx,gy): 1=visited | -1=obstacle }
          sel tidak ada di dict = belum dikunjungi (unvisited)
    spiral=True: favor belok kiri → 5-10% lebih efisien (Spiral BFS)
    """
    start = (pos['x'], pos['y'])
    queue = deque()
    queue.append((start[0], start[1], facing, []))
    seen = {start}

    while queue:
        cx, cy, cd, path = queue.popleft()

        # Spiral: utamakan lurus dulu, lalu kiri, kanan, balik
        dirs = [(cd + i) % 4 for i in ([0, 1, 3, 2] if spiral else [0, 1, 2, 3])]

        for d in dirs:
            nx, ny = cx + _dx(d), cy + _dy(d)
            key = (nx, ny)

            if key in seen:
                continue
            seen.add(key)

            val = grid.get(key)
            if val == -1:
                continue                    # obstacle, skip

            new_path = path + [d]

            if val is None:
                return new_path             # ← sel target ditemukan!

            queue.append((nx, ny, d, new_path))   # visited, bisa dilewati

    return None   # semua sel sudah dijelajahi


# ══════════════════════════════════════════════
# VacBotRobot — adapter ke API dungba88
# ══════════════════════════════════════════════

class VacBotRobot:
    """
    Mengimplementasikan interface Robot dari dungba88/cleaner_robot:
      turn_left(), turn_right(), move()

    Tidak mengeksekusi langsung ke hardware.
    Mencatat niat ke _cmd_queue; SweeperController yang mengeksekusi.
    """

    def __init__(self):
        self.facing   = 0              # 0=kanan,1=atas,2=kiri,3=bawah
        self.grid_pos = {'x': 0, 'y': 0}
        self._sensors: dict = {}
        self._cmd_queue: list = []

    def update_sensors(self, data: dict):
        self._sensors = data

    def turn_left(self):
        self.facing = (self.facing + 1) % 4
        self._cmd_queue.append("left_90")

    def turn_right(self):
        self.facing = (self.facing + 3) % 4
        self._cmd_queue.append("right_90")

    def move(self) -> bool:
        """Return True jika bisa maju (tidak ada obstacle di depan)."""
        clear = self._sensors.get("ultra_front", 999.0) > OBSTACLE_THRESH_CM
        if clear:
            self.grid_pos['x'] += _dx(self.facing)
            self.grid_pos['y'] += _dy(self.facing)
            self._cmd_queue.append("forward")
        else:
            self._cmd_queue.append("blocked")
        return clear

    def pop_cmd(self):
        return self._cmd_queue.pop(0) if self._cmd_queue else None


# ══════════════════════════════════════════════
# SweeperController
# ══════════════════════════════════════════════

class SweeperController:
    """
    State machine yang mengeksekusi algoritma BFS Sweeper ke robot fisik.

    State:
      IDLE    → panggil BFS, tentukan langkah berikutnya
      TURNING → kirim turn, tunggu yaw berubah ~90° (dari MPU6050)
      MOVING  → kirim forward, tunggu encoder bertambah CELL_SIZE_M
      DONE    → seluruh area sudah disapu
    """

    IDLE    = "idle"
    TURNING = "turning"
    MOVING  = "moving"
    DONE    = "done"

    def __init__(self):
        self.robot   = VacBotRobot()
        self.grid    = {(0, 0): 1}    # grid map BFS
        self._spiral = True
        self._state  = self.IDLE
        self._path: list = []
        self._active = False

        # state TURNING
        self._turn_dir        = "left"
        self._turn_start_yaw  = 0.0
        self._turn_yaw_locked = False

        # state MOVING
        self._move_start_enc = None

    # ── Public ──────────────────────────────

    def activate(self):
        self._active = True
        self._state  = self.IDLE
        self._path   = []
        print("[Sweeper] Aktif — BFS coverage mode")

    def deactivate(self):
        self._active = False
        print("[Sweeper] Nonaktif")

    @property
    def active(self) -> bool:
        return self._active

    def is_done(self) -> bool:
        return self._state == self.DONE

    def get_grid(self) -> dict:
        """Return salinan grid map untuk visualisasi."""
        return dict(self.grid)

    def tick(self, sensor_data: dict, pose: RobotPose, enc_avg: int) -> tuple:
        """
        Dipanggil setiap frame dari Navigator.
        Return (command: str, speed: int).
        """
        if not self._active or self._state == self.DONE:
            return ("stop", 0)

        self.robot.update_sensors(sensor_data)

        if self._state == self.IDLE:
            return self._do_idle(sensor_data)
        if self._state == self.TURNING:
            return self._do_turning(pose)
        if self._state == self.MOVING:
            return self._do_moving(sensor_data, enc_avg)

        return ("stop", 0)

    # ── State: IDLE ─────────────────────────

    def _do_idle(self, sensor_data: dict) -> tuple:
        if not self._path:
            self._path = _bfs_nearest_unvisited(
                self.robot.grid_pos, self.robot.facing, self.grid, self._spiral
            )
            if self._path is None:
                print("[Sweeper] Selesai! Semua area sudah disapu.")
                self._state = self.DONE
                return ("stop", 0)
            self._path.reverse()   # BFS return path target→start, kita balik
            print(f"[Sweeper] {len(self._path)} langkah ke target berikutnya")

        return self._next_step(sensor_data)

    def _next_step(self, sensor_data: dict) -> tuple:
        if not self._path:
            self._state = self.IDLE
            return ("stop", 0)

        target_dir = self._path.pop()
        turns      = (target_dir - self.robot.facing) % 4

        if turns == 0:
            # Hadap benar → coba maju
            can = self.robot.move()
            self.robot.pop_cmd()
            if can:
                gx = self.robot.grid_pos['x']
                gy = self.robot.grid_pos['y']
                self.grid[(gx, gy)] = 1
                self._move_start_enc = None
                self._state = self.MOVING
                return ("forward", 150)
            else:
                # Obstacle → tandai, minta BFS ulang
                bx = self.robot.grid_pos['x'] + _dx(self.robot.facing)
                by = self.robot.grid_pos['y'] + _dy(self.robot.facing)
                self.grid[(bx, by)] = -1
                self._path = []
                self._state = self.IDLE
                return ("stop", 0)

        elif turns == 3:
            # Lebih efisien putar kanan 1x daripada kiri 3x
            self.robot.turn_right()
            self.robot.pop_cmd()
            self._path.append(target_dir)   # kembalikan ke path
            self._begin_turn("right")
            return ("right", 140)
        else:
            self.robot.turn_left()
            self.robot.pop_cmd()
            self._path.append(target_dir)
            self._begin_turn("left")
            return ("left", 140)

    # ── State: TURNING ───────────────────────

    def _begin_turn(self, direction: str):
        self._turn_dir        = direction
        self._turn_yaw_locked = False
        self._state           = self.TURNING

    def _do_turning(self, pose: RobotPose) -> tuple:
        """Tunggu yaw berubah ±90° dari titik mulai putar."""
        if not self._turn_yaw_locked:
            self._turn_start_yaw  = pose.theta
            self._turn_yaw_locked = True

        delta = math.degrees(pose.theta - self._turn_start_yaw)
        while delta >  180: delta -= 360
        while delta < -180: delta += 360

        target = 90.0 if self._turn_dir == "left" else -90.0

        if abs(delta - target) < TURN_DONE_DEG:
            self._turn_yaw_locked = False
            self._state = self.IDLE
            return ("stop", 0)

        return ("left" if self._turn_dir == "left" else "right", 140)

    # ── State: MOVING ────────────────────────

    def _do_moving(self, sensor_data: dict, enc_avg: int) -> tuple:
        """Tunggu robot maju 1 sel (CELL_SIZE_M) berdasarkan encoder."""
        if self._move_start_enc is None:
            self._move_start_enc = enc_avg

        dist_moved    = (enc_avg - self._move_start_enc) * DIST_PER_TICK
        front_blocked = sensor_data.get("ultra_front", 999.0) < OBSTACLE_THRESH_CM

        if dist_moved >= CELL_SIZE_M or front_blocked:
            if front_blocked:
                bx = self.robot.grid_pos['x'] + _dx(self.robot.facing)
                by = self.robot.grid_pos['y'] + _dy(self.robot.facing)
                self.grid[(bx, by)] = -1
            self._state = self.IDLE
            return ("stop", 0)

        # Catat obstacle samping sambil jalan
        f = self.robot.facing
        if sensor_data.get("ultra_left",  999.0) < OBSTACLE_THRESH_CM:
            lx = self.robot.grid_pos['x'] + _dx((f + 1) % 4)
            ly = self.robot.grid_pos['y'] + _dy((f + 1) % 4)
            self.grid[(lx, ly)] = -1
        if sensor_data.get("ultra_right", 999.0) < OBSTACLE_THRESH_CM:
            rx = self.robot.grid_pos['x'] + _dx((f + 3) % 4)
            ry = self.robot.grid_pos['y'] + _dy((f + 3) % 4)
            self.grid[(rx, ry)] = -1

        return ("forward", 150)


# ══════════════════════════════════════════════
# Navigator — kelas utama yang dipakai main.py
# ══════════════════════════════════════════════

class Navigator:
    """
    Kelas utama navigation.
    Menggabungkan dead reckoning, obstacle cloud, dan BFS Sweeper.

    Cara pakai di main.py:
        nav = Navigator()

        # Setiap frame:
        nav.update(sensor_data)

        # Ambil command sesuai mode:
        cmd, spd = nav.get_auto_command(sensor_data)    # mode auto
        cmd, spd = nav.get_sweep_command(sensor_data)   # mode sweep

        # Akses sweeper:
        nav.sweeper.activate()
        nav.sweeper.deactivate()
        nav.sweeper.is_done()
        nav.get_sweep_grid()    # untuk visualisasi
    """

    def __init__(self):
        self.pose = RobotPose()

        self._prev_enc_left  = 0
        self._prev_enc_right = 0

        self.path_history: list = [(0.0, 0.0)]
        self.obstacles          = deque(maxlen=MAX_OBSTACLE_POINTS)

        # BFS Sweeper — selalu dibuat, tidak perlu install apapun
        self.sweeper = SweeperController()

    # ── Update setiap frame ──────────────────

    def update(self, sensor_data: dict):
        """Panggil setiap frame. Update posisi dan obstacle map."""
        enc_left  = sensor_data.get("enc_left",  0)
        enc_right = sensor_data.get("enc_right", 0)
        yaw       = sensor_data.get("yaw",       self.pose.theta)

        dl = enc_left  - self._prev_enc_left
        dr = enc_right - self._prev_enc_right
        self._prev_enc_left  = enc_left
        self._prev_enc_right = enc_right

        # Dead reckoning: d = rata-rata dua encoder × meter/tick
        d = ((dl + dr) / 2.0) * DIST_PER_TICK
        self.pose.theta = yaw
        self.pose.x    += d * math.cos(self.pose.theta)
        self.pose.y    += d * math.sin(self.pose.theta)

        if self._dist_from_last() > 0.01:
            self.path_history.append((self.pose.x, self.pose.y))

        self._update_obstacles(sensor_data)

    def _dist_from_last(self) -> float:
        if not self.path_history:
            return float("inf")
        lx, ly = self.path_history[-1]
        return math.hypot(self.pose.x - lx, self.pose.y - ly)

    def _update_obstacles(self, data: dict):
        MAX_RANGE = 3.0
        sensors = [
            ("ultra_front",  0,              SENSOR_FRONT_OFFSET),
            ("ultra_left",   math.pi / 2,    SENSOR_SIDE_OFFSET),
            ("ultra_right", -math.pi / 2,    SENSOR_SIDE_OFFSET),
        ]
        for key, angle_offset, offset in sensors:
            d = data.get(key, 999.0) / 100.0   # cm → m
            if d < MAX_RANGE:
                a = self.pose.theta + angle_offset
                self.obstacles.append((
                    self.pose.x + (offset + d) * math.cos(a),
                    self.pose.y + (offset + d) * math.sin(a),
                ))

    # ── Mode commands ────────────────────────

    def get_auto_command(self, sensor_data: dict) -> tuple:
        """Mode AUTO: obstacle avoidance reaktif."""
        front = sensor_data.get("ultra_front", 999.0)
        left  = sensor_data.get("ultra_left",  999.0)
        right = sensor_data.get("ultra_right", 999.0)

        if front > OBSTACLE_THRESH_CM:
            return ("forward", 160)
        if right > left:
            return ("right", 140)
        if left > right:
            return ("left", 140)
        return ("backward", 130)

    def get_sweep_command(self, sensor_data: dict) -> tuple:
        """Mode SWEEP: BFS coverage algorithm."""
        enc_avg = (sensor_data.get("enc_left", 0) + sensor_data.get("enc_right", 0)) // 2
        return self.sweeper.tick(sensor_data, self.pose, enc_avg)

    def get_sweep_grid(self) -> dict:
        """Kembalikan grid BFS untuk divisualisasikan di pygame."""
        return self.sweeper.get_grid()

    # ── Utility ─────────────────────────────

    def reset_pose(self):
        self.pose             = RobotPose()
        self.path_history     = [(0.0, 0.0)]
        self._prev_enc_left   = 0
        self._prev_enc_right  = 0
        print("[Nav] Pose di-reset ke origin.")
