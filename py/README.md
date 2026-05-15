# VacBot SLAM Navigator

A real-time indoor SLAM and navigation system for a differential-drive robot.
Two coupled subsystems: an ESP32 handles sensors, state estimation, and motor
control; a Python host runs occupancy-grid SLAM, A\* planning, and a Pygame
visualizer. Communication is over UDP on a shared WiFi network.

This is a production-grade design, not a demo. Every module shares the same
frame convention, units, and message schema so the pipeline composes cleanly.

---

## 1. System architecture

```
              UDP (10 Hz telemetry)
   ┌────────────┐ ─────────────────► ┌──────────────────┐
   │   ESP32    │                    │   PC (Python)    │
   │            │                    │                  │
   │ sensors    │                    │ network.py       │
   │  ultrasonic│                    │ slam.py          │
   │  encoders  │                    │ astar.py         │
   │  MPU6050   │ ◄───────────────── │ visualizer.py    │
   │ fusion     │   UDP (commands)   │ main.py          │
   │ motor ctrl │                    │                  │
   └────────────┘                    └──────────────────┘
```

**Pipeline, every PC tick (~30 FPS):**

1. `network.py` returns the latest telemetry (`x, y, theta, distances[3]`)
   if it's fresh (<500 ms old).
2. `slam.py` updates the occupancy grid with Bresenham ray casts from the
   robot pose along the front/left/right ultrasonic beams.
3. If a goal is set, `astar.py` replans on the inflated grid at ~2 Hz.
4. `main.py` computes a differential-drive command toward the next waypoint
   on the path (or executes WASD manual override).
5. `network.py` sends `{left_speed, right_speed}` back to the ESP32.
6. `visualizer.py` renders the grid, robot, path, and sensor rays.

**ESP32 scheduling (cooperative, non-blocking, no `delay()`):**

| Task          | Rate    | Notes                                           |
|---------------|---------|-------------------------------------------------|
| Ultrasonic    | ~20 Hz  | Round-robin, echo `CHANGE` ISR, 10 µs trigger   |
| Encoders      | event   | RISING ISR, direction inferred from PWM sign    |
| IMU read      | 50 Hz   | Raw I²C, gyro Z integrated to angle             |
| Fusion        | 50 Hz   | Complementary filter, α = 0.98                  |
| Control       | 50 Hz   | PWM ramp, deadband, clamp ±255                  |
| Comms TX/RX   | 10 Hz   | ArduinoJson, UDP, timeout-to-stop               |

Loop iteration stays well under the 20 ms budget; nothing blocks except the
mandatory 10 µs HC-SR04 trigger pulse.

---

## 2. Shared contracts (do not change in one module without the others)

**Frame.** x forward (robot nose), y left, θ CCW positive. All angles
normalized to (−π, π].

**Grid.** 100 × 100 cells at 0.05 m, origin at the center cell (50, 50).
`gx = origin + round(x / res)`, `gy = origin + round(y / res)`. The
visualizer renders so +x is screen-up and +y is screen-left, matching the
robot's view from above.

**Cell values.** `−1` unknown, `0` free, `1` occupied (after thresholding
log-odds at ±L\_THRESH).

**Sensor mounting angles.** front = 0, left = +π/2, right = −π/2.

**UDP schema.**

Robot → PC:
```json
{"x": 0.12, "y": -0.03, "theta": 0.41, "distances": [1.20, 0.85, 2.10], "timestamp": 12345}
```

PC → Robot:
```json
{"left_speed": 120, "right_speed": 140}
```

PWMs are clamped to [−255, 255]. Distances are in meters; values outside
[0.02, 2.50] m or with jumps >0.50 m frame-to-frame are rejected on the
robot side before transmission.

---

## 3. Hardware & wiring

| Component           | Pin(s)                                          |
|---------------------|-------------------------------------------------|
| MPU6050 (I²C)       | SDA=D21, SCL=D22                                |
| Ultrasonic FRONT    | TRIG=D19, ECHO=D18                              |
| Ultrasonic LEFT     | TRIG=D25, ECHO=D26                              |
| Ultrasonic RIGHT    | TRIG=D27, ECHO=D33                              |
| Encoder RIGHT       | D16                                             |
| Encoder LEFT        | D17                                             |
| L298N ENA / IN1/IN2 | D32 / D14 / D13   — drives the **right** motor  |
| L298N ENB / IN3/IN4 | D4  / D2  / D15   — drives the **left** motor   |
| Buzzer              | D23                                             |
| Power               | 2× 18650 → buck converter → ESP32 / motor logic |

**Single-channel encoders.** The wheels have one channel each, so we infer
direction from the sign of the last commanded PWM on that motor. This is
the standard workaround; expect slight overcount during direction reversals.
Replace with quadrature encoders if you need precision odometry.

**OLED.** The spec lists an SSD1306 sharing the I²C bus on D21/D22. The
firmware doesn't draw to it (the spec says to simplify visualization on
the robot side first) but the bus is free for a display library if you
want to add one later.

---

## 4. Software dependencies

**Arduino IDE (or PlatformIO) for ESP32:**

- `ESP32` board package (Espressif), v2.x or v3.x
- `ArduinoJson` v6.x (Benoît Blanchon)
- `WiFi.h`, `WiFiUdp.h`, `Wire.h` — bundled with the ESP32 core

**Python 3.9+ on the host:**

```
cd vacbot_slam/pc
pip install -r requirements.txt
```

Contents: `numpy`, `pygame`.

---

## 5. First-time setup

### 5.1 Configure WiFi and IPs in `esp32/main.ino`

Near the top of the file:

```cpp
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASSWORD";
IPAddress PC_IP(192, 168, 1, 100);   // your host PC's IP
const uint16_t PC_PORT = 5005;
const uint16_t LOCAL_PORT = 5006;
```

### 5.2 Flash the calibration firmware first

Open `esp32/calibration.ino`, flash, then open the Serial Monitor at
**115200 baud**. The calibration tool lets you verify every sensor and
tune the wheel parameters before running the real firmware.

Commands (single character + Enter):

| Key      | Action                                          |
|----------|-------------------------------------------------|
| `h`      | Help                                            |
| `m`      | Show all sensor streams                         |
| `u`      | Ultrasonic distances only                       |
| `e`      | Encoder counts only                             |
| `i`      | IMU raw gyro + accel                            |
| `a`      | Integrated yaw angle                            |
| `f / b`  | Drive forward / backward at current PWM         |
| `l / r`  | Rotate left / right at current PWM              |
| `x`      | Stop                                            |
| `+ / -`  | Increase / decrease jog PWM (±15)               |
| `c`      | Calibrate gyro bias (keep robot still 2 s)      |
| `z`      | Zero encoders                                   |
| `1 <v>`  | Set wheel\_radius (m)                           |
| `2 <v>`  | Set wheel\_base (m)                             |
| `3 <v>`  | Set ticks\_per\_revolution                      |
| `p`      | Print current parameters                        |

**Calibration procedure:**

1. Place the robot still on a flat surface, run `c` to learn the gyro bias.
2. Run `u` and wave your hand at each sensor in turn — confirm front/left/right
   distances respond correctly.
3. Run `e` and rotate each wheel by hand one full revolution — confirm the
   count matches `ticks_per_revolution` (adjust with `3`).
4. Run `a`, then drive `l` or `r` for a known angle (e.g. 90°) — confirm the
   yaw matches. Sign-flip the gyro in software if it's reversed.
5. Run `m`, then `f` for a measured distance — measure the actual travel and
   adjust `wheel_radius` with `1` until the encoder-reported distance matches.
6. Record the final `wheel_radius`, `wheel_base`, and `ticks_per_revolution`
   so you can copy them into `main.ino`.

### 5.3 Copy calibrated values into `main.ino`

```cpp
float wheel_radius = 0.03f;
float wheel_base   = 0.14f;
int   ticks_per_rev = 360;
```

Reflash with `esp32/main.ino`. Open Serial Monitor briefly to confirm WiFi
connection — the firmware prints its IP. Note that IP.

### 5.4 Configure the host

Edit the top of `pc/main.py`:

```python
ROBOT_IP        = "192.168.1.50"   # ESP32 IP from above
ROBOT_PORT      = 5006             # must match LOCAL_PORT
PC_LISTEN_PORT  = 5005             # must match PC_PORT
```

Make sure your host's firewall allows inbound UDP on 5005.

---

## 6. Running the system

With the robot powered up and on the same network:

```
cd vacbot_slam/pc
python main.py
```

You should see the Pygame window: a grey grid (unknown), the robot as a
green triangle at the center, and a small info panel on the right showing
live pose and sensor distances.

### Controls

| Key / mouse              | Action                                       |
|--------------------------|----------------------------------------------|
| Left-click on grid       | Set navigation goal (A\* will plan & drive)  |
| `W / S`                  | Manual forward / reverse (overrides A\*)     |
| `A / D`                  | Manual rotate CCW / CW                       |
| `R`                      | Reset map and pose estimate                  |
| `O`                      | Save map to `maps/map.npy`                   |
| `L`                      | Load map from `maps/map.npy`                 |
| `ESC` or close window    | Quit (sends a stop command on the way out)   |

The save key is `O` rather than `S` to avoid conflicting with WASD reverse.

---

## 7. Failure handling

| Condition                | Behavior                                              |
|--------------------------|-------------------------------------------------------|
| WiFi disconnect (ESP32)  | Auto-reconnect; motors stopped while down             |
| UDP timeout (>500 ms)    | ESP32 stops motors until packets resume               |
| Invalid ultrasonic       | Out-of-range or jump >0.5 m/frame → reading discarded |
| Encoder stall (>1.2 s)   | ESP32 stops motors and pulses the buzzer              |
| SLAM update exception    | Frame skipped, pipeline continues                     |
| Telemetry stale on PC    | `network.latest()` returns `None`; no commands sent   |

---

## 8. File map

```
vacbot_slam/
├── esp32/
│   ├── main.ino          ESP32 firmware: sensors, fusion, control, UDP
│   └── calibration.ino   Serial-driven calibration & jog utility
├── pc/
│   ├── network.py        Threaded UDP send/receive with staleness check
│   ├── slam.py           Log-odds occupancy grid + Bresenham raycasting
│   ├── astar.py          8-connected A* with corner-cut prevention
│   ├── visualizer.py     Pygame rendering and input
│   ├── main.py           Orchestrator (pipeline + control law)
│   └── requirements.txt  numpy, pygame
├── maps/                 Saved occupancy grids (.npy)
└── README.md             This file
```

---

## 9. Tuning hints

- **Drifting yaw?** Recalibrate gyro bias (`c` in calibration), and consider
  dropping α from 0.98 to ~0.95 so the IMU pulls back the encoder integration
  more aggressively.
- **Path plans through tight gaps?** Increase inflation radius in
  `slam.py:inflated()` from 1 cell to 2.
- **Robot oscillates near waypoints?** Increase the lookahead distance
  (cells skipped along the path) in `main.py`, or reduce `Kp` from 1.5 to 1.0.
- **Motors won't start moving below a threshold?** Raise `DEADBAND` in
  `main.ino` until you find the lowest PWM that reliably moves your robot.
