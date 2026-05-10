"""
main.py — VacBot Final
=======================
Entry point aplikasi PC VacBot V2.

Kontrol:
  W/A/S/D atau Arrow  → gerak manual
  SPACE               → berhenti
  TAB                 → ganti mode: manual → auto → sweep → manual
  G                   → langsung toggle sweep on/off
  R                   → reset odometry ke origin
  ESC                 → keluar

Mode:
  manual  → kontrol keyboard
  auto    → obstacle avoidance reaktif sederhana
  sweep   → BFS coverage (menyapu seluruh ruangan otomatis)
"""

import pygame
import sys
import time

from receiver  import UDPReceiver
from navigation    import Navigator
from visualization import Visualizer

DEFAULT_SPEED    = 160
PACKET_TIMEOUT_S = 2.0

KEY_COMMANDS = {
    pygame.K_w:     "forward",
    pygame.K_UP:    "forward",
    pygame.K_s:     "backward",
    pygame.K_DOWN:  "backward",
    pygame.K_a:     "left",
    pygame.K_LEFT:  "left",
    pygame.K_d:     "right",
    pygame.K_RIGHT: "right",
    pygame.K_SPACE: "stop",
}

MODES = ["manual", "auto", "sweep"]


def main():
    udp = UDPReceiver()
    nav = Navigator()
    viz = Visualizer()

    udp.start()

    mode_idx     = 0
    mode         = MODES[mode_idx]
    pressed_keys = set()

    print("[Main] VacBot V2 siap.")
    print("[Main] WASD=gerak | TAB=ganti mode | G=sweep | R=reset | ESC=keluar")

    running = True
    while running:

        # ── Event loop ────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:

                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_TAB:
                    _set_mode_idx(mode, nav, udp, (mode_idx + 1) % len(MODES))
                    mode_idx = (mode_idx + 1) % len(MODES)
                    mode     = MODES[mode_idx]
                    _on_mode_enter(mode, nav)
                    print(f"[Main] Mode: {mode.upper()}")

                elif event.key == pygame.K_g:
                    if mode != "sweep":
                        _set_mode_idx(mode, nav, udp, MODES.index("sweep"))
                        mode_idx = MODES.index("sweep")
                        mode     = "sweep"
                        _on_mode_enter(mode, nav)
                    else:
                        _set_mode_idx(mode, nav, udp, MODES.index("manual"))
                        mode_idx = MODES.index("manual")
                        mode     = "manual"
                    print(f"[Main] Mode: {mode.upper()}")

                elif event.key == pygame.K_r:
                    nav.reset_pose()

                elif event.key in KEY_COMMANDS:
                    pressed_keys.add(event.key)

            elif event.type == pygame.KEYUP:
                if event.key in KEY_COMMANDS:
                    pressed_keys.discard(event.key)

        # ── Sensor data ───────────────────────
        sensor_data = udp.get_data()

        # ── Command berdasarkan mode ──────────
        if mode == "manual":
            cmd = _resolve_keys(pressed_keys)
            spd = DEFAULT_SPEED if cmd != "stop" else 0
            udp.send_command(cmd, spd)

        elif mode == "auto":
            cmd, spd = nav.get_auto_command(sensor_data)
            udp.send_command(cmd, spd)

        elif mode == "sweep":
            cmd, spd = nav.get_sweep_command(sensor_data)
            udp.send_command(cmd, spd)
            if nav.sweeper.is_done():
                print("[Main] Sweep selesai! Kembali ke manual.")
                mode_idx = MODES.index("manual")
                mode     = "manual"

        # ── Update dead reckoning ─────────────
        nav.update(sensor_data)

        # ── Render ───────────────────────────
        viz.render(
            pose         = nav.pose,
            path_history = nav.path_history,
            obstacles    = nav.obstacles,
            sensor_data  = sensor_data,
            mode         = mode,
            packet_ok    = udp.get_packet_rate() < PACKET_TIMEOUT_S,
            sweep_grid   = nav.get_sweep_grid() if mode == "sweep" else {},
        )

    # ── Cleanup ───────────────────────────────
    print("[Main] Mematikan...")
    udp.send_command("stop", 0)
    time.sleep(0.2)
    udp.stop()
    pygame.quit()
    sys.exit(0)


# ── Helpers ──────────────────────────────────

def _on_mode_enter(mode: str, nav: Navigator):
    """Aksi saat masuk ke mode baru."""
    if mode == "sweep":
        nav.sweeper.activate()

def _set_mode_idx(old_mode: str, nav: Navigator, udp: UDPReceiver, new_idx: int):
    """Aksi saat keluar dari mode lama."""
    if old_mode in ("auto", "sweep"):
        udp.send_command("stop", 0)
    if old_mode == "sweep":
        nav.sweeper.deactivate()

def _resolve_keys(pressed: set) -> str:
    """Konversi set tombol yang ditekan ke 1 perintah gerak."""
    for k in (pygame.K_w, pygame.K_UP):
        if k in pressed: return "forward"
    for k in (pygame.K_s, pygame.K_DOWN):
        if k in pressed: return "backward"
    for k in (pygame.K_a, pygame.K_LEFT):
        if k in pressed: return "left"
    for k in (pygame.K_d, pygame.K_RIGHT):
        if k in pressed: return "right"
    return "stop"


if __name__ == "__main__":
    main()
