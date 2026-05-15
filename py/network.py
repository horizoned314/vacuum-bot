"""
network.py - UDP link between PC and robot.

Schema:

    Robot -> PC (telemetry):
        { "x": float, "y": float, "theta": float,
          "distances": [front, left, right],   # meters, -1 = invalid
          "timestamp": int }                   # robot millis()

    PC -> Robot (command):
        { "left_speed": int, "right_speed": int }   # [-255, 255]

Robust to:
    - packet loss (latest() returns None when stale)
    - corrupt / partial JSON (silently discarded)
    - socket errors (logged, never raises out of the worker thread)
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Optional, Dict, Any


class UDPLink:
    def __init__(self, robot_ip: str, robot_port: int, listen_port: int,
                 stale_after: float = 0.5):
        self.robot_addr = (robot_ip, robot_port)
        self.stale_after = stale_after

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", listen_port))
        self.sock.settimeout(0.05)

        self._lock = threading.Lock()
        self._latest: Optional[Dict[str, Any]] = None
        self._latest_time: float = 0.0
        self._running = True

        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    # ---------------- worker ----------------
    def _rx_loop(self) -> None:
        while self._running:
            try:
                data, _addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(msg, dict):
                continue
            if not all(k in msg for k in ("x", "y", "theta", "distances")):
                continue
            d = msg.get("distances")
            if not isinstance(d, (list, tuple)) or len(d) != 3:
                continue
            with self._lock:
                self._latest = msg
                self._latest_time = time.time()

    # ---------------- public api ----------------
    def latest(self) -> Optional[Dict[str, Any]]:
        """Return most recent telemetry dict, or None if stale / never received."""
        with self._lock:
            if self._latest is None:
                return None
            if time.time() - self._latest_time > self.stale_after:
                return None
            return dict(self._latest)

    def send_cmd(self, left_speed: int, right_speed: int) -> None:
        """Send a motor command. Values clamped to [-255, 255]."""
        l = max(-255, min(255, int(left_speed)))
        r = max(-255, min(255, int(right_speed)))
        payload = json.dumps({"left_speed": l, "right_speed": r}).encode("utf-8")
        try:
            self.sock.sendto(payload, self.robot_addr)
        except OSError:
            pass  # network down - higher level will see telemetry go stale

    def close(self) -> None:
        self._running = False
        try:
            self.sock.close()
        except OSError:
            pass
