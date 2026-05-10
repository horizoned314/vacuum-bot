"""
udp_receiver.py
===============
Handles bidirectional UDP communication with the ESP32.

- Receives JSON telemetry (yaw, encoders, ultrasonics) from robot
- Sends JSON motion commands to robot
- Non-blocking receive using setblocking(False)
"""

import socket
import json
import threading
import time

# ─────────────────────────────────────────────
# Network Configuration
# ─────────────────────────────────────────────
ESP32_IP        = "192.168.1.100"   # Must match ESP32 static IP
PC_LISTEN_PORT  = 4210              # Port ESP32 sends TO
ESP32_SEND_PORT = 4211              # Port ESP32 listens ON

BUFFER_SIZE = 1024


class UDPReceiver:
    """
    Thread-safe UDP handler for robot communication.
    Runs a background thread for receiving packets.
    Provides send() method for outgoing commands.
    """

    def __init__(self):
        # Latest parsed telemetry (thread-safe via lock)
        self._data = {
            "yaw": 0.0,
            "enc_left": 0,
            "enc_right": 0,
            "ultra_front": 999.0,
            "ultra_left": 999.0,
            "ultra_right": 999.0,
        }
        self._lock = threading.Lock()
        self._running = False

        # Receive socket: listens for ESP32 telemetry
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.bind(("", PC_LISTEN_PORT))
        self._recv_sock.setblocking(False)

        # Send socket: sends commands to ESP32
        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._last_packet_time = 0.0
        self._packet_count = 0

    def start(self):
        """Start background receive thread."""
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        print(f"[UDP] Receiver started on port {PC_LISTEN_PORT}")
        print(f"[UDP] Sending commands to {ESP32_IP}:{ESP32_SEND_PORT}")

    def stop(self):
        """Stop background thread and close sockets."""
        self._running = False
        self._recv_sock.close()
        self._send_sock.close()
        print("[UDP] Stopped.")

    def get_data(self) -> dict:
        """Return a copy of the latest telemetry data (thread-safe)."""
        with self._lock:
            return dict(self._data)

    def send_command(self, move: str, speed: int = 150):
        """
        Send a motion command to the robot.
        move: 'forward' | 'backward' | 'left' | 'right' | 'stop'
        speed: 0–255 (PWM duty cycle)
        """
        cmd = {"move": move, "speed": speed}
        payload = json.dumps(cmd).encode("utf-8")
        try:
            self._send_sock.sendto(payload, (ESP32_IP, ESP32_SEND_PORT))
        except OSError as e:
            print(f"[UDP] Send error: {e}")

    def get_packet_rate(self) -> float:
        """Returns time since last received packet (seconds)."""
        if self._last_packet_time == 0:
            return float("inf")
        return time.time() - self._last_packet_time

    # ─────────────────────────────────────────
    # Internal
    # ─────────────────────────────────────────

    def _receive_loop(self):
        """Background thread: continuously receive and parse UDP packets."""
        while self._running:
            try:
                raw, _ = self._recv_sock.recvfrom(BUFFER_SIZE)
                self._parse_packet(raw)
            except BlockingIOError:
                # No data available — yield CPU briefly
                time.sleep(0.005)
            except OSError:
                break   # Socket was closed

    def _parse_packet(self, raw: bytes):
        """Parse JSON from raw UDP bytes and update internal state."""
        try:
            data = json.loads(raw.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            print(f"[UDP] Parse error: {e}")
            return

        # Only update keys that exist in the packet (graceful if fields missing)
        with self._lock:
            for key in self._data:
                if key in data:
                    self._data[key] = data[key]

        self._last_packet_time = time.time()
        self._packet_count += 1
