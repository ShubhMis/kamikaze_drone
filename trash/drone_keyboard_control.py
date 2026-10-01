#!/usr/bin/env python3
"""
drone_keyboard_control.py  v2
════════════════════════════════════════════════════════════════════
Hold-key controller for the PX4-style drone in Gazebo Harmonic.

  HOLD a movement key → drone moves continuously.
  RELEASE → drone stops.
  SPACE   → ARM toggle (debounced: 300 ms lockout after each press).

Controls:
  SPACE       — Arm / Disarm (toggle, debounced)
  W / ↑       — Climb (hold)
  S / ↓       — Descend (hold)
  A / ←       — Strafe left (hold)
  D / →       — Strafe right (hold)
  I           — Pitch forward (hold)
  K           — Pitch back (hold)
  J           — Yaw left (hold)
  L           — Yaw right (hold)
  X           — Emergency STOP
  Q / Ctrl-C  — Quit

Usage:
  Terminal 1: bash run_px4_sim.sh  →  click ▶ Play
  Terminal 2: python3 drone_keyboard_control.py
════════════════════════════════════════════════════════════════════
"""

import subprocess
import sys
import time
import threading
import tty
import termios
import select

# ─── CONFIG ──────────────────────────────────────────────────────
LINEAR_SPEED  = 2.0    # m/s
YAW_SPEED     = 1.2    # rad/s
SEND_HZ       = 20     # how often to resend the current cmd (keeps drone moving)
ARM_DEBOUNCE  = 0.40   # seconds — ignore extra SPACE presses within this window

CMD_VEL_TOPIC = "/drone/cmd_vel"
ENABLE_TOPIC  = "/drone/enable"


# ─── GZ HELPERS ──────────────────────────────────────────────────
def gz_pub_twist(lx=0.0, ly=0.0, lz=0.0, az=0.0):
    payload = (
        f"linear: {{x: {lx:.2f}, y: {ly:.2f}, z: {lz:.2f}}}, "
        f"angular: {{x: 0.00, y: 0.00, z: {az:.2f}}}"
    )
    subprocess.Popen(
        ["gz", "topic", "-t", CMD_VEL_TOPIC, "-m", "gz.msgs.Twist", "-p", payload],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def gz_enable(state: bool):
    payload = "data: true" if state else "data: false"
    subprocess.Popen(
        ["gz", "topic", "-t", ENABLE_TOPIC, "-m", "gz.msgs.Boolean", "-p", payload],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    print(f"  [Motor] {'ARMED ✓' if state else 'DISARMED'}")


# ─── RAW KEYPRESS ────────────────────────────────────────────────
def get_key_raw(fd):
    """Read one keypress (non-blocking, 50ms timeout). Returns '' if nothing."""
    ready, _, _ = select.select([sys.stdin], [], [], 0.05)
    if not ready:
        return ''
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        # Arrow key: ESC [ A/B/C/D
        r2, _, _ = select.select([sys.stdin], [], [], 0.05)
        if r2 and sys.stdin.read(1) == '[':
            r3, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r3:
                ch3 = sys.stdin.read(1)
                return {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}.get(ch3, '')
        return ''
    return ch


# ─── CMD SENDER THREAD ───────────────────────────────────────────
class CmdSender:
    """Background thread that re-sends the current velocity at SEND_HZ."""
    def __init__(self):
        self._cmd   = (0.0, 0.0, 0.0, 0.0)   # lx, ly, lz, az
        self._lock  = threading.Lock()
        self._stop  = False
        self._armed = False
        t = threading.Thread(target=self._run, daemon=True)
        t.start()

    def set_cmd(self, lx, ly, lz, az):
        with self._lock:
            self._cmd = (lx, ly, lz, az)

    def set_armed(self, armed):
        self._armed = armed

    def stop(self):
        self._stop = True

    def _run(self):
        interval = 1.0 / SEND_HZ
        while not self._stop:
            if self._armed:
                with self._lock:
                    lx, ly, lz, az = self._cmd
                gz_pub_twist(lx, ly, lz, az)
            time.sleep(interval)


# ─── KEY MAP ─────────────────────────────────────────────────────
KEY_MAP = {
    'w':     ( 0,  0,  LINEAR_SPEED,  0,          'CLIMB'),
    'UP':    ( 0,  0,  LINEAR_SPEED,  0,          'CLIMB'),
    's':     ( 0,  0, -LINEAR_SPEED,  0,          'DESCEND'),
    'DOWN':  ( 0,  0, -LINEAR_SPEED,  0,          'DESCEND'),
    'a':     ( 0,  LINEAR_SPEED,  0,  0,          'STRAFE LEFT'),
    'LEFT':  ( 0,  LINEAR_SPEED,  0,  0,          'STRAFE LEFT'),
    'd':     ( 0, -LINEAR_SPEED,  0,  0,          'STRAFE RIGHT'),
    'RIGHT': ( 0, -LINEAR_SPEED,  0,  0,          'STRAFE RIGHT'),
    'i':     ( LINEAR_SPEED,  0,  0,  0,          'FORWARD'),
    'k':     (-LINEAR_SPEED,  0,  0,  0,          'BACKWARD'),
    'j':     ( 0,  0,  0,  YAW_SPEED,             'YAW LEFT'),
    'l':     ( 0,  0,  0, -YAW_SPEED,             'YAW RIGHT'),
    'x':     ( 0,  0,  0,  0,                     'STOP'),
}

BANNER = """
╔══════════════════════════════════════════════════════════════╗
║          PX4 DRONE — KEYBOARD CONTROLLER  v2                 ║
╠══════════════════════════════════════════════════════════════╣
║  SPACE       — ARM/DISARM (toggle, debounced)                ║
║  W / ↑       — Climb (hold)   S / ↓   — Descend (hold)      ║
║  I           — Forward (hold) K       — Backward (hold)      ║
║  A / ←       — Left (hold)    D / →   — Right (hold)         ║
║  J           — Yaw left       L       — Yaw right            ║
║  X           — STOP all       Q/Ctrl-C — Quit                 ║
╚══════════════════════════════════════════════════════════════╝
"""


# ─── MAIN ────────────────────────────────────────────────────────
def main():
    print(BANNER)
    print("  Make sure gz sim is running and ▶ Play is pressed.\n")
    print("  Press SPACE to ARM, then HOLD movement keys to fly.\n")

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    sender = CmdSender()
    armed  = False
    last_arm_time = 0.0
    prev_key = ''

    try:
        tty.setraw(fd)
        while True:
            key = get_key_raw(fd)

            # ── QUIT ──────────────────────────────────────────
            if key in ('q', 'Q', '\x03'):
                break

            # ── ARM TOGGLE (debounced) ────────────────────────
            if key == ' ':
                now = time.monotonic()
                if now - last_arm_time > ARM_DEBOUNCE:
                    last_arm_time = now
                    armed = not armed
                    gz_enable(armed)
                    sender.set_armed(armed)
                    if not armed:
                        sender.set_cmd(0, 0, 0, 0)
                continue

            if not armed:
                if key and key != prev_key and key != '':
                    # Only show warning once per unique non-empty key
                    pass  # silently ignore
                prev_key = key
                continue

            # ── MOVEMENT ─────────────────────────────────────
            if key in KEY_MAP:
                lx, ly, lz, az, label = KEY_MAP[key]
                sender.set_cmd(lx, ly, lz, az)
                vstr = f"vx={lx:+.1f} vy={ly:+.1f} vz={lz:+.1f} wz={az:+.1f}"
                print(f"\r  [{label:12s}]  {vstr}    ", end='', flush=True)
            elif key == '':
                # No key pressed — stop movement
                sender.set_cmd(0, 0, 0, 0)

            prev_key = key

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n\n  [Quit] Stopping drone...")
        sender.set_cmd(0, 0, 0, 0)
        time.sleep(0.1)
        gz_pub_twist()
        gz_enable(False)
        sender.stop()
        print("  Done.")


if __name__ == "__main__":
    main()
