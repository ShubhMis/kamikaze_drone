#!/usr/bin/env python3
"""
intercept_manual.py — Kalman-Filter Click-to-Intercept
══════════════════════════════════════════════════════════════════
Click any target in the FOV → drone steers and dives at it.

Tracking:
  Kalman Filter (4-state: x, y, vx, vy in pixel space) fused with
  OpenCV CSRT visual tracker.  When CSRT loses the target, the KF
  continues predicting for up to PREDICT_TIMEOUT seconds so the
  drone keeps flying toward the last known trajectory instead of
  aborting.

Guidance:
  • Yaw:   direct proportional  az = -K * yaw_err_rad  (from pixelToAngle)
  • Pitch: PID on pitch_err_rad drives lz (descend toward target)
  • Forward: constant CRUISE_SPEED while tracking

Reference:
  Dhanur_New/utils/calculations.py  — pixelToAngle, PID
  PN_simulation_3D.py               — proportional navigation concept

Usage:
  python3 kamikaze/intercept_manual.py --alt 30 --speed 4.0
══════════════════════════════════════════════════════════════════
"""

import argparse
import base64
import csv
import json
import math
import os
import subprocess
import sys
import threading
import time


import cv2
import numpy as np


# ═══════════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(description="Kalman-Filter Click-to-Intercept")
    p.add_argument("--alt",   type=float, default=50.0,
                   help="Mission altitude metres (default 50)")
    p.add_argument("--speed", type=float, default=4.0,
                   help="Intercept forward speed m/s (default 4.0)")
    p.add_argument("--drone", type=str,   default="drone")
    p.add_argument("--world", type=str,   default="px4_baylands_world")
    p.add_argument("--log",   action="store_true",
                   help="Write CSV flight log for PID tuning")
    return p.parse_args()


# ═══════════════════════════════════════════════════════════════
#  CAMERA GEOMETRY  (matches Gazebo SDF camera)
# ═══════════════════════════════════════════════════════════════

IMG_W    = 1280
IMG_H    = 960
HFOV_RAD = 1.74
FX       = IMG_W / (2.0 * math.tan(HFOV_RAD / 2.0))
FY       = FX
CX, CY   = IMG_W / 2.0, IMG_H / 2.0


# ═══════════════════════════════════════════════════════════════
#  PID  (identical to Dhanur_New/utils/calculations.py)
# ═══════════════════════════════════════════════════════════════

class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, i_lim=10.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.I = 0.0
        self.last_e = 0.0
        self.last_t = time.time()
        self.i_lim = i_lim
        # Expose last P/D terms for logging
        self.last_P = 0.0
        self.last_D = 0.0

    def compute(self, e):
        t = time.time()
        dt = max(1e-6, t - self.last_t)
        P = self.kp * e
        self.I += e * dt
        self.I = max(-self.i_lim, min(self.I, self.i_lim))
        D = self.kd * ((e - self.last_e) / dt)
        out = P + self.ki * self.I + D
        self.last_e = e
        self.last_t = t
        self.last_P = P
        self.last_D = D
        return out

    def reset(self):
        self.I = 0.0
        self.last_e = 0.0
        self.last_t = time.time()
        self.last_P = 0.0
        self.last_D = 0.0


# ═══════════════════════════════════════════════════════════════
#  KALMAN FILTER & TRACKING REMOVED
# ═══════════════════════════════════════════════════════════════


# ═══════════════════════════════════════════════════════════════
#  MATHS  (pixelToAngle matches Dhanur_New exactly)
# ═══════════════════════════════════════════════════════════════

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def normalize_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def quat_to_yaw(x, y, z, w):
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def pixel_to_angle(px, py):
    """Return (yaw_err_rad, pitch_err_rad) from frame centre.
    Identical to Dhanur_New pixelToAngle but in radians."""
    dx = px - CX
    dy = py - CY
    return (math.atan2(dx, FX), math.atan2(dy, FY))


# ═══════════════════════════════════════════════════════════════
#  GAZEBO TRANSPORT  (native → subprocess fallback)
# ═══════════════════════════════════════════════════════════════

def _gz_one(topic, timeout=2.0):
    while True:
        try:
            r = subprocess.run(
                ["gz","topic","-e","--json-output","-n","1","-t", topic],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                timeout=timeout)
            if r.stdout:
                yield r.stdout
        except subprocess.TimeoutExpired:
            pass
        except Exception:
            time.sleep(0.05)


def build_transport(world, drone):
    CAMERA_TOPIC = (f"/world/{world}/model/{drone}"
                    f"/link/camera_link/sensor/camera/image")
    POSE_TOPIC   = f"/world/{world}/dynamic_pose/info"

    # try native gz.transport
    for tr_mod, ms_mod, ps_mod in [
        ("gz.transport13", "gz.msgs10.image_pb2", "gz.msgs10.pose_v_pb2"),
        ("gz.transport12", "gz.msgs9.image_pb2",  "gz.msgs9.pose_v_pb2"),
    ]:
        try:
            import importlib
            tr = importlib.import_module(tr_mod)
            ms = importlib.import_module(ms_mod)
            ps = importlib.import_module(ps_mod)
            GzNode, GzImage, Pose_V = tr.Node, ms.Image, ps.Pose_V

            class _CamNative:
                def __init__(self):
                    self._s = [None]; self._l = threading.Lock()
                    n = GzNode(); n.subscribe(GzImage, CAMERA_TOPIC, self._cb)
                def _cb(self, m):
                    w,h = m.width, m.height
                    arr = np.frombuffer(m.data, dtype=np.uint8)
                    if arr.size < w*h*3: return
                    bgr = cv2.cvtColor(arr[:w*h*3].reshape(h,w,3), cv2.COLOR_RGB2BGR)
                    with self._l: self._s[0] = bgr
                def get_frame(self):
                    with self._l: f=self._s[0]; return f.copy() if f is not None else None
                def stop(self): pass

            class _PoseNative:
                def __init__(self):
                    self._l = threading.Lock()
                    self._dp = None; self._yaw = 0.0; self._rdy = False
                    n = GzNode(); n.subscribe(Pose_V, POSE_TOPIC, self._cb)
                def _cb(self, m):
                    with self._l:
                        for p in m.pose:
                            if p.name == drone:
                                po = p.position; o = p.orientation
                                self._dp  = (po.x, po.y, po.z)
                                self._yaw = quat_to_yaw(o.x,o.y,o.z,o.w)
                                self._rdy = True
                def ready(self):
                    with self._l: return self._rdy
                def drone(self):
                    with self._l: return self._dp
                def yaw(self):
                    with self._l: return self._yaw
                def stop(self): pass

            cam  = _CamNative()
            pose = _PoseNative()
            print("[Cam]  gz.transport native ✓")
            print("[Pose] gz.transport native ✓")
            return cam, pose

        except Exception:
            continue

    # subprocess fallback
    print("[WARN] gz.transport not found — subprocess fallback")

    class _CamSub:
        def __init__(self):
            self._raw=[None]; self._rl=threading.Lock()
            self._ev=threading.Event()
            self._s=[None];  self._fl=threading.Lock()
            self._stop=False
            threading.Thread(target=self._stream, daemon=True).start()
            threading.Thread(target=self._decode, daemon=True).start()
            print("[Cam]  subprocess fallback")
        def _stream(self):
            for raw in _gz_one(CAMERA_TOPIC):
                if self._stop: break
                with self._rl: self._raw[0]=raw
                self._ev.set()
        def _decode(self):
            while not self._stop:
                if not self._ev.wait(0.5): continue
                with self._rl: raw=self._raw[0]; self._raw[0]=None; self._ev.clear()
                if raw is None: continue
                try:
                    m=json.loads(raw)
                    w,h=int(m.get("width",0)),int(m.get("height",0))
                    d=m.get("data","")
                    if not w or not h or not d: continue
                    pix=base64.b64decode(d)
                    if len(pix)<w*h*3: continue
                    arr=np.frombuffer(pix[:w*h*3],dtype=np.uint8).reshape(h,w,3)
                    bgr=cv2.cvtColor(arr,cv2.COLOR_RGB2BGR)
                    with self._fl: self._s[0]=bgr
                except Exception: pass
        def get_frame(self):
            with self._fl: f=self._s[0]; return f.copy() if f is not None else None
        def stop(self): self._stop=True

    class _PoseSub:
        def __init__(self):
            self._l=threading.Lock()
            self._dp=None; self._yaw=0.0; self._rdy=False
            self._stop=False
            threading.Thread(target=self._run, daemon=True).start()
            print("[Pose] subprocess fallback")
        def _run(self):
            for raw in _gz_one(POSE_TOPIC):
                if self._stop: break
                try:
                    m=json.loads(raw)
                    with self._l:
                        for p in m.get("pose",[]):
                            if p.get("name")==drone:
                                pos=p.get("position",{})
                                ori=p.get("orientation",{})
                                self._dp=(float(pos.get("x",0)),
                                          float(pos.get("y",0)),
                                          float(pos.get("z",0)))
                                self._yaw=quat_to_yaw(
                                    float(ori.get("x",0)),float(ori.get("y",0)),
                                    float(ori.get("z",0)),float(ori.get("w",1)))
                                self._rdy=True
                except Exception: pass
        def ready(self):
            with self._l: return self._rdy
        def drone(self):
            with self._l: return self._dp
        def yaw(self):
            with self._l: return self._yaw
        def stop(self): self._stop=True

    return _CamSub(), _PoseSub()


# ═══════════════════════════════════════════════════════════════
#  GAZEBO COMMANDS
# ═══════════════════════════════════════════════════════════════

def gz_twist(drone, lx=0.0, ly=0.0, lz=0.0, az=0.0):
    msg = (f"linear: {{x:{lx:.4f},y:{ly:.4f},z:{lz:.4f}}},"
           f"angular: {{x:0,y:0,z:{az:.4f}}}")
    subprocess.Popen(
        ["gz","topic","-t",f"/{drone}/cmd_vel",
         "-m","gz.msgs.Twist","-p",msg],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def gz_enable(drone, on):
    subprocess.Popen(
        ["gz","topic","-t",f"/{drone}/enable",
         "-m","gz.msgs.Boolean",
         "-p",f"data: {'true' if on else 'false'}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ═══════════════════════════════════════════════════════════════
#  HUD
# ═══════════════════════════════════════════════════════════════

FT   = cv2.FONT_HERSHEY_DUPLEX
FTS  = cv2.FONT_HERSHEY_SIMPLEX
AA   = cv2.LINE_AA
C_W  = (255,255,255)
C_Y  = (  0,220,255)
C_G  = (  0,200, 80)
C_R  = (  0,  0,220)
C_O  = (  0,165,255)
C_D  = (160,160,160)
C_B  = (255,100,  0)   # blue for KF prediction


def draw_hud(frame, dp, yaw_rad, phase, kf_pos,
             mission_alt, speed, target_loc=None):
    out = frame.copy()
    h, w = out.shape[:2]
    cx, cy = int(CX), int(CY)

    # crosshair at centre
    cv2.line(out, (cx-20, cy), (cx+20, cy), C_W, 1, AA)
    cv2.line(out, (cx, cy-20), (cx, cy+20), C_W, 1, AA)

    # semi-transparent top bar
    ov = out.copy()
    cv2.rectangle(ov, (0,0), (w, 80), (0,0,0), -1)
    cv2.addWeighted(ov, 0.55, out, 0.45, 0, out)

    # global position
    x_w = dp[0] if dp else 0.0
    y_w = dp[1] if dp else 0.0
    z_w = dp[2] if dp else 0.0
    cv2.putText(out, "DRONE POSITION", (12, 26), FT, 0.70, C_D, 1, AA)
    pos_txt = f"X:{x_w:8.2f}m   Y:{y_w:8.2f}m   Z:{z_w:8.2f}m"
    cv2.putText(out, pos_txt, (12, 60), FT, 0.90, C_W, 2, AA)

    if target_loc is not None:
        tx_g, ty_g = target_loc
        cv2.putText(out, "TARGET EST (GROUND)", (12, 100), FT, 0.70, C_D, 1, AA)
        tgt_txt = f"X:{tx_g:8.2f}m   Y:{ty_g:8.2f}m"
        cv2.putText(out, tgt_txt, (12, 134), FT, 0.90, C_R, 2, AA)

    # heading / alt / speed
    hdg_deg = (math.degrees(yaw_rad) % 360.0) if yaw_rad else 0.0
    course  = (90.0 - hdg_deg) % 360.0
    r1 = f"HDG: {hdg_deg:05.1f}   CRS: {course:05.1f}"
    r2 = f"ALT: {z_w:5.1f}m   SPD: {speed:.1f}m/s"
    tw1 = cv2.getTextSize(r1, FT, 0.75, 2)[0][0]
    tw2 = cv2.getTextSize(r2, FT, 0.75, 2)[0][0]
    cv2.putText(out, r1, (w - tw1 - 12, 30), FT, 0.75, C_Y, 2, AA)
    cv2.putText(out, r2, (w - tw2 - 12, 65), FT, 0.75, C_Y, 2, AA)

    # phase badge
    ph_col = {"IDLE": (120,120,120), "CLIMB": C_G, "TRACK": C_R,
              "MANUAL": C_O, "KF_PREDICT": C_B}.get(phase, C_D)
    cv2.putText(out, f"[ {phase} ]", (12, h - 50), FT, 1.0, ph_col, 2, AA)

    # Fixed click target visualization
    if phase in ("TRACK", "KF_PREDICT") and kf_pos is not None:
        kx, ky = int(kf_pos[0]), int(kf_pos[1])
        col = C_O 
        label = "TARGET PIXEL"
        cv2.circle(out, (kx, ky), 6, col, -1)
        cv2.line(out, (cx, cy), (kx, ky), col, 2, AA)
        cv2.putText(out, label, (kx + 10, ky - 10), FT, 0.55, col, 2, AA)

    # hint bar
    hint = "CLICK: intercept  |  RCLICK: hover  |  WASD/IJKL: manual  |  Q: quit"
    ov2 = out.copy()
    cv2.rectangle(ov2, (0, h-40), (w, h), (0,0,0), -1)
    cv2.addWeighted(ov2, 0.50, out, 0.50, 0, out)
    tw = cv2.getTextSize(hint, FTS, 0.45, 1)[0][0]
    cv2.putText(out, hint, (w//2 - tw//2, h - 14), FTS, 0.45, C_D, 1, AA)

    return out


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════

def main():
    args         = parse_args()
    MISSION_ALT  = args.alt
    CRUISE_SPEED = args.speed
    DRONE        = args.drone
    WORLD        = args.world

    # ── Tuning constants ─────────────────────────────────────
    ALT_KP, ALT_KD       = 1.2, 0.4
    PID_PITCH_KP          = 1.20     # ↑ from 0.60 — P was only 0.4 at 40° error
    PID_PITCH_KD          = 0.10     # ↓ from 0.35 — D was ±2.4, drowning P
    PID_YAW_KP            = 4.0      # ↑ from 3.5 — slightly more yaw authority
    PID_YAW_KD            = 0.4      # ↓ from 1.5 — D was ±6.4, way too aggressive
    YAW_RATE_MAX          = 2.5      # rad/s max yaw rate
    VZ_MAX                = 5.0      # ↑ from 4.0 — allow faster dive/climb
    PREDICT_TIMEOUT       = 2.0      # seconds of KF-only prediction before abort
    CSRT_BOX_SIZE         = 80       # initial CSRT bbox size (pixels)

    print("\n" + "═"*60)
    print("  INTERCEPT MANUAL — Kalman Filter Click-to-Intercept")
    print("═"*60)

    # ── Wait for Gazebo ──────────────────────────────────────
    print(f"\n  Waiting for Gazebo world '{WORLD}' …")
    for _ in range(60):
        r = subprocess.run(["gz","topic","-l"],
                           capture_output=True, text=True, timeout=5)
        if WORLD in r.stdout:
            print("  ✓ Gazebo ready")
            break
        time.sleep(1)
    else:
        print("  ✗ Gazebo not found. Run the sim first.")
        sys.exit(1)

    cam, poses = build_transport(WORLD, DRONE)

    # ── CSV Flight Logger ────────────────────────────────────
    csv_file = None
    csv_writer = None
    if args.log:
        import datetime
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(os.path.dirname(__file__), f"flight_log_{ts}.csv")
        csv_file = open(log_path, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            "t", "phase", "src",
            "px_x", "px_y",             # target pixel position
            "yaw_err_deg", "pitch_err_deg",  # raw errors before deadband
            "yaw_err_db", "pitch_err_db",    # after deadband
            "yaw_P", "yaw_D", "yaw_cmd",     # PID yaw internals
            "pitch_P", "pitch_D", "pitch_cmd", # PID pitch internals
            "az", "vz",                       # final commands
            "alt", "drone_x", "drone_y", "drone_yaw_deg",
            "tgt_X", "tgt_Y",                 # estimated ground target
        ])
        print(f"  ✎ Flight log → {log_path}")

    # ── PID objects ──────────────────────────────────────────
    pid_alt   = PID(kp=ALT_KP, kd=ALT_KD)
    pid_pitch = PID(kp=PID_PITCH_KP, kd=PID_PITCH_KD)
    pid_yaw   = PID(kp=PID_YAW_KP, kd=PID_YAW_KD)

    # ── State ────────────────────────────────────────────────
    phase        = "INIT"
    armed        = False
    kf_pos       = None          # (x, y) clicked target pixel
    current_frame = None
    hover_alt    = None
    manual_cmd   = {"lx": 0, "ly": 0, "lz": 0, "az": 0}
    last_key_time = 0

    # ── Keepalive ────────────────────────────────────────────
    stop_ev = threading.Event()
    def keepalive():
        while not stop_ev.is_set():
            if armed: gz_enable(DRONE, True)
            stop_ev.wait(0.25)
    threading.Thread(target=keepalive, daemon=True).start()

    # ── OpenCV window + mouse ────────────────────────────────
    WIN = "INTERCEPT_MANUAL"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 960)


    def on_mouse(event, x, y, flags, _):
        nonlocal phase, kf_pos, hover_alt

        if event == cv2.EVENT_LBUTTONDOWN:
            if not poses.ready() or current_frame is None:
                return

            # Store the clicked pixel position permanently
            kf_pos = (float(x), float(y))

            pid_yaw.reset()
            pid_pitch.reset()
            pid_alt.reset()
            phase = "TRACK"
            hover_alt = None
            print(f"\n  [CLICK] Target locked at ({x}, {y})")

        elif event == cv2.EVENT_RBUTTONDOWN:
            kf_pos = None
            if phase in ("TRACK", "KF_PREDICT", "MANUAL"):
                phase = "IDLE"
                hover_alt = None
            print("\n  [RIGHT-CLICK] → HOVER")

    # Register mouse callback — retry until Qt window handle is live
    # Flush event loop
    cv2.waitKey(10)

    cv2.setMouseCallback(WIN, on_mouse)


    print(f"\n  Drone      : {DRONE}")
    print(f"  Altitude   : {MISSION_ALT} m   (--alt)")
    print(f"  Fwd speed  : {CRUISE_SPEED} m/s (--speed)")
    print(f"\n  LEFT-CLICK  → lock target")
    print(f"  RIGHT-CLICK → hover")
    print(f"  WASD/IJKL   → manual override")
    print(f"  Q / ESC     → quit\n")

    CONTROL_HZ = 20
    dt_target  = 1.0 / CONTROL_HZ
    prev_t     = time.monotonic()

    try:
        while True:
            t0     = time.monotonic()
            dt_act = max(0.001, t0 - prev_t)
            prev_t = t0

            dp  = poses.drone()
            dy  = poses.yaw()
            alt = dp[2] if dp else 0.0

            frame = cam.get_frame()
            current_frame = frame

            # ── INIT: wait for pose ─────────────────────────
            if phase == "INIT":
                if poses.ready() and dp:
                    print(f"  ✓ Pose: ({dp[0]:.1f}, {dp[1]:.1f}, {dp[2]:.1f})")
                    gz_enable(DRONE, True)
                    armed = True
                    time.sleep(0.3)
                    gz_twist(DRONE, lz=0.1)
                    pid_alt.reset()
                    phase = "CLIMB"
                    print(f"  → CLIMB to {MISSION_ALT}m")
                else:
                    if frame is not None:
                        hud = draw_hud(frame, dp, dy, "WAIT", None,
                                       MISSION_ALT, 0)
                        cv2.imshow(WIN, hud)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    time.sleep(0.1)
                    continue

            # ── CLIMB ───────────────────────────────────────
            if phase == "CLIMB":
                alt_err = MISSION_ALT - alt
                vz = clamp(pid_alt.compute(alt_err), -2.0, 2.5)
                if alt < 2.0: vz = max(vz, 2.5)

                desired_yaw = 0.0
                yaw_cmd = clamp(
                    1.2 * normalize_angle(desired_yaw - dy),
                    -0.8, 0.8) if dy else 0.0

                gz_twist(DRONE, lz=vz, az=yaw_cmd)
                print(f"\r  CLIMB  alt:{alt:.1f}/{MISSION_ALT:.0f}m  "
                      f"Δ{alt_err:+.1f}   ", end="", flush=True)

                if abs(alt_err) < 2.0 and alt > 4.0:
                    pid_alt.reset()
                    phase = "IDLE"
                    hover_alt = alt
                    print(f"\n  ✓ Altitude reached. Click to intercept.")

            # ── IDLE: hover at current alt ──────────────────
            elif phase == "IDLE":
                if hover_alt is None:
                    hover_alt = max(alt, 1.0)
                    pid_alt.reset()
                alt_err = hover_alt - alt
                vz = clamp(pid_alt.compute(alt_err), -1.5, 1.5)
                gz_twist(DRONE, lz=vz)

            # ── MANUAL: keyboard control ────────────────────
            elif phase == "MANUAL":
                if time.monotonic() - last_key_time < 0.25:
                    gz_twist(DRONE, **manual_cmd)
                    print(f"\r  MANUAL  lx:{manual_cmd['lx']:+.1f} "
                          f"ly:{manual_cmd['ly']:+.1f} "
                          f"lz:{manual_cmd['lz']:+.1f} "
                          f"az:{manual_cmd['az']:+.1f}  "
                          f"alt:{alt:.1f}m   ", end="", flush=True)
                else:
                    print("\n  [MANUAL] Key released → HOVER")
                    phase = "IDLE"
                    hover_alt = None
            # ── TRACK / KF_PREDICT ──────────────────────────
            elif phase in ("TRACK", "KF_PREDICT"):
                if kf_pos is None or frame is None:
                    phase = "IDLE"
                    hover_alt = None
                else:
                    est_x, est_y = kf_pos
                    # Step 5: Geolocalization (Raycast to Ground Z=0)
                    target_loc = None
                    if dp is not None and dy is not None:
                        # Ray in Camera frame
                        x_c = (est_x - CX) / FX
                        y_c = (est_y - CY) / FY
                        # Ray in Drone ENU frame (Forward=X, Left=Y, Up=Z)
                        r_x = 1.0
                        r_y = -x_c
                        r_z = -y_c
                        # Rotate Ray by Drone Yaw
                        cos_y = math.cos(dy)
                        sin_y = math.sin(dy)
                        R_x = r_x * cos_y - r_y * sin_y
                        R_y = r_x * sin_y + r_y * cos_y
                        R_z = r_z
                        # Intersect with ground (Z = 0)
                        if R_z < -1e-3:  # Looking downwards
                            S = -dp[2] / R_z
                            target_X = dp[0] + S * R_x
                            target_Y = dp[1] + S * R_y
                            target_loc = (target_X, target_Y)

                    # Step 6: Compute guidance
                    yaw_err_raw, pitch_err_raw = pixel_to_angle(est_x, est_y)
                    yaw_err_rad = yaw_err_raw
                    pitch_err_rad = pitch_err_raw

                    # Deadband logic for stability
                    if abs(math.degrees(yaw_err_rad)) < 2.0: yaw_err_rad = 0.0
                    if abs(math.degrees(pitch_err_rad)) < 2.0: pitch_err_rad = 0.0

                    # Yaw: PID drives az (damped rotation)
                    yaw_cmd = pid_yaw.compute(yaw_err_rad)
                    az = clamp(-yaw_cmd, -YAW_RATE_MAX, YAW_RATE_MAX)

                    # Pitch: PID drives vz (descend toward ground target, climb to air target)
                        pitch_cmd = pid_pitch.compute(pitch_err_rad)
                        vz = clamp(-pitch_cmd * 6.0, -VZ_MAX, VZ_MAX)

                        gz_twist(DRONE, lx=CRUISE_SPEED, ly=0.0, lz=vz, az=az)

                        src = "CLICK"
                        print(f"\r  {phase}  "
                              f"src:{src}  "
                              f"yaw:{math.degrees(yaw_err_raw):+.1f}°  "
                              f"pitch:{math.degrees(pitch_err_raw):+.1f}°  "
                              f"az:{az:+.3f}  vz:{vz:+.2f}  "
                              f"alt:{alt:.1f}m   ",
                              end="", flush=True)

                        # CSV logging
                        if csv_writer:
                            tgt_x = target_loc[0] if target_loc else ""
                            tgt_y = target_loc[1] if target_loc else ""
                            csv_writer.writerow([
                                f"{t0:.3f}", phase, src,
                                f"{est_x:.1f}", f"{est_y:.1f}",
                                f"{math.degrees(yaw_err_raw):.2f}",
                                f"{math.degrees(pitch_err_raw):.2f}",
                                f"{math.degrees(yaw_err_rad):.2f}",
                                f"{math.degrees(pitch_err_rad):.2f}",
                                f"{pid_yaw.last_P:.4f}", f"{pid_yaw.last_D:.4f}",
                                f"{yaw_cmd:.4f}",
                                f"{pid_pitch.last_P:.4f}", f"{pid_pitch.last_D:.4f}",
                                f"{pitch_cmd:.4f}",
                                f"{az:.4f}", f"{vz:.4f}",
                                f"{alt:.2f}",
                                f"{dp[0]:.2f}" if dp else "",
                                f"{dp[1]:.2f}" if dp else "",
                                f"{math.degrees(dy):.2f}" if dy else "",
                                tgt_x, tgt_y,
                            ])

            # ── Render ──────────────────────────────────────
            if frame is not None:
                spd = CRUISE_SPEED if phase in ("TRACK", "KF_PREDICT") else 0.0
                hud = draw_hud(frame, dp, dy, phase, kf_pos,
                               MISSION_ALT, spd, target_loc if phase in ("TRACK", "KF_PREDICT") else None)
                cv2.imshow(WIN, hud)

            # ── Keyboard ────────────────────────────────────
            key = cv2.waitKeyEx(1)
            if key != -1:
                k = key & 0xFFFF
                if k in (ord('q'), 27):
                    print("\n  [QUIT]")
                    break
                elif k == ord('c'):
                    kf_pos = None
                    if phase in ("TRACK", "KF_PREDICT", "MANUAL"):
                        phase = "IDLE"
                        hover_alt = None
                    print("\n  [C] cleared → HOVER")
                elif phase in ("IDLE", "MANUAL", "TRACK", "KF_PREDICT"):
                    moved = False
                    lx = ly = lz = az_k = 0.0

                    if   k == ord('i'): lx = CRUISE_SPEED; moved = True
                    elif k == ord('k'): lx = -CRUISE_SPEED; moved = True
                    elif k == ord('a'): ly = CRUISE_SPEED; moved = True
                    elif k == ord('d'): ly = -CRUISE_SPEED; moved = True
                    elif k in (ord('w'), 65362): lz = VZ_MAX; moved = True
                    elif k in (ord('s'), 65364): lz = -VZ_MAX; moved = True
                    elif k == ord('j'): az_k = YAW_RATE_MAX; moved = True
                    elif k == ord('l'): az_k = -YAW_RATE_MAX; moved = True

                    if moved:
                        kf_pos = None
                        phase = "MANUAL"
                        manual_cmd = {"lx": lx, "ly": ly, "lz": lz, "az": az_k}
                        last_key_time = time.monotonic()

            # ── Pace ────────────────────────────────────────
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, dt_target - elapsed))

    except KeyboardInterrupt:
        print("\n  [Ctrl-C] abort")

    finally:
        print("  Stopping …")
        for _ in range(6):
            gz_twist(DRONE)
            time.sleep(0.04)
        gz_enable(DRONE, False)
        stop_ev.set()
        cam.stop(); poses.stop()
        cv2.destroyAllWindows()
        if csv_file:
            csv_file.close()
            print(f"  ✎ Log saved.")
        print("  ✓ Done.\n")


if __name__ == "__main__":
    main()
