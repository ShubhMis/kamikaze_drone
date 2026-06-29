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
import json
import math
import os
import subprocess
import sys
import threading
import time

# Force X11 backend — prevents Qt NULL window handle on Wayland/XWayland
os.environ["QT_QPA_PLATFORM"] = "xcb"

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
        return out

    def reset(self):
        self.I = 0.0
        self.last_e = 0.0
        self.last_t = time.time()


# ═══════════════════════════════════════════════════════════════
#  KALMAN FILTER  (4-state pixel tracker)
# ═══════════════════════════════════════════════════════════════

class PixelKalmanFilter:
    """
    State:  [x, y, vx, vy]  in pixel coordinates.
    Measurement: [x, y]  from CSRT or click.

    This filter smooths the tracking and predicts forward when
    the visual tracker temporarily loses the target, preventing
    the drone from aborting and flying off randomly.
    """

    def __init__(self, x0, y0, process_noise=5.0, meas_noise=3.0):
        self.kf = cv2.KalmanFilter(4, 2)

        # State: [x, y, vx, vy]
        self.kf.statePost = np.array([[x0], [y0], [0.0], [0.0]], dtype=np.float32)

        # Transition matrix (constant velocity model)
        dt = 1.0 / 20.0  # ~20 Hz control loop
        self.kf.transitionMatrix = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1],
        ], dtype=np.float32)

        # Measurement matrix: we observe x, y
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float32)

        # Process noise covariance
        q = process_noise
        self.kf.processNoiseCov = np.array([
            [q,   0,   0,   0],
            [0,   q,   0,   0],
            [0,   0, q*4,   0],
            [0,   0,   0, q*4],
        ], dtype=np.float32)

        # Measurement noise covariance
        r = meas_noise
        self.kf.measurementNoiseCov = np.array([
            [r, 0],
            [0, r],
        ], dtype=np.float32)

        # Error covariance
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 10.0

    def predict(self):
        """Predict next state. Returns (x, y)."""
        pred = self.kf.predict()
        return float(pred[0, 0]), float(pred[1, 0])

    def correct(self, mx, my):
        """Update with measurement. Returns corrected (x, y)."""
        meas = np.array([[mx], [my]], dtype=np.float32)
        corrected = self.kf.correct(meas)
        return float(corrected[0, 0]), float(corrected[1, 0])

    def get_position(self):
        """Current estimated position."""
        s = self.kf.statePost
        return float(s[0, 0]), float(s[1, 0])

    def get_velocity(self):
        """Current estimated pixel velocity."""
        s = self.kf.statePost
        return float(s[2, 0]), float(s[3, 0])


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


def draw_hud(frame, dp, yaw_rad, phase, kf_pos, kf_bbox,
             mission_alt, speed, csrt_ok):
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
    cv2.putText(out, "GLOBAL POSITION", (12, 26), FT, 0.70, C_D, 1, AA)
    pos_txt = f"X:{x_w:8.2f}m   Y:{y_w:8.2f}m   Z:{z_w:8.2f}m"
    cv2.putText(out, pos_txt, (12, 60), FT, 0.90, C_W, 2, AA)

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

    # KF / tracker visualization
    if phase in ("TRACK", "KF_PREDICT") and kf_pos is not None:
        kx, ky = int(kf_pos[0]), int(kf_pos[1])
        # KF estimated centre
        col = C_O if csrt_ok else C_B
        label = "TRACKING" if csrt_ok else "KF PREDICT"
        cv2.circle(out, (kx, ky), 6, col, -1)
        cv2.line(out, (cx, cy), (kx, ky), col, 2, AA)
        cv2.putText(out, label, (kx + 10, ky - 10), FT, 0.55, col, 2, AA)

    # CSRT bbox
    if kf_bbox is not None and csrt_ok:
        tx, ty, tw_b, th_b = kf_bbox
        cv2.rectangle(out, (tx, ty), (tx+tw_b, ty+th_b), C_O, 2, AA)

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
    PID_PITCH_KP          = 0.20
    PID_PITCH_KD          = 0.05
    YAW_RATE_MAX          = 1.5      # rad/s
    VZ_MAX                = 3.0      # m/s
    YAW_KP_DIRECT         = 2.5      # rad/s per rad error
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

    # ── PID objects ──────────────────────────────────────────
    pid_alt   = PID(kp=ALT_KP, kd=ALT_KD)
    pid_pitch = PID(kp=PID_PITCH_KP, kd=PID_PITCH_KD)

    # ── State ────────────────────────────────────────────────
    phase        = "INIT"
    armed        = False
    tracker      = None          # CSRT tracker
    kf           = None          # Kalman filter
    csrt_ok      = False         # was CSRT successful this frame?
    last_csrt_ok = 0.0           # timestamp of last successful CSRT update
    kf_pos       = None          # (x, y) from KF
    kf_bbox      = None          # (x, y, w, h) last known CSRT bbox
    current_frame = None
    hover_alt    = None
    manual_cmd   = {"lx": 0, "ly": 0, "lz": 0, "az": 0}
    last_key_time = 0
    mouse_cb_set = False

    # ── Keepalive ────────────────────────────────────────────
    stop_ev = threading.Event()
    def keepalive():
        while not stop_ev.is_set():
            if armed: gz_enable(DRONE, True)
            stop_ev.wait(0.25)
    threading.Thread(target=keepalive, daemon=True).start()

    # ── OpenCV window + mouse ────────────────────────────────
    WIN = "INTERCEPT — Kalman Filter Tracker"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 960)
    # Pump Qt event loop until handle is confirmed live
    blank = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
    cv2.imshow(WIN, blank)
    for _ in range(10):
        cv2.waitKey(1)
    time.sleep(0.2)
    for _ in range(5):
        cv2.waitKey(1)

    def on_mouse(event, x, y, flags, _):
        nonlocal phase, tracker, kf, csrt_ok, last_csrt_ok
        nonlocal kf_pos, kf_bbox, hover_alt

        if event == cv2.EVENT_LBUTTONDOWN:
            if not poses.ready() or current_frame is None:
                return

            # Init CSRT around click
            bsz = CSRT_BOX_SIZE
            x1 = max(0, x - bsz // 2)
            y1 = max(0, y - bsz // 2)
            w_b = min(IMG_W - x1, bsz)
            h_b = min(IMG_H - y1, bsz)
            bbox = (x1, y1, w_b, h_b)

            tracker = cv2.TrackerCSRT_create()
            tracker.init(current_frame, bbox)

            # Init Kalman Filter at click position
            kf = PixelKalmanFilter(float(x), float(y))
            kf_pos = (float(x), float(y))
            kf_bbox = (x1, y1, w_b, h_b)
            csrt_ok = True
            last_csrt_ok = time.monotonic()

            pid_pitch.reset()
            pid_alt.reset()
            phase = "TRACK"
            hover_alt = None
            print(f"\n  [CLICK] Target locked at ({x}, {y})")

        elif event == cv2.EVENT_RBUTTONDOWN:
            tracker = None
            kf = None
            kf_pos = None
            kf_bbox = None
            csrt_ok = False
            if phase in ("TRACK", "KF_PREDICT", "MANUAL"):
                phase = "IDLE"
                hover_alt = None
            print("\n  [RIGHT-CLICK] → HOVER")

    # Mouse callback will be set on the first rendered frame


    print(f"\n  Drone      : {DRONE}")
    print(f"  Altitude   : {MISSION_ALT} m   (--alt)")
    print(f"  Fwd speed  : {CRUISE_SPEED} m/s (--speed)")
    print(f"\n  LEFT-CLICK  → lock target (Kalman + CSRT)")
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
                        hud = draw_hud(frame, dp, dy, "WAIT", None, None,
                                       MISSION_ALT, 0, False)
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
                if kf is None or frame is None:
                    phase = "IDLE"
                    hover_alt = None
                else:
                    # Step 1: KF predict
                    pred_x, pred_y = kf.predict()

                    # Step 2: Try CSRT update
                    csrt_ok = False
                    if tracker is not None:
                        ok, bbox = tracker.update(frame)
                        if ok:
                            bx, by, bw, bh = bbox
                            meas_x = bx + bw / 2.0
                            meas_y = by + bh / 2.0

                            # Only accept if measurement is within frame
                            if 0 <= meas_x <= IMG_W and 0 <= meas_y <= IMG_H:
                                kf.correct(meas_x, meas_y)
                                kf_bbox = (int(bx), int(by), int(bw), int(bh))
                                csrt_ok = True
                                last_csrt_ok = time.monotonic()

                    # Step 3: Get KF position (corrected if CSRT ok, predicted if not)
                    est_x, est_y = kf.get_position()

                    # Clamp to frame bounds
                    est_x = clamp(est_x, 0, IMG_W)
                    est_y = clamp(est_y, 0, IMG_H)
                    kf_pos = (est_x, est_y)

                    # Step 4: Check timeout
                    time_since_csrt = time.monotonic() - last_csrt_ok
                    if not csrt_ok and time_since_csrt > PREDICT_TIMEOUT:
                        print(f"\n  [KF] Prediction timeout ({PREDICT_TIMEOUT}s) → HOVER")
                        tracker = None
                        kf = None
                        kf_pos = None
                        kf_bbox = None
                        phase = "IDLE"
                        hover_alt = None
                    else:
                        # Update phase label
                        phase = "TRACK" if csrt_ok else "KF_PREDICT"

                        # Step 5: Compute guidance
                        yaw_err_rad, pitch_err_rad = pixel_to_angle(est_x, est_y)

                        # Yaw: direct proportional
                        az = clamp(-YAW_KP_DIRECT * yaw_err_rad,
                                   -YAW_RATE_MAX, YAW_RATE_MAX)

                        # Pitch: PID drives vz (descend toward ground target)
                        pitch_cmd = pid_pitch.compute(pitch_err_rad)
                        vz = clamp(-pitch_cmd * 3.5, -VZ_MAX, VZ_MAX)

                        gz_twist(DRONE, lx=CRUISE_SPEED, ly=0.0, lz=vz, az=az)

                        src = "CSRT" if csrt_ok else f"KF({time_since_csrt:.1f}s)"
                        print(f"\r  {phase}  "
                              f"src:{src}  "
                              f"yaw:{math.degrees(yaw_err_rad):+.1f}°  "
                              f"pitch:{math.degrees(pitch_err_rad):+.1f}°  "
                              f"az:{az:+.3f}  vz:{vz:+.2f}  "
                              f"alt:{alt:.1f}m   ",
                              end="", flush=True)

                        # Re-init CSRT from KF prediction if we lost it
                        # but KF is still within frame and timeout hasn't hit
                        if not csrt_ok and time_since_csrt > 0.5:
                            rx = int(clamp(est_x - CSRT_BOX_SIZE/2, 0, IMG_W - CSRT_BOX_SIZE))
                            ry = int(clamp(est_y - CSRT_BOX_SIZE/2, 0, IMG_H - CSRT_BOX_SIZE))
                            new_bbox = (rx, ry, CSRT_BOX_SIZE, CSRT_BOX_SIZE)
                            tracker = cv2.TrackerCSRT_create()
                            tracker.init(frame, new_bbox)
                            kf_bbox = new_bbox

            # ── Render ──────────────────────────────────────
            if frame is not None:
                spd = CRUISE_SPEED if phase in ("TRACK", "KF_PREDICT") else 0.0
                hud = draw_hud(frame, dp, dy, phase, kf_pos, kf_bbox,
                               MISSION_ALT, spd, csrt_ok)
                cv2.imshow(WIN, hud)
                
                if not mouse_cb_set:
                    try:
                        cv2.setMouseCallback(WIN, on_mouse)
                        mouse_cb_set = True
                    except cv2.error:
                        pass  # Window not fully mapped by Wayland yet; try again next frame

            # ── Keyboard ────────────────────────────────────
            key = cv2.waitKeyEx(1)
            if key != -1:
                k = key & 0xFFFF
                if k in (ord('q'), 27):
                    print("\n  [QUIT]")
                    break
                elif k == ord('c'):
                    tracker = kf = None
                    kf_pos = kf_bbox = None
                    csrt_ok = False
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
                        tracker = kf = None
                        kf_pos = kf_bbox = None
                        csrt_ok = False
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
        print("  ✓ Done.\n")


if __name__ == "__main__":
    main()
