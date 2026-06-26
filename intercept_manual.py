#!/usr/bin/env python3
"""
intercept_manual.py — Click-to-Intercept
══════════════════════════════════════════════════════════════════
Click any pixel in the FOV window → drone steers toward that point.
No blob detection. No tracker. No search phase.

How it works:
  1. ARM + CLIMB to --alt metres.
  2. Show live camera feed. Drone hovers.
  3. LEFT-CLICK  → set intercept pixel. Drone PID-steers toward it.
  4. RIGHT-CLICK → clear target. Drone hovers in place.
  5. 'q' → quit.

PID:
  yaw_err  (pixel_x - cx) → pid_yaw  → gz angular.z (turn)
  pitch_err(pixel_y - cy) → pid_pitch → gz linear.z  (climb/sink)
  Forward speed is constant while a target pixel is set.

HUD (simple):
  • Global position (X / Y / Z) large and clear — top-left
  • Heading, altitude, speed — top-right
  • Crosshair at frame centre
  • Yellow circle at clicked pixel + line from crosshair
  • Yaw error / pitch error to clicked point

Usage:
  python3 kamikaze/intercept_manual.py --alt 50 --speed 4.0
  python3 kamikaze/intercept_manual.py --alt 30
══════════════════════════════════════════════════════════════════
"""

import argparse
import base64
import json
import math
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
    p = argparse.ArgumentParser(description="Click-to-Intercept")
    p.add_argument("--alt",   type=float, default=50.0,
                   help="Mission altitude metres (default 50)")
    p.add_argument("--speed", type=float, default=4.0,
                   help="Intercept forward speed m/s (default 4.0)")
    p.add_argument("--drone", type=str,   default="drone")
    p.add_argument("--world", type=str,   default="px4_baylands_world")
    return p.parse_args()


# ═══════════════════════════════════════════════════════════════
#  CAMERA GEOMETRY
# ═══════════════════════════════════════════════════════════════

IMG_W    = 1280
IMG_H    = 960
HFOV_RAD = 1.74
FX       = IMG_W / (2.0 * math.tan(HFOV_RAD / 2.0))
FY       = FX
CX, CY   = IMG_W / 2.0, IMG_H / 2.0


# ═══════════════════════════════════════════════════════════════
#  PID
# ═══════════════════════════════════════════════════════════════

class PID:
    def __init__(self, kp, kd=0.0, ki=0.0, i_lim=8.0):
        self.kp, self.kd, self.ki = kp, kd, ki
        self.I = self.last_e = 0.0
        self.last_t = time.time()
        self.i_lim  = i_lim

    def compute(self, e):
        now = time.time()
        dt  = max(1e-4, now - self.last_t)
        P   = self.kp * e
        self.I = max(-self.i_lim, min(self.i_lim, self.I + e * dt))
        D   = self.kd * (e - self.last_e) / dt
        self.last_e, self.last_t = e, now
        return P + self.ki * self.I + D

    def reset(self):
        self.I = self.last_e = 0.0
        self.last_t = time.time()


# ═══════════════════════════════════════════════════════════════
#  MATHS
# ═══════════════════════════════════════════════════════════════

def clamp(v, lo, hi): return max(lo, min(hi, v))

def normalize_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def quat_to_yaw(x, y, z, w):
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def world_to_body(vx, vy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*vx + s*vy, -s*vx + c*vy

def pixel_to_angle(px, py):
    """Return (yaw_offset_rad, pitch_offset_rad) from frame centre."""
    return (math.atan2(px - CX, FX),
            math.atan2(py - CY, FY))


# ═══════════════════════════════════════════════════════════════
#  GAZEBO TRANSPORT  (native → subprocess fallback)
# ═══════════════════════════════════════════════════════════════

def _gz_one(topic, timeout=2.0):
    """Yield raw bytes of one Gazebo message via subprocess."""
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

    # ── try native gz.transport ───────────────────────────────
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

    # ── subprocess fallback ───────────────────────────────────
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
#  SIMPLE HUD DRAW
# ═══════════════════════════════════════════════════════════════

FT   = cv2.FONT_HERSHEY_DUPLEX
FTS  = cv2.FONT_HERSHEY_SIMPLEX
AA   = cv2.LINE_AA
C_W  = (255,255,255)
C_Y  = (  0,220,255)   # yellow-cyan  (headings)
C_G  = (  0,200, 80)   # green  (ok)
C_R  = (  0,  0,220)   # red    (active)
C_O  = (  0,165,255)   # orange (click target)
C_D  = (160,160,160)   # dim


def draw_hud(frame, dp, yaw_rad, phase, tracked_bbox,
             mission_alt, speed):
    out = frame.copy()
    h, w = out.shape[:2]
    cx, cy = int(CX), int(CY)

    # ── semi-transparent top bar ──────────────────────────────
    ov = out.copy()
    cv2.rectangle(ov, (0,0), (w, 80), (0,0,0), -1)
    cv2.addWeighted(ov, 0.55, out, 0.45, 0, out)

    # ── global position — large, top-left ─────────────────────
    x_w = dp[0] if dp else 0.0
    y_w = dp[1] if dp else 0.0
    z_w = dp[2] if dp else 0.0
    cv2.putText(out, "GLOBAL POSITION", (12, 26),
                FT, 0.70, C_D, 1, AA)
    pos_txt = f"X:{x_w:8.2f}m   Y:{y_w:8.2f}m   Z:{z_w:8.2f}m"
    cv2.putText(out, pos_txt, (12, 60),
                FT, 0.90, C_W, 2, AA)

    # ── right side: heading / alt / speed ────────────────────
    hdg_deg = (math.degrees(yaw_rad) % 360.0) if yaw_rad else 0.0
    course   = (90.0 - hdg_deg) % 360.0
    r1 = f"HDG: {hdg_deg:05.1f}°   CRS: {course:05.1f}°"
    r2 = f"ALT: {z_w:5.1f}m / {mission_alt:.0f}m   SPD: {speed:.1f}m/s"
    tw1 = cv2.getTextSize(r1, FT, 0.75, 2)[0][0]
    tw2 = cv2.getTextSize(r2, FT, 0.75, 2)[0][0]
    cv2.putText(out, r1, (w - tw1 - 12, 30), FT, 0.75, C_Y, 2, AA)
    cv2.putText(out, r2, (w - tw2 - 12, 65), FT, 0.75, C_Y, 2, AA)

    # ── phase badge ───────────────────────────────────────────
    ph_col = {
        "IDLE":       (120,120,120),
        "CLIMB":      C_G,
        "INTERCEPT":  C_R,
    }.get(phase, C_D)
    cv2.putText(out, f"[ {phase} ]", (12, h - 14),
                FT, 1.0, ph_col, 2, AA)

    # ── if TRACK: show bounding box ───────────────────────────
    if phase == "TRACK" and tracked_bbox is not None:
        tx, ty, tw, th = tracked_bbox
        cv2.rectangle(out, (tx, ty), (tx+tw, ty+th), C_O, 2, AA)
        bcx, bcy = tx + tw//2, ty + th//2
        cv2.circle(out, (bcx, bcy), 4, C_O, -1)
        cv2.line(out, (cx, cy), (bcx, bcy), C_O, 1, AA)
        cv2.putText(out, "TRACKING", (tx, ty - 6),
                    FT, 0.60, C_O, 2, AA)

    # ── hint bar ──────────────────────────────────────────────
    hint = "LEFT-CLICK: set intercept point   RIGHT-CLICK: hover   Q: quit"
    ov2 = out.copy()
    cv2.rectangle(ov2, (0, h-40), (w, h), (0,0,0), -1)
    cv2.addWeighted(ov2, 0.50, out, 0.50, 0, out)
    tw = cv2.getTextSize(hint, FTS, 0.45, 1)[0][0]
    cv2.putText(out, hint, (w//2 - tw//2, h - 14),
                FTS, 0.45, C_D, 1, AA)

    return out


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════

def main():
    args      = parse_args()
    MISSION_ALT  = args.alt
    CRUISE_SPEED = args.speed
    DRONE        = args.drone
    WORLD        = args.world

    # PID gains
    ALT_KP,       ALT_KD       = 1.2,  0.4   # altitude hold
    YAW_RATE_MAX  = 1.5   # rad/s clamp on az
    VZ_MAX        = 2.5   # m/s clamp on vz
    # Direct proportional yaw gain (rad/s per rad of pixel error)
    # At 45° (0.785 rad) error: 2.5 * 0.785 = 1.96 → clamped to 1.5 rad/s
    # At  5° (0.087 rad) error: 2.5 * 0.087 = 0.22 rad/s  (gentle)
    YAW_KP_DIRECT = 2.5

    print("\n" + "═"*60)
    print("  INTERCEPT MANUAL — Click-to-Intercept")
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
    pid_alt   = PID(ALT_KP, ALT_KD)

    # ── State ────────────────────────────────────────────────
    phase      = "INIT"
    armed      = False
    tracker    = None
    tracked_bbox = None
    
    # We need access to 'frame' in the mouse callback for tracker init
    current_frame = None
    hover_alt = None
    manual_cmd = {"lx": 0, "ly": 0, "lz": 0, "az": 0}
    last_key_time = 0

    # ── Keepalive ────────────────────────────────────────────
    stop_ev = threading.Event()
    def keepalive():
        while not stop_ev.is_set():
            if armed: gz_enable(DRONE, True)
            stop_ev.wait(0.25)
    threading.Thread(target=keepalive, daemon=True).start()

    # ── OpenCV window + mouse ─────────────────────────────────
    WIN = "INTERCEPT MANUAL - Click to set target"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 960)
    cv2.waitKey(1)   # flush Qt event loop so window handle is valid

    def on_mouse(event, x, y, flags, _):
        nonlocal phase, tracker, tracked_bbox
        if event == cv2.EVENT_LBUTTONDOWN:
            if not poses.ready() or current_frame is None: return
            
            # Create a 60x60 bounding box around the click
            box_w, box_h = 60, 60
            x1 = max(0, x - box_w // 2)
            y1 = max(0, y - box_h // 2)
            w_box = min(IMG_W - x1, box_w)
            h_box = min(IMG_H - y1, box_h)
            bbox = (x1, y1, w_box, h_box)
            
            tracker = cv2.TrackerCSRT_create()
            tracker.init(current_frame, bbox)
            
            pid_yaw.reset(); pid_pitch.reset(); pid_alt.reset()
            phase = "TRACK"
            tracked_bbox = bbox
            print(f"\n  [CLICK] Tracker initialized at {bbox} | diving to target")
        
        elif event == cv2.EVENT_RBUTTONDOWN:
            tracker = None
            tracked_bbox = None
            if phase in ["TRACK", "MANUAL"]:
                phase = "IDLE"
                hover_alt = None
            print("\n  [RIGHT-CLICK] cleared → HOVER")

    cv2.setMouseCallback(WIN, on_mouse)

    print(f"\n  Drone      : {DRONE}")
    print(f"  Altitude   : {MISSION_ALT} m   (--alt)")
    print(f"  Fwd speed  : {CRUISE_SPEED} m/s (--speed)")
    print(f"\n  LEFT-CLICK  → intercept clicked point")
    print(f"  RIGHT-CLICK → hover")
    print(f"  WASD/IJKL   → manual override (OpenCV window must be focused)")
    print(f"  Q           → quit\n")

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

            # ── INIT: wait for pose ──────────────────────────
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
                        placeholder = draw_hud(
                            frame, dp, dy, "WAIT FOR POSE",
                            None, None, MISSION_ALT, 0)
                        cv2.imshow(WIN, placeholder)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    time.sleep(0.1)
                    continue

            # ── CLIMB ────────────────────────────────────────
            if phase == "CLIMB":
                alt_err = MISSION_ALT - alt
                vz = clamp(pid_alt.compute(alt_err), -2.0, 2.5)
                if alt < 2.0: vz = max(vz, 2.5)

                # Gentle yaw to face north (0°) while climbing
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
                    print(f"\n  ✓ Altitude reached. Click to intercept.")

            # ── IDLE: hover ──────────────────────────────────
            elif phase == "IDLE":
                if hover_alt is None:
                    hover_alt = max(alt, 1.0)
                    pid_alt.reset()
                alt_err = hover_alt - alt
                vz = clamp(pid_alt.compute(alt_err), -1.5, 1.5)
                gz_twist(DRONE, lz=vz)

            # ── MANUAL: keyboard control ─────────────────────
            elif phase == "MANUAL":
                if time.monotonic() - last_key_time < 0.2:
                    gz_twist(DRONE, **manual_cmd)
                    print(f"\r  MANUAL  lx:{manual_cmd['lx']:+.1f} ly:{manual_cmd['ly']:+.1f} lz:{manual_cmd['lz']:+.1f} az:{manual_cmd['az']:+.1f}  alt:{alt:.1f}m   ", end="", flush=True)
                else:
                    print("\n  [MANUAL] Key released → HOVER")
                    phase = "IDLE"
                    hover_alt = None

            # ── TRACK: pixel-align yaw and pitch ───────────────────────
            elif phase == "TRACK":
                if tracker is None or frame is None:
                    phase = "IDLE"
                    hover_alt = None
                else:
                    ok, bbox = tracker.update(frame)
                    if ok:
                        x1, y1, w_box, h_box = bbox
                        tracked_bbox = (int(x1), int(y1), int(w_box), int(h_box))

                        bcx = x1 + w_box / 2.0
                        bcy = y1 + h_box / 2.0

                        # ── Pixel → angle (same as Dhanur tracker.py pixelToAngle) ─
                        yaw_err_rad, pitch_err_rad = pixel_to_angle(bcx, bcy)

                        # ── YAW: direct proportional ───────────────────────
                        az = clamp(-YAW_KP_DIRECT * yaw_err_rad, -YAW_RATE_MAX, YAW_RATE_MAX)

                        # ── ALTITUDE / PITCH: dive at target ───────────────────
                        # pitch_err_rad > 0 → target is below centre → descend
                        pitch_cmd = pid_pitch.compute(pitch_err_rad)
                        vz = clamp(-pitch_cmd * 3.5, -VZ_MAX, VZ_MAX)

                        gz_twist(DRONE, lx=CRUISE_SPEED, ly=0.0, lz=vz, az=az)

                        print(f"\r  TRACK  "
                              f"yaw_err: {math.degrees(yaw_err_rad):+.1f}°  "
                              f"pitch_err: {math.degrees(pitch_err_rad):+.1f}°  "
                              f"az: {az:+.3f}  vz: {vz:+.2f}  "
                              f"alt: {alt:.1f}m   ",
                              end="", flush=True)
                    else:
                        print("\n  [TRACK] Target lost — hovering")
                        tracker = None
                        tracked_bbox = None
                        phase = "IDLE"
                        hover_alt = None

            # ── Render FOV ───────────────────────────────────
            if frame is not None:
                spd = CRUISE_SPEED if phase == "TRACK" else 0.0
                hud = draw_hud(frame, dp, dy, phase, tracked_bbox,
                               MISSION_ALT, spd)
                cv2.imshow(WIN, hud)

            key = cv2.waitKeyEx(1)
            if key != -1:
                k = key & 0xFFFF
                if k in (ord('q'), 27): # q or ESC
                    print("\n  [QUIT] q pressed")
                    break
                elif k == ord('c'):       # keyboard clear
                    tracker = tracked_bbox = None
                    if phase in ["TRACK", "MANUAL"]:
                        phase = "IDLE"
                        hover_alt = None
                    print("\n  [C] target cleared → HOVER")
                elif phase in ["IDLE", "MANUAL", "TRACK"]:
                    # Keyboard override
                    moved = False
                    lx = ly = lz = az = 0.0
                    
                    if k == ord('i'): lx = CRUISE_SPEED; moved=True
                    elif k == ord('k'): lx = -CRUISE_SPEED; moved=True
                    elif k == ord('a'): ly = CRUISE_SPEED; moved=True
                    elif k == ord('d'): ly = -CRUISE_SPEED; moved=True
                    elif k in (ord('w'), 65362): lz = VZ_MAX; moved=True
                    elif k in (ord('s'), 65364): lz = -VZ_MAX; moved=True
                    elif k == ord('j'): az = YAW_RATE_MAX; moved=True
                    elif k == ord('l'): az = -YAW_RATE_MAX; moved=True
                    
                    if moved:
                        tracker = tracked_bbox = None
                        phase = "MANUAL"
                        manual_cmd = {"lx": lx, "ly": ly, "lz": lz, "az": az}
                        last_key_time = time.monotonic()

            # ── Pace to CONTROL_HZ ────────────────────────────
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
