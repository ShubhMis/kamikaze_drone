#!/usr/bin/env python3
"""
kamikaze_intercept.py  v6.0 — CV Detect → PID Intercept
═══════════════════════════════════════════════════════════════════════
Architecture (from Dhanur_New reference):
  1. CLIMB   — reach target altitude (passed as CLI arg), face forward
  2. APPROACH — fly forward at fixed heading, blob-detect target drone
  3. TRACK   — Dhanur-style PID keeps target centered in FOV:
                 yaw_cmd  = PID(yaw_error)   → gz cmd_vel angular.z
                 pitch_cmd= PID(pitch_err)   → gz cmd_vel linear.z
               simultaneously fly forward at pursuit speed
  4. SEARCH  — Dhanur-style 45° step rotation when target lost
  5. STRIKE  — GT distance < STRIKE_DIST → cut motors

Detection:
  Dark-blob detector (dark UAV against bright sky), identical to v5.
  TargetTracker spatial memory rejects false positives (trees etc.).

FOV Window:
  • Heading tape + compass rose (from px4_camera_fov_viewer)
  • Live: current heading / yaw_err / pitch_err / phase / alt / speed
  • Detection box + arrow from crosshair to target
  • Ground-rejection line, tracker last-known orange circle
  • All text crisp: FONT_HERSHEY_DUPLEX + LINE_AA + thickness=2

Usage:
  python3 kamikaze/kamikaze_intercept.py --alt 50
  python3 kamikaze/kamikaze_intercept.py --alt 30 --speed 4.0
  Keys: q=quit
═══════════════════════════════════════════════════════════════════════
"""

import argparse
import math
import subprocess
import sys
import threading
import time
import json
import base64

import cv2
import numpy as np


# ═══════════════════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(description="Kamikaze Intercept v6.0")
    p.add_argument("--alt",    type=float, default=50.0,
                   help="Mission altitude in metres (default 50)")
    p.add_argument("--speed",  type=float, default=3.0,
                   help="Cruise / approach speed m/s (default 3.0)")
    p.add_argument("--target", type=str,   default="target_drone",
                   help="Gazebo model name of target")
    p.add_argument("--drone",  type=str,   default="drone",
                   help="Gazebo model name of kamikaze drone")
    p.add_argument("--world",  type=str,   default="px4_baylands_world",
                   help="Gazebo world name")
    return p.parse_args()


# ═══════════════════════════════════════════════════════════════════════
#  FIXED CONFIG (camera geometry, PIDs, detector thresholds)
# ═══════════════════════════════════════════════════════════════════════

HFOV_RAD   = 1.74          # camera HFOV (x500_mono_cam SDF)
IMG_W      = 1280
IMG_H      = 960
FX         = IMG_W / (2.0 * math.tan(HFOV_RAD / 2.0))   # focal length pixels
FY         = FX
CX, CY     = IMG_W / 2.0, IMG_H / 2.0
VFOV_DEG   = math.degrees(2 * math.atan(math.tan(HFOV_RAD / 2) * IMG_H / IMG_W))
HFOV_DEG   = math.degrees(HFOV_RAD)

CONTROL_HZ = 20            # main loop frequency

# ── Flight ────────────────────────────────────────────────────────────
CLIMB_VZ         = 2.5    # m/s vertical
CLIMB_VH         = 1.5    # m/s horizontal during climb (small drift toward target)
MIN_ENGAGE_SPEED = 2.0
MAX_ENGAGE_SPEED = 6.0
PURSUIT_GAIN     = 0.8    # speed = dist * gain (clamped)
TERMINAL_DIST    = 8.0    # m — switch to TERMINAL mode (full speed)
STRIKE_DIST      = 1.5    # m — GT distance → STRIKE

# ── PID: CLIMB / altitude hold ────────────────────────────────────────
ALT_KP, ALT_KD = 1.0, 0.3
YAW_RATE_NAV   = 0.8      # rad/s — approach / climb yaw rate limit

# ── PID: vision tracking (Dhanur-style) ──────────────────────────────
# Dhanur kp=3.0 kd=0.8 for yaw; kp=2.0 kd=0.9 for pitch (on gimbal)
# Here we map directly to cmd_vel so smaller gains suit:
PID_YAW_KP,   PID_YAW_KD   = 0.10, 0.03
PID_PITCH_KP, PID_PITCH_KD = 0.20, 0.05
YAW_RATE_TRACK   = 1.5    # rad/s limit during TRACK
YAW_RATE_TERMINAL= 2.5

# ── Dhanur thresholds ─────────────────────────────────────────────────
COURSE_THRESHOLD = 12.0   # deg — yaw error before drone turns
PITCH_THRESHOLD  = 17.0   # deg — pitch error before height change
FORCE_TURN_DEG   = 45.0   # deg step for search rotation (Dhanur: 45)
SEARCH_HOLD_S    = 2.0    # seconds to hold at each search step
SEARCH_STEPS     = 8      # 8 × 45° = 360°
MAX_SEARCH_CYCLES= 3

# ── Detector ─────────────────────────────────────────────────────────
DET_DARK_THRESH  = 120    # threshold for dark-blob mask
DET_MIN_AREA     = 15
DET_MAX_AREA     = 1200   # approach ROI
DET_TRACK_MAX_AREA= 9000  # wider during TRACK
DET_MAX_ASPECT   = 8.0
DET_SKY_CONTRAST = 20
DET_ROI_MX       = 0.15   # margin x for approach/search
DET_ROI_MY       = 0.10   # margin y
DET_TRACK_MX     = 0.05   # wider ROI during TRACK
DET_TRACK_MY     = 0.05
DET_GROUND_Y     = 0.72   # reject blobs below this fraction (trees/ground)

CONFIRM_FRAMES   = 2      # consecutive detections before TRACK
LOSS_TIMEOUT_S   = 6.0    # seconds before TRACK → SEARCH
SEARCH_TIMEOUT_S = 20.0   # approach no-det seconds before SEARCH

# ── Gazebo topics ─────────────────────────────────────────────────────
TARGET_SPAWN = (155, 152)  # used only for initial heading estimate


# ═══════════════════════════════════════════════════════════════════════
#  UTILITIES
# ═══════════════════════════════════════════════════════════════════════

def clamp(v, lo, hi):   return max(lo, min(hi, v))
def vec3_sub(a, b):     return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def vec3_len(v):        return math.sqrt(v[0]**2+v[1]**2+v[2]**2)

def normalize_angle(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def quat_to_yaw(x, y, z, w):
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def world_to_body(vx, vy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*vx + s*vy, -s*vx + c*vy

def pixel_to_angle(bx, by):
    """Return (yaw_err_deg, pitch_err_deg) of pixel (bx,by) from frame centre."""
    return (math.degrees(math.atan2(bx - CX, FX)),
            math.degrees(math.atan2(by - CY, FY)))


# ═══════════════════════════════════════════════════════════════════════
#  PID  (Dhanur calculations.py — identical interface)
# ═══════════════════════════════════════════════════════════════════════

class PID:
    def __init__(self, kp, kd=0.0, ki=0.0, i_lim=10.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.I = 0.0
        self.last_e = 0.0
        self.last_t = time.time()
        self.i_lim  = i_lim

    def compute(self, e):
        now = time.time()
        dt  = max(1e-6, now - self.last_t)
        P   = self.kp * e
        self.I = clamp(self.I + e * dt, -self.i_lim, self.i_lim)
        D   = self.kd * (e - self.last_e) / dt
        self.last_e = e
        self.last_t = now
        return P + self.ki * self.I + D

    def reset(self):
        self.I = 0.0
        self.last_e = 0.0
        self.last_t = time.time()


# ═══════════════════════════════════════════════════════════════════════
#  TARGET TRACKER  (spatial memory → reject false positives)
# ═══════════════════════════════════════════════════════════════════════

class TargetTracker:
    """Remembers last known pixel position. Rejects new detections that
    jump more than `max_drift_base + max_drift_rate*dt` pixels from last
    known position — filters out trees, ground objects."""

    def __init__(self, max_drift_base=130, max_drift_rate=260):
        self.cx    = None
        self.cy    = None
        self.area  = None
        self.t     = 0.0
        self.conf  = 0
        self.active= False
        self._db   = max_drift_base
        self._dr   = max_drift_rate

    def filter(self, det, now):
        if det is None:
            return None
        x1, y1, x2, y2, score = det
        bcx = (x1+x2) / 2.0
        bcy = (y1+y2) / 2.0
        area = (x2-x1) * (y2-y1)

        if self.active and self.conf >= 2:
            dt  = max(0.001, now - self.t)
            max_drift = self._db + self._dr * dt
            if abs(bcx - self.cx) > max_drift or abs(bcy - self.cy) > max_drift:
                return None          # spatial rejection
            if self.area and self.area > 0:
                ratio = area / self.area
                if ratio > 6.0 or ratio < 0.15:
                    return None      # size rejection

        self.cx   = bcx
        self.cy   = bcy
        self.area = area
        self.t    = now
        self.conf = min(self.conf + 1, 60)
        self.active = True
        return det

    def lost_for(self, now):
        return now - self.t if self.t > 0 else 999.0

    def last_pixel(self):
        return (self.cx, self.cy) if self.active and self.cx is not None else None

    def reset(self):
        self.cx = self.cy = self.area = None
        self.t  = 0.0
        self.conf  = 0
        self.active= False


# ═══════════════════════════════════════════════════════════════════════
#  DARK-BLOB DETECTOR
# ═══════════════════════════════════════════════════════════════════════

def detect_target(frame, roi_mx, roi_my, max_area, reject_ground=True):
    """Detect dark UAV against bright sky.  Returns (x1,y1,x2,y2,score) or None."""
    h, w = frame.shape[:2]
    rx1, ry1 = int(w * roi_mx),  int(h * roi_my)
    rx2, ry2 = int(w * (1-roi_mx)), int(h * (1-roi_my))
    ground_y  = int(h * DET_GROUND_Y) if reject_ground else h

    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(blurred, DET_DARK_THRESH, 255, cv2.THRESH_BINARY_INV)

    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5)
    mask = cv2.dilate(mask, k3, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_score = None, 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < DET_MIN_AREA or area > max_area:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        if max(bw,bh) / max(min(bw,bh), 1) > DET_MAX_ASPECT:
            continue
        bcx, bcy = x + bw/2, y + bh/2
        if not (rx1 <= bcx <= rx2 and ry1 <= bcy <= ry2):
            continue
        if bcy > ground_y:
            continue

        margin = max(20, int(max(bw,bh)*0.5))
        sx1, sy1 = max(0,x-margin), max(0,y-margin)
        sx2, sy2 = min(w,x+bw+margin), min(h,y+bh+margin)
        surround = gray[sy1:sy2, sx1:sx2].copy()
        iy, ix = y-sy1, x-sx1
        surround[iy:iy+bh, ix:ix+bw] = 0
        nz = surround[surround > 0]
        if len(nz) < 10:
            continue
        contrast = float(np.mean(nz)) - float(np.mean(gray[y:y+bh, x:x+bw]))
        if contrast < DET_SKY_CONTRAST:
            continue

        # sky-preference weight (higher = more likely sky)
        vp = bcy / h
        sky_w = 3.0 - 4.0*vp if vp < 0.5 else 1.0 - 0.8*(vp - 0.5)
        score = area * contrast * sky_w
        if score > best_score:
            best_score = score
            best = (x, y, x+bw, y+bh, score)

    return best


# ═══════════════════════════════════════════════════════════════════════
#  FOV DISPLAY
# ═══════════════════════════════════════════════════════════════════════

C_CROSS  = (180, 180,  60)
C_HDG    = (  0, 220, 255)
C_NORTH  = ( 60,  80, 255)
C_DIM    = (160, 160, 160)
C_OK     = (  0, 200,  50)
C_WARN   = (  0, 200, 200)
C_ERR    = (  0,  60, 220)
C_TRACK  = (  0,   0, 255)
C_SEARCH = (  0, 200, 200)
FT       = cv2.FONT_HERSHEY_DUPLEX


def _draw_heading_tape(img, hdg_deg):
    """Heading tape at top of image (from px4_camera_fov_viewer)."""
    h, w = img.shape[:2]
    tape_y, tape_h, ppd = 0, 26, 5
    cv2.rectangle(img, (0, tape_y), (w, tape_y+tape_h), (20,20,20), -1)
    hdg = hdg_deg % 360.0
    for delta in range(-90, 91, 10):
        deg = (hdg + delta) % 360
        x   = w//2 + int(delta * ppd)
        if not (0 <= x < w):
            continue
        th = 8 if deg % 30 == 0 else 4
        cv2.line(img, (x, tape_y+tape_h-th), (x, tape_y+tape_h), (160,160,160), 1)
        if deg % 30 == 0:
            lbl = f"{int(deg)}"
            tw  = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)[0][0]
            cv2.putText(img, lbl, (x-tw//2, tape_y+tape_h-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_DIM, 1, cv2.LINE_AA)
    mx = w//2
    pts = np.array([[mx-7, tape_y+2], [mx+7, tape_y+2],
                    [mx,   tape_y+tape_h-1]], np.int32)
    cv2.fillPoly(img, [pts], C_HDG)
    # heading value label
    lbl = f"{int(hdg):03d}°"
    tw, th2 = cv2.getTextSize(lbl, FT, 0.60, 2)[0]
    lx = mx - tw//2
    ly = tape_y + tape_h + th2 + 2
    cv2.rectangle(img, (lx-4, tape_y+tape_h+1), (lx+tw+4, ly+3), (20,20,20), -1)
    cv2.putText(img, lbl, (lx, ly), FT, 0.60, C_HDG, 2, cv2.LINE_AA)


def _draw_compass(img, hdg_deg, radius=52):
    h, w = img.shape[:2]
    cx, cy = w - radius - 14, radius + 14
    ov = img.copy()
    cv2.circle(ov,  (cx,cy), radius+4, (12,12,12), -1)
    cv2.addWeighted(ov, 0.65, img, 0.35, 0, img)
    cv2.circle(img, (cx,cy), radius, (110,110,110), 1, cv2.LINE_AA)
    for ta in range(0, 360, 45):
        r  = math.radians(ta - hdg_deg)
        ri, ro = radius-7, radius
        cv2.line(img,
                 (int(cx+ri*math.sin(r)), int(cy-ri*math.cos(r))),
                 (int(cx+ro*math.sin(r)), int(cy-ro*math.cos(r))),
                 (100,100,100), 1, cv2.LINE_AA)
    for label, angle in [("N",0),("E",90),("S",180),("W",270)]:
        r  = math.radians(angle - hdg_deg)
        lr = radius - 16
        lx = int(cx + lr*math.sin(r))
        ly = int(cy - lr*math.cos(r))
        cv2.putText(img, label, (lx-4, ly+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34,
                    C_NORTH if label=="N" else (180,180,180), 1, cv2.LINE_AA)
    nl = radius - 10
    cv2.arrowedLine(img, (cx, cy+nl//3), (cx, cy-nl),
                    C_HDG, 2, cv2.LINE_AA, tipLength=0.25)
    cv2.circle(img, (cx,cy), 3, (255,255,255), -1)


class FOVDisplay:
    def __init__(self, win="KAMIKAZE INTERCEPT"):
        self._win = win
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 960, 720)
        self._hud = {}

    def update(self, **kw):
        self._hud.update(kw)

    def render(self, frame, det, tracker):
        out  = frame.copy()
        h, w = out.shape[:2]
        hud  = self._hud
        cx, cy = int(CX), int(CY)

        phase     = hud.get("phase",    "---")
        hdg_deg   = hud.get("hdg",      0.0)
        alt       = hud.get("alt",      0.0)
        spd       = hud.get("spd",      0.0)
        yaw_err   = hud.get("yaw_err",  0.0)
        pitch_err = hud.get("pitch_err",0.0)
        dist      = hud.get("dist",     999.0)
        trk_conf  = hud.get("trk_conf", 0)
        srch_step = hud.get("srch_step",0)
        mission_alt = hud.get("mission_alt", 50.0)

        # ── Heading tape + compass ────────────────────────────────────
        _draw_heading_tape(out, hdg_deg)
        _draw_compass(out, hdg_deg)

        # ── Ground rejection line ─────────────────────────────────────
        gy = int(h * DET_GROUND_Y)
        cv2.line(out, (0, gy), (w, gy), (0, 0, 160), 1, cv2.LINE_AA)

        # ── Detection ROI box ─────────────────────────────────────────
        if phase == "TRACK":
            rmx, rmy = DET_TRACK_MX, DET_TRACK_MY
        else:
            rmx, rmy = DET_ROI_MX, DET_ROI_MY
        rx1, ry1 = int(w*rmx), int(h*rmy)
        rx2, ry2 = int(w*(1-rmx)), int(h*(1-rmy))
        cv2.rectangle(out, (rx1,ry1), (rx2,ry2), (80,80,80), 1, cv2.LINE_AA)

        # ── Crosshair ─────────────────────────────────────────────────
        cv2.line(out,   (cx-40, cy), (cx+40, cy), C_CROSS, 2)
        cv2.line(out,   (cx, cy-40), (cx, cy+40), C_CROSS, 2)
        cv2.circle(out, (cx, cy), 5, C_CROSS, 2)

        # ── Last-known tracker position ───────────────────────────────
        if tracker and tracker.last_pixel():
            lpx, lpy = tracker.last_pixel()
            cv2.circle(out, (int(lpx), int(lpy)), 14, (0,165,255), 2, cv2.LINE_AA)
            cv2.putText(out, "LAST", (int(lpx)+16, int(lpy)+5),
                        FT, 0.45, (0,165,255), 1, cv2.LINE_AA)

        # ── Detection box + arrow ─────────────────────────────────────
        if det is not None:
            x1, y1, x2, y2, _ = det
            bcx, bcy = (x1+x2)//2, (y1+y2)//2
            barea = (x2-x1)*(y2-y1)
            cv2.rectangle(out, (x1,y1), (x2,y2), C_OK, 2)
            cv2.circle(out,    (bcx, bcy), 5, (0,0,255), -1)
            cv2.arrowedLine(out, (cx,cy), (bcx,bcy),
                            (255,0,255), 2, tipLength=0.12)
            cv2.putText(out,
                f"TGT  yaw:{yaw_err:+.1f}°  pitch:{pitch_err:+.1f}°  a:{barea}",
                (x1, max(y1-12,12)), FT, 0.65, C_OK, 2, cv2.LINE_AA)

        # ── Phase colour ──────────────────────────────────────────────
        if phase == "TRACK":
            ph_col = C_TRACK
            ph_txt = f"TRACKING  trk:{trk_conf}"
        elif phase == "SEARCH":
            ph_col = C_SEARCH
            ph_txt = f"SEARCHING  step:{srch_step}/{SEARCH_STEPS}"
        elif phase in ("CLIMB", "APPROACH"):
            ph_col = C_OK
            ph_txt = phase
        elif phase == "STRIKE":
            ph_col = (0,0,255)
            ph_txt = "** STRIKE **"
        else:
            ph_col = C_DIM
            ph_txt = phase

        # ── Top banner ────────────────────────────────────────────────
        ov = out.copy()
        banner_h = 58
        cv2.rectangle(ov, (0,0), (w, banner_h), (0,0,0), -1)
        cv2.addWeighted(ov, 0.50, out, 0.50, 0, out)

        # Line 1: phase + dist + time
        cv2.putText(out, f"{ph_txt}   DIST:{dist:.1f}m   T:{hud.get('t',0):.0f}s",
                    (10, 30), FT, 0.80, ph_col, 2, cv2.LINE_AA)
        # Right side: tracking mode
        trk_label = hud.get("trk_mode","")
        if trk_label:
            tw = cv2.getTextSize(trk_label, FT, 0.75, 2)[0][0]
            cv2.putText(out, trk_label, (w-tw-12, 30),
                        FT, 0.75, ph_col, 2, cv2.LINE_AA)

        # ── Bottom telemetry bar ──────────────────────────────────────
        ov2 = out.copy()
        bar_h = 100
        py    = h - bar_h
        cv2.rectangle(ov2, (0,py), (w,h), (0,0,0), -1)
        cv2.addWeighted(ov2, 0.40, out, 0.60, 0, out)

        g = C_OK
        # Row 1
        cv2.putText(out,
            f"HDG:{hdg_deg:05.1f}°   YAW_ERR:{yaw_err:+6.1f}°   "
            f"PITCH_ERR:{pitch_err:+6.1f}°   ALT:{alt:.1f}m/{mission_alt:.0f}m",
            (10, py+28), FT, 0.60, g, 2, cv2.LINE_AA)
        # Row 2
        cv2.putText(out,
            f"SPD:{spd:.1f}m/s   COURSE:{(90-hdg_deg)%360:.0f}°   "
            f"TRK_CONF:{trk_conf}   {'NO DET' if det is None else 'DET OK'}",
            (10, py+62), FT, 0.60, g, 2, cv2.LINE_AA)

        return out

    def show(self, img):
        cv2.imshow(self._win, img)

    def key(self, ms=1):
        return cv2.waitKey(ms) & 0xFF

    def close(self):
        cv2.destroyAllWindows()


# ═══════════════════════════════════════════════════════════════════════
#  GAZEBO TRANSPORT (native gz.transport or subprocess fallback)
#  Identical pattern to px4_camera_fov_viewer.py
# ═══════════════════════════════════════════════════════════════════════

def _build_transport(world, drone_model, target_model):
    CAMERA_TOPIC = (f"/world/{world}/model/{drone_model}"
                    f"/link/camera_link/sensor/camera/image")
    POSE_TOPIC   = f"/world/{world}/dynamic_pose/info"

    # ── Try native gz.transport ───────────────────────────────────────
    for versions in [("gz.transport13","gz.msgs10"), ("gz.transport12","gz.msgs9")]:
        try:
            import importlib
            tr = importlib.import_module(versions[0])
            ms = importlib.import_module(versions[1]+".image_pb2")
            ps = importlib.import_module(versions[1]+".pose_v_pb2")
            GzNode, GzImage, Pose_V = tr.Node, ms.Image, ps.Pose_V

            class CamNative:
                def __init__(self):
                    self._slot = [None]; self._lock = threading.Lock()
                    self._node = GzNode()
                    self._node.subscribe(GzImage, CAMERA_TOPIC, self._cb)
                    print(f"[Cam]  gz.transport native ✓")
                def _cb(self, msg):
                    w,h = msg.width, msg.height
                    if not w or not h: return
                    arr = np.frombuffer(msg.data, dtype=np.uint8)
                    if arr.size < w*h*3: return
                    bgr = cv2.cvtColor(arr[:w*h*3].reshape((h,w,3)), cv2.COLOR_RGB2BGR)
                    with self._lock: self._slot[0] = bgr
                def get_frame(self):
                    with self._lock:
                        f = self._slot[0]; return f.copy() if f is not None else None
                def stop(self): pass

            class PoseNative:
                def __init__(self):
                    self._lock = threading.Lock()
                    self._drone  = None; self._target = None
                    self._yaw    = 0.0;  self._ready  = False
                    self._node   = GzNode()
                    self._node.subscribe(Pose_V, POSE_TOPIC, self._cb)
                    print(f"[Pose] gz.transport native ✓")
                def _cb(self, msg):
                    with self._lock:
                        for pose in msg.pose:
                            p = pose.position; o = pose.orientation
                            xyz = (p.x, p.y, p.z)
                            if pose.name == drone_model:
                                self._drone = xyz
                                self._yaw   = quat_to_yaw(o.x,o.y,o.z,o.w)
                                self._ready = True
                            elif (pose.name == target_model
                                  or target_model in pose.name
                                  or "target" in pose.name.lower()):
                                self._target = xyz
                def ready(self):
                    with self._lock: return self._ready
                def drone(self):
                    with self._lock: return self._drone
                def yaw(self):
                    with self._lock: return self._yaw
                def target(self):
                    with self._lock: return self._target
                def stop(self): pass

            return CamNative(), PoseNative(), "gz.transport"
        except (ImportError, Exception):
            continue

    # ── Subprocess fallback ───────────────────────────────────────────
    print("[WARN] gz.transport not found — using subprocess fallback")

    def _fetch_one(topic, timeout=2.0):
        while True:
            try:
                r = subprocess.run(
                    ["gz","topic","-e","--json-output","-n","1","-t",topic],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    timeout=timeout)
                if r.stdout:
                    yield r.stdout
            except subprocess.TimeoutExpired:
                pass
            except Exception:
                time.sleep(0.1)

    class CamSub:
        def __init__(self):
            self._raw  = [None]; self._rl = threading.Lock()
            self._ev   = threading.Event()
            self._slot = [None]; self._fl = threading.Lock()
            self._stop = False
            threading.Thread(target=self._stream, daemon=True).start()
            threading.Thread(target=self._decode, daemon=True).start()
            print(f"[Cam]  subprocess fallback")
        def _stream(self):
            for raw in _fetch_one(CAMERA_TOPIC):
                if self._stop: break
                with self._rl: self._raw[0] = raw
                self._ev.set()
        def _decode(self):
            while not self._stop:
                if not self._ev.wait(0.5): continue
                with self._rl:
                    raw = self._raw[0]; self._raw[0] = None; self._ev.clear()
                if raw is None: continue
                try:
                    msg = json.loads(raw)
                    w,h = int(msg.get("width",0)), int(msg.get("height",0))
                    data = msg.get("data","")
                    if not w or not h or not data: continue
                    pix = base64.b64decode(data)
                    if len(pix) < w*h*3: continue
                    arr = np.frombuffer(pix[:w*h*3],dtype=np.uint8).reshape((h,w,3))
                    bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                    with self._fl: self._slot[0] = bgr
                except Exception: pass
        def get_frame(self):
            with self._fl:
                f = self._slot[0]; return f.copy() if f is not None else None
        def stop(self): self._stop = True

    class PoseSub:
        def __init__(self):
            self._lock   = threading.Lock()
            self._drone  = None; self._target = None
            self._yaw    = 0.0;  self._ready  = False
            self._stop   = False
            threading.Thread(target=self._run, daemon=True).start()
            print(f"[Pose] subprocess fallback")
        def _run(self):
            for raw in _fetch_one(POSE_TOPIC, timeout=2.0):
                if self._stop: break
                try:
                    msg = json.loads(raw)
                    with self._lock:
                        for p in msg.get("pose",[]):
                            n   = p.get("name","")
                            pos = p.get("position",{})
                            ori = p.get("orientation",{})
                            xyz = (float(pos.get("x",0)),
                                   float(pos.get("y",0)),
                                   float(pos.get("z",0)))
                            if n == drone_model:
                                self._drone = xyz
                                self._yaw   = quat_to_yaw(
                                    float(ori.get("x",0)), float(ori.get("y",0)),
                                    float(ori.get("z",0)), float(ori.get("w",1)))
                                self._ready = True
                            elif (n == target_model or target_model in n
                                  or "target" in n.lower()):
                                self._target = xyz
                except Exception: pass
        def ready(self):
            with self._lock: return self._ready
        def drone(self):
            with self._lock: return self._drone
        def yaw(self):
            with self._lock: return self._yaw
        def target(self):
            with self._lock: return self._target
        def stop(self): self._stop = True

    return CamSub(), PoseSub(), "subprocess"


# ═══════════════════════════════════════════════════════════════════════
#  GAZEBO COMMANDS
# ═══════════════════════════════════════════════════════════════════════

def gz_twist(drone_model, lx=0.0, ly=0.0, lz=0.0, az=0.0):
    topic = f"/{drone_model}/cmd_vel"
    msg   = (f"linear: {{x: {lx:.4f}, y: {ly:.4f}, z: {lz:.4f}}}, "
             f"angular: {{x: 0, y: 0, z: {az:.4f}}}")
    subprocess.Popen(
        ["gz","topic","-t", topic,"-m","gz.msgs.Twist","-p", msg],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def gz_enable(drone_model, on: bool):
    subprocess.Popen(
        ["gz","topic","-t", f"/{drone_model}/enable",
         "-m","gz.msgs.Boolean",
         "-p", f"data: {'true' if on else 'false'}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ═══════════════════════════════════════════════════════════════════════
#  FSM
# ═══════════════════════════════════════════════════════════════════════

class FSM:
    def __init__(self):
        self.phase       = "INIT"
        self.phase_start = time.time()
        self.t0          = None
        self._log        = []

    def go(self, new_phase, reason=""):
        old = self.phase
        self.phase       = new_phase
        self.phase_start = time.time()
        if self.t0 is None and new_phase == "ARM":
            self.t0 = time.time()
        self._log.append((old, new_phase, reason))
        print(f"\n  [FSM] {old} → {new_phase}  {reason}")

    def elapsed(self):
        return time.time() - self.phase_start

    def mission_t(self):
        return time.time() - self.t0 if self.t0 else 0.0

    @property
    def done(self):
        return self.phase in ("DONE","ABORT","STRIKE")

    def print_log(self):
        print("\n  ── Transition Log ──")
        for a, b, r in self._log:
            print(f"    {a:>10} → {b:<10}  {r}")
        print()


# ═══════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════

def main():
    args = parse_args()
    MISSION_ALT  = args.alt
    CRUISE_SPEED = args.speed
    DRONE        = args.drone
    TARGET       = args.target
    WORLD        = args.world

    CMD_VEL_TOPIC = f"/{DRONE}/cmd_vel"   # for display only

    print("\n" + "═"*65)
    print("  KAMIKAZE INTERCEPT v6.0 — CV Detect → PID Intercept")
    print("═"*65)

    # ── Wait for Gazebo ──────────────────────────────────────────────
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

    # ── Transport ────────────────────────────────────────────────────
    cam, poses, mode = _build_transport(WORLD, DRONE, TARGET)

    # ── Objects ──────────────────────────────────────────────────────
    fov     = FOVDisplay()
    fsm     = FSM()
    tracker = TargetTracker()

    pid_alt   = PID(ALT_KP, ALT_KD)
    pid_yaw   = PID(PID_YAW_KP,   PID_YAW_KD)
    pid_pitch = PID(PID_PITCH_KP, PID_PITCH_KD)

    dt_target = 1.0 / CONTROL_HZ
    armed     = False

    # ── Keepalive thread ─────────────────────────────────────────────
    stop_ev = threading.Event()
    def keepalive():
        while not stop_ev.is_set():
            if armed: gz_enable(DRONE, True)
            stop_ev.wait(0.25)
    threading.Thread(target=keepalive, daemon=True).start()

    # ── Detection state ──────────────────────────────────────────────
    det_count     = 0
    last_det_t    = 0.0

    # ── Search state ─────────────────────────────────────────────────
    srch_target_yaw = 0.0
    srch_holding    = False
    srch_hold_start = 0.0
    srch_step       = 0
    srch_cycle      = 0

    # ── Approach heading (locked at end of CLIMB) ────────────────────
    approach_hdg = 0.0     # radians

    # HUD cache
    yaw_err_disp = pitch_err_disp = 0.0

    print(f"\n  Drone      : {DRONE}")
    print(f"  Target     : {TARGET}")
    print(f"  World      : {WORLD}")
    print(f"  Altitude   : {MISSION_ALT} m  (arg --alt)")
    print(f"  Speed      : {CRUISE_SPEED} m/s  (arg --speed)")
    print(f"  Camera     : {IMG_W}×{IMG_H}  HFOV={HFOV_DEG:.1f}°")
    print(f"  Transport  : {mode}")
    print(f"  PID yaw    : kp={PID_YAW_KP}  kd={PID_YAW_KD}")
    print(f"  PID pitch  : kp={PID_PITCH_KP}  kd={PID_PITCH_KD}")
    print(f"  Search     : {FORCE_TURN_DEG}° × {SEARCH_STEPS} = 360°  hold={SEARCH_HOLD_S}s")
    print(f"  Strike     : GT dist < {STRIKE_DIST} m")
    print(f"\n  Press 'q' in FOV window to abort\n")

    prev_t = time.monotonic()

    try:
        while not fsm.done:
            t0     = time.monotonic()
            dt_act = max(0.001, t0 - prev_t)
            prev_t = t0
            now    = time.time()

            # ── Read sensors ─────────────────────────────────────────
            dp    = poses.drone()
            dy    = poses.yaw()          # radians
            tp    = poses.target()
            frame = cam.get_frame()

            # ── Detection + tracker filter ────────────────────────────
            raw_det = None
            det     = None
            if frame is not None:
                in_track = (fsm.phase == "TRACK")
                raw_det = detect_target(
                    frame,
                    roi_mx   = DET_TRACK_MX if in_track else DET_ROI_MX,
                    roi_my   = DET_TRACK_MY if in_track else DET_ROI_MY,
                    max_area = DET_TRACK_MAX_AREA if in_track else DET_MAX_AREA,
                    reject_ground = not in_track,
                )
                det = tracker.filter(raw_det, now)

            rejected = (raw_det is not None and det is None)  # FP filtered

            if det is not None:
                det_count  = min(det_count + 1, CONFIRM_FRAMES + 20)
                last_det_t = now
                x1,y1,x2,y2,_ = det
                bcx, bcy = (x1+x2)/2.0, (y1+y2)/2.0
                yaw_err_disp, pitch_err_disp = pixel_to_angle(bcx, bcy)
            else:
                det_count = 0

            # ── Distances ─────────────────────────────────────────────
            dist3d     = vec3_len(vec3_sub(tp, dp)) if (dp and tp) else 999.0
            gt_ok      = (dp is not None and tp is not None)
            hdg_deg    = (math.degrees(dy) % 360.0) if dy else 0.0
            alt        = dp[2] if dp else 0.0
            course_deg = (90.0 - hdg_deg) % 360.0  # compass course

            # ── HUD update ────────────────────────────────────────────
            fov.update(
                phase       = fsm.phase,
                hdg         = hdg_deg,
                alt         = alt,
                spd         = 0.0,
                yaw_err     = yaw_err_disp,
                pitch_err   = pitch_err_disp,
                dist        = dist3d,
                trk_conf    = tracker.conf,
                srch_step   = srch_step,
                t           = fsm.mission_t(),
                mission_alt = MISSION_ALT,
            )

            # ══════════════════════════════════════════════════════════
            #  INIT
            # ══════════════════════════════════════════════════════════
            if fsm.phase == "INIT":
                if poses.ready() and dp:
                    print(f"  ✓ Pose acquired: drone=({dp[0]:.1f},{dp[1]:.1f},{dp[2]:.1f})")
                    fsm.go("ARM", "pose ready")
                else:
                    time.sleep(0.5)
                    _show_fov(fov, frame, det, tracker); continue

            # ══════════════════════════════════════════════════════════
            #  ARM
            # ══════════════════════════════════════════════════════════
            elif fsm.phase == "ARM":
                gz_enable(DRONE, True)
                armed = True
                time.sleep(0.3)
                gz_twist(DRONE, lz=0.1)
                time.sleep(0.2)
                pid_alt.reset()
                last_det_t = now
                tracker.reset()
                det_count  = 0
                fsm.go("CLIMB", f"mission alt={MISSION_ALT}m")

            # ══════════════════════════════════════════════════════════
            #  CLIMB  — reach MISSION_ALT, face toward TARGET_SPAWN
            # ══════════════════════════════════════════════════════════
            elif fsm.phase == "CLIMB":
                if not dp: time.sleep(dt_target); continue

                alt_err = MISSION_ALT - alt
                vz = clamp(pid_alt.compute(alt_err), -2.0, CLIMB_VZ)
                if alt < 2.0: vz = max(vz, CLIMB_VZ)

                # Yaw toward target spawn (initial heading only)
                dx_ = TARGET_SPAWN[0] - dp[0]
                dy_ = TARGET_SPAWN[1] - dp[1]
                hd  = math.sqrt(dx_*dx_ + dy_*dy_)
                yaw_cmd = 0.0
                vxw = vyw = 0.0
                if hd > 0.5:
                    desired  = math.atan2(dy_, dx_)
                    yaw_cmd  = clamp(1.5*normalize_angle(desired - dy),
                                     -YAW_RATE_NAV, YAW_RATE_NAV)
                    s        = min(CLIMB_VH, hd * 0.12)
                    vxw, vyw = dx_/hd*s, dy_/hd*s

                bx, by = world_to_body(vxw, vyw, dy)
                gz_twist(DRONE, lx=bx, ly=by, lz=vz, az=yaw_cmd)
                fov.update(spd=math.sqrt(vxw**2+vyw**2))
                print(f"\r  CLIMB  alt:{alt:5.1f}/{MISSION_ALT:.0f}m  "
                      f"Δ{alt_err:+.1f}  hdg:{hdg_deg:05.1f}°   ",
                      end="", flush=True)

                if abs(alt_err) < 3.0 and alt > 5.0:
                    approach_hdg = dy
                    pid_alt.reset()
                    last_det_t = now
                    tracker.reset(); det_count = 0
                    print(f"\n  ✓ Altitude reached. Heading locked: {hdg_deg:.1f}°")
                    fsm.go("APPROACH", f"alt={alt:.1f}m  hdg={hdg_deg:.1f}°")

            # ══════════════════════════════════════════════════════════
            #  APPROACH  — fly forward at fixed heading, scan for target
            # ══════════════════════════════════════════════════════════
            elif fsm.phase == "APPROACH":
                if not dp: time.sleep(dt_target); continue

                # Hold altitude
                alt_err = MISSION_ALT - alt
                vz = clamp(pid_alt.compute(alt_err), -2.0, 1.5)

                # Yaw: vision PID if detecting, else hold locked heading
                if det is not None:
                    yaw_cmd = clamp(-pid_yaw.compute(yaw_err_disp),
                                    -YAW_RATE_NAV, YAW_RATE_NAV)
                else:
                    hdg_err = normalize_angle(approach_hdg - dy)
                    yaw_cmd = clamp(1.5 * hdg_err,
                                    -YAW_RATE_NAV, YAW_RATE_NAV)

                gz_twist(DRONE, lx=CRUISE_SPEED, ly=0.0, lz=vz, az=yaw_cmd)
                fov.update(spd=CRUISE_SPEED)

                ds = f"det[{det_count}/{CONFIRM_FRAMES}] trk:{tracker.conf}" \
                     if det else ("FP_rej" if rejected else "---")
                print(f"\r  APPR  hdg:{hdg_deg:05.1f}°  alt:{alt:.1f}  [{ds}]   ",
                      end="", flush=True)

                # Confirm detection → TRACK
                if det_count >= CONFIRM_FRAMES:
                    pid_yaw.reset(); pid_pitch.reset()
                    fsm.go("TRACK", "target confirmed")
                    continue

                # Search after timeout
                if now - last_det_t > SEARCH_TIMEOUT_S:
                    srch_step       = 0
                    srch_cycle      = 0
                    srch_target_yaw = normalize_angle(dy + math.radians(FORCE_TURN_DEG))
                    srch_holding    = False
                    tracker.reset(); det_count = 0
                    fsm.go("SEARCH", f"no det for {SEARCH_TIMEOUT_S}s")

            # ══════════════════════════════════════════════════════════
            #  SEARCH  — Dhanur handleMissingTarget style 360° scan
            #  Rotate in 45° steps, hold 2s each, return to APPROACH
            # ══════════════════════════════════════════════════════════
            elif fsm.phase == "SEARCH":
                if not dp: time.sleep(dt_target); continue

                alt_err = MISSION_ALT - alt
                vz = clamp(pid_alt.compute(alt_err), -1.5, 1.5)

                # Detection during search → TRACK
                if det_count >= CONFIRM_FRAMES:
                    pid_yaw.reset(); pid_pitch.reset()
                    fsm.go("TRACK", "target found during search")
                    continue

                if not srch_holding:
                    yaw_err_s = normalize_angle(srch_target_yaw - dy)
                    if abs(yaw_err_s) < math.radians(5.0):
                        srch_holding    = True
                        srch_hold_start = now
                        gz_twist(DRONE, lz=vz)
                    else:
                        az = clamp(1.5 * yaw_err_s, -0.6, 0.6)
                        gz_twist(DRONE, lz=vz, az=az)
                else:
                    gz_twist(DRONE, lz=vz)
                    if now - srch_hold_start > SEARCH_HOLD_S:
                        srch_step += 1
                        if srch_step >= SEARCH_STEPS:
                            srch_cycle += 1
                            if srch_cycle >= MAX_SEARCH_CYCLES:
                                fsm.go("ABORT", f"{srch_cycle} full 360° — lost")
                            else:
                                last_det_t = now
                                tracker.reset(); det_count = 0
                                fsm.go("APPROACH",
                                       f"360° scan #{srch_cycle} done")
                        else:
                            srch_target_yaw = normalize_angle(
                                srch_target_yaw + math.radians(FORCE_TURN_DEG))
                            srch_holding = False

                fov.update(srch_step=srch_step)
                print(f"\r  SEARCH  {'HOLD' if srch_holding else 'TURN'}  "
                      f"step:{srch_step}/{SEARCH_STEPS}  "
                      f"cycle:{srch_cycle+1}/{MAX_SEARCH_CYCLES}   ",
                      end="", flush=True)

            # ══════════════════════════════════════════════════════════
            #  TRACK  — Dhanur onTargetDetected PID control
            #
            #  yaw_err  → pid_yaw  → gz_twist angular.z  (turn drone)
            #  pitch_err→ pid_pitch→ gz_twist linear.z   (change height)
            #  Also fly forward at pursuit speed.
            #  When lost: yaw toward last pixel → re-acquire.
            # ══════════════════════════════════════════════════════════
            elif fsm.phase == "TRACK":
                if not dp: time.sleep(dt_target); continue

                # GT strike check
                if gt_ok and dist3d < STRIKE_DIST:
                    fsm.go("STRIKE", f"dist={dist3d:.2f}m"); continue

                if det is not None:
                    # ── Target VISIBLE ────────────────────────────────
                    # Dhanur: yaw_cmd = pid_yaw.compute(yaw_err)
                    #         pitch_cmd = pid_pitch.compute(pitch_err)
                    # Dhanur clips both: yaw_cmd max±40°, pitch_cmd max±20°
                    # Here we map directly to rad/s and m/s respectively.

                    yaw_cmd_raw   = pid_yaw.compute(yaw_err_disp)
                    pitch_cmd_raw = pid_pitch.compute(pitch_err_disp)

                    # Dhanur: clamp yaw ±40, pitch ±20
                    yaw_cmd_raw   = clamp(yaw_cmd_raw,   -40, 40)
                    pitch_cmd_raw = clamp(pitch_cmd_raw, -20, 20)

                    # Scale to angular velocity and vertical velocity
                    # (negate: positive yaw_err = target right → turn right = positive az)
                    # Camera is body-fixed, so yaw_err sign: target right → bcx > CX
                    # pixel_to_angle: positive dx → positive yaw_err
                    # to centre: turn right (positive az in Gazebo twist)
                    az = clamp(yaw_cmd_raw * 0.04,
                               -YAW_RATE_TRACK, YAW_RATE_TRACK)

                    # pitch_err positive = target below centre → drone must go down
                    vz_vis = clamp(-pitch_cmd_raw * 0.08, -3.5, 3.5)

                    # Pursuit forward speed
                    if gt_ok and dist3d < TERMINAL_DIST:
                        trk_mode = "TERMINAL"
                        fwd = MAX_ENGAGE_SPEED
                        az  = clamp(az * 1.5, -YAW_RATE_TERMINAL, YAW_RATE_TERMINAL)
                    else:
                        trk_mode = "PURSUIT"
                        fwd = CRUISE_SPEED

                    # Altitude hold weighted with vision pitch
                    alt_err  = MISSION_ALT - alt
                    vz_alt   = clamp(pid_alt.compute(alt_err), -2.0, 2.0)
                    vz       = 0.6*vz_vis + 0.4*vz_alt

                    gz_twist(DRONE, lx=fwd, ly=0.0, lz=vz, az=az)
                    fov.update(spd=fwd, trk_mode=trk_mode)

                    bbox_area = (det[2]-det[0])*(det[3]-det[1])
                    print(f"\r  TRACK[{trk_mode:8s}]  dist:{dist3d:5.1f}  "
                          f"fwd:{fwd:.1f}  az:{az:+.3f}  vz:{vz:+.1f}  "
                          f"yaw_e:{yaw_err_disp:+.1f}°  "
                          f"p_e:{pitch_err_disp:+.1f}°  a:{bbox_area}   ",
                          end="", flush=True)

                else:
                    # ── Target LOST ───────────────────────────────────
                    lost = tracker.lost_for(now)

                    if lost > LOSS_TIMEOUT_S:
                        srch_step       = 0
                        srch_cycle      = 0
                        srch_target_yaw = normalize_angle(
                            dy + math.radians(FORCE_TURN_DEG))
                        srch_holding    = False
                        # keep tracker memory for false-positive filter
                        fsm.go("SEARCH", f"lost {lost:.1f}s"); continue

                    # Steer toward last known pixel (Dhanur gimbal memory)
                    lp = tracker.last_pixel()
                    if lp:
                        ye, pe = pixel_to_angle(lp[0], lp[1])
                        az  = clamp(-pid_yaw.compute(ye)*0.04,
                                    -YAW_RATE_TRACK, YAW_RATE_TRACK)
                        vz  = clamp(-pid_pitch.compute(pe)*0.08, -2.0, 2.0)
                        fwd = 1.5   # creep forward
                    else:
                        az = 0.0; vz = 0.0; fwd = 1.0

                    gz_twist(DRONE, lx=fwd, ly=0.0, lz=vz, az=az)
                    fov.update(spd=fwd, trk_mode=f"LOST {lost:.1f}s")
                    print(f"\r  TRACK[LOST {lost:.1f}/{LOSS_TIMEOUT_S:.0f}s]  "
                          f"trk:{tracker.conf}  lp:{lp}   ",
                          end="", flush=True)

            # ══════════════════════════════════════════════════════════
            #  STRIKE
            # ══════════════════════════════════════════════════════════
            elif fsm.phase == "STRIKE":
                print(f"\n\n  {'█'*50}")
                print(f"  ██  STRIKE CONFIRMED  dist={dist3d:.2f}m  ██")
                print(f"  {'█'*50}\n")
                gz_twist(DRONE)          # stop
                gz_enable(DRONE, False)
                armed = False
                fsm.go("DONE", "target eliminated")

            # ── FOV render ───────────────────────────────────────────
            if frame is not None:
                rendered = fov.render(frame, det, tracker)
                fov.show(rendered)

            if fov.key(1) == ord('q'):
                print("\n  [ABORT] q pressed")
                fsm.go("ABORT", "operator")
                break

            # ── Pace to CONTROL_HZ ────────────────────────────────────
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, dt_target - elapsed))

    except KeyboardInterrupt:
        print("\n  [ABORT] Ctrl-C")

    finally:
        print("  Stopping …")
        for _ in range(5):
            gz_twist(DRONE)
            time.sleep(0.05)
        gz_enable(DRONE, False)
        stop_ev.set()
        poses.stop(); cam.stop(); fov.close()
        fsm.print_log()
        print("  ✓ Shutdown complete.\n")


def _show_fov(fov, frame, det, tracker):
    if frame is not None:
        fov.show(fov.render(frame, det, tracker))
    fov.key(1)


if __name__ == "__main__":
    main()
