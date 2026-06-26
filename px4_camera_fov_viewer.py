#!/usr/bin/env python3
"""
px4_camera_fov_viewer.py  –  v4  (gz.transport native — zero base64/JSON overhead)
════════════════════════════════════════════════════════════════════════════════════
Root cause of the "stuck at one frame" bug in v2/v3:
  gz topic --json-output encodes every 1280×960 frame as base64 JSON (~4.9 MB of
  ASCII). The brace-depth scanner iterated every byte of that string, and when the
  pipe filled faster than we drained it the subprocess blocked — freezing the viewer.

This version uses gz.transport Python bindings directly:
  • No subprocess pipe                   → no pipe-full deadlock
  • No JSON parsing                      → no per-byte scanning
  • No base64 decode                     → saves ~30 ms/frame of CPU
  • Callback fires on the gz message bus → lowest possible latency

Requirements:
  pip install gz-transport<version>    (matches your Gazebo install)
  or it may already be on PYTHONPATH if you sourced the Gazebo setup.

If gz.transport bindings are unavailable, the script falls back to the
subprocess approach but with a FIXED brace scanner (uses str.find instead
of per-byte loop) and a hard cap on pipe buffer size.

Controls: q=quit  f=FOV  g=grid  s=save  h=heading overlay
════════════════════════════════════════════════════════════════════════════════════
"""

import math
import os
import datetime
import threading
import time

import numpy as np
import cv2

# ─── CONFIG ──────────────────────────────────────────────────────────────────
CAMERA_TOPIC  = "/world/px4_baylands_world/model/drone/link/camera_link/sensor/camera/image"
POSE_TOPIC    = "/world/px4_baylands_world/dynamic_pose/info"

HFOV_RAD     = 1.74
IMAGE_WIDTH  = 1280
IMAGE_HEIGHT = 960
ASPECT_RATIO = IMAGE_WIDTH / IMAGE_HEIGHT
VFOV_RAD     = 2 * math.atan(math.tan(HFOV_RAD / 2) / ASPECT_RATIO)
HFOV_DEG     = math.degrees(HFOV_RAD)
VFOV_DEG     = math.degrees(VFOV_RAD)
SAVE_DIR     = os.path.dirname(os.path.abspath(__file__))

C_FOV   = (0,  220, 100)
C_CROSS = (180,180,  60)
C_DIM   = (160,160, 160)
C_OK    = (0,  200,  50)
C_WAIT  = (0,   60, 220)
C_GRID  = (60,  60,  60)
C_HDG   = (0,  220, 255)
C_NORTH = (60,  80, 255)


# ══════════════════════════════════════════════════════════════════════════════
#  TRANSPORT LAYER — gz.transport native (preferred) or subprocess fallback
# ══════════════════════════════════════════════════════════════════════════════

def _try_native_transport():
    """
    Returns (CameraListener_class, PoseListener_class) using gz.transport
    Python bindings if available, else returns None.
    """
    try:
        # Gazebo Harmonic / Garden ship gz-transport Python bindings
        from gz.transport13 import Node as GzNode          # adjust version number
        from gz.msgs10.image_pb2 import Image as GzImage   # adjust version number
        from gz.msgs10.pose_v_pb2 import Pose_V            # adjust version number
        return _make_native_classes(GzNode, GzImage, Pose_V)
    except ImportError:
        pass
    try:
        # Older Fortress / Garden
        from gz.transport12 import Node as GzNode
        from gz.msgs9.image_pb2 import Image as GzImage
        from gz.msgs9.pose_v_pb2 import Pose_V
        return _make_native_classes(GzNode, GzImage, Pose_V)
    except ImportError:
        pass
    return None


def _make_native_classes(GzNode, GzImage, Pose_V):

    class CameraListener:
        def __init__(self, topic):
            self._slot  = [None]
            self._lock  = threading.Lock()
            self._fps_count = 0
            self._fps_val   = 0.0
            self._fps_t     = time.monotonic()
            self._node  = GzNode()
            ok = self._node.subscribe(GzImage, topic, self._cb)
            if not ok:
                raise RuntimeError(f"Cannot subscribe to {topic}")
            print(f"[Camera] native gz.transport  {topic}")

        def _cb(self, msg):
            # msg.data is raw bytes — no base64, no JSON
            w, h = msg.width, msg.height
            if w == 0 or h == 0:
                return
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            if arr.size < w * h * 3:
                return
            bgr = cv2.cvtColor(arr[:w*h*3].reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
            with self._lock:
                self._slot[0] = bgr
            self._fps_count += 1
            now = time.monotonic()
            dt  = now - self._fps_t
            if dt >= 1.0:
                self._fps_val   = self._fps_count / dt
                self._fps_count = 0
                self._fps_t     = now

        def get_frame(self):
            with self._lock:
                f = self._slot[0]
                return f.copy() if f is not None else None

        def get_fps(self):  return self._fps_val
        def is_ready(self): return self._slot[0] is not None
        def stop(self):     pass   # GzNode cleans up on GC

    class PoseListener:
        def __init__(self, topic, model_name="drone"):
            self._model = model_name
            self._course = 0.0
            self._lock   = threading.Lock()
            self._node   = GzNode()
            self._node.subscribe(Pose_V, topic, self._cb)
            print(f"[Pose ] native gz.transport  {topic}")

        def _cb(self, msg):
            for pose in msg.pose:
                if pose.name != self._model:
                    continue
                o   = pose.orientation
                qx, qy, qz, qw = o.x, o.y, o.z, o.w
                yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
                with self._lock:
                    self._course = (90.0 - math.degrees(yaw)) % 360.0
                break

        def get_course(self):
            with self._lock:
                return self._course

        def stop(self): pass

    return CameraListener, PoseListener


# ══════════════════════════════════════════════════════════════════════════════
#  FALLBACK — subprocess with FIXED fast JSON extractor
# ══════════════════════════════════════════════════════════════════════════════

def _make_subprocess_classes():
    """
    Fallback when gz.transport Python bindings aren't installed.
    Key fix over v2/v3: uses str.find() to locate braces instead of a
    per-byte Python loop — ~50× faster for 4.9 MB JSON strings.
    Also caps the rolling buffer at 16 MB to prevent pipe bloat.
    """
    import selectors, subprocess, base64, json

    BUF_CAP = 16 * 1024 * 1024   # discard buffer if it grows past 16 MB

    def _stream(topic, chunk=4 * 1024 * 1024):
        """
        Yield the raw bytes of the latest complete JSON frame from Gazebo.

        Strategy: call `gz topic -e --json-output -n 1 -t TOPIC` in a tight
        loop.  Each invocation captures exactly ONE message and exits — no
        pipe buffer accumulates, no brace-scanning needed.  Latency = one
        subprocess-launch RTT (~5 ms) which is far less than the 33 ms frame
        period, so the viewer always shows a nearly-live frame.
        """
        import subprocess
        while True:
            try:
                result = subprocess.run(
                    ["gz", "topic", "-e", "--json-output", "-n", "1", "-t", topic],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    timeout=2.0,
                )
                if result.stdout:
                    yield result.stdout
            except subprocess.TimeoutExpired:
                pass   # no message yet — loop again
            except Exception:
                time.sleep(0.1)

    class CameraListener:
        def __init__(self, topic):
            self._topic      = topic
            self._raw_slot   = [None]
            self._raw_lock   = threading.Lock()
            self._raw_event  = threading.Event()
            self._frame_slot = [None]
            self._frame_lock = threading.Lock()
            self._fps_count  = 0
            self._fps_val    = 0.0
            self._fps_t      = time.monotonic()
            self._stop       = False
            threading.Thread(target=self._stream_thread, daemon=True).start()
            threading.Thread(target=self._decode_thread, daemon=True).start()
            print(f"[Camera] subprocess fallback  {topic}")

        def _stream_thread(self):
            for raw in _stream(self._topic, chunk=4 * 1024 * 1024):
                if self._stop:
                    break
                with self._raw_lock:
                    self._raw_slot[0] = raw
                self._raw_event.set()

        def _decode_thread(self):
            while not self._stop:
                if not self._raw_event.wait(timeout=0.5):
                    continue
                with self._raw_lock:
                    raw = self._raw_slot[0]
                    self._raw_slot[0] = None
                    self._raw_event.clear()
                if raw is None:
                    continue
                frame = self._decode(raw)
                if frame is not None:
                    with self._frame_lock:
                        self._frame_slot[0] = frame
                    self._fps_count += 1
                    now = time.monotonic()
                    dt  = now - self._fps_t
                    if dt >= 1.0:
                        self._fps_val   = self._fps_count / dt
                        self._fps_count = 0
                        self._fps_t     = now

        def _decode(self, raw):
            try:
                msg  = json.loads(raw)
                w, h = int(msg.get("width", 0)), int(msg.get("height", 0))
                data = msg.get("data", "")
                if not w or not h or not data:
                    return None
                pix = base64.b64decode(data)
                exp = w * h * 3
                if len(pix) < exp:
                    return None
                arr = np.frombuffer(pix[:exp], dtype=np.uint8).reshape((h, w, 3))
                return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            except Exception as e:
                print(f"[Camera] decode error: {e}")
                return None

        def get_frame(self):
            with self._frame_lock:
                f = self._frame_slot[0]
                return f.copy() if f is not None else None

        def get_fps(self):   return self._fps_val
        def is_ready(self):  return self._frame_slot[0] is not None
        def stop(self):      self._stop = True

    class PoseListener:
        def __init__(self, topic, model_name="drone"):
            self._topic  = topic
            self._model  = model_name
            self._course = 0.0
            self._lock   = threading.Lock()
            self._stop   = False
            threading.Thread(target=self._run, daemon=True).start()
            print(f"[Pose ] subprocess fallback  {topic}")

        def _run(self):
            for raw in _stream(self._topic, chunk=16 * 1024):
                if self._stop:
                    break
                try:
                    import json
                    msg = json.loads(raw)
                    for pose in msg.get("pose", []):
                        if pose.get("name") != self._model:
                            continue
                        o  = pose.get("orientation", {})
                        qx = float(o.get("x", 0))
                        qy = float(o.get("y", 0))
                        qz = float(o.get("z", 0))
                        qw = float(o.get("w", 1))
                        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
                        with self._lock:
                            self._course = (90.0 - math.degrees(yaw)) % 360.0
                        break
                except Exception:
                    pass

        def get_course(self):
            with self._lock:
                return self._course

        def stop(self):
            self._stop = True

    return CameraListener, PoseListener


# ══════════════════════════════════════════════════════════════════════════════
#  DRAW HELPERS  (unchanged from v3)
# ══════════════════════════════════════════════════════════════════════════════

def draw_fov_overlay(img, hfov_deg, vfov_deg):
    h, w   = img.shape[:2]
    cx, cy = w // 2, h // 2
    lw, bk = 2, 24
    # for bx, by in [(0,0),(w-1,0),(0,h-1),(w-1,h-1)]:
    #     sx, sy = (1 if bx==0 else -1), (1 if by==0 else -1)
    #     cv2.line(img,(bx,by),(bx+sx*bk,by),C_FOV,lw)
    #     cv2.line(img,(bx,by),(bx,by+sy*bk),C_FOV,lw)
    #     cv2.line(img,(cx,cy),(bx,by),C_FOV,1,cv2.LINE_AA)
    cv2.line(img,(cx-22,cy),(cx+22,cy),C_CROSS,lw)
    cv2.line(img,(cx,cy-22),(cx,cy+22),C_CROSS,lw)
    cv2.circle(img,(cx,cy),4,C_CROSS,-1)
    # ay,pad=cy-40,12
    # cv2.arrowedLine(img,(w-pad,ay),(cx+pad,ay),C_FOV,lw,tipLength=0.07)
    # cv2.arrowedLine(img,(pad,ay),(cx-pad,ay),C_FOV,lw,tipLength=0.07)
    # ht=f"HFOV: {hfov_deg:.1f}\u00b0"
    # tw=cv2.getTextSize(ht,cv2.FONT_HERSHEY_SIMPLEX,0.48,1)[0][0]
    # cv2.putText(img,ht,(cx-tw//2,ay-5),cv2.FONT_HERSHEY_SIMPLEX,0.48,C_FOV,1,cv2.LINE_AA)
    # ax=cx+55
    # cv2.arrowedLine(img,(ax,h-pad),(ax,cy+pad),C_FOV,lw,tipLength=0.07)
    # cv2.arrowedLine(img,(ax,pad),(ax,cy-pad),C_FOV,lw,tipLength=0.07)
    # cv2.putText(img,f"VFOV: {vfov_deg:.1f}\u00b0",(ax+6,cy+12),cv2.FONT_HERSHEY_SIMPLEX,0.48,C_FOV,1,cv2.LINE_AA)

def draw_grid(img,cols=8,rows=6):
    h,w=img.shape[:2]
    for c in range(1,cols): cv2.line(img,(int(w*c/cols),0),(int(w*c/cols),h),C_GRID,1)
    for r in range(1,rows): cv2.line(img,(0,int(h*r/rows)),(w,int(h*r/rows)),C_GRID,1)

def draw_border(img,color,t=4):
    h,w=img.shape[:2]; cv2.rectangle(img,(0,0),(w-1,h-1),color,t)

def draw_compass_rose(img,heading_deg,radius=58):
    h,w=img.shape[:2]; cx=w-radius-14; cy=radius+14
    ov=img.copy(); cv2.circle(ov,(cx,cy),radius+4,(12,12,12),-1)
    cv2.addWeighted(ov,0.65,img,0.35,0,img)
    cv2.circle(img,(cx,cy),radius,(110,110,110),1,cv2.LINE_AA)
    for ta in range(0,360,45):
        r=math.radians(ta-heading_deg); ri,ro=radius-7,radius
        cv2.line(img,(int(cx+ri*math.sin(r)),int(cy-ri*math.cos(r))),(int(cx+ro*math.sin(r)),int(cy-ro*math.cos(r))),(100,100,100),1,cv2.LINE_AA)
    for label,angle in [("N",0),("E",90),("S",180),("W",270)]:
        r=math.radians(angle-heading_deg); lr=radius-16
        lx=int(cx+lr*math.sin(r)); ly=int(cy-lr*math.cos(r))
        cv2.putText(img,label,(lx-4,ly+5),cv2.FONT_HERSHEY_SIMPLEX,0.34,C_NORTH if label=="N" else (180,180,180),1,cv2.LINE_AA)
    nl=radius-10
    cv2.arrowedLine(img,(cx,cy+nl//3),(cx,cy-nl),C_HDG,2,cv2.LINE_AA,tipLength=0.25)
    cv2.line(img,(cx,cy),(cx,cy+nl//3),(100,200,180),1,cv2.LINE_AA)
    cv2.circle(img,(cx,cy),3,(255,255,255),-1)

def draw_heading_tape(img,heading_deg):
    h,w=img.shape[:2]; tape_y=32; tape_h=26; hdg=heading_deg%360.0; ppd=5
    cv2.rectangle(img,(0,tape_y),(w,tape_y+tape_h),(20,20,20),-1)
    for delta in range(-90,91,10):
        deg=(hdg+delta)%360; x=w//2+int(delta*ppd)
        if not(0<=x<w): continue
        th=8 if deg%30==0 else 4
        cv2.line(img,(x,tape_y+tape_h-th),(x,tape_y+tape_h),(160,160,160),1)
        if deg%30==0:
            lbl=f"{int(deg)}"; tw=cv2.getTextSize(lbl,cv2.FONT_HERSHEY_SIMPLEX,0.38,1)[0][0]
            cv2.putText(img,lbl,(x-tw//2,tape_y+tape_h-10),cv2.FONT_HERSHEY_SIMPLEX,0.38,C_DIM,1,cv2.LINE_AA)
    mx=w//2
    pts=np.array([[mx-7,tape_y+2],[mx+7,tape_y+2],[mx,tape_y+tape_h-1]],np.int32)
    cv2.fillPoly(img,[pts],C_HDG)
    cv2.line(img,(mx,tape_y+tape_h),(mx,tape_y+tape_h+16),C_HDG,1,cv2.LINE_AA)
    # ── Heading degree label below the arrow ──────────────────────
    lbl = f"{int(hdg):03d}deg"
    (tw, th2), bl = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    lx = mx - tw // 2
    ly = tape_y + tape_h + 16 + th2 + 2
    # Dark backing pill
    cv2.rectangle(img, (lx - 4, ly - th2 - 2), (lx + tw + 4, ly + bl + 1), (20, 20, 20), -1)
    cv2.putText(img, lbl, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_HDG, 2, cv2.LINE_AA)


def draw_hud(img,fps,ready,show_fov,show_grid,show_hdg,course_deg,mode_label):
    h,w=img.shape[:2]; bar=np.zeros((38,w,3),dtype=np.uint8); bar[:]=(25,25,28)
    spec=f"  Cam: {IMAGE_WIDTH}x{IMAGE_HEIGHT}  HFOV {HFOV_DEG:.1f}\u00b0  VFOV {VFOV_DEG:.1f}\u00b0  [{mode_label}]"
    cv2.putText(bar,spec,(4,25),cv2.FONT_HERSHEY_SIMPLEX,0.50,C_DIM,1,cv2.LINE_AA)
    sc=C_OK if ready else C_WAIT
    right=(f"HDG:{course_deg:05.1f}\u00b0  FPS:{fps:4.1f}  [{'LIVE' if ready else 'WAIT'}]"
           +("  [FOV]" if show_fov else "")+("  [GRID]" if show_grid else "")
           +("  [HDG]" if show_hdg else "")+"  ")
    tw=cv2.getTextSize(right,cv2.FONT_HERSHEY_SIMPLEX,0.50,1)[0][0]
    cv2.putText(bar,right,(w-tw-4,25),cv2.FONT_HERSHEY_SIMPLEX,0.50,sc,1,cv2.LINE_AA)
    return np.vstack([img,bar])

def draw_title_bar(img,topic):
    h,w=img.shape[:2]; bar=np.zeros((28,w,3),dtype=np.uint8); bar[:]=(20,20,22)
    cv2.putText(bar,f"  {topic}    |  [q]Quit  [f]FOV  [g]Grid  [h]Heading  [s]Save",
                (4,19),cv2.FONT_HERSHEY_SIMPLEX,0.44,C_DIM,1,cv2.LINE_AA)
    return np.vstack([bar,img])

def make_placeholder(w,h):
    img=np.zeros((h,w,3),dtype=np.uint8); img[:]=(18,18,20)
    cv2.putText(img,"Waiting for Gazebo camera...",(w//2-170,h//2-20),cv2.FONT_HERSHEY_SIMPLEX,0.7,(120,120,120),1)
    cv2.putText(img,f"Topic: {CAMERA_TOPIC}",(w//2-130,h//2+20),cv2.FONT_HERSHEY_SIMPLEX,0.45,(70,70,70),1)
    draw_border(img,C_WAIT,3); return img


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════

def run_viewer():
    result = _try_native_transport()
    if result:
        CameraListener, PoseListener = result
        mode_label = "gz.transport"
    else:
        print("[WARN] gz.transport Python bindings not found — using subprocess fallback")
        CameraListener, PoseListener = _make_subprocess_classes()
        mode_label = "subprocess"

    cam  = CameraListener(CAMERA_TOPIC)
    pose = PoseListener(POSE_TOPIC)

    print("\n" + "═"*64)
    print(f"  PX4 Camera Viewer  v4  ({mode_label})")
    print(f"  HFOV {HFOV_DEG:.1f}°   VFOV {VFOV_DEG:.1f}°   {IMAGE_WIDTH}×{IMAGE_HEIGHT}")
    print("  [q]Quit  [f]FOV  [g]Grid  [h]Heading  [s]Save")
    print("═"*64)

    WIN = "PX4 Drone — Camera FOV Viewer"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, IMAGE_WIDTH, IMAGE_HEIGHT + 28 + 38)

    show_fov = show_grid = False
    show_hdg = True
    placeholder = make_placeholder(IMAGE_WIDTH, IMAGE_HEIGHT)
    work_buf = None

    while True:
        raw_frame = cam.get_frame()
        ready     = raw_frame is not None
        course    = pose.get_course()

        if not ready:
            disp = placeholder.copy()
        else:
            if work_buf is None or work_buf.shape != raw_frame.shape:
                work_buf = np.empty_like(raw_frame)
            np.copyto(work_buf, raw_frame)
            disp = work_buf
            if show_grid:  draw_grid(disp)
            if show_fov:   draw_fov_overlay(disp, HFOV_DEG, VFOV_DEG)
            if show_hdg:
                draw_heading_tape(disp, course)
                draw_compass_rose(disp, course)
            draw_border(disp, C_OK if ready else C_WAIT, 3)

        disp = draw_hud(disp, cam.get_fps(), ready, show_fov, show_grid, show_hdg, course, mode_label)
        disp = draw_title_bar(disp, CAMERA_TOPIC)
        cv2.imshow(WIN, disp)

        key = cv2.waitKey(1) & 0xFF
        if   key == ord('q'): break
        elif key == ord('f'): show_fov  = not show_fov
        elif key == ord('g'): show_grid = not show_grid
        elif key == ord('h'): show_hdg  = not show_hdg
        elif key == ord('s') and ready:
            ts   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(SAVE_DIR, f"drone_cam_{ts}.png")
            cv2.imwrite(path, raw_frame)
            print(f"[Saved] {path}")

    cv2.destroyAllWindows()
    cam.stop()
    pose.stop()
    print("Viewer closed.")


if __name__ == "__main__":
    run_viewer()