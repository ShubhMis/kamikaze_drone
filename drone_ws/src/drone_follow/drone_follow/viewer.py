"""Read-only monocular camera HUD. No control publishers or services.

ROS callbacks retain a bounded handful of raw images. HighGUI stays on the main
thread, so GUI work never blocks detector/PX4 callbacks in other processes.
Detections are paired by exact image stamp; live command/attitude have their
own age checks and are explicitly labelled as live rather than time-aligned.
"""
from collections import OrderedDict, deque
import json
import math
import os
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data as SENSOR
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleOdometry

WINDOW = 'Drone camera | Q / Esc closes viewer only'
GREEN = (110, 220, 105)
CYAN = (245, 210, 60)
ORANGE = (70, 170, 250)
WHITE = (235, 239, 245)
GREY = (155, 166, 182)


def window_closed():
    """Probe a supported property without changing the window mode.

    OpenCV 4.5.4 GTK returns -1 for WND_PROP_VISIBLE even on an open window.
    FULLSCREEN is supported by GTK/Qt: 0 means an ordinary, open window;
    a missing window instead returns -1 or raises cv2.error.
    """
    try:
        return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN) < 0
    except cv2.error:
        return True


def select_frame(images, observations, now, max_wait, previous_stamp):
    """Prefer the newest completed image/detection pair, without a growing delay.

    A previous pair may stay on screen for at most max_wait wall seconds after
    its image arrived. Then show the newest raw image, with detection pending.
    Never attach an old box to a new frame or rewind to an already-skipped frame.
    """
    if not images:
        return None, None
    newest = next(reversed(images))
    if newest in observations:
        return images[newest], observations[newest]
    for stamp in reversed(images):
        if (stamp in observations and now-images[stamp][1] <= max_wait
                and (previous_stamp is None or stamp >= previous_stamp)):
            return images[stamp], observations[stamp]
    return images[newest], None


def attitude_degrees(q):
    """Hamilton w,x,y,z FRD->NED: heading clockwise, pitch positive nose-up."""
    q = np.asarray(q, dtype=float)
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) < .5:
        return None
    w, x, y, z = q / np.linalg.norm(q)
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
    return math.degrees(yaw) % 360, math.degrees(pitch)


class Viewer(Node):
    def __init__(self, config):
        super().__init__('camera_viewer')
        self.lock = threading.Lock()
        self.images = OrderedDict()
        self.pair_wait = config['baseline'].get('viewer_pair_wait_s', .08)
        self.received_frames = 0
        self.observations = OrderedDict()
        self.camera = None
        self.metrics = ({}, 0.)
        self.attitude = (None, 0.)
        self.sim_time = None
        self.arrivals = deque(maxlen=60)
        detector = config['detector']['ros__parameters']
        ns = config['follower']['ros__parameters']['px4_namespace']
        # Reliable delivery avoids large-image fragment loss. Depth=1 bounds
        # queued history. Four references allow asynchronous detection pairing.
        self.create_subscription(Image, detector['rgb_topic'], self.image,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(CameraInfo, detector['info_topic'], self.calibration, SENSOR)
        self.create_subscription(String, '/target/observation', self.observation, 10)
        self.create_subscription(String, '/follow/metrics', self.control, 10)
        self.create_subscription(VehicleOdometry, ns+'/fmu/out/vehicle_odometry', self.odom, SENSOR)
        self.create_subscription(Clock, '/clock', self.clock, SENSOR)

    def image(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        with self.lock:
            self.images[stamp] = (msg, now)
            while len(self.images) > 4:
                self.images.popitem(last=False)
            self.received_frames += 1
            self.arrivals.append(now)

    def calibration(self, msg):
        with self.lock:
            self.camera = msg

    def observation(self, msg):
        data = json.loads(msg.data)
        # Integer nanoseconds avoid float-rounding mismatches with sensor stamps.
        stamp = data.get('stamp_ns', round(data['stamp_s']*1e9))
        with self.lock:
            self.observations[stamp] = data
            while len(self.observations) > 60:
                self.observations.popitem(last=False)

    def control(self, msg):
        data = json.loads(msg.data)
        with self.lock:
            self.metrics = (data, time.monotonic())

    def odom(self, msg):
        angles = attitude_degrees(msg.q) if msg.pose_frame == VehicleOdometry.POSE_FRAME_NED else None
        with self.lock:
            self.attitude = (angles, time.monotonic())

    def clock(self, msg):
        with self.lock:
            self.sim_time = msg.clock.sec+msg.clock.nanosec*1e-9

    def snapshot(self, now, previous_stamp):
        with self.lock:
            selected, observation = select_frame(self.images, self.observations,
                                                 now, self.pair_wait, previous_stamp)
            return (selected, observation, self.camera, self.metrics,
                    self.attitude, self.sim_time, list(self.arrivals))


def text(image, value, x, y, color=WHITE, scale=.52):
    cv2.putText(image, value, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, 1, cv2.LINE_AA)


def draw_hud(frame, observation, info, metrics, attitude, sim_time, arrivals,
             received, stamp_s, stale_s, now, displayed=()):
    """Render camera-local errors separately from current flight telemetry."""
    h, w = frame.shape[:2]
    header = 32
    canvas = np.full((max(h, 490)+header, w+435, 3), (24, 28, 35), dtype=np.uint8)
    canvas[header:header+h, :w] = frame
    gap = now-received if received is not None else None
    fresh_frame = gap is not None and gap < stale_s
    cx, cy = w/2, h/2
    if info is not None and (info.width, info.height) == (w, h):
        cx, cy = info.k[2], info.k[5]
    centre = (round(cx), round(cy)+header)
    cv2.drawMarker(canvas, centre, CYAN, cv2.MARKER_CROSS, 24, 1)
    text(canvas, 'RGB CAMERA | centre crosshair', 12, 22, CYAN)

    detected = fresh_frame and observation is not None and observation.get('visible', False)
    if detected:
        u, v = observation['u_px'], observation['v_px']
        point = (round(u), round(v)+header)
        bbox = observation.get('bbox_xywh_px', [u-8, v-8, 16, 16])
        x, y, bw, bh = bbox
        cv2.rectangle(canvas, (max(0,round(x)-3), max(header,round(y)+header-3)),
                      (min(w-1,round(x+bw)+3),min(h+header-1,round(y+bh)+header+3)), GREEN, 2)
        cv2.circle(canvas, point, 3, GREEN, -1)
        cv2.arrowedLine(canvas, point, centre, CYAN, 1, cv2.LINE_AA, tipLength=.12)
        text(canvas, 'RED MARKER', max(5,min(w-130,round(x))), max(header+18,round(y)+header-9), GREEN)
        # Fixed crop magnification is display-only; no scale/range inference.
        half = 24
        x0, y0 = max(0,round(u)-half), max(0,round(v)-half)
        crop = frame[y0:min(h,y0+2*half), x0:min(w,x0+2*half)]
        if crop.size and w >= 180 and h >= 180:
            zoom = cv2.resize(crop, (144,144), interpolation=cv2.INTER_NEAREST)
            canvas[header+h-154:header+h-10, w-154:w-10] = zoom
            cv2.rectangle(canvas,(w-155,header+h-155),(w-9,header+h-9),GREEN,1)
            text(canvas,'TARGET DETAIL',w-154,header+h-164,GREEN,.45)

    if received is None:
        warning = 'WAITING FOR CAMERA'
    elif not fresh_frame:
        warning = f'STALE CAMERA: frozen for {gap:.1f}s'
        canvas[header:header+h,:w] = (canvas[header:header+h,:w]*.45).astype(np.uint8)
    else:
        warning = ''
    if warning:
        text(canvas, warning, 20, header+45, ORANGE,.65)

    px = w+16
    line = 25
    def row(index, label, color=WHITE):
        text(canvas,label,px,header+index*line,color)
    row(0,'IMAGE MEASUREMENT',CYAN)
    if not fresh_frame:
        row(1,'Detection: unavailable / camera stale',ORANGE)
    elif observation is None:
        row(1,'Detection: pending for this frame',GREY)
    elif not detected:
        row(1,'Detection: '+observation.get('reason','not visible'),ORANGE)
    else:
        row(1,'Detection: RED MARKER',GREEN)
        row(2,f'Error: u {observation["error_u_px"]:+.1f} px | v {observation["error_v_px"]:+.1f} px')
        x, y = observation['x_normalized'], observation['y_normalized']
        yaw_error = math.degrees(math.atan(x))
        pitch_error = math.degrees(math.atan2(-y, math.sqrt(1+x*x)))
        row(3,f'Yaw align: {yaw_error:+.2f} deg (+right)')
        row(4,f'Pitch align: {pitch_error:+.2f} deg (+up)')
    row(5,'Align = camera bearing, not attitude command',GREY)
    row(7,'LIVE PX4 ATTITUDE',CYAN)
    angles, attitude_wall = attitude
    if angles is not None and now-attitude_wall < stale_s:
        row(8,f'Yaw {angles[0]:6.2f} deg | pitch {angles[1]:+6.2f} deg')
        row(9,'Heading: NED clockwise | pitch: nose up',GREY)
    else:
        row(8,'Attitude: waiting / stale',ORANGE)
    row(11,'LIVE CONTROLLER OUTPUT',CYAN)
    data, control_wall = metrics
    if data and now-control_wall < stale_s:
        if data.get('publishing_setpoint'):
            row(12,f'Yaw rate: {math.degrees(data["yaw_rate_rps"]):+.2f} deg/s',GREEN)
            climb = -data['velocity_ned_mps'][2]
            row(13,f'Climb speed: {climb:+.3f} m/s (+up)',GREEN)
        else:
            row(12,'External commands PAUSED',ORANGE)
            row(13,'PX4 owns the current flight manoeuvre',GREY)
        row(15,data.get('state','unknown')[:48],GREEN if data.get('enabled') else GREY)
    else:
        row(12,'Controller output: waiting / stale',ORANGE)
    row(14,'Pitch command: NONE (vertical-speed control)',GREY)
    row(16,'Live telemetry is not exposure-time aligned',GREY)
    # Count arrivals, not UI redraws: repeating a frame is not additional FPS.
    fps = ((len(arrivals)-1)/(arrivals[-1]-arrivals[0])
           if fresh_frame and len(arrivals)>1 and arrivals[-1]>arrivals[0] else 0.)
    age = None if sim_time is None or stamp_s is None else sim_time-stamp_s
    shown = ((len(displayed)-1)/(displayed[-1]-displayed[0])
             if fresh_frame and len(displayed)>1 and displayed[-1]>displayed[0] else 0.)
    row(17,f'Received {fps:.1f} | shown {shown:.1f} FPS',CYAN)
    row(18,'Age: '+('--' if age is None else f'{age*1000:.0f} ms sim')+
        (' | queued: --' if gap is None else f' | queued: {gap*1000:.0f} ms wall'),GREY)
    row(19,'Q / Esc: close viewer; terminal Ctrl+C: stop',GREY)
    return canvas


def main():
    run = Path(os.environ['BASELINE_RUN_DIR'])
    config = yaml.safe_load((run/'config.yaml').read_text())
    c = config['baseline']
    cv2.setNumThreads(1)
    rclpy.init()
    node = Viewer(config)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    errors = []
    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as exc:
            errors.append(exc)
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    bridge = CvBridge()
    frame = np.zeros((480,640,3),dtype=np.uint8)
    decoded_stamp = None
    displayed = deque(maxlen=60)
    # A run may use --duration 0. Keep timing samples bounded (~5 min at 20 Hz).
    samples = deque(maxlen=6000)
    displayed_count = matched_count = 0
    saved = set()
    first_enabled = None
    reason = 'ROS shutdown'
    interval = 1.0/c['viewer_fps']
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW,1075,522)
        print('VIEWER: window opened; waiting for RGB and stamped detections', flush=True)
        while rclpy.ok():
            started = time.monotonic()
            if errors:
                raise errors[0]
            latest, observation, info, metrics, attitude, sim_time, arrivals = node.snapshot(started, decoded_stamp)
            received = stamp_s = None
            if latest:
                msg, received = latest
                key = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                stamp_s = key*1e-9
                if key != decoded_stamp:
                    frame = bridge.imgmsg_to_cv2(msg,'bgr8')
                    decoded_stamp = key
                    displayed.append(started)
                    samples.append([started, started-received, observation is not None])
                    displayed_count += 1
                    matched_count += int(observation is not None)
                elif observation is not None and samples and not samples[-1][2]:
                    samples[-1][2] = True
                    matched_count += 1
            canvas = draw_hud(frame,observation,info,metrics,attitude,sim_time,arrivals,
                              received,stamp_s,c['viewer_stale_s'],started,displayed)
            cv2.imshow(WINDOW,canvas)
            # Two bounded diagnostic snapshots, not per-frame disk recording.
            if metrics[0].get('enabled') and first_enabled is None:
                first_enabled = started
            if c.get('record', False) and observation is not None and observation.get('visible') and received is not None and started-received < c['viewer_stale_s']:
                name = 'viewer_tracking.png' if first_enabled is not None and started-first_enabled > 2 else 'viewer_first.png'
                if name not in saved:
                    cv2.imwrite(str(run/name), canvas)
                    saved.add(name)
                    print(f'VIEWER: live RGB + matched detection; saved {name}', flush=True)
            key = cv2.waitKey(1) & 0xff
            if key in (27,ord('q'),ord('Q')):
                reason = 'Q / Esc'
                break
            if window_closed():
                reason = 'window close button'
                break
            time.sleep(max(0.,interval-(time.monotonic()-started)))
    except KeyboardInterrupt:
        reason = 'stack shutdown / SIGINT'
    except Exception:
        reason = 'viewer error; inspect component log'
        raise
    finally:
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()
        elapsed = samples[-1][0]-samples[0][0] if len(samples)>1 else 0.
        gaps = np.diff([s[0] for s in samples])
        queue = [s[1] for s in samples]
        summary = dict(exit_reason=reason, received_frames=node.received_frames,
            displayed_frames=displayed_count, matched_display_frames=matched_count,
            timing_window_frames=len(samples),
            displayed_fps_wall=(len(samples)-1)/elapsed if elapsed else 0.,
            display_gap_p95_ms=float(np.percentile(gaps,95)*1000) if len(gaps) else None,
            display_gap_max_ms=float(max(gaps)*1000) if len(gaps) else None,
            callback_to_display_p95_ms=float(np.percentile(queue,95)*1000) if queue else None,
            timing_note='Wall time at render submission, not measured monitor refresh or end-to-end camera latency')
        (run/'viewer.json').write_text(json.dumps(summary,indent=2))
        print(f'VIEWER: {reason}; {displayed_count} distinct frames, '+
              f'{summary["displayed_fps_wall"]:.1f} FPS; diagnostics in viewer.json',flush=True)


if __name__ == '__main__':
    main()
