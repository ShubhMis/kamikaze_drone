# Complete depth-approach files

Generated from the updated workspace files. `monocular.py` is included unchanged. Default mode is `centre`; set `follower.ros__parameters.mode` to `approach` to enable forward motion.

PX4 velocity is local NED, so forward velocity is rotated by heading: `[fwd*cos(yaw), fwd*sin(yaw), down]`. The detector publishes normalized pixel errors in x/y and median optical-axis depth in metres in z.

Validation: 30 tests, launcher preflight and SDF validation passed. No approach flight was run.

## [drone_follow/guidance.py](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/drone_follow/guidance.py)

```python
"""Depth-assisted stand-off approach for the level-camera Gazebo SITL demo."""
from dataclasses import dataclass
import math

from .monocular import centring_command


@dataclass
class Settings:
    target_range_m: float = 10.0
    min_range_m: float = 2.0
    max_fwd_speed: float = 2.0
    fwd_gain: float = 0.5
    yaw_gain: float = 0.8
    vertical_gain: float = 0.7
    max_yaw_rate: float = 0.3
    max_vertical_speed: float = 0.4
    range_margin: float = 1.0
    brake_accel: float = 1.0
    latency: float = 0.4

    def validate(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('Guidance settings must be finite')
        if min(self.min_range_m, self.max_fwd_speed, self.brake_accel) <= 0:
            raise ValueError('Range, speed and braking limits must be positive')
        if min(self.fwd_gain, self.range_margin, self.latency) < 0:
            raise ValueError('Gain, range margin and latency must be nonnegative')
        if self.target_range_m <= self.min_range_m + self.range_margin:
            raise ValueError('target_range_m must exceed min_range_m + range_margin')
        centring_command(0, 0, self.yaw_gain, self.vertical_gain,
                         self.max_yaw_rate, self.max_vertical_speed)


def approach_command(bearing_x, bearing_y, range_m, own_fwd_speed, c):
    """Return (heading-forward m/s, NED down m/s, NED yaw rate rad/s).

    Bearings are normalized pixel errors, not components of a unit vector.
    Forward is horizontal along the vehicle heading; the adapter must rotate
    it into NED north/east. Range is median optical-axis depth in metres.
    This approach-only command stops at the target range; it never reverses.
    The braking bound constrains the command, not actual stopping distance.
    """
    c.validate()
    down, yaw_rate = centring_command(
        bearing_x, bearing_y, c.yaw_gain, c.vertical_gain,
        c.max_yaw_rate, c.max_vertical_speed)
    if (not math.isfinite(range_m) or not math.isfinite(own_fwd_speed)
            or range_m <= c.min_range_m):
        return 0.0, down, yaw_rate

    fwd = max(0.0, min(c.max_fwd_speed,
                       c.fwd_gain * (range_m - c.target_range_m)))
    # Retain the archived envelope: v*latency + v^2/(2*a) <= clearance.
    clearance = max(0.0, range_m - c.min_range_m - c.range_margin)
    a, t = c.brake_accel, c.latency
    allowed = max(0.0, math.sqrt((a*t)**2 + 2*a*clearance) - a*t)
    # If current motion already exhausts that clearance, request braking now.
    closing = max(0.0, own_fwd_speed)
    stopping_distance = closing*t + closing**2/(2*a)
    if stopping_distance >= clearance:
        fwd = 0.0
    return min(fwd, allowed), down, yaw_rate
```

## [drone_follow/follower.py](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/drone_follow/follower.py)

```python
"""ROS/PX4 adapter for centring and depth-assisted approach in Gazebo SITL."""
from dataclasses import fields
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as SENSOR
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from px4_msgs.msg import VehicleOdometry, VehicleStatus, OffboardControlMode, TrajectorySetpoint
from .monocular import centring_command
from .guidance import Settings, approach_command


class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        defaults = dict(vars(Settings()), px4_namespace='/px4_1', mode='centre',
                        max_measurement_age_s=0.3, minimum_detections=6)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = lambda key: self.get_parameter(key).value
        if self.p('mode') not in ('centre', 'approach'):
            raise ValueError('mode must be centre or approach')
        self.guidance_settings().validate()
        centring_command(0, 0, self.p('yaw_gain'), self.p('vertical_gain'),
                         self.p('max_yaw_rate'), self.p('max_vertical_speed'))
        ns = self.p('px4_namespace')
        self.odom_msg = self.status_msg = None
        self.odom_wall = self.status_wall = self.target_wall = 0.0
        self.target_msg = None
        self.target_count = 0
        self.reset_counter = None
        self.enabled = self.fault = False
        self.last_tick = None
        self.mode_text = ''
        self.create_subscription(VehicleOdometry, ns+'/fmu/out/vehicle_odometry', self.odom, SENSOR)
        self.create_subscription(VehicleStatus, ns+'/fmu/out/vehicle_status', self.status, SENSOR)
        self.create_subscription(Vector3Stamped, '/target/bearing_camera', self.target, 5)
        self.mode_pub = self.create_publisher(OffboardControlMode, ns+'/fmu/in/offboard_control_mode', 10)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, ns+'/fmu/in/trajectory_setpoint', 10)
        self.diag = self.create_publisher(String, '/follow/state', 5)
        self.metrics = self.create_publisher(String, '/follow/metrics', 10)
        self.create_service(SetBool, '/follow/enable', self.enable)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def guidance_settings(self):
        return Settings(**{f.name: self.p(f.name) for f in fields(Settings)})

    def heading(self):
        q = np.asarray(self.odom_msg.q, dtype=float)
        norm = float(np.linalg.norm(q))
        if not math.isfinite(norm) or norm < 0.5:
            raise ValueError('Invalid attitude')
        w, x, y, z = q/norm
        return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

    def in_offboard(self):
        return (self.status_msg is not None
                and self.status_msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and self.status_msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def odom(self, msg):
        if self.reset_counter is not None and self.reset_counter != msg.reset_counter:
            self.fault = self.fault or self.in_offboard()
            self.enabled = False
            self.target_count = 0
        self.reset_counter = msg.reset_counter
        self.odom_msg, self.odom_wall = msg, time.monotonic()

    def status(self, msg):
        self.status_msg, self.status_wall = msg, time.monotonic()

    def healthy(self):
        m = self.odom_msg
        return (not self.fault and m is not None and self.status_msg is not None
                and time.monotonic()-self.odom_wall < 0.3
                and time.monotonic()-self.status_wall < 2.0
                and m.pose_frame == VehicleOdometry.POSE_FRAME_NED
                and m.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED
                and np.all(np.isfinite(list(m.position)+list(m.velocity)+list(m.q))))

    def target(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        v = msg.vector
        if (msg.header.frame_id != 'follow_camera_optical'
                or not all(math.isfinite(x) for x in (v.x, v.y))
                or not 0 <= self.now()-stamp < self.p('max_measurement_age_s')):
            return
        if self.target_msg:
            prev = self.target_msg.header.stamp
            dt = stamp-(prev.sec+prev.nanosec*1e-9)
            if dt <= 0:
                return
            if dt > self.p('max_measurement_age_s'):
                self.target_count = 0
        self.target_msg, self.target_wall = msg, time.monotonic()
        self.target_count += 1

    def fresh(self):
        if not self.target_msg:
            return False
        t = self.target_msg.header.stamp
        range_valid = (math.isfinite(self.target_msg.vector.z)
                       and self.target_msg.vector.z > 0)
        return (self.p('mode') in ('centre', 'approach')
                and (self.p('mode') == 'centre' or range_valid)
                and self.target_count >= self.p('minimum_detections')
                and 0 <= self.now()-(t.sec+t.nanosec*1e-9) < self.p('max_measurement_age_s')
                and time.monotonic()-self.target_wall < self.p('max_measurement_age_s'))

    def enable(self, req, res):
        if not req.data:
            self.enabled = False
            res.success, res.message = True, 'Following disabled; zero velocity requested'
        elif self.healthy() and self.in_offboard() and self.fresh():
            self.enabled = True
            res.success = True
            res.message = ('Approach enabled; depth-based stand-off control'
                           if self.p('mode') == 'approach' else
                           'Centring enabled; horizontal velocity remains zero')
        else:
            res.success, res.message = False, (
                'Need healthy armed Offboard hover and fresh detections; '
                'approach also requires valid depth')
        return res

    def may_stream(self):
        # uXRCE input writes the same uORB TrajectorySetpoint used by autonomous
        # takeoff/landing. Publishing zero during landing can prevent PX4's
        # demanded-descent check from latching ground contact. Prime only in
        # disarmed/hold modes; let the autopilot own other manoeuvres completely.
        s = self.status_msg
        return (s.arming_state != VehicleStatus.ARMING_STATE_ARMED or
                s.nav_state in (VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                                VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
                                VehicleStatus.NAVIGATION_STATE_POSCTL))

    def tick(self):
        now = self.now()
        if self.last_tick is not None and not 0 < now-self.last_tick <= 0.3:
            self.enabled = False
            self.target_count = 0
        self.last_tick = now
        fwd = north = east = down = yaw_rate = 0.0
        state = 'READY / ZERO VELOCITY'
        if not self.healthy():
            self.enabled = False
            state = 'STATE INVALID: no Offboard heartbeat'
        elif not self.may_stream():
            self.enabled = False
            state = 'AUTOPILOT MANOEUVRE: external setpoints paused'
        else:
            if not self.in_offboard():
                self.enabled = False
            if self.enabled and not self.fresh():
                self.enabled = False
                state = 'TARGET LOST OR RANGE INVALID: following disabled'
            elif self.enabled:
                v = self.target_msg.vector
                try:
                    if self.p('mode') == 'approach':
                        heading = self.heading()
                        cosine, sine = math.cos(heading), math.sin(heading)
                        velocity = self.odom_msg.velocity
                        own_fwd_speed = velocity[0]*cosine + velocity[1]*sine
                        fwd, down, yaw_rate = approach_command(
                            v.x, v.y, v.z, own_fwd_speed, self.guidance_settings())
                        # TrajectorySetpoint is local NED, not body-forward.
                        north, east = fwd*cosine, fwd*sine
                        state = 'APPROACH / DEPTH STAND-OFF'
                    else:
                        down, yaw_rate = centring_command(v.x, v.y,
                            self.p('yaw_gain'), self.p('vertical_gain'),
                            self.p('max_yaw_rate'), self.p('max_vertical_speed'))
                        state = 'CENTRING / NO FORWARD APPROACH'
                except ValueError as exc:
                    self.enabled = False
                    fwd = north = east = down = yaw_rate = 0.0
                    state = f'CONTROL INVALID: {exc}'
            mode = OffboardControlMode(timestamp=int(now*1e6), velocity=True)
            sp = TrajectorySetpoint()
            sp.timestamp = mode.timestamp
            sp.position = [float('nan')]*3
            sp.acceleration = [float('nan')]*3
            sp.jerk = [float('nan')]*3
            sp.velocity = [float(north), float(east), float(down)]
            sp.yaw, sp.yawspeed = float('nan'), float(yaw_rate)
            self.mode_pub.publish(mode)
            self.sp_pub.publish(sp)
        if state != self.mode_text:
            self.get_logger().info(state)
            self.mode_text = state
        self.diag.publish(String(data=state))
        age = None
        if self.target_msg:
            t = self.target_msg.header.stamp
            age = now-(t.sec+t.nanosec*1e-9)
        self.metrics.publish(String(data=json.dumps(dict(stamp_s=now, state=state,
            enabled=self.enabled, measurement_age_s=age, fresh=self.fresh(),
            measurement_stamp_s=None if age is None else now-age,
            publishing_setpoint=self.healthy() and self.may_stream(),
            mode=self.p('mode'), forward_speed_mps=fwd,
            velocity_ned_mps=[north,east,down], yaw_rate_rps=yaw_rate))))


def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

## [drone_follow/detector.py](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/drone_follow/detector.py)

```python
"""Red-marker bearing with optional aligned depth; no vehicle/target pose input.

Vector3Stamped carries normalized pixel errors in x/y and median optical-axis
depth in metres in z. Missing depth is NaN; RGB-only centring still works.
"""
from collections import deque
import json
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import qos_profile_sensor_data as SENSOR
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
from cv_bridge import CvBridge


class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        # A small image does not benefit from OpenCV spawning a worker per CPU.
        # Fixed threading avoids stealing render/flight-control CPU time.
        cv2.setNumThreads(1)
        for key, value in dict(rgb_topic='/follow_camera/image',
                              depth_topic='/follow_camera/depth_image',
                              info_topic='/follow_camera/camera_info',
                              max_image_age_s=0.2, min_marker_pixels=12,
                              max_depth_skew_s=0.08, max_depth=45.0,
                              min_depth_pixels=8).items():
            self.declare_parameter(key, value)
        self.info = None
        self.depth_frames = deque(maxlen=5)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Vector3Stamped, '/target/bearing_camera', 5)
        self.observation = self.create_publisher(String, '/target/observation', 10)
        self.debug = self.create_publisher(Image, '/target/debug_image', 1)
        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value,
                                 self.calibration, SENSOR)
        self.create_subscription(Image, self.get_parameter('rgb_topic').value,
                                 self.images, QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Image, self.get_parameter('depth_topic').value,
                                 self.depth, SENSOR)

    def calibration(self, msg):
        self.info = msg

    def depth(self, msg):
        if self.depth_frames and self.stamp(msg) < self.stamp(self.depth_frames[-1]):
            self.depth_frames.clear()
        self.depth_frames.append(msg)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

    def median_depth(self, stamp, mask):
        """Median over marker pixels in a recent, registered depth image."""
        if not self.depth_frames:
            return float('nan')
        depth = min(self.depth_frames, key=lambda msg: abs(self.stamp(msg)-stamp))
        age = self.get_clock().now().nanoseconds*1e-9 - self.stamp(depth)
        if (abs(self.stamp(depth)-stamp) > self.get_parameter('max_depth_skew_s').value
                or not 0 <= age <= self.get_parameter('max_image_age_s').value
                or depth.encoding not in ('32FC1', '16UC1')):
            return float('nan')
        try:
            d = self.bridge.imgmsg_to_cv2(depth, 'passthrough').astype(float)
        except Exception as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5)
            return float('nan')
        if d.shape != mask.shape:
            return float('nan')
        if depth.encoding == '16UC1':
            d *= 0.001  # uint16 millimetres -> metres; float32 is already metres
        valid = mask & np.isfinite(d) & (d > 0.3) & (d < self.get_parameter('max_depth').value)
        if np.count_nonzero(valid) < self.get_parameter('min_depth_pixels').value:
            return float('nan')
        return float(np.median(d[valid]))

    def images(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        age = self.get_clock().now().nanoseconds*1e-9 - stamp
        data = dict(stamp_s=stamp,
                    stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
                    image_age_s=age, visible=False,
                    reason='no_calibration', frame_id=msg.header.frame_id,
                    width=msg.width, height=msg.height)
        try:
            if self.info is None:
                return
            if not 0 <= age <= self.get_parameter('max_image_age_s').value:
                data['reason'] = 'stale_image'
                return
            k = self.info.k
            if k[0] <= 0 or k[4] <= 0 or (msg.width, msg.height) != (self.info.width, self.info.height):
                data['reason'] = 'invalid_calibration'
                return
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 120, 80), (10, 255, 255))
            mask |= cv2.inRange(hsv, (170, 120, 80), (179, 255, 255))
            n, labels, stats, centres = cv2.connectedComponentsWithStats(mask)
            candidates = [i for i in range(1, n)
                          if stats[i, cv2.CC_STAT_AREA] >= self.get_parameter('min_marker_pixels').value]
            data['reason'] = 'no_marker' if not candidates else 'ambiguous_markers'
            if len(candidates) == 1:
                index = candidates[0]
                u, v = map(float, centres[index])
                x, y = (u-k[2])/k[0], (v-k[5])/k[4]
                ray = Vector3Stamped()
                ray.header = msg.header
                # Mixed bearing/range payload, not a geometric unit vector.
                ray.header.frame_id = 'follow_camera_optical'
                range_m = self.median_depth(stamp, labels == index)
                ray.vector.x, ray.vector.y, ray.vector.z = x, y, range_m
                self.pub.publish(ray)
                data.update(visible=True, reason='detected', u_px=u, v_px=v,
                            error_u_px=u-k[2], error_v_px=v-k[5], x_normalized=x,
                            y_normalized=y, range_m=range_m if math.isfinite(range_m) else None,
                            range_valid=math.isfinite(range_m),
                            area_px=int(stats[index, cv2.CC_STAT_AREA]),
                            bbox_xywh_px=[int(value) for value in stats[index, :4]])
                bgr = bgr.copy()
                bgr[labels == index] = (0, 255, 0)
            if self.debug.get_subscription_count():
                debug = self.bridge.cv2_to_imgmsg(bgr, 'bgr8')
                debug.header = msg.header
                self.debug.publish(debug)
        except Exception as exc:
            data.update(reason='image_error', error=str(exc))
            self.get_logger().error(str(exc), throttle_duration_sec=5)
        finally:
            # Negative detections matter for visibility statistics, too.
            # SIGINT may invalidate ROS during image processing. Do not mask
            # that normal shutdown with an invalid-context publish exception.
            if self.context.ok():
                try:
                    self.observation.publish(String(data=json.dumps(data, allow_nan=False)))
                except RCLError:
                    if self.context.ok():
                        raise


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

## [drone_follow/monocular.py](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/drone_follow/monocular.py)

```python
"""Range-free image centring; no ROS, simulator state or paper-specific law.

Optical axes are right/down/forward. The vertical mapping assumes level hover
with a forward camera. It is not a global IBVS stability guarantee.
"""
import math


def centring_command(x, y, yaw_gain, vertical_gain, max_yaw_rate, max_vertical_speed):
    """Normalized pixel error -> NED down speed [m/s], yaw rate [rad/s].

    Target right -> positive NED yaw. Target below -> positive NED down.
    No horizontal translation or distance estimate is generated.
    """
    if not all(math.isfinite(v) for v in (x, y, yaw_gain, vertical_gain,
                                         max_yaw_rate, max_vertical_speed)):
        raise ValueError('Nonfinite input')
    if min(yaw_gain, vertical_gain) < 0 or min(max_yaw_rate, max_vertical_speed) <= 0:
        raise ValueError('Gains must be nonnegative and limits positive')
    down = max(-max_vertical_speed, min(max_vertical_speed, vertical_gain*y))
    yaw = max(-max_yaw_rate, min(max_yaw_rate, yaw_gain*math.atan(x)))
    return down, yaw
```

## [config/follow.yaml](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/config/follow.yaml)

```yaml
# Run and controller knobs. Camera resolution/extrinsics remain in the model SDF.
baseline:
  px4_dir: ~/PX4-Autopilot
  ros_domain_id: 42
  agent_port: 8888
  ready_timeout_s: 90.0
  stale_timeout_s: 3.0
  takeoff_altitude_m: 3.0
  follower_yaw_deg: 12.0  # initial scene heading offset; try 10.0 for centring
  flight_timeout_s: 90.0
  scenario: observe  # observe, hover, centre; centre enables the selected follower mode
  duration_s: 0.0    # 0 = continue until you press Ctrl+C
  auto_land: false  # no scheduled landing; PX4's own failsafes remain active
  keep_open: true   # report completion/failure without shutting down the simulation
  record: false     # opt in with --record for CSV, images, source snapshots and ULogs
  headless: false
  viewer: true       # camera window; --headless disables it unless --viewer is given
  viewer_fps: 60.0   # 3 refresh slots per 20 Hz camera frame; no invented measurements
  viewer_stale_s: 0.5  # freeze warning after this many wall seconds without a frame
  viewer_pair_wait_s: 0.08  # max age of a buffered image/detection pair, wall seconds
detector:
  ros__parameters:
    use_sim_time: true
    rgb_topic: /follow_camera/image
    depth_topic: /follow_camera/depth_image
    info_topic: /follow_camera/camera_info
    max_image_age_s: 0.2
    min_marker_pixels: 12
    max_depth_skew_s: 0.08
    max_depth: 45.0
    min_depth_pixels: 8
follower:
  ros__parameters:
    use_sim_time: true
    px4_namespace: /px4_1
    mode: centre              # approach opts into depth-based forward motion
    yaw_gain: 0.8              # rad/s per rad of horizontal bearing error
    vertical_gain: 0.6         # m/s per normalized vertical error, level hover
    max_yaw_rate: 0.3          # rad/s
    max_vertical_speed: 0.4    # m/s
    target_range_m: 10.0
    min_range_m: 2.0
    max_fwd_speed: 2.0
    fwd_gain: 0.5
    range_margin: 1.0
    brake_accel: 1.0           # assumed braking acceleration, m/s^2
    latency: 0.4               # braking-envelope reaction time, s
    max_measurement_age_s: 0.3
    minimum_detections: 6
leader:
  ros__parameters:
    use_sim_time: true
    px4_namespace: /px4_2
```

## [config/bridge.yaml](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/config/bridge.yaml)

```yaml
- ros_topic_name: /clock
  gz_topic_name: /world/follow_world/clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
- ros_topic_name: /follow_camera/image
  gz_topic_name: /follow_camera/image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS
- ros_topic_name: /follow_camera/depth_image
  gz_topic_name: /follow_camera/depth_image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS
- ros_topic_name: /follow_camera/camera_info
  gz_topic_name: /follow_camera/camera_info
  ros_type_name: sensor_msgs/msg/CameraInfo
  gz_type_name: gz.msgs.CameraInfo
  direction: GZ_TO_ROS
```

## [models/x500_follower/model.sdf](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/models/x500_follower/model.sdf)

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="x500_follower">
    <!-- Inherit PX4's real X500 dynamics, motors, IMU, GPS and meshes. -->
    <include merge="true"><uri>model://x500</uri></include>
    <link name="follow_camera_link">
      <pose relative_to="base_link">0.30 0 0.10 0 0 0</pose>
      <inertial><mass>0.01</mass><inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial>
      <sensor name="follow_camera" type="rgbd_camera">
        <always_on>true</always_on><update_rate>20</update_rate>
        <topic>/follow_camera</topic>
        <camera>
          <horizontal_fov>1.3962634</horizontal_fov>
          <image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>50</far></clip>
          <depth_camera><clip><near>0.1</near><far>50</far></clip></depth_camera>
          <camera_info_topic>/follow_camera/camera_info</camera_info_topic>
        </camera>
      </sensor>
    </link>
    <joint name="follow_camera_joint" type="fixed"><parent>base_link</parent><child>follow_camera_link</child></joint>
  </model>
</sdf>
```

## [models/x500_follower/model.config](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/models/x500_follower/model.config)

```xml
<?xml version="1.0"?>
<model><name>X500 follower RGB-D</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><author><name>Example</name></author><description>PX4 X500 with a forward RGB-D camera for centring and optional stand-off approach.</description></model>
```
