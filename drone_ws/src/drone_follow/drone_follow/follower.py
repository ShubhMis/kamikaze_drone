"""Monocular follower — two modes selectable via 'mode' ROS 2 parameter.

mode='centre'    (default):
    Yaw+vertical centring only. No forward approach. Uses monocular.py.
    Identical to the original behaviour.

mode='intercept':
    PNG-IBVS guidance from arXiv:2409.17497v2 (Yan et al. 2025).
    Commands a full NED velocity vector toward the target using Proportional
    Navigation on the Line-of-Sight angles derived from the monocular bearing.
    No depth sensor or range estimate needed.
    When detections are lost, following latches off and state is reset.
"""
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
from .png_ibvs import intercept_command, make_state, rotation


class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        defaults = dict(
            # ── Shared ──────────────────────────────────────────────────────
            px4_namespace='/px4_1',
            mode='centre',                    # 'centre' | 'intercept'
            max_measurement_age_s=0.3,
            minimum_detections=6,
            # ── Centre mode (monocular.py) ──────────────────────────────────
            yaw_gain=0.8,
            vertical_gain=0.7,
            max_yaw_rate=0.3,
            max_vertical_speed=0.4,
            # ── Intercept mode (png_ibvs.py) ────────────────────────────────
            approach_speed=3.0,               # closing speed v_d (m/s)
            png_gain_y=3.0,                   # K_y — paper recommends 3
            png_gain_z=3.0,                   # K_z
            fov_kp=0.03,                      # yaw PD proportional gain
            fov_kd=0.01,                      # yaw PD derivative gain
            fov_ka=2.0,                       # vertical FOV correction gain
            max_speed=4.0,                    # hard velocity magnitude limit (m/s)
            max_vertical_speed_intercept=2.0, # hard NED-down limit in intercept mode
            camera_mount_q=[1.0, 0.0, 0.0, 0.0],  # identity = forward-facing camera
        )
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = lambda key: self.get_parameter(key).value

        # Validate centring command at startup (unchanged from original).
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

        # PNG-IBVS state — created fresh each time following is enabled.
        # None when in centre mode or before first enable.
        self.png_state = None

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
        if (msg.header.frame_id != 'follow_camera_optical' or v.z <= 0
                or not all(math.isfinite(x) for x in (v.x, v.y, v.z))
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
        return (self.target_count >= self.p('minimum_detections')
                and 0 <= self.now()-(t.sec+t.nanosec*1e-9) < self.p('max_measurement_age_s')
                and time.monotonic()-self.target_wall < self.p('max_measurement_age_s'))

    def enable(self, req, res):
        if not req.data:
            # Disable: clear state regardless of mode.
            self.enabled = False
            self.png_state = None
            res.success, res.message = True, 'Following disabled; zero velocity requested'
        elif self.healthy() and self.in_offboard() and self.fresh():
            self.enabled = True
            mode = self.p('mode')
            if mode == 'intercept':
                # Fresh state every enable — don't carry over stale LOS angles
                # from a previous engagement (which could have been in a completely
                # different direction).
                self.png_state = make_state()
                res.success, res.message = True, 'Intercept enabled — PNG-IBVS active'
            else:
                self.png_state = None
                res.success, res.message = True, 'Centring enabled; horizontal velocity remains zero'
        else:
            res.success, res.message = False, 'Need healthy armed Offboard hover and six fresh detections'
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
            self.png_state = None   # reset integration on clock gap
        self.last_tick = now

        # Default output: zero velocity, zero yaw rate.
        vel_ned = np.zeros(3)   # [north, east, down] m/s
        yaw_rate = 0.0
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
                # Target lost — latch off and wipe integration state.
                self.enabled = False
                self.png_state = None
                mode_str = self.p('mode')
                state = f'TARGET LOST: {mode_str} disabled; re-enable after reacquisition'

            elif self.enabled:
                # ── Bearing from detector ────────────────────────────────────
                v = self.target_msg.vector
                # bx = normalised horizontal bearing (right positive)
                # by = normalised vertical bearing   (down positive)
                # vector.z is 1/norm so v.x/v.z and v.y/v.z give bx, by.
                bx = v.x / v.z
                by = v.y / v.z

                mode_str = self.p('mode')

                if mode_str == 'intercept' and self.png_state is not None:
                    # ── INTERCEPT MODE — PNG-IBVS ────────────────────────────
                    # Build the rotation matrix from the drone's current attitude.
                    # odom_msg.q is [w, x, y, z] in PX4 Hamilton convention.
                    try:
                        R = rotation(self.odom_msg.q)
                    except ValueError as exc:
                        self.get_logger().warning(str(exc), throttle_duration_sec=2)
                        state = 'INTERCEPT FAULT: bad attitude quaternion'
                    else:
                        # Current NED velocity — needed for vertical FOV correction.
                        own_vel = np.asarray(self.odom_msg.velocity, dtype=float)

                        # Time step since last tick (clamped for numerical safety).
                        dt = float(np.clip(now - (self.png_state.prev_time
                                                   if self.png_state.prev_time else now - 0.05),
                                           0.005, 0.2))
                        self.png_state.prev_time = now

                        # Call the full PNG-IBVS guidance law.
                        vel_ned, yaw_rate = intercept_command(
                            bx=bx, by=by,
                            state=self.png_state,
                            R_body_ned=R,
                            own_vel_ned=own_vel,
                            dt=dt,
                            approach_speed=self.p('approach_speed'),
                            Ky=self.p('png_gain_y'),
                            Kz=self.p('png_gain_z'),
                            kp=self.p('fov_kp'),
                            kd=self.p('fov_kd'),
                            ka=self.p('fov_ka'),
                            max_speed=self.p('max_speed'),
                            max_yaw_rate=self.p('max_yaw_rate'),
                            max_vertical=self.p('max_vertical_speed_intercept'),
                            mount_q=self.p('camera_mount_q'),
                        )
                        state = 'INTERCEPTING'

                else:
                    # ── CENTRE MODE — original monocular.py behaviour ─────────
                    down_cmd, yaw_rate = centring_command(
                        bx, by,
                        self.p('yaw_gain'), self.p('vertical_gain'),
                        self.p('max_yaw_rate'), self.p('max_vertical_speed'),
                    )
                    vel_ned[2] = down_cmd   # only vertical, north and east stay 0
                    state = 'CENTRING / NO FORWARD APPROACH'

            # ── Build and publish PX4 setpoint ──────────────────────────────
            # OffboardControlMode is the heartbeat PX4 requires to stay in
            # Offboard mode. If it stops for > COM_OF_LOSS_T seconds, PX4
            # executes its configured Offboard-loss failsafe.
            mode = OffboardControlMode(timestamp=int(now*1e6), velocity=True)
            sp = TrajectorySetpoint()
            sp.timestamp = mode.timestamp
            sp.position = [float('nan')] * 3       # not used — velocity control
            sp.acceleration = [float('nan')] * 3   # not used
            sp.jerk = [float('nan')] * 3            # not used
            sp.velocity = [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])]
            sp.yaw = float('nan')                   # yaw is controlled via yawspeed
            sp.yawspeed = float(yaw_rate)
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
            velocity_ned_mps=[float(v) for v in vel_ned], yaw_rate_rps=yaw_rate))))


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
