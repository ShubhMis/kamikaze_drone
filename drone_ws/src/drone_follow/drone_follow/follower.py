"""Single-controller monocular PNG-IBVS follower.

The active runtime has one visual guidance path: ``intercept_command`` from
``png_ibvs.py``.  There is no centre/intercept selector and no fallback visual
controller. Enable, target-loss and PX4-health states only decide whether that
one controller may run. Loss always clears its complete controller history. If
automatic reacquisition is enabled, the node holds zero velocity until a fresh
detection streak is available, then starts a new controller state.

This node still uses PX4's NED velocity/yaw-rate Offboard interface.  The exact
relationship between this approximation and Yan et al. (2025), including the
paper's body-rate/lift output, is recorded in PAPER_CONTROLLER_CONTRACT.md.
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
from .configuration import CONTROLLER_KEYS, validate_controller_parameters
from .png_ibvs import intercept_command, make_state, rotation
from .state_history import VehicleStateHistory, odometry_sample_time


class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        defaults = dict(
            px4_namespace='/px4_1',
            max_measurement_age_s=0.3,
            minimum_detections=6,
            auto_reacquire=True,
            reacquire_timeout_s=5.0,
            state_history_s=2.0,
            max_state_extrapolation_s=0.03,
            max_odometry_sample_age_s=0.1,
            # Current paper-inspired velocity-interface reconstruction.
            approach_speed=3.0,
            max_yaw_rate=0.3,
            png_gain_y=3.0,
            png_gain_z=3.0,                   # K_z
            fov_kp=0.03,
            fov_kd=0.01,
            fov_ka=2.0,
            max_speed=4.0,
            max_vertical_speed=2.0,
            camera_mount_q=[1.0, 0.0, 0.0, 0.0],
        )
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = lambda key: self.get_parameter(key).value

        validate_controller_parameters({key: self.p(key) for key in CONTROLLER_KEYS})

        ns = self.p('px4_namespace')
        self.odom_msg = self.status_msg = None
        self.odom_wall = self.status_wall = self.target_wall = 0.0
        self.target_msg = None
        self.target_count = 0
        self.reset_counter = None
        self.enabled = self.fault = False
        self.pursuit_requested = False
        self.lost_since = None
        self.last_tick = None
        self.state_text = ''

        # Created fresh for every engagement; never reused after a reset.
        self.png_state = None
        self.state_history = VehicleStateHistory(
            duration_s=self.p('state_history_s'),
            max_extrapolation_s=self.p('max_state_extrapolation_s'))
        self.last_processed_target_stamp = None
        self.last_aligned_state_stamp = None
        self.last_command_velocity = np.zeros(3)
        self.last_command_yaw_rate = 0.0

        self.create_subscription(VehicleOdometry, ns+'/fmu/out/vehicle_odometry', self.odom, SENSOR)
        self.create_subscription(VehicleStatus, ns+'/fmu/out/vehicle_status', self.status, SENSOR)
        self.create_subscription(Vector3Stamped, '/target/bearing_camera', self.target, 5)
        self.offboard_pub = self.create_publisher(OffboardControlMode, ns+'/fmu/in/offboard_control_mode', 10)
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
        receive_time = self.now()
        if self.reset_counter is not None and self.reset_counter != msg.reset_counter:
            self.fault = self.fault or self.in_offboard()
            self.reset_guidance(clear_target_count=True)
            self.state_history.clear()
        self.reset_counter = msg.reset_counter
        self.odom_msg, self.odom_wall = msg, time.monotonic()
        try:
            stamp = odometry_sample_time(
                receive_time, msg.timestamp, msg.timestamp_sample,
                self.p('max_odometry_sample_age_s'))
            self.state_history.add(stamp, msg.q, msg.velocity)
        except ValueError as exc:
            self.state_history.clear()
            self.get_logger().warning(str(exc), throttle_duration_sec=2)

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

    def reset_guidance(self, clear_target_count=False, keep_request=False):
        """Stop control and discard state; optionally retain visual-loss intent."""
        self.enabled = False
        self.png_state = None
        self.last_processed_target_stamp = None
        self.last_aligned_state_stamp = None
        self.last_command_velocity = np.zeros(3)
        self.last_command_yaw_rate = 0.0
        if not keep_request:
            self.pursuit_requested = False
            self.lost_since = None
        if clear_target_count:
            self.target_count = 0

    def start_guidance(self):
        """Start one engagement with no state carried from an earlier target."""
        self.enabled = True
        self.pursuit_requested = True
        self.lost_since = None
        self.png_state = make_state()
        self.last_processed_target_stamp = None
        self.last_aligned_state_stamp = None
        self.last_command_velocity = np.zeros(3)
        self.last_command_yaw_rate = 0.0

    def begin_reacquisition(self, now):
        """Handle visual loss without preserving any controller history."""
        keep_request = self.p('auto_reacquire')
        self.reset_guidance(clear_target_count=True, keep_request=keep_request)
        if keep_request:
            self.lost_since = float(now)

    def reacquiring(self):
        return (self.pursuit_requested and not self.enabled
                and self.lost_since is not None)

    def reacquisition_expired(self, now):
        return (self.reacquiring()
                and now-self.lost_since > self.p('reacquire_timeout_s'))

    def target_stamp(self):
        if self.target_msg is None:
            return None
        stamp = self.target_msg.header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def has_unprocessed_target(self):
        stamp = self.target_stamp()
        return (stamp is not None
                and (self.last_processed_target_stamp is None
                     or stamp > self.last_processed_target_stamp + 1e-9))

    def enable(self, req, res):
        if not req.data:
            self.reset_guidance()
            res.success, res.message = True, 'Following disabled; zero velocity requested'
        elif self.healthy() and self.in_offboard() and self.fresh():
            # Never carry LOS or derivative history across engagements.
            self.start_guidance()
            res.success, res.message = True, 'PNG-IBVS pursuit enabled'
        else:
            res.success = False
            res.message = ('Need healthy armed Offboard hover and '
                           f'{self.p("minimum_detections")} fresh detections')
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
            self.reset_guidance(clear_target_count=True)
            self.state_history.clear()
        self.last_tick = now

        # Default output: zero velocity, zero yaw rate. While actively pursuing,
        # duplicate timer ticks reuse the last command without reprocessing the
        # same camera observation.
        vel_ned = np.zeros(3)   # [north, east, down] m/s
        yaw_rate = 0.0
        processed_observation = False
        reacquired_observation = False
        state = 'READY / ZERO VELOCITY'

        if not self.healthy():
            self.reset_guidance()
            state = 'STATE INVALID: no Offboard heartbeat'
        elif not self.may_stream():
            self.reset_guidance()
            state = 'AUTOPILOT MANOEUVRE: external setpoints paused'
        else:
            if not self.in_offboard():
                self.reset_guidance()

            if self.enabled and not self.fresh():
                # Never propagate LOS/derivative state across a visual gap.
                self.begin_reacquisition(now)
                state = ('TARGET LOST: zero velocity; rebuilding detections'
                         if self.reacquiring()
                         else 'TARGET LOST: pursuit disabled')

            if self.reacquisition_expired(now):
                self.reset_guidance(clear_target_count=True)
                state = 'REACQUISITION TIMEOUT: pursuit disabled'
            elif self.reacquiring() and self.fresh():
                # A complete fresh streak is available. Start a brand-new PNG
                # state and process the newest observation below in this tick.
                self.start_guidance()
                reacquired_observation = True
            elif self.reacquiring():
                state = 'TARGET LOST: zero velocity; rebuilding detections'

            if self.enabled:
                # ── Bearing from detector ────────────────────────────────────
                v = self.target_msg.vector
                # bx = normalised horizontal bearing (right positive)
                # by = normalised vertical bearing   (down positive)
                # vector.z is 1/norm so v.x/v.z and v.y/v.z give bx, by.
                bx = v.x / v.z
                by = v.y / v.z

                if self.png_state is not None and self.has_unprocessed_target():
                    measurement_stamp = self.target_stamp()
                    self.last_aligned_state_stamp = None
                    aligned_state = self.state_history.at(measurement_stamp)
                    if aligned_state is None:
                        vel_ned = self.last_command_velocity.copy()
                        yaw_rate = self.last_command_yaw_rate
                        state = 'WAITING FOR ODOMETRY AT IMAGE TIME'
                    else:
                        # Build the rotation from attitude interpolated at image
                        # exposure time, rather than from the newest odometry.
                        self.last_aligned_state_stamp = aligned_state.stamp_s
                        try:
                            R = rotation(aligned_state.q)
                        except ValueError as exc:
                            self.get_logger().warning(str(exc), throttle_duration_sec=2)
                            self.reset_guidance()
                            state = 'INTERCEPT FAULT: bad attitude quaternion'
                        else:
                            own_vel = aligned_state.velocity
                            dt = (measurement_stamp - self.png_state.prev_time
                                  if self.png_state.prev_time else 0.05)
                            self.png_state.prev_time = measurement_stamp

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
                                max_vertical=self.p('max_vertical_speed'),
                                mount_q=self.p('camera_mount_q'),
                            )
                            self.last_processed_target_stamp = measurement_stamp
                            self.last_command_velocity = np.asarray(vel_ned, dtype=float).copy()
                            self.last_command_yaw_rate = float(yaw_rate)
                            processed_observation = True
                            state = ('TARGET REACQUIRED / INTERCEPTING'
                                     if reacquired_observation else 'INTERCEPTING')
                elif self.png_state is not None:
                    vel_ned = self.last_command_velocity.copy()
                    yaw_rate = self.last_command_yaw_rate
                    state = 'INTERCEPTING'
                else:
                    # This is an invariant failure, not another controller path.
                    self.reset_guidance()
                    state = 'CONTROLLER STATE INVALID: pursuit disabled'

            # ── Build and publish PX4 setpoint ──────────────────────────────
            # OffboardControlMode is the heartbeat PX4 requires to stay in
            # Offboard mode. If it stops for > COM_OF_LOSS_T seconds, PX4
            # executes its configured Offboard-loss failsafe.
            offboard = OffboardControlMode(timestamp=int(now*1e6), velocity=True)
            sp = TrajectorySetpoint()
            sp.timestamp = offboard.timestamp
            sp.position = [float('nan')] * 3       # not used — velocity control
            sp.acceleration = [float('nan')] * 3   # not used
            sp.jerk = [float('nan')] * 3            # not used
            sp.velocity = [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])]
            sp.yaw = float('nan')                   # yaw is controlled via yawspeed
            sp.yawspeed = float(yaw_rate)
            self.offboard_pub.publish(offboard)
            self.sp_pub.publish(sp)
        if state != self.state_text:
            self.get_logger().info(state)
            self.state_text = state
        self.diag.publish(String(data=state))
        measurement_stamp = self.target_stamp()
        age = None if measurement_stamp is None else now - measurement_stamp
        reacquisition_age = (now-self.lost_since if self.reacquiring() else None)
        self.metrics.publish(String(data=json.dumps(dict(stamp_s=now, state=state,
            controller='png_ibvs_velocity', enabled=self.enabled,
            pursuit_requested=self.pursuit_requested,
            reacquiring=self.reacquiring(),
            reacquired_observation=reacquired_observation,
            reacquisition_age_s=reacquisition_age,
            measurement_age_s=age, fresh=self.fresh(),
            measurement_stamp_s=measurement_stamp,
            observation_processed=processed_observation,
            aligned_state_stamp_s=self.last_aligned_state_stamp,
            state_alignment_error_s=(None if measurement_stamp is None
                                     or self.last_aligned_state_stamp is None
                                     else self.last_aligned_state_stamp-measurement_stamp),
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
