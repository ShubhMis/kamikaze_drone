"""Target drone: hold its hover point, then optionally fly a slow figure eight."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as SENSOR
from std_srvs.srv import SetBool
from px4_msgs.msg import VehicleOdometry, VehicleStatus, OffboardControlMode, TrajectorySetpoint


class Leader(Node):
    def __init__(self):
        super().__init__('leader')
        self.declare_parameter('px4_namespace', '/px4_2')
        ns = self.get_parameter('px4_namespace').value.rstrip('/')
        self.odom_msg = self.status_msg = None
        self.odom_wall = self.status_wall = 0.0
        self.origin = None
        self.active = False
        self.start = 0.0
        self.last_time = None
        self.reset_counter = None
        self.fault = False
        self.create_subscription(VehicleOdometry, ns+'/fmu/out/vehicle_odometry', self.odom, SENSOR)
        self.create_subscription(VehicleStatus, ns+'/fmu/out/vehicle_status', self.status, SENSOR)
        self.mode_pub = self.create_publisher(OffboardControlMode, ns+'/fmu/in/offboard_control_mode', 10)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, ns+'/fmu/in/trajectory_setpoint', 10)
        self.create_service(SetBool, '/target/move', self.move)
        self.create_timer(0.05, self.tick)

    def offboard(self):
        s = self.status_msg
        return s is not None and s.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and s.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def odom(self, msg):
        if self.reset_counter is not None and self.reset_counter != msg.reset_counter:
            self.active = False
            self.fault = self.offboard()
            self.origin = None
        self.reset_counter = msg.reset_counter
        self.odom_msg, self.odom_wall = msg, time.monotonic()

    def status(self, msg):
        self.status_msg, self.status_wall = msg, time.monotonic()

    def healthy(self):
        return (not self.fault and self.odom_msg is not None
                and self.odom_msg.pose_frame == VehicleOdometry.POSE_FRAME_NED
                and np.all(np.isfinite(self.odom_msg.position))
                and time.monotonic()-self.odom_wall < 0.3
                and time.monotonic()-self.status_wall < 2.0)

    def move(self, req, res):
        if not self.healthy() or not self.offboard():
            res.success, res.message = False, 'Target must be hovering in armed Offboard mode'
            return res
        self.active = req.data
        self.origin = np.asarray(self.odom_msg.position, float)
        self.start = self.get_clock().now().nanoseconds*1e-9
        res.success = True
        res.message = 'Slow path enabled' if req.data else 'Holding current position'
        return res

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last_time is not None and now < self.last_time:
            self.fault = True
        self.last_time = now
        if not self.healthy():
            self.active = False
            return
        # Never overwrite PX4's own takeoff/landing trajectory setpoints.
        if (self.status_msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and self.status_msg.nav_state not in (
                    VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
                    VehicleStatus.NAVIGATION_STATE_POSCTL)):
            self.active = False
            return
        if not self.offboard() or self.origin is None:
            self.active = False
            self.origin = np.asarray(self.odom_msg.position, float)
        point = self.origin.copy()
        if self.active:
            t = now-self.start
            # NED north/east offsets, <0.5 m/s combined. Starts at the hover point.
            point += np.array([2.0*math.sin(0.16*t), 4.0*math.sin(0.08*t), 0.0])
        mode = OffboardControlMode()
        mode.timestamp = int(now*1e6)
        mode.position = True
        sp = TrajectorySetpoint()
        sp.timestamp = mode.timestamp
        sp.position = list(map(float, point))
        sp.velocity = [float('nan')]*3
        sp.acceleration = [float('nan')]*3
        sp.jerk = [float('nan')]*3
        sp.yaw, sp.yawspeed = float('nan'), 0.0
        self.mode_pub.publish(mode)
        self.sp_pub.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = Leader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
