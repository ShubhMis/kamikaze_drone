"""Monocular red-marker measurement. No depth or vehicle/target pose input."""
import json
import math
import cv2
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
                              info_topic='/follow_camera/camera_info',
                              max_image_age_s=0.2, min_marker_pixels=12).items():
            self.declare_parameter(key, value)
        self.info = None
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Vector3Stamped, '/target/bearing_camera', 5)
        self.observation = self.create_publisher(String, '/target/observation', 10)
        self.debug = self.create_publisher(Image, '/target/debug_image', 1)
        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value,
                                 self.calibration, SENSOR)
        self.create_subscription(Image, self.get_parameter('rgb_topic').value,
                                 self.images, QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))

    def calibration(self, msg):
        self.info = msg

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
                # This unit ray has no metric range; frame convention is explicit.
                ray.header.frame_id = 'follow_camera_optical'
                norm = math.sqrt(x*x+y*y+1)
                ray.vector.x, ray.vector.y, ray.vector.z = x/norm, y/norm, 1/norm
                self.pub.publish(ray)
                data.update(visible=True, reason='detected', u_px=u, v_px=v,
                            error_u_px=u-k[2], error_v_px=v-k[5], x_normalized=x,
                            y_normalized=y, area_px=int(stats[index, cv2.CC_STAT_AREA]),
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
