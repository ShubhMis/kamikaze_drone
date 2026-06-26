#!/usr/bin/env python3
"""
px4_ros2_bridge.py
════════════════════════════════════════════════════════════════════
ROS2 Jazzy bridge node: Gazebo Harmonic ↔ ROS2

Bridges Gazebo topics to ROS2 so you can:
  • View the drone camera in RViz2
  • Record camera feed with ros2 bag record
  • Access flight data from other ROS2 nodes

Bridges:
  Gazebo /drone/camera/image  →  ROS2 sensor_msgs/Image  on /drone/camera/image
  Gazebo /imu                 →  ROS2 sensor_msgs/Imu    on /imu
  Gazebo /air_pressure        →  ROS2 sensor_msgs/FluidPressure on /air_pressure

Prerequisites (already installed if you followed README):
  sudo apt install ros-jazzy-ros-gz-bridge

Usage:
  # Terminal 1: gz sim px4_drone_world.sdf
  # Terminal 2: source /opt/ros/jazzy/setup.bash
  #             python3 px4_ros2_bridge.py
  # Terminal 3: rviz2   (add Image display, topic /drone/camera/image)
════════════════════════════════════════════════════════════════════
"""

import subprocess
import sys
import os
import signal

# ─────────────────────────────────────────────────────────────────
#  Bridge config: list of (gz_topic, gz_type, ros_topic, ros_type)
# ─────────────────────────────────────────────────────────────────
BRIDGES = [
    # Camera image
    (
        "/world/px4_drone_world/model/drone/model/mono_cam/link/camera_link/sensor/camera/image",
        "gz.msgs.Image",
        "/drone/camera/image",
        "sensor_msgs/msg/Image",
        "GZ_TO_ROS",
    ),
    # IMU
    (
        "/imu",
        "gz.msgs.IMU",
        "/imu",
        "sensor_msgs/msg/Imu",
        "GZ_TO_ROS",
    ),
    # Barometer / fluid pressure
    (
        "/air_pressure",
        "gz.msgs.FluidPressure",
        "/air_pressure",
        "sensor_msgs/msg/FluidPressure",
        "GZ_TO_ROS",
    ),
]


def build_bridge_args():
    """
    Build the --ros-args parameter string for ros_gz_bridge.
    Each bridge entry: gz_topic@gz_type@direction@ros_type
    """
    args = []
    for gz_topic, gz_type, ros_topic, ros_type, direction in BRIDGES:
        if direction == "GZ_TO_ROS":
            pair = f"[{gz_type}]"     # gz → ros  uses  [ prefix
        elif direction == "ROS_TO_GZ":
            pair = f"]{gz_type}["    # ros → gz  uses  ] prefix
        else:
            pair = f"@{gz_type}"     # bidirectional

        args.append(
            f"{ros_topic}@{ros_type}{pair}"
            if direction != "GZ_TO_ROS" else
            f"{ros_topic}@{ros_type}[{gz_type}"
        )
    return args


def main():
    print("═" * 62)
    print("  PX4 Gazebo ↔ ROS2 Bridge")
    print("  ROS distro : jazzy")
    print("  Bridges    :")
    for _, _, ros_topic, ros_type, _ in BRIDGES:
        print(f"    {ros_topic:35s} → {ros_type}")
    print("═" * 62)
    print()

    # Source ROS2 if not already done
    ros_setup = "/opt/ros/jazzy/setup.bash"
    if not os.path.exists(ros_setup):
        print(f"[ERROR] ROS2 Jazzy not found at {ros_setup}")
        print("        Install with: sudo apt install ros-jazzy-desktop")
        sys.exit(1)

    # Build bridge parameter pairs
    bridge_pairs = build_bridge_args()

    cmd = (
        ["bash", "-c",
         f"source {ros_setup} && ros2 run ros_gz_bridge parameter_bridge " +
         " ".join(bridge_pairs)]
    )

    print("[Bridge] Launching ros_gz_bridge with:")
    for p in bridge_pairs:
        print(f"  {p}")
    print()
    print("[Bridge] Press Ctrl+C to stop.\n")

    proc = subprocess.Popen(cmd)

    def _on_signal(sig, frame):
        print("\n[Bridge] Stopping...")
        proc.terminate()
        sys.exit(0)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        proc.wait()
    except KeyboardInterrupt:
        proc.terminate()


if __name__ == "__main__":
    main()
