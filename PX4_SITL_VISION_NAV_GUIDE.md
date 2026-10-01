# End-to-End Guide: From PX4 Clone to Camera-Based Autonomous Navigation

This guide walks you through setting up **PX4 SITL (Software-In-The-Loop)**, **Gazebo Sim**, and **ROS 2** from scratch, with all optional customization points (world scenery, physics, cameras, multi-vehicle spawning) and how to plug in your custom vision navigation algorithm.

---

## Architecture Overview

```
                      +---------------------------------------+
                      |         Gazebo Sim (Physics)          |
                      |  - World Scenery (.sdf) & Physics     |
                      |  - Camera Sensor (RGB / Depth)        |
                      |  - Moving Target / Dynamic Objects    |
                      +-------+-----------------------+-------+
                              |                       |
               Simulated IMU  |                       | /camera/image
               & Motor Forces |                       | (gz.msgs.Image)
                              v                       v
                      +---------------+       +---------------+
                      |   PX4 SITL    |       | ros_gz_bridge |
                      | (Flight Core) |       +-------+-------+
                      +-------+-------+               |
                              | uORB                  | sensor_msgs/Image
                              v                       |
                      +---------------+               |
                      | MicroXRCEAgent|               |
                      +-------+-------+               |
                              | px4_msgs              |
                              v                       v
               +----------------------------------------------+
               |        Your Custom Python Algorithm          |
               |  1. Ingests camera image via OpenCV          |
               |  2. Computes steering/target error           |
               |  3. Streams Offboard velocity setpoints      |
               +----------------------------------------------+
```

---

## Step 1: Clone PX4-Autopilot & Install System Dependencies

### 1.1 Clone the Repository
Clone PX4 with its submodules. Using a stable release branch (e.g. `release/1.15`) is strongly recommended:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.15
cd PX4-Autopilot
```

> If you already cloned without `--recursive`, run:
> ```bash
> git submodule update --init --recursive
> ```

### 1.2 Run PX4 Ubuntu Setup Script
PX4 provides an automated setup script that installs build tools, compilers (GCC/Clang), Python packages, and the recommended Gazebo version:

```bash
bash ./Tools/setup/ubuntu.sh
```
*Note: A system reboot or re-login is recommended after running this script to apply user group changes (e.g. `dialout`).*

---

## Step 2: Build PX4 SITL & Verify Baseline Simulation

Compile PX4 for headless/GUI SITL with Gazebo:

```bash
cd PX4-Autopilot
# Build and launch default quadcopter (x500) in Gazebo Sim
make px4_sitl gz_x500
```

**Verification:**
- Gazebo opens with an `x500` quadcopter on an empty runway.
- In the PX4 terminal prompt (`px4>`), type:
  ```bash
  commander arm
  commander takeoff
  ```
- The drone should spin its props, lift off to ~2.5m, and hover.
- Type `commander land` and exit (`Ctrl+C`).

---

## Step 3: Install Middleware (Micro XRCE-DDS Agent & ROS 2)

PX4 communicates with external companion code via **uXRCE-DDS**.

### 3.1 Install ROS 2 (Humble or Jazzy)
Follow the standard ROS 2 installation for your Ubuntu version:
```bash
sudo apt update && sudo apt install -y ros-humble-desktop ros-humble-ros-gz-bridge
```

### 3.2 Build `MicroXRCEAgent`
```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 3.3 Create a ROS 2 Workspace & Clone `px4_msgs`
`px4_msgs` provides the ROS 2 message definitions matching PX4's internal uORB topics:

```bash
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src
git clone https://github.com/PX4/px4_msgs.git -b release/1.15
cd ~/drone_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs
```

---

## [OPTIONAL MODIFICATION A] Custom World & Scenery

You can place your drone into custom environments (urban cities, forests, indoor warehouses) without touching PX4.

1. **Create an SDF World File** (e.g., `worlds/custom_world.sdf`):
   ```xml
   <?xml version="1.0"?>
   <sdf version="1.9">
     <world name="custom_world">
       <!-- Sun & Environment Lighting -->
       <light name="sun" type="directional">
         <pose>0 0 15 0 0 0</pose>
         <diffuse>0.9 0.9 0.9 1</diffuse>
         <direction>-0.5 0.1 -1</direction>
       </light>

       <!-- Ground Plane -->
       <model name="ground_plane">
         <static>true</static>
         <link name="link">
           <collision name="collision">
             <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
           </collision>
           <visual name="visual">
             <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
             <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
           </visual>
         </link>
       </model>

       <!-- Custom Static Obstacles or Visual Markers -->
       <model name="target_marker">
         <static>true</static>
         <pose>5.0 0.0 1.0 0 0 0</pose>
         <link name="link">
           <visual name="visual">
             <geometry><box><size>0.8 0.8 0.8</size></box></geometry>
             <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
           </visual>
           <collision name="collision">
             <geometry><box><size>0.8 0.8 0.8</size></box></geometry>
           </collision>
         </link>
       </model>
     </world>
   </sdf>
   ```

2. **Tell PX4 to use your custom world**:
   ```bash
   export PX4_GZ_WORLD=/path/to/custom_world.sdf
   make px4_sitl gz_x500
   ```

---

## [OPTIONAL MODIFICATION B] Physics Engine & Environment Tuning

Tune physics behavior inside the `<world>` section of your `.sdf` file:

```xml
<!-- 1. Real-Time Factor & Step Size -->
<physics name="sim_physics" type="ignored">
  <!-- 0.002 = 500 Hz simulation step (PX4 default) -->
  <max_step_size>0.002</max_step_size>
  <!-- 1.0 = Real time; 0.0 = As fast as CPU allows (great for RL / batch runs) -->
  <real_time_factor>1.0</real_time_factor>
</physics>

<!-- 2. Custom Gravity (e.g. Earth, Moon, or Zero-G) -->
<gravity>0 0 -9.8066</gravity>

<!-- 3. Dynamic Wind & Gust Disturbances -->
<plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
  <force_approximation_scaling_factor>0.001</force_approximation_scaling_factor>
  <horizontal>
    <magnitude>
      <time_for_rise>10</time_for_rise>
      <sin>
        <amplitude_percent>0.3</amplitude_percent>
        <period>8</period>
      </sin>
    </magnitude>
    <direction><constant>1.57</constant></direction>
  </horizontal>
</plugin>
```

---

## [OPTIONAL MODIFICATION C] Mount & Tune the Camera Sensor

PX4 provides pre-configured camera models in `Tools/simulation/gz/models/` (e.g., `x500_depth`, `x500_mono`). To customize resolution, field-of-view, frame rate, or mount angle:

1. Open or copy `Tools/simulation/gz/models/x500_depth/model.sdf`.
2. Locate the `<sensor name="camera" type="camera">` or `<sensor name="depth_camera" type="depth_camera">` block:
   ```xml
   <sensor name="front_camera" type="camera">
     <pose>0.15 0 0 0 0.17 0</pose> <!-- x y z roll pitch yaw; 0.17 rad is ~10 deg downward tilt -->
     <camera>
       <horizontal_fov>1.3962634</horizontal_fov> <!-- 80 degrees in radians -->
       <image>
         <width>640</width>
         <height>480</height>
         <format>R8G8B8</format>
       </image>
       <clip>
         <near>0.1</near>
         <far>100.0</far>
       </clip>
     </camera>
     <always_on>1</always_on>
     <update_rate>30</update_rate> <!-- 30 FPS -->
     <topic>/camera/image_raw</topic>
   </sensor>
   ```

3. Launch with your camera model:
   ```bash
   make px4_sitl gz_x500_depth
   ```

---

## [OPTIONAL MODIFICATION D] Multi-Vehicle Simulation & Dynamic Targets

### D.1 Spawning Multiple PX4 Drones
Each drone needs:
1. A unique Gazebo model name.
2. A unique PX4 instance ID (`-i <id>`).
3. Distinct DDS / MAVLink port offsets (handled automatically by `-i`).

```bash
# Terminal 1: Drone 0 (Spawns at origin)
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_NAME=x500_0 ./build/px4_sitl_default/bin/px4 -i 0

# Terminal 2: Drone 1 (Spawns 2 meters to the side)
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_NAME=x500_1 PX4_GZ_MODEL_POSE="2.0,0,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 1
```
In ROS 2, topics will be namespaced:
- Drone 0: `/px4_1/fmu/in/...`
- Drone 1: `/px4_2/fmu/in/...`

### D.2 Spawning a Moving Ground Target
To test tracking without a second drone, add a moving target to the SDF world using the trajectory follower plugin:
```xml
<model name="moving_target">
  <pose>5 0 0.2 0 0 0</pose>
  <link name="link">
    <visual name="v"><geometry><sphere><radius>0.3</radius></sphere></geometry><material><diffuse>0 1 0 1</diffuse></material></visual>
    <collision name="c"><geometry><sphere><radius>0.3</radius></sphere></geometry></collision>
  </link>
  <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
    <line>
      <waypoint>5 0 0.2</waypoint>
      <waypoint>15 0 0.2</waypoint>
      <waypoint>15 10 0.2</waypoint>
      <waypoint>5 10 0.2</waypoint>
    </line>
    <velocity>1.5</velocity>
    <loop>true</loop>
  </plugin>
</model>
```

---

## Step 4: Bridge Gazebo Camera to ROS 2

Gazebo streams images over internal Gazebo Transport. Use `ros_gz_bridge` to expose them as standard ROS 2 `sensor_msgs/msg/Image`:

```bash
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
```

You can view the live stream in rqt or OpenCV:
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

---

## Step 5: Start the Micro XRCE-DDS Agent

In a separate terminal, start the agent so ROS 2 can exchange uORB messages with PX4:

```bash
MicroXRCEAgent udp4 -p 8888
```

---

## Step 6: Plug In Your Custom Python Vision & Navigation Algorithm

Save this script as `vision_navigator.py` in your workspace. It handles:
1. Subscribing to `/camera/image_raw` and converting to OpenCV.
2. Arming the drone and switching into **Offboard Mode**.
3. Maintaining the required **20 Hz heartbeat**.
4. Sending 3D velocity and yaw rate setpoints calculated by your vision logic.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

class VisionNavigator(Node):
    def __init__(self):
        super().__init__('vision_navigator')
        self.bridge = CvBridge()

        # Best effort QoS profile for PX4 uORB topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos)

        # Publishers to PX4
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # State variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.armed = False
        self.offboard_setpoint_counter = 0

        # Control outputs (North, East, Down velocity in m/s, yaw rate in rad/s)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        # 20 Hz Control Loop Timer (Heartbeat)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Vision Navigator Initialized.")

    def status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def camera_callback(self, msg):
        """Processes incoming camera frame with your vision algorithm."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_image.shape

        # ==============================================================
        # >>> INSERT YOUR CUSTOM ALGORITHM HERE <<<
        # Example: Simple Color/Centroid Tracking (Red target)
        # ==============================================================
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        moments = cv2.moments(mask)

        if moments["m00"] > 500:
            # Centroid coordinates
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])

            # Normalized errors (-1.0 to 1.0)
            err_x = (cx - (w / 2.0)) / (w / 2.0)
            err_y = (cy - (h / 2.0)) / (h / 2.0)

            # Visual Servoing Control Law
            self.yaw_rate = -1.0 * err_x        # Turn towards target
            self.vz = 0.5 * err_y               # Adjust altitude to centre target
            self.vx = 1.0                       # Move forward toward target
        else:
            # Target lost: slow down or hover
            self.vx = 0.0
            self.vz = 0.0
            self.yaw_rate = 0.0

    def timer_callback(self):
        """Streams Offboard mode heartbeat and trajectory setpoints at 20 Hz."""
        # 1. Stream OffboardControlMode (tells PX4 we control Velocity)
        mode = OffboardControlMode()
        mode.position = False
        mode.velocity = True
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(mode)

        # 2. Publish Trajectory Setpoint (NED frame)
        sp = TrajectorySetpoint()
        sp.position = [float('nan'), float('nan'), float('nan')] # NaN tells PX4 to ignore position
        sp.velocity = [float(self.vx), float(self.vy), float(self.vz)]
        sp.yawspeed = float(self.yaw_rate)
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(sp)

        # 3. Arm and engage Offboard mode after streaming 10 initial setpoints
        if self.offboard_setpoint_counter == 10:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # 6 = Offboard
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step 7: Complete Run Orchestration

To run the full stack together:

```bash
# Terminal 1: MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888

# Terminal 2: PX4 SITL + Gazebo with Camera Drone
cd ~/PX4-Autopilot
export PX4_GZ_WORLD=/path/to/custom_world.sdf  # (Optional: your world)
make px4_sitl gz_x500_depth

# Terminal 3: Ros-Gazebo Bridge
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image

# Terminal 4: Your Custom Python Navigation Algorithm
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash
python3 vision_navigator.py
```

---

## Pro-Tips for Fast Iteration

1. **Headless Execution for Speed**:
   Set `HEADLESS=1 make px4_sitl gz_x500` to run simulation without rendering 3D GUI windows. Cameras still render off-screen via GPU/EGL, saving massive CPU/GPU resources.
2. **Rosbag Recording**:
   Record flight trials for offline metric evaluation:
   ```bash
   ros2 bag record /camera/image_raw /fmu/out/vehicle_odometry /fmu/in/trajectory_setpoint
   ```
3. **Disable RC Failsafe in SITL**:
   In SITL, PX4 might refuse arming without a manual joystick/RC transmitter connected. In SITL, set `NAV_RCL_ACT=0` and `COM_RCL_EXST=0` so it accepts purely programmatic commands.
