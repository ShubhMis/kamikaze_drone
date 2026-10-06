# Single-controller monocular PNG-IBVS SITL

This ROS 2 package runs two PX4 X500 vehicles in Gazebo and gives the follower
one active visual-control path. A calibrated monocular bearing is converted to
a PNG-inspired three-dimensional NED velocity command plus yaw rate. There is
no centre/intercept mode selector and no centring fallback.

The detector is intentionally unchanged: it still tracks the simulated red
marker and publishes `/target/bearing_camera`. A delayed Kalman filter is not
implemented yet. Detection replacement and DKF integration are deferred until
the controller mathematics is made coherent.

The current controller is a **velocity-interface reconstruction**, not a full
reproduction of Yan et al., *Precise Interception Flight Targets by Image-based
Visual Servoing of Multicopter*. Read
[PAPER_CONTROLLER_CONTRACT.md](PAPER_CONTROLLER_CONTRACT.md) before changing
the controller. It maps Eqs. (3)-(23) to code and records every missing or
different term.

## Run

From the repository root:

```bash
bash run_baseline.sh --check

# Camera, detector and controller health only; does not arm.
bash run_baseline.sh --headless --viewer --scenario observe

# Take off both vehicles and hold in PX4 Offboard without visual pursuit.
bash run_baseline.sh --headless --viewer --scenario hover

# Enable the one PNG-IBVS velocity controller after takeoff.
bash run_baseline.sh --headless --viewer --scenario intercept --record
```

`--duration 0` keeps the selected scenario running until Ctrl+C. A positive
duration ends the measured interval. Landing occurs only when `auto_land: true`
is explicitly configured. PX4 health and Offboard-loss checks remain active.

The flight sequence for `intercept` is:

```text
WAITING -> PREFLIGHT -> ARMING -> TAKEOFF -> OFFBOARD
        -> ENABLING -> INTERCEPTING
```

The stage names describe experiment lifecycle. They do not select different
controllers.

## Active flow

```text
Gazebo RGB camera (20 Hz)
    -> detector.py
    -> /target/bearing_camera, stamped optical-frame unit ray
    -> follower.py health/freshness gate
    -> png_ibvs.intercept_command()
    -> PX4 TrajectorySetpoint.velocity + yawspeed
```

`follower.py` always creates a fresh `PNGIBVSState` when pursuit is enabled.
Disable, target loss, estimator reset, invalid PX4 state, leaving Offboard, or a
clock discontinuity clears the complete controller state. A visual-only loss
can retain the operator's pursuit request: the node commands zero velocity,
rebuilds the configured fresh-detection streak, and starts a new state. Other
faults and explicit disable always require another enable request.

## Scenarios

| Scenario | Arms | Visual controller |
|---|---:|---:|
| `observe` | No | Disabled |
| `hover` | Yes | Disabled; PX4 zero-velocity Offboard setpoint |
| `intercept` | Yes | Single PNG-IBVS velocity reconstruction |

`centre` is intentionally rejected. Git history and saved runs preserve the
former centring baseline.

## Configuration

The main file is [config/follow.yaml](config/follow.yaml).

For a target that starts the airborne phase higher than the follower, set:

```yaml
baseline:
  takeoff_altitude_m: 3.0
  target_takeoff_altitude_offset_m: 3.0
```

This commands 3 m above home for the follower and 6 m for the target. To move
the ground spawn horizontally, edit the target `<pose>` in
`worlds/follow_world.sdf`; its six values are Gazebo ENU `x y z roll pitch
yaw`. Keep the raw spawn `z` on the ground and use the takeoff offset for an
airborne altitude difference.

| Parameter | Meaning |
|---|---|
| `takeoff_altitude_m` | Follower takeoff height above its PX4 home position |
| `target_takeoff_altitude_offset_m` | Extra target height; currently 3 m above the follower |
| `max_measurement_age_s` | Maximum accepted bearing age before control stops and visual reacquisition begins |
| `minimum_detections` | New detections required before enable |
| `auto_reacquire` | Retain pursuit intent after visual-only loss while commanding zero velocity |
| `reacquire_timeout_s` | Maximum time allowed to rebuild a fresh detection streak |
| `png_gain_y`, `png_gain_z` | Current LOS-angle update gains |
| `fov_kp`, `fov_kd` | Current horizontal FOV yaw feedback gains |
| `speed_increment_mps` | Bounded speed increment for the explicit `vd = ||vnow|| + ka` interpretation of Eq. (14) |
| `max_speed` | NED velocity magnitude limit |
| `max_vertical_speed` | NED-down component limit |
| `max_yaw_rate` | Yaw-rate limit |
| `camera_mount_q` | Camera-FRD to body-FRD mount rotation |

The current values are retained for reproducibility. They are not certified
flight tuning and should not be presented as paper-equivalent gains.

## Important implementation status

Implemented:

- calibrated monocular unit bearing with the image timestamp;
- optical-to-camera-FRD-to-body-to-NED direction conversion;
- LOS azimuth/elevation calculation;
- bounded desired-speed magnitude from current speed plus `speed_increment_mps`;
- one persistent PNG-inspired velocity-direction state;
- exposure-time attitude and own-velocity alignment;
- one guidance-state update per distinct camera observation;
- concurrent horizontal yaw feedback;
- full reset on disable/loss/health failure;
- three-dimensional command recording and viewer display.

Still missing or different from the paper:

- measured vehicle velocity angles in Eq. (8) and the Eq. (9) state anchor;
- consistent pixel versus normalized FOV-error units;
- the stated vertical-excursion behavior of Eqs. (15)-(16);
- desired acceleration, lift direction, attitude, body-rate and lift control in
  Eqs. (17)-(23);
- the 200 Hz paper controller rate;
- DKF delay correction.

## Verification

Run the isolated package tests with the ROS/PX4 workspace sourced:

```bash
cd /home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow
PYTHONNOUSERSITE=1 /usr/bin/python3 -m unittest discover -s test -v
```

Then validate configuration and launch construction:

```bash
cd /home/crl/Desktop/Shubh/kamikaze
bash run_baseline.sh --check
```

Passing isolated tests verifies software contracts and current mathematics. It
does not establish closed-loop stability or equivalence to the paper. Use
`--record` for SITL evaluation so configuration, source hashes, measurements
and PX4 logs are retained.

### Python and `px4_msgs`

`px4_msgs` is generated in this ROS workspace rather than installed as a normal
system Python package. The repository `pyrightconfig.json` gives Pylance/Pyright
the built ROS and PX4 message paths. Reload the editor after the first build.
For direct terminal imports, source both environments first:

```bash
source /opt/ros/humble/setup.bash
source /home/crl/Desktop/Shubh/kamikaze/drone_ws/install/setup.bash
```

## Key files

| File | Responsibility |
|---|---|
| `drone_follow/detector.py` | Existing red-marker observation source; unchanged in this phase |
| `drone_follow/png_ibvs.py` | Current pure velocity-interface controller mathematics |
| `drone_follow/follower.py` | Single-controller lifecycle and PX4 adapter |
| `drone_follow/leader.py` | Target hold and optional figure-eight setpoints |
| `drone_follow/experiment.py` | Scenario sequencing and measurements |
| `drone_follow/viewer.py` | Read-only camera and command display |
| `PAPER_CONTROLLER_CONTRACT.md` | Equation, frame, unit, rate, state and gap register |
| `ARCHITECTURE.md` | Active runtime architecture |
| `EXPERIMENT_LOG.md` | Historical run evidence; old centring entries remain historical |
