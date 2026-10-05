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
clock discontinuity clears the complete state. Reacquisition requires the
configured number of fresh detections and another explicit enable request.

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

| Parameter | Meaning |
|---|---|
| `max_measurement_age_s` | Maximum accepted bearing age before pursuit latches off |
| `minimum_detections` | New detections required before enable |
| `png_gain_y`, `png_gain_z` | Current LOS-angle update gains |
| `fov_kp`, `fov_kd` | Current horizontal FOV yaw feedback gains |
| `approach_speed` | Current fixed velocity magnitude; a documented departure from paper Eq. (14) |
| `fov_ka` | Current custom vertical correction; not attributable to paper Eqs. (15)-(16) |
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
- one persistent PNG-inspired velocity-direction state;
- concurrent horizontal yaw feedback;
- full reset on disable/loss/health failure;
- three-dimensional command recording and viewer display.

Still missing or different from the paper:

- measured vehicle velocity angles in Eq. (8) and the Eq. (9) state anchor;
- exposure-time attitude interpolation;
- consistent pixel versus normalized FOV-error units;
- a justified interpretation of Eq. (14);
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

## Key files

| File | Responsibility |
|---|---|
| `drone_follow/detector.py` | Existing red-marker observation source; unchanged in this phase |
| `drone_follow/png_ibvs.py` | Current pure velocity-interface controller mathematics |
| `drone_follow/follower.py` | Single-controller lifecycle and PX4 adapter |
| `drone_follow/experiment.py` | Scenario sequencing and measurements |
| `drone_follow/viewer.py` | Read-only camera and command display |
| `PAPER_CONTROLLER_CONTRACT.md` | Equation, frame, unit, rate, state and gap register |
| `ARCHITECTURE.md` | Active runtime architecture |
| `EXPERIMENT_LOG.md` | Historical run evidence; old centring entries remain historical |
