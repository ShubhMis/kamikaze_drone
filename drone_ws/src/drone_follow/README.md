# Monocular Gazebo/PX4 baseline

This is a small, reproducible **image-centring demonstration**, not an
implementation of either research paper. It uses a normal RGB camera and a red
marker. **There is no depth camera, depth subscription, metric target range, or
forward-approach command.** The original RGB-D package is preserved in
`runs/drone-follow-before-monocular.tar.gz`. Unused range-guidance and manual
launch files were removed from the active source tree after archiving them.

Two separate systems were found in the initial audit. The older scripts control
Gazebo's velocity plugin directly. This package controls two PX4 SITL vehicles
through ROS 2. Both use physical rotor models; neither name nor a subscription
alone proves which controller is active. The older scripts are now under `trash/`;
the current session did not move or delete them.

**Validation status:** on 26 September the viewer's immediate-exit bug was
reproduced and fixed. A bounded centre flight displayed 751 distinct frames at
20.0 FPS with matched detections and both vehicles landed/disarmed. A final
camera-only run displayed 336/336 received frames with matched detections.
Horizontal centring improved in the flight, but vertical error did not converge.
See [EXPERIMENT_LOG.md](EXPERIMENT_LOG.md) for measurements and limitations.
The 28 September interactive-lifetime and reduced-logging changes passed
11 isolated tests and launch-description checks; no new simulation was run.

## Start here

From `/home/crl/Desktop/Shubh/kamikaze`, run:

```bash
bash run_baseline.sh --check
bash run_baseline.sh --headless --viewer
```

The default `observe` scenario starts a headless world, DDS agent, two PX4
instances, camera bridge, detector, controller and sequencer. It waits for real
messages and stays open until you press Ctrl+C. It **does not arm**.
`--headless --viewer` shows just the live camera while
Gazebo runs without its extra 3D window. Omit `--headless` for both windows.
Use `--no-viewer` to hide the camera, `--debug` to show component output and
`--record` to save detailed experiment data. `--duration 0` (now the default)
keeps the selected behaviour running until Ctrl+C.

```bash
# Simulated takeoff and zero-velocity Offboard hover; stays open:
bash run_baseline.sh --headless --viewer --scenario hover

# Simulated takeoff and monocular yaw/vertical centring; stays open:
bash run_baseline.sh --headless --viewer --scenario centre
```

These flight scenarios use ordinary PX4 arming checks; they never force-arm.
QGroundControl is optional: the sequencer uses local PX4 command clients. It
sets RC-free operation and disables the GCS-disconnection action **only in its
own fresh SITL parameter directories**. Ctrl+C stops the entire owned simulation;
it is not a landing command for a physical drone.

Expected flight progress is `WAITING → READY → PREFLIGHT → ARMING → TAKEOFF
→ OFFBOARD → RECORDING/CENTRING`. There is **no scheduled landing or timed
shutdown** with the defaults. A positive `--duration 20` stops centring after
20 wall seconds and leaves the session open; it does not land or close Gazebo.
The follower then requests zero velocity. This is not guaranteed position hold.

A missing/stale stream, rejected command or dropped track reports `FAILED`,
stops sequencing and requests centring disabled if its service is available.
Required-component exits are reported while surviving processes stay open.
PX4's own failsafes remain active and can initiate landing; they were not removed.
Terminal Ctrl+C is how you stop this simulated stack. Closing the optional viewer
does not stop it. Historical bounded tests can still be selected explicitly with
`auto_land: true`, `keep_open: false` and a positive duration in your config.

The launcher uses a clean child environment with system Python 3.10 and
`PYTHONNOUSERSITE=1`. It does not change your Conda installation or shell. It
uses ROS domain 42 and a private Gazebo partition. It refuses a second baseline
run, an existing PX4 process, or an occupied DDS port. It signals only its own
process group; it never uses `pkill`/`killall`.

## Live stream and correction overlays

The camera window opens automatically with a normal GUI launch. For only the
camera window, which avoids the extra Gazebo GUI rendering load, use:

```bash
bash run_baseline.sh --headless --viewer --scenario centre
```

The left pane shows the raw RGB stream, centre crosshair, green detection box,
centroid, line to centre and magnified target detail. The right pane separates:

- **Image errors:** pixel offsets plus yaw/pitch bearing alignment in degrees.
  Positive yaw means target right; positive pitch alignment means target above.
- **Actual attitude:** PX4 estimated yaw (NED heading) and nose-up pitch.
- **Commands being published:** yaw rate in degrees/s and climb speed in m/s.
  During PX4 takeoff/landing it says external commands are paused.
- **Pitch command: NONE:** this baseline corrects vertical image error with
  vertical speed. A displayed pitch bearing error is not a commanded pitch angle.
- Camera arrival FPS, simulation image age, controller state and stale-stream warning.

The viewer is read-only: closing it with **Q / Esc / the window close button**
does not affect the experiment. **Terminal Ctrl+C** stops the whole simulation.

For low latency, the viewer runs separately, retains at most four raw images,
uses one OpenCV worker, and redraws at up to 60 Hz. The source camera is 20 Hz;
refreshing the window faster does not create additional measurements. The HUD
counts received frames and distinct displayed frames separately. A matched
image/detection pair can remain on screen for at most `viewer_pair_wait_s`
(80 ms after image receipt) while a newer detection arrives. It then falls back
to the newest raw image with detection marked pending. An old box is never
pasted over a different image. A frozen stream is explicitly marked stale.

The final camera check measured 19.99 distinct displayed FPS, 19.5 ms p95 from
viewer image callback to first display submission, and a maximum inter-frame
gap of 67.5 ms. These are software timings from one short run, not a measured
monitor refresh rate or an end-to-end camera latency guarantee. The 20 Hz
source itself limits smoothness; no synthetic frames or detector smoothing are used.

If the viewer exits, inspect terminal errors and `viewer.json`; `--debug` exposes
component stdout, and `--record` saves detailed output. On this machine OpenCV
uses GTK, where `WND_PROP_VISIBLE` returns an unsupported value even when a
window is open. The viewer now checks window existence through a supported
property and logs its exit reason. No dependency upgrade was needed.

## One configuration, saved with every run

Edit [`config/follow.yaml`](config/follow.yaml), or pass `--config /path/to/file.yaml`.
The camera geometry is intentionally still declared in the model SDF.

| Setting | Meaning |
|---|---|
| `baseline.px4_dir` | Built PX4 checkout; defaults to `~/PX4-Autopilot`. |
| `baseline.follower_yaw_deg` | Initial scene heading offset. Try `10.0` to make horizontal centring visible. This is scenario generation, not a target measurement. |
| `baseline.viewer`, `viewer_fps`, `viewer_stale_s` | Enable viewer, cap display refresh, and set wall-time freeze warning. CLI `--viewer` / `--no-viewer` overrides enable. |
| `baseline.viewer_pair_wait_s` | Maximum wall age of a buffered image/detection pair; default 0.08 s. Display only; never changes controller input timing. |
| `baseline.duration_s` | Behaviour duration after readiness/hover/enable; 0 means indefinite. |
| `baseline.auto_land` | Default false. An explicitly bounded flight lands only if this is true. |
| `baseline.keep_open` | Default true. Completion/failure does not request simulator shutdown. |
| `baseline.record` | Default false. `--record` enables detailed files and independent source snapshots. |
| `baseline.takeoff_altitude_m` | Commanded height above takeoff location, 3 m by default. |
| `follower.ros__parameters.yaw_gain` | Yaw response to bearing error; default 0.8 s⁻¹. |
| `follower.ros__parameters.vertical_gain` | Downward speed per normalized vertical error; default 0.6 m/s. |
| `follower.ros__parameters.max_measurement_age_s` | Detection expiry, 0.3 s. A lost track disables centring. |

A small experiment: set `follower_yaw_deg: 10.0`, run `centre`, then change only
`yaw_gain` from `0.8` to `0.4` and repeat. Compare `error_u_px` over time in the two
CSV files (run with `--record`), including visibility and measurement age. Expect a slower response;
this is a hypothesis to check, not a promised result. Do not change scenarios,
sensor information or command limits between comparisons.

Normal sessions keep a small `runs/YYYYMMDD_HHMMSS_microseconds/` directory:
configuration, source hashes/version manifest, effective world, camera intrinsics,
low-rate stage/command events, launch status, viewer summary and final result.
PX4 parameter/work files and generated launch files are needed at runtime.
`source/` is a link to the workspace, not a duplicated package; avoid changing
source while that session is running. Minimal sessions retain neither per-frame
CSV/JSONL nor screenshots nor PX4 ULogs. Existing saved runs are not deleted.

Use `--record` when collecting research evidence. It adds:

- `config.yaml`, effective `world.sdf`, source snapshot and SHA-256 manifest;
- `logs/`: component output combined into the launch log, plus ROS node logs;
- `measurements.csv`: pixel error, visibility, image/control/measurement timestamps,
  latest command and both vehicles' estimated positions for evaluation only;
- `events.jsonl`: image/control events, scenario transitions and PX4 command results;
- `camera.json` and sample images by phase;
- with the viewer: `viewer_first.png`, `viewer_tracking.png` when centring is
  enabled, and `viewer.json` with exit reason and display timing (last 6,000
  distinct frames for timing statistics, whole-run frame counts);
- `result.json`: outcome, frame timing, visibility and pixel-error summary;
- `px4_follower/`, `px4_target/`: independent parameter files and PX4 ULogs.

These samples are matched by callback arrival, not retrospectively synchronized.
`publishing_setpoint` distinguishes a requested zero from a paused publisher;
the two vehicles' local NED origins must be aligned before comparing positions.
The CSV is an initial diagnostic record, not yet a calibrated delay experiment.
Run artifacts are ignored by Git; retain the runs you use as research evidence.

## Understand the flow

1. Gazebo renders the RGB camera and emits image/calibration/clock messages.
2. `ros_gz_bridge` translates them into ROS messages. It carries no flight commands.
3. `detector.py` finds one red component, uses camera calibration to form a unit
   optical bearing, and logs both accepted and rejected observations.
4. `follower.py` checks image age and its own PX4 state. `monocular.py` computes
   yaw rate and vertical speed. Horizontal speed stays zero.
5. DDS carries the setpoint to PX4's velocity, attitude and rate controllers.
6. PX4's native Gazebo bridge sends rotor speeds; motor plugins apply forces;
   the moved camera supplies the next image.

For example, with focal length 381 px and a target 80 px right of centre,
`x=80/381=0.210`. The requested NED yaw rate is
`0.8 atan(0.210)=0.166 rad/s` (about 9.5°/s) to the right. This command contains
**no distance estimate**. The vertical sign assumes a forward camera and
approximately level hover; the guide describes the limits.

Read [ARCHITECTURE.md](ARCHITECTURE.md) for every active script, exact topics,
frames, clocks, equations, failure behavior and architecture choice.
[LEGACY_AUDIT.md](LEGACY_AUDIT.md) covers the older scripts and their actual
control plugins. [EXPERIMENT_LOG.md](EXPERIMENT_LOG.md) separates verified
results, failures and pending milestones.

There is no `intercept` command. `centre` is the existing yaw/vertical image
centring mode; it does not move toward the target. The final section of
[ARCHITECTURE.md](ARCHITECTURE.md#11-learning-path-before-you-add-a-non-contact-approach)
explains the uploaded papers, the missing range information and a staged path
for you to implement a non-contact approach to a defined stand-off. No approach
or paper-controller code has been added.

## Existing-machine build and troubleshooting

Normal operation needs no rebuild: the root launcher uses workspace source by
default, or an independent copy with `--record`, through your existing ROS
environment. Run only `bash run_baseline.sh`;
the redundant manual launch scripts were removed. If rebuilding the ROS package
for installation, the normal command remains `colcon build --symlink-install
--packages-select drone_follow` from a sourced `drone_ws` shell.

Do not start a second world with `make px4_sitl gz_x500` alongside this launcher.
The old `/target/point_ned` topic was removed with depth; current topics include
`/target/bearing_camera`, `/target/observation` and `/follow/metrics`.
The built-in viewer replaces the need for a separate `rqt_image_view` terminal.
If it exits unexpectedly, inspect terminal errors and `viewer.json`; use `--debug`
for more terminal output or `--record` to save component output.

## Why these directories remain

| Path | Why it exists / whether to edit it |
|---|---|
| Root `run_baseline.sh` | The single entry command. |
| `drone_ws/src/drone_follow/` | The small project-owned package: edit its configuration and Python here. |
| `drone_ws/src/px4_msgs/` | PX4 ROS message definitions. The many small `.msg` files are upstream dependencies, not extra controllers. Keep them for rebuilding. |
| `drone_ws/build/`, `drone_ws/install/` | Generated ROS build and runtime files. The symlink installation refers back to the build/source tree; deleting either breaks the current runnable workspace until rebuilt. Do not edit them. |
| `runs/` | Saved configurations, code snapshots, measurements, PX4 logs and recovery archives. They are results, not active code. Preserved, including your latest runs. |
| `PX4-Autopilot/` inside this repository | Model assets and checkout used by the earlier Gazebo project. The new baseline uses `baseline.px4_dir` (currently `~/PX4-Autopilot`). It is not safe to call the older assets unused by the whole repository. |
| Root legacy scripts/worlds, `Dhanur_New/`, `P_Nav/` | Earlier projects preserved. Not launched by the new baseline. See the legacy audit before deleting them or their assets. |
| `.git/` | Version history. Keep it. |

Removed from the active package: old `guidance.py`, its old test, `env.sh`,
`start_world.sh`, `start_px4.sh`, `demo.launch.py`, and the duplicate inner
`run_baseline.sh`. The root launcher now contains its own environment setup.
A verified recovery archive is `runs/cleanup_20260926_111608.tar.gz`.
Old Python caches and colcon build logs were also deleted; they regenerate as needed.

The necessary runtime Python files now have separate jobs: detector, pure
controller math, PX4 adapter, target generator, experiment monitor and viewer.
Combining them into one large file would hide the sensing/control boundaries.

<details>
<summary>Reproduce prerequisites on a fresh Ubuntu 22.04 machine</summary>

Use ROS Humble, Gazebo Harmonic, PX4 release/1.15 and matching px4_msgs. After
configuring the official ROS and Gazebo apt repositories:

```bash
sudo apt install gz-harmonic ros-humble-ros-base ros-humble-ros-gzharmonic \
  ros-humble-cv-bridge ros-humble-std-srvs ros-humble-rqt-image-view \
  ros-humble-rosidl-default-generators python3-colcon-common-extensions \
  python3-numpy python3-opencv python3-yaml git cmake build-essential
```

Humble's default Fortress bridge conflicts with the Harmonic bridge. Do not
replace an existing project checkout or upgrade dependencies just to rerun a
successful baseline. Exact tested commits are saved in the run manifest.

```bash
cd ~
git clone --recursive --branch release/1.15 https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash Tools/setup/ubuntu.sh --no-nuttx
# Follow the setup script's logout/reboot instructions before building.
git submodule update --init --recursive
make px4_sitl_default

cd ~
git clone --branch v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
# This release references an unavailable upstream branch; use the release tag.
sed -i 's/set(_fastdds_tag 2\.12\.x)/set(_fastdds_tag v2.12.1)/' CMakeLists.txt
cmake -S . -B build \
  -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt \
  -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog
cmake --build build -j4
# Install only after a successful build:
sudo cmake --install build
sudo ldconfig
```

Place this repository at a chosen location. If `drone_ws/src/px4_msgs` does not
already exist, clone `https://github.com/PX4/px4_msgs.git` with `--branch
release/1.15` there. Then from `drone_ws`:

```bash
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
/usr/bin/colcon build --symlink-install --cmake-clean-cache --cmake-args \
  -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
```

The launcher fails early with an actionable missing-path/import error; it does
not install or upgrade system packages automatically.
</details>

## Local checks and next milestone

```bash
cd /home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow
PYTHONNOUSERSITE=1 /usr/bin/python3 -m unittest discover -s test -v
```

The three retained tests check monocular command signs, limits and an ideal
yaw response. The six obsolete range-guidance tests were archived with that code. Tests of simplified equations do not prove flight performance.

Next: repeat stationary centring before trying the existing slow target path.
Metric stand-off and forward approach remain disabled until a range source and
its assumptions are agreed and validated. Options such as known target size or
motion-based estimation need separate tests; none is silently substituted for
a depth sensor. Controlled delay/dropout comparisons come after this baseline.
