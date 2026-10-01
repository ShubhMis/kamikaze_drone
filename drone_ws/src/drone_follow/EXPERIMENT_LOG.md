# First-session experiment and change log

This is the evidence record for 25–26 September 2026. The user asked on
26 September for **no further assistant-run experiments**. No simulation was
started after that request; final work was documentation and static inspection.

## Existing work preserved

- Root `run_px4_sim.sh`, keyboard/viewer/controller scripts and worlds were not
  modified in this session. Pre-existing changes to `intercept_manual.py`,
  `px4_baylands_world.sdf`, and the deletion of root `README.MD` were left intact.
- The original ROS package was archived before the monocular change at
  `runs/drone-follow-before-monocular.tar.gz`.
- The original pure `guidance.py` was initially retained, then archived and
  removed during the requested source cleanup below.
- No dependency upgrades, ROS migration, containers, hardware control or paper
  controller implementation were performed.

## Observed runs (historical; not tests of every final edit)

Run paths are relative to repository-root `runs/`. Each listed run contains its
own source/configuration where applicable, logs and result evidence.

| Run | Action and observed outcome | Limits |
|---|---|---|
| `audit-legacy/20260925_113620` | Original Baylands Gazebo velocity-plugin path, no PX4/ROS. Small 0.4 m/s body-z command for 3 simulation seconds, then zero for 5. 247 camera frames; monotonic stamps; approximately 30 Hz. Final z ≈1.284 m; last two seconds varied ≈4.8 mm vertically. | Horizontal drift ≈0.384 m; not stable position hold or visual tracking. Static target absent from dynamic-pose topic. See `LEGACY_AUDIT.md`. |
| `audit-ros-original` | First diagnostic harness failed to supply PX4's startup resource path after selecting a fresh working directory. | Harness error, not evidence that the original package cannot run. |
| `audit-ros-original-2` | Corrected harness ran the old ROS/PX4 stack and reached `READY / ZERO VELOCITY`. | Communication readiness only; no tracking or takeoff commanded. |
| `20260925_163703_806477` | New monocular `observe` completed. 206 recorded observations, all marker-visible; no non-increasing timestamps. Stack stopped after the bounded recording. | Vehicles remained unarmed. Historical `camera_hz_sim` used the median interval, not total-window throughput. |
| `20260925_163806_145376` | `hover`: both took off and entered Offboard. Target landed/disarmed; follower reached ground but did not complete landing detection. Run correctly failed on landing timeout. | ULog showed commanded-descent/low-throttle checks alternating while external zero setpoints competed with landing. |
| `20260925_164130_404503` | Added takeoff/landing publication guard. `centre` with initial follower yaw 10° reduced horizontal error, then a 0.3 s measurement gap disabled centring. Run correctly failed. | Detector accepted received images, but transport/processing gaps triggered the freshness latch. No automatic reacquisition was hidden. |
| `20260925_164252_258454` | After reliable RGB subscription and single-threaded OpenCV, a 15 s centring phase completed and both vehicles landed/disarmed. 835 observations, all visible, monotonically increasing image stamps. Mean received rate computed afterward from stored stamps ≈20.004 Hz simulation time. | One successful run, not a statistical repeatability study. Horizontal error ≈62.05→−0.23 px; vertical ≈−26.08→−12.13 px. Vertical convergence remained partial. |

The successful run's `result.json` contains a historical `camera_hz_sim` of
20.83 Hz because it used the median of quantized 0.048/0.052 s intervals.
The current recorder instead calculates total received intervals divided by
total positive stamp duration and saves the median interval separately. Do not
compare those historical/new fields as if the definitions were identical.

## Implemented changes and validation boundary

1. Replaced the active RGB-D sensor with a monocular RGB camera and removed the
   depth bridge/subscription. Removed active `/target/point_ned`; now publish an
   optical unit bearing and observation diagnostics.
2. Added range-free `monocular.py`, with yaw-rate and vertical-velocity commands.
   Horizontal velocity stays zero. Red segmentation is a controlled marker
   baseline, not a general drone detector. No known-size range estimate is used.
3. Added a root command, clean Python environment, ROS launch of all components,
   readiness checks, isolated run directories and process ownership.
4. Added per-component logs, saved source/configuration, image/calibration
   snapshots, timestamped observations/control metrics and bounded scenarios.
5. Corrected external setpoint publication during PX4-owned takeoff/landing.
6. Nine unit tests passed before the final supervisor refinements: six retained
   guidance tests and three monocular sign/limit/ideal-yaw tests. They are not
   flight-validation evidence.

**After the successful run**, code review led to additional changes:

- Cleanup addresses the owned process group even if its launch parent exits first.
- Readiness requires fresh, advancing, calibrated image observations.
- Offboard entry waits for hold mode and two seconds of fresh heartbeat streaming.
- CSV includes control and used-measurement timestamps, publication status and
  target estimated position for evaluation only.
- Frame-rate reporting uses total received stamp duration.
- ROS launch is loaded from the saved source snapshot; model metadata now says
  monocular rather than RGB-D.

These final changes have been statically inspected, **not rerun in simulation**.
The attempted repeat was rejected by automatic approval review because the
account usage limit had been reached. The user subsequently requested no more
assistant-run experiments. Dedicated duplicate-launch, forced-child-failure and
Ctrl+C lifecycle tests remain for the user; normal complete-stack shutdown was
observed in earlier successful and failed runs.

## Milestones and next user actions

| Milestone | Status |
|---|---|
| Audit and preserve actual existing paths | Complete; legacy source and bounded flight evidence documented. |
| Single-command start, logging and shutdown | Implemented; normal bounded shutdown observed. Final refinements await your run. |
| Stable flight/camera timestamps | Bounded PX4 takeoff/Offboard/landing observed; camera timing recorded. Not a long-duration stability claim. |
| Centre stationary target | One successful monocular run; horizontal error nearly removed, vertical error partly reduced. |
| Slowly moving target | Existing leader generator retained; not evaluated with this monocular controller. |
| Metric stand-off / controlled forward approach | Not implemented in the monocular baseline. Requires a justified range/scale estimate. |
| Controlled delay and dropout evaluation | Not implemented; timestamped observations and loss handling are present. Natural transport loss was observed. |
| Faithful paper controller / comparative research claims | Not implemented; relevant equations and ambiguities documented in `ARCHITECTURE.md`. |

Start with `bash run_baseline.sh --headless --scenario observe`, then `hover`,
then `centre`. The README gives exact commands. For a visible horizontal error,
set only `baseline.follower_yaw_deg: 10.0` in `config/follow.yaml`. Keep all other
settings unchanged when comparing gains. Inspect `result.json`, the CSV, and
component logs; do not infer centring accuracy from a green `PASSED` alone.

To explore the existing moving-target generator later, keep a `centre` run alive
with `--duration 0`, then call `/target/move` with `data: true` in a shell sourced
for ROS domain 42. This is an **unevaluated next milestone**, not a tested recipe
for stand-off tracking. The current controller rotates and changes height; it
cannot follow horizontal target translation or maintain metric separation.

## 26 September: requested live viewer and cleanup

No simulator, flight experiment or GUI was started by the assistant for this
change. Existing run records were inspected read-only:

- `20260926_110133_623371`: observe passed, 531/531 visible images, ~20.006 Hz.
- `20260926_110309_476800`: hover passed and both disarmed, 703/703 visible images.
- `20260926_110401_997940`: centre passed and both disarmed, 914/914 visible images;
  pixel error approximately (−2.72, −6.99) → (−0.71, −0.94).

Those runs precede the new viewer and do not validate its GUI or frame pacing.
The user's current `follower_yaw_deg: 12.0` was preserved.

Added an optional read-only `viewer.py`, launched automatically in GUI mode or
explicitly with `--headless --viewer`. The HUD shows stamp-matched marker box,
centroid, image error, camera yaw/pitch alignment, measured PX4 yaw/pitch,
published yaw rate and climb speed, state, arrival FPS and staleness. It clearly
labels pitch as not directly commanded. No controller law or camera source rate
was changed. Detector observations now include bounding boxes and integer stamps.

Removed seven inactive/redundant source files after creating and checksum-verifying
`runs/cleanup_20260926_111608.tar.gz`: old guidance and test, the three manual
startup helpers, `demo.launch.py` and the duplicate inner shell launcher. The
root shell launcher now sets up the environment directly. Matching installed
aliases were removed. Deleted regenerated Python caches (~172 kB) and old
colcon build logs (~4.04 MB). Preserved all recorded runs, prior projects,
PX4/model assets, message definitions, and required ROS build/install output.
The cleanup manifest is `runs/cleanup_20260926_111608.tar.json`.

The viewer and updated launcher require a user-run GUI check. Static syntax,
configuration and reference consistency checks are the validation performed here.

## 26 September: viewer failure reproduced and live camera fix verified

The user subsequently authorized simulator checks for the camera issue and
requested explanation, but no implementation, of an approach/paper controller.
The supplied run `20260926_112150_322521` showed the viewer exiting cleanly
before readiness; the user then interrupted the flight during takeoff.

**Cause:** the installed OpenCV 4.5.4 uses GTK3. A desktop probe returned
`WND_PROP_VISIBLE=-1` on an open window. The viewer treated every value `<1`
as closed and exited on its first display iteration. The upstream GTK property
implementation also lacks a visibility case:
[OpenCV 4.5.4 GTK source](https://github.com/opencv/opencv/blob/4.5.4/modules/highgui/src/window_gtk.cpp#L2001).
`WND_PROP_FULLSCREEN` returned 0 for the open ordinary window and raised
`cv2.error` after it was displayed and destroyed. This supported property is
now used as an existence probe; it does not change the window mode.

| Run | Configuration / result | What was observed |
|---|---|---|
| `20260926_112913_782706` | Original viewer, observe 10 s | Reproduced early clean viewer exit; sensor scenario still passed. |
| `20260926_113239_810164` | Fixed viewer, centre 12 s, initial yaw 12 deg, viewer refresh 30 Hz | Completed takeoff, centring, landing and disarm of both vehicles. Viewer stayed open until shutdown: 752 received / 751 distinct displayed / 751 matched frames, 20.000 displayed FPS. Display gap p95 66.9 ms, max 67.7 ms; callback-to-first-display p95 35.4 ms. |
| `20260926_113454_060209` | Final viewer, observe 10 s, refresh 60 Hz | No arming. 336 received / 336 distinct displayed / 336 matched frames, 19.991 displayed FPS. Display gap p95 67.0 ms, max 67.5 ms; callback-to-first-display p95 19.5 ms. Viewer and detector stderr empty; both exited cleanly on stack shutdown. |

Matched-frame count means a distinct displayed image had an exact-stamp
observation while it was on screen; it is not a count of new camera frames
generated by GUI redraws. Timing measures software render submissions, not
physical monitor refresh or end-to-end sensor latency. Camera source remains
20 Hz; changing GUI refresh reduced measured queue delay in these runs, but did
not remove the roughly 67 ms largest image interval. These short runs do not
establish performance under other graphics loads.

The centre run's recorder received 749/749 visible, valid images, with strictly
increasing stamps and ~20.004 Hz in simulation time. During 244 centring samples,
horizontal error changed +82.48 → +0.07 px, vertical error -12.19 → -33.56 px.
The successful procedure therefore **does not validate vertical convergence**.
No controller gains, dynamics, camera geometry, flight sequencing or approach
logic were changed to improve that result. The detector/recorder and viewer
start and stop at slightly different times, so their total frame counts differ.

Changes were confined to the window check, bounded display pairing (four raw
images, maximum buffered-pair age 80 ms), distinct-frame timing and two HUD
snapshots, viewer refresh/config validation, and viewer exit reporting. The
flight run also exposed a detector shutdown race: SIGINT during OpenCV work
caused its `finally` block to publish into an already-invalid ROS context.
The final version skips that diagnostic publication only during shutdown and
still propagates errors in a live context. This does not change detector maths.

The normal DDS agent exit code -2 after the launch sends SIGINT is a shutdown
signal, not an observed camera failure. No test left owned PX4 processes running.
Syntax and isolated pairing checks covered exact timestamp pairing, bounded
waiting, no backwards display, negative detections and an empty image buffer.
The saved first/tracking HUD PNGs were visually inspected. Closing through the
desktop window close button remains a user interaction check; programmatic
window destruction and stack-driven viewer shutdown were checked.

See section 11 of `ARCHITECTURE.md` for the requested paper reading and staged
non-contact stand-off learning plan. It is documentation only. `centre` remains
the same range-free yaw/vertical baseline; no `intercept` mode was added.

## 28 September: interactive lifetime and opt-in detailed recording

User requested no scheduled landing, no assistant-initiated simulation exit,
and less routine logging. No simulator or flight was started/stopped this turn.
Defaults are now `duration_s: 0`, `auto_land: false`, `keep_open: true`,
`record: false`. The existing centring mathematics and command interface remain
unchanged; no interception controller was added.

An indefinite session continues its selected behaviour until Ctrl+C. An explicit
positive duration completes the interval and disables centring without landing;
the sequencer remains alive. A failed stream/state check stops sequencing and
requests centring disabled when the service is available, while leaving the
simulation open. Required-component exits are reported without requesting global
shutdown. PX4 failsafes remain active. A failed result remains failed even when
the user subsequently presses Ctrl+C.

Minimal mode keeps configuration, hashes/versions, effective world, runtime PX4
files, calibration, low-rate events and summaries. It links workspace source
instead of duplicating it. `--record` opts into independent source snapshots,
per-frame CSV/JSONL, screenshots, full component output and PX4 ULogs. Minimal
mode passes ROS's external-file-log disable flag and generates a per-run
`px4-rc.params` startup hook that preserves the installed hook then sets
`SDLOG_MODE=-1`. No upstream PX4 files or old run artifacts were changed.
Long sessions use bounded frame-timing storage and aggregate error statistics.

Validation: 11 isolated tests passed, including the original three controller
checks plus indefinite/finite session behavior, explicit landing opt-in, stale
camera failure without exit, minimal/full event recording, bounded statistics
and user interruption. Both launch descriptions were constructed without
executing them; the keep-open process-exit handler was invoked with a synthetic
exit event and returned only a status message. ROS accepted the logging flag.
Both source-link and source-copy snapshot paths were checked in temporary
directories. Python syntax passed. These are lifecycle/configuration checks,
not live flight or ULog-suppression measurements.
