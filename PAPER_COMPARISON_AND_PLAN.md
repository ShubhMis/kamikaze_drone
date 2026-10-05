# Simulation comparison and implementation plan

Reviewed 29 September 2026; active implementation status updated 5 October 2026. Scope: compare the executable project with arXiv:2409.17497v2 and keep delayed estimation and learned detection out of the current controller refactor.

The project is a useful PX4/ROS 2/Gazebo visual-control foundation. The active package now has one PNG-IBVS velocity-interface controller and no centre/intercept selector. It remains an unvalidated adaptation of some paper concepts, not a faithful implementation of the complete paper. `drone_ws/src/drone_follow/PAPER_CONTROLLER_CONTRACT.md` is authoritative for equation-level implementation status.

## 1. Reference and interpretation

Reference: Yan et al., *Precise Interception Flight Targets by Image-based Visual Servoing of Multicopter*, supplied [v2 PDF](/home/crl/Downloads/Papers/2409.17497v2.pdf), dated 4 April 2025. Page numbers below are PDF pages.

The paper studies interception of a visible airborne target. It combines monocular image measurements, own-vehicle state, proportional-navigation-based guidance, field-of-view (FOV) control, and delayed image estimation. It does not implement damage diagnosis, stable inspection distance, physical repair, or mission replacement. Its target information is monocular; that does not mean the whole vehicle uses only a camera. Own attitude, velocity, and inertial measurements are also needed.

Figure 3, page 3, gives the architecture: camera -> detector and delayed Kalman filter -> PNG/IBVS controller -> desired body angular velocity and lift -> flight controller -> motors. It labels the camera at 20 Hz, IMU at 100 Hz, detector/DKF block at 50 Hz, and IBVS controller at 200 Hz. Those labels distinguish rates of sensing, estimation, and control; they do not mean there are 50 new camera images each second.

The final controller is in Eqs. (17)-(23), pages 4-5. Desired velocity in Eq. (10) is an intermediate quantity. Stopping the implementation there and sending velocity to PX4 changes the closed-loop system.

The FOV objective in Eq. (11) is horizontal centering plus bounded vertical image excursion. It does not require both image coordinates to converge to zero. This distinction must appear in evaluation metrics.

The stability discussion, page 5, assumes initial visibility and bounded target maneuverability and relies on superior follower speed. A modified, delayed, sampled controller with different saturation and PX4 loops does not inherit that argument automatically. Constant bearing by itself also does not establish decreasing separation; the target can be receding.

The paper's reported 0.089 m CEP comes from 50 static-target simulation trials (pages 5-6). CEP is a 50th-percentile radial interception-error measure, not general position RMSE or guaranteed accuracy. Table II (page 8) reports 100%, 80%, 80%, and 40% success for 0, 2, 4, and 6 m/s wind, respectively, with five indoor trials per condition; those trials use a balloon. These results do not establish damaged-drone assistance performance. The abstract's percentage improvement and the main text's comparison numbers also differ, so cite the exact table/metric rather than repeating one percentage as universal.

## 2. What your current code actually implements

The authoritative implementation is `drone_ws/src/drone_follow/drone_follow/`, its launch files, and `config/follow.yaml`. The obsolete alternate-controller code dump and two-stage design note have been removed from the active tree; Git history retains them.

| Component | Current implementation | Assessment against the paper |
|---|---|---|
| Simulator and autopilot | Two PX4 X500 SITL vehicles, ROS 2, Gazebo | Appropriate infrastructure. The paper uses RflySim; changing simulators is unnecessary for a methodological comparison, but exact numerical reproduction requires documenting model differences. |
| Camera | Rigid forward-facing 640x480 RGB camera, 20 Hz, 80-degree horizontal FOV | Suitable monocular baseline. Intrinsics, FOV, mounting and dynamics must be reported as experimental differences. |
| Target detector | HSV thresholding; accepts exactly one red connected component | Controlled marker detector. The paper uses target detection with DKF and cites YOLOv7; your component does not demonstrate general drone recognition. The paper's real tests also use red balloons. |
| Measurement | Calibrated optical unit ray, timestamp retained | Correctly represents a bearing, with no metric range. `vector.z` is not depth. |
| Coordinate transform | Optical right/down/forward -> camera FRD -> body -> NED | Coherent for the current level forward camera. Main remaining issue is time alignment, not a demonstrated wholesale axis-sign error. |
| Guidance | Stored desired angles updated from LOS changes | Paper-inspired, but not shown equivalent to the measured-velocity-angle expression in Eq. (9). |
| Horizontal FOV control | PD feedback on normalized horizontal error | Paper defines pixel error and normalized error separately. Direct reuse of its numeric gains is not justified without reconciling units. |
| Vertical FOV control | Downward velocity bias proportional to own horizontal speed and timer interval | The claimed paper equation in the comment does not appear in the supplied paper. This is a substantive implementation departure. |
| Delay handling | Rejects stale measurements; uses latest odometry | No DKF, image-time attitude interpolation, or delayed-observation correction. |
| Output | NED velocity plus yaw speed through `TrajectorySetpoint` at 20 Hz | Keeps PX4 velocity and attitude loops. Different from final body-rate/thrust output in the paper. |
| Leader behavior | Hover; optional slow planar figure-eight service | The experiment sequencer does not activate the motion service. Current runs do not cover the paper's moving-target experiments. |
| Evaluation | Visibility, image error, session completion | No valid evidence yet of paper-level performance. Full command/trajectory logging and explicit success definitions are missing. |

The existing separation of detector, follower adapter, pure controller math, leader, viewer, and experiment sequencer is worth preserving. Freshness checks, odometry reset checks, armed-Offboard gating, and suspension of external setpoints during autopilot takeoff/landing are useful existing features.

## 3. Current findings

1. **Controller-selection ambiguity is resolved.** `follow.yaml` contains no controller `mode`; the flight scenario is named `intercept`; enabling always creates fresh PNG state; no centring fallback exists. `observe` still does not arm, and `hover` does not enable visual pursuit.

2. **The control interface is different.** [follower.py:262](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/drone_follow/follower.py:262) publishes velocity-mode `OffboardControlMode` and `TrajectorySetpoint`. A `yawspeed` field alongside velocity is not the paper's body angular-rate vector. This is a legitimate baseline if labeled as an adaptation; it is not Eq. (23).

3. **The vertical correction is unsupported.** [png_ibvs.py:303](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/drone_follow/png_ibvs.py:303) claims the paper gives vertical acceleration proportional to horizontal speed. The supplied paper instead discusses desired speed and attitude-induced FOV motion in Eqs. (14)-(18). [The implementation at line 407](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/drone_follow/png_ibvs.py:407) always adds a positive NED-down term during horizontal motion. This added correction has no vertical image-error feedback and changes with timer interval; the separate LOS guidance does use vertical bearing. The correction cannot be attributed to the paper as written.

4. **Image and attitude timestamps are mixed.** [follower.py:218](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/drone_follow/follower.py:218) transforms a stored image bearing using the newest odometry attitude. During camera rotation, this can create apparent world-frame bearing changes that are timing artifacts. Reusing that bearing on another timer tick is not a new camera observation.

5. **The yaw gain convention is unresolved.** [png_ibvs.py:281](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/drone_follow/png_ibvs.py:281) specifies normalized error, while paper Eq. (3) defines pixel error and Eq. (13) uses that symbol. Numeric gain equality does not imply equivalent feedback. Because the paper also mixes conventions around Eq. (4), resolve this explicitly rather than blindly multiplying gains.

6. **Guidance state is not established as equivalent.** [png_ibvs.py:245](/home/crl/Desktop/Shubh/kamikaze/drone_ws/src/drone_follow/drone_follow/png_ibvs.py:245) accumulates previous desired angles. Eq. (9) uses the previous velocity angle, whose definition in Eq. (8) comes from vehicle velocity. Desired and measured direction differ when the vehicle lags or saturates. Document any approximation and test it before claiming equivalence.

7. **Experiment labeling is corrected, while success semantics remain limited.** Results identify the single PNG-IBVS velocity reconstruction and record north/east/down commands. A completed interval still proves process completion rather than paper-level tracking or interception performance.

8. **Bearing-only sensing is not itself a paper mismatch.** The paper deliberately uses target angles. Adding target position telemetry or depth to the guidance loop would change its sensing assumptions. For future assistance, an independently justified relative range/pose estimate will be needed to certify distance, but that design decision is deferred here. Simulator truth can be used for evaluation without feeding it into control.

## 4. Evidence from your saved experiments

Thirteen historical runs on 28 September used `follower.mode: intercept`: eleven failed while the old sequencer called the active phase `CENTRING`, two were interrupted in `WAITING` or `TAKEOFF`, and none passed. These labels describe the historical source snapshot, not the current single-controller configuration.

The package manifest hashes for the first failed run, `20260928_115415_747624`, and latest failed run, `20260928_164927_477079`, match the current package source files. These failures therefore apply to the current source at launch; they are not merely failures of an unrelated older implementation.

The [latest result](/home/crl/Desktop/Shubh/kamikaze/runs/20260928_164927_477079/result.json:2) contains 490 images, 482 marked visible, about 20 Hz camera rate, and active-stage first/last pixel errors of approximately `[79.73, -171.28]` and `[-250.41, 15.49]`. It reports aggregate image-error RMS of 103.79 px. Active control lasted about 6.8 simulation seconds before the failure.

The high whole-session visibility fraction includes periods outside active following and does not prove stable closed-loop tracking. All thirteen runs used minimal recording, so full histories are unavailable. The result cannot establish whether the underlying event was a FOV exit, perception ambiguity, collision, or another cause. Do not infer the physical cause from the generic failure string.

The pre-refactor audit reported 36 isolated tests. The current suite is rerun and reported with the source change that produced it; test counts are not treated as flight evidence. These tests do not launch Gazebo or prove closed-loop tracking, DKF performance, or paper equivalence. Several math tests check the current formulas against their own expected outputs; an incorrect paper interpretation can still pass them.

## 5. Implementation sequence

The deliverables and acceptance gates below are proposed engineering criteria, not requirements or results quoted from the paper. Keep each stage reviewable and preserve earlier baselines.

### Stage 1: Make the baseline unambiguous and reproducible — completed for controller selection

The active package uses one controller, one `intercept` scenario name, and explicit disable/reset/re-enable behavior. The former centring module and test are retained only in Git history. Effective configuration, source hashes, PX4 version, simulator version and termination reason continue to be saved with recorded runs.

Add complete north/east/down velocity commands, yaw requests, measured velocity/attitude, controller state, image exposure time, arrival time, used-state time, and dropped-frame reasons to recording. Keep wall-time process supervision separate from simulation-time dynamics. Split “session completed” from “control objective achieved.” Make research recordings immutable snapshots rather than relying on a live source symlink.

Acceptance status: there is no controller-selection combination to mismatch; `centre` is rejected by the launcher; `intercept` is the only visual-flight scenario. A fresh recorded SITL run is still required to validate the refactored runtime end to end.

### Stage 2: Establish the mathematical contract before editing control laws

Create a short equation-to-code design note for Eqs. (3)-(23): every symbol, unit, coordinate frame, observation timestamp, update rate, and source of state must be explicit. Separate pixels from normalized coordinates, camera/body/world directions, desired from measured velocity, and physical force from normalized actuator demand. Document the camera translation as well as rotation.

Resolve the following printed ambiguities rather than silently guessing: pixel versus normalized interaction equations; the velocity increment/time-step convention around Eqs. (14)/(17); rotation-axis normalization and degenerate rotations in Eq. (19); the inconsistent rotation transpose around Eq. (20); and the mass, acceleration, angular-vector label/axis conventions around Eq. (23). A notation inconsistency is not proof that the authors' experimental code had the same error.

Acceptance: each retained term has a source or is labeled as your adaptation, each expression is dimensionally consistent, and independent geometric examples establish the frame conventions. A review of the printed equations alone is insufficient to assert the original implementation's undocumented choices.

### Stage 3: Validate geometry and timestamps independently of guidance

Add a pure geometry module and an own-state history buffer. Transform each bearing using the vehicle orientation corresponding to the image timestamp, with explicit time-base conversion and interpolation. Reject measurements outside the available history and process each distinct observation once. A predictor may run between observations, but must label its output as predicted.

Use stationary-scene replay with changing yaw, pitch and roll to verify that camera rotation is removed consistently. Account for camera motion about its nonzero mounting offset when making close-range geometric comparisons. Compare projected pixel locations against known scene geometry and test nonidentity orientations, not only level flight.

Acceptance: camera rotation does not generate unexplained inertial target motion; frame transformations agree with independently generated geometry; duplicate/out-of-order/late data have defined behavior. Choose tolerances from calibration and simulation precision and document them before running the sweep.

### Stage 4: Add and evaluate delayed visual estimation

Keep `detector.py` as a replaceable measurement source and add a dedicated estimator module/node with timestamped outputs and uncertainty. First preserve the red-marker detector so changes in perception do not confound the estimator comparison. Later a learned detector can replace it behind the same interface.

Retrieve and study the paper's cited DKF source, reference [28], *High-speed Interception Multicopter Control by Image-based Visual Servoing*, before claiming the same DKF. The supplied paper references this earlier estimator rather than fully specifying it. Specify the state, process/measurement models, noise assumptions, and delayed-update procedure. An ordinary filter applied at arrival time is not automatically a DKF.

Evaluate raw observations and estimated current observations against the same timestamped replay truth under controlled delay, jitter, dropout and outliers. Keep a history sufficient for the supported delay; handle updates older than that history explicitly. Measure prediction error as well as smoothness, since a smoother signal can still be badly delayed.

Acceptance: the supported delay range, accuracy, uncertainty behavior, and reset/reacquisition policy are demonstrated. Delayed data must not silently be treated as current data. A replay comparison identifies which change helped and which cases became worse.

### Stage 5: Keep control-interface choices explicit

Preserve the current velocity-interface path as a separately named paper-inspired baseline. Reassess unsupported terms using the mathematical contract from Stage 2. Do not relabel a velocity/yaw request as the paper's complete controller. Keep pure model/controller work separate from the ROS/PX4 adapter so numerical behavior can be assessed offline.

A closer architecture comparison requires the paper's final attitude-control stage and a separately validated body-rate/thrust adapter. This is a change in which loops PX4 retains, not merely a message rename. Establish conventions, thrust-model calibration, limits, and hover behavior on a single-vehicle simulation before evaluating any two-vehicle task. Desired force in newtons cannot be copied to normalized `thrust_body`.

The local PX4 checkout reports `v1.15.4-4-g85df8c2281`. Its commander code enables velocity, attitude, rate, and allocation stages in velocity Offboard mode; body-rate mode retains rate control and allocation. This agrees with the [PX4 v1.15 Offboard documentation](https://docs.px4.io/v1.15/en/flight_modes/offboard). Pin the software/message versions for experiments.

Acceptance: chosen interface and retained loops are recorded; single-vehicle hover and small bounded orientation/velocity responses are characterized; timing overruns, saturation, mode transition, and stale-command behavior are measured. A new adapter cannot claim the paper's closed-loop guarantees based on interface similarity alone.

### Stage 6: Build a fair tracking and FOV comparison

Extend `leader.py` and the experiment configuration with explicit, reproducible hover and prescribed-motion scenarios. The present optional movement service must actually be invoked by the chosen scenario; merely defining it is not a moving-target experiment. Keep truth in a separate evaluation node and put both vehicles in a verified common frame before subtracting positions; separate PX4 local origins cannot be assumed identical.

Start with stationary target observation and bounded tracking runs, then controlled slow target motion and changes in initial bearing. Sweep perception delay/noise and visibility conditions separately before combining them. Use identical scene seeds, sensor assumptions, initial states and actuator limits for comparisons. When comparing with the paper, document its static and CV/CA/sinusoidal motion categories, initial conditions, and simulator/model differences explicitly.

Run ablations that isolate current baseline, timing correction, estimator, and justified FOV-control changes. These isolate contributions better than changing all modules at once. Predeclare the metrics and thresholds, retain every failed run, and report per-scenario results with the number of trials and uncertainty. A proposed starting batch is 30 repeatable runs per condition; it is a study-design choice, not a sufficiency guarantee. The paper's static experiment used 50.

Acceptance: figures can be regenerated from saved data; failures and visibility loss have recorded causes or explicit unknown labels; advantages persist across repeated runs rather than one favorable video. If the experiment deliberately stops short or uses a tracking-only objective, label it accordingly and do not compare its error directly to interception CEP.

### Stage 7: Decide the assistance task after the comparison

At this point choose inspection at separation, cooperative rendezvous, mission handover, or physical docking as a new objective. Each changes the state estimate, completion criteria, and assumptions. Stable distance and near-zero relative velocity are not outcomes provided by the paper's interception objective. There is no reason to select or implement those behaviors during this comparison review.

Acceptance: a written task specification defines what “help” means, what malfunction states are in scope, and what observable result counts as success. Then design and evaluate that extension independently of the paper-reproduction claim.

## 6. Metrics to record before another comparison campaign

| Metric | Purpose |
|---|---|
| Controller identity and lifecycle phase | Confirm the fixed controller and distinguish disabled, active and failed intervals. |
| Horizontal error RMS/percentiles | Quantify horizontal centering in pixels and normalized coordinates. |
| Vertical excursion and FOV margin | Evaluate the paper's stated visibility objective; do not demand vertical zero error by default. |
| Per-active-phase visibility and longest dropout | Avoid preflight observations inflating tracking performance. |
| Exposure-to-control age and timer jitter | Characterize the delay problem the DKF is intended to address. |
| Estimated versus truth bearing at matched times | Test geometry/estimation independently of controller performance. |
| Commanded versus measured velocity/attitude/rates | Separate guidance error from actuator and inner-loop lag. |
| Saturation fraction and acceleration | Identify unachievable commands and loss of smoothness. |
| Evaluator-only separation and relative velocity | Understand the relative trajectory without leaking target truth into monocular control. |
| Completion, abort and failure counts | Separate technical session completion from objective success. |

## 7. What can currently be claimed

Supported: two-vehicle PX4 SITL infrastructure, monocular red-marker bearing extraction, one active PNG-inspired velocity controller, explicit lifecycle reset behavior, and isolated tests.

Not supported by current evidence: faithful reproduction of the complete paper; equivalent DKF or FOV behavior; the paper's accuracy/stability claims; successful current 3D closed-loop tracking; autonomous inspection, assistance, repair, or mission replacement.

The next implementation task is the controller-math gap in the contract: measured velocity angles and Eq. (9), pixel/normalized FOV units, the Eq. (14) speed interpretation, and eventually Eqs. (17)-(23). Detector replacement and DKF are deliberately deferred until that algorithm path is coherent and replay-tested.
