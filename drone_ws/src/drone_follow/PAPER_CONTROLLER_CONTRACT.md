# Paper Controller Contract: Research Draft

Status: **CURRENT SINGLE-CONTROLLER BINDING COMPLETE; PAPER REPRODUCTION INCOMPLETE**

Prepared: 2026-10-01. Implementation binding updated: 2026-10-05.

This document records the supplied paper's terminology, the active software binding, and unresolved specification questions. It is the contract for the current single-controller velocity-interface reconstruction. It does not claim that the current code reproduces the paper's complete body-rate/lift controller.

Current implementation priority is the controller algorithm and command interface. DKF reconstruction is **DEFERRED** until the controller gaps in Eqs. (8)-(23) are resolved and replay-tested.

## 1. Source and evidence

Hailong Yan, Kun Yang, Yixiao Cheng, Zihao Wang, and Dawei Li, *Precise Interception Flight Targets by Image-based Visual Servoing of Multicopter*, arXiv:2409.17497v2, dated 4 April 2025 on the supplied PDF.

- Local source: [2409.17497v2.pdf](/home/crl/Downloads/Papers/2409.17497v2.pdf).
- File SHA-256: `3439d323bb19e0e7c79e4ba34dc44af356aff32866f44ed641a0bfe4727e8b1f`.
- Page references below use the PDF's physical page order, starting at 1.
- Equations (3)-(8): page 3.
- Equations (9)-(23): page 4; the explanation following (23) continues on page 5.
- Pages 3-5 were visually inspected to check notation against text extraction.
- Reference [28] was identified in the bibliography on page 10. Its full text was not supplied or reviewed here.

Evidence labels:

| Label | Meaning |
|---|---|
| PAPER | Explicitly stated or visibly printed in the supplied PDF. |
| ANALYSIS | A reading or dimensional observation made in this review. |
| OPEN | The supplied source does not settle the question. |
| NOT SPECIFIED | The source does not provide the requested software-level detail. |

An ANALYSIS entry must not be presented as an author-confirmed correction. NOT SPECIFIED does not mean zero, identity, or a default value.

## 2. Scope of the method described in the paper

**PAPER, Sections II-D and III-A through III-C, pages 3-4:** the method combines guidance, field-of-view feedback, and attitude/lift control. Figure 3 includes target detection and a delayed Kalman filter upstream of control.

**PAPER, Equation (11) and adjacent prose, page 4:** the horizontal image error is driven toward the image centre, while the vertical image excursion is bounded. These are different objectives; the vertical objective is not stated as convergence of vertical error to zero.

**ANALYSIS:** the reviewed sections do not define a preliminary CENTRE state, a centring dwell time, or CENTRE/INTERCEPT switching thresholds. Those features cannot be attributed to these equations.

**PAPER, Section III-D, page 5:** the stability argument has assumptions about initial visibility, target manoeuvrability, and relative motion. A conditional theoretical argument is not evidence that an arbitrary software reconstruction is equivalent to the paper.

## 3. Equation register

This register accounts for every equation from (3) through (23). It identifies the subject of each equation without supplying an executable control sequence.

| Equation | PDF page | Subject | Review status |
|---|---:|---|---|
| (3) | 3 | Definition of image-plane error | Definition explicit; software timing unspecified. |
| (4) | 3 | Image-motion interaction matrix | OPEN: pixel/normalized error notation, issue Q4. |
| (5) | 3 | Geometric line-of-sight direction | Mathematical frame names provided; software conventions unspecified. |
| (6) | 3 | Guidance relationship between angular rates | PAPER; numerical realization not specified here. |
| (7) | 3 | Line-of-sight angle definitions | PAPER; general singular-case conventions not supplied. |
| (8) | 3 | Velocity-direction angle definitions | OPEN at zero velocity, issue Q8. |
| (9) | 4 | Discrete desired-angle expression | Uses current/previous indices; timestamp semantics unspecified. |
| (10) | 4 | Desired velocity magnitude and direction | Distinguishes scalar speed from vector velocity. |
| (11) | 4 | Horizontal error and vertical excursion objectives | PAPER; not a two-stage state-machine definition. |
| (12) | 4 | Horizontal image-motion relationship | OPEN: same normalization issue as (4). |
| (13) | 4 | Horizontal image feedback | Error/gain unit interpretation depends on Q4. |
| (14) | 4 | Desired-speed expression | OPEN: dimensional/time interpretation, issue Q2. |
| (15) | 4 | Components of vertical line-of-sight variation | PAPER; does not define a software state transition. |
| (16) | 4 | Relation between vertical image excursion and field of view | Depends on the stated image geometry and Q4. |
| (17) | 4 | Desired acceleration expression | OPEN: meaning and timing of `dt`, issue Q8. |
| (18) | 4 | Desired lift-direction expression | Normalization is printed; degenerate-case behavior unspecified. |
| (19) | 4 | Desired attitude and incremental rotation | OPEN: rotation-axis interpretation, issue Q5. |
| (20) | 4 | Attitude Lyapunov candidate | OPEN: transpose inconsistency with adjacent expressions, issue Q6. |
| (21) | 4 | Derivative used in attitude analysis | Read alongside Q6; not independently validated here. |
| (22) | 4 | Attitude-related angular-velocity term | OPEN: component labels across (22)-(23), issue Q7. |
| (23) | 4-5 | Angular-velocity and lift outputs with bounds | OPEN: notation, dimensional consistency, and external interface, Q3/Q7/Q9. |

## 4. Reading dictionary

These are paper-level meanings, not ROS message fields or implementation variable assignments. Units marked ANALYSIS follow the stated physical meaning or dimensional interpretation; the paper does not provide a complete unit dictionary.

| Notation | Meaning in the paper | Units or mathematical type | Evidence/location |
|---|---|---|---|
| Superscript `i` | Image pixel coordinate system | Image coordinates | PAPER, Section II-C, p. 3 |
| Superscript `c` | Camera coordinate system | Spatial frame | PAPER, Sections II-A/II-C, pp. 2-3 |
| Superscript `b` | Body coordinate system | Spatial frame | PAPER, Sections II-A/II-B, pp. 2-3 |
| Superscript `e` | Earth-fixed coordinate system | Spatial frame | PAPER, Sections II-A/II-B, pp. 2-3 |
| `u`, `v` | Target image position components | Pixels | PAPER, Section II-C |
| `u0`, `v0` | Reference image-centre components | Pixels | PAPER, Eq. (3) and prose |
| `e`, `ex`, `ey` | Image error vector and components | Pixels under Eq. (3); see Q4 for later usage | PAPER/OPEN, Eqs. (3)-(4) |
| Barred `e`, `ex`, `ey` | Normalized image-error quantities | Dimensionless | PAPER, prose after Eq. (4) |
| `foc` | Camera focal length in the image model | Pixel-compatible scale in the printed normalization | ANALYSIS, Eqs. (4)-(5) |
| `Ls` | Image-motion interaction matrix | Matrix with columns of different dimensions | PAPER/OPEN, Eq. (4), Q4 |
| `q`, `qy`, `qz` | Line-of-sight angles | Angular quantities | PAPER, Eqs. (6)-(7) |
| `sigma` and its components | Velocity-direction angles | Angular quantities | PAPER, Eqs. (6), (8)-(9) |
| `nt`, `nv` and components | Line-of-sight and velocity-direction unit vectors | Dimensionless vectors | PAPER, Eqs. (5), (7)-(8) |
| `K`, `Ky`, `Kz` | Guidance proportionality parameters | Dimensionless in the stated angular relationships | PAPER/ANALYSIS, Eqs. (6), (9) |
| Subscript `d` | Desired quantity in the relevant expression | Inherits the quantity's units | PAPER, Eqs. (9)-(23) |
| `vd`, `vnow` | Desired/current speed or velocity, depending on scalar/vector typography | Speed/velocity quantities | PAPER; preserve boldface distinction, Eqs. (10), (14), (17) |
| `ka` | Parameter in the desired-speed discussion | OPEN | PAPER, Eq. (14) and adjacent prose; Q2 |
| `kp`, `kd` | Horizontal-feedback parameters | OPEN until image-error convention is settled | PAPER, Eq. (13); Q4 |
| `Delta ey`, `epsilon` | Vertical image excursion and its bound | Pixel quantities under Eq. (3)'s convention | PAPER/ANALYSIS, Eqs. (11), (16) |
| `Delta qd`, `Delta qg`, `Delta qy` | Angular variations discussed in FOV analysis | Angular quantities | PAPER, Eq. (15) and prose |
| `alpha_vfov` | Vertical camera field-of-view angle | Angle | PAPER, Eq. (16) |
| `ad`, `g` | Desired acceleration and gravitational acceleration | Acceleration | PAPER, Eqs. (2), (17)-(18) |
| `m` | Vehicle mass | Mass | PAPER, Eq. (2) and prose |
| `f`, `fd`, `fm` | Lift-related quantities and maximum lift | Force under the physical model; see Q3 | PAPER, Eqs. (2), (23) |
| `nf`, `nfd` | Lift directions | Dimensionless direction vectors | PAPER, Eqs. (18)-(19) |
| `R`, frame-labelled `R`, `Rd`, `Rtilt` | Rotations and desired/incremental rotations | Dimensionless matrices | PAPER, Eqs. (5), (19)-(22) |
| `r`, `phi` | Rotation-axis-related quantity and rotation angle | Vector and angle; axis convention OPEN | PAPER, Eq. (19); Q5 |
| `L1` | Attitude Lyapunov candidate | Dimensionless scalar | PAPER/ANALYSIS, Eq. (20); Q6 |
| `omega`, labelled omega terms, `w_psi`, `omega_m` | Angular velocity quantities and bound | Angular velocity; component assignment OPEN | PAPER, Eqs. (13), (21)-(23), p. 5; Q7 |
| `k`, `k-1` | Current and previous sample indices | Dimensionless indices | PAPER, Eq. (9) |
| `t`, `dt`, dot over a quantity | Time, time interval, and time derivative | Time and corresponding rate units | PAPER/OPEN, Eqs. (6), (17), (21); Q8 |
| `I`, transpose, norm, trace, cross-product matrix, `vex` | Linear-algebra notation/operators | Mathematical operators | PAPER, Eqs. (2), (5), (19)-(23) |

This dictionary groups related components. It is not a certified, exhaustive symbol-to-code mapping. In particular, a frame superscript `e` must not be confused with the image-error symbol `e`.

## 5. Completed example record: horizontal image error

- **Symbol:** `ex`.
- **Meaning:** horizontal target-image displacement relative to the reference image centre.
- **Source:** Eq. (3), Section II-C, PDF p. 3.
- **Descriptive analysis label:** `image_error_x_px`; this is a proposed document label, not an existing code binding.
- **Type:** scalar component of a two-component image-error vector.
- **Units:** pixels under the Eq. (3) definition.
- **Frame:** image pixel coordinate system.
- **Sign:** follows increasing `u`; the paper does not specify a particular camera driver's axis mapping.
- **Timestamp:** NOT SPECIFIED as a software field. The distinction between a captured observation and a predicted estimate remains necessary for interpreting a result.
- **Producer:** conceptual image-position observation and reference centre; concrete software owner NOT SPECIFIED.
- **Consumer:** mathematical image-error relationships; concrete software owner NOT SPECIFIED.
- **Update rate:** no per-symbol sampling contract supplied. Figure 3's block labels do not establish this signal's full timestamp semantics.
- **Initialization:** NOT SPECIFIED.
- **Reset behavior:** NOT SPECIFIED.
- **Validity:** depends on a defined observation and reference image coordinate system.
- **Evidence:** PAPER for the definition; OPEN for later normalization and software details.
- **Unresolved question:** Q4, distinguishing pixel error from normalized image error in subsequent expressions.

## 6. Active software binding

The active ROS package contains one visual-control path:

```text
/target/bearing_camera
        -> follower.py
        -> png_ibvs.intercept_command()
        -> PX4 TrajectorySetpoint velocity + yawspeed
```

There is no controller-mode parameter, centring fallback, dwell timer, hysteresis, or CENTRE/INTERCEPT transition. `DISABLED`, `INTERCEPTING`, target-loss and PX4-health labels are lifecycle conditions only.

### 6.1 Inputs, frames, units and timestamps

| Software value | Producer | Frame and units | Timestamp/update semantics | Consumer |
|---|---|---|---|---|
| `Vector3Stamped /target/bearing_camera` | `detector.py` | Unit ray in `follow_camera_optical`: x right, y down, z forward | Original image header; nominal camera rate 20 Hz | `follower.target()` |
| `bx = ray.x/ray.z`, `by = ray.y/ray.z` | `follower.py` | Dimensionless normalized image coordinates | Recovered from the latest accepted bearing | `intercept_command()` |
| `error_u_px`, `error_v_px` | `detector.py` diagnostics | Image pixels relative to `CameraInfo` principal point | Original image timestamp | Experiment/viewer only; the active controller does not consume them |
| aligned own attitude | PX4 `VehicleOdometry.q` history | Hamilton quaternion; body FRD to local NED | Quaternion SLERP at the accepted image timestamp; bounded latest-sample hold only for minor callback skew | `rotation()` |
| aligned own velocity | PX4 `VehicleOdometry.velocity` history | NED m/s | Linear interpolation at the accepted image timestamp; same bounded hold policy | Current-speed magnitude used by the Eq. (14) reconstruction |
| `camera_mount_q` | `follow.yaml` | Camera FRD to body FRD quaternion | Static run parameter | `bearing_to_ned()` |
| `vel_ned` | `intercept_command()` | NED m/s | Follower timer, currently 20 Hz | PX4 `TrajectorySetpoint.velocity` |
| `yaw_rate` | `intercept_command()` | rad/s, PX4 NED yaw-rate convention | Follower timer, currently 20 Hz | PX4 `TrajectorySetpoint.yawspeed` |

PX4 publication-to-sample age is subtracted from ROS receipt time so odometry
history remains in the image clock domain without assuming equal absolute PX4
and ROS epochs. DDS transport delay remains an unmeasured residual. A distinct
image timestamp advances the guidance state at most once; intervening 20 Hz
setpoint ticks repeat the last command. Camera translation is not used when
forming the direction-only LOS.

### 6.2 Persistent state and reset contract

`PNGIBVSState` currently owns `sigma_yd`, `sigma_zd`, previous LOS angles, previous horizontal normalized error, the previous control time, and an initialization flag. A fresh state is created on every successful `/follow/enable=true` request.

`Follower.reset_guidance()` clears the whole state and disables pursuit on:

- explicit disable;
- PX4 estimator reset;
- invalid odometry/status health;
- leaving an allowed PX4 streaming state or armed Offboard;
- simulation-time discontinuity; or
- stale/lost target measurement.

After target loss, the detection counter is also cleared. When bounded
automatic reacquisition is configured, visual-only loss retains the operator's
pursuit request while publishing zero velocity. The configured number of new
detections must arrive before a fresh controller state is created. Timeout,
explicit disable, estimator reset, health failure, PX4-mode departure, and
clock discontinuity cancel that request. No previous LOS, desired-angle, or
derivative history survives reacquisition. This lifecycle behavior is an
engineering policy, not a behavior specified by the paper.

### 6.3 Equation-to-code conformance matrix

| Equation | Current code binding | Status |
|---|---|---|
| (3) | `detector.py` computes pixel errors; `follower.py` consumes normalized coordinates | Partial: both representations exist, but only normalized error reaches control |
| (4) | No interaction-matrix implementation | Missing |
| (5) | `bearing_to_ned()` constructs and rotates the camera ray using exposure-time-aligned attitude | Partial: geometric ray/rotation is implemented, but camera translation and the paper's complete upstream timing contract are not |
| (6) | Approximated by the discrete update in `png_update()` | Partial |
| (7) | `los_angles()` uses `atan2` in NED | Implemented with explicit NED convention |
| (8) | No measured-velocity-direction calculation | Missing, including zero-speed handling |
| (9) | `png_update()` accumulates the previous desired angles | Mismatch: the paper anchors the update to the previous measured velocity angles |
| (10) | `intercept_command()` builds a direction and multiplies it by the bounded desired-speed magnitude | Partial; direction is implemented, while the downstream paper controller remains absent |
| (11) | Horizontal FOV feedback exists | Partial; the stated vertical-excursion objective is not implemented |
| (12) | No image interaction equation | Missing |
| (13) | `fov_yaw_rate()` applies PD feedback | Partial: it uses normalized `bx`, while the paper's gain/error convention is unresolved |
| (14) | `desired_speed = clip(||vnow|| + speed_increment_mps, 0, max_speed)` | Partial: literal bounded software reconstruction; `ka` is explicitly interpreted as an m/s increment because the paper's unit/time contract is ambiguous |
| (15)-(16) | No vertical-excursion controller | Missing; the former unsupported downward velocity bias has been removed |
| (17)-(23) | PX4 velocity/yaw-rate interface substitutes for desired acceleration, lift direction, attitude, body rates and physical lift | Missing from the current controller |

Consequently, the active module is named and reported as a **single PNG-IBVS velocity-interface reconstruction**. It must not be described as a complete reproduction of Eqs. (3)-(23).

### 6.4 Rates

| Block | Paper Figure 3 | Current project |
|---|---:|---:|
| Camera | 20 Hz | 20 Hz configured in the follower model |
| Detector/DKF | 50 Hz | Detector follows camera arrivals; no DKF |
| IMU | 100 Hz | PX4/Gazebo rate is platform-configured; controller retains and aligns odometry to image time |
| IBVS controller | 200 Hz | 20 Hz (`create_timer(0.05, ...)`) |

Repeating a stored camera bearing on another timer invocation is not a new
observation. The implementation keys guidance updates by the accepted image
timestamp and reuses the last command between observations.

### 6.5 Active parameters

| Parameter | Units | Current role | Paper status |
|---|---|---|---|
| `png_gain_y`, `png_gain_z` | dimensionless | LOS-angle increment gain | Related to Eq. (9), but current state anchor differs |
| `fov_kp` | rad/s per normalized-error unit | Yaw proportional term | Error-unit convention differs from the printed pixel definition |
| `fov_kd` | rad per normalized-error unit | Yaw derivative term | Same convention issue |
| `speed_increment_mps` | m/s | Increment in `clip(||vnow|| + speed_increment_mps, 0, max_speed)` | Explicit software interpretation of `ka` in Eq. (14); author-intended units remain open |
| `max_speed` | m/s | Velocity magnitude saturation | Engineering bound |
| `max_vertical_speed` | m/s | NED-down component saturation | Engineering bound |
| `max_yaw_rate` | rad/s | Yaw-rate saturation | Engineering/PX4 bound |

## 7. Unresolved-question register

### Q1. Delayed Kalman filter specification — DEFERRED

**PAPER:** Section II-D, p. 3, refers to DKF work [28].

**DEFERRED:** the supplied paper does not provide a complete standalone estimator specification covering its state, measurement model, covariance definitions, delayed-update handling, initialization, and resets. No DKF work is part of the current algorithm phase.

Bibliographic entry, p. 10: K. Yang, C. Bai, Z. She, and Q. Quan, *High-speed interception multicopter control by image-based visual servoing*, IEEE Transactions on Control Systems Technology, vol. 33, no. 1, pp. 119-135, 2025.

**Evidence required:** the full reference or an authoritative supplementary specification. Identifying its title does not mean its contents have been reviewed.

### Q2. Equation (14): units and elapsed time

**PAPER:** the printed desired-speed expression adds `ka` to current speed. Nearby prose also uses `ka/g` in an angular bound.

**ANALYSIS:** these appearances do not establish one unambiguous physical unit for `ka` under ordinary dimensional reading. The role of elapsed time is not explicit in Eq. (14).

**IMPLEMENTED SOFTWARE POLICY:** `ka` is represented by
`speed_increment_mps` and used in the bounded expression
`clip(||vnow|| + speed_increment_mps, 0, max_speed)`. This is the closest
literal reconstruction used by this project; it is not a claim that the paper
specified m/s units. Author-intended units and time interpretation remain
OPEN.

### Q3. Equation (23): physical dimensions

**PAPER:** Eq. (2) identifies lift as a force, and Eq. (17) identifies `ad` as acceleration. The printed lift expression in Eq. (23) contains a subtraction involving `ad` and `m g`.

**ANALYSIS:** acceleration and mass times acceleration have different dimensions. The printed expression needs clarification against the preceding definitions.

**OPEN:** whether this is a typographical issue, a normalization omitted from the presentation, or another convention. No force formula is repaired here.

### Q4. Equations (4), (12), and related feedback: pixel versus normalized error

**PAPER:** Eq. (3) defines an image-pixel error; prose after Eq. (4) separately defines normalized error. The interaction matrix uses normalized coordinates while the derivative on the left is written without the corresponding bar.

**ANALYSIS:** that notation leaves the intended derivative units unclear. Consequently, numerical feedback gains cannot be interpreted independently of the chosen image-error convention.

**OPEN:** the intended pixel/normalized convention throughout these expressions. A focal length expressed in physical sensor units must not silently be equated with a focal length expressed in image pixels.

### Q5. Equation (19): rotation representation

**PAPER:** the axis-related quantity is introduced as a cross product and used in a rotation expression. The nearby prose also contains `Rtitle` while the displayed expression uses `Rtilt`.

**OPEN:** the intended axis convention, treatment of degenerate directions, and consistency of the rotation description. A corrected rotation construction is not supplied in this review.

### Q6. Equations (20)-(21): transpose consistency

**PAPER:** Eq. (20) prints the candidate without a transpose on `Rd`; the following norm identity and derivative use a transposed `Rd`.

**OPEN:** the authoritative intended definition and associated derivation. These expressions should not be declared mutually verified based only on a visual similarity.

### Q7. Equations (22)-(23) and following prose: angular-velocity labels

**PAPER:** Eq. (22) assigns an attitude-related term to `omega_1`; Eq. (23) combines two terms, while the following page again uses `omega_1` for a different described contribution.

**OPEN:** the intended labels and component conventions. The paper's symbol names alone do not establish an autopilot-axis mapping.

### Q8. Sampling, timestamps, and initialization

**PAPER:** Eq. (9) uses current and previous sample indices; Eq. (17) uses `dt`; Figure 3 labels subsystem rates.

**IMPLEMENTED SOFTWARE POLICY:** accepted image stamps identify distinct
guidance samples. Own attitude and velocity are aligned to each image stamp;
the first controller interval is 50 ms; subsequent intervals use consecutive
image stamps; and simulation-time discontinuities clear both controller and
own-state history. This is an explicit reconstruction policy, not a claim that
the paper specified these software details. Zero-velocity direction and the
paper's intended multi-rate correspondence remain OPEN.

### Q9. Mathematical outputs versus an external interface

**PAPER:** Section II-D names a flight controller as the downstream component. The reviewed equations do not specify a versioned software API or the conversion to a particular command representation.

**OPEN:** implementation equivalence at that boundary. Physical force and a normalized command have different meanings. No PX4 command conversion, calibration, or command-publication logic is included here.

### Q10. Reproduction evidence

**ANALYSIS:** resolving notation in a document does not itself demonstrate equivalence of an implementation. This review has not inspected an authors' implementation or established closed-loop equivalence.

**OPEN:** authoritative answers to Q1-Q9 and independent reproduction evidence. No system is certified as paper-faithful by this document.

## 8. Specification status

| Requested item | Status in this draft |
|---|---|
| Source version and page references | Recorded. |
| Equation coverage, (3)-(23) | Every equation indexed. |
| General notation and units | Reading dictionary supplied; ambiguities labelled. |
| Exact symbol-to-code bindings | Recorded for the active velocity-interface reconstruction. |
| Concrete software producers/consumers | Recorded in Section 6. |
| Timestamp and update semantics | Current behavior recorded; paper-equivalent alignment remains open under Q8. |
| Initialization and reset behavior | Current software behavior recorded; author-intended behavior remains unspecified. |
| DKF reconstruction | DEFERRED until the controller algorithm is complete; reference [28] not reviewed. |
| Corrected control laws or numerical gains | Not supplied. |
| Autopilot output conversion | OPEN; no interface mapping supplied. |
| Exact reproduction claim | Not supported by this review. |

## 9. Review completion

- [x] Identify the exact supplied PDF and its checksum.
- [x] Visually inspect the pages containing Eqs. (3)-(23).
- [x] Account for all equations in the requested range.
- [x] Separate printed definitions from review observations.
- [x] Record the bibliographic identity of reference [28].
- [x] Keep unresolved dimensions and notation visible.
- [x] Bind current code values, frames, units, rates and resets.
- [x] Mark every current departure from Eqs. (3)-(23).
- [ ] Obtain authoritative clarification of the unresolved questions.
- [ ] Establish any claim of exact reproduction.

For this artifact, use the description **implementation contract for the current single-controller velocity reconstruction, with unresolved paper-equivalence questions**. It is not evidence that the complete paper controller has been reproduced.
