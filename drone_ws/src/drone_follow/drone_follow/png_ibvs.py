"""PNG-IBVS interceptor guidance — arXiv:2409.17497v2 (Yan et al., 2025).

Monocular-only implementation: no depth sensor or range estimate needed.
The algorithm operates entirely on the 2D bearing angle from the camera.

Key insight from the paper:
  PNG shapes the VELOCITY DIRECTION so the Line-of-Sight (LOS) angular rate
  converges to zero. When LOS rate = 0, the interceptor is on a collision
  course with the target. Range is never required for this guarantee.

Data flow each tick:
  bearing (bx, by)           ← detector.py / /target/bearing_camera
  attitude quaternion q       ← odom_msg.q from PX4
        │
        ▼
  bearing_to_ned()            → LOS unit vector in NED
        │
        ▼
  los_angles()                → (q_y elevation, q_z azimuth)
        │
        ▼
  png_update()                → (sigma_yd, sigma_zd) desired velocity angles
        │
        ▼
  intercept_command()         → (velocity_ned [3], yaw_rate)
        │
        ▼
  TrajectorySetpoint          → PX4 Offboard via follower.py
"""

import math
import dataclasses
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# 1. QUATERNION → ROTATION MATRIX
# ─────────────────────────────────────────────────────────────────────────────

def rotation(q):
    """PX4 Hamilton quaternion [w, x, y, z] → body FRD to local NED (3×3).

    PX4 always gives attitude in Hamilton convention with:
      body frame:  FRD  (Forward-Right-Down)
      world frame: NED  (North-East-Down)

    This rotation matrix R satisfies:
      v_NED = R @ v_body

    Usage:
      R = rotation(odom_msg.q)
      los_ned = R @ bearing_in_body_frd
    """
    q = np.asarray(q, dtype=float)
    # Normalise to unit quaternion — PX4 should give a unit quaternion,
    # but floating point transmission can introduce tiny errors.
    norm = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or norm < 0.5:
        raise ValueError(f"Invalid quaternion: {q}")
    w, x, y, z = q / norm

    # Standard formula for rotation matrix from unit quaternion.
    # Each element is derived from expanding R = (w²+x²-y²-z²)I + 2vvᵀ + 2w[v]×
    # where v = [x, y, z].
    return np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - z*w),      2*(x*z + y*w)    ],
        [2*(x*y + z*w),      1 - 2*(x*x + z*z),  2*(y*z - x*w)    ],
        [2*(x*z - y*w),      2*(y*z + x*w),       1 - 2*(x*x + y*y)],
    ])


# ─────────────────────────────────────────────────────────────────────────────
# 2. CAMERA BEARING → LOS UNIT VECTOR IN NED
# ─────────────────────────────────────────────────────────────────────────────

def bearing_to_ned(bx, by, R_body_ned, mount_q=None):
    """Convert normalised pixel bearing to a LOS unit vector in NED.

    Args:
        bx: (u - cx) / fx  — horizontal bearing (right positive in camera)
        by: (v - cy) / fy  — vertical bearing   (down positive in camera)
        R_body_ned: 3×3 rotation matrix from rotation(odom_msg.q)
        mount_q: optional [w,x,y,z] quaternion for camera→body rotation.
                 Pass None (or identity) if camera faces forward, level.

    Returns:
        n_t: (3,) unit vector pointing toward target, in NED frame.

    Step-by-step:
        1. Build a 3D ray in camera OPTICAL frame:
              optical = [bx, by, 1.0]  (right, down, forward)
        2. Convert to camera FRD (Forward-Right-Down):
              frd = [1.0, bx, by]      (forward=optical-z, right=optical-x, down=optical-y)
        3. Normalise to unit vector.
        4. Rotate by camera mount quaternion (if camera is tilted on the body).
        5. Rotate by body→NED rotation matrix from PX4 odometry.
    """
    # Step 1+2: bearing in camera FRD frame.
    # The detector publishes vector.x = bx, vector.y = by, vector.z = 1/norm (unit).
    # We reconstruct the un-normalised ray: forward=1, right=bx, down=by.
    cam_frd = np.array([1.0, bx, by])

    # Step 3: normalise.
    cam_frd = cam_frd / np.linalg.norm(cam_frd)

    # Step 4: apply camera mount rotation (e.g. if camera is tilted down 15°).
    # For a forward-facing level camera, mount_q = [1,0,0,0] → identity → no-op.
    if mount_q is not None:
        R_mount = rotation(np.asarray(mount_q, dtype=float))
        body_frd = R_mount @ cam_frd
    else:
        body_frd = cam_frd  # camera already aligned with body FRD

    # Step 5: rotate from body FRD → NED using PX4 odometry attitude.
    n_t = R_body_ned @ body_frd
    return n_t / np.linalg.norm(n_t)  # ensure unit vector after rotation


# ─────────────────────────────────────────────────────────────────────────────
# 3. LOS UNIT VECTOR → LOS ANGLES (elevation q_y, azimuth q_z)
# ─────────────────────────────────────────────────────────────────────────────

def los_angles(n_t_ned):
    """Decompose a NED LOS unit vector into elevation and azimuth angles.

    Args:
        n_t_ned: (3,) unit vector [north, east, down] pointing to target.

    Returns:
        q_y: elevation angle in radians.  Positive = target is ABOVE horizon.
        q_z: azimuth  angle in radians.  Positive = target is to the EAST (right).

    Why two separate angles?
        The paper splits 3D PNG into two independent 2D PNG controllers:
          - Vertical plane   → q_y → sigma_yd → NED-down velocity component
          - Horizontal plane → q_z → sigma_zd → NED-north/east velocity component

        This decoupling is valid because the two planes are approximately
        orthogonal during a normal interception engagement.

    Convention (NED):
        n = north (+X), e = east (+Y), d = down (+Z)
        Elevation: how far above the horizon is the target?
          q_y = atan2(-d, sqrt(n²+e²))   ← negative because NED down is positive
        Azimuth: which compass direction is the target?
          q_z = atan2(e, n)
    """
    n, e, d = map(float, n_t_ned)
    horiz = math.sqrt(n*n + e*e)

    # Elevation: positive when target is above the horizon (d is negative in NED).
    q_y = math.atan2(-d, horiz)

    # Azimuth: standard compass bearing, 0 = north, pi/2 = east.
    q_z = math.atan2(e, n)

    return q_y, q_z


# ─────────────────────────────────────────────────────────────────────────────
# 4. PERSISTENT STATE
# ─────────────────────────────────────────────────────────────────────────────

@dataclasses.dataclass
class PNGIBVSState:
    """Mutable state carried across ticks.

    PNG is an INTEGRATING controller — the desired velocity angle accumulates
    LOS angle changes over time. Without storing the previous values we can
    only do pure-pursuit (proportional), which causes aggressive late-stage
    corrections and poor accuracy.

    Fields:
        sigma_yd:  Desired velocity elevation angle (integrated, radians).
        sigma_zd:  Desired velocity azimuth angle (integrated, radians).
        prev_q_y:  LOS elevation at previous tick (for finite-difference).
        prev_q_z:  LOS azimuth at previous tick.
        prev_ex:   Horizontal pixel error at previous tick (for yaw PD derivative).
        prev_time: Monotonic time of previous tick (seconds).
        initialised: False until the first measurement has been processed.
    """
    sigma_yd:    float = 0.0
    sigma_zd:    float = 0.0
    prev_q_y:    float = 0.0
    prev_q_z:    float = 0.0
    prev_ex:     float = 0.0
    prev_time:   float = 0.0
    initialised: bool  = False


def make_state():
    """Create a fresh PNGIBVSState. Call this each time following is enabled."""
    return PNGIBVSState()


# ─────────────────────────────────────────────────────────────────────────────
# 5. PNG UPDATE — THE HEART OF THE GUIDANCE LAW
# ─────────────────────────────────────────────────────────────────────────────

def png_update(state, q_y, q_z, dt, Ky=3.0, Kz=3.0):
    """One PNG integration step. Updates state IN-PLACE and returns angles.

    PNG law (continuous):
        d(sigma_d)/dt = K * d(q)/dt

    Discrete equivalent (what we implement):
        sigma_yd[k] = sigma_yd[k-1] + Ky * (q_y[k] - q_y[k-1])
        sigma_zd[k] = sigma_zd[k-1] + Kz * (q_z[k] - q_z[k-1])

    Intuition:
        If LOS is rotating UPWARD at rate dq_y/dt, we tilt our velocity
        direction upward by K times that rate. This DAMPS the LOS rotation
        (like a feedback loop). As LOS rate → 0, sigma_d stops changing.
        The interceptor is then flying straight at the target.

        K=3 is the sweet spot: K<2 → won't converge, K>6 → too aggressive/noisy.
        (Layman et al., also confirmed by paper §III-B for multirotors.)

    Args:
        state:  PNGIBVSState (modified in place).
        q_y, q_z: current LOS angles from los_angles().
        dt:     time since last tick (seconds).
        Ky, Kz: PNG gain constants (paper: 3.0 for both).

    Returns:
        (sigma_yd, sigma_zd): current desired velocity angles.
    """
    if not state.initialised:
        # First tick after enable: seed the state from current LOS.
        # Seed sigma_d to point straight at the target right now.
        state.sigma_yd = q_y
        state.sigma_zd = q_z
        state.prev_q_y = q_y
        state.prev_q_z = q_z    
        state.initialised = True
        return state.sigma_yd, state.sigma_zd

    # Finite-difference LOS rate × gain = how much to steer this tick.
    dq_y = q_y - state.prev_q_y
    dq_z = q_z - state.prev_q_z

    # Wrap azimuth delta to [-π, π] to handle 0°/360° discontinuity.
    dq_z = (dq_z + math.pi) % (2 * math.pi) - math.pi

    # Integrate: accumulate desired velocity angle.
    state.sigma_yd += Ky * dq_y
    state.sigma_zd += Kz * dq_z

    # Physical limits: can't point more than 89° up/down or 180° sideways.
    state.sigma_yd = float(np.clip(state.sigma_yd, -math.pi * 0.49, math.pi * 0.49))
    state.sigma_zd = float(np.clip(state.sigma_zd, -math.pi, math.pi))

    # Store for next tick.
    state.prev_q_y = q_y
    state.prev_q_z = q_z

    return state.sigma_yd, state.sigma_zd


# ─────────────────────────────────────────────────────────────────────────────
# 6. FOV YAW CONTROLLER — PD on horizontal pixel error
# ─────────────────────────────────────────────────────────────────────────────

def fov_yaw_rate(ex, state, dt, kp=0.03, kd=0.01, max_yaw_rate=0.6):
    """PD controller: horizontal pixel error → yaw rate command.

    Why PD not just P?
        Pure proportional (your current monocular.py) has overshoot —
        the drone swings past center. The derivative term damps the
        overshoot, giving a much crisper response.

        Paper equation (§III-A):
            ω_ψ = kp * ex + kd * d(ex)/dt

    Why only horizontal (ex)?
        Yaw decouples from the multicopter's translation dynamics.
        Yaw rate purely rotates the camera left/right. Vertical (ey)
        is handled by the velocity NED-down component.

    Args:
        ex:           Normalised horizontal pixel error = (u - u0) / fx = bx
                      Already available from detector: vector.x / vector.z
        state:        PNGIBVSState (reads/writes prev_ex).
        dt:           Tick period (seconds).
        kp, kd:       PD gains (paper: 0.03, 0.01).
        max_yaw_rate: Rad/s limit.

    Returns:
        yaw_rate: Desired yaw rate in rad/s (positive = turn right/clockwise).
    """
    # Derivative: how fast is the target moving left/right in the image?
    de_x = (ex - state.prev_ex) / dt if dt > 1e-6 else 0.0
    state.prev_ex = ex

    raw = kp * ex + kd * de_x
    return float(np.clip(raw, -max_yaw_rate, max_yaw_rate))


# ─────────────────────────────────────────────────────────────────────────────
# 7. VERTICAL FOV CORRECTION — couples pitch dynamics to vertical velocity
# ─────────────────────────────────────────────────────────────────────────────

def fov_vertical_correction(own_vel_ned, ka=2.0):
    """Compute a small NED-down velocity correction to compensate pitch coupling.

    Why needed?
        When the drone pitches forward to close in, its camera tilts down.
        This makes the target APPEAR to drift upward in the image even if the
        target hasn't moved. Without correction, the drone over-corrects
        altitude and oscillates vertically.

    Paper equation (§III-B):
        a_vertical = ka * v_horizontal

        The dynamics-induced LOS error is bounded by:
            Δq_d ≤ arctan(ka / g)    (≈11° for ka=2, g=9.8)

    Args:
        own_vel_ned: (3,) current NED velocity [north, east, down] m/s.
        ka:          gain (paper: 2.0, range 1–3).

    Returns:
        delta_down: small NED-down correction in m/s (positive = descend).
                    This is added to the PNG velocity command each tick.
    """
    v_horiz = math.sqrt(own_vel_ned[0]**2 + own_vel_ned[1]**2)
    # A forward speed of 3 m/s → delta_down = 2 * 3 * 0.05 ≈ 0.3 m/s per tick.
    # (The 0.05 dt factor is applied in intercept_command so this returns rate.)
    return float(ka * v_horiz)


# ─────────────────────────────────────────────────────────────────────────────
# 8. TOP-LEVEL: INTERCEPT COMMAND
# ─────────────────────────────────────────────────────────────────────────────

def intercept_command(bx, by, state, R_body_ned, own_vel_ned, dt,
                      approach_speed=3.0, Ky=3.0, Kz=3.0,
                      kp=0.03, kd=0.01, ka=2.0,
                      max_speed=4.0, max_yaw_rate=0.6, max_vertical=2.0,
                      mount_q=None):
    """Full IBVS+PNG command for one 20 Hz tick.

    Args:
        bx, by:        Normalised bearing from detector (vector.x/z, vector.y/z).
        state:         PNGIBVSState — modified in place each tick.
        R_body_ned:    3×3 rotation from rotation(odom_msg.q).
        own_vel_ned:   (3,) current NED velocity from odom_msg.velocity.
        dt:            Seconds since last tick.
        approach_speed: Desired closing speed in m/s (v_d in paper).
        Ky, Kz:        PNG gains (default 3.0 — paper recommendation).
        kp, kd:        FOV yaw PD gains (default 0.03, 0.01 — paper values).
        ka:            Vertical FOV correction gain (default 2.0 — paper).
        max_speed:     Hard NED velocity magnitude limit (m/s).
        max_yaw_rate:  Hard yaw rate limit (rad/s).
        max_vertical:  Hard NED-down speed limit (m/s).
        mount_q:       Camera mount quaternion (None = identity = forward-facing).

    Returns:
        vel_ned:   (3,) numpy array — desired NED velocity [north, east, down] m/s.
        yaw_rate:  float — desired yaw rate in rad/s.

    Full equation chain:
        bx, by → bearing_to_ned() → n_t [NED]
               → los_angles()     → q_y, q_z
               → png_update()     → sigma_yd, sigma_zd
               → n_vd (desired velocity direction unit vector)
               × approach_speed   → vel_ned (before corrections)
               + fov_vertical_correction() → final vel_ned
        bx     → fov_yaw_rate()   → yaw_rate
    """
    if dt <= 0 or dt > 0.5:
        # Stale or backwards clock — do nothing safe.
        return np.zeros(3), 0.0

    # ── Step 1: Camera bearing → LOS unit vector in NED ──────────────────────
    n_t = bearing_to_ned(bx, by, R_body_ned, mount_q)

    # ── Step 2: LOS vector → elevation and azimuth angles ────────────────────
    q_y, q_z = los_angles(n_t)

    # ── Step 3: PNG integration → desired velocity direction angles ───────────
    sigma_yd, sigma_zd = png_update(state, q_y, q_z, dt, Ky, Kz)

    # ── Step 4: Build the desired velocity DIRECTION unit vector from angles ──
    # Paper eq. (8):
    #   n_vd = [cos(σ_yd)*sin(σ_zd),   ← north
    #           cos(σ_yd)*cos(σ_zd),   ← east   NOTE: paper's y-axis is east
    #          -sin(σ_yd)]             ← down (negative: up is positive elevation)
    #
    # NED convention: north=X, east=Y, down=Z
    # sigma_yd is elevation (positive = above horizon → velocity points up → NED-down < 0)
    # sigma_zd is azimuth   (positive = east of north)
    cos_y = math.cos(sigma_yd)
    n_vd = np.array([
        cos_y * math.cos(sigma_zd),   # north (X in NED)
        cos_y * math.sin(sigma_zd),   # east  (Y in NED)
        -math.sin(sigma_yd),           # down  (Z in NED, negative = upward)
    ])

    # ── Step 5: Scale by approach speed to get velocity command ──────────────
    vel_ned = approach_speed * n_vd

    # ── Step 6: FOV vertical correction — compensate pitch-coupling ──────────
    # This adds a small downward drift proportional to forward speed to keep
    # the target from drifting in the image due to pitch attitude change.
    # The correction is tiny (≪ approach_speed) and bounded by max_vertical.
    v_corr_rate = fov_vertical_correction(own_vel_ned, ka)
    # Apply as a small per-tick delta to NED-down component.
    vel_ned[2] = float(np.clip(vel_ned[2] + v_corr_rate * dt,
                                -max_vertical, max_vertical))

    # ── Step 7: Hard speed limit on full 3D vector ───────────────────────────
    speed = float(np.linalg.norm(vel_ned))
    if speed > max_speed:
        vel_ned = vel_ned * (max_speed / speed)

    # ── Step 8: FOV yaw PD controller ────────────────────────────────────────
    # ex = bx (horizontal bearing = normalised pixel error, already computed by detector)
    yaw_rate = fov_yaw_rate(bx, state, dt, kp, kd, max_yaw_rate)

    return vel_ned, yaw_rate
