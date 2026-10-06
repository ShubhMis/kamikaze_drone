"""Unit tests for png_ibvs.py — run without ROS or Gazebo.

Covers every function in the module. Tests are ordered from simplest
(rotation matrix) to most integrated (intercept_command).

Run with:
    cd ~/drone_ws/src/drone_follow
    python3 -m unittest test.test_png_ibvs -v
"""
import math
import unittest
import numpy as np
import sys
import os

# Allow running from repo root without installing.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from drone_follow.png_ibvs import (
    rotation, bearing_to_ned, los_angles,
    PNGIBVSState, make_state, png_update,
    fov_yaw_rate, intercept_command,
)


class TestRotation(unittest.TestCase):
    """rotation(q): quaternion → 3×3 rotation matrix."""

    def test_identity_quaternion(self):
        """Identity quaternion [1,0,0,0] should give identity matrix."""
        R = rotation([1.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(R, np.eye(3), atol=1e-10,
                                   err_msg="Identity quaternion must give identity matrix")

    def test_yaw_90_deg(self):
        """90° yaw (around NED-down axis) rotates north → east.
        Quaternion for 90° yaw: [cos45°, 0, 0, sin45°] = [√2/2, 0, 0, √2/2].
        A vector pointing north (body +X = NED +N) should become east (NED +E).
        """
        half = math.pi / 4
        q = [math.cos(half), 0.0, 0.0, math.sin(half)]  # 90° yaw
        R = rotation(q)
        body_north = np.array([1.0, 0.0, 0.0])  # forward in body FRD
        world_result = R @ body_north
        # After 90° yaw, forward should point EAST (NED +Y).
        np.testing.assert_allclose(world_result, [0.0, 1.0, 0.0], atol=1e-10,
                                   err_msg="90° yaw should rotate north → east")

    def test_unit_quaternion_normalisation(self):
        """Slightly non-unit quaternion should be normalised, not raise."""
        q = [1.001, 0.0, 0.0, 0.0]  # norm ≈ 1.001
        R = rotation(q)
        # Result should still be approximately identity.
        np.testing.assert_allclose(R, np.eye(3), atol=1e-3)

    def test_invalid_quaternion_raises(self):
        """Near-zero quaternion (norm < 0.5) should raise ValueError."""
        with self.assertRaises(ValueError):
            rotation([0.0, 0.0, 0.0, 0.1])

    def test_output_is_proper_rotation(self):
        """Output must be orthogonal (R @ Rᵀ = I) and det = +1."""
        q = [math.cos(0.3), math.sin(0.3)*0.6, math.sin(0.3)*0.8, 0.0]
        R = rotation(q / np.linalg.norm(q))
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10,
                                   err_msg="Rotation matrix must be orthogonal")
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=10,
                               msg="Determinant of rotation matrix must be +1")


class TestBearingToNED(unittest.TestCase):
    """bearing_to_ned(): camera bearing → LOS unit vector in NED."""

    def test_straight_ahead_identity_attitude(self):
        """Target directly ahead (bx=0, by=0), drone level facing north.
        Expect LOS → north = [1, 0, 0] in NED.
        """
        R = np.eye(3)  # no rotation — body FRD aligned with NED
        n_t = bearing_to_ned(0.0, 0.0, R)
        np.testing.assert_allclose(n_t, [1.0, 0.0, 0.0], atol=1e-10,
                                   err_msg="Boresight bearing should point north with identity attitude")

    def test_unit_vector_output(self):
        """Output must always be a unit vector regardless of bearing."""
        R = np.eye(3)
        for bx, by in [(0.5, 0.3), (-0.2, 0.8), (0.0, -0.5)]:
            n_t = bearing_to_ned(bx, by, R)
            self.assertAlmostEqual(np.linalg.norm(n_t), 1.0, places=10,
                                   msg=f"bearing_to_ned({bx},{by}) must return unit vector")

    def test_target_right_goes_east(self):
        """Target to the right (bx > 0) with drone facing north → LOS has +east component."""
        R = np.eye(3)
        n_t = bearing_to_ned(1.0, 0.0, R)
        self.assertGreater(n_t[1], 0.0,
                           msg="Target to the right (bx>0) should have positive east component")

    def test_target_below_goes_down(self):
        """Target below camera (by > 0) with drone level → LOS has +down component."""
        R = np.eye(3)
        n_t = bearing_to_ned(0.0, 1.0, R)
        self.assertGreater(n_t[2], 0.0,
                           msg="Target below camera (by>0) should have positive NED-down component")


class TestLOSAngles(unittest.TestCase):
    """los_angles(): NED unit vector → (q_y elevation, q_z azimuth)."""

    def test_north_bearing_zero_angles(self):
        """LOS pointing due north → elevation=0, azimuth=0."""
        q_y, q_z = los_angles([1.0, 0.0, 0.0])
        self.assertAlmostEqual(q_y, 0.0, places=10)
        self.assertAlmostEqual(q_z, 0.0, places=10)

    def test_east_bearing_azimuth_90(self):
        """LOS pointing due east → elevation=0, azimuth=π/2."""
        q_y, q_z = los_angles([0.0, 1.0, 0.0])
        self.assertAlmostEqual(q_y, 0.0, places=10)
        self.assertAlmostEqual(q_z, math.pi / 2, places=10)

    def test_target_above_positive_elevation(self):
        """LOS pointing up-north → positive elevation angle."""
        # NED-up = negative NED-down (d < 0).
        n_t = np.array([1.0, 0.0, -1.0])
        n_t /= np.linalg.norm(n_t)
        q_y, q_z = los_angles(n_t)
        self.assertGreater(q_y, 0.0,
                           msg="Target above horizon should give positive elevation angle")

    def test_roundtrip_consistency(self):
        """los_angles then recover direction vector should give same bearing."""
        # Pick an arbitrary NED direction.
        q_y_in = 0.3   # elevation
        q_z_in = 0.8   # azimuth
        # Reconstruct NED vector from angles (same formula as intercept_command).
        n = math.cos(q_y_in) * math.cos(q_z_in)
        e = math.cos(q_y_in) * math.sin(q_z_in)
        d = -math.sin(q_y_in)
        n_t = np.array([n, e, d])
        q_y_out, q_z_out = los_angles(n_t)
        self.assertAlmostEqual(q_y_out, q_y_in, places=10)
        self.assertAlmostEqual(q_z_out, q_z_in, places=10)


class TestPNGUpdate(unittest.TestCase):
    """png_update(): discrete PNG integration."""

    def test_first_tick_seeds_from_current_los(self):
        """On the first call, sigma_d should match current q exactly (no history)."""
        state = make_state()
        q_y, q_z = 0.3, 0.5
        sy, sz = png_update(state, q_y, q_z, dt=0.05)
        self.assertAlmostEqual(sy, q_y, places=10,
                               msg="First tick sigma_yd should equal initial q_y")
        self.assertAlmostEqual(sz, q_z, places=10,
                               msg="First tick sigma_zd should equal initial q_z")

    def test_constant_los_no_change(self):
        """If LOS is constant (no rotation), sigma_d should not change after seeding."""
        state = make_state()
        q_y, q_z = 0.2, 0.4
        png_update(state, q_y, q_z, dt=0.05)   # seed
        sy1, sz1 = png_update(state, q_y, q_z, dt=0.05)
        sy2, sz2 = png_update(state, q_y, q_z, dt=0.05)
        self.assertAlmostEqual(sy1, sy2, places=10,
                               msg="Constant LOS should not change sigma_yd")
        self.assertAlmostEqual(sz1, sz2, places=10,
                               msg="Constant LOS should not change sigma_zd")

    def test_rotating_los_steers_sigma(self):
        """Increasing LOS angle should drive sigma_d in the same direction."""
        state = make_state()
        # Start at q_y = 0.1, rotate upward each tick.
        png_update(state, 0.1, 0.0, dt=0.05)  # seed
        sy1, _ = png_update(state, 0.15, 0.0, dt=0.05)
        sy2, _ = png_update(state, 0.20, 0.0, dt=0.05)
        self.assertGreater(sy2, sy1,
                           msg="Upward-rotating LOS should increase sigma_yd")

    def test_sigma_clamped_to_physical_limits(self):
        """Even with extreme LOS rates, sigma must stay within physical limits."""
        state = make_state()
        png_update(state, 0.0, 0.0, dt=0.05)  # seed
        # Feed a huge LOS jump.
        sy, sz = png_update(state, math.pi, math.pi * 2, dt=0.05, Ky=100.0, Kz=100.0)
        self.assertLessEqual(abs(sy), math.pi * 0.49 + 1e-9)
        self.assertLessEqual(abs(sz), math.pi + 1e-9)


class TestFOVYawRate(unittest.TestCase):
    """fov_yaw_rate(): PD yaw controller on pixel error."""

    def test_zero_error_zero_output(self):
        state = make_state()
        state.prev_ex = 0.0
        yr = fov_yaw_rate(0.0, state, dt=0.05)
        self.assertAlmostEqual(yr, 0.0, places=10)

    def test_positive_error_positive_yaw(self):
        """Target to the right (bx > 0) → positive yaw rate (turn right)."""
        state = make_state()
        state.prev_ex = 0.0
        yr = fov_yaw_rate(0.5, state, dt=0.05)
        self.assertGreater(yr, 0.0)

    def test_derivative_term_damps_overshoot(self):
        """When error is reducing (approaching zero), derivative is negative,
        which reduces the yaw command vs pure proportional."""
        state = make_state()
        state.prev_ex = 0.8  # was far right
        # Now error is 0.5 — error reduced, so de/dt < 0
        yr_pd = fov_yaw_rate(0.5, state, dt=0.05, kp=0.03, kd=0.01)
        yr_p_only = 0.03 * 0.5  # pure proportional baseline
        self.assertLess(yr_pd, yr_p_only,
                        msg="Derivative term should reduce command when error is decreasing")

    def test_clamped_to_max(self):
        state = make_state()
        state.prev_ex = 0.0
        yr = fov_yaw_rate(100.0, state, dt=0.05, max_yaw_rate=0.6)
        self.assertAlmostEqual(yr, 0.6, places=5)


class TestInterceptCommand(unittest.TestCase):
    """intercept_command(): end-to-end integration test."""

    def test_target_straight_ahead_velocity_northward(self):
        """Target directly ahead (bx=by=0), drone level facing north.
        Expect velocity to be primarily northward (positive NED-north).
        """
        state = make_state()
        R = np.eye(3)
        own_vel = np.zeros(3)
        vel, yr = intercept_command(
            bx=0.0, by=0.0, state=state, R_body_ned=R,
            own_vel_ned=own_vel, dt=0.05, speed_increment_mps=3.0,
        )
        self.assertGreater(vel[0], 2.0,
                           msg="Straight-ahead target should produce mainly northward velocity")
        self.assertAlmostEqual(yr, 0.0, places=3,
                               msg="Centred target should produce near-zero yaw rate")

    def test_velocity_magnitude_bounded(self):
        """Velocity magnitude must never exceed max_speed."""
        state = make_state()
        R = np.eye(3)
        own_vel = np.array([2.0, 0.5, 0.0])
        for bx, by in [(0.5, -0.3), (-0.8, 0.2), (0.0, 0.9)]:
            vel, _ = intercept_command(
                bx=bx, by=by, state=state, R_body_ned=R,
                own_vel_ned=own_vel, dt=0.05,
                speed_increment_mps=5.0, max_speed=4.0,
            )
            speed = np.linalg.norm(vel)
            self.assertLessEqual(speed, 4.0 + 1e-9,
                                 msg=f"Speed {speed:.3f} exceeds max_speed for bx={bx}, by={by}")

    def test_eq14_speed_increment(self):
        state = make_state()
        vel, _ = intercept_command(
            bx=0.0, by=0.0, state=state, R_body_ned=np.eye(3),
            own_vel_ned=np.array([1.5, 0.0, 0.0]), dt=0.05,
            speed_increment_mps=0.5, max_speed=4.0,
        )
        self.assertAlmostEqual(np.linalg.norm(vel), 2.0, places=10)

    def test_level_target_has_no_unconditional_downward_bias(self):
        state = make_state()
        vel, _ = intercept_command(
            bx=0.0, by=0.0, state=state, R_body_ned=np.eye(3),
            own_vel_ned=np.array([2.0, 0.0, 0.0]), dt=0.05,
            speed_increment_mps=1.0,
        )
        self.assertAlmostEqual(vel[2], 0.0, places=10)

    def test_stale_dt_returns_zero(self):
        """dt <= 0 or > 0.5 should return zero velocity safely."""
        state = make_state()
        R = np.eye(3)
        vel, yr = intercept_command(0.1, 0.1, state, R, np.zeros(3), dt=-0.1)
        np.testing.assert_array_equal(vel, [0.0, 0.0, 0.0])
        self.assertEqual(yr, 0.0)

    def test_yaw_rate_clamped(self):
        """Extreme bearing error should be clamped to max_yaw_rate."""
        state = make_state()
        R = np.eye(3)
        _, yr = intercept_command(
            bx=50.0, by=0.0, state=state, R_body_ned=R,
            own_vel_ned=np.zeros(3), dt=0.05,
            max_yaw_rate=0.6,
        )
        self.assertLessEqual(abs(yr), 0.6 + 1e-9)


if __name__ == '__main__':
    unittest.main(verbosity=2)
