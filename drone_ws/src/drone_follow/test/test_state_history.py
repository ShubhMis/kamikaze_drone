"""Tests for exposure-time own-state alignment."""

import math
import unittest

import numpy as np

from drone_follow.state_history import (
    VehicleStateHistory,
    odometry_sample_time,
    quaternion_slerp,
)


class StateHistoryTests(unittest.TestCase):
    def test_px4_sample_age_is_mapped_into_ros_receive_time(self):
        stamp = odometry_sample_time(
            receive_time_s=12.0,
            timestamp_us=5_000_000,
            timestamp_sample_us=4_980_000,
            max_sample_age_s=0.1,
        )
        self.assertAlmostEqual(stamp, 11.98)

    def test_invalid_sample_age_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'sample age'):
            odometry_sample_time(12.0, 5_000_000, 4_000_000, 0.1)

    def test_interpolates_velocity_and_attitude_at_exposure(self):
        history = VehicleStateHistory(duration_s=2.0)
        q0 = [1.0, 0.0, 0.0, 0.0]
        q1 = [math.cos(math.pi/4), 0.0, 0.0, math.sin(math.pi/4)]
        history.add(1.0, q0, [0.0, 0.0, 0.0])
        history.add(2.0, q1, [2.0, 4.0, 6.0])

        state = history.at(1.5)
        self.assertAlmostEqual(state.stamp_s, 1.5)
        np.testing.assert_allclose(state.velocity, [1.0, 2.0, 3.0])
        expected = [math.cos(math.pi/8), 0.0, 0.0, math.sin(math.pi/8)]
        np.testing.assert_allclose(state.q, expected, atol=1e-10)

    def test_quaternion_sign_does_not_create_long_rotation(self):
        result = quaternion_slerp([1.0, 0.0, 0.0, 0.0],
                                  [-1.0, 0.0, 0.0, 0.0], 0.5)
        np.testing.assert_allclose(result, [1.0, 0.0, 0.0, 0.0])

    def test_out_of_history_measurement_is_rejected(self):
        history = VehicleStateHistory(duration_s=1.0, max_extrapolation_s=0.02)
        history.add(1.0, [1, 0, 0, 0], [0, 0, 0])
        history.add(2.0, [1, 0, 0, 0], [0, 0, 0])
        self.assertIsNone(history.at(0.99))
        self.assertIsNone(history.at(2.03))
        self.assertIsNotNone(history.at(2.02))

    def test_non_monotonic_state_is_rejected(self):
        history = VehicleStateHistory()
        history.add(2.0, [1, 0, 0, 0], [0, 0, 0])
        with self.assertRaisesRegex(ValueError, 'monotonic'):
            history.add(1.0, [1, 0, 0, 0], [0, 0, 0])


if __name__ == '__main__':
    unittest.main()
