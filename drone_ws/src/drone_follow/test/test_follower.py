"""Lifecycle tests for the single follower controller."""

from types import SimpleNamespace
import unittest

from drone_follow.follower import Follower
from drone_follow.png_ibvs import PNGIBVSState
import numpy as np


class FollowerLifecycleTests(unittest.TestCase):
    def follower(self):
        follower = Follower.__new__(Follower)
        follower.enabled = False
        follower.pursuit_requested = False
        follower.lost_since = None
        follower.png_state = None
        follower.target_count = 6
        follower.last_processed_target_stamp = None
        follower.last_aligned_state_stamp = None
        follower.last_command_velocity = np.ones(3)
        follower.last_command_yaw_rate = 0.2
        follower.healthy = lambda: True
        follower.in_offboard = lambda: True
        follower.fresh = lambda: True
        parameters = {'auto_reacquire': True, 'reacquire_timeout_s': 5.0}
        follower.p = parameters.__getitem__
        return follower

    def test_enable_always_creates_fresh_png_state(self):
        follower = self.follower()
        response = SimpleNamespace(success=None, message='')
        Follower.enable(follower, SimpleNamespace(data=True), response)
        first = follower.png_state
        self.assertTrue(response.success)
        self.assertTrue(follower.enabled)
        self.assertTrue(follower.pursuit_requested)
        self.assertIsInstance(first, PNGIBVSState)

        Follower.enable(follower, SimpleNamespace(data=True), response)
        self.assertIsInstance(follower.png_state, PNGIBVSState)
        self.assertIsNot(follower.png_state, first)

    def test_disable_clears_all_controller_state(self):
        follower = self.follower()
        follower.enabled = True
        follower.png_state = PNGIBVSState(initialised=True)
        response = SimpleNamespace(success=None, message='')
        Follower.enable(follower, SimpleNamespace(data=False), response)
        self.assertTrue(response.success)
        self.assertFalse(follower.enabled)
        self.assertFalse(follower.pursuit_requested)
        self.assertIsNone(follower.lost_since)
        self.assertIsNone(follower.png_state)
        np.testing.assert_array_equal(follower.last_command_velocity, np.zeros(3))
        self.assertEqual(follower.last_command_yaw_rate, 0.0)

    def test_loss_reset_can_require_fresh_detections(self):
        follower = self.follower()
        follower.enabled = True
        follower.png_state = PNGIBVSState(initialised=True)
        Follower.reset_guidance(follower, clear_target_count=True)
        self.assertFalse(follower.enabled)
        self.assertIsNone(follower.png_state)
        self.assertEqual(follower.target_count, 0)

    def test_visual_loss_arms_bounded_reacquisition_with_fresh_state(self):
        follower = self.follower()
        follower.enabled = True
        follower.pursuit_requested = True
        old_state = PNGIBVSState(initialised=True)
        follower.png_state = old_state

        Follower.begin_reacquisition(follower, now=12.0)
        self.assertFalse(follower.enabled)
        self.assertTrue(follower.pursuit_requested)
        self.assertTrue(Follower.reacquiring(follower))
        self.assertEqual(follower.target_count, 0)
        self.assertIsNone(follower.png_state)
        np.testing.assert_array_equal(follower.last_command_velocity, np.zeros(3))

        Follower.start_guidance(follower)
        self.assertTrue(follower.enabled)
        self.assertFalse(Follower.reacquiring(follower))
        self.assertIsInstance(follower.png_state, PNGIBVSState)
        self.assertIsNot(follower.png_state, old_state)

    def test_reacquisition_timeout_is_bounded(self):
        follower = self.follower()
        follower.pursuit_requested = True
        follower.lost_since = 10.0
        self.assertFalse(Follower.reacquisition_expired(follower, 15.0))
        self.assertTrue(Follower.reacquisition_expired(follower, 15.01))

    def test_auto_reacquisition_can_be_disabled(self):
        follower = self.follower()
        follower.enabled = True
        follower.pursuit_requested = True
        follower.p = {'auto_reacquire': False, 'reacquire_timeout_s': 5.0}.__getitem__
        Follower.begin_reacquisition(follower, now=12.0)
        self.assertFalse(follower.pursuit_requested)
        self.assertFalse(Follower.reacquiring(follower))

    def test_each_target_stamp_is_consumed_only_once(self):
        follower = self.follower()
        follower.target_msg = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=50)))
        self.assertTrue(Follower.has_unprocessed_target(follower))
        follower.last_processed_target_stamp = Follower.target_stamp(follower)
        self.assertFalse(Follower.has_unprocessed_target(follower))


if __name__ == '__main__':
    unittest.main()
