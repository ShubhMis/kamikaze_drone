"""Lifecycle tests for the single follower controller."""

from types import SimpleNamespace
import unittest

from drone_follow.follower import Follower
from drone_follow.png_ibvs import PNGIBVSState


class FollowerLifecycleTests(unittest.TestCase):
    def follower(self):
        follower = Follower.__new__(Follower)
        follower.enabled = False
        follower.png_state = None
        follower.target_count = 6
        follower.healthy = lambda: True
        follower.in_offboard = lambda: True
        follower.fresh = lambda: True
        return follower

    def test_enable_always_creates_fresh_png_state(self):
        follower = self.follower()
        response = SimpleNamespace(success=None, message='')
        Follower.enable(follower, SimpleNamespace(data=True), response)
        first = follower.png_state
        self.assertTrue(response.success)
        self.assertTrue(follower.enabled)
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
        self.assertIsNone(follower.png_state)

    def test_loss_reset_can_require_fresh_detections(self):
        follower = self.follower()
        follower.enabled = True
        follower.png_state = PNGIBVSState(initialised=True)
        Follower.reset_guidance(follower, clear_target_count=True)
        self.assertFalse(follower.enabled)
        self.assertIsNone(follower.png_state)
        self.assertEqual(follower.target_count, 0)


if __name__ == '__main__':
    unittest.main()

