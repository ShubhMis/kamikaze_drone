"""Sign, command limits and simple closed-loop yaw checks; not flight proof."""
import math
import unittest
from drone_follow.monocular import centring_command


class MonocularTests(unittest.TestCase):
    def command(self, x, y):
        return centring_command(x, y, .8, .6, .3, .4)

    def test_frame_signs(self):
        down, yaw = self.command(.2, -.1)
        self.assertGreater(yaw, 0)  # target right: positive NED heading change
        self.assertLess(down, 0)   # target above: climb (negative NED down)

    def test_level_target_holds_and_bounds(self):
        self.assertEqual(self.command(0, 0), (0, 0))
        for x in (-100, 100):
            down, yaw = self.command(x, x)
            self.assertLessEqual(abs(down), .4)
            self.assertLessEqual(abs(yaw), .3)
        with self.assertRaises(ValueError):
            self.command(float('nan'), 0)

    def test_stationary_bearing_converges_in_ideal_yaw_plant(self):
        # Fixed target, no translation, ideal yaw-rate inner loop: error_dot=-yaw.
        error = math.radians(15)
        for _ in range(200):
            _, yaw = self.command(math.tan(error), 0)
            error -= yaw*.05
        self.assertLess(abs(error), math.radians(.01))
