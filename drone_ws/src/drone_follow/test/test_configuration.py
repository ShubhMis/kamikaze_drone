"""Single-controller configuration regression tests."""

import copy
from pathlib import Path
import re
import sys
import unittest

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))

import baseline
from drone_follow.configuration import validate_controller_parameters


class ConfigurationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load((PACKAGE/'config/follow.yaml').read_text())
        cls.parameters = cls.config['follower']['ros__parameters']

    def test_current_configuration_is_valid_and_single_controller(self):
        validate_controller_parameters(self.parameters)
        self.assertEqual(baseline.SCENARIOS, ('observe', 'hover', 'intercept'))
        self.assertNotIn('centre', baseline.SCENARIOS)
        self.assertNotIn('mode', self.parameters)

    def test_obsolete_selector_and_centre_keys_are_rejected(self):
        for key, value in (
            ('mode', 'intercept'),
            ('yaw_gain', 0.8),
            ('vertical_gain', 0.7),
            ('centre_hold_time_s', 0.5),
            ('recenter_error_threshold', 0.15),
            ('max_vertical_speed_intercept', 2.0),
        ):
            with self.subTest(key=key):
                parameters = copy.deepcopy(self.parameters)
                parameters[key] = value
                with self.assertRaisesRegex(ValueError, 'obsolete'):
                    validate_controller_parameters(parameters)

    def test_unknown_and_invalid_values_are_rejected(self):
        parameters = copy.deepcopy(self.parameters)
        parameters['unexpected_controller'] = 1
        with self.assertRaisesRegex(ValueError, 'unknown'):
            validate_controller_parameters(parameters)

        parameters = copy.deepcopy(self.parameters)
        parameters['approach_speed'] = parameters['max_speed'] + 1
        with self.assertRaisesRegex(ValueError, 'must not exceed'):
            validate_controller_parameters(parameters)

        parameters = copy.deepcopy(self.parameters)
        parameters['camera_mount_q'] = [0.0, 0.0, 0.0, 0.0]
        with self.assertRaisesRegex(ValueError, 'invalid norm'):
            validate_controller_parameters(parameters)

    def test_every_follower_parameter_lookup_is_declared(self):
        source = (PACKAGE/'drone_follow/follower.py').read_text()
        lookups = set(re.findall(r"self\.p\('([^']+)'\)", source))
        self.assertLessEqual(lookups, set(self.parameters))
        self.assertNotIn('mode', lookups)
        self.assertNotIn('centring_command', source)


if __name__ == '__main__':
    unittest.main()

