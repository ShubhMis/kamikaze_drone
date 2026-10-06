"""Sequencer checks without ROS initialization, sockets, PX4 or Gazebo."""
from collections import deque
import io
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml
from px4_msgs.msg import VehicleStatus
from drone_follow.experiment import Experiment, takeoff_altitude


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        # Call the real sequencing/statistics methods without starting a Node.
        self.node = Experiment.__new__(Experiment)
        n = self.node
        n.run = Path(self.directory.name)
        config = Path(__file__).resolve().parents[1]/'config/follow.yaml'
        n.c = yaml.safe_load(config.read_text())['baseline']
        n.c['scenario'] = 'intercept'
        n.record = False
        n.started = n.stage_start = time.monotonic()-100
        n.stage = 'INTERCEPTING'
        n.sim_time = 10.
        n.done = n.halted = False
        n.exit_code = 1
        n.metrics = {'enabled': True}
        n.enable = Mock()
        n.command = Mock()
        n.events = io.StringIO()
        n.csvfile = n.writer = None
        n.count = n.visible = n.valid_images = n.nonincreasing_stamps = 0
        n.stamps = deque(maxlen=6000)
        n.errors = {}
        n.altitudes = {}
        n.last = {k: time.monotonic() for k in (
            'clock','camera','calibration','detector','valid_image','controller',
            'odom1','odom2','status1','status2')}
        n.status = {i: SimpleNamespace(arming_state=VehicleStatus.ARMING_STATE_ARMED,
            nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD) for i in (1,2)}

    def test_target_takeoff_altitude_includes_configured_offset(self):
        follower_altitude = takeoff_altitude(self.node.c, 1)
        target_altitude = takeoff_altitude(self.node.c, 2)
        self.assertEqual(follower_altitude, self.node.c['takeoff_altitude_m'])
        self.assertEqual(target_altitude-follower_altitude,
                         self.node.c['target_takeoff_altitude_offset_m'])

    def test_default_continues_without_landing(self):
        n = self.node
        self.assertEqual(n.c['duration_s'], 0)
        self.assertFalse(n.c['auto_land'])
        self.assertTrue(n.c['keep_open'])
        self.assertFalse(n.c['record'])
        n.advance()
        self.assertFalse(n.done or n.halted)
        n.command.assert_not_called()
        self.assertFalse((n.run/'result.json').exists())

    def test_intercept_session_waits_during_bounded_reacquisition(self):
        n = self.node
        n.metrics = {'enabled': False, 'reacquiring': True}
        n.advance()
        self.assertFalse(n.done or n.halted)
        n.command.assert_not_called()

    @patch('drone_follow.experiment.rclpy.ok', return_value=True)
    def test_finite_interval_stops_pursuit_but_keeps_session(self, _):
        n = self.node
        n.c['duration_s'] = 1.
        n.advance()
        self.assertTrue(n.halted)
        self.assertFalse(n.done)
        n.command.assert_not_called()
        self.assertFalse(n.enable.call_async.call_args.args[0].data)
        result = json.loads((n.run/'result.json').read_text())
        self.assertEqual(result['status'], 'passed')
        self.assertEqual(result['controller'], 'png_ibvs_velocity')
        self.assertIn('single PNG-IBVS', result['sensing'])

    @patch('drone_follow.experiment.rclpy.ok', return_value=True)
    def test_stale_camera_reports_failure_without_exit_or_land(self, _):
        n = self.node
        n.last['camera'] = 0.
        n.advance()
        self.assertTrue(n.halted)
        self.assertFalse(n.done)
        n.command.assert_not_called()
        result = json.loads((n.run/'result.json').read_text())
        self.assertEqual(result['status'], 'failed')
        self.assertIn('camera', result['reason'])
        # Later timer callbacks do not keep issuing commands or error events.
        events = n.events.getvalue()
        n.tick()
        self.assertEqual(n.events.getvalue(), events)

    def test_landing_requires_explicit_opt_in(self):
        n = self.node
        n.c.update(duration_s=1., auto_land=True)
        n.advance()
        self.assertEqual(n.stage, 'LANDING')
        self.assertEqual(n.command.call_count, 2)
        n.command.assert_any_call(1, 'commander', 'land')
        n.command.assert_any_call(2, 'commander', 'land')

    def test_minimal_events_skip_per_frame_records(self):
        n = self.node
        n.event('image', stamp_s=1.)
        n.event('control', stamp_s=1.)
        self.assertEqual(n.events.getvalue(), '')
        n.event('stage')
        self.assertEqual(json.loads(n.events.getvalue())['kind'], 'stage')

    def test_detailed_events_remain_opt_in(self):
        n = self.node
        n.record = True
        n.event('image', stamp_s=1.)
        self.assertEqual(json.loads(n.events.getvalue())['kind'], 'image')

    def test_indefinite_statistics_use_bounded_storage(self):
        n = self.node
        for i in range(6100):
            n.image_observation(SimpleNamespace(data=json.dumps(dict(
                stamp_s=i*.05, image_age_s=0.01, visible=True, reason='detected',
                error_u_px=3., error_v_px=4.))))
        self.assertEqual(len(n.stamps), 6000)
        self.assertEqual(n.count, 6100)
        self.assertEqual(n.errors['INTERCEPTING']['samples'], 6100)
        self.assertEqual(n.errors['INTERCEPTING']['squared_sum'], 6100*25.)
        self.assertEqual(n.events.getvalue(), '')
        self.assertFalse((n.run/'measurements.csv').exists())

    def test_recorded_row_contains_complete_velocity_command(self):
        n = self.node
        n.writer = Mock()
        n.csvfile = object()
        n.odom = {}
        n.metrics = dict(
            stamp_s=2.0,
            measurement_stamp_s=1.95,
            measurement_age_s=0.05,
            publishing_setpoint=True,
            velocity_ned_mps=[1.25, -0.5, 0.2],
            yaw_rate_rps=0.1,
            state='INTERCEPTING',
        )
        n.image_observation(SimpleNamespace(data=json.dumps(dict(
            stamp_s=2.0,
            image_age_s=0.01,
            visible=True,
            reason='detected',
            u_px=320.0,
            v_px=240.0,
            error_u_px=0.0,
            error_v_px=0.0,
            x_normalized=0.0,
            y_normalized=0.0,
        ))))
        row = n.writer.writerow.call_args.args[0]
        self.assertEqual(row['command_north_mps'], 1.25)
        self.assertEqual(row['command_east_mps'], -0.5)
        self.assertEqual(row['command_down_mps'], 0.2)

    @patch('drone_follow.experiment.rclpy.ok', return_value=False)
    def test_manual_stop_saves_interrupted_summary(self, _):
        n = self.node
        n.finish(False, 'Session stopped by user', status='interrupted')
        self.assertEqual(n.exit_code, 0)
        self.assertTrue(n.done)
        self.assertEqual(json.loads((n.run/'result.json').read_text())['status'], 'interrupted')
        n.command.assert_not_called()


if __name__ == '__main__':
    unittest.main()
