"""Wall-clock readiness, interactive SITL sessions and optional measurements.

This node alone sees both vehicles for experiment sequencing. Neither target
telemetry nor any simulator pose is passed to the visual controller.
"""
import csv
from collections import deque
import json
import math
import os
from pathlib import Path
import subprocess
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as SENSOR
from sensor_msgs.msg import Image, CameraInfo
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import SetBool
from px4_msgs.msg import VehicleOdometry, VehicleStatus, OffboardControlMode


class Experiment(Node):
    def __init__(self):
        # Intentionally wall clock: must still fail if Gazebo /clock stops.
        super().__init__('baseline_experiment')
        self.run = Path(os.environ['BASELINE_RUN_DIR'])
        self.c = yaml.safe_load((self.run/'config.yaml').read_text())['baseline']
        self.record = self.c.get('record', False)
        self.started = self.stage_start = time.monotonic()
        self.stage = 'WAITING'
        self.done = False
        self.halted = False
        self.exit_code = 1
        self.last = {}
        self.odom = {}
        self.status = {}
        self.metrics = {}
        self.observation = {}
        self.sim_time = 0.0
        self.initial = {}
        self.hold_since = None
        self.future = None
        self.count = self.visible = 0
        self.valid_images = 0
        # Interactive sessions can be indefinite. Keep a bounded timing window
        # and aggregate errors/extrema instead of retaining every observation.
        self.stamps = deque(maxlen=6000)
        self.nonincreasing_stamps = 0
        self.errors = {}
        self.saved_phases = set()
        self.altitudes = {}
        self.events = open(self.run/'events.jsonl', 'w', buffering=1)
        self.csvfile = (open(self.run/'measurements.csv', 'w', newline='', buffering=1)
                        if self.record else None)
        fields = ['wall_s','sim_s','stage','image_stamp_s','image_age_s','visible','reason',
                  'u_px','v_px','error_u_px','error_v_px','x_normalized','y_normalized',
                  'control_stamp_s','control_measurement_stamp_s','control_measurement_age_s',
                  'publishing_setpoint','command_north_mps','command_east_mps',
                  'command_down_mps','command_yaw_rps',
                  'follower_north_m','follower_east_m','follower_down_m',
                  'target_north_m','target_east_m','target_down_m','state']
        self.writer = csv.DictWriter(self.csvfile, fieldnames=fields) if self.csvfile else None
        if self.writer:
            self.writer.writeheader()
        self.create_subscription(Clock, '/clock', self.clock, SENSOR)
        self.create_subscription(String, '/target/observation', self.image_observation, 10)
        self.create_subscription(String, '/follow/metrics', self.control_metrics, 10)
        self.create_subscription(Image, '/follow_camera/image', self.image, SENSOR)
        self.create_subscription(CameraInfo, '/follow_camera/camera_info', self.camera_info, SENSOR)
        for i in (1,2):
            self.create_subscription(OffboardControlMode, f'/px4_{i}/fmu/in/offboard_control_mode',
                                     lambda m, i=i: self.touch(f'offboard_sp{i}'), 10)
            self.create_subscription(VehicleOdometry, f'/px4_{i}/fmu/out/vehicle_odometry',
                                     lambda m, i=i: self.vehicle_odom(i,m), SENSOR)
            self.create_subscription(VehicleStatus, f'/px4_{i}/fmu/out/vehicle_status',
                                     lambda m, i=i: self.vehicle_status(i,m), SENSOR)
        self.enable = self.create_client(SetBool, '/follow/enable')
        self.create_timer(0.2, self.tick)
        self.event('start', config=self.c)
        print('WAITING: advancing clock, camera/calibration, detector, controller, two PX4 estimates', flush=True)

    def event(self, kind, **data):
        if not self.record and kind in ('control', 'image'):
            return
        self.events.write(json.dumps(dict(kind=kind, wall_s=time.monotonic()-self.started,
                                         sim_s=self.sim_time, stage=self.stage, **data))+'\n')

    def touch(self, key):
        self.last[key] = time.monotonic()

    def clock(self, msg):
        t = msg.clock.sec+msg.clock.nanosec*1e-9
        if t > self.sim_time:
            self.touch('clock')
        self.sim_time = t

    def vehicle_odom(self, i, msg):
        self.odom[i] = msg
        if (msg.pose_frame == VehicleOdometry.POSE_FRAME_NED
                and np.all(np.isfinite(list(msg.position)+list(msg.velocity)))):
            self.touch(f'odom{i}')
            altitude = float(-msg.position[2])
            low, high = self.altitudes.get(i, (altitude, altitude))
            self.altitudes[i] = [min(low, altitude), max(high, altitude)]

    def vehicle_status(self, i, msg):
        self.status[i] = msg
        self.touch(f'status{i}')

    def camera_info(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.touch('calibration')
            if not (self.run/'camera.json').exists():
                (self.run/'camera.json').write_text(json.dumps(dict(width=msg.width,height=msg.height,
                    K=list(msg.k),frame=msg.header.frame_id, range_source='none'), indent=2))

    def image(self, msg):
        self.touch('camera')
        if self.record and self.stage not in self.saved_phases:
            import cv2
            from cv_bridge import CvBridge
            cv2.imwrite(str(self.run/f'camera_{self.stage.lower()}.png'),
                        CvBridge().imgmsg_to_cv2(msg, 'bgr8'))
            self.saved_phases.add(self.stage)

    def control_metrics(self, msg):
        self.metrics = json.loads(msg.data)
        self.touch('controller')
        self.event('control', **self.metrics)

    def image_observation(self, msg):
        data = json.loads(msg.data)
        self.observation = data
        self.touch('detector')
        self.count += 1
        self.visible += int(data['visible'])
        if (data['reason'] in ('detected','no_marker','ambiguous_markers')
                and 0 <= data['image_age_s'] <= .2
                and (not self.stamps or data['stamp_s'] > self.stamps[-1])):
            self.valid_images += 1
            self.touch('valid_image')
        if self.stamps and data['stamp_s'] <= self.stamps[-1]:
            self.nonincreasing_stamps += 1
        self.stamps.append(data['stamp_s'])
        if data['visible'] and not self.halted:
            point = [data['error_u_px'], data['error_v_px']]
            stats = self.errors.setdefault(self.stage, dict(samples=0, first=point,
                                                            last=point, squared_sum=0.))
            stats['samples'] += 1
            stats['last'] = point
            stats['squared_sum'] += point[0]**2+point[1]**2
        self.event('image', **data)
        if self.writer is None:
            return
        pos = self.odom[1].position if 1 in self.odom else [None]*3
        target_pos = self.odom[2].position if 2 in self.odom else [None]*3
        velocity_command = self.metrics.get('velocity_ned_mps', [0, 0, 0])
        out = dict(wall_s=time.monotonic()-self.started,sim_s=self.sim_time,stage=self.stage,
                   image_stamp_s=data['stamp_s'],image_age_s=data['image_age_s'],
                   visible=data['visible'],reason=data['reason'],
                   control_stamp_s=self.metrics.get('stamp_s'),
                   control_measurement_stamp_s=self.metrics.get('measurement_stamp_s'),
                   control_measurement_age_s=self.metrics.get('measurement_age_s'),
                   publishing_setpoint=self.metrics.get('publishing_setpoint'),
                   command_north_mps=velocity_command[0],
                   command_east_mps=velocity_command[1],
                   command_down_mps=velocity_command[2],
                   command_yaw_rps=self.metrics.get('yaw_rate_rps'),
                   follower_north_m=pos[0],follower_east_m=pos[1],follower_down_m=pos[2],
                   target_north_m=target_pos[0],target_east_m=target_pos[1],target_down_m=target_pos[2],
                   state=self.metrics.get('state'))
        for key in ('u_px','v_px','error_u_px','error_v_px','x_normalized','y_normalized'):
            out[key] = data.get(key)
        self.writer.writerow(out)

    def command(self, i, module, *args):
        binary = Path(self.c['px4_dir'])/'build/px4_sitl_default/bin'/('px4-'+module)
        result = subprocess.run([str(binary),'--instance',str(i),*map(str,args)],
                                capture_output=True,text=True,timeout=3)
        self.event('px4_command', instance=i,module=module,args=args,
                   returncode=result.returncode,output=result.stdout+result.stderr)
        if result.returncode:
            raise RuntimeError(f'PX4 {i} {module} {args}: {result.stdout}{result.stderr}')

    def transition(self, stage):
        self.stage, self.stage_start = stage, time.monotonic()
        self.hold_since = None
        self.event('stage')
        print(f'{stage}: {self.c["scenario"]}',flush=True)

    def finish(self, passed, reason, status=None):
        dt = np.diff(self.stamps)
        valid = dt[dt > 0]
        result = dict(status=status or ('passed' if passed else 'failed'),reason=reason,scenario=self.c['scenario'],
                      controller=self.metrics.get('controller', 'png_ibvs_velocity'),
                      stage=self.stage,wall_seconds=time.monotonic()-self.started,
                      images=self.count,visible_images=self.visible,
                      valid_images=self.valid_images,
                      visible_fraction=self.visible/max(self.count,1),
                      camera_hz_sim=float(len(valid)/sum(valid)) if len(valid) else None,
                      median_frame_interval_s=float(np.median(valid)) if len(valid) else None,
                      timing_window_images=len(self.stamps),
                      nonincreasing_image_stamps=self.nonincreasing_stamps,
                      altitude_extrema_ned_m={str(i):v for i,v in self.altitudes.items()},
                      sensing='monocular bearing; no metric range; single PNG-IBVS velocity-interface reconstruction')
        for stage in ('RECORDING','INTERCEPTING'):
            stats = self.errors.get(stage)
            if stats:
                result[stage.lower()+'_pixel_error'] = dict(samples=stats['samples'],
                    first=stats['first'], last=stats['last'],
                    rms=math.sqrt(stats['squared_sum']/stats['samples']))
        (self.run/'result.json').write_text(json.dumps(result,indent=2))
        self.event('finish',status=result['status'],reason=reason)
        print(f'{result["status"].upper()}: {reason}',flush=True)
        self.exit_code = 0 if passed or status == 'interrupted' else 1
        self.halted = True
        self.done = status == 'interrupted' or not self.c.get('keep_open', True)
        if not self.done:
            # Stop the experimental correction if its sequence ends or fails;
            # keep Gazebo/ROS/PX4 alive for inspection. Do not send LAND here.
            if rclpy.ok() and self.enable.service_is_ready():
                self.enable.call_async(SetBool.Request(data=False))
            print('SESSION OPEN: sequencing stopped; no landing requested. '
                  'PX4 failsafes remain active. Ctrl+C stops the simulation.', flush=True)

    def tick(self):
        if self.done or self.halted:
            return
        try:
            self.advance()
        except Exception as exc:
            self.finish(False,str(exc))

    def advance(self):
        now = time.monotonic()
        required = ['clock','camera','calibration','detector','valid_image','controller','odom1','odom2','status1','status2']
        missing = [k for k in required if now-self.last.get(k,0) > self.c['stale_timeout_s']]
        if self.stage == 'WAITING':
            if missing or self.valid_images < 5 or self.metrics.get('state','').startswith('STATE INVALID'):
                if now-self.started > self.c['ready_timeout_s']:
                    self.finish(False,'Readiness timeout: '+', '.join(missing or ['healthy controller']))
                return
            self.initial = {i:np.array(self.odom[i].position) for i in (1,2)}
            print('READY: both PX4 estimates + monocular frames + detector + controller',flush=True)
            if self.c['scenario'] == 'observe':
                self.transition('RECORDING')
            else:
                for i in (1,2):
                    for key,value in [('COM_RC_IN_MODE',4),('NAV_DLL_ACT',0),
                                      ('MIS_TAKEOFF_ALT',self.c['takeoff_altitude_m']),
                                      ('COM_OF_LOSS_T',0.5),('COM_OBL_RC_ACT',4)]:
                        self.command(i,'param','set',key,value)
                self.transition('PREFLIGHT')
            return
        if missing:
            self.finish(False,'Stale required stream(s): '+', '.join(missing))
            return
        if self.stage not in ('RECORDING','INTERCEPTING') and now-self.stage_start > self.c['flight_timeout_s']:
            self.finish(False,'Flight-stage timeout: '+self.stage)
            return
        if self.stage == 'PREFLIGHT':
            if all(self.status[i].pre_flight_checks_pass for i in (1,2)):
                for i in (1,2):
                    self.command(i,'commander','arm')
                self.transition('ARMING')
        elif self.stage == 'ARMING':
            if all(self.status[i].arming_state == VehicleStatus.ARMING_STATE_ARMED for i in (1,2)):
                for i in (1,2):
                    self.command(i,'commander','takeoff')
                self.transition('TAKEOFF')
        elif self.stage == 'TAKEOFF':
            stable = all(abs(float(self.odom[i].position[2]-self.initial[i][2])+self.c['takeoff_altitude_m']) < .5
                         and np.linalg.norm(self.odom[i].velocity) < .35
                         and self.status[i].nav_state in (VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
                                                         VehicleStatus.NAVIGATION_STATE_POSCTL)
                         and now-self.last.get(f'offboard_sp{i}',0) < .25 for i in (1,2))
            self.hold_since = (self.hold_since or now) if stable else None
            if self.hold_since and now-self.hold_since > 2:
                for i in (1,2):
                    self.command(i,'commander','mode','offboard')
                self.transition('OFFBOARD')
        elif self.stage == 'OFFBOARD':
            if all(self.status[i].nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD for i in (1,2)):
                if self.c['scenario'] == 'hover':
                    self.transition('RECORDING')
                elif self.enable.service_is_ready() and self.metrics.get('fresh'):
                    self.future = self.enable.call_async(SetBool.Request(data=True))
                    self.transition('ENABLING')
        elif self.stage == 'ENABLING':
            if self.future.done():
                response=self.future.result()
                if not response.success:
                    raise RuntimeError(response.message)
                self.transition('INTERCEPTING')
        elif self.stage in ('RECORDING','INTERCEPTING'):
            if self.c['scenario'] != 'observe':
                if not all(self.status[i].arming_state == VehicleStatus.ARMING_STATE_ARMED and
                           self.status[i].nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD for i in (1,2)):
                    raise RuntimeError('Vehicle left armed Offboard during recording')
                if self.stage == 'INTERCEPTING' and not self.metrics.get('enabled'):
                    raise RuntimeError('Pursuit disabled (e.g. detection loss); no automatic reacquisition')
            if self.c['duration_s'] and now-self.stage_start >= self.c['duration_s']:
                if self.c['scenario'] == 'observe':
                    self.finish(True,'Sensor/communication baseline completed; no flight commanded')
                elif self.c.get('auto_land', False):
                    self.enable.call_async(SetBool.Request(data=False))
                    for i in (1,2):
                        self.command(i,'commander','land')
                    self.transition('LANDING')
                else:
                    self.finish(True,'Requested interval completed; pursuit stopped, no landing requested')
        elif self.stage == 'LANDING':
            if all(self.status[i].arming_state != VehicleStatus.ARMING_STATE_ARMED for i in (1,2)):
                self.finish(True,'Bounded '+self.c['scenario']+' experiment completed and both vehicles disarmed')


def main():
    rclpy.init()
    node = Experiment()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node,timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.halted:
            node.finish(False, 'Session stopped by user', status='interrupted')
        if node.csvfile:
            node.csvfile.close()
        node.events.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == '__main__':
    raise SystemExit(main())
