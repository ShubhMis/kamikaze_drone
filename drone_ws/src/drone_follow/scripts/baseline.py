"""Preflight, run snapshot, exclusive ownership and ROS launch lifecycle.

No control mathematics lives here. Only this run's process group is signalled.
ROS launch owns per-component startup/logging/failure propagation.
"""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
import math
import xml.etree.ElementTree as ET
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
SCENARIOS = ('observe', 'hover', 'intercept')


def preflight(config):
    c = config['baseline']
    px4 = Path(c['px4_dir']).expanduser().resolve()
    c['px4_dir'] = str(px4)
    required = [px4/'build/px4_sitl_default/bin/px4',
                px4/'Tools/simulation/gz/models/x500/model.sdf']
    for path in required:
        if not path.exists():
            raise RuntimeError(f'Missing {path}; build the documented PX4 v1.15 checkout first')
    for binary in ['gz', 'ros2', 'MicroXRCEAgent']:
        if not shutil.which(binary):
            raise RuntimeError(f'Missing executable: {binary}')
    # Fail early on the actual imports that failed during the initial setup.
    import rclpy, cv2, cv_bridge, numpy
    from px4_msgs.msg import VehicleOdometry
    from std_srvs.srv import SetBool
    from drone_follow import detector, follower
    from drone_follow.configuration import validate_controller_parameters
    from ament_index_python.packages import get_package_prefix
    get_package_prefix('ros_gz_bridge')
    validate_controller_parameters(config.get('follower', {}).get('ros__parameters'))
    if c['scenario'] not in SCENARIOS:
        raise RuntimeError('scenario must be observe, hover or intercept')
    for key in ('record', 'auto_land', 'keep_open'):
        if not isinstance(c[key], bool):
            raise RuntimeError(f'{key} must be a YAML boolean (true or false)')
    for key in ('ready_timeout_s', 'stale_timeout_s', 'flight_timeout_s', 'takeoff_altitude_m'):
        if not 0 < float(c[key]) < 600:
            raise RuntimeError(f'Invalid {key}')
    # Older saved configs predate the per-target offset and remain valid.
    target_offset = float(c.get('target_takeoff_altitude_offset_m', 0.0))
    if (not math.isfinite(target_offset) or abs(target_offset) > 20
            or float(c['takeoff_altitude_m']) + target_offset <= 0):
        raise RuntimeError('target_takeoff_altitude_offset_m must keep target altitude '
                           'positive and be within +/-20 m')
    if not 0 <= c['duration_s'] <= 600:
        raise RuntimeError('duration_s must be 0 (until Ctrl+C) or at most 600 seconds')
    if not math.isfinite(c['follower_yaw_deg']) or abs(c['follower_yaw_deg']) > 25:
        raise RuntimeError('follower_yaw_deg must be within +/-25 degrees for this visible-target baseline')
    if not 1 <= c['viewer_fps'] <= 60 or not 0 < c['viewer_stale_s'] <= 5:
        raise RuntimeError('viewer_fps must be 1..60 and viewer_stale_s must be >0..5')
    if not 0 <= c['viewer_pair_wait_s'] < c['viewer_stale_s']:
        raise RuntimeError('viewer_pair_wait_s must be nonnegative and less than viewer_stale_s')
    if c['viewer'] and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise RuntimeError('No desktop display available; use --headless or --no-viewer')
    # PX4 command-line client IPC uses instance numbers outside Gazebo partitions.
    # Refuse other instances rather than risk controlling or terminating them.
    for path in Path('/proc').glob('[0-9]*/comm'):
        try:
            if path.read_text().strip() == 'px4':
                raise RuntimeError(f'Existing PX4 process {path.parent.name}; stop that run yourself first')
        except (FileNotFoundError, PermissionError):
            pass
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind(('0.0.0.0', c['agent_port']))
        except OSError as exc:
            raise RuntimeError(f'DDS UDP port {c["agent_port"]} is busy: {exc}')
    return dict(python=sys.version, executable=sys.executable,
                numpy=numpy.__version__, opencv=cv2.__version__)


def snapshot(run, config, versions):
    (run/'config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    # Detailed experiments retain independent source. Interactive sessions use
    # a link so repeated launches do not duplicate the whole package.
    source = run/'source'
    if config['baseline']['record']:
        shutil.copytree(PACKAGE, source, ignore=shutil.ignore_patterns('__pycache__'))
    else:
        source.symlink_to(PACKAGE, target_is_directory=True)
    versions['source_mode'] = 'snapshot' if config['baseline']['record'] else 'live_workspace_link'
    hashes = {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in source.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    versions['source_sha256'] = hashes
    # Initial geometry is scenario generation, never a controller measurement.
    world = ET.parse(source/'worlds/follow_world.sdf')
    for include in world.findall('./world/include'):
        if include.findtext('name') == 'follower':
            pose = include.find('pose')
            values = pose.text.split()
            values[5] = str(math.radians(config['baseline']['follower_yaw_deg']))
            pose.text = ' '.join(values)
    world.write(run/'world.sdf', encoding='unicode', xml_declaration=True)
    versions['effective_world_sha256'] = hashlib.sha256((run/'world.sdf').read_bytes()).hexdigest()
    versions['gz_sim_version'] = subprocess.run(['gz','sim','--force-version','8','--version'],
        capture_output=True,text=True,timeout=10).stdout.strip()
    versions['ros_distro'] = os.environ.get('ROS_DISTRO')
    versions['dds_agent_version'] = subprocess.run(['MicroXRCEAgent','--version'],
        capture_output=True,text=True,timeout=10).stdout.strip()
    for key, path in [('px4_commit', config['baseline']['px4_dir']),
                      ('px4_msgs_commit', PACKAGE.parent/'px4_msgs')]:
        r = subprocess.run(['git', '-C', str(path), 'rev-parse', 'HEAD'],
                           capture_output=True, text=True)
        versions[key] = r.stdout.strip()
    (run/'manifest.json').write_text(json.dumps(versions, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=PACKAGE/'config/follow.yaml')
    parser.add_argument('--headless', action='store_true')
    view = parser.add_mutually_exclusive_group()
    view.add_argument('--viewer', dest='viewer', action='store_true', default=None,
                      help='show live camera even with --headless')
    view.add_argument('--no-viewer', dest='viewer', action='store_false',
                      help='disable live camera window')
    parser.add_argument('--debug', action='store_true')
    recording = parser.add_mutually_exclusive_group()
    recording.add_argument('--record', dest='record', action='store_true', default=None,
                           help='save detailed measurements, images, source and PX4 flight logs')
    recording.add_argument('--no-record', dest='record', action='store_false')
    parser.add_argument('--check', action='store_true', help='preflight only')
    parser.add_argument('--scenario', choices=SCENARIOS)
    parser.add_argument('--duration', type=float, help='wall seconds after ready; 0 until Ctrl+C')
    args = parser.parse_args()
    run = None
    with open(REPO/'.baseline.lock', 'w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('A baseline run already owns this workspace')
        config = yaml.safe_load(args.config.read_text())
        c = config['baseline']
        if args.headless:
            c['headless'] = True
        # Old saved configs can still be used after the viewer was added.
        c.setdefault('viewer_fps', 60.0)
        c.setdefault('viewer_stale_s', 0.5)
        c.setdefault('viewer_pair_wait_s', 0.08)
        c.setdefault('record', False)
        c.setdefault('auto_land', False)
        c.setdefault('keep_open', True)
        if args.record is not None:
            c['record'] = args.record
        c['viewer'] = (args.viewer if args.viewer is not None else
                       bool(c.get('viewer', True)) and not c['headless'])
        if args.scenario:
            c['scenario'] = args.scenario
        if args.duration is not None:
            c['duration_s'] = args.duration
        versions = preflight(config)
        print('CHECK OK: system Python, ROS, camera libraries, PX4 and DDS agent', flush=True)
        if args.check:
            return 0
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        run = REPO/'runs'/stamp
        run.mkdir(parents=True)
        snapshot(run, config, versions)
        env = os.environ.copy()
        env.update(BASELINE_RUN_DIR=str(run), ROS_LOG_DIR=str(run/'logs'),
                   ROS_DOMAIN_ID=str(c['ros_domain_id']),
                   GZ_PARTITION='drone_baseline_'+stamp,
                   BASELINE_DEBUG='1' if args.debug else '0')
        print(f'RUN {run}\nSCENARIO {c["scenario"]}; ROS domain {c["ros_domain_id"]}; monocular bearing, no range', flush=True)
        print(f'Recording: {"detailed" if c["record"] else "minimal"}; '
              f'scheduled landing: {c["auto_land"]}; keep simulation open: {c["keep_open"]}. '
              'Ctrl+C stops this stack.', flush=True)
        # Subprocess group ownership contains ros2 launch and all its descendants.
        child = subprocess.Popen(['ros2', 'launch', str(run/'source/launch/baseline.launch.py')],
                                 env=env, start_new_session=True)
        interrupted = False
        def signal_group(signum):
            try:
                os.killpg(child.pid, signum)
                return True
            except ProcessLookupError:
                return False
        def stop(signum, frame):
            nonlocal interrupted
            interrupted = True
            signal_group(signal.SIGINT)
        old_int = signal.signal(signal.SIGINT, stop)
        old_term = signal.signal(signal.SIGTERM, stop)
        try:
            while child.poll() is None and not interrupted:
                time.sleep(0.2)
        finally:
            # The launch parent can die before its children. PGID belongs to
            # this run even when the group leader has already been reaped.
            signal_group(signal.SIGINT)
            try:
                child.wait(timeout=12)
            except subprocess.TimeoutExpired:
                signal_group(signal.SIGKILL)
                child.wait()
            deadline = time.monotonic()+3
            while signal_group(0) and time.monotonic() < deadline:
                time.sleep(.1)
            if signal_group(0):
                signal_group(signal.SIGTERM)
                time.sleep(.2)
                signal_group(signal.SIGKILL)
            signal.signal(signal.SIGINT, old_int)
            signal.signal(signal.SIGTERM, old_term)
        # ROS launch may return zero after a component failure; use explicit result.
        result_path = run/'result.json'
        result = json.loads(result_path.read_text()) if result_path.exists() else {}
        if interrupted and result.get('status') != 'failed':
            result.update(status='interrupted', reason='Ctrl+C / termination requested')
        elif result.get('status') != 'passed':
            result.setdefault('status', 'failed')
            result.setdefault('reason', 'Component exited before experiment completion; inspect logs')
        result['launch_returncode'] = child.returncode
        result_path.write_text(json.dumps(result, indent=2))
        print(f'{result["status"].upper()}: {result.get("reason", "")}\nLogs/results: {run}', flush=True)
        return 0 if result['status'] in ('passed', 'interrupted') else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print(f'FAILED: {exc}', file=sys.stderr)
        sys.exit(1)
