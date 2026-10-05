"""Complete stack with opt-in recording and user-controlled session lifetime."""
import json
import os
from pathlib import Path
import shlex
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    run = Path(os.environ['BASELINE_RUN_DIR'])
    package = run/'source'
    config = yaml.safe_load((run/'config.yaml').read_text())
    c = config['baseline']
    px4 = Path(c['px4_dir'])
    recording = c.get('record', False)
    debug = os.environ['BASELINE_DEBUG'] == '1'
    # Minimal mode drops repetitive component stdout; errors remain on screen.
    # Detailed mode uses one shared log, avoiding three duplicate logs/component.
    output = ({'both': ['log', 'screen']} if debug else {'both': 'log'}) if recording else (
        'screen' if debug else {'stderr': 'screen'})
    ros_logging_args = [] if recording else ['--disable-external-lib-logs']
    env = dict(GZ_SIM_RESOURCE_PATH=str(package/'models')+':'+str(px4/'Tools/simulation/gz/models'),
               PX4_GZ_STANDALONE='1', PX4_SYS_AUTOSTART='4001', PX4_GZ_WORLD='follow_world',
               PX4_UXRCE_DDS_PORT=str(c['agent_port']))
    # Ignore any prior PX4_SIM_MODEL that would request a second vehicle spawn.
    env['PX4_SIM_MODEL'] = ''
    if not recording:
        # The installed PX4 rcS searches PATH for this documented startup hook
        # after airframe defaults and before rc.logging. Preserve the upstream
        # hook, then disable file logging only in these fresh SITL instances.
        hooks = run/'px4_hooks'
        hooks.mkdir()
        original = px4/'build/px4_sitl_default/etc/init.d-posix/px4-rc.params'
        (hooks/'px4-rc.params').write_text(
            '. '+shlex.quote(str(original))+'\nparam set SDLOG_MODE -1\n')
        env['PATH'] = str(hooks)+':'+os.environ['PATH']
    gz = ['gz', 'sim', '--force-version', '8', '-r']
    if c['headless']:
        gz += ['-s', '--headless-rendering']
    processes = [
        ExecuteProcess(name='gazebo', cmd=gz+[str(run/'world.sdf')],
                       additional_env=env, output=output),
        ExecuteProcess(name='dds_agent', cmd=['MicroXRCEAgent','udp4','-p',str(c['agent_port'])], output=output),
    ]
    for instance, name in [(1, 'follower'), (2, 'target')]:
        wd = run/('px4_'+name)
        wd.mkdir()
        processes.append(ExecuteProcess(name='px4_'+name,
            cmd=[str(px4/'build/px4_sitl_default/bin/px4'),
                 str(px4/'build/px4_sitl_default/etc'), '-i', str(instance), '-d', '-w', str(wd)],
            additional_env={**env, 'PX4_GZ_MODEL_NAME':name}, output=output))
    processes.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
        name='camera_bridge', parameters=[{'config_file':str(package/'config/bridge.yaml')}],
        arguments=['--ros-args', *ros_logging_args], output=output))
    # Run the saved Python source via the package module search path, making the
    # run independent of later edits. PX4 and ROS dependencies are versioned in manifest.
    source_env = {'PYTHONPATH':str(package)+':'+os.environ.get('PYTHONPATH','')}
    for name in ('detector', 'follower', 'leader'):
        paramfile = run/(name+'_params.yaml')
        paramfile.write_text(yaml.safe_dump({name:config[name]}))
        processes.append(ExecuteProcess(name=name,
            cmd=['/usr/bin/python3','-c', f'from drone_follow.{name} import main; main()',
                 '--ros-args', '--params-file', str(paramfile), *ros_logging_args],
            additional_env=source_env, output=output))
    processes.append(ExecuteProcess(name='experiment',
        cmd=['/usr/bin/python3', str(package/'drone_follow/experiment.py'),
             '--ros-args', *ros_logging_args], output='both' if recording else 'screen'))
    def exited(event, context):
        if context.is_shutdown:
            return []
        name = event.action.process_details['name']
        if c.get('keep_open', True):
            with (run/'component_exits.jsonl').open('a') as log:
                log.write(json.dumps(dict(component=name, returncode=event.returncode))+'\n')
            return [LogInfo(msg=f'{name} exited ({event.returncode}); simulation remains open. '
                                'Inspect the terminal; Ctrl+C stops the remaining owned processes.')]
        return [LogInfo(msg=f'{name} exited ({event.returncode}); stopping owned stack'),
                EmitEvent(event=Shutdown(reason=f'{name} exited ({event.returncode})'))]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=p, on_exit=exited)) for p in processes]
    if c.get('viewer', False):
        viewer = ExecuteProcess(name='camera_viewer',
            cmd=['/usr/bin/python3', str(package/'drone_follow/viewer.py'),
                 '--ros-args', *ros_logging_args], output=output)
        # Closing a read-only viewer must not interrupt a flight or its landing.
        def viewer_exited(event, context):
            if context.is_shutdown:
                return []
            return [LogInfo(msg=f'Camera viewer exited ({event.returncode}); simulation continues. '
                                'Use terminal Ctrl+C to stop the owned stack; inspect viewer log on error.')]
        handlers.append(RegisterEventHandler(
            OnProcessExit(target_action=viewer, on_exit=viewer_exited)))
        processes.append(viewer)
    return LaunchDescription(handlers+processes)
