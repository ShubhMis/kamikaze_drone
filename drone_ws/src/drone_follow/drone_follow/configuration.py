"""Validation shared by the ROS node and the standalone launcher."""

import math


CONTROLLER_KEYS = {
    'px4_namespace',
    'max_measurement_age_s',
    'minimum_detections',
    'approach_speed',
    'max_yaw_rate',
    'png_gain_y',
    'png_gain_z',
    'fov_kp',
    'fov_kd',
    'fov_ka',
    'max_speed',
    'max_vertical_speed',
    'camera_mount_q',
}

OPTIONAL_ROS_KEYS = {'use_sim_time'}

OBSOLETE_CONTROLLER_KEYS = {
    'mode',
    'yaw_gain',
    'vertical_gain',
    'centre_error_threshold',
    'center_error_threshold',
    'centre_hold_time',
    'centre_hold_time_s',
    'center_hold_time',
    'recenter_error_threshold',
    'max_vertical_speed_intercept',
}


def _finite_number(parameters, name):
    value = parameters[name]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return value


def validate_controller_parameters(parameters):
    """Reject stale selector keys and invalid single-controller settings."""
    if not isinstance(parameters, dict):
        raise ValueError('follower.ros__parameters must be a mapping')

    obsolete = sorted(OBSOLETE_CONTROLLER_KEYS.intersection(parameters))
    if obsolete:
        raise ValueError('obsolete controller parameter(s): ' + ', '.join(obsolete))

    unknown = sorted(set(parameters) - CONTROLLER_KEYS - OPTIONAL_ROS_KEYS)
    if unknown:
        raise ValueError('unknown controller parameter(s): ' + ', '.join(unknown))

    missing = sorted(CONTROLLER_KEYS - set(parameters))
    if missing:
        raise ValueError('missing controller parameter(s): ' + ', '.join(missing))

    namespace = parameters['px4_namespace']
    if not isinstance(namespace, str) or not namespace.startswith('/') or namespace.endswith('/'):
        raise ValueError('px4_namespace must be an absolute namespace without a trailing slash')

    if 'use_sim_time' in parameters and not isinstance(parameters['use_sim_time'], bool):
        raise ValueError('use_sim_time must be a YAML boolean')

    age = _finite_number(parameters, 'max_measurement_age_s')
    if not 0 < age <= 10:
        raise ValueError('max_measurement_age_s must be >0 and <=10')

    detections = parameters['minimum_detections']
    if isinstance(detections, bool) or not isinstance(detections, int) or not 1 <= detections <= 100:
        raise ValueError('minimum_detections must be an integer from 1 to 100')

    positive = (
        'approach_speed', 'max_yaw_rate', 'png_gain_y', 'png_gain_z',
        'max_speed', 'max_vertical_speed',
    )
    values = {name: _finite_number(parameters, name) for name in positive}
    if any(value <= 0 for value in values.values()):
        raise ValueError(', '.join(positive) + ' must all be positive')
    if values['approach_speed'] > values['max_speed']:
        raise ValueError('approach_speed must not exceed max_speed')
    if values['max_vertical_speed'] > values['max_speed']:
        raise ValueError('max_vertical_speed must not exceed max_speed')
    if values['max_speed'] > 20 or values['max_yaw_rate'] > 10:
        raise ValueError('controller speed/rate limit is outside the supported validation range')
    if values['png_gain_y'] > 10 or values['png_gain_z'] > 10:
        raise ValueError('PNG gains must be <=10')

    for name in ('fov_kp', 'fov_kd', 'fov_ka'):
        value = _finite_number(parameters, name)
        if not 0 <= value <= 10:
            raise ValueError(f'{name} must be between 0 and 10')

    mount = parameters['camera_mount_q']
    if (not isinstance(mount, (list, tuple)) or len(mount) != 4
            or any(isinstance(value, bool) or not isinstance(value, (int, float))
                   or not math.isfinite(float(value)) for value in mount)):
        raise ValueError('camera_mount_q must contain four finite numbers')
    norm = math.sqrt(sum(float(value) ** 2 for value in mount))
    if norm < 0.5:
        raise ValueError('camera_mount_q has invalid norm')

