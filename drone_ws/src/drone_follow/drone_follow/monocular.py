"""Range-free image centring; no ROS, simulator state or paper-specific law.

Optical axes are right/down/forward. The vertical mapping assumes level hover
with a forward camera. It is not a global IBVS stability guarantee.
"""
import math


def centring_command(x, y, yaw_gain, vertical_gain, max_yaw_rate, max_vertical_speed):
    """Normalized pixel error -> NED down speed [m/s], yaw rate [rad/s].

    Target right -> positive NED yaw. Target below -> positive NED down.
    No horizontal translation or distance estimate is generated.
    """
    if not all(math.isfinite(v) for v in (x, y, yaw_gain, vertical_gain,
                                         max_yaw_rate, max_vertical_speed)):
        raise ValueError('Nonfinite input')
    if min(yaw_gain, vertical_gain) < 0 or min(max_yaw_rate, max_vertical_speed) <= 0:
        raise ValueError('Gains must be nonnegative and limits positive')
    down = max(-max_vertical_speed, min(max_vertical_speed, vertical_gain*y))
    yaw = max(-max_yaw_rate, min(max_yaw_rate, yaw_gain*math.atan(x)))
    return down, yaw
