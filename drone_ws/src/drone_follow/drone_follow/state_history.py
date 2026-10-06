"""Timestamped own-vehicle state interpolation for visual measurements.

Camera messages are stamped in the ROS clock used by the simulation. PX4 DDS
timestamps may use a different absolute epoch, but the difference between a
message's publication timestamp and ``timestamp_sample`` remains meaningful.
We therefore place odometry in ROS time by subtracting that sample age from
the ROS receipt time, then interpolate it at the image exposure timestamp.
"""

from bisect import bisect_left
from dataclasses import dataclass

import numpy as np


_STAMP_EPSILON_S = 1e-9


def odometry_sample_time(receive_time_s, timestamp_us, timestamp_sample_us,
                         max_sample_age_s=0.1):
    """Return an odometry sample timestamp in the ROS clock domain.

    ``timestamp_us`` and ``timestamp_sample_us`` share PX4's transmitted clock
    domain, so their difference is independent of the absolute DDS time
    offset. Transport delay is not observable from this message and remains a
    bounded residual that is recorded separately by the caller.
    """
    values = (receive_time_s, timestamp_us, timestamp_sample_us, max_sample_age_s)
    if not all(np.isfinite(value) for value in values):
        raise ValueError('odometry timestamps must be finite')
    if receive_time_s < 0 or timestamp_us <= 0 or timestamp_sample_us <= 0:
        raise ValueError('odometry timestamps must be positive')
    if max_sample_age_s <= 0:
        raise ValueError('max_sample_age_s must be positive')

    sample_age_s = (float(timestamp_us) - float(timestamp_sample_us)) * 1e-6
    if sample_age_s < -_STAMP_EPSILON_S or sample_age_s > max_sample_age_s:
        raise ValueError(f'invalid odometry sample age: {sample_age_s:.6f}s')
    return float(receive_time_s - max(sample_age_s, 0.0))


def quaternion_slerp(q0, q1, fraction):
    """Interpolate Hamilton quaternions while preserving the shortest arc."""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    if q0.shape != (4,) or q1.shape != (4,) or not 0 <= fraction <= 1:
        raise ValueError('SLERP requires two quaternions and a fraction in [0, 1]')
    if not np.all(np.isfinite(q0)) or not np.all(np.isfinite(q1)):
        raise ValueError('quaternions must be finite')
    n0, n1 = np.linalg.norm(q0), np.linalg.norm(q1)
    if n0 < 0.5 or n1 < 0.5:
        raise ValueError('quaternion norm is invalid')
    q0, q1 = q0 / n0, q1 / n1

    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        result = q0 + fraction * (q1 - q0)
        return result / np.linalg.norm(result)

    angle = np.arccos(dot)
    scale = np.sin(angle)
    result = (np.sin((1.0 - fraction) * angle) / scale * q0
              + np.sin(fraction * angle) / scale * q1)
    return result / np.linalg.norm(result)


@dataclass(frozen=True)
class VehicleState:
    """Own attitude and velocity at a ROS-clock timestamp."""

    stamp_s: float
    q: np.ndarray
    velocity: np.ndarray


class VehicleStateHistory:
    """Small ordered buffer with quaternion/velocity interpolation."""

    def __init__(self, duration_s=2.0, max_extrapolation_s=0.03):
        if duration_s <= 0 or not 0 <= max_extrapolation_s <= duration_s:
            raise ValueError('invalid vehicle-state history limits')
        self.duration_s = float(duration_s)
        self.max_extrapolation_s = float(max_extrapolation_s)
        self._states = []

    def clear(self):
        self._states.clear()

    def __len__(self):
        return len(self._states)

    @property
    def oldest_stamp_s(self):
        return None if not self._states else self._states[0].stamp_s

    @property
    def newest_stamp_s(self):
        return None if not self._states else self._states[-1].stamp_s

    def add(self, stamp_s, q, velocity):
        q = np.asarray(q, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        if (not np.isfinite(stamp_s) or q.shape != (4,) or velocity.shape != (3,)
                or not np.all(np.isfinite(q)) or not np.all(np.isfinite(velocity))):
            raise ValueError('vehicle state must be finite and correctly shaped')
        if np.linalg.norm(q) < 0.5:
            raise ValueError('vehicle-state quaternion norm is invalid')

        state = VehicleState(float(stamp_s), q.copy(), velocity.copy())
        if self._states and stamp_s < self._states[-1].stamp_s - _STAMP_EPSILON_S:
            raise ValueError('vehicle-state timestamps must be monotonic')
        if self._states and abs(stamp_s - self._states[-1].stamp_s) <= _STAMP_EPSILON_S:
            self._states[-1] = state
        else:
            self._states.append(state)

        cutoff = stamp_s - self.duration_s
        first_kept = bisect_left([item.stamp_s for item in self._states], cutoff)
        if first_kept:
            del self._states[:first_kept]

    def at(self, stamp_s):
        """Return an interpolated state, or ``None`` outside available history."""
        if not self._states or not np.isfinite(stamp_s):
            return None
        stamps = [item.stamp_s for item in self._states]
        index = bisect_left(stamps, stamp_s)

        if index < len(stamps) and abs(stamps[index] - stamp_s) <= _STAMP_EPSILON_S:
            state = self._states[index]
            return VehicleState(state.stamp_s, state.q.copy(), state.velocity.copy())
        if index == 0:
            return None
        if index == len(stamps):
            state = self._states[-1]
            # Treat the configured boundary as inclusive. Decimal timestamps
            # such as 2.02 cannot always be represented exactly in binary.
            if stamp_s - state.stamp_s > self.max_extrapolation_s + _STAMP_EPSILON_S:
                return None
            return VehicleState(state.stamp_s, state.q.copy(), state.velocity.copy())

        before, after = self._states[index - 1], self._states[index]
        fraction = (stamp_s - before.stamp_s) / (after.stamp_s - before.stamp_s)
        velocity = before.velocity + fraction * (after.velocity - before.velocity)
        q = quaternion_slerp(before.q, after.q, fraction)
        return VehicleState(float(stamp_s), q, velocity)
