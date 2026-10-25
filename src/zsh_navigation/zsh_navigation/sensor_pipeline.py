"""ROS-free input timing, scan projection and command unit contracts."""
import math

from .pose_source import interpolate_pose


def inputs_reason(now, stamps, max_ages, future_tolerance=.05):
    if not math.isfinite(now):
        return 'INVALID_CLOCK'
    for name, limit in max_ages.items():
        stamp = stamps.get(name)
        if stamp is None:
            return 'WAIT_' + name.upper()
        if not math.isfinite(stamp):
            return 'INVALID_' + name.upper() + '_STAMP'
        age = now - stamp
        if age < -future_tolerance:
            return name.upper() + '_FROM_FUTURE'
        if age > limit:
            return name.upper() + '_TIMEOUT'
    return ''


def steering_to_twist(speed, steering, wheelbase=.143):
    if not all(math.isfinite(x) for x in (speed, steering, wheelbase)):
        raise ValueError('nonfinite command')
    if wheelbase <= 0 or abs(steering) >= math.pi / 2:
        raise ValueError('invalid vehicle geometry or steering')
    return speed, speed * math.tan(steering) / wheelbase


def project_scan(ranges, angle_min, angle_increment, stamp, time_increment,
                 history, laser_in_body=(-.03, 0., 0.), stamp_is_end=True,
                 reversed_time=True, range_min=.07, range_max=3., max_gap=.20):
    """Project every valid beam using a bracketed pose at acquisition time.

    A missing pose bracket returns None, so callers can defer the scan until
    localization catches up. No latest-pose substitution or extrapolation.
    """
    if not ranges:
        raise ValueError('empty laser scan')
    if (not all(math.isfinite(v) for v in
                (angle_min, angle_increment, stamp, time_increment, range_min,
                 range_max, max_gap, *laser_in_body)) or
            angle_increment == 0 or time_increment < 0 or
            not 0 <= range_min < range_max or max_gap <= 0):
        raise ValueError('invalid scan metadata')
    duration = (len(ranges) - 1) * time_increment
    start = stamp - duration if stamp_is_end else stamp
    sx, sy, sa = laser_in_body
    points = []
    for i, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance < range_max:
            continue
        beam = len(ranges) - 1 - i if reversed_time else i
        pose = interpolate_pose(history, start + beam * time_increment, max_gap)
        if pose is None:
            return None
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        angle = yaw + sa + angle_min + i * angle_increment
        points.append((x + c * sx - s * sy + distance * math.cos(angle),
                       y + s * sx + c * sy + distance * math.sin(angle)))
    return points


class ObservationMemory:
    """Bounded static-obstacle memory; overload is explicit, never silent."""

    def __init__(self, lifetime=2., voxel=.0125, maximum=1200):
        if (not math.isfinite(lifetime) or not math.isfinite(voxel) or
                lifetime <= 0 or voxel <= 0 or maximum < 1):
            raise ValueError('invalid observation memory settings')
        self.lifetime, self.voxel, self.maximum = lifetime, voxel, maximum
        self.points = {}

    def update(self, points, stamp):
        if not math.isfinite(stamp):
            raise ValueError('invalid observation time')
        updated = {k: p for k, p in self.points.items()
                   if 0 <= stamp - p[2] <= self.lifetime}
        for x, y in points:
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError('invalid observation point')
            # The known arena perimeter is a hard course constraint, rather
            # than a cone. Interior structures remain obstacle evidence.
            if .10 < x < 4.90 and .10 < y < 4.90:
                updated[(round(x / self.voxel), round(y / self.voxel))] = (x, y, stamp)
        if len(updated) > self.maximum:
            raise ValueError('OBSERVATION_OVERLOAD')
        self.points = updated

    def current(self, now):
        self.points = {k: p for k, p in self.points.items()
                       if 0 <= now - p[2] <= self.lifetime}
        return [(x, y) for x, y, _ in self.points.values()]
