"""Exercise ROS messages and adapter logic without constructing a ROS node."""
import math
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from std_srvs.srv import SetBool

import zsh_navigation.full_race_navigation as adapter
from zsh_navigation.race_course import START
from zsh_navigation.race_navigation import Control
from zsh_navigation.sensor_pipeline import ObservationMemory
from collections import deque


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Runtime:
    """Only application state; no DDS, timers, services or physical outputs."""
    stamp = staticmethod(adapter.FullRaceNavigation.stamp)
    accept_pose = adapter.FullRaceNavigation.accept_pose

    def __init__(self):
        self.p = dict(pose_frame='lio_hybird_odom', pose_child_frame='base_footprint',
                      motion_child_frame='base_footprint', field_offset=(0., 0., 0.),
                      body_to_pose_sensor=(0., 0., 0.), hardware_calibrated=False,
                      profile='sprint', maximum_duration=300., maximum_speed=.30,
                      maximum_control_seconds=10., fallback_inflation=.14,
                      cone_section_radius=.030, cone_base_half_width=.11)
        self.clock = 10.
        self.stamps = dict(pose=10., motion=10., scan=10.)
        self.max_ages = dict(pose=.3, motion=.3, scan=.4)
        self.pose = START
        self.history = deque()
        self.pending_scans = deque()
        self.memory = ObservationMemory()
        self.v = self.omega = 0.
        self.enabled = False
        self.failure = ''
        self.started = 10.
        self.last_clock = None
        self.last_mode = ''
        self.elapsed_control = 0.
        self.candidate_pub, self.drive_pub, self.status_pub = Publisher(), Publisher(), Publisher()
        self.mission = SimpleNamespace(stage=SimpleNamespace(name='OUTBOUND_A'), finished=False,
                                       failure='', command=lambda *args: Control(.1, .2, 'TRACKING'))
        self.stages = []
        self.tuning = None

    def now(self):
        return self.clock

    def consume_scans(self, now):
        pass

    def fail(self, reason):
        self.failure = self.failure or reason

    def get_logger(self):
        return SimpleNamespace(info=lambda _: None)


def odometry(x=.54, stamp=10.1):
    message = Odometry()
    message.header.frame_id = 'lio_hybird_odom'
    message.child_frame_id = 'base_footprint'
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    message.pose.pose.position.x = x
    message.pose.pose.position.y = .30
    message.pose.pose.orientation.w = 1.
    return message


def test_valid_pose_uses_message_frame_and_measurement_stamp():
    state = Runtime()
    adapter.FullRaceNavigation.on_pose(state, odometry(.55))
    assert state.pose == pytest.approx((.55, .3, 0.))
    assert state.history[-1][0] == pytest.approx(10.1)
    assert not state.failure


@pytest.mark.parametrize('fault', ['parent', 'child', 'quaternion', 'jump'])
def test_invalid_localization_is_latched_without_replacing_pose(fault):
    state, message = Runtime(), odometry()
    if fault == 'parent':
        message.header.frame_id = 'wrong'
    elif fault == 'child':
        message.child_frame_id = 'wrong'
    elif fault == 'quaternion':
        message.pose.pose.orientation.w = 0.
    else:
        message.pose.pose.position.x = 2.
    adapter.FullRaceNavigation.on_pose(state, message)
    assert state.failure
    assert state.pose == START


def test_disabled_adapter_does_not_contend_for_drive_control():
    state = Runtime()
    adapter.FullRaceNavigation.tick(state)
    assert not state.drive_pub.messages
    assert state.candidate_pub.messages[-1].linear.x == 0.


def test_active_adapter_converts_steering_to_twist_units():
    state = Runtime()
    state.enabled = True
    adapter.FullRaceNavigation.tick(state)
    command = state.drive_pub.messages[-1]
    assert command.linear.x == pytest.approx(.1)
    assert command.angular.z == pytest.approx(.1 * math.tan(.2) / .143)


def test_stale_localization_stops_and_does_not_silently_resume():
    state = Runtime()
    state.enabled = True
    state.stamps['pose'] = 9.
    adapter.FullRaceNavigation.tick(state)
    assert state.failure == 'POSE_TIMEOUT'
    assert state.drive_pub.messages[-1].linear.x == 0.
    state.stamps['pose'] = 10.
    adapter.FullRaceNavigation.tick(state)
    assert state.drive_pub.messages[-1].linear.x == 0.


def test_nonfinite_solver_output_never_reaches_drive():
    state = Runtime()
    state.enabled = True
    state.mission.command = lambda *args: Control(.1, math.nan, 'TRACKING')
    adapter.FullRaceNavigation.tick(state)
    assert state.failure.startswith('CONTROL_ERROR')
    assert state.drive_pub.messages[-1].linear.x == 0.


def test_drive_requires_calibration_and_vehicle_at_rest():
    state = Runtime()
    request, response = SetBool.Request(data=True), SetBool.Response()
    result = adapter.FullRaceNavigation.on_enable(state, request, response)
    assert not result.success and result.message == 'HARDWARE_CALIBRATION_REQUIRED'
    assert not state.enabled
    state.p['hardware_calibrated'] = True
    state.v = .10
    result = adapter.FullRaceNavigation.on_enable(state, request, SetBool.Response())
    assert not result.success and result.message == 'VEHICLE_NOT_STOPPED'


def test_preview_enable_is_explicit_and_repeated_enable_does_not_restart(monkeypatch):
    state = Runtime()
    state.drive_pub = None
    original = object()
    monkeypatch.setattr(adapter, 'RaceMission', lambda *args, **kwargs: original)
    result = adapter.FullRaceNavigation.on_enable(state, SetBool.Request(data=True), SetBool.Response())
    assert result.success and result.message == 'PREVIEW_ENABLED'
    assert state.enabled and state.mission is original
    result = adapter.FullRaceNavigation.on_enable(state, SetBool.Request(data=True), SetBool.Response())
    assert result.success and result.message == 'ALREADY_ENABLED'
    assert state.mission is original


def test_near_future_scan_waits_without_discarding_existing_obstacles():
    state = Runtime()
    state.p['scan_max_age'] = .4
    state.p['scan_stamp_is_end'] = True
    message = LaserScan()
    message.header.stamp.sec = 10
    message.header.stamp.nanosec = 20000000
    message.ranges = [1.]
    state.pending_scans.append(message)
    state.memory.update([(1., 1.)], 10.)
    adapter.FullRaceNavigation.consume_scans(state, 10.)
    assert len(state.pending_scans) == 1
    assert state.stamps['scan'] == 10.
    assert state.memory.current(10.) == [(1., 1.)]
