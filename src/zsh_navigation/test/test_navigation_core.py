"""Offline algorithm and input-contract regressions; no ROS or drive output."""
import math
from pathlib import Path

import numpy as np
import pytest

from zsh_navigation.race_course import START, Stage, build_course, audit_course, allowed_point, legal_pose, wrap
from zsh_navigation.race_navigation import RaceMission, clamp
from zsh_navigation.race_mpc import SamplingMPC, allowed_points, steering_transition
from zsh_navigation.hybrid_corridor import HybridCorridor
from zsh_navigation.obstacle_cloud import ObstacleCloud
from zsh_navigation.pose_source import field_body_pose, interpolate_pose
from zsh_navigation.sensor_pipeline import inputs_reason, project_scan, steering_to_twist, ObservationMemory

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('direction', ['CW', 'CCW'])
def test_full_course_geometry(direction):
    stages = build_course(ROOT, direction)
    audit = audit_course(stages)
    assert not audit['illegal_reference_poses']
    assert not audit['stage_gaps']
    assert 20 < audit['length_m'] < 23
    assert len(stages) == 9 and any(s.direction < 0 for s in stages)


def test_vectorized_boundary_matches_scalar_definition():
    x, y = np.meshgrid(np.linspace(-.01, 5.01, 71), np.linspace(-.01, 5.01, 71))
    expected = np.array([[allowed_point(a, b) for a, b in zip(xs, ys)] for xs, ys in zip(x, y)])
    assert np.array_equal(allowed_points(x, y), expected)


def test_field_alignment_removes_sensor_offset_in_rotated_body():
    actual = field_body_pose((1., 2., math.pi / 2), (3., 4., math.pi / 2), (.2, 0., 0.))
    assert actual[:2] == pytest.approx((1.2, 5.))
    assert abs(actual[2]) == pytest.approx(math.pi)


def test_pose_interpolation_crosses_pi_without_full_rotation_and_refuses_extrapolation():
    history = [(1., (0., 0., math.radians(179))), (1.1, (.1, 0., math.radians(-179)))]
    middle = interpolate_pose(history, 1.05)
    assert middle[:2] == pytest.approx((.05, 0.))
    assert abs(middle[2]) == pytest.approx(math.pi)
    assert interpolate_pose(history, 1.11) is None
    assert interpolate_pose(history, .99) is None
    assert interpolate_pose([(1., (0., 0., 0.)), (2., (1., 0., 0.))], 1.5) is None


@pytest.mark.parametrize('stamp_is_end,reversed_time', [(False, False), (True, False), (False, True), (True, True)])
def test_per_beam_projection_uses_correct_acquisition_time(stamp_is_end, reversed_time):
    history = [(1., (0., 0., 0.)), (1.1, (.1, 0., 0.))]
    result = project_scan([1., 1.], 0., .01, 1.1 if stamp_is_end else 1., .1,
                          history, laser_in_body=(0., 0., 0.),
                          stamp_is_end=stamp_is_end, reversed_time=reversed_time)
    assert result[0][0] == pytest.approx(1.1 if reversed_time else 1.)
    assert result[1][0] == pytest.approx(math.cos(.01) + (0 if reversed_time else .1))


def test_missing_pose_bracket_defers_scan_instead_of_using_newest_pose():
    assert project_scan([1.], 0., .01, 1.1, 0., [(1., (0., 0., 0.))]) is None


@pytest.mark.parametrize('v,delta', [(.2, .3), (-.2, .3), (.2, -.3), (-.2, -.3), (0., .3)])
def test_twist_conversion_preserves_reverse_sign(v, delta):
    speed, omega = steering_to_twist(v, delta)
    assert speed == v
    assert omega == pytest.approx(v * math.tan(delta) / .143)
    if v:
        assert math.atan(.143 * omega / v) == pytest.approx(delta)


@pytest.mark.parametrize('value', [math.nan, math.inf])
def test_nonfinite_commands_rejected(value):
    with pytest.raises(ValueError):
        steering_to_twist(value, 0.)


def test_sensor_freshness_uses_measurement_time():
    limits = {'pose': .3, 'motion': .3, 'scan': .4}
    assert inputs_reason(2., {'pose': None}, limits) == 'WAIT_POSE'
    assert inputs_reason(2., dict(pose=1.6, motion=2., scan=2.), limits) == 'POSE_TIMEOUT'
    assert inputs_reason(2., dict(pose=2., motion=2., scan=2.2), limits) == 'SCAN_FROM_FUTURE'
    assert inputs_reason(2., dict(pose=1.9, motion=1.9, scan=1.8), limits) == ''


def test_observation_expiry_and_overload_are_explicit():
    memory = ObservationMemory(lifetime=1., maximum=2)
    memory.update([(1., 1.), (2., 1.)], 1.)
    assert len(memory.current(1.5)) == 2
    with pytest.raises(ValueError, match='OVERLOAD'):
        memory.update([(3., 1.)], 1.5)
    assert len(memory.points) == 2
    assert memory.current(2.1) == []


def test_cone_fit_restores_base_and_uncertain_points_remain_obstacles():
    points = [(2. + .03 * math.cos(a), 1. + .03 * math.sin(a)) for a in np.linspace(0, 2*math.pi, 16, endpoint=False)]
    cloud = ObstacleCloud(points, base_half_width=.12)
    assert cloud.fitted == 1
    assert cloud.boxes[0] == pytest.approx([2., 1., .12, .12], abs=.001)
    uncertain = ObstacleCloud([(2., 1.)], inflation=.14)
    assert uncertain.fitted == 0
    assert uncertain.boxes[0] == pytest.approx([2., 1., .14, .14])


def test_square_base_collision_is_checked_not_just_its_center_point():
    mpc = SamplingMPC()
    # The base overlaps the front bumper even though its centre is outside it.
    boxes = np.array([[1.30, 1., .11, .11]])
    assert not mpc.clear(np.array([1.]), np.array([1.]), np.array([0.]), boxes, .14)[0]
    assert mpc.clear(np.array([.6]), np.array([1.]), np.array([0.]), boxes, .14)[0]


def test_second_order_transition_has_semigroup_property():
    def matrix(dt):
        return np.array(steering_transition(3.29, 16.42, dt)).reshape(2, 2)
    assert np.allclose(matrix(.03) @ matrix(.03), matrix(.06), atol=1e-12)


def test_analytic_connector_preserves_heading_and_rejects_obstruction():
    planner = HybridCorridor()
    path = planner.connector((1., 1., 0.), (2., 1.), 0., 1, np.empty((0, 2)), .14)
    assert path and path[-1] == pytest.approx((2., 1.))
    assert max(abs(p[1] - 1.) for p in path) < 1e-9
    assert planner.connector((1., 1., 0.), (2., 1.), 0., 1, np.array([[1.5, 1., .11, .11]]), .14) is None


def test_blocked_start_never_produces_forward_motion_or_false_completion():
    mission = RaceMission(build_course(ROOT))
    assert mission.command(START, 0., [(.78, .3)], 0.).speed == 0.
    mission.command(START, 0., [(.78, .3)], 13.)
    assert mission.failure == 'NO_SAFE_ACTION' and not mission.finished


@pytest.mark.parametrize('direction', ['CW', 'CCW'])
def test_complete_mission_in_ideal_kinematics_and_stop_before_gear_change(direction):
    mission = RaceMission(build_course(ROOT, direction))
    pose = list(START)
    v = steer = now = 0.
    previous_direction = 1
    reverse_seen = False
    while now < 150 and not mission.finished and not mission.failure:
        command = mission.command(pose, v, [], now)
        if mission.stage.direction != previous_direction:
            assert abs(v) < .025
            previous_direction = mission.stage.direction
        reverse_seen |= v < -.04
        for _ in range(5):
            v += clamp(command.speed - v, -.012, .012)
            steer += clamp(command.steering - steer, -.028, .028)
            pose[2] = wrap(pose[2] + v * math.tan(steer) / .143 * .01)
            pose[0] += v * math.cos(pose[2]) * .01
            pose[1] += v * math.sin(pose[2]) * .01
        assert legal_pose(pose, margin=0.)
        now += .05
    assert mission.finished and not mission.failure
    assert reverse_seen and len(mission.events) == 9
    assert math.dist(pose[:2], START[:2]) < .10 and abs(v) < .025
