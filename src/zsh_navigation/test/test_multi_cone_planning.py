"""Multi-cone geometry and steering-continuity regressions, without ROS."""
import math
from pathlib import Path
import numpy as np
from zsh_navigation.race_course import build_course,path_yaw,Stage,line
from zsh_navigation.race_navigation import CourseFollower
from zsh_navigation.obstacle_cloud import ObstacleCloud
from zsh_navigation.race_mpc import SamplingMPC
ROOT=Path(__file__).resolve().parents[1]


def test_steering_commands_obey_rate_and_acceleration_bounds_before_collision_check():
    route=[(i*.025,1.) for i in range(120)]
    mpc=SamplingMPC();last=rate=0.
    for i in range(12):
        result=mpc.solve((.3,1.,.15),.4,0.,route,12,[],.14,.55,1,current_input=last)
        assert result is not None
        new=(result[1]-last)/.05
        assert abs(new)<=1.5+1e-8
        assert abs(new-rate)<=5*.05+1e-8
        rate=new;last=result[1]


def test_observed_square_base_and_ring_detour_do_not_enter_inner_green_area():
    stage=build_course(ROOT)[5]
    i=min(range(len(stage.points)),key=lambda i:math.dist(stage.points[i],(1.3,4.31)))
    pose=(*stage.points[i],path_yaw(stage.points,i))
    points=ObstacleCloud([(2.7+.027*math.cos(j*math.pi/12),4.45+.027*math.sin(j*math.pi/12)) for j in range(24)])
    assert points.fitted==1 and np.allclose(points.boxes[0,:2],(2.7,4.45))
    follower=CourseFollower(stage);follower.index=follower.nominal_index=i
    follower.try_replan(pose,points,0.)
    assert follower.replanned and follower.hybrid.reason=='FRENET_CORRIDOR_FOUND'
    assert all(follower.clear((*p,path_yaw(follower.points,k)),points) for k,p in enumerate(follower.points))


def test_centered_cone_in_half_meter_ring_is_reported_blocked():
    stage=build_course(ROOT)[5]
    i=min(range(len(stage.points)),key=lambda i:math.dist(stage.points[i],(1.3,4.31)))
    points=ObstacleCloud([(2.7+.027*math.cos(j*math.pi/12),4.31+.027*math.sin(j*math.pi/12)) for j in range(24)])
    follower=CourseFollower(stage);follower.index=follower.nominal_index=i
    follower.try_replan((*stage.points[i],path_yaw(stage.points,i)),points,0.)
    assert not follower.replanned and follower.blocked_reason=='NO_KINEMATIC_CORRIDOR'


def test_narrow_ring_corridor_tolerates_observed_center_bias_without_skipping_gap():
    # Actual one-sided laser fit: 15mm longitudinal / 5mm lateral centre bias.
    # Keep a 25mm field reserve and the full 20cm square cone base plus margin.
    stage=Stage('C_RING_SEGMENT',line((1.2,4.31),(3.7,4.31)),1,.65,True)
    points=ObstacleCloud([(2.68455+.027*math.cos(j*math.pi/12),
                           4.44515+.027*math.sin(j*math.pi/12)) for j in range(24)])
    follower=CourseFollower(stage)
    follower.try_replan((1.2,4.31,0.),points,0.)
    assert follower.replanned
    assert follower.clear_route(follower.points,points)
    assert follower.mpc.boundary_margin==.025


def test_two_cones_rejoin_before_ring_corner_and_keep_existing_side_offset():
    stage=build_course(ROOT)[5];pose=(1.32,4.22,.03)
    points=ObstacleCloud([(x+.030*math.cos(j*math.pi/12),
                           4.45+.030*math.sin(j*math.pi/12))
                          for x in (2.35,3.) for j in range(24)])
    follower=CourseFollower(stage)
    follower.nominal_index=min(range(len(stage.points)),key=lambda i:math.dist(pose[:2],stage.points[i]))
    follower.try_replan(pose,points,0.)
    assert follower.replanned
    assert follower.points[-1]==stage.points[-1]
    assert follower.clear_route(follower.points,points)
    # No initial swing back towards the two occupied cone bases.
    assert max(y for x,y in follower.points if 1.35<x<1.8)<4.25
