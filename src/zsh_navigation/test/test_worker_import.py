"""A spawned planner must import the installed-style package independently."""
from concurrent.futures import ProcessPoolExecutor
import multiprocessing

from zsh_navigation.race_course import Stage, line
from zsh_navigation.race_navigation import _plan_in_process
from zsh_navigation.race_tuning import tuning_for
from zsh_navigation.obstacle_cloud import ObstacleCloud


def test_spawned_planner_accepts_packaged_course_and_obstacle_snapshot():
    stage = Stage('OUTBOUND_A', line((1., 1.), (2., 1.)), 1, .3, True)
    with ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn')) as pool:
        result = pool.submit(_plan_in_process, stage, [], tuning_for('sprint'),
                             0, (1., 1., 0.), ObstacleCloud([]),
                             stage.points, 0., .035, .01).result(timeout=15)
    assert result['route']
    assert result['route'][0] == (1., 1.)
    assert result['route'][-1] == (2., 1.)
