import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from exploration_scene1 import generate_exploration_launch_description


def generate_launch_description():
    planner_params = {
        'angle_pen': 0.1,
        'update_cen_thre': 1,
        'unknown_num_thre': 300,
        'minrange': 30.0,
        'limit_max_square': False,
        'use_go_end_nearest': False,
    }
    map_params = {
        'map_w': 316,
        'map_h': 316,
        'mapinitox': -5.0,
        'mapinitoy': -5.0,
        'merge_size': 9.0,
        'safe_obs_dis': 1.0,
    }
    return generate_exploration_launch_description(planner_params, map_params)
