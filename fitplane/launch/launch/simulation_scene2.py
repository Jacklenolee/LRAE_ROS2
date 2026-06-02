import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from simulation_scene1 import generate_scene_launch_description


def generate_launch_description():
    return generate_scene_launch_description('scene_2', '-27', '-27', '0.3')
