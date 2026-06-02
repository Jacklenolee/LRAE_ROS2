import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from simulation_scene1 import generate_scene_launch_description


def generate_launch_description():
    return generate_scene_launch_description('scene_4', '0.0', '0.0', '0.5')
