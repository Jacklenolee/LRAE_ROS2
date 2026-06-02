import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(__file__))

from simulation_scene1 import _ranger_controller_actions


def _vehicle_condition(vehicle_name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration('vehicle'), "' == '", vehicle_name, "'"]))


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    is_ranger = _vehicle_condition('ranger')
    is_scout = _vehicle_condition('scout')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo client GUI if true',
        ),
        DeclareLaunchArgument(
            'vehicle',
            default_value='ranger',
            choices=['ranger', 'scout'],
            description='Vehicle model to spawn: ranger or scout',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fitplane'),
                    'launch',
                    'world.py',
                ])
            ]),
            launch_arguments={
                'world_name': 'scene_2',
                'gui': gui,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ranger_mini'),
                    'launch',
                    'ranger_mini_spawn.launch.py',
                ])
            ]),
            condition=is_ranger,
            launch_arguments={
                'start_x': '-27',
                'start_y': '-27',
                'start_z': '0.3',
                'robot_name': 'ranger_mini_v2',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('scout_gazebo_sim'),
                    'launch',
                    'scout_v2_spawn.launch.py',
                ])
            ]),
            condition=is_scout,
            launch_arguments={
                'start_x': '-27',
                'start_y': '-27',
                'start_z': '0.3',
                'robot_name': 'scout_v2',
            }.items(),
        ),
        TimerAction(
            period=8.0,
            actions=_ranger_controller_actions(),
            condition=is_ranger,
        ),
        Node(
            package='fitplane',
            executable='Traversibility_mapping',
            name='Traversibility_mapping',
            output='screen',
            on_exit=Shutdown(),
            parameters=[
                {'PointCloud_Map_topic': '/explored_cloud'},
                {'Grid_Map_topic': '/grid_map'},
            ],
        ),
    ])
