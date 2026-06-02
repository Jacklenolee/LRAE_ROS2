import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_dir = os.path.join(get_package_share_directory('gazebo_ros'),'launch')
    world = os.path.join(get_package_share_directory('simworld'), 'worlds', 'map_scene_1.world')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_dir, '/gzserver.launch.py']),
            launch_arguments={
                'world': world
                }.items(),
        ),
        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_dir ,'/gzclient.launch.py'])),
    ])
