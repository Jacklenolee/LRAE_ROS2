from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fitplane',
            executable='Traversibility_mapping',
            name='Traversibility_mapping',
            output='screen',
            on_exit=Shutdown(),
            parameters=[
                {'PointCloud_Map_topic': '/registered_point_cloud'},
                {'Grid_Map_topic': '/grid_map'},
            ],
        ),
    ])
