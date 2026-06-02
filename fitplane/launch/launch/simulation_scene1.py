from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        PathJoinSubstitution([
        FindPackageShare('fitplane'),
        'launch/world.py'
        ])
        ]),
        launch_arguments={
        'world_name': 'map_scene_1'
        }.items()
        ),

        # 包含 scout 机器人 spawn launch 文件。这里不要使用
        # scout_v2_empty_world.launch.py，否则会再次启动一个 Gazebo world。
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        PathJoinSubstitution([
        FindPackageShare('scout_gazebo_sim'),
        'launch/scout_v2_spawn.launch.py'
        ])
        ]),
        launch_arguments={
        'start_x': '-14',
        'start_y': '-14',
        'start_z': '0.3',
        'robot_name': 'scout_v2'
        }.items()
        ),

        # 启动slam_sim_output节点
        Node(
            package='sensor_conversion',
            executable='slam_sim_output_node',
            name='slam_sim_output',
            output='screen',
            remappings=[
                ('/odometry_init', '/laser_odom_init'),
                ('/registered_scan', '/registered_point_cloud'),
                ('/odometry', '/base_pose_ground_truth'),              #/lio_sam/mapping/odometry
                ('/point_cloud', '/velodyne_points')   #/lio_sam/mapping/cloud_registered
            ],
            parameters=[
                {'frame_id': 'map'},
                {'child_frame_id': 'sensor'},
                {'down_voxel_size': 0.1},
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='sensor_conversion',
            executable='map_generator_node',
            name='map_generator_node',
            output='screen',
            remappings=[
                ('/registered_scan', '/registered_point_cloud'),
            ],
            parameters=[
                {'frame_id': 'map'},
                {'child_frame_id': 'base_link'},
                {'down_voxel_size': 0.1},
                {'explored_area_voxel_size': 0.1},
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'world',
                '--child-frame-id', 'map',
            ]
        ),
        # 启动 sensor 到 base_link 的静态 tf 变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor_baselink',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '-0.5',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'sensor',
                '--child-frame-id', 'base_link',
            ]
        ),
        # 启动Traversibility_mapping节点
        Node(
            package='fitplane',
            executable='Traversibility_mapping',
            name='Traversibility_mapping',
            output='screen',
            parameters=[
                {'PointCloud_Map_topic': '/registered_point_cloud'},
                {'Grid_Map_topic': '/grid_map'}
            ]
        ),
        Node(
        package='rviz2',
        executable='rviz2',
        name='demo',
        arguments=['-d', os.path.join(FindPackageShare('simworld').find('simworld'), 'launch', 'demo.rviz')]
       )
    ])
