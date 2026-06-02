from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _vehicle_condition(vehicle_name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration('vehicle'), "' == '", vehicle_name, "'"]))


def _ranger_controller_actions():
    return [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['forward_position_controller'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['forward_velocity_controller'],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('four_ws_control'),
                    'launch',
                    'four_ws_control.launch.py',
                ])
            ])
        ),
    ]


def generate_scene_launch_description(world_name, start_x, start_y, start_z):
    gui = LaunchConfiguration('gui')
    vehicle = LaunchConfiguration('vehicle')
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
                'world_name': world_name,
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
                'start_x': start_x,
                'start_y': start_y,
                'start_z': start_z,
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
                'start_x': start_x,
                'start_y': start_y,
                'start_z': start_z,
                'robot_name': 'scout_v2',
            }.items(),
        ),

        TimerAction(
            period=8.0,
            actions=_ranger_controller_actions(),
            condition=is_ranger,
        ),

        Node(
            package='sensor_conversion',
            executable='slam_sim_output_node',
            name='slam_sim_output',
            output='log',
            remappings=[
                ('/odometry_init', '/laser_odom_init'),
                ('/registered_scan', '/registered_point_cloud'),
                ('/odometry', '/base_pose_ground_truth'),
                ('/point_cloud', '/velodyne_points'),
            ],
            parameters=[
                {'frame_id': 'map'},
                {'child_frame_id': 'sensor'},
                {'down_voxel_size': 0.1},
                {'use_sim_time': True},
            ],
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
            ],
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
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor_baselink',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '-0.3',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'sensor',
                '--child-frame-id', 'base_link',
            ],
        ),
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
        Node(
            package='rviz2',
            executable='rviz2',
            name='demo',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('simworld'),
                    'launch',
                    'demo.rviz',
                ]),
            ],
        ),
    ])


def generate_launch_description():
    return generate_scene_launch_description('scene_1', '-14', '-14', '0.3')
