import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float_arg(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _bool_arg(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _declared_args():
    return [
        DeclareLaunchArgument('sensorOffsetX', default_value='0'),
        DeclareLaunchArgument('sensorOffsetY', default_value='0'),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0'),
        DeclareLaunchArgument('twoWayDrive', default_value='true'),
        DeclareLaunchArgument('maxSpeed', default_value='1.0'),
        DeclareLaunchArgument('autonomyMode', default_value='true'),
        DeclareLaunchArgument('autonomySpeed', default_value='1.0'),
        DeclareLaunchArgument('joyToSpeedDelay', default_value='2.0'),
        DeclareLaunchArgument('goalX', default_value='0'),
        DeclareLaunchArgument('goalY', default_value='0'),
    ]


def _local_planner_params(path_folder):
    return [
        {'pathFolder': path_folder},
        {'vehicleLength': 0.6},
        {'vehicleWidth': 0.6},
        {'sensorOffsetX': _float_arg('sensorOffsetX')},
        {'sensorOffsetY': _float_arg('sensorOffsetY')},
        {'twoWayDrive': _bool_arg('twoWayDrive')},
        {'laserVoxelSize': 0.05},
        {'terrainVoxelSize': 0.1},
        {'useTerrainAnalysis': True},
        {'checkObstacle': True},
        {'checkRotObstacle': False},
        {'adjacentRange': 4.25},
        {'obstacleHeightThre': 0.3},
        {'groundHeightThre': 0.1},
        {'costHeightThre': 0.5},
        {'costScore': 0.02},
        {'useCost': False},
        {'pointPerPathThre': 2},
        {'minRelZ': -0.5},
        {'maxRelZ': 0.25},
        {'maxSpeed': _float_arg('maxSpeed')},
        {'dirWeight': 0.02},
        {'dirThre': 90.0},
        {'dirToVehicle': False},
        {'pathScale': 1.25},
        {'minPathScale': 0.75},
        {'pathScaleStep': 0.25},
        {'pathScaleBySpeed': True},
        {'minPathRange': 1.0},
        {'pathRangeStep': 0.5},
        {'pathRangeBySpeed': True},
        {'pathCropByGoal': True},
        {'autonomyMode': _bool_arg('autonomyMode')},
        {'autonomySpeed': _float_arg('autonomySpeed')},
        {'joyToSpeedDelay': _float_arg('joyToSpeedDelay')},
        {'joyToCheckObstacleDelay': 5.0},
        {'goalClearRange': 0.5},
        {'goalX': _float_arg('goalX')},
        {'goalY': _float_arg('goalY')},
    ]


def _path_follower_params():
    return [
        {'sensorOffsetX': _float_arg('sensorOffsetX')},
        {'sensorOffsetY': _float_arg('sensorOffsetY')},
        {'pubSkipNum': 1},
        {'twoWayDrive': _bool_arg('twoWayDrive')},
        {'lookAheadDis': 0.5},
        {'yawRateGain': 7.5},
        {'stopYawRateGain': 7.5},
        {'maxYawRate': 90.0},
        {'maxSpeed': _float_arg('maxSpeed')},
        {'maxAccel': 2.0},
        {'switchTimeThre': 1.0},
        {'dirDiffThre': 0.1},
        {'stopDisThre': 0.2},
        {'slowDwnDisThre': 0.85},
        {'useInclRateToSlow': False},
        {'inclRateThre': 120.0},
        {'slowRate1': 0.25},
        {'slowRate2': 0.5},
        {'slowTime1': 2.0},
        {'slowTime2': 2.0},
        {'useInclToStop': False},
        {'inclThre': 45.0},
        {'stopTime': 5.0},
        {'noRotAtStop': False},
        {'noRotAtGoal': True},
        {'autonomyMode': _bool_arg('autonomyMode')},
        {'autonomySpeed': _float_arg('autonomySpeed')},
        {'joyToSpeedDelay': _float_arg('joyToSpeedDelay')},
    ]


def generate_exploration_launch_description(planner_params, map_params):
    local_planner_dir = get_package_share_directory('local_planner')
    path_folder = os.path.join(local_planner_dir, 'paths')

    nodes = [
        Node(
            package='lrae_planner',
            executable='lrae_planner_node',
            name='lrae_planner_node',
            output='screen',
            parameters=[planner_params],
        ),
        Node(
            package='lrae_planner',
            executable='exploration_map_merge',
            name='exploration_map_merge',
            output='screen',
            parameters=[map_params],
        ),
        Node(
            package='gen_local_goal',
            executable='gen_local_goal_node',
            name='gen_local_goal_node',
            output='screen',
        ),
        Node(
            package='local_planner',
            executable='localPlanner',
            name='localPlanner',
            output='screen',
            on_exit=Shutdown(),
            remappings=[
                ('/state_estimation', '/laser_odom_init'),
                ('/way_point', '/look_ahead_goal'),
                ('/terrain_map', '/local_traversibility_ponit_cloud'),
            ],
            parameters=_local_planner_params(path_folder),
        ),
        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            on_exit=Shutdown(),
            parameters=_path_follower_params(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vehicleTransPublisher',
            arguments=[
                '--x', PythonExpression(['-(', LaunchConfiguration('sensorOffsetX'), ')']),
                '--y', PythonExpression(['-(', LaunchConfiguration('sensorOffsetY'), ')']),
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'sensor',
                '--child-frame-id', 'vehicle',
            ],
        ),
    ]

    return LaunchDescription(_declared_args() + nodes)


def generate_launch_description():
    planner_params = {
        'angle_pen': 0.45,
        'update_cen_thre': 6,
        'unknown_num_thre': 200,
        'minrange': 20.0,
        'limit_max_square': False,
        'use_go_end_nearest': False,
    }
    map_params = {
        'map_w': 216,
        'map_h': 216,
        'mapinitox': -5.0,
        'mapinitoy': -5.0,
        'merge_size': 9.0,
        'safe_obs_dis': 1.0,
    }
    return generate_exploration_launch_description(planner_params, map_params)
