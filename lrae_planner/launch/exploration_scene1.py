from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 声明所有参数
    declared_args = [
        DeclareLaunchArgument('sensorOffsetX', default_value='0.0'),
        DeclareLaunchArgument('sensorOffsetY', default_value='0.0'),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.0'),
        DeclareLaunchArgument('twoWayDrive', default_value='true'),
        DeclareLaunchArgument('maxSpeed', default_value='1.0'),
        DeclareLaunchArgument('autonomyMode', default_value='true'),
        DeclareLaunchArgument('autonomySpeed', default_value='1.0'),
        DeclareLaunchArgument('joyToSpeedDelay', default_value='2.0'),
        DeclareLaunchArgument('goalX', default_value='0.0'),
        DeclareLaunchArgument('goalY', default_value='0.0'),
    ]

    # 获取包路径
    local_planner_dir = get_package_share_directory('local_planner')
    path_folder = os.path.join(local_planner_dir, 'paths')

    # 定义各节点
    nodes = [
        Node(
            package='lrae_planner',
            executable='lrae_planner_node',
            name='lrae_planner_node',
            output='screen',
            parameters=[
                {'angle_pen': 0.45},
                {'update_cen_thre': 6},
                {'unknown_num_thre': 200},
                {'minrange': 20.0},
                {'limit_max_square':False},
                {'use_go_end_nearest': False},
            ]
        ),
        Node(
            package='lrae_planner',
            executable='exploration_map_merge',
            name='exploration_map_merge',
            output='screen',
            parameters=[
                {'map_w': 216},
                {'map_h': 216},
                {'mapinitox': -5.0},
                {'mapinitoy': -5.0},
                {'merge_size': 9.0},   #地图合并的有效范围，点与机器人位置不能超过一定距离
                {'safe_obs_dis': 1.0}, #安全障碍物距离  
            ]
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
            parameters=[
                {'pathFolder': path_folder},
                {'vehicleLength': 0.6},
                {'vehicleWidth': 0.6},
                {'sensorOffsetX': ParameterValue(LaunchConfiguration('sensorOffsetX'), value_type=float)},
                {'sensorOffsetY': ParameterValue(LaunchConfiguration('sensorOffsetY'), value_type=float)},
                {'twoWayDrive': ParameterValue(LaunchConfiguration('twoWayDrive'), value_type=bool)},
                {'laserVoxelSize': 0.05},
                {'terrainVoxelSize': 0.1},
                {'useTerrainAnalysis': True},
                {'checkObstacle': True},
                {'checkRotObstacle': False},
                {'adjacentRange': 4.25},
                {'obstacleHeightThre': 0.3},   #修改 原0.15
                {'groundHeightThre': 0.1},
                {'costHeightThre': 0.5},       #修改 原0.1
                {'costScore': 0.02},
                {'useCost': False},
                {'pointPerPathThre': 2},
                {'minRelZ': -0.5},
                {'maxRelZ': 0.25},
                {'maxSpeed': ParameterValue(LaunchConfiguration('maxSpeed'), value_type=float)},
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
                {'autonomyMode': ParameterValue(LaunchConfiguration('autonomyMode'), value_type=bool)},
                {'autonomySpeed': ParameterValue(LaunchConfiguration('autonomySpeed'), value_type=float)},
                {'joyToSpeedDelay': ParameterValue(LaunchConfiguration('joyToSpeedDelay'), value_type=float)},
                {'joyToCheckObstacleDelay': 5.0},
                {'goalClearRange': 0.5},
                {'goalX': ParameterValue(LaunchConfiguration('goalX'), value_type=float)},
                {'goalY': ParameterValue(LaunchConfiguration('goalY'), value_type=float)},
            ]
        ),
        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            on_exit=Shutdown(),
            parameters=[
                {'sensorOffsetX': ParameterValue(LaunchConfiguration('sensorOffsetX'), value_type=float)},
                {'sensorOffsetY': ParameterValue(LaunchConfiguration('sensorOffsetY'), value_type=float)},
                {'pubSkipNum': 1},
                {'twoWayDrive': ParameterValue(LaunchConfiguration('twoWayDrive'), value_type=bool)},
                {'lookAheadDis': 0.5},
                {'yawRateGain': 7.5},
                {'stopYawRateGain': 7.5},
                {'maxYawRate': 90.0},
                {'maxSpeed': ParameterValue(LaunchConfiguration('maxSpeed'), value_type=float)},
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
                {'autonomyMode': ParameterValue(LaunchConfiguration('autonomyMode'), value_type=bool)},
                {'autonomySpeed': ParameterValue(LaunchConfiguration('autonomySpeed'), value_type=float)},
                {'joyToSpeedDelay': ParameterValue(LaunchConfiguration('joyToSpeedDelay'), value_type=float)},
            ]
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
            ]
        )
    ]

    return LaunchDescription(declared_args + nodes)
