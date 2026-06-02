from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
                {'limit_max_square':True},
                {'use_go_end_nearest': True},
                {'end_neacen_disthre': 10.0},
                {'end_cur_disrate': 2.0}
            ]
        ),
        Node(
            package='lrae_planner',
            executable='exploration_map_merge',
            name='exploration_map_merge',
            output='screen',
            parameters=[
                {'map_w': 200},
                {'map_h': 200},
                {'mapinitox': -10.0},
                {'mapinitoy': -10.0},
                {'merge_size': 9.0},
                {'safe_obs_dis': 1.0}
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
            remappings=[
                ('/state_estimation', '/laser_odom_init'),
                ('/way_point', '/look_ahead_goal'),
                ('/terrain_map', '/local_traversibility_ponit_cloud'),
            ],
            parameters=[
                {'pathFolder': path_folder},
                {'vehicleLength': 2.5},
                {'vehicleWidth': 2.5},
                {'sensorOffsetX': LaunchConfiguration('sensorOffsetX')},
                {'sensorOffsetY': LaunchConfiguration('sensorOffsetY')},
                {'twoWayDrive': LaunchConfiguration('twoWayDrive')},
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
                {'maxSpeed': LaunchConfiguration('maxSpeed')},
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
                {'autonomyMode': LaunchConfiguration('autonomyMode')},
                {'autonomySpeed': LaunchConfiguration('autonomySpeed')},
                {'joyToSpeedDelay': LaunchConfiguration('joyToSpeedDelay')},
                {'joyToCheckObstacleDelay': 5.0},
                {'goalClearRange': 0.5},
                {'goalX': LaunchConfiguration('goalX')},
                {'goalY': LaunchConfiguration('goalY')},
            ]
        ),
        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            parameters=[
                {'sensorOffsetX': LaunchConfiguration('sensorOffsetX')},
                {'sensorOffsetY': LaunchConfiguration('sensorOffsetY')},
                {'pubSkipNum': 1},
                {'twoWayDrive': LaunchConfiguration('twoWayDrive')},
                {'lookAheadDis': 0.5},
                {'yawRateGain': 7.5},
                {'stopYawRateGain': 7.5},
                {'maxYawRate': 90.0},
                {'maxSpeed': LaunchConfiguration('maxSpeed')},
                {'maxAccel': 2.0},
                {'switchTimeThre': 1.0},
                {'dirDiffThre': 0.1},
                {'stopDisThre': 0.25},
                {'slowDwnDisThre': 0.89},
                {'useInclRateToSlow': True},
                {'inclRateThre': 35.0},
                {'slowRate1': 0.25},
                {'slowRate2': 0.5},
                {'slowTime1': 2.0},
                {'slowTime2': 2.0},
                {'useInclToStop': False},
                {'inclThre': 45.0},
                {'stopTime': 5.0},
                {'noRotAtStop': False},
                {'noRotAtGoal': True},
                {'autonomyMode': LaunchConfiguration('autonomyMode')},
                {'autonomySpeed': LaunchConfiguration('autonomySpeed')},
                {'joyToSpeedDelay': LaunchConfiguration('joyToSpeedDelay')},
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vehicleTransPublisher',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
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
