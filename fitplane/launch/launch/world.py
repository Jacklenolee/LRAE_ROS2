from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    paused = LaunchConfiguration('paused')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    verbose = LaunchConfiguration('verbose')
    world_name = LaunchConfiguration('world_name')

    world_filename = PythonExpression(["'map_' + '", world_name, "' + '.world'"])
    world_path = PathJoinSubstitution([
        FindPackageShare('simworld'),
        'worlds',
        world_filename,
    ])
    gazebo_server = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gzserver.launch.py',
    ])
    gazebo_client = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gzclient.launch.py',
    ])
    start_gui = IfCondition(
        PythonExpression(["'", gui, "' == 'true' and '", headless, "' == 'false'"])
    )

    return LaunchDescription([
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('world_name', default_value='scene_2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_server]),
            launch_arguments={
                'world': world_path,
                'pause': paused,
                'verbose': verbose,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_client]),
            condition=start_gui,
            launch_arguments={
                'verbose': verbose,
            }.items(),
        ),
    ])
