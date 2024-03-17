from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression

from launch_ros.actions import Node

def generate_launch_description():
    world_package = get_package_share_directory('aws_robomaker_racetrack_world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    robot_name = LaunchConfiguration("robot_name", default="")

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', PythonExpression(['"/robot_description"']), 
            '-entity', PythonExpression(['"', robot_name, '"']), 
            '-x', position_x,
            '-y', position_y,
            '-Y', orientation_yaw
        ]
    )

    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={"verbose": "false"}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=[PythonExpression(['"',join(world_package, 'worlds'),'" + "/racetrack_day.world"']),'']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value = use_sim_time),
        DeclareLaunchArgument("position_x", default_value="10.0"),
        DeclareLaunchArgument("position_y", default_value="10.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="2.35"),
        DeclareLaunchArgument("odometry_source", default_value = odometry_source),
        gazebo,
        spawn_entity
    ])