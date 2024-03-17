from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression

from launch_ros.actions import Node

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    # Get bcr_bot package's share directory path
    world_package = get_package_share_directory('aws_robomaker_racetrack_world')

    # Retrieve launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    robot_name = LaunchConfiguration("robot_name", default="")

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', PythonExpression(['"/robot_description"']), #PythonExpression(['"', robot_namespace, '/robot_description"']),
            '-entity', PythonExpression(['"', robot_name, '"']), #default enitity name _bcr_bot
            '-z', "0.3",
            '-x', position_x,
            '-y', position_y,
            '-Y', orientation_yaw
        ]
    )

    # Include the Gazebo launch file
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={"verbose": "false"}.items(),
    )

    return LaunchDescription([
        # Declare launch arguments
        #DeclareLaunchArgument('world', default_value=[PythonExpression(['"',join(world_package, 'worlds'),'" + "/racetrack_day.world"']),'']),
        DeclareLaunchArgument('world', default_value = 'empty.world'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value = use_sim_time),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value = odometry_source),
        #DeclareLaunchArgument("robot_namespace", default_value = robot_namespace),
        gazebo,
        spawn_entity
    ])