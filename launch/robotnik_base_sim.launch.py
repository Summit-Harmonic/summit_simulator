from os.path import join
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
       DeclareLaunchArgument(
            "robot_id", 
            default_value="robot", 
            description="Robot ID",
        )
    )
    declared_arguments.append(
       DeclareLaunchArgument(
            "config_controllers", 
            default_value=join(get_package_share_directory("summit_simulator"),'config','summit_xl_base.yaml'), 
            description="Controller config file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_config", 
            default_value=join(get_package_share_directory("summit_simulator"),'config','summit_xl_base.xacro'), 
            description="Description config file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "simulation_package",
            default_value="summit_simulator",
            description="Description package with robot URDF/xacro files made by IRL.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="summit_xl_description",
            description="Description package with robot URDF/xacro files made by manufacturer.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="summit_xl_std.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bringup_package",
            default_value='robot_bringup',
            description="Bringup package with information about the sensors and launchers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value='true',
            description="Time configuration for simulation process.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "twist_mux_file",
            default_value=join(get_package_share_directory("summit_simulator"),'config','twist_mux.yaml'),
        )
    )

    simulation_package = LaunchConfiguration("simulation_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_id = LaunchConfiguration('robot_id')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("summit_simulator"), "robots", description_file]),
            " ",
            "prefix:=", prefix
        ]
    )
    robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        #namespace=robot_id,
        output='screen',
        parameters=[{
          'use_sim_time': use_sim_time,
          'robot_description': robot_description_param,
          'publish_frequency': 100.0,
          'frame_prefix': '',
        }],
    )

    twist_mux_launcher = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare(simulation_package), "launch", "twist_mux.launch.py"]
        ),
    )

    nodes = [
        robot_state_publisher_node,
        
    ]

    launchers = [
        twist_mux_launcher,
    ]

    return LaunchDescription(declared_arguments + nodes + launchers)
