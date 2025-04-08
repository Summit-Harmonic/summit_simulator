from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions

def generate_launch_description():
    description_file = LaunchConfiguration("description_file", default="summit_xl_std.urdf.xacro")
    prefix = LaunchConfiguration("prefix", default="")
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("robot_description"), "robots", description_file]),
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

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    #twist_mux_launcher = IncludeLaunchDescription(
    #    PathJoinSubstitution(
    #        [FindPackageShare("summit_simulator"), "launch", "twist_mux.launch.py"]
    #    ),
    #)

    nodes = [
        robot_state_publisher_node,
        #joint_state_publisher_gui_node
        
    ]

    launchers = [
        #twist_mux_launcher,
    ]

    return LaunchDescription(nodes + launchers)
