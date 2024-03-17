from os.path import join
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression

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
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher gui automatically \
        with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
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
            default_value=join(get_package_share_directory("robot_bringup"),'config/bringup','twist_mux.yaml'),
        )
    )

    # Initialize Arguments
    simulation_package = LaunchConfiguration("simulation_package")
    description_package = LaunchConfiguration("description_package")
    bringup_package = LaunchConfiguration("bringup_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_id = LaunchConfiguration('robot_id')
    config_controllers = LaunchConfiguration('config_controllers')
    description_config = LaunchConfiguration('description_config')

    # Get URDF via xacro
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

    robot_description_control = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("summit_simulator"), "config", "summit_xl_base.xacro"]),
            " ",
            "use_gazebo_classic:=true",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(simulation_package), "config", "summit.rviz"]
    )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     #namespace= robot_namespace,
    #     output='screen',
    # )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        #namespace= robot_namespace,
        output='screen',
        parameters=[{
          'use_sim_time': use_sim_time,
          'robot_description': robot_description_param,
          'publish_frequency': 100.0,
          'frame_prefix': '',
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        #namespace= robot_namespace,
        parameters=[
            {'robot_description': robot_description_control},
            config_controllers,
        ],
        # remappings=[
        # ('robotnik_base_control/odom', [robot_id,'/robotnik_base_control/odom']),
        # ('robotnik_base_control/cmd_vel_unstamped', [robot_id,'/robotnik_base_control/cmd_vel']),
        # ('joint_states', [robot_id,'/joint_states']),
        # ],
        output='screen',
    )

    controller_joint_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
    )

    controller_base_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotnik_base_control', '--controller-manager', 'controller_manager'],
    )

    # Include the twist mux launch file
    twist_mux_launcher = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare(simulation_package), "launch", "twist_mux.launch.py"]
        ),
    )

    nodes = [
        # joint_state_publisher_node,
        robot_state_publisher_node,
        controller_joint_node,
        controller_base_node,
        #ros2_control_node,
        rviz_node,
    ]

    launchers = [
        twist_mux_launcher,
    ]

    return LaunchDescription(declared_arguments + nodes + launchers)
