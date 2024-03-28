from os.path import join
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
       DeclareLaunchArgument(
            "config_controllers", 
            default_value=join(get_package_share_directory("summit_simulator"),'config','summit_xl_base.yaml'), 
            description="Controller config file",
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

    '''
    prefix = LaunchConfiguration("prefix")
    config_controllers = LaunchConfiguration('config_controllers')

    robot_description_control = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("summit_simulator"), "config", "summit_xl_base.xacro"]),
            " ",
            "use_gazebo_classic:=true",
        ]
    )

    
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_control},
            config_controllers,
        ],
        remappings=[
        ('robotnik_base_control/odom', [prefix,'/robotnik_base_control/odom']),
        ('robotnik_base_control/cmd_vel_unstamped', [prefix,'/robotnik_base_control/cmd_vel']),
        ('joint_states', [prefix,'/joint_states']),
      ],
        output='screen',
    )
    '''
    
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

    nodes = [
        controller_joint_node,
        controller_base_node,
    ]


    return LaunchDescription(declared_arguments + nodes)