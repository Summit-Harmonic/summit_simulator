from os.path import join
from os import environ, pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, OpaqueFunction, GroupAction, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from controller_manager.launch_utils import generate_load_controller_launch_description

# Function to get the model paths
def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = join(package_path, "share")

        model_paths += model_path

    if 'GZ_SIM_RESOURCE_PATH' in environ:
        model_paths += pathsep + environ['GZ_SIM_RESOURCE_PATH']

    return model_paths

# Launch description
def generate_launch_description():
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description="use_sim_time simulation parameter"
    )
    model_path = ''
    resource_path = ''

    pkg_path = get_package_share_directory('robot_description')
    model_path += join(pkg_path, 'models')
    resource_path += pkg_path + model_path

    if 'GZ_SIM_MODEL_PATH' in environ:
        model_path += pathsep+environ['GZ_SIM_MODEL_PATH']
    if 'GZ_SIM_RESOURCE_PATH' in environ:
        resource_path += pathsep+environ['GZ_SIM_RESOURCE_PATH']

    model_path = get_model_paths(['robot_description'])

    robot_description_launcher = IncludeLaunchDescription(
       PathJoinSubstitution(
           [FindPackageShare("robot_description"), "launch", "robot_description.launch.py"]
       )
    )

    # Get the world file from the launch arguments
    world_arg = DeclareLaunchArgument(
        'world', default_value=join(
            get_package_share_directory('urjc_excavation_world'),
            'worlds',
            'urjc_excavation.world'))

    # Launch Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', LaunchConfiguration('world')]}.items()
    )

    # Launch Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [' -g ']}.items(),
    )

    # Rviz config and launching
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("summit_simulator"), "rviz", "summit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Spawning robot 
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-model",
            "summit_xl",
            "-topic",
            "robot_description",
            "-use_sim_time",
            "True",
        ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        parameters=[
            {
                'config_file': join(
                    pkg_path, 'config', 'summit_bridge.yaml'
                ),
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    # Image bridge
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/front_camera_right/image",
            "/front_camera_left/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True,
             'camera.image.compressed.jpeg_quality': 75},
        ],
    )

    # Load joint state broadcaster controller
    joint_state_broadcaster = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name='joint_state_broadcaster',
                controller_params_file=join(
                    pkg_path, 'config', 'summit_controllers.yaml'))
        ],
    )

    # Load robotnik_base_controller controller
    base_controller = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name='robotnik_base_control',
                controller_params_file=join(
                    pkg_path, 'config', 'summit_controllers.yaml')
            )
        ],
    )

    # When gazebo exits, run the joint state broadcaster
    on_gazebo_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[
                joint_state_broadcaster,
                base_controller,
            ],
        )
    )

    # When gazebo exits, run the joint state broadcaster
    RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo_spawn_robot,
            on_start=[
                robot_description_launcher
            ]
        )
    )

    # Twist stamped
    twist_stamped = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
            }],
        remappings={('cmd_vel_out', '/robotnik_base_control/cmd_vel'),
                    ('cmd_vel_in', '/cmd_vel')},
    )

    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_MODEL_PATH', model_path))
    ld.add_action(world_arg)
    ld.add_action(robot_description_launcher)
    ld.add_action(declare_sim_time)
    ld.add_action(bridge)
    ld.add_action(gz_image_bridge_node)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(rviz_node)
    ld.add_action(gazebo_spawn_robot)
    ld.add_action(on_gazebo_exit)
    ld.add_action(twist_stamped)
    return ld