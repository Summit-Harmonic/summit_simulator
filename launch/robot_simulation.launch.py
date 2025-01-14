from os.path import join
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from controller_manager.launch_utils import generate_load_controller_launch_description


def generate_launch_description():
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description="use_sim_time simulation parameter"
    )
    pkg_share_folder = get_package_share_directory('robot_description')

    # Load joint state broadcaster controller
    joint_state_broadcaster = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name='joint_state_broadcaster',
                controller_params_file=join(
                    pkg_share_folder, 'config', 'joint_state_broadcaster.yaml'))
        ],
    )

    # Load robotnik_base_controller controller
    base_controller = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name='robotnik_base_controller',
                controller_params_file=join(
                    pkg_share_folder, 'config', 'summit_xl_base.yaml')
            )
        ],
    )

    ld = LaunchDescription()
    ld.add_action(joint_state_broadcaster)
    ld.add_action(base_controller)
    ld.add_action(declare_sim_time)
    return ld