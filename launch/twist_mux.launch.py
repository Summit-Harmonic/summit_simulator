import launch, launch_ros
from ament_index_python.packages import get_package_share_directory

def read_params(ld : launch.LaunchDescription):
  robot_simulation = get_package_share_directory('summit_simulator')
  robot_id = launch.substitutions.LaunchConfiguration('robot_id')
  config = launch.substitutions.LaunchConfiguration('config')

  ld.add_action(launch.actions.DeclareLaunchArgument(
    'robot_id', default_value='robot', description='Robot ID',
  ))

  ret = {
    'robot_id': robot_id,
    'config': config,
  }
  return ret


def generate_launch_description():
  ld = launch.LaunchDescription()


  ld.add_action(launch_ros.actions.Node(
    package='twist_mux',
    executable='twist_mux',
    remappings={('cmd_vel_out', 'robotnik_base_control/cmd_vel')},
    output='screen',
  ))

  return ld
