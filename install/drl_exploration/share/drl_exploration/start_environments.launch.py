import os
import yaml

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    ld = LaunchDescription()
    package_name = 'drl_exploration'
    package_share_directory = get_package_share_directory(package_name)
    workspace_path = os.path.abspath(os.path.join(package_share_directory, '../../../..'))

    yaml_file_path = os.path.join(
        workspace_path,
        'src',
        'drl_exploration',
        'config',
        'launch_params.yaml'
    )
    params = load_yaml(yaml_file_path)
    num_envs = params['env']['num_envs']



    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')
    tb3_pkg_dir = get_package_share_directory('turtlebot3_gazebo')


    world = LaunchConfiguration('world', default=os.path.join(tb3_pkg_dir, 'worlds', 'empty_world.world'))
    gui = LaunchConfiguration('gui', default='true')
    verbose = LaunchConfiguration('verbose', default='false')


    for i in range(num_envs):
        set_URI_cmd = SetEnvironmentVariable(
            'GAZEBO_MASTER_URI', f'http://localhost:{11345 + i}'
            )

        gz_server_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzserver.launch.py')
                ),
            launch_arguments={
                'world': world,
                'verbose': verbose
            }.items()
        )

        gz_client_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzclient.launch.py')
                ),
            condition=IfCondition(gui),
            launch_arguments={
                'verbose': verbose
            }.items()
        )

        ld.add_action(set_URI_cmd)
        ld.add_action(gz_server_cmd)
        ld.add_action(gz_client_cmd)

    return ld