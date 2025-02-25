import os
import yaml
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def load_yaml(file_path):
    """
    Load a YAML file and return its content as a dictionary.

    :param file_path: Path to the YAML file.
    :return: Parsed YAML content as a dictionary.
    """
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)
    
def generate_launch_description():
    launch_actions = []
    tb3_multi_env_spawner_pkg_dir = get_package_share_directory('tb3_multi_env_spawner')

    tb3_multi_env_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_multi_env_spawner_pkg_dir, 'launch', 'tb3_multi_env_spawner.launch.py')
        )
    )


    yaml_file_path = os.path.join(
        os.getcwd(),
        'src',
        'tb3_multi_env_spawner',
        'config',
        'launch_params.yaml'
    )

    # Load launch parameters from YAML
    params = load_yaml(yaml_file_path)

    # Extract parameters from the YAML file
    num_envs = params['env']['num_envs']

    drl_exploration_cmd = Node(
        package='drl_exploration',
        executable='drl_exploration',
        name='drl_exploration',
        output='screen',
        parameters=[{'num_envs': num_envs}]
    )




    launch_actions.append(tb3_multi_env_spawner_cmd, drl_exploration_cmd)
    return LaunchDescription(launch_actions)