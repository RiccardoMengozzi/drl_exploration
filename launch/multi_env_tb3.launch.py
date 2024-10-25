import os
import yaml
import random

from utils.utils import create_multi_env_world
from utils.utils import generate_centers
from utils.utils import get_random_pose

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)
    


def generate_launch_description():

    package_name = 'drl_exploration'
    package_share_directory = get_package_share_directory(package_name)
    workspace_path = os.path.abspath(os.path.join(package_share_directory, '../../../..'))
    world_path = os.path.join(workspace_path, 'src', package_name, 'worlds')
    models_spawn_points_dir = os.path.join(workspace_path, 'src', package_name, 'extras', 'models_spawn_points')
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')

    yaml_file_path = os.path.join(
        workspace_path,
        'src',
        'drl_exploration',
        'config',
        'launch_params.yaml'
    )
    params = load_yaml(yaml_file_path)

    gui = params['gazebo']['gui']
    verbose = params['launch']['verbose']
    gz_verbose = params['gazebo']['verbose']
    num_envs = params['env']['num_envs']
    mode = params['env']['mode'] 
    env_available_models = params['env']['available_models']
    env_model = params['env']['model']
    env_models = params['env']['models']
    env_model_step_x = params['env']['model_step_x']
    env_model_step_y = params['env']['model_step_y']
    random_pose = params['robot']['random_pose']
    robot_pos_x = params['robot']['pose']['x']
    robot_pos_y = params['robot']['pose']['y']
    robot_pos_yaw = params['robot']['pose']['yaw']

    if verbose:
        print("\n\n----------------- LAUNCH PARAMETERS -----------------\n")
        print(f"[INFO] [multi_env_tb3.launch.py] GUI: {gui}")
        print(f"[INFO] [multi_env_tb3.launch.py] Number of environments: {num_envs}")
        print(f"[INFO] [multi_env_tb3.launch.py] Mode: {mode}")
        print(f"[INFO] [multi_env_tb3.launch.py] Available models: {env_available_models}")
        print(f"[INFO] [multi_env_tb3.launch.py] Models: {env_models}")
        print(f"[INFO] [multi_env_tb3.launch.py] Model step x: {env_model_step_x}")
        print(f"[INFO] [multi_env_tb3.launch.py] Model step y: {env_model_step_y}")
        print(f"[INFO] [multi_env_tb3.launch.py] Random pose: {random_pose}")
        print(f"[INFO] [multi_env_tb3.launch.py] Robot pose x: {robot_pos_x}")
        print(f"[INFO] [multi_env_tb3.launch.py] Robot pose y: {robot_pos_y}")
        print(f"[INFO] [multi_env_tb3.launch.py] Robot pose yaw: {robot_pos_yaw}")
        print("\n----------------------------------------------------\n")



    if len(env_available_models) == 0:
        raise ValueError("No models available for spawning. Please add models to the 'available_models' list in the launch_params.yaml file.")
    if len(env_models) != num_envs and mode == 'multiple_models':
        raise ValueError("Number of models in the 'models' list should be equal to the number of environments. \
                         Modify the 'models' list in the launch_params.yaml file.")
    if any([model not in env_available_models for model in env_models]):
        raise ValueError("One or more models in the 'models' list are not available in the 'available_models' list. ")

    # Prepare list of models depending on the mode
    if mode == 'random_models':
        env_models = []
        env_models = [random.choice(env_available_models) for _ in range(num_envs)]

    # Just copy same model for all environments
    if mode == 'single_model':
        env_models = [env_model for _ in range(num_envs)]


    world_name = create_multi_env_world(num_envs, 
                                        env_models,
                                        env_model_step_x,
                                        env_model_step_y,
                                        mode=mode        
                                        )

    world = os.path.join(world_path, world_name)

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
        
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    model_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    
    launch_actions = []
    envs_centers = generate_centers(num_envs, step_x=env_model_step_x, step_y=env_model_step_y)


    set_models_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',f'{workspace_path}/src/drl_exploration/models:{os.environ["GAZEBO_MODEL_PATH"]}'
    )

    gz_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzserver.launch.py')
            ),
        launch_arguments={
            'world': world,
            'verbose': str(gz_verbose)
        }.items()
    )

    gz_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzclient.launch.py')
            ),
        condition=IfCondition(str(gui)),
        launch_arguments={
            'verbose': str(gz_verbose)
        }.items()
    )
    launch_actions.append(set_models_path)
    launch_actions.append(gz_server_cmd)
    launch_actions.append(gz_client_cmd)

    for i in range(num_envs):
        namespace = f'env_{i}'
        env_center = envs_centers[i]
        env_model = env_models[i]
        if random_pose:
            robot_init_pose = get_random_pose(models_spawn_points_dir, env_model, env_center)
        else:
            robot_init_pose = [robot_pos_x, robot_pos_y, robot_pos_yaw]

        robot_state_pub_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc
            }],
        )   

        spawn_tb3_cmd = Node(
            package='drl_exploration',
            executable='spawn_tb3',
            output='screen',
            arguments=[
                '-urdf', model_path,
                '-n', f'{namespace}_tb3',
                '-ns', namespace,
                '-x', str(robot_init_pose[0]),
                '-y', str(robot_init_pose[1]),
                '-z', '0.01',
                '-yaw', str(robot_init_pose[2]),
            ],
        )
        # spawn_tb3_cmd = Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     output='screen',
        #     arguments=[
        #         '-entity', f'{namespace}_tb3',
        #         '-x', robot_pos_x + str((i * 15)),
        #         '-y', robot_pos_y + str((i * 15)),
        #         '-z', '0.01',
        #         '-Y', robot_pos_yaw,
        #         '-file', model_path,
        #         '-robot_namespace', namespace
        #     ]
        # )

        launch_actions.append(robot_state_pub_cmd)
        launch_actions.append(spawn_tb3_cmd)

        
    return LaunchDescription(launch_actions)