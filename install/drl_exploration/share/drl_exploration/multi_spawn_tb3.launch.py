import os
import yaml

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

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
    pos_x = params['robot']['pose']['x']
    pos_y = params['robot']['pose']['y']
    yaw = params['robot']['pose']['yaw']


    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    file = LaunchConfiguration('file', default=urdf_path)
    entity = LaunchConfiguration('entity', default=TURTLEBOT3_MODEL)
    pos_x = LaunchConfiguration('x', default=pos_x)
    pos_y = LaunchConfiguration('y', default=pos_y)
    yaw = LaunchConfiguration('Y', default=yaw)


    # use_sim_time_arg_cmd = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='false',
    #     description='Use simulation (Gazebo) clock if true'
    # )
    # urdf_file_arg_cmd = DeclareLaunchArgument(
    #     'file',
    #     default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),'models',model_folder,'model.sdf'),
    #     description='Turtlebot3 urdf file'
    # )
    # entity_arg_cmd = DeclareLaunchArgument(
    #     'entity',
    #     default_value='burger',
    #     description='Turtlebot3 model type'
    # )
    # pos_x_arg_cmd = DeclareLaunchArgument(
    #     'x',
    #     default_value='0.0',
    #     description='Turtlebot3 x position'
    # )
    # pos_y_arg_cmd = DeclareLaunchArgument(
    #     'y',
    #     default_value='0.0',
    #     description='Turtlebot3 y position'
    # )
    # yaw_arg_cmd = DeclareLaunchArgument(
    #     'Y',
    #     default_value='0.0',
    #     description='Turtlebot3 yaw position'
    # )

    # ld.add_action(use_sim_time_arg_cmd)
    # ld.add_action(urdf_file_arg_cmd)
    # ld.add_action(entity_arg_cmd)
    # ld.add_action(pos_x_arg_cmd)
    # ld.add_action(pos_y_arg_cmd)
    # ld.add_action(yaw_arg_cmd)


    for i in range(num_envs):


        robot_state_pub_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),


        spawn_tb3_cmd = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', entity,
                '-file', file,
                '-x', pos_x,
                '-y', pos_y,
                '-z', '0.01',
                '-Y', yaw
            ],
            output='screen',
        )

        ld.add_action(robot_state_pub_cmd)
        ld.add_action(spawn_tb3_cmd)

    return ld