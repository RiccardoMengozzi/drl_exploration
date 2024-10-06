import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():


    turtlebot_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    turtlebot_cartographer_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot_launch_file_dir, '/turtlebot3_world.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot_cartographer_launch_file_dir, '/cartographer.launch.py']),
            launch_arguments={'use_sim_time': 'true'}.items()
        )
                            
    ])