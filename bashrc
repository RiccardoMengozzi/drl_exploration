#da modificare con cartelle giuste
#export GAZEBO_MODEL_PATH="/home/mengo/turtlebot3_ws/src/drl_exploration/models/fuel_models:/home/mengo/turtlebot3_ws/src/drl_exploration/models/aws_models"
# automatic commands at terminal opening
source /opt/ros/humble/setup.bash
. /usr/share/gazebo/setup.sh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger