import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mengo/turtlebot3_ws/src/drl_exploration/install/drl_exploration'
