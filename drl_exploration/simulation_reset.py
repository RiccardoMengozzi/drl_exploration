

import time
import os
import signal
import subprocess

from .ROS_interface import ROSInterface

NODES_NAME = [
    'cartographer_node',
    'cartographer_occupancy_grid_node',
    'rviz2'
]


class SimulationReset:
    def __init__(self, ros_interface: ROSInterface):  
        self._reset_finished = False
        self.ros_interface = ros_interface


    def reset_done(self):
        return self._reset_finished
    
    def reset_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                self.ros_interface.get_logger().info('[simulation_reset] Simulation reset successfully')
                self._reset_finished = True
            else:
                self.ros_interface.get_logger().error('[simulation_reset] Failed to reset simulation')
        except Exception as e:
            self.ros_interface.get_logger().error(f'[simulation_reset] Service call failed: {str(e)}')

    def launch_ros2_file(self):
        command = "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True"

        try:
            # Start the process
            process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            # Get the PID of the launched process
            pid = process.pid
            time.sleep(2)
            self.ros_interface.get_logger().info('[simulation_reset] Simulation has been reset successfully')

            return pid  # Return the PID of the launch process

        except Exception as e:
            self.ros_interface.get_logger().error(f'[simulation_reset] Error launching the ROS 2 file: {e}')
            return None

    def find_process_pids(self, process_names):
        pids = []
        for name in process_names:
            try:
                # Use pgrep to find PIDs of the specified processes
                output = subprocess.check_output(['pgrep', '-f', name])
                pids.extend(map(int, output.decode().strip().split('\n')))
                self.ros_interface.get_logger().info(f'[simulation_reset] Found: {name}, PID = {pids[-1]}')
            except subprocess.CalledProcessError:
                self.ros_interface.get_logger().error(f'[simulation_reset] No process found for {name}.')
        return pids

    def kill_processes(self, pids):
        for pid in pids:
            try:
                # Send SIGTERM to the process
                os.kill(pid, signal.SIGTERM)
                self.ros_interface.get_logger().info(f'[simulation_reset] Successfully sent SIGTERM to PID {pid}.')
                
                # Wait for the process to terminate
                time.sleep(0.5)  # Give it a moment to shut down
                if os.path.exists(f'/proc/{pid}'):
                    self.ros_interface.get_logger().error(f'[simulation_reset] Process {pid} is still running, sending SIGKILL.')
                    os.kill(pid, signal.SIGKILL)  # Force kill if it's still running
            except OSError as e:
                self.ros_interface.get_logger().error(f'[simulation_reset] Error killing process {pid}: {e}')

    def reset_simulation(self):
        pids = self.find_process_pids(NODES_NAME)
        self.kill_processes(pids)
        future = self.ros_interface.reset_simulation()
        if future:
            _ = self.launch_ros2_file()
            return True
        else:
            return False
