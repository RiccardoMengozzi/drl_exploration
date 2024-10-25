import os
import math
import json
import numpy as np
import random
from ament_index_python.packages import get_package_share_directory


def generate_centers(n_points, step_x=15, step_y=15):
    # Calculate grid dimensions to be as close to square as possible
    cols = math.ceil(math.sqrt(n_points))  # Number of columns
    rows = math.ceil(n_points / cols)      # Calculate corresponding rows

    points = []
    
    for row in range(rows):
        for col in range(cols):
            # Determine x and y positions
            if row % 2 == 0:  # Even row: left to right
                x = col * step_x
            else:  # Odd row: right to left
                x = (cols - 1 - col) * step_x
            
            y = row * step_y

            # Append the point to the list if within the total number of points
            if len(points) < n_points:
                points.append((x, y))
    
    return points


def create_multi_env_world(num_envs, 
                           env_models, 
                           env_model_step_x=15, 
                           env_model_step_y=15, 
                           mode='single_model'):
    
        # Define the beginning of the SDF world
    try:
        sdf_start = '''<?xml version="1.0"?>
<sdf version="1.6">
    <world name="default">

        <include>
            <uri>model://ground_plane</uri>
        </include>

        <include>
            <uri>model://sun</uri>
        </include>

        <scene>
            <shadows>false</shadows>
        </scene>

        <gui fullscreen='0'>
            <camera name='user_camera'>
                <pose frame=''>0.319654 -0.235002 9.29441 0 1.5138 0.009599</pose>
                <view_controller>orbit</view_controller>
                <projection_type>perspective</projection_type>
            </camera>
        </gui>

        <physics type="ode">
            <real_time_update_rate>1000.0</real_time_update_rate>
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1</real_time_factor>
            <ode>
                <solver>
                    <type>quick</type>
                    <iters>150</iters>
                    <precon_iters>0</precon_iters>
                    <sor>1.400000</sor>
                    <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
                </solver>
                <constraints>
                    <cfm>0.00001</cfm>
                    <erp>0.2</erp>
                    <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
                    <contact_surface_layer>0.01000</contact_surface_layer>
                </constraints>
            </ode>
        </physics>\n'''

    # Generate the big house blocks dynamically
        house_blocks = ""
        centers = generate_centers(num_envs, step_x=env_model_step_x, step_y=env_model_step_y)

        for i in range(num_envs):
            env_model = env_models[i]
            center = centers[i]
            house_blocks += f'''


    <!--<model name="aws_robomaker_warehouse_ShelfF_01_001">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ShelfF_01</uri>
            <pose>-5.795143 -0.956635 0 0 0 0</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_WallB_01_001">-->
        <include>
            <uri>model://aws_robomaker_warehouse_WallB_01</uri>
            <pose>0.0 0.0 0 0 0 0</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ShelfE_01_001">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ShelfE_01</uri>
            <pose>4.73156 0.57943 0 0 0 0</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ShelfE_01_002">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ShelfE_01</uri>
            <pose>4.73156 -4.827049 0 0 0 0</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ShelfE_01_003">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ShelfE_01</uri>
            <pose>4.73156 -8.6651 0 0 0 0</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ShelfD_01_001">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ShelfD_01</uri>
            <pose>4.73156 -1.242668 0 0 0 0</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ShelfD_01_002">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ShelfD_01</uri>
            <pose>4.73156 -3.038551 0 0 0 0</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ShelfD_01_003">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ShelfD_01</uri>
            <pose>4.73156 -6.750542 0 0 0 0</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_GroundB_01_001">-->
        <include>
            <uri>model://aws_robomaker_warehouse_GroundB_01</uri>
            <pose>0.0 0.0 -0.090092 0 0 0</pose>
        </include>
    <!--</model>-->


    <!--<model name="aws_robomaker_warehouse_Bucket_01_020">-->
        <include>
            <uri>model://aws_robomaker_warehouse_Bucket_01</uri>
            <pose>0.433449 9.631706 0 0 0 -1.563161</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_Bucket_01_021">-->
        <include>
            <uri>model://aws_robomaker_warehouse_Bucket_01</uri>
            <pose>-1.8321 -6.3752 0 0 0 -1.563161</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_Bucket_01_022">-->
        <include>
            <uri>model://aws_robomaker_warehouse_Bucket_01</uri>
            <pose>0.433449 8.59 0 0 0 -1.563161</pose>
        </include>
    <!--</model>-->

    <!--<model name='aws_robomaker_warehouse_ClutteringA_01_016'>-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringA_01</uri>
            <pose>5.708138 8.616844 -0.017477 0 0 0</pose>
        </include>
    <!--</model>-->

    <!--<model name='aws_robomaker_warehouse_ClutteringA_01_017'>-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringA_01</uri>
            <pose>3.408638 8.616844 -0.017477 0 0 0</pose>
        </include>
    <!--</model>-->

    <!--<model name='aws_robomaker_warehouse_ClutteringA_01_018'>-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringA_01</uri>
            <pose>-1.491287 5.222435 -0.017477 0 0 -1.583185</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringC_01_027">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringC_01</uri>
            <pose>3.324959 3.822449 -0.012064 0 0 1.563871</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringC_01_028">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringC_01</uri>
            <pose>5.54171 3.816475 -0.015663 0 0 -1.583191</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringC_01_029">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringC_01</uri>
            <pose>5.384239 6.137154 0 0 0 3.150000</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringC_01_030">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringC_01</uri>
            <pose>3.236 6.137154 0 0 0 3.150000</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringC_01_031">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringC_01</uri>
            <pose>-1.573677 2.301994 -0.015663 0 0 -3.133191</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringC_01_032">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringC_01</uri>
            <pose>-1.2196 9.407 -0.015663 0 0 1.563871</pose>
        </include>
    <!--</model>-->

    <!--<model name='aws_robomaker_warehouse_ClutteringD_01_005'>-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringD_01</uri>
            <pose>-1.634682 -7.811813 -0.319559 0 0 0</pose>
        </include>
    <!--</model>-->

    <!--<model name='aws_robomaker_warehouse_TrashCanC_01_002'>-->
        <include>
            <uri>model://aws_robomaker_warehouse_TrashCanC_01</uri>
            <pose>-1.592441 7.715420 0.190610 0 0 0</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringB_01_003">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringB_01</uri>
            <pose>-5.10642 5.85081 -0.001234 0 0 3.138998</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringB_01_004">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringB_01</uri>
            <pose>-2.090845 5.85081 -0.001234 0 0 3.138998</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringB_01_005">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringB_01</uri>
            <pose>-0.304201 6.300127 -0.001234 0 0 3.138998</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringB_01_006">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringB_01</uri>
            <pose>2.988739 6.300127 -0.001234 0 0 3.138998</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringB_01_007">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringB_01</uri>
            <pose>5.500815 5.85081 -0.001234 0 0 3.138998</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringB_01_008">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringB_01</uri>
            <pose>5.500815 2.533025 -0.001234 0 0 3.138998</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringB_01_009">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringB_01</uri>
            <pose>3.272072 2.477133 -0.001234 0 0 3.138998</pose>
        </include>
    <!--</model>-->

    <!--<model name="aws_robomaker_warehouse_ClutteringB_01_010">-->
        <include>
            <uri>model://aws_robomaker_warehouse_ClutteringB_01</uri>
            <pose>0.978055 2.542203 -0.001234 0 0 3.138998</pose>
        </include>
    <!--</model>-->\n'''   

            # house_blocks += f'''
            # <include>
            #     <name>{env_model}{i}</name>
            #     <uri>model://{env_model}</uri>
            #     <pose>{center[0]} {center[1]} 0 0 0 0</pose>
            # </include>\n'''

        # Define the end of the SDF world
        sdf_end = '''
    </world>
</sdf>
'''

        sdf_content = sdf_start + house_blocks + sdf_end
        print("sdf_content: ", sdf_content)
        package_name = 'drl_exploration'
        package_share_directory = get_package_share_directory(package_name)
        workspace_path = os.path.abspath(os.path.join(package_share_directory, '../../../..'))

        save_path = os.path.join(workspace_path, "src", package_name, 'worlds')

        file_name = f"multi_env_{env_model}.world"
        if mode == 'random_models':
            file_name = "multi_env_multi_random_models.world"
        elif mode == 'multiple_models':
            file_name = f"multi_env_multi_models.world"
        file_path = os.path.join(save_path, file_name)

        with open(file_path, "w") as f:
            f.write(sdf_content)


        print(f"[INFO] [world generator] File '{file_name}' generated successfully with {num_envs} environments.")   
        return file_name
    
    except Exception as e:
        print(f"[ERROR] [world generator]: {e}")
        return None
    

def get_random_pose(models_spawn_points_dir, env_model, env_center):
    with open(os.path.join(models_spawn_points_dir, f'{env_model}_spawn_points.json'), 'r') as file:
        data = json.load(file)
    # Access the starting coordinates
    x_start = data['x_start']
    y_start = data['y_start']
    resolution = data['resolution']

    # Access the grid and convert it to a numpy array
    grid = np.array(data['grid'])
    # Find the indices of the cells where the value is 1
    indices = np.argwhere(grid == 1)
    # Select a random coordinate from the indices
    random_index = random.choice(indices)

    # Adjust the coordinates based on x_start and y_start

    random_coordinate = (x_start + random_index[0] * resolution, y_start + random_index[1] * resolution)  # (x, y)
    random_yaw = np.random.uniform(0, 2*np.pi)
    random_pose = [random_coordinate[0] + env_center[0], random_coordinate[1] + env_center[1], random_yaw]


    return random_pose
