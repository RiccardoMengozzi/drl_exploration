from rclpy.node import Node
from stable_baselines3 import PPO
import os   
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor


from .ROS_interface import ROSInterface
from .simulation_reset import SimulationReset
from .exploration_env import ExplorationEnv

import wandb
from wandb.integration.sb3 import WandbCallback
from stable_baselines3.common.callbacks import BaseCallback

N_ITERS = 100000


class SaveModelByEpisodeCallback(BaseCallback):
    def __init__(self, save_freq: int, save_path: str, verbose=0):
        super(SaveModelByEpisodeCallback, self).__init__(verbose)
        self.save_freq = save_freq  # Number of episodes between saves
        self.save_path = save_path    # Path to save the model
        self.step = 0         # Keep track of the number of episodes

    def _on_step(self) -> bool:
        self.step += 1
            
        # Check if we need to save the model
        if self.step % self.save_freq == 0:
            # Create the save directory if it doesn't exist
            os.makedirs(self.save_path, exist_ok=True)

            # Save the model
            self.model.save(os.path.join(self.save_path, f"_{self.step}.zip"))
            if self.verbose > 0:
                print(f"Model saved to {self.save_path}/_{self.step}.zip")

        return True  # Continue training


class CustomLoggingCallback(BaseCallback):
    def __init__(self, env: ExplorationEnv, verbose=0):
        super(CustomLoggingCallback, self).__init__(verbose)
        self.env = env
        self.episode_steps = 0
        self.episodes = 0
        self.steps = 0
        self.episode_reward = 0.0
        self.known_map_percentage = 0.0

    def _on_step(self) -> bool:
        # Each time _on_step is called, it means one step was taken in the environment
        self.episode_steps += 1
        self.steps += 1

        # Retrieve the reward for this step

        # Access done and truncated from the locals
        done = self.locals['dones'][0]  # 'dones' is a list, use [0] for non-vectorized envs
        truncated = self.locals['infos'][0].get('TimeLimit.truncated', False)  # Check for truncation

        # rollout ends when either done or truncated is True
        if done or truncated:
            self.episode_reward += self.locals['rewards'][0]  # 'rewards' is a list, use [0] for non-vectorized envs
            self.episodes += 1
            self.known_map_percentage = self.env.get_known_map_percentage()

            custom_data = {
                'steps': self.steps,
                'episodes': self.episodes,
                'episode_steps': self.episode_steps,
                'reward': self.episode_reward,
                'known_map_percentage': self.known_map_percentage
            }
            wandb.log(custom_data)
            # Reset counters for the next episode
            self.episode_steps = 0
            self.episode_reward = 0.0
            self.known_map_percentage = 0.0
        return True



class Agent(Node):
    def __init__(self, ros_interface: ROSInterface, sim_reset: SimulationReset, models_directory: str, logs_directory: str):
        super().__init__('agent')
        self.save_model_path = models_directory
        self.log_path = logs_directory
        self.ros_int = ros_interface
        self.sim_reset = sim_reset
        self.env = ExplorationEnv(ros_interface=self.ros_int, sim_reset=self.sim_reset)        
        self.env = Monitor(self.env)

    def train(self):

        os.makedirs(self.save_model_path, exist_ok=True)

        config = {
                "entity": "RiccardoMengozzi",
                "policy_type": "MultiInputPolicy",
                "total_timesteps": N_ITERS,
                "learning_rate":1e-3,
                }
        
        run = wandb.init(
            project="drl_exploration",
            config=config,
            sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
            mode='online'
            )
        
        wand_cb = WandbCallback(gradient_save_freq=100,
                                verbose=2)


        model = PPO("MultiInputPolicy", self.env, verbose=1, 
                    tensorboard_log=self.log_path)
        callbacks = [wand_cb, CustomLoggingCallback(self.env), SaveModelByEpisodeCallback(save_freq=10, save_path=f"{self.save_model_path}/{run.name}")]
        try:
            print('start learning')
            model.learn(total_timesteps=N_ITERS, log_interval=1, callback=callbacks, )  
            model.save(os.path.join(f"{self.save_model_path}/{run.name}_FINAL"))
            print('Training Completed')

        except KeyboardInterrupt:
            print('Training Interrupted')

    def eval(self):
        pass

    def test(self):
        self.env.test()
        import time
        time.sleep(1)
        pass

    def check(self):
        check_env(self.env, warn=True)

