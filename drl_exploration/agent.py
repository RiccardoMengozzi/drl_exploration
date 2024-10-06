import rclpy
from rclpy.node import Node
from stable_baselines3 import PPO
import os   
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
import gymnasium as gym
from gymnasium.envs.registration import register


import wandb
from wandb.integration.sb3 import WandbCallback

N_ITERS = 10000


class Agent(Node):
    def __init__(self, ros_interface):
        super().__init__('agent')
        self.ros_int = ros_interface

        register(
            id="ExplorationEnv-v0",
            entry_point="drl_exploration.exploration_env:ExplorationEnv",
            max_episode_steps=500)
        self.env = gym.make("ExplorationEnv-v0", ros_interface=self.ros_int) 
        self.env = Monitor(self.env)

    def train(self):
        script_path = os.path.dirname(os.path.realpath(__file__))
        save_model_path = os.path.join(os.path.dirname(script_path), 'models')
        os.makedirs(save_model_path, exist_ok=True)

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
        
        wand_cb = WandbCallback(gradient_save_freq=1,
                                model_save_freq=1,
                                model_save_path=save_model_path,
                                verbose=2)

        callbacks = [wand_cb]



        save_model_path = os.path.join("drl_exploration", "models")
        log_path = os.path.join("drl_exploration", "log")
        model = PPO("MultiInputPolicy", self.env, verbose=1, 
                    tensorboard_log=log_path)
        try:
            print('start learning')
            model.learn(total_timesteps=N_ITERS, log_interval=1, callback=callbacks, progress_bar=True)  
            model.save(os.path.join(save_model_path))
            print('Training Completed')
        except KeyboardInterrupt:
            print('Training Interrupted')
            model.save(os.path.join(save_model_path))

    def eval(self):
        pass

    def test(self):
        self.env.test()
        import time
        time.sleep(1)
        pass

    def check(self):
        check_env(self.env, warn=True)

