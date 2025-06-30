import rclpy

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from SelfBalancingEnv import SelfBalancingEnv

if __name__ == '__main__':
    rclpy.init()
    #check_env(SelfBalancingEnv(render_mode=None, maxStep=1))

    env = SelfBalancingEnv(render_mode=None, maxStep=1000)
    
    try:
        model = PPO.load("ppo_bipedal_standing", env=env)
        print("Loaded existing model")
    except:
        model = PPO("MlpPolicy", env, n_steps=2500)
        print("Training new model")

    model.learn(total_timesteps=500000, progress_bar=True)
    model.save("ppo_balancing")

    env.close()
