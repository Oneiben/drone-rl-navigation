from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper
from stable_baselines3 import PPO

env = "./simulation-path" # Replace with the actual built simulation path
unity_env = UnityEnvironment(env,no_graphics_monitor=False, no_graphics=False, worker_id=292)  
env = UnityToGymWrapper(unity_env, uint8_visual=True)  # Assuming you need grayscale images as uint8

model = PPO.load("./logs/best_model/best_model.zip", env=env)

obs = env.reset()
total_rewards = []

for i in range(10000):
    action, _states = model.predict(obs, deterministic=True)    
    obs, rewards, dones, info = env.step(action)
    total_rewards.append(rewards)

    # Check if episode has ended, then reset
    if dones:
        print(sum(total_rewards))
        
        total_rewards = []

        obs = env.reset()  # Reset the environment

    env.render("human")
