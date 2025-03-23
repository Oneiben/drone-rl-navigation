from stable_baselines3 import PPO
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
import torch as th
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback, BaseCallback
from stable_baselines3.common.vec_env import SubprocVecEnv
import numpy as np
import time
import os

# Utility function to create multiple environments for parallel training
def make_env(env, rank, seed=0):
    def _init():
        time.sleep(rank * 0.1)
        # Initialize Unity environment for each worker
        unity_env = UnityEnvironment(env, no_graphics_monitor=False, no_graphics=False, worker_id=rank)
        env = UnityToGymWrapper(unity_env, uint8_visual=True)
        env.seed(seed + rank)
        return env
    return _init

def main():
    # Path to Unity environment
    env = "./simulation-path" # Replace with the actual built simulation path

    # Check if there’s an existing model to load
    model_path = "./logs/best_model/best_model.zip"
    continue_training = os.path.exists(model_path)

    # Create parallel environments to speed up training
    num_envs = 4  # Number of parallel environments
    envs = [make_env(env, i) for i in range(num_envs)]
    env = SubprocVecEnv(envs)  # Parallel environments using subprocesses


    # Define custom neural network architecture for CNN-based input
    policy_kwargs = dict(
        activation_fn=th.nn.ReLU,
        net_arch=[256, 256, 128, 64]  # Custom architecture
    )

    # Define evaluation callback
    eval_callback = EvalCallback(
        env,
        callback_on_new_best=None,
        verbose=1,
        best_model_save_path='./logs/best_model/',
        log_path='./logs/results/',
        eval_freq=5000,
        n_eval_episodes=5
    )

    # Define checkpoint callback to save model periodically
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,  # Save every 10,000 steps
        save_path='./logs/checkpoints/',
        name_prefix='ppo_checkpoint'
    )

    success_callback = SuccessCallback(
        success_threshold=10,
        success_reward=180,
        verbose=1)

    # Initialize or load PPO model with CNN policy
    if continue_training:
        print("Loading existing model for continued training...")
        model = PPO.load(model_path, env=env, policy_kwargs=policy_kwargs, tensorboard_log='./logs/tensorboard/')
    else:
        print("No existing model found. Starting fresh training...")
        model = PPO(
            "CnnPolicy",
            env,
            policy_kwargs=policy_kwargs,
            verbose=1,
            tensorboard_log='./logs/tensorboard/',
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            gamma=0.99,
            gae_lambda=0.95,
            ent_coef=0.01,
            vf_coef=0.5,
            max_grad_norm=0.5
        )

    # Train the model with callbacks for evaluation, early stopping, and periodic saving
    model.learn(
        total_timesteps=20000000,
        callback=[eval_callback, checkpoint_callback, success_callback],
        tb_log_name="train_"
    )

    # Save the trained model after further training
    model.save("./navigation_finaly")

if __name__ == '__main__':
    main()
