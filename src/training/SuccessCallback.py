from stable_baselines3.common.callbacks import BaseCallback

class SuccessCallback(BaseCallback):
    """
    Custom callback to stop training when the model reaches 5(default) consecutive successful episodes.
    A successful episode is defined as having a cumulative reward of 100 or more.
    """
    def __init__(self, success_threshold=5, success_reward=100, verbose=1):
        super(SuccessCallback, self).__init__(verbose)
        self.success_threshold = success_threshold  # Stop when reaching 5 consecutive successes
        self.success_reward = success_reward  # Minimum reward to count as success
        self.consecutive_successes = 0  # Counter for consecutive successful episodes
        self.ep_reward = 0  # Track cumulative reward for the current episode

    def _on_step(self) -> bool:
        # Get current reward from the environment
        rewards = self.locals["rewards"]  # Access rewards from current step
        dones = self.locals["dones"]  # Access "done" flags for episode completion
        
        # Update cumulative episode reward
        self.ep_reward += sum(rewards)  # Sum in case of multiple environments

        # If an episode ends, check if it's a success
        if any(dones):  
            if self.ep_reward >= self.success_reward:
                self.consecutive_successes += 1  # Increase streak
                print(f"✅ Successful Episode! Total: {self.consecutive_successes}/{self.success_threshold}")
            else:
                self.consecutive_successes = 0  # Reset streak if episode fails
            
            # Reset episode reward for next episode
            self.ep_reward = 0  

        # Stop training if we reach the success threshold
        if self.consecutive_successes >= self.success_threshold:
            print("🎉 Training Stopped: Reached 5 consecutive successful episodes!")
            return False  # Stop training
        
        return True  # Continue training
