import gymnasium as gym
import numpy as np
from gymnasium.vector import SyncVectorEnv, AsyncVectorEnv

# Environment creation function
def make_env(env_id, seed):
    def _init():
        env = gym.make(env_id)
        env.reset(seed=seed)
        return env
    return _init

# Create 4 vectorized environments
num_envs = 10
env_id = "CartPole-v1"
env_fns = [make_env(env_id, seed=i) for i in range(num_envs)]

# Choose vectorized environment type
vec_env = SyncVectorEnv(env_fns)
# vec_env = AsyncVectorEnv(env_fns)  # Uncomment to try async version

# Interact with the vectorized environment
obs, _ = vec_env.reset()
print("Initial observations:", obs)

for _ in range(10):  # Run 5 steps
    actions = np.array([vec_env.single_action_space.sample() for _ in range(num_envs)], dtype=np.int64)
    obs, rewards, terminations, truncations, infos = vec_env.step(actions)
    print("Step:")
    print("  Actions:", actions)
    print("  Observations:", obs)
    print("  Rewards:", rewards)
    print("  Terminations:", terminations)

vec_env.close()
