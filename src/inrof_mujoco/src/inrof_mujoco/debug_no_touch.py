import numpy as np
from inrof_mujoco.push_ball_rl import PushBallEnv

n_episodes = 50
returns = []
lengths = []
successes = []
final_distances = []
ball_displacements = []
min_robot_ball_distances = []

for ep in range(n_episodes):
    env = PushBallEnv()
    obs, info = env.reset(seed=ep)

    action = np.zeros(env.action_space.shape, dtype=env.action_space.dtype)
    ep_return = 0.0
    ep_len = 0
    terminated = False
    truncated = False

    start_ball_pos = obs[4:6].copy()
    min_robot_ball = float("inf")
    last_info = {}

    while not (terminated or truncated):
        obs, reward, terminated, truncated, info = env.step(action)
        ep_return += reward
        ep_len += 1
        last_info = info
        min_robot_ball = min(min_robot_ball, info["distance_robot_to_ball"])

    returns.append(ep_return)
    lengths.append(ep_len)
    final_distances.append(last_info["distance_to_target"])
    successes.append(last_info["distance_to_target"] < env.success_threshold)
    ball_displacements.append(np.linalg.norm(obs[4:6] - start_ball_pos))
    min_robot_ball_distances.append(min_robot_ball)

print(f"episodes: {n_episodes}")
print(f"no-touch mean_reward: {np.mean(returns):.3f} +/- {np.std(returns):.3f}")
print(f"mean_ep_length: {np.mean(lengths):.1f}")
print(f"success_rate: {np.mean(successes):.3f}")
print(f"mean_final_distance: {np.mean(final_distances):.3f}")
print(f"mean_ball_displacement: {np.mean(ball_displacements):.3f}")
print(f"mean_min_robot_ball_distance: {np.mean(min_robot_ball_distances):.3f}")