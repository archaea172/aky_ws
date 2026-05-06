import gymnasium as gym
import numpy as np
import mujoco
from gymnasium import spaces
import inrof_swerm

class SwermEnv(gym.Env):
    def __init__(self, xml_path: str, robot_num: int = 5, ball_num: int = 3):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.robot_num = robot_num
        self.ball_num = ball_num
        self.max_steps = 2000
        self.step_count = 0
        self.frame_skip = 5
        self.target_position = np.array([5.0, 5.0])
        self.leader_offset_scale = 2.0
        self.success_threshold = 0.25
        self.time_penalty = 0.01
        self.success_bonus = 10.0
        self.vx_ids = np.array([self.model.actuator(f"robot_{i}_vx").id for i in range(robot_num)])
        self.vy_ids = np.array([self.model.actuator(f"robot_{i}_vy").id for i in range(robot_num)])
        self.robot_x_qpos_ids = np.empty(robot_num, dtype=np.intp)
        self.robot_y_qpos_ids = np.empty(robot_num, dtype=np.intp)
        self.robot_x_qvel_ids = np.empty(robot_num, dtype=np.intp)
        self.robot_y_qvel_ids = np.empty(robot_num, dtype=np.intp)
        for i in range(robot_num):
            x_joint_id = self.model.joint(f"robot_{i}_x").id
            y_joint_id = self.model.joint(f"robot_{i}_y").id
            self.robot_x_qpos_ids[i] = self.model.jnt_qposadr[x_joint_id]
            self.robot_y_qpos_ids[i] = self.model.jnt_qposadr[y_joint_id]
            self.robot_x_qvel_ids[i] = self.model.jnt_dofadr[x_joint_id]
            self.robot_y_qvel_ids[i] = self.model.jnt_dofadr[y_joint_id]
        self._robot_pos_matrix = np.empty((2, robot_num), dtype=np.float64)
        self._robot_vel_matrix = np.empty((2, robot_num), dtype=np.float64)
        self._leader_pos = np.empty(2, dtype=np.float64)
        self._balls_pos = np.empty((2, ball_num), dtype=np.float64)
        self._prev_ball_distance_sum = 0.0

        obs_dim = robot_num * 4 + ball_num * 4 + 2  # robot pos(2) + robot vel(2) + ball pos(2) + ball vel(2) + target pos(2)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self.boid_params = inrof_swerm.BoidPrams()
        self.boid_params.boid_num = robot_num
        self.boid_params.max_vel = 0.2
        self.boid_params.Ir = 100.0
        self.boid_params.Ir_min = 0.01
        self.boid_params.k_separation = 10.0
        self.boid_params.k_alignment = 1.1
        self.boid_params.k_gravity = 0.5
        self.k_follow = 0.5
        self.follower_core = inrof_swerm.FollowerCore(self.boid_params, self.k_follow)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.step_count = 0
        self._prev_ball_distance_sum = self._compute_ball_distance_sum()
        return self._get_obs(), {}

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self._robot_pos_matrix[0, :] = self.data.qpos[self.robot_x_qpos_ids]
        self._robot_pos_matrix[1, :] = self.data.qpos[self.robot_y_qpos_ids]
        self._robot_vel_matrix[0, :] = self.data.qvel[self.robot_x_qvel_ids]
        self._robot_vel_matrix[1, :] = self.data.qvel[self.robot_y_qvel_ids]
        swarm_center = self._robot_pos_matrix.mean(axis=1)
        self._leader_pos[:] = swarm_center + self.leader_offset_scale * action

        cmd_vels = self.follower_core.update_vels(
            self._robot_pos_matrix,
            self._robot_vel_matrix,
            self._leader_pos,
        )
        self.data.ctrl[self.vx_ids] = cmd_vels[0, :]
        self.data.ctrl[self.vy_ids] = cmd_vels[1, :]

        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1

        obs = self._get_obs()
        balls_pos = self._get_ball_positions()
        reward, ball_distance_sum, success = self._get_reward(balls_pos)
        self._prev_ball_distance_sum = ball_distance_sum
        terminated = success
        truncated = self.step_count >= self.max_steps

        return obs, reward, terminated, truncated, {"ball_distance_sum": ball_distance_sum, "success": success}

    def _get_obs(self):
        values = []
        for i in range(self.robot_num):
            body = self.data.body(f"robot_{i}")
            values.extend(body.xpos[:2])
            values.extend(body.cvel[:2])

        for i in range(self.ball_num):
            body = self.data.body(f"ball_{i}")
            values.extend(body.xpos[:2])
            values.extend(body.cvel[:2])

        values.extend(self.target_position)

        return np.array(values, dtype=np.float32)
    
    def _get_reward(self, balls_pos):
        diff = balls_pos - self.target_position[:, None]
        distances = np.sqrt(np.sum(diff * diff, axis=0))
        ball_distance_sum = float(distances.sum())
        progress_reward = self._prev_ball_distance_sum - ball_distance_sum
        proximity_reward = -0.1 * ball_distance_sum
        success = bool(np.all(distances < self.success_threshold))
        reward = progress_reward + proximity_reward - self.time_penalty
        if success:
            reward += self.success_bonus
        return reward, ball_distance_sum, success

    def _get_ball_positions(self):
        for i in range(self.ball_num):
            body = self.data.body(f"ball_{i}")
            self._balls_pos[0, i] = body.xpos[0]
            self._balls_pos[1, i] = body.xpos[1]
        return self._balls_pos

    def _compute_ball_distance_sum(self):
        balls_pos = self._get_ball_positions()
        diff = balls_pos - self.target_position[:, None]
        return float(np.sqrt(np.sum(diff * diff, axis=0)).sum())

from stable_baselines3 import PPO
env = SwermEnv(xml_path="models/boid.xml", robot_num=5, ball_num=3)
model = PPO("MlpPolicy", env, verbose=1, device="cpu")
model.learn(total_timesteps=5_000_000)
model.save("ppo_swerm")
