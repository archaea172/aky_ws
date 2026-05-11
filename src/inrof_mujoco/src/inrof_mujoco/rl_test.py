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
        self.max_steps = 10000
        self.step_count = 0
        self.frame_skip = 5
        self.target_position = np.array([5.0, 5.0])
        self.leader_offset_scale = 2.0
        self.success_threshold = 0.25
        self.time_penalty = 0.01
        self.success_bonus = 20.0
        self.partial_success_bonus = 3.0
        self.contact_threshold = 0.18
        self.ball_progress_scale = 8.0
        self.approach_progress_scale = 4.0
        self.target_speed_scale = 1.0
        self.contact_bonus_scale = 0.25
        self.distance_penalty_scale = 0.01
        self.vx_ids = np.array([self.model.actuator(f"robot_{i}_vx").id for i in range(robot_num)])
        self.vy_ids = np.array([self.model.actuator(f"robot_{i}_vy").id for i in range(robot_num)])
        self.robot_x_qpos_ids = np.empty(robot_num, dtype=np.intp)
        self.robot_y_qpos_ids = np.empty(robot_num, dtype=np.intp)
        self.robot_x_qvel_ids = np.empty(robot_num, dtype=np.intp)
        self.robot_y_qvel_ids = np.empty(robot_num, dtype=np.intp)
        self.ball_x_qvel_ids = np.empty(ball_num, dtype=np.intp)
        self.ball_y_qvel_ids = np.empty(ball_num, dtype=np.intp)
        for i in range(robot_num):
            x_joint_id = self.model.joint(f"robot_{i}_x").id
            y_joint_id = self.model.joint(f"robot_{i}_y").id
            self.robot_x_qpos_ids[i] = self.model.jnt_qposadr[x_joint_id]
            self.robot_y_qpos_ids[i] = self.model.jnt_qposadr[y_joint_id]
            self.robot_x_qvel_ids[i] = self.model.jnt_dofadr[x_joint_id]
            self.robot_y_qvel_ids[i] = self.model.jnt_dofadr[y_joint_id]
        for i in range(ball_num):
            x_joint_id = self.model.joint(f"ball_{i}_x").id
            y_joint_id = self.model.joint(f"ball_{i}_y").id
            self.ball_x_qvel_ids[i] = self.model.jnt_dofadr[x_joint_id]
            self.ball_y_qvel_ids[i] = self.model.jnt_dofadr[y_joint_id]
        self._robot_pos_matrix = np.empty((2, robot_num), dtype=np.float64)
        self._robot_vel_matrix = np.empty((2, robot_num), dtype=np.float64)
        self._leader_pos = np.empty(2, dtype=np.float64)
        self._balls_pos = np.empty((2, ball_num), dtype=np.float64)
        self._balls_vel_matrix = np.empty((2, ball_num), dtype=np.float64)
        self._prev_ball_distance_sum = 0.0
        self._prev_robot_ball_distance_sum = 0.0
        self._ball_reached_target = np.zeros(ball_num, dtype=bool)

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
        self._update_robot_state()
        self._update_ball_state()
        self._prev_ball_distance_sum = self._compute_ball_distance_sum()
        self._prev_robot_ball_distance_sum = self._compute_robot_ball_distance_sum()
        self._ball_reached_target[:] = self._compute_ball_target_distances() < self.success_threshold
        return self._get_obs(), {}

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self._update_robot_state()
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

        self._update_robot_state()
        self._update_ball_state()
        obs = self._get_obs()
        reward, ball_distance_sum, robot_ball_distance_sum, reached_balls, success = self._get_reward()
        self._prev_ball_distance_sum = ball_distance_sum
        self._prev_robot_ball_distance_sum = robot_ball_distance_sum
        terminated = success
        truncated = self.step_count >= self.max_steps

        return obs, reward, terminated, truncated, {
            "ball_distance_sum": ball_distance_sum,
            "robot_ball_distance_sum": robot_ball_distance_sum,
            "reached_balls": reached_balls,
            "success": success,
        }

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
    
    def _get_reward(self):
        target_diff = self.target_position[:, None] - self._balls_pos
        ball_target_distances = np.sqrt(np.sum(target_diff * target_diff, axis=0))
        ball_distance_sum = float(ball_target_distances.sum())
        robot_ball_diff = self._robot_pos_matrix[:, :, None] - self._balls_pos[:, None, :]
        robot_ball_distances = np.sqrt(np.sum(robot_ball_diff * robot_ball_diff, axis=0))
        closest_robot_distances = robot_ball_distances.min(axis=0)
        robot_ball_distance_sum = float(closest_robot_distances.sum())

        ball_progress_reward = self.ball_progress_scale * (self._prev_ball_distance_sum - ball_distance_sum)
        approach_reward = self.approach_progress_scale * (
            self._prev_robot_ball_distance_sum - robot_ball_distance_sum
        )
        target_directions = target_diff / np.maximum(ball_target_distances[None, :], 1e-6)
        target_speed_reward = self.target_speed_scale * float(
            np.clip(np.sum(self._balls_vel_matrix * target_directions, axis=0), 0.0, None).sum()
        )
        contact_bonus = self.contact_bonus_scale * float(
            np.count_nonzero(closest_robot_distances < self.contact_threshold)
        )
        reached_target = ball_target_distances < self.success_threshold
        newly_reached = reached_target & ~self._ball_reached_target
        partial_success_bonus = self.partial_success_bonus * float(np.count_nonzero(newly_reached))
        self._ball_reached_target |= reached_target
        success = bool(np.all(reached_target))

        reward = (
            ball_progress_reward
            + approach_reward
            + target_speed_reward
            + contact_bonus
            + partial_success_bonus
            - self.distance_penalty_scale * ball_distance_sum
            - self.time_penalty
        )
        if success:
            reward += self.success_bonus
        return reward, ball_distance_sum, robot_ball_distance_sum, int(np.count_nonzero(reached_target)), success

    def _update_robot_state(self):
        self._robot_pos_matrix[0, :] = self.data.qpos[self.robot_x_qpos_ids]
        self._robot_pos_matrix[1, :] = self.data.qpos[self.robot_y_qpos_ids]
        self._robot_vel_matrix[0, :] = self.data.qvel[self.robot_x_qvel_ids]
        self._robot_vel_matrix[1, :] = self.data.qvel[self.robot_y_qvel_ids]

    def _update_ball_state(self):
        for i in range(self.ball_num):
            body = self.data.body(f"ball_{i}")
            self._balls_pos[0, i] = body.xpos[0]
            self._balls_pos[1, i] = body.xpos[1]
        self._balls_vel_matrix[0, :] = self.data.qvel[self.ball_x_qvel_ids]
        self._balls_vel_matrix[1, :] = self.data.qvel[self.ball_y_qvel_ids]

    def _compute_ball_distance_sum(self):
        diff = self._balls_pos - self.target_position[:, None]
        return float(np.sqrt(np.sum(diff * diff, axis=0)).sum())

    def _compute_robot_ball_distance_sum(self):
        robot_ball_diff = self._robot_pos_matrix[:, :, None] - self._balls_pos[:, None, :]
        robot_ball_distances = np.sqrt(np.sum(robot_ball_diff * robot_ball_diff, axis=0))
        return float(robot_ball_distances.min(axis=0).sum())

    def _compute_ball_target_distances(self):
        diff = self._balls_pos - self.target_position[:, None]
        return np.sqrt(np.sum(diff * diff, axis=0))

from stable_baselines3 import PPO
env = SwermEnv(xml_path="models/boid.xml", robot_num=5, ball_num=3)
model = PPO("MlpPolicy", env, verbose=1, device="cpu")
model.learn(total_timesteps=50_000_000)
model.save("ppo_swerm")
