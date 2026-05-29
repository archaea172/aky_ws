import gymnasium as gym
import numpy as np
import mujoco
from gymnasium import spaces
import inrof_swerm

class PointEnv(gym.Env):
    def __init__(self, xml_path, robot_num=5):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.robot_num = robot_num
        self.step_count = 0
        self.max_steps = 500
        self.frame_skip = 5
        self.target_position = np.array([3.0, 3.0], dtype=np.float64)
        self.success_threshold = 0.25
        self.leader_offset_scale = 2.0

        self.vx_ids = np.array([
            self.model.actuator(f"robot_{i}_vx").id
            for i in range(robot_num)
        ])
        self.vy_ids = np.array([
            self.model.actuator(f"robot_{i}_vy").id
            for i in range(robot_num)
        ])

        self.robot_x_qvel_ids = np.empty(robot_num, dtype=np.intp)
        self.robot_y_qvel_ids = np.empty(robot_num, dtype=np.intp)
        
        for i in range(robot_num):
            x_joint_id = self.model.joint(f"robot_{i}_x").id
            y_joint_id = self.model.joint(f"robot_{i}_y").id
            self.robot_x_qvel_ids[i] = self.model.jnt_dofadr[x_joint_id]
            self.robot_y_qvel_ids[i] = self.model.jnt_dofadr[y_joint_id]
            
        self._robot_pos_matrix = np.empty((2, robot_num), dtype=np.float64)
        self._robot_vel_matrix = np.empty((2, robot_num), dtype=np.float64)
        self._leader_pos = np.empty(2, dtype=np.float64)

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.robot_num * 4 + 2,),
            dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )
        
        boid_params = inrof_swerm.BoidPrams()
        boid_params.boid_num = robot_num
        boid_params.max_vel = 0.2
        boid_params.Ir = 100.0
        boid_params.Ir_min = 0.01
        boid_params.k_separation = 1.0
        boid_params.k_alignment = 1.1
        boid_params.k_gravity = 1.0
        boid_params.k_wall = 0.0

        self.follower_core = inrof_swerm.FollowerCore(boid_params, 0.5)
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

        self.step_count = 0
        self._update_robot_state()

        obs = self._get_obs()
        info = {}

        return obs, info
    
    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self._update_robot_state()

        self.step_count += 1
        swerm_center = self._robot_pos_matrix.mean(axis=1)
        self._leader_pos[:] = swerm_center + self.leader_offset_scale * action

        cmd_vels = self.follower_core.update_vels(
            self._robot_pos_matrix,
            self._robot_vel_matrix,
            self._leader_pos
        )

        self.data.ctrl[self.vx_ids] = cmd_vels[0, :]
        self.data.ctrl[self.vy_ids] = cmd_vels[1, :]

        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

        self._update_robot_state()

        obs = self._get_obs()
        reward, mean_dist = self._get_reward()

        terminated = mean_dist < self.success_threshold
        truncated = self.step_count >= self.max_steps
        info = {
            "mean_dist": mean_dist,
            "target_position": self.target_position.copy(),
        }

        return obs, reward, terminated, truncated, info
    
    def _get_obs(self):
        values = []
        values.extend(self.target_position)

        for i in range(self.robot_num):
            values.extend(self._robot_pos_matrix[:, i])
            values.extend(self._robot_vel_matrix[:, i])

        return np.array(values, dtype=np.float32)
    
    def _get_reward(self):
        diff = self._robot_pos_matrix - self.target_position[:, None]
        dist_sq = np.sum(diff * diff, axis=0)

        reward = float(np.mean(1.0 / (1.0 + dist_sq)))

        mean_dist = float(np.mean(np.sqrt(dist_sq)))

        if mean_dist < self.success_threshold:
            reward += 10.0

        reward -= 0.01

        return reward, mean_dist
    
    def _update_robot_state(self):
        for i in range(self.robot_num):
            body = self.data.body(f"robot_{i}")
            self._robot_pos_matrix[0, i] = body.xpos[0]
            self._robot_pos_matrix[1, i] = body.xpos[1]

        self._robot_vel_matrix[0, :] = self.data.qvel[self.robot_x_qvel_ids]
        self._robot_vel_matrix[1, :] = self.data.qvel[self.robot_y_qvel_ids]
