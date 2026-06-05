import gymnasium as gym
import numpy as np
import mujoco
from gymnasium import spaces
from pathlib import Path
import inrof_swerm

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XML_PATH = PROJECT_ROOT / "models" / "push_ball.xml"

class PushBallEnv(gym.Env):
    def __init__(self, xml_path=DEFAULT_XML_PATH):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)

        self.step_count = 0
        self.max_steps = 500
        self.frame_skip = 5
        self.success_threshold = 0.25
        self.leader_offset_scale = 2.0

        self.vx_id = self.model.actuator("robot_vx").id
        self.vy_id = self.model.actuator("robot_vy").id

        self.robot_x_qpos_id = None
        self.robot_y_qpos_id = None
        self.robot_x_qpos_id = None
        self.robot_y_qpos_id = None

        x_joint_id = self.model.joint("robot_x").id
        y_joint_id = self.model.joint("robot_y").id
        self.robot_x_qpos_id = self.model.jnt_qposadr[x_joint_id]
        self.robot_y_qpos_id = self.model.jnt_qposadr[y_joint_id]
        self.robot_x_qvel_id = self.model.jnt_dofadr[x_joint_id]
        self.robot_y_qvel_id = self.model.jnt_dofadr[y_joint_id]

        self.ball_x_qpos_id = self.model.jnt_qposadr[self.model.joint("ball_x").id]
        self.ball_y_qpos_id = self.model.jnt_qposadr[self.model.joint("ball_y").id]

        self._robot_pos_matrix = np.empty((2, 1), dtype=np.float64)
        self._robot_vel_matrix = np.empty((2, 1), dtype=np.float64)

        self._ball_pos = np.empty((2, 1), dtype=np.float64)

        self._leader_pos = np.zeros(2, dtype=np.float64)

        self.ball_target_pos = np.array([3.0, 3.0], dtype=np.float64)

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(1 * 4 + 2 + 2,),  # 4 for robot position and velocity, 2 for ball position, 2 for ball target position
            dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),  # 2 for velocity commands in x and y directions
            dtype=np.float32
        )

        
        boid_params = inrof_swerm.BoidPrams()
        boid_params.boid_num = 1
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
        
        boid_params = inrof_swerm.BoidPrams()
        boid_params.boid_num = 1
        boid_params.max_vel = 0.2
        boid_params.Ir = 100.0
        boid_params.Ir_min = 0.01
        boid_params.k_separation = 1.0
        boid_params.k_alignment = 1.1
        boid_params.k_gravity = 1.0
        boid_params.k_wall = 0.0

        self.follower_core = inrof_swerm.FollowerCore(boid_params, 0.5)

        self.step_count = 0

        self._update_robot_state()

        obs = self._get_obs()
        info = {}

        return obs, info
    
    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)

        self._update_robot_state()

        self.step_count += 1
        swerm_center = self._robot_pos_matrix.max(axis=1)
        self._leader_pos[:] = swerm_center + self.leader_offset_scale * action

        cmd_vels = self.follower_core.update_vels(
            self._robot_pos_matrix,
            self._robot_vel_matrix,
            self._leader_pos
        )

        self.data.ctrl[self.vx_id] = cmd_vels[0, 0]
        self.data.ctrl[self.vy_id] = cmd_vels[1, 0]

        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

        self._update_robot_state()

        obs = self._get_obs()
        reward, info = self._get_reward()

        terminated = bool(info["distance_to_target"] < self.success_threshold)
        truncated = bool(self.step_count >= self.max_steps)

        return obs, reward, terminated, truncated, info

    def _get_obs(self):
        values = []
        values.extend(self._robot_pos_matrix[:, 0])
        values.extend(self._robot_vel_matrix[:, 0])
        values.extend(self._ball_pos[:, 0])
        values.extend(self.ball_target_pos)

        return np.array(values, dtype=np.float32)
    
    def _get_reward(self):
        distance_to_target = np.linalg.norm(self._ball_pos[:, 0] - self.ball_target_pos)
        reward = -distance_to_target

        info = {
            "distance_to_target": distance_to_target
        }

        return reward, info
    
    def _update_robot_state(self):
        body = self.data.body("robot")
        self._robot_pos_matrix[0, 0] = body.xpos[0]
        self._robot_pos_matrix[1, 0] = body.xpos[1]

        self._robot_vel_matrix[0, 0] = self.data.qvel[self.robot_x_qvel_id]
        self._robot_vel_matrix[1, 0] = self.data.qvel[self.robot_y_qvel_id]

        ball_body = self.data.body("ball")
        self._ball_pos[0, 0] = ball_body.xpos[0]
        self._ball_pos[1, 0] = ball_body.xpos[1]

def run_env_check(xml_path=DEFAULT_XML_PATH):
    from stable_baselines3.common.env_checker import check_env

    env = PushBallEnv(xml_path)
    check_env(env, warn=True)

def main():
    import argparse

    parser = argparse.ArgumentParser(description="Run environment check for PushBallEnv.")
    parser.add_argument("--xml_path", type=str, default=str(DEFAULT_XML_PATH))
    args = parser.parse_args()
    run_env_check(args.xml_path)

if __name__ == "__main__":
    main()