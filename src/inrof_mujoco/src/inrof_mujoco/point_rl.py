import gymnasium as gym
import numpy as np
import mujoco
from gymnasium import spaces
import inrof_swerm
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XML_PATH = PROJECT_ROOT / "models" / "point.xml"


class PointEnv(gym.Env):
    def __init__(self, xml_path=DEFAULT_XML_PATH, robot_num=5):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)

        self.robot_num = robot_num
        self.step_count = 0
        self.max_steps = 600
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
        self._prev_robot_vel_matrix = np.zeros((2, self.robot_num), dtype=np.float64)

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

        self.target_position = self.np_random.uniform(
            low=np.array([-3.0, -3.0]),
            high=np.array([3.5, 3.5]),
        ).astype(np.float64)

        target_body_id = self.model.body("target_body").id
        self.model.body_pos[target_body_id, 0:2] = self.target_position
        mujoco.mj_forward(self.model, self.data)
        
        boid_params = inrof_swerm.BoidPrams()
        boid_params.boid_num = self.robot_num
        boid_params.max_vel = 0.2
        boid_params.Ir = 100.0
        boid_params.Ir_min = 0.01
        boid_params.k_separation = self.np_random.uniform(
            low=0.1,
            high=3.0,
        )
        boid_params.k_alignment = 1.1
        boid_params.k_gravity = 1.0
        boid_params.k_wall = 0.0

        self.follower_core = inrof_swerm.FollowerCore(boid_params, 0.5)
        
        min_distance_sq = 0.12 ** 2
        ROBOT_XY = []
        while len(ROBOT_XY) < self.robot_num:
            candidate = self.np_random.uniform(
                low=np.array([0.1, 0.1]),
                high=np.array([0.8, 0.4]),
            ).astype(np.float64)
            if all(
                (candidate[0] - x) ** 2 + (candidate[1] - y) ** 2 >= min_distance_sq
                for x, y in ROBOT_XY
            ):
                ROBOT_XY.append(candidate)

        robot_xy = np.asarray(ROBOT_XY, dtype=np.float64)
        base_xy = np.array([
            self.model.body(f"robot_{i}").pos[:2]
            for i in range(self.robot_num)
        ])
        qpos_xy = robot_xy - base_xy
        self.data.qpos[self.robot_x_qpos_ids] = qpos_xy[:, 0]
        self.data.qpos[self.robot_y_qpos_ids] = qpos_xy[:, 1]

        self.data.qvel[self.robot_x_qvel_ids] = 0.0
        self.data.qvel[self.robot_y_qvel_ids] = 0.0
        self.data.ctrl[:] = 0.0

        mujoco.mj_forward(self.model, self.data)

        self.step_count = 0
        self._update_robot_state()
        self._prev_robot_vel_matrix[:] = self._robot_vel_matrix

        obs = self._get_obs()
        info = {
            "target_position": self.target_position.copy(),
        }

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
        reward, info_1 = self._get_reward()
        self._prev_robot_vel_matrix[:] = self._robot_vel_matrix

        mean_dist = info_1["target_error"]
        terminated = mean_dist < self.success_threshold
        truncated = self.step_count >= self.max_steps
        info = info_1 | {"mean_dist": mean_dist}

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
        robot_dists = np.sqrt(np.sum(diff * diff, axis=0))
        target_error = float(np.mean(robot_dists))

        vel_delta = self._robot_vel_matrix - self._prev_robot_vel_matrix
        accel_penalty = np.mean(np.sum(vel_delta * vel_delta, axis=0))

        speed = np.sqrt(np.sum(self._robot_vel_matrix * self._robot_vel_matrix, axis=0))
        mean_speed = float(np.mean(speed))

        center = self._robot_pos_matrix.mean(axis=1)
        offsets = self._robot_pos_matrix - center[:, None]
        spread = float(np.mean(np.sqrt(np.sum(offsets * offsets, axis=0))))

        reward = 0.0

        reward -= 1.0 * target_error
        reward -= 0.2 * accel_penalty
        reward -= 0.1 * mean_speed
        reward -= 0.5 * spread

        info = {
            "target_error": target_error,
            "accel_penalty": accel_penalty,
            "mean_speed": mean_speed,
            "spread": spread,
        }

        return reward, info
    
    def _update_robot_state(self):
        for i in range(self.robot_num):
            body = self.data.body(f"robot_{i}")
            self._robot_pos_matrix[0, i] = body.xpos[0]
            self._robot_pos_matrix[1, i] = body.xpos[1]

        self._robot_vel_matrix[0, :] = self.data.qvel[self.robot_x_qvel_ids]
        self._robot_vel_matrix[1, :] = self.data.qvel[self.robot_y_qvel_ids]


def run_env_check(xml_path=DEFAULT_XML_PATH, robot_num=5):
    from stable_baselines3.common.env_checker import check_env

    env = PointEnv(xml_path=xml_path, robot_num=robot_num)
    check_env(env, warn=True)


def train(total_timesteps=10_000, xml_path=DEFAULT_XML_PATH, robot_num=5, save_path=None):
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv

    env = make_vec_env(
        lambda: PointEnv(xml_path=xml_path, robot_num=robot_num),
        n_envs=8,
        vec_env_cls=SubprocVecEnv,
    )
    model = PPO("MlpPolicy", env, verbose=1, device="cpu", n_steps=128, batch_size=256)
    model.learn(total_timesteps=total_timesteps)

    if save_path is not None:
        model.save(str(save_path))

    return model


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Check and train the point-goal MuJoCo RL environment.")
    parser.add_argument("command", choices=("check", "train", "check-train"), nargs="?", default="check-train")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML_PATH)
    parser.add_argument("--robot-num", type=int, default=5)
    parser.add_argument("--timesteps", type=int, default=10_000)
    parser.add_argument("--save-path", type=Path, default=PROJECT_ROOT / "ppo_point")
    args = parser.parse_args()

    if args.command in ("check", "check-train"):
        run_env_check(xml_path=args.xml, robot_num=args.robot_num)
        print("check_env passed")

    if args.command in ("train", "check-train"):
        train(
            total_timesteps=args.timesteps,
            xml_path=args.xml,
            robot_num=args.robot_num,
            save_path=args.save_path,
        )


if __name__ == "__main__":
    main()
