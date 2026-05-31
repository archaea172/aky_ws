import gymnasium as gym
import numpy as np
import mujoco
from mujoco import mjx
from gymnasium import spaces
from pathlib import Path
import jax
import jax.numpy as jnp
from functools import partial

from inrof_mujoco.follower_core_mjx import FollowerCoreMjx
from inrof_mujoco.follower_core_mjx import FollowerParams
from inrof_mujoco.follower_core_mjx import update_vels_jit

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XML_PATH = PROJECT_ROOT / "models" / "point.xml"

@jax.jit
def mjx_forward(model, data):
    return mjx.forward(model, data)


@partial(jax.jit, static_argnames=("frame_skip",))
def mjx_policy_step(
    mjx_model,
    data,
    action,
    target_position,
    prev_robot_vel,
    params,
    robot_body_ids,
    robot_x_qvel_ids,
    robot_y_qvel_ids,
    vx_ids,
    vy_ids,
    leader_offset_scale,
    frame_skip
):
    robot_pos = data.xpos[robot_body_ids, :2].T
    robot_vel = jnp.stack([
        data.qvel[robot_x_qvel_ids],
        data.qvel[robot_y_qvel_ids],
    ])

    swarm_center = jnp.mean(robot_pos, axis=1)
    leader_pos = swarm_center + leader_offset_scale * action

    cmd_vels = update_vels_jit(
        robot_pos,
        robot_vel,
        leader_pos,
        params,
    )

    ctrl = data.ctrl
    ctrl = ctrl.at[vx_ids].set(cmd_vels[0, :].astype(ctrl.dtype))
    ctrl = ctrl.at[vy_ids].set(cmd_vels[1, :].astype(ctrl.dtype))
    data = data.replace(ctrl=ctrl)

    def step_once(carry, _):
        return mjx.step(mjx_model, carry), None

    data, _ = jax.lax.scan(step_once, data, xs=None, length=frame_skip)

    next_robot_pos = data.xpos[robot_body_ids, :2].T
    next_robot_vel = jnp.stack([
        data.qvel[robot_x_qvel_ids],
        data.qvel[robot_y_qvel_ids],
    ])

    obs = _get_obs_jax(target_position, next_robot_pos, next_robot_vel)
    reward, reward_info = _get_reward_jax(
        target_position,
        next_robot_pos,
        next_robot_vel,
        prev_robot_vel,
    )

    return data, obs, reward, reward_info, next_robot_vel

def _get_obs_jax(target_position, robot_pos, robot_vel):
    # target_position: (2,)
    # robot_pos, robot_vel: (2, robot_num)
    robot_state = jnp.concatenate([
        robot_pos.T,
        robot_vel.T,
    ], axis=1).reshape(-1)

    return jnp.concatenate([target_position, robot_state]).astype(jnp.float32)

def _get_reward_jax(target_position, robot_pos, robot_vel, prev_robot_vel):
    diff = robot_pos - target_position[:, None]
    robot_dists = jnp.sqrt(jnp.sum(diff * diff, axis=0))
    target_error = jnp.mean(robot_dists)

    vel_delta = robot_vel - prev_robot_vel
    accel_penalty = jnp.mean(jnp.sum(vel_delta * vel_delta, axis=0))

    speed = jnp.sqrt(jnp.sum(robot_vel * robot_vel, axis=0))
    mean_speed = jnp.mean(speed)

    center = jnp.mean(robot_pos, axis=1)
    offsets = robot_pos - center[:, None]
    spread = jnp.mean(jnp.sqrt(jnp.sum(offsets * offsets, axis=0)))

    reward = (
        -1.0 * target_error
        -0.2 * accel_penalty
        -0.1 * mean_speed
        -0.5 * spread
    )

    reward_info = {
        "target_error": target_error,
        "accel_penalty": accel_penalty,
        "mean_speed": mean_speed,
        "spread": spread,
    }

    return reward, reward_info

class MJXPointEnv(gym.Env):
    def __init__(self, xml_path=DEFAULT_XML_PATH, robot_num=5):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.mjx_model = mjx.put_model(self.model)
        self.data = mjx.make_data(self.mjx_model)
        
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
        self.robot_body_ids = np.array([
            self.model.body(f"robot_{i}").id
            for i in range(robot_num)
        ])
        self.vx_ids_jax = jnp.asarray(self.vx_ids, dtype=jnp.int32)
        self.vy_ids_jax = jnp.asarray(self.vy_ids, dtype=jnp.int32)
        self.robot_body_ids_jax = jnp.asarray(self.robot_body_ids, dtype=jnp.int32)

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

        self.robot_x_qvel_ids_jax = jnp.asarray(self.robot_x_qvel_ids, dtype=jnp.int32)
        self.robot_y_qvel_ids_jax = jnp.asarray(self.robot_y_qvel_ids, dtype=jnp.int32)
            
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

        self.follower_params = FollowerParams(
            0.2,
            1.0,
            1.1,
            1.0,
            0.0,
            0.5,
            0,
            100,
            0.01,
        )

        self.follower_core = FollowerCoreMjx(self.follower_params)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.data = mjx.make_data(self.mjx_model)

        self.target_position = self.np_random.uniform(
            low=np.array([-3.0, -3.0]),
            high=np.array([3.5, 3.5]),
        ).astype(np.float64)

        target_body_id = self.model.body("target_body").id
        self.model.body_pos[target_body_id, 0:2] = self.target_position
        
        self.follower_params = FollowerParams(
            0.2,
            self.np_random.uniform(
                low=0.1,
                high=3.0,
            ),
            1.1,
            1.0,
            0.0,
            0.5,
            0,
            100,
            0.01,
        )

        self.follower_core = FollowerCoreMjx(self.follower_params)
        
        min_distance_sq = 0.12 ** 2
        ROBOT_XY = []
        while len(ROBOT_XY) < self.robot_num:
            candidate = self.np_random.uniform(
                low=np.array([-1.0, -1.0]),
                high=np.array([1.0, 1.0]),
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

        qpos = np.asarray(jax.device_get(self.data.qpos)).copy()
        qvel = np.asarray(jax.device_get(self.data.qvel)).copy()
        ctrl = np.asarray(jax.device_get(self.data.ctrl)).copy()

        qpos[self.robot_x_qpos_ids] = qpos_xy[:, 0]
        qpos[self.robot_y_qpos_ids] = qpos_xy[:, 1]
        qvel[self.robot_x_qvel_ids] = 0.0
        qvel[self.robot_y_qvel_ids] = 0.0
        ctrl[:] = 0.0

        self.data = self.data.replace(
            qpos=jnp.asarray(qpos, dtype=self.data.qpos.dtype),
            qvel=jnp.asarray(qvel, dtype=self.data.qvel.dtype),
            ctrl=jnp.asarray(ctrl, dtype=self.data.ctrl.dtype),
        )

        self.data = mjx_forward(self.mjx_model, self.data)

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
        action_jax = jnp.asarray(action, dtype=self.data.ctrl.dtype)
        self.data, obs, reward, reward_info, next_robot_vel = mjx_policy_step(
            self.mjx_model,
            self.data,
            action_jax,
            jnp.asarray(self.target_position, dtype=self.data.qpos.dtype),
            jnp.asarray(self._prev_robot_vel_matrix, dtype=self.data.qpos.dtype),
            self.follower_params,
            self.robot_body_ids_jax,
            self.robot_x_qvel_ids_jax,
            self.robot_y_qvel_ids_jax,
            self.vx_ids_jax,
            self.vy_ids_jax,
            self.leader_offset_scale,
            self.frame_skip,
        )

        self.step_count += 1

        obs_np = np.asarray(jax.device_get(obs), dtype=np.float32)
        reward_float = float(jax.device_get(reward))

        info_values = jax.device_get(reward_info)
        info = {
            # "target_error": float(info_values["target_error"]),
            # "accel_penalty": float(info_values["accel_penalty"]),
            # "mean_speed": float(info_values["mean_speed"]),
            # "spread": float(info_values["spread"]),
            "mean_dist": float(info_values["target_error"]),
        }

        self._prev_robot_vel_matrix[:, :] = np.asarray(
            jax.device_get(next_robot_vel),
            dtype=np.float64,
        )
        terminated = info["target_error"] < self.success_threshold
        truncated = self.step_count >= self.max_steps

        return obs_np, reward_float, terminated, truncated, info
    
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
        robot_xpos = jax.device_get(self.data.xpos[self.robot_body_ids_jax, :2])
        self._robot_pos_matrix[:, :] = np.asarray(robot_xpos, dtype=np.float64).T

        qvel = np.asarray(jax.device_get(self.data.qvel), dtype=np.float64)
        self._robot_vel_matrix[0, :] = qvel[self.robot_x_qvel_ids]
        self._robot_vel_matrix[1, :] = qvel[self.robot_y_qvel_ids]

        

def run_env_check(xml_path=DEFAULT_XML_PATH, robot_num=5):
    from stable_baselines3.common.env_checker import check_env

    env = MJXPointEnv(xml_path=xml_path, robot_num=robot_num)
    check_env(env, warn=True)


def train(total_timesteps=10_000, xml_path=DEFAULT_XML_PATH, robot_num=5, save_path=None):
    from stable_baselines3 import PPO

    env = MJXPointEnv(xml_path=xml_path, robot_num=robot_num)
    model = PPO("MlpPolicy", env, verbose=1, device="cpu", n_steps=512, batch_size=128)
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
    parser.add_argument("--save-path", type=Path, default=PROJECT_ROOT / "ppo_point_gpu")
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
