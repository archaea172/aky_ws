import gymnasium as gym
import math
import numpy as np
import mujoco
from gymnasium import spaces
from pathlib import Path
import inrof_swerm

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XML_PATH = PROJECT_ROOT / "models" / "push_ball.xml"

class PushBallEnv(gym.Env):
    def __init__(self, stage, xml_path=DEFAULT_XML_PATH):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)

        self.step_count = 0
        self.max_steps = 500
        self.frame_skip = 5
        self.stage = stage
        if stage == 1:
            self.success_threshold = 0.25
        elif stage >= 2:
            self.success_threshold = 0.15
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
        self.ball_x_qvel_id = self.model.jnt_dofadr[self.model.joint("ball_x").id]
        self.ball_y_qvel_id = self.model.jnt_dofadr[self.model.joint("ball_y").id]

        self._robot_pos_matrix = np.empty((2, 1), dtype=np.float64)
        self._robot_vel_matrix = np.empty((2, 1), dtype=np.float64)
        self._pre_robot_pos_matrix = np.empty((2, 1), dtype=np.float64)

        self._ball_pos = np.empty((2, 1), dtype=np.float64)
        self._pre_ball_pos = np.empty((2, 1), dtype=np.float64)
        self._ball_vel_matrix = np.empty((2, 1), dtype=np.float64)

        self._leader_pos = np.zeros(2, dtype=np.float64)

        self.ball_target_pos = np.array([1.5, 1.5], dtype=np.float64)

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(1 * 4 + 2 * 2 + 2,),  # 4 for robot position and velocity, 2 for ball position and veolocity, 2 for ball velocity, 2 for ball target position
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
        if self.stage >= 3:
            robot_pos = self.np_random.uniform(
                low=np.array([-0.5, -0.5]),
                high=np.array([0.5, 0.5]),
            ).astype(np.float64)
            base_xy = np.array([
                self.model.body("robot").pos[:2]
            ])
            qpos_xy = robot_pos - base_xy
            self.data.qpos[self.robot_x_qpos_id] = qpos_xy[:, 0]
            self.data.qpos[self.robot_y_qpos_id] = qpos_xy[:, 1]
            self.data.qvel[self.robot_x_qvel_id] = 0.0
            self.data.qvel[self.robot_y_qvel_id] = 0.0
            mujoco.mj_forward(self.model, self.data)
        if self.stage >=4:
            self.ball_target_pos = self.np_random.uniform(
                low=np.array([1.0, 1.0]),
                high=np.array([2.0, 2.0]),
            ).astype(np.float64)

            target_body_id = self.model.body("target_body").id
            self.model.body_pos[target_body_id, 0:2] = self.ball_target_pos

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
        self._pre_ball_pos[:] = self._ball_pos
        self._pre_robot_pos_matrix[:] = self._robot_pos_matrix
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
        values.extend(self._ball_vel_matrix[:, 0])
        values.extend(self.ball_target_pos)

        return np.array(values, dtype=np.float32)
    
    def _get_reward(self):
        ball_x = self._ball_pos[0, 0]
        ball_y = self._ball_pos[1, 0]
        target_x = self.ball_target_pos[0]
        target_y = self.ball_target_pos[1]
        ball_to_target_x = target_x - ball_x
        ball_to_target_y = target_y - ball_y
        distance_to_target = math.hypot(ball_to_target_x, ball_to_target_y)
        
        robot_to_ball_x = ball_x - self._robot_pos_matrix[0, 0]
        robot_to_ball_y = ball_y - self._robot_pos_matrix[1, 0]
        distance_robot_to_ball = math.hypot(robot_to_ball_x, robot_to_ball_y)
        prev_distance_robot_to_ball = math.hypot(
            self._pre_ball_pos[0, 0] - self._pre_robot_pos_matrix[0, 0],
            self._pre_ball_pos[1, 0] - self._pre_robot_pos_matrix[1, 0],
        )
        progress_robot_to_ball = prev_distance_robot_to_ball - distance_robot_to_ball

        prev_distance = math.hypot(
            target_x - self._pre_ball_pos[0, 0],
            target_y - self._pre_ball_pos[1, 0],
        )
        progress = prev_distance - distance_to_target

        target_dir = np.array([ball_to_target_x, ball_to_target_y], dtype=np.float64)
        target_norm = np.linalg.norm(target_dir)
        if target_norm > 1e-6:
            target_dir /= target_norm
        else:
            target_dir[:] = 0.0

        force = self._get_robot_ball_contact_force()

        force_to_target = np.dot(force[:2], target_dir)
        force_threshold = 0.01
        force_reward = np.clip(force_to_target / force_threshold, -1.0, 1.0)

        reward = 0.0
        if distance_to_target < self.success_threshold:
            reward += 100.0

        is_contact = self._is_robot_ball_contact()
        if is_contact:
            reward += 0.1
            reward -= 0.3 * force_reward
            reward += 10.0 * progress
        else:
            reward += np.exp(-distance_robot_to_ball) - 1.0



        info = {
            "distance_to_target": distance_to_target,
            "distance_robot_to_ball": distance_robot_to_ball,
            "reward": reward,
            "is_contact": is_contact,
            "force_to_target": force_to_target,
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
        self._ball_vel_matrix[0, 0] = self.data.qvel[self.ball_x_qvel_id]
        self._ball_vel_matrix[1, 0] = self.data.qvel[self.ball_y_qvel_id]

    def _is_robot_ball_contact(self):
        robot_geom_id = self.model.geom("robot_cylinder").id
        ball_geom_id = self.model.geom("ball_geom").id

        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom_pair = {contact.geom1, contact.geom2}

            if geom_pair == {robot_geom_id, ball_geom_id}:
                return True

        return False
    def _get_robot_ball_contact_force(self):
        robot_geom_id = self.model.geom("robot_cylinder").id
        ball_geom_id = self.model.geom("ball_geom").id

        total_force_world = np.zeros(3)

        for i in range(self.data.ncon):
            contact = self.data.contact[i]

            if {contact.geom1, contact.geom2} != {robot_geom_id, ball_geom_id}:
                continue

            force_torque_contact = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force_torque_contact)

            force_contact = force_torque_contact[:3]

            # contact.frame の先頭3つが法線方向、次が接線方向
            frame = contact.frame.reshape(3, 3)
            force_world = (
                force_contact[0] * frame[0]
                + force_contact[1] * frame[1]
                + force_contact[2] * frame[2]
            )

            total_force_world += force_world

        return total_force_world

def run_env_check(xml_path=DEFAULT_XML_PATH):
    from stable_baselines3.common.env_checker import check_env

    env = PushBallEnv(2, xml_path)
    check_env(env, warn=True)

def train(
    total_timesteps=10000,
    xml_path=DEFAULT_XML_PATH,
    save_path=None,
    best_model_save_path=None,
    eval_freq=10000,
    n_eval_episodes=10,
):
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import EvalCallback
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv

    n_envs = 16
    env = make_vec_env(
        lambda: PushBallEnv(1, xml_path),
        n_envs=n_envs,
        vec_env_cls=SubprocVecEnv
    )
    eval_env = make_vec_env(
        lambda: PushBallEnv(1, xml_path),
        n_envs=1
    )

    if best_model_save_path is None:
        if save_path is None:
            best_model_save_path = PROJECT_ROOT / "push_ball_best"
        else:
            save_path = Path(save_path)
            best_model_save_path = save_path.with_name(f"{save_path.stem}_best")

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(best_model_save_path),
        log_path=str(best_model_save_path),
        eval_freq=max(eval_freq // n_envs, 1),
        n_eval_episodes=n_eval_episodes,
        deterministic=True,
        render=False,
    )

    model = PPO("MlpPolicy", env, verbose=1, device="cpu", n_steps=128, batch_size=256)
    model.learn(total_timesteps=total_timesteps, callback=eval_callback)
    if save_path is not None:
        model.save(str(save_path))

    return model

def additional_train(
    total_timesteps=10000,
    xml_path=DEFAULT_XML_PATH,
    pre_model_path=None,
    save_path=None,
    best_model_save_path=None,
    eval_freq=10000,
    n_eval_episodes=10,
    stage=2,
):
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import EvalCallback
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv

    n_envs = 16
    env = make_vec_env(
        lambda: PushBallEnv(stage, xml_path),
        n_envs=n_envs,
        vec_env_cls=SubprocVecEnv
    )
    eval_env = make_vec_env(
        lambda: PushBallEnv(stage, xml_path),
        n_envs=1
    )

    if best_model_save_path is None:
        if save_path is None:
            best_model_save_path = PROJECT_ROOT / "push_ball_best"
        else:
            save_path = Path(save_path)
            best_model_save_path = save_path.with_name(f"{save_path.stem}_best")

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(best_model_save_path),
        log_path=str(best_model_save_path),
        eval_freq=max(eval_freq // n_envs, 1),
        n_eval_episodes=n_eval_episodes,
        deterministic=True,
        render=False,
    )

    model = PPO.load(str(pre_model_path), env=env, device="cpu")
    model.learn(
        total_timesteps=total_timesteps,
        callback=eval_callback,
        reset_num_timesteps=False,
    )
    if save_path is not None:
        model.save(str(save_path))

    return model


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Run environment check for PushBallEnv.")
    parser.add_argument("command", choices=["check", "train", "retrain"], help="Command to execute: 'check' to run environment check, 'train' to train the model, 'retrain' to continue training from a pre-trained model.")
    parser.add_argument("--xml_path", type=str, default=str(DEFAULT_XML_PATH))
    parser.add_argument("--save_path", type=str, default=str(PROJECT_ROOT / "push_ball"), help="Path to save the trained model.")
    parser.add_argument("--total_timesteps", type=int, default=10000, help="Total timesteps for training the model.")
    parser.add_argument("--best_model_save_path", type=str, default=None, help="Directory to save the best evaluated model.")
    parser.add_argument("--eval_freq", type=int, default=10000, help="Evaluate every this many environment timesteps.")
    parser.add_argument("--n_eval_episodes", type=int, default=10, help="Number of episodes per evaluation.")
    parser.add_argument("--pre_model_path", type=str, default=None, help="Path to a pre-trained model to load before training.")
    parser.add_argument("--stage", type=int, default=2, help="Stage of the environment to train on.")
    args = parser.parse_args()
    if args.command == "check":
        run_env_check(args.xml_path)
    elif args.command == "train":
        train(
            total_timesteps=args.total_timesteps,
            xml_path=args.xml_path,
            save_path=args.save_path,
            best_model_save_path=args.best_model_save_path,
            eval_freq=args.eval_freq,
            n_eval_episodes=args.n_eval_episodes,
        )
    elif args.command == "retrain":
        if args.pre_model_path is None:
            print("Error: --pre_model_path must be specified for additional training.")
            return
        save_path = Path(args.save_path)
        save_path = save_path.with_name(f"{save_path.stem}_stage{args.stage}{save_path.suffix}")
        additional_train(
            total_timesteps=args.total_timesteps,
            xml_path=args.xml_path,
            pre_model_path=args.pre_model_path,
            save_path=save_path,
            best_model_save_path=args.best_model_save_path,
            eval_freq=args.eval_freq,
            n_eval_episodes=args.n_eval_episodes,
            stage=args.stage,
        )

if __name__ == "__main__":
    main()