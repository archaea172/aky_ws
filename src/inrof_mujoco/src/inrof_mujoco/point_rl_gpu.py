import gymnasium as gym
import numpy as np
import mujoco
from mujoco import mjx
from gymnasium import spaces
import inrof_swerm
from pathlib import Path
import jax
import jax.numpy as jnp
from functools import partial

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XML_PATH = PROJECT_ROOT / "models" / "point.xml"

@jax.jit
def mjx_forward(model, data):
    return mjx.forward(model, data)


@partial(jax.jit, static_argnames=("frame_skip",))
def mjx_step_frames(model, data, ctrl, frame_skip):
    data = data.replace(ctrl=ctrl)

    def step_once(carry, _):
        return mjx.step(model, carry), None

    data, _ = jax.lax.scan(step_once, data, xs=None, length=frame_skip)
    return data

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
        self.data = mjx.make_data(self.mjx_model)

        self.target_position = self.np_random.uniform(
            low=np.array([-3.0, -3.0]),
            high=np.array([3.5, 3.5]),
        ).astype(np.float64)

        target_body_id = self.model.body("target_body").id
        self.model.body_pos[target_body_id, 0:2] = self.target_position
        
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