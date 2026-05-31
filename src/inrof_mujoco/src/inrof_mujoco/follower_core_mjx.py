from typing import NamedTuple
import jax
import jax.numpy as jnp

class FollowerParams(NamedTuple):
    max_vel: float
    k_separation: float
    k_alignment: float
    k_gravity: float
    k_wall: float
    k_follow: float
    field: float # 一旦使わない
    ir: float
    ir_min: float

class FollowerCoreMjx():
    def __init__(self, follower_params: FollowerParams):
        self.params = follower_params

    def update_vels_jax(self, pos, vel, leader_pos):
        return update_vels_jax(pos, vel, leader_pos, self.params)

    def update_vels_jit(self, pos, vel, leader_pos):
        return update_vels_jit(pos, vel, leader_pos, self.params)

    
def _masked_mean(values, mask):
    # values: (2, n, n), mask: (n, n)
    summed = jnp.sum(jnp.where(mask[None, :, :], values, 0.0), axis=2)
    count = jnp.sum(mask, axis=1).astype(values.dtype)
    return summed / jnp.maximum(count, 1.0)[None, :]
    
def update_vels_jax(pos, vel, leader_pos, params):
    # pos, vel: (2, n)
    # leader_pos: (2,)
    robot_num = pos.shape[1]

    diff = pos[:, :, None] - pos[:, None, :]
    dist2 = jnp.sum(diff * diff, axis=0)

    eye = jnp.eye(robot_num, dtype=bool)
    mask = (
        (dist2 < params.ir * params.ir)
        & (dist2 > params.ir_min * params.ir_min)
        & ~eye
    )
    dist2_safe = jnp.maximum(dist2, 1e-12)

    separation_terms = diff / (dist2_safe[None, :, :] ** 1.5)
    separation = _masked_mean(separation_terms, mask)

    vel_norm = jnp.linalg.norm(vel, axis=0)
    vel_dir = vel / jnp.maximum(vel_norm, 1.0)[None, :]
    alignment_terms = jnp.broadcast_to(vel_dir[:, None, :], diff.shape)
    alignment = _masked_mean(alignment_terms, mask)

    dist = jnp.sqrt(dist2_safe)
    gravity_terms = diff / jnp.maximum(dist, 1.0)[None, :, :]
    gravity = -_masked_mean(gravity_terms, mask)

    follow_diff = leader_pos[:, None] - pos
    follow_norm = jnp.linalg.norm(follow_diff, axis=0)
    follow = follow_diff / jnp.maximum(follow_norm, 0.01)[None, :]

    cmd_raw = (
        params.k_separation * separation +
        params.k_alignment * alignment +
        params.k_gravity * gravity +
        params.k_follow * follow
    )

    cmd_norm = jnp.linalg.norm(cmd_raw, axis=0)
    scale = jnp.minimum(1.0, params.max_vel / jnp.maximum(cmd_norm, 1e-12))
    return cmd_raw * scale[None, :]

update_vels_jit = jax.jit(update_vels_jax)
