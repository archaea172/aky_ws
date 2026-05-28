#!/usr/bin/env python3
"""Visualize SwermEnv with MuJoCo viewer."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
from mujoco import viewer

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from inrof_mujoco.rl_test import SwermEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open a MuJoCo GUI for SwermEnv debugging.")
    parser.add_argument("--xml", type=Path, default=PROJECT_ROOT / "models" / "boid.xml")
    parser.add_argument("--robot-num", type=int, default=5)
    parser.add_argument("--ball-num", type=int, default=3)
    parser.add_argument("--mode", choices=("fixed", "circle", "random"), default="fixed")
    parser.add_argument("--action", type=float, nargs=2, default=(0.0, 1.0))
    parser.add_argument("--circle-speed", type=float, default=0.04)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means until the window closes.")
    parser.add_argument("--print-every", type=int, default=20, help="Print one debug line every N env steps. 0 disables logs.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--contacts", action="store_true", help="Show contact points in the viewer and print contact geom pairs.")
    parser.add_argument("--no-realtime", action="store_false", dest="realtime", help="Run as fast as possible.")
    parser.set_defaults(realtime=True)
    return parser.parse_args()


def action_for_step(args: argparse.Namespace, step: int, rng: np.random.Generator) -> np.ndarray:
    if args.mode == "random":
        return rng.uniform(-1.0, 1.0, size=2).astype(np.float32)
    if args.mode == "circle":
        angle = step * args.circle_speed
        return np.array([math.cos(angle), math.sin(angle)], dtype=np.float32)
    return np.array(args.action, dtype=np.float32)


def contact_pairs(env: SwermEnv, limit: int = 6) -> str:
    pairs: list[str] = []
    for idx in range(min(env.data.ncon, limit)):
        contact = env.data.contact[idx]
        geom1 = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1) or str(contact.geom1)
        geom2 = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2) or str(contact.geom2)
        pairs.append(f"{geom1}<->{geom2}")
    extra = "" if env.data.ncon <= limit else f" +{env.data.ncon - limit}"
    return ", ".join(pairs) + extra


def main() -> None:
    args = parse_args()
    rng = np.random.default_rng(args.seed)
    env = SwermEnv(xml_path=str(args.xml), robot_num=args.robot_num, ball_num=args.ball_num)
    env.reset(seed=args.seed)

    sim_dt = env.model.opt.timestep * env.frame_skip
    start_time = time.perf_counter()
    next_tick = start_time
    step = 0

    with viewer.launch_passive(env.model, env.data) as gui:
        gui.cam.lookat[:] = np.array([2.5, 2.5, 0.0])
        gui.cam.distance = 7.0
        gui.cam.azimuth = 90.0
        gui.cam.elevation = -65.0
        gui.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(args.contacts)
        gui.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = int(args.contacts)

        while gui.is_running():
            if args.duration > 0.0 and time.perf_counter() - start_time >= args.duration:
                break

            action = action_for_step(args, step, rng)
            _, reward, terminated, truncated, info = env.step(action)

            if args.print_every and step % args.print_every == 0:
                contacts = f" contacts=[{contact_pairs(env)}]" if args.contacts and env.data.ncon else ""
                print(
                    f"step={step:05d} action={action.round(3).tolist()} "
                    f"reward={reward: .3f} ball_dist={info['ball_distance_sum']:.3f} "
                    f"robot_ball_dist={info['robot_ball_distance_sum']:.3f} ncon={env.data.ncon}{contacts}",
                    flush=True,
                )

            if terminated or truncated:
                env.reset(seed=args.seed)
                step = 0
                start_time = time.perf_counter()
                next_tick = start_time
            else:
                step += 1

            gui.sync()

            if args.realtime:
                next_tick += sim_dt
                sleep_time = next_tick - time.perf_counter()
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
                else:
                    next_tick = time.perf_counter()


if __name__ == "__main__":
    main()
