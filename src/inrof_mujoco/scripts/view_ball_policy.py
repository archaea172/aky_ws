#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from mujoco import viewer
from stable_baselines3 import PPO

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from inrof_mujoco.push_ball_rl import PushBallEnv

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open a MuJoCo GUI for a trained PushBallEnv policy.")
    parser.add_argument("--model", type=Path, default=PROJECT_ROOT / "push_ball")
    parser.add_argument("--xml", type=Path, default=PROJECT_ROOT / "models" / "push_ball.xml")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means until the window closes.")
    parser.add_argument("--print-every", type=int, default=20, help="Print one debug line every N env steps. 0 disables logs.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--stochastic", action="store_true", help="Sample actions from the policy instead of using deterministic actions.")
    parser.add_argument("--no-realtime", action="store_false", dest="realtime", help="Run as fast as possible.")
    parser.set_defaults(realtime=True)
    return parser.parse_args()

def main() -> None:
    args = parse_args()
    env = PushBallEnv(xml_path=args.xml)
    model = PPO.load(str(args.model), env=env, device="cpu")
    obs, _ = env.reset(seed=args.seed)

    sim_dt = env.model.opt.timestep * env.frame_skip
    start_time = time.perf_counter()
    next_tick = start_time
    step = 0

    with viewer.launch_passive(env.model, env.data) as gui:
        gui.cam.lookat[:] = np.array([1.8, 1.8, 0.0])
        gui.cam.distance = 6.5
        gui.cam.azimuth = 90.0
        gui.cam.elevation = -65.0

        while gui.is_running():
            if args.duration > 0.0 and time.perf_counter() - start_time >= args.duration:
                break

            action, _ = model.predict(obs, deterministic=not args.stochastic)
            obs, reward, terminated, truncated, info = env.step(action)

            if args.print_every > 0 and step % args.print_every == 0:
                print(f"Step: {step}, Reward: {reward:.3f}, Info: {info}, Action: {action}")


            if terminated or truncated:
                obs, _ = env.reset()
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