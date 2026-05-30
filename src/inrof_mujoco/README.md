# inrof_mujoco

## Visual debug

Run the MuJoCo GUI from the package root:

```bash
cd src/inrof_mujoco
uv run python scripts/view_rl_env.py --mode fixed --action 0 1 --contacts
```

Useful modes:

```bash
uv run python scripts/view_rl_env.py --mode circle --contacts
uv run python scripts/view_rl_env.py --mode random --print-every 5
```

The viewer uses the same `SwermEnv` step path as training, but `rl_test.py` no longer starts PPO training when imported.
