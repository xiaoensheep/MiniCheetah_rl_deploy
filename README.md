# MiniCheetah_rl_deploy

Mini Cheetah RL policy deployment framework for sim2sim and future sim2real work.

The supported robot identity in this repository is Mini Cheetah. MuJoCo is the primary simulation backend; real-hardware control is intentionally blocked until the Cheetah-Software hardware bridge and safety contracts are implemented.

## Build

```bash
cmake -S . -B build_mini -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DUSE_UDP_SIM=ON -DSEND_REMOTE=OFF -DBUILD_TESTING=ON
cmake --build build_mini --target rl_deploy -j2
```

## Test

```bash
ctest --test-dir build_mini --output-on-failure
```

## Run Simulation

Use two terminals:

```bash
conda activate mujoco
python interface/robot/simulation/mujoco_simulation.py
```

```bash
./build_mini/rl_deploy
```

The MuJoCo backend starts from the policy metadata stand pose by default and
holds that pose until `rl_deploy` connects. For startup-pose debugging, use
`--initial-pose crouch`; for headless smoke tests, use `--no-viewer --duration 5`.
Simulation builds use a wider RL entry joint-position tolerance to account for
MuJoCo contact settling; hardware builds keep the stricter default gate.

Keyboard commands: `z` stand up, wait for `stand up success`, `c` enter RL
control, `w/s/a/d` linear command, `q/e` yaw command. Run the MuJoCo terminal
first, then start `rl_deploy` from the repository root.

You can confirm the policy is actually running when the `rl_deploy` terminal
prints `rl_control`, `[ONNX ENTER]`, `[RL DEBUG] raw_action`, and
`[RL DEBUG] target_joint_pos`. The MuJoCo terminal should show changing
`Target Pos` values after RL entry, and the episode log should contain
`state_machine_state":"rl_control"` records.

When RL control is entered in simulation builds, structured episode logs are written to
`logs/sim_episode_*.jsonl`. Override the path with
`MINI_CHEETAH_EPISODE_LOG=/path/to/file.jsonl`.
