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
cd interface/robot/simulation
python mujoco_simulation.py
```

```bash
./build_mini/rl_deploy
```

Keyboard commands: `z` stand up, `c` enter RL control, `wasd` linear command, `q/e` yaw command.
