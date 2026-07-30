# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Agent Skills

### Issue Tracker

Issues are tracked in this repo's GitHub Issues using the `gh` CLI. See `docs/agents/issue-tracker.md`.

### Triage Labels

This repo uses the default mattpocock/skills triage labels: `needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix`. See `docs/agents/triage-labels.md`.

### Domain Docs

This is a single-context repo: read root `CONTEXT.md` and root `docs/adr/` when they exist. See `docs/agents/domain.md`.

## Project Overview

MiniCheetah_rl_deploy is a reinforcement-learning policy deployment framework for MIT Mini Cheetah replication work. It is being migrated toward a strict sim2sim/sim2real contract:

- MuJoCo simulation and future real hardware must expose the same `RobotInterface` semantics.
- The ONNX policy runner reads policy metadata from `policy/ppo/policy_metadata.json`.
- Real hardware is not enabled yet; the Mini Cheetah hardware bridge must be implemented against Cheetah-Software and pass safety contract tests before closed-loop RL is allowed.

## Build Commands

### Simulation

```bash
cmake -S . -B build_mini -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DUSE_UDP_SIM=ON -DSEND_REMOTE=OFF -DBUILD_TESTING=ON
cmake --build build_mini --target rl_deploy -j2
```

### Tests

```bash
ctest --test-dir build_mini --output-on-failure
```

### Real Hardware

Real-hardware build/run is intentionally blocked until the Cheetah-Software bridge exists:

```bash
cmake -S . -B build_mini -DUSE_CHEETAH_SOFTWARE_BRIDGE=ON
```

That option currently fails fast by design.

## Running MuJoCo Sim2sim

Simulation uses two terminals.

Terminal 1:

```bash
conda activate mujoco
cd interface/robot/simulation
python mujoco_simulation.py
```

Terminal 2:

```bash
./build_mini/rl_deploy
```

Keyboard controls in simulation:

- `z`: stand up
- `c`: enter RL control
- `wasd`: velocity command
- `q/e`: yaw command

## Architecture

### State Machine

The central control loop transitions through:

`Idle -> StandUp -> RLControl -> JointDamping -> Idle`

Important files:

- `state_machine/state_machine.hpp`: creates robot/user interfaces and states.
- `state_machine/standup_state.hpp`: moves to the policy default pose before RL entry.
- `state_machine/policy_entry_gate.*`: validates the initial state contract before RL.
- `state_machine/rl_control_state_onnx.hpp`: owns the policy control state and posture fallback.

### Interface Layer

- `interface/robot/robot_interface.h`: robot-facing contract.
- `interface/robot/simulation/simulation_interface.hpp`: UDP simulation adapter.
- `interface/robot/simulation/mujoco_simulation.py`: supported Mini Cheetah MuJoCo simulator.
- `interface/robot/hardware/hardware_interface.hpp`: Mini Cheetah real-hardware placeholder.

### Policy Runner

- `run_policy/mini_cheetah_policy_runner_onnx.h`
- `run_policy/mini_cheetah_policy_runner_onnx.cpp`
- `run_policy/policy_metadata.*`

The policy runner loads `policy/ppo/policy.onnx`, checks metadata, builds the observation, runs ONNXRuntime, applies action scaling, and outputs PD target joint positions.

### Contracts And Logs

- `interface/robot/simulation/simulation_packet_codec.*`: canonical UDP packet format and command safety limits.
- `logging/episode_log.*`: JSONL episode log format for future offline replay.
- `tests/`: contract tests for metadata, policy entry gate, packet codec, and episode logs.

## Key Files

| Task | File |
|------|------|
| Change policy metadata | `policy/ppo/policy_metadata.json` |
| Change policy runner | `run_policy/mini_cheetah_policy_runner_onnx.cpp` |
| Tune entry checks | `state_machine/policy_entry_gate.cpp` |
| Tune stand-up/default pose | `state_machine/parameters/mini_cheetah_control_parameters.cpp` |
| Inspect simulation packet semantics | `interface/robot/simulation/simulation_packet_codec.cpp` |
| Add hardware bridge | `interface/robot/hardware/hardware_interface.hpp` |
