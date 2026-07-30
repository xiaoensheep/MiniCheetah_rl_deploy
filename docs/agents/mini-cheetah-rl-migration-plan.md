# Mini Cheetah RL Deployment Migration Plan

## Goal

Migrate `MiniCheetah_rl_deploy` into a Mini Cheetah RL deployment framework that uses MuJoCo for sim2sim and Cheetah-Software only as the low-level hardware layer for sim2real. Simulation and hardware must satisfy the same Deployment Interface Contract before closed-loop hardware RL is allowed.

## Core Decisions

- Keep `MiniCheetah_rl_deploy` as the main policy stack; do not embed the RL policy into Cheetah-Software's high-level controller framework.
- Use Cheetah-Software only behind the Hardware Bridge for low-level motor, IMU, board, and transport access.
- Treat MuJoCo as the only supported simulation backend in the current migration stage.
- Require Policy Metadata for every `policy.onnx`.
- Use `FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf` as the Canonical Joint Order.
- Treat `q_des = default_q + action_scale * action` as the current supported action semantics.
- Require contract tests, Shadow Mode, hardware safety gates, and final safety clamps before closed-loop hardware RL.

## Milestones

### 1. Stabilize MuJoCo Sim2Sim

Acceptance criteria:

- MuJoCo launches from the project-owned Mini Cheetah MJCF.
- `rl_deploy` can enter idle, standup, and RL states without packet layout errors.
- The robot stands with `cmd_vel = [0, 0, 0]` for a defined duration.
- Debug output confirms finite observations, finite actions, and target joint positions within limits.

### 2. Add Policy Metadata

Acceptance criteria:

- `policy/ppo/policy.onnx` has a colocated metadata file.
- Metadata declares obs layout, obs dimension, action dimension, action semantics, action scale, default joint pose, joint order, velocity scales, command layout, decimation, and timing.
- `MiniCheetahPolicyRunnerONNX` loads these values from metadata instead of relying on hardcoded constants.
- Running without matching metadata is refused.

### 3. Define and Test the Deployment Interface Contract

Acceptance criteria:

- Contract tests verify joint order, units, signs, coordinate frames, timestamps, IMU semantics, base velocity semantics, command columns, limits, and damping behavior.
- MuJoCo Simulation Backend passes the contract tests.
- Contract tests are runnable without connected hardware.

### 4. Gate Policy Entry

Acceptance criteria:

- `StandUpState -> RLControl` is a request, not an unconditional transition.
- Entry checks include joint error from default pose, base orientation, base height, joint velocity, base velocity, and policy timing readiness.
- Failed checks keep the robot in standup or move it to damping.

### 5. Add Episode Logs and Offline Replay

Acceptance criteria:

- Simulation records structured episode logs.
- Logs include state machine state, user command, observation, raw action, clipped action, target joint position, measured joint state, IMU, base velocity, safety gates, clamp decisions, and timing.
- Offline replay can run logged observations through the policy runner.

### 6. Implement Cheetah-Software Hardware Bridge Skeleton

Acceptance criteria:

- Hardware code is split into Hardware Adapter and Interface Mapper.
- Hardware Adapter owns Cheetah-Software low-level I/O only.
- Interface Mapper owns unit conversion, sign conventions, joint mapping, state packaging, command conversion, and contract validation.
- No Cheetah-Software FSM, MPC, BalanceController, or locomotion controller enters the Policy Stack.

### 7. Run Hardware Shadow Mode

Acceptance criteria:

- Real Mini Cheetah state is read at deployment frequency.
- ONNX policy runs from real observations.
- Policy outputs are logged but not sent to motors.
- Observations are finite and physically reasonable.
- Actions do not saturate persistently.
- Timing stays within policy metadata tolerance.
- Shadow Mode logs are comparable to MuJoCo logs.

### 8. Enable Zero-Velocity Closed-Loop Standing

Acceptance criteria:

- Closed-loop RL is allowed only for `cmd_vel = [0, 0, 0]`.
- Hardware Safety Gate passes before entering RL.
- Final Safety Clamp enforces position, velocity, gain, torque, step-change, stale-command, communication, posture, temperature, and fault limits.
- Emergency Stop Contract is implemented and requires human recovery before re-entry.
- The robot stands safely for the target duration.

### 9. Gradually Open Velocity Commands

Acceptance criteria:

- Forward, lateral, and yaw commands are enabled only after zero-velocity standing passes.
- Each command axis has explicit range limits and test cases.
- Logs show command tracking without unsafe action saturation or repeated clamp intervention.

## Immediate Next Task

Wire structured episode logging into the simulation RL control path. The log and offline replay contracts now exist, and `MiniCheetahPolicyRunnerONNX` can replay a logged observation without commanding motors; the next step is to emit real episode records during MuJoCo sim2sim runs so replay compares actual rollout data.

## References

- `CONTEXT.md`
- `docs/adr/0001-robot-interface-contract-boundary.md`
- `docs/adr/0002-require-contract-tests-before-hardware-rl.md`
- `docs/adr/0004-require-policy-metadata.md`
- `docs/adr/0005-use-fr-fl-rr-rl-canonical-joint-order.md`
- `docs/adr/0018-use-gated-migration-milestones.md`
- `docs/adr/0019-diagnose-deployment-mismatch-before-policy-quality.md`
