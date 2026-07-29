# Mini Cheetah RL Deployment

This context describes the language used for deploying reinforcement-learning policies to the Mini Cheetah while keeping simulation and hardware behavior comparable.

## Language

**Deployment Interface Contract**:
The robot-facing semantics that both simulation and hardware must share: state meaning, command meaning, units, coordinate frames, joint order, timing expectations, and safety behavior.
_Avoid_: SDK interface, simulator interface

**Deployment Contract Mismatch**:
A disagreement between policy metadata, simulation behavior, hardware behavior, or training assumptions that can make a valid policy unsafe after deployment.
_Avoid_: bad policy, random sim2real issue

**Hardware Bridge**:
The robot-specific adapter that makes the physical Mini Cheetah satisfy the Deployment Interface Contract.
_Avoid_: hardware SDK, real robot code

**Hardware Adapter**:
The hardware bridge component that performs physical Mini Cheetah I/O through the low-level hardware layer.
_Avoid_: mapper, policy interface

**Interface Mapper**:
The hardware bridge component that converts between low-level hardware data and the Deployment Interface Contract.
_Avoid_: driver, SDK wrapper

**Low-Level Hardware Layer**:
The Cheetah-Software layer that exposes motor, IMU, board, and transport data close to the physical robot without owning locomotion state or gait decisions.
_Avoid_: Cheetah controller, high-level controller

**Policy Stack**:
The MiniCheetah_rl_deploy state machine, policy runner, safety checks, and command generation path that own RL deployment behavior.
_Avoid_: Cheetah-Software control stack, MPC stack

**Policy Metadata**:
The policy-specific deployment contract that describes observation layout, action layout, scales, default pose, joint order, control timing, and training-time robot assumptions for one policy artifact.
_Avoid_: runner constants, policy notes

**Canonical Joint Order**:
The project-wide joint ordering used at the Deployment Interface Contract boundary: `FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf`.
_Avoid_: motor index order, policy order, simulator order

**Policy Entry Gate**:
The safety and distribution check that decides whether the current robot state is close enough to the policy's expected initial state to enter RL control.
_Avoid_: mode switch, keyboard command

**Shadow Mode**:
A hardware deployment mode that reads real robot state and runs the policy without sending policy actions to the motors.
_Avoid_: dry run, fake control

**Closed-Loop RL Control**:
A hardware deployment mode in which policy outputs are converted into motor commands and sent to the physical robot.
_Avoid_: real mode, hardware mode

**Zero-Velocity Standing Trial**:
The first closed-loop hardware RL trial type, where commanded linear and angular velocity are all zero and the policy is evaluated only for safe standing.
_Avoid_: first walk, simple hardware test

**Migration Milestone**:
A gated project stage that must satisfy explicit acceptance criteria before the next Mini Cheetah deployment stage begins.
_Avoid_: progress step, todo item

**Base Linear Velocity Observation**:
The policy observation component representing the robot base linear velocity in the body frame, with semantics matching the policy's training environment.
_Avoid_: velocity guess, optional velocity

**State Estimator**:
The component that produces derived robot state required by the policy when the hardware does not measure it directly.
_Avoid_: sensor patch, observer

**Action Semantics**:
The meaning of policy outputs before they are converted into the Deployment Interface Contract command.
_Avoid_: output format, control type

**Target Joint Position Action**:
An action semantics where normalized policy outputs are scaled and added to the policy default joint pose to produce desired joint positions.
_Avoid_: torque action, raw action

**Policy Timing Contract**:
The policy-specific timing expectations for inference frequency, command update frequency, simulator timestep, and control decimation.
_Avoid_: loop rate note, timing setting

**Simulation Backend**:
The physics-backed adapter that makes a simulator satisfy the Deployment Interface Contract.
_Avoid_: fake robot, sim-only interface

**Supported Simulation Backend**:
A simulation backend that the project actively validates against the Deployment Interface Contract.
_Avoid_: available simulator, legacy backend

**Legacy Lite3 Path**:
Code, documentation, assets, build options, or debug output inherited from Lite3_rl_deploy that has not been revalidated for Mini Cheetah.
_Avoid_: supported fallback, old option

**Contract Test**:
A test that verifies a Simulation Backend or Hardware Bridge satisfies the Deployment Interface Contract before it is trusted by the policy stack.
_Avoid_: smoke test, driver test

**Hardware Safety Gate**:
A required check that prevents hardware RL control from starting until the relevant Contract Tests and runtime safety conditions pass.
_Avoid_: optional safety check, manual checklist

**Final Safety Clamp**:
The non-bypassable hardware-side limiter that constrains outgoing motor commands before they reach the physical Mini Cheetah.
_Avoid_: upper-layer check, debug limit

**Emergency Stop Contract**:
The required behavior for detecting unsafe control, stopping policy authority, commanding a safe hardware response, and requiring explicit human recovery before control resumes.
_Avoid_: damping state, lose-control check

**Episode Log**:
A structured record of one deployment run containing robot state, commands, observations, actions, safety decisions, and timing.
_Avoid_: console output, debug print

**Offline Replay**:
A tool workflow that reprocesses recorded deployment data through the policy stack to compare simulation and hardware behavior without commanding motors.
_Avoid_: log viewer, playback video
