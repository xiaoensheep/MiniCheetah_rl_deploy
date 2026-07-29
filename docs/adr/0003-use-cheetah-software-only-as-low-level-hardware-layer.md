# Use Cheetah-Software Only as the Low-Level Hardware Layer

The Mini Cheetah hardware bridge should reuse Cheetah-Software only at the low-level hardware boundary: motor state, motor commands, IMU data, board communication, and transport structures. It should not reuse Cheetah-Software's high-level FSM, Convex MPC, BalanceController, or locomotion controllers because `MiniCheetah_rl_deploy` already owns the policy stack, state transitions, safety checks, and command semantics.

## Consequences

There should be one high-level controller at runtime. The RL deployment state machine must remain the owner of when the robot is idle, standing, running policy control, damping, or refusing unsafe control. Cheetah-Software integration code belongs behind the hardware bridge and should not leak into the policy runner or simulation backend.
