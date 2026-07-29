# Add a Final Safety Clamp in the Hardware Bridge

The Mini Cheetah hardware bridge must include a non-bypassable final safety clamp before commands reach the motors. This clamp should constrain desired position, desired velocity, gains, feedforward torque, per-step command changes, stale commands, communication loss, unsafe IMU state, and hardware fault signals independently of the policy runner and high-level state machine.

## Consequences

State-machine checks and policy-runner checks remain necessary, but they are not sufficient for hardware safety. Simulation should reuse or test equivalent clamp behavior where possible, while the physical hardware bridge must enforce it as the last command boundary.
