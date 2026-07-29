# Define Emergency Stop as a Contract

Emergency stop behavior must be defined as a deployment contract rather than treated as a side effect of `JointDampingState`. Triggers should include user stop, keyboard or gamepad loss, communication timeout, policy timeout, unsafe posture, invalid command, stale command, and hardware fault signals; after triggering, policy authority must stop, a hardware-safe command must be applied, and human confirmation must be required before re-entering stand-up or RL control.

## Consequences

`JointDampingState` may implement one emergency-stop response, but it is not the full emergency-stop system. Simulation and hardware should expose the same emergency-stop state semantics even if the final actuator behavior differs.
