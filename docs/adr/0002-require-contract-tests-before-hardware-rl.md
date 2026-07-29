# Require Contract Tests Before Hardware RL

Hardware RL control must be blocked until the Mini Cheetah hardware bridge passes contract tests shared with the simulation backend. The first bridge being able to compile, stream state, or send motor commands is not sufficient; it must prove that joint order, units, coordinate frames, default pose semantics, IMU semantics, base velocity semantics, PD command semantics, limits, and fallback behavior match the deployment interface contract.

## Consequences

The hardware entry point should have a safety gate that prevents entering RL mode when contract tests or required runtime checks have not passed. Development may bring up low-level hardware communication separately, but that is not the same milestone as enabling policy control on the physical robot.
