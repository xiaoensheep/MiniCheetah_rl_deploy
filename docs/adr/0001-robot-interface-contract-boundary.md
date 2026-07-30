# Keep Simulation and Hardware Behind One Robot Interface Contract

We will keep `MiniCheetah_rl_deploy` as the deployment framework and treat Cheetah-Software as the source for the Mini Cheetah hardware bridge, rather than letting Cheetah-Software's controller architecture replace the state machine and policy runner. Simulation and hardware may use different transport mechanisms, but they must expose the same robot-facing semantics through `RobotInterface`: joint order, units, coordinate frames, timestamps, IMU meaning, base velocity meaning, and PD plus feedforward command meaning.

## Considered Options

- Make Cheetah-Software the main framework and embed the ONNX policy runner inside it.
- Keep this standalone deployment framework as the main framework and implement a Mini Cheetah hardware bridge behind the existing robot interface.

## Consequences

Policy code and state transitions should not depend directly on MuJoCo, UDP, SPI, LCM, or Cheetah-Software internals. Any mismatch between simulation and hardware must be represented as a violation of the Deployment Interface Contract, not hidden inside policy runner code.
