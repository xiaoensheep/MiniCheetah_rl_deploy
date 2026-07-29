# Do Not Zero-Fill Required Base Velocity Observation

If a policy's metadata declares base linear velocity as part of its observation, the hardware bridge must provide body-frame base linear velocity with the same semantics as the training environment. The hardware path must not silently fill this observation with zero unless the policy was trained with that same convention.

## Consequences

Hardware deployment of a policy that requires base linear velocity depends on either a validated state estimator or retraining a policy that does not require that observation. A default `RobotInterface::GetBaseLinearVelocity()` implementation returning zero is unsafe for closed-loop hardware RL unless the policy metadata explicitly allows it.
