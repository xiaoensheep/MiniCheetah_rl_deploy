# Treat Policy Timing as a Deployment Contract

Policy timing must be treated as part of the policy metadata. The metadata should declare the training-time policy inference frequency, simulator timestep, command update frequency, PD update frequency, and control decimation, and both simulation and hardware deployments must validate that their runtime timing stays within acceptable tolerance.

## Consequences

Shadow Mode may record timing violations, but closed-loop hardware RL must refuse to start or exit to a safe state when policy timing requirements are not met. Thread scheduling, UDP timing, hardware bridge timing, and policy runner decimation are therefore safety-relevant behavior rather than incidental implementation details.
