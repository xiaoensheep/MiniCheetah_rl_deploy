# Start Hardware RL With Zero-Velocity Standing

The first closed-loop hardware RL stage will only allow zero-velocity standing commands. Forward, lateral, and yaw commands must remain disabled until standing stability, deployment interface semantics, policy timing, safety gates, final safety clamps, emergency stop behavior, and episode logging have been validated.

## Consequences

Early hardware RL testing should separate standing stability from command-following behavior. Velocity command ranges should be opened gradually only after zero-velocity standing trials meet their acceptance criteria.
