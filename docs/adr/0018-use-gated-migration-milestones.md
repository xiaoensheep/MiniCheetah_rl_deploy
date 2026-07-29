# Use Gated Migration Milestones

The Mini Cheetah migration should proceed through explicit gated milestones rather than open-ended trial and error: MuJoCo sim2sim standing with policy metadata, deployment-interface contract tests for MuJoCo, low-level Cheetah-Software hardware I/O without RL commands, offline interface-mapper contract tests, hardware Shadow Mode, zero-velocity closed-loop standing, and then gradual velocity command enablement.

## Consequences

Each milestone needs acceptance criteria that decide whether the project can safely move forward. A later stage should not be used to compensate for unresolved uncertainty in an earlier stage.
