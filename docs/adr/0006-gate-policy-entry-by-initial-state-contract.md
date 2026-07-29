# Gate Policy Entry by the Initial State Contract

Entering RL control must be gated by checks that the current robot state is close enough to the policy's expected initial state. The stand-up target should come from, or be explicitly validated against, the policy metadata default joint pose, and the transition should verify joint position error, base orientation, base height, base velocity, and joint velocity before allowing the policy runner to command the robot.

## Consequences

The `StandUpState -> RLControl` transition should not be a blind response to a user key press. A user command may request policy entry, but the policy stack must refuse the transition when the robot is outside the policy entry gate and keep the robot standing or move it to damping.
