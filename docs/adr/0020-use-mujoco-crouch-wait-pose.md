# Use a MuJoCo Crouch Wait Pose Before Stand-Up

MuJoCo sim2sim should start Mini Cheetah in the prepared crouch keyframe, not in the policy stand pose. The simulator and simulation `IdleState` may actively hold this keyframe before the user requests stand-up.

This is a simulation startup convenience, not the policy initial state. Pressing `z` still transitions through `StandUpState`, whose target remains the policy metadata default joint pose. RL entry checks still happen after stand-up.

This does not enable hardware RL or define a hardware startup command. The future Cheetah-Software hardware bridge must define its own safe boot and recovery behavior behind `RobotInterface` contract tests before closed-loop hardware RL is allowed.

`JointDampingState` is a stop/recovery state. It should not automatically return to idle hold; a human must explicitly request stand-up again.
