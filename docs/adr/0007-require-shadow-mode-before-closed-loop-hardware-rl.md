# Require Shadow Mode Before Closed-Loop Hardware RL

Closed-loop hardware RL must be preceded by Shadow Mode on the physical Mini Cheetah. In Shadow Mode, the hardware bridge reads real state, constructs observations at the real deployment rate, runs the ONNX policy, and records policy outputs, but it does not send policy actions to the motors.

## Consequences

Closed-loop RL should remain disabled until Shadow Mode demonstrates stable timing, finite observations, reasonable units, non-saturated policy outputs, target joint positions within limits, and action distributions comparable to simulation under similar state and command conditions.
