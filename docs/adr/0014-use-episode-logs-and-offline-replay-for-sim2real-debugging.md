# Use Episode Logs and Offline Replay for Sim2Real Debugging

Simulation and hardware runs should produce a shared episode log format containing timestamps, state-machine state, user command, observations, raw actions, clipped actions, target joint positions, measured joint state, IMU data, base velocity, safety gate state, clamp decisions, and timing. Offline replay should use these logs to rerun or inspect policy-stack behavior without commanding motors.

## Consequences

Console debug prints are not sufficient for diagnosing Mini Cheetah sim2real issues. Shadow Mode and closed-loop runs should create data that can be compared against MuJoCo behavior, policy metadata, and contract tests after the run.
