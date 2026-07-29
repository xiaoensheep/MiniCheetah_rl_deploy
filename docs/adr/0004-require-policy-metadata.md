# Require Policy Metadata

Every ONNX policy used by `MiniCheetah_rl_deploy` must have policy metadata that describes the observation layout, action layout, scales, default joint pose, joint order, control timing, and training-time robot assumptions. These values should be treated as part of the policy artifact rather than hidden as constants inside the ONNX policy runner.

## Consequences

The policy runner should move toward loading metadata such as observation dimension, action dimension, `action_scale`, `default_joint_pos`, velocity scales, command layout, decimation, and joint order from a policy-specific file. Replacing `policy.onnx` without replacing or validating its metadata should be considered unsafe.
