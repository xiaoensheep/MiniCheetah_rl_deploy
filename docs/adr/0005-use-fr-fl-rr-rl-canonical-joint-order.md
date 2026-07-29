# Use FR-FL-RR-RL as the Canonical Joint Order

`MiniCheetah_rl_deploy` will use `FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf` as the canonical joint order at the deployment interface contract boundary. Simulators, policy metadata, Cheetah-Software motor indices, debug output, and contract tests must either use this order directly or pass through an explicit, tested mapping.

## Consequences

Implicit `0..11` assumptions are unsafe and should be removed from policy and hardware code. Any policy trained with a different joint order must declare that order in its policy metadata so the policy stack can validate or map it before running.
