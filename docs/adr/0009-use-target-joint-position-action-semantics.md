# Use Target Joint Position Action Semantics

The initial Mini Cheetah policy deployment path will treat policy outputs as target joint position actions: normalized action values are scaled by policy metadata and added to the policy default joint pose to produce desired joint positions. The deployment framework then sends those desired positions through PD plus feedforward command semantics.

## Consequences

Torque policies, absolute-position policies, and other action semantics are out of scope until policy metadata and contract tests explicitly support them. Replacing a policy with one that has different action semantics without changing metadata and validation is unsafe.
