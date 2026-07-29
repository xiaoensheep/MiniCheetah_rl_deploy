# Split Hardware I/O From Interface Mapping

The Mini Cheetah hardware bridge should separate physical I/O from deployment-interface mapping. A hardware adapter should own Cheetah-Software communication, while an interface mapper should own joint order mapping, unit conversion, sign conventions, state packaging, command conversion, and safety-relevant validation against the deployment interface contract.

## Consequences

The most dangerous bridge logic should be testable without a connected robot. Contract tests should target the interface mapper directly, and the hardware adapter should remain a thin boundary around low-level hardware communication.
