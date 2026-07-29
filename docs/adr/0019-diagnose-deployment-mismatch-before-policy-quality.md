# Diagnose Deployment Mismatch Before Policy Quality

When a Mini Cheetah policy runs but quickly becomes unstable, the first diagnosis should be deployment contract mismatch rather than policy quality. Policy training quality should be blamed only after policy metadata, joint order, default pose, observation scaling, base velocity, projected gravity, action semantics, timing, and policy entry state have been validated against the training assumptions.

## Consequences

The preferred debugging workflow is to compare the same robot state across the training export path, ONNX policy runner, MuJoCo simulation backend, and eventually hardware Shadow Mode. A policy should not be dismissed as poorly trained until these deployment semantics have been shown to match.
