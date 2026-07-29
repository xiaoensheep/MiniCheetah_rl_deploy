# Remove Lite3 Identity and Legacy Paths

`MiniCheetah_rl_deploy` should be documented and maintained as a Mini Cheetah deployment project, not as a renamed Lite3 project with ambiguous legacy paths. Lite3, Jueying, DeepRobotics SDK, and unsupported simulator references should be cleaned up or clearly marked as legacy unless they are revalidated against the Mini Cheetah deployment interface contract.

## Consequences

Documentation, build options, directory names, comments, debug output, and unsupported source paths should be migrated in stages so future development does not mistake inherited Lite3 functionality for supported Mini Cheetah functionality.
