# Remove Legacy Source Identity and Paths

`MiniCheetah_rl_deploy` should be documented and maintained as a Mini Cheetah deployment project, not as a renamed source project with ambiguous legacy paths. Unsupported source-project robot descriptions, SDK paths, and simulator references should be cleaned up or clearly marked as legacy unless they are revalidated against the Mini Cheetah deployment interface contract.

## Consequences

Documentation, build options, directory names, comments, debug output, and unsupported source paths should be migrated in stages so future development does not mistake inherited functionality for supported Mini Cheetah functionality.
