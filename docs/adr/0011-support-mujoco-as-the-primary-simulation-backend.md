# Support MuJoCo as the Primary Simulation Backend

The current Mini Cheetah deployment effort will treat MuJoCo as the supported simulation backend. PyBullet, RaiSim, Lite3 descriptions, and Lite3 SDK paths are legacy from the source project unless explicitly revalidated for the Mini Cheetah deployment interface contract.

## Consequences

Interface changes should be validated first against MuJoCo and the Mini Cheetah MJCF. Legacy simulator paths may be removed, disabled, or marked deprecated rather than kept superficially compiling without contract tests.
