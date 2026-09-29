# Unitree H2 Closed-Loop Robot Simulation Asset

## Overview

This package contains a simulation-ready, closed-loop [Unitree H2](https://www.unitree.com/) robot in Universal Scene Description (USD) format. The supported entrypoint is [`h2.usda`](h2.usda).

The model preserves six physical linkage closures in the ankles, knees, and waist. Each closure is a standard `PhysicsSphericalJoint` with `physics:excludeFromArticulation = true`, allowing Newton maximal-coordinate solvers, including VBD, to import the closures as loop joints.

## Asset structure

The package follows the Isaac Sim asset-structure guidance:

- `Source/h2_import.usdc` is the immutable imported source stage.
- `Payload/GeometryLibrary.usdc` contains the binary mesh library.
- `Payload/Geometry.usda` contains the flattened body hierarchy and mesh instances.
- `Payload/Materials.usda` contains visual materials.
- `Payload/Physics.usda` contains engine-neutral USD/Newton rigid bodies, collisions, joints, masses, and inertias.
- `Payload/Mujoco.usda` contains MuJoCo-specific APIs, attributes, and actuators.
- `Payload/Robot.usda` applies `IsaacRobotAPI` and enumerates robot links and joints.
- `Payload/Contents.usda` composes the feature layers behind the payload in `h2.usda`.

Rigid bodies are direct children of `/H2/Geometry`, and joints are direct children of `/H2/Joints`. This keeps the simulation hierarchy flat while preserving source world transforms and joint endpoints.

## Sources and reproducibility

The source model is [`robots/h2_description/H2_loop.xml`](https://github.com/unitreerobotics/unitree_ros/blob/7d6075f7f58588b189b940130e3edab3c839b2df/robots/h2_description/H2_loop.xml) from Unitree's `unitree_ros` repository at revision `7d6075f7f58588b189b940130e3edab3c839b2df`.

The source stage was converted with `mujoco-usd-converter==0.2.0`. Its six loop joints have `MjcEqualityConnectAPI` removed while retaining `physics:excludeFromArticulation`, endpoints, and anchors. No body, mass, inertia, collision, anchor, or tree-joint properties were changed. Source provenance is recorded in [`SOURCE.json`](SOURCE.json).

Regenerate the structured layers with an environment that provides OpenUSD Python bindings:

```bash
python unitree_h2/tools/build_structured_usd.py \
  --source unitree_h2/usd_structured/Source/h2_import.usdc \
  --output unitree_h2/usd_structured
```

`BUILD.json` records SHA-256 hashes for the generated package.

## License

This model is released under a [BSD-3-Clause License](../LICENSE).
