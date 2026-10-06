# Unitree H2 Closed-Loop Robot Simulation Asset

## Overview

This package contains a simulation-ready, closed-loop [Unitree H2](https://www.unitree.com/) robot in Universal Scene Description (USD) format. The supported entrypoint is [`h2.usda`](h2.usda).

The model preserves six physical linkage closures in the ankles, knees, and waist. Each closure is a standard `PhysicsSphericalJoint` with `physics:excludeFromArticulation = true`, allowing Newton maximal-coordinate solvers, including VBD, to import the closures as loop joints.

## Asset structure

The package follows the Isaac Sim asset-structure guidance:

- `Source/h2_import.usdc` is the immutable imported source stage.
- `Payload/GeometryLibrary.usdc` contains 38 shared mesh definitions for 59 mesh instances.
- `Payload/Geometry.usda` contains the flattened body hierarchy and mesh instances, with transforms and appearance authored per instance. Collision-only shapes have `purpose = "guide"`, keeping them out of default-purpose rendering without removing their physics properties.
- `Payload/Materials.usda` contains visual materials.
- `Payload/Physics.usda` contains engine-neutral USD/Newton rigid bodies, collisions, joints, masses, and inertias.
- `Payload/Mujoco.usda` contains MuJoCo-specific APIs, attributes, and actuators.
- `Payload/Robot.usda` applies `IsaacRobotAPI` and enumerates robot links and joints.
- `Payload/Contents.usda` composes the feature layers behind the payload in `h2.usda`.

Rigid bodies are direct children of `/H2/Geometry`, and joints are direct children of `/H2/Joints`. This keeps the simulation hierarchy flat while preserving source world transforms and joint endpoints.

## Newton joint dynamics

The Physics layer applies `NewtonJointAPI` and mirrors MuJoCo armature, friction, and passive damping, so the default `ModelBuilder.add_usd()` import preserves them without an explicit `SchemaResolverMjc`. Angular `newton:damping` is authored per degree (`mjc:damping * pi / 180`); Newton converts it back to per-radian SI damping during import. Passive damping is separate from drive damping.

Joints with upstream effort limits have zero-gain angular `PhysicsDriveAPI` drives whose `maxForce` preserves the symmetric joint effort limit. These carry torque limits without prescribing a position controller or adding passive damping through a drive. Joints without authored effort limits have no drives. The MuJoCo layer retains the original joint attributes and 31 actuators; authoring USD limits does not add MuJoCo motors.

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

### Archived source storage

The immutable `Source/h2_import.usdc` snapshot is intentionally retained as the exact input to the restructuring builder. It adds 5,670,697 bytes (about 5.67 MB) and includes geometry also present in the runtime library. Keeping it makes rebuilding independent of converter availability or changes to its dependencies; the pinned upstream MJCF and converter provenance remain recorded in `SOURCE.json`.

This is an intentional storage cost for reproducibility. The runtime entrypoint does not reference or load the source snapshot, so it adds no robot bodies or simulated mass. Visual and collision instances share runtime mesh-library entries independently of this archive.

## Verification

Validate source-property preservation, body placement, loop closures, geometry sharing, USD targets and composition, package hashes, and byte-for-byte regeneration with OpenUSD bindings:

```bash
python unitree_h2/tools/validate_structured_usd.py
```

With Newton and its USD/MuJoCo import dependencies installed, also check that default-import armature, passive damping, friction, and effort limits match the MuJoCo resolver:

```bash
python unitree_h2/tools/validate_structured_usd.py --newton
```

Run the ten-second posture smoke test with the G1 example's import options, PD gains, and six substeps per frame:

```bash
python unitree_h2/tools/validate_structured_usd.py --simulate --device cpu
# On a CUDA-capable host:
python unitree_h2/tools/validate_structured_usd.py --simulate --device cuda:0 --world-count 4
```

The smoke test uses MuJoCo contacts and checks standing pelvis height, uprightness, finite body states, and all six loop-anchor separations throughout the run. It deliberately applies synthetic PD drives to passive DOFs as the G1 example does; it does not validate a controller restricted to H2's 31 motors. The CPU backend supports one simulated world.

Verified with Newton `3d437e20791c3114ec40b53ce478a7118a84eed7`, MuJoCo/MuJoCo-Warp 3.14.0, Warp 1.18.0, OpenUSD 26.5, and Newton USD schemas 0.5.0. The default-import CPU smoke test retained all six loop constraints, settled at a pelvis height of 1.036 m, and measured a maximum loop-anchor error of 1.31 mm. The original asset under the same default-import test fell around four seconds and settled at 0.101 m; adding the MuJoCo resolver to the original asset produced the same standing result as the fixed default import. CUDA execution was not verified on the test host because its GPU driver was unavailable.

## License

This model is released under a [BSD-3-Clause License](../LICENSE).
