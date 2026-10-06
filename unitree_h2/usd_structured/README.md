# Unitree H2 Closed-Loop Robot Simulation Asset

This asset is the standard structured output of `mujoco-usd-converter==0.5.0`, with three small compatibility adjustments described below. Load [`H2Loop.usda`](H2Loop.usda), whose default prim is `/H2Loop`.

The source is Unitree's [`robots/h2_description/H2_loop.xml`](https://github.com/unitreerobotics/unitree_ros/blob/7d6075f7f58588b189b940130e3edab3c839b2df/robots/h2_description/H2_loop.xml) at revision `7d6075f7f58588b189b940130e3edab3c839b2df`. It preserves the six ankle, knee, and waist linkage closures. Each closure retains the converter's `PhysicsSphericalJoint`, `physics:excludeFromArticulation = true`, and `MjcEqualityConnectAPI`; the tested Newton importer converts these to loop joints without removing the MuJoCo API.

## Standard asset structure

The package retains the converter's normal nested body hierarchy and layers, matching the other structured robot assets in this repository:

- `H2Loop.usda` is the lightweight asset interface, with the robot behind a payload.
- `Payload/Contents.usda` composes the geometry, materials, and physics layers.
- `Payload/Geometry.usda` contains the nested bodies and 59 mesh instances.
- `Payload/GeometryLibrary.usdc` contains 38 shared mesh definitions, including shared entries for all 21 visual/collision pairs.
- `Payload/Materials.usda` and `Payload/MaterialsLibrary.usdc` contain the converter's material definitions.
- `Payload/Physics.usda` contains USD, Newton, and MuJoCo physics attributes, joints, and the original 31 actuators together.

Nested rigid bodies require OpenUSD 25.11 or newer for full support. There is no custom flattened hierarchy, separate MuJoCo/Robot layer, or duplicate converted-source archive.

## Compatibility adjustments

[`finalize_usd.py`](../tools/finalize_usd.py) applies only these changes to the normal CLI output:

1. Remove the source scene's floor from the geometry, physics, and material-binding layers, and refresh the cached asset bounds. Consumers supply their own ground/environment.
2. Mark collision-only geometry with `purpose = "guide"`. Collision prims and their physics properties stay intact, while visual prims keep their visibility and appearance.
3. Author zero-gain USD drives whose `maxForce` preserves the source's symmetric joint effort limits in Newton's default import. These carry limits without prescribing a position controller or mixing passive damping with drive damping; they do not add MuJoCo actuators.

The converter already authors `NewtonJointAPI`, armature, friction, and correctly scaled passive damping. Those attributes, the original MuJoCo joint dynamics and equality metadata, the nested transforms, and both shared libraries are left unchanged.

## Reproduction

Use Python 3.12 and the pinned conversion dependencies:

```bash
python3.12 -m venv /tmp/h2-conversion-env
/tmp/h2-conversion-env/bin/python -m pip install -r unitree_h2/tools/conversion-requirements.txt

git clone https://github.com/unitreerobotics/unitree_ros.git /tmp/unitree_ros
git -C /tmp/unitree_ros checkout 7d6075f7f58588b189b940130e3edab3c839b2df

/tmp/h2-conversion-env/bin/mujoco_usd_converter \
  /tmp/unitree_ros/robots/h2_description/H2_loop.xml /tmp/h2-converted
# Validate against this directory before finalizing if using --converted-source.
/tmp/h2-conversion-env/bin/python unitree_h2/tools/finalize_usd.py /tmp/h2-converted
```

Use `H2_loop.xml`, which includes the physical linkages. The standard converter keeps `/H2Loop` and `H2Loop.usda`; consumers of the earlier PR revision should update their old `h2.usda` entrypoint and `/H2` paths.

[`SOURCE.json`](SOURCE.json) records the pinned source, conversion versions, MJCF/mesh hashes, and hashes of the unmodified converter output. [`BUILD.json`](BUILD.json) records hashes of the checked-in package after the adjustments. The finalizer updates this manifest; after changing documentation or provenance, rerun it to refresh the hashes. Source availability is required for regeneration; the 5.67 MB duplicate source snapshot has been removed.

The converter emits warnings that lights and the built-in checker texture are unsupported. Neither is needed for this robot asset; the scene floor is removed.

## Verification

With OpenUSD bindings installed, validate composition, targets, hashes, nested structure, loop APIs, shared geometry, and collision purpose:

```bash
python unitree_h2/tools/validate_structured_usd.py
```

For comparison with a fresh converter output, run the same converter command with `/tmp/h2-raw` as its output directory, without running the finalizer there. With MuJoCo installed, also verify source hashes, all body masses/inertia tensors/world poses, and loop endpoints/anchors against the pinned MJCF:

```bash
python unitree_h2/tools/validate_structured_usd.py \
  --converted-source /tmp/h2-raw \
  --mjcf /tmp/unitree_ros/robots/h2_description/H2_loop.xml
```

With Newton's USD import and simulation dependencies installed:

```bash
python unitree_h2/tools/validate_structured_usd.py --newton
python unitree_h2/tools/validate_structured_usd.py --simulate --device cpu
# On a CUDA-capable host:
python unitree_h2/tools/validate_structured_usd.py --simulate --device cuda:0 --world-count 4
```

The default-import check compares armature, passive damping, friction, and effort limits with the MuJoCo resolver. The ten-second smoke test uses the G1 example's import options, PD gains, six substeps/frame, and MuJoCo contacts. It checks finite body states, standing pelvis height, uprightness, and all six loop-anchor separations throughout the run. As in the G1 setup, it applies synthetic PD drives to passive DOFs; it does not validate a controller restricted to H2's 31 motors. The CPU backend supports one simulated world.

Verified with Newton `3d437e20791c3114ec40b53ce478a7118a84eed7`, MuJoCo/MuJoCo-Warp 3.14.0, Warp 1.18.0, OpenUSD 26.5, and Newton USD schemas 0.5.0. The default-import CPU test stayed upright, settled at a pelvis height of 1.036 m, and measured a maximum loop-anchor error of 1.35 mm. Import without MuJoCo custom-attribute registration also retained all six ball loop joints. A fresh conversion plus the compatibility patch reproduced the package byte for byte. GPU execution and Newton-contact simulations were not verified because the host's GPU driver was unavailable.

## License

This model is released under a [BSD-3-Clause License](../LICENSE).
