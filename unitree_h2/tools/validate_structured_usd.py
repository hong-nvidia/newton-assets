"""Validate the H2 package against its source, optionally importing and simulating it."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import tempfile
from pathlib import Path

from build_structured_usd import _flatten_bodies_and_joints, build
from pxr import Gf, Usd, UsdGeom, UsdPhysics


def validate_package(package: Path) -> None:
    """Check preservation, geometry sharing, USD composition, hashes, and rebuilding."""
    source_path = package / "Source/h2_import.usdc"
    source = Usd.Stage.Open(str(source_path))
    stage = Usd.Stage.Open(str(package / "h2.usda"))
    assert stage and not stage.GetCompositionErrors(), "USD composition failed"
    assert str(stage.GetDefaultPrim().GetPath()) == "/H2"
    assert source_path.resolve() not in {
        Path(layer.realPath).resolve()
        for layer in stage.GetUsedLayers()
        if layer.realPath
    }
    provenance = json.loads((package / "SOURCE.json").read_text())
    assert (
        hashlib.sha256(source_path.read_bytes()).hexdigest()
        == provenance["imported_source_sha256"]
    )
    manifest = json.loads((package / "BUILD.json").read_text())
    assert set(manifest["files"]) == {
        p.relative_to(package).as_posix()
        for p in package.rglob("*")
        if p.is_file() and p.name != "BUILD.json"
    }
    for name, digest in manifest["files"].items():
        assert hashlib.sha256((package / name).read_bytes()).hexdigest() == digest, name

    # Check world placement against the nested source, independently of normalization.
    source_cache, runtime_cache = UsdGeom.XformCache(), UsdGeom.XformCache()
    bodies = [p for p in source.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    assert len(bodies) == 44
    assert sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse()) == 44
    for body in bodies:
        runtime_body = stage.GetPrimAtPath("/H2/Geometry/" + body.GetName())
        assert Gf.IsClose(
            source_cache.GetLocalToWorldTransform(body),
            runtime_cache.GetLocalToWorldTransform(runtime_body),
            1e-10,
        ), body.GetName()
    assert sum(p.GetTypeName() == "MjcActuator" for p in stage.Traverse()) == 31
    loops = [
        p
        for p in stage.Traverse()
        if p.IsA(UsdPhysics.Joint)
        and p.GetAttribute("physics:excludeFromArticulation").Get()
    ]
    assert len(loops) == 6
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI) and UsdGeom.Imageable(prim):
            assert UsdGeom.Imageable(prim).ComputePurpose() == UsdGeom.Tokens.guide
        targets = [t for rel in prim.GetRelationships() for t in rel.GetTargets()]
        targets += [t for attr in prim.GetAttributes() for t in attr.GetConnections()]
        for target in targets:
            assert stage.GetObjectAtPath(target), (prim.GetPath(), target)

    library = Usd.Stage.Open(str(package / "Payload/GeometryLibrary.usdc"))
    assert sum(p.IsA(UsdGeom.Mesh) for p in library.Traverse()) == 38
    meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
    assert len(meshes) == 59
    collision_pairs = 0
    for mesh in meshes:
        if not mesh.HasAPI(UsdPhysics.CollisionAPI):
            continue
        assert UsdGeom.Imageable(mesh).ComputePurpose() == UsdGeom.Tokens.guide
        visual = mesh.GetParent().GetChild(mesh.GetName().removesuffix("_1"))
        if visual and visual != mesh and visual.IsA(UsdGeom.Mesh):
            assert not visual.HasAPI(UsdPhysics.CollisionAPI)
            assert UsdGeom.Imageable(visual).ComputePurpose() == UsdGeom.Tokens.default_
            assert mesh.GetMetadata("references") == visual.GetMetadata("references")
            collision_pairs += 1
    assert collision_pairs == 21

    with tempfile.TemporaryDirectory(prefix="h2-validation-") as temp:
        normalized = _flatten_bodies_and_joints(source_path, Path(temp) / "source.usda")
        # Compare every original attribute and relationship, including topology,
        # appearance, joint anchors, masses, inertia, and collision properties.
        for prim in normalized.Traverse():
            if str(prim.GetPath()).startswith("/Flattened_Prototype"):
                continue
            runtime = stage.GetPrimAtPath(prim.GetPath())
            assert runtime, prim.GetPath()
            for attr in prim.GetAuthoredAttributes():
                if attr.GetName() == "extentsHint":
                    # The entrypoint recomputes the converter's cached bounds.
                    bounds = (
                        UsdGeom.BBoxCache(
                            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
                        )
                        .ComputeWorldBound(runtime)
                        .ComputeAlignedBox()
                    )
                    low, high = runtime.GetAttribute(attr.GetName()).Get()
                    assert all(
                        low[i] <= bounds.GetMin()[i] + 1e-5
                        and high[i] >= bounds.GetMax()[i] - 1e-5
                        for i in range(3)
                    )
                    continue
                if attr.GetName() == "purpose" and prim.HasAPI(UsdPhysics.CollisionAPI):
                    continue
                assert attr.Get() == runtime.GetAttribute(attr.GetName()).Get(), (
                    attr.GetPath()
                )
            for rel in prim.GetAuthoredRelationships():
                assert (
                    rel.GetTargets()
                    == runtime.GetRelationship(rel.GetName()).GetTargets()
                ), rel.GetPath()
        rebuilt = Path(temp) / "package"
        shutil.copytree(package, rebuilt)
        build(rebuilt / "Source/h2_import.usdc", rebuilt)
        for name in manifest["files"]:
            assert (package / name).read_bytes() == (rebuilt / name).read_bytes(), name
    print(
        "PASS: source properties and 44 body transforms preserved; six loops and 31 actuators; 59 instances / 38 meshes / 21 shared pairs; hashes, targets, composition, and byte-for-byte rebuild"
    )


def import_robot(entrypoint: Path, *, mjc_only: bool = False):
    """Import with the G1 example's configuration and selected schema resolver."""
    import newton
    import warp as wp
    from newton.usd import SchemaResolverMjc

    builder = newton.ModelBuilder()
    newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
    builder.default_joint_cfg = newton.ModelBuilder.JointDofConfig(
        limit_ke=1e3, limit_kd=1e1, friction=1e-5
    )
    builder.default_shape_cfg.ke = 1e3
    builder.default_shape_cfg.kd = 2e2
    builder.default_shape_cfg.kf = 1e3
    builder.default_shape_cfg.mu = 0.75
    kwargs = {"schema_resolvers": [SchemaResolverMjc()]} if mjc_only else {}
    builder.add_usd(
        str(entrypoint),
        xform=wp.transform(wp.vec3(0, 0, 0.2)),
        collapse_fixed_joints=True,
        enable_self_collisions=False,
        hide_collision_shapes=True,
        skip_mesh_approximation=True,
        **kwargs,
    )
    return builder


def validate_import(package: Path):
    """Compare the default import with independently resolved MuJoCo joint values."""
    import numpy as np

    default = import_robot(package / "h2.usda")
    reference = import_robot(package / "h2.usda", mjc_only=True)
    assert default.joint_label == reference.joint_label
    for name in (
        "joint_armature",
        "joint_damping",
        "joint_friction",
        "joint_effort_limit",
    ):
        np.testing.assert_allclose(
            getattr(default, name),
            getattr(reference, name),
            rtol=1e-6,
            atol=1e-9,
            err_msg=name,
        )
    stage = Usd.Stage.Open(str(package / "h2.usda"))
    for index, label in enumerate(default.joint_label):
        prim = stage.GetPrimAtPath(label)
        if not prim or not prim.GetAttribute("mjc:armature").HasAuthoredValue():
            continue
        start, end = default.joint_qd_start[index : index + 2]
        np.testing.assert_allclose(
            default.joint_armature[start:end],
            prim.GetAttribute("mjc:armature").Get(),
            rtol=1e-6,
        )
        np.testing.assert_allclose(
            default.joint_damping[start:end],
            prim.GetAttribute("mjc:damping").Get(),
            rtol=1e-6,
        )
        np.testing.assert_allclose(
            default.joint_friction[start:end],
            prim.GetAttribute("mjc:frictionloss").Get(),
            rtol=1e-6,
        )
        limit = prim.GetAttribute("mjc:actuatorfrcrange:max").Get()
        drive = UsdPhysics.DriveAPI(prim, "angular")
        if limit is not None:
            np.testing.assert_allclose(
                default.joint_effort_limit[start:end], limit, rtol=1e-6
            )
            assert (
                drive
                and drive.GetStiffnessAttr().Get() == drive.GetDampingAttr().Get() == 0
            )
        else:
            assert not drive
    print(
        "PASS: default Newton import matches MuJoCo armature, passive damping, friction, and effort limits"
    )
    return default


def simulate(builder, world_count: int, device: str) -> None:
    """Run a ten-second G1-style posture test, including drives on passive DOFs."""
    import newton
    import numpy as np
    import warp as wp

    for index in range(6, builder.joint_dof_count):
        builder.joint_target_ke[index] = 500.0
        builder.joint_target_kd[index] = 10.0
        builder.joint_target_mode[index] = int(newton.JointTargetMode.POSITION)
    builder.approximate_meshes("bounding_box")
    worlds = newton.ModelBuilder()
    worlds.replicate(builder, world_count)
    worlds.default_shape_cfg.ke = 1e3
    worlds.default_shape_cfg.kd = 2e2
    worlds.add_ground_plane()
    model = worlds.finalize()
    solver = newton.solvers.SolverMuJoCo(
        model,
        use_mujoco_cpu=device == "cpu",
        separate_worlds=world_count > 1,
        use_mujoco_contacts=True,
        solver="newton",
        integrator="implicitfast",
        njmax=300,
        nconmax=150,
        cone="elliptic",
        impratio=100,
        iterations=100,
        ls_iterations=50,
    )
    state, next_state, control = model.state(), model.state(), model.control()
    assert solver.mj_model.neq == 6, "MuJoCo must retain all six loop constraints"
    newton.eval_fk(model, model.joint_q, model.joint_qd, state)
    pelvis = [
        i for i, label in enumerate(model.body_label) if label.endswith("/pelvis")
    ]
    loops = [
        i for i, label in enumerate(model.joint_label) if label.endswith("_connect")
    ]
    assert len(pelvis) == world_count and len(loops) == 6 * world_count
    parent, child = model.joint_parent.numpy(), model.joint_child.numpy()
    parent_anchor, child_anchor = model.joint_X_p.numpy(), model.joint_X_c.numpy()
    min_height, min_upright, max_error = math.inf, 1.0, 0.0
    for _ in range(600):
        for _ in range(6):
            state.clear_forces()
            solver.step(state, next_state, control, None, 1.0 / 360.0)
            state, next_state = next_state, state
        body = state.body_q.numpy()
        assert np.isfinite(body).all(), "Non-finite body state"
        min_height = min(min_height, float(body[pelvis, 2].min()))
        min_upright = min(
            min_upright,
            min(
                float(wp.quat_rotate(wp.quat(body[i, 3:]), wp.vec3(0, 0, 1))[2])
                for i in pelvis
            ),
        )
        for i in loops:
            a = wp.transform_point(
                wp.transform(body[parent[i], :3], body[parent[i], 3:]),
                wp.vec3(parent_anchor[i, :3]),
            )
            b = wp.transform_point(
                wp.transform(body[child[i], :3], body[child[i], 3:]),
                wp.vec3(child_anchor[i, :3]),
            )
            max_error = max(max_error, float(wp.length(a - b)))
    print(
        json.dumps(
            {
                "device": device,
                "world_count": world_count,
                "duration_s": 10,
                "pelvis_final_m": body[pelvis, 2].tolist(),
                "pelvis_min_m": min_height,
                "upright_min": min_upright,
                "loop_anchor_max_m": max_error,
            },
            indent=2,
        )
    )
    assert min_height > 0.9, "H2 fell or lost standing height"
    assert min_upright > 0.95, "H2 lost upright posture"
    assert max_error < 0.005, "Loop anchors separated by more than 5 mm"
    print("PASS: H2 stayed upright and all six loop closures stayed within 5 mm")


def main() -> None:
    """Run structural checks and optional Newton import and simulation checks."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--package",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "usd_structured",
    )
    parser.add_argument(
        "--newton", action="store_true", help="Also validate Newton's default import"
    )
    parser.add_argument(
        "--simulate", action="store_true", help="Also run a ten-second posture test"
    )
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--world-count", type=int, default=1)
    args = parser.parse_args()
    if args.world_count < 1:
        parser.error("--world-count must be positive")
    if args.simulate and args.device == "cpu" and args.world_count != 1:
        parser.error(
            "Newton's native MuJoCo CPU backend simulates one world; use --world-count 1 or a CUDA device"
        )
    validate_package(args.package.resolve())
    if args.newton or args.simulate:
        import warp as wp

        with wp.ScopedDevice(args.device):
            builder = validate_import(args.package.resolve())
            if args.simulate:
                simulate(builder, args.world_count, args.device)


if __name__ == "__main__":
    main()
