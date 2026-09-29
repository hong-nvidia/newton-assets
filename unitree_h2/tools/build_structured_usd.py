#!/usr/bin/env python3
"""Build the reviewable H2 USD package from the immutable imported stage."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


ROOT_PATH = Sdf.Path("/H2")
GEOMETRY_PATH = ROOT_PATH.AppendChild("Geometry")
JOINTS_PATH = ROOT_PATH.AppendChild("Joints")
PHYSICS_PATH = ROOT_PATH.AppendChild("Physics")
MATERIALS_PATH = ROOT_PATH.AppendChild("Materials")

PHYSICS_PREFIXES = ("physics:", "newton:", "physx:", "material:binding:physics")
MUJOCO_PREFIXES = ("mjc:",)
PHYSICS_APIS = ("Physics", "Newton", "Physx", "MaterialBinding")
MUJOCO_APIS = ("Mjc",)


def _stage_metadata(stage: Usd.Stage) -> None:
    """Apply package-wide stage metadata to a structured asset layer."""
    stage.SetDefaultPrim(stage.GetPrimAtPath(ROOT_PATH))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.GetRootLayer().customLayerData = {
        "creator": "unitree_h2/tools/build_structured_usd.py"
    }


def _new_stage(path: Path) -> Usd.Stage:
    """Create a structured asset layer with the standard H2 root prim."""
    stage = Usd.Stage.CreateNew(str(path))
    root = stage.DefinePrim(ROOT_PATH, "Xform")
    root.SetMetadata("kind", "component")
    _stage_metadata(stage)
    return stage


def _authored_api_names(prim: Usd.Prim) -> list[str]:
    """Return the API schemas explicitly authored on a prim."""
    op = prim.GetMetadata("apiSchemas")
    if not op:
        return []
    return list(op.GetAddedOrExplicitItems())


def _set_api_names(prim: Usd.Prim, names: list[str]) -> None:
    """Replace the API schemas explicitly authored on a prim."""
    if names:
        op = Sdf.TokenListOp()
        op.prependedItems = names
        prim.SetMetadata("apiSchemas", op)
    elif prim.HasMetadata("apiSchemas"):
        prim.ClearMetadata("apiSchemas")


def _copy_properties(
    source_layer: Sdf.Layer,
    source_prim: Usd.Prim,
    target_layer: Sdf.Layer,
    target_prim: Usd.Prim,
    prefixes: tuple[str, ...],
) -> None:
    """Copy authored properties whose names begin with selected prefixes."""
    for prop in source_prim.GetAuthoredProperties():
        if prop.GetName().startswith(prefixes):
            Sdf.CopySpec(source_layer, prop.GetPath(), target_layer, target_prim.GetPath().AppendProperty(prop.GetName()))


def _flatten_bodies_and_joints(source: Path, output: Path) -> Usd.Stage:
    """Normalize the source into flat body and joint namespaces."""
    imported = Usd.Stage.Open(str(source))
    if not imported:
        raise RuntimeError(f"Could not open source stage: {source}")
    imported.GetRootLayer().Export(str(output))
    stage = Usd.Stage.Open(str(output))
    source_root = stage.GetDefaultPrim()
    if not source_root:
        raise RuntimeError("Source stage has no default prim")
    source_root_path = source_root.GetPath()
    editor = Usd.NamespaceEditor(stage)
    if not editor.MovePrimAtPath(source_root_path, ROOT_PATH) or not editor.ApplyEdits():
        raise RuntimeError(f"Could not rename source root to {ROOT_PATH}")
    root = stage.GetPrimAtPath(ROOT_PATH)
    stage.SetDefaultPrim(root)
    root.SetAssetInfoByKey("name", "H2")
    stage.GetRootLayer().documentation = ""
    cache = UsdGeom.XformCache()
    bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    body_world = {str(p.GetPath()): cache.GetLocalToWorldTransform(p) for p in bodies}

    names = [p.GetName() for p in bodies]
    if len(names) != len(set(names)):
        raise RuntimeError("Rigid-body names are not unique; flattening would be ambiguous")

    # Move leaves first so every body becomes a direct child of Geometry.
    for old_path in sorted(body_world, key=lambda value: value.count("/"), reverse=True):
        prim = stage.GetPrimAtPath(old_path)
        if not prim:
            continue
        destination = GEOMETRY_PATH.AppendChild(prim.GetName())
        if prim.GetPath() == destination:
            continue
        editor = Usd.NamespaceEditor(stage)
        if not editor.MovePrimAtPath(prim.GetPath(), destination) or not editor.ApplyEdits():
            raise RuntimeError(f"Could not move {old_path} to {destination}")

    geometry_world = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(GEOMETRY_PATH))
    for old_path, world in body_world.items():
        destination = GEOMETRY_PATH.AppendChild(Sdf.Path(old_path).name)
        prim = stage.GetPrimAtPath(destination)
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(world * geometry_world.GetInverse())

    if not stage.GetPrimAtPath(JOINTS_PATH):
        stage.DefinePrim(JOINTS_PATH, "Scope")
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
    joint_names = [p.GetName() for p in joints]
    if len(joint_names) != len(set(joint_names)):
        raise RuntimeError("Joint names are not unique; flattening would be ambiguous")
    for joint in joints:
        destination = JOINTS_PATH.AppendChild(joint.GetName())
        editor = Usd.NamespaceEditor(stage)
        if not editor.MovePrimAtPath(joint.GetPath(), destination) or not editor.ApplyEdits():
            raise RuntimeError(f"Could not move {joint.GetPath()} to {destination}")

    stage.GetRootLayer().Save()
    return stage


def _build_geometry(flat: Usd.Stage, payload: Path) -> None:
    """Build the mesh library and its lightweight geometry layer."""
    library_path = payload / "GeometryLibrary.usdc"
    library = Usd.Stage.CreateNew(str(library_path))
    library.DefinePrim("/Geometry", "Scope")
    meshes = [p for p in flat.Traverse() if p.IsA(UsdGeom.Mesh)]
    mesh_names: set[str] = set()
    for index, mesh in enumerate(meshes):
        name = f"mesh_{index:03d}_{mesh.GetName()}"
        if name in mesh_names:
            raise RuntimeError(f"Duplicate mesh library name: {name}")
        mesh_names.add(name)
        library_prim_path = Sdf.Path("/Geometry").AppendChild(name)
        Sdf.CopySpec(flat.GetRootLayer(), mesh.GetPath(), library.GetRootLayer(), library_prim_path)
        library_prim = library.GetPrimAtPath(library_prim_path)
        for prop in list(library_prim.GetAuthoredProperties()):
            if prop.GetName().startswith(PHYSICS_PREFIXES + MUJOCO_PREFIXES):
                library_prim.RemoveProperty(prop.GetName())
        _set_api_names(
            library_prim,
            [
                api
                for api in _authored_api_names(library_prim)
                if not api.startswith(PHYSICS_APIS + MUJOCO_APIS)
            ],
        )
    library.GetRootLayer().Save()

    geometry_path = payload / "Geometry.usda"
    flat.GetRootLayer().Export(str(geometry_path))
    geometry = Usd.Stage.Open(str(geometry_path))
    geometry.RemovePrim(JOINTS_PATH)
    geometry.RemovePrim(PHYSICS_PATH)
    geometry.RemovePrim(MATERIALS_PATH)
    geometry.RemovePrim(Sdf.Path("/Flattened_Prototype_1"))

    for prim in list(geometry.Traverse()):
        if prim.IsA(UsdGeom.Mesh):
            source_index = next(i for i, p in enumerate(meshes) if str(p.GetPath()) == str(prim.GetPath()))
            name = f"mesh_{source_index:03d}_{prim.GetName()}"
            path = prim.GetPath()
            geometry.RemovePrim(path)
            instance = geometry.DefinePrim(path, "Mesh")
            instance.GetReferences().AddReference("./GeometryLibrary.usdc", Sdf.Path("/Geometry").AppendChild(name))
            continue
        for prop in list(prim.GetAuthoredProperties()):
            if prop.GetName().startswith(PHYSICS_PREFIXES + MUJOCO_PREFIXES):
                prim.RemoveProperty(prop.GetName())
        _set_api_names(
            prim,
            [name for name in _authored_api_names(prim) if not name.startswith(PHYSICS_APIS + MUJOCO_APIS)],
        )
    geometry.GetRootLayer().Save()


def _build_materials(flat: Usd.Stage, payload: Path) -> None:
    """Extract the asset's material hierarchy into its own layer."""
    stage = _new_stage(payload / "Materials.usda")
    if flat.GetPrimAtPath(MATERIALS_PATH):
        Sdf.CopySpec(flat.GetRootLayer(), MATERIALS_PATH, stage.GetRootLayer(), MATERIALS_PATH)
    stage.GetRootLayer().Save()


def _build_feature_layer(
    flat: Usd.Stage,
    path: Path,
    prefixes: tuple[str, ...],
    api_prefixes: tuple[str, ...],
    include_joints: bool,
    include_actuators: bool,
) -> None:
    """Build a layer containing one selected group of simulation features."""
    stage = _new_stage(path)
    source_layer = flat.GetRootLayer()
    if include_joints:
        Sdf.CopySpec(source_layer, JOINTS_PATH, stage.GetRootLayer(), JOINTS_PATH)
        for joint in [p for p in stage.Traverse() if p.GetPath().HasPrefix(JOINTS_PATH)]:
            for prop in list(joint.GetAuthoredProperties()):
                if prop.GetName().startswith(MUJOCO_PREFIXES):
                    joint.RemoveProperty(prop.GetName())
            _set_api_names(
                joint,
                [name for name in _authored_api_names(joint) if name.startswith(api_prefixes)],
            )
    if include_actuators:
        stage.DefinePrim(PHYSICS_PATH, "Scope")
    for prim in flat.Traverse():
        if include_joints and prim.GetPath().HasPrefix(JOINTS_PATH):
            continue
        if prim.GetTypeName() == "MjcActuator":
            if include_actuators:
                Sdf.CopySpec(source_layer, prim.GetPath(), stage.GetRootLayer(), prim.GetPath())
            continue
        properties = [p for p in prim.GetAuthoredProperties() if p.GetName().startswith(prefixes)]
        apis = [name for name in _authored_api_names(prim) if name.startswith(api_prefixes)]
        if not properties and not apis:
            continue
        if prim.GetPath().HasPrefix(PHYSICS_PATH):
            target = stage.DefinePrim(prim.GetPath(), prim.GetTypeName())
        else:
            target = stage.OverridePrim(prim.GetPath())
        _copy_properties(source_layer, prim, stage.GetRootLayer(), target, prefixes)
        _set_api_names(target, apis)
    if include_joints:
        scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(9.81)
        _set_api_names(scene.GetPrim(), ["NewtonSceneAPI"])
        scene.GetPrim().CreateAttribute("newton:maxSolverIterations", Sdf.ValueTypeNames.Int).Set(100)
        scene.GetPrim().CreateAttribute("newton:timeStepsPerSecond", Sdf.ValueTypeNames.Int).Set(500)
    elif include_actuators:
        scene = stage.OverridePrim("/PhysicsScene")
        _set_api_names(scene, ["MjcSceneAPI"])
        scene.CreateAttribute("mjc:compiler:angle", Sdf.ValueTypeNames.Token).Set("radian")
    stage.GetRootLayer().Save()


def _build_robot(flat: Usd.Stage, payload: Path) -> None:
    """Author Isaac robot metadata and body and joint relationships."""
    stage = _new_stage(payload / "Robot.usda")
    root = stage.OverridePrim(ROOT_PATH)
    _set_api_names(root, ["IsaacRobotAPI"])
    root.CreateAttribute("isaac:description", Sdf.ValueTypeNames.String).Set(
        "Unitree H2 closed-loop humanoid"
    )
    links = [p.GetPath() for p in flat.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    joints = [p.GetPath() for p in flat.Traverse() if p.IsA(UsdPhysics.Joint)]
    root.CreateRelationship("isaac:physics:robotLinks").SetTargets(links)
    root.CreateRelationship("isaac:physics:robotJoints").SetTargets(joints)
    stage.GetRootLayer().Save()


def _write_composition(output_root: Path, flat: Usd.Stage) -> None:
    """Write the payload composition and public H2 asset entry point."""
    payload = output_root / "Payload"
    contents = _new_stage(payload / "Contents.usda")
    contents.GetRootLayer().subLayerPaths = [
        "./Robot.usda",
        "./Mujoco.usda",
        "./Physics.usda",
        "./Materials.usda",
        "./Geometry.usda",
    ]
    contents.GetRootLayer().Save()

    interface = _new_stage(output_root / "h2.usda")
    root = interface.GetPrimAtPath(ROOT_PATH)
    _set_api_names(root, ["GeomModelAPI"])
    root.SetAssetInfoByKey("name", "H2")
    root.GetPayloads().AddPayload("./Payload/Contents.usda")
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(
        flat.GetPrimAtPath(ROOT_PATH)
    ).ComputeAlignedBox()
    root.CreateAttribute("extentsHint", Sdf.ValueTypeNames.Float3Array).Set(
        [Gf.Vec3f(bbox.GetMin()), Gf.Vec3f(bbox.GetMax())]
    )
    physics_scene = interface.DefinePrim("/PhysicsScene", "PhysicsScene")
    physics_scene.GetPayloads().AddPayload("./Payload/Contents.usda", "/PhysicsScene")
    interface.GetRootLayer().Save()


def build(source: Path, output_root: Path) -> None:
    """Generate the complete structured H2 package and hash manifest."""
    try:
        source_ref = source.relative_to(output_root.parent).as_posix()
    except ValueError:
        source_ref = source.as_posix()

    payload = output_root / "Payload"
    payload.mkdir(parents=True, exist_ok=True)
    work = payload / ".h2_flat_work.usda"
    flat = _flatten_bodies_and_joints(source, work)
    _build_geometry(flat, payload)
    _build_materials(flat, payload)
    _build_feature_layer(flat, payload / "Physics.usda", PHYSICS_PREFIXES, PHYSICS_APIS, True, False)
    _build_feature_layer(flat, payload / "Mujoco.usda", MUJOCO_PREFIXES, MUJOCO_APIS, False, True)
    _build_robot(flat, payload)
    _write_composition(output_root, flat)
    work.unlink()

    manifest_path = output_root / "BUILD.json"
    files = {}
    for path in sorted(output_root.rglob("*")):
        if path.is_file() and path != manifest_path:
            files[path.relative_to(output_root).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps({"source": source_ref, "files": files}, indent=2) + "\n")


def main() -> None:
    """Parse command-line arguments and build the requested asset package."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    build(args.source.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()
