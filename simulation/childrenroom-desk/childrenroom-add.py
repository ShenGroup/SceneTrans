# Headless version - adapted from kitchen_test_editor.py
# Suitable for running in containers / non-GUI environments
# Run via ./python.sh kitchen_test_headless.py
# Features: randomize 3 bottle prims, placed on two counter surfaces
# Pair differences come from different randomized positions and bottle tip-over states

from isaacsim import SimulationApp

# 1. Create SimulationApp (headless)
simulation_app = SimulationApp(launch_config={"headless": True})

import os
import random
import json
import argparse
from pathlib import Path

import omni.replicator.core as rep
import omni.usd
from pxr import Sdf, UsdGeom, UsdPhysics, Gf
import omni.physx
import omni.syntheticdata as syn
import numpy as np


def run_kitchen_example(
    pair_count: int = 4,
    warmup_k: int = 3,
    resolution=(1024, 768),
    focal_length: float | None = None,
    output_dir: str = "/workspace/output/childrenroom_desk_add/3_items/120-139",
):
    """
    Headless version of the kitchen scene example
    Only randomizes the specified 3 bottle prims on the counter surfaces
    Each randomization randomly decides whether the bottle is tipped over (rotate 0 or 90 degrees around X axis)
    """

    # 2. Open the kitchen scene
    usd_path = "/workspace/assets/Interactive_scene/childrenroom/Interactive_childrenroom.usd"
    print(f"[kitchen_headless] Opening stage: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)

    stage = omni.usd.get_context().get_stage()

    if not stage:
        print("[kitchen_headless] Error: Failed to open stage!")
        return

    # Keep the scene as is, do not modify any lighting settings
    print("[kitchen_headless] Scene loaded, preserving original lighting")

    # 3.2 Remove the physics scene and disable global physics to avoid being pushed by physics after randomization
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene":
            stage.RemovePrim(prim.GetPath())
            print(f"[kitchen_headless] Removed PhysicsScene: {prim.GetPath()}")

    import carb.settings
    carb.settings.get_settings().set("/physics/enabled", False)
    print("[kitchen_headless] Physics disabled via carb settings")

    # 3. Configure render settings to eliminate ghosting caused by temporal accumulation
    # RTSubframes: force the renderer to resample every frame
    rep.settings.carb_settings("/omni/replicator/RTSubframes", 64)
    print("[kitchen_headless] RTSubframes set to 16 (to eliminate ghosting)")



    # 4. Pair with the on_frame trigger so the orchestrator drives writing to disk
    rep.orchestrator.set_capture_on_play(True)

    # 4. Find an existing Camera in the scene, using OmniverseKit_Persp
    desired_camera = "/OmniverseKit_Persp"
    camera_list = []
    
    cam_prim = stage.GetPrimAtPath(desired_camera)
    if cam_prim and cam_prim.IsValid():
        camera_list = [desired_camera]
        print(f"[kitchen_headless] Using camera: {desired_camera}")
    else:
        # Try to find another camera in the scene as a fallback
        print(f"[kitchen_headless] Warning: Specified camera not found: {desired_camera}")
        for prim in stage.Traverse():
            if prim.GetTypeName() == "Camera":
                camera_list = [prim.GetPath().pathString]
                print(f"[kitchen_headless] Fallback to camera: {camera_list[0]}")
                break

    if not camera_list:
        print("[kitchen_headless] No camera found, creating a default one.")
        camera_prim = stage.DefinePrim("/World/Camera", "Camera")
        camera_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Float3).Set((0.0, 150.0, 600.0))
        camera_prim.CreateAttribute("focalLength", Sdf.ValueTypeNames.Float).Set(35.0)
        camera_list = ["/World/Camera"]

    # Optional: adjust the camera focal length
    if focal_length is not None:
        for cam_path in camera_list:
            cam_prim = stage.GetPrimAtPath(cam_path)
            cam_schema = UsdGeom.Camera(cam_prim)
            if cam_schema and cam_schema.GetFocalLengthAttr():
                cam_schema.GetFocalLengthAttr().Set(float(focal_length))
                print(f"[kitchen_headless] Set focalLength={focal_length} for {cam_path}")

    # 5. BasicWriter: output to /workspace/output/kitchen_headless
    writer = rep.writers.get("BasicWriter")
    out_dir = Path(output_dir)

    # 6. Only randomize the specified 3 bottle prims
    def find_bottle_prims():
        """Return only the 3 bottle prims specified by the user"""
        specific_prims = [
            # "/World/model_computer_9",
            "/World/light1",
            # "/World/light2",
            "/World/model_book2_05",
            "/World/model_electric_fan_8",
        ]
        
        # Verify whether these prims exist
        valid_prims = []
        for path in specific_prims:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                valid_prims.append(path)
                print(f"[kitchen_headless] Found specified prim: {path}")
            else:
                print(f"[kitchen_headless] Warning: Specified prim not found: {path}")
        
        return valid_prims

    # 7. Check and remove the colliders from the objects (randomized objects don't need CollisionAPI)
    def remove_collision_from_bottles(bottle_paths):
        """Check and remove CollisionAPI from objects; randomized objects don't need colliders"""
        for path in bottle_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                print(f"[kitchen_headless] ❌ Prim not found: {path}")
                continue

            removed_count = 0

            # Disable instanceable
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                print(f"[kitchen_headless] Made prim non-instanceable: {path}")

            # Check and remove the CollisionAPI from the Xform itself
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
                print(f"[kitchen_headless] ✓ Removed CollisionAPI from Xform: {path}")
                removed_count += 1
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                print(f"[kitchen_headless] ✓ Removed RigidBodyAPI from Xform: {path}")
                removed_count += 1

            # Recursively check CollisionAPI on all child nodes
            def remove_collision_from_children(parent_prim, depth=0):
                nonlocal removed_count
                if depth > 10:
                    return
                for child in parent_prim.GetChildren():
                    child_path = child.GetPath().pathString
                    child_type = child.GetTypeName()

                    # Check and remove CollisionAPI
                    if child.HasAPI(UsdPhysics.CollisionAPI):
                        child.RemoveAPI(UsdPhysics.CollisionAPI)
                        print(f"[kitchen_headless] ✓ Removed CollisionAPI from {child_type}: {child_path}")
                        removed_count += 1
                    if child.HasAPI(UsdPhysics.RigidBodyAPI):
                        child.RemoveAPI(UsdPhysics.RigidBodyAPI)
                        print(f"[kitchen_headless] ✓ Removed RigidBodyAPI from {child_type}: {child_path}")
                        removed_count += 1

                    # Continue recursing into child nodes
                    remove_collision_from_children(child, depth + 1)
            
            remove_collision_from_children(prim)
            
            if removed_count > 0:
                print(f"[kitchen_headless] Physics cleanup done: {path} (removed {removed_count} CollisionAPI)")
            else:
                print(f"[kitchen_headless] No CollisionAPI found on: {path}")

    # 8. Add semantic labels to the randomized objects (used for instance/semantic segmentation)
    def add_semantic_labels(prim_paths):
        """
        Add semantic labels to the specified prims; the label name is the last segment of the path
        Use both approaches to ensure compatibility:
        1. Write semantic attributes directly via USD primvars (takes effect immediately)
        2. Use rep.modify.semantics (used by the Replicator graph)
        """
        print("[kitchen_headless] Adding semantic labels using dual approach...")
        
        added_count = 0

        def clear_old_semantics(prim):
            for attr in list(prim.GetAttributes()):
                if attr.GetName().startswith("primvars:semantics:"):
                    prim.RemoveProperty(attr.GetName())

        def set_semantic_primvar(prim, prim_name: str):
            # Use the UsdGeom primvar API with token array type and constant interpolation
            clear_old_semantics(prim)
            primvar = UsdGeom.PrimvarsAPI(prim).CreatePrimvar(
                "semantics:class", Sdf.ValueTypeNames.TokenArray, UsdGeom.Tokens.constant
            )
            primvar.Set([prim_name])

        def collect_mesh_children(parent_prim, depth=0, max_depth=6):
            if depth > max_depth:
                return []
            meshes = []
            for child in parent_prim.GetChildren():
                child_type = child.GetTypeName()
                if child_type == "Mesh":
                    meshes.append(child)
                elif child_type in ["Xform", "Scope", ""]:
                    meshes.extend(collect_mesh_children(child, depth + 1, max_depth))
            return meshes
        
        for path in prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[kitchen_headless] ❌ Prim not found: {path}")
                continue
            
            # Automatically assign a semantic label based on the prim path (use the last segment)
            segments = [seg for seg in path.split("/") if seg]
            prim_name = segments[-1] if segments else "object"

            try:
                # Disable instanceable to ensure primvars can be written
                if prim.IsInstanceable():
                    prim.SetInstanceable(False)

                # Write primvars on the Xform
                set_semantic_primvar(prim, prim_name)

                # Write primvars on the child Meshes so segmentation can read them
                mesh_children = collect_mesh_children(prim)
                for mesh_prim in mesh_children:
                    if mesh_prim.IsInstanceable():
                        mesh_prim.SetInstanceable(False)
                    set_semantic_primvar(mesh_prim, prim_name)

                # Add semantic labels via rep.modify.semantics
                prim_group = rep.get.prims(path_pattern=path)
                with prim_group:
                    rep.modify.semantics([("class", prim_name)])

                added_count += 1
                print(f"[kitchen_headless] ✓ Semantic label added: {path} -> '{prim_name}' (meshes: {len(mesh_children)})")

                # Print verification (root + first mesh)
                def log_semantic_attrs(target_prim, label):
                    attrs = [
                        (attr.GetName(), attr.Get())
                        for attr in target_prim.GetAttributes()
                        if "semantic" in attr.GetName().lower()
                    ]
                    if attrs:
                        for name, val in attrs:
                            print(f"[kitchen_headless]   {label} attr {name} = {val}")
                    else:
                        print(f"[kitchen_headless]   {label} has no semantic attrs")

                log_semantic_attrs(prim, f"{path}")
                if mesh_children:
                    log_semantic_attrs(mesh_children[0], f"{mesh_children[0].GetPath().pathString}")
                
            except Exception as e:
                print(f"[kitchen_headless] ❌ Failed to add label to {path}: {e}")
        
        return added_count

    # 9. Find and verify prims
    candidate_paths = find_bottle_prims()
    
    if not candidate_paths:
        print("[kitchen_headless] No specified prims were found; cannot proceed.")
        return
    
    print(f"[kitchen_headless] Will randomize {len(candidate_paths)} prims")

    # 9.1 Resolve the movable prim (determined via bbox changes)
    def resolve_movable_prim(path, max_depth=6, delta=1.0):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            return None

        def candidate_prims(root_prim):
            candidates = []
            queue = [(root_prim, 0)]
            while queue:
                current, depth = queue.pop(0)
                if depth > max_depth:
                    continue
                if current.GetName() == "Looks":
                    continue
                if current.GetTypeName() in ["Xform", "Mesh", "Scope", ""]:
                    candidates.append(current)
                for child in current.GetChildren():
                    queue.append((child, depth + 1))
            return candidates

        def bbox_min_z(target_prim):
            bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
            world_bbox = bbox_cache.ComputeWorldBound(target_prim)
            if not world_bbox or world_bbox.GetRange().IsEmpty():
                return None
            return world_bbox.GetRange().GetMin()[2]

        for cand in candidate_prims(prim):
            xformable = UsdGeom.Xformable(cand)
            if not xformable:
                continue
            before_z = bbox_min_z(cand)
            if before_z is None:
                continue

            ops = xformable.GetOrderedXformOps()
            translate_op = None
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break

            created = False
            if translate_op:
                old = translate_op.Get()
                old = old if old is not None else Gf.Vec3d(0, 0, 0)
            else:
                translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
                old = Gf.Vec3d(0, 0, 0)
                created = True

            translate_op.Set(Gf.Vec3d(old[0] + delta, old[1], old[2]))
            after_z = bbox_min_z(cand)

            # Restore
            translate_op.Set(old)
            if created:
                try:
                    xformable.RemoveXformOp(translate_op)
                except Exception:
                    pass

            if after_z is not None and abs(after_z - before_z) > 1e-4:
                return cand.GetPath().pathString

        return path

    prim_path_map = {}
    for path in candidate_paths:
        movable = resolve_movable_prim(path)
        if movable:
            prim_path_map[path] = movable
            if movable != path:
                print(f"[kitchen_headless] Using movable prim: {path} -> {movable}")
        else:
            print(f"[kitchen_headless] Warning: No movable prim resolved for {path}")

    if not prim_path_map:
        print("[kitchen_headless] No movable prims resolved; cannot proceed.")
        return

    reverse_prim_map = {movable: orig for orig, movable in prim_path_map.items()}
    movable_paths = list(prim_path_map.values())
    
    # Remove physics colliders from these prims (if any)
    remove_collision_from_bottles(candidate_paths)

    # Apply semantic labels to all objects to be randomized
    added = add_semantic_labels(candidate_paths)
    print(f"[kitchen_headless] Semantic labels added: {added} prims labeled")

    # 10. Use the specified plane for random placement
    cabinet_prims = [
        "World/Plane",
    ]

    def normalize_prim_path(path: str) -> str:
        """Ensure the path starts with /, accommodating inputs without a leading slash"""
        return path if path.startswith("/") else f"/{path}"

    # Verify whether the cabinet prims exist
    valid_cabinet_prims = []
    for cabinet_prim in cabinet_prims:
        cabinet_prim = normalize_prim_path(cabinet_prim)
        cabinet_prim_obj = stage.GetPrimAtPath(cabinet_prim)
        if cabinet_prim_obj and cabinet_prim_obj.IsValid():
            valid_cabinet_prims.append(cabinet_prim)
            print(f"[kitchen_headless] ✓ Found cabinet prim: {cabinet_prim}")
        else:
            print(f"[kitchen_headless] ❌ Warning: Cabinet prim not found: {cabinet_prim}")
    
    if not valid_cabinet_prims:
        print(f"[kitchen_headless] ❌ Error: No valid cabinet prims found!")
        return
    
    # 10.1 Get the surface Z height (used for Z compensation)
    def get_surface_z_height(surface_prim_paths):
        """
        Get the highest Z coordinate of the surface, used as the reference height for object placement
        """
        print(f"\n[Surface-Z] ========== Computing surface Z height ==========")
        max_z = float('-inf')
        for path in surface_prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[Surface-Z] ❌ Invalid prim: {path}")
                continue

            print(f"[Surface-Z] Checking: {path}, type: {prim.GetTypeName()}")

            bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
            world_bbox = bbox_cache.ComputeWorldBound(prim)

            if world_bbox and not world_bbox.GetRange().IsEmpty():
                bbox_min = world_bbox.GetRange().GetMin()
                bbox_max = world_bbox.GetRange().GetMax()
                print(f"[Surface-Z]   bbox: min=({bbox_min[0]:.4f}, {bbox_min[1]:.4f}, {bbox_min[2]:.4f}), max=({bbox_max[0]:.4f}, {bbox_max[1]:.4f}, {bbox_max[2]:.4f})")
                bbox_max_z = bbox_max[2]
                if bbox_max_z > max_z:
                    max_z = bbox_max_z
            else:
                print(f"[Surface-Z]   ⚠️ Unable to compute bbox or bbox is empty")

                # Try getting transform as a fallback
                xformable = UsdGeom.Xformable(prim)
                if xformable:
                    world_transform = xformable.ComputeLocalToWorldTransform(0)
                    translation = world_transform.ExtractTranslation()
                    print(f"[Surface-Z]   Transform position: ({translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f})")
                    # Use the transform's Z as a fallback
                    if translation[2] > max_z:
                        max_z = translation[2]
                        print(f"[Surface-Z]   Using transform Z as surface height: {translation[2]:.4f}")

        result = max_z if max_z != float('-inf') else 0.0
        print(f"[Surface-Z] Final surface Z height: {result}")
        print(f"[Surface-Z] ==========================================\n")
        return result

    # Get the surface height (take the max of surface_z from bounds_list)
    # Note: get_surface_z_height may return 0, so we update it after computing bounds_list
    surface_z_height = get_surface_z_height(valid_cabinet_prims)

    # 10.2 Z compensation function: called after randomization + rotation, to prevent objects sinking into the plane
    def apply_z_compensation(prim_surface_z_map, prim_display_map=None):
        """
        Based on the object's current pose bounding box, compensate the Z height so the bottom rests on the surface

        Core logic:
        1. Get the current Z value of the object's translate op
        2. Compute the gap between the bbox bottom and the target surface Z
        3. Adjust Z so that bbox bottom = surface_z of the plane this object is on

        Args:
            prim_surface_z_map: {prim_path: surface_z} the plane Z height each object is placed on
        """
        if not prim_surface_z_map:
            print(f"\n[Z-comp] ⚠️ No objects need compensation")
            return

        print(f"\n[Z-comp] ========== Starting Z compensation ==========")

        bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])

        for path, target_surface_z in prim_surface_z_map.items():
            display_name = prim_display_map.get(path, path) if prim_display_map else path
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[Z-comp] ❌ Invalid prim: {display_name}")
                continue

            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                print(f"[Z-comp] ❌ Unable to get Xformable: {display_name}")
                continue

            # Get the current translate op
            ops = xformable.GetOrderedXformOps()
            translate_op = None
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break

            if not translate_op:
                print(f"[Z-comp] ⚠️ {display_name}: no translate op, skipping")
                continue

            current_pos = translate_op.Get()
            if not current_pos:
                print(f"[Z-comp] ⚠️ {display_name}: translate op has no value")
                continue

            current_z = current_pos[2]

            # Compute the world bounding box at the current pose
            bbox_cache.Clear()
            world_bbox = bbox_cache.ComputeWorldBound(prim)
            if not world_bbox or world_bbox.GetRange().IsEmpty():
                print(f"[Z-comp] ⚠️ Unable to compute bbox: {display_name}")
                continue

            bbox_min_z = world_bbox.GetRange().GetMin()[2]

            # Compute required Z adjustment: align the bbox bottom to the surface_z of the plane this object is on
            z_adjustment = target_surface_z - bbox_min_z
            new_z = current_z + z_adjustment

            print(f"[Z-comp] {display_name}: target_z={target_surface_z:.4f}, current_z={current_z:.4f}, bbox_min_z={bbox_min_z:.4f}")
            print(f"[Z-comp]   adjustment={z_adjustment:.4f}, new_z={new_z:.4f}")

            # Set the new position
            new_pos = Gf.Vec3d(current_pos[0], current_pos[1], new_z)
            translate_op.Set(new_pos)

            # Verify
            bbox_cache.Clear()
            new_bbox = bbox_cache.ComputeWorldBound(prim)
            if new_bbox:
                new_min_z = new_bbox.GetRange().GetMin()[2]
                print(f"[Z-comp] ✓ Verify: new bbox_min_z = {new_min_z:.4f} (expected {target_surface_z:.4f})")

        print(f"[Z-comp] ========== Z compensation done ==========\n")

    # Whether to enable Z compensation (can be turned off if only forcing the Z value)
    use_z_compensation = False

    # 11. Find the actual Mesh prim (scatter_2d requires a direct Mesh, not a container)
    def find_actual_mesh(prim_path, depth=0, max_depth=3):
        """
        Recursively look for the actual Mesh child node under a prim
        Because scatter_2d requires a direct Mesh prim, not a container that contains Meshes
        """
        if depth > max_depth:
            return None
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return None
        if prim.GetTypeName() == "Mesh":
            return prim_path
        for child in prim.GetChildren():
            child_path = child.GetPath().pathString
            if "Looks" in child.GetName() or child.GetTypeName() == "Scope":
                continue
            if child.GetTypeName() == "Mesh":
                return child_path
            if child.GetTypeName() in ["Xform", "Scope", ""]:
                result = find_actual_mesh(child_path, depth + 1, max_depth)
                if result:
                    return result
        return None
    
    # Find the actual mesh prims for all cabinets
    actual_cabinet_meshes = []
    for cabinet_prim in valid_cabinet_prims:
        actual_mesh = find_actual_mesh(cabinet_prim)
        if actual_mesh:
            actual_cabinet_meshes.append(actual_mesh)
            print(f"[kitchen_headless] Using mesh for scatter_2d: {actual_mesh}")
        else:
            # If no child Mesh is found, try to use this prim directly
            print(f"[kitchen_headless] Will try using prim directly: {cabinet_prim}")
            actual_cabinet_meshes.append(cabinet_prim)

    if not actual_cabinet_meshes:
        print("[kitchen_headless] ❌ Error: No valid meshes found for scatter_2d")
        return

    # Get the cabinet prims as the surface for scatter_2d
    surface_nodes = []
    for mesh_path in actual_cabinet_meshes:
        prim_node = rep.get.prims(path_pattern=mesh_path)
        if prim_node:
            surface_nodes.append(prim_node)
            print(f"[kitchen_headless] Added surface: {mesh_path}")
    
    if not surface_nodes:
        print("[kitchen_headless] ❌ Error: No valid surface found for scatter_2d")
        return
    
    # If there are multiple surfaces, create a group; otherwise use the single surface directly
    if len(surface_nodes) == 1:
        surface = surface_nodes[0]
    else:
        surface = rep.create.group(surface_nodes)
        print(f"[kitchen_headless] Created surface group with {len(surface_nodes)} meshes")

    # 11.5 Compute the world coordinate bounds of each surface (used for USD API randomization)
    def get_each_surface_bounds(surface_prim_paths):
        """
        Get the world coordinate bounds of each surface (stored separately, not merged)
        Use the world transform matrix + the Mesh's extent attribute to compute bounds precisely
        Returns: [(min_x, max_x, min_y, max_y, surface_z), ...]
        """
        surface_bounds_list = []

        print(f"\n[Surface-Bounds] ========== Computing bounds of each surface ==========")

        def find_mesh_in_prim(parent_prim, depth=0, max_depth=5):
            """Recursively find a Mesh child node under a prim"""
            if depth > max_depth:
                return None
            if parent_prim.GetTypeName() == "Mesh":
                return parent_prim
            for child in parent_prim.GetChildren():
                if child.GetTypeName() == "Mesh":
                    return child
                result = find_mesh_in_prim(child, depth + 1, max_depth)
                if result:
                    return result
            return None
        
        for path in surface_prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[Surface-Bounds] ❌ Invalid prim: {path}")
                continue

            print(f"[Surface-Bounds] Checking: {path}, type: {prim.GetTypeName()}")

            # First print the prim's world position (for diagnostics)
            xformable = UsdGeom.Xformable(prim)
            if xformable:
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                translation = world_transform.ExtractTranslation()
                print(f"[Surface-Bounds]   World position: ({translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f})")

            # Find the actual Mesh prim
            mesh_prim = find_mesh_in_prim(prim)
            target_prim = mesh_prim if mesh_prim else prim
            target_path = target_prim.GetPath().pathString
            print(f"[Surface-Bounds]   Using prim: {target_path}, type: {target_prim.GetTypeName()}")

            # Method 1: try to get the Mesh's extent attribute
            extent_attr = target_prim.GetAttribute("extent")
            local_extent = None
            if extent_attr and extent_attr.HasValue():
                local_extent = extent_attr.Get()
                if local_extent and len(local_extent) >= 2:
                    print(f"[Surface-Bounds]   Local extent: min={local_extent[0]}, max={local_extent[1]}")

            # Get the world transform matrix
            target_xformable = UsdGeom.Xformable(target_prim)
            if not target_xformable:
                print(f"[Surface-Bounds] ❌ Unable to get Xformable: {target_path}")
                continue

            world_transform = target_xformable.ComputeLocalToWorldTransform(0)

            # If there is extent, use its four corners to compute the world bounds
            if local_extent and len(local_extent) >= 2:
                local_min = local_extent[0]
                local_max = local_extent[1]

                # Build 8 corner points in local space (for 3D box) or 4 corner points (for a plane)
                local_corners = [
                    Gf.Vec3d(local_min[0], local_min[1], local_min[2]),
                    Gf.Vec3d(local_max[0], local_min[1], local_min[2]),
                    Gf.Vec3d(local_min[0], local_max[1], local_min[2]),
                    Gf.Vec3d(local_max[0], local_max[1], local_min[2]),
                    Gf.Vec3d(local_min[0], local_min[1], local_max[2]),
                    Gf.Vec3d(local_max[0], local_min[1], local_max[2]),
                    Gf.Vec3d(local_min[0], local_max[1], local_max[2]),
                    Gf.Vec3d(local_max[0], local_max[1], local_max[2]),
                ]
            else:
                # Fallback: assume a unit plane
                print(f"[Surface-Bounds]   ⚠️ No extent, falling back to unit plane assumption")
                local_corners = [
                    Gf.Vec3d(-0.5, -0.5, 0),
                    Gf.Vec3d(0.5, -0.5, 0),
                    Gf.Vec3d(-0.5, 0.5, 0),
                    Gf.Vec3d(0.5, 0.5, 0),
                ]

            # Transform to world coordinates and compute the bounds
            min_x, max_x = float('inf'), float('-inf')
            min_y, max_y = float('inf'), float('-inf')
            max_z = float('-inf')
            
            for corner in local_corners:
                world_corner = world_transform.TransformAffine(corner)
                min_x = min(min_x, world_corner[0])
                max_x = max(max_x, world_corner[0])
                min_y = min(min_y, world_corner[1])
                max_y = max(max_y, world_corner[1])
                max_z = max(max_z, world_corner[2])
            
            surface_z = max_z

            print(f"[Surface-Bounds]   Computed bounds: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}], Z={surface_z:.2f}")

            # Add 5% margin
            range_x = max_x - min_x
            range_y = max_y - min_y
            margin_x = range_x * 0.05
            margin_y = range_y * 0.1
            min_x += margin_x
            max_x -= margin_x
            min_y += margin_y
            max_y -= margin_y

            # Use the bounds with margin
            bounds = (min_x, max_x, min_y, max_y, surface_z)
            surface_bounds_list.append(bounds)

            print(f"[Surface-Bounds]   Final bounds: X=[{bounds[0]:.2f}, {bounds[1]:.2f}], Y=[{bounds[2]:.2f}, {bounds[3]:.2f}], Z={surface_z:.2f}")

        print(f"[Surface-Bounds] {len(surface_bounds_list)} valid surfaces in total")
        print(f"[Surface-Bounds] ==========================================\n")

        return surface_bounds_list

    # Get the bounds list of each surface
    surface_bounds_list = get_each_surface_bounds(valid_cabinet_prims)

    # Get the correct surface_z_height from bounds_list (take the max)
    # This is more reliable than get_surface_z_height
    if surface_bounds_list:
        surface_z_height = max(bounds[4] for bounds in surface_bounds_list)
        print(f"[kitchen_headless] ✓ surface_z_height updated from bounds_list to: {surface_z_height:.4f}")

    # 11.6 Pure USD API randomization function (full control, does not rely on the Replicator trigger)
    import random as py_random

    def manual_randomize_objects(prim_path_map, bounds_list):
        """
        Randomize object positions and rotations using the pure USD API
        Each object randomly selects a plane, then is placed within that plane's bounds

        Args:
            prim_path_map: {original_path: movable_path}
            bounds_list: list of surface bounds [(min_x, max_x, min_y, max_y, surface_z), ...]

        Returns:
            dict: {prim_path: surface_z} the plane Z height each object is placed on
        """
        prim_surface_z_map = {}

        if not bounds_list:
            print(f"[Manual-Rand] ❌ No valid surface bounds")
            return prim_surface_z_map

        print(f"\n[Manual-Rand] ========== Starting USD API randomization ==========")
        print(f"[Manual-Rand] Number of available surfaces: {len(bounds_list)}")

        for orig_path, move_path in prim_path_map.items():
            prim = stage.GetPrimAtPath(move_path)
            if not prim or not prim.IsValid():
                print(f"[Manual-Rand] ❌ Invalid prim: {move_path} (orig {orig_path})")
                continue

            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                print(f"[Manual-Rand] ❌ Unable to get Xformable: {path}")
                continue

            # Randomly select a plane
            selected_bounds = py_random.choice(bounds_list)
            min_x, max_x, min_y, max_y, surface_z = selected_bounds
            surface_idx = bounds_list.index(selected_bounds)

            # Record the surface Z height this object is placed on
            prim_surface_z_map[move_path] = surface_z

            # Randomly generate a position (within the bounds of the selected plane)
            rand_x = py_random.uniform(min_x, max_x)
            rand_y = py_random.uniform(min_y, max_y)
            # Force Z directly, do not rely on bbox compensation
            rand_z = surface_z

            # Random rotation: X axis 0 or 90 degrees (standing or tipped over), Z axis 0-360 degrees
            rand_rot_x = py_random.choice([0, 0])
            rand_rot_y = 0
            rand_rot_z = py_random.uniform(0, 360)

            # Get or create xform ops
            ops = xformable.GetOrderedXformOps()
            translate_op = None
            rotate_op = None

            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                    rotate_op = op

            # Set position
            if translate_op:
                translate_op.Set(Gf.Vec3d(rand_x, rand_y, rand_z))
            else:
                translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
                translate_op.Set(Gf.Vec3d(rand_x, rand_y, rand_z))

            # Set rotation
            if rotate_op:
                rotate_op.Set(Gf.Vec3f(rand_rot_x, rand_rot_y, rand_rot_z))
            else:
                rotate_op = xformable.AddRotateXYZOp(UsdGeom.XformOp.PrecisionFloat)
                rotate_op.Set(Gf.Vec3f(rand_rot_x, rand_rot_y, rand_rot_z))

            state = "standing" if rand_rot_x == 0 else "tipped over"

            # Verify whether the position is within the bounds
            in_range_x = min_x <= rand_x <= max_x
            in_range_y = min_y <= rand_y <= max_y
            if not in_range_x or not in_range_y:
                print(f"[Manual-Rand] ❌ {path}: position out of bounds!")
                print(f"[Manual-Rand]   bounds: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}]")
                print(f"[Manual-Rand]   pos: X={rand_x:.2f} {'✓' if in_range_x else '❌'}, Y={rand_y:.2f} {'✓' if in_range_y else '❌'}")

            print(f"[Manual-Rand] ✓ {orig_path}: plane{surface_idx}(Z={surface_z:.2f}), bounds=X[{min_x:.2f},{max_x:.2f}],Y[{min_y:.2f},{max_y:.2f}], pos=({rand_x:.2f}, {rand_y:.2f}, {rand_z:.2f}), rot=({rand_rot_x}, {rand_rot_y}, {rand_rot_z:.0f}) [{state}]")

        print(f"[Manual-Rand] ========== Randomization complete ==========\n")
        return prim_surface_z_map

    # 12. Render for each camera
    for cam_path in camera_list:
        print(f"\n[kitchen_headless] ========== Rendering camera: {cam_path} ==========")

        render_product = rep.create.render_product(cam_path, resolution)

        cam_out_dir = out_dir
        raw_dir = cam_out_dir / "_raw_frames"
        os.makedirs(raw_dir, exist_ok=True)
        
        # First run warmup frames to flush the temporal cache (remove afterimages of the original position)
        # warmup frames = 3 * warmup_k, ensuring it is a multiple of 3
        warmup_frames = 3 * warmup_k
        print(f"[kitchen_headless] Running {warmup_frames} warmup frames (k={warmup_k}) with randomization...")
        rep.orchestrator.set_capture_on_play(False)

        # During warmup, move the bottles to random positions directly via the USD API
        # This flushes the temporal cache of the original positions
        # Move once every 3 frames, for a total of warmup_k moves
        import random as py_random
        for warmup_i in range(warmup_frames):
            # Move the bottles every 3 frames to refresh the cache
            if warmup_i % 3 == 0:
                for path in candidate_paths:
                    target_path = prim_path_map.get(path, path)
                    prim = stage.GetPrimAtPath(target_path)
                    if prim:
                        xformable = UsdGeom.Xformable(prim)
                        if xformable:
                            # Get the existing translate op
                            for op in xformable.GetOrderedXformOps():
                                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                    # Slightly adjust the position to refresh the cache
                                    current = op.Get()
                                    if current:
                                        new_pos = (
                                            current[0] + py_random.uniform(-5, 5),
                                            current[1] + py_random.uniform(-5, 5),
                                            current[2]
                                        )
                                        op.Set(new_pos)
                                    break
            rep.orchestrator.step()
        
        print(f"[kitchen_headless] Warmup done, original positions flushed from cache.")

        # Diagnostic: pull a frame of segmentation data from the annotator to confirm it is non-zero
        def log_annotator_sample(render_prod):
            import numpy as np

            semantic_anno = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
            instance_anno = rep.AnnotatorRegistry.get_annotator("instance_segmentation")
            semantic_anno.attach(render_prod)
            instance_anno.attach(render_prod)

            # Manually step one frame while capture is disabled
            rep.orchestrator.step()

            sem = semantic_anno.get_data()
            ins = instance_anno.get_data()
            def summarize(arr, name):
                # Compatible with different return formats (dict or ndarray)
                if isinstance(arr, dict) and "data" in arr:
                    arr = arr["data"]
                if isinstance(arr, dict) and "semantic" in arr:
                    arr = arr["semantic"]
                arr_np = np.array(arr)
                uniq = np.unique(arr_np)
                print(
                    f"[diagnostic] {name}: shape {arr_np.shape}, min {uniq.min()}, max {uniq.max()}, "
                    f"unique count {len(uniq)}, sample {uniq[:10]}"
                )
            summarize(sem, "semantic_segmentation")
            summarize(ins, "instance_segmentation")

            semantic_anno.detach()
            instance_anno.detach()

        # log_annotator_sample(render_product)  # Temporarily disable diagnostics to avoid extra step()

        # Clean up old files in the directory
        for old_file in raw_dir.glob("*.*"):
            old_file.unlink(missing_ok=True)

        # Now initialize and attach the writer (after warmup)
        writer.initialize(
            output_dir=str(raw_dir),
            rgb=True,
            distance_to_camera=True,           # depth map
            semantic_segmentation=True,        # semantic segmentation map
            colorize_semantic_segmentation=True,  # colorized semantic segmentation
            instance_segmentation=True,        # instance segmentation map
            colorize_instance_segmentation=True,  # colorized instance segmentation
            bounding_box_2d_tight=False
        )
        writer.attach(render_product)

        # Enable capture
        rep.orchestrator.set_capture_on_play(True)
        print(f"[kitchen_headless] Writer attached, starting capture...")

        # Create the pair plan
        total_frames = pair_count * 2
        pair_records = []
        
        for pair_idx in range(pair_count):
            pair_records.append({
                "scene_type": "childrenroom_desk",
                "pair_id": f"pair_{pair_idx:04d}",
                "camera": cam_path,
                "coordinates": {},  # Will be filled in during rendering
            })

        print(f"[kitchen_headless] Generating {pair_count} pairs ({total_frames} frames)")

        # Define a function to get world coordinates
        def get_world_coordinates(prim_paths, prim_map=None):
            """Get the world coordinates of the specified prims"""
            coords = {}
            for path in prim_paths:
                target_path = prim_map.get(path, path) if prim_map else path
                prim = stage.GetPrimAtPath(target_path)
                if prim and prim.IsValid():
                    xformable = UsdGeom.Xformable(prim)
                    if xformable:
                        # Get the world transform matrix
                        world_transform = xformable.ComputeLocalToWorldTransform(0)
                        # Extract the translation part (world coordinates)
                        translation = world_transform.ExtractTranslation()
                        coords[path] = [translation[0], translation[1], translation[2]]
            return coords

        def get_2d_center_coordinates_from_projection(prim_paths, camera_path, image_width, image_height, prim_map=None):
            """
            Use the camera projection matrix to project 3D world coordinates to 2D pixel coordinates (does not rely on semantic segmentation)

            Args:
                prim_paths: list of prim paths
                camera_path: camera path
                image_width: image width
                image_height: image height

            Returns:
                dict: {prim_path: [pixel_x, pixel_y]} or {prim_path: None} if projection fails
            """
            coords_2d = {}

            # Get the camera prim
            camera_prim = stage.GetPrimAtPath(camera_path)
            if not camera_prim or not camera_prim.IsValid():
                print(f"[kitchen_headless] Warning: Camera not found: {camera_path}")
                for path in prim_paths:
                    coords_2d[path] = None
                return coords_2d
            
            camera = UsdGeom.Camera(camera_prim)
            if not camera:
                print(f"[kitchen_headless] Warning: Failed to get UsdGeom.Camera for {camera_path}")
                for path in prim_paths:
                    coords_2d[path] = None
                return coords_2d
            
            # Get the camera's world transform matrix (camera to world)
            camera_xformable = UsdGeom.Xformable(camera_prim)
            camera_world_transform = camera_xformable.ComputeLocalToWorldTransform(0)
            # World-to-camera transform (inverse matrix)
            world_to_camera = camera_world_transform.GetInverse()

            # Get camera parameters
            focal_length = camera.GetFocalLengthAttr().Get()  # mm
            h_aperture = camera.GetHorizontalApertureAttr().Get()  # mm
            v_aperture = camera.GetVerticalApertureAttr().Get()  # mm

            if focal_length is None or h_aperture is None:
                print(f"[kitchen_headless] Warning: Camera parameters not available")
                for path in prim_paths:
                    coords_2d[path] = None
                return coords_2d

            # If v_aperture is None, compute it from the aspect ratio
            if v_aperture is None or v_aperture == 0:
                v_aperture = h_aperture * (image_height / image_width)
            
            for path in prim_paths:
                target_path = prim_map.get(path, path) if prim_map else path
                prim = stage.GetPrimAtPath(target_path)
                if not prim or not prim.IsValid():
                    coords_2d[path] = None
                    continue
                
                xformable = UsdGeom.Xformable(prim)
                if not xformable:
                    coords_2d[path] = None
                    continue
                
                # Get the world coordinates of the object
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                world_pos = world_transform.ExtractTranslation()

                # Convert world coordinates into the camera coordinate system
                # Use the TransformAffine method for 3D points
                world_pos_3d = Gf.Vec3d(world_pos[0], world_pos[1], world_pos[2])
                camera_pos = world_to_camera.TransformAffine(world_pos_3d)

                # Position in the camera coordinate system
                cam_x = camera_pos[0]
                cam_y = camera_pos[1]
                cam_z = camera_pos[2]

                # The camera looks along -Z, so z must be negative for the object to be visible
                if cam_z >= 0:
                    # The object is behind the camera
                    print(f"[kitchen_headless]   {path} is behind camera (z={cam_z:.2f})")
                    coords_2d[path] = None
                    continue

                # Perspective projection (pinhole camera model)
                # NDC coordinates (normalized device coordinates)
                ndc_x = (focal_length * cam_x) / (-cam_z * h_aperture / 2)
                ndc_y = (focal_length * cam_y) / (-cam_z * v_aperture / 2)

                # NDC range is [-1, 1]; convert to pixel coordinates
                # Note: the y axis points downward in the image
                pixel_x = int((ndc_x + 1) * 0.5 * image_width)
                pixel_y = int((1 - ndc_y) * 0.5 * image_height)  # Flip Y

                # Check whether it is inside the image bounds
                if 0 <= pixel_x < image_width and 0 <= pixel_y < image_height:
                    coords_2d[path] = [pixel_x, pixel_y]
                    print(f"[kitchen_headless]   Projected {path}: world ({world_pos[0]:.1f}, {world_pos[1]:.1f}, {world_pos[2]:.1f}) -> pixel ({pixel_x}, {pixel_y})")
                else:
                    print(f"[kitchen_headless]   {path} projected outside image: ({pixel_x}, {pixel_y})")
                    coords_2d[path] = [pixel_x, pixel_y]  # Still saved, even if out of bounds

            return coords_2d

        # 13-15. Randomize directly using the USD API, without the Replicator trigger
        # This way we have full control over when randomization runs, avoiding Z compensation being overwritten

        frame_counter = [0]
        frame_coordinates = []
        frame_2d_coordinates = []

        # Warmup steps: run multiple frames to flush the renderer's temporal accumulation cache and eliminate ghosting
        # After each randomization, run 3 frames to let the renderer stabilize, for a total of 5 randomizations = 15 frames
        warmup_randomizations = 5
        warmup_steps_per_rand = 3
        print(f"[kitchen_headless] Running warmup: {warmup_randomizations} randomizations × {warmup_steps_per_rand} steps = {warmup_randomizations * warmup_steps_per_rand} frames...")

        # Detach writer during warmup to avoid capturing these frames
        writer.detach()

        for warmup_i in range(warmup_randomizations):
            prim_z_map = manual_randomize_objects(prim_path_map, surface_bounds_list)
            if use_z_compensation:
                apply_z_compensation(prim_z_map, reverse_prim_map)
            # After each randomization, run multiple frames to refresh temporal accumulation
            for _ in range(warmup_steps_per_rand):
                rep.orchestrator.step()

        # Re-attach the writer to start the official capture
        writer.attach(render_product)
        print(f"[kitchen_headless] Warmup complete, writer re-attached.")

        # 16. Main render loop (remove version)
        # Each pair: first randomly place and render A (hidden), then show the object and render B (visible)
        flush_frames_before_capture = 6  # Number of frames to flush before capture
        print(f"[kitchen_headless] Running main render loop with {pair_count} pairs...")
        print(f"[kitchen_headless] Using {flush_frames_before_capture} flush frames before each capture to eliminate ghosting")

        # Ensure the initial state is visible
        for path in candidate_paths:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                prim.GetAttribute("visibility").Set("inherited")

        for pair_idx in range(pair_count):
            print(f"\n[kitchen_headless] ===== Pair {pair_idx} =====")

            # Step 1: randomize object position and rotation (only once, used for the same A/B position)
            prim_z_map = manual_randomize_objects(prim_path_map, surface_bounds_list)

            # Step 2: optional Z compensation (when off, only uses forced Z)
            if use_z_compensation:
                apply_z_compensation(prim_z_map, reverse_prim_map)

            # ------- Frame A: hidden -------
            for path in candidate_paths:
                prim = stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    prim.GetAttribute("visibility").Set("invisible")
            writer.detach()
            for _ in range(flush_frames_before_capture):
                rep.orchestrator.step()
            writer.attach(render_product)
            rep.orchestrator.step()
            print(f"[kitchen_headless] Pair {pair_idx}: captured A (hidden)")

            # A EN None（EN）
            null_coords = {path: None for path in candidate_paths}
            frame_coordinates.append(null_coords)
            frame_2d_coordinates.append(null_coords)

            # ------- B EN:visible -------
            for path in candidate_paths:
                prim = stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    prim.GetAttribute("visibility").Set("inherited")
            writer.detach()
            for _ in range(flush_frames_before_capture):
                rep.orchestrator.step()
            writer.attach(render_product)
            rep.orchestrator.step()
            print(f"[kitchen_headless] Pair {pair_idx}: captured B (visible)")

            # EN
            print(f"[Verify] ===== B EN =====")
            for path in candidate_paths:
                target_path = prim_path_map.get(path, path)
                prim = stage.GetPrimAtPath(target_path)
                if prim and prim.IsValid():
                    xformable = UsdGeom.Xformable(prim)
                    if xformable:
                        for op in xformable.GetOrderedXformOps():
                            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                pos = op.Get()
                                print(f"[Verify] {path}: translate = ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
                                break
                        bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
                        world_bbox = bbox_cache.ComputeWorldBound(prim)
                        if world_bbox and not world_bbox.GetRange().IsEmpty():
                            bbox_min = world_bbox.GetRange().GetMin()
                            print(f"[Verify]   bbox_min_z = {bbox_min[2]:.4f}")

            # EN B EN 2D coordinates
            current_coords = get_world_coordinates(candidate_paths, prim_path_map)
            current_2d_coords = get_2d_center_coordinates_from_projection(
                candidate_paths, cam_path, resolution[0], resolution[1], prim_path_map
            )
            frame_coordinates.append(current_coords)
            frame_2d_coordinates.append(current_2d_coords)

            # EN, prepare next pair
            for path in candidate_paths:
                prim = stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    prim.GetAttribute("visibility").Set("inherited")
        
        print(f"[kitchen_headless] Rendering complete. Recorded {len(frame_coordinates)} world coord frames, {len(frame_2d_coordinates)} 2D coord frames.")
        
        # EN
        rep.orchestrator.wait_until_complete()
        print(f"[kitchen_headless] All frames written to disk.")

        # 17. reorganize outputs（EN RGB、depth map、segmentation map）
        import shutil
        
        rgb_frames = sorted(raw_dir.glob("rgb_*.png"))
        depth_frames = sorted(raw_dir.glob("distance_to_camera_*.npy"))
        if not depth_frames:
            depth_frames = sorted(raw_dir.glob("distance_to_camera_*.png"))
        instance_frames = sorted(raw_dir.glob("instance_segmentation_*.png"))
        semantic_frames = sorted(raw_dir.glob("semantic_segmentation_*.png"))
        
        print(f"[kitchen_headless] Found {len(rgb_frames)} RGB frames")
        print(f"[kitchen_headless] Found {len(depth_frames)} depth frames")
        print(f"[kitchen_headless] Found {len(instance_frames)} instance segmentation frames")
        print(f"[kitchen_headless] Found {len(semantic_frames)} semantic segmentation frames")
        print(f"[kitchen_headless] Recorded {len(frame_coordinates)} world coordinate snapshots")
        print(f"[kitchen_headless] Recorded {len(frame_2d_coordinates)} 2D coordinate snapshots")
        
        # EN pair pair
        # EN:warmup EN writer detached, EN, EN
        # EN = pair_count * 2
        min_frames_needed = pair_count * 2
        actual_rgb = len(rgb_frames)
        actual_coords = len(frame_coordinates)
        
        print(f"[kitchen_headless] Expected: {min_frames_needed} frames (warmup not captured)")
        print(f"[kitchen_headless] Actual: {actual_rgb} RGB frames, {actual_coords} coord snapshots")
        
        if actual_rgb < min_frames_needed:
            print(f"[kitchen_headless] ⚠️ Warning: Need {min_frames_needed} frames but found {actual_rgb}")
            # EN pair count
            pair_count = min(pair_count, actual_rgb // 2)
        
        if actual_coords < min_frames_needed:
            print(f"[kitchen_headless] ⚠️ Warning: Need {min_frames_needed} coordinate snapshots but found {actual_coords}")
        
        print(f"[kitchen_headless] Will create {pair_count} pairs")
        
        for idx in range(pair_count):
            record = pair_records[idx]
            pair_dir = cam_out_dir / record["pair_id"]
            pair_dir.mkdir(parents=True, exist_ok=True)
            
            # EN（warmup EN）
            coord_a_idx = idx * 2
            coord_b_idx = idx * 2 + 1
            
            # EN:start from 0 start
            frame_a_idx = idx * 2
            frame_b_idx = idx * 2 + 1
            
            # EN（EN）
            actual_2d_coords = len(frame_2d_coordinates)
            if coord_a_idx < actual_coords and coord_b_idx < actual_coords:
                coords_a = frame_coordinates[coord_a_idx]
                coords_b = frame_coordinates[coord_b_idx]
                
                # get2Dcoordinates（EN）
                coords_2d_a = frame_2d_coordinates[coord_a_idx] if coord_a_idx < actual_2d_coords else {}
                coords_2d_b = frame_2d_coordinates[coord_b_idx] if coord_b_idx < actual_2d_coords else {}
                
                # build coordinates dict, EN2DEN
                for prim_path in candidate_paths:
                    record["coordinates"][prim_path] = {
                        "world_coordinate_A": coords_a.get(prim_path, None),
                        "world_coordinate_B": coords_b.get(prim_path, None),
                        "pixel_center_A": coords_2d_a.get(prim_path, None),
                        "pixel_center_B": coords_2d_b.get(prim_path, None),
                    }
            else:
                print(f"[kitchen_headless] ⚠️ Coordinates not available for pair {idx}")
            
            # RGB（EN, skip warmup frames）
            if frame_a_idx < len(rgb_frames) and frame_b_idx < len(rgb_frames):
                shutil.copy2(rgb_frames[frame_a_idx], pair_dir / "A_rgb.png")
                shutil.copy2(rgb_frames[frame_b_idx], pair_dir / "B_rgb.png")
            
            # depth map
            if frame_a_idx < len(depth_frames) and frame_b_idx < len(depth_frames):
                depth_ext = depth_frames[frame_a_idx].suffix
                shutil.copy2(depth_frames[frame_a_idx], pair_dir / f"A_depth{depth_ext}")
                shutil.copy2(depth_frames[frame_b_idx], pair_dir / f"B_depth{depth_ext}")
                
                # EN npy format, EN png visualization
                if depth_ext == ".npy":
                    depth_npy_to_png(depth_frames[frame_a_idx], pair_dir / "A_depth.png")
                    depth_npy_to_png(depth_frames[frame_b_idx], pair_dir / "B_depth.png")
            
            # EN
            if frame_a_idx < len(instance_frames) and frame_b_idx < len(instance_frames):
                shutil.copy2(instance_frames[frame_a_idx], pair_dir / "A_instance_segmentation.png")
                shutil.copy2(instance_frames[frame_b_idx], pair_dir / "B_instance_segmentation.png")
            
            # EN
            if frame_a_idx < len(semantic_frames) and frame_b_idx < len(semantic_frames):
                shutil.copy2(semantic_frames[frame_a_idx], pair_dir / "A_semantic_segmentation.png")
                shutil.copy2(semantic_frames[frame_b_idx], pair_dir / "B_semantic_segmentation.png")
            
            # save metadata
            meta_path = pair_dir / "metadata.json"
            meta_path.write_text(json.dumps(record, indent=2))
            
            print(f"[kitchen_headless] Created {record['pair_id']}: A=frame{frame_a_idx}, B=frame{frame_b_idx}")
        
        # cleanup raw EN
        for leftover in raw_dir.glob("*.*"):
            leftover.unlink(missing_ok=True)

        writer.detach()
        render_product.destroy()

    # 18. done
    rep.orchestrator.wait_until_complete()
    print("\n[kitchen_headless] ========== Done! ==========")
    print(f"Output saved to: {out_dir}")


def depth_npy_to_png(npy_path, png_path):
    """
    EN npy EN png image
    """
    import numpy as np
    from PIL import Image
    
    try:
        # EN
        depth_data = np.load(npy_path)
        
        # EN
        if len(depth_data.shape) > 2:
            depth_data = depth_data[:, :, 0] if depth_data.shape[2] >= 1 else depth_data.squeeze()
        
        # EN（EN inf, nan）
        valid_mask = np.isfinite(depth_data)
        if not valid_mask.any():
            print(f"[depth_npy_to_png] Warning: No valid depth values in {npy_path}")
            return False
        
        # EN
        min_val = depth_data[valid_mask].min()
        max_val = depth_data[valid_mask].max()
        
        # EN 0-255 range
        if max_val > min_val:
            normalized = (depth_data - min_val) / (max_val - min_val)
        else:
            normalized = np.zeros_like(depth_data)
        
        # EN 0
        normalized[~valid_mask] = 0
        
        # invert:EN, EN
        normalized = 1.0 - normalized
        
        # EN 8-bit image
        depth_uint8 = (normalized * 255).astype(np.uint8)
        
        # EN png
        img = Image.fromarray(depth_uint8)
        img.save(png_path)
        return True
        
    except Exception as e:
        print(f"[depth_npy_to_png] Error converting {npy_path}: {e}")
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Kitchen scene headless rendering")
    parser.add_argument("--pair-count", type=int, default=10, help="Number of image pairs to generate")
    parser.add_argument("--warmup-k", type=int, default=3, help="Warmup multiplier (warmup_frames = 3 * k)")
    parser.add_argument("--width", type=int, default=1024, help="Image width")
    parser.add_argument("--height", type=int, default=768, help="Image height")
    parser.add_argument("--focal-length", type=float, default=None, help="Camera focal length (optional)")
    parser.add_argument("--output-dir", type=str, default="/workspace/output/childrenroom_desk_add/3_items/120-139", help="Output directory")
    args = parser.parse_args()

    run_kitchen_example(
        pair_count=args.pair_count,
        warmup_k=args.warmup_k,
        resolution=(args.width, args.height),
        focal_length=args.focal_length,
        output_dir=args.output_dir,
    )
    simulation_app.close()
