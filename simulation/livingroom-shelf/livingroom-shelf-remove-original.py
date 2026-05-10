# Headless version - adapted from kitchen_test_editor.py
# Suitable for running in container / no GUI environment
# Run with ./python.sh kitchen_test_headless.py
# Feature: randomize 3 bottle prims, place on two cabinet planes
# pair differences are reflected by different randomized positions and bottle fallen states

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
    resolution=(1024, 768),
    focal_length: float | None = None,
    pair_start_index: int = 0,
    mark_centers: bool = False,
    hide_batch_size: int = 1,
    output_dir: str = "/workspace/output/livingroom_shelf_remove_original/3_items",
):
    """
    Headless version of the scene example
    Uses fixed camera /World/Camera
    Initial frame + sequence frames randomly hiding specified prims, constructs pairs by adjacent frames
    """

    # 2. Open the kitchen scene
    usd_path = "/workspace/assets/Interactive_scene/largelivingroom/Interactive_largelivingroom.usd"
    print(f"[kitchen_headless] Opening stage: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)

    stage = omni.usd.get_context().get_stage()

    if not stage:
        print("[kitchen_headless] Error: Failed to open stage!")
        return

    # Keep the scene as is, do not modify any lighting settings
    print("[kitchen_headless] Scene loaded, preserving original lighting")

    # 3.2 Remove physics scene and disable global physics, to avoid being pushed by physics after randomization
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene":
            stage.RemovePrim(prim.GetPath())
            print(f"[kitchen_headless] Removed PhysicsScene: {prim.GetPath()}")

    import carb.settings
    carb.settings.get_settings().set("/physics/enabled", False)
    print("[kitchen_headless] Physics disabled via carb settings")

    # 3. Configure render settings to eliminate ghosting caused by temporal accumulation
    # RTSubframes: force the renderer to resample each frame
    rep.settings.carb_settings("/omni/replicator/RTSubframes", 32)
    print("[kitchen_headless] RTSubframes set to 16 (to eliminate ghosting)")



    # 4. Pair with on_frame trigger, let orchestrator drive write to disk
    rep.orchestrator.set_capture_on_play(True)

    # 4. Use only the fixed camera /World/Camera
    desired_camera = "/World/Camera"
    cam_prim = stage.GetPrimAtPath(desired_camera)
    if not cam_prim or not cam_prim.IsValid():
        print(f"[kitchen_headless] Camera not found, creating: {desired_camera}")
        cam_prim = stage.DefinePrim(desired_camera, "Camera")
        cam_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Float3).Set((0.0, 150.0, 600.0))
        cam_prim.CreateAttribute("focalLength", Sdf.ValueTypeNames.Float).Set(35.0)
    camera_list = [desired_camera]
    print(f"[kitchen_headless] Using camera: {desired_camera}")

    # Optional: adjust camera focal length
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

    # 6. Only use the specified 3 bottle prims for randomization
    def find_bottle_prims():
        """Only return the 3 bottle prims specified by the user"""
        specific_prims = [
            "/World/model_book_5",
            "/World/model_potted_plant002",
            "/World/model_book_4",
            "/World/model_book_2",
            "/World/model_book8",
            "/World/model_book_7",
            "/World/model_book6",
            "/World/model_book2_02",
            "/World/model_book2_03",
            "/World/model_book2_04",
            "/World/model_book2_05",
            "/World/model_book6_05",
            "/World/model_book8_03",
            "/World/model_book_12",
            "/World/model_book_15",
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

    # 7. Check and remove collision from objects (randomized objects do not need CollisionAPI)
    def remove_collision_from_bottles(bottle_paths):
        """Check and remove CollisionAPI from objects; randomized objects do not need collision"""
        for path in bottle_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                print(f"[kitchen_headless] ❌ Prim not found: {path}")
                continue
            
            removed_count = 0

            # Cancel instanceable
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                print(f"[kitchen_headless] Made prim non-instanceable: {path}")

            # Check and remove CollisionAPI on the Xform itself
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
                    
                    # Continue recursively checking child nodes
                    remove_collision_from_children(child, depth + 1)
            
            remove_collision_from_children(prim)
            
            if removed_count > 0:
                print(f"[kitchen_headless] Physics cleanup done: {path} (removed {removed_count} CollisionAPI)")
            else:
                print(f"[kitchen_headless] No CollisionAPI found on: {path}")

    # 8. Add semantic labels to randomized objects (for instance/semantic segmentation)
    def add_semantic_labels(prim_paths):
        """
        Add semantic labels to specified prims; the label name is the last segment of the path
        Use two approaches at the same time to ensure compatibility:
        1. Directly use USD primvars to write semantic attributes (takes effect immediately)
        2. Use rep.modify.semantics (for Replicator graph)
        """
        print("[kitchen_headless] Adding semantic labels using dual approach...")
        
        added_count = 0

        def clear_old_semantics(prim):
            for attr in list(prim.GetAttributes()):
                if attr.GetName().startswith("primvars:semantics:"):
                    prim.RemoveProperty(attr.GetName())

        def set_semantic_primvar(prim, prim_name: str):
            # Use UsdGeom primvar API, type token array, constant interpolation
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
            
            # Automatically assign semantic label based on prim path (take the last segment)
            segments = [seg for seg in path.split("/") if seg]
            prim_name = segments[-1] if segments else "object"

            try:
                # Cancel instanceable to ensure primvars can be written
                if prim.IsInstanceable():
                    prim.SetInstanceable(False)

                # Write primvars on the Xform
                set_semantic_primvar(prim, prim_name)

                # Write primvars on child Meshes to ensure segmentation can read them
                mesh_children = collect_mesh_children(prim)
                for mesh_prim in mesh_children:
                    if mesh_prim.IsInstanceable():
                        mesh_prim.SetInstanceable(False)
                    set_semantic_primvar(mesh_prim, prim_name)

                # Use rep.modify.semantics to add semantic labels
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

    # 9.1 Resolve the movable prim (judged by bbox changes)
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
    
    # Remove physics collision from these prims (if any)
    remove_collision_from_bottles(candidate_paths)

    # Apply semantic labels to all objects to be randomized
    added = add_semantic_labels(candidate_paths)
    print(f"[kitchen_headless] Semantic labels added: {added} prims labeled")

    # 10. Use the specified plane for random placement
    cabinet_prims = [
        "World/Plane",
    ]

    def normalize_prim_path(path: str) -> str:
        """Ensure the path starts with /, compatible with input without leading slash"""
        return path if path.startswith("/") else f"/{path}"

    # Verify whether cabinet prims exist
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
    
    # 10.1 Get surface Z height (for Z compensation)
    def get_surface_z_height(surface_prim_paths):
        """
        Get the highest Z coordinate of the surface as the reference height for object placement
        """
        print(f"\n[Surface-Z] ========== Compute surface Z height ==========")
        max_z = float('-inf')
        for path in surface_prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[Surface-Z] ❌ Prim invalid: {path}")
                continue

            print(f"[Surface-Z] Check: {path}, type: {prim.GetTypeName()}")
            
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
                print(f"[Surface-Z]   ⚠️ Failed to compute bbox or bbox is empty")

                # Try getting transform as a fallback
                xformable = UsdGeom.Xformable(prim)
                if xformable:
                    world_transform = xformable.ComputeLocalToWorldTransform(0)
                    translation = world_transform.ExtractTranslation()
                    print(f"[Surface-Z]   Transform position: ({translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f})")
                    # Use transform's Z as a fallback
                    if translation[2] > max_z:
                        max_z = translation[2]
                        print(f"[Surface-Z]   Using transform Z as surface height: {translation[2]:.4f}")

        result = max_z if max_z != float('-inf') else 0.0
        print(f"[Surface-Z] Final surface Z height: {result}")
        print(f"[Surface-Z] ==========================================\n")
        return result

    # Get surface height (take max of surface_z from bounds_list)
    # Note: get_surface_z_height may return 0, so we update again after bounds_list is computed
    surface_z_height = get_surface_z_height(valid_cabinet_prims)

    # 11. Find the actual Mesh prim (scatter_2d requires a direct Mesh, not a container)
    def find_actual_mesh(prim_path, depth=0, max_depth=3):
        """
        Recursively find the actual Mesh child node under a prim
        Because scatter_2d requires a direct Mesh prim, not a container holding Mesh
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
    
    # Find actual mesh prims for all cabinets
    actual_cabinet_meshes = []
    for cabinet_prim in valid_cabinet_prims:
        actual_mesh = find_actual_mesh(cabinet_prim)
        if actual_mesh:
            actual_cabinet_meshes.append(actual_mesh)
            print(f"[kitchen_headless] Using mesh for scatter_2d: {actual_mesh}")
        else:
            # If no child Mesh is found, try using the prim directly
            print(f"[kitchen_headless] Will try using prim directly: {cabinet_prim}")
            actual_cabinet_meshes.append(cabinet_prim)
    
    if not actual_cabinet_meshes:
        print("[kitchen_headless] ❌ Error: No valid meshes found for scatter_2d")
        return
    
    # Get cabinet prims as the surface for scatter_2d
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
    
    # 11.5 Compute world coordinate range of each surface (for USD API randomization)
    def get_each_surface_bounds(surface_prim_paths):
        """
        Get world coordinate range for each surface (stored separately, not merged)
        Use world transform matrix + Mesh's extent attribute to precisely compute the range
        Returns: [(min_x, max_x, min_y, max_y, surface_z), ...]
        """
        surface_bounds_list = []

        print(f"\n[Surface-Bounds] ========== Compute each surface range ==========")

        def find_mesh_in_prim(parent_prim, depth=0, max_depth=5):
            """Recursively find Mesh child nodes under a prim"""
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
                print(f"[Surface-Bounds] ❌ Prim invalid: {path}")
                continue

            print(f"[Surface-Bounds] Check: {path}, type: {prim.GetTypeName()}")

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
                print(f"[Surface-Bounds] ❌ Failed to get Xformable: {target_path}")
                continue

            world_transform = target_xformable.ComputeLocalToWorldTransform(0)

            # If extent exists, use its four corners to compute world range
            if local_extent and len(local_extent) >= 2:
                local_min = local_extent[0]
                local_max = local_extent[1]

                # Build 8 corners of local space (for 3D box) or 4 corners (for plane)
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
                # Fallback: assume unit plane
                print(f"[Surface-Bounds]   ⚠️ No extent, assuming unit plane")
                local_corners = [
                    Gf.Vec3d(-0.5, -0.5, 0),
                    Gf.Vec3d(0.5, -0.5, 0),
                    Gf.Vec3d(-0.5, 0.5, 0),
                    Gf.Vec3d(0.5, 0.5, 0),
                ]
            
            # Transform to world coordinates and compute range
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

            print(f"[Surface-Bounds]   Computed range: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}], Z={surface_z:.2f}")

            # Add 5% margin
            range_x = max_x - min_x
            range_y = max_y - min_y
            margin_x = range_x * 0.05
            margin_y = range_y * 0.1
            min_x += margin_x
            max_x -= margin_x
            min_y += margin_y
            max_y -= margin_y

            # Use the range with margin
            bounds = (min_x, max_x, min_y, max_y, surface_z)
            surface_bounds_list.append(bounds)

            print(f"[Surface-Bounds]   Final range: X=[{bounds[0]:.2f}, {bounds[1]:.2f}], Y=[{bounds[2]:.2f}, {bounds[3]:.2f}], Z={surface_z:.2f}")

        print(f"[Surface-Bounds] Total {len(surface_bounds_list)} valid surfaces")
        print(f"[Surface-Bounds] ==========================================\n")

        return surface_bounds_list

    # Get the range of each surface (list)
    surface_bounds_list = get_each_surface_bounds(valid_cabinet_prims)

    # Get the correct surface_z_height from bounds_list (take max)
    # This is more reliable than get_surface_z_height
    if surface_bounds_list:
        surface_z_height = max(bounds[4] for bounds in surface_bounds_list)
        print(f"[kitchen_headless] ✓ surface_z_height updated from bounds_list to: {surface_z_height:.4f}")

    # 11.6 Pure USD API randomization function (full control, does not rely on Replicator trigger)
    import random as py_random

    def manual_randomize_objects(prim_path_map, bounds_list):
        """
        Use pure USD API to randomize object position and rotation
        Each object randomly selects a plane, then is placed within that plane's range

        Args:
            prim_path_map: {original_path: movable_path}
            bounds_list: list of surface ranges [(min_x, max_x, min_y, max_y, surface_z), ...]

        Returns:
            dict: {prim_path: surface_z} the plane Z height each object is placed on
        """
        prim_surface_z_map = {}

        if not bounds_list:
            print(f"[Manual-Rand] ❌ No valid surface ranges")
            return prim_surface_z_map

        print(f"\n[Manual-Rand] ========== Start USD API randomization ==========")
        print(f"[Manual-Rand] Available surface count: {len(bounds_list)}")

        for orig_path, move_path in prim_path_map.items():
            prim = stage.GetPrimAtPath(move_path)
            if not prim or not prim.IsValid():
                print(f"[Manual-Rand] ❌ Prim invalid: {move_path} (orig {orig_path})")
                continue

            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                print(f"[Manual-Rand] ❌ Failed to get Xformable: {path}")
                continue

            # Randomly select a plane
            selected_bounds = py_random.choice(bounds_list)
            min_x, max_x, min_y, max_y, surface_z = selected_bounds
            surface_idx = bounds_list.index(selected_bounds)

            # Record the surface Z height where this object is placed
            prim_surface_z_map[move_path] = surface_z

            # Randomly generate position (within the selected plane range)
            rand_x = py_random.uniform(min_x, max_x)
            rand_y = py_random.uniform(min_y, max_y)
            # Force Z directly, do not rely on bbox compensation
            rand_z = surface_z

            # Random rotation: X axis 0 or 90 degrees (standing or fallen), Z axis 0-360 degrees
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

            state = "standing" if rand_rot_x == 0 else "fallen"

            # Verify whether the position is within range
            in_range_x = min_x <= rand_x <= max_x
            in_range_y = min_y <= rand_y <= max_y
            if not in_range_x or not in_range_y:
                print(f"[Manual-Rand] ❌ {path}: position out of range!")
                print(f"[Manual-Rand]   bounds: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}]")
                print(f"[Manual-Rand]   pos: X={rand_x:.2f} {'✓' if in_range_x else '❌'}, Y={rand_y:.2f} {'✓' if in_range_y else '❌'}")

            print(f"[Manual-Rand] ✓ {orig_path}: plane{surface_idx}(Z={surface_z:.2f}), bounds=X[{min_x:.2f},{max_x:.2f}],Y[{min_y:.2f},{max_y:.2f}], pos=({rand_x:.2f}, {rand_y:.2f}, {rand_z:.2f}), rot=({rand_rot_x}, {rand_rot_y}, {rand_rot_z:.0f}) [{state}]")

        print(f"[Manual-Rand] ========== Randomization complete ==========\n")
        return prim_surface_z_map

    # 12. Render for each camera
    pairs_created_total = 0
    for cam_path in camera_list:
        print(f"\n[kitchen_headless] ========== Rendering camera: {cam_path} ==========")

        render_product = rep.create.render_product(cam_path, resolution)

        cam_out_dir = out_dir
        raw_dir = cam_out_dir / "_raw_frames"
        os.makedirs(raw_dir, exist_ok=True)
        

        
        # Clean up old files in the directory
        for old_file in raw_dir.glob("*.*"):
            old_file.unlink(missing_ok=True)

        # Now initialize and attach writer
        writer.initialize(
            output_dir=str(raw_dir),
            rgb=True,
            distance_to_camera=True,           # depth map
            instance_segmentation=True,        # instance segmentation map
            colorize_instance_segmentation=True,  # colorized instance segmentation
            bounding_box_2d_tight=False
        )
        writer.attach(render_product)
        # Disable capture by default; controlled by capture_frame
        rep.orchestrator.set_capture_on_play(False)
        print(f"[kitchen_headless] Writer attached, capture controlled per frame")

        # Create pair plan: hide k prims each time
        hide_batch_size = max(int(hide_batch_size), 1)
        total_pairs = len(candidate_paths) // hide_batch_size
        total_frames = total_pairs * 2
        pair_records = []
        for pair_idx in range(total_pairs):
            pair_id = f"pair_{pair_start_index + pair_idx:04d}"
            pair_records.append({
                "scene_type": "childrenroom_desk",
                "pair_id": pair_id,
                "camera": cam_path,
                "coordinates": {},  # will be filled in during rendering
            })

        print(f"[kitchen_headless] Generating {total_pairs} pairs ({total_frames} frames)")

        # Define the function to get world coordinates (consistent with childrenroom)
        def get_world_coordinates(prim_paths, prim_map=None):
            """Get world coordinates of specified prims"""
            coords = {}
            for path in prim_paths:
                target_path = prim_map.get(path, path) if prim_map else path
                prim = stage.GetPrimAtPath(target_path)
                if prim and prim.IsValid():
                    xformable = UsdGeom.Xformable(prim)
                    if xformable:
                        world_transform = xformable.ComputeLocalToWorldTransform(0)
                        translation = world_transform.ExtractTranslation()
                        coords[path] = [translation[0], translation[1], translation[2]]
            return coords

        def get_2d_center_coordinates_from_projection(prim_paths, camera_path, image_width, image_height, prim_map=None):
            """
            Use the camera projection matrix to project 3D world coordinates to 2D pixel coordinates (does not depend on semantic segmentation)

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
            # World to camera transform (inverse matrix)
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
            
            # If v_aperture is None, compute from aspect ratio
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
                
                # Get the object's world coordinates
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                world_pos = world_transform.ExtractTranslation()

                # Convert world coordinates to camera coordinate system
                # Use TransformAffine method to handle 3D points
                world_pos_3d = Gf.Vec3d(world_pos[0], world_pos[1], world_pos[2])
                camera_pos = world_to_camera.TransformAffine(world_pos_3d)

                # Position in camera coordinate system
                cam_x = camera_pos[0]
                cam_y = camera_pos[1]
                cam_z = camera_pos[2]

                # Camera looks toward -Z, so z must be negative to be visible
                if cam_z >= 0:
                    # Object is behind the camera
                    print(f"[kitchen_headless]   {path} is behind camera (z={cam_z:.2f})")
                    coords_2d[path] = None
                    continue

                # Perspective projection (pinhole camera model)
                # NDC coordinates (normalized device coordinates)
                ndc_x = (focal_length * cam_x) / (-cam_z * h_aperture / 2)
                ndc_y = (focal_length * cam_y) / (-cam_z * v_aperture / 2)

                # NDC range is [-1, 1], convert to pixel coordinates
                # Note: y axis points downward in the image
                pixel_x = int((ndc_x + 1) * 0.5 * image_width)
                pixel_y = int((1 - ndc_y) * 0.5 * image_height)  # flip Y

                # Check whether within image range
                if 0 <= pixel_x < image_width and 0 <= pixel_y < image_height:
                    coords_2d[path] = [pixel_x, pixel_y]
                    print(f"[kitchen_headless]   Projected {path}: world ({world_pos[0]:.1f}, {world_pos[1]:.1f}, {world_pos[2]:.1f}) -> pixel ({pixel_x}, {pixel_y})")
                else:
                    print(f"[kitchen_headless]   {path} projected outside image: ({pixel_x}, {pixel_y})")
                    coords_2d[path] = [pixel_x, pixel_y]  # still save, even outside boundary

            return coords_2d

        # 13-15. Use USD API directly (currently not randomizing)
        frame_coordinates = []
        frame_2d_coordinates = []
        frame_hidden_targets = []

        # Do not perform warmup randomization; go directly to capture

        # 16. Main render sequence: initial frame + sequence randomly hiding one prim
        flush_frames_before_capture = 6  # number of flush frames before capture
        print(f"[kitchen_headless] Running hide sequence with {total_frames} frames...")
        print(f"[kitchen_headless] Using {flush_frames_before_capture} flush frames before each capture to eliminate ghosting")

        # Warmup: do nothing, just run a fixed number of frames
        warmup_frames = 40
        print(f"[kitchen_headless] Warmup {warmup_frames} frames before capture...")
        rep.orchestrator.set_capture_on_play(False)
        for _ in range(warmup_frames):
            rep.orchestrator.step()

        # No randomization; only perform hide sequence (cumulative)

        def apply_visibility(hidden_paths: set[str]):
            def set_visibility_recursive(target_prim, visibility, depth=0, max_depth=6):
                if not target_prim or not target_prim.IsValid() or depth > max_depth:
                    return
                if target_prim.GetName() != "Looks":
                    imageable = UsdGeom.Imageable(target_prim)
                    if imageable:
                        vis_attr = imageable.GetVisibilityAttr()
                        if vis_attr and vis_attr.IsValid():
                            vis_attr.Set(visibility)
                for child in target_prim.GetChildren():
                    set_visibility_recursive(child, visibility, depth + 1, max_depth)

            for path in candidate_paths:
                visibility = "invisible" if path in hidden_paths else "inherited"
                prim = stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    set_visibility_recursive(prim, visibility)

        captured_rgb_frames = []
        captured_depth_frames = []
        captured_instance_frames = []
        # Semantic segmentation has been removed and is no longer recorded

        def snapshot_files(patterns):
            files = []
            for pattern in patterns:
                files.extend(raw_dir.glob(pattern))
            return set(files)

        def pick_new_file(before_set, patterns):
            after_set = snapshot_files(patterns)
            new_files = list(after_set - before_set)
            if not new_files:
                return None
            return max(new_files, key=lambda p: p.stat().st_mtime)

        def capture_frame(frame_label: str, hidden_paths: set[str], hidden_target: str | None):
            before_rgb = snapshot_files(["rgb_*.png"])
            before_depth_npy = snapshot_files(["distance_to_camera_*.npy"])
            before_depth_png = snapshot_files(["distance_to_camera_*.png"])
            before_instance = snapshot_files(["instance_segmentation_*.png"])
            apply_visibility(hidden_paths)
            rep.orchestrator.set_capture_on_play(False)
            for _ in range(flush_frames_before_capture):
                rep.orchestrator.step()
            # Trigger capture, ensure at least 1 frame is generated
            rgb_file = None
            depth_file = None
            instance_file = None
            for _ in range(3):
                rep.orchestrator.set_capture_on_play(True)
                rep.orchestrator.step()
                rep.orchestrator.set_capture_on_play(False)
                rgb_file = pick_new_file(before_rgb, ["rgb_*.png"])
                depth_file = pick_new_file(before_depth_npy, ["distance_to_camera_*.npy"])
                if depth_file is None:
                    depth_file = pick_new_file(before_depth_png, ["distance_to_camera_*.png"])
                instance_file = pick_new_file(before_instance, ["instance_segmentation_*.png"])
                if rgb_file:
                    break
            print(f"[kitchen_headless] Captured frame: {frame_label}")

            # Record world and 2D coordinates of the current frame (hidden prims are set to None)
            current_coords = get_world_coordinates(candidate_paths, prim_path_map)
            current_2d_coords = get_2d_center_coordinates_from_projection(
                candidate_paths, cam_path, resolution[0], resolution[1], prim_path_map
            )
            if hidden_target:
                current_coords = {path: None for path in candidate_paths}
                current_2d_coords = {path: None for path in candidate_paths}
            else:
                for hidden in hidden_paths:
                    current_coords[hidden] = None
                    current_2d_coords[hidden] = None
            frame_coordinates.append(current_coords)
            frame_2d_coordinates.append(current_2d_coords)
            frame_hidden_targets.append(hidden_target)

            if rgb_file is None:
                print(f"[kitchen_headless] Warning: No RGB captured for {frame_label}")
            captured_rgb_frames.append(rgb_file)
            captured_depth_frames.append(depth_file)
            captured_instance_frames.append(instance_file)

        # Each pair: A is the initial state; B is a randomly hidden batch (non-cumulative)
        hide_batches = [
            py_random.sample(candidate_paths, hide_batch_size)
            for _ in range(total_pairs)
        ]
        for hide_idx, batch in enumerate(hide_batches):
            print(f"[kitchen_headless] Hide step {hide_idx + 1}/{len(hide_batches)}: {batch}")
            capture_frame(f"pair_{hide_idx:02d}_A", set(), None)
            capture_frame(f"pair_{hide_idx:02d}_B_hide_{batch}", set(batch), batch[0] if batch else None)

        print(f"[kitchen_headless] Rendering complete. Recorded {len(frame_coordinates)} world coord frames, {len(frame_2d_coordinates)} 2D coord frames.")
        
        # Wait for all write operations to complete
        rep.orchestrator.wait_until_complete()
        print(f"[kitchen_headless] All frames written to disk.")

        # 17. Reorganize output (including RGB, depth map, segmentation map)
        import shutil
        
        def sort_frames(frame_paths):
            def frame_index(path_obj):
                stem = path_obj.stem
                if "_" in stem:
                    suffix = stem.rsplit("_", 1)[-1]
                    if suffix.isdigit():
                        return int(suffix)
                return 0
            return sorted(frame_paths, key=frame_index)

        rgb_frames = [p for p in captured_rgb_frames if p]
        depth_frames = [p for p in captured_depth_frames if p]
        instance_frames = [p for p in captured_instance_frames if p]
        if not rgb_frames:
            rgb_frames = sort_frames(raw_dir.glob("rgb_*.png"))
        if not depth_frames:
            depth_frames = sort_frames(raw_dir.glob("distance_to_camera_*.npy"))
            if not depth_frames:
                depth_frames = sort_frames(raw_dir.glob("distance_to_camera_*.png"))
        if not instance_frames:
            instance_frames = sort_frames(raw_dir.glob("instance_segmentation_*.png"))
        
        print(f"[kitchen_headless] Found {len(rgb_frames)} RGB frames")
        print(f"[kitchen_headless] Found {len(depth_frames)} depth frames")
        print(f"[kitchen_headless] Found {len(instance_frames)} instance segmentation frames")
        print(f"[kitchen_headless] Recorded {len(frame_coordinates)} world coordinate snapshots")
        print(f"[kitchen_headless] Recorded {len(frame_2d_coordinates)} 2D coordinate snapshots")
        
        # Verify there are enough frames to create pairs
        min_frames_needed = total_frames
        actual_rgb = len(rgb_frames)
        actual_coords = len(frame_coordinates)
        actual_2d_coords = len(frame_2d_coordinates)

        print(f"[kitchen_headless] Expected: {min_frames_needed} frames")
        print(f"[kitchen_headless] Actual: {actual_rgb} RGB frames, {actual_coords} coord snapshots")

        if actual_rgb < min_frames_needed:
            print(f"[kitchen_headless] ⚠️ Warning: Need {min_frames_needed} frames but found {actual_rgb}")

        if actual_coords < min_frames_needed:
            print(f"[kitchen_headless] ⚠️ Warning: Need {min_frames_needed} coordinate snapshots but found {actual_coords}")

        available_frames = min(actual_rgb, actual_coords, actual_2d_coords)
        available_pairs = available_frames // 2
        pairs_to_create = min(total_pairs, available_pairs)
        pairs_created_total += pairs_to_create
        print(f"[kitchen_headless] Will create {pairs_to_create} pairs")

        for idx in range(pairs_to_create):
            record = pair_records[idx]
            pair_dir = cam_out_dir / record["pair_id"]
            pair_dir.mkdir(parents=True, exist_ok=True)
            
            # Coordinate index and frame file index are the same
            coord_a_idx = idx * 2
            coord_b_idx = idx * 2 + 1

            # Frame file index: starts directly from 0
            frame_a_idx = idx * 2
            frame_b_idx = idx * 2 + 1

            # Fill in coordinate information (only record hidden targets of current pair)
            if coord_a_idx < actual_coords and coord_b_idx < actual_coords:
                coords_a = frame_coordinates[coord_a_idx]
                coords_b = frame_coordinates[coord_b_idx]
                
                coords_2d_a = frame_2d_coordinates[coord_a_idx] if coord_a_idx < actual_2d_coords else {}
                coords_2d_b = frame_2d_coordinates[coord_b_idx] if coord_b_idx < actual_2d_coords else {}
                
                batch = hide_batches[idx] if idx < len(hide_batches) else []
                for hidden_target in batch:
                    record["coordinates"][hidden_target] = {
                        "world_coordinate_A": coords_a.get(hidden_target, None),
                        "world_coordinate_B": None,
                        "pixel_center_A": coords_2d_a.get(hidden_target, None),
                        "pixel_center_B": None,
                    }
            else:
                print(f"[kitchen_headless] ⚠️ Coordinates not available for pair {idx}")
            
            # RGB (use frame file index, skip warmup frames)
            if frame_a_idx < len(rgb_frames) and frame_b_idx < len(rgb_frames):
                shutil.copy2(rgb_frames[frame_a_idx], pair_dir / "A_rgb.png")
                shutil.copy2(rgb_frames[frame_b_idx], pair_dir / "B_rgb.png")
                if mark_centers:
                    from PIL import Image, ImageDraw

                    def draw_center(src_path, dst_path, pixel):
                        if not pixel:
                            return
                        try:
                            img = Image.open(src_path).convert("RGB")
                            draw = ImageDraw.Draw(img)
                            x, y = int(pixel[0]), int(pixel[1])
                            r = 4
                            draw.ellipse((x - r, y - r, x + r, y + r), fill=(255, 0, 0))
                            img.save(dst_path)
                        except Exception as e:
                            print(f"[kitchen_headless] Warning: Failed to draw center on {src_path}: {e}")
            
            # depth map
            if frame_a_idx < len(depth_frames) and frame_b_idx < len(depth_frames):
                depth_ext = depth_frames[frame_a_idx].suffix
                shutil.copy2(depth_frames[frame_a_idx], pair_dir / f"A_depth{depth_ext}")
                shutil.copy2(depth_frames[frame_b_idx], pair_dir / f"B_depth{depth_ext}")

                # If npy format, generate png visualization
                if depth_ext == ".npy":
                    depth_npy_to_png(depth_frames[frame_a_idx], pair_dir / "A_depth.png")
                    depth_npy_to_png(depth_frames[frame_b_idx], pair_dir / "B_depth.png")

            # instance segmentation map
            if frame_a_idx < len(instance_frames) and frame_b_idx < len(instance_frames):
                shutil.copy2(instance_frames[frame_a_idx], pair_dir / "A_instance_segmentation.png")
                shutil.copy2(instance_frames[frame_b_idx], pair_dir / "B_instance_segmentation.png")

            # Save metadata
            meta_path = pair_dir / "metadata.json"
            meta_path.write_text(json.dumps(record, indent=2))

            if mark_centers and record["coordinates"]:
                # Only mark the first hidden target of this pair
                batch = hide_batches[idx] if idx < len(hide_batches) else []
                target = batch[0] if batch else None
                if target and target in record["coordinates"]:
                    coords = record["coordinates"][target]
                    draw_center(pair_dir / "A_rgb.png", pair_dir / "A_rgb_center.png", coords.get("pixel_center_A"))
                    draw_center(pair_dir / "B_rgb.png", pair_dir / "B_rgb_center.png", coords.get("pixel_center_B"))
            
            print(f"[kitchen_headless] Created {record['pair_id']}: A=frame{frame_a_idx}, B=frame{frame_b_idx}")
        
        # Clean up files in the raw directory
        for leftover in raw_dir.glob("*.*"):
            leftover.unlink(missing_ok=True)

        writer.detach()
        render_product.destroy()

    # 18. Done
    rep.orchestrator.wait_until_complete()
    print("\n[kitchen_headless] ========== Done! ==========")
    print(f"Output saved to: {out_dir}")
    return pair_start_index + pairs_created_total


def depth_npy_to_png(npy_path, png_path):
    """
    Convert depth map npy file to a visualizable png image
    """
    import numpy as np
    from PIL import Image

    try:
        # Load depth data
        depth_data = np.load(npy_path)

        # Handle possibly multi-channel data
        if len(depth_data.shape) > 2:
            depth_data = depth_data[:, :, 0] if depth_data.shape[2] >= 1 else depth_data.squeeze()

        # Handle invalid values (such as inf, nan)
        valid_mask = np.isfinite(depth_data)
        if not valid_mask.any():
            print(f"[depth_npy_to_png] Warning: No valid depth values in {npy_path}")
            return False

        # Get the range of valid values
        min_val = depth_data[valid_mask].min()
        max_val = depth_data[valid_mask].max()

        # Normalize to 0-255 range
        if max_val > min_val:
            normalized = (depth_data - min_val) / (max_val - min_val)
        else:
            normalized = np.zeros_like(depth_data)

        # Set invalid values to 0
        normalized[~valid_mask] = 0

        # Invert: near is bright, far is dark
        normalized = 1.0 - normalized

        # Convert to 8-bit image
        depth_uint8 = (normalized * 255).astype(np.uint8)

        # Save as png
        img = Image.fromarray(depth_uint8)
        img.save(png_path)
        return True

    except Exception as e:
        print(f"[depth_npy_to_png] Error converting {npy_path}: {e}")
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Kitchen scene headless rendering")
    parser.add_argument("--run-count", type=int, default=1, help="Number of runs to repeat (reload USD each time)")
    parser.add_argument("--width", type=int, default=1024, help="Image width")
    parser.add_argument("--height", type=int, default=768, help="Image height")
    parser.add_argument("--focal-length", type=float, default=None, help="Camera focal length (optional)")
    parser.add_argument("--mark-centers", action="store_true", help="Draw center markers on RGB images")
    parser.add_argument("--hide-batch-size", type=int, default=1, help="How many prims to hide per pair")
    parser.add_argument("--output-dir", type=str, default="/workspace/output/livingroom_shelf_remove_original/3_items", help="Output directory")
    args = parser.parse_args()

    run_count = args.run_count
    pair_start_index = 0
    for run_idx in range(run_count):
        print(f"\n[kitchen_headless] ========== Run {run_idx + 1}/{run_count} ==========")
        result = run_kitchen_example(
            resolution=(args.width, args.height),
            focal_length=args.focal_length,
            pair_start_index=pair_start_index,
            mark_centers=args.mark_centers,
            hide_batch_size=args.hide_batch_size,
            output_dir=args.output_dir,
        )
        if result is None:
            print("[kitchen_headless] Run aborted due to earlier errors.")
            break
        created_pairs = result - pair_start_index
        print(f"[kitchen_headless] Run {run_idx + 1} completed, created {created_pairs} pairs")
        pair_start_index = result
    simulation_app.close()
