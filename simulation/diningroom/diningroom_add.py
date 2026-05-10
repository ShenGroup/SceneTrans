# Headless version - adapted from the template script
# Suitable for running in container / no-GUI environments
# Run with ./python.sh diningroom_add.py
# Features: randomize specified diningroom prims, place on multiple planes
# Pair differences are expressed via newly added targets in Frame A/B (A is post-hide, B is pre-hide)

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


def run_diningroom_example(
    pair_count: int = 4,
    warmup_k: int = 3,
    num_changes: int = 1,
    origin: bool = False,
    semantic_segmentation: bool = False,
    resolution=(1024, 768),
    focal_length: float | None = None,
    output_dir: str = "/workspace/output/diningroom_add/2_items",
):
    """
    Headless version of the diningroom scene example
    - origin=False: Frame B randomly places controlled objects within the plane, Frame A hides the sampled objects
    - origin=True: Frame B keeps the initial state, Frame A only hides the sampled objects
    """

    # 2. Open the diningroom scene
    usd_path = "/workspace/assets/kujiale_0003/kujiale_0003.usda"
    print(f" Opening stage: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)

    stage = omni.usd.get_context().get_stage()

    if not stage:
        print(" Error: Failed to open stage!")
        return

    # Keep the scene as-is, do not modify any lighting settings
    print(" Scene loaded, preserving original lighting")

    # 3.2 Remove the physics scene and disable global physics to avoid being pushed away by physics after randomization
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene":
            stage.RemovePrim(prim.GetPath())
            print(f" Removed PhysicsScene: {prim.GetPath()}")

    import carb.settings
    carb.settings.get_settings().set("/physics/enabled", False)
    print(" Physics disabled via carb settings")

    # 3. Configure render settings to eliminate ghosting from temporal accumulation
    # RTSubframes: force the renderer to resample every frame
    rep.settings.carb_settings("/omni/replicator/RTSubframes", 50)
    print(" RTSubframes set to 16 (to eliminate ghosting)")



    # 4. Pair with the on_frame trigger to let the orchestrator drive the write to disk
    rep.orchestrator.set_capture_on_play(True)

    # 4. Look up an existing Camera in the scene, using OmniverseKit_Persp
    desired_camera = "/Root/diningroom"
    camera_list = []
    
    cam_prim = stage.GetPrimAtPath(desired_camera)
    if cam_prim and cam_prim.IsValid():
        camera_list = [desired_camera]
        print(f" Using camera: {desired_camera}")
    else:
        # Try to find another camera in the scene as a fallback
        print(f" Warning: Specified camera not found: {desired_camera}")
        for prim in stage.Traverse():
            if prim.GetTypeName() == "Camera":
                camera_list = [prim.GetPath().pathString]
                print(f" Fallback to camera: {camera_list[0]}")
                break

    if not camera_list:
        print(" No camera found, creating a default one.")
        camera_prim = stage.DefinePrim("/World/Camera", "Camera")
        camera_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Float3).Set((0.0, 150.0, 600.0))
        camera_prim.CreateAttribute("focalLength", Sdf.ValueTypeNames.Float).Set(35.0)
        camera_list = ["/OmniverseKit_Persp"]

    # Optional: adjust camera focal length
    if focal_length is not None:
        for cam_path in camera_list:
            cam_prim = stage.GetPrimAtPath(cam_path)
            cam_schema = UsdGeom.Camera(cam_prim)
            if cam_schema and cam_schema.GetFocalLengthAttr():
                cam_schema.GetFocalLengthAttr().Set(float(focal_length))
                print(f" Set focalLength={focal_length} for {cam_path}")

    # 5. BasicWriter: output to the diningroom add directory
    writer = rep.writers.get("BasicWriter")
    out_dir = Path(output_dir)

    # 6. Collect valid prims from the specified prim list, and randomly sample within each pair
    def find_valid_target_prims():
        """Return all valid prims from the user-specified list, grouped by category"""
        glass_prims = [
            "/Root/Meshes/livingroom_767839/wine_glass_0000",
        ]

        kettle_prims = [
            "/Root/Meshes/kitchen_914234340/kettle_0000",
        ]

        cookware_prims = [
            "/Root/Meshes/kitchen_914234340/cookware_0000",
        ]

        plate_prims = [
            "/Root/Meshes/livingroom_767839/plate_0000",
        ]

        blender_prims = [
            "/Root/Meshes/kitchen_914234340/blender_0000",
        ]

        chair_prims = [ 
            "/Root/Meshes/livingroom_767839/chair_0002",
            "/Root/Meshes/livingroom_767839/chair_0003",
            "/Root/Meshes/livingroom_767839/chair_0004",
            "/Root/Meshes/livingroom_767839/chair_0005",
            "/Root/Meshes/livingroom_767839/chair_0006",
        ]



        prim_groups = {
            "chair": chair_prims,
            "glass": glass_prims,
            "kettle": kettle_prims,
            "cookware": cookware_prims,
            "plate": plate_prims,
            "blender": blender_prims,
        }

        # Verify these prims exist
        valid_prim_groups = {}
        for label, prim_list in prim_groups.items():
            valid_prims = []
            for path in prim_list:
                prim = stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    valid_prims.append(path)
                    print(f" Found specified {label} prim: {path}")
                else:
                    print(f" Warning: Specified {label} prim not found: {path}")
            valid_prim_groups[label] = valid_prims

        return valid_prim_groups

    def sample_pair_prims(valid_prims):
        """Randomly sample num_changes prims at the beginning of each pair"""
        if num_changes <= 0:
            print(f" Invalid num_changes={num_changes}, must be > 0")
            return []

        if not valid_prims:
            return []

        sample_k = min(num_changes, len(valid_prims))
        if sample_k < num_changes:
            print(
                f" Warning: num_changes={num_changes} exceeds valid prim count={len(valid_prims)}; using {sample_k}"
            )

        selected_prims = random.sample(valid_prims, sample_k)
        return selected_prims

    # 7. Check and remove the object's collider (randomized objects do not need CollisionAPI)
    def remove_collision_from_bottles(bottle_paths):
        """Check and remove the object's CollisionAPI; randomized objects do not need a collider"""
        for path in bottle_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                print(f" ❌ Prim not found: {path}")
                continue

            removed_count = 0

            # Disable instanceable
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                print(f" Made prim non-instanceable: {path}")

            # Check and remove the Xform's own CollisionAPI
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
                print(f" ✓ Removed CollisionAPI from Xform: {path}")
                removed_count += 1
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                print(f" ✓ Removed RigidBodyAPI from Xform: {path}")
                removed_count += 1
            
            # Recursively check the CollisionAPI of all children
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
                        print(f" ✓ Removed CollisionAPI from {child_type}: {child_path}")
                        removed_count += 1
                    if child.HasAPI(UsdPhysics.RigidBodyAPI):
                        child.RemoveAPI(UsdPhysics.RigidBodyAPI)
                        print(f" ✓ Removed RigidBodyAPI from {child_type}: {child_path}")
                        removed_count += 1
                    
                    # Continue recursively checking child nodes
                    remove_collision_from_children(child, depth + 1)
            
            remove_collision_from_children(prim)
            
            if removed_count > 0:
                print(f" Physics cleanup done: {path} (removed {removed_count} CollisionAPI)")
            else:
                print(f" No CollisionAPI found on: {path}")

    # 8. Add semantic labels to randomized objects (for instance/semantic segmentation)
    def add_semantic_labels(prim_label_map):
        """
        Add semantic labels to specified prims (set per prim_groups category)
        Use two approaches at once to ensure compatibility:
        1. Write semantic attributes directly via USD primvars (effective immediately)
        2. Use rep.modify.semantics (for the Replicator graph)
        """
        print(" Adding semantic labels using dual approach...")
        
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
        
        for path, prim_name in prim_label_map.items():
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f" ❌ Prim not found: {path}")
                continue

            try:
                # Disable instanceable to ensure primvars can be written
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

                # Add semantic labels via rep.modify.semantics
                prim_group = rep.get.prims(path_pattern=path)
                with prim_group:
                    rep.modify.semantics([("class", prim_name)])

                added_count += 1
                print(f" ✓ Semantic label added: {path} -> '{prim_name}' (meshes: {len(mesh_children)})")

                # Print verification (root + first mesh)
                def log_semantic_attrs(target_prim, label):
                    attrs = [
                        (attr.GetName(), attr.Get())
                        for attr in target_prim.GetAttributes()
                        if "semantic" in attr.GetName().lower()
                    ]
                    if attrs:
                        for name, val in attrs:
                            print(f"   {label} attr {name} = {val}")
                    else:
                        print(f"   {label} has no semantic attrs")

                log_semantic_attrs(prim, f"{path}")
                if mesh_children:
                    log_semantic_attrs(mesh_children[0], f"{mesh_children[0].GetPath().pathString}")
                
            except Exception as e:
                print(f" ❌ Failed to add label to {path}: {e}")
        
        return added_count

    # 9. Find and verify prims
    valid_prim_groups = find_valid_target_prims()
    valid_candidate_paths = []
    path_label_map = {}
    for label, paths in valid_prim_groups.items():
        valid_candidate_paths.extend(paths)
        for p in paths:
            path_label_map[p] = label
    non_chair_randomize_candidate_paths = []
    for label, paths in valid_prim_groups.items():
        if label != "chair":
            non_chair_randomize_candidate_paths.extend(paths)

    if not valid_candidate_paths:
        print(" No specified prims were found; cannot proceed.")
        return
    group_count_msg = ", ".join(
        f"{label}={len(paths)}" for label, paths in valid_prim_groups.items()
    )
    print(
        f" Found {len(valid_candidate_paths)} valid prims "
        f"({group_count_msg})"
    )

    # 9.1 Resolve the movable prim (determined by bbox change)
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
    for path in valid_candidate_paths:
        movable = resolve_movable_prim(path)
        if movable:
            prim_path_map[path] = movable
            if movable != path:
                print(f" Using movable prim: {path} -> {movable}")
        else:
            print(f" Warning: No movable prim resolved for {path}")

    if not prim_path_map:
        print(" No movable prims resolved; cannot proceed.")
        return

    # All categories except chair participate in random placement; chair stays in its current position
    non_chair_prim_path_map = {
        path: prim_path_map[path]
        for path in non_chair_randomize_candidate_paths
        if path in prim_path_map
    }
    print(f" Non-chair prims for plane randomization: {len(non_chair_prim_path_map)}")

    reverse_prim_map = {movable: orig for orig, movable in prim_path_map.items()}
    movable_paths = list(prim_path_map.values())
    
    # Remove the physics colliders from these prims (if any)
    remove_collision_from_bottles(valid_candidate_paths)

    # Apply semantic labels to all objects to be randomized
    added = add_semantic_labels(path_label_map)
    print(f" Semantic labels added: {added} prims labeled")

    # 10. Use the specified planes for random placement
    cabinet_prims = [
        "Root/diningroom_plane",
    ]

    def normalize_prim_path(path: str) -> str:
        """Ensure the path starts with /, compatible with inputs lacking a leading slash"""
        return path if path.startswith("/") else f"/{path}"

    # Verify cabinet prims exist
    valid_cabinet_prims = []
    for cabinet_prim in cabinet_prims:
        cabinet_prim = normalize_prim_path(cabinet_prim)
        cabinet_prim_obj = stage.GetPrimAtPath(cabinet_prim)
        if cabinet_prim_obj and cabinet_prim_obj.IsValid():
            valid_cabinet_prims.append(cabinet_prim)
            print(f" ✓ Found cabinet prim: {cabinet_prim}")
        else:
            print(f" ❌ Warning: Cabinet prim not found: {cabinet_prim}")
    
    if not valid_cabinet_prims:
        print(f" ❌ Error: No valid cabinet prims found!")
        return
    
    # 10.1 Get the surface Z height (for Z compensation)
    def get_surface_z_height(surface_prim_paths):
        """
        Get the highest Z coordinate of the surface, used as the reference height for object placement
        """
        print(f"\n[Surface-Z] ========== Compute surface Z height ==========")
        max_z = float('-inf')
        for path in surface_prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[Surface-Z] ❌ Invalid Prim: {path}")
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
                print(f"[Surface-Z]   ⚠️ Failed to compute bbox or bbox is empty")

                # Try to fall back to the transform
                xformable = UsdGeom.Xformable(prim)
                if xformable:
                    world_transform = xformable.ComputeLocalToWorldTransform(0)
                    translation = world_transform.ExtractTranslation()
                    print(f"[Surface-Z]   Transform position: ({translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f})")
                    # Use the transform's Z as a fallback
                    if translation[2] > max_z:
                        max_z = translation[2]
                        print(f"[Surface-Z]   Using transform Z as the surface height: {translation[2]:.4f}")

        result = max_z if max_z != float('-inf') else 0.0
        print(f"[Surface-Z] Final surface Z height: {result}")
        print(f"[Surface-Z] ==========================================\n")
        return result

    # Get the surface height (take the maximum surface_z from bounds_list)
    # Note: get_surface_z_height may return 0, so we update after computing bounds_list
    surface_z_height = get_surface_z_height(valid_cabinet_prims)

    # 10.2 Z compensation function: invoked after randomization + rotation, to prevent objects from sinking into the plane
    def apply_z_compensation(prim_surface_z_map, prim_display_map=None):
        """
        Compensate the Z height based on the object's current-pose bounding box, so the bottom rests on the surface

        Core logic:
        1. Get the current Z value from the object's translate op
        2. Compute the gap between the bbox bottom and the target surface Z
        3. Adjust Z so that bbox bottom = surface_z of the plane this object sits on

        Args:
            prim_surface_z_map: {prim_path: surface_z} the plane Z height each object is placed on
        """
        if not prim_surface_z_map:
            print(f"\n[Z-comp] ⚠️ No objects need compensation")
            return

        print(f"\n[Z-comp] ========== Start Z compensation ==========")
        
        bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
        
        for path, target_surface_z in prim_surface_z_map.items():
            display_name = prim_display_map.get(path, path) if prim_display_map else path
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[Z-comp] ❌ Invalid Prim: {display_name}")
                continue

            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                print(f"[Z-comp] ❌ Failed to get Xformable: {display_name}")
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
                print(f"[Z-comp] ⚠️ Failed to compute bbox: {display_name}")
                continue

            bbox_min_z = world_bbox.GetRange().GetMin()[2]

            # Compute the required Z adjustment: align the bbox bottom to the surface_z of the plane this object sits on
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

    # Whether to enable Z compensation (can be disabled if Z is forced directly)
    use_z_compensation = True

    # 11. Find the actual Mesh prim (scatter_2d requires a direct Mesh, not a container)
    def find_actual_mesh(prim_path, depth=0, max_depth=3):
        """
        Recursively find the actual Mesh child under prim
        Because scatter_2d requires a direct Mesh prim, not a container that holds a Mesh
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
            print(f" Using mesh for scatter_2d: {actual_mesh}")
        else:
            # If no child Mesh is found, try using the prim directly
            print(f" Will try using prim directly: {cabinet_prim}")
            actual_cabinet_meshes.append(cabinet_prim)

    if not actual_cabinet_meshes:
        print(" ❌ Error: No valid meshes found for scatter_2d")
        return

    # Use cabinet prims as the surface for scatter_2d
    surface_nodes = []
    for mesh_path in actual_cabinet_meshes:
        prim_node = rep.get.prims(path_pattern=mesh_path)
        if prim_node:
            surface_nodes.append(prim_node)
            print(f" Added surface: {mesh_path}")
    
    if not surface_nodes:
        print(" ❌ Error: No valid surface found for scatter_2d")
        return
    
    # If there are multiple surfaces, create a group; otherwise use the single surface directly
    if len(surface_nodes) == 1:
        surface = surface_nodes[0]
    else:
        surface = rep.create.group(surface_nodes)
        print(f" Created surface group with {len(surface_nodes)} meshes")

    # 11.5 Compute the world coordinate range of each surface (for USD API randomization)
    def get_each_surface_bounds(surface_prim_paths):
        """
        Get the world coordinate range of each surface (stored separately, not merged)
        Use the world transform matrix + Mesh extent attribute to precisely compute the range
        Returns: [(min_x, max_x, min_y, max_y, surface_z), ...]
        """
        surface_bounds_list = []

        print(f"\n[Surface-Bounds] ========== Compute range of each surface ==========")

        def find_mesh_in_prim(parent_prim, depth=0, max_depth=5):
            """Recursively find the Mesh child under prim"""
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
                print(f"[Surface-Bounds] ❌ Invalid Prim: {path}")
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
                print(f"[Surface-Bounds] ❌ Failed to get Xformable: {target_path}")
                continue

            world_transform = target_xformable.ComputeLocalToWorldTransform(0)

            # If extent is available, use its four corners to compute the world range
            if local_extent and len(local_extent) >= 2:
                local_min = local_extent[0]
                local_max = local_extent[1]

                # Build 8 corner points in local space (for 3D boxes) or 4 corners (for planes)
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
                print(f"[Surface-Bounds]   ⚠️ No extent, falling back to unit-plane assumption")
                local_corners = [
                    Gf.Vec3d(-0.5, -0.5, 0),
                    Gf.Vec3d(0.5, -0.5, 0),
                    Gf.Vec3d(-0.5, 0.5, 0),
                    Gf.Vec3d(0.5, 0.5, 0),
                ]

            # Transform to world coordinates and compute the range
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

            # Add a 5% margin
            range_x = max_x - min_x
            range_y = max_y - min_y
            margin_x = range_x * 0.05
            margin_y = range_y * 0.1
            min_x += margin_x
            max_x -= margin_x
            min_y += margin_y
            max_y -= margin_y

            # Use the range with margins
            bounds = (min_x, max_x, min_y, max_y, surface_z)
            surface_bounds_list.append(bounds)

            print(f"[Surface-Bounds]   Final range: X=[{bounds[0]:.2f}, {bounds[1]:.2f}], Y=[{bounds[2]:.2f}, {bounds[3]:.2f}], Z={surface_z:.2f}")

        print(f"[Surface-Bounds] {len(surface_bounds_list)} valid surfaces in total")
        print(f"[Surface-Bounds] ==========================================\n")

        return surface_bounds_list

    # Get the range of each surface (list)
    surface_bounds_list = get_each_surface_bounds(valid_cabinet_prims)

    # Get the correct surface_z_height from bounds_list (take the maximum)
    # This is more reliable than get_surface_z_height
    if surface_bounds_list:
        surface_z_height = max(bounds[4] for bounds in surface_bounds_list)
        print(f" ✓ surface_z_height updated from bounds_list to: {surface_z_height:.4f}")

    # 11.6 Pure USD API randomization function (full control, no dependence on the Replicator trigger)
    import random as py_random

    def manual_randomize_objects(prim_path_map, bounds_list):
        """
        Randomize object position and rotation using the pure USD API
        Each object randomly chooses a plane, then is placed within that plane's range

        Args:
            prim_path_map: {original_path: movable_path}
            bounds_list: list of surface ranges [(min_x, max_x, min_y, max_y, surface_z), ...]

        Returns:
            dict: {prim_path: surface_z} the plane Z height each object is placed on
        """
        prim_surface_z_map = {}
        # Record (x, y) already placed on each surface, for the minimum spacing constraint
        placed_points_by_surface = {idx: [] for idx in range(len(bounds_list))}
        max_position_attempts = 40

        if not bounds_list:
            print(f"[Manual-Rand] ❌ No valid surface ranges")
            return prim_surface_z_map

        print(f"\n[Manual-Rand] ========== Start USD API randomization ==========")
        print(f"[Manual-Rand] Available surface count: {len(bounds_list)}")

        for orig_path, move_path in prim_path_map.items():
            prim = stage.GetPrimAtPath(move_path)
            if not prim or not prim.IsValid():
                print(f"[Manual-Rand] ❌ Invalid Prim: {move_path} (orig {orig_path})")
                continue

            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                print(f"[Manual-Rand] ❌ Failed to get Xformable: {path}")
                continue

            # Randomly choose a plane
            surface_idx = py_random.randrange(len(bounds_list))
            min_x, max_x, min_y, max_y, surface_z = bounds_list[surface_idx]

            # Record the surface Z height this object is placed on
            prim_surface_z_map[move_path] = surface_z

            # Randomly generate a position (within the chosen plane), constrained by minimum spacing to placed objects
            range_x = max_x - min_x
            range_y = max_y - min_y
            # Fixed minimum 2D spacing of 5cm (assuming scene units are meters)
            min_spacing = 0.08
            existing_points = placed_points_by_surface[surface_idx]

            rand_x = None
            rand_y = None
            used_attempts = 0
            best_candidate = None
            best_candidate_min_dist = -1.0

            for attempt in range(1, max_position_attempts + 1):
                cand_x = py_random.uniform(min_x, max_x)
                cand_y = py_random.uniform(min_y, max_y)
                used_attempts = attempt

                if not existing_points:
                    rand_x, rand_y = cand_x, cand_y
                    break

                dists = [
                    ((cand_x - ex_x) ** 2 + (cand_y - ex_y) ** 2) ** 0.5
                    for ex_x, ex_y in existing_points
                ]
                nearest_dist = min(dists) if dists else float("inf")

                if nearest_dist >= min_spacing:
                    rand_x, rand_y = cand_x, cand_y
                    break

                if nearest_dist > best_candidate_min_dist:
                    best_candidate_min_dist = nearest_dist
                    best_candidate = (cand_x, cand_y)

            if rand_x is None or rand_y is None:
                if best_candidate is not None:
                    rand_x, rand_y = best_candidate
                    print(
                        f"[Manual-Rand] ⚠️ {orig_path}: still cannot satisfy minimum spacing after {max_position_attempts} attempts "
                        f"(target={min_spacing:.3f}, best={best_candidate_min_dist:.3f}), using the best candidate point"
                    )
                else:
                    rand_x = py_random.uniform(min_x, max_x)
                    rand_y = py_random.uniform(min_y, max_y)
                    print(
                        f"[Manual-Rand] ⚠️ {orig_path}: no usable candidate-point record, falling back to plain random sampling"
                    )

            placed_points_by_surface[surface_idx].append((rand_x, rand_y))
            # Force Z directly, without relying on bbox compensation
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
            
            # Set the position
            if translate_op:
                translate_op.Set(Gf.Vec3d(rand_x, rand_y, rand_z))
            else:
                translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
                translate_op.Set(Gf.Vec3d(rand_x, rand_y, rand_z))

            # Set the rotation
            if rotate_op:
                rotate_op.Set(Gf.Vec3f(rand_rot_x, rand_rot_y, rand_rot_z))
            else:
                rotate_op = xformable.AddRotateXYZOp(UsdGeom.XformOp.PrecisionFloat)
                rotate_op.Set(Gf.Vec3f(rand_rot_x, rand_rot_y, rand_rot_z))
            
            state = "standing" if rand_rot_x == 0 else "fallen"

            # Verify the position is within range
            in_range_x = min_x <= rand_x <= max_x
            in_range_y = min_y <= rand_y <= max_y
            if not in_range_x or not in_range_y:
                print(f"[Manual-Rand] ❌ {orig_path}: position out of range!")
                print(f"[Manual-Rand]   bounds: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}]")
                print(f"[Manual-Rand]   pos: X={rand_x:.2f} {'✓' if in_range_x else '❌'}, Y={rand_y:.2f} {'✓' if in_range_y else '❌'}")

            print(
                f"[Manual-Rand] ✓ {orig_path}: plane{surface_idx}(Z={surface_z:.2f}), "
                f"bounds=X[{min_x:.2f},{max_x:.2f}],Y[{min_y:.2f},{max_y:.2f}], "
                f"pos=({rand_x:.2f}, {rand_y:.2f}, {rand_z:.2f}), "
                f"rot=({rand_rot_x}, {rand_rot_y}, {rand_rot_z:.0f}) [{state}], "
                f"min_spacing={min_spacing:.3f}, attempts={used_attempts}"
            )

        print(f"[Manual-Rand] ========== Randomization done ==========\n")
        return prim_surface_z_map

    # 12. Render for each camera
    for cam_path in camera_list:
        print(f"\n ========== Rendering camera: {cam_path} ==========")

        render_product = rep.create.render_product(cam_path, resolution)

        cam_out_dir = out_dir
        raw_dir = cam_out_dir / "_raw_frames"
        os.makedirs(raw_dir, exist_ok=True)
        
        # First run warmup frames to flush the temporal cache (remove residual ghosting from the original positions)
        # warmup frame count = 3 * warmup_k, ensuring it is a multiple of 3
        warmup_frames = 3 * warmup_k
        if origin:
            print(f" Running {warmup_frames} warmup frames (k={warmup_k}) in origin mode (no reposition).")
        else:
            print(f" Running {warmup_frames} warmup frames (k={warmup_k}) with randomization...")
        rep.orchestrator.set_capture_on_play(False)

        # During warmup, move objects directly to random positions via the USD API
        # This flushes the temporal cache from the original positions
        # Move once every 3 frames, for a total of warmup_k moves
        import random as py_random
        for warmup_i in range(warmup_frames):
            # Move objects once every 3 frames to refresh the cache
            if (not origin) and warmup_i % 3 == 0:
                for path in non_chair_randomize_candidate_paths:
                    target_path = prim_path_map.get(path, path)
                    prim = stage.GetPrimAtPath(target_path)
                    if prim:
                        xformable = UsdGeom.Xformable(prim)
                        if xformable:
                            # Get the existing translate op
                            for op in xformable.GetOrderedXformOps():
                                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                    # Nudge the position to refresh the cache
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
        
        if origin:
            print(f" Warmup done in origin mode (positions preserved).")
        else:
            print(f" Warmup done, original positions flushed from cache.")

        # Diagnostic: pull a frame of segmentation data directly from the annotator to confirm it is non-zero
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

        # log_annotator_sample(render_product)  # Diagnostic temporarily disabled to avoid extra step()

        # Clean up old files in the directory
        for old_file in raw_dir.glob("*.*"):
            old_file.unlink(missing_ok=True)

        # Now initialize and attach the writer (after warmup)
        writer.initialize(
            output_dir=str(raw_dir),
            rgb=True,
            distance_to_camera=True,           # depth map
            semantic_segmentation=semantic_segmentation,        # semantic segmentation map (toggleable)
            colorize_semantic_segmentation=semantic_segmentation,  # colorized semantic segmentation (toggleable)
            instance_segmentation=True,        # instance segmentation map
            colorize_instance_segmentation=True,  # colorized instance segmentation
            bounding_box_2d_tight=False
        )
        writer.attach(render_product)

        # Enable capture
        rep.orchestrator.set_capture_on_play(True)
        print(f" Writer attached, starting capture...")

        # Create the pair plan
        total_frames = pair_count * 2
        pair_records = []
        
        for pair_idx in range(pair_count):
            pair_records.append({
                "scene_type": "diningroom",
                "change_type": "add",
                "pair_id": f"pair_{pair_idx:04d}",
                "camera": cam_path,
                "num_changes": 0,
                "selected_prims": [],
                "coordinates": {},  # to be filled in during rendering
            })

        print(f" Generating {pair_count} pairs ({total_frames} frames)")

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
                        # Extract the translation (world coordinate)
                        translation = world_transform.ExtractTranslation()
                        coords[path] = [translation[0], translation[1], translation[2]]
            return coords

        def get_2d_center_coordinates_from_projection(prim_paths, camera_path, image_width, image_height, prim_map=None):
            """
            Project 3D world coordinates to 2D pixel coordinates using the camera projection matrix (no dependence on semantic segmentation)

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
                print(f" Warning: Camera not found: {camera_path}")
                for path in prim_paths:
                    coords_2d[path] = None
                return coords_2d
            
            camera = UsdGeom.Camera(camera_prim)
            if not camera:
                print(f" Warning: Failed to get UsdGeom.Camera for {camera_path}")
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
                print(f" Warning: Camera parameters not available")
                for path in prim_paths:
                    coords_2d[path] = None
                return coords_2d
            
            # If v_aperture is None, derive it from the aspect ratio
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
                
                # Get the object's world coordinate
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                world_pos = world_transform.ExtractTranslation()

                # Transform world coordinates into camera space
                # Use TransformAffine to handle 3D points
                world_pos_3d = Gf.Vec3d(world_pos[0], world_pos[1], world_pos[2])
                camera_pos = world_to_camera.TransformAffine(world_pos_3d)

                # Position in camera space
                cam_x = camera_pos[0]
                cam_y = camera_pos[1]
                cam_z = camera_pos[2]

                # The camera looks at -Z, so z must be negative to be visible
                if cam_z >= 0:
                    # Object is behind the camera
                    print(f"   {path} is behind camera (z={cam_z:.2f})")
                    coords_2d[path] = None
                    continue

                # Perspective projection (pinhole camera model)
                # NDC coordinates (normalized device coordinates)
                ndc_x = (focal_length * cam_x) / (-cam_z * h_aperture / 2)
                ndc_y = (focal_length * cam_y) / (-cam_z * v_aperture / 2)

                # NDC range is [-1, 1], convert to pixel coordinates
                # Note: the y axis points downward in the image
                pixel_x = int((ndc_x + 1) * 0.5 * image_width)
                pixel_y = int((1 - ndc_y) * 0.5 * image_height)  # flip Y

                # Check whether it is within the image range
                if 0 <= pixel_x < image_width and 0 <= pixel_y < image_height:
                    coords_2d[path] = [pixel_x, pixel_y]
                    print(f"   Projected {path}: world ({world_pos[0]:.1f}, {world_pos[1]:.1f}, {world_pos[2]:.1f}) -> pixel ({pixel_x}, {pixel_y})")
                else:
                    print(f"   {path} projected outside image: ({pixel_x}, {pixel_y})")
                    coords_2d[path] = [pixel_x, pixel_y]  # still save, even outside the bounds

            return coords_2d

        def set_prims_visibility(prim_paths, visible: bool, prim_map=None):
            """
            Set prim visibility.
            - visible=True: show
            - visible=False: hide
            """
            changed = []
            for path in prim_paths:
                target_path = prim_map.get(path, path) if prim_map else path
                prim = stage.GetPrimAtPath(target_path)
                if not prim or not prim.IsValid():
                    print(f" Warning: Prim not found for visibility toggle: {target_path}")
                    continue
                if not prim.IsA(UsdGeom.Imageable):
                    print(f" Warning: Prim is not Imageable: {target_path}")
                    continue

                imageable = UsdGeom.Imageable(prim)
                if visible:
                    imageable.MakeVisible()
                else:
                    imageable.MakeInvisible()
                changed.append(path)
            return changed

        # 13-15. Randomize directly via the USD API, without using a Replicator trigger
        # This gives us full control over when randomization happens, avoiding Z compensation being overwritten

        frame_counter = [0]
        frame_coordinates = []
        frame_2d_coordinates = []

        # Warmup step: run multiple frames to refresh the renderer's temporal accumulation cache and eliminate ghosting
        # Run 3 frames after each randomization to let the renderer stabilize, 5 randomizations total = 15 frames
        warmup_randomizations = 5
        warmup_steps_per_rand = 3
        if origin:
            print(
                f" Running warmup in origin mode: {warmup_randomizations} iterations × "
                f"{warmup_steps_per_rand} steps (no reposition)"
            )
        else:
            print(f" Running warmup: {warmup_randomizations} randomizations × {warmup_steps_per_rand} steps = {warmup_randomizations * warmup_steps_per_rand} frames...")
        
        # Detach writer during warmup to avoid capturing these frames
        writer.detach()

        for warmup_i in range(warmup_randomizations):
            if origin:
                set_prims_visibility(valid_candidate_paths, True, prim_path_map)
            else:
                prim_z_map = manual_randomize_objects(non_chair_prim_path_map, surface_bounds_list)
                if use_z_compensation:
                    apply_z_compensation(prim_z_map, reverse_prim_map)
            # Run several frames after each randomization to refresh temporal accumulation
            for _ in range(warmup_steps_per_rand):
                rep.orchestrator.step()

        # Re-attach writer to start the formal capture
        writer.attach(render_product)
        print(f" Warmup complete, writer re-attached.")

        # 16. Main render loop
        # Run several frames before each capture to refresh the temporal accumulation cache and remove ghosting between A/B frames
        flush_frames_before_capture = 6  # number of flush frames before capture (increased to 6)
        print(f" Running main render loop with {total_frames} frames...")
        print(f" Using {flush_frames_before_capture} flush frames before each capture to eliminate ghosting")

        for frame_idx in range(total_frames):
            frame_counter[0] += 1
            print(f"\n ===== Frame {frame_idx} =====")

            # Resample the set of prims to hide once per pair (A/B two frames)
            pair_idx = frame_idx // 2
            # B is pre-hide (originally visible), A is post-hide
            if frame_idx % 2 == 0:
                sampled_paths = sample_pair_prims(valid_candidate_paths)
                sampled_paths = [p for p in sampled_paths if p in prim_path_map]
                pair_records[pair_idx]["selected_prims"] = sampled_paths
                pair_records[pair_idx]["num_changes"] = len(sampled_paths)
                print(f" Pair {pair_idx}: selected {len(sampled_paths)} prim(s) to hide in A: {sampled_paths}")

                # Frame B (pre-hide): first ensure all controlled objects are visible, then perform normal randomization
                set_prims_visibility(valid_candidate_paths, True, prim_path_map)
                tracked_paths = list(prim_path_map.keys())
                tracked_prim_map = prim_path_map

                if origin:
                    print(f" Pair {pair_idx}: B frame keeps original layout (origin mode)")
                else:
                    # B randomizes categories other than chair; chair stays in its current position
                    prim_z_map = manual_randomize_objects(non_chair_prim_path_map, surface_bounds_list)
                    if use_z_compensation:
                        apply_z_compensation(prim_z_map, reverse_prim_map)
            else:
                sampled_paths = pair_records[pair_idx].get("selected_prims", [])
                tracked_paths = list(prim_path_map.keys())
                tracked_prim_map = prim_path_map
                # Frame A (post-hide): no longer move objects, only hide the sampled objects
                hidden_paths = set_prims_visibility(sampled_paths, False, prim_path_map)
                print(f" Pair {pair_idx}: A frame hidden {len(hidden_paths)} prim(s): {hidden_paths}")

            # Step 3: flush frames - detach writer, run several frames to refresh the temporal accumulation cache
            writer.detach()
            for _ in range(flush_frames_before_capture):
                rep.orchestrator.step()

            # Step 4: render and capture - attach writer, capture the final frame
            writer.attach(render_product)
            rep.orchestrator.step()
            print(f" Frame {frame_idx}: rendered and captured (after {flush_frames_before_capture} flush frames)")

            # Verify the actual object positions after capture
            print(f"[Verify] ===== Position verification after capture =====")
            for path in tracked_paths:
                target_path = tracked_prim_map.get(path, path)
                prim = stage.GetPrimAtPath(target_path)
                if prim and prim.IsValid():
                    xformable = UsdGeom.Xformable(prim)
                    if xformable:
                        # Get the value of the translate op
                        for op in xformable.GetOrderedXformOps():
                            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                pos = op.Get()
                                print(f"[Verify] {path}: translate = ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
                                break
                        # Compute bbox
                        bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
                        world_bbox = bbox_cache.ComputeWorldBound(prim)
                        if world_bbox and not world_bbox.GetRange().IsEmpty():
                            bbox_min = world_bbox.GetRange().GetMin()
                            print(f"[Verify]   bbox_min_z = {bbox_min[2]:.4f}")

            # Record world coordinates after rendering
            current_coords = get_world_coordinates(tracked_paths, tracked_prim_map)
            frame_coordinates.append(current_coords)

            # Compute 2D pixel coordinates via camera projection
            current_2d_coords = get_2d_center_coordinates_from_projection(
                tracked_paths, cam_path, resolution[0], resolution[1], tracked_prim_map
            )
            frame_2d_coordinates.append(current_2d_coords)
            print(f" Frame {frame_idx}: recorded coords for {len(current_coords)} prims")

        print(f" Rendering complete. Recorded {len(frame_coordinates)} world coord frames, {len(frame_2d_coordinates)} 2D coord frames.")

        # Wait for all write operations to complete
        rep.orchestrator.wait_until_complete()
        print(f" All frames written to disk.")

        # 17. Reorganize output (includes RGB, depth map, segmentation)
        import shutil
        
        rgb_frames = sorted(raw_dir.glob("rgb_*.png"))
        depth_frames = sorted(raw_dir.glob("distance_to_camera_*.npy"))
        if not depth_frames:
            depth_frames = sorted(raw_dir.glob("distance_to_camera_*.png"))
        instance_frames = sorted(raw_dir.glob("instance_segmentation_*.png"))
        semantic_frames = sorted(raw_dir.glob("semantic_segmentation_*.png")) if semantic_segmentation else []
        
        print(f" Found {len(rgb_frames)} RGB frames")
        print(f" Found {len(depth_frames)} depth frames")
        print(f" Found {len(instance_frames)} instance segmentation frames")
        print(f" Found {len(semantic_frames)} semantic segmentation frames")
        print(f" Recorded {len(frame_coordinates)} world coordinate snapshots")
        print(f" Recorded {len(frame_2d_coordinates)} 2D coordinate snapshots")
        
        # Verify there are enough frames to create pairs
        # New logic: writer is detached during warmup, no files are produced; only formal render frames
        # Expected frame count = pair_count * 2
        min_frames_needed = pair_count * 2
        actual_rgb = len(rgb_frames)
        actual_coords = len(frame_coordinates)
        
        print(f" Expected: {min_frames_needed} frames (warmup not captured)")
        print(f" Actual: {actual_rgb} RGB frames, {actual_coords} coord snapshots")
        
        if actual_rgb < min_frames_needed:
            print(f" ⚠️ Warning: Need {min_frames_needed} frames but found {actual_rgb}")
            # Create as many pairs as possible
            pair_count = min(pair_count, actual_rgb // 2)
        
        if actual_coords < min_frames_needed:
            print(f" ⚠️ Warning: Need {min_frames_needed} coordinate snapshots but found {actual_coords}")
        
        print(f" Will create {pair_count} pairs")
        
        for idx in range(pair_count):
            record = pair_records[idx]
            pair_dir = cam_out_dir / record["pair_id"]
            pair_dir.mkdir(parents=True, exist_ok=True)
            
            # Coordinate indices and frame file indices are the same (warmup produces no files)
            # Capture order is B(pre-hide) -> A(post-hide); swap to A/B naming on export
            coord_b_src_idx = idx * 2
            coord_a_src_idx = idx * 2 + 1
            
            # Frame file indices: start from 0
            frame_b_src_idx = idx * 2
            frame_a_src_idx = idx * 2 + 1

            # Populate coordinate information (using coordinate indices)
            actual_2d_coords = len(frame_2d_coordinates)
            if coord_a_src_idx < actual_coords and coord_b_src_idx < actual_coords:
                coords_a = frame_coordinates[coord_a_src_idx]
                coords_b = frame_coordinates[coord_b_src_idx]

                # Get 2D coordinates (if available)
                coords_2d_a = frame_2d_coordinates[coord_a_src_idx] if coord_a_src_idx < actual_2d_coords else {}
                coords_2d_b = frame_2d_coordinates[coord_b_src_idx] if coord_b_src_idx < actual_2d_coords else {}

                # Build the coordinates dict including world coordinates and 2D center pixel coordinates
                selected_paths_for_pair = pair_records[idx].get("selected_prims", [])
                for prim_path in selected_paths_for_pair:
                    # add task: A is post-hide (coordinate null), B is pre-hide (record actual coordinate)
                    record["coordinates"][prim_path] = {
                        "world_coordinate_A": None,
                        "world_coordinate_B": coords_b.get(prim_path, None),
                        "pixel_center_A": None,
                        "pixel_center_B": coords_2d_b.get(prim_path, None),
                    }
            else:
                print(f" ⚠️ Coordinates not available for pair {idx}")
            
            # RGB (use frame file indices, skipping warmup frames)
            if frame_a_src_idx < len(rgb_frames) and frame_b_src_idx < len(rgb_frames):
                shutil.copy2(rgb_frames[frame_a_src_idx], pair_dir / "A_rgb.png")
                shutil.copy2(rgb_frames[frame_b_src_idx], pair_dir / "B_rgb.png")

            # Depth map
            if frame_a_src_idx < len(depth_frames) and frame_b_src_idx < len(depth_frames):
                depth_ext = depth_frames[frame_a_src_idx].suffix
                shutil.copy2(depth_frames[frame_a_src_idx], pair_dir / f"A_depth{depth_ext}")
                shutil.copy2(depth_frames[frame_b_src_idx], pair_dir / f"B_depth{depth_ext}")

                # If npy format, generate a png visualization
                if depth_ext == ".npy":
                    depth_npy_to_png(depth_frames[frame_a_src_idx], pair_dir / "A_depth.png")
                    depth_npy_to_png(depth_frames[frame_b_src_idx], pair_dir / "B_depth.png")

            # Instance segmentation map
            if frame_a_src_idx < len(instance_frames) and frame_b_src_idx < len(instance_frames):
                shutil.copy2(instance_frames[frame_a_src_idx], pair_dir / "A_instance_segmentation.png")
                shutil.copy2(instance_frames[frame_b_src_idx], pair_dir / "B_instance_segmentation.png")

            # Semantic segmentation map (disabled by default)
            if semantic_segmentation and frame_a_src_idx < len(semantic_frames) and frame_b_src_idx < len(semantic_frames):
                shutil.copy2(semantic_frames[frame_a_src_idx], pair_dir / "A_semantic_segmentation.png")
                shutil.copy2(semantic_frames[frame_b_src_idx], pair_dir / "B_semantic_segmentation.png")

            # Save metadata
            record_to_save = {k: v for k, v in record.items() if k != "selected_prims"}
            meta_path = pair_dir / "metadata.json"
            meta_path.write_text(json.dumps(record_to_save, indent=2))
            
            print(f" Created {record['pair_id']}: A=frame{frame_a_src_idx}(hidden), B=frame{frame_b_src_idx}(visible)")
        
        # Clean up files in the raw directory
        for leftover in raw_dir.glob("*.*"):
            leftover.unlink(missing_ok=True)

        writer.detach()
        render_product.destroy()

    # 18. Done
    rep.orchestrator.wait_until_complete()
    print("\n ========== Done! ==========")
    print(f"Output saved to: {out_dir}")


def depth_npy_to_png(npy_path, png_path):
    """
    Convert a depth map npy file into a visualizable png image
    """
    import numpy as np
    from PIL import Image

    try:
        # Load depth data
        depth_data = np.load(npy_path)

        # Handle possible multi-channel data
        if len(depth_data.shape) > 2:
            depth_data = depth_data[:, :, 0] if depth_data.shape[2] >= 1 else depth_data.squeeze()

        # Handle invalid values (e.g., inf, nan)
        valid_mask = np.isfinite(depth_data)
        if not valid_mask.any():
            print(f"[depth_npy_to_png] Warning: No valid depth values in {npy_path}")
            return False

        # Get the range of valid values
        min_val = depth_data[valid_mask].min()
        max_val = depth_data[valid_mask].max()

        # Normalize to the 0-255 range
        if max_val > min_val:
            normalized = (depth_data - min_val) / (max_val - min_val)
        else:
            normalized = np.zeros_like(depth_data)

        # Set invalid values to 0
        normalized[~valid_mask] = 0

        # Invert: near is bright, far is dark
        normalized = 1.0 - normalized

        # Convert to an 8-bit image
        depth_uint8 = (normalized * 255).astype(np.uint8)

        # Save as png
        img = Image.fromarray(depth_uint8)
        img.save(png_path)
        return True
        
    except Exception as e:
        print(f"[depth_npy_to_png] Error converting {npy_path}: {e}")
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Diningroom scene headless rendering")
    parser.add_argument("--pair-count", type=int, default=10, help="Number of image pairs to generate")
    parser.add_argument("--warmup-k", type=int, default=3, help="Warmup multiplier (warmup_frames = 3 * k)")
    parser.add_argument("--num-changes", type=int, default=1, help="Number of prims sampled from all category pools")
    parser.add_argument(
        "--origin",
        action="store_true",
        help="Keep B frame as original layout; only hide sampled prims in A frame",
    )
    parser.add_argument("--semantic-segmentation", action="store_true", help="Enable semantic segmentation output (default: disabled)")
    parser.add_argument("--width", type=int, default=1024, help="Image width")
    parser.add_argument("--height", type=int, default=768, help="Image height")
    parser.add_argument("--focal-length", type=float, default=None, help="Camera focal length (optional)")
    parser.add_argument("--output-dir", type=str, default="/workspace/output/diningroom_add/2_items", help="Output directory")
    args = parser.parse_args()

    run_diningroom_example(
        pair_count=args.pair_count,
        warmup_k=args.warmup_k,
        num_changes=args.num_changes,
        origin=args.origin,
        semantic_segmentation=args.semantic_segmentation,
        resolution=(args.width, args.height),
        focal_length=args.focal_length,
        output_dir=args.output_dir,
    )
    simulation_app.close()
