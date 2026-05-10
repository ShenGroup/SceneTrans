# Headless EN - EN kitchen_test_editor.py EN
# EN/EN GUI EN
# use ./python.sh kitchen_test_headless.py EN
# EN:randomize3EN prims, EN
# pair EN

from isaacsim import SimulationApp

# 1. EN SimulationApp（headless）
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
    output_dir: str = "/workspace/output/kitchen_headless/3_items/500-549",
):
    """
    Headless EN
    EN3EN prims, EN
    EN（ENXEN0EN90EN）
    """
    
    # 2. EN
    usd_path = "/workspace/assets/Lightwheel_oz5iukPxYq_KitchenRoom/KitchenRoom_RSS.usd"
    print(f"[kitchen_headless] Opening stage: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)

    stage = omni.usd.get_context().get_stage()

    if not stage:
        print("[kitchen_headless] Error: Failed to open stage!")
        return

    # EN, EN
    print("[kitchen_headless] Scene loaded, preserving original lighting")

    # 3.2 EN, EN
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene":
            stage.RemovePrim(prim.GetPath())
            print(f"[kitchen_headless] Removed PhysicsScene: {prim.GetPath()}")

    import carb.settings
    carb.settings.get_settings().set("/physics/enabled", False)
    print("[kitchen_headless] Physics disabled via carb settings")

    # 3. EN
    # RTSubframes: EN
    rep.settings.carb_settings("/omni/replicator/RTSubframes", 16)
    print("[kitchen_headless] RTSubframes set to 16 (to eliminate ghosting)")



    # 4. EN on_frame EN, EN orchestrator EN
    rep.orchestrator.set_capture_on_play(True)

    # 4. EN Camera, use OmniverseKit_Persp
    desired_camera = "/OmniverseKit_Persp"
    camera_list = []
    
    cam_prim = stage.GetPrimAtPath(desired_camera)
    if cam_prim and cam_prim.IsValid():
        camera_list = [desired_camera]
        print(f"[kitchen_headless] Using camera: {desired_camera}")
    else:
        # EN
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

    # EN:EN
    if focal_length is not None:
        for cam_path in camera_list:
            cam_prim = stage.GetPrimAtPath(cam_path)
            cam_schema = UsdGeom.Camera(cam_prim)
            if cam_schema and cam_schema.GetFocalLengthAttr():
                cam_schema.GetFocalLengthAttr().Set(float(focal_length))
                print(f"[kitchen_headless] Set focalLength={focal_length} for {cam_path}")

    # 5. BasicWriter:EN /workspace/output/kitchen_headless
    writer = rep.writers.get("BasicWriter")
    out_dir = Path(output_dir)

    # 6. EN3EN prims EN
    def find_bottle_prims():
        """EN3EN prims"""
        specific_prims = [
            "/root/Kitchen_Paper",#single
            # "/root/Kitchen_Bottle",
            # "/root/Kitchen_Bottle_01",
            # "/root/Kitchen_bottle002",
            # "/root/Kitchen_bottle003",
            # "/root/Kitchen_bottle004",
            # "/root/Kitchen_KnifeHolders001",
            # "/root/SM_P_Choppingboard_01",
            # "/root/Kitchen_Box07",
            "/root/Kitchen_Basket", #multi
            # "/root/Kitchen_Basket/Kitchen_Basket001",
            # "/root/Kitchen_Basket/Kitchen_Basket002",
            # "/root/Kitchen_Basket/Kitchen_Basket003",
            # "/root/Kitchen_Basket/Kitchen_Basket004",
            # "/root/Kitchen_Basket/Kitchen_Basket005",
            # "/root/Kitchen_Basket/Kitchen_Basket006",
            # "/root/Kitchen_Basket/Kitchen_Basket007",
            # "/root/Kitchen_Box/Kitchen_Box001",
            # "/root/Kitchen_Box/Kitchen_Box002",
            # "/root/Kitchen_Box/Kitchen_Box003",
            # "/root/Kitchen_Box/Kitchen_Box004",
            "/root/Toaster003",
            # "/root/Kitchen_Hookrack001/Kitchen_Hookrack001",
            # "/root/Kitchen_Hookrack001/Kitchen_Hookrack002",
            # "/root/Kitchen_Hookrack001/Kitchen_Hookrack003",
            # "/root/Kitchen_Hookrack001/Kitchen_Hookrack004",
            # "/root/Kitchen_Hookrack001/Kitchen_Hookrack005",
            # "/root/Kitchen_Hookrack001/Kitchen_Hookrack006",
            # "/root/Kitchen_Hookrack001/Kitchen_Hookrack007",
        ]
        
        # EN prims EN
        valid_prims = []
        for path in specific_prims:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                valid_prims.append(path)
                print(f"[kitchen_headless] Found specified prim: {path}")
            else:
                print(f"[kitchen_headless] Warning: Specified prim not found: {path}")
        
        return valid_prims

    # 7. EN（randomize EN CollisionAPI）
    def remove_collision_from_bottles(bottle_paths):
        """EN CollisionAPI, randomize EN"""
        for path in bottle_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                print(f"[kitchen_headless] ❌ Prim not found: {path}")
                continue
            
            removed_count = 0

            # EN instanceable
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                print(f"[kitchen_headless] Made prim non-instanceable: {path}")
            
            # EN Xform EN CollisionAPI
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
                print(f"[kitchen_headless] ✓ Removed CollisionAPI from Xform: {path}")
                removed_count += 1
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                print(f"[kitchen_headless] ✓ Removed RigidBodyAPI from Xform: {path}")
                removed_count += 1
            
            # EN CollisionAPI
            def remove_collision_from_children(parent_prim, depth=0):
                nonlocal removed_count
                if depth > 10:
                    return
                for child in parent_prim.GetChildren():
                    child_path = child.GetPath().pathString
                    child_type = child.GetTypeName()
                    
                    # EN CollisionAPI
                    if child.HasAPI(UsdPhysics.CollisionAPI):
                        child.RemoveAPI(UsdPhysics.CollisionAPI)
                        print(f"[kitchen_headless] ✓ Removed CollisionAPI from {child_type}: {child_path}")
                        removed_count += 1
                    if child.HasAPI(UsdPhysics.RigidBodyAPI):
                        child.RemoveAPI(UsdPhysics.RigidBodyAPI)
                        print(f"[kitchen_headless] ✓ Removed RigidBodyAPI from {child_type}: {child_path}")
                        removed_count += 1
                    
                    # EN
                    remove_collision_from_children(child, depth + 1)
            
            remove_collision_from_children(prim)
            
            if removed_count > 0:
                print(f"[kitchen_headless] Physics cleanup done: {path} (removed {removed_count} CollisionAPI)")
            else:
                print(f"[kitchen_headless] No CollisionAPI found on: {path}")

    # 8. EN randomize EN（EN instance/semantic segmentation）
    def add_semantic_labels(prim_paths):
        """
        EN prims EN, EN
        EN:
        1. EN USD primvars EN（EN）
        2. use rep.modify.semantics（EN Replicator graph）
        """
        print("[kitchen_headless] Adding semantic labels using dual approach...")
        
        added_count = 0

        def clear_old_semantics(prim):
            for attr in list(prim.GetAttributes()):
                if attr.GetName().startswith("primvars:semantics:"):
                    prim.RemoveProperty(attr.GetName())

        def set_semantic_primvar(prim, prim_name: str):
            # use UsdGeom primvar API, type token array, constant EN
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
            
            # EN prim EN（EN）
            segments = [seg for seg in path.split("/") if seg]
            prim_name = segments[-1] if segments else "object"
            
            try:
                # EN instanceable, EN primvars
                if prim.IsInstanceable():
                    prim.SetInstanceable(False)

                # EN Xform EN primvars
                set_semantic_primvar(prim, prim_name)

                # EN Mesh EN primvars, EN
                mesh_children = collect_mesh_children(prim)
                for mesh_prim in mesh_children:
                    if mesh_prim.IsInstanceable():
                        mesh_prim.SetInstanceable(False)
                    set_semantic_primvar(mesh_prim, prim_name)

                # use rep.modify.semantics EN
                prim_group = rep.get.prims(path_pattern=path)
                with prim_group:
                    rep.modify.semantics([("class", prim_name)])
                
                added_count += 1
                print(f"[kitchen_headless] ✓ Semantic label added: {path} -> '{prim_name}' (meshes: {len(mesh_children)})")

                # EN（root + EN mesh）
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

    # 9. EN prims
    candidate_paths = find_bottle_prims()
    
    if not candidate_paths:
        print("[kitchen_headless] No specified prims were found; cannot proceed.")
        return
    
    print(f"[kitchen_headless] Will randomize {len(candidate_paths)} prims")
    
    # EN prim EN（EN）
    remove_collision_from_bottles(candidate_paths)
    
    # EN
    added = add_semantic_labels(candidate_paths)
    print(f"[kitchen_headless] Semantic labels added: {added} prims labeled")

    # 10. EN
    cabinet_prims = [
        "/root/Plane_01",
        "/root/Plane_02",
    ]
    
    # EN prims EN
    valid_cabinet_prims = []
    for cabinet_prim in cabinet_prims:
        cabinet_prim_obj = stage.GetPrimAtPath(cabinet_prim)
        if cabinet_prim_obj and cabinet_prim_obj.IsValid():
            valid_cabinet_prims.append(cabinet_prim)
            print(f"[kitchen_headless] ✓ Found cabinet prim: {cabinet_prim}")
        else:
            print(f"[kitchen_headless] ❌ Warning: Cabinet prim not found: {cabinet_prim}")
    
    if not valid_cabinet_prims:
        print(f"[kitchen_headless] ❌ Error: No valid cabinet prims found!")
        return
    
    # 10.1 EN Z EN（EN Z compensation）
    def get_surface_z_height(surface_prim_paths):
        """
        EN Z coordinates, EN
        """
        print(f"\n[Surface-Z] ========== EN Z EN ==========")
        max_z = float('-inf')
        for path in surface_prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[Surface-Z] ❌ Prim invalid: {path}")
                continue
            
            print(f"[Surface-Z] check: {path}, type: {prim.GetTypeName()}")
            
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
                print(f"[Surface-Z]   ⚠️ EN bbox EN bbox EN")
                
                # EN transform EN
                xformable = UsdGeom.Xformable(prim)
                if xformable:
                    world_transform = xformable.ComputeLocalToWorldTransform(0)
                    translation = world_transform.ExtractTranslation()
                    print(f"[Surface-Z]   Transform position: ({translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f})")
                    # use transform EN Z EN
                    if translation[2] > max_z:
                        max_z = translation[2]
                        print(f"[Surface-Z]   use transform Z EN: {translation[2]:.4f}")
        
        result = max_z if max_z != float('-inf') else 0.0
        print(f"[Surface-Z] EN Z EN: {result}")
        print(f"[Surface-Z] ==========================================\n")
        return result
    
    # EN（EN bounds_list EN surface_z EN）
    # EN:get_surface_z_height EN 0, EN bounds_list EN
    surface_z_height = get_surface_z_height(valid_cabinet_prims)
    
    # 10.2 Z EN:EN + EN, EN
    def apply_z_compensation(prim_surface_z_map):
        """
        EN bounding box, compensation Z EN
        
        EN:
        1. EN translate op EN Z value
        2. EN bbox EN Z EN
        3. EN Z, EN bbox EN = EN surface_z
        
        Args:
            prim_surface_z_map: {prim_path: surface_z} EN Z EN
        """
        if not prim_surface_z_map:
            print(f"\n[Z-comp] ⚠️ EN")
            return
        
        print(f"\n[Z-comp] ========== start Z compensation ==========")
        
        bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
        
        for path, target_surface_z in prim_surface_z_map.items():
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[Z-comp] ❌ Prim invalid: {path}")
                continue
            
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                print(f"[Z-comp] ❌ EN Xformable: {path}")
                continue
            
            # EN translate op
            ops = xformable.GetOrderedXformOps()
            translate_op = None
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break
            
            if not translate_op:
                print(f"[Z-comp] ⚠️ {path}: EN translate op, EN")
                continue
            
            current_pos = translate_op.Get()
            if not current_pos:
                print(f"[Z-comp] ⚠️ {path}: translate op EN")
                continue
            
            current_z = current_pos[2]
            
            # EN world bounding box
            bbox_cache.Clear()
            world_bbox = bbox_cache.ComputeWorldBound(prim)
            if not world_bbox or world_bbox.GetRange().IsEmpty():
                print(f"[Z-comp] ⚠️ EN bbox: {path}")
                continue
            
            bbox_min_z = world_bbox.GetRange().GetMin()[2]
            
            # EN Z EN:EN bbox EN surface_z
            z_adjustment = target_surface_z - bbox_min_z
            new_z = current_z + z_adjustment
            
            print(f"[Z-comp] {path}: target_z={target_surface_z:.4f}, current_z={current_z:.4f}, bbox_min_z={bbox_min_z:.4f}")
            print(f"[Z-comp]   adjustment={z_adjustment:.4f}, new_z={new_z:.4f}")
            
            # EN
            new_pos = Gf.Vec3d(current_pos[0], current_pos[1], new_z)
            translate_op.Set(new_pos)
            
            # verify
            bbox_cache.Clear()
            new_bbox = bbox_cache.ComputeWorldBound(prim)
            if new_bbox:
                new_min_z = new_bbox.GetRange().GetMin()[2]
                print(f"[Z-comp] ✓ verify: EN bbox_min_z = {new_min_z:.4f} (EN {target_surface_z:.4f})")
        
        print(f"[Z-comp] ========== Z EN ==========\n")

    # 11. EN Mesh prim（scatter_2d EN Mesh, EN）
    def find_actual_mesh(prim_path, depth=0, max_depth=3):
        """
        EN prim EN Mesh EN
        EN scatter_2d EN Mesh prim, EN Mesh EN
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
    
    # EN mesh prims
    actual_cabinet_meshes = []
    for cabinet_prim in valid_cabinet_prims:
        actual_mesh = find_actual_mesh(cabinet_prim)
        if actual_mesh:
            actual_cabinet_meshes.append(actual_mesh)
            print(f"[kitchen_headless] Using mesh for scatter_2d: {actual_mesh}")
        else:
            # EN Mesh, EN prim
            print(f"[kitchen_headless] Will try using prim directly: {cabinet_prim}")
            actual_cabinet_meshes.append(cabinet_prim)
    
    if not actual_cabinet_meshes:
        print("[kitchen_headless] ❌ Error: No valid meshes found for scatter_2d")
        return
    
    # EN prims EN scatter_2d EN
    surface_nodes = []
    for mesh_path in actual_cabinet_meshes:
        prim_node = rep.get.prims(path_pattern=mesh_path)
        if prim_node:
            surface_nodes.append(prim_node)
            print(f"[kitchen_headless] Added surface: {mesh_path}")
    
    if not surface_nodes:
        print("[kitchen_headless] ❌ Error: No valid surface found for scatter_2d")
        return
    
    # EN, EN；EN
    if len(surface_nodes) == 1:
        surface = surface_nodes[0]
    else:
        surface = rep.create.group(surface_nodes)
        print(f"[kitchen_headless] Created surface group with {len(surface_nodes)} meshes")
    
    # 11.5 EN（EN USD API randomize）
    def get_each_surface_bounds(surface_prim_paths):
        """
        EN（EN, EN）
        EN + Mesh EN extent EN
        EN: [(min_x, max_x, min_y, max_y, surface_z), ...]
        """
        surface_bounds_list = []
        
        print(f"\n[Surface-Bounds] ========== EN ==========")
        
        def find_mesh_in_prim(parent_prim, depth=0, max_depth=5):
            """EN prim EN Mesh EN"""
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
            
            print(f"[Surface-Bounds] check: {path}, type: {prim.GetTypeName()}")
            
            # EN prim EN（EN）
            xformable = UsdGeom.Xformable(prim)
            if xformable:
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                translation = world_transform.ExtractTranslation()
                print(f"[Surface-Bounds]   EN: ({translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f})")
            
            # EN Mesh prim
            mesh_prim = find_mesh_in_prim(prim)
            target_prim = mesh_prim if mesh_prim else prim
            target_path = target_prim.GetPath().pathString
            print(f"[Surface-Bounds]   use prim: {target_path}, type: {target_prim.GetTypeName()}")
            
            # EN1:EN Mesh EN extent EN
            extent_attr = target_prim.GetAttribute("extent")
            local_extent = None
            if extent_attr and extent_attr.HasValue():
                local_extent = extent_attr.Get()
                if local_extent and len(local_extent) >= 2:
                    print(f"[Surface-Bounds]   EN extent: min={local_extent[0]}, max={local_extent[1]}")
            
            # EN
            target_xformable = UsdGeom.Xformable(target_prim)
            if not target_xformable:
                print(f"[Surface-Bounds] ❌ EN Xformable: {target_path}")
                continue
            
            world_transform = target_xformable.ComputeLocalToWorldTransform(0)
            
            # EN extent, EN
            if local_extent and len(local_extent) >= 2:
                local_min = local_extent[0]
                local_max = local_extent[1]
                
                # EN8EN（EN3DEN）EN4EN（EN）
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
                # fallback:EN
                print(f"[Surface-Bounds]   ⚠️ EN extent, EN")
                local_corners = [
                    Gf.Vec3d(-0.5, -0.5, 0),
                    Gf.Vec3d(0.5, -0.5, 0),
                    Gf.Vec3d(-0.5, 0.5, 0),
                    Gf.Vec3d(0.5, 0.5, 0),
                ]
            
            # EN
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
            
            print(f"[Surface-Bounds]   EN: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}], Z={surface_z:.2f}")
            
            # EN 5% EN
            range_x = max_x - min_x
            range_y = max_y - min_y
            margin_x = range_x * 0.05
            margin_y = range_y * 0.1
            min_x += margin_x
            max_x -= margin_x
            min_y += margin_y
            max_y -= margin_y
            
            # EN
            bounds = (min_x, max_x, min_y, max_y, surface_z)
            surface_bounds_list.append(bounds)
            
            print(f"[Surface-Bounds]   EN: X=[{bounds[0]:.2f}, {bounds[1]:.2f}], Y=[{bounds[2]:.2f}, {bounds[3]:.2f}], Z={surface_z:.2f}")
        
        print(f"[Surface-Bounds] EN {len(surface_bounds_list)} EN")
        print(f"[Surface-Bounds] ==========================================\n")
        
        return surface_bounds_list
    
    # EN（EN）
    surface_bounds_list = get_each_surface_bounds(valid_cabinet_prims)
    
    # EN bounds_list EN surface_z_height（EN）
    # EN get_surface_z_height EN
    if surface_bounds_list:
        surface_z_height = max(bounds[4] for bounds in surface_bounds_list)
        print(f"[kitchen_headless] ✓ surface_z_height EN bounds_list EN: {surface_z_height:.4f}")
    
    # 11.6 EN USD API EN（EN, EN Replicator trigger）
    import random as py_random
    
    def manual_randomize_objects(prim_paths, bounds_list):
        """
        EN USD API EN
        EN, EN
        
        Args:
            prim_paths: EN prim EN
            bounds_list: EN [(min_x, max_x, min_y, max_y, surface_z), ...]
        
        Returns:
            dict: {prim_path: surface_z} EN Z EN
        """
        prim_surface_z_map = {}
        
        if not bounds_list:
            print(f"[Manual-Rand] ❌ EN")
            return prim_surface_z_map
        
        print(f"\n[Manual-Rand] ========== start USD API randomize ==========")
        print(f"[Manual-Rand] EN: {len(bounds_list)}")
        
        for path in prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[Manual-Rand] ❌ Prim invalid: {path}")
                continue
            
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                print(f"[Manual-Rand] ❌ EN Xformable: {path}")
                continue
            
            # EN
            selected_bounds = py_random.choice(bounds_list)
            min_x, max_x, min_y, max_y, surface_z = selected_bounds
            surface_idx = bounds_list.index(selected_bounds)
            
            # EN Z EN
            prim_surface_z_map[path] = surface_z
            
            # EN（EN）
            rand_x = py_random.uniform(min_x, max_x)
            rand_y = py_random.uniform(min_y, max_y)
            rand_z = surface_z  # EN, EN Z EN
            
            # EN:X EN 0 EN 90 EN（EN）, Z EN 0-360 EN
            rand_rot_x = py_random.choice([0, 0])
            rand_rot_y = 0
            rand_rot_z = py_random.uniform(0, 360)
            
            # EN xform ops
            ops = xformable.GetOrderedXformOps()
            translate_op = None
            rotate_op = None
            
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                    rotate_op = op
            
            # EN
            if translate_op:
                translate_op.Set(Gf.Vec3d(rand_x, rand_y, rand_z))
            else:
                translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
                translate_op.Set(Gf.Vec3d(rand_x, rand_y, rand_z))
            
            # EN
            if rotate_op:
                rotate_op.Set(Gf.Vec3f(rand_rot_x, rand_rot_y, rand_rot_z))
            else:
                rotate_op = xformable.AddRotateXYZOp(UsdGeom.XformOp.PrecisionFloat)
                rotate_op.Set(Gf.Vec3f(rand_rot_x, rand_rot_y, rand_rot_z))
            
            state = "EN" if rand_rot_x == 0 else "EN"
            
            # EN
            in_range_x = min_x <= rand_x <= max_x
            in_range_y = min_y <= rand_y <= max_y
            if not in_range_x or not in_range_y:
                print(f"[Manual-Rand] ❌ {path}: EN!")
                print(f"[Manual-Rand]   bounds: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}]")
                print(f"[Manual-Rand]   pos: X={rand_x:.2f} {'✓' if in_range_x else '❌'}, Y={rand_y:.2f} {'✓' if in_range_y else '❌'}")
            
            print(f"[Manual-Rand] ✓ {path}: plane{surface_idx}(Z={surface_z:.2f}), bounds=X[{min_x:.2f},{max_x:.2f}],Y[{min_y:.2f},{max_y:.2f}], pos=({rand_x:.2f}, {rand_y:.2f}, {rand_z:.2f}), rot=({rand_rot_x}, {rand_rot_y}, {rand_rot_z:.0f}) [{state}]")
        
        print(f"[Manual-Rand] ========== EN ==========\n")
        return prim_surface_z_map

    # 12. EN
    for cam_path in camera_list:
        print(f"\n[kitchen_headless] ========== Rendering camera: {cam_path} ==========")

        render_product = rep.create.render_product(cam_path, resolution)

        cam_out_dir = out_dir
        raw_dir = cam_out_dir / "_raw_frames"
        os.makedirs(raw_dir, exist_ok=True)
        
        # EN（EN）
        # warmup EN = 3 * warmup_k, EN 3 EN
        warmup_frames = 3 * warmup_k
        print(f"[kitchen_headless] Running {warmup_frames} warmup frames (k={warmup_k}) with randomization...")
        rep.orchestrator.set_capture_on_play(False)
        
        # EN warmup EN, EN USD API EN
        # EN
        # EN 3 EN, EN warmup_k EN
        import random as py_random
        for warmup_i in range(warmup_frames):
            # EN 3 EN, EN
            if warmup_i % 3 == 0:
                for path in candidate_paths:
                    prim = stage.GetPrimAtPath(path)
                    if prim:
                        xformable = UsdGeom.Xformable(prim)
                        if xformable:
                            # EN translate op
                            for op in xformable.GetOrderedXformOps():
                                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                    # EN
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

        # EN:start from annotator EN, EN
        def log_annotator_sample(render_prod):
            import numpy as np

            semantic_anno = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
            instance_anno = rep.AnnotatorRegistry.get_annotator("instance_segmentation")
            semantic_anno.attach(render_prod)
            instance_anno.attach(render_prod)

            # capture EN step EN
            rep.orchestrator.step()

            sem = semantic_anno.get_data()
            ins = instance_anno.get_data()
            def summarize(arr, name):
                # EN（dict EN ndarray）
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

        # log_annotator_sample(render_product) # EN, EN step()
        
        # EN
        for old_file in raw_dir.glob("*.*"):
            old_file.unlink(missing_ok=True)
        
        # EN attach writer（warmup EN）
        writer.initialize(
            output_dir=str(raw_dir),
            rgb=True,
            distance_to_camera=True,           # depth map (depth map)
            semantic_segmentation=True,        # EN
            colorize_semantic_segmentation=True,  # EN
            instance_segmentation=True,        # EN
            colorize_instance_segmentation=True,  # EN
            bounding_box_2d_tight=False
        )
        writer.attach(render_product)
        
        # EN
        rep.orchestrator.set_capture_on_play(True)
        print(f"[kitchen_headless] Writer attached, starting capture...")

        # EN pair EN
        total_frames = pair_count * 2
        pair_records = []
        
        for pair_idx in range(pair_count):
            pair_records.append({
                "scene_type": "kitchen",
                "pair_id": f"pair_{pair_idx:04d}",
                "camera": cam_path,
                "coordinates": {},  # EN
            })

        print(f"[kitchen_headless] Generating {pair_count} pairs ({total_frames} frames)")

        # EN
        def get_world_coordinates(prim_paths):
            """EN prims EN"""
            coords = {}
            for path in prim_paths:
                prim = stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    xformable = UsdGeom.Xformable(prim)
                    if xformable:
                        # EN
                        world_transform = xformable.ComputeLocalToWorldTransform(0)
                        # EN（EN）
                        translation = world_transform.ExtractTranslation()
                        coords[path] = [translation[0], translation[1], translation[2]]
            return coords

        def get_2d_center_coordinates_from_projection(prim_paths, camera_path, image_width, image_height):
            """
            EN3DEN2DEN（EN）
            
            Args:
                prim_paths: prim EN
                camera_path: EN
                image_width: EN
                image_height: EN
            
            Returns:
                dict: {prim_path: [pixel_x, pixel_y]} EN {prim_path: None} EN
            """
            coords_2d = {}
            
            # EN prim
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
            
            # EN（EN）
            camera_xformable = UsdGeom.Xformable(camera_prim)
            camera_world_transform = camera_xformable.ComputeLocalToWorldTransform(0)
            # EN（EN）
            world_to_camera = camera_world_transform.GetInverse()
            
            # EN
            focal_length = camera.GetFocalLengthAttr().Get()  # mm
            h_aperture = camera.GetHorizontalApertureAttr().Get()  # mm
            v_aperture = camera.GetVerticalApertureAttr().Get()  # mm
            
            if focal_length is None or h_aperture is None:
                print(f"[kitchen_headless] Warning: Camera parameters not available")
                for path in prim_paths:
                    coords_2d[path] = None
                return coords_2d
            
            # if v_aperture EN None, EN
            if v_aperture is None or v_aperture == 0:
                v_aperture = h_aperture * (image_height / image_width)
            
            for path in prim_paths:
                prim = stage.GetPrimAtPath(path)
                if not prim or not prim.IsValid():
                    coords_2d[path] = None
                    continue
                
                xformable = UsdGeom.Xformable(prim)
                if not xformable:
                    coords_2d[path] = None
                    continue
                
                # EN
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                world_pos = world_transform.ExtractTranslation()
                
                # EN
                # use TransformAffine EN3DEN
                world_pos_3d = Gf.Vec3d(world_pos[0], world_pos[1], world_pos[2])
                camera_pos = world_to_camera.TransformAffine(world_pos_3d)
                
                # EN
                cam_x = camera_pos[0]
                cam_y = camera_pos[1]
                cam_z = camera_pos[2]
                
                # EN -Z, EN z EN
                if cam_z >= 0:
                    # EN
                    print(f"[kitchen_headless]   {path} is behind camera (z={cam_z:.2f})")
                    coords_2d[path] = None
                    continue
                
                # EN（EN）
                # NDC coordinates (EN)
                ndc_x = (focal_length * cam_x) / (-cam_z * h_aperture / 2)
                ndc_y = (focal_length * cam_y) / (-cam_z * v_aperture / 2)
                
                # NDC EN [-1, 1], EN
                # EN:y EN
                pixel_x = int((ndc_x + 1) * 0.5 * image_width)
                pixel_y = int((1 - ndc_y) * 0.5 * image_height)  # EN Y
                
                # EN
                if 0 <= pixel_x < image_width and 0 <= pixel_y < image_height:
                    coords_2d[path] = [pixel_x, pixel_y]
                    print(f"[kitchen_headless]   Projected {path}: world ({world_pos[0]:.1f}, {world_pos[1]:.1f}, {world_pos[2]:.1f}) -> pixel ({pixel_x}, {pixel_y})")
                else:
                    print(f"[kitchen_headless]   {path} projected outside image: ({pixel_x}, {pixel_y})")
                    coords_2d[path] = [pixel_x, pixel_y]  # EN, EN
            
            return coords_2d

        # 13-15. EN USD API randomize, EN Replicator trigger
        # EN, EN Z EN
        
        frame_counter = [0]
        frame_coordinates = []
        frame_2d_coordinates = []
        
        # EN:EN, EN
        # EN 3 EN, EN 5 EN = 15 frame
        warmup_randomizations = 5
        warmup_steps_per_rand = 3
        print(f"[kitchen_headless] Running warmup: {warmup_randomizations} randomizations × {warmup_steps_per_rand} steps = {warmup_randomizations * warmup_steps_per_rand} frames...")
        
        # Detach writer EN warmup EN, EN
        writer.detach()
        
        for warmup_i in range(warmup_randomizations):
            prim_z_map = manual_randomize_objects(candidate_paths, surface_bounds_list)
            apply_z_compensation(prim_z_map)
            # EN
            for _ in range(warmup_steps_per_rand):
                rep.orchestrator.step()
        
        # EN attach writer EN
        writer.attach(render_product)
        print(f"[kitchen_headless] Warmup complete, writer re-attached.")
        
        # 16. EN
        # EN, EN A/B EN
        flush_frames_before_capture = 6  # EN（EN 6 frame）
        print(f"[kitchen_headless] Running main render loop with {total_frames} frames...")
        print(f"[kitchen_headless] Using {flush_frames_before_capture} flush frames before each capture to eliminate ghosting")
        
        for frame_idx in range(total_frames):
            frame_counter[0] += 1
            print(f"\n[kitchen_headless] ===== Frame {frame_idx} =====")
            
            # step 1:use USD API EN
            prim_z_map = manual_randomize_objects(candidate_paths, surface_bounds_list)
            
            # step 2:EN Z compensation（EN）
            apply_z_compensation(prim_z_map)
            
            # step 3:EN - detach writer, EN
            writer.detach()
            for _ in range(flush_frames_before_capture):
                rep.orchestrator.step()
            
            # step 4:EN - attach writer, EN
            writer.attach(render_product)
            rep.orchestrator.step()
            print(f"[kitchen_headless] Frame {frame_idx}: rendered and captured (after {flush_frames_before_capture} flush frames)")
            
            # EN
            print(f"[Verify] ===== EN =====")
            for path in candidate_paths:
                prim = stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    xformable = UsdGeom.Xformable(prim)
                    if xformable:
                        # get translate op EN
                        for op in xformable.GetOrderedXformOps():
                            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                pos = op.Get()
                                print(f"[Verify] {path}: translate = ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
                                break
                        # EN bbox
                        bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
                        world_bbox = bbox_cache.ComputeWorldBound(prim)
                        if world_bbox and not world_bbox.GetRange().IsEmpty():
                            bbox_min = world_bbox.GetRange().GetMin()
                            print(f"[Verify]   bbox_min_z = {bbox_min[2]:.4f}")
            
            # EN
            current_coords = get_world_coordinates(candidate_paths)
            frame_coordinates.append(current_coords)
            
            # EN2DEN
            current_2d_coords = get_2d_center_coordinates_from_projection(
                candidate_paths, cam_path, resolution[0], resolution[1]
            )
            frame_2d_coordinates.append(current_2d_coords)
            print(f"[kitchen_headless] Frame {frame_idx}: recorded coords for {len(current_coords)} prims")
        
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
    parser.add_argument("--output-dir", type=str, default="/workspace/output/kitchen_headless/3_items/500-549", help="Output directory")
    args = parser.parse_args()

    run_kitchen_example(
        pair_count=args.pair_count,
        warmup_k=args.warmup_k,
        resolution=(args.width, args.height),
        focal_length=args.focal_length,
        output_dir=args.output_dir,
    )
    simulation_app.close()
