# Headless 版本 - 基于 kitchen_test_editor.py 改编
# 适合在容器/无 GUI 环境中运行
# 使用 ./python.sh kitchen_test_headless.py 运行
# 特性：随机化3个瓶子 prims，在两个柜台平面上放置
# pair 差异通过不同的随机化位置和瓶子倒伏状态体现

from isaacsim import SimulationApp

# 1. 创建 SimulationApp（headless）
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
    pair_count: int = 10,
    num_changes: int = 1,
    output_dir: str = "/workspace/output/livingroom_shelf_move/4_item",
):
    """
    Headless 版本的场景示例
    使用固定相机 /World/Camera
    初始帧 + 随机隐藏指定 prims 的序列帧，按相邻帧构造 pair
    """
    
    # 2. 打开厨房场景
    usd_path = "/workspace/assets/Interactive_scene/largelivingroom/Interactive_largelivingroom.usd"
    print(f"[kitchen_headless] Opening stage: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)

    stage = omni.usd.get_context().get_stage()

    if not stage:
        print("[kitchen_headless] Error: Failed to open stage!")
        return

    # 场景保持原样，不修改任何灯光设置
    print("[kitchen_headless] Scene loaded, preserving original lighting")

    # 3.2 删除物理场景并关闭全局物理，避免随机化后被物理推走
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene":
            stage.RemovePrim(prim.GetPath())
            print(f"[kitchen_headless] Removed PhysicsScene: {prim.GetPath()}")

    import carb.settings
    carb.settings.get_settings().set("/physics/enabled", False)
    print("[kitchen_headless] Physics disabled via carb settings")

    # 3. 配置渲染设置来消除时间累积造成的鬼影
    # RTSubframes: 强制渲染器在每帧重新采样
    rep.settings.carb_settings("/omni/replicator/RTSubframes", 32)
    print("[kitchen_headless] RTSubframes set to 16 (to eliminate ghosting)")



    # 4. 配合 on_frame 触发器，让 orchestrator 驱动写盘
    rep.orchestrator.set_capture_on_play(True)

    # 4. 只使用固定相机 /World/Camera
    desired_camera = "/World/Camera"
    cam_prim = stage.GetPrimAtPath(desired_camera)
    if not cam_prim or not cam_prim.IsValid():
        print(f"[kitchen_headless] Camera not found, creating: {desired_camera}")
        cam_prim = stage.DefinePrim(desired_camera, "Camera")
        cam_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Float3).Set((0.0, 150.0, 600.0))
        cam_prim.CreateAttribute("focalLength", Sdf.ValueTypeNames.Float).Set(35.0)
    camera_list = [desired_camera]
    print(f"[kitchen_headless] Using camera: {desired_camera}")

    # 可选：调整相机焦距
    if focal_length is not None:
        for cam_path in camera_list:
            cam_prim = stage.GetPrimAtPath(cam_path)
            cam_schema = UsdGeom.Camera(cam_prim)
            if cam_schema and cam_schema.GetFocalLengthAttr():
                cam_schema.GetFocalLengthAttr().Set(float(focal_length))
                print(f"[kitchen_headless] Set focalLength={focal_length} for {cam_path}")

    # 5. BasicWriter：输出到 /workspace/output/kitchen_headless
    writer = rep.writers.get("BasicWriter")
    out_dir = Path(output_dir)

    # 6. 只使用指定的3个瓶子 prims 进行随机化
    def find_bottle_prims():
        specific_prims = [
            "/World/model_book_5",
            # "/World/model_book_4",#不适合
            "/World/model_book_2",
            "/World/model_book8",
            "/World/model_book_7",
            # "/World/model_book6",#不行
            "/World/model_book2_02",
            "/World/model_book2_03",
            "/World/model_book2_04",
            "/World/model_book2_05",
            "/World/model_book6_05",
            "/World/model_book8_03",
            # "/World/model_book_12",#不行
            "/World/model_book_15",
            "/World/model_book_14",
            "/World/model_book_16",
        ]
        
        # 验证这些 prims 是否存在
        valid_prims = []
        for path in specific_prims:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                valid_prims.append(path)
                print(f"[kitchen_headless] Found specified prim: {path}")
            else:
                print(f"[kitchen_headless] Warning: Specified prim not found: {path}")
        
        return valid_prims

    # 7. 检查并移除物体的碰撞体（randomize 的物体不需要 CollisionAPI）
    def remove_collision_from_bottles(bottle_paths):
        """检查并移除物体的 CollisionAPI，randomize 的物体不需要碰撞体"""
        for path in bottle_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                print(f"[kitchen_headless] ❌ Prim not found: {path}")
                continue
            
            removed_count = 0

            # 取消 instanceable
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                print(f"[kitchen_headless] Made prim non-instanceable: {path}")
            
            # 检查并移除 Xform 本身的 CollisionAPI
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
                print(f"[kitchen_headless] ✓ Removed CollisionAPI from Xform: {path}")
                removed_count += 1
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                print(f"[kitchen_headless] ✓ Removed RigidBodyAPI from Xform: {path}")
                removed_count += 1
            
            # 递归检查所有子节点的 CollisionAPI
            def remove_collision_from_children(parent_prim, depth=0):
                nonlocal removed_count
                if depth > 10:
                    return
                for child in parent_prim.GetChildren():
                    child_path = child.GetPath().pathString
                    child_type = child.GetTypeName()
                    
                    # 检查并移除 CollisionAPI
                    if child.HasAPI(UsdPhysics.CollisionAPI):
                        child.RemoveAPI(UsdPhysics.CollisionAPI)
                        print(f"[kitchen_headless] ✓ Removed CollisionAPI from {child_type}: {child_path}")
                        removed_count += 1
                    if child.HasAPI(UsdPhysics.RigidBodyAPI):
                        child.RemoveAPI(UsdPhysics.RigidBodyAPI)
                        print(f"[kitchen_headless] ✓ Removed RigidBodyAPI from {child_type}: {child_path}")
                        removed_count += 1
                    
                    # 继续递归检查子节点
                    remove_collision_from_children(child, depth + 1)
            
            remove_collision_from_children(prim)
            
            if removed_count > 0:
                print(f"[kitchen_headless] Physics cleanup done: {path} (removed {removed_count} CollisionAPI)")
            else:
                print(f"[kitchen_headless] No CollisionAPI found on: {path}")

    # 8. 为 randomize 的物体添加语义标签（用于 instance/semantic segmentation）
    def add_semantic_labels(prim_paths):
        """
        为指定的 prims 添加语义标签，标签名取路径的最后一段
        同时使用两种方式确保兼容性：
        1. 直接使用 USD primvars 写入语义属性（立即生效）
        2. 使用 rep.modify.semantics（用于 Replicator graph）
        """
        print("[kitchen_headless] Adding semantic labels using dual approach...")
        
        added_count = 0

        def clear_old_semantics(prim):
            for attr in list(prim.GetAttributes()):
                if attr.GetName().startswith("primvars:semantics:"):
                    prim.RemoveProperty(attr.GetName())

        def set_semantic_primvar(prim, prim_name: str):
            # 使用 UsdGeom primvar API，类型 token，constant 插值
            clear_old_semantics(prim)
            primvar = UsdGeom.PrimvarsAPI(prim).CreatePrimvar(
                "semantics:class", Sdf.ValueTypeNames.Token, UsdGeom.Tokens.constant
            )
            primvar.Set(prim_name)

        def collect_mesh_children(parent_prim, depth=0, max_depth=10):
            if depth > max_depth:
                return []
            meshes = []
            for child in parent_prim.GetChildren():
                child_type = child.GetTypeName()
                if child_type == "Mesh":
                    meshes.append(child)
                else:
                    meshes.extend(collect_mesh_children(child, depth + 1, max_depth))
            return meshes
        
        for path in prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[kitchen_headless] ❌ Prim not found: {path}")
                continue
            
            # 统一使用固定语义标签
            prim_name = "book"
            
            try:
                # 取消 instanceable，确保可以写 primvars
                if prim.IsInstanceable():
                    prim.SetInstanceable(False)

                # 在 Xform 上写 primvars
                set_semantic_primvar(prim, prim_name)

                # 在子 Mesh 上写 primvars，保证分割能读取到
                mesh_children = collect_mesh_children(prim)
                for mesh_prim in mesh_children:
                    if mesh_prim.IsInstanceable():
                        mesh_prim.SetInstanceable(False)
                    set_semantic_primvar(mesh_prim, prim_name)

                # 使用 rep.modify.semantics 添加语义标签
                prim_group = rep.get.prims(path_pattern=path)
                with prim_group:
                    rep.modify.semantics([("class", prim_name)])
                # 也在每个 Mesh 上设置 rep.modify.semantics
                for mesh_prim in mesh_children:
                    mesh_group = rep.get.prims(path_pattern=mesh_prim.GetPath().pathString)
                    with mesh_group:
                        rep.modify.semantics([("class", prim_name)])
                
                added_count += 1
                print(f"[kitchen_headless] ✓ Semantic label added: {path} -> '{prim_name}' (meshes: {len(mesh_children)})")

                # 打印验证（root + 第一个 mesh）
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

    # 9. 查找并验证 prims
    candidate_paths = find_bottle_prims()
    
    if not candidate_paths:
        print("[kitchen_headless] No specified prims were found; cannot proceed.")
        return
    
    print(f"[kitchen_headless] Will randomize {len(candidate_paths)} prims")

    # 9.0 先隐藏占位置的盆栽 prim，避免遮挡平面
    hidden_prim_path = "/World/model_potted_plant002"

    def set_prim_visibility(prim_path: str, visibility: str):
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            print(f"[kitchen_headless] Warning: Prim not found for visibility: {prim_path}")
            return

        def set_visibility_recursive(target_prim, visibility_value, depth=0, max_depth=6):
            if not target_prim or not target_prim.IsValid() or depth > max_depth:
                return
            if target_prim.GetName() != "Looks":
                imageable = UsdGeom.Imageable(target_prim)
                if imageable:
                    vis_attr = imageable.GetVisibilityAttr()
                    if vis_attr and vis_attr.IsValid():
                        vis_attr.Set(visibility_value)
            for child in target_prim.GetChildren():
                set_visibility_recursive(child, visibility_value, depth + 1, max_depth)

        set_visibility_recursive(prim, visibility)

    # 隐藏盆栽
    set_prim_visibility(hidden_prim_path, "invisible")
    print(f"[kitchen_headless] Hidden prim: {hidden_prim_path}")

    # 只从 other items 中随机选择移动目标
    movable_candidate_paths = [p for p in candidate_paths if p != hidden_prim_path]
    if not movable_candidate_paths:
        print("[kitchen_headless] No movable prims left after hiding the plant; cannot proceed.")
        return

    # 9.1 解析可移动的 prim（通过 bbox 变化判断）
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

            # 还原
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
    
    # 移除这些 prim 的物理碰撞体（如果有）
    remove_collision_from_bottles(candidate_paths)
    
    # 应用语义标签到所有待随机化的物体
    added = add_semantic_labels(candidate_paths)
    print(f"[kitchen_headless] Semantic labels added: {added} prims labeled")

    # 10. 使用指定的平面进行随机放置
    cabinet_prims = [
        "World/Plane",
    ]

    def normalize_prim_path(path: str) -> str:
        """确保路径以 / 开头，兼容输入不带前导斜杠的情况"""
        return path if path.startswith("/") else f"/{path}"
    
    # 验证柜台 prims 是否存在
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
    
    # 10.1 获取表面 Z 高度（用于 Z 补偿）
    def get_surface_z_height(surface_prim_paths):
        """
        获取表面的最高 Z 坐标，作为物体放置的参考高度
        """
        print(f"\n[Surface-Z] ========== 计算表面 Z 高度 ==========")
        max_z = float('-inf')
        for path in surface_prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[Surface-Z] ❌ Prim 无效: {path}")
                continue
            
            print(f"[Surface-Z] 检查: {path}, 类型: {prim.GetTypeName()}")
            
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
                print(f"[Surface-Z]   ⚠️ 无法计算 bbox 或 bbox 为空")
                
                # 尝试获取 transform 作为备选
                xformable = UsdGeom.Xformable(prim)
                if xformable:
                    world_transform = xformable.ComputeLocalToWorldTransform(0)
                    translation = world_transform.ExtractTranslation()
                    print(f"[Surface-Z]   Transform 位置: ({translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f})")
                    # 使用 transform 的 Z 作为备选
                    if translation[2] > max_z:
                        max_z = translation[2]
                        print(f"[Surface-Z]   使用 transform Z 作为表面高度: {translation[2]:.4f}")
        
        result = max_z if max_z != float('-inf') else 0.0
        print(f"[Surface-Z] 最终表面 Z 高度: {result}")
        print(f"[Surface-Z] ==========================================\n")
        return result
    
    # 获取表面高度（从 bounds_list 的 surface_z 取最大值）
    # 注意：get_surface_z_height 可能返回 0，所以我们在 bounds_list 计算后再更新
    surface_z_height = get_surface_z_height(valid_cabinet_prims)
    
    # 11. 查找实际的 Mesh prim（scatter_2d 需要直接的 Mesh，不能是容器）
    def find_actual_mesh(prim_path, depth=0, max_depth=3):
        """
        递归查找 prim 下实际的 Mesh 子节点
        因为 scatter_2d 需要直接的 Mesh prim，不能是包含 Mesh 的容器
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
    
    # 查找所有柜台的实际 mesh prims
    actual_cabinet_meshes = []
    for cabinet_prim in valid_cabinet_prims:
        actual_mesh = find_actual_mesh(cabinet_prim)
        if actual_mesh:
            actual_cabinet_meshes.append(actual_mesh)
            print(f"[kitchen_headless] Using mesh for scatter_2d: {actual_mesh}")
        else:
            # 如果找不到子 Mesh，尝试直接使用该 prim
            print(f"[kitchen_headless] Will try using prim directly: {cabinet_prim}")
            actual_cabinet_meshes.append(cabinet_prim)
    
    if not actual_cabinet_meshes:
        print("[kitchen_headless] ❌ Error: No valid meshes found for scatter_2d")
        return
    
    # 获取柜台 prims 作为 scatter_2d 的表面
    surface_nodes = []
    for mesh_path in actual_cabinet_meshes:
        prim_node = rep.get.prims(path_pattern=mesh_path)
        if prim_node:
            surface_nodes.append(prim_node)
            print(f"[kitchen_headless] Added surface: {mesh_path}")
    
    if not surface_nodes:
        print("[kitchen_headless] ❌ Error: No valid surface found for scatter_2d")
        return
    
    # 如果有多个表面，创建一个组；否则直接使用单个表面
    if len(surface_nodes) == 1:
        surface = surface_nodes[0]
    else:
        surface = rep.create.group(surface_nodes)
        print(f"[kitchen_headless] Created surface group with {len(surface_nodes)} meshes")
    
    # 11.5 计算每个表面的世界坐标范围（用于 USD API 随机化）
    def get_each_surface_bounds(surface_prim_paths):
        """
        获取每个表面的世界坐标范围（分开存储，不合并）
        使用世界变换矩阵 + Mesh 的 extent 属性来精确计算范围
        返回: [(min_x, max_x, min_y, max_y, surface_z), ...]
        """
        surface_bounds_list = []
        
        print(f"\n[Surface-Bounds] ========== 计算每个表面范围 ==========")
        
        def find_mesh_in_prim(parent_prim, depth=0, max_depth=5):
            """递归查找 prim 下的 Mesh 子节点"""
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
                print(f"[Surface-Bounds] ❌ Prim 无效: {path}")
                continue
            
            print(f"[Surface-Bounds] 检查: {path}, 类型: {prim.GetTypeName()}")
            
            # 首先打印 prim 的世界位置（用于诊断）
            xformable = UsdGeom.Xformable(prim)
            if xformable:
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                translation = world_transform.ExtractTranslation()
                print(f"[Surface-Bounds]   世界位置: ({translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f})")
            
            # 查找实际的 Mesh prim
            mesh_prim = find_mesh_in_prim(prim)
            target_prim = mesh_prim if mesh_prim else prim
            target_path = target_prim.GetPath().pathString
            print(f"[Surface-Bounds]   使用 prim: {target_path}, 类型: {target_prim.GetTypeName()}")
            
            # 方法1：尝试获取 Mesh 的 extent 属性
            extent_attr = target_prim.GetAttribute("extent")
            local_extent = None
            if extent_attr and extent_attr.HasValue():
                local_extent = extent_attr.Get()
                if local_extent and len(local_extent) >= 2:
                    print(f"[Surface-Bounds]   本地 extent: min={local_extent[0]}, max={local_extent[1]}")
            
            # 获取世界变换矩阵
            target_xformable = UsdGeom.Xformable(target_prim)
            if not target_xformable:
                print(f"[Surface-Bounds] ❌ 无法获取 Xformable: {target_path}")
                continue
            
            world_transform = target_xformable.ComputeLocalToWorldTransform(0)
            
            # 如果有 extent，用它的四个角来计算世界范围
            if local_extent and len(local_extent) >= 2:
                local_min = local_extent[0]
                local_max = local_extent[1]
                
                # 构建本地空间的8个角点（用于3D盒子）或4个角点（用于平面）
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
                # 备选：假设单位平面
                print(f"[Surface-Bounds]   ⚠️ 无 extent，使用单位平面假设")
                local_corners = [
                    Gf.Vec3d(-0.5, -0.5, 0),
                    Gf.Vec3d(0.5, -0.5, 0),
                    Gf.Vec3d(-0.5, 0.5, 0),
                    Gf.Vec3d(0.5, 0.5, 0),
                ]
            
            # 变换到世界坐标并计算范围
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
            
            print(f"[Surface-Bounds]   计算后范围: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}], Z={surface_z:.2f}")
            
            # 添加 5% 边距余量
            range_x = max_x - min_x
            range_y = max_y - min_y
            margin_x = range_x * 0.05
            margin_y = range_y * 0.1
            min_x += margin_x
            max_x -= margin_x
            min_y += margin_y
            max_y -= margin_y
            
            # 使用带边距的范围
            bounds = (min_x, max_x, min_y, max_y, surface_z)
            surface_bounds_list.append(bounds)
            
            print(f"[Surface-Bounds]   最终范围: X=[{bounds[0]:.2f}, {bounds[1]:.2f}], Y=[{bounds[2]:.2f}, {bounds[3]:.2f}], Z={surface_z:.2f}")
        
        print(f"[Surface-Bounds] 共 {len(surface_bounds_list)} 个有效表面")
        print(f"[Surface-Bounds] ==========================================\n")
        
        return surface_bounds_list
    
    # 获取每个表面的范围（列表）
    surface_bounds_list = get_each_surface_bounds(valid_cabinet_prims)
    
    # 从 bounds_list 获取正确的 surface_z_height（取最大值）
    # 这比 get_surface_z_height 更可靠
    if surface_bounds_list:
        surface_z_height = max(bounds[4] for bounds in surface_bounds_list)
        print(f"[kitchen_headless] ✓ surface_z_height 从 bounds_list 更新为: {surface_z_height:.4f}")

    # 11.6 记录完整初始姿态（用于每个 pair 的 A 帧复位）
    def capture_initial_xform_state(target_paths):
        state = {}
        for path in target_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                continue
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                continue
            ops_state = []
            for op in xformable.GetOrderedXformOps():
                op_name = op.GetName()
                op_suffix = ""
                if op_name and op_name.startswith("xformOp:"):
                    parts = op_name.split(":")
                    if len(parts) > 2:
                        op_suffix = parts[2]
                ops_state.append({
                    "type": op.GetOpType(),
                    "precision": op.GetPrecision(),
                    "suffix": op_suffix,
                    "value": op.Get(),
                })
            state[path] = {
                "reset_xform_stack": xformable.GetResetXformStack(),
                "ops": ops_state,
            }
        return state

    def restore_initial_xform_state(state_map):
        for path, state in state_map.items():
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                continue
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                continue

            # 先清空现有的 xformOpOrder
            try:
                xformable.ClearXformOpOrder()
            except Exception:
                pass

            # 恢复 resetXformStack
            reset_stack = state.get("reset_xform_stack", False)
            try:
                xformable.SetResetXformStack(reset_stack)
            except Exception:
                pass

            # 按原顺序恢复所有 ops
            for op_state in state.get("ops", []):
                op_type = op_state.get("type")
                op_precision = op_state.get("precision")
                op_suffix = op_state.get("suffix", "")
                op_value = op_state.get("value")
                try:
                    new_op = xformable.AddXformOp(op_type, op_precision, op_suffix)
                    if op_value is not None:
                        new_op.Set(op_value)
                except Exception:
                    pass

    # 11.6 Z 补偿（单个物体）：让 bbox 底部贴合表面
    def apply_z_compensation_single(move_path: str, target_surface_z: float):
        prim = stage.GetPrimAtPath(move_path)
        if not prim or not prim.IsValid():
            print(f"[Z-comp] ❌ Prim 无效: {move_path}")
            return

        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            print(f"[Z-comp] ❌ 无法获取 Xformable: {move_path}")
            return

        ops = xformable.GetOrderedXformOps()
        translate_op = None
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if not translate_op:
            print(f"[Z-comp] ⚠️ {move_path}: 没有 translate op，跳过")
            return

        current_pos = translate_op.Get()
        if not current_pos:
            print(f"[Z-comp] ⚠️ {move_path}: translate op 无值")
            return

        bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
        world_bbox = bbox_cache.ComputeWorldBound(prim)
        if not world_bbox or world_bbox.GetRange().IsEmpty():
            print(f"[Z-comp] ⚠️ 无法计算 bbox: {move_path}")
            return

        bbox_min_z = world_bbox.GetRange().GetMin()[2]
        z_adjustment = target_surface_z - bbox_min_z
        new_z = current_pos[2] + z_adjustment
        translate_op.Set(Gf.Vec3d(current_pos[0], current_pos[1], new_z))
        print(f"[Z-comp] {move_path}: target_z={target_surface_z:.4f}, bbox_min_z={bbox_min_z:.4f}, new_z={new_z:.4f}")
    
    # 11.6 纯 USD API 随机化函数（完全控制，不依赖 Replicator trigger）
    import random as py_random
    
    def manual_randomize_objects(prim_path_map, bounds_list):
        """
        使用纯 USD API 随机化物体位置和旋转
        每个物体随机选择一个平面，然后在该平面范围内放置
        
        Args:
            prim_path_map: {original_path: movable_path}
            bounds_list: 表面范围列表 [(min_x, max_x, min_y, max_y, surface_z), ...]
        
        Returns:
            dict: {prim_path: surface_z} 每个物体被放到的平面 Z 高度
        """
        prim_surface_z_map = {}
        
        if not bounds_list:
            print(f"[Manual-Rand] ❌ 没有有效的表面范围")
            return prim_surface_z_map
        
        print(f"\n[Manual-Rand] ========== 开始 USD API 随机化 ==========")
        print(f"[Manual-Rand] 可用表面数量: {len(bounds_list)}")
        
        for orig_path, move_path in prim_path_map.items():
            prim = stage.GetPrimAtPath(move_path)
            if not prim or not prim.IsValid():
                print(f"[Manual-Rand] ❌ Prim 无效: {move_path} (orig {orig_path})")
                continue
            
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                print(f"[Manual-Rand] ❌ 无法获取 Xformable: {path}")
                continue
            
            # 随机选择一个平面
            selected_bounds = py_random.choice(bounds_list)
            min_x, max_x, min_y, max_y, surface_z = selected_bounds
            surface_idx = bounds_list.index(selected_bounds)
            
            # 记录这个物体被放到的表面 Z 高度
            prim_surface_z_map[move_path] = surface_z
            
            # 随机生成位置（在选中的平面范围内）
            rand_x = py_random.uniform(min_x, max_x)
            rand_y = py_random.uniform(min_y, max_y)
            # 直接强制 Z，不依赖 bbox 补偿
            rand_z = surface_z
            
            # 随机旋转：X 轴 0 或 90 度（站立或倒下），Z 轴 0-360 度
            rand_rot_x = py_random.choice([0, 0])
            rand_rot_y = 0
            rand_rot_z = py_random.uniform(0, 360)
            
            # 获取或创建 xform ops
            ops = xformable.GetOrderedXformOps()
            translate_op = None
            rotate_op = None
            
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                    rotate_op = op
            
            # 设置位置
            if translate_op:
                translate_op.Set(Gf.Vec3d(rand_x, rand_y, rand_z))
            else:
                translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
                translate_op.Set(Gf.Vec3d(rand_x, rand_y, rand_z))
            
            # 设置旋转
            if rotate_op:
                rotate_op.Set(Gf.Vec3f(rand_rot_x, rand_rot_y, rand_rot_z))
            else:
                rotate_op = xformable.AddRotateXYZOp(UsdGeom.XformOp.PrecisionFloat)
                rotate_op.Set(Gf.Vec3f(rand_rot_x, rand_rot_y, rand_rot_z))
            
            state = "站立" if rand_rot_x == 0 else "倒下"
            
            # 验证位置是否在范围内
            in_range_x = min_x <= rand_x <= max_x
            in_range_y = min_y <= rand_y <= max_y
            if not in_range_x or not in_range_y:
                print(f"[Manual-Rand] ❌ {path}: 位置超出范围!")
                print(f"[Manual-Rand]   bounds: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}]")
                print(f"[Manual-Rand]   pos: X={rand_x:.2f} {'✓' if in_range_x else '❌'}, Y={rand_y:.2f} {'✓' if in_range_y else '❌'}")
            
            print(f"[Manual-Rand] ✓ {orig_path}: 平面{surface_idx}(Z={surface_z:.2f}), bounds=X[{min_x:.2f},{max_x:.2f}],Y[{min_y:.2f},{max_y:.2f}], pos=({rand_x:.2f}, {rand_y:.2f}, {rand_z:.2f}), rot=({rand_rot_x}, {rand_rot_y}, {rand_rot_z:.0f}) [{state}]")
        
        print(f"[Manual-Rand] ========== 随机化完成 ==========\n")
        return prim_surface_z_map

    # 12. 为每个相机渲染
    pairs_created_total = 0
    for cam_path in camera_list:
        print(f"\n[kitchen_headless] ========== Rendering camera: {cam_path} ==========")

        render_product = rep.create.render_product(cam_path, resolution)

        cam_out_dir = out_dir
        raw_dir = cam_out_dir / "_raw_frames"
        os.makedirs(raw_dir, exist_ok=True)
        

        
        # 清理目录中的旧文件
        for old_file in raw_dir.glob("*.*"):
            old_file.unlink(missing_ok=True)
        
        # 现在初始化并 attach writer
        writer.initialize(
            output_dir=str(raw_dir),
            rgb=True,
            distance_to_camera=True,              # 深度图 (depth map)
            instance_segmentation=True,           # 实例分割图
            colorize_instance_segmentation=True,  # 彩色实例分割
            semantic_segmentation=True,           # 语义分割图
            colorize_semantic_segmentation=True,  # 彩色语义分割
            bounding_box_2d_tight=False
        )
        writer.attach(render_product)
        # 默认关闭捕获，由 capture_frame 控制
        rep.orchestrator.set_capture_on_play(False)
        print(f"[kitchen_headless] Writer attached, capture controlled per frame")

        # 创建 pair 计划：每个 pair 随机移动一个 prim
        total_pairs = max(int(pair_count), 1)
        total_frames = total_pairs * 2
        changes_per_pair = max(int(num_changes), 1)
        changes_per_pair = min(changes_per_pair, len(movable_candidate_paths))
        pair_records = []
        for pair_idx in range(total_pairs):
            pair_id = f"pair_{pair_start_index + pair_idx:04d}"
            pair_records.append({
                "scene_type": "livingroom_shelf",
                "change_type": "move",
                "num_changes": changes_per_pair,
                "pair_id": pair_id,
                "camera": cam_path,
                "coordinates": {},  # 将在渲染时填充
            })

        print(f"[kitchen_headless] Generating {total_pairs} pairs ({total_frames} frames)")

        # 定义获取世界坐标的函数（与 childrenroom 一致）
        def get_world_coordinates(prim_paths, prim_map=None):
            """获取指定 prims 的世界坐标"""
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
            使用相机投影矩阵将3D世界坐标投影到2D像素坐标（不依赖语义分割）
            
            Args:
                prim_paths: prim 路径列表
                camera_path: 相机路径
                image_width: 图像宽度
                image_height: 图像高度
            
            Returns:
                dict: {prim_path: [pixel_x, pixel_y]} 或 {prim_path: None} 如果投影失败
            """
            coords_2d = {}
            
            # 获取相机 prim
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
            
            # 获取相机的世界变换矩阵（相机到世界）
            camera_xformable = UsdGeom.Xformable(camera_prim)
            camera_world_transform = camera_xformable.ComputeLocalToWorldTransform(0)
            # 世界到相机的变换（逆矩阵）
            world_to_camera = camera_world_transform.GetInverse()
            
            # 获取相机参数
            focal_length = camera.GetFocalLengthAttr().Get()  # mm
            h_aperture = camera.GetHorizontalApertureAttr().Get()  # mm
            v_aperture = camera.GetVerticalApertureAttr().Get()  # mm
            
            if focal_length is None or h_aperture is None:
                print(f"[kitchen_headless] Warning: Camera parameters not available")
                for path in prim_paths:
                    coords_2d[path] = None
                return coords_2d
            
            # 如果 v_aperture 为 None，从宽高比计算
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
                
                # 获取物体的世界坐标
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                world_pos = world_transform.ExtractTranslation()
                
                # 将世界坐标转换到相机坐标系
                # 使用 TransformAffine 方法处理3D点
                world_pos_3d = Gf.Vec3d(world_pos[0], world_pos[1], world_pos[2])
                camera_pos = world_to_camera.TransformAffine(world_pos_3d)
                
                # 相机坐标系中的位置
                cam_x = camera_pos[0]
                cam_y = camera_pos[1]
                cam_z = camera_pos[2]
                
                # 相机看向 -Z，所以 z 应该是负的才能被看到
                if cam_z >= 0:
                    # 物体在相机后面
                    print(f"[kitchen_headless]   {path} is behind camera (z={cam_z:.2f})")
                    coords_2d[path] = None
                    continue
                
                # 透视投影（针孔相机模型）
                # NDC 坐标 (归一化设备坐标)
                ndc_x = (focal_length * cam_x) / (-cam_z * h_aperture / 2)
                ndc_y = (focal_length * cam_y) / (-cam_z * v_aperture / 2)
                
                # NDC 范围是 [-1, 1]，转换到像素坐标
                # 注意：y 轴在图像中是向下的
                pixel_x = int((ndc_x + 1) * 0.5 * image_width)
                pixel_y = int((1 - ndc_y) * 0.5 * image_height)  # 翻转 Y
                
                # 检查是否在图像范围内
                if 0 <= pixel_x < image_width and 0 <= pixel_y < image_height:
                    coords_2d[path] = [pixel_x, pixel_y]
                    print(f"[kitchen_headless]   Projected {path}: world ({world_pos[0]:.1f}, {world_pos[1]:.1f}, {world_pos[2]:.1f}) -> pixel ({pixel_x}, {pixel_y})")
                else:
                    print(f"[kitchen_headless]   {path} projected outside image: ({pixel_x}, {pixel_y})")
                    coords_2d[path] = [pixel_x, pixel_y]  # 仍然保存，即使在边界外
            
            return coords_2d

        # 13-15. 直接使用 USD API（当前不随机化）
        frame_coordinates = []
        frame_2d_coordinates = []
        frame_move_targets = []

        # 16. 主渲染序列：A=放置前，B=放置后
        flush_frames_before_capture = 6  # 捕获前刷新的帧数
        print(f"[kitchen_headless] Running move sequence with {total_frames} frames...")
        print(f"[kitchen_headless] Using {flush_frames_before_capture} flush frames before each capture to eliminate ghosting")

        # 预热：不做任何操作，仅运行固定帧数
        warmup_frames = 40
        print(f"[kitchen_headless] Warmup {warmup_frames} frames before capture...")
        rep.orchestrator.set_capture_on_play(False)
        for _ in range(warmup_frames):
            rep.orchestrator.step()

        captured_rgb_frames = []
        captured_depth_frames = []
        captured_instance_frames = []
        captured_semantic_frames = []

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

        def capture_frame(frame_label: str, move_targets: list[str] | None):
            before_rgb = snapshot_files(["rgb_*.png"])
            before_depth_npy = snapshot_files(["distance_to_camera_*.npy"])
            before_depth_png = snapshot_files(["distance_to_camera_*.png"])
            before_instance = snapshot_files(["instance_segmentation_*.png"])
            before_semantic = snapshot_files(["semantic_segmentation_*.png"])
            set_prim_visibility(hidden_prim_path, "invisible")
            rep.orchestrator.set_capture_on_play(False)
            for _ in range(flush_frames_before_capture):
                rep.orchestrator.step()
            # 触发捕获，确保至少生成 1 帧
            rgb_file = None
            depth_file = None
            instance_file = None
            semantic_file = None
            for _ in range(3):
                rep.orchestrator.set_capture_on_play(True)
                rep.orchestrator.step()
                rep.orchestrator.set_capture_on_play(False)
                rgb_file = pick_new_file(before_rgb, ["rgb_*.png"])
                depth_file = pick_new_file(before_depth_npy, ["distance_to_camera_*.npy"])
                if depth_file is None:
                    depth_file = pick_new_file(before_depth_png, ["distance_to_camera_*.png"])
                instance_file = pick_new_file(before_instance, ["instance_segmentation_*.png"])
                semantic_file = pick_new_file(before_semantic, ["semantic_segmentation_*.png"])
                if rgb_file:
                    break
            print(f"[kitchen_headless] Captured frame: {frame_label}")

            # 记录当前帧的世界坐标与 2D 坐标（只记录移动目标）
            current_coords = {}
            current_2d_coords = {}
            if move_targets:
                current_coords = get_world_coordinates(move_targets, prim_path_map)
                current_2d_coords = get_2d_center_coordinates_from_projection(
                    move_targets, cam_path, resolution[0], resolution[1], prim_path_map
                )
            frame_coordinates.append(current_coords)
            frame_2d_coordinates.append(current_2d_coords)
            frame_move_targets.append(move_targets or [])

            if rgb_file is None:
                print(f"[kitchen_headless] Warning: No RGB captured for {frame_label}")
            captured_rgb_frames.append(rgb_file)
            captured_depth_frames.append(depth_file)
            captured_instance_frames.append(instance_file)
            captured_semantic_frames.append(semantic_file)

        # 每个 pair 随机移动 k 个物体，A=移动前，B=移动后
        pair_targets = []

        def move_target_to_plane(target_path: str):
            move_path = prim_path_map.get(target_path, target_path)
            prim = stage.GetPrimAtPath(move_path)
            if not prim or not prim.IsValid():
                print(f"[Move] ❌ Prim 无效: {move_path}")
                return None, None
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                print(f"[Move] ❌ 无法获取 Xformable: {move_path}")
                return None, None

            selected_bounds = py_random.choice(surface_bounds_list)
            min_x, max_x, min_y, max_y, surface_z = selected_bounds
            rand_x = py_random.uniform(min_x, max_x)
            rand_y = py_random.uniform(min_y, max_y)

            ops = xformable.GetOrderedXformOps()
            translate_op = None
            rotate_op = None
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                    rotate_op = op

            # Z 先归零（由后续补偿对齐到平面，避免叠加初始高度）
            base_z = 0.0
            if translate_op:
                translate_op.Set(Gf.Vec3d(rand_x, rand_y, base_z))
            else:
                translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
                translate_op.Set(Gf.Vec3d(rand_x, rand_y, base_z))

            rand_rot_x = 0
            rand_rot_y = 0
            rand_rot_z = py_random.uniform(0, 360)
            if rotate_op:
                rotate_op.Set(Gf.Vec3f(rand_rot_x, rand_rot_y, rand_rot_z))
            else:
                rotate_op = xformable.AddRotateXYZOp(UsdGeom.XformOp.PrecisionFloat)
                rotate_op.Set(Gf.Vec3f(rand_rot_x, rand_rot_y, rand_rot_z))

            print(f"[Move] ✓ {target_path} -> ({rand_x:.2f}, {rand_y:.2f}, {base_z:.2f}) rotZ={rand_rot_z:.0f}")
            return move_path, surface_z

        # 记录初始姿态（移动目标是可移动 prim 的映射路径）
        initial_xform_state = capture_initial_xform_state(
            [prim_path_map.get(p, p) for p in movable_candidate_paths]
        )

        for pair_idx in range(total_pairs):
            if changes_per_pair >= len(movable_candidate_paths):
                move_targets = list(movable_candidate_paths)
            else:
                move_targets = py_random.sample(movable_candidate_paths, k=changes_per_pair)
            pair_targets.append(move_targets)
            print(f"[kitchen_headless] Move pair {pair_idx + 1}/{total_pairs}: {move_targets}")

            # 每个 pair 都先恢复到初始状态，确保 A 帧一致
            restore_initial_xform_state(initial_xform_state)

            capture_frame(f"pair_{pair_idx:02d}_A", move_targets)
            for move_target in move_targets:
                move_path, surface_z = move_target_to_plane(move_target)
                if move_path and surface_z is not None:
                    apply_z_compensation_single(move_path, surface_z)
            capture_frame(f"pair_{pair_idx:02d}_B_move_{move_targets}", move_targets)

        print(f"[kitchen_headless] Rendering complete. Recorded {len(frame_coordinates)} world coord frames, {len(frame_2d_coordinates)} 2D coord frames.")
        
        # 等待所有写入操作完成
        rep.orchestrator.wait_until_complete()
        print(f"[kitchen_headless] All frames written to disk.")

        # 17. 重新整理输出（包含 RGB、深度图、分割图）
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
        semantic_frames = [p for p in captured_semantic_frames if p]
        if not rgb_frames:
            rgb_frames = sort_frames(raw_dir.glob("rgb_*.png"))
        if not depth_frames:
            depth_frames = sort_frames(raw_dir.glob("distance_to_camera_*.npy"))
            if not depth_frames:
                depth_frames = sort_frames(raw_dir.glob("distance_to_camera_*.png"))
        if not instance_frames:
            instance_frames = sort_frames(raw_dir.glob("instance_segmentation_*.png"))
        if not semantic_frames:
            semantic_frames = sort_frames(raw_dir.glob("semantic_segmentation_*.png"))
        
        print(f"[kitchen_headless] Found {len(rgb_frames)} RGB frames")
        print(f"[kitchen_headless] Found {len(depth_frames)} depth frames")
        print(f"[kitchen_headless] Found {len(instance_frames)} instance segmentation frames")
        print(f"[kitchen_headless] Found {len(semantic_frames)} semantic segmentation frames")
        print(f"[kitchen_headless] Recorded {len(frame_coordinates)} world coordinate snapshots")
        print(f"[kitchen_headless] Recorded {len(frame_2d_coordinates)} 2D coordinate snapshots")
        
        # 验证有足够的帧来创建 pair 对
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
            
            # 坐标索引和帧文件索引相同
            coord_a_idx = idx * 2
            coord_b_idx = idx * 2 + 1
            
            # 帧文件索引：直接从 0 开始
            frame_a_idx = idx * 2
            frame_b_idx = idx * 2 + 1
            
            # 填充坐标信息（记录当前 pair 的移动目标）
            if coord_a_idx < actual_coords and coord_b_idx < actual_coords:
                coords_a = frame_coordinates[coord_a_idx]
                coords_b = frame_coordinates[coord_b_idx]
                
                coords_2d_a = frame_2d_coordinates[coord_a_idx] if coord_a_idx < actual_2d_coords else {}
                coords_2d_b = frame_2d_coordinates[coord_b_idx] if coord_b_idx < actual_2d_coords else {}
                
                targets = pair_targets[idx] if idx < len(pair_targets) else []
                for target in targets:
                    record["coordinates"][target] = {
                        "world_coordinate_A": coords_a.get(target, None),
                        "world_coordinate_B": coords_b.get(target, None),
                        "pixel_center_A": coords_2d_a.get(target, None),
                        "pixel_center_B": coords_2d_b.get(target, None),
                    }
            else:
                print(f"[kitchen_headless] ⚠️ Coordinates not available for pair {idx}")
            
            # RGB（使用帧文件索引，跳过预热帧）
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
            
            # 深度图
            if frame_a_idx < len(depth_frames) and frame_b_idx < len(depth_frames):
                depth_ext = depth_frames[frame_a_idx].suffix
                shutil.copy2(depth_frames[frame_a_idx], pair_dir / f"A_depth{depth_ext}")
                shutil.copy2(depth_frames[frame_b_idx], pair_dir / f"B_depth{depth_ext}")
                
                # 如果是 npy 格式，生成 png 可视化
                if depth_ext == ".npy":
                    depth_npy_to_png(depth_frames[frame_a_idx], pair_dir / "A_depth.png")
                    depth_npy_to_png(depth_frames[frame_b_idx], pair_dir / "B_depth.png")
            
            # 实例分割图
            if frame_a_idx < len(instance_frames) and frame_b_idx < len(instance_frames):
                shutil.copy2(instance_frames[frame_a_idx], pair_dir / "A_instance_segmentation.png")
                shutil.copy2(instance_frames[frame_b_idx], pair_dir / "B_instance_segmentation.png")
            
            # 语义分割图
            if frame_a_idx < len(semantic_frames) and frame_b_idx < len(semantic_frames):
                shutil.copy2(semantic_frames[frame_a_idx], pair_dir / "A_semantic_segmentation.png")
                shutil.copy2(semantic_frames[frame_b_idx], pair_dir / "B_semantic_segmentation.png")
            
            # 保存 metadata
            meta_path = pair_dir / "metadata.json"
            meta_path.write_text(json.dumps(record, indent=2))

            if mark_centers and record["coordinates"]:
                targets = pair_targets[idx] if idx < len(pair_targets) else []
                for target in targets:
                    if target in record["coordinates"]:
                        coords = record["coordinates"][target]
                        draw_center(pair_dir / "A_rgb.png", pair_dir / "A_rgb_center.png", coords.get("pixel_center_A"))
                        draw_center(pair_dir / "B_rgb.png", pair_dir / "B_rgb_center.png", coords.get("pixel_center_B"))
            
            print(f"[kitchen_headless] Created {record['pair_id']}: A=frame{frame_a_idx}, B=frame{frame_b_idx}")
        
        # 清理 raw 目录中的文件
        for leftover in raw_dir.glob("*.*"):
            leftover.unlink(missing_ok=True)

        writer.detach()
        render_product.destroy()

    # 18. 完成
    rep.orchestrator.wait_until_complete()
    print("\n[kitchen_headless] ========== Done! ==========")
    print(f"Output saved to: {out_dir}")
    return pair_start_index + pairs_created_total


def depth_npy_to_png(npy_path, png_path):
    """
    将深度图的 npy 文件转换为可视化的 png 图像
    """
    import numpy as np
    from PIL import Image
    
    try:
        # 加载深度数据
        depth_data = np.load(npy_path)
        
        # 处理可能的多通道数据
        if len(depth_data.shape) > 2:
            depth_data = depth_data[:, :, 0] if depth_data.shape[2] >= 1 else depth_data.squeeze()
        
        # 处理无效值（如 inf, nan）
        valid_mask = np.isfinite(depth_data)
        if not valid_mask.any():
            print(f"[depth_npy_to_png] Warning: No valid depth values in {npy_path}")
            return False
        
        # 获取有效值的范围
        min_val = depth_data[valid_mask].min()
        max_val = depth_data[valid_mask].max()
        
        # 归一化到 0-255 范围
        if max_val > min_val:
            normalized = (depth_data - min_val) / (max_val - min_val)
        else:
            normalized = np.zeros_like(depth_data)
        
        # 将无效值设为 0
        normalized[~valid_mask] = 0
        
        # 反转：近处亮，远处暗
        normalized = 1.0 - normalized
        
        # 转换为 8-bit 图像
        depth_uint8 = (normalized * 255).astype(np.uint8)
        
        # 保存为 png
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
    parser.add_argument("--pair-count", type=int, default=10, help="Number of pairs to generate")
    parser.add_argument("--num-changes", type=int, default=1, help="How many prims to move per pair")
    parser.add_argument("--output-dir", type=str, default="/workspace/output/livingroom_shelf_move/4_item", help="Output directory")
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
            pair_count=args.pair_count,
            num_changes=args.num_changes,
            output_dir=args.output_dir,
        )
        if result is None:
            print("[kitchen_headless] Run aborted due to earlier errors.")
            break
        created_pairs = result - pair_start_index
        print(f"[kitchen_headless] Run {run_idx + 1} completed, created {created_pairs} pairs")
        pair_start_index = result
    simulation_app.close()
