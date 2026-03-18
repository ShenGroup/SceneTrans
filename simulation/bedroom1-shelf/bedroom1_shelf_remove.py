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
    pair_count: int = 4,
    warmup_k: int = 3,
    num_changes: int = 1,
    semantic_segmentation: bool = False,
    resolution=(1024, 768),
    focal_length: float | None = None,
    output_dir: str = "/workspace/output/bedroom1_shelf_remove/1_item",
):
    """
    Headless 版本的厨房场景示例
    只随机化特定的3个瓶子 prims，在柜台平面上
    每次随机化时会随机决定瓶子是否倒下（绕X轴旋转0或90度）
    """
    
    # 2. 打开厨房场景
    usd_path = "/workspace/assets/kujiale_0003/kujiale_0003.usda"
    print(f" Opening stage: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)

    stage = omni.usd.get_context().get_stage()

    if not stage:
        print(" Error: Failed to open stage!")
        return

    # 场景保持原样，不修改任何灯光设置
    print(" Scene loaded, preserving original lighting")

    # 3.2 删除物理场景并关闭全局物理，避免随机化后被物理推走
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene":
            stage.RemovePrim(prim.GetPath())
            print(f" Removed PhysicsScene: {prim.GetPath()}")

    import carb.settings
    carb.settings.get_settings().set("/physics/enabled", False)
    print(" Physics disabled via carb settings")

    # 3. 配置渲染设置来消除时间累积造成的鬼影
    # RTSubframes: 强制渲染器在每帧重新采样
    rep.settings.carb_settings("/omni/replicator/RTSubframes", 64)
    print(" RTSubframes set to 16 (to eliminate ghosting)")



    # 4. 配合 on_frame 触发器，让 orchestrator 驱动写盘
    rep.orchestrator.set_capture_on_play(True)

    # 4. 查找场景中已有的 Camera，使用 OmniverseKit_Persp
    desired_camera = "/Root/bedroom1_shelf"
    camera_list = []
    
    cam_prim = stage.GetPrimAtPath(desired_camera)
    if cam_prim and cam_prim.IsValid():
        camera_list = [desired_camera]
        print(f" Using camera: {desired_camera}")
    else:
        # 尝试查找场景中其他相机作为备选
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

    # 可选：调整相机焦距
    if focal_length is not None:
        for cam_path in camera_list:
            cam_prim = stage.GetPrimAtPath(cam_path)
            cam_schema = UsdGeom.Camera(cam_prim)
            if cam_schema and cam_schema.GetFocalLengthAttr():
                cam_schema.GetFocalLengthAttr().Set(float(focal_length))
                print(f" Set focalLength={focal_length} for {cam_path}")

    # 5. BasicWriter：输出到 /workspace/output/kitchen_headless
    writer = rep.writers.get("BasicWriter")
    out_dir = Path(output_dir)

    # 6. 从指定 prim 列表中收集有效 prim，并在每个 pair 内进行随机采样
    def find_valid_bottle_prims():
        """返回用户指定列表中所有有效 prim"""
        specific_prims = [
            "/Root/Meshes/bedroom_767840/ornament_0002",
            "/Root/Meshes/bedroom_767840/ornament_0000",
            "/Root/Meshes/bedroom_767840/vase_0000",
            "/Root/Meshes/bedroom_767840/ornament_0001",
        ]
        
        # 验证这些 prims 是否存在
        valid_prims = []
        for path in specific_prims:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                valid_prims.append(path)
                print(f" Found specified prim: {path}")
            else:
                print(f" Warning: Specified prim not found: {path}")

        return valid_prims

    def sample_pair_prims(valid_prims):
        """在每个 pair 开始时随机采样 num_changes 个 prim"""
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

    # 7. 检查并移除物体的碰撞体（randomize 的物体不需要 CollisionAPI）
    def remove_collision_from_bottles(bottle_paths):
        """检查并移除物体的 CollisionAPI，randomize 的物体不需要碰撞体"""
        for path in bottle_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                print(f" ❌ Prim not found: {path}")
                continue
            
            removed_count = 0

            # 取消 instanceable
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                print(f" Made prim non-instanceable: {path}")
            
            # 检查并移除 Xform 本身的 CollisionAPI
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
                print(f" ✓ Removed CollisionAPI from Xform: {path}")
                removed_count += 1
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                print(f" ✓ Removed RigidBodyAPI from Xform: {path}")
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
                        print(f" ✓ Removed CollisionAPI from {child_type}: {child_path}")
                        removed_count += 1
                    if child.HasAPI(UsdPhysics.RigidBodyAPI):
                        child.RemoveAPI(UsdPhysics.RigidBodyAPI)
                        print(f" ✓ Removed RigidBodyAPI from {child_type}: {child_path}")
                        removed_count += 1
                    
                    # 继续递归检查子节点
                    remove_collision_from_children(child, depth + 1)
            
            remove_collision_from_children(prim)
            
            if removed_count > 0:
                print(f" Physics cleanup done: {path} (removed {removed_count} CollisionAPI)")
            else:
                print(f" No CollisionAPI found on: {path}")

    # 8. 为 randomize 的物体添加语义标签（用于 instance/semantic segmentation）
    def add_semantic_labels(prim_paths):
        """
        为指定的 prims 添加语义标签，统一使用 ornament
        同时使用两种方式确保兼容性：
        1. 直接使用 USD primvars 写入语义属性（立即生效）
        2. 使用 rep.modify.semantics（用于 Replicator graph）
        """
        print(" Adding semantic labels using dual approach...")
        
        added_count = 0

        def clear_old_semantics(prim):
            for attr in list(prim.GetAttributes()):
                if attr.GetName().startswith("primvars:semantics:"):
                    prim.RemoveProperty(attr.GetName())

        def set_semantic_primvar(prim, prim_name: str):
            # 使用 UsdGeom primvar API，类型 token array，constant 插值
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
                print(f" ❌ Prim not found: {path}")
                continue
            
            # 所有目标 prim 统一语义标签
            prim_name = "ornament"
            
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
                
                added_count += 1
                print(f" ✓ Semantic label added: {path} -> '{prim_name}' (meshes: {len(mesh_children)})")

                # 打印验证（root + 第一个 mesh）
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

    # 9. 查找并验证 prims
    valid_candidate_paths = find_valid_bottle_prims()
    
    if not valid_candidate_paths:
        print(" No specified prims were found; cannot proceed.")
        return
    print(f" Found {len(valid_candidate_paths)} valid prims in specific_prims pool")

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

    reverse_prim_map = {movable: orig for orig, movable in prim_path_map.items()}
    movable_paths = list(prim_path_map.values())
    
    # 移除这些 prim 的物理碰撞体（如果有）
    remove_collision_from_bottles(valid_candidate_paths)
    
    # 应用语义标签到所有待随机化的物体
    added = add_semantic_labels(valid_candidate_paths)
    print(f" Semantic labels added: {added} prims labeled")

    # 10. 使用指定的平面进行随机放置
    cabinet_prims = [
        "Root/bedroom1_shelf_plane",
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
            print(f" ✓ Found cabinet prim: {cabinet_prim}")
        else:
            print(f" ❌ Warning: Cabinet prim not found: {cabinet_prim}")
    
    if not valid_cabinet_prims:
        print(f" ❌ Error: No valid cabinet prims found!")
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
    
    # 10.2 Z 补偿函数：在随机化 + 旋转后调用，防止物体陷入平面
    def apply_z_compensation(prim_surface_z_map, prim_display_map=None):
        """
        根据物体当前姿态的 bounding box，补偿 Z 高度使底部贴合表面
        
        核心逻辑：
        1. 获取物体当前的 translate op 中的 Z 值
        2. 计算 bbox 底部与目标表面 Z 的差距
        3. 调整 Z，使 bbox 底部 = 该物体所在平面的 surface_z
        
        Args:
            prim_surface_z_map: {prim_path: surface_z} 每个物体被放到的平面 Z 高度
        """
        if not prim_surface_z_map:
            print(f"\n[Z-comp] ⚠️ 没有需要补偿的物体")
            return
        
        print(f"\n[Z-comp] ========== 开始 Z 补偿 ==========")
        
        bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
        
        for path, target_surface_z in prim_surface_z_map.items():
            display_name = prim_display_map.get(path, path) if prim_display_map else path
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[Z-comp] ❌ Prim 无效: {display_name}")
                continue
            
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                print(f"[Z-comp] ❌ 无法获取 Xformable: {display_name}")
                continue
            
            # 获取当前 translate op
            ops = xformable.GetOrderedXformOps()
            translate_op = None
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break
            
            if not translate_op:
                print(f"[Z-comp] ⚠️ {display_name}: 没有 translate op，跳过")
                continue
            
            current_pos = translate_op.Get()
            if not current_pos:
                print(f"[Z-comp] ⚠️ {display_name}: translate op 无值")
                continue
            
            current_z = current_pos[2]
            
            # 计算当前姿态下的 world bounding box
            bbox_cache.Clear()
            world_bbox = bbox_cache.ComputeWorldBound(prim)
            if not world_bbox or world_bbox.GetRange().IsEmpty():
                print(f"[Z-comp] ⚠️ 无法计算 bbox: {display_name}")
                continue
            
            bbox_min_z = world_bbox.GetRange().GetMin()[2]
            
            # 计算需要的 Z 调整：让 bbox 底部对齐到该物体所在平面的 surface_z
            z_adjustment = target_surface_z - bbox_min_z
            new_z = current_z + z_adjustment
            
            print(f"[Z-comp] {display_name}: target_z={target_surface_z:.4f}, current_z={current_z:.4f}, bbox_min_z={bbox_min_z:.4f}")
            print(f"[Z-comp]   adjustment={z_adjustment:.4f}, new_z={new_z:.4f}")
            
            # 设置新位置
            new_pos = Gf.Vec3d(current_pos[0], current_pos[1], new_z)
            translate_op.Set(new_pos)
            
            # 验证
            bbox_cache.Clear()
            new_bbox = bbox_cache.ComputeWorldBound(prim)
            if new_bbox:
                new_min_z = new_bbox.GetRange().GetMin()[2]
                print(f"[Z-comp] ✓ 验证: 新 bbox_min_z = {new_min_z:.4f} (期望 {target_surface_z:.4f})")
        
        print(f"[Z-comp] ========== Z 补偿完成 ==========\n")

    # 是否启用 Z 补偿（如仅强制设置 Z，可关闭）
    use_z_compensation = True

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
            print(f" Using mesh for scatter_2d: {actual_mesh}")
        else:
            # 如果找不到子 Mesh，尝试直接使用该 prim
            print(f" Will try using prim directly: {cabinet_prim}")
            actual_cabinet_meshes.append(cabinet_prim)
    
    if not actual_cabinet_meshes:
        print(" ❌ Error: No valid meshes found for scatter_2d")
        return
    
    # 获取柜台 prims 作为 scatter_2d 的表面
    surface_nodes = []
    for mesh_path in actual_cabinet_meshes:
        prim_node = rep.get.prims(path_pattern=mesh_path)
        if prim_node:
            surface_nodes.append(prim_node)
            print(f" Added surface: {mesh_path}")
    
    if not surface_nodes:
        print(" ❌ Error: No valid surface found for scatter_2d")
        return
    
    # 如果有多个表面，创建一个组；否则直接使用单个表面
    if len(surface_nodes) == 1:
        surface = surface_nodes[0]
    else:
        surface = rep.create.group(surface_nodes)
        print(f" Created surface group with {len(surface_nodes)} meshes")
    
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
        print(f" ✓ surface_z_height 从 bounds_list 更新为: {surface_z_height:.4f}")
    
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
    for cam_path in camera_list:
        print(f"\n ========== Rendering camera: {cam_path} ==========")

        render_product = rep.create.render_product(cam_path, resolution)

        cam_out_dir = out_dir
        raw_dir = cam_out_dir / "_raw_frames"
        os.makedirs(raw_dir, exist_ok=True)
        
        # 先运行暖机帧来刷新时间缓存（移除原始位置的残影）
        # warmup 帧数 = 3 * warmup_k，确保是 3 的倍数
        warmup_frames = 3 * warmup_k
        print(f" Running {warmup_frames} warmup frames (k={warmup_k}) with randomization...")
        rep.orchestrator.set_capture_on_play(False)
        
        # 在 warmup 期间，直接用 USD API 移动瓶子到随机位置
        # 这样可以刷掉原始位置的时间缓存
        # 每 3 帧移动一次，共移动 warmup_k 次
        import random as py_random
        for warmup_i in range(warmup_frames):
            # 每隔 3 帧移动一次瓶子，刷新缓存
            if warmup_i % 3 == 0:
                for path in valid_candidate_paths:
                    target_path = prim_path_map.get(path, path)
                    prim = stage.GetPrimAtPath(target_path)
                    if prim:
                        xformable = UsdGeom.Xformable(prim)
                        if xformable:
                            # 获取现有的 translate op
                            for op in xformable.GetOrderedXformOps():
                                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                    # 微调位置来刷新缓存
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
        
        print(f" Warmup done, original positions flushed from cache.")

        # 诊断：直接从 annotator 拉取一帧分割数据，确认非零
        def log_annotator_sample(render_prod):
            import numpy as np

            semantic_anno = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
            instance_anno = rep.AnnotatorRegistry.get_annotator("instance_segmentation")
            semantic_anno.attach(render_prod)
            instance_anno.attach(render_prod)

            # capture 关闭状态下手动 step 一帧
            rep.orchestrator.step()

            sem = semantic_anno.get_data()
            ins = instance_anno.get_data()
            def summarize(arr, name):
                # 兼容不同返回格式（dict 或 ndarray）
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

        # log_annotator_sample(render_product)  # 临时禁用诊断，避免额外的 step()
        
        # 清理目录中的旧文件
        for old_file in raw_dir.glob("*.*"):
            old_file.unlink(missing_ok=True)
        
        # 现在初始化并 attach writer（warmup 之后）
        writer.initialize(
            output_dir=str(raw_dir),
            rgb=True,
            distance_to_camera=True,           # 深度图 (depth map)
            semantic_segmentation=semantic_segmentation,        # 语义分割图（开关控制）
            colorize_semantic_segmentation=semantic_segmentation,  # 彩色语义分割（开关控制）
            instance_segmentation=True,        # 实例分割图
            colorize_instance_segmentation=True,  # 彩色实例分割
            bounding_box_2d_tight=False
        )
        writer.attach(render_product)
        
        # 开启捕获
        rep.orchestrator.set_capture_on_play(True)
        print(f" Writer attached, starting capture...")

        # 创建 pair 计划
        total_frames = pair_count * 2
        pair_records = []
        
        for pair_idx in range(pair_count):
            pair_records.append({
                "scene_type": "bedroom1_shelf",
                "change_type": "remove",
                "pair_id": f"pair_{pair_idx:04d}",
                "camera": cam_path,
                "num_changes": 0,
                "selected_prims": [],
                "coordinates": {},  # 将在渲染时填充
            })

        print(f" Generating {pair_count} pairs ({total_frames} frames)")

        # 定义获取世界坐标的函数
        def get_world_coordinates(prim_paths, prim_map=None):
            """获取指定 prims 的世界坐标"""
            coords = {}
            for path in prim_paths:
                target_path = prim_map.get(path, path) if prim_map else path
                prim = stage.GetPrimAtPath(target_path)
                if prim and prim.IsValid():
                    xformable = UsdGeom.Xformable(prim)
                    if xformable:
                        # 获取世界变换矩阵
                        world_transform = xformable.ComputeLocalToWorldTransform(0)
                        # 提取平移部分（世界坐标）
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
                print(f" Warning: Camera parameters not available")
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
                    print(f"   {path} is behind camera (z={cam_z:.2f})")
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
                    print(f"   Projected {path}: world ({world_pos[0]:.1f}, {world_pos[1]:.1f}, {world_pos[2]:.1f}) -> pixel ({pixel_x}, {pixel_y})")
                else:
                    print(f"   {path} projected outside image: ({pixel_x}, {pixel_y})")
                    coords_2d[path] = [pixel_x, pixel_y]  # 仍然保存，即使在边界外
            
            return coords_2d

        def set_prims_visibility(prim_paths, visible: bool, prim_map=None):
            """
            设置 prim 可见性。
            - visible=True: 显示
            - visible=False: 隐藏
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

        # 13-15. 直接使用 USD API 随机化，不使用 Replicator trigger
        # 这样我们完全控制何时执行随机化，避免 Z 补偿被覆盖
        
        frame_counter = [0]
        frame_coordinates = []
        frame_2d_coordinates = []
        
        # 预热步骤：运行多帧来刷新渲染器的时间累积缓存，消除鬼影
        # 每次随机化后运行 3 帧让渲染器稳定，总共 5 次随机化 = 15 帧
        warmup_randomizations = 5
        warmup_steps_per_rand = 3
        print(f" Running warmup: {warmup_randomizations} randomizations × {warmup_steps_per_rand} steps = {warmup_randomizations * warmup_steps_per_rand} frames...")
        
        # Detach writer 在 warmup 期间，避免捕获这些帧
        writer.detach()
        
        for warmup_i in range(warmup_randomizations):
            prim_z_map = manual_randomize_objects(prim_path_map, surface_bounds_list)
            if use_z_compensation:
                apply_z_compensation(prim_z_map, reverse_prim_map)
            # 每次随机化后运行多帧让时间累积刷新
            for _ in range(warmup_steps_per_rand):
                rep.orchestrator.step()
        
        # 重新 attach writer 开始正式捕获
        writer.attach(render_product)
        print(f" Warmup complete, writer re-attached.")
        
        # 16. 主渲染循环
        # 每帧在捕获前先运行几帧刷新时间累积缓存，消除 A/B 帧之间的鬼影
        flush_frames_before_capture = 6  # 捕获前刷新的帧数（增加到 6 帧）
        print(f" Running main render loop with {total_frames} frames...")
        print(f" Using {flush_frames_before_capture} flush frames before each capture to eliminate ghosting")
        
        for frame_idx in range(total_frames):
            frame_counter[0] += 1
            print(f"\n ===== Frame {frame_idx} =====")

            # 每个 pair（A/B 两帧）重新采样一次待移除 prim 集合
            pair_idx = frame_idx // 2
            if frame_idx % 2 == 0:
                sampled_paths = sample_pair_prims(valid_candidate_paths)
                sampled_paths = [p for p in sampled_paths if p in prim_path_map]
                pair_records[pair_idx]["selected_prims"] = sampled_paths
                pair_records[pair_idx]["num_changes"] = len(sampled_paths)
                print(f" Pair {pair_idx}: selected {len(sampled_paths)} prim(s) to hide in B: {sampled_paths}")

                # A 帧：先确保所有受控物体可见，再进行正常随机化
                set_prims_visibility(valid_candidate_paths, True, prim_path_map)
                tracked_paths = list(prim_path_map.keys())
                tracked_prim_map = prim_path_map

                # A 使用 USD API 随机化所有受控物体位置和旋转
                prim_z_map = manual_randomize_objects(tracked_prim_map, surface_bounds_list)
                if use_z_compensation:
                    apply_z_compensation(prim_z_map, reverse_prim_map)
            else:
                sampled_paths = pair_records[pair_idx].get("selected_prims", [])
                tracked_paths = list(prim_path_map.keys())
                tracked_prim_map = prim_path_map
                # B 帧：不再移动物体，只隐藏抽中的物体
                hidden_paths = set_prims_visibility(sampled_paths, False, prim_path_map)
                print(f" Pair {pair_idx}: B frame hidden {len(hidden_paths)} prim(s): {hidden_paths}")
            
            # 步骤 3：刷新帧 - detach writer，运行几帧刷新时间累积缓存
            writer.detach()
            for _ in range(flush_frames_before_capture):
                rep.orchestrator.step()
            
            # 步骤 4：渲染并捕获 - attach writer，捕获最终帧
            writer.attach(render_product)
            rep.orchestrator.step()
            print(f" Frame {frame_idx}: rendered and captured (after {flush_frames_before_capture} flush frames)")
            
            # 验证捕获后物体的实际位置
            print(f"[Verify] ===== 捕获后位置验证 =====")
            for path in tracked_paths:
                target_path = tracked_prim_map.get(path, path)
                prim = stage.GetPrimAtPath(target_path)
                if prim and prim.IsValid():
                    xformable = UsdGeom.Xformable(prim)
                    if xformable:
                        # 获取 translate op 的值
                        for op in xformable.GetOrderedXformOps():
                            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                pos = op.Get()
                                print(f"[Verify] {path}: translate = ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
                                break
                        # 计算 bbox
                        bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
                        world_bbox = bbox_cache.ComputeWorldBound(prim)
                        if world_bbox and not world_bbox.GetRange().IsEmpty():
                            bbox_min = world_bbox.GetRange().GetMin()
                            print(f"[Verify]   bbox_min_z = {bbox_min[2]:.4f}")
            
            # 渲染完成后记录世界坐标
            current_coords = get_world_coordinates(tracked_paths, tracked_prim_map)
            frame_coordinates.append(current_coords)
            
            # 使用相机投影计算2D像素坐标
            current_2d_coords = get_2d_center_coordinates_from_projection(
                tracked_paths, cam_path, resolution[0], resolution[1], tracked_prim_map
            )
            frame_2d_coordinates.append(current_2d_coords)
            print(f" Frame {frame_idx}: recorded coords for {len(current_coords)} prims")
        
        print(f" Rendering complete. Recorded {len(frame_coordinates)} world coord frames, {len(frame_2d_coordinates)} 2D coord frames.")
        
        # 等待所有写入操作完成
        rep.orchestrator.wait_until_complete()
        print(f" All frames written to disk.")

        # 17. 重新整理输出（包含 RGB、深度图、分割图）
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
        
        # 验证有足够的帧来创建 pair 对
        # 新逻辑：warmup 期间 writer detached，不产生文件，只有正式渲染帧
        # 预期帧数 = pair_count * 2
        min_frames_needed = pair_count * 2
        actual_rgb = len(rgb_frames)
        actual_coords = len(frame_coordinates)
        
        print(f" Expected: {min_frames_needed} frames (warmup not captured)")
        print(f" Actual: {actual_rgb} RGB frames, {actual_coords} coord snapshots")
        
        if actual_rgb < min_frames_needed:
            print(f" ⚠️ Warning: Need {min_frames_needed} frames but found {actual_rgb}")
            # 尽可能创建能创建的 pair 数量
            pair_count = min(pair_count, actual_rgb // 2)
        
        if actual_coords < min_frames_needed:
            print(f" ⚠️ Warning: Need {min_frames_needed} coordinate snapshots but found {actual_coords}")
        
        print(f" Will create {pair_count} pairs")
        
        for idx in range(pair_count):
            record = pair_records[idx]
            pair_dir = cam_out_dir / record["pair_id"]
            pair_dir.mkdir(parents=True, exist_ok=True)
            
            # 坐标索引和帧文件索引相同（warmup 不产生文件）
            coord_a_idx = idx * 2
            coord_b_idx = idx * 2 + 1
            
            # 帧文件索引：直接从 0 开始
            frame_a_idx = idx * 2
            frame_b_idx = idx * 2 + 1
            
            # 填充坐标信息（使用坐标索引）
            actual_2d_coords = len(frame_2d_coordinates)
            if coord_a_idx < actual_coords and coord_b_idx < actual_coords:
                coords_a = frame_coordinates[coord_a_idx]
                coords_b = frame_coordinates[coord_b_idx]
                
                # 获取2D坐标（如果可用）
                coords_2d_a = frame_2d_coordinates[coord_a_idx] if coord_a_idx < actual_2d_coords else {}
                coords_2d_b = frame_2d_coordinates[coord_b_idx] if coord_b_idx < actual_2d_coords else {}
                
                # 构建 coordinates 字典，包含世界坐标和2D中心像素坐标
                selected_paths_for_pair = pair_records[idx].get("selected_prims", [])
                for prim_path in selected_paths_for_pair:
                    # remove 任务：B 帧被隐藏的物体坐标写 null
                    record["coordinates"][prim_path] = {
                        "world_coordinate_A": coords_a.get(prim_path, None),
                        "world_coordinate_B": None,
                        "pixel_center_A": coords_2d_a.get(prim_path, None),
                        "pixel_center_B": None,
                    }
            else:
                print(f" ⚠️ Coordinates not available for pair {idx}")
            
            # RGB（使用帧文件索引，跳过预热帧）
            if frame_a_idx < len(rgb_frames) and frame_b_idx < len(rgb_frames):
                shutil.copy2(rgb_frames[frame_a_idx], pair_dir / "A_rgb.png")
                shutil.copy2(rgb_frames[frame_b_idx], pair_dir / "B_rgb.png")
            
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
            
            # 语义分割图（默认关闭）
            if semantic_segmentation and frame_a_idx < len(semantic_frames) and frame_b_idx < len(semantic_frames):
                shutil.copy2(semantic_frames[frame_a_idx], pair_dir / "A_semantic_segmentation.png")
                shutil.copy2(semantic_frames[frame_b_idx], pair_dir / "B_semantic_segmentation.png")
            
            # 保存 metadata
            meta_path = pair_dir / "metadata.json"
            meta_path.write_text(json.dumps(record, indent=2))
            
            print(f" Created {record['pair_id']}: A=frame{frame_a_idx}, B=frame{frame_b_idx}")
        
        # 清理 raw 目录中的文件
        for leftover in raw_dir.glob("*.*"):
            leftover.unlink(missing_ok=True)

        writer.detach()
        render_product.destroy()

    # 18. 完成
    rep.orchestrator.wait_until_complete()
    print("\n ========== Done! ==========")
    print(f"Output saved to: {out_dir}")


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
    parser.add_argument("--pair-count", type=int, default=10, help="Number of image pairs to generate")
    parser.add_argument("--warmup-k", type=int, default=3, help="Warmup multiplier (warmup_frames = 3 * k)")
    parser.add_argument("--num-changes", type=int, default=1, help="Number of prims sampled from specific_prims")
    parser.add_argument("--semantic-segmentation", action="store_true", help="Enable semantic segmentation output (default: disabled)")
    parser.add_argument("--width", type=int, default=1024, help="Image width")
    parser.add_argument("--height", type=int, default=768, help="Image height")
    parser.add_argument("--focal-length", type=float, default=None, help="Camera focal length (optional)")
    parser.add_argument("--output-dir", type=str, default="/workspace/output/bedroom1_shelf_remove/1_item", help="Output directory")
    args = parser.parse_args()

    run_kitchen_example(
        pair_count=args.pair_count,
        warmup_k=args.warmup_k,
        num_changes=args.num_changes,
        semantic_segmentation=args.semantic_segmentation,
        resolution=(args.width, args.height),
        focal_length=args.focal_length,
        output_dir=args.output_dir,
    )
    simulation_app.close()
