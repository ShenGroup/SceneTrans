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
    resolution=(1024, 768),
    focal_length: float | None = None,
):
    """
    Headless 版本的厨房场景示例
    只随机化特定的3个瓶子 prims，在柜台平面上
    每次随机化时会随机决定瓶子是否倒下（绕X轴旋转0或90度）
    """
    
    # 2. 打开厨房场景
    usd_path = "/workspace/assets/Lightwheel_oz5iukPxYq_KitchenRoom/KitchenRoom_RSS.usd"
    print(f"[kitchen_headless] Opening stage: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)

    stage = omni.usd.get_context().get_stage()

    if not stage:
        print("[kitchen_headless] Error: Failed to open stage!")
        return

    # 场景保持原样，不修改任何灯光设置
    print("[kitchen_headless] Scene loaded, preserving original lighting")

    # 3. 配置渲染设置来消除时间累积造成的鬼影
    # RTSubframes: 强制渲染器在每帧重新采样
    rep.settings.carb_settings("/omni/replicator/RTSubframes", 80)
    print("[kitchen_headless] RTSubframes set to 80 (to eliminate ghosting)")



    # 4. 配合 on_frame 触发器，让 orchestrator 驱动写盘
    rep.orchestrator.set_capture_on_play(True)

    # 4. 查找场景中已有的 Camera，使用 OmniverseKit_Persp
    desired_camera = "/OmniverseKit_Persp"
    camera_list = []
    
    cam_prim = stage.GetPrimAtPath(desired_camera)
    if cam_prim and cam_prim.IsValid():
        camera_list = [desired_camera]
        print(f"[kitchen_headless] Using camera: {desired_camera}")
    else:
        # 尝试查找场景中其他相机作为备选
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
    out_dir = Path("/workspace/output/kitchen_headless")

    # 6. 只使用指定的3个瓶子 prims 进行随机化
    def find_bottle_prims():
        """只返回用户指定的3个瓶子 prims"""
        specific_prims = [
            # "/root/Kitchen_Bottle005",
            # "/root/Kitchen_Bottle006",
            # "/root/Kitchen_Bottle007",
            "/root/Kitchen_Paper",
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
        为指定的 prims 添加语义标签
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
                print(f"[kitchen_headless] ❌ Prim not found: {path}")
                continue
            
            # 根据 prim 路径自动分配语义标签
            if "Paper" in path:
                prim_name = "paper"
            else:
                prim_name = "bottle"
            
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
    
    # 移除这些 prim 的物理碰撞体（如果有）
    remove_collision_from_bottles(candidate_paths)
    
    # 应用语义标签到所有待随机化的物体
    added = add_semantic_labels(candidate_paths)
    print(f"[kitchen_headless] Semantic labels added: {added} prims labeled")

    # 10. 使用指定的平面进行随机放置
    cabinet_prims = [
        "/root/Plane_01",
        "/root/Plane_02",
    ]
    
    # 验证柜台 prims 是否存在
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

    # 12. 为每个相机渲染
    for cam_path in camera_list:
        cam_slug = cam_path.strip("/").replace("/", "_")
        print(f"\n[kitchen_headless] ========== Rendering camera: {cam_path} ==========")

        render_product = rep.create.render_product(cam_path, resolution)

        cam_out_dir = out_dir / cam_slug
        raw_dir = cam_out_dir / "_raw_frames"
        os.makedirs(raw_dir, exist_ok=True)
        
        # 先运行暖机帧来刷新时间缓存（移除原始位置的残影）
        # warmup 帧数 = 3 * warmup_k，确保是 3 的倍数
        warmup_frames = 3 * warmup_k
        print(f"[kitchen_headless] Running {warmup_frames} warmup frames (k={warmup_k}) with randomization...")
        rep.orchestrator.set_capture_on_play(False)
        
        # 在 warmup 期间，直接用 USD API 移动瓶子到随机位置
        # 这样可以刷掉原始位置的时间缓存
        # 每 3 帧移动一次，共移动 warmup_k 次
        import random as py_random
        for warmup_i in range(warmup_frames):
            # 每隔 3 帧移动一次瓶子，刷新缓存
            if warmup_i % 3 == 0:
                for path in candidate_paths:
                    prim = stage.GetPrimAtPath(path)
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
        
        print(f"[kitchen_headless] Warmup done, original positions flushed from cache.")

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
            semantic_segmentation=True,        # 语义分割图
            colorize_semantic_segmentation=True,  # 彩色语义分割
            instance_segmentation=True,        # 实例分割图
            colorize_instance_segmentation=True,  # 彩色实例分割
            bounding_box_2d_tight=False
        )
        writer.attach(render_product)
        
        # 开启捕获
        rep.orchestrator.set_capture_on_play(True)
        print(f"[kitchen_headless] Writer attached, starting capture...")

        # 创建 pair 计划
        total_frames = pair_count * 2
        pair_records = []
        
        for pair_idx in range(pair_count):
            pair_records.append({
                "scene_type": "kitchen",
                "pair_id": f"pair_{pair_idx:04d}",
                "camera": cam_path,
                "coordinates": {},  # 将在渲染时填充
            })

        print(f"[kitchen_headless] Generating {pair_count} pairs ({total_frames} frames)")

        # 定义获取世界坐标的函数
        def get_world_coordinates(prim_paths):
            """获取指定 prims 的世界坐标"""
            coords = {}
            for path in prim_paths:
                prim = stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    xformable = UsdGeom.Xformable(prim)
                    if xformable:
                        # 获取世界变换矩阵
                        world_transform = xformable.ComputeLocalToWorldTransform(0)
                        # 提取平移部分（世界坐标）
                        translation = world_transform.ExtractTranslation()
                        coords[path] = [translation[0], translation[1], translation[2]]
            return coords

        def get_2d_center_coordinates_from_projection(prim_paths, camera_path, image_width, image_height):
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
                prim = stage.GetPrimAtPath(path)
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

        # 13. 定义随机化函数
        frame_counter = [0]  # 使用列表以便在闭包中修改
        frame_coordinates = []  # 存储每帧的世界坐标
        frame_2d_coordinates = []  # 存储每帧的2D中心像素坐标
        
        # 保存初始变换状态（在第一次随机化前记录）
        initial_transforms = {}
        for path in candidate_paths:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                xformable = UsdGeom.Xformable(prim)
                if xformable:
                    # 保存初始的局部变换矩阵
                    initial_transforms[path] = xformable.GetLocalTransformation()
        print(f"[kitchen_headless] Saved initial transforms for {len(initial_transforms)} prims")
        
        def randomize_bottles():
            """使用 scatter_2d 在柜台上随机散布瓶子，并随机决定是否倒下"""
            frame_counter[0] += 1
            print(f"  [Randomizer] randomize_bottles called (frame {frame_counter[0]})")
            
            # 在随机化之前，重置每个物体的旋转到初始状态，防止累积
            for path in candidate_paths:
                prim = stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    xformable = UsdGeom.Xformable(prim)
                    if xformable:
                        # 重置旋转到 (0, 0, 0)
                        for op in xformable.GetOrderedXformOps():
                            if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                                op.Set(Gf.Vec3f(0, 0, 0))
                                break
            
            # 获取所有物体 prims
            bottle_nodes = []
            for path in candidate_paths:
                prim_node = rep.get.prims(path_pattern=path, prim_types=['Xform'])
                bottle_nodes.append(prim_node)
            
            # 创建物体组
            bottles = rep.create.group(bottle_nodes)
            
            # 应用随机化（pivot 已在函数外部设置一次）
            with bottles:
                # 使用 scatter_2d 随机放置位置
                rep.randomizer.scatter_2d(
                    surface_prims=surface,
                    check_for_collisions=0,  # 0 = 关闭碰撞检测
                )
                
                # 随机旋转
                # X轴：随机 0 或 90 度（模拟瓶子是否倒下）
                # Y轴：保持不变  
                # Z轴：随机 0-360 度（水平方向随机）
                rep.modify.pose(
                    rotation=rep.distribution.combine([
                        rep.distribution.choice([0, 90]),  # X轴：站立或倒下
                        rep.distribution.uniform(0, 0),     # Y轴：保持不变
                        rep.distribution.uniform(0, 360),   # Z轴：水平随机旋转
                    ])
                )
            
            # scatter_2d 完成后，物体已经被放置在表面上
            # 由于我们在函数开头重置了旋转，scatter_2d 每次会重新计算位置
            # 这里不需要额外的 Z 偏移，因为之前的累积问题已经通过重置解决
            # 如果仍然有穿模问题，可以取消下面的注释来添加固定偏移
            # z_offset = 0.8  # 向上偏移
            # for path in candidate_paths:
            #     prim = stage.GetPrimAtPath(path)
            #     if prim and prim.IsValid():
            #         xformable = UsdGeom.Xformable(prim)
            #         if xformable:
            #             for op in xformable.GetOrderedXformOps():
            #                 if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            #                     current_pos = op.Get()
            #                     if current_pos:
            #                         op.Set(Gf.Vec3d(current_pos[0], current_pos[1], current_pos[2] + z_offset))
            #                     break
            
            return bottles.node

        # 14. 在注册 randomizer 之前，为所有物体设置一次 pivot（只执行一次，不会累积）
        for path in candidate_paths:
            prim_node = rep.get.prims(path_pattern=path, prim_types=['Xform'])
            with prim_node:
                rep.modify.pose(pivot=(0, 0, -0.4))
        print(f"[kitchen_headless] Set pivot=(0, 0, -0.4) for {len(candidate_paths)} Xform prims (one time only)")
        
        # 15. 注册随机化器
        rep.randomizer.register(randomize_bottles)

        # 15. 配置触发器（使用 on_frame 确保 Replicator 图正确执行语义标签）
        with rep.trigger.on_frame(num_frames=total_frames + 1):  # +1 因为第一个 step 用于预热
            rep.randomizer.randomize_bottles()
        
        # 16. 预热步骤：运行一帧让触发器开始工作，但不记录这帧的坐标
        # 这帧会被捕获但我们稍后会丢弃它（通过只使用后 total_frames 帧）
        print(f"[kitchen_headless] Running pre-capture step to initialize trigger...")
        rep.orchestrator.step()
        
        # 17. 逐帧运行渲染并记录坐标
        print(f"[kitchen_headless] Running orchestrator with {total_frames} frames...")
        
        for frame_idx in range(total_frames):
            # 执行一帧渲染（触发器会自动调用 randomize_bottles）
            rep.orchestrator.step()
            
            # 渲染完成后记录世界坐标
            current_coords = get_world_coordinates(candidate_paths)
            frame_coordinates.append(current_coords)
            
            # 使用相机投影计算2D像素坐标（基于 simulation，不依赖分割）
            current_2d_coords = get_2d_center_coordinates_from_projection(
                candidate_paths, cam_path, resolution[0], resolution[1]
            )
            frame_2d_coordinates.append(current_2d_coords)
            print(f"[kitchen_headless] Frame {frame_idx}: recorded world coords for {len(current_coords)} prims, 2D coords for {sum(1 for v in current_2d_coords.values() if v is not None)} prims")
        
        print(f"[kitchen_headless] Rendering complete. Recorded {len(frame_coordinates)} world coord frames, {len(frame_2d_coordinates)} 2D coord frames.")
        
        # 等待所有写入操作完成
        rep.orchestrator.wait_until_complete()
        print(f"[kitchen_headless] All frames written to disk.")

        # 17. 重新整理输出（包含 RGB、深度图、分割图）
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
        
        # 验证有足够的帧来创建 pair 对
        # 注意：第一帧是预热帧，需要跳过，所以实际可用帧数 = 总帧数 - 1
        min_frames_needed = pair_count * 2
        actual_rgb = len(rgb_frames)
        usable_rgb = actual_rgb - 1  # 跳过第一帧（预热帧）
        actual_coords = len(frame_coordinates)
        
        print(f"[kitchen_headless] Usable frames: {usable_rgb} (skipping first pre-capture frame)")
        
        if usable_rgb < min_frames_needed:
            print(f"[kitchen_headless] ⚠️ Warning: Need {min_frames_needed} usable frames but found {usable_rgb}")
            # 尽可能创建能创建的 pair 数量
            pair_count = min(pair_count, usable_rgb // 2)
        
        if actual_coords < min_frames_needed:
            print(f"[kitchen_headless] ⚠️ Warning: Need {min_frames_needed} coordinate snapshots but found {actual_coords}")
        
        print(f"[kitchen_headless] Will create {pair_count} pairs")
        
        for idx in range(pair_count):
            record = pair_records[idx]
            pair_dir = cam_out_dir / record["pair_id"]
            pair_dir.mkdir(parents=True, exist_ok=True)
            
            # 坐标索引：使用原始索引（0-based），因为坐标数组只在渲染循环中记录
            coord_a_idx = idx * 2
            coord_b_idx = idx * 2 + 1
            
            # 帧文件索引：跳过第一帧（预热帧），所以 +1
            frame_a_idx = idx * 2 + 1  # 从索引 1 开始（跳过索引 0 的预热帧）
            frame_b_idx = idx * 2 + 2
            
            # 填充坐标信息（使用坐标索引）
            actual_2d_coords = len(frame_2d_coordinates)
            if coord_a_idx < actual_coords and coord_b_idx < actual_coords:
                coords_a = frame_coordinates[coord_a_idx]
                coords_b = frame_coordinates[coord_b_idx]
                
                # 获取2D坐标（如果可用）
                coords_2d_a = frame_2d_coordinates[coord_a_idx] if coord_a_idx < actual_2d_coords else {}
                coords_2d_b = frame_2d_coordinates[coord_b_idx] if coord_b_idx < actual_2d_coords else {}
                
                # 构建 coordinates 字典，包含世界坐标和2D中心像素坐标
                for prim_path in candidate_paths:
                    record["coordinates"][prim_path] = {
                        "world_coordinate_A": coords_a.get(prim_path, None),
                        "world_coordinate_B": coords_b.get(prim_path, None),
                        "pixel_center_A": coords_2d_a.get(prim_path, None),
                        "pixel_center_B": coords_2d_b.get(prim_path, None),
                    }
            else:
                print(f"[kitchen_headless] ⚠️ Coordinates not available for pair {idx}")
            
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
            
            # 语义分割图
            if frame_a_idx < len(semantic_frames) and frame_b_idx < len(semantic_frames):
                shutil.copy2(semantic_frames[frame_a_idx], pair_dir / "A_semantic_segmentation.png")
                shutil.copy2(semantic_frames[frame_b_idx], pair_dir / "B_semantic_segmentation.png")
            
            # 保存 metadata
            meta_path = pair_dir / "metadata.json"
            meta_path.write_text(json.dumps(record, indent=2))
            
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
    parser.add_argument("--pair-count", type=int, default=4, help="Number of image pairs to generate")
    parser.add_argument("--warmup-k", type=int, default=3, help="Warmup multiplier (warmup_frames = 3 * k)")
    parser.add_argument("--width", type=int, default=1024, help="Image width")
    parser.add_argument("--height", type=int, default=768, help="Image height")
    parser.add_argument("--focal-length", type=float, default=None, help="Camera focal length (optional)")
    args = parser.parse_args()

    run_kitchen_example(
        pair_count=args.pair_count,
        warmup_k=args.warmup_k,
        resolution=(args.width, args.height),
        focal_length=args.focal_length,
    )
    simulation_app.close()
