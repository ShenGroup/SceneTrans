# Script Editor 版本 - 基于 hospital_test_editor.py 改编
# 适合在 Isaac Sim GUI 中运行，支持 timeline 播放
# 特性：随机化3个瓶子 prims，在两个柜台平面上放置
# pair 差异通过不同的随机化位置和瓶子倒伏状态体现

import os
import random
import json
from pathlib import Path

import omni.replicator.core as rep
import omni.usd
from pxr import Sdf, UsdGeom, UsdPhysics
import omni.physx
import omni.timeline


def run_kitchen_example_editor(
    pair_count: int = 4,
    warmup_frames: int = 10,
    resolution=(1024, 768),
    diff_mode: str = "random",
    focal_length: float | None = None,
):
    """
    Script Editor 版本的厨房场景示例
    使用 timeline 播放模式，可以在 GUI 中逐帧观看
    只随机化特定的3个瓶子 prims，在柜台平面上
    每次随机化时会随机决定瓶子是否倒下（绕Y轴旋转90度）
    """
    
    # 1. 获取当前已打开的stage（假设用户已经打开了厨房场景）
    stage = omni.usd.get_context().get_stage()
    
    if not stage:
        print("[kitchen_test_editor] Error: No stage is currently open!")
        print("[kitchen_test_editor] Please open a kitchen scene first.")
        return
    
    print(f"[kitchen_test_editor] Using current stage: {stage.GetRootLayer().identifier}")

    # 场景保持原样，不修改任何灯光设置
    print("[kitchen_test_editor] Scene loaded, preserving original lighting")

    # 2. 使用指定的相机
    desired_camera = "/OmniverseKit_Persp"
    camera_list = [desired_camera]
    
    # 验证相机是否存在
    cam_prim = stage.GetPrimAtPath(desired_camera)
    if cam_prim and cam_prim.IsValid():
        print(f"[kitchen_test_editor] Using camera: {desired_camera}")
    else:
        print(f"[kitchen_test_editor] Warning: Specified camera not found: {desired_camera}")
        # 尝试查找场景中其他相机作为备选
        for prim in stage.Traverse():
            if prim.GetTypeName() == "Camera":
                camera_list = [prim.GetPath().pathString]
                print(f"[kitchen_test_editor] Fallback to camera: {camera_list[0]}")
                break

    if not camera_list:
        # 如果场景里没有相机，就创建一个简单相机
        print("[kitchen_test_editor] No camera found, creating a default one.")
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
                print(f"[kitchen_test_editor] Set focalLength={focal_length} for {cam_path}")

    # 3. BasicWriter：输出到 /workspace/output/kitchen_test_editor
    writer = rep.writers.get("BasicWriter")
    out_dir = Path("/workspace/output/kitchen_test_editor")

    def ensure_translate_op(path: str):
        prim = stage.GetPrimAtPath(path)
        if not prim:
            return None
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            return None
        # 如果是 instance，先取消 instanceable，再创建 translate op
        if prim.IsInstanceable():
            prim.SetInstanceable(False)
        translate_op = None
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
            translate_op.Set((0.0, 0.0, 0.0))
        val = translate_op.Get()
        if val is None:
            val = (0.0, 0.0, 0.0)
        return tuple(val)

    # 4. 只使用指定的3个瓶子 prims 进行随机化
    def find_bottle_prims():
        """只返回用户指定的3个瓶子 prims"""
        specific_prims = [
            "/root/Kitchen_Bottle005",
            "/root/Kitchen_Bottle006",
            "/root/Kitchen_Bottle007",
        ]
        
        # 验证这些 prims 是否存在
        valid_prims = []
        for path in specific_prims:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                valid_prims.append(path)
                print(f"[kitchen_test_editor] Found specified prim: {path}")
            else:
                print(f"[kitchen_test_editor] Warning: Specified prim not found: {path}")
        
        return valid_prims

    # 5. 为物体添加碰撞体（只添加 CollisionAPI，用于 scatter_2d 碰撞检测）
    def enable_collision_on_bottles(bottle_paths):
        """只为物体添加 CollisionAPI，用于 scatter_2d 的碰撞检测"""
        for path in bottle_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                continue
            
            mesh_count = 0

            # 取消 instanceable 以便添加碰撞体
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
            
            # 为 Xform 本身添加 CollisionAPI
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
            
            # 为所有子 Mesh 添加 CollisionAPI
            def add_collision_to_meshes(parent_prim, depth=0):
                nonlocal mesh_count
                if depth > 5:
                    return
                for child in parent_prim.GetChildren():
                    child_type = child.GetTypeName()
                    
                    if child_type == "Mesh":
                        mesh_count += 1
                        if not child.HasAPI(UsdPhysics.CollisionAPI):
                            UsdPhysics.CollisionAPI.Apply(child)
                    else:
                        add_collision_to_meshes(child, depth + 1)
            
            add_collision_to_meshes(prim)
            print(f"[kitchen_test_editor] Collision ready: {path} (mesh colliders: {mesh_count})")

    # 6. 查找并验证 prims
    candidate_paths = find_bottle_prims()
    
    if not candidate_paths:
        print("[kitchen_test_editor] No specified prims were found; cannot proceed.")
        return
    
    print(f"[kitchen_test_editor] Will randomize {len(candidate_paths)} prims")
    
    # 为这些 prim 添加碰撞体
    enable_collision_on_bottles(candidate_paths)
    
    # 7. 为 randomize 的物体添加语义标签（用于 instance/semantic segmentation）
    def add_semantic_labels(prim_paths):
        """
        为指定的 prims 添加语义标签
        同时使用两种方式确保兼容性：
        1. 直接使用 USD primvars 写入语义属性（立即生效）
        2. 使用 rep.modify.semantics（用于 Replicator graph）
        """
        print("[kitchen_test_editor] Adding semantic labels using dual approach...")
        
        added_count = 0
        
        for path in prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                print(f"[kitchen_test_editor] ❌ Prim not found: {path}")
                continue
            
            # 统一使用 "bottle" 作为语义标签测试
            prim_name = "bottle"  # 测试：统一标签
            
            try:
                # 方法 1: 直接使用 USD primvars 写入语义属性
                # 这是 Replicator 读取语义标签的标准格式
                # 参考 Omniverse Replicator 源码中的语义属性格式
                semantic_type = "Semantics"
                primvar_path = f"primvars:semantics:{semantic_type}"
                
                # 创建语义标签属性
                type_attr = prim.CreateAttribute(
                    f"{primvar_path}:params:semanticType", 
                    Sdf.ValueTypeNames.String
                )
                type_attr.Set("class")
                
                data_attr = prim.CreateAttribute(
                    f"{primvar_path}:params:semanticData", 
                    Sdf.ValueTypeNames.String
                )
                data_attr.Set(prim_name)
                
                print(f"[kitchen_test_editor] ✓ USD primvars created: {path}")
                print(f"[kitchen_test_editor]   {primvar_path}:params:semanticType = 'class'")
                print(f"[kitchen_test_editor]   {primvar_path}:params:semanticData = '{prim_name}'")
                
                # 方法 2: 同时使用 rep.modify.semantics（确保 Replicator graph 也能识别）
                prim_group = rep.get.prims(path_pattern=path)
                with prim_group:
                    rep.modify.semantics([("class", prim_name)])
                
                added_count += 1
                print(f"[kitchen_test_editor] ✓ rep.modify.semantics added: {path} -> '{prim_name}'")
                
            except Exception as e:
                print(f"[kitchen_test_editor] ❌ Failed to add label to {path}: {e}")
                import traceback
                traceback.print_exc()
        
        # 验证：检查语义属性是否被正确添加
        print("[kitchen_test_editor] Verifying semantic labels after creation...")
        for path in prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                continue
            found_semantic = False
            for attr in prim.GetAttributes():
                attr_name = attr.GetName()
                if "semantic" in attr_name.lower():
                    attr_value = attr.Get()
                    print(f"[kitchen_test_editor]   VERIFIED: {path}: {attr_name} = {attr_value}")
                    found_semantic = True
            if not found_semantic:
                print(f"[kitchen_test_editor]   {path}: ⚠️ No semantic attributes found after creation!")
        
        return added_count
    
    # 应用语义标签到所有待随机化的物体
    added = add_semantic_labels(candidate_paths)
    print(f"[kitchen_test_editor] Semantic labels added: {added} prims labeled")
    
    if added == 0:
        print("[kitchen_test_editor] ⚠️ WARNING: No semantic labels were added! Segmentation will be empty.")
    
    # 7. 辅助函数：列出场景中所有可能的柜台 prims
    def list_cabinet_prims():
        """列出场景中所有可能是柜台的 prims"""
        cabinet_candidates = []
        print("\n[kitchen_test_editor] Searching for cabinet prims in the scene...")
        
        for prim in stage.Traverse():
            path_str = prim.GetPath().pathString
            name_lower = prim.GetName().lower()
            
            # 跳过 Looks/Materials 等非几何节点
            if "/Looks" in path_str or prim.GetTypeName() == "Material":
                continue
            
            # 查找包含 cabinet/counter/table 关键词的 prim
            if any(keyword in name_lower for keyword in ["cabinet", "counter", "table", "top"]):
                prim_type = prim.GetTypeName()
                cabinet_candidates.append((path_str, prim_type))
                print(f"  - {path_str} (Type: {prim_type})")
        
        return cabinet_candidates
    
    # 8. 使用指定的平面进行随机放置
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
            print(f"[kitchen_test_editor] ✓ Found cabinet prim: {cabinet_prim}")
        else:
            print(f"[kitchen_test_editor] ❌ Warning: Cabinet prim not found: {cabinet_prim}")
    
    if not valid_cabinet_prims:
        print(f"\n[kitchen_test_editor] ❌ Error: No valid cabinet prims found!")
        print(f"[kitchen_test_editor] Searching for available cabinet prims...\n")
        
        # 列出所有可能的柜台
        available_cabinets = list_cabinet_prims()
        
        if available_cabinets:
            print(f"\n[kitchen_test_editor] Found {len(available_cabinets)} potential cabinet prims.")
            print(f"[kitchen_test_editor] Please update the 'cabinet_prims' variable to use these paths:")
            for path, prim_type in available_cabinets:
                print(f"  \"{path}\"  # Type: {prim_type}")
            print(f"\n[kitchen_test_editor] Then run the script again.\n")
        else:
            print(f"[kitchen_test_editor] No cabinet prims found in the scene.")
        
        return
    
    # 9. 查找实际的 Mesh prim（scatter_2d 需要直接的 Mesh，不能是容器）
    def find_actual_mesh(prim_path, depth=0, max_depth=3):
        """
        递归查找 prim 下实际的 Mesh 子节点
        因为 scatter_2d 需要直接的 Mesh prim，不能是包含 Mesh 的容器
        
        Args:
            prim_path: 要搜索的 prim 路径
            depth: 当前递归深度
            max_depth: 最大递归深度
        """
        if depth > max_depth:
            return None
            
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return None
        
        indent = "  " * depth
        
        # 如果本身就是 Mesh，直接返回
        if prim.GetTypeName() == "Mesh":
            print(f"[kitchen_test_editor] {indent}✓ Found Mesh: {prim_path}")
            return prim_path
        
        # 否则递归搜索子节点中的 Mesh
        if depth == 0:
            print(f"[kitchen_test_editor] Searching for Mesh children under: {prim_path}")
        
        for child in prim.GetChildren():
            child_path = child.GetPath().pathString
            child_type = child.GetTypeName()
            
            # 跳过 Looks/Scope 等非几何节点
            if "Looks" in child.GetName() or child_type == "Scope":
                print(f"[kitchen_test_editor] {indent}  Skipping: {child.GetName()} (Type: {child_type})")
                continue
            
            print(f"[kitchen_test_editor] {indent}  Checking: {child.GetName()} (Type: {child_type})")
            
            # 如果是 Mesh，找到了！
            if child_type == "Mesh":
                print(f"[kitchen_test_editor] {indent}  ✓ Found Mesh: {child_path}")
                return child_path
            
            # 如果是 Xform 或其他容器，递归搜索
            if child_type in ["Xform", "Scope", ""]:
                result = find_actual_mesh(child_path, depth + 1, max_depth)
                if result:
                    return result
        
        if depth == 0:
            print(f"[kitchen_test_editor] Warning: No Mesh found under {prim_path}")
        return None
    
    # 查找所有柜台的实际 mesh prims
    actual_cabinet_meshes = []
    for cabinet_prim in valid_cabinet_prims:
        actual_mesh = find_actual_mesh(cabinet_prim)
        if actual_mesh:
            actual_cabinet_meshes.append(actual_mesh)
            print(f"[kitchen_test_editor] Using mesh for scatter_2d: {actual_mesh}")
        else:
            # 如果找不到子 Mesh，尝试直接使用该 prim（可能本身就是 Mesh）
            print(f"[kitchen_test_editor] Will try using prim directly: {cabinet_prim}")
            actual_cabinet_meshes.append(cabinet_prim)
    
    if not actual_cabinet_meshes:
        print("[kitchen_test_editor] ❌ Error: No valid meshes found for scatter_2d")
        return
    
    # 获取柜台 prims 作为 scatter_2d 的表面
    # 注意：path_pattern 只接受字符串，不接受列表
    # 所以需要为每个 mesh 分别获取 prim 节点，然后组合
    surface_nodes = []
    for mesh_path in actual_cabinet_meshes:
        prim_node = rep.get.prims(path_pattern=mesh_path)
        if prim_node:
            surface_nodes.append(prim_node)
            print(f"[kitchen_test_editor] Added surface: {mesh_path}")
    
    if not surface_nodes:
        print("[kitchen_test_editor] ❌ Error: No valid surface found for scatter_2d")
        return
    
    # 如果有多个表面，创建一个组；否则直接使用单个表面
    if len(surface_nodes) == 1:
        surface = surface_nodes[0]
    else:
        surface = rep.create.group(surface_nodes)
        print(f"[kitchen_test_editor] Created surface group with {len(surface_nodes)} meshes")

    # 10. 为每个相机分别渲染
    for cam_path in camera_list:
        cam_slug = cam_path.strip("/").replace("/", "_")
        print(f"[kitchen_test_editor] Rendering camera: {cam_path}")

        # 基于该相机 prim 创建 render product
        render_product = rep.create.render_product(cam_path, resolution)

        cam_out_dir = out_dir / cam_slug
        raw_dir = cam_out_dir / "_raw_frames"
        os.makedirs(raw_dir, exist_ok=True)
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

        # 创建 pair 计划
        total_frames = pair_count * 2
        pair_records = []
        
        for pair_idx in range(pair_count):
            pair_records.append(
                {
                    "pair_id": f"pair_{pair_idx:04d}",
                    "camera": cam_path,
                    "difference": "different_randomization",
                    "description": "A and B frames have different random placements and bottle orientations (standing or fallen)",
                }
            )

        print(f"[kitchen_test_editor] Generating {pair_count} pairs ({total_frames} frames)")

        # 定义随机化函数
        def randomize_bottles():
            """使用 scatter_2d 在柜台上随机散布瓶子，并随机决定是否倒下"""
            # 获取所有物体 prims
            bottle_nodes = []
            for path in candidate_paths:
                prim_node = rep.get.prims(path_pattern=path, prim_types=['Xform'])
                bottle_nodes.append(prim_node)
            
            # 创建物体组
            bottles = rep.create.group(bottle_nodes)
            
            # 应用随机化：关闭碰撞检测（避免因无效几何体导致崩溃）
            with bottles:
                rep.randomizer.scatter_2d(
                    surface_prims=surface,
                    check_for_collisions=0,  # 0 = 关闭碰撞检测（避免崩溃）
                )
                
                # 随机旋转：
                # X轴：随机 0 或 90 度（模拟瓶子是否倒下）
                # Z轴：随机 0-360 度（水平方向随机）
                # 使用 choice 来实现 0 或 90 的随机选择
                rep.modify.pose(
                    rotation=rep.distribution.combine([
                        rep.distribution.choice([0, 90]),  # X轴：站立或倒下
                        rep.distribution.uniform(0, 0),     # Y轴：保持不变
                        rep.distribution.uniform(0, 360),   # Z轴：水平随机旋转
                    ])
                )
            
            return bottles.node

        # 11. 注册随机化器
        rep.randomizer.register(randomize_bottles)

        # 12. 配置触发器 - 确保每帧都调用随机化
        print(f"\n[kitchen_test_editor] ========================================")
        print(f"[kitchen_test_editor] Configuring trigger for {total_frames} frames")
        print(f"[kitchen_test_editor] ========================================\n")
        
        with rep.trigger.on_frame(num_frames=total_frames):
            rep.randomizer.randomize_bottles()
        
        # 13. 重要：启用捕获模式
        rep.orchestrator.set_capture_on_play(True)
        
        # 14. 设置 timeline 的帧范围（Editor 模式特有）
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_start_time(0)
        timeline.set_end_time(total_frames / 24.0)  # 假设 24 fps
        timeline.set_current_time(0)
        
        # 保存 pair metadata
        metadata_file = cam_out_dir / "pair_metadata.json"
        metadata_file.parent.mkdir(parents=True, exist_ok=True)
        metadata_file.write_text(json.dumps(pair_records, indent=2))

    # 最终提示
    print(f"\n" + "=" * 60)
    print(f"SETUP COMPLETE!")
    print(f"=" * 60)
    print(f"\n1. 在 Timeline 中按 PLAY 按钮")
    print(f"2. 等待渲染完成后运行: reorganize_pairs_from_raw()")
    print(f"\n" + "=" * 60)



def check_randomization_results():
    """
    检查随机化结果的函数
    在渲染完成后调用此函数来验证是否真正随机化了
    """
    global _randomization_tracker
    
    if '_randomization_tracker' not in globals():
        print("[CHECK] Error: No randomization tracker found. Please run the main script first.")
        return
    
    tracker = _randomization_tracker
    call_count = tracker["call_count"]
    frame_positions = tracker["frame_positions"]
    
    print(f"\n" + "=" * 80)
    print(f"RANDOMIZATION VERIFICATION REPORT")
    print(f"=" * 80)
    print(f"\n总调用次数: {call_count}")
    print(f"记录的帧数: {len(frame_positions)}")
    
    if call_count == 0:
        print("\n❌ 错误：随机化器从未被调用！")
        print("   可能原因：")
        print("   1. Timeline 没有播放")
        print("   2. rep.trigger.on_frame 没有正确触发")
        return
    
    if len(frame_positions) < 2:
        print("\n❌ 错误：记录的帧数太少，无法验证随机化")
        return
    
    # 检查每个物体在不同帧之间的位置是否变化
    print(f"\n检查物体位置变化:")
    
    # 获取第一帧和第二帧的位置
    frame_keys = sorted(frame_positions.keys())
    if len(frame_keys) < 2:
        print("❌ 只有一帧数据，无法比较")
        return
    
    frame1 = frame_keys[0]
    frame2 = frame_keys[1]
    
    pos_frame1 = frame_positions[frame1]
    pos_frame2 = frame_positions[frame2]
    
    all_same = True
    for path in pos_frame1:
        if path in pos_frame2:
            pos1 = pos_frame1[path]
            pos2 = pos_frame2[path]
            
            # 计算位置差异
            diff = ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2 + (pos1[2] - pos2[2])**2)**0.5
            
            if diff > 0.1:  # 如果位置变化大于 0.1 单位
                print(f"  ✅ {path}: 位置有变化 (移动了 {diff:.2f} 单位)")
                all_same = False
            else:
                print(f"  ⚠️  {path}: 位置基本相同 (移动了 {diff:.2f} 单位)")
    
    if all_same:
        print(f"\n❌ 结论：所有物体在不同帧之间位置基本相同")
        print(f"   scatter_2d 可能在 Editor 模式下不能每帧重新随机化")
        print(f"\n建议：检查 replicator 配置")
    else:
        print(f"\n✅ 结论：物体位置在不同帧之间有变化，随机化正常工作！")
    
    print(f"\n所有帧的详细位置记录:")
    for frame_num in sorted(frame_positions.keys())[:5]:  # 只显示前5帧
        print(f"\nFrame #{frame_num}:")
        for path, pos in frame_positions[frame_num].items():
            print(f"  {path}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
    
    if len(frame_positions) > 5:
        print(f"\n... (还有 {len(frame_positions) - 5} 帧未显示)")
    
    print(f"\n" + "=" * 80)


def depth_npy_to_png(npy_path, png_path):
    """
    将深度图的 npy 文件转换为可视化的 png 图像
    
    Args:
        npy_path: 输入的 npy 文件路径
        png_path: 输出的 png 文件路径
    """
    import numpy as np
    from PIL import Image
    
    try:
        # 加载深度数据
        depth_data = np.load(npy_path)
        
        # 处理可能的多通道数据（有时深度图可能有额外的维度）
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
        
        # 归一化到 0-255 范围（近处亮，远处暗）
        if max_val > min_val:
            normalized = (depth_data - min_val) / (max_val - min_val)
        else:
            normalized = np.zeros_like(depth_data)
        
        # 将无效值设为 0（黑色）
        normalized[~valid_mask] = 0
        
        # 反转：近处亮（白），远处暗（黑）- 更直观的可视化
        normalized = 1.0 - normalized
        
        # 转换为 8-bit 图像
        depth_uint8 = (normalized * 255).astype(np.uint8)
        
        # 保存为 png
        img = Image.fromarray(depth_uint8)  # 灰度图
        img.save(png_path)
        return True
        
    except Exception as e:
        print(f"[depth_npy_to_png] Error converting {npy_path}: {e}")
        return False


def reorganize_pairs_from_raw(output_base_dir: str = "/workspace/output/kitchen_test_editor"):
    """
    后处理函数：将 _raw_frames 中的图像重新组织成 pair 结构
    
    这个函数应该在 timeline 播放完成、所有帧都渲染完成后运行。
    它会读取每个相机的 _raw_frames 目录和 pair_metadata.json，
    然后创建 pair_xxxx 目录结构。
    
    深度图会同时保留 npy 格式和生成 png 可视化图像。
    
    Args:
        output_base_dir: 输出根目录路径
    """
    import json
    import shutil
    from pathlib import Path
    
    base_dir = Path(output_base_dir)
    
    if not base_dir.exists():
        print(f"[reorganize_pairs] Error: Output directory not found: {base_dir}")
        return
    
    # 遍历所有相机目录
    for cam_dir in base_dir.iterdir():
        if not cam_dir.is_dir():
            continue
        
        raw_dir = cam_dir / "_raw_frames"
        metadata_file = cam_dir / "pair_metadata.json"
        
        if not raw_dir.exists() or not metadata_file.exists():
            print(f"[reorganize_pairs] Skipping {cam_dir.name}: missing _raw_frames or pair_metadata.json")
            continue
        
        # 读取 pair metadata
        try:
            with open(metadata_file, 'r') as f:
                pair_records = json.load(f)
        except Exception as e:
            print(f"[reorganize_pairs] Error reading {metadata_file}: {e}")
            continue
        
        # 获取所有 raw frames - 支持 rgb, depth, instance_segmentation
        # RGB 图像
        rgb_frames = sorted(raw_dir.glob("rgb_*.png"))
        # 深度图 (distance_to_camera)
        depth_frames = sorted(raw_dir.glob("distance_to_camera_*.npy"))
        # 如果深度图是 png 格式
        if not depth_frames:
            depth_frames = sorted(raw_dir.glob("distance_to_camera_*.png"))
        # 实例分割图
        instance_frames = sorted(raw_dir.glob("instance_segmentation_*.png"))
        # 语义分割图
        semantic_frames = sorted(raw_dir.glob("semantic_segmentation_*.png"))
        
        total_expected = len(pair_records) * 2
        
        if len(rgb_frames) < total_expected:
            print(f"[reorganize_pairs] Warning: {cam_dir.name} has {len(rgb_frames)} RGB frames, expected {total_expected}")
            continue
        
        print(f"[reorganize_pairs] Processing {cam_dir.name}: {len(pair_records)} pairs")
        print(f"[reorganize_pairs]   RGB frames: {len(rgb_frames)}")
        print(f"[reorganize_pairs]   Depth frames: {len(depth_frames)}")
        print(f"[reorganize_pairs]   Instance segmentation frames: {len(instance_frames)}")
        print(f"[reorganize_pairs]   Semantic segmentation frames: {len(semantic_frames)}")
        
        # 为每个 pair 创建目录
        for idx, record in enumerate(pair_records):
            pair_dir = cam_dir / record["pair_id"]
            pair_dir.mkdir(parents=True, exist_ok=True)
            
            # 获取对应的帧索引
            frame_a_idx = idx * 2
            frame_b_idx = idx * 2 + 1
            
            # 处理 RGB 图像
            if frame_a_idx < len(rgb_frames) and frame_b_idx < len(rgb_frames):
                try:
                    rgb_frames[frame_a_idx].rename(pair_dir / "A_rgb.png")
                    rgb_frames[frame_b_idx].rename(pair_dir / "B_rgb.png")
                except Exception as e:
                    print(f"[reorganize_pairs] Rename failed, using copy: {e}")
                    shutil.copy2(rgb_frames[frame_a_idx], pair_dir / "A_rgb.png")
                    shutil.copy2(rgb_frames[frame_b_idx], pair_dir / "B_rgb.png")
            else:
                print(f"[reorganize_pairs] Warning: Not enough RGB frames for {record['pair_id']}")
            
            # 处理深度图 - 同时保留 npy 和生成 png 可视化
            if frame_a_idx < len(depth_frames) and frame_b_idx < len(depth_frames):
                depth_ext = depth_frames[frame_a_idx].suffix
                
                # 复制/移动 npy 文件
                try:
                    # 使用 copy 而不是 rename，以便后续生成 png
                    shutil.copy2(depth_frames[frame_a_idx], pair_dir / f"A_depth{depth_ext}")
                    shutil.copy2(depth_frames[frame_b_idx], pair_dir / f"B_depth{depth_ext}")
                except Exception as e:
                    print(f"[reorganize_pairs] Copy failed for depth: {e}")
                
                # 如果是 npy 格式，额外生成 png 可视化
                if depth_ext == ".npy":
                    depth_npy_to_png(
                        depth_frames[frame_a_idx], 
                        pair_dir / "A_depth.png"
                    )
                    depth_npy_to_png(
                        depth_frames[frame_b_idx], 
                        pair_dir / "B_depth.png"
                    )
                    # 删除原始文件（已复制到 pair_dir）
                    depth_frames[frame_a_idx].unlink(missing_ok=True)
                    depth_frames[frame_b_idx].unlink(missing_ok=True)
            
            # 处理实例分割图
            if frame_a_idx < len(instance_frames) and frame_b_idx < len(instance_frames):
                try:
                    instance_frames[frame_a_idx].rename(pair_dir / "A_instance_segmentation.png")
                    instance_frames[frame_b_idx].rename(pair_dir / "B_instance_segmentation.png")
                except Exception as e:
                    print(f"[reorganize_pairs] Rename failed for instance seg, using copy: {e}")
                    shutil.copy2(instance_frames[frame_a_idx], pair_dir / "A_instance_segmentation.png")
                    shutil.copy2(instance_frames[frame_b_idx], pair_dir / "B_instance_segmentation.png")
            
            # 处理语义分割图
            if frame_a_idx < len(semantic_frames) and frame_b_idx < len(semantic_frames):
                try:
                    semantic_frames[frame_a_idx].rename(pair_dir / "A_semantic_segmentation.png")
                    semantic_frames[frame_b_idx].rename(pair_dir / "B_semantic_segmentation.png")
                except Exception as e:
                    print(f"[reorganize_pairs] Rename failed for semantic seg, using copy: {e}")
                    shutil.copy2(semantic_frames[frame_a_idx], pair_dir / "A_semantic_segmentation.png")
                    shutil.copy2(semantic_frames[frame_b_idx], pair_dir / "B_semantic_segmentation.png")
            
            # 保存 pair 的 metadata
            meta_path = pair_dir / "metadata.json"
            with open(meta_path, 'w') as f:
                json.dump(record, f, indent=2)
            
            print(f"[reorganize_pairs] Created {record['pair_id']}: rgb, depth, instance_seg, semantic_seg, metadata")
        
        # 清理剩余的 raw frames（如果有）
        remaining_files = list(raw_dir.glob("*.png")) + list(raw_dir.glob("*.npy"))
        if remaining_files:
            print(f"[reorganize_pairs] Cleaning up {len(remaining_files)} remaining raw files")
            for f in remaining_files:
                f.unlink(missing_ok=True)
        
        print(f"[reorganize_pairs] Completed {cam_dir.name}: {len(pair_records)} pairs organized")
    
    print(f"\n[reorganize_pairs] All done! Pairs are organized in: {base_dir}")


# 在 Script Editor 中运行时，直接调用函数
if __name__ == "__main__":
    print("=" * 80)
    print("Kitchen Test - Script Editor Mode")
    print("随机化3个瓶子 prims，在柜台平面上")
    print("每次随机化会随机决定瓶子是站立还是倒下（绕X轴旋转0或90度）")
    print("=" * 80)
    print("\n使用方法:")
    print("1. 运行此脚本设置场景和随机化器")
    print("2. 在 Timeline 中按 PLAY 按钮，等待所有帧渲染完成")
    print("3. 渲染完成后，在 Script Editor 中运行:")
    print("   check_randomization_results()")
    print("   这将验证随机化是否正常工作\n")
    print("=" * 80)
    
    # 可以在这里修改参数
    run_kitchen_example_editor(
        pair_count=4,           # 生成 4 对图像 = 8 帧
        warmup_frames=10,       # 暖机帧数（实际上 editor 模式不使用）
        resolution=(1024, 768), # 图片分辨率 (宽, 高) - 可修改为 (1920, 1080) 等
        diff_mode="random",     # 差异模式（已被修改为始终使用位置差异）
        focal_length=None,      # 焦距（可选）
    )

# ============================================================================
# 验证和后处理函数 - 在 Timeline 播放完成后，可以单独运行以下命令
# ============================================================================
# check_randomization_results()  # 验证随机化是否工作
# reorganize_pairs_from_raw()    # 整理 raw frames 为 pairs
