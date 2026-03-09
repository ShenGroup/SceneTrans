# Script Editor 版本 - 基于 hospital_test.py 改编
# 适合在 Isaac Sim GUI 中运行，支持 timeline 播放
# 特性：只随机化4个指定的 prims，只在 /Root/Geo_Floor_Costum4 平面上放置
# pair 差异通过不同的随机化位置体现

import os
import random
import json
from pathlib import Path

import omni.replicator.core as rep
import omni.usd
from pxr import Sdf, UsdGeom, UsdPhysics
import omni.physx
import omni.timeline


def run_hospital_example_editor(
    pair_count: int = 4,
    warmup_frames: int = 10,
    resolution=(1024, 768),
    diff_mode: str = "random",
    focal_length: float | None = None,
):
    """
    Script Editor 版本的医院场景示例
    使用 timeline 播放模式，可以在 GUI 中逐帧观看
    只随机化特定的4个 prims，只在指定的地面平面上
    """
    
    # 1. 打开医院场景
    usd_path = "/workspace/assets/Hospital/hospital.usd"
    print(f"[hospital_test_editor] Opening stage: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)

    stage = omni.usd.get_context().get_stage()

    # 场景保持原样，不修改任何灯光设置
    print("[hospital_test_editor] Scene loaded, preserving original lighting")

    # 2. 查找场景中已有的 Camera，只保留一个主相机
    # 如果你想用其他相机，可以修改这里
    desired_cameras = {"/Root/Camera_01"}  # 只使用这一个相机
    camera_list = []
    for prim in stage.Traverse():
        if prim.GetTypeName() == "Camera":
            camera_list.append(prim.GetPath().pathString)
    camera_list = [c for c in camera_list if c in desired_cameras]
    print(f"[hospital_test_editor] Cameras found: {camera_list}")

    if not camera_list:
        # 如果场景里没有相机，就创建一个简单相机
        print("[hospital_test_editor] No camera found, creating a default one.")
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
                print(f"[hospital_test_editor] Set focalLength={focal_length} for {cam_path}")

    # 3. BasicWriter：输出到 /workspace/output/hospital_test_editor
    writer = rep.writers.get("BasicWriter")
    out_dir = Path("/workspace/output/hospital_test_editor")

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
            print(f"[hospital_test_editor] Made prim non-instanceable to enable motion: {path}")
        translate_op = None
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
            translate_op.Set((0.0, 0.0, 0.0))
            print(f"[hospital_test_editor] Added translate op to prim: {path}")
        val = translate_op.Get()
        if val is None:
            val = (0.0, 0.0, 0.0)
        return tuple(val)

    # 4. 只使用指定的4个 prims 进行随机化
    def find_wheelchair_prims():
        """只返回用户指定的4个 prims"""
        specific_prims = [
            "/Root/SM_WheelChair_01a2",
            "/Root/SM_SupplyCart_02a2_91",
            "/Root/SM_WheelChair_01a_37",
            "/Root/SM_SupplyCart_02a_28",
        ]
        
        # 验证这些 prims 是否存在
        valid_prims = []
        for path in specific_prims:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                valid_prims.append(path)
                print(f"[hospital_test_editor] Found specified prim: {path}")
            else:
                print(f"[hospital_test_editor] Warning: Specified prim not found: {path}")
        
        return valid_prims

    # 5. 为物体添加物理属性和变换操作符（运动学模式，防止掉落）
    def enable_physics_on_wheelchairs(wheelchair_paths):
        """为物体添加运动学刚体属性和必要的变换操作符（不受重力影响）"""
        for path in wheelchair_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                continue
            
            # 取消 instanceable 以便添加物理属性
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                print(f"[hospital_test_editor] Made prim non-instanceable for physics: {path}")
            
            # 添加变换操作符（用于 rep.modify.pose）
            xformable = UsdGeom.Xformable(prim)
            if xformable:
                # 1. 检查并添加 translate 操作符（用于位置）
                has_translate = False
                for op in xformable.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        has_translate = True
                        break
                
                if not has_translate:
                    translate_op = xformable.AddTranslateOp()
                    translate_op.Set((0.0, 0.0, 0.0))
                    print(f"[hospital_test_editor] Added translate op to: {path}")
                
                # 2. 检查并添加 rotateXYZ 操作符（用于旋转）
                has_rotate = False
                for op in xformable.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                        has_rotate = True
                        break
                
                if not has_rotate:
                    rotate_op = xformable.AddRotateXYZOp()
                    rotate_op.Set((0.0, 0.0, 0.0))
                    print(f"[hospital_test_editor] Added rotateXYZ op to: {path}")
            
            # 添加运动学刚体组件（不受重力影响，适合 replicator 控制的物体）
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
                # 设置为运动学模式：不受物理力影响，但可以被脚本移动
                rigid_body.CreateKinematicEnabledAttr().Set(True)
                print(f"[hospital_test_editor] Added Kinematic RigidBodyAPI to: {path}")
            else:
                # 如果已经有刚体，确保它是运动学模式
                rigid_body = UsdPhysics.RigidBodyAPI(prim)
                rigid_body.CreateKinematicEnabledAttr().Set(True)
                print(f"[hospital_test_editor] Set existing RigidBodyAPI to Kinematic mode: {path}")
            
            # 添加碰撞组件（用于 scatter_2d 的碰撞检测）
            # 1. 为 Xform 本身添加 CollisionAPI
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
                print(f"[hospital_test_editor] Added CollisionAPI to Xform: {path}")
            else:
                print(f"[hospital_test_editor] Xform already has CollisionAPI: {path}")
            
            # 2. 为所有子 Mesh 添加 CollisionAPI（这对于 scatter_2d 碰撞检测很重要！）
            print(f"[hospital_test_editor] Checking child meshes for CollisionAPI...")
            mesh_count = 0
            def add_collision_to_meshes(parent_prim, depth=0):
                nonlocal mesh_count
                if depth > 5:  # 限制递归深度
                    return
                for child in parent_prim.GetChildren():
                    child_path = child.GetPath().pathString
                    child_type = child.GetTypeName()
                    
                    if child_type == "Mesh":
                        mesh_count += 1
                        if not child.HasAPI(UsdPhysics.CollisionAPI):
                            UsdPhysics.CollisionAPI.Apply(child)
                            print(f"  ✓ Added CollisionAPI to Mesh: {child_path}")
                        else:
                            print(f"  ℹ Mesh already has CollisionAPI: {child_path}")
                    else:
                        # 递归检查子节点
                        add_collision_to_meshes(child, depth + 1)
            
            add_collision_to_meshes(prim)
            print(f"[hospital_test_editor] Processed {mesh_count} mesh(es) under {path}")


    # 注意：碰撞体检测代码已移除
    # 请在 Isaac Sim GUI 中手动为场景物体添加碰撞体：
    # 1. 在 Stage 面板选择物体
    # 2. 右键 → Physics → Colliders Preset
    # 3. 保存场景（可选）

    # NEW: 为特定的4个椅子添加 CollisionAPI（在随机化之前）
    print("\n[hospital_test_editor] ========== DEBUG: Searching for Chair prims ==========")
    specific_chairs = ["SM_Chair_02a4", "SM_Chair_02a5", "SM_Chair_02a6", "SM_Chair_02a7"]
    chair_collision_paths = []  # 记录添加了 CollisionAPI 的椅子路径
    
    for prim in stage.Traverse():
        path_str = prim.GetPath().pathString
        prim_type = prim.GetTypeName()
        
        # 查找所有包含 "Chair_02a" 的 prims（用于调试）
        if "Chair_02a" in path_str:
            print(f"  [DEBUG] Found: {path_str} (Type: {prim_type})")
            
            # 检查是否是我们指定的4个椅子
            for chair_name in specific_chairs:
                if chair_name in path_str:
                    # 为这个 prim 添加 CollisionAPI（不论是 Xform 还是 Mesh）
                    if not prim.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI.Apply(prim)
                        print(f"    ✓ Added CollisionAPI to: {path_str}")
                        chair_collision_paths.append(path_str)
                    else:
                        print(f"    ℹ Already has CollisionAPI: {path_str}")
                        chair_collision_paths.append(path_str)
                    break
    
    print(f"[hospital_test_editor] Added CollisionAPI to {len(chair_collision_paths)} chair prim(s)")
    print("[hospital_test_editor] ========== END DEBUG ==========\n")

    # 6. 查找并验证 prims
    candidate_paths = find_wheelchair_prims()
    
    if not candidate_paths:
        print("[hospital_test_editor] No specified prims were found; cannot proceed.")
        return
    
    print(f"[hospital_test_editor] Will randomize {len(candidate_paths)} prims")
    
    # 为这些 prim 启用物理（运动学模式）
    enable_physics_on_wheelchairs(candidate_paths)
    
    # 7. 辅助函数：列出场景中所有可能的地板 prims
    def list_floor_prims():
        """列出场景中所有可能是地板的 prims"""
        floor_candidates = []
        print("\n[hospital_test_editor] Searching for floor prims in the scene...")
        
        for prim in stage.Traverse():
            path_str = prim.GetPath().pathString
            name_lower = prim.GetName().lower()
            
            # 跳过 Looks/Materials 等非几何节点
            if "/Looks" in path_str or prim.GetTypeName() == "Material":
                continue
            
            # 查找包含 floor/ground/geo 关键词的 prim
            if any(keyword in name_lower for keyword in ["floor", "ground", "geo_floor"]):
                prim_type = prim.GetTypeName()
                floor_candidates.append((path_str, prim_type))
                print(f"  - {path_str} (Type: {prim_type})")
        
        return floor_candidates
    
    # 8. 使用指定的地面平面进行随机放置
    floor_prim = "/Root/Geo_Floor_Costum14_5/Geo_Floor_Costum4"  # 用户指定的地面平面
    
    # 验证地面 prim 是否存在
    floor_prim_obj = stage.GetPrimAtPath(floor_prim)
    if not floor_prim_obj or not floor_prim_obj.IsValid():
        print(f"\n[hospital_test_editor] ❌ Error: Specified floor prim not found: {floor_prim}")
        print(f"[hospital_test_editor] Searching for available floor prims...\n")
        
        # 列出所有可能的地板
        available_floors = list_floor_prims()
        
        if available_floors:
            print(f"\n[hospital_test_editor] Found {len(available_floors)} potential floor prims.")
            print(f"[hospital_test_editor] Please update the 'floor_prim' variable to one of these paths:")
            for path, prim_type in available_floors:
                print(f"  floor_prim = \"{path}\"  # Type: {prim_type}")
            print(f"\n[hospital_test_editor] Then run the script again.\n")
        else:
            print(f"[hospital_test_editor] No floor prims found in the scene.")
        
        return
    
    print(f"[hospital_test_editor] ✓ Using floor prim: {floor_prim}")
    
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
            print(f"[hospital_test_editor] {indent}✓ Found Mesh: {prim_path}")
            return prim_path
        
        # 否则递归搜索子节点中的 Mesh
        if depth == 0:
            print(f"[hospital_test_editor] Searching for Mesh children under: {prim_path}")
        
        for child in prim.GetChildren():
            child_path = child.GetPath().pathString
            child_type = child.GetTypeName()
            
            # 跳过 Looks/Scope 等非几何节点
            if "Looks" in child.GetName() or child_type == "Scope":
                print(f"[hospital_test_editor] {indent}  Skipping: {child.GetName()} (Type: {child_type})")
                continue
            
            print(f"[hospital_test_editor] {indent}  Checking: {child.GetName()} (Type: {child_type})")
            
            # 如果是 Mesh，找到了！
            if child_type == "Mesh":
                print(f"[hospital_test_editor] {indent}  ✓ Found Mesh: {child_path}")
                return child_path
            
            # 如果是 Xform 或其他容器，递归搜索
            if child_type in ["Xform", "Scope", ""]:
                result = find_actual_mesh(child_path, depth + 1, max_depth)
                if result:
                    return result
        
        if depth == 0:
            print(f"[hospital_test_editor] Warning: No Mesh found under {prim_path}")
        return None
    
    # 查找实际的 mesh prim
    actual_floor_mesh = find_actual_mesh(floor_prim)
    print(f"[hospital_test_editor] Using mesh for scatter_2d: {actual_floor_mesh}")
    
    # 注意：已移除自动碰撞检测
    # 请确保在 GUI 中已为场景物体添加了碰撞体（Physics → Colliders Preset）
    
    # 获取地面 prim 作为 scatter_2d 的表面
    surface = rep.get.prims(path_pattern=actual_floor_mesh)
    if not surface:
        print("[hospital_test_editor] ❌ Error: No valid surface found for scatter_2d")
        return


    # 10. 为每个相机分别渲染
    for cam_path in camera_list:
        cam_slug = cam_path.strip("/").replace("/", "_")
        print(f"[hospital_test_editor] Rendering camera: {cam_path}")

        # 基于该相机 prim 创建 render product
        render_product = rep.create.render_product(cam_path, resolution)

        cam_out_dir = out_dir / cam_slug
        raw_dir = cam_out_dir / "_raw_frames"
        os.makedirs(raw_dir, exist_ok=True)
        writer.initialize(output_dir=str(raw_dir), rgb=True, bounding_box_2d_tight=False)
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
                    "description": "A and B frames have different random placements of all objects",
                }
            )

        print(f"[hospital_test_editor] Generating {pair_count} pairs ({total_frames} frames)")

        # 定义随机化函数
        def randomize_wheelchairs():
            """使用 scatter_2d 在地面上随机散布"""
            # 获取所有物体 prims
            wheelchair_nodes = []
            for path in candidate_paths:
                prim_node = rep.get.prims(path_pattern=path, prim_types=['Xform'])
                wheelchair_nodes.append(prim_node)
            
            # 创建物体组
            wheelchairs = rep.create.group(wheelchair_nodes)
            
            # 应用随机化（简化版：使用 check_for_collisions=3 避开所有物体）
            with wheelchairs:
                print(f"  [Randomizer] Using check_for_collisions=3 (avoid all objects)")
                rep.randomizer.scatter_2d(
                    surface_prims=surface,
                    check_for_collisions=3  # 3 = 不与任何东西碰撞
                )
                
                # Z 轴随机旋转
                rep.modify.pose(
                    rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360))
                )
            
            return wheelchairs.node

        # 11. 注册随机化器
        rep.randomizer.register(randomize_wheelchairs)

        # 12. 配置触发器 - 确保每帧都调用随机化
        print(f"\n[hospital_test_editor] ========================================")
        print(f"[hospital_test_editor] Configuring trigger for {total_frames} frames")
        print(f"[hospital_test_editor] ========================================\n")
        
        with rep.trigger.on_frame(num_frames=total_frames):
            rep.randomizer.randomize_wheelchairs()
        
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
        print(f"\n建议：将脚本中的 USE_SCATTER_2D 设置为 False，使用手动随机化模式")
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


def reorganize_pairs_from_raw(output_base_dir: str = "/workspace/output/hospital_test_editor"):
    """
    后处理函数：将 _raw_frames 中的图像重新组织成 pair 结构
    
    这个函数应该在 timeline 播放完成、所有帧都渲染完成后运行。
    它会读取每个相机的 _raw_frames 目录和 pair_metadata.json，
    然后创建 pair_xxxx 目录结构。
    
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
        
        # 获取所有 raw frames
        frames = sorted(raw_dir.glob("rgb_*.png"))
        total_expected = len(pair_records) * 2
        
        if len(frames) < total_expected:
            print(f"[reorganize_pairs] Warning: {cam_dir.name} has {len(frames)} frames, expected {total_expected}")
            continue
        
        print(f"[reorganize_pairs] Processing {cam_dir.name}: {len(pair_records)} pairs from {len(frames)} frames")
        
        # 为每个 pair 创建目录
        for idx, record in enumerate(pair_records):
            pair_dir = cam_dir / record["pair_id"]
            pair_dir.mkdir(parents=True, exist_ok=True)
            
            # 获取对应的帧
            frame_a_idx = idx * 2
            frame_b_idx = idx * 2 + 1
            
            if frame_a_idx >= len(frames) or frame_b_idx >= len(frames):
                print(f"[reorganize_pairs] Warning: Not enough frames for {record['pair_id']}")
                continue
            
            frame_a = frames[frame_a_idx]
            frame_b = frames[frame_b_idx]
            
            # 复制（或移动）到 pair 目录
            # 使用 rename 来移动文件（更高效）
            try:
                frame_a.rename(pair_dir / "A.png")
                frame_b.rename(pair_dir / "B.png")
            except Exception as e:
                # 如果 rename 失败（可能跨文件系统），则使用 copy
                print(f"[reorganize_pairs] Rename failed, using copy: {e}")
                shutil.copy2(frame_a, pair_dir / "A.png")
                shutil.copy2(frame_b, pair_dir / "B.png")
            
            # 保存 pair 的 metadata
            meta_path = pair_dir / "metadata.json"
            with open(meta_path, 'w') as f:
                json.dump(record, f, indent=2)
            
            print(f"[reorganize_pairs] Created {record['pair_id']}: A.png, B.png, metadata.json")
        
        # 清理剩余的 raw frames（如果有）
        remaining_frames = list(raw_dir.glob("rgb_*.png"))
        if remaining_frames:
            print(f"[reorganize_pairs] Cleaning up {len(remaining_frames)} remaining raw frames")
            for frame in remaining_frames:
                frame.unlink(missing_ok=True)
        
        print(f"[reorganize_pairs] Completed {cam_dir.name}: {len(pair_records)} pairs organized")
    
    print(f"\n[reorganize_pairs] All done! Pairs are organized in: {base_dir}")


# 在 Script Editor 中运行时，直接调用函数
if __name__ == "__main__":
    print("=" * 80)
    print("Hospital Test - Script Editor Mode")
    print("只随机化指定的4个 prims，只在 /Root/Geo_Floor_Costum4 平面上")
    print("=" * 80)
    print("\n使用方法:")
    print("1. 运行此脚本设置场景和随机化器")
    print("2. 在 Timeline 中按 PLAY 按钮，等待所有帧渲染完成")
    print("3. 渲染完成后，在 Script Editor 中运行:")
    print("   check_randomization_results()")
    print("   这将验证随机化是否正常工作\n")
    print("=" * 80)
    
    # 可以在这里修改参数
    run_hospital_example_editor(
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