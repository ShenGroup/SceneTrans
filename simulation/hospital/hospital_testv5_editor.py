# Script Editor 版本 - 基于 hospital_testv5.py 改编
# 适合在 Isaac Sim GUI 中运行，支持 timeline 播放
# 修复：轮椅使用运动学刚体，不会掉到地板下面

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
    """
    
    # 1. 打开医院场景
    usd_path = "/workspace/assets/Hospital/hospital.usd"
    print(f"[hospital_testv5_editor] Opening stage: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)

    stage = omni.usd.get_context().get_stage()

    # 场景保持原样，不修改任何灯光设置
    print("[hospital_testv5_editor] Scene loaded, preserving original lighting")

    # 2. 查找场景中已有的 Camera，仅保留指定的两个视角
    desired_cameras = {"/Root/Camera", "/OmniverseKit_Persp"}
    camera_list = []
    for prim in stage.Traverse():
        if prim.GetTypeName() == "Camera":
            camera_list.append(prim.GetPath().pathString)
    camera_list = [c for c in camera_list if c in desired_cameras]
    print(f"[hospital_testv5_editor] Cameras found: {camera_list}")

    if not camera_list:
        # 如果场景里没有相机，就创建一个简单相机
        print("[hospital_testv5_editor] No camera found, creating a default one.")
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
                print(f"[hospital_testv5_editor] Set focalLength={focal_length} for {cam_path}")

    # 3. BasicWriter：输出到 /workspace/output/hospital_testv5_editor
    writer = rep.writers.get("BasicWriter")
    out_dir = Path("/workspace/output/hospital_testv5_editor")

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
            print(f"[hospital_testv5_editor] Made prim non-instanceable to enable motion: {path}")
        translate_op = None
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
            translate_op.Set((0.0, 0.0, 0.0))
            print(f"[hospital_testv5_editor] Added translate op to prim: {path}")
        val = translate_op.Get()
        if val is None:
            val = (0.0, 0.0, 0.0)
        return tuple(val)

    # 4. 查找地面平面 prim（floor）用于随机放置
    def find_floor_prim():
        """查找场景中的地面平面，必须是实际的 Mesh"""
        floor_candidates = []
        
        # 第一遍：查找所有包含 floor/ground 的 prim
        for prim in stage.Traverse():
            name_lower = prim.GetName().lower()
            path_lower = prim.GetPath().pathString.lower()
            
            # 跳过 Looks 等非几何节点
            if "/looks" in path_lower or prim.GetTypeName() == "Material":
                continue
            
            if "floor" in name_lower or "ground" in name_lower or "floor" in path_lower:
                # 只接受 Mesh 类型
                if prim.GetTypeName() == "Mesh":
                    floor_candidates.append(prim.GetPath().pathString)
                    print(f"[hospital_testv5_editor] Found floor mesh: {prim.GetPath().pathString}")
        
        # 如果找到多个，选择第一个
        if floor_candidates:
            return floor_candidates[0]
        
        # 如果没找到 Mesh，尝试找 Xform/Scope 容器下的 Mesh 子节点
        print("[hospital_testv5_editor] No direct floor mesh found, searching for mesh children...")
        for prim in stage.Traverse():
            name_lower = prim.GetName().lower()
            path_lower = prim.GetPath().pathString.lower()
            
            if "floor" in name_lower or "ground" in name_lower or "floor" in path_lower:
                # 检查子节点是否有 Mesh
                for child in prim.GetChildren():
                    if child.GetTypeName() == "Mesh":
                        print(f"[hospital_testv5_editor] Found floor mesh in children: {child.GetPath().pathString}")
                        return child.GetPath().pathString
        
        print("[hospital_testv5_editor] Warning: No floor prim found")
        return None

    # 5. 预先挑选轮椅物件
    def find_wheelchair_prims(max_count: int = 15):
        excluded_types = {
            "Camera",
            "DistantLight",
            "SphereLight",
            "DomeLight",
            "CylinderLight",
            "RectLight",
        }
        # 只随机轮椅相关的 prim
        allowed_semantics = [
            "wheelchair",
            "wheel_chair",
            "sm_wheelchair",
        ]
        keyword_forced = [
            "wheelchair",
            "wheel_chair",
        ]
        disallowed_keywords = [
            "wall",
            "floor",
            "ceiling",
            "window",
            "door",
            "pillar",
            "column",
            "beam",
            "ceilling",
        ]
        forced_paths = []
        semantic_hits = []
        candidates = []
        for prim in stage.Traverse():
            if prim.IsPseudoRoot():
                continue
            if prim.GetTypeName() in excluded_types:
                continue
            if prim.GetName().lower().startswith("camera"):
                continue
            # 过滤掉 Looks/Materials 等非几何节点
            path_str = prim.GetPath().pathString
            if "/Looks" in path_str or prim.GetTypeName() == "Material":
                continue
            img = UsdGeom.Imageable(prim)
            if not img:
                continue
            name_lower = prim.GetName().lower()

            # 如果名字命中关键词，先放入强制列表
            if any(k in name_lower for k in keyword_forced):
                forced_paths.append(prim.GetPath().pathString)
                continue

            # 先尝试按语义标签筛选（大小写不敏感）
            prim_sem_match = False
            for attr in prim.GetAttributes():
                name_lower = attr.GetName().lower()
                if "semantic" not in name_lower and "semantics" not in name_lower:
                    continue
                if "class" not in name_lower:
                    continue
                try:
                    val = attr.Get()
                except Exception:
                    continue
                if val is None:
                    continue
                if isinstance(val, (list, tuple, set)):
                    vals = [str(v).lower() for v in val]
                else:
                    vals = [str(val).lower()]
                if any(v in allowed_semantics for v in vals):
                    prim_sem_match = True
                    break
            if prim_sem_match:
                semantic_hits.append(prim.GetPath().pathString)
                if len(semantic_hits) >= max_count:
                    break
                continue

            # 退化：按名字关键词过滤（避免墙/地等）
            if any(bad in name_lower for bad in disallowed_keywords):
                continue
            if not any(good in name_lower for good in allowed_semantics):
                continue
            candidates.append(prim.GetPath().pathString)
            if len(candidates) >= max_count:
                break

        forced_list = forced_paths
        return forced_list or (semantic_hits if semantic_hits else candidates)

    # 6. 为轮椅添加物理属性和变换操作符（修复：使用运动学刚体，防止掉落）
    def enable_physics_on_wheelchairs(wheelchair_paths):
        """为轮椅添加运动学刚体属性和必要的变换操作符（不受重力影响）"""
        for path in wheelchair_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                continue
            
            # 取消 instanceable 以便添加物理属性
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                print(f"[hospital_testv5_editor] Made prim non-instanceable for physics: {path}")
            
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
                    print(f"[hospital_testv5_editor] Added translate op to: {path}")
                
                # 2. 检查并添加 rotateXYZ 操作符（用于旋转）
                has_rotate = False
                for op in xformable.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                        has_rotate = True
                        break
                
                if not has_rotate:
                    rotate_op = xformable.AddRotateXYZOp()
                    rotate_op.Set((0.0, 0.0, 0.0))
                    print(f"[hospital_testv5_editor] Added rotateXYZ op to: {path}")
            
            # 添加运动学刚体组件（不受重力影响，适合 replicator 控制的物体）
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
                # 设置为运动学模式：不受物理力影响，但可以被脚本移动
                rigid_body.CreateKinematicEnabledAttr().Set(True)
                print(f"[hospital_testv5_editor] Added Kinematic RigidBodyAPI to: {path}")
            else:
                # 如果已经有刚体，确保它是运动学模式
                rigid_body = UsdPhysics.RigidBodyAPI(prim)
                rigid_body.CreateKinematicEnabledAttr().Set(True)
                print(f"[hospital_testv5_editor] Set existing RigidBodyAPI to Kinematic mode: {path}")
            
            # 添加碰撞组件（用于 scatter_2d 的碰撞检测）
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
                print(f"[hospital_testv5_editor] Added CollisionAPI to: {path}")

    # 7. 使用指定的4个 prim 进行随机化
    
    # 直接指定要随机化的 prim 路径（根据用户提供的4个对象）
    candidate_paths = [
        "/Root/SM_WheelChair_01a2",
        "/Root/SM_SupplyCart_02a2_91",
        "/Root/SM_WheelChair_01a_37",
        "/Root/SM_SupplyCart_02a_28",
    ]
    
    print(f"[hospital_testv5_editor] Using {len(candidate_paths)} specified prims for randomization:")
    for path in candidate_paths:
        print(f"  - {path}")
    
    # 验证这些 prim 是否存在
    existing_paths = []
    for path in candidate_paths:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            existing_paths.append(path)
            print(f"  ✓ Found: {path}")
        else:
            print(f"  ✗ Not found: {path}")
    
    if not existing_paths:
        print("[hospital_testv5_editor] None of the specified prims were found; cannot proceed.")
        return
    
    candidate_paths = existing_paths
    print(f"[hospital_testv5_editor] Will randomize {len(candidate_paths)} prims")
    
    # 为这些 prim 启用物理（运动学模式）
    enable_physics_on_wheelchairs(candidate_paths)
    
    # 8. 使用指定的地面平面进行随机放置
    floor_prim = "/Root/Geo_Floor_Costum2"  # 用户指定的地面平面
    
    # 验证地面 prim 是否存在
    floor_prim_obj = stage.GetPrimAtPath(floor_prim)
    if not floor_prim_obj or not floor_prim_obj.IsValid():
        print(f"[hospital_testv5_editor] Error: Specified floor prim not found: {floor_prim}")
        print(f"[hospital_testv5_editor] Please check the prim path in the stage tree.")
        return
    
    print(f"[hospital_testv5_editor] ✓ Using floor prim: {floor_prim}")
    
    # 9. 为每个相机分别渲染（恢复原始的多相机循环）
    for cam_path in camera_list:
        cam_slug = cam_path.strip("/").replace("/", "_")
        print(f"[hospital_testv5_editor] Rendering camera: {cam_path}")

        # 基于该相机 prim 创建 render product
        render_product = rep.create.render_product(cam_path, resolution)

        cam_out_dir = out_dir / cam_slug
        raw_dir = cam_out_dir / "_raw_frames"
        os.makedirs(raw_dir, exist_ok=True)
        print(f"[hospital_testv5_editor] Output directory: {cam_out_dir}")
        writer.initialize(output_dir=str(raw_dir), rgb=True, bounding_box_2d_tight=False)
        writer.attach(render_product)

        # 9. 配置轮椅随机化
        print(f"[hospital_testv5_editor] Using official scatter_2d pattern for {len(candidate_paths)} wheelchairs on floor: {floor_prim}")
        
        # 获取地面 prim 作为表面
        surface = rep.get.prims(path_pattern=floor_prim) if floor_prim else None
        
        if not surface:
            print("[hospital_testv5_editor] Warning: No valid surface found for scatter_2d")
            continue
        
        # 创建 pair 计划
        total_frames = pair_count * 2
        pair_records = []
        
        # 为每个 pair 生成元数据
        for pair_idx in range(pair_count):
            # 选择一个目标轮椅进行差异化
            target_path = random.choice(candidate_paths)
            
            if diff_mode == "hide":
                difference_type = "hide"
            elif diff_mode == "move":
                difference_type = "move"
            else:
                difference_type = random.choice(["hide", "move"])
            
            pair_records.append(
                {
                    "pair_id": f"pair_{pair_idx:04d}",
                    "camera": cam_path,
                    "target_path": target_path,
                    "difference": difference_type,
                }
            )

        print(f"[hospital_testv5_editor] Prepared {pair_count} pairs for {cam_path}; {len(candidate_paths)} wheelchairs.")

        # 10. 为每一帧准备 visibility 序列
        visibility_sequences = {}
        for path in candidate_paths:
            vis_seq = []
            for frame_idx in range(total_frames):
                pair_idx = frame_idx // 2
                is_b_frame = (frame_idx % 2) == 1
                
                # 默认可见
                visible = True
                
                # 如果是 B 帧且该物体是目标，检查是否需要隐藏
                if is_b_frame and pair_idx < len(pair_records):
                    record = pair_records[pair_idx]
                    if record["target_path"] == path and record["difference"] == "hide":
                        visible = False
                
                vis_seq.append(visible)
            
            visibility_sequences[path] = vis_seq

        # 11. 定义随机化函数 - 官方推荐模式（修复：添加垂直偏移）
        def randomize_wheelchairs():
            """
            官方推荐的 scatter_2d 随机化模式
            这个函数会在每帧被调用，确保真正的随机化
            修复：添加垂直偏移，防止轮椅掉到地板下面
            """
            # 获取所有轮椅 prims（只获取顶层 Xform，不包括子节点如 Looks）
            wheelchair_nodes = []
            for path in candidate_paths:
                # 关键：指定 prim_types=['Xform'] 只获取可变换的顶层 prim
                prim_node = rep.get.prims(path_pattern=path, prim_types=['Xform'])
                wheelchair_nodes.append(prim_node)
            
            # 创建轮椅组
            wheelchairs = rep.create.group(wheelchair_nodes)
            
            # 在轮椅组的上下文中应用随机化
            with wheelchairs:
                # 1. 应用 scatter_2d - 在地面上随机散布
                # 注意：轮椅已设置为运动学刚体（kinematic），不会受重力影响
                rep.randomizer.scatter_2d(
                    surface_prims=surface,
                    check_for_collisions=True
                )
                
                # 2. 应用随机旋转和垂直偏移（修复：添加垂直偏移）
                rep.modify.pose(
                    position=rep.distribution.uniform((0, 5, 0), (0, 10, 0)),  # Y 轴偏移 5-10 cm
                    rotation=rep.distribution.uniform((0, 0, 0), (0, 360, 0))  # 绕 Y 轴随机旋转
                )
            
            # 单独应用每个轮椅的可见性（因为每个轮椅的可见性序列不同）
            for idx, path in enumerate(candidate_paths):
                prim_node = wheelchair_nodes[idx]
                with prim_node:
                    vis_seq = visibility_sequences.get(path, [True] * total_frames)
                    rep.modify.visibility(rep.distribution.sequence(vis_seq))
            
            return wheelchairs.node

        # 12. 注册随机化器 - 官方推荐模式
        rep.randomizer.register(randomize_wheelchairs)

        # 13. 渲染帧 - 在每帧调用注册的随机化器
        with rep.trigger.on_frame(num_frames=total_frames):
            rep.randomizer.randomize_wheelchairs()
        
        # 14. 设置 timeline 的帧范围（Editor 模式特有）
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_start_time(0)
        timeline.set_end_time(total_frames / 24.0)  # 假设 24 fps
        timeline.set_current_time(0)
        
        print(f"[hospital_testv5_editor] Timeline configured for {cam_path}:")
        print(f"  - Total frames: {total_frames}")
        print(f"  - Duration: {total_frames / 24.0:.2f} seconds (at 24 fps)")
        print(f"  - Output will be saved to: {raw_dir}")
        
        # 保存 pair metadata
        metadata_file = cam_out_dir / "pair_metadata.json"
        metadata_file.parent.mkdir(parents=True, exist_ok=True)
        metadata_file.write_text(json.dumps(pair_records, indent=2))
        print(f"[hospital_testv5_editor] Pair metadata saved to: {metadata_file}")

    # 15. 最终提示
    print(f"\n[hospital_testv5_editor] Setup complete! Press PLAY in the timeline to start rendering.")
    print(f"[hospital_testv5_editor] You can step frame-by-frame using the timeline controls.")
    print(f"[hospital_testv5_editor] Total cameras configured: {len(camera_list)}")
    print(f"\n[hospital_testv5_editor] After rendering, run reorganize_pairs_from_raw() to organize frames into pairs.")


def reorganize_pairs_from_raw(output_base_dir: str = "/workspace/output/hospital_testv5_editor"):
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
        
        # 获取所有 rawframes
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
        
        # 清理剩余的 rawframes（如果有）
        remaining_frames = list(raw_dir.glob("rgb_*.png"))
        if remaining_frames:
            print(f"[reorganize_pairs] Cleaning up {len(remaining_frames)} remaining rawframes")
            for frame in remaining_frames:
                frame.unlink(missing_ok=True)
        
        print(f"[reorganize_pairs] Completed {cam_dir.name}: {len(pair_records)} pairs organized")
    
    print(f"\n[reorganize_pairs] All done! Pairs are organized in: {base_dir}")


# 在 Script Editor 中运行时，直接调用函数
if __name__ == "__main__":
    print("=" * 80)
    print("Hospital Test V5 - Script Editor Mode (Fixed: Wheelchairs won't fall)")
    print("=" * 80)
    print("\n使用方法:")
    print("1. 运行此脚本设置场景和随机化器")
    print("2. 在 Timeline 中按 PLAY 按钮，等待所有帧渲染完成")
    print("3. 渲染完成后，在 Script Editor 中运行:")
    print("   reorganize_pairs_from_raw()")
    print("   这将把 rawframes 重新组织成 pair 结构\n")
    print("=" * 80)
    
    # 可以在这里修改参数
    run_hospital_example_editor(
        pair_count=4,           # 生成 4 对图像 = 8 帧
        warmup_frames=10,       # 暖机帧数
        diff_mode="random",     # 差异模式: "hide", "move", "random"
        focal_length=None,      # 焦距（可选）
    )
