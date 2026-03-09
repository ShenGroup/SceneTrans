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
from pxr import Sdf, UsdGeom, UsdPhysics
import omni.physx


def run_hospital_example(
    pair_count: int = 4,
    warmup_frames: int = 10,
    resolution=(1024, 768),
    diff_mode: str = "random",
    focal_length: float | None = None,
):
    # 2. 直接打开医院场景（保持作者预设的灯光和相机）
    usd_path = "/workspace/assets/Hospital/hospital.usd"
    print(f"[hospital_test] Opening stage: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)

    stage = omni.usd.get_context().get_stage()

    # 场景保持原样，不修改任何灯光设置
    print("[hospital_test] Scene loaded, preserving original lighting")

    # 3. 配合 on_frame 触发器，让 orchestrator 驱动写盘
    rep.orchestrator.set_capture_on_play(True)

    # 4. 查找场景中已有的 Camera，仅保留指定的两个视角
    desired_cameras = {"/Root/Camera", "/OmniverseKit_Persp"}
    camera_list = []
    for prim in stage.Traverse():
        if prim.GetTypeName() == "Camera":
            camera_list.append(prim.GetPath().pathString)
    camera_list = [c for c in camera_list if c in desired_cameras]
    print(f"[hospital_test] Cameras found: {camera_list}")

    if not camera_list:
        # 如果场景里没有相机，就创建一个简单相机
        print("[hospital_test] No camera found, creating a default one.")
        camera_prim = stage.DefinePrim("/World/Camera", "Camera")
        camera_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Float3).Set((0.0, 150.0, 600.0))
        # 默认焦距 35mm，可被用户参数覆盖
        camera_prim.CreateAttribute("focalLength", Sdf.ValueTypeNames.Float).Set(35.0)
        camera_list = ["/World/Camera"]

    # 可选：调整相机焦距（更大焦距 = 视角更窄，更近）
    if focal_length is not None:
        for cam_path in camera_list:
            cam_prim = stage.GetPrimAtPath(cam_path)
            cam_schema = UsdGeom.Camera(cam_prim)
            if cam_schema and cam_schema.GetFocalLengthAttr():
                cam_schema.GetFocalLengthAttr().Set(float(focal_length))
                print(f"[hospital_test] Set focalLength={focal_length} for {cam_path}")

    # 5. BasicWriter：输出到 /workspace/output/hospital_test
    writer = rep.writers.get("BasicWriter")
    out_dir = Path("/workspace/output/hospital_test")

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
            print(f"[hospital_test] Made prim non-instanceable to enable motion: {path}")
        translate_op = None
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
            translate_op.Set((0.0, 0.0, 0.0))
            print(f"[hospital_test] Added translate op to prim: {path}")
        val = translate_op.Get()
        if val is None:
            val = (0.0, 0.0, 0.0)
        return tuple(val)

    # 7. 查找地面平面 prim（floor）用于随机放置 - 只使用指定的平面
    def find_floor_prim():
        """只使用用户指定的地面平面 /Root/Geo_Floor_Costum14_5/Geo_Floor_Costum4"""
        specific_floor = "/Root/Geo_Floor_Costum14_5/Geo_Floor_Costum4"
        
        prim = stage.GetPrimAtPath(specific_floor)
        if prim and prim.IsValid():
            print(f"[hospital_test] Using specified floor: {specific_floor}")
            return specific_floor
        else:
            print(f"[hospital_test] Warning: Specified floor not found: {specific_floor}")
            return None

    # 7b. 获取地面的边界范围
    def get_floor_bounds(floor_path):
        """获取地面 mesh 的边界，用于随机放置"""
        if not floor_path:
            return None
        
        floor_prim = stage.GetPrimAtPath(floor_path)
        if not floor_prim:
            return None
        
        # 获取 mesh 的边界框
        mesh = UsdGeom.Mesh(floor_prim)
        if not mesh:
            return None
        
        # 获取点数据来计算边界
        points_attr = mesh.GetPointsAttr()
        if not points_attr:
            return None
        
        points = points_attr.Get()
        if not points or len(points) == 0:
            return None
        
        # 计算 XZ 平面的边界（Y 是高度）
        min_x = min(p[0] for p in points)
        max_x = max(p[0] for p in points)
        min_z = min(p[2] for p in points)
        max_z = max(p[2] for p in points)
        y_floor = sum(p[1] for p in points) / len(points)  # 平均高度
        
        bounds = {
            'min_x': min_x,
            'max_x': max_x,
            'min_z': min_z,
            'max_z': max_z,
            'y': y_floor,
            'width': max_x - min_x,
            'depth': max_z - min_z
        }
        
        print(f"[hospital_test] Floor bounds: X[{min_x:.1f}, {max_x:.1f}], Z[{min_z:.1f}, {max_z:.1f}], Y={y_floor:.1f}")
        print(f"[hospital_test] Floor size: {bounds['width']:.1f} x {bounds['depth']:.1f}")
        
        return bounds

    # 8. 预先挑选轮椅物件 - 只返回特定的4个 prims
    def find_wheelchair_prims(max_count: int = 15):
        # 只使用用户指定的4个 prims
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
                print(f"[hospital_test] Found specified prim: {path}")
            else:
                print(f"[hospital_test] Warning: Specified prim not found: {path}")
        
        return valid_prims

    # 9. 为轮椅添加物理属性和变换操作符
    def enable_physics_on_wheelchairs(wheelchair_paths):
        """为轮椅添加刚体、碰撞属性和必要的变换操作符"""
        for path in wheelchair_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                continue
            
            # 取消 instanceable 以便添加物理属性
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                print(f"[hospital_test] Made prim non-instanceable for physics: {path}")
            
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
                    # 添加 translate 操作符
                    translate_op = xformable.AddTranslateOp()
                    translate_op.Set((0.0, 0.0, 0.0))
                    print(f"[hospital_test] Added translate op to: {path}")
                
                # 2. 检查并添加 rotateXYZ 操作符（用于旋转）
                has_rotate = False
                for op in xformable.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                        has_rotate = True
                        break
                
                if not has_rotate:
                    # 添加 rotateXYZ 操作符
                    rotate_op = xformable.AddRotateXYZOp()
                    rotate_op.Set((0.0, 0.0, 0.0))
                    print(f"[hospital_test] Added rotateXYZ op to: {path}")
            
            # 添加刚体组件（RigidBodyAPI）
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI.Apply(prim)
                print(f"[hospital_test] Added RigidBodyAPI to: {path}")
            
            # 添加碰撞组件（CollisionAPI）
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
                print(f"[hospital_test] Added CollisionAPI to: {path}")

    # 10. 查找轮椅和地面
    floor_prim = find_floor_prim()
    candidate_paths = find_wheelchair_prims()
    print(f"[hospital_test] Found {len(candidate_paths)} wheelchair prims: {candidate_paths}")
    
    if not candidate_paths:
        print("[hospital_test] No wheelchair prims found; fallback to single snapshot.")
        rep.orchestrator.step()
    else:
        # 为轮椅启用物理
        enable_physics_on_wheelchairs(candidate_paths)
        
        # 如果没有找到地面，创建一个默认地面平面
        if not floor_prim:
            print("[hospital_test] Creating default ground plane")
            ground = stage.DefinePrim("/World/GroundPlane", "Xform")
            ground_mesh = UsdGeom.Mesh.Define(stage, "/World/GroundPlane/Mesh")
            # 创建一个大平面
            ground_mesh.CreatePointsAttr([(-500, 0, -500), (500, 0, -500), (500, 0, 500), (-500, 0, 500)])
            ground_mesh.CreateFaceVertexCountsAttr([4])
            ground_mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
            ground_mesh.CreateNormalsAttr([(0, 1, 0)] * 4)
            # 添加碰撞
            UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
            floor_prim = "/World/GroundPlane"
        
        for cam_path in camera_list:
            cam_slug = cam_path.strip("/").replace("/", "_")
            print(f"[hospital_test] Rendering camera: {cam_path}")

            # 基于该相机 prim 创建 render product
            render_product = rep.create.render_product(cam_path, resolution)

            cam_out_dir = out_dir / cam_slug
            raw_dir = cam_out_dir / "_raw_frames"
            os.makedirs(raw_dir, exist_ok=True)
            print(f"[hospital_test] Output directory: {cam_out_dir}")
            writer.initialize(output_dir=str(raw_dir), rgb=True, bounding_box_2d_tight=False)
            writer.attach(render_product)

            # 暖机帧，避免首帧过暗（不记录）
            rep.orchestrator.set_capture_on_play(False)
            for _ in range(warmup_frames):
                rep.orchestrator.step()
            rep.orchestrator.set_capture_on_play(True)

            # 11. 使用官方推荐的 scatter_2d 模式
            print(f"[hospital_test] Using official scatter_2d pattern for {len(candidate_paths)} wheelchairs on floor: {floor_prim}")
            
            # 获取地面 prim 作为表面
            surface = rep.get.prims(path_pattern=floor_prim) if floor_prim else None
            
            if not surface:
                print("[hospital_test] Warning: No valid surface found for scatter_2d")
                continue
            
            # 创建 pair 计划 - A和B帧使用不同的随机化放置
            total_frames = pair_count * 2
            pair_records = []
            
            # 为每个 pair 生成元数据
            # pair的差异是通过A帧和B帧的不同随机化位置来创建的
            for pair_idx in range(pair_count):
                pair_records.append(
                    {
                        "pair_id": f"pair_{pair_idx:04d}",
                        "camera": cam_path,
                        "difference": "different_randomization",
                        "description": "A and B frames have different random placements of all objects",
                    }
                )

            print(f"[hospital_test] Prepared {pair_count} pairs for {cam_path}; {len(candidate_paths)} wheelchairs.")

            # 12. 为每一帧准备 visibility 序列
            # 所有物体在所有帧中都可见，差异通过位置变化体现
            visibility_sequences = {}
            for path in candidate_paths:
                # 所有物体在所有帧都可见
                vis_seq = [True] * total_frames
                visibility_sequences[path] = vis_seq

            # 13. 定义随机化函数 - 官方推荐模式
            def randomize_wheelchairs():
                """
                官方推荐的 scatter_2d 随机化模式
                这个函数会在每帧被调用，确保真正的随机化
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
                    rep.randomizer.scatter_2d(
                        surface_prims=surface,
                        check_for_collisions=True
                    )
                    
                    # 2. 应用随机旋转（只在水平面上，绕 Y 轴）
                    # 使用 modify.pose 而不是 randomizer.rotation
                    rep.modify.pose(
                        rotation=rep.distribution.uniform((0, 0, 0), (0, 360, 0))
                    )
                
                # 单独应用每个轮椅的可见性（因为每个轮椅的可见性序列不同）
                for idx, path in enumerate(candidate_paths):
                    prim_node = wheelchair_nodes[idx]
                    with prim_node:
                        vis_seq = visibility_sequences.get(path, [True] * total_frames)
                        rep.modify.visibility(rep.distribution.sequence(vis_seq))
                
                return wheelchairs.node

            # 14. 注册随机化器 - 官方推荐模式
            rep.randomizer.register(randomize_wheelchairs)

            # 15. 渲染帧 - 在每帧调用注册的随机化器
            with rep.trigger.on_frame(num_frames=total_frames):
                rep.randomizer.randomize_wheelchairs()
            
            rep.orchestrator.run_until_complete()

            # 13. 重新整理输出，把两帧划入 pair 目录，并存下元信息
            frames = sorted(raw_dir.glob("rgb_*.png"))
            if len(frames) >= total_frames:
                for idx, record in enumerate(pair_records):
                    pair_dir = cam_out_dir / record["pair_id"]
                    pair_dir.mkdir(parents=True, exist_ok=True)
                    frame_a = frames[idx * 2]
                    frame_b = frames[idx * 2 + 1]
                    frame_a.rename(pair_dir / "A.png")
                    frame_b.rename(pair_dir / "B.png")
                    meta_path = pair_dir / "metadata.json"
                    meta_path.write_text(json.dumps(record, indent=2))
                # 清理中间产物
                for leftover in raw_dir.glob("rgb_*.png"):
                    leftover.unlink(missing_ok=True)
            else:
                print(f"[hospital_test] Expected {total_frames} frames, found {len(frames)}. Skipping regroup for {cam_path}.")

            # 清理 render product & writer
            writer.detach()
            render_product.destroy()

    # 14. 等所有数据写完
    rep.orchestrator.wait_until_complete()
    print("[hospital_test] Done writing.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--pair-count", type=int, default=4, help="Number of pairs per camera view.")
    parser.add_argument("--warmup-frames", type=int, default=10, help="Frames to warm up before capture starts.")
    parser.add_argument("--diff-mode", choices=["hide", "move", "random"], default="random", help="How to create the difference between pair images.")
    parser.add_argument("--focal-length", type=float, default=None, help="Override camera focal length (mm). Larger = narrower FOV.")
    args = parser.parse_args()

    run_hospital_example(
        pair_count=args.pair_count,
        warmup_frames=args.warmup_frames,
        diff_mode=args.diff_mode,
        focal_length=args.focal_length,
    )
    simulation_app.close()
