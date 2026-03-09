# Headless 版本 - 用于调试 scatter_2d 碰撞避让问题
# 使用 ./python.sh hospital_test_headless.py 运行

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
    focal_length: float | None = None,
):
    """
    Headless 版本的医院场景示例
    包含碰撞避让调试功能
    """
    
    # 2. 打开医院场景
    usd_path = "/workspace/assets/Hospital/hospital.usd"
    print(f"[hospital_headless] Opening stage: {usd_path}")
    omni.usd.get_context().open_stage(usd_path)

    stage = omni.usd.get_context().get_stage()

    # 场景保持原样，不修改任何灯光设置
    print("[hospital_headless] Scene loaded, preserving original lighting")

    # 3. 配合 on_frame 触发器，让 orchestrator 驱动写盘
    rep.orchestrator.set_capture_on_play(True)

    # 4. 查找场景中已有的 Camera，只使用一个主相机
    desired_cameras = {"/Root/Camera_01"}  # 只使用这一个相机
    camera_list = []
    for prim in stage.Traverse():
        if prim.GetTypeName() == "Camera":
            camera_list.append(prim.GetPath().pathString)
    camera_list = [c for c in camera_list if c in desired_cameras]
    print(f"[hospital_headless] Cameras found: {camera_list}")

    if not camera_list:
        print("[hospital_headless] No camera found, creating a default one.")
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
                print(f"[hospital_headless] Set focalLength={focal_length} for {cam_path}")

    # 5. BasicWriter：输出到 /workspace/output/hospital_headless
    writer = rep.writers.get("BasicWriter")
    out_dir = Path("/workspace/output/hospital_headless")

    # 6. 只使用指定的4个 prims 进行随机化
    def find_wheelchair_prims():
        """只返回用户指定的4个 prims"""
        specific_prims = [
            "/Root/SM_WheelChair_01a2",
            "/Root/SM_SupplyCart_02a2_91",
            "/Root/SM_WheelChair_01a_37",
            "/Root/SM_SupplyCart_02a_28",
        ]
        
        valid_prims = []
        for path in specific_prims:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                valid_prims.append(path)
                print(f"[hospital_headless] Found specified prim: {path}")
            else:
                print(f"[hospital_headless] Warning: Specified prim not found: {path}")
        
        return valid_prims

    # 7. 为物体添加物理属性
    def enable_physics_on_wheelchairs(wheelchair_paths):
        """为物体添加运动学刚体属性和 CollisionAPI"""
        for path in wheelchair_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                continue
            
            # 取消 instanceable
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
                print(f"[hospital_headless] Made prim non-instanceable: {path}")
            
            # 添加变换操作符
            xformable = UsdGeom.Xformable(prim)
            if xformable:
                has_translate = False
                has_rotate = False
                for op in xformable.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        has_translate = True
                    if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                        has_rotate = True
                
                if not has_translate:
                    translate_op = xformable.AddTranslateOp()
                    translate_op.Set((0.0, 0.0, 0.0))
                    print(f"[hospital_headless] Added translate op to: {path}")
                
                if not has_rotate:
                    rotate_op = xformable.AddRotateXYZOp()
                    rotate_op.Set((0.0, 0.0, 0.0))
                    print(f"[hospital_headless] Added rotateXYZ op to: {path}")
            
            # 添加运动学刚体
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
                rigid_body.CreateKinematicEnabledAttr().Set(True)
                print(f"[hospital_headless] Added Kinematic RigidBodyAPI to: {path}")
            
            # 为 Xform 添加 CollisionAPI
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
                print(f"[hospital_headless] Added CollisionAPI to Xform: {path}")
            else:
                print(f"[hospital_headless] Xform already has CollisionAPI: {path}")
            
            # 为所有子 Mesh 添加 CollisionAPI
            print(f"[hospital_headless] Checking child meshes for CollisionAPI...")
            mesh_count = 0
            def add_collision_to_meshes(parent_prim, depth=0):
                nonlocal mesh_count
                if depth > 5:
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
                        add_collision_to_meshes(child, depth + 1)
            
            add_collision_to_meshes(prim)
            print(f"[hospital_headless] Processed {mesh_count} mesh(es) under {path}")

    # 9. 查找并验证 prims
    candidate_paths = find_wheelchair_prims()
    
    if not candidate_paths:
        print("[hospital_headless] No specified prims were found; cannot proceed.")
        return
    
    print(f"[hospital_headless] Will randomize {len(candidate_paths)} prims")
    
    # 为这些 prim 启用物理
    enable_physics_on_wheelchairs(candidate_paths)
    
    # 10. 获取地面 mesh（简化版：不再需要设置椅子障碍物）
    floor_prim = "/Root/Geo_Floor_Costum14_5/Geo_Floor_Costum4"
    
    # 查找实际的 mesh prim
    def find_actual_mesh(prim_path, depth=0, max_depth=3):
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
    
    actual_floor_mesh = find_actual_mesh(floor_prim)
    print(f"[hospital_headless] Using floor mesh: {actual_floor_mesh}")
    
    surface = rep.get.prims(path_pattern=actual_floor_mesh)
    if not surface:
        print("[hospital_headless] Error: No valid surface for scatter_2d")
        return

    # 11. 为每个相机渲染
    for cam_path in camera_list:
        cam_slug = cam_path.strip("/").replace("/", "_")
        print(f"\n[hospital_headless] ========== Rendering camera: {cam_path} ==========")

        render_product = rep.create.render_product(cam_path, resolution)

        cam_out_dir = out_dir / cam_slug
        raw_dir = cam_out_dir / "_raw_frames"
        os.makedirs(raw_dir, exist_ok=True)
        writer.initialize(output_dir=str(raw_dir), rgb=True, bounding_box_2d_tight=False)
        writer.attach(render_product)

        # 暖机帧
        rep.orchestrator.set_capture_on_play(False)
        for _ in range(warmup_frames):
            rep.orchestrator.step()
        rep.orchestrator.set_capture_on_play(True)

        # 创建 pair 计划
        total_frames = pair_count * 2
        pair_records = []
        
        for pair_idx in range(pair_count):
            pair_records.append({
                "pair_id": f"pair_{pair_idx:04d}",
                "camera": cam_path,
                "difference": "different_randomization",
            })

        print(f"[hospital_headless] Generating {pair_count} pairs ({total_frames} frames)")

        # 12. 定义随机化函数（简化版）
        def randomize_wheelchairs():
            """
            使用 scatter_2d 在地面上随机散布
            check_for_collisions=3 表示不与场景中任何物体碰撞
            """
            wheelchair_nodes = []
            for path in candidate_paths:
                prim_node = rep.get.prims(path_pattern=path, prim_types=['Xform'])
                wheelchair_nodes.append(prim_node)
            
            wheelchairs = rep.create.group(wheelchair_nodes)
            
            with wheelchairs:
                # 简化版：使用 check_for_collisions=3 避开场景中所有物体
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

        # 13. 注册随机化器
        rep.randomizer.register(randomize_wheelchairs)

        # 14. 配置触发器
        with rep.trigger.on_frame(num_frames=total_frames):
            rep.randomizer.randomize_wheelchairs()
        
        # 16. 运行渲染
        print(f"[hospital_headless] Running orchestrator...")
        rep.orchestrator.run_until_complete()

        # 17. 重新整理输出
        frames = sorted(raw_dir.glob("rgb_*.png"))
        print(f"[hospital_headless] Found {len(frames)} frames")
        
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
                print(f"[hospital_headless] Created {record['pair_id']}")
            
            # 清理
            for leftover in raw_dir.glob("rgb_*.png"):
                leftover.unlink(missing_ok=True)
        else:
            print(f"[hospital_headless] Expected {total_frames} frames, found {len(frames)}")

        writer.detach()
        render_product.destroy()

    # 18. 完成
    rep.orchestrator.wait_until_complete()
    print("\n[hospital_headless] ========== Done! ==========")
    print(f"Output saved to: {out_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--pair-count", type=int, default=4)
    parser.add_argument("--warmup-frames", type=int, default=10)
    parser.add_argument("--focal-length", type=float, default=None)
    args = parser.parse_args()

    run_hospital_example(
        pair_count=args.pair_count,
        warmup_frames=args.warmup_frames,
        focal_length=args.focal_length,
    )
    simulation_app.close()
