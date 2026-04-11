import open3d as o3d
import os

def main():
    # 地图路径
    map_path = "/home/nvidia/luckrobot/mid360s_ws/map/test.pcd"
    
    # 1. 检查文件是否存在
    if not os.path.exists(map_path):
        print(f"❌ 错误：地图文件不存在：{map_path}")
        return

    # 2. 读取地图
    print(f"📂 正在读取地图：{map_path} ...")
    pcd = o3d.io.read_point_cloud(map_path)
    
    if pcd.is_empty():
        print(f"❌ 错误：地图文件是空的！")
        return

    print(f"✅ 原始地图点数：{len(pcd.points)}")
    print(f"✅ 原始地图是否有颜色：{pcd.HasColors()}")
    print(f"✅ 原始地图是否有法向量：{pcd.HasNormals()}")

    # 3. 如果已经有法向量，先清空（避免重复计算）
    if pcd.HasNormals():
        print("⚠️  地图已经有法向量，将重新计算...")
        pcd.normals_.clear()

    # 4. 计算法向量
    print("🔄 正在计算法向量（邻域K=16）...")
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(16)
    )

    # 5. 统一法向量朝向（非常重要！）
    print("🔄 正在统一法向量朝向...")
    pcd.orient_normals_consistent_tangent_plane(16)

    # 6. 保存地图（覆盖原文件）
    print(f"💾 正在保存地图到：{map_path} ...")
    o3d.io.write_point_cloud(map_path, pcd, write_ascii=False)

    # 7. 验证保存结果
    print("🔍 正在验证保存结果...")
    pcd_check = o3d.io.read_point_cloud(map_path)
    print(f"✅ 保存后地图点数：{len(pcd_check.points)}")
    print(f"✅ 保存后地图是否有法向量：{pcd_check.HasNormals()}")
    
    if pcd_check.HasNormals():
        print("\n🎉🎉🎉 地图法向量计算完成！可以启动定位了！🎉🎉🎉")
    else:
        print("\n❌ 法向量保存失败！")

if __name__ == "__main__":
    main()