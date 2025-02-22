#include "main.h"

// 读取LAS文件
PointCloud::Ptr readLASFile(const std::string &filename)
{
    pdal::StageFactory factory;
    pdal::Stage &reader = *factory.createStage("readers.las");
    pdal::Options options;
    options.add("filename", filename);
    reader.setOptions(options);

    pdal::PointTable table;
    reader.prepare(table);
    pdal::PointViewSet viewSet = reader.execute(table);
    pdal::PointViewPtr view = *viewSet.begin();

    PointCloud::Ptr cloud(new PointCloud);
    cloud->width = view->size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width);

    for (pdal::PointId id = 0; id < view->size(); ++id)
    {
        auto &p = cloud->points[id];
        p.x = view->getFieldAs<double>(pdal::Dimension::Id::X, id);
        p.y = view->getFieldAs<double>(pdal::Dimension::Id::Y, id);
        p.z = view->getFieldAs<double>(pdal::Dimension::Id::Z, id);
    }

    return cloud;
}

// 可视化点云
void visualizeCloud(PointCloud::Ptr cloud, const std::string &viewerName)
{
    pcl::visualization::CloudViewer viewer(viewerName);
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
}

// 计算树高（最大Z-最小Z）
float computeTreeHeight(PointCloud::Ptr cloud)
{
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    return max_pt.z - min_pt.z;
}

// 计算树干高度（简化版：取前5%高度作为主干）
float computeTrunkHeight(PointCloud::Ptr cloud)
{
    // 按高度排序
    std::sort(cloud->points.begin(), cloud->points.end(),
              [](const PointT &a, const PointT &b)
              { return a.z < b.z; });

    // 取前5%高度的点云
    int trunk_end = cloud->size() * 0.05;
    float max_trunk_z = cloud->points[trunk_end].z;
    return max_trunk_z - cloud->points[0].z;
}

// 计算胸径（1.3m处半径估算）
float computeDBH(PointCloud::Ptr cloud, float ground_z)
{
    const float measure_height = 1.3; // 胸高1.3米
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(ground_z + measure_height - 0.1,
                         ground_z + measure_height + 0.1);

    PointCloud::Ptr slice(new PointCloud);
    pass.filter(*slice);

    if (slice->empty())
        return 0.0f;

    // 估算半径（取xy平面最大距离）
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*slice, centroid);

    float max_distance = 0.0f;
    for (const auto &p : *slice)
    {
        float dx = p.x - centroid[0];
        float dy = p.y - centroid[1];
        max_distance = std::max(max_distance, sqrtf(dx * dx + dy * dy));
    }

    return max_distance * 2; // 直径
}

// 修改后的计算树冠体积函数
float computeCanopyVolume(PointCloud::Ptr cloud, float trunk_height)
{
    // 分离树冠
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_pt.z + trunk_height, max_pt.z);

    PointCloud::Ptr canopy(new PointCloud);
    pass.filter(*canopy);

    if (canopy->empty())
        return 0.0f;

    // 计算凸包体积
    pcl::ConvexHull<PointT> hull;
    hull.setInputCloud(canopy);
    PointCloud::Ptr surface_hull(new PointCloud);
    hull.reconstruct(*surface_hull);

    // 计算OBB包围盒
    pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
    feature_extractor.setInputCloud(canopy);
    feature_extractor.compute();

    PointT min_obb, max_obb, position;
    Eigen::Matrix3f rotational_matrix;
    bool obb_valid = feature_extractor.getOBB(min_obb, max_obb, position, rotational_matrix);

    if (!obb_valid)
    {
        std::cerr << "无效的OBB计算结果" << std::endl;
        return 0.0f;
    }

    // 计算实际旋转后的尺寸
    Eigen::Vector3f size = max_obb.getVector3fMap() - min_obb.getVector3fMap();
    size = rotational_matrix.inverse() * size; // 考虑旋转方向
    return std::abs(size.x() * size.y() * size.z());
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <las_file>" << std::endl;
        return -1;
    }

    // 读取数据
    PointCloud::Ptr cloud = readLASFile(argv[1]);
    std::cout << "点云点数: " << cloud->size() << std::endl;

    // 计算各项参数
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    float tree_height = computeTreeHeight(cloud);
    float trunk_height = computeTrunkHeight(cloud);
    float dbh = computeDBH(cloud, min_pt.z);
    float canopy_volume = computeCanopyVolume(cloud, trunk_height);
    if (canopy_volume < 0.001f)
    {
        std::cerr << "警告：树冠体积计算可能存在误差" << std::endl;
    }

    // 输出结果
    std::cout << "\n===== 测量结果 =====" << std::endl; // ← 使用英文符号
    std::cout << "Tree Height: " << tree_height << " m" << std::endl;
    std::cout << "Trunk Height: " << trunk_height << " m" << std::endl;
    std::cout << "DBH: " << dbh << " m" << std::endl;
    std::cout << "Canopy Volume: " << canopy_volume << " m³" << std::endl; // ← 使用³符号

    // 可视化原始点云
    visualizeCloud(cloud, "原始点云");

    return 0;
}