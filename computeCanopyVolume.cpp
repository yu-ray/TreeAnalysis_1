#include "../include/process.h" 

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