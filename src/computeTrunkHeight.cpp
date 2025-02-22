#include "main.h"

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