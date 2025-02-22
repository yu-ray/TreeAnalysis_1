#include "main.h"

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