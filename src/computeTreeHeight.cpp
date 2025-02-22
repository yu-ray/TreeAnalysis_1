#include "process.h"

// 计算树高（最大Z-最小Z）
float computeTreeHeight(PointCloud::Ptr cloud)
{
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    return max_pt.z - min_pt.z;
}