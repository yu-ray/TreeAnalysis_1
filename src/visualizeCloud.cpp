#include "process.h"


// 可视化点云
void visualizeCloud(PointCloud::Ptr cloud, const std::string &viewerName)
{
    pcl::visualization::CloudViewer viewer(viewerName);
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
}