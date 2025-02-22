#include <iostream>
#include "process.h"

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