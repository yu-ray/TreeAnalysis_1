#include <iostream>
#include "process.h"

using namespace std;

int main()
{
    // 读取数据
    PointCloud::Ptr cloud = readLASFile(tree01.las);
    cout << "点云点数: " << cloud->size() << endl;

    // 计算各项参数
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    float tree_height = computeTreeHeight(cloud);
    float trunk_height = computeTrunkHeight(cloud);
    float dbh = computeDBH(cloud, min_pt.z);
    float canopy_volume = computeCanopyVolume(cloud, trunk_height);
    if (canopy_volume < 0.001f)
    {
        cerr << "警告：树冠体积计算可能存在误差" << endl;
    }

    // 输出结果
    cout << "\n===== 测量结果 =====" << endl; // ← 使用英文符号
    cout << "Tree Height: " << tree_height << " m" << endl;
    cout << "Trunk Height: " << trunk_height << " m" << endl;
    cout << "DBH: " << dbh << " m" << endl;
    cout << "Canopy Volume: " << canopy_volume << " m³" << endl; // ← 使用³符号

    // 可视化原始点云
    visualizeCloud(cloud, "原始点云");

    return 0;
}