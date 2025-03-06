#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>

#include <pdal/Options.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Stage.hpp>
#include <pdal/PluginDirectory.hpp>
#include <pdal/PluginManager.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>	
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <boost/thread/thread.hpp>

typedef pcl::PointXYZRGB PointT;
using namespace std;

// 使用PDAL库读取LAS文件，并转换为PCL点云
void readlasPC(pcl::PointCloud<PointT>::Ptr &cloud0, const string &filename)
{
    pdal::Option las_opt("filename", filename);
    pdal::Options las_opts;
    las_opts.add(las_opt);
    pdal::LasReader las_reader;
    pdal::PointTable table;
    las_reader.setOptions(las_opts);
    las_reader.prepare(table);
    pdal::PointViewSet point_view_set = las_reader.execute(table);
    pdal::PointViewPtr point_view = *point_view_set.begin();
    pdal::LasHeader las_header = las_reader.header();

    cout << "读取点云数据..." << endl;
    // 直接使用原始坐标，不再使用scale和offset
    for (pdal::PointId id = 0; id < point_view->size(); ++id)
    {
        double x = point_view->getFieldAs<double>(pdal::Dimension::Id::X, id);
        double y = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, id);
        double z = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, id);

        double red = point_view->getFieldAs<double>(pdal::Dimension::Id::Red, id);
        double green = point_view->getFieldAs<double>(pdal::Dimension::Id::Green, id);
        double blue = point_view->getFieldAs<double>(pdal::Dimension::Id::Blue, id);

        PointT point;
        point.x = x;
        point.y = y;
        point.z = z;
        const double color_scale = 255.0 / 65535.0;
        point.r = static_cast<uint8_t>(red * color_scale);
        point.g = static_cast<uint8_t>(green * color_scale);
        point.b = static_cast<uint8_t>(blue * color_scale);
        
        cloud0->push_back(point);
    }
    cout << "读取点云数据成功，共" << point_view->size() << "个点" << endl;
}

// 利用垂直分层方法实现树干与树冠分离，同时增加调试输出信息
// 参数说明：
//   binHeight：分层高度（单位：米）
//   trunkDiameterThreshold：若某层水平直径超过此阈值，认为进入树冠区域
void segmentTree(const pcl::PointCloud<PointT>::Ptr &cloud, 
                 pcl::PointCloud<PointT>::Ptr &trunkCloud, 
                 pcl::PointCloud<PointT>::Ptr &crownCloud,
                 float binHeight = 0.5f,
                 float trunkDiameterThreshold = 1.2f)
{
    // 获取整体点云的Z范围
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    float minZ = min_pt.z;
    float maxZ = max_pt.z;
    
    int numBins = static_cast<int>(ceil((maxZ - minZ) / binHeight));
    vector<pcl::PointCloud<PointT>::Ptr> bins(numBins);
    for (int i = 0; i < numBins; i++) {
        bins[i] = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
    }
    
    // 将点根据z值分配到各个层
    for (const auto &pt : cloud->points)
    {
        int binIndex = static_cast<int>((pt.z - minZ) / binHeight);
        if (binIndex >= 0 && binIndex < numBins)
            bins[binIndex]->points.push_back(pt);
    }
    
    // 从低层开始判断，连续满足水平直径小于阈值的层归为树干
    bool crownFound = false;
    for (int i = 0; i < numBins; i++) {
        if (bins[i]->points.empty()){
            cout << "Bin " << i << " is empty." << endl;
            continue;
        }
            
        // 计算当前层的水平范围（XY方向）以及Z范围
        float bin_min_x = std::numeric_limits<float>::max();
        float bin_max_x = std::numeric_limits<float>::lowest();
        float bin_min_y = std::numeric_limits<float>::max();
        float bin_max_y = std::numeric_limits<float>::lowest();
        float bin_min_z = std::numeric_limits<float>::max();
        float bin_max_z = std::numeric_limits<float>::lowest();
        
        for (const auto &pt : bins[i]->points) {
            bin_min_x = std::min(bin_min_x, pt.x);
            bin_max_x = std::max(bin_max_x, pt.x);
            bin_min_y = std::min(bin_min_y, pt.y);
            bin_max_y = std::max(bin_max_y, pt.y);
            bin_min_z = std::min(bin_min_z, pt.z);
            bin_max_z = std::max(bin_max_z, pt.z);
        }
        float diameter = std::max(bin_max_x - bin_min_x, bin_max_y - bin_min_y);
        
        // 输出当前层的调试信息
        cout << "Bin " << i 
             << " z range: [" << bin_min_z << ", " << bin_max_z << "], "
             << "XY range: x [" << bin_min_x << ", " << bin_max_x << "], "
             << "y [" << bin_min_y << ", " << bin_max_y << "], "
             << "Diameter: " << diameter 
             << ", Point Count: " << bins[i]->points.size();
        
        // 判断当前层归属
        if (!crownFound && diameter <= trunkDiameterThreshold)
        {
            cout << " -> Classified as Trunk." << endl;
            *trunkCloud += *(bins[i]);
        }
        else {
            if (!crownFound)
                cout << " -> Crown detected from this bin." << endl;
            else
                cout << " -> Classified as Crown." << endl;
            crownFound = true;
            *crownCloud += *(bins[i]);
        }
    }
}

int main()
{
    cout << "Hello, World!" << endl;

    // 设置数据路径（请根据实际路径修改）
    string filename = "D:/swjtu/ku_VSCode/TreeAnalysis_1/tree01.las";
    cout << "加载文件: " << filename << endl;
    pcl::PointCloud<PointT>::Ptr cloud0(new pcl::PointCloud<PointT>);
    
    // 读取las点云
    readlasPC(cloud0, filename);

    // 输出整体点云范围
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cloud0, min_pt, max_pt);
    cout << "点云范围:\n"
         << "X: " << min_pt.x << " - " << max_pt.x << "\n"
         << "Y: " << min_pt.y << " - " << max_pt.y << "\n"
         << "Z: " << min_pt.z << " - " << max_pt.z << endl;
         
    // 分割树干与树冠
    pcl::PointCloud<PointT>::Ptr trunkCloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr crownCloud(new pcl::PointCloud<PointT>);
    segmentTree(cloud0, trunkCloud, crownCloud);
    
    // 计算整体树高（这里简单认为整体高度即点云最大与最小z值的差）
    float treeHeight = max_pt.z - min_pt.z;
    
    // 计算树干高度：利用树干点云的z范围
    pcl::PointXYZRGB trunk_min_pt, trunk_max_pt;
    float trunkHeight = 0.0f;
    if (!trunkCloud->empty()) {
        pcl::getMinMax3D(*trunkCloud, trunk_min_pt, trunk_max_pt);
        trunkHeight = trunk_max_pt.z - trunk_min_pt.z;
    }
    
    cout << "树高: " << treeHeight << " m" << endl;
    cout << "树干高: " << trunkHeight << " m" << endl;
    
    // 可视化：使用不同颜色显示树干（红色）和树冠（绿色）
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0.5, 0.5, 0.5);
    viewer.initCameraParameters();
    
    pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_color(trunkCloud, 255, 0, 0); // 红色显示树干
    pcl::visualization::PointCloudColorHandlerCustom<PointT> crown_color(crownCloud, 0, 255, 0);  // 绿色显示树冠
    
    if (!trunkCloud->empty())
        viewer.addPointCloud<PointT>(trunkCloud, trunk_color, "trunk cloud");
    if (!crownCloud->empty())
        viewer.addPointCloud<PointT>(crownCloud, crown_color, "crown cloud");
    
    viewer.addCoordinateSystem(0.1, "global");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "trunk cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "crown cloud");
    
    // 根据整体点云设置相机视角
    double range_x = max_pt.x - min_pt.x;
    double range_y = max_pt.y - min_pt.y;
    double range_z = max_pt.z - min_pt.z;
    viewer.setCameraPosition(
        (min_pt.x + max_pt.x) / 2 - 2 * range_x,
        (min_pt.y + max_pt.y) / 2 - 2 * range_y,
        max_pt.z + 5 * range_z,
        (min_pt.x + max_pt.x) / 2,
        (min_pt.y + max_pt.y) / 2,
        (min_pt.z + max_pt.z) / 2,
        0, 1, 0);
        
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
    
    return 0;
}
