#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <deque>
#include <limits>

// PCL 头文件
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/convex_hull.h>

// PDAL 头文件（用于读取 LAS 文件）
#include <pdal/PointView.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/Options.hpp>

typedef pcl::PointXYZRGB PointT;

using namespace std;

// ----------------------- 树基本信息结构 -----------------------
struct TreeStructure
{
    double trunkHeight = 0;   // 树干高度（从地面到第一个分叉处）
    double treeHeight = 0;    // 树总高度
    double diameter = 0;      // 树木胸径（区域横向尺寸的平均值）
    double crownVolume = 0;   // 树冠体积（基于包围盒粗略估计）
    double crownDiameter = 0; // 树冠冠径（基于包围盒粗略估计）
};

// ----------------------- 点云读取 -----------------------
void readlasPC(pcl::PointCloud<PointT>::Ptr &cloud, const string &filename)
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

    for (pdal::PointId id = 0; id < point_view->size(); ++id)
    {
        double x = point_view->getFieldAs<double>(pdal::Dimension::Id::X, id);
        double y = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, id);
        double z = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, id);
        PointT point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->push_back(point);
    }
}

// ----------------------- 基于种子点的树干/树冠分割 -----------------------
// 思路：
// 1. 从树低部（例如最低1.0 m）中提取种子候选区域，利用欧式聚类选取最大聚类作为种子；
// 2. 在整棵树中进行区域生长，但只增长到预设高度（例如地面起6.0 m以内），
//    将区域生长得到的点作为树干，其余点归为树冠。
// 基于种子点的树干/树冠分割方法（修改后）
// 目标：仅在较低部分生长区域作为树干，其余为树冠
TreeStructure segmentTreeSeedPoint(const pcl::PointCloud<PointT>::Ptr &inputCloud,
                                   pcl::PointCloud<PointT>::Ptr &trunkCloud,
                                   pcl::PointCloud<PointT>::Ptr &crownCloud)
{
    // 获取全树点云范围
    PointT minPt, maxPt;
    pcl::getMinMax3D(*inputCloud, minPt, maxPt);
    double treeHeight = maxPt.z - minPt.z;

    // 提取低部种子候选区域（例如 z 在 [minPt.z, minPt.z + 1.0]）
    pcl::PointCloud<PointT>::Ptr seedCandidates(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(inputCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(minPt.z, minPt.z + 0.2);
    pass.filter(*seedCandidates);

    // 欧式聚类：选择最大聚类作为种子
    vector<pcl::PointIndices> clusterIndices;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(seedCandidates);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.15); // 稍微收紧容差
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(seedCandidates);
    ec.extract(clusterIndices);

    pcl::PointCloud<PointT>::Ptr seedCluster(new pcl::PointCloud<PointT>);
    size_t maxSize = 0;
    for (const auto &indices : clusterIndices)
    {
        if (indices.indices.size() > maxSize)
        {
            maxSize = indices.indices.size();
            seedCluster->clear();
            for (const auto &idx : indices.indices)
            {
                seedCluster->push_back(seedCandidates->points[idx]);
            }
        }
    }
    // 如果种子候选为空，则返回全部点为树冠
    if (seedCluster->empty())
    {
        trunkCloud->clear();
        *crownCloud = *inputCloud;
        TreeStructure treeData;
        treeData.treeHeight = treeHeight;
        return treeData;
    }

    // 构建全树 KD-tree
    pcl::search::KdTree<PointT>::Ptr fullTree(new pcl::search::KdTree<PointT>);
    fullTree->setInputCloud(inputCloud);
    vector<bool> visited(inputCloud->size(), false);
    deque<int> queue;

    // 修改：将 trunkHeightLimit 降低到 3.0 m
    double trunkHeightLimit = 3.0; // 生长上限仅为地面起 3.0 m
    // 同时收紧区域生长的邻域搜索半径
    double regionTolerance = 0.105; // 邻域搜索半径设为 0.2 m

    // 初始化区域生长：以种子聚类中的点在全树中的最近邻作为起点
    for (const auto &pt : seedCluster->points)
    {
        vector<int> indices(1);
        vector<float> sqrDistances(1);
        fullTree->nearestKSearch(pt, 1, indices, sqrDistances);
        if (!visited[indices[0]])
        {
            // 仅当该点 z 值不超过生长上限时加入
            if (inputCloud->points[indices[0]].z <= minPt.z + trunkHeightLimit)
            {
                visited[indices[0]] = true;
                queue.push_back(indices[0]);
                trunkCloud->push_back(inputCloud->points[indices[0]]);
            }
        }
    }

    // 区域生长（仅在较低高度内生长）
    while (!queue.empty())
    {
        int currIdx = queue.front();
        queue.pop_front();
        vector<int> neighbors;
        vector<float> sqrDists;
        fullTree->radiusSearch(inputCloud->points[currIdx], regionTolerance, neighbors, sqrDists);
        for (int nIdx : neighbors)
        {
            // 仅添加 z 值在生长上限内的点
            if (!visited[nIdx] && inputCloud->points[nIdx].z <= minPt.z + trunkHeightLimit)
            {
                visited[nIdx] = true;
                queue.push_back(nIdx);
                trunkCloud->push_back(inputCloud->points[nIdx]);
            }
        }
    }

    // 剩余点归为树冠
    crownCloud->clear();
    for (size_t i = 0; i < inputCloud->size(); i++)
    {
        if (!visited[i])
            crownCloud->push_back(inputCloud->points[i]);
    }

    // 重新计算树干高度：仅取区域生长得到的点中最大 z 值（这就是分叉前的树干高度）
    PointT minTrunk, maxTrunk;
    if (!trunkCloud->empty())
        pcl::getMinMax3D(*trunkCloud, minTrunk, maxTrunk);
    TreeStructure treeData;
    treeData.treeHeight = treeHeight;
    treeData.trunkHeight = (trunkCloud->empty() ? 0 : maxTrunk.z - minPt.z);

    // 计算胸径：取树干区域在水平面（X、Y）的包围盒宽度平均值
    if (!trunkCloud->empty())
    {
        double dx = maxTrunk.x - minTrunk.x;
        double dy = maxTrunk.y - minTrunk.y;
        treeData.diameter = (dx + dy) / 2.0;
    }
    else
        treeData.diameter = 0;

    // 计算树冠体积和冠径（基于树冠点云包围盒）
    if (!crownCloud->empty())
    {
        PointT minCrown, maxCrown;
        pcl::getMinMax3D(*crownCloud, minCrown, maxCrown);
        double crownWidth = maxCrown.x - minCrown.x;
        double crownDepth = maxCrown.y - minCrown.y;
        double crownHeight = maxCrown.z - minCrown.z;
        treeData.crownVolume = crownWidth * crownDepth * crownHeight;
        treeData.crownDiameter = max(crownWidth, crownDepth);
    }
    else
    {
        treeData.crownVolume = 0;
        treeData.crownDiameter = 0;
    }
    return treeData;
}

// ----------------------- 整体分割流程 -----------------------
// 此处只采用种子点算法进行树干与树冠分割（数据只包含一棵树）
int main(int argc, char **argv)
{
    string filename = "D:/swjtu/ku_VSCode/TreeAnalysis_1/tree02.las";
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    readlasPC(cloud, filename);

    PointT minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    cout << "点云范围: X(" << minPt.x << " - " << maxPt.x
         << "), Y(" << minPt.y << " - " << maxPt.y
         << "), Z(" << minPt.z << " - " << maxPt.z << ")" << endl;

    pcl::PointCloud<PointT>::Ptr trunkCloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr crownCloud(new pcl::PointCloud<PointT>);
    TreeStructure treeData = segmentTreeSeedPoint(cloud, trunkCloud, crownCloud);

    cout << "树干点数: " << trunkCloud->size() << endl;
    cout << "树冠点数: " << crownCloud->size() << endl;
    cout << "树高: " << treeData.treeHeight << " m" << endl;
    cout << "树干高: " << treeData.trunkHeight << " m" << endl;
    cout << "树木胸径: " << treeData.diameter << " m" << endl;
    cout << "冠径: " << treeData.crownDiameter << " m" << endl;
    cout << "树冠体积: " << treeData.crownVolume << " m^3" << endl;

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    if (!trunkCloud->empty())
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_color(trunkCloud, 255, 0, 0);
        viewer.addPointCloud<PointT>(trunkCloud, trunk_color, "trunk cloud");
    }
    if (!crownCloud->empty())
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointT> crown_color(crownCloud, 0, 255, 0);
        viewer.addPointCloud<PointT>(crownCloud, crown_color, "crown cloud");
        PointT minCrown, maxCrown;
        pcl::getMinMax3D(*crownCloud, minCrown, maxCrown);
        viewer.addCube(minCrown.x, maxCrown.x,
                       minCrown.y, maxCrown.y,
                       minCrown.z, maxCrown.z,
                       0.0, 0.0, 1.0, "crown_bbox");
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                           pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                           "crown_bbox");
    }

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << minPt.x, minPt.y, minPt.z;
    viewer.addCoordinateSystem(1.0, transform);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return 0;
}
