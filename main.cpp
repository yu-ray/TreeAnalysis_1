#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

// PCL 头文件
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

// PDAL 头文件（用于读取 LAS 文件）
#include <pdal/PointView.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/Options.hpp>

typedef pcl::PointXYZRGB PointT;

// 用于存储树的基本结构信息
struct TreeStructure
{
    double trunkHeight = 0;   // 树干高度（从地面到树干末端）
    double treeHeight = 0;    // 树总高度
    double diameter = 0;      // 树木胸径（利用拟合圆的半径估算）
    double crownVolume = 0;   // 树冠体积（粗略估计）
    double crownDiameter = 0; // 树冠冠径（水平最大扩展）
};

// 读取 LAS 文件，将点云存入 cloud 中
void readlasPC(pcl::PointCloud<PointT>::Ptr &cloud, const std::string &filename)
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

// 用于保存每个切片拟合圆的参数
struct CircleParameters
{
    double centerX;
    double centerY;
    double radius;
    double z; // 当前切片中间高度
};

// 计算两个拟合圆之间的差异（圆心偏移 + 半径变化）
double computeCircleDifference(const CircleParameters &a, const CircleParameters &b)
{
    double centerDiff = std::sqrt(std::pow(a.centerX - b.centerX, 2) + std::pow(a.centerY - b.centerY, 2));
    double radiusDiff = std::fabs(a.radius - b.radius);
    return centerDiff + radiusDiff;
}

// 根据相邻切片对当前切片拟合结果计算拟合误差
double computeFittingError(const std::vector<CircleParameters> &circleParams, size_t idx)
{
    double error = 0.0;
    size_t n = circleParams.size();
    if (n == 0)
        return error;
    if (idx == 0 && n > 1)
    {
        error = computeCircleDifference(circleParams[idx], circleParams[idx + 1]);
    }
    else if (idx == n - 1 && n > 1)
    {
        error = computeCircleDifference(circleParams[idx], circleParams[idx - 1]);
    }
    else if (n > 2)
    {
        error = computeCircleDifference(circleParams[idx], circleParams[idx - 1]) +
                computeCircleDifference(circleParams[idx], circleParams[idx + 1]);
    }
    return error;
}

// 利用切片方法处理下半部分点云，
// 1. 累积稳定切片归为树干（同时保存拟合圆参数）；
// 2. 一旦检测到当前切片拟合质量较差，则停止树干累积，
//    后续切片均直接归为树冠。
// 同时增加过滤步骤，剔除与拟合圆偏差过大的点。
// 增加后处理：计算所有稳定切片的平均圆心与半径，判断最后一层是否为离群值，若是则移入树冠。
void segmentBySliceWithFittingAccuracy(const pcl::PointCloud<PointT>::Ptr &lowerCloud,
                                       pcl::PointCloud<PointT>::Ptr &trunkCloud,
                                       pcl::PointCloud<PointT>::Ptr &crownCloud,
                                       double &firstStableRadius)
{
    double sliceThickness = 0.2;     // 切片厚度（m）
    int minInliers = 5;              // 有效拟合所需的最小内点数
    double errorThreshold = 5.0;     // 邻近层拟合误差阈值（放宽要求）
    double baseErrorThreshold = 5.0; // 与最低层拟合结果比较的误差阈值（放宽要求）
    double circleTolerance = 0.1;    // 点与拟合圆半径允许偏差（m）

    // 后处理参数：允许的偏差范围（相对于所有稳定切片的平均值）
    double maxCenterDeviation = 0.2; // 圆心允许偏差
    double maxRadiusDeviation = 0.1; // 半径允许偏差

    PointT minLower, maxLower;
    pcl::getMinMax3D(*lowerCloud, minLower, maxLower);
    double minZ = minLower.z;
    double maxZ = maxLower.z;

    std::vector<CircleParameters> circleParams;
    std::vector<pcl::PointCloud<PointT>::Ptr> trunkSlices; // 保存归为树干的切片

    bool accumulateTrunk = true; // 是否继续累积树干切片

    std::cout << "Processing lowerCloud slices from z = " << minZ << " to " << maxZ << std::endl;

    // 按z切片处理下半部分点云
    for (double currentZ = minZ; currentZ < maxZ; currentZ += sliceThickness)
    {
        pcl::PointCloud<PointT>::Ptr slice(new pcl::PointCloud<PointT>);
        pcl::PassThrough<PointT> passSlice;
        passSlice.setInputCloud(lowerCloud);
        passSlice.setFilterFieldName("z");
        passSlice.setFilterLimits(currentZ, currentZ + sliceThickness);
        passSlice.filter(*slice);

        std::cout << "Slice [" << currentZ << ", " << currentZ + sliceThickness
                  << "] contains " << slice->size() << " points." << std::endl;
        if (slice->empty())
            continue;

        // 如果已停止累积树干，则将当前切片全部归入树冠
        if (!accumulateTrunk)
        {
            *crownCloud += *slice;
            continue;
        }

        // 在x-y平面上用RANSAC拟合二维圆
        pcl::SACSegmentation<PointT> segCircle;
        segCircle.setOptimizeCoefficients(true);
        segCircle.setModelType(pcl::SACMODEL_CIRCLE2D);
        segCircle.setMethodType(pcl::SAC_RANSAC);
        segCircle.setDistanceThreshold(0.05);
        segCircle.setInputCloud(slice);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        segCircle.segment(*inliers, *coefficients);
        std::cout << "Fitted circle in slice: inliers = " << inliers->indices.size() << std::endl;

        if (inliers->indices.size() < (size_t)minInliers)
        {
            std::cout << "Insufficient inliers (" << inliers->indices.size()
                      << "), stopping trunk accumulation." << std::endl;
            accumulateTrunk = false;
            *crownCloud += *slice;
            continue;
        }

        // 初步拟合得到圆参数
        CircleParameters cp;
        cp.centerX = coefficients->values[0];
        cp.centerY = coefficients->values[1];
        cp.radius = coefficients->values[2];
        cp.z = currentZ + sliceThickness / 2.0;

        // 过滤步骤：剔除与拟合圆偏差过大的点
        pcl::PointCloud<PointT>::Ptr filteredSlice(new pcl::PointCloud<PointT>);
        for (const auto &pt : slice->points)
        {
            double dist = std::sqrt(std::pow(pt.x - cp.centerX, 2) + std::pow(pt.y - cp.centerY, 2));
            if (std::fabs(dist - cp.radius) <= circleTolerance)
                filteredSlice->push_back(pt);
            else
                crownCloud->push_back(pt); // 偏差过大的直接归入树冠
        }
        std::cout << "After filtering, slice has " << filteredSlice->size() << " points." << std::endl;
        if (filteredSlice->size() < (size_t)minInliers)
        {
            std::cout << "After filtering, insufficient points (" << filteredSlice->size()
                      << "), stopping trunk accumulation." << std::endl;
            accumulateTrunk = false;
            *crownCloud += *slice;
            continue;
        }

        // 对过滤后点重新拟合，获得更准确的圆参数
        segCircle.setInputCloud(filteredSlice);
        pcl::PointIndices::Ptr inliersFiltered(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficientsFiltered(new pcl::ModelCoefficients);
        segCircle.segment(*inliersFiltered, *coefficientsFiltered);
        cp.centerX = coefficientsFiltered->values[0];
        cp.centerY = coefficientsFiltered->values[1];
        cp.radius = coefficientsFiltered->values[2];

        // 第一层直接保存为参考圆；后续层与参考圆比较
        if (circleParams.empty())
        {
            circleParams.push_back(cp);
            trunkSlices.push_back(filteredSlice);
        }
        else
        {
            double refError = computeCircleDifference(cp, circleParams[0]);
            std::cout << "Slice at z = " << cp.z << " has base error = " << refError << std::endl;
            if (refError > baseErrorThreshold)
            {
                std::cout << "Slice deviates from base circle (error = " << refError
                          << "), stopping trunk accumulation." << std::endl;
                accumulateTrunk = false;
                *crownCloud += *slice;
                continue;
            }
            else
            {
                circleParams.push_back(cp);
                trunkSlices.push_back(filteredSlice);
            }
        }
    } // end for

    // 后处理：如果累积了多个树干切片，则计算平均圆参数，
    // 并检测最后一片是否为离群值，若是，则将其移入树冠
    if (!circleParams.empty() && circleParams.size() > 1)
    {
        double sumCenterX = 0, sumCenterY = 0, sumRadius = 0;
        for (const auto &cp : circleParams)
        {
            sumCenterX += cp.centerX;
            sumCenterY += cp.centerY;
            sumRadius += cp.radius;
        }
        double avgCenterX = sumCenterX / circleParams.size();
        double avgCenterY = sumCenterY / circleParams.size();
        double avgRadius = sumRadius / circleParams.size();

        CircleParameters lastCp = circleParams.back();
        double centerDev = std::sqrt(std::pow(lastCp.centerX - avgCenterX, 2) +
                                     std::pow(lastCp.centerY - avgCenterY, 2));
        double radiusDev = std::fabs(lastCp.radius - avgRadius);
        std::cout << "Average trunk circle: center (" << avgCenterX << ", " << avgCenterY
                  << "), radius " << avgRadius << std::endl;
        std::cout << "Last trunk slice deviation: centerDev = " << centerDev
                  << ", radiusDev = " << radiusDev << std::endl;
        if (centerDev > maxCenterDeviation || radiusDev > maxRadiusDeviation)
        {
            std::cout << "Last trunk slice is an outlier, moving it to crown." << std::endl;
            // 移除最后一个累积的树干切片
            circleParams.pop_back();
            if (!trunkSlices.empty())
            {
                *crownCloud += *(trunkSlices.back());
                trunkSlices.pop_back();
            }
        }
    }

    // 合并所有累积的树干切片点，构造 trunkCloud
    for (size_t i = 0; i < trunkSlices.size(); ++i)
    {
        *trunkCloud += *(trunkSlices[i]);
    }
    // 设置 firstStableRadius（如果存在）
    if (!circleParams.empty())
        firstStableRadius = circleParams[0].radius;
    else
        firstStableRadius = 0.0;
}

// 整体树分割函数，将输入点云分为树干和树冠，并计算冠体积与冠径
TreeStructure segmentTree(const pcl::PointCloud<PointT>::Ptr &inputCloud,
                          pcl::PointCloud<PointT>::Ptr &trunkCloud,
                          pcl::PointCloud<PointT>::Ptr &crownCloud)
{
    TreeStructure treeData;
    PointT minPt, maxPt;
    pcl::getMinMax3D(*inputCloud, minPt, maxPt);
    treeData.treeHeight = maxPt.z - minPt.z;

    // 将整棵树点云按高度分为下半部分和上半部分
    pcl::PointCloud<PointT>::Ptr lowerCloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr upperCloud(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(inputCloud);
    pass.setFilterFieldName("z");
    // 下半部分：从最低点到树高中点
    pass.setFilterLimits(minPt.z, minPt.z + treeData.treeHeight * 0.5);
    pass.filter(*lowerCloud);
    // 上半部分：中点至最高点
    pass.setFilterLimits(minPt.z + treeData.treeHeight * 0.5, maxPt.z);
    pass.filter(*upperCloud);

    pcl::PointCloud<PointT>::Ptr lowerTrunk(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr lowerCrown(new pcl::PointCloud<PointT>);
    double firstStableRadius = 0.0;
    segmentBySliceWithFittingAccuracy(lowerCloud, lowerTrunk, lowerCrown, firstStableRadius);

    // 树干为下半部分中累积的稳定部分，树冠为下半部分剩余部分加上上半部分
    *trunkCloud = *lowerTrunk;
    *crownCloud = *lowerCrown + *upperCloud;

    if (!trunkCloud->empty())
    {
        PointT minTrunk, maxTrunk;
        pcl::getMinMax3D(*trunkCloud, minTrunk, maxTrunk);
        treeData.trunkHeight = maxTrunk.z - minPt.z;
    }
    else
    {
        treeData.trunkHeight = 0;
    }
    // 利用第一层稳定切片的拟合圆半径估计树干直径
    treeData.diameter = 2 * firstStableRadius;

    // 计算树冠体积和冠径（利用树冠点云包围盒粗略估计）
    if (!crownCloud->empty())
    {
        PointT minCrown, maxCrown;
        pcl::getMinMax3D(*crownCloud, minCrown, maxCrown);
        double crownWidth = maxCrown.x - minCrown.x;
        double crownDepth = maxCrown.y - minCrown.y;
        double crownHeight = maxCrown.z - minCrown.z;
        treeData.crownVolume = crownWidth * crownDepth * crownHeight;
        treeData.crownDiameter = std::max(crownWidth, crownDepth);
    }
    else
    {
        treeData.crownVolume = 0;
        treeData.crownDiameter = 0;
    }

    return treeData;
}

int main(int argc, char **argv)
{
    std::string filename = "D:/swjtu/ku_VSCode/TreeAnalysis_1/tree01.las";
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    readlasPC(cloud, filename);

    PointT minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    std::cout << "点云范围: X(" << minPt.x << " - " << maxPt.x
              << "), Y(" << minPt.y << " - " << maxPt.y
              << "), Z(" << minPt.z << " - " << maxPt.z << ")" << std::endl;

    pcl::PointCloud<PointT>::Ptr trunkCloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr crownCloud(new pcl::PointCloud<PointT>);
    TreeStructure treeData = segmentTree(cloud, trunkCloud, crownCloud);

    std::cout << "树干点数: " << trunkCloud->size() << std::endl;
    std::cout << "树冠点数: " << crownCloud->size() << std::endl;
    std::cout << "树高: " << treeData.treeHeight << " m" << std::endl;
    std::cout << "树干高: " << treeData.trunkHeight << " m" << std::endl;
    std::cout << "树木胸径: " << treeData.diameter << " m" << std::endl;
    std::cout << "冠径: " << treeData.crownDiameter << " m" << std::endl;
    std::cout << "树冠体积: " << treeData.crownVolume << " m^3" << std::endl;

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_color(trunkCloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> crown_color(crownCloud, 0, 255, 0);
    if (!trunkCloud->empty())
        viewer.addPointCloud<PointT>(trunkCloud, trunk_color, "trunk cloud");
    if (!crownCloud->empty())
        viewer.addPointCloud<PointT>(crownCloud, crown_color, "crown cloud");

    if (!crownCloud->empty())
    {
        PointT minCrown, maxCrown;
        pcl::getMinMax3D(*crownCloud, minCrown, maxCrown);
        // 添加一个线框立方体显示树冠点云的包围盒
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
