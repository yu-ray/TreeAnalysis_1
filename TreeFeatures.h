// TreeFeatures.h - 头文件仅保留声明
#pragma once
#include <pcl/point_types.h> // 必须包含点类型定义
#include <pcl/point_cloud.h>

struct TreeFeatures {
    float height;
    float dbh;
    float crown_volume;
    
    using PointCloudPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;
    static TreeFeatures compute(const PointCloudPtr& cloud);
};
