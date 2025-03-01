// TreeFeatures.cpp
#include "TreeFeatures.h"
#include <pcl/common/io.h>        // 添加copyPointCloud
#include <pcl/features/moment_of_inertia_estimation.h>

namespace {
// 辅助函数实现
float fit_circle_diameter(const std::vector<pcl::PointXYZRGB>& points);
}

TreeFeatures TreeFeatures::compute(const PointCloudPtr& cloud) {
    // Z范围计算
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    for (const auto& p : *cloud) {
        min_z = std::min(min_z, p.z);
        max_z = std::max(max_z, p.z);
    }
    
    const float height = max_z - min_z;
    const float breast_height = min_z + height / 3;

    // 胸径切片
    std::vector<pcl::PointXYZRGB> slice;
    for (const auto& p : *cloud) {
        if (std::abs(p.z - breast_height) < 0.1f) {
            slice.push_back(p);
        }
    }

    // 体积计算
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_xyz);

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_xyz);
    feature_extractor.compute();

    pcl::PointXYZ min_pt, max_pt, position;
    Eigen::Matrix3f rotational_matrix;
    feature_extractor.getOBB(min_pt, max_pt, position, rotational_matrix);

    return {
        height,
        fit_circle_diameter(slice),
        (max_pt.x - min_pt.x) * (max_pt.y - min_pt.y) * (max_pt.z - min_pt.z)
    };
}

namespace {
float fit_circle_diameter(const std::vector<pcl::PointXYZRGB>& points) {
    if (points.size() < 3) return 0.0f;
    
    Eigen::MatrixXf A(points.size(), 3);
    Eigen::VectorXf b(points.size());
    
    for (size_t i = 0; i < points.size(); ++i) {
        const float x = points[i].x;
        const float y = points[i].y;
        A(i, 0) = 2 * x;
        A(i, 1) = 2 * y;    
        A(i, 2) = 1.0f;
        b(i) = x * x + y * y;
    }
    
    Eigen::Vector3f params = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    return 2 * std::sqrt(params[0] * params[0] + params[1] * params[1] + params[2]);
}
}