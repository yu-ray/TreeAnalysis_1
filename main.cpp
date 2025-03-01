// main.cpp
#include "TreeFeatures.h"
#include "ColorLUT.h"
#include <iostream>
#include <cmath>       // 添加isfinite函数支持
#include <Windows.h>   // 在包含PDAL头文件前包含Windows头
#include <filesystem>  // 添加文件系统支持
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp> 
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pcl/visualization/pcl_visualizer.h>

// 显式声明点云类型
using PointT = pcl::PointXYZRGB;
using CloudPtr = pcl::PointCloud<PointT>::Ptr;
using namespace std;


CloudPtr read_las_optimized(const string& filename) {
    using namespace pdal;  // 添加PDAL命名空间

    if (!std::filesystem::exists(filename)) throw runtime_error("File not found");
    
    pdal::LasReader reader;  // 明确使用pdal命名空间
    pdal::Options opts;
    opts.add("filename", filename);
    reader.setOptions(opts);

    pdal::PointTable table;
    reader.prepare(table);

    // 修改此处获取header的方式
    const pdal::LasHeader& header = reader.header();  // 添加命名空间限定
    const pdal::point_count_t count = header.pointCount();

    CloudPtr cloud = pcl::make_shared<pcl::PointCloud<PointT>>();
    cloud->reserve(count);

    // 分块读取
    constexpr point_count_t BLOCK_SIZE = 1000000;
    point_count_t remaining = count;

    while (remaining > 0) {
        PointViewSet viewSet = reader.execute(table);
        PointViewPtr view = *viewSet.begin();
        const point_count_t actual_count = min(BLOCK_SIZE, remaining);

        {
            const size_t start_idx = cloud->size();
            cloud->resize(start_idx + view->size());

            // 顺序处理点云数据转换
            for (size_t i = 0; i < view->size(); ++i) {
                auto& pcl_point = (*cloud)[start_idx + i];
                
                pcl_point.x = view->getFieldAs<double>(Dimension::Id::X, i);
                pcl_point.y = view->getFieldAs<double>(Dimension::Id::Y, i);
                pcl_point.z = view->getFieldAs<double>(Dimension::Id::Z, i);

                if (!isfinite(pcl_point.x)) pcl_point.x = 0;
                if (!isfinite(pcl_point.y)) pcl_point.y = 0;
                if (!isfinite(pcl_point.z)) pcl_point.z = 0;

                const auto color_conv = [](uint16_t v) -> uint8_t {
                    return ColorLUT::convert(v); // ✅ 通过公共接口访问
                };

                pcl_point.r = ColorLUT::convert(view->getFieldAs<uint16_t>(Dimension::Id::Red, i));
                pcl_point.g = ColorLUT::convert(view->getFieldAs<uint16_t>(Dimension::Id::Green, i));
                pcl_point.b = ColorLUT::convert(view->getFieldAs<uint16_t>(Dimension::Id::Blue, i));
            }

            cloud->width = cloud->size();
            cloud->height = 1;
            cloud->is_dense = false;
        }

        remaining -= actual_count;
    }

    return cloud;
}

int main() {
    try {
        cout << "Hello, World!" << endl;
        auto cloud = read_las_optimized("tree01.las");
        auto features = TreeFeatures::compute(cloud);

        // 输出结果
        cout << "=== Tree Analysis Report ===\n"
            << "Height:       " << features.height << " m\n"
            << "DBH:         " << features.dbh << " m\n"
            << "Crown Volume: " << features.crown_volume << " m³\n"
            << "===========================\n";

        // 可选可视化（调试时启用）
        pcl::visualization::PCLVisualizer viewer("Cloud");
        viewer.addPointCloud(cloud);
        viewer.spin();

        return EXIT_SUCCESS;
    }
    catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return EXIT_FAILURE;
    }
}

