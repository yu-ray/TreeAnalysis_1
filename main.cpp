#include <iostream>
#include <vector>
#include <memory>

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

// pdal库读写las
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

	// 头文件信息
	unsigned int PointCount = las_header.pointCount();
	double scale_x = las_header.scaleX();
	double scale_y = las_header.scaleY();
	double scale_z = las_header.scaleZ();
	double offset_x = las_header.offsetX();
	double offset_y = las_header.offsetY();
	double offset_z = las_header.offsetZ();

	// 读点
	cout << "读取点云数据..." << endl;
	cout << "Scale factors: (" 
	<< las_header.scaleX() << ", "
	<< las_header.scaleY() << ", "
	<< las_header.scaleZ() << ")\n"
	<< "Offsets: ("
	<< las_header.offsetX() << ", "
	<< las_header.offsetY() << ", "
	<< las_header.offsetZ() << ")" << endl;

    const double color_scale = 255.0 / 65535.0;  // 正确的颜色缩放比例
	for (pdal::PointId id = 0; id < point_view->size(); ++id)
	{
		double x = point_view->getFieldAs<double>(pdal::Dimension::Id::X, id);
		double y = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, id);
		double z = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, id);

		double red = point_view->getFieldAs<double>(pdal::Dimension::Id::Red, id);
		double green = point_view->getFieldAs<double>(pdal::Dimension::Id::Green, id);
		double blue = point_view->getFieldAs<double>(pdal::Dimension::Id::Blue, id);

        PointT point;
        point.x = x*scale_x + offset_x;
		point.y = y*scale_y + offset_y;
		point.z = z*scale_z + offset_z;
        point.r = static_cast<uint8_t>(red * color_scale);
        point.g = static_cast<uint8_t>(green * color_scale);
        point.b = static_cast<uint8_t>(blue * color_scale);
        
        cloud0->push_back(point);
	}
	cout << "读取点云数据成功，共"<<point_view->size()<<"个点" << endl;
}

int main()
{
	cout << "Hello, World!" << endl;

	string filename = ("D:/swjtu/ku_VSCode/TreeAnalysis_1/tree01.las"); // 使用原始字符串
	cout << "加载文件: " << filename << endl;
	pcl::PointCloud<PointT>::Ptr cloud0(new pcl::PointCloud<PointT>);

	readlasPC(cloud0, filename);

	// 输出点云范围
	pcl::PointXYZRGB min_pt, max_pt;
	pcl::getMinMax3D(*cloud0, min_pt, max_pt);
	cout << "点云范围:\n"
     << "X: " << min_pt.x << " - " << max_pt.x << "\n"
     << "Y: " << min_pt.y << " - " << max_pt.y << "\n"
     << "Z: " << min_pt.z << " - " << max_pt.z << endl;

	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	// 设置可视化参数
	viewer.setBackgroundColor(0.5, 0.5, 0.5); // 灰色背景
	viewer.initCameraParameters();           // 初始化相机参数
	// 使用RGB颜色处理
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud0);
	viewer.addPointCloud<PointT>(cloud0, rgb, "sample cloud");
	// 调整坐标系参数
	viewer.addCoordinateSystem(0.1, "global");

	// 计算动态参数
	const double range_x = max_pt.x - min_pt.x;
	const double range_y = max_pt.y - min_pt.y;
	const double range_z = max_pt.z - min_pt.z;
	// 设置相机参数
	viewer.setCameraPosition(
		(min_pt.x + max_pt.x)/2 - 2*range_x,  // 相机X位置
		(min_pt.y + max_pt.y)/2 - 2*range_y,  // 相机Y位置
		max_pt.z + 5*range_z,                // 相机Z位置（5倍高度上方）
		(min_pt.x + max_pt.x)/2,             // 目标点X
		(min_pt.y + max_pt.y)/2,             // 目标点Y
		(min_pt.z + max_pt.z)/2,             // 目标点Z
		0, 1, 0);                            // 上方向

    // 调整点云显示大小
	viewer.setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
		1,  // 增大点尺寸
		"sample cloud");
	
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	return 0;
}