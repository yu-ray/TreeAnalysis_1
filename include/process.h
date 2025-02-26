#ifndef PROCESS_H
#define PROCESS_H

//文件引用
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pdal/pdal.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Options.hpp>
#include <pdal/io/LasReader.hpp>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr readLASFile(const std::string &filename);
void visualizeCloud(PointCloud::Ptr cloud, const std::string &viewerName);
float computeTreeHeight(PointCloud::Ptr cloud);
float computeTrunkHeight(PointCloud::Ptr cloud);
float computeDBH(PointCloud::Ptr cloud, float ground_z);
float computeCanopyVolume(PointCloud::Ptr cloud, float trunk_height);

#endif // PROCESS_H