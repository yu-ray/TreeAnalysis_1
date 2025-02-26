#include "../include/process.h" 

// 读取LAS文件
PointCloud::Ptr readLASFile(const std::string &filename)
{
    pdal::StageFactory factory;
    pdal::Stage &reader = *factory.createStage("readers.las");
    pdal::Options options;
    options.add("filename", filename);
    reader.setOptions(options);

    pdal::PointTable table;
    reader.prepare(table);
    pdal::PointViewSet viewSet = reader.execute(table);
    pdal::PointViewPtr view = *viewSet.begin();

    PointCloud::Ptr cloud(new PointCloud);
    cloud->width = view->size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width);

    for (pdal::PointId id = 0; id < view->size(); ++id)
    {
        auto &p = cloud->points[id];
        p.x = view->getFieldAs<double>(pdal::Dimension::Id::X, id);
        p.y = view->getFieldAs<double>(pdal::Dimension::Id::Y, id);
        p.z = view->getFieldAs<double>(pdal::Dimension::Id::Z, id);
    }

    return cloud;
}