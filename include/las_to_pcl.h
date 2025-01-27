#include <string>

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/Options.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr las_to_XYZ(std::string file_path){
    pdal::Option las_opt("filename", file_path);
    pdal::Options las_opts;
    las_opts.add(las_opt);

    pdal::PointTable table;

    pdal::LasReader las_reader;
    las_reader.setOptions(las_opts);
    las_reader.prepare(table);
    
    pdal::PointViewSet point_view_set = las_reader.execute(table);
    pdal::PointViewPtr point_view = *point_view_set.begin();
    pdal::Dimension::IdList dims = point_view->dims();
    pdal::LasHeader las_header = las_reader.header();

    double x_offset = las_header.offsetX();
    double y_offset = las_header.offsetY();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(point_view->size());
    for (pdal::PointId idx = 0; idx < point_view->size(); ++idx){
        pcl::PointXYZ point;
        point.x = point_view->getFieldAs<double>(pdal::Dimension::Id::X, idx) - x_offset;
        point.y = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, idx) - y_offset;
        point.z = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);
        cloud->push_back(point); 
    }

    return cloud; 
}