#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "las_to_pcl.h"

int main(int argc, char* argv[]){
    if (argc != 2){
        std::cerr << "Usage: " << argv[0] << " <path_to_las_file>" << std::endl;
        return 1;
    }

    std::string file_path = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = las_to_XYZ(file_path);

    std::cout << "Read " << cloud->size() << " points." << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->spin();
}