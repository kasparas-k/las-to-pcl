# las-to-pcl
A C++ header containing a function to read .las and .laz data into PCL PointCloud objects by using PDAL. Due to the more geometric applications of PCL, as opposed to the geographic oriented applications of PDAL, the **las data will be offset by the `X` and `Y` offsets specified in the header**. `Z` values are not offset as they're expected to fit within the `float` and `double` precision limits.

Currently supports reading las data into PCL PointCloud objects with point types:
* `XYZ`

Will add support soon:
* `XYZRGB`
* `XYZI`

However as it is now, the header can be easily modified and expanded to suit your needs.

## Dependencies

This header depends on:
* [Boost](https://www.boost.org/)
* [Eigen](https://eigen.tuxfamily.org/)
* [PCL](https://github.com/PointCloudLibrary/pcl)
* [PDAL](https://github.com/PDAL/PDAL) 

PDAL will give warnings if [Proj](https://github.com/OSGeo/PROJ) is not installed. The visualization example additionally depends on [Vtk](https://vtk.org/).

In a Conda environment, all of the dependencies can be installed from the conda-forge channel, guaranteeing relatively up-to-date versions regardless of your OS package repository:

```
conda install -c conda-forge boost eigen pcl=1.14.1 proj pdal
```

## Usage

To read `.las` / `.laz` files into PCL `PointCloud` objects, just include this header and call the appropriate function with the string containing path to your data:

```c++
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "las_to_pcd.h"

int main(){
    std::string las_path = "data/pc.laz";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = las_to_XYZ(las_path);

    // ... do something with the point cloud in PCL
}
```
An example of reading and visualizing the point cloud, along with a `CMakeLists.txt` to compile your code, is provided in [example](example/). You can try the example by following these steps:

```bash
cd example
mkdir build
cd build
cmake ..
make

./read_and_visualize data/pc.laz
```
