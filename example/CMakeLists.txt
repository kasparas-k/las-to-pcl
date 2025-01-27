cmake_minimum_required(VERSION 3.13)
project(read_and_visualize)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PDAL REQUIRED CONFIG)
find_package(PCL REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/../include)
add_executable(${PROJECT_NAME} read_and_visualize.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE ${PDAL_LIBRARIES} ${PCL_LIBRARIES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PDAL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_directories(${PROJECT_NAME} PRIVATE ${PDAL_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})