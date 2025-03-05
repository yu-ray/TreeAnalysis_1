cmake_minimum_required(VERSION 3.13)
cmake_policy(SET CMP0074 NEW)
project(TreeAnalysis)

set(PCL_DIR "D:/ku_3rdPart/PCL_1.14.1/cmake") 
set(OPENNI2_INCLUDE_DIRS "D:/ku_3rdPart/PCL_1.14.1/3rdParty/OpenNI2/Include")
set(OPENNI2_LIBRARIES "D:/ku_3rdPart/PCL_1.14.1/3rdParty/OpenNI2/Lib/OpenNI2.lib")
set(QHULL_INCLUDE_DIR "D:/ku_3rdPart/PCL_1.14.1/3rdParty/Qhull/include")
set(PDAL_DIR "D:/ku_3rdPart/PCL_1.14.1/3rdParty/PDAL/lib/cmake/PDAL")

find_package(PCL REQUIRED)
find_package(PDAL REQUIRED)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_definitions(-DNOMINMAX)

add_executable(TreeAnalysis main.cpp)

target_include_directories(TreeAnalysis PRIVATE ${PCL_INCLUDE_DIRS} ${PDAL_INCLUDE_DIRS})
target_link_libraries(TreeAnalysis ${PCL_LIBRARIES} ${PDAL_LIBRARIES})
