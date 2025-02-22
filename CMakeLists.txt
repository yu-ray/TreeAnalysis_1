cmake_minimum_required(VERSION 3.13)
project(TreeAnalysis)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#找三方库
find_package(PCL REQUIRED)
find_package(PDAL REQUIRED)

add_definitions(${PCL_DEFINITIONS})

#设置目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

#生成动态库
aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_LIST)
add_library(common SHARED ${SRC_LIST})

#添加头文件目录
target_include_directories(common PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${PDAL_INCLUDE_DIRS}
)

target_link_directories(common PUBLIC
    ${PCL_LIBRARY_DIRS}
    ${PDAL_LIBRARY_DIRS}
)

target_link_libraries(common PUBLIC
    ${PCL_LIBRARIES}
    ${PDAL_LIBRARIES}
)

#可执行文件
add_executable(TreeAnalysis ${SRC_LIST})

target_include_directories(TreeAnalysis PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(TreeAnalysis PUBLIC common)
