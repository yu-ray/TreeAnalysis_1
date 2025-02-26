cmake_minimum_required(VERSION 3.13)
cmake_policy(SET CMP0144 OLD)
project(TreeAnalysis)

set(CMAKE_CXX_STANDARD 17)  # 确保使用 C++17
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 显式设置 QHULL 路径
set(QHULL_ROOT "D:/swjtu/ku_3part/PCL-1.14.1/3rdParty/Qhull")
set(QHULL_INCLUDE_DIR "${QHULL_ROOT}/include")
set(QHULL_LIBRARY "${QHULL_ROOT}/lib/qhullstatic.lib")

# 显式设置 Eigen3 路径
set(EIGEN3_ROOT "D:/swjtu/ku_3part/PCL-1.14.1/3rdParty/Eigen")
set(EIGEN3_DIR "D:/swjtu/ku_3part/PCL-1.14.1/3rdParty/Eigen3/share/eigen3/cmake")

# 查找依赖库
find_package(PCL 1.14.1 REQUIRED COMPONENTS common io filters visualization)
find_package(PDAL REQUIRED)
find_package(Eigen3 REQUIRED)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# 动态库配置
aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_LIST)
add_library(common SHARED ${SRC_LIST})

# 头文件包含
target_include_directories(common PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${PDAL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(common PUBLIC
    ${PCL_LIBRARIES}
    ${PDAL_LIBRARIES}
)

# 可执行文件（仅链接动态库）
add_executable(TreeAnalysis ${PROJECT_SOURCE_DIR}/main.cpp)
target_link_libraries(TreeAnalysis PUBLIC common)