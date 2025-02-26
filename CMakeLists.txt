cmake_minimum_required(VERSION 3.13)
cmake_policy(SET CMP0144 OLD)
project(TreeAnalysis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 显式设置第三方库路径
set(QHULL_ROOT "D:/swjtu/ku_3part/PCL-1.14.1/3rdParty/Qhull")
set(QHULL_INCLUDE_DIR "${QHULL_ROOT}/include")
set(QHULL_LIBRARY "${QHULL_ROOT}/lib/qhullstatic.lib")

set(EIGEN3_ROOT "D:/swjtu/ku_3part/PCL-1.14.1/3rdParty/Eigen")
set(EIGEN3_DIR "${EIGEN3_ROOT}/share/eigen3/cmake")  # 根据实际路径调整

# 查找依赖库
find_package(PCL 1.14.1 REQUIRED COMPONENTS common io filters visualization)
find_package(PDAL REQUIRED)
find_package(Eigen3 REQUIRED)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# 动态库配置（假设所有源文件在同一目录）
file(GLOB SRC_LIST 
    "${PROJECT_SOURCE_DIR}/*.cpp" 
    "${PROJECT_SOURCE_DIR}/src/*.cpp"  # 如果保留 src 目录
)
list(REMOVE_ITEM SRC_LIST "${PROJECT_SOURCE_DIR}/main.cpp")  # 排除 main.cpp

add_library(common SHARED ${SRC_LIST})

# 头文件包含（当前目录和第三方库）
target_include_directories(common PUBLIC
    ${PROJECT_SOURCE_DIR}          # 当前目录直接包含头文件
    ${PCL_INCLUDE_DIRS}
    ${PDAL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(common PUBLIC
    ${PCL_LIBRARIES}
    ${PDAL_LIBRARIES}
)

# 可执行文件（仅包含 main.cpp）
add_executable(TreeAnalysis ${PROJECT_SOURCE_DIR}/main.cpp)
target_link_libraries(TreeAnalysis PUBLIC common)