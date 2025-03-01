cmake_minimum_required(VERSION 3.13)
cmake_policy(SET CMP0074 NEW)
project(TreeAnalysis)

set(OPENNI2_INCLUDE_DIRS "D:/swjtu/ku_3part/PCL-1.14.1/3rdParty/OpenNI2/Include")
set(OPENNI2_LIBRARIES "D:/swjtu/ku_3part/PCL-1.14.1/3rdParty/OpenNI2/Lib/OpenNI2.lib")
set(QHULL_INCLUDE_DIR "D:/swjtu/ku_3part/PCL-1.14.1/3rdParty/Qhull/include")

find_package(PCL REQUIRED)
find_package(PDAL REQUIRED)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(TreeAnalysis
    main.cpp
    TreeFeatures.cpp
    ColorLUT.cpp
)

target_compile_options(TreeAnalysis PRIVATE
    /bigobj
    /Zc:inline
    /Zc:__cplusplus
)

target_compile_definitions(TreeAnalysis PRIVATE
    _WINSOCKAPI_   # 防止windows.h自动包含winsock.h
    NOMINMAX       # 禁用Windows的min/max宏
)

# 添加Windows SDK库的正确链接顺序
if(WIN32)
    target_link_libraries(TreeAnalysis
        ws2_32.lib
        advapi32.lib
    )
endif()

target_include_directories(TreeAnalysis PRIVATE ${PCL_INCLUDE_DIRS} ${PDAL_INCLUDE_DIRS})
target_link_libraries(TreeAnalysis ${PCL_LIBRARIES} ${PDAL_LIBRARIES})
