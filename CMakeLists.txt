cmake_minimum_required(VERSION 3.13)
project(TreeAnalysis)

find_package(PCL REQUIRED)
find_package(PDAL REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${PDAL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${PDAL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
aux_source_directory(. SRC_LIST)
add_executable(TreeAnalysis ${SRC_LIST})

target_link_libraries(TreeAnalysis
    ${PCL_LIBRARIES}
    ${PDAL_LIBRARIES}
)

if(MSVC)
    add_compile_options(/utf-8)
    add_compile_definitions(_SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING)
endif()

