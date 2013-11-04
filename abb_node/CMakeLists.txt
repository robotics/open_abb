cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(ABB_PATH ${PROJECT_SOURCE_DIR}/packages/abb_communications)
set(MATVEC_PATH ${PROJECT_SOURCE_DIR}/packages/matVec)

rosbuild_gensrv()

rosbuild_add_library(matVec ${MATVEC_PATH}/HomogTransf.cpp
                            ${MATVEC_PATH}/Mat.cpp
                            ${MATVEC_PATH}/Polynom.cpp
                            ${MATVEC_PATH}/Quaternion.cpp
                            ${MATVEC_PATH}/RotMat.cpp
                            ${MATVEC_PATH}/Vec.cpp)
rosbuild_add_library(abb_comm ${ABB_PATH}/abb_comm.cpp)

include_directories(${ABB_PATH})
include_directories(${MATVEC_PATH})

link_directories(${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(abb_node src/abb_node.cpp)
target_link_libraries(abb_node abb_comm matVec)
