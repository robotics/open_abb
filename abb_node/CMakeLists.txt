# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(abb_node)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp tf sensor_msgs geometry_msgs)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(ABB_PATH ${PROJECT_SOURCE_DIR}/packages/abb_communications)
set(MATVEC_PATH ${PROJECT_SOURCE_DIR}/packages/matVec)

add_service_files(
  FILES
  # TODO: List your msg files here
)

add_library(matVec ${MATVEC_PATH}/HomogTransf.cpp
                            ${MATVEC_PATH}/Mat.cpp
                            ${MATVEC_PATH}/Polynom.cpp
                            ${MATVEC_PATH}/Quaternion.cpp
                            ${MATVEC_PATH}/RotMat.cpp
                            ${MATVEC_PATH}/Vec.cpp)
add_library(abb_comm ${ABB_PATH}/abb_comm.cpp)

include_directories(${ABB_PATH})
include_directories(${MATVEC_PATH})

link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(abb_node src/abb_node.cpp)
target_link_libraries(abb_node abb_comm matVec)
## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp tf sensor_msgs geometry_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)