cmake_minimum_required(VERSION 2.8.3)
project(voxblox_ground_truth)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#################
# GAZEBO PLUGIN #
#################
include_directories(${GAZEBO_INCLUDE_DIRS})
cs_add_library(voxblox_ground_truth_plugin SHARED
    src/user_interfaces/gazebo_plugin.cpp
    src/sdf_creator.cpp
    src/sdf_visualizer.cpp
    src/triangle_geometer.cpp)

##########
# EXPORT #
##########
cs_install()
cs_export()