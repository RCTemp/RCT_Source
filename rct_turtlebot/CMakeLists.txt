cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

find_package(Eigen)
find_package(PCL)
include_directories(${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_boost_directories()

# move_base
rosbuild_add_executable(bin/move_base src/move_base.cpp)
rosbuild_link_boost(bin/move_base system)
rosbuild_add_executable(bin/pose_pub src/pose_pub.cpp)

# each_turtlebot_node
rosbuild_add_executable(bin/teleop_node_conveyor src/teleop_node_conveyor.cpp)
rosbuild_add_executable(bin/npc_node_conveyor src/npc_node_conveyor.cpp)
rosbuild_add_executable(bin/random_goals_publisher src/random_goals_publisher.cpp)
