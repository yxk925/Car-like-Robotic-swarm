cmake_minimum_required(VERSION 2.8.3)
project(decomp_ros_msgs)

set(CMAKE_CXX_FLAGS "-std=c++17 -g")

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_install()

cs_export()
