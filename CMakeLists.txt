cmake_minimum_required(VERSION 2.8.3)
project(autonomous_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  rospy
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS  roscpp
                  pcl_conversions
                  pcl_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lidar_filtering src/lidar_filtering.cpp)
target_link_libraries(lidar_filtering ${catkin_LIBRARIES})