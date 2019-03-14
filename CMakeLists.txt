cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
    cfg/segmentation.cfg
    cfg/crop.cfg
    cfg/downsample.cfg
    cfg/statistical_removal.cfg
)

catkin_package(CATKIN_DEPENDS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs

  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(
  include	
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(stat_remove src/statistical_removal.cpp)
target_link_libraries(stat_remove ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(crop src/crop.cpp)
target_link_libraries(crop ${catkin_LIBRARIES} ${PCL_LIBARARIES})

add_executable(downsample src/downsample.cpp)
target_link_libraries(downsample ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(segment src/segmentation.cpp)
target_link_libraries(segment ${catkin_LIBRARIES} ${PCL_LIBARARIES})