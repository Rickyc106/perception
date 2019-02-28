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
)

catkin_package(CATKIN_DEPENDS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs

  INCLUDE_DIRS include
  LIBRARIES
    crop
)

include_directories(
  include	
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(crop src/crop.cpp)
target_link_libraries(crop ${catkin_LIBRARIES} ${PCL_LIBARARIES})

add_library(segment src/segmentation.cpp)
target_link_libraries(segment ${catkin_LIBRARIES} ${PCL_LIBARARIES})

add_executable(demo src/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES} ${PCL_LIBARARIES} crop segment)

add_dependencies(segment ${PROJECT_NAME}_gencfg)