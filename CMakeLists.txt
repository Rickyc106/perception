cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  dynamic_reconfigure
  message_generation
)

generate_dynamic_reconfigure_options(
    cfg/crop.cfg
    cfg/downsample.cfg
    cfg/segmentation.cfg
    cfg/statistical_removal.cfg
    cfg/classify.cfg
)

add_message_files(
  FILES
  vector.msg
  indices.msg
  PointCloudArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  message_runtime

  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(
  include	
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(crop src/crop.cpp)
target_link_libraries(crop ${catkin_LIBRARIES} ${PCL_LIBARARIES})

add_executable(downsample src/downsample.cpp)
target_link_libraries(downsample ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(segment src/segmentation.cpp)
add_dependencies(segment perception_generate_messages_cpp)
target_link_libraries(segment ${catkin_LIBRARIES} ${PCL_LIBARARIES})

add_executable(stat_remove src/statistical_removal.cpp)
target_link_libraries(stat_remove ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(classify src/classify.cpp)
target_link_libraries(classify ${catkin_LIBRARIES} ${PCL_LIBRARIES})