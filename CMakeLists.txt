cmake_minimum_required(VERSION 2.8.3)
project(laser_mapping)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  sensor_msgs 
  genmsg 
  tf
  pcl_ros
  laser_geometry)
find_package(PCL 1.3 REQUIRED)

catkin_package()

include_directories(
# include
${catkin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(${PROJECT_NAME}_node src/laser_mapping_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

