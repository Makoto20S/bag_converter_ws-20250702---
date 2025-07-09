cmake_minimum_required(VERSION 3.0.2)
project(bag_converter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  # message_generation 
  roslib
)

find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# add_message_files(
#   FILES
#   cloud_info.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
#   geometry_msgs
# )

# 在 include_directories 中添加 include 文件夹
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include  # 添加这一行
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    message_filters
    tf2
    tf2_ros
    tf2_geometry_msgs
    pcl_ros
    pcl_conversions
    # message_runtime  
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  src  # 添加src目录到包含路径，这样可以找到头文件
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(bag_to_cloud_info_node src/main.cpp src/bag_to_cloud_info.cpp)
target_link_libraries(bag_to_cloud_info_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})