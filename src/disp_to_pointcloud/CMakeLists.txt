cmake_minimum_required(VERSION 2.8.3)
project(disp_to_pointcloud)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package( PCL REQUIRED )
find_package( OpenCV REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS cv_bridge cv_bridge image_transport pcl_ros roscpp sensor_msgs std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable (disp_to_pointcloud_node src/disp_to_pointcloud.cpp)
target_link_libraries( disp_to_pointcloud_node
  ${catkin_LIBRARIES}
  ${PCL_LIBS}
  ${OPENCV_LIBS}
)


