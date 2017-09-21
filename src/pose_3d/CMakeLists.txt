cmake_minimum_required(VERSION 2.8.3)
project(pose_3d)

## Add support for C++11, supported in ROS Kinetic and newer
set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall")



find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  rospy
  sensor_msgs
  image_recognition_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy
)

include_directories(
  include

  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories( 
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/lib 
  /usr/lib 
  /usr/lib/aarch64-linux-gnu
)


add_executable(${PROJECT_NAME}_node src/pose_3d.cpp)


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}

  # libopencv_core.so
  # libopencv_highgui.so 
  # libopencv_imgproc.so 
  # libopencv_contrib.so
  # libopencv_features2d.so
  # libopencv_calib3d.so
)
