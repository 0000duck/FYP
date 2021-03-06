cmake_minimum_required(VERSION 2.8.3)
project(sgbm_ros)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  image_recognition_msgs
  depth_image_proc
  image_transport
  cv_bridge
  #pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package( OpenCV REQUIRED )
find_package( CUDA REQUIRED )
#find_package( PCL REQUIRED)
catkin_package(
   CATKIN_DEPENDS roscpp rospy
)

include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3 -lineinfo
    -gencode=arch=compute_30,code=sm_30
    -gencode=arch=compute_35,code=sm_35
    -gencode=arch=compute_50,code=sm_50
    -gencode=arch=compute_52,code=sm_52
    -gencode=arch=compute_62,code=sm_62
    )

cuda_add_executable(
    sgbm_ros_node
    #src/main.cu 
    src/sgbm_ros.cu src/median_filter.cu src/hamming_cost.cu src/disparity_method.cu src/debug.cu src/costs.cu)

target_link_libraries( sgbm_ros_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
)
