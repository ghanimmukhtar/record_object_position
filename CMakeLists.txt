cmake_minimum_required(VERSION 2.8.3)
project(record_object_position)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  moveit_ros_planning_interface
  image_geometry
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

find_package(OpenCV REQUIRED COMPONENTS features2d nonfree)

# EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS system )
find_package(PCL 1.7 REQUIRED)

include_directories(/usr/local/include)
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_path(RGBD_UTILS_INCLUDE_DIR rgbd_utils/rgbd_subscriber.hpp)
find_library(RGBD_UTILS_LIBRARY NAMES rgbd_utils)

find_path(IMAGE_PROCESSING_INCLUDE_DIR image_processing/MotionDetection.h)
find_library(IMAGE_PROCESSING_LIBRARY NAMES image_processing)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES record_object_position
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(record_object_position src/record_object_position.cpp)
target_link_libraries(record_object_position /usr/local/lib/libaruco.so aruco ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} rgbd_utils image_processing)
