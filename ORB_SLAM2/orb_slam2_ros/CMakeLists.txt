cmake_minimum_required(VERSION 2.8)
project(orb_slam2_ros)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

ADD_DEFINITIONS("-std=c++11")

find_package(OpenCV 3.1.0 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  orb_slam2_lib
  roscpp
  tf
  std_msgs
  sensor_msgs
  image_transport
  image_geometry
  cv_bridge
  pcl_ros
  pcl_conversions
  message_generation
  octomap_ros
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ORBState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES 
  CATKIN_DEPENDS message_runtime std_msgs
  #DEPENDS 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${octomap_ros_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib/include
)

add_library(PublisherUtils
 ${PROJECT_SOURCE_DIR}/include/PublisherUtils.h
 ${PROJECT_SOURCE_DIR}/src/PublisherUtils.cpp
 ${PROJECT_SOURCE_DIR}/include/PublisherUtils_impl.h
)

add_dependencies(PublisherUtils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(ScaleCorrector
 ${PROJECT_SOURCE_DIR}/include/ScaleCorrector.h
 ${PROJECT_SOURCE_DIR}/src/ScaleCorrector.cpp
)

add_dependencies(ScaleCorrector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(ROSSystemBuilder
 ${PROJECT_SOURCE_DIR}/include/ROSSystemBuilder.h
 ${PROJECT_SOURCE_DIR}/src/ROSSystemBuilder.cpp
)

add_dependencies(ROSSystemBuilder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(LIBS
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${octomap_ros_LIBRARIES}
  orb_slam2_lib
  PublisherUtils
  ScaleCorrector
  ROSSystemBuilder
)

# Node for monocular camera
add_executable(Mono
  src/ros_mono.cc
  src/ROSPublisher.cpp
)

add_dependencies(Mono ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(Mono
  ${LIBS}
)

# Node for monocular camera (Augmented Reality Demo)
#add_executable(MonoAR
#src/AR/ros_mono_ar.cc
#include/AR/ViewerAR.h
#src/AR/ViewerAR.cc
#)
#
#target_link_libraries(MonoAR
#${LIBS}
#)

# Node for stereo camera
add_executable(Stereo
  src/ros_stereo.cc
  src/ROSPublisher.cpp
)

add_dependencies(Stereo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(Stereo
  ${LIBS}
)

# Node for RGB-D camera
add_executable(RGBD
  src/ros_rgbd.cc
  src/ROSPublisher.cpp
)

add_dependencies(RGBD ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(RGBD
  ${LIBS}
)
