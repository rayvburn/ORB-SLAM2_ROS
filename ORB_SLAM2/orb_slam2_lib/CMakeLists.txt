cmake_minimum_required(VERSION 2.8)
project(orb_slam2_lib)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "Using flag -std=c++0x.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

find_package(OpenCV 3.1.0 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED)
catkin_package()

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

set(SRCS
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/IMapPublisher.cc
  src/DefaultImpls.cc
  src/IPublisherThread.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
  src/bin_vocabulary.cc
)

set(LIBS
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  DBoW2
  g2o
)

add_library(${PROJECT_NAME} SHARED ${SRCS})
target_link_libraries(${PROJECT_NAME} ${LIBS})

# Build tools - https://github.com/poine/ORB_SLAM2
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
add_executable(bin_vocabulary
${PROJECT_SOURCE_DIR}/src/bin_vocabulary.cc)
target_link_libraries(bin_vocabulary ${PROJECT_NAME})
