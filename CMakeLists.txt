cmake_minimum_required(VERSION 3.0.2)
project(distance_map_layer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

# set(CMAKE_CXX_CLANG_TIDY
#   clang-tidy;
#   -header-filter=${PROJECT_SOURCE_DIR}/include/distance_map_layer;
#   -warnings-as-errors=*;
# )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  roscpp
)

# Glog
find_package(glog REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  message(STATUS ${OpenCV_INCLUDE_DIRS})
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES distance_map_layer
  CATKIN_DEPENDS costmap_2d roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(distance_map_layer src/distance_map_layer.cpp)
target_link_libraries(distance_map_layer ${catkin_LIBRARIES} ${OpenCV_LIBS} glog::glog)
