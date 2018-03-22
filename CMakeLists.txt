cmake_minimum_required(VERSION 2.8.3)
project(loam_kitti)

#SET( CMAKE_CXX_COMPILER "g++") #设定编译器

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# 增加PCL库的依赖
FIND_PACKAGE( PCL 1.2 REQUIRED )

# 增加opencv的依赖
FIND_PACKAGE( OpenCV  REQUIRED )

FIND_PACKAGE( Eigen3  REQUIRED )










INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include )




ADD_EXECUTABLE(loam_kitti src/main.cpp)
TARGET_LINK_LIBRARIES( loam_kitti
${OpenCV_LIBS} 
${PCL_LIBRARIES}

)

