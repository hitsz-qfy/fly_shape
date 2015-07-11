cmake_minimum_required(VERSION 2.8.3)
project(fly_shape)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fly_shape
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(fly_line
   src/fly_line.cpp
 )

 add_executable(fly_shape src/fly_shape.cpp)
 add_dependencies(fly_shape fly_shape_generate_messages_cpp)
 target_link_libraries(fly_shape
   ${catkin_LIBRARIES}
   fly_line
 )
