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
  control_msgs
  qfy_dynamixel
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
 add_dependencies(fly_shape fly_shape_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
 target_link_libraries(fly_shape
   ${catkin_LIBRARIES}
   fly_line
 )


 add_library(qfy_hover
   src/qfy_hover.cpp
 )

 add_executable(qfy_hover_node src/qfy_hover_node.cpp)
 add_dependencies(qfy_hover_node fly_shape_generate_messages_cpp ${catkin_EXPORTED_TARGETS}) 
 target_link_libraries(qfy_hover_node
   ${catkin_LIBRARIES}
   qfy_hover
 )

add_executable(qfy_test src/qfy_test.cpp)
add_dependencies(qfy_test fly_shape_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
 target_link_libraries(qfy_test
   ${catkin_LIBRARIES}
 )

add_executable(qfy_tango src/qfy_tango.cpp)
add_dependencies(qfy_tango ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(qfy_tango
   ${catkin_LIBRARIES}
 )

add_executable(uav_track src/uav_track.cpp)
add_dependencies(uav_track ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(uav_track
   ${catkin_LIBRARIES}
 )

install(PROGRAMS
  scripts/srv_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
