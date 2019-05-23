cmake_minimum_required(VERSION 2.8.3)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

project(catch_control)
#add_compile_options(-std=c++11)
set(CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH} $ENV{QT5_INSTALL_PATH})

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  sensor_msgs
  trac_ik_lib
  trajectory_msgs
  rosbag
        message_generation
        robotiq_85_msgs
        eigen_conversions
        eigen_stl_containers
        eigen_typekit
        geometry_msgs
        moveit_ros_planning_interface

)


catkin_package(

  CATKIN_DEPENDS
        robotiq_85_msgs

)



include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIR})

add_executable(test2 src/catch_bottle.cpp src/catch_bottle.h)
target_link_libraries(test2 ${catkin_LIBRARIES})

add_executable(mymoveit_node src/mymoveit.cpp src/mymoveit.h)
add_dependencies(mymoveit_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mymoveit_node ${catkin_LIBRARIES})


add_executable(joint_state_maintain_node src/joint_state_maintain.cpp src/joint_state_maintain.h)
add_dependencies(joint_state_maintain_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_state_maintain_node ${catkin_LIBRARIES})

add_executable(plan src/plan.cpp)
target_link_libraries(plan ${catkin_LIBRARIES})

