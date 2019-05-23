# Install script for directory: /home/dzf/catch_bottle-master

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/dzf/catch_bottle-master/cmake-build-debug/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/dzf/catch_bottle-master/cmake-build-debug/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/dzf/catch_bottle-master/cmake-build-debug/catkin_generated/installspace/setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/dzf/catch_bottle-master/cmake-build-debug/catkin_generated/installspace/setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/dzf/catch_bottle-master/cmake-build-debug/catkin_generated/installspace/setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/dzf/catch_bottle-master/cmake-build-debug/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/dzf/catch_bottle-master/cmake-build-debug/gtest/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/robotiq_85_gripper-master/robotiq_85_driver/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/robotiq_85_gripper-master/robotiq_85_gripper/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/universal_robot/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur_description/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur_e_description/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur_msgs/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/cortex_bridge/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/robotiq_85_gripper-master/robotiq_85_bringup/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/robotiq_85_gripper-master/robotiq_85_description/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/robotiq_85_gripper-master/robotiq_85_msgs/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/robotiq_85_gripper-master/si_utils/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur_bringup/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur_driver/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur_e_gazebo/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur_gazebo/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/ur_modern_driver/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/ur5e_robot_ikfast_manipulator_plugin/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur_kinematics/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/catch_control/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/UR5e_robotiq_moveit_config/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/robotiq_85_gripper-master/robotiq_85_moveit_config/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur10_e_moveit_config/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur10_moveit_config/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur3_e_moveit_config/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur3_moveit_config/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur5_e_moveit_config/cmake_install.cmake")
  include("/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur5_moveit_config/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/dzf/catch_bottle-master/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
