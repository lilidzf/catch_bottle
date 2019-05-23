execute_process(COMMAND "/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/dzf/catch_bottle-master/cmake-build-debug/universal_robot-ur_e/ur_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
