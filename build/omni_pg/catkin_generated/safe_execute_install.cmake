execute_process(COMMAND "/home/user/catkin_ws_pg_omni/build/omni_pg/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/user/catkin_ws_pg_omni/build/omni_pg/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
