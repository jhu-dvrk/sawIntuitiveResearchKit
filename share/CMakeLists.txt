cmake_minimum_required (VERSION 3.10)
project (dvrk_config VERSION 2.2.1)

# ROS 1
find_package (catkin QUIET)
if (catkin_FOUND)
  catkin_package()
else (catkin_FOUND)

  # ROS 2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (_to_be_installed arm collection console deprecated hid igtl io kinematic pid schemas socket-streamer tool)

    foreach (_dir ${_to_be_installed})
      install (DIRECTORY
        ${_dir}
        DESTINATION share/${PROJECT_NAME})
    endforeach ()

    ament_package ()

  endif (ament_cmake_FOUND)

endif (catkin_FOUND)
