#
# (C) Copyright 2020-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (dvrk_arms_from_ros VERSION 2.3.0)


# first test for ROS1
set (ROS1_depends cisst_ros_crtk roscpp roslib)

find_package (catkin QUIET
  COMPONENTS
  ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
  catkin_package (
    INCLUDE_DIRS include
    LIBRARIES dvrk_arm_from_ros
    CATKIN_DEPENDS ${ROS1_depends})
  include_directories (${catkin_INCLUDE_DIRS})
else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS_VERSION 2)
    set (ROS2_depends rclcpp
      std_msgs std_srvs
      geometry_msgs sensor_msgs diagnostic_msgs
      tf2_ros tf2_msgs
      cisst_msgs crtk_msgs
      cisst_ros_bridge cisst_ros_crtk
      )
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

# find cisst and make sure the required libraries have been compiled
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonXML
  cisstCommonQt
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  )

find_package (cisst 1.3.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

    # load cisst configuration
  include (${CISST_USE_FILE})

  # set paths
  cisst_set_output_path ()

  include_directories (
    ${dvrk_arms_from_ros_SOURCE_DIR}/include
    ${CATKIN_DEVEL_PREFIX}/include
    )

  add_library (dvrk_arm_from_ros
    include/dvrk_arm_from_ros.h
    include/dvrk_psm_from_ros.h
    src/dvrk_arm_from_ros.cpp
    src/dvrk_psm_from_ros.cpp)

  # target_link_libraries (
  #  dvrk_arm_from_ros
  #  ${catkin_LIBRARIES}
  #  )
  cisst_target_link_libraries (dvrk_arm_from_ros ${REQUIRED_CISST_LIBRARIES})

  install (TARGETS dvrk_arm_from_ros
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include)

  if (ROS_VERSION EQUAL 2)
    ament_target_dependencies (
      dvrk_arm_from_ros
      ${ROS2_depends}
      )
    ament_package ()
  endif ()

endif (cisst_FOUND_AS_REQUIRED)
