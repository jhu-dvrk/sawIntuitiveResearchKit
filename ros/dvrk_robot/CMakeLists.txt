#
# (C) Copyright 2015-2025 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.16)
project (dvrk_robot VERSION 2.4.0)

# first test for ROS1
set (ROS1_depends cisst_ros_crtk saw_robot_io_1394_ros saw_controllers_ros roscpp roslib)

find_package (catkin QUIET
  COMPONENTS
  ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
  catkin_package (
    INCLUDE_DIRS include
    LIBRARIES dvrk_utilities
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
      saw_robot_io_1394_ros saw_controllers_ros
      )
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

# find cisst and make sure the required libraries have been compiled
set (
  REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonXML
  cisstCommonQt
  cisstVector
  cisstVectorQt
  cisstOSAbstraction
  cisstMultiTask
  cisstMultiTaskQt
  cisstParameterTypes
  cisstParameterTypesQt
  cisstRobot
  cisstNumerical
  cisstQt
  )

find_package (cisst 1.4.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # set paths
  cisst_set_output_path ()

  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  find_package (sawRobotIO1394          2.4.0 REQUIRED)
  find_package (sawControllers          2.3.0 REQUIRED)
  find_package (sawIntuitiveResearchKit 2.4.0 REQUIRED)

  include_directories (
    ${dvrk_robot_SOURCE_DIR}/include
    ${CATKIN_DEVEL_PREFIX}/include
    ${sawRobotIO1394_INCLUDE_DIR}
    ${sawControllers_INCLUDE_DIR}
    ${sawIntuitiveResearchKit_INCLUDE_DIR}
    )

  link_directories (
    ${sawRobotIO1394_LIBRARY_DIR}
    ${sawControllers_LIBRARY_DIR}
    ${sawIntuitiveResearchKit_LIBRARY_DIR}
    )

  add_executable (dvrk_pid src/dvrk_pid.cpp)
  target_link_libraries (
    dvrk_pid
    ${sawRobotIO1394_LIBRARIES}
    ${sawControllers_LIBRARIES}
    ${catkin_LIBRARIES}
    )
  cisst_target_link_libraries (dvrk_pid ${REQUIRED_CISST_LIBRARIES})


  add_library (dvrk_utilities
    include/dvrk_utilities/system_ROS.h
    src/system_ROS.cpp)

  target_link_libraries (
    dvrk_utilities
    ${sawRobotIO1394_LIBRARIES}
    ${sawControllers_LIBRARIES}
    ${sawIntuitiveResearchKit_LIBRARIES}
    ${catkin_LIBRARIES}
    )
  cisst_target_link_libraries (dvrk_utilities ${REQUIRED_CISST_LIBRARIES})

  add_executable (dvrk_system src/dvrk_system.cpp)
  target_link_libraries (
    dvrk_system
    ${sawRobotIO1394_LIBRARIES}
    ${sawControllers_LIBRARIES}
    ${sawIntuitiveResearchKit_LIBRARIES}
    ${catkin_LIBRARIES}
    )
  cisst_target_link_libraries (dvrk_system ${REQUIRED_CISST_LIBRARIES})
  target_link_libraries (dvrk_system dvrk_utilities)

  install (TARGETS dvrk_utilities dvrk_pid dvrk_system
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include)

  if (ROS_VERSION EQUAL 2)
    ament_target_dependencies (
      dvrk_utilities
      ${ROS2_depends}
      )
    ament_target_dependencies (
      dvrk_pid
      ${ROS2_depends}
      )
    ament_package ()
  endif ()

endif (cisst_FOUND_AS_REQUIRED)
