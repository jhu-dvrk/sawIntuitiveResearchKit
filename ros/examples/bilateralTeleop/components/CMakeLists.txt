#
# (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required(VERSION 3.10)
project (saw_intuitive_research_kit_example_bilateral_teleop VERSION 2.3.0)

# first test for ROS1
set (ROS1_depends
  cisst_ros_crtk roscpp roslib)

find_package (catkin QUIET
  COMPONENTS
  ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")
  catkin_package (INCLUDE_DIRS include "${CATKIN_DEVEL_PREFIX}/include"
    LIBRARIES saw_intuitive_research_kit_example_bilateral_teleop
    CATKIN_DEPENDS ${ROS1_depends})
  include_directories (${catkin_INCLUDE_DIRS})
else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS_VERSION 2)
    set (ROS2_depends
      rclcpp
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
set (
  REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot
  cisstNumerical)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)
  # load cisst configuration
  include (${CISST_USE_FILE})

  # set paths
  cisst_set_output_path ()

  # sawIntuitiveResearchKit has been compiled within cisst, we should
  # find it automatically
  find_package(sawIntuitiveResearchKit 2.3.0 REQUIRED)

  if (sawIntuitiveResearchKit_FOUND)

    set (saw_intuitive_research_kit_example_bilateral_teleop_INCLUDE_DIR
      "${saw_intuitive_research_kit_example_bilateral_teleop_SOURCE_DIR}/include")

    include_directories (
      ${saw_intuitive_research_kit_example_bilateral_teleop_INCLUDE_DIR}
      ${CMAKE_BINARY_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}
      ${sawIntuitiveResearchKit_INCLUDE_DIR}
      ${catkin_INCLUDE_DIRS})

    link_directories (
      ${sawIntuitiveResearchKit_LIBRARY_DIR})

    add_library (sawIntuitiveResearchKitBilateralTeleop
      "include/mtsBilateralTeleOperationPSMROSBridge.h"
      "include/mtsBilateralTeleOperationPSM.h"
      "code/mtsBilateralTeleOperationPSMROSBridge.cpp"
      "code/mtsBilateralTeleOperationPSM.cpp"
    )

    cisst_target_link_libraries (sawIntuitiveResearchKitBilateralTeleop
      ${REQUIRED_CISST_LIBRARIES})

    # link against non cisst libraries and cisst components
    target_link_libraries (sawIntuitiveResearchKitBilateralTeleop
      ${sawIntuitiveResearchKit_LIBRARIES}
      ${catkin_LIBRARIES}
      )

    if (ROS_VERSION EQUAL 2)
      ament_target_dependencies (
        sawIntuitiveResearchKitBilateralTeleop
        ${ROS2_depends}
        )
      ament_package ()
    endif ()

  endif (sawIntuitiveResearchKit_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
