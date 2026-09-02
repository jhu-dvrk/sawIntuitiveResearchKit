#
# (C) Copyright 2011-2026 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required(VERSION 3.16)
project (saw_intuitive_research_kit_example_psm_derived VERSION 2.5.0)

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
    LIBRARIES saw_intuitive_research_kit_example_psm_derived
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
  cisstNumerical
)

find_package (cisst 1.5 REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# set paths
cisst_set_output_path ()

# sawIntuitiveResearchKit has been compiled within cisst, we should
# find it automatically
find_package(sawIntuitiveResearchKit 2.5.0 REQUIRED)

# use the Qt major version selected by cisst
if (NOT (CISST_QT_VERSION STREQUAL "6" OR CISST_QT_VERSION STREQUAL "5"))
  message (FATAL_ERROR "CISST_QT_VERSION must be 6 or 5 for ${PROJECT_NAME}")
endif ()
find_package (Qt${CISST_QT_VERSION} REQUIRED COMPONENTS Widgets Core)

if (sawIntuitiveResearchKit_FOUND)

  # for Qt widgets
  set (CMAKE_AUTOMOC ON)
  set (CMAKE_INCLUDE_CURRENT_DIR ON)

  set (saw_intuitive_research_kit_example_psm_derived_INCLUDE_DIR
    "${saw_intuitive_research_kit_example_psm_derived_SOURCE_DIR}/include")

  include_directories (
    ${saw_intuitive_research_kit_example_psm_derived_INCLUDE_DIR}
    ${CMAKE_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${sawIntuitiveResearchKit_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS})

  link_directories (
    ${sawIntuitiveResearchKit_LIBRARY_DIR})

  add_library (sawIntuitiveResearchKitDerivedPSM
    "include/mtsDerivedPSM.h"
    "include/mtsDerivedPSMQtWidget.h"
    "include/mtsDerivedPSMROSBridge.h"
    "code/mtsDerivedPSM.cpp"
    "code/mtsDerivedPSMQtWidget.cpp"
    "code/mtsDerivedPSMROSBridge.cpp")

  target_link_libraries (sawIntuitiveResearchKitDerivedPSM
    ${cisst_LIBRARIES}
    ${sawIntuitiveResearchKit_LIBRARIES}
    Qt${CISST_QT_VERSION}::Core
    Qt${CISST_QT_VERSION}::Widgets
    ${catkin_LIBRARIES})

  if (ROS_VERSION EQUAL 2)
    foreach (_dep ${ROS2_depends})
      target_link_libraries (sawIntuitiveResearchKitDerivedPSM ${${_dep}_TARGETS})
    endforeach ()
    ament_package ()
  endif ()

endif (sawIntuitiveResearchKit_FOUND)
