#
# (C) Copyright 2011-2023 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required(VERSION 3.10)
project (saw_intuitive_research_kit_example_psm_derived VERSION 2.2.1)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical
     cisstQt)

find_package (catkin REQUIRED
              COMPONENTS
              cisst_ros_crtk
              roscpp
              roslib
              )

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)
  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (saw_intuitive_research_kit_example_psm_derived_IS_CATKIN_BUILT)
  if (saw_intuitive_research_kit_example_psm_derived_IS_CATKIN_BUILT)
    set (LIBRARY_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/lib")
  endif ()

  # sawIntuitiveResearchKit has been compiled within cisst, we should
  # find it automatically
  find_package(sawIntuitiveResearchKit 2.2.0 REQUIRED)

  if (sawIntuitiveResearchKit_FOUND)

    # for Qt widgets
    set (CMAKE_AUTOMOC ON)

    file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")
    catkin_package (INCLUDE_DIRS include "${CATKIN_DEVEL_PREFIX}/include"
      LIBRARIES saw_intuitive_research_kit_example_psm_derived
      CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)

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

    cisst_target_link_libraries (sawIntuitiveResearchKitDerivedPSM
      ${REQUIRED_CISST_LIBRARIES})

    # link against non cisst libraries and cisst components
    target_link_libraries (sawIntuitiveResearchKitDerivedPSM
      ${sawIntuitiveResearchKit_LIBRARIES}
      ${catkin_LIBRARIES})

  endif (sawIntuitiveResearchKit_FOUND)

endif (cisst_FOUND)
