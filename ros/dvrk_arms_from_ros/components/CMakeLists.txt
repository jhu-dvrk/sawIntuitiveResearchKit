#
# (C) Copyright 2020-2023 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (dvrk_arms_from_ros VERSION 2.2.1)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

## find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED
              COMPONENTS
              cisst_ros_crtk
              roscpp
              roslib
              )

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

find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  include (${CISST_USE_FILE})

  # include_directories(include)
  include_directories (${catkin_INCLUDE_DIRS})

  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

  catkin_package (INCLUDE_DIRS include "${CATKIN_DEVEL_PREFIX}/include"
                  LIBRARIES dvrk_arm_from_ros
                  CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)

  include_directories (
    ${dvrk_arms_from_ros_SOURCE_DIR}/include
    ${CATKIN_DEVEL_PREFIX}/include
    )

  add_library (dvrk_arm_from_ros
               include/dvrk_arm_from_ros.h
               include/dvrk_psm_from_ros.h
               src/dvrk_arm_from_ros.cpp
               src/dvrk_psm_from_ros.cpp)

  target_link_libraries (
    dvrk_arm_from_ros
    ${catkin_LIBRARIES}
    )
    cisst_target_link_libraries (dvrk_arm_from_ros ${REQUIRED_CISST_LIBRARIES})

endif (cisst_FOUND_AS_REQUIRED)
