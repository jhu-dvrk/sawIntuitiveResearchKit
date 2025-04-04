#
# CMakeLists for sawIntuitiveResearchKit tests
#
# (C) Copyright 2019-2023 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required(VERSION 3.16)
project (sawIntuitiveResearchKitTests VERSION 2.3.0)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstNumerical
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot
  cisstTestsDriver)

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawIntuitiveResearchKitTests_IS_CATKIN_BUILT)
  if (sawIntuitiveResearchKitTests_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # sawIntuitiveResearchKit has been compiled within cisst, we should find it automatically
  find_package (sawIntuitiveResearchKit REQUIRED)

  if (sawIntuitiveResearchKit_FOUND)

    # sawIntuitiveResearchKit configuration
    include_directories (${sawIntuitiveResearchKit_INCLUDE_DIR})
    link_directories (${sawIntuitiveResearchKit_LIBRARY_DIR})

    add_executable (sawIntuitiveResearchKitTests
      robManipulatorTest.cpp
      robManipulatorTest.h)

    set_property (TARGET sawIntuitiveResearchKitTests PROPERTY FOLDER "sawIntuitiveResearchKit")

    # link against non cisst libraries and cisst components
    target_link_libraries (sawIntuitiveResearchKitTests
                           ${sawIntuitiveResearchKit_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawIntuitiveResearchKitTests ${REQUIRED_CISST_LIBRARIES})

  endif (sawIntuitiveResearchKit_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
