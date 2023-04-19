#
# (C) Copyright 2023 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawIntuitiveResearchKitConfigGenerator VERSION 2.1.0)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.1.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawIntuitiveResearchKitConfigGenerator_IS_CATKIN_BUILT)
  if (sawIntuitiveResearchKitConfigGenerator_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  set (sawIntuitiveResearchKitConfigGenerator_FILES
    ${sawIntuitiveResearchKitConfigGenerator_SOURCE_DIR}/calParser.py
    ${sawIntuitiveResearchKitConfigGenerator_SOURCE_DIR}/dvrk-config-generator.py)

  add_custom_target (sawIntuitiveResearchKitConfigGenerator ALL)
  foreach (_file ${sawIntuitiveResearchKitConfigGenerator_FILES})
    add_custom_command (TARGET sawIntuitiveResearchKitConfigGenerator PRE_BUILD
      COMMAND ${CMAKE_COMMAND} -E
      copy ${_file} ${EXECUTABLE_OUTPUT_PATH})
  endforeach ()

  install (
    FILES ${sawIntuitiveResearchKitConfigGenerator_FILES}
    COMPONENT sawIntuitiveResearchKit-Applications
    DESTINATION bin)

endif (cisst_FOUND_AS_REQUIRED)
