#
# (C) Copyright 2023-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.16)
project (sawIntuitiveResearchKitSDCardUpdater VERSION 2.4.0)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.4.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  set (sawIntuitiveResearchKitSDCardUpdater_FILES
    ${sawIntuitiveResearchKitSDCardUpdater_SOURCE_DIR}/dvrk-sd-card-updater.py)

  add_custom_target (sawIntuitiveResearchKitSDCardUpdater ALL)
  foreach (_file ${sawIntuitiveResearchKitSDCardUpdater_FILES})
    add_custom_command (TARGET sawIntuitiveResearchKitSDCardUpdater PRE_BUILD
      COMMAND ${CMAKE_COMMAND} -E
      copy ${_file} ${EXECUTABLE_OUTPUT_PATH})
  endforeach ()

  install (
    FILES ${sawIntuitiveResearchKitSDCardUpdater_FILES}
    COMPONENT sawIntuitiveResearchKit-Applications
    DESTINATION bin)

endif (cisst_FOUND_AS_REQUIRED)
