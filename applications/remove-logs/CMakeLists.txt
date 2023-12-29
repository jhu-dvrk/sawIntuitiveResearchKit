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
project (sawIntuitiveResearchKitRemoveLogs VERSION 2.2.1)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.2.1 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  set (sawIntuitiveResearchKitRemoveLogs_FILES
    ${sawIntuitiveResearchKitRemoveLogs_SOURCE_DIR}/dvrk-remove-logs.py)

  add_custom_target (sawIntuitiveResearchKitRemoveLogs ALL)
  foreach (_file ${sawIntuitiveResearchKitRemoveLogs_FILES})
    add_custom_command (TARGET sawIntuitiveResearchKitRemoveLogs PRE_BUILD
      COMMAND ${CMAKE_COMMAND} -E
      copy ${_file} ${EXECUTABLE_OUTPUT_PATH})
  endforeach ()

  install (
    FILES ${sawIntuitiveResearchKitRemoveLogs_FILES}
    COMPONENT sawIntuitiveResearchKit-Applications
    DESTINATION bin)

endif (cisst_FOUND_AS_REQUIRED)
