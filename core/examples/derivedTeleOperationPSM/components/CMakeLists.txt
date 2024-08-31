#
# (C) Copyright 2011-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawIntuitiveResearchKitDerivedTeleOperationPSM VERSION 2.3.0)

set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot
  cisstNumerical)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)
  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # sawIntuitiveResearchKit has been compiled within cisst, we should
  # find it automatically
  find_package (sawTextToSpeech        1.4.0 REQUIRED)
  find_package (sawRobotIO1394         2.3.0 REQUIRED)
  find_package (sawControllers         2.2.0 REQUIRED)
  find_package(sawIntuitiveResearchKit 2.3.0 REQUIRED)

  if (sawIntuitiveResearchKit_FOUND)

    include_directories (
      ${CMAKE_BINARY_DIR}
      ${sawIntuitiveResearchKit_INCLUDE_DIR})

    link_directories (
      ${sawIntuitiveResearchKit_LIBRARY_DIR}
      ${sawTextToSpeech_LIBRARY_DIR}
      ${sawRobotIO1394_LIBRARY_DIR}
      ${sawControllers_LIBRARY_DIR})

    set (sawIntuitiveResearchKitDerivedTeleOperationPSM_INCLUDE_DIR
         "${sawIntuitiveResearchKitDerivedTeleOperationPSM_SOURCE_DIR}/include")

    include_directories (${sawIntuitiveResearchKitDerivedTeleOperationPSM_INCLUDE_DIR})

    add_library (sawIntuitiveResearchKitDerivedTeleOperationPSM
                 "include/mtsDerivedTeleOperationPSM.h"
                 "code/mtsDerivedTeleOperationPSM.cpp")

    cisst_target_link_libraries (sawIntuitiveResearchKitDerivedTeleOperationPSM
                                 ${REQUIRED_CISST_LIBRARIES})

    # link against non cisst libraries and cisst components
    target_link_libraries (sawIntuitiveResearchKitDerivedTeleOperationPSM
                           ${sawIntuitiveResearchKit_LIBRARIES})

  endif (sawIntuitiveResearchKit_FOUND)

endif (cisst_FOUND)
