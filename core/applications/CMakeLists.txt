#
# (C) Copyright 2009-2025 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.16)
project (sawIntuitiveResearchKitApplications VERSION 2.3.0)

project (sawIntuitiveResearchKitApplications)

# applications in separate directories
add_subdirectory (gripper-calibration)
add_subdirectory (si-potentiometers-calibration)
add_subdirectory (config-generator)
add_subdirectory (sd-card-updater)
add_subdirectory (remove-logs)
add_subdirectory (suj-bluetooth-reset)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonXML
  cisstCommonQt
  cisstVector
  cisstVectorQt
  cisstNumerical
  cisstRobot
  cisstOSAbstraction
  cisstMultiTask
  cisstMultiTaskQt
  cisstParameterTypes
  cisstParameterTypesQt
  cisstQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.3.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawIntuitiveResearchKitExamples_IS_CATKIN_BUILT)
  if (sawIntuitiveResearchKitExamples_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # saw components have been compiled within cisst, we should find them automatically
  find_package (sawRobotIO1394          2.3.0 REQUIRED)
  find_package (sawControllers          2.2.0 REQUIRED)
  find_package (sawIntuitiveResearchKit 2.3.0 REQUIRED)
  find_package (sawTextToSpeech         1.4.0 REQUIRED)

  if (sawRobotIO1394_FOUND AND sawControllers_FOUND
      AND sawIntuitiveResearchKit_FOUND AND sawTextToSpeech_FOUND)

    # saw components configuration
    include_directories (${sawRobotIO1394_INCLUDE_DIR}
                         ${sawIntuitiveResearchKit_INCLUDE_DIR}
                         ${sawControllers_INCLUDE_DIR}
                         ${sawConstrainController_INCLUDE_DIR}
                         ${sawTextToSpeech_INCLUDE_DIR})

    link_directories (${sawRobotIO1394_LIBRARY_DIR}
                      ${sawIntuitiveResearchKit_LIBRARY_DIR}
                      ${sawControllers_LIBRARY_DIR}
                      ${sawTextToSpeech_LIBRARY_DIR})

    # examples using Qt
    if (CISST_HAS_QT)

      # main program used to configure PID
      add_executable (sawIntuitiveResearchKitPID main_PID.cpp)
      set_target_properties (sawIntuitiveResearchKitPID PROPERTIES
                             COMPONENT sawIntuitiveResearchKit-Applications
                             FOLDER "sawIntuitiveResearchKit")
      # link against non cisst libraries and cisst components
      target_link_libraries (sawIntuitiveResearchKitPID
                             ${sawIntuitiveResearchKit_LIBRARIES}
                             ${sawRobotIO1394_LIBRARIES}
                             ${sawControllers_LIBRARIES})
      # link against cisst libraries (and dependencies)
      cisst_target_link_libraries (sawIntuitiveResearchKitPID ${REQUIRED_CISST_LIBRARIES})

      install (TARGETS sawIntuitiveResearchKitPID
               COMPONENT sawIntuitiveResearchKit-Applications
               RUNTIME DESTINATION bin
               LIBRARY DESTINATION lib
               ARCHIVE DESTINATION lib)

      # applications using JSON
      if (CISST_HAS_JSON)

        # main program used to configure Console
        add_executable (sawIntuitiveResearchKitSystem main_system.cpp)
        set_target_properties (sawIntuitiveResearchKitSystem PROPERTIES
                               COMPONENT sawIntuitiveResearchKit-Applications
                               FOLDER "sawIntuitiveResearchKit")
        # link against non cisst libraries and cisst components
        target_link_libraries (sawIntuitiveResearchKitSystem
                               ${sawIntuitiveResearchKit_LIBRARIES}
                               ${sawRobotIO1394_LIBRARIES}
                               ${sawControllers_LIBRARIES}
                               ${sawTextToSpeech_LIBRARIES})
        # link against cisst libraries (and dependencies)
        cisst_target_link_libraries (sawIntuitiveResearchKitSystem ${REQUIRED_CISST_LIBRARIES})

        install (TARGETS sawIntuitiveResearchKitSystem
                 COMPONENT sawIntuitiveResearchKit-Applications
                 RUNTIME DESTINATION bin
                 LIBRARY DESTINATION lib
                 ARCHIVE DESTINATION lib)

      endif (CISST_HAS_JSON)

    endif (CISST_HAS_QT)

  endif () # components found

endif (cisst_FOUND_AS_REQUIRED)
