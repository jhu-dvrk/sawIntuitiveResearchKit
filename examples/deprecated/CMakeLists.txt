#
# (C) Copyright 2009-2017 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonXML
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
find_package (cisst 1.0.9 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawIntuitiveResearchKitExamples_IS_CATKIN_BUILT)
  if (sawIntuitiveResearchKitExamples_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # saw components have been compiled within cisst, we should find them automatically
  find_package (sawRobotIO1394          1.5.0 REQUIRED)
  find_package (sawControllers          1.5.0 REQUIRED)
  find_package (sawIntuitiveResearchKit 1.5.0 REQUIRED)
  find_package (sawTextToSpeech REQUIRED)

  if (sawRobotIO1394_FOUND AND sawControllers_FOUND
      AND sawIntuitiveResearchKit_FOUND AND sawTextToSpeech_FOUND)

    # saw components configuration
    include_directories (${sawRobotIO1394_INCLUDE_DIR}
                         ${sawIntuitiveResearchKit_INCLUDE_DIR}
                         ${sawControllers_INCLUDE_DIR}
                         ${sawTextToSpeech_INCLUDE_DIR})

    link_directories (${sawRobotIO1394_LIBRARY_DIR}
                      ${sawIntuitiveResearchKit_LIBRARY_DIR}
                      ${sawControllers_LIBRARY_DIR}
                      ${sawTextToSpeech_LIBRARY_DIR})

    # examples using Qt
    if (CISST_HAS_QT)

      # main program used to configure TeleOperation --- TO BE DEPRECATED
      add_executable (sawIntuitiveResearchKitQtTeleOperation mainQtTeleOperation.cpp)
      set_property (TARGET sawIntuitiveResearchKitQtTeleOperation PROPERTY FOLDER "sawIntuitiveResearchKit")
      # link against non cisst libraries and cisst components
      target_link_libraries (sawIntuitiveResearchKitQtTeleOperation
                             ${sawRobotIO1394_LIBRARIES}
                             ${sawIntuitiveResearchKit_LIBRARIES}
                             ${sawControllers_LIBRARIES}
                             ${sawTextToSpeech_LIBRARIES})
      # link against cisst libraries (and dependencies)
      cisst_target_link_libraries (sawIntuitiveResearchKitQtTeleOperation ${REQUIRED_CISST_LIBRARIES})

      # main program used to configure Haptics
      add_executable (sawIntuitiveResearchKitQtHaptics mainQtHaptics.cpp)
      set_property (TARGET sawIntuitiveResearchKitQtHaptics PROPERTY FOLDER "sawIntuitiveResearchKit")
      # link against non cisst libraries and cisst components
      target_link_libraries (sawIntuitiveResearchKitQtHaptics
                             ${sawRobotIO1394_LIBRARIES}
                             ${sawIntuitiveResearchKit_LIBRARIES}
                             ${sawControllers_LIBRARIES}
                             ${sawTextToSpeech_LIBRARIES})
      # link against cisst libraries (and dependencies)
      cisst_target_link_libraries (sawIntuitiveResearchKitQtHaptics ${REQUIRED_CISST_LIBRARIES})

      # example using JSON
      if (CISST_HAS_JSON)

        add_executable (sawIntuitiveResearchKitQtTeleOperationJSON
                         mainQtTeleOperationJSON.cpp)
        set_property (TARGET sawIntuitiveResearchKitQtTeleOperationJSON PROPERTY FOLDER "sawIntuitiveResearchKit")
        # link against non cisst libraries and cisst components
        target_link_libraries (sawIntuitiveResearchKitQtTeleOperationJSON
                               ${sawRobotIO1394_LIBRARIES}
                               ${sawIntuitiveResearchKit_LIBRARIES}
                               ${sawControllers_LIBRARIES}
                               ${sawTextToSpeech_LIBRARIES})
        # link against cisst libraries (and dependencies)
        cisst_target_link_libraries (sawIntuitiveResearchKitQtTeleOperationJSON ${REQUIRED_CISST_LIBRARIES})

        # main program used to configure TeleOperation
        qt_wrap_cpp (sawIntuitiveResearchKitQtUDPServer sawIntuitiveResearchKitQtUDPServer_QT_WRAP mainQtUDPServer.h)
        add_executable (sawIntuitiveResearchKitQtUDPServer
                        mainQtUDPServer.cpp
                        ${sawIntuitiveResearchKitQtUDPServer_QT_WRAP})
        set_property (TARGET sawIntuitiveResearchKitQtUDPServer PROPERTY FOLDER "sawIntuitiveResearchKit")
        # link against cisst libraries (and dependencies)
        cisst_target_link_libraries (sawIntuitiveResearchKitQtUDPServer ${REQUIRED_CISST_LIBRARIES})

      endif (CISST_HAS_JSON)

    endif (CISST_HAS_QT)

  endif () # components found

endif (cisst_FOUND_AS_REQUIRED)
