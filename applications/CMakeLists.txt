#
# (C) Copyright 2009-2016 Johns Hopkins University (JHU), All Rights Reserved.
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
find_package (cisst 1.0.8 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawIntuitiveResearchKitExamples_IS_CATKIN_BUILT)
  if (sawIntuitiveResearchKitExamples_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # saw components have been compiled within cisst, we should find them automatically
  find_package (sawRobotIO1394          1.4.0 REQUIRED)
  find_package (sawControllers          1.4.0 REQUIRED)
  find_package (sawConstraintController 1.1.0 REQUIRED)
  find_package (sawIntuitiveResearchKit 1.4.0 REQUIRED)
  find_package (sawTextToSpeech REQUIRED)

  if (sawRobotIO1394_FOUND AND sawControllers_FOUND
      AND sawIntuitiveResearchKit_FOUND AND sawTextToSpeech_FOUND
      AND sawConstraintController_FOUND)

    # saw components configuration
    include_directories (${sawRobotIO1394_INCLUDE_DIR}
                         ${sawIntuitiveResearchKit_INCLUDE_DIR}
                         ${sawControllers_INCLUDE_DIR}
                         ${sawConstrainController_INCLUDE_DIR}
                         ${sawTextToSpeech_INCLUDE_DIR})

    link_directories (${sawRobotIO1394_LIBRARY_DIR}
                      ${sawIntuitiveResearchKit_LIBRARY_DIR}
                      ${sawControllers_LIBRARY_DIR}
                      ${sawConstraintController_LIBRARY_DIR}
                      ${sawTextToSpeech_LIBRARY_DIR})

    # main program used to calibrate the MTM gripper
    add_executable (sawIntuitiveResearchKitGripperCalibration GripperCalibration.cpp)
    set_property (TARGET sawIntuitiveResearchKitGripperCalibration PROPERTY FOLDER "sawIntuitiveResearchKit")

    # link against non cisst libraries and cisst components
    target_link_libraries (sawIntuitiveResearchKitGripperCalibration
                           ${sawRobotIO1394_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawIntuitiveResearchKitGripperCalibration ${REQUIRED_CISST_LIBRARIES})

    # examples using Qt
    if (CISST_HAS_QT)

      # main program used to configure PID
      add_executable (sawIntuitiveResearchKitQtPID mainQtPID.cpp)
      set_property (TARGET sawIntuitiveResearchKitQtPID PROPERTY FOLDER "sawIntuitiveResearchKit")
      # link against non cisst libraries and cisst components
      target_link_libraries (sawIntuitiveResearchKitQtPID
                             ${sawIntuitiveResearchKit_LIBRARIES}
                             ${sawRobotIO1394_LIBRARIES}
                             ${sawControllers_LIBRARIES})
      # link against cisst libraries (and dependencies)
      cisst_target_link_libraries (sawIntuitiveResearchKitQtPID ${REQUIRED_CISST_LIBRARIES})

      # main program used to test a single arm
      add_executable (sawIntuitiveResearchKitQtArm mainQtArm.cpp)
      set_property (TARGET sawIntuitiveResearchKitQtArm PROPERTY FOLDER "sawIntuitiveResearchKit")
      # link against non cisst libraries and cisst components
      target_link_libraries (sawIntuitiveResearchKitQtArm
                             ${sawIntuitiveResearchKit_LIBRARIES}
                             ${sawRobotIO1394_LIBRARIES}
                             ${sawControllers_LIBRARIES}
                             ${sawTextToSpeech_LIBRARIES})
      # link against cisst libraries (and dependencies)
      cisst_target_link_libraries (sawIntuitiveResearchKitQtArm ${REQUIRED_CISST_LIBRARIES})

      # applications using JSON
      if (CISST_HAS_JSON)

        # main program used to configure Console
        add_executable (sawIntuitiveResearchKitQtConsoleJSON mainQtConsoleJSON.cpp)
        set_property (TARGET sawIntuitiveResearchKitQtConsoleJSON PROPERTY FOLDER "sawIntuitiveResearchKit")
        # link against non cisst libraries and cisst components
        target_link_libraries (sawIntuitiveResearchKitQtConsoleJSON
                               ${sawIntuitiveResearchKit_LIBRARIES}
                               ${sawRobotIO1394_LIBRARIES}
                               ${sawControllers_LIBRARIES}
                               ${sawTextToSpeech_LIBRARIES})
        # link against cisst libraries (and dependencies)
        cisst_target_link_libraries (sawIntuitiveResearchKitQtConsoleJSON ${REQUIRED_CISST_LIBRARIES})

      endif (CISST_HAS_JSON)

    endif (CISST_HAS_QT)

  endif () # components found

endif (cisst_FOUND_AS_REQUIRED)
