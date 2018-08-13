#
# (C) Copyright 2011-2018 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (sawIntuitiveResearchKit)

cmake_minimum_required (VERSION 2.8)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonXML
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical)

find_package (cisst 1.0.10 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # make sure CISST_USE_SI_UNITS is defined and set to 1
  if (NOT DEFINED CISST_USE_SI_UNITS)
    message (FATAL_ERROR "You need to upgrade to a version of cisst that defines CISST_USE_SI_UNITS (and set CISST_USE_SI_UNITS to ON)")
  endif ()
  if (NOT CISST_USE_SI_UNITS)
    message (FATAL_ERROR "You need to make sure CISST_USE_SI_UNITS to ON")
  endif ()

  # saw components have been compiled within cisst, we should find them automatically
  find_package (sawTextToSpeech         1.2.0 REQUIRED)
  find_package (sawRobotIO1394          1.6.0 REQUIRED)
  find_package (sawControllers          1.6.0 REQUIRED)

  if (sawRobotIO1394_FOUND AND sawControllers_FOUND)

    # saw components configuration
    include_directories (${sawTextToSpeech_INCLUDE_DIR}
                         ${sawRobotIO1394_INCLUDE_DIR}
                         ${sawControllers_INCLUDE_DIR})

    link_directories (${sawTextToSpeech_LIBRARY_DIR}
                      ${sawRobotIO1394_LIBRARY_DIR}
                      ${sawControllers_LIBRARY_DIR})

    # catkin/ROS paths
    cisst_is_catkin_build (sawIntuitiveResearchKit_IS_CATKIN_BUILT)
    if (sawIntuitiveResearchKit_IS_CATKIN_BUILT)
      set (sawIntuitiveResearchKit_CONFIG_FILE_DIR
           "${CATKIN_DEVEL_PREFIX}/share/sawIntuitiveResearchKit/cmake")
      set (LIBRARY_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/lib")
    else ()
      set (sawIntuitiveResearchKit_CONFIG_FILE_DIR "${sawIntuitiveResearchKit_BINARY_DIR}")
    endif ()

    # create/configure file for find_package (sawIntuitiveResearchKit)
    set (sawIntuitiveResearchKit_INCLUDE_DIR
         "${sawIntuitiveResearchKit_SOURCE_DIR}/include"
         "${sawIntuitiveResearchKit_BINARY_DIR}/include")
    set (sawIntuitiveResearchKit_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set (sawIntuitiveResearchKit_LIBRARIES sawIntuitiveResearchKit)

    # Set the version number
    set (sawIntuitiveResearchKit_VERSION_MAJOR "1")
    set (sawIntuitiveResearchKit_VERSION_MINOR "6")
    set (sawIntuitiveResearchKit_VERSION_PATCH "0")
    set (sawIntuitiveResearchKit_VERSION "${sawIntuitiveResearchKit_VERSION_MAJOR}.${sawIntuitiveResearchKit_VERSION_MINOR}.${sawIntuitiveResearchKit_VERSION_PATCH}")

    # Generate sawIntuitiveResearchKitRevision.h
    configure_file ("${sawIntuitiveResearchKit_SOURCE_DIR}/code/sawIntuitiveResearchKitRevision.h.in"
                    "${sawIntuitiveResearchKit_BINARY_DIR}/include/sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h")

    # Generate sawIntuitiveResearchKitConfig.h
    configure_file ("${sawIntuitiveResearchKit_SOURCE_DIR}/code/sawIntuitiveResearchKitConfig.h.in"
                    "${sawIntuitiveResearchKit_BINARY_DIR}/include/sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h")

    # create data type using the data generator
    cisst_data_generator (sawIntuitiveResearchKit
                          "${sawIntuitiveResearchKit_BINARY_DIR}/include" # where to save the file
                          "sawIntuitiveResearchKit/"    # sub directory for include
                          code/mtsIntuitiveResearchKitArmTypes.cdg
                          code/socketMessages.cdg)

    include_directories (${sawIntuitiveResearchKit_INCLUDE_DIR})
    set (sawIntuitiveResearchKit_HEADER_DIR
         "${sawIntuitiveResearchKit_SOURCE_DIR}/include/sawIntuitiveResearchKit")

    set (HEADER_FILES
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsStateMachine.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKit.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitArm.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitMTM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitPSM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitECM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitSUJ.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsTeleOperationPSM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsTeleOperationECM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitConsole.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsDaVinciHeadSensor.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsDaVinciEndoscopeFocus.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitUDPStreamer.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsSocketBasePSM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsSocketClientPSM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsSocketServerPSM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/robManipulatorPSMSnake.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/robManipulatorRMRC.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsPSMCompensation.h
        )

    set (SOURCE_FILES
         code/mtsStateMachine.cpp
         code/mtsIntuitiveResearchKitArm.cpp
         code/mtsIntuitiveResearchKitMTM.cpp
         code/mtsIntuitiveResearchKitPSM.cpp
         code/mtsIntuitiveResearchKitECM.cpp
         code/mtsIntuitiveResearchKitSUJ.cpp
         code/mtsTeleOperationPSM.cpp
         code/mtsTeleOperationECM.cpp
         code/mtsIntuitiveResearchKitConsole.cpp
         code/mtsDaVinciHeadSensor.cpp
         code/mtsDaVinciEndoscopeFocus.cpp
         code/mtsIntuitiveResearchKitUDPStreamer.cpp
         code/mtsSocketBasePSM.cpp
         code/mtsSocketClientPSM.cpp
         code/mtsSocketServerPSM.cpp
         code/robManipulatorRMRC.cpp
         code/robManipulatorPSMSnake.cpp
         code/mtsPSMCompensation.cpp
        )

    add_library (sawIntuitiveResearchKit
                 ${HEADER_FILES} ${SOURCE_FILES}
                 ${sawIntuitiveResearchKit_CISST_DG_SRCS}
                 ${sawIntuitiveResearchKit_CISST_DG_HDRS})
    cisst_target_link_libraries (sawIntuitiveResearchKit ${REQUIRED_CISST_LIBRARIES})
    set_property (TARGET sawIntuitiveResearchKit PROPERTY FOLDER "sawIntuitiveResearchKit")

    # link against non cisst libraries and cisst components
    target_link_libraries (sawIntuitiveResearchKit
                           ${sawTextToSpeech_LIBRARIES}
                           ${sawRobotIO1394_LIBRARIES}
                           ${sawControllers_LIBRARIES})

    # add Qt code
    add_subdirectory (code/Qt)
    set (sawIntuitiveResearchKit_LIBRARIES ${sawIntuitiveResearchKit_LIBRARIES} ${sawIntuitiveResearchKitQt_LIBRARIES})

    set (sawIntuitiveResearchKit_CMAKE_CONFIG_FILE
         "${sawIntuitiveResearchKit_CONFIG_FILE_DIR}/sawIntuitiveResearchKitConfig.cmake")

    configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawIntuitiveResearchKitConfig.cmake.in"
                    "${sawIntuitiveResearchKit_CMAKE_CONFIG_FILE}"
                    @ONLY)

    cisst_add_config_version ("${sawIntuitiveResearchKit_CONFIG_FILE_DIR}/sawIntuitiveResearchKitConfigVersion.cmake"
                              VERSION ${sawIntuitiveResearchKit_VERSION}
                              DESTINATION ${CISST_CMAKE_INSTALL_SUFFIX}/saw
                              COMPONENT sawIntuitiveResearchKit)

    # Install targets for headers and library
    install (DIRECTORY
             ${sawIntuitiveResearchKit_SOURCE_DIR}/include/sawIntuitiveResearchKit
             ${sawIntuitiveResearchKit_BINARY_DIR}/include/sawIntuitiveResearchKit
             DESTINATION include
             PATTERN .svn EXCLUDE)

    install (TARGETS sawIntuitiveResearchKit
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

    install (FILES "${sawIntuitiveResearchKit_CMAKE_CONFIG_FILE}"
             DESTINATION "share/sawIntuitiveResearchKit")

  endif (sawRobotIO1394_FOUND AND sawControllers_FOUND)

endif (cisst_FOUND)
