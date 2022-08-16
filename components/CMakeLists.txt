#
# (C) Copyright 2011-2022 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawIntuitiveResearchKit VERSION 2.1.0)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonXML
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical)

find_package (cisst 1.1.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

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
  find_package (sawTextToSpeech         1.3.0 REQUIRED)
  find_package (sawRobotIO1394          2.1.0 REQUIRED)
  find_package (sawControllers          2.0.0 REQUIRED)

  if (sawRobotIO1394_FOUND AND sawControllers_FOUND)

    # catkin/ROS paths
    cisst_set_output_path ()

    # external project for gattlib
    include (ExternalProject)

    set (SAW_GATTLIB_GIT_REPOSITORY https://github.com/jhu-cisst-external/gattlib)
    ExternalProject_Add (gattlibExternal
                         PREFIX gattlib
                         GIT_REPOSITORY    ${SAW_GATTLIB_GIT_REPOSITORY}
                         CMAKE_CACHE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
                                          -DGATTLIB_BUILD_EXAMPLES:BOOL=OFF
                                          -DGATTLIB_BUILD_DOCS:BOOL=OFF
                                          -DGATTLIB_PYTHON_INTERFACE:BOOL=OFF
                                          -DGATTLIB_BUILD_EXAMPLES:BOOL=OFF
                                          -DGATTLIB_SHARED_LIB:BOOL=ON
                                          -DCMAKE_INSTALL_PREFIX:FILEPATH=${CMAKE_CURRENT_BINARY_DIR}/gattlib
                         INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/gattlib
                         UPDATE_COMMAND ""
                         )
    set (gattlib_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/gattlib/include")
    set (gattlib_LIBRARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/gattlib/lib")
    set (gattlib_LIBRARIES gattlib)

    # saw components configuration
    include_directories (${sawTextToSpeech_INCLUDE_DIR}
                         ${sawRobotIO1394_INCLUDE_DIR}
                         ${sawControllers_INCLUDE_DIR}
                         ${gattlib_INCLUDE_DIR})

    link_directories (${sawTextToSpeech_LIBRARY_DIR}
                      ${sawRobotIO1394_LIBRARY_DIR}
                      ${sawControllers_LIBRARY_DIR}
                      ${gattlib_LIBRARY_DIR})

    # create/configure file for find_package (sawIntuitiveResearchKit)
    set (sawIntuitiveResearchKit_INCLUDE_DIR
         "${sawIntuitiveResearchKit_SOURCE_DIR}/include"
         "${sawIntuitiveResearchKit_BINARY_DIR}/include")
    set (sawIntuitiveResearchKit_HEADER_DIR
         "${sawIntuitiveResearchKit_SOURCE_DIR}/include/sawIntuitiveResearchKit")
    set (sawIntuitiveResearchKit_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set (sawIntuitiveResearchKit_LIBRARIES sawIntuitiveResearchKit)

    include_directories (${sawIntuitiveResearchKit_INCLUDE_DIR})

    if (CISST_HAS_QT)
      add_subdirectory (code/Qt)
      set (sawIntuitiveResearchKit_LIBRARIES ${sawIntuitiveResearchKit_LIBRARIES} sawIntuitiveResearchKitQt)
    endif ()

    # add all config files for this component
    cisst_add_config_files (sawIntuitiveResearchKit)

    # Generate sawIntuitiveResearchKitConfig.h
    configure_file ("${sawIntuitiveResearchKit_SOURCE_DIR}/code/sawIntuitiveResearchKitConfig.h.in"
                    "${sawIntuitiveResearchKit_BINARY_DIR}/include/sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h")

    # create data type using the data generator
    cisst_data_generator (sawIntuitiveResearchKit
                          "${sawIntuitiveResearchKit_BINARY_DIR}/include" # where to save the file
                          "sawIntuitiveResearchKit/"    # sub directory for include
                          code/mtsIntuitiveResearchKitArmTypes.cdg
                          code/mtsIntuitiveResearchKitToolTypes.cdg
                          code/mtsIntuitiveResearchKitEndoscopeTypes.cdg
                          code/socketMessages.cdg)


    set (HEADER_FILES
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsStateMachine.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKit.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitArm.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitMTM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitPSM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitECM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitSUJ.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitSUJSi.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsTeleOperationPSM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsTeleOperationECM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitConsole.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsDaVinciHeadSensor.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsDaVinciEndoscopeFocus.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitUDPStreamer.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsSocketBasePSM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsSocketClientPSM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsSocketServerPSM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsToolList.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/robManipulatorECM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/robManipulatorMTM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/robManipulatorPSMSnake.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsPSMCompensation.h
        )

    set (SOURCE_FILES
         code/mtsStateMachine.cpp
         code/mtsIntuitiveResearchKitArm.cpp
         code/mtsIntuitiveResearchKitMTM.cpp
         code/mtsIntuitiveResearchKitPSM.cpp
         code/mtsIntuitiveResearchKitECM.cpp
         code/mtsIntuitiveResearchKitSUJ.cpp
         code/mtsIntuitiveResearchKitSUJSi.cpp
         code/mtsTeleOperationPSM.cpp
         code/mtsTeleOperationECM.cpp
         code/mtsIntuitiveResearchKitConsole.cpp
         code/mtsDaVinciHeadSensor.cpp
         code/mtsDaVinciEndoscopeFocus.cpp
         code/mtsIntuitiveResearchKitUDPStreamer.cpp
         code/mtsSocketBasePSM.cpp
         code/mtsSocketClientPSM.cpp
         code/mtsSocketServerPSM.cpp
         code/mtsToolList.cpp
         code/robManipulatorECM.cpp
         code/robManipulatorMTM.cpp
         code/robManipulatorPSMSnake.cpp
         code/mtsPSMCompensation.cpp
         code/robGravityCompensationMTM.cpp
         code/robGravityCompensationMTM.h
         )

    add_library (sawIntuitiveResearchKit
                 ${HEADER_FILES} ${SOURCE_FILES}
                 ${sawIntuitiveResearchKit_CISST_DG_SRCS}
                 ${sawIntuitiveResearchKit_CISST_DG_HDRS})
    cisst_target_link_libraries (sawIntuitiveResearchKit ${REQUIRED_CISST_LIBRARIES})
    set_target_properties (sawIntuitiveResearchKit PROPERTIES
                           VERSION ${sawIntuitiveResearchKit_VERSION}
                           FOLDER "sawIntuitiveResearchKit")
    add_dependencies (sawIntuitiveResearchKit gattlibExternal)

    # link against non cisst libraries and cisst components
    target_link_libraries (sawIntuitiveResearchKit
                           ${sawTextToSpeech_LIBRARIES}
                           ${sawRobotIO1394_LIBRARIES}
                           ${sawControllers_LIBRARIES}
                           ${gattlib_LIBRARIES})

    # Install targets for headers and library
    install (DIRECTORY
             ${sawIntuitiveResearchKit_SOURCE_DIR}/include/sawIntuitiveResearchKit
             ${sawIntuitiveResearchKit_BINARY_DIR}/include/sawIntuitiveResearchKit
             DESTINATION include
             COMPONENT sawIntuitiveResearchKit-dev
             PATTERN .svn EXCLUDE)

    install (TARGETS sawIntuitiveResearchKit COMPONENT sawIntuitiveResearchKit
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

  endif (sawRobotIO1394_FOUND AND sawControllers_FOUND)

endif (cisst_FOUND)
