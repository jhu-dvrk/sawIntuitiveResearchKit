#
# (C) Copyright 2011-2025 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.16)
project (sawIntuitiveResearchKit VERSION 2.3.1)

set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonXML
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot
  cisstNumerical)

find_package (cisst 1.3.1 REQUIRED ${REQUIRED_CISST_LIBRARIES})

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
  find_package (sawTextToSpeech         1.4.0 REQUIRED)
  find_package (sawRobotIO1394          2.3.0 REQUIRED)
  find_package (sawControllers          2.2.0 REQUIRED)

  if (sawRobotIO1394_FOUND AND sawControllers_FOUND)

    # catkin/ROS paths
    cisst_set_output_path ()

    # gattlib is not available on Windows
    set (OPT_MSG "Compile code to support the SI SUJ over bluetooth (requires gattlib)")
    if (WIN32 OR APPLE)
      option (sawIntuitiveResearchKit_HAS_SUJ_Si ${OPT_MSG} OFF)
    else ()
      option (sawIntuitiveResearchKit_HAS_SUJ_Si ${OPT_MSG} ON)
    endif ()

    set (OPT_MSG "Compile code to support the HID head sensor like Goovis HD (requires hidapi-hidraw)")
    if (WIN32 OR APPLE)
      option (sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR ${OPT_MSG} OFF)
    else ()
      option (sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR ${OPT_MSG} ON)
    endif ()

    if (sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR)
      # there must be a more standard way to find a unix lib...
      find_file (hidapi_HEADER_FILE
        hidapi/hidapi.h
        HINTS /usr/include)
      if (hidapi_HEADER_FILE)
        string (REPLACE "/hidapi/hidapi.h" "" hidapi_INCLUDE_DIR ${hidapi_HEADER_FILE})
        include_directories (${hidapi_INCLUDE_DIR})
        set (hidapi_LIBRARIES "hidapi-hidraw")
      endif ()
    endif ()

    # external project for gattlib
    if (sawIntuitiveResearchKit_HAS_SUJ_Si)
      include (ExternalProject)

      set (SAW_GATTLIB_GIT_REPOSITORY https://github.com/jhu-cisst-external/gattlib)
      ExternalProject_Add (gattlibExternal
                           PREFIX gattlib
                           GIT_REPOSITORY    ${SAW_GATTLIB_GIT_REPOSITORY}
                           GIT_TAG          "dvrk-2.2"
                           CMAKE_CACHE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
                                            -DGATTLIB_BUILD_EXAMPLES:BOOL=OFF
                                            -DGATTLIB_BUILD_DOCS:BOOL=OFF
                                            -DGATTLIB_PYTHON_INTERFACE:BOOL=OFF
                                            -DGATTLIB_BUILD_EXAMPLES:BOOL=OFF
                                            -DGATTLIB_SHARED_LIB:BOOL=ON
                                            -DCMAKE_INSTALL_PREFIX:FILEPATH=${CMAKE_INSTALL_PREFIX}
                           UPDATE_COMMAND ""
                           INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
                           )
      set (gattlib_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
      set (gattlib_LIBRARY_DIR "${CMAKE_INSTALL_PREFIX}/lib")
      set (gattlib_LIBRARIES gattlib)

      include_directories (${gattlib_INCLUDE_DIR})
      link_directories (${gattlib_LIBRARY_DIR})
    endif ()

    # saw components configuration
    include_directories (${sawTextToSpeech_INCLUDE_DIR}
                         ${sawRobotIO1394_INCLUDE_DIR}
                         ${sawControllers_INCLUDE_DIR})

    link_directories (${sawTextToSpeech_LIBRARY_DIR}
                      ${sawRobotIO1394_LIBRARY_DIR}
                      ${sawControllers_LIBRARY_DIR})

    # create/configure file for find_package (sawIntuitiveResearchKit)
    set (sawIntuitiveResearchKit_INCLUDE_DIR
         "${sawIntuitiveResearchKit_SOURCE_DIR}/include"
         "${sawIntuitiveResearchKit_BINARY_DIR}/include")
    set (sawIntuitiveResearchKit_HEADER_DIR
         "${sawIntuitiveResearchKit_SOURCE_DIR}/include/sawIntuitiveResearchKit")
    set (sawIntuitiveResearchKit_LIBRARY_DIR
      "${LIBRARY_OUTPUT_PATH}")

    if (sawIntuitiveResearchKit_HAS_SUJ_Si)
      set (sawIntuitiveResearchKit_LIBRARY_DIR
        ${sawIntuitiveResearchKit_LIBRARY_DIR}
        "${gattlib_LIBRARY_DIR}")
    endif ()

    set (sawIntuitiveResearchKit_LIBRARIES sawIntuitiveResearchKit)

    include_directories (BEFORE ${sawIntuitiveResearchKit_INCLUDE_DIR})

    if (CISST_HAS_QT)
      add_subdirectory (code/Qt)
      set (sawIntuitiveResearchKit_LIBRARIES ${sawIntuitiveResearchKit_LIBRARIES} sawIntuitiveResearchKitQt)
    endif ()

    # add all config files for this component
    cisst_add_config_files (sawIntuitiveResearchKit)

    # Generate sawIntuitiveResearchKitConfig.h
    set (sawIntuitiveResearchKit_SOURCE_CONFIG_DIR "${sawIntuitiveResearchKit_SOURCE_DIR}/../../share")
    configure_file ("${sawIntuitiveResearchKit_SOURCE_DIR}/code/sawIntuitiveResearchKitConfig.h.in"
                    "${sawIntuitiveResearchKit_BINARY_DIR}/include/sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h")

    set (_HEADER_FILES
      mtsStateMachine.h
      mtsIntuitiveResearchKit.h
      mtsIntuitiveResearchKitArm.h
      mtsIntuitiveResearchKitMTM.h
      mtsIntuitiveResearchKitPSM.h
      mtsIntuitiveResearchKitECM.h
      mtsIntuitiveResearchKitSUJ.h
      mtsIntuitiveResearchKitSUJSi.h
      mtsIntuitiveResearchKitSUJFixed.h
      mtsTeleOperationPSM.h
      mtsTeleOperationECM.h
      mtsDaVinciHeadSensor.h
      mtsDaVinciEndoscopeFocus.h
      mtsToolList.h
      robManipulatorECM.h
      robManipulatorMTM.h
      robManipulatorPSMSnake.h
      mtsPSMCompensation.h
      prmActuatorJointCouplingCheck.h
      prmConfigurationJointFromManipulator.h
      robGravityCompensation.h

      arm_proxy.h
      console.h
      IO_proxy.h
      system.h
      teleop_proxy.h
      teleop_ECM_proxy.h
      teleop_PSM_proxy.h
    )

    set (_SOURCE_FILES
      mtsStateMachine.cpp
      mtsIntuitiveResearchKit.cpp
      mtsIntuitiveResearchKitArm.cpp
      mtsIntuitiveResearchKitMTM.cpp
      mtsIntuitiveResearchKitPSM.cpp
      mtsIntuitiveResearchKitECM.cpp
      mtsIntuitiveResearchKitSUJ.cpp
      mtsIntuitiveResearchKitSUJSi.cpp
      mtsIntuitiveResearchKitSUJFixed.cpp
      mtsTeleOperationPSM.cpp
      mtsTeleOperationECM.cpp
      mtsDaVinciHeadSensor.cpp
      mtsDaVinciEndoscopeFocus.cpp
      mtsToolList.cpp
      robManipulatorECM.cpp
      robManipulatorMTM.cpp
      robManipulatorPSMSnake.cpp
      mtsPSMCompensation.cpp
      prmActuatorJointCouplingCheck.cpp
      prmConfigurationJointFromManipulator.cpp
      robGravityCompensationMTM.cpp
      robGravityCompensationMTM.h

      arm_proxy.cpp
      console.cpp
      IO_proxy.cpp
      system.cpp
      teleop_proxy.cpp
      teleop_ECM_proxy.cpp
      teleop_PSM_proxy.cpp
    )

    set (_CDG_FILES
      arm_configuration.cdg
      arm_proxy_configuration.cdg
      arm_type.cdg
      base_frame.cdg
      console_configuration.cdg
      generation.cdg
      IO_proxy_configuration.cdg
      simulation.cdg
      system_configuration.cdg
      teleop_ECM_configuration.cdg
      teleop_PSM_configuration.cdg
      teleop_proxy_configuration.cdg
      teleop_type.cdg
      mtsIntuitiveResearchKitControlTypes.cdg
      mtsIntuitiveResearchKitToolTypes.cdg
      mtsIntuitiveResearchKitEndoscopeTypes.cdg)

    if (sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR)
      list (APPEND _HEADER_FILES mtsHIDHeadSensor.h)
      list (APPEND _SOURCE_FILES mtsHIDHeadSensor.cpp)
      list (APPEND _CDG_FILES    mtsHIDHeadSensorConfiguration.cdg)
    endif (sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR)

    foreach (f ${_HEADER_FILES})
      list (APPEND HEADER_FILES "${sawIntuitiveResearchKit_HEADER_DIR}/${f}")
    endforeach ()

    foreach (f ${_SOURCE_FILES})
      list (APPEND SOURCE_FILES "code/${f}")
    endforeach ()

    foreach (f ${_CDG_FILES})
      list (APPEND CDG_FILES "code/${f}")
    endforeach ()

    # create data type using the data generator
    cisst_data_generator (sawIntuitiveResearchKit
                          "${sawIntuitiveResearchKit_BINARY_DIR}/include" # where to save the file
                          "sawIntuitiveResearchKit/"    # sub directory for include
                          ${CDG_FILES})

    add_library (sawIntuitiveResearchKit
                 ${HEADER_FILES} ${SOURCE_FILES}
                 ${sawIntuitiveResearchKit_CISST_DG_SRCS}
                 ${sawIntuitiveResearchKit_CISST_DG_HDRS})
    cisst_target_link_libraries (sawIntuitiveResearchKit ${REQUIRED_CISST_LIBRARIES})
    set_target_properties (sawIntuitiveResearchKit PROPERTIES
                           VERSION ${sawIntuitiveResearchKit_VERSION}
                           FOLDER "sawIntuitiveResearchKit")
    if (sawIntuitiveResearchKit_HAS_SUJ_Si)
      add_dependencies (sawIntuitiveResearchKit gattlibExternal)
    endif ()

    # link against non cisst libraries and cisst components
    target_link_libraries (
      sawIntuitiveResearchKit
      ${sawTextToSpeech_LIBRARIES}
      ${sawRobotIO1394_LIBRARIES}
      ${sawControllers_LIBRARIES})

    if (sawIntuitiveResearchKit_HAS_SUJ_Si)
      target_link_libraries (
        sawIntuitiveResearchKit
        ${gattlib_LIBRARIES})
    endif ()

    if (sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR)
      target_link_libraries (
        sawIntuitiveResearchKit
        ${hidapi_LIBRARIES})
    endif ()

    # Install targets for headers and library
    install (DIRECTORY
             ${sawIntuitiveResearchKit_SOURCE_DIR}/include/sawIntuitiveResearchKit
             ${sawIntuitiveResearchKit_BINARY_DIR}/include/sawIntuitiveResearchKit
             DESTINATION include
             COMPONENT sawIntuitiveResearchKit-dev)

    install (TARGETS sawIntuitiveResearchKit COMPONENT sawIntuitiveResearchKit
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

  endif (sawRobotIO1394_FOUND AND sawControllers_FOUND)

endif (cisst_FOUND)
