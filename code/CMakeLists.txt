#
# (C) Copyright 2011-2015 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonXML
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstRobot
     cisstNumerical)

find_package (cisst 1.0.6 REQUIRED ${REQUIRED_CISST_LIBRARIES})

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

  # catkin/ROS paths
  cisst_is_catkin_build (sawIntuitiveResearchKit_IS_CATKIN_BUILT)
  if (sawIntuitiveResearchKit_IS_CATKIN_BUILT)
    set (sawIntuitiveResearchKit_CONFIG_FILE_DIR "${CATKIN_DEVEL_PREFIX}/cmake")
    set (LIBRARY_OUTPUT_PATH "${sawIntuitiveResearchKit_BINARY_DIR}/lib")
  else ()
    set (sawIntuitiveResearchKit_CONFIG_FILE_DIR "${sawIntuitiveResearchKit_BINARY_DIR}")
  endif ()

  # saw components have been compiled within cisst, we should find them automatically
  cisst_find_saw_component (sawRobotIO1394 1.3.0 REQUIRED)
  cisst_find_saw_component (sawControllers 1.3.0 REQUIRED)
  cisst_find_saw_component (sawConstraintController REQUIRED)


  if (sawRobotIO1394_FOUND AND sawControllers_FOUND AND sawConstraintController_FOUND)

    # saw components configuration
    include_directories (${sawRobotIO1394_INCLUDE_DIR}
                         ${sawControllers_INCLUDE_DIR}
                         ${sawConstraintController_INCLUDE_DIR})

    link_directories (${sawRobotIO1394_LIBRARY_DIR}
                      ${sawControllers_LIBRARY_DIR}
                      ${sawConstraintController_LIBRARY_DIR})

    # catkin/ROS paths
    cisst_is_catkin_build (sawIntuitiveResearchKit_IS_CATKIN_BUILT)
    if (sawIntuitiveResearchKit_IS_CATKIN_BUILT)
      set (sawIntuitiveResearchKit_CONFIG_FILE_DIR "${CATKIN_DEVEL_PREFIX}/cmake")
      set (LIBRARY_OUTPUT_PATH "${sawIntuitiveResearchKit_BINARY_DIR}/lib")
    else ()
      set (sawIntuitiveResearchKit_CONFIG_FILE_DIR "${sawIntuitiveResearchKit_BINARY_DIR}")
    endif ()

    # create/configure file for find_package (sawIntuitiveResearchKit)
    set (sawIntuitiveResearchKit_INCLUDE_DIR "${sawIntuitiveResearchKit_SOURCE_DIR}/include"
                                             "${sawIntuitiveResearchKit_BINARY_DIR}/include")
    set (sawIntuitiveResearchKit_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set (sawIntuitiveResearchKit_LIBRARIES sawIntuitiveResearchKit)

    # Set the version number
    set (sawIntuitiveResearchKit_VERSION_MAJOR "1")
    set (sawIntuitiveResearchKit_VERSION_MINOR "3")
    set (sawIntuitiveResearchKit_VERSION_PATCH "0")
    set (sawIntuitiveResearchKit_VERSION "${sawIntuitiveResearchKit_VERSION_MAJOR}.${sawIntuitiveResearchKit_VERSION_MINOR}.${sawIntuitiveResearchKit_VERSION_PATCH}")

    # Generate sawIntuitiveResearchKitRevision.h
    configure_file ("${sawIntuitiveResearchKit_SOURCE_DIR}/code/sawIntuitiveResearchKitRevision.h.in"
                    "${sawIntuitiveResearchKit_BINARY_DIR}/include/sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h")

    # create data type using the data generator
    cisst_data_generator (sawIntuitiveResearchKit
                          "${CISST_BINARY_DIR}/include" # where to save the file
                          "sawIntuitiveResearchKit/"    # sub directory for include
                          mtsIntuitiveResearchKitArmTypes.cdg
                          mtsTeleOperationPSMTypes.cdg
                          mtsTeleOperationECMTypes.cdg
                          mtsVFDataFollow.cdg)

    include_directories (${sawIntuitiveResearchKit_INCLUDE_DIR})
    set (sawIntuitiveResearchKit_HEADER_DIR
         "${sawIntuitiveResearchKit_SOURCE_DIR}/include/sawIntuitiveResearchKit")

    set (HEADER_FILES
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsStateMachine.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitArm.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitMTM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitPSM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitECM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitSUJ.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitOptimizer.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsTeleOperationPSM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsTeleOperationECM.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitConsole.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitUDPStreamer.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsVFFollow.h
         ${sawIntuitiveResearchKit_HEADER_DIR}/mtsVFFollowJacobian.h
        )

    set (SOURCE_FILES
         mtsStateMachine.cpp
         mtsIntuitiveResearchKitArm.cpp
         mtsIntuitiveResearchKitMTM.cpp
         mtsIntuitiveResearchKitPSM.cpp
         mtsIntuitiveResearchKitECM.cpp
         mtsIntuitiveResearchKitSUJ.cpp
         mtsIntuitiveResearchKitOptimizer.cpp
         mtsTeleOperationPSM.cpp
         mtsTeleOperationECM.cpp
         mtsIntuitiveResearchKitConsole.cpp
         mtsIntuitiveResearchKitUDPStreamer.cpp
         mtsVFFollow.cpp
         mtsVFFollowJacobian.cpp
        )

    add_library (sawIntuitiveResearchKit
                ${HEADER_FILES} ${SOURCE_FILES}
                ${sawIntuitiveResearchKit_CISST_DG_SRCS}
                ${sawIntuitiveResearchKit_CISST_DG_HDRS})
    cisst_target_link_libraries (sawIntuitiveResearchKit ${REQUIRED_CISST_LIBRARIES})
    set_property (TARGET sawIntuitiveResearchKit PROPERTY FOLDER "sawIntuitiveResearchKit")

    # link against non cisst libraries and cisst components
    target_link_libraries (sawIntuitiveResearchKit
                           ${sawRobotIO1394_LIBRARIES}
                           ${sawControllers_LIBRARIES}
                           ${sawConstraintController_LIBRARIES})

    # add Qt code
    add_subdirectory (Qt)
    set (sawIntuitiveResearchKit_LIBRARIES ${sawIntuitiveResearchKit_LIBRARIES} ${sawIntuitiveResearchKitQt_LIBRARIES})

    configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawIntuitiveResearchKitConfig.cmake.in"
                    "${sawIntuitiveResearchKit_CONFIG_FILE_DIR}/sawIntuitiveResearchKitConfig.cmake"
                    @ONLY@)

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

    install (FILES "${sawIntuitiveResearchKit_BINARY_DIR}/sawIntuitiveResearchKitConfig.cmake"
             DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

  endif (sawRobotIO1394_FOUND AND sawControllers_FOUND AND sawConstraintController_FOUND)

endif (cisst_FOUND)
