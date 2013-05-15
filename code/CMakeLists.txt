#
# $Id$
#
# (C) Copyright 2011-2013 Johns Hopkins University (JHU), All Rights
# Reserved.
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

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # create/configure file for find_package (sawIntuitiveResearchKit)
  set (sawIntuitiveResearchKit_INCLUDE_DIR "${sawIntuitiveResearchKit_SOURCE_DIR}/include")
  set (sawIntuitiveResearchKit_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawIntuitiveResearchKit_LIBRARIES sawIntuitiveResearchKit)

  include_directories (${sawIntuitiveResearchKit_INCLUDE_DIR})
  set (sawIntuitiveResearchKit_HEADER_DIR "${sawIntuitiveResearchKit_INCLUDE_DIR}/sawIntuitiveResearchKit")

  set (HEADER_FILES
       ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitPSM.h
      )

  set (SOURCE_FILES
       mtsIntuitiveResearchKitPSM.cpp
      )

  add_library (sawIntuitiveResearchKit ${HEADER_FILES} ${SOURCE_FILES})
  cisst_target_link_libraries (sawIntuitiveResearchKit ${REQUIRED_CISST_LIBRARIES})
  set_property (TARGET sawIntuitiveResearchKit PROPERTY FOLDER "sawIntuitiveResearchKit")

  # add Qt code
  # add_subdirectory (Qt)
  # set (sawIntuitiveResearchKit_LIBRARIES ${sawIntuitiveResearchKit_LIBRARIES} ${sawIntuitiveResearchKitQt_LIBRARIES})

  configure_file (
    "${CMAKE_CURRENT_SOURCE_DIR}/sawIntuitiveResearchKitConfig.cmake.in"
    "${sawIntuitiveResearchKit_BINARY_DIR}/sawIntuitiveResearchKitConfig.cmake"
    @ONLY@)

  # Install targets for headers and library
  install (DIRECTORY
           ${sawIntuitiveResearchKit_INCLUDE_DIR}/sawIntuitiveResearchKit
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install (TARGETS sawIntuitiveResearchKit
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawIntuitiveResearchKit_BINARY_DIR}/sawIntuitiveResearchKitConfig.cmake"
           DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

endif (cisst_FOUND)
