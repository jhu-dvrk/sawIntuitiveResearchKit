#
# (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.16)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstParameterTypes
     cisstQt
)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.3.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawIntuitiveResearchKitSystemWizard_IS_CATKIN_BUILT)
  if (sawIntuitiveResearchKitSystemWizard_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  if (CISST_HAS_QT AND CISST_HAS_JSON)
    set (CMAKE_AUTOMOC ON)
    
    set (HEADER_FILES
         include/accordion.hpp
         include/arm_editor.hpp
         include/config_editor.hpp
         include/config_sources.hpp
         include/enum_list_model.hpp
         include/io_editor.hpp
         include/mainwindow.hpp
    )

    set (SOURCE_FILES
         src/accordion.cpp
         src/arm_editor.cpp
         src/config_editor.cpp
         src/config_sources.cpp
         src/io_editor.cpp
         src/mainwindow.cpp
    )

    add_executable(sawIntuitiveResearchKitSystemWizard main.cpp ${HEADER_FILES} ${SOURCE_FILES})
    target_compile_features(sawIntuitiveResearchKitSystemWizard PUBLIC cxx_std_17)

    target_include_directories(sawIntuitiveResearchKitSystemWizard PRIVATE include)

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawIntuitiveResearchKitSystemWizard ${REQUIRED_CISST_LIBRARIES})

    set_target_properties (sawIntuitiveResearchKitSystemWizard PROPERTIES
                           COMPONENT sawIntuitiveResearchKit-Applications
                           FOLDER "sawIntuitiveResearchKit")

    install (TARGETS sawIntuitiveResearchKitSystemWizard
              COMPONENT sawIntuitiveResearchKit-Applications
              RUNTIME DESTINATION bin
              LIBRARY DESTINATION lib
              ARCHIVE DESTINATION lib)

  endif() # Qt/JSON support found
endif (cisst_FOUND_AS_REQUIRED)
