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
     cisstCommonQt
     cisstCommonXML
     cisstVector
     cisstParameterTypes
     cisstQt
)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.3.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})
find_package (sawIntuitiveResearchKit 2.3.1 REQUIRED)

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (dVRKConfigWizard_IS_CATKIN_BUILT)
  if (dVRKConfigWizard_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  if (CISST_HAS_QT AND CISST_HAS_JSON)
    set (CMAKE_AUTOMOC ON)

    set (HEADER_FILES
         include/accordion.hpp
         include/arm_editor.hpp
         include/arm_view.hpp
         include/config_editor.hpp
         include/config_sources.hpp
         include/console_editor.hpp
         include/console_inputs_editor.hpp
         include/editor.hpp
         include/file_selector.hpp
         include/io_editor.hpp
         include/io_view.hpp
         include/list_view.hpp
         include/mainwindow.hpp
         include/system_launcher.hpp
         include/teleop_editor.hpp
         include/teleop_view.hpp

         include/models/enum_list_model.hpp
         include/models/config_model.hpp
         include/models/list_model.hpp
    )

    set (SOURCE_FILES
         src/accordion.cpp
         src/arm_editor.cpp
         src/arm_view.cpp
         src/config_editor.cpp
         src/config_sources.cpp
         src/console_editor.cpp
         src/console_inputs_editor.cpp
         src/editor.cpp
         src/file_selector.cpp
         src/io_editor.cpp
         src/io_view.cpp
         src/list_view.cpp
         src/mainwindow.cpp
         src/system_launcher.cpp
         src/teleop_editor.cpp
         src/teleop_view.cpp
    )

    qt5_add_resources (SOURCE_FILES resources/dvrk_logo.qrc)

    add_executable (dvrk-system-config-wizard main.cpp ${HEADER_FILES} ${SOURCE_FILES})
    target_compile_features (dvrk-system-config-wizard PUBLIC cxx_std_17)

    target_include_directories (dvrk-system-config-wizard PRIVATE include)
    target_include_directories (dvrk-system-config-wizard PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    target_include_directories (dvrk-system-config-wizard PRIVATE ${sawIntuitiveResearchKit_INCLUDE_DIR})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (dvrk-system-config-wizard ${REQUIRED_CISST_LIBRARIES})

    set_target_properties (dvrk-system-config-wizard PROPERTIES
                           COMPONENT sawIntuitiveResearchKit-Applications
                           FOLDER "sawIntuitiveResearchKit")

    install (TARGETS dvrk-system-config-wizard
              COMPONENT sawIntuitiveResearchKit-Applications
              RUNTIME DESTINATION bin
              LIBRARY DESTINATION lib
              ARCHIVE DESTINATION lib)

  endif() # Qt/JSON support found
endif (cisst_FOUND_AS_REQUIRED)
