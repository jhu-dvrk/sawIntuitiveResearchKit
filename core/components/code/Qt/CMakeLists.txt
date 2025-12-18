#
# (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.
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
  cisstVectorQt
  cisstNumerical
  cisstOSAbstraction
  cisstMultiTask
  cisstMultiTaskQt
  cisstParameterTypes
  cisstParameterTypesQt
  cisstQt)

# cisst
find_package (cisst 1.4.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # cisst
  include (${CISST_USE_FILE})

  # for Qt generated header files
  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  set (CMAKE_AUTOMOC ON)
  qt5_add_resources (sawIntuitiveResearchKit_QT_RESOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/logo.qrc)

  add_library (sawIntuitiveResearchKitQt
               ${sawIntuitiveResearchKit_HEADER_DIR}/system_Qt.h
               system_Qt.cpp
               ${sawIntuitiveResearchKit_HEADER_DIR}/system_Qt_widget.h
               system_Qt_widget.cpp
               ${sawIntuitiveResearchKit_HEADER_DIR}/console_Qt_widget.h
               console_Qt_widget.cpp
               ${sawIntuitiveResearchKit_HEADER_DIR}/mtsTeleOperationPSMQtWidget.h
               mtsTeleOperationPSMQtWidget.cpp
               ${sawIntuitiveResearchKit_HEADER_DIR}/mtsTeleOperationECMQtWidget.h
               mtsTeleOperationECMQtWidget.cpp
               ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitArmQtWidget.h
               mtsIntuitiveResearchKitArmQtWidget.cpp
               ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitECMQtWidget.h
               mtsIntuitiveResearchKitECMQtWidget.cpp
               ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitMTMQtWidget.h
               mtsIntuitiveResearchKitMTMQtWidget.cpp
               ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitPSMQtWidget.h
               mtsIntuitiveResearchKitPSMQtWidget.cpp
               ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitSUJQtWidget.h
               mtsIntuitiveResearchKitSUJQtWidget.cpp
               ${sawIntuitiveResearchKit_HEADER_DIR}/mtsDaVinciEndoscopeFocusQtWidget.h
               mtsDaVinciEndoscopeFocusQtWidget.cpp
               ${sawIntuitiveResearchKit_QT_WRAP_CPP}
               ${sawIntuitiveResearchKit_QT_RESOURCES}
               )
  set_target_properties (sawIntuitiveResearchKitQt PROPERTIES
                         VERSION ${sawIntuitiveResearchKit_VERSION}
                         FOLDER "sawIntuitiveResearchKit")
  cisst_target_link_libraries (sawIntuitiveResearchKitQt ${REQUIRED_CISST_LIBRARIES})
  target_link_libraries (sawIntuitiveResearchKitQt
                         ${sawIntuitiveResearchKit_LIBRARIES})

  # make sure the new library is known by the parent folder to add to the config file
  set (sawIntuitiveResearchKitQt_LIBRARIES sawIntuitiveResearchKitQt PARENT_SCOPE)

  install (TARGETS sawIntuitiveResearchKitQt COMPONENT sawIntuitiveResearchKit-Qt
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
