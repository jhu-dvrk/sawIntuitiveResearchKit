#
#
# (C) Copyright 2013 Johns Hopkins University (JHU), All Rights
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
     cisstVectorQt
     cisstNumerical
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstParameterTypesQt
     cisstQt)

# cisst
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # cisst
  include (${CISST_USE_FILE})

  # for Qt generated header files
  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  qt4_wrap_cpp (SAW_INTUITIVE_RESEARCH_KIT_QT_WRAP_CPP
                ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitConsoleQtWidget.h
                ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitArmQtWidget.h)
  add_library (sawIntuitiveResearchKitQt
               ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitConsoleQtWidget.h
               mtsIntuitiveResearchKitConsoleQtWidget.cpp
               ${sawIntuitiveResearchKit_HEADER_DIR}/mtsIntuitiveResearchKitArmQtWidget.h
               mtsIntuitiveResearchKitArmQtWidget.cpp
               ${SAW_INTUITIVE_RESEARCH_KIT_QT_WRAP_CPP})
  set_property (TARGET sawIntuitiveResearchKitQt PROPERTY FOLDER "sawIntuitiveResearchKit")
  cisst_target_link_libraries (sawIntuitiveResearchKitQt ${REQUIRED_CISST_LIBRARIES})

  # make sure the new library is known by the parent folder to add to the config file
  set (sawIntuitiveResearchKitQt_LIBRARIES sawIntuitiveResearchKitQt PARENT_SCOPE)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
