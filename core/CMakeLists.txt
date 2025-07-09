#
# (C) Copyright 2009-2025 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required(VERSION 3.16)
project (sawIntuitiveResearchKitCore VERSION 2.3.1)

find_package (cisst REQUIRED)
include (${CISST_USE_FILE})
cisst_cpack_settings (
  VENDOR "JHU"
  MAINTAINER "anton.deguet@jhu.edu")

add_subdirectory (components)

set (sawIntuitiveResearchKit_DIR "${sawIntuitiveResearchKitCore_BINARY_DIR}/components")
add_subdirectory (applications)
add_subdirectory (examples)

include (CPack)
cpack_add_component (sawIntuitiveResearchKit)
cpack_add_component (sawIntuitiveResearchKit-dev
                     DEPENDS sawIntuitiveResearchKit)
cpack_add_component (sawIntuitiveResearchKit-Qt
                     DEPENDS sawIntuitiveResearchKit)
cpack_add_component (sawIntuitiveResearchKit-Applications
                     DEPENDS sawIntuitiveResearchKit sawIntuitiveResearchKit-Qt)
