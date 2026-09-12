#
# (C) Copyright 2009-2026 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required(VERSION 3.16)
if (POLICY CMP0144)
  cmake_policy (SET CMP0144 NEW)
  set (CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif ()
project (sawIntuitiveResearchKitCore VERSION 2.5.0)

find_package (cisst 1.5 REQUIRED)
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
