#
# (C) Copyright 2022-2026 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.16)
project (sawIntuitiveResearchKitSiPotentiometersCalibration VERSION 2.5.0)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.5 REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# catkin/ROS paths
cisst_is_catkin_build (sawIntuitiveResearchKitSiPotentiometersCalibration_IS_CATKIN_BUILT)
if (sawIntuitiveResearchKitSiPotentiometersCalibration_IS_CATKIN_BUILT)
  set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
endif ()

# saw components have been compiled within cisst, we should find them automatically
find_package (sawRobotIO1394 2.5.0 REQUIRED)
find_package (sawIntuitiveResearchKit 2.5.0 REQUIRED)

if (sawRobotIO1394_FOUND)

  # saw components configuration
  include_directories (${sawRobotIO1394_INCLUDE_DIR}
                       ${sawIntuitiveResearchKit_INCLUDE_DIR}
                       "${sawIntuitiveResearchKit_DIR}/include")
  link_directories (${sawRobotIO1394_LIBRARY_DIR})

  # main program used to calibrate the MTM gripper
  add_executable (sawIntuitiveResearchKitSiPotentiometersCalibration SiPotentiometersCalibration.cpp)
  if (TARGET sawIntuitiveResearchKitFullRevision)
    add_dependencies (sawIntuitiveResearchKitSiPotentiometersCalibration sawIntuitiveResearchKitFullRevision)
  endif ()
  set_property (TARGET sawIntuitiveResearchKitSiPotentiometersCalibration PROPERTY FOLDER "sawIntuitiveResearchKit")

  # link against non cisst libraries and cisst components
  target_link_libraries (sawIntuitiveResearchKitSiPotentiometersCalibration PRIVATE
                         ${sawRobotIO1394_LIBRARIES}
                         ${cisst_LIBRARIES})

  install (
    TARGETS sawIntuitiveResearchKitSiPotentiometersCalibration
    COMPONENT sawIntuitiveResearchKit-Applications
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

endif () # components found
