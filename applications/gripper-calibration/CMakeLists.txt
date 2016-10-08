#
# (C) Copyright 2009-2016 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonXML
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.0.8 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawIntuitiveResearchKitGripperCalibration_IS_CATKIN_BUILT)
  if (sawIntuitiveResearchKitGripperCalibration_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # saw components have been compiled within cisst, we should find them automatically
  find_package (sawRobotIO1394          1.4.0 REQUIRED)

  if (sawRobotIO1394_FOUND)

    # saw components configuration
    include_directories (${sawRobotIO1394_INCLUDE_DIR})
    link_directories (${sawRobotIO1394_LIBRARY_DIR})

    # main program used to calibrate the MTM gripper
    add_executable (sawIntuitiveResearchKitGripperCalibration GripperCalibration.cpp)
    set_property (TARGET sawIntuitiveResearchKitGripperCalibration PROPERTY FOLDER "sawIntuitiveResearchKit")

    # link against non cisst libraries and cisst components
    target_link_libraries (sawIntuitiveResearchKitGripperCalibration
                           ${sawRobotIO1394_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawIntuitiveResearchKitGripperCalibration ${REQUIRED_CISST_LIBRARIES})

  endif () # components found

endif (cisst_FOUND_AS_REQUIRED)
