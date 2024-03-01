cmake_minimum_required (VERSION 3.10)
project (dvrk_config VERSION 2.2.1)

find_package (catkin QUIET)
if (catkin_FOUND)
  catkin_package()
endif ()

install (DIRECTORY
         "${dvrk_config_SOURCE_DIR}"
         DESTINATION share/sawIntuitiveResearchKit
         COMPONENT sawIntuitiveResearchKit-Share
         PATTERN "CMakeLists.txt" EXCLUDE)
