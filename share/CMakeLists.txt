cmake_minimum_required (VERSION 2.8.3)
project (dvrk_config)

find_package (catkin)
if (catkin_FOUND)
  catkin_package()
endif ()

install (DIRECTORY
         "${dvrk_config_SOURCE_DIR}"
         DESTINATION share/sawIntuitiveResearchKit
         COMPONENT sawIntuitiveResearchKit-Share
         PATTERN "CMakeLists.txt" EXCLUDE)
