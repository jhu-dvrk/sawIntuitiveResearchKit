cmake_minimum_required (VERSION 3.16)
project (dvrk_hrsv_widget VERSION 2.3.0)

# first test for ROS1
set (ROS1_depends roscpp std_msgs diagnostic_msgs sensor_msgs)

find_package (catkin QUIET
  COMPONENTS
  ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
  add_definitions (-DROS1)
  catkin_package (
    CATKIN_DEPENDS ${ROS1_depends})
  include_directories (${catkin_INCLUDE_DIRS})
else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS_VERSION 2)
    add_definitions (-DROS2)
    set (ROS2_depends rclcpp std_msgs diagnostic_msgs sensor_msgs)
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

# find Qt
find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)

set (CMAKE_AUTOMOC ON)
set (CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories (
  ${catkin_INCLUDE_DIRS}
  ${dvrk_hrsv_widget_SOURCE_DIR}/include
  ${dvrk_hrsv_widget_BINARY_DIR}/include
)

add_executable (hrsv_widget
  include/hrsv_widget.h
  src/hrsv_widget.cpp
  src/main.cpp)

target_link_libraries (
  hrsv_widget
  Qt5::Widgets
  ${catkin_LIBRARIES})


install (TARGETS hrsv_widget
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

if (ROS_VERSION EQUAL 2)
  ament_target_dependencies (
    hrsv_widget
    ${ROS2_depends}
    )
  ament_package ()
endif ()
