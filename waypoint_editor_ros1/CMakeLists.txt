cmake_minimum_required(VERSION 3.0.2)
project(waypoint_editor)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  pluginlib
  interactive_markers
  visualization_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  std_msgs
  actionlib
  move_base_msgs
)

find_package(Qt5 REQUIRED COMPONENTS Widgets Concurrent)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rviz
    pluginlib
    interactive_markers
    visualization_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_srvs
    std_msgs
    actionlib
    move_base_msgs
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Explicit MOC for Q_OBJECT headers in include/ (AUTOMOC may miss them)
qt5_wrap_cpp(MOC_FILES
  include/waypoint_editor/rviz/waypoint_editor_panel.hpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# RViz plugin shared library
add_library(${PROJECT_NAME} SHARED
  src/core/waypoint_sequence.cpp
  src/io/waypoint_csv.cpp
  src/io/waypoint_yaml.cpp
  src/io/waypoint_nav_yaml.cpp
  src/rviz/waypoint_editor_tool.cpp
  src/rviz/waypoint_editor_panel.cpp
  ${MOC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Concurrent
  ${YAML_CPP_LIBRARIES}
)

# waypoint_follower executable
add_executable(waypoint_follower_node
  src/waypoint_follower_node.cpp
  src/io/waypoint_nav_yaml.cpp
  src/core/waypoint_sequence.cpp
)

target_link_libraries(waypoint_follower_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS waypoint_follower_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY icons launch rviz data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
