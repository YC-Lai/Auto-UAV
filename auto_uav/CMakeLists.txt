cmake_minimum_required(VERSION 3.0.2)
project(auto_uav)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  auto_uav_msgs
  geometry_msgs
  mavros_msgs
  message_generation
  pluginlib
  roscpp
  rospy
  rviz
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES auto_uav
#  CATKIN_DEPENDS auto_uav_msgs geometry_msgs mavros_msgs message_generation pluginlib roscpp rospy rviz std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}       # To include UI header file
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

# Avoid keyword definition to avaid conflicts with boost or xapian etc
# e.g. http://muddyazian.blogspot.de/2012/04/getting-qt-app-working-with-boost-using.html
add_definitions(-DQT_NO_KEYWORDS)

# Define gui source file
set(${PROJECT_NAME}_SRCS
  src/gcsGui.cpp
  src/gcsLogi.cpp
)

# Define gui header file
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/gcsGui.hpp
  include/${PROJECT_NAME}/gcsLogi.hpp
)

# Define ui file
set(${PROJECT_NAME}_UIS
  resource/gcsGui.ui
)

# Create header from ui file (uic)
if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Generate header for ui with rviz_QT_VERSION: ${rviz_QT_VERSION}")
    qt4_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
    qt4_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
else()
    message(STATUS "Generate header for ui with rviz_QT_VERSION: ${rviz_QT_VERSION}")
    qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
    qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
endif()

## Add library is needed in order to generate the header file from ui file.
add_library(gcsGui
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_UIS_H}
  ${${PROJECT_NAME}_MOCS} 
)

target_link_libraries(gcsGui
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

# Define server source & header file
set(${PROJECT_NAME}_SERVER_SRCS
  src/server_node.cpp
  src/server.cpp
  include/${PROJECT_NAME}/server.hpp
)

add_executable(server_node ${${PROJECT_NAME}_SERVER_SRCS})
target_link_libraries(server_node ${catkin_LIBRARIES})