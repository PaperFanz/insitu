cmake_minimum_required(VERSION 3.0.2)
project(adam_filter_package)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    qt_build 
    nodelet
    pluginlib
    roscpp
    insitu
    topic_tools
    ros_type_introspection
)
find_package(OpenCV 4.2 REQUIRED)
find_package(jsoncpp REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Qt5 COMPONENTS Core Gui Widgets)

catkin_package(
  LIBRARIES adam_filter_package
  CATKIN_DEPENDS nodelet pluginlib roscpp insitu
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    inc
    ${catkin_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

set(adam_filter_package_HDRS
    inc/Heartbeat/Heartbeat.hpp
    inc/Heartbeat/Heartbeat_dialog.hpp
)

qt5_wrap_cpp(adam_filter_package_MOCS ${adam_filter_package_HDRS})

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/Heartbeat/Heartbeat.cpp
    src/Heartbeat/Heartbeat_dialog.cpp

    ${adam_filter_package_MOCS}
)

target_link_libraries(${PROJECT_NAME} 
    jsoncpp_lib
    Qt5::Core
    Qt5::Widgets
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

## Install plugins
install(FILES plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
