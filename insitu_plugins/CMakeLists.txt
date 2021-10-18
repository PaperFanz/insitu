cmake_minimum_required(VERSION 3.0.2)
project(insitu_plugins)

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
    insitu_utils
    topic_tools
    ros_type_introspection
)
find_package(OpenCV 4.2 REQUIRED)
find_package(jsoncpp REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Qt5 COMPONENTS Core Gui Widgets)

catkin_package(
  LIBRARIES insitu_plugins
  CATKIN_DEPENDS nodelet pluginlib roscpp insitu insitu_utils
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

set(insitu_plugins_HDRS
    inc/Label/Label.hpp

    inc/Crosshair/Crosshair.hpp

    inc/Transparent/Transparent.hpp

    inc/Notify/Notify.hpp

    inc/Stats/Stats.hpp
    
    inc/Heartbeat/Heartbeat.hpp
)

set(insitu_dialogs_HDRS
    inc/Label/Label_dialog.hpp

    inc/Crosshair/Crosshair_dialog.hpp

    inc/Transparent/Transparent_dialog.hpp

    inc/Notify/Notify_dialog.hpp

    inc/Stats/Stats_dialog.hpp
    
    inc/Heartbeat/Heartbeat_dialog.hpp
)

qt5_wrap_cpp(insitu_dialogs_MOCS ${insitu_dialogs_HDRS})

## Declare a C++ library
add_library(${PROJECT_NAME}
    ${insitu_dialogs_MOCS}
    ${insitu_plugins_HDRS}

    src/Label/Label.cpp
    src/Label/Label_dialog.cpp

    src/Crosshair/Crosshair.cpp
    src/Crosshair/Crosshair_dialog.cpp

    src/Transparent/Transparent.cpp
    src/Transparent/Transparent_dialog.cpp

    src/Notify/Notify.cpp
    src/Notify/Notify_dialog.cpp

    src/Stats/Stats.cpp
    src/Stats/Stats_dialog.cpp

    src/Heartbeat/Heartbeat.cpp
    src/Heartbeat/Heartbeat_dialog.cpp
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

