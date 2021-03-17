cmake_minimum_required(VERSION 3.0.2)
project(insitu_plugins)

add_compile_options(-std=c++14 -fPIC)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    qt_build 
    nodelet
    pluginlib
    roscpp
    insitu
)
find_package(jsoncpp REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Qt5 COMPONENTS Core Gui Widgets)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES insitu_plugins
#  CATKIN_DEPENDS nodelet pluginlib roscpp
#  DEPENDS system_lib
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
    inc/Crosshair/Crosshair.hpp
    inc/Crosshair/Crosshair_dialog.hpp
)

qt5_wrap_cpp(insitu_plugins_MOCS ${insitu_plugins_HDRS})

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/Crosshair/Crosshair.cpp
    src/Crosshair/Crosshair_dialog.cpp

    ${insitu_plugins_MOCS}
)

target_link_libraries(${PROJECT_NAME} 
    jsoncpp_lib
    Qt5::Core
    Qt5::Widgets
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
