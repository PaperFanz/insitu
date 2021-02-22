cmake_minimum_required(VERSION 3.0.2)
project(insitu_plugins)

add_compile_options(-std=c++11 -fPIC)

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

set(insitu_plugin_HDRS
    inc/${PROJECT_NAME}/text_filter.hpp
    inc/${PROJECT_NAME}/crosshair.hpp
    inc/${PROJECT_NAME}/crosshair_dialog.hpp
)

qt5_wrap_cpp(insitu_plugin_MOCS ${insitu_plugin_HDRS})

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}/text_filter.cpp
    src/${PROJECT_NAME}/crosshair.cpp
    src/${PROJECT_NAME}/crosshair_dialog.cpp

    ${insitu_plugin_MOCS}
)

target_link_libraries(${PROJECT_NAME} 
    Qt5::Core
    Qt5::Widgets
    ${catkin_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

## Install plugins
install(FILES insitu_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
