cmake_minimum_required(VERSION 3.0.2)
project(insitu_plugins)

set(CMAKE_CXX_STANDARD 14)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    qt_build 
    nodelet
    pluginlib
    roscpp
    insitu
    tf2
    tf2_msgs
    tf2_ros
    tf2_geometry_msgs
)
find_package(jsoncpp REQUIRED)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Qt5 COMPONENTS Core Gui Widgets)
find_package(Eigen3 REQUIRED)

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
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

set(insitu_plugins_HDRS
    inc/Label/Label.hpp
    inc/Label/Label_dialog.hpp

    inc/Crosshair/Crosshair.hpp
    inc/Crosshair/Crosshair_dialog.hpp

    inc/Transparent/Transparent.hpp
    inc/Transparent/Transparent_dialog.hpp

    inc/Manipulation/Manipulation.hpp
    inc/Manipulation/Manipulation_dialog.hpp

    inc/Manip_boundary/Manip_boundary.hpp
    inc/Manip_boundary/Manip_boundary_dialog.hpp

)

qt5_wrap_cpp(insitu_plugins_MOCS ${insitu_plugins_HDRS})

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/Label/Label.cpp
    src/Label/Label_dialog.cpp

    src/Crosshair/Crosshair.cpp
    src/Crosshair/Crosshair_dialog.cpp

    src/Transparent/Transparent.cpp
    src/Transparent/Transparent_dialog.cpp

    src/Manipulation/Manipulation.cpp
    src/Manipulation/Manipulation_dialog.cpp

    src/Manip_boundary/Manip_boundary.cpp
    src/Manip_boundary/Manip_boundary_dialog.cpp


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
