# CMAKE
cmake_minimum_required(VERSION 3.0.2)
project(insitu)

add_subdirectory(lib)

# CATKIN
# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp)
include_directories(${catkin_INCLUDE_DIRS})
add_compile_options(-std=c++11 -fPIC)

# specify the C++ standard
set(CMAKE_CXX_COMPILER "/usr/bin/clang++-10")
set(CMAKE_C_COMPILER "/usr/bin/clang-10")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# QT ENVIRONMENT
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# non-ROS:
find_package(Qt5 COMPONENTS Core Gui Widgets)

# other resources
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} res/*.qrc)
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})

# ROS components
find_package(catkin REQUIRED COMPONENTS
    image_transport
    sensor_msgs
    geometry_msgs
    cv_bridge
)

catkin_package(
    INCLUDE_DIRS inc
)

include_directories(
    inc
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

# Sources
set(INSITU_SRC
    src/main.cpp
    src/main_window.cpp
    src/mainwindowdesign.cpp
    src/addmodedialog.cpp
    src/addviewdialog.cpp
    src/insitu_utils.cpp
    src/filtered_view.cpp

    inc/main_window.hpp
    inc/mainwindowdesign.hpp
    inc/addmodedialog.hpp
    inc/addviewdialog.hpp
    inc/insitu_utils.hpp
    inc/filtered_view.hpp
)

# Binaries
add_executable(insitu
    ${INSITU_SRC}
    ${QT_RESOURCES_CPP}
    ${QT_MOC_HPP}
)

target_link_libraries(insitu 
    Qt5::Core
    Qt5::Widgets
    ${catkin_LIBRARIES}
)

# catkin install dirs
install(TARGETS insitu
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

