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

catkin_package(
    INCLUDE_DIRS inc
)

include_directories(
    inc/insitu
    inc/insitu_common
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

# Sources
set(INSITU_SRC
    src/insitu/main.cpp
    src/insitu/main_window.cpp
    src/insitu/mainwindowdesign.cpp
    src/insitu/addmodedialog.cpp
    src/insitu/addviewdialog.cpp

    inc/insitu/main_window.hpp
    inc/insitu/mainwindowdesign.hpp
    inc/insitu/addmodedialog.hpp
    inc/insitu/addviewdialog.hpp
)

set(INSITU_COMMON_SRC
    src/insitu_common/filter.cpp
    src/insitu_common/mode.cpp
    src/insitu_common/model.cpp
    src/insitu_common/view.cpp

    inc/insitu_common/filter.hpp
    inc/insitu_common/mode.hpp
    inc/insitu_common/model.hpp
    inc/insitu_common/view.hpp
)

# Binaries
add_executable(insitu
    ${INSITU_SRC} ${INSITU_COMMON_SRC}
    ${QT_RESOURCES_CPP} ${QT_MOC_HPP}
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

