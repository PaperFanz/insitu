# CMAKE
cmake_minimum_required(VERSION 3.0.2)
project(insitu)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# CATKIN
# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS 
    qt_build 
    roscpp
)
find_package(OpenCV 4.2 REQUIRED)
find_package(jsoncpp REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

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
    pluginlib
    nodelet
)

catkin_package(
    INCLUDE_DIRS inc lib
    DEPENDS OpenCV
)

include_directories(
    inc
    lib
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

# Sources
set(INSITU_SRC
    src/main.cpp
    src/main_window.cpp
    src/main_window_design.cpp
    src/add_mode_dialog.cpp
    src/add_view_dialog.cpp
    src/add_filter_dialog.cpp
    src/insitu_utils.cpp
    src/mode_container.cpp
    src/filtered_view.cpp
    src/filter_factory.cpp
    src/filter_card.cpp
    src/filter_tree_item.cpp
    src/filter_properties.cpp
    src/filter_graphics_item.cpp
    src/filter_graphics_view.cpp
    src/filter_graphics_scene.cpp

    inc/main_window.hpp
    inc/main_window_design.hpp
    inc/add_mode_dialog.hpp
    inc/add_view_dialog.hpp
    inc/add_filter_dialog.hpp
    inc/insitu_utils.hpp
    inc/mode_container.hpp
    inc/filtered_view.hpp
    inc/filter_factory.hpp
    inc/filter_card.hpp
    inc/filter_tree_item.hpp
    inc/filter_properties.hpp
    inc/filter_graphics_item.hpp
    inc/filter_graphics_view.hpp
    inc/filter_graphics_scene.hpp

    lib/insitu/filter.hpp
)

# Binaries
add_executable(${PROJECT_NAME}
    ${INSITU_SRC}
    ${QT_RESOURCES_CPP}
    ${QT_MOC_HPP}
)

target_link_libraries(${PROJECT_NAME} 
    jsoncpp_lib
    Qt5::Core
    Qt5::Widgets
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)

# catkin install dirs
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY lib/insitu/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
)

catkin_install_python(PROGRAMS scripts/new_insitu_filter
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

