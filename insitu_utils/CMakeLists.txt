# CMAKE
cmake_minimum_required(VERSION 3.0.2)
project(insitu_utils)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# CATKIN
find_package(catkin REQUIRED)
find_package(OpenCV 4.2 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS lib
    CATKIN_DEPENDS insitu
    DEPENDS OpenCV
)

include_directories(
    lib
)

# Sources
set(INSITU_UTILS_SRC
    lib/insitu_utils/painter.hpp
    src/painter.cpp
)

# Binaries
add_library(${PROJECT_NAME}
    ${INSITU_UTILS_SRC}
)

target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)

# catkin install dirs
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
)

install(DIRECTORY lib/insitu_utils/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
)

