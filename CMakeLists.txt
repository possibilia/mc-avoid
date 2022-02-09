cmake_minimum_required(VERSION 3.10)

project(ObstacleAvoid)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include(GNUInstallDirs)

find_library(a1lidarrpi NAMES a1lidarrpi HINTS "${CMAKE_INSTALL_LIBDIR}")
find_library(alphabot NAMES alphabot HINTS "${CMAKE_INSTALL_LIBDIR}")

set(LIBSRC control.cpp)
set(LIBINCLUDE control.h)

add_library(control STATIC ${LIBSRC})

add_executable(main main.cpp)
target_link_libraries(main control alphabot a1lidarrpi)
