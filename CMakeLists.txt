cmake_minimum_required(VERSION 3.10)

project(ObstacleAvoid)

find_package(Threads)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_options(-Wall)

include(GNUInstallDirs)

set(LIBSRC control.cpp)
set(LIBINCLUDE control.h)

add_library(control STATIC ${LIBSRC})

target_link_libraries(control ${CMAKE_THRED_LIBS_INIT} pigpio rt cpptimer)

find_library(a1lidarrpi NAMES a1lidarrpi HINTS "${CMAKE_INSTALL_LIBDIR}")
find_library(alphabot NAMES alphabot HINTS "${CMAKE_INSTALL_LIBDIR}")

add_executable(main main.cpp)
target_link_libraries(main control alphabot a1lidarrpi)
