cmake_minimum_required(VERSION 3.10)

project(ObstacleAvoid)

find_package(Threads)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_options(-Wall)

include(GNUInstallDirs)

find_package(Boost COMPONENTS format)

find_library(alphabot NAMES alphabot HINTS "${CMAKE_INSTALL_LIBDIR}")
find_library(a1lidarrpi NAMES a1lidarrpi HINTS "${CMAKE_INSTALL_LIBDIR}")

add_executable(test test.cpp)
target_link_libraries(test alphabot a1lidarrpi pigpio pthread, boost)

add_executable(exp exp.cpp)
target_link_libraries(exp alphabot a1lidarrpi pigpio pthread)
