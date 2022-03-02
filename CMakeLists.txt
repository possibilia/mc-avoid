cmake_minimum_required(VERSION 3.10)

project(ObstacleAvoid)

find_package(Threads)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_options(-Wall)

include(GNUInstallDirs)

find_library(alphabot NAMES alphabot HINTS "${CMAKE_INSTALL_LIBDIR}")
find_library(a1lidarrpi NAMES a1lidarrpi HINTS "${CMAKE_INSTALL_LIBDIR}")

target_link_libraries(a1lidarrpi ${CMAKE_THREAD_LIBS_INIT} pigpio rt)

add_executable(main main.cpp)
target_link_libraries(main alphabot a1lidarrpi)
