cmake_minimum_required(VERSION 3.10)

project(ObstacleAvoid)

find_package(Threads)
find_package(CURL REQUIRED) 

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_options(-Wall)

include(GNUInstallDirs)

find_library(alphabot NAMES alphabot HINTS "${CMAKE_INSTALL_LIBDIR}")
find_library(a1lidarrpi NAMES a1lidarrpi HINTS "${CMAKE_INSTALL_LIBDIR}")

add_executable(test test.cpp)
include_directories(${CURL_INCLUDE_DIR})
target_link_libraries(test alphabot a1lidarrpi pigpio pthread ${CURL_LIBRARIES})

add_executable(exp exp.cpp)
target_link_libraries(exp alphabot a1lidarrpi pigpio pthread)

add_executable(control control.cpp)
target_link_libraries(control alphabot a1lidarrpi pigpio pthread)

add_executable(runaway runaway.cpp)
target_link_libraries(runaway alphabot a1lidarrpi pigpio pthread)

add_executable(curlTest curlTest.cpp)
target_link_libraries(curlTest ${CURL_LIBRARIES})

add_executable(step_exp step_exp.cpp)
target_link_libraries(step_exp alphabot a1lidarrpi pigpio pthread)

