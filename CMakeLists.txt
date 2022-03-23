cmake_minimum_required(VERSION 3.10)

project(ObstacleAvoid)

find_package(Threads)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_options(-Wall)

include(GNUInstallDirs)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.74 REQUIRED COMPONENTS format)
message(STATUS "Boost version: ${Boost_VERSION}")

if(NOT TARGET Boost::format)
    add_library(Boost::format IMPORTED INTERFACE)
    set_property(TARGET Boost::format PROPERTY
        INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})
    set_property(TARGET Boost::format PROPERTY
        INTERFACE_LINK_LIBRARIES ${Boost_LIBRARIES})
endif()

find_library(alphabot NAMES alphabot HINTS "${CMAKE_INSTALL_LIBDIR}")
find_library(a1lidarrpi NAMES a1lidarrpi HINTS "${CMAKE_INSTALL_LIBDIR}")

add_executable(test test.cpp)
target_link_libraries(test alphabot a1lidarrpi pigpio pthread PUBLIC Boost::format)

add_executable(exp exp.cpp)
target_link_libraries(exp alphabot a1lidarrpi pigpio pthread)
