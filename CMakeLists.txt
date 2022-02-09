cmake_minimum_required(VERSION 3.10)

project(ObstacleAvoid)

include(GNUInstallDirs)

find_package(a1lidarrpi)
find_package(alphabot)

set(LIBSRC control.cpp)
set(LIBINCLUDE control.h)

add_library(control STATIC ${LIBSRC})

set_target_properties(control PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  PUBLIC_HEADER "${LIBINCLUDE}")

target_link_libraries(control a1lidarrpi alphabot)

add_executable(main main.cpp)
target_link_libraries(main control)
