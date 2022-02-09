cmake_minimum_required(VERSION 3.10)

project(ObstacleAvoid)

set(LIBSRC control.cpp)

add_library(control STATIC ${LIBSRC})

add_executable(main main.cpp)
target_link_libraries(main control)
