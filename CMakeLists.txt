cmake_minimum_required(VERSION 3.10)

set(LIBSRC control.cpp)

add_library(control STATIC ${LIBSRC})

# set the project name
project(ObstacleAvoid)

# add the executable
add_executable(main main.cpp)
target_link_libraries(main control)
