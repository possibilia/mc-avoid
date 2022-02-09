cmake_minimum_required(VERSION 3.10)

project(ObstacleAvoid)

set(LIBSRC 
	control.cpp 
	../alphabot/alphabot.cpp 
	../rplidar_rpi/a1lidarrpi.cpp)

add_library(control STATIC ${LIBSRC})

add_executable(main main.cpp)
target_link_libraries(main control)
