cmake_minimum_required(VERSION 3.10.0)

project(lib LANGUAGES CXX)
include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 11)

add_compile_options(-Wall)

#find_package (Threads REQUIRED)
#find_package (Curses REQUIRED)
#find_package (OpenCV REQUIRED)
#find_package (iir REQUIRED)

#include_directories(${OpenCV_INCLUDE_DIRS})

add_library(lib
  STATIC
  lib.cpp
  )

# target_link_libraries(lib box2d iir::iir_static)

set_target_properties(lib PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  PUBLIC_HEADER lib.h)

target_include_directories(lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
