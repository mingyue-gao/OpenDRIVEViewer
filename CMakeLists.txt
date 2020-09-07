cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

project(odv)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

add_subdirectory(third-party)
add_subdirectory(carla)
add_subdirectory(odv)
