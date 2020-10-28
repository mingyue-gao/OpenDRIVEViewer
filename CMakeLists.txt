cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

project(odv)

set(CMAKE_POSITION_INDEPENDENT_CODE FALSE)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -no-pie")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(third-party)
add_subdirectory(carla)
add_subdirectory(odv)
add_subdirectory(basicdrawing)
