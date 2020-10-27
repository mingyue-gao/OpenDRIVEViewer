cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(odv_viewer
  renderarea.cpp
  viewer.cpp
  )

set_target_properties(odv_viewer PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS OFF
  POSITION_INDEPENDENT_CODE OFF
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
  )

target_link_libraries(odv_viewer
  PRIVATE $<BUILD_INTERFACE:odv_fr>
  PRIVATE $<BUILD_INTERFACE:Qt5::Widgets>
  )

target_include_directories(odv_viewer
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../third-party>
  )
