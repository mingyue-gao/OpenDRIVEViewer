cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

add_executable(odv_viewer
  viewer.cpp
  )

set_target_properties(odv_viewer PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS OFF
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
  FOLDER OpenDRIVEViewer3D/odv
  )

target_link_libraries(odv_viewer
  PRIVATE $<BUILD_INTERFACE:odv_fr>
  )

target_include_directories(odv_viewer
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../third-party>
  )
