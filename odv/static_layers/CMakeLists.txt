cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

add_library(static_layers STATIC
  map_data.cpp
  )

set_target_properties(static_layers PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS OFF
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
  )

target_link_libraries(static_layers
  PUBLIC $<BUILD_INTERFACE:odv_fr>
  PUBLIC $<BUILD_INTERFACE:CONAN_PKG::eigen>
  PRIVATE $<BUILD_INTERFACE:CONAN_PKG::boost>
  )
