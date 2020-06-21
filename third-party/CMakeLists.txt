cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

project(odv_3rdparty)

add_library(odv_3rdparty STATIC
  moodycamel/ConcurrentQueue.h
  odrSpiral/odrSpiral.cpp
  odrSpiral/odrSpiral.h
  pugixml/pugiconfig.hpp
  pugixml/pugixml.cpp
  pugixml/pugixml.hpp
  )

set_target_properties(odv_3rdparty PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS OFF
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
  FOLDER OpenDRIVEViewer3D/third-party
  )
