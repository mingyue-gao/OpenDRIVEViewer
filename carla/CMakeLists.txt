cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

add_library(odv_fr STATIC
  geom/BoundingBox.h
  geom/CubicPolynomial.h
  geom/GeoLocation.cpp
  geom/GeoLocation.h
  geom/Location.h
  geom/Math.cpp
  geom/Math.h
  geom/Mesh.cpp
  geom/Mesh.h
  geom/Rotation.h
  geom/Rtree.h
  geom/Transform.h
  geom/Vector2D.h
  geom/Vector3D.h
  opendrive/parser/ControllerParser.cpp
  opendrive/parser/ControllerParser.h
  opendrive/parser/GeometryParser.cpp
  opendrive/parser/GeometryParser.h
  opendrive/parser/GeoReferenceParser.cpp
  opendrive/parser/GeoReferenceParser.h
  opendrive/parser/JunctionParser.cpp
  opendrive/parser/JunctionParser.h
  opendrive/parser/LaneParser.cpp
  opendrive/parser/LaneParser.h
  opendrive/parser/ObjectParser.cpp
  opendrive/parser/ObjectParser.h
  opendrive/parser/ProfilesParser.cpp
  opendrive/parser/ProfilesParser.h
  opendrive/parser/RoadParser.cpp
  opendrive/parser/RoadParser.h
  opendrive/parser/SignalParser.cpp
  opendrive/parser/SignalParser.h
  opendrive/parser/TrafficGroupParser.cpp
  opendrive/parser/TrafficGroupParser.h
  opendrive/OpenDriveParser.cpp
  opendrive/OpenDriveParser.h
  road/element/Geometry.cpp
  road/element/Geometry.h
  road/element/LaneCrossingCalculator.cpp
  road/element/LaneCrossingCalculator.h
  road/element/LaneMarking.cpp
  road/element/LaneMarking.h
  road/element/RoadInfo.h
  road/element/RoadInfoCrosswalk.h
  road/element/RoadInfoElevation.h
  road/element/RoadInfoGeometry.h
  road/element/RoadInfoIterator.h
  road/element/RoadInfoLaneAccess.h
  road/element/RoadInfoLaneBorder.h
  road/element/RoadInfoLaneHeight.h
  road/element/RoadInfoLaneMaterial.h
  road/element/RoadInfoLaneOffset.h
  road/element/RoadInfoLaneRule.h
  road/element/RoadInfoLaneVisibility.h
  road/element/RoadInfoLaneWidth.h
  road/element/RoadInfoMarkRecord.h
  road/element/RoadInfoMarkTypeLine.h
  road/element/RoadInfoSignal.h
  road/element/RoadInfoSpeed.h
  road/element/RoadInfoVisitor.h
  road/element/Waypoint.cpp
  road/element/Waypoint.h
  road/object/RepeatRecord.h
  road/Controller.h
  road/InformationSet.h
  road/Junction.h
  road/Lane.cpp
  road/Lane.h
  road/LaneSection.cpp
  road/LaneSection.h
  road/LaneSectionMap.h
  road/LaneValidity.h
  road/Map.cpp
  road/Map.h
  road/MapBuilder.cpp
  road/MapBuilder.h
  road/MapData.cpp
  road/MapData.h
  road/MeshFactory.cpp
  road/MeshFactory.h
  road/Object.h
  road/Road.cpp
  road/Road.h
  road/RoadElementSet.h
  road/RoadTypes.h
  road/Signal.h
  road/SignalType.cpp
  road/SignalType.h
  AtomicList.h
  AtomicSharedPtr.h
  # Buffer.cpp
  # Buffer.h
  # BufferPool.h
  Debug.h
  Exception.cpp
  Exception.h
  FileSystem.cpp
  FileSystem.h
  Functional.h
  Iterator.h
  ListView.h
  Logging.h
  Memory.h
  MoveHandler.h
  NonCopyable.h
  Platform.h
  PythonUtil.h
  RecurrentSharedFuture.h
  Sockets.h
  StopWatch.h
  StringUtil.cpp
  StringUtil.h
  ThreadGroup.h
  ThreadPool.h
  Time.h
  TypeTraits.h
  )

set_target_properties(odv_fr PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS OFF
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
  FOLDER OpenDRIVEViewer3D/carla
  )

target_link_libraries(odv_fr
  PUBLIC $<BUILD_INTERFACE:CONAN_PKG::boost>
  PUBLIC $<BUILD_INTERFACE:odv_3rdparty>
  )

target_include_directories(odv_fr
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../third-party>
  )
