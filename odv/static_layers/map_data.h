#ifndef ODR_VIEWER_MAP_DATA_H
#define ODR_VIEWER_MAP_DATA_H

#include <Eigen/Eigen>
#include <QPointF>
#include <QVector>
#include <vector>
#include "carla/geom/Mesh.h"
#include "carla/road/element/LaneMarking.h"

namespace odv {

using idx_t = std::size_t;

using MarkColor = carla::road::element::LaneMarking::Color;
using MarkType = carla::road::element::LaneMarking::Type;

struct RoadMark
{
  MarkType type;
  MarkColor color;
  double width;
  double s_offset = 0.;
};

struct Boundary
{
  std::vector<QPointF> points;
  std::vector<RoadMark> road_marks;
};

struct Lane
{
  idx_t left_boundary;
  idx_t right_boundary;
  std::vector<std::pair<idx_t, idx_t>> vertices;
};

struct MapData
{
  std::vector<Boundary> lane_boundaries;
  std::vector<Lane> lanes;
  carla::geom::Mesh mesh;
};


MapData load_map(const std::string& map_path);
  
} /* namespace odv */ 

#endif /* end of include guard: ODR_VIEWER_MAP_DATA_H */
