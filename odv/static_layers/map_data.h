#ifndef ODR_VIEWER_MAP_DATA_H
#define ODR_VIEWER_MAP_DATA_H

#include <Eigen/Eigen>
#include <QPointF>
#include <vector>
#include "carla/geom/Mesh.h"

namespace odv {

using idx_t = std::size_t;

struct Boundary
{

};

struct Lane
{
  idx_t left_boundary;
  idx_t right_boundary;
  std::vector<std::pair<idx_t, idx_t>> meshes;
};

struct MapData
{
  std::vector<std::vector<QPointF>> lane_boundaries;
  std::vector<Lane> lanes;
  carla::geom::Mesh mesh;
};


MapData load_map(const std::string& map_path);
  
} /* namespace odv */ 

#endif /* end of include guard: ODR_VIEWER_MAP_DATA_H */
