#ifndef ODR_VIEWER_MAP_DATA_H
#define ODR_VIEWER_MAP_DATA_H

#include <Eigen/Eigen>
#include <QPointF>
#include <vector>
#include "carla/geom/Mesh.h"

namespace odv {

struct MapData
{
  std::vector<std::vector<QPointF>> lane_boundaries;
  carla::geom::Mesh mesh;
};


MapData load_map(const std::string& map_path);
  
} /* namespace odv */ 

#endif /* end of include guard: ODR_VIEWER_MAP_DATA_H */
