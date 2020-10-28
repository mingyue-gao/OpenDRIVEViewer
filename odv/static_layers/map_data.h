#ifndef ODR_VIEWER_MAP_DATA_H
#define ODR_VIEWER_MAP_DATA_H

#include <Eigen/Eigen>
#include <vector>

namespace odv {

struct MapData
{
  std::vector<Eigen::Matrix3Xd> lane_boundaries;
};


MapData load_map(const std::string& map_path);
  
} /* namespace odv */ 

#endif /* end of include guard: ODR_VIEWER_MAP_DATA_H */
