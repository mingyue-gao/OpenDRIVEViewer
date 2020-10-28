#include "map_data.h"
#include <Eigen/src/Core/Matrix.h>

#include "carla/opendrive/OpenDriveParser.h"
#include "carla/road/Map.h"
#include "carla/road/MeshFactory.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <stdexcept>

namespace bf = boost::filesystem;

using namespace carla;

namespace {

static constexpr double sample_distance = 1.0; // unit m
  
}

namespace odv {

MapData load_map(const std::string& map_path)
{
  if (!bf::exists(map_path))
    throw std::runtime_error("File not found, " + map_path);

  MapData map_data;

  auto odr = opendrive::OpenDriveParser::LoadFile(map_path);
  auto meshes = odr->GenerateMesh(1.);
  auto& odr_data = odr->GetMap();

  geom::MeshFactory mesh_factory;
  mesh_factory.road_param.resolution = static_cast<float>(sample_distance);
  mesh_factory.road_param.extra_lane_width = 0.;

  for (const auto& r : odr_data.GetRoads())
  {
    if (r.first != 508)
      continue;

    for (const auto& ls : r.second.GetLaneSections())
    {
      for (const auto& l_pair : ls.GetLanes())
      {
        const auto& l = l_pair.second;
        const auto lane_meshes = mesh_factory.Generate(l);
        const auto& vertices = lane_meshes->GetVertices();
        auto n = vertices.size();
        assert(n % 2 == 0);
        int step = 1;
        Eigen::Matrix3Xd lb(3, n / step);
        for (std::size_t i = 0; i < n; i += step)
        {
          const auto& v = vertices[i];
          lb.col(i / step) = Eigen::Vector3d{v.x, v.y, v.z};
        }
        map_data.lane_boundaries.push_back(lb);
      }
    }
  }

  return map_data;
}

} /* namespace odv */ 
