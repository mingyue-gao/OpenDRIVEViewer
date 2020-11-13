#include "map_data.h"

#include "carla/opendrive/OpenDriveParser.h"
#include "carla/road/Lane.h"
#include "carla/road/LaneSection.h"
#include "carla/road/Map.h"
#include "carla/road/MeshFactory.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <stdexcept>

namespace bf = boost::filesystem;

using namespace carla;

namespace {

static constexpr double sample_distance = 0.1; // unit m
  
}

namespace odv {

MapData load_map(const std::string& map_path)
{
  if (!bf::exists(map_path))
    throw std::runtime_error("File not found, " + map_path);

  MapData map_data;

  auto odr = opendrive::OpenDriveParser::LoadFile(map_path);
  auto meshes = odr->GenerateMesh(sample_distance);
  auto& odr_data = odr->GetMap();

  geom::MeshFactory mesh_factory;
  mesh_factory.road_param.resolution = static_cast<float>(sample_distance);
  mesh_factory.road_param.extra_lane_width = 0.;

  map_data.mesh += meshes;

  for (const auto& r : odr_data.GetRoads())
  {
    for (const auto& ls : r.second.GetLaneSections())
    {
      std::size_t lb_idx = map_data.lane_boundaries.size();
      for (const auto& l_pair : ls.GetLanes())
      {
        const auto& l = l_pair.second;
        if (l.GetId() == 0)
          continue;

        if (l.GetType() != road::Lane::LaneType::Driving)
          continue;

        const auto lane_meshes = mesh_factory.Generate(l);
        const auto& vertices = lane_meshes->GetVertices();
        auto n = vertices.size();
        assert(n % 2 == 0);
        int step = 2;
        std::vector<QPointF> lb(n / step);
        std::size_t i = l.GetId() < 0 ? 0 : 1;
        for (; i < n; i += step)
        {
          const auto& v = vertices[i];
          lb[i / step] = QPointF{v.x, v.y};
        }
        std::cout << "printing " << l.GetId() << ", " << n << ", " << lb.size() << std::endl;
        map_data.lane_boundaries.push_back(lb);

        if (l.GetId() == -1)
        {
          std::vector<QPointF> mlb(n / step);
          for (std::size_t i = 1; i < n; i += step)
          {
            const auto& v = vertices[i];
            mlb[i / step] = QPointF{v.x, v.y};
          }
          map_data.lane_boundaries.push_back(mlb);
        }
      }

      for (const auto& l_pair : ls.GetLanes())
      {
        const auto& l = l_pair.second;
        if (l.GetId() == 0)
          continue;

        if (l.GetType() != road::Lane::LaneType::Driving)
          continue;

        Lane the_lane;
        auto right_idx = lb_idx;
        auto left_idx = lb_idx + 1;
        the_lane.right_boundary = lb_idx;
        the_lane.left_boundary = lb_idx + 1;

        auto& lane_bs = map_data.lane_boundaries;
        auto& right_b = lane_bs[right_idx];
        auto& left_b = lane_bs[left_idx];
        auto n = right_b.size();
        std::vector<std::pair<idx_t, idx_t>> lane_meshes;
        for (std::size_t i = 0; i < n; ++i)
        {
          lane_meshes.emplace_back(right_idx, i);
          lane_meshes.emplace_back(left_idx, i);
        }
        the_lane.meshes = lane_meshes;

        assert(lb_idx + 1 < map_data.lane_boundaries.size());
        ++lb_idx;
        map_data.lanes.push_back(the_lane);
      }
    }
  }

  return map_data;
}

} /* namespace odv */ 
