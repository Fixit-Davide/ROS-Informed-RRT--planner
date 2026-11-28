#include "uav_nav_core/voxel_map.hpp"

#include <vector>

namespace uav_nav_core
{

void toMsg(const VoxelMap & map, uav_nav_msgs::msg::VoxelMap & msg)
{
  msg.header = map.header;
  msg.cells.clear();
  if (!map.snapshot) {
    msg.resolution = 0.0;
    return;
  }
  msg.resolution = map.snapshot->resolution();
  msg.cells.reserve(3 * map.snapshot->size());
  for (const auto & c : map.snapshot->voxels()) {
    msg.cells.push_back(c.x);
    msg.cells.push_back(c.y);
    msg.cells.push_back(c.z);
  }
}

void fromMsg(const uav_nav_msgs::msg::VoxelMap & msg, VoxelMap & map)
{
  map.header = msg.header;
  std::vector<Bonxai::CoordT> cells;
  cells.reserve(msg.cells.size() / 3);
  for (size_t i = 0; i + 2 < msg.cells.size(); i += 3) {
    cells.push_back({msg.cells[i], msg.cells[i + 1], msg.cells[i + 2]});
  }
  // A zero resolution would make every distance query divide by zero: keep the map empty.
  if (!(msg.resolution > 0.0)) {cells.clear();}
  map.snapshot = std::make_shared<const OccupancySnapshot>(
    msg.resolution > 0.0 ? msg.resolution : 0.2, cells);
}

}  // namespace uav_nav_core
