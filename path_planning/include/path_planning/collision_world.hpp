// Collision checks of an axis-aligned robot box against a snapshot of the occupancy map.
#ifndef PATH_PLANNING__COLLISION_WORLD_HPP_
#define PATH_PLANNING__COLLISION_WORLD_HPP_

#include <memory>

#include <Eigen/Core>

#include "path_planning/occupancy_map.hpp"

namespace path_planning
{

class CollisionWorld
{
public:
  // `map` may be null or empty (no scan yet): every state is then considered free.
  CollisionWorld(std::shared_ptr<const OccupancySnapshot> map, const Eigen::Vector3d & box_size);

  bool hasMap() const {return map_ && map_->size() > 0;}
  // True if the box centred at `p` overlaps no occupied voxel (touching faces do not count).
  bool isFree(const Eigen::Vector3d & p) const;
  // Distance from the box centred at `p` to the closest occupied voxel (0 if overlapping).
  double clearance(const Eigen::Vector3d & p) const;
  // Discretised straight-line check between `a` and `b`.
  bool isSegmentFree(const Eigen::Vector3d & a, const Eigen::Vector3d & b, double step) const;

private:
  std::shared_ptr<const OccupancySnapshot> map_;
  Eigen::Vector3d half_;
};

}  // namespace path_planning

#endif  // PATH_PLANNING__COLLISION_WORLD_HPP_
