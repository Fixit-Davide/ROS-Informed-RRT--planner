// Collision checks of an axis-aligned robot box against a snapshot of the occupancy map.
#ifndef UAV_NAV_CORE__COLLISION_WORLD_HPP_
#define UAV_NAV_CORE__COLLISION_WORLD_HPP_

#include <memory>

#include <Eigen/Core>

#include "uav_nav_core/occupancy_map.hpp"

namespace uav_nav_core
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
  Eigen::Vector3d boxSize() const {return 2.0 * half_;}
  // Discretised straight-line check between `a` and `b`.
  bool isSegmentFree(const Eigen::Vector3d & a, const Eigen::Vector3d & b, double step) const;

private:
  std::shared_ptr<const OccupancySnapshot> map_;
  Eigen::Vector3d half_;
};

}  // namespace uav_nav_core

#endif  // UAV_NAV_CORE__COLLISION_WORLD_HPP_
