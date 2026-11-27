// Immutable snapshot of the octomap wrapped as an FCL collision object.
#ifndef PATH_PLANNING__COLLISION_WORLD_HPP_
#define PATH_PLANNING__COLLISION_WORLD_HPP_

#include <memory>

#include <Eigen/Core>
#include <fcl/fcl.h>
#include <octomap/octomap.h>

namespace path_planning
{

class CollisionWorld
{
public:
  // `tree` may be null (no map received yet): every state is then considered free.
  CollisionWorld(
    std::shared_ptr<const octomap::OcTree> tree,
    std::shared_ptr<fcl::CollisionGeometryd> robot);

  bool hasMap() const {return static_cast<bool>(tree_obj_);}

  // True if the robot geometry centred at `p` (axis aligned) does not hit an occupied voxel.
  bool isFree(const Eigen::Vector3d & p) const;

  // Distance from the robot geometry centred at `p` to the closest occupied voxel.
  double clearance(const Eigen::Vector3d & p) const;

  // Discretised straight-line check between `a` and `b`.
  bool isSegmentFree(const Eigen::Vector3d & a, const Eigen::Vector3d & b, double step) const;

private:
  std::shared_ptr<const octomap::OcTree> octree_;
  std::shared_ptr<fcl::CollisionGeometryd> robot_;
  std::unique_ptr<fcl::CollisionObjectd> tree_obj_;
};

}  // namespace path_planning

#endif  // PATH_PLANNING__COLLISION_WORLD_HPP_
