#include "path_planning/collision_world.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace path_planning
{

CollisionWorld::CollisionWorld(
  std::shared_ptr<const octomap::OcTree> tree,
  std::shared_ptr<fcl::CollisionGeometryd> robot)
: octree_(std::move(tree)), robot_(std::move(robot))
{
  if (octree_ && octree_->size() > 0) {
    auto geom = std::make_shared<fcl::OcTreed>(octree_);
    tree_obj_ = std::make_unique<fcl::CollisionObjectd>(geom);
  }
}

bool CollisionWorld::isFree(const Eigen::Vector3d & p) const
{
  if (!tree_obj_) {return true;}
  fcl::Transform3d tf = fcl::Transform3d::Identity();
  tf.translation() = p;
  fcl::CollisionObjectd robot_obj(robot_, tf);
  fcl::CollisionRequestd request(1, false);
  fcl::CollisionResultd result;
  fcl::collide(&robot_obj, tree_obj_.get(), request, result);
  return !result.isCollision();
}

double CollisionWorld::clearance(const Eigen::Vector3d & p) const
{
  if (!tree_obj_) {return std::numeric_limits<double>::max();}
  fcl::Transform3d tf = fcl::Transform3d::Identity();
  tf.translation() = p;
  fcl::CollisionObjectd robot_obj(robot_, tf);
  fcl::DistanceRequestd request;
  fcl::DistanceResultd result;
  fcl::distance(&robot_obj, tree_obj_.get(), request, result);
  return std::max(0.0, result.min_distance);
}

bool CollisionWorld::isSegmentFree(
  const Eigen::Vector3d & a, const Eigen::Vector3d & b, double step) const
{
  if (!tree_obj_) {return true;}
  const double len = (b - a).norm();
  const int n = std::max(1, static_cast<int>(std::ceil(len / std::max(step, 1e-3))));
  for (int i = 0; i <= n; ++i) {
    if (!isFree(a + (b - a) * (static_cast<double>(i) / n))) {return false;}
  }
  return true;
}

}  // namespace path_planning
