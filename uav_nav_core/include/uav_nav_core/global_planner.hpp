// Plugin interface of the planner server's global stage (like nav2_core::GlobalPlanner, but
// 3D and against a CollisionWorld instead of a costmap).
#ifndef UAV_NAV_CORE__GLOBAL_PLANNER_HPP_
#define UAV_NAV_CORE__GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "uav_nav_core/collision_world.hpp"

namespace uav_nav_core
{

struct Bounds
{
  Eigen::Vector3d min, max;
};

class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<GlobalPlanner>;
  virtual ~GlobalPlanner() = default;

  // Declare and read the parameters, which live under "<name>." on the server node.
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name) = 0;
  virtual void cleanup() {}

  // Route from start to goal inside bounds whose segments are collision free for the box of
  // `world` (isSegmentFree), or empty if there is none. Called from a worker thread.
  virtual std::vector<Eigen::Vector3d> plan(
    const CollisionWorld & world, const Bounds & bounds, const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal) = 0;
};

}  // namespace uav_nav_core

#endif  // UAV_NAV_CORE__GLOBAL_PLANNER_HPP_
