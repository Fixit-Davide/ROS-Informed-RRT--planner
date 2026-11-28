// Plugin interface of the planner server's local stage: the flyable path along the first
// part of the global route, inside a window around it.
#ifndef UAV_NAV_CORE__LOCAL_PLANNER_HPP_
#define UAV_NAV_CORE__LOCAL_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "uav_nav_core/collision_world.hpp"
#include "uav_nav_core/global_planner.hpp"

namespace uav_nav_core
{

class LocalPlanner
{
public:
  using Ptr = std::shared_ptr<LocalPlanner>;
  virtual ~LocalPlanner() = default;

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name) = 0;
  virtual void cleanup() {}

  // Collision-free path from start to goal inside `window` (empty if none was found within
  // solve_time [s]). `cancelled` returns true when the request was superseded: stop early.
  // Called from a worker thread.
  virtual std::vector<Eigen::Vector3d> plan(
    const std::shared_ptr<const CollisionWorld> & world, const Bounds & window,
    const Eigen::Vector3d & start, const Eigen::Vector3d & goal, double solve_time,
    const std::function<bool()> & cancelled) = 0;
};

}  // namespace uav_nav_core

#endif  // UAV_NAV_CORE__LOCAL_PLANNER_HPP_
