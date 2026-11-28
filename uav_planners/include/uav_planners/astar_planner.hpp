// Global planner plugin: the A* route (uav_planners::AStar) over the whole planning volume.
#ifndef UAV_PLANNERS__ASTAR_PLANNER_HPP_
#define UAV_PLANNERS__ASTAR_PLANNER_HPP_

#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

#include "uav_nav_core/global_planner.hpp"
#include "uav_planners/astar.hpp"

namespace uav_planners
{

class AStarPlanner : public uav_nav_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name) override;
  std::vector<Eigen::Vector3d> plan(
    const uav_nav_core::CollisionWorld & world, const uav_nav_core::Bounds & bounds,
    const Eigen::Vector3d & start, const Eigen::Vector3d & goal) override;

private:
  AStarOptions options_;
  rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};
  bool warned_resolution_{false};
};

}  // namespace uav_planners

#endif  // UAV_PLANNERS__ASTAR_PLANNER_HPP_
