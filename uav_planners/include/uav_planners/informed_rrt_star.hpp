// Local planner plugin: OMPL Informed-RRT* (or RRT* for the "balanced" objective) inside the
// window the planner server gives it, against the box of the CollisionWorld.
#ifndef UAV_PLANNERS__INFORMED_RRT_STAR_HPP_
#define UAV_PLANNERS__INFORMED_RRT_STAR_HPP_

#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

#include "uav_nav_core/local_planner.hpp"

namespace uav_planners
{

class InformedRRTStar : public uav_nav_core::LocalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name) override;
  std::vector<Eigen::Vector3d> plan(
    const std::shared_ptr<const uav_nav_core::CollisionWorld> & world,
    const uav_nav_core::Bounds & window, const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal, double solve_time,
    const std::function<bool()> & cancelled) override;

private:
  double range_{4.0}, cost_threshold_{10.0}, length_weight_{10.0}, clearance_weight_{1.0};
  double check_resolution_{0.1}, goal_tolerance_{0.3};
  bool k_nearest_{false}, smooth_{true};
  std::string objective_{"path_length_cost_to_go"};
  rclcpp::Logger logger_{rclcpp::get_logger("InformedRRTStar")};
};

}  // namespace uav_planners

#endif  // UAV_PLANNERS__INFORMED_RRT_STAR_HPP_
