#include "uav_planners/astar_planner.hpp"

#include <chrono>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

namespace uav_planners
{

namespace
{

template<typename T>
T declareOrGet(rclcpp_lifecycle::LifecycleNode & node, const std::string & name, const T & def)
{
  if (!node.has_parameter(name)) {node.declare_parameter<T>(name, def);}
  return node.get_parameter(name).get_value<T>();
}

}  // namespace

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name)
{
  auto node = parent.lock();
  if (!node) {throw std::runtime_error("AStarPlanner: the parent node is gone");}
  logger_ = node->get_logger().get_child(name);
  options_.resolution = declareOrGet<double>(*node, name + ".resolution", 0.4);
  options_.snap_radius = declareOrGet<double>(*node, name + ".snap_radius", 1.2);
  options_.shortcut_step = declareOrGet<double>(*node, name + ".shortcut_step", 0.1);
  options_.max_expansions =
    static_cast<size_t>(declareOrGet<int64_t>(*node, name + ".max_expansions", 500000));
  if (!(options_.resolution > 0.0) || !(options_.shortcut_step > 0.0)) {
    throw std::invalid_argument(name + ".resolution and .shortcut_step must be > 0");
  }
  RCLCPP_INFO(
    logger_, "A*: %.2f m lattice, snap %.2f m, %zu expansions max", options_.resolution,
    options_.snap_radius, options_.max_expansions);
}

std::vector<Eigen::Vector3d> AStarPlanner::plan(
  const uav_nav_core::CollisionWorld & world, const uav_nav_core::Bounds & bounds,
  const Eigen::Vector3d & start, const Eigen::Vector3d & goal)
{
  if (!warned_resolution_ && options_.resolution > world.boxSize().minCoeff()) {
    RCLCPP_WARN(
      logger_, "resolution %.2f m is larger than the robot box: A* moves may cut corners of "
      "obstacles", options_.resolution);
    warned_resolution_ = true;
  }
  const auto t0 = std::chrono::steady_clock::now();
  const AStar astar(options_);
  const auto res = astar.plan(world, bounds.min, bounds.max, start, goal);
  const double ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (!res.success) {
    RCLCPP_WARN(
      logger_, "No route to the goal (%zu nodes expanded, %.0f ms)", res.expansions, ms);
    return {};
  }
  RCLCPP_INFO(
    logger_, "Route: %zu points, %.1f m, %zu nodes expanded, %.0f ms", res.path.size(),
    res.length, res.expansions, ms);
  return res.path;
}

}  // namespace uav_planners

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(uav_planners::AStarPlanner, uav_nav_core::GlobalPlanner)
