// A* over a coarse 3D lattice of robot positions, checked against a CollisionWorld, followed
// by line-of-sight shortcutting. The route is only a guide for the local planner, which plans
// the flyable path along it. No ROS here: AStarPlanner wraps it as a planner server plugin.
#ifndef UAV_PLANNERS__ASTAR_HPP_
#define UAV_PLANNERS__ASTAR_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "uav_nav_core/collision_world.hpp"

namespace uav_planners
{

using uav_nav_core::CollisionWorld;

struct AStarOptions
{
  double resolution{0.4};         // [m] lattice spacing; keep it <= the smallest box size
  double snap_radius{1.2};        // [m] how far an invalid start / goal may be moved
  double shortcut_step{0.1};      // [m] sampling step of the line-of-sight checks
  size_t max_expansions{500000};  // give up after expanding this many nodes
};

struct AStarResult
{
  bool success{false};
  std::vector<Eigen::Vector3d> path;  // start, shortcut lattice nodes, goal
  double length{0.0};                 // [m]
  size_t expansions{0};
};

class AStar
{
public:
  explicit AStar(const AStarOptions & options) : options_(options) {}

  // Route from start to goal inside [bounds_min, bounds_max]. Lattice nodes are positions of
  // the robot box; a node is usable when world.isFree() holds there. Moves go to the 26
  // neighbours without cutting corners, cost = Euclidean length, heuristic = Euclidean
  // distance to the goal. Shortcuts are checked with isSegmentFree() every shortcut_step.
  AStarResult plan(
    const CollisionWorld & world, const Eigen::Vector3d & bounds_min,
    const Eigen::Vector3d & bounds_max, const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal) const;

private:
  AStarOptions options_;
};

}  // namespace uav_planners

#endif  // UAV_PLANNERS__ASTAR_HPP_
